#!/usr/bin/env python3
"""
FFB bridge for ESP-DrivingSimulator.

This version is intentionally profile-free.
All effect behavior is configured by directly editing constants below.

Transport:
- HID Output report ID 0x20
- Payload: int16 torque (LE), uint8 gain, then padding to 8 bytes

Requires:
- pip install hidapi
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    import hid  # type: ignore
except Exception as exc:
    print("Failed to import hid. Install with: pip install hidapi")
    raise SystemExit(1) from exc


# -----------------------------------------------------------------------------
# HID and general limits
# -----------------------------------------------------------------------------
REPORT_ID = 0x20
GAMEPAD_REPORT_ID = 0x01
MIN_TORQUE = -1000
MAX_TORQUE = 1000

# -----------------------------------------------------------------------------
# Core loop and signal conditioning (edit directly)
# -----------------------------------------------------------------------------
ASSIST_UPDATE_HZ = 250.0
POS_LPF_MS = 12.0
VEL_LPF_MS = 22.0
ACC_LPF_MS = 14.0

# Estimated HID units per steering degree (tune from your logs/hardware).
UNITS_PER_DEG = 115.0

# Center behavior and calibration.
CENTER_DEADBAND_UNITS = 700.0
DEFAULT_AUTO_CENTER_SAMPLES = 180
DEFAULT_CENTER_OFFSET_UNITS = 0

# Global output shaping.
GLOBAL_STRENGTH = 1.0
GLOBAL_MAX_TORQUE = 700
TORQUE_RATE_LIMIT = 3800.0  # torque units per second

# -----------------------------------------------------------------------------
# Effect toggles (enable/disable each torque source)
# -----------------------------------------------------------------------------
ENABLE_EFFECT_SPRING = False
ENABLE_EFFECT_DAMPER = True
ENABLE_EFFECT_FRICTION = False
ENABLE_EFFECT_INERTIA = False
ENABLE_EFFECT_ENDWALLS = True

# -----------------------------------------------------------------------------
# Effect gains and PID constants
# Each effect uses its own PID and its own error source (pos / vel / acc).
# -----------------------------------------------------------------------------

# SPRING (position-centered effect)
# Error source: position from center (normalized)
K_P_CENTER = 1500.0
K_I_CENTER = 600.0
K_D_CENTER = 0.0
I_LIMIT_CENTER = 20.0

# DAMPER (velocity resistance)
# Error source: steering velocity (normalized) with setpoint = 0
K_P_DAMPER = 200.0
K_I_DAMPER = 0.0
K_D_DAMPER = 0.0
I_LIMIT_DAMPER = 0.0

# FRICTION (static + very low-speed holding)
# Error source: velocity at very low speed and non-zero position
K_P_FRICTION = 200.0
K_I_FRICTION = 0.0
K_D_FRICTION = 0.0
I_LIMIT_FRICTION = 0.0
FRICTION_ACTIVE_POS_UNITS = 500.0
FRICTION_ACTIVE_VEL_UNITS = 300.0
FRICTION_BREAKAWAY_TORQUE = 55.0

# INERTIA (acceleration resistance)
# Error source: steering acceleration (normalized)
K_P_INERTIA = 40.0
K_I_INERTIA = 0.0
K_D_INERTIA = 0.0
I_LIMIT_INERTIA = 0.0

# ENDWALLS (soft/hard stop near max steering)
# Error source: position outside limit + velocity outside limit direction
ENDWALL_LIMIT_DEG = 270.0
ENDWALL_BAND_DEG = 24.0
K_P_ENDWALL = 600.0
K_I_ENDWALL = 0.0
K_D_ENDWALL = 0.0
I_LIMIT_ENDWALL = 0.0
ENDWALL_DAMP_K = 110.0
ENDWALL_MAX_TORQUE = 500.0


@dataclass
class PIDController:
    kp: float
    ki: float
    kd: float
    i_limit: float
    integral: float = 0.0
    prev_error: float = 0.0
    initialized: bool = False

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def update(self, error: float, dt: float, anti_windup: bool = True) -> float:
        if dt <= 0.0:
            return self.kp * error

        if not self.initialized:
            self.prev_error = error
            self.initialized = True

        derivative = (error - self.prev_error) / dt

        if anti_windup:
            # Integrate gently and clamp to avoid runaway accumulation.
            self.integral += error * dt
            if self.i_limit > 0.0:
                self.integral = clamp(self.integral, -self.i_limit, self.i_limit)

        self.prev_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def open_device(vid: int, pid: int):
    dev = hid.device()
    dev.open(vid, pid)
    dev.set_nonblocking(1)
    return dev


def send_torque(dev, torque: int, gain: int) -> None:
    torque_i = int(clamp(torque, MIN_TORQUE, MAX_TORQUE))
    gain_i = int(clamp(gain, 0, 255))

    lo = torque_i & 0xFF
    hi = (torque_i >> 8) & 0xFF
    payload = [lo, hi, gain_i, 0, 0, 0, 0, 0]

    # hidapi on Windows expects report-id as first byte.
    dev.write([REPORT_ID] + payload)


def _signed16(lo: int, hi: int) -> int:
    value = (hi << 8) | lo
    if value >= 0x8000:
        value -= 0x10000
    return int(value)


def decode_axis_x(packet, mode: str = "auto") -> Optional[int]:
    # Auto mode locks parser by report length to avoid frame-to-frame switching.
    if mode == "auto":
        if len(packet) >= 18:
            return _signed16(packet[1], packet[2])
        if len(packet) >= 17:
            return _signed16(packet[0], packet[1])
        return None

    if mode == "rid" and len(packet) >= 3:
        return _signed16(packet[1], packet[2])

    if mode == "legacy" and len(packet) >= 2:
        return _signed16(packet[0], packet[1])

    return None


def run_axis_monitor(dev, axis_mode: str, update_hz: float, center_samples: int) -> None:
    print(
        f"Axis monitor: axis_mode={axis_mode}, update_hz={update_hz}, "
        f"auto_center_samples={center_samples}"
    )

    dt = 1.0 / max(1.0, update_hz)
    last_print = time.perf_counter()

    center_sum = 0.0
    center_count = 0
    center_bias = 0.0
    centered = center_samples <= 0
    prev = None

    while True:
        packet = dev.read(64)
        now = time.perf_counter()

        if packet:
            raw = decode_axis_x(packet, mode=axis_mode)
            if raw is None:
                time.sleep(0.001)
                continue

            if not centered:
                center_sum += raw
                center_count += 1
                if center_count >= center_samples:
                    center_bias = center_sum / float(center_count)
                    centered = True
                    print(f"Auto-center done: bias={int(center_bias)}")
                continue

            err = int(raw - center_bias)
            jump = 0 if prev is None else int(raw - prev)
            prev = raw

            if now - last_print >= 0.2:
                print(f"raw={raw:6d} err={err:6d} jump={jump:6d} len={len(packet):2d}")
                last_print = now
        else:
            time.sleep(min(0.002, dt))


def run_assist(
    dev,
    gain: int,
    axis_mode: str,
    auto_center_samples: int,
    center_offset_units: int,
    global_strength: float,
    global_max_torque: int,
    endwall_limit_deg: float,
    units_per_deg: float,
) -> None:
    print(
        "Assist mode (PID by effect): "
        f"gain={gain}, axis_mode={axis_mode}, auto_center_samples={auto_center_samples}, "
        f"center_offset_units={center_offset_units}, max_torque={global_max_torque}, "
        f"endwall_limit_deg={endwall_limit_deg}, units_per_deg={units_per_deg:.2f}"
    )

    # Controllers per effect.
    pid_center = PIDController(K_P_CENTER * global_strength, K_I_CENTER * global_strength, K_D_CENTER * global_strength, I_LIMIT_CENTER)
    pid_damper = PIDController(K_P_DAMPER * global_strength, K_I_DAMPER * global_strength, K_D_DAMPER * global_strength, I_LIMIT_DAMPER)
    pid_friction = PIDController(K_P_FRICTION * global_strength, K_I_FRICTION * global_strength, K_D_FRICTION * global_strength, I_LIMIT_FRICTION)
    pid_inertia = PIDController(K_P_INERTIA * global_strength, K_I_INERTIA * global_strength, K_D_INERTIA * global_strength, I_LIMIT_INERTIA)
    pid_endwall = PIDController(K_P_ENDWALL * global_strength, K_I_ENDWALL * global_strength, K_D_ENDWALL * global_strength, I_LIMIT_ENDWALL)

    dt_target = 1.0 / max(1.0, ASSIST_UPDATE_HZ)
    last_t = time.perf_counter()
    last_print = last_t

    pos_filt = 0.0
    prev_pos_filt = 0.0
    vel_filt = 0.0
    prev_vel_filt = 0.0
    acc_filt = 0.0
    torque_prev = 0.0

    center_acc = 0.0
    center_count = 0
    center_bias = float(center_offset_units)
    center_done = auto_center_samples <= 0

    while True:
        packet = dev.read(64)
        now = time.perf_counter()

        if packet:
            raw = decode_axis_x(packet, mode=axis_mode)
            if raw is None:
                time.sleep(0.001)
                continue

            if not center_done:
                center_acc += float(raw)
                center_count += 1
                if center_count >= auto_center_samples:
                    center_bias = (center_acc / float(center_count)) + float(center_offset_units)
                    center_done = True
                    print(f"Auto-center done: bias={int(center_bias)} (offset_arg={center_offset_units})")
                continue

            dt = max(0.0005, now - last_t)

            # Error relative to center in HID units.
            pos_raw = clamp(raw - center_bias, -32768.0, 32767.0)

            # Filtering chain: position -> velocity -> acceleration.
            pos_tau = max(0.0, POS_LPF_MS / 1000.0)
            vel_tau = max(0.0, VEL_LPF_MS / 1000.0)
            acc_tau = max(0.0, ACC_LPF_MS / 1000.0)

            pos_alpha = 1.0 if pos_tau <= 0.0 else dt / (pos_tau + dt)
            vel_alpha = 1.0 if vel_tau <= 0.0 else dt / (vel_tau + dt)
            acc_alpha = 1.0 if acc_tau <= 0.0 else dt / (acc_tau + dt)

            pos_filt += pos_alpha * (pos_raw - pos_filt)
            vel_raw = (pos_filt - prev_pos_filt) / dt
            vel_filt += vel_alpha * (vel_raw - vel_filt)
            acc_raw = (vel_filt - prev_vel_filt) / dt
            acc_filt += acc_alpha * (acc_raw - acc_filt)

            # Position error with deadband shaping.
            if abs(pos_filt) <= CENTER_DEADBAND_UNITS:
                pos_eff = 0.0
            else:
                pos_eff = math.copysign(abs(pos_filt) - CENTER_DEADBAND_UNITS, pos_filt)

            pos_norm = pos_eff / max(1.0, (32767.0 - CENTER_DEADBAND_UNITS))
            vel_norm = vel_filt / max(1.0, (UNITS_PER_DEG * 120.0))
            acc_norm = acc_filt / max(1.0, (UNITS_PER_DEG * 2500.0))

            # -----------------------------------------------------------------
            # Effect 1: SPRING (position PID, setpoint = 0)
            # error source: position
            # -----------------------------------------------------------------
            torque_spring = 0.0
            if ENABLE_EFFECT_SPRING:
                error_center = -pos_norm
                torque_spring = pid_center.update(error_center, dt, anti_windup=True)

            # -----------------------------------------------------------------
            # Effect 2: DAMPER (velocity PID, setpoint = 0)
            # error source: velocity
            # -----------------------------------------------------------------
            torque_damper = 0.0
            if ENABLE_EFFECT_DAMPER:
                error_vel = -vel_norm
                torque_damper = pid_damper.update(error_vel, dt, anti_windup=False)

            # -----------------------------------------------------------------
            # Effect 3: FRICTION (low-speed PID + breakaway)
            # error source: velocity (only active when displaced and near stop)
            # -----------------------------------------------------------------
            torque_friction = 0.0
            if ENABLE_EFFECT_FRICTION and abs(pos_eff) > FRICTION_ACTIVE_POS_UNITS:
                if abs(vel_filt) < FRICTION_ACTIVE_VEL_UNITS:
                    error_vel_slow = -vel_norm
                    torque_friction = pid_friction.update(error_vel_slow, dt, anti_windup=False)

                    # Breakaway assistance to overcome static friction when stuck.
                    if abs(vel_filt) < (0.5 * FRICTION_ACTIVE_VEL_UNITS):
                        torque_friction += -math.copysign(FRICTION_BREAKAWAY_TORQUE, pos_eff)
                else:
                    pid_friction.reset()

            # -----------------------------------------------------------------
            # Effect 4: INERTIA (acceleration PID, setpoint = 0)
            # error source: acceleration
            # -----------------------------------------------------------------
            torque_inertia = 0.0
            if ENABLE_EFFECT_INERTIA:
                error_acc = -acc_norm
                torque_inertia = pid_inertia.update(error_acc, dt, anti_windup=False)

            # -----------------------------------------------------------------
            # Effect 5: ENDWALLS (position PID outside range + extra damping)
            # error source: overflow past steering limit
            # -----------------------------------------------------------------
            torque_endwall = 0.0
            if ENABLE_EFFECT_ENDWALLS:
                abs_deg = abs(pos_filt) / max(1.0, units_per_deg)
                overflow_deg = max(0.0, abs_deg - endwall_limit_deg)

                if overflow_deg > 0.0:
                    wall_pos_norm = clamp(overflow_deg / max(1.0, ENDWALL_BAND_DEG), 0.0, 1.0)
                    # Push back inward.
                    error_wall = -math.copysign(wall_pos_norm, pos_filt)
                    torque_endwall = pid_endwall.update(error_wall, dt, anti_windup=False)

                    # Extra damping while outside range.
                    torque_endwall += -(ENDWALL_DAMP_K * vel_norm)
                    torque_endwall = clamp(torque_endwall, -ENDWALL_MAX_TORQUE, ENDWALL_MAX_TORQUE)
                else:
                    pid_endwall.reset()

            torque = torque_spring + torque_damper + torque_friction + torque_inertia + torque_endwall

            # Torque slew-rate limiter.
            max_delta = max(1.0, TORQUE_RATE_LIMIT * dt)
            delta = torque - torque_prev
            if delta > max_delta:
                torque = torque_prev + max_delta
            elif delta < -max_delta:
                torque = torque_prev - max_delta

            torque_i = int(clamp(round(torque), -global_max_torque, global_max_torque))
            send_torque(dev, torque_i, gain)

            torque_prev = float(torque_i)
            prev_pos_filt = pos_filt
            prev_vel_filt = vel_filt
            last_t = now

            if now - last_print >= 0.5:
                print(
                    f"raw={int(raw):6d} pos={int(pos_filt):6d} vel={int(vel_filt):7d} "
                    f"acc={int(acc_filt):8d} "
                    f"spr={int(torque_spring):4d} dam={int(torque_damper):4d} "
                    f"fri={int(torque_friction):4d} inr={int(torque_inertia):4d} "
                    f"wal={int(torque_endwall):4d} tq={torque_i:5d}"
                )
                last_print = now
        else:
            time.sleep(min(0.002, dt_target))


def run_sine(dev, gain: int, amp: int, hz: float, update_hz: float) -> None:
    print(f"Sine mode: amp={amp}, hz={hz}, update_hz={update_hz}, gain={gain}")
    t0 = time.perf_counter()
    dt = 1.0 / max(1.0, update_hz)

    while True:
        t = time.perf_counter() - t0
        torque = int(amp * math.sin(2.0 * math.pi * hz * t))
        send_torque(dev, torque, gain)
        time.sleep(dt)


def parse_udp_payload(text: str) -> Optional[Tuple[int, Optional[int]]]:
    text = text.strip()
    if not text:
        return None

    if text.startswith("{"):
        data = json.loads(text)
        torque = int(data.get("torque", 0))
        gain = data.get("gain")
        return torque, (int(gain) if gain is not None else None)

    return int(text), None


def run_udp(dev, listen_ip: str, port: int, default_gain: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((listen_ip, port))
    print(f"UDP mode listening on {listen_ip}:{port}, default_gain={default_gain}")

    while True:
        data, _ = sock.recvfrom(2048)
        text = data.decode("utf-8", errors="ignore")
        parsed = parse_udp_payload(text)
        if parsed is None:
            continue
        torque, gain = parsed
        send_torque(dev, torque, default_gain if gain is None else gain)


def main() -> int:
    parser = argparse.ArgumentParser(description="ESP Driving Simulator FFB HID bridge (PID effects)")
    parser.add_argument("--vid", type=lambda s: int(s, 0), default=0x303A, help="USB VID")
    parser.add_argument("--pid", type=lambda s: int(s, 0), default=0x1001, help="USB PID")
    parser.add_argument("--gain", type=int, default=140, help="Default gain 0..255")

    sub = parser.add_subparsers(dest="mode", required=True)

    sp_assist = sub.add_parser("assist", help="FFB assist with PID effects")
    sp_assist.add_argument("--axis-mode", choices=["auto", "rid", "legacy"], default="auto")
    sp_assist.add_argument("--auto-center-samples", type=int, default=DEFAULT_AUTO_CENTER_SAMPLES)
    sp_assist.add_argument("--center-offset", type=int, default=DEFAULT_CENTER_OFFSET_UNITS)
    sp_assist.add_argument("--strength", type=float, default=GLOBAL_STRENGTH)
    sp_assist.add_argument("--max-torque", type=int, default=GLOBAL_MAX_TORQUE)
    sp_assist.add_argument("--endstop-deg", type=float, default=ENDWALL_LIMIT_DEG)
    sp_assist.add_argument("--units-per-deg", type=float, default=UNITS_PER_DEG)

    sp_axis = sub.add_parser("axis", help="Monitor wheel position diagnostics")
    sp_axis.add_argument("--axis-mode", choices=["auto", "rid", "legacy"], default="auto")
    sp_axis.add_argument("--update-hz", type=float, default=120.0)
    sp_axis.add_argument("--auto-center-samples", type=int, default=80)

    sp_sine = sub.add_parser("sine", help="Generate sine torque test")
    sp_sine.add_argument("--amp", type=int, default=220)
    sp_sine.add_argument("--hz", type=float, default=0.8)
    sp_sine.add_argument("--update-hz", type=float, default=100.0)

    sp_udp = sub.add_parser("udp", help="Forward torque from UDP packets")
    sp_udp.add_argument("--listen-ip", default="127.0.0.1")
    sp_udp.add_argument("--port", type=int, default=34345)

    args = parser.parse_args()

    try:
        dev = open_device(args.vid, args.pid)
    except Exception as exc:
        print(f"Unable to open HID device VID=0x{args.vid:04X} PID=0x{args.pid:04X}: {exc}")
        return 2

    print(f"Connected HID device VID=0x{args.vid:04X} PID=0x{args.pid:04X}")

    try:
        if args.mode == "assist":
            run_assist(
                dev=dev,
                gain=args.gain,
                axis_mode=args.axis_mode,
                auto_center_samples=args.auto_center_samples,
                center_offset_units=args.center_offset,
                global_strength=args.strength,
                global_max_torque=int(clamp(args.max_torque, 80, 950)),
                endwall_limit_deg=args.endstop_deg,
                units_per_deg=max(1.0, args.units_per_deg),
            )
        elif args.mode == "axis":
            run_axis_monitor(
                dev=dev,
                axis_mode=args.axis_mode,
                update_hz=args.update_hz,
                center_samples=args.auto_center_samples,
            )
        elif args.mode == "sine":
            run_sine(dev, args.gain, args.amp, args.hz, args.update_hz)
        elif args.mode == "udp":
            run_udp(dev, args.listen_ip, args.port, args.gain)
        else:
            print("Unknown mode")
            return 3
    except KeyboardInterrupt:
        print("Stopping bridge")
    finally:
        try:
            send_torque(dev, 0, args.gain)
        except Exception:
            pass
        dev.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
