#!/usr/bin/env python3
"""ESP wheel bridge: Serial telemetry <-> vJoy axes/buttons <-> FFB torque back to ESP.

Protocol (little-endian):
- ESP -> PC telemetry (14 bytes):
  [0]=0xAB [1]=0x10 [2]=seq [3:5]=x [5:7]=y [7:9]=z [9:13]=buttons_u32 [13]=crc_xor
- PC -> ESP torque command (8 bytes):
  [0]=0xAB [1]=0x20 [2]=seq [3:5]=torque_i16 [5]=gain_u8 [6]=flags [7]=crc_xor
- PC -> ESP control command (7 bytes):
    [0]=0xAB [1]=0x30 [2]=seq [3]=cmd_id [4:6]=value_i16 [6]=crc_xor
"""

from __future__ import annotations

import argparse
import ctypes
import datetime
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable

import serial


BRIDGE_START    = 0xAB
PKT_TELEMETRY   = 0x10
PKT_TORQUE      = 0x20
PKT_CONTROL     = 0x30

TELEMETRY_LEN   = 14
TORQUE_LEN      = 8
CONTROL_LEN     = 7

CMD_STARTUP     = 0x01

STARTUP_FLAG_AUTO_CALIB = 0x0001

HID_USAGE_X     = 0x30
HID_USAGE_Y     = 0x31
HID_USAGE_Z     = 0x32

ERROR_SUCCESS   = 0

EFF_START       = 1
EFF_SOLO        = 2
EFF_STOP        = 3

CTRL_ENACT      = 1
CTRL_DISACT     = 2
CTRL_STOPALL    = 3
CTRL_DEVRST     = 4
CTRL_DEVPAUSE   = 5
CTRL_DEVCONT    = 6

# HID PID output packet types (matched by Ffb_h_Type)
PT_EFFREP       = 0x01  # Set Effect Report
PT_ENVREP       = 0x02  # Set Envelope Report
PT_CONDREP      = 0x03  # Set Condition Report
PT_PRIDREP      = 0x04  # Set Periodic Report
PT_CONSTREP     = 0x05  # Set Constant Force Report
PT_RAMPREP      = 0x06  # Set Ramp Force Report
PT_EFOPREP      = 0x0A  # Effect Operation Report
PT_BLKFRREP     = 0x0B  # Block Free Report
PT_CTRLREP      = 0x0C  # Device Control
PT_GAINREP      = 0x0D  # Device Gain Report
PT_NEWEFREP     = 0x11  # Create New Effect (Feature report, id 0x01 + 0x10 offset)

ET_NONE         = 0
ET_CONST        = 1
ET_RAMP         = 2
ET_SQR          = 3
ET_SINE         = 4
ET_TRNGL        = 5
ET_STUP         = 6
ET_STDN         = 7
ET_SPRNG        = 8
ET_DMPR         = 9
ET_INRT         = 10
ET_FRCTN        = 11
ET_CSTM         = 12


class FFB_DATA(ctypes.Structure):
    _fields_ = [
        ("size", ctypes.c_uint32),
        ("cmd", ctypes.c_uint32),
        ("data", ctypes.POINTER(ctypes.c_ubyte)),
    ]


class FFB_EFF_CONSTANT(ctypes.Structure):
    _fields_ = [
        ("EffectBlockIndex", ctypes.c_ubyte),
        ("Magnitude", ctypes.c_int32),  # -10000..10000
    ]


class FFB_EFF_OP(ctypes.Structure):
    _fields_ = [
        ("EffectBlockIndex", ctypes.c_ubyte),
        ("EffectOp", ctypes.c_int),
        ("LoopCount", ctypes.c_ubyte),
    ]


class FFB_EFF_RAMP(ctypes.Structure):
    _fields_ = [
        ("EffectBlockIndex", ctypes.c_ubyte),
        ("Start", ctypes.c_int32),
        ("End", ctypes.c_int32),
    ]


class FFB_EFF_REPORT(ctypes.Structure):
    _fields_ = [
        ("EffectBlockIndex", ctypes.c_ubyte),
        ("EffectType", ctypes.c_int),
        ("Duration", ctypes.c_uint16),
        ("TrigerRpt", ctypes.c_uint16),
        ("SamplePrd", ctypes.c_uint16),
        ("Gain", ctypes.c_ubyte),
        ("TrigerBtn", ctypes.c_ubyte),
        ("Polar", ctypes.c_int32),
        ("Direction", ctypes.c_ubyte),
        ("DirY", ctypes.c_ubyte),
    ]


class FFB_EFF_PERIOD(ctypes.Structure):
    _fields_ = [
        ("EffectBlockIndex", ctypes.c_ubyte),
        ("Magnitude", ctypes.c_uint32),
        ("Offset", ctypes.c_int32),
        ("Phase", ctypes.c_uint32),
        ("Period", ctypes.c_uint32),
    ]


class FFB_EFF_COND(ctypes.Structure):
    _fields_ = [
        ("EffectBlockIndex", ctypes.c_ubyte),
        ("isY", ctypes.c_int32),
        ("CenterPointOffset", ctypes.c_int32),
        ("PosCoeff", ctypes.c_int32),
        ("NegCoeff", ctypes.c_int32),
        ("PosSatur", ctypes.c_uint32),
        ("NegSatur", ctypes.c_uint32),
        ("DeadBand", ctypes.c_int32),
    ]


class FFB_EFF_ENVLP(ctypes.Structure):
    _fields_ = [
        ("EffectBlockIndex", ctypes.c_ubyte),
        ("AttackLevel", ctypes.c_uint32),
        ("FadeLevel", ctypes.c_uint32),
        ("AttackTime", ctypes.c_uint32),
        ("FadeTime", ctypes.c_uint32),
    ]


@dataclass
class AxisRange:
    min_val: int
    max_val: int


class BridgeLogger:
    def __init__(self, path: str | None) -> None:
        self.path = path
        self._lock = threading.Lock()
        self._file = None
        if path:
            # Line-buffered append mode so logs survive abrupt exits.
            self._file = open(path, "a", encoding="utf-8", buffering=1)
            self.log("[LOG] ---- bridge session started ----")

    def log(self, message: str) -> None:
        if not self._file:
            return
        ts = datetime.datetime.now().isoformat(timespec="milliseconds")
        line = f"{ts} {message}"
        with self._lock:
            self._file.write(line + "\n")

    def close(self) -> None:
        if not self._file:
            return
        with self._lock:
            try:
                self._file.write(f"{datetime.datetime.now().isoformat(timespec='milliseconds')} [LOG] ---- bridge session ended ----\n")
                self._file.flush()
            finally:
                self._file.close()
                self._file = None


class TorqueState:
    def __init__(
        self,
        max_torque: int,
        min_torque: int = 0,
        use_min_torque_comp: bool = False,
        input_deadzone: int = 0,
        min_torque_input: int = 0,
        torque_response_gamma: float = 1.0,
    ) -> None:
        self.max_torque = max(1, int(max_torque))
        self.min_torque = max(0, int(min_torque))
        self.use_min_torque_comp = bool(use_min_torque_comp)
        self.input_deadzone = max(0, int(input_deadzone))
        self.min_torque_input = max(0, int(min_torque_input))
        self.torque_response_gamma = max(0.1, float(torque_response_gamma))
        self.device_gain = 255
        self._effect_contribution: dict[int, float] = {}
        self._effect_active: dict[int, bool] = {}
        self.x_norm = 0.0
        self.x_vel = 0.0
        self.x_acc = 0.0
        self._last_motion_ts = 0.0
        self._lock = threading.Lock()

    def set_device_gain(self, gain: int) -> None:
        with self._lock:
            self.device_gain = max(0, min(255, int(gain)))

    def set_effect_contribution(self, block_index: int, contribution: float) -> None:
        with self._lock:
            self._effect_contribution[int(block_index)] = max(-1.0, min(1.0, float(contribution)))

    def set_effect_active(self, block_index: int, active: bool) -> None:
        with self._lock:
            self._effect_active[int(block_index)] = bool(active)

    def stop_effect_block(self, block_index: int) -> None:
        with self._lock:
            self._effect_active[int(block_index)] = False
            self._effect_contribution[int(block_index)] = 0.0

    def clear_all_effects(self) -> None:
        with self._lock:
            self._effect_active.clear()
            self._effect_contribution.clear()

    def update_motion(self, x_raw: int, now: float | None = None) -> None:
        now_ts = time.monotonic() if now is None else float(now)
        x = max(-32768, min(32767, int(x_raw)))
        x_norm = x / 32767.0 if x >= 0 else x / 32768.0

        with self._lock:
            if self._last_motion_ts > 0.0:
                dt = max(1e-6, now_ts - self._last_motion_ts)
                new_vel = (x_norm - self.x_norm) / dt
                self.x_acc = (new_vel - self.x_vel) / dt
                self.x_vel = new_vel
            self.x_norm = x_norm
            self._last_motion_ts = now_ts

    def snapshot(self) -> tuple[int, dict[int, float], dict[int, bool], dict[str, float]]:
        with self._lock:
            return (
                int(self.device_gain),
                dict(self._effect_contribution),
                dict(self._effect_active),
                {
                    "x_norm": self.x_norm,
                    "x_vel": self.x_vel,
                    "x_acc": self.x_acc,
                },
            )

    def compute_torque(self) -> int:
        with self._lock:
            summed = 0.0
            for block, contribution in self._effect_contribution.items():
                if not self._effect_active.get(block, False):
                    continue
                summed += contribution

            # Keep total effect bounded before max_torque scaling.
            summed = max(-1.0, min(1.0, summed))
            scaled = summed * self.max_torque
            scaled *= self.device_gain / 255.0
        torque = int(round(scaled))

        # Invert output sign to match this wheel's motor/driver direction.
        torque = -torque

        # Optional low-force compensation so small FFB values can overcome motor/driver stiction.
        if torque == 0:
            return 0

        sign = 1 if torque > 0 else -1
        mag = max(0, min(self.max_torque, abs(int(torque))))

        if mag <= self.input_deadzone:
            return 0

        if self.use_min_torque_comp and self.min_torque > 0 and self.min_torque < self.max_torque:
            in_start = max(0, min(self.max_torque, self.min_torque_input))
            if mag <= in_start:
                mag = self.min_torque
            else:
                in_span = max(1, self.max_torque - in_start)
                norm = max(0.0, min(1.0, (mag - in_start) / in_span))
                norm = norm ** self.torque_response_gamma
                out_span = self.max_torque - self.min_torque
                mag = int(round(self.min_torque + (norm * out_span)))

        return sign * max(0, min(self.max_torque, mag))


class FFBEffectsManager:
    # Torque output tunables: edit these to match your wheel motor/driver behavior.
    MAX_TORQUE              = 270
    USE_MIN_TORQUE_COMP     = True
    MIN_TORQUE              = 160
    INPUT_DEADZONE          = 5
    MIN_TORQUE_INPUT        = 0
    TORQUE_RESPONSE_GAMMA   = 1.0

    # Global force scale tunables: edit here, no CLI args needed.
    GLOBAL_FORCE_SCALE      = 1.0
    PERIODIC_FORCE_SCALE    = 1.0
    CONDITION_FORCE_SCALE   = 1.0
    CONSTANT_OVERLAY_SCALE  = 1.0

    # Per-effect scale tunables.
    CONST_FORCE_SCALE       = 1.0
    RAMP_FORCE_SCALE        = 0.5
    SINE_FORCE_SCALE        = 0.5
    SQUARE_FORCE_SCALE      = 0.5
    TRIANGLE_FORCE_SCALE    = 0.5
    SAW_UP_FORCE_SCALE      = 0.5
    SAW_DOWN_FORCE_SCALE    = 0.5
    SPRING_FORCE_SCALE      = 0.5
    DAMPER_FORCE_SCALE      = 0.5
    INERTIA_FORCE_SCALE     = 0.5
    FRICTION_FORCE_SCALE    = 0.5

    def __init__(
        self,
        dll: ctypes.CDLL,
        torque_state: TorqueState,
        log_cb: Callable[[str], None],
        debug_enabled: bool,
    ) -> None:
        self.dll = dll
        self.torque_state = torque_state
        self._log_cb = log_cb
        self.debug_enabled = debug_enabled

        self._effect_report: dict[int, FFB_EFF_REPORT] = {}
        self._constant: dict[int, int] = {}
        self._ramp: dict[int, tuple[int, int]] = {}
        self._periodic: dict[int, FFB_EFF_PERIOD] = {}
        self._condition_x: dict[int, FFB_EFF_COND] = {}
        self._envelope: dict[int, FFB_EFF_ENVLP] = {}
        self._effect_type: dict[int, int] = {}
        self._effect_active: dict[int, bool] = {}
        self._effect_start_ts: dict[int, float] = {}
        self._device_paused = False

    @staticmethod
    def _normalize_signed_10000(value: int) -> int:
        v = int(value)
        if v > 32767:
            v -= 65536
        return max(-10000, min(10000, v))

    @staticmethod
    def _normalize_unsigned_10000(value: int) -> int:
        return max(0, min(10000, int(value)))

    @staticmethod
    def _u8_to_signed_norm(raw: int) -> float:
        v = int(raw) & 0xFF
        signed = v - 256 if v >= 128 else v
        if signed == -128:
            return -1.0
        return max(-1.0, min(1.0, signed / 127.0))

    @staticmethod
    def _effect_type_name(effect_type: int) -> str:
        names = {
            ET_NONE:    "none",
            ET_CONST:   "constant",
            ET_RAMP:    "ramp",
            ET_SQR:     "square",
            ET_SINE:    "sine",
            ET_TRNGL:   "triangle",
            ET_STUP:    "sawtooth_up",
            ET_STDN:    "sawtooth_down",
            ET_SPRNG:   "spring",
            ET_DMPR:    "damper",
            ET_INRT:    "inertia",
            ET_FRCTN:   "friction",
            ET_CSTM:    "custom",
        }
        return names.get(int(effect_type), f"unknown_{int(effect_type)}")

    def _log(self, msg: str) -> None:
        if self.debug_enabled:
            self._log_cb(msg)

    def _direction_x_scale(self, report: FFB_EFF_REPORT) -> float:
        if bool(report.Polar):
            angle_deg = (int(report.Direction) & 0xFF) * (360.0 / 255.0)
            return math.cos(math.radians(angle_deg))
        return self._u8_to_signed_norm(int(report.Direction))

    def _envelope_scale(self, block: int, now: float, duration_ms: int) -> float:
        env = self._envelope.get(block)
        if env is None:
            return 1.0

        start_ts = self._effect_start_ts.get(block, now)
        elapsed_ms = max(0.0, (now - start_ts) * 1000.0)

        attack_level = self._normalize_unsigned_10000(env.AttackLevel) / 10000.0
        fade_level = self._normalize_unsigned_10000(env.FadeLevel) / 10000.0
        out = 1.0

        if env.AttackTime > 0 and elapsed_ms < env.AttackTime:
            t = elapsed_ms / float(env.AttackTime)
            out *= attack_level + (1.0 - attack_level) * t

        if duration_ms > 0 and duration_ms != 0xFFFF and env.FadeTime > 0:
            remaining = duration_ms - elapsed_ms
            if remaining <= env.FadeTime:
                t = max(0.0, remaining / float(env.FadeTime))
                out *= fade_level + (1.0 - fade_level) * t

        return max(0.0, min(1.0, out))

    def _apply_condition_force(self, cond: FFB_EFF_COND, signal_value: float) -> float:
        center = self._normalize_signed_10000(cond.CenterPointOffset) / 10000.0
        deadband = max(0.0, min(1.0, abs(int(cond.DeadBand)) / 10000.0))
        pos_coeff = self._normalize_signed_10000(cond.PosCoeff) / 10000.0
        neg_coeff = self._normalize_signed_10000(cond.NegCoeff) / 10000.0
        pos_sat = self._normalize_unsigned_10000(cond.PosSatur) / 10000.0
        neg_sat = self._normalize_unsigned_10000(cond.NegSatur) / 10000.0

        err = signal_value - center
        if abs(err) <= deadband:
            return 0.0

        if err > 0:
            force = err * pos_coeff
            return max(-pos_sat, min(pos_sat, force))
        force = err * neg_coeff
        return max(-neg_sat, min(neg_sat, force))

    def _compute_periodic_wave(self, effect_type: int, phase: float) -> float:
        two_pi = 2.0 * math.pi
        p = phase % two_pi
        u = p / two_pi

        if effect_type == ET_SINE:
            return math.sin(p)
        if effect_type == ET_SQR:
            return 1.0 if math.sin(p) >= 0.0 else -1.0
        if effect_type == ET_TRNGL:
            return 2.0 * abs(2.0 * (u - math.floor(u + 0.5))) - 1.0
        if effect_type == ET_STUP:
            return 2.0 * u - 1.0
        if effect_type == ET_STDN:
            return 1.0 - 2.0 * u
        return 0.0

    def handle_effect_report(self, pkt: ctypes.POINTER(FFB_DATA), now: float) -> None:
        eff_report = FFB_EFF_REPORT()
        if self.dll.Ffb_h_Eff_Report(pkt, ctypes.byref(eff_report)) != ERROR_SUCCESS:
            return

        block = int(eff_report.EffectBlockIndex)
        self._effect_report[block] = eff_report
        self._effect_type[block] = int(eff_report.EffectType)
        self._log(
            "EffReport "
            f"block={block} type={self._effect_type_name(eff_report.EffectType)} "
            f"duration={int(eff_report.Duration)} gain={int(eff_report.Gain)} "
            f"polar={int(bool(eff_report.Polar))} dir={int(eff_report.Direction)} dirY={int(eff_report.DirY)}"
        )

    def handle_constant(self, pkt: ctypes.POINTER(FFB_DATA), now: float) -> None:
        const_eff = FFB_EFF_CONSTANT()
        if self.dll.Ffb_h_Eff_Constant(pkt, ctypes.byref(const_eff)) != ERROR_SUCCESS:
            return
        block = int(const_eff.EffectBlockIndex)
        mag = self._normalize_signed_10000(const_eff.Magnitude)
        self._constant[block] = mag

        # Some games (e.g. BeamNG) stream constant force without Effect Report / Effect Op.
        # Auto-run only this orphan constant path, so ACC-style typed effects keep normal Start/Stop semantics.
        if block not in self._effect_report:
            self._effect_type[block] = ET_CONST
            if mag != 0:
                self._effect_active[block] = True
                self._effect_start_ts.setdefault(block, now)
                self.torque_state.set_effect_active(block, True)
            else:
                self._effect_active[block] = False
                self._effect_start_ts.pop(block, None)
                self.torque_state.stop_effect_block(block)

        self._log(f"Constant block={block} magnitude={mag}")

    def handle_ramp(self, pkt: ctypes.POINTER(FFB_DATA)) -> None:
        ramp = FFB_EFF_RAMP()
        if self.dll.Ffb_h_Eff_Ramp(pkt, ctypes.byref(ramp)) != ERROR_SUCCESS:
            return
        block = int(ramp.EffectBlockIndex)
        start = self._normalize_signed_10000(ramp.Start)
        end = self._normalize_signed_10000(ramp.End)
        self._ramp[block] = (start, end)
        if block not in self._effect_report:
            self._effect_type[block] = ET_RAMP
        self._log(f"Ramp block={block} start={start} end={end}")

    def handle_periodic(self, pkt: ctypes.POINTER(FFB_DATA)) -> None:
        per = FFB_EFF_PERIOD()
        if self.dll.Ffb_h_Eff_Period(pkt, ctypes.byref(per)) != ERROR_SUCCESS:
            return
        block = int(per.EffectBlockIndex)
        self._periodic[block] = per
        if block in self._effect_report:
            self._effect_type[block] = int(self._effect_report[block].EffectType)
        self._log(
            f"Periodic block={block} mag={int(per.Magnitude)} off={int(per.Offset)} "
            f"phase={int(per.Phase)} period={int(per.Period)}"
        )

    def handle_condition(self, pkt: ctypes.POINTER(FFB_DATA)) -> None:
        cond = FFB_EFF_COND()
        if self.dll.Ffb_h_Eff_Cond(pkt, ctypes.byref(cond)) != ERROR_SUCCESS:
            return
        block = int(cond.EffectBlockIndex)
        if bool(cond.isY):
            self._log(f"Condition block={block} axis=Y (ignored for wheel X)")
            return
        self._condition_x[block] = cond
        if block in self._effect_report:
            self._effect_type[block] = int(self._effect_report[block].EffectType)
        self._log(
            "Condition "
            f"block={block} axis=X center={int(cond.CenterPointOffset)} dead={int(cond.DeadBand)} "
            f"posCoeff={int(cond.PosCoeff)} negCoeff={int(cond.NegCoeff)} "
            f"posSat={int(cond.PosSatur)} negSat={int(cond.NegSatur)}"
        )

    def handle_envelope(self, pkt: ctypes.POINTER(FFB_DATA)) -> None:
        env = FFB_EFF_ENVLP()
        if self.dll.Ffb_h_Eff_Envlp(pkt, ctypes.byref(env)) != ERROR_SUCCESS:
            return
        block = int(env.EffectBlockIndex)
        self._envelope[block] = env
        self._log(
            f"Envelope block={block} attackLevel={int(env.AttackLevel)} fadeLevel={int(env.FadeLevel)} "
            f"attackTime={int(env.AttackTime)} fadeTime={int(env.FadeTime)}"
        )

    def handle_operation(self, pkt: ctypes.POINTER(FFB_DATA), now: float) -> None:
        eff_op = FFB_EFF_OP()
        if self.dll.Ffb_h_EffOp(pkt, ctypes.byref(eff_op)) != ERROR_SUCCESS:
            return

        block = int(eff_op.EffectBlockIndex)
        op = int(eff_op.EffectOp)
        if op in (EFF_START, EFF_SOLO):
            self._effect_active[block] = True
            self._effect_start_ts[block] = now
            self.torque_state.set_effect_active(block, True)
        elif op == EFF_STOP:
            self._effect_active[block] = False
            self._effect_start_ts.pop(block, None)
            self.torque_state.stop_effect_block(block)
        self._log(f"EffOp block={block} op={op} loops={int(eff_op.LoopCount)}")

    def handle_device_control(self, pkt: ctypes.POINTER(FFB_DATA)) -> None:
        ctrl = ctypes.c_int(0)
        if self.dll.Ffb_h_DevCtrl(pkt, ctypes.byref(ctrl)) != ERROR_SUCCESS:
            return

        c = int(ctrl.value)
        if c == CTRL_STOPALL:
            self._effect_active = {k: False for k in self._effect_active}
            self.torque_state.clear_all_effects()
        elif c == CTRL_ENACT:
            self._device_paused = False
        elif c == CTRL_DISACT:
            self._device_paused = True
        elif c == CTRL_DEVRST:
            self._effect_active.clear()
            self._effect_start_ts.clear()
            self.torque_state.clear_all_effects()
            self._device_paused = False
        elif c == CTRL_DEVPAUSE:
            self._device_paused = True
        elif c == CTRL_DEVCONT:
            self._device_paused = False
        self._log(f"DevCtrl ctrl={c}")

    def handle_device_gain(self, pkt: ctypes.POINTER(FFB_DATA)) -> None:
        gain = ctypes.c_ubyte(0)
        if self.dll.Ffb_h_DevGain(pkt, ctypes.byref(gain)) != ERROR_SUCCESS:
            return
        self.torque_state.set_device_gain(int(gain.value))
        self._log(f"DevGain={int(gain.value)}")

    def handle_effect_new(self, pkt: ctypes.POINTER(FFB_DATA)) -> None:
        eff_type = ctypes.c_int(0)
        if self.dll.Ffb_h_EffNew(pkt, ctypes.byref(eff_type)) == ERROR_SUCCESS:
            self._log(f"EffNew type={self._effect_type_name(int(eff_type.value))}")

    def process_packet(self, pkt: ctypes.POINTER(FFB_DATA), now: float) -> None:
        ptype = ctypes.c_int(0)
        if self.dll.Ffb_h_Type(pkt, ctypes.byref(ptype)) != ERROR_SUCCESS:
            return
        t = int(ptype.value) & 0xFFFF
        self._log(f"Packet type=0x{t:04X}")

        if t == PT_EFFREP:
            self.handle_effect_report(pkt, now)
        elif t == PT_ENVREP:
            self.handle_envelope(pkt)
        elif t == PT_CONDREP:
            self.handle_condition(pkt)
        elif t == PT_PRIDREP:
            self.handle_periodic(pkt)
        elif t == PT_CONSTREP:
            self.handle_constant(pkt, now)
        elif t == PT_RAMPREP:
            self.handle_ramp(pkt)
        elif t == PT_EFOPREP:
            self.handle_operation(pkt, now)
        elif t == PT_CTRLREP:
            self.handle_device_control(pkt)
        elif t == PT_GAINREP:
            self.handle_device_gain(pkt)
        elif t == PT_BLKFRREP:
            self._log("BlockFree report received")
        elif t == PT_NEWEFREP:
            self.handle_effect_new(pkt)

    def tick(self, now: float) -> None:
        _, _, _, motion = self.torque_state.snapshot()
        if self._device_paused:
            for block in list(self._effect_active.keys()):
                self.torque_state.set_effect_contribution(block, 0.0)
            return

        for block, active in list(self._effect_active.items()):
            if not active:
                self.torque_state.set_effect_contribution(block, 0.0)
                continue

            report = self._effect_report.get(block)
            effect_type = int(self._effect_type.get(block, ET_NONE))
            duration_ms = 0xFFFF
            x_scale = 1.0
            gain_scale = 1.0
            if report is not None:
                duration_ms = int(report.Duration)
                x_scale = self._direction_x_scale(report)
                gain_scale = max(0.0, min(1.0, int(report.Gain) / 255.0))

            if effect_type == ET_NONE and block in self._constant:
                effect_type = ET_CONST
            elif effect_type == ET_NONE and block in self._ramp:
                effect_type = ET_RAMP

            elapsed_ms = max(0.0, (now - self._effect_start_ts.get(block, now)) * 1000.0)
            if duration_ms not in (0, 0xFFFF) and elapsed_ms > duration_ms:
                self._effect_active[block] = False
                self.torque_state.stop_effect_block(block)
                continue
            env_scale = self._envelope_scale(block, now, duration_ms)
            force = 0.0
            constant_base = self._constant.get(block, 0) / 10000.0

            if effect_type == ET_CONST:
                force = (constant_base * x_scale) * self.CONST_FORCE_SCALE
            elif effect_type == ET_RAMP:
                start, end = self._ramp.get(block, (0, 0))
                if duration_ms in (0, 0xFFFF):
                    ramp_mag = end
                else:
                    k = max(0.0, min(1.0, elapsed_ms / duration_ms))
                    ramp_mag = int(round(start + (end - start) * k))
                force = (ramp_mag / 10000.0) * x_scale * self.RAMP_FORCE_SCALE
            elif effect_type in (ET_SQR, ET_SINE, ET_TRNGL, ET_STUP, ET_STDN):
                per = self._periodic.get(block)
                if per is not None:
                    wave_scale = self.PERIODIC_FORCE_SCALE
                    if effect_type == ET_SINE:
                        wave_scale *= self.SINE_FORCE_SCALE
                    elif effect_type == ET_SQR:
                        wave_scale *= self.SQUARE_FORCE_SCALE
                    elif effect_type == ET_TRNGL:
                        wave_scale *= self.TRIANGLE_FORCE_SCALE
                    elif effect_type == ET_STUP:
                        wave_scale *= self.SAW_UP_FORCE_SCALE
                    elif effect_type == ET_STDN:
                        wave_scale *= self.SAW_DOWN_FORCE_SCALE

                    period_ms = max(1.0, float(per.Period))
                    phase = (2.0 * math.pi) * (elapsed_ms / period_ms)
                    phase += (2.0 * math.pi) * (float(per.Phase) / 36000.0)
                    wave = self._compute_periodic_wave(effect_type, phase)
                    mag = self._normalize_unsigned_10000(per.Magnitude) / 10000.0
                    off = self._normalize_signed_10000(per.Offset) / 10000.0
                    force = (off + wave * mag) * x_scale * wave_scale
            elif effect_type == ET_SPRNG:
                cond = self._condition_x.get(block)
                if cond is not None:
                    force = self._apply_condition_force(cond, motion["x_norm"]) * self.CONDITION_FORCE_SCALE * self.SPRING_FORCE_SCALE
            elif effect_type == ET_DMPR:
                cond = self._condition_x.get(block)
                if cond is not None:
                    force = self._apply_condition_force(cond, motion["x_vel"]) * self.CONDITION_FORCE_SCALE * self.DAMPER_FORCE_SCALE
            elif effect_type == ET_INRT:
                cond = self._condition_x.get(block)
                if cond is not None:
                    force = self._apply_condition_force(cond, motion["x_acc"]) * self.CONDITION_FORCE_SCALE * self.INERTIA_FORCE_SCALE
            elif effect_type == ET_FRCTN:
                cond = self._condition_x.get(block)
                if cond is not None:
                    signal = 0.0
                    if motion["x_vel"] > 0.0:
                        signal = 1.0
                    elif motion["x_vel"] < 0.0:
                        signal = -1.0
                    force = self._apply_condition_force(cond, signal) * self.CONDITION_FORCE_SCALE * self.FRICTION_FORCE_SCALE

            if effect_type != ET_CONST:
                force += constant_base * self.CONSTANT_OVERLAY_SCALE

            force *= gain_scale
            force *= env_scale
            force *= self.GLOBAL_FORCE_SCALE
            self.torque_state.set_effect_contribution(block, max(-1.0, min(1.0, force)))


class VJoyBridge:
    def __init__(
        self,
        dll_path: str | None,
        vjoy_id: int,
        torque_state: TorqueState,
        logger: BridgeLogger | None = None,
        debug_ffb: bool = False,
        debug_print_interval: float = 0.05,
    ) -> None:
        self.vjoy_id = vjoy_id
        self.torque_state = torque_state
        self.logger = logger
        self.debug_ffb = debug_ffb
        self.debug_print_interval = max(0.0, float(debug_print_interval))
        self._last_ffb_debug_ts = 0.0
        self.dll = self._load_dll(dll_path)
        self._setup_prototypes()
        ffb_debug_enabled = self.debug_ffb or (self.logger is not None)
        self.effects = FFBEffectsManager(self.dll, self.torque_state, self._log_ffb, ffb_debug_enabled)
        self._ffb_cb_ref = None
        self.axis_ranges: dict[int, AxisRange] = {}

    def _log_ffb(self, msg: str) -> None:
        if self.logger is not None:
            self.logger.log(f"[FFB] {msg}")

        if not self.debug_ffb:
            return
        now = time.monotonic()
        if (now - self._last_ffb_debug_ts) < self.debug_print_interval:
            return
        self._last_ffb_debug_ts = now
        print(f"[FFB] {msg}", flush=True)

    @staticmethod
    def _load_dll(explicit_path: str | None) -> ctypes.CDLL:
        candidates = []
        if explicit_path:
            candidates.append(explicit_path)
        candidates.extend(
            [
                r"C:\Program Files\vJoy\x64\vJoyInterface.dll",
                r"C:\Program Files\vJoy\vJoyInterface.dll",
                "vJoyInterface.dll",
            ]
        )

        for path in candidates:
            try:
                # vJoyInterface exports are declared as __cdecl.
                return ctypes.CDLL(path)
            except OSError:
                continue
        raise RuntimeError("No se pudo cargar vJoyInterface.dll. Ajusta --vjoy-dll.")

    def _setup_prototypes(self) -> None:
        d = self.dll

        d.vJoyEnabled.argtypes = []
        d.vJoyEnabled.restype = ctypes.c_bool

        d.AcquireVJD.argtypes = [ctypes.c_uint]
        d.AcquireVJD.restype = ctypes.c_bool

        d.RelinquishVJD.argtypes = [ctypes.c_uint]
        d.RelinquishVJD.restype = None

        d.ResetVJD.argtypes = [ctypes.c_uint]
        d.ResetVJD.restype = ctypes.c_bool

        d.SetAxis.argtypes = [ctypes.c_long, ctypes.c_uint, ctypes.c_uint]
        d.SetAxis.restype = ctypes.c_bool

        d.SetBtn.argtypes = [ctypes.c_bool, ctypes.c_uint, ctypes.c_ubyte]
        d.SetBtn.restype = ctypes.c_bool

        d.GetVJDAxisMin.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_long)]
        d.GetVJDAxisMin.restype = ctypes.c_bool

        d.GetVJDAxisMax.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_long)]
        d.GetVJDAxisMax.restype = ctypes.c_bool

        d.Ffb_h_DevGain.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(ctypes.c_ubyte)]
        d.Ffb_h_DevGain.restype = ctypes.c_uint32

        d.Ffb_h_Eff_Constant.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(FFB_EFF_CONSTANT)]
        d.Ffb_h_Eff_Constant.restype = ctypes.c_uint32

        d.Ffb_h_EffOp.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(FFB_EFF_OP)]
        d.Ffb_h_EffOp.restype = ctypes.c_uint32

        d.Ffb_h_Eff_Report.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(FFB_EFF_REPORT)]
        d.Ffb_h_Eff_Report.restype = ctypes.c_uint32

        d.Ffb_h_Eff_Ramp.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(FFB_EFF_RAMP)]
        d.Ffb_h_Eff_Ramp.restype = ctypes.c_uint32

        d.Ffb_h_Eff_Period.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(FFB_EFF_PERIOD)]
        d.Ffb_h_Eff_Period.restype = ctypes.c_uint32

        d.Ffb_h_Eff_Cond.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(FFB_EFF_COND)]
        d.Ffb_h_Eff_Cond.restype = ctypes.c_uint32

        d.Ffb_h_Eff_Envlp.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(FFB_EFF_ENVLP)]
        d.Ffb_h_Eff_Envlp.restype = ctypes.c_uint32

        d.Ffb_h_EffNew.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(ctypes.c_int)]
        d.Ffb_h_EffNew.restype = ctypes.c_uint32

        d.Ffb_h_DevCtrl.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(ctypes.c_int)]
        d.Ffb_h_DevCtrl.restype = ctypes.c_uint32

        d.Ffb_h_Type.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(ctypes.c_int)]
        d.Ffb_h_Type.restype = ctypes.c_uint32

        # FfbGenCB signature: void CALLBACK cb(PVOID data, PVOID userdata)
        self._cb_type = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)
        d.FfbRegisterGenCB.argtypes = [self._cb_type, ctypes.c_void_p]
        d.FfbRegisterGenCB.restype = None

    def start(self) -> None:
        if not self.dll.vJoyEnabled():
            raise RuntimeError("vJoy no esta habilitado.")
        if not self.dll.AcquireVJD(self.vjoy_id):
            raise RuntimeError(f"No se pudo adquirir vJoy ID {self.vjoy_id}.")
        self.dll.ResetVJD(self.vjoy_id)

        self.axis_ranges[HID_USAGE_X] = self._get_axis_range(HID_USAGE_X)
        self.axis_ranges[HID_USAGE_Y] = self._get_axis_range(HID_USAGE_Y)
        self.axis_ranges[HID_USAGE_Z] = self._get_axis_range(HID_USAGE_Z)

        def _ffb_cb(data_ptr: int, _user_data: int) -> None:
            if not data_ptr:
                return
            pkt = ctypes.cast(data_ptr, ctypes.POINTER(FFB_DATA))
            self.effects.process_packet(pkt, time.monotonic())

        self._ffb_cb_ref = self._cb_type(_ffb_cb)
        self.dll.FfbRegisterGenCB(self._ffb_cb_ref, None)

    def stop(self) -> None:
        try:
            self.dll.ResetVJD(self.vjoy_id)
            self.dll.RelinquishVJD(self.vjoy_id)
        except Exception:
            pass

    def _get_axis_range(self, usage: int) -> AxisRange:
        min_v = ctypes.c_long(0)
        max_v = ctypes.c_long(0)
        if not self.dll.GetVJDAxisMin(self.vjoy_id, usage, ctypes.byref(min_v)):
            raise RuntimeError(f"No se pudo leer minimo del eje usage=0x{usage:02X}")
        if not self.dll.GetVJDAxisMax(self.vjoy_id, usage, ctypes.byref(max_v)):
            raise RuntimeError(f"No se pudo leer maximo del eje usage=0x{usage:02X}")
        return AxisRange(min_v.value, max_v.value)

    def update_axes_buttons(self, x: int, y: int, z: int, buttons: int) -> None:
        vx = map_i16_to_axis(x, self.axis_ranges[HID_USAGE_X])
        vy = map_i16_to_axis(y, self.axis_ranges[HID_USAGE_Y])
        vz = map_i16_to_axis(z, self.axis_ranges[HID_USAGE_Z])

        self.dll.SetAxis(vx, self.vjoy_id, HID_USAGE_X)
        self.dll.SetAxis(vy, self.vjoy_id, HID_USAGE_Y)
        self.dll.SetAxis(vz, self.vjoy_id, HID_USAGE_Z)

        for i in range(32):
            pressed = bool((buttons >> i) & 1)
            self.dll.SetBtn(pressed, self.vjoy_id, i + 1)

    def tick_effects(self, now: float) -> None:
        self.effects.tick(now)


def crc_xor(payload: bytes) -> int:
    crc = 0
    for b in payload:
        crc ^= b
    return crc & 0xFF


def map_i16_to_axis(value: int, axis_range: AxisRange) -> int:
    value = max(-32768, min(32767, int(value)))
    span = axis_range.max_val - axis_range.min_val
    norm = (value + 32768) / 65535.0
    mapped = axis_range.min_val + int(round(norm * span))
    return max(axis_range.min_val, min(axis_range.max_val, mapped))


def build_torque_packet(seq: int, torque: int, gain: int = 255, flags: int = 0) -> bytes:
    torque = max(-32768, min(32767, int(torque)))
    gain = max(0, min(255, int(gain)))
    flags = max(0, min(255, int(flags)))

    msg = bytearray(TORQUE_LEN)
    msg[0] = BRIDGE_START
    msg[1] = PKT_TORQUE
    msg[2] = seq & 0xFF
    msg[3] = torque & 0xFF
    msg[4] = (torque >> 8) & 0xFF
    msg[5] = gain
    msg[6] = flags
    msg[7] = crc_xor(msg[:-1])
    return bytes(msg)


def build_control_packet(seq: int, cmd: int, value: int = 0) -> bytes:
    cmd = max(0, min(255, int(cmd)))
    value = max(-32768, min(32767, int(value)))

    msg = bytearray(CONTROL_LEN)
    msg[0] = BRIDGE_START
    msg[1] = PKT_CONTROL
    msg[2] = seq & 0xFF
    msg[3] = cmd
    msg[4] = value & 0xFF
    msg[5] = (value >> 8) & 0xFF
    msg[6] = crc_xor(msg[:-1])
    return bytes(msg)


def send_startup_command(
    ser: serial.Serial,
    startup_flags: int,
    logger: BridgeLogger | None,
    retries: int = 3,
    retry_delay: float = 0.05,
) -> None:
    packet = build_control_packet(0, CMD_STARTUP, startup_flags)
    for attempt in range(retries):
        ser.write(packet)
        if logger and logger.path:
            logger.log(
                f"[BOOT] startup_cmd attempt={attempt + 1}/{retries} flags=0x{startup_flags:04X} pkt={packet.hex(' ')}"
            )
        time.sleep(retry_delay)


def parse_telemetry_frame(frame: bytes) -> tuple[int, int, int, int, int]:
    if len(frame) != TELEMETRY_LEN:
        raise ValueError("telemetry len invalido")
    if frame[0] != BRIDGE_START or frame[1] != PKT_TELEMETRY:
        raise ValueError("cabecera invalida")
    if crc_xor(frame[:-1]) != frame[-1]:
        raise ValueError("crc invalido")

    seq = frame[2]
    x = int.from_bytes(frame[3:5], "little", signed=True)
    y = int.from_bytes(frame[5:7], "little", signed=True)
    z = int.from_bytes(frame[7:9], "little", signed=True)
    buttons = int.from_bytes(frame[9:13], "little", signed=False)
    return seq, x, y, z, buttons


def open_serial(args: argparse.Namespace) -> serial.Serial:
    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=0.005,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )

    # Avoid toggling auto-reset lines repeatedly on some USB-UART bridges.
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass

    if args.startup_delay > 0:
        time.sleep(args.startup_delay)

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass

    return ser


def run(args: argparse.Namespace) -> int:
    logger = BridgeLogger(args.log_file)
    torque_state = TorqueState(
        max_torque=FFBEffectsManager.MAX_TORQUE,
        min_torque=FFBEffectsManager.MIN_TORQUE,
        use_min_torque_comp=FFBEffectsManager.USE_MIN_TORQUE_COMP,
        input_deadzone=FFBEffectsManager.INPUT_DEADZONE,
        min_torque_input=FFBEffectsManager.MIN_TORQUE_INPUT,
        torque_response_gamma=FFBEffectsManager.TORQUE_RESPONSE_GAMMA,
    )
    vjoy = VJoyBridge(
        args.vjoy_dll,
        args.vjoy_id,
        torque_state,
        logger=logger,
        debug_ffb=args.debug_ffb,
        debug_print_interval=args.debug_print_interval,
    )
    ser = open_serial(args)

    stop_event = threading.Event()

    def _sig_handler(_sig: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    vjoy.start()

    startup_flags = 0
    if args.auto_calib:
        startup_flags |= STARTUP_FLAG_AUTO_CALIB

    send_startup_command(ser, startup_flags, logger)

    print(f"Bridge iniciado. Serial={args.port} vJoyID={args.vjoy_id}")
    print(f"Startup flags enviados al ESP: 0x{startup_flags:04X}", flush=True)
    print(
        "[TUNE] "
        f"MAX_TORQUE={torque_state.max_torque} "
        f"MIN_COMP={'on' if torque_state.use_min_torque_comp else 'off'} "
        f"MIN_TORQUE={torque_state.min_torque} "
        f"DEADZONE={torque_state.input_deadzone} "
        f"MIN_INPUT={torque_state.min_torque_input} "
        f"GAMMA={torque_state.torque_response_gamma}",
        flush=True,
    )
    if logger.path:
        print(f"Logging detallado en: {logger.path}")
        logger.log(f"[RUN] start serial={args.port} baud={args.baud} vjoy_id={args.vjoy_id}")

    rx = bytearray()
    tx_seq = 0
    last_print = time.monotonic()
    last_axes = (0, 0, 0)
    last_tx_debug = 0.0
    forced_torque = None
    if args.force_torque is not None:
        forced_torque = max(-32768, min(32767, int(args.force_torque)))
        print(f"[TEST] Modo torque fijo activo: torque={forced_torque}", flush=True)
        if logger.path:
            logger.log(f"[TEST] constant_torque_enabled torque={forced_torque}")

    try:
        while not stop_event.is_set():
            chunk = ser.read(256)
            if chunk:
                rx.extend(chunk)

            # Resync parser on start byte.
            while len(rx) >= TELEMETRY_LEN:
                if rx[0] != BRIDGE_START:
                    del rx[0]
                    continue
                if len(rx) < TELEMETRY_LEN:
                    break

                frame = bytes(rx[:TELEMETRY_LEN])
                try:
                    _seq, x, y, z, buttons = parse_telemetry_frame(frame)
                    torque_state.update_motion(x)
                    vjoy.update_axes_buttons(x, y, z, buttons)
                    last_axes = (x, y, z)
                    if logger.path:
                        logger.log(
                            "[RX] "
                            f"seq={_seq:3d} x={x:6d} y={y:6d} z={z:6d} "
                            f"buttons=0x{buttons:08X} raw={frame.hex(' ')}"
                        )
                except ValueError:
                    if logger.path:
                        logger.log(f"[RX_ERR] invalid telemetry frame raw={frame.hex(' ')}")
                    del rx[0]
                    continue
                del rx[:TELEMETRY_LEN]

            now = time.monotonic()
            vjoy.tick_effects(now)
            if forced_torque is None:
                torque = torque_state.compute_torque()
            else:
                torque = forced_torque
            tx_pkt = build_torque_packet(tx_seq, torque, 255, 0)
            ser.write(tx_pkt)

            if logger.path:
                dev_gain, eff_force, eff_active, motion = torque_state.snapshot()
                logger.log(
                    "[TX] "
                    f"seq={tx_seq:3d} torque={torque:5d} gain={dev_gain:3d} "
                    f"pkt={tx_pkt.hex(' ')} "
                    f"effects_force={ {k: round(v, 4) for k, v in eff_force.items()} } "
                    f"active={eff_active} "
                    f"motion={ {k: round(v, 4) for k, v in motion.items()} }"
                )

            tx_seq = (tx_seq + 1) & 0xFF

            if args.debug_tx:
                if (now - last_tx_debug) >= args.debug_print_interval:
                    dev_gain, eff_force, eff_active, motion = torque_state.snapshot()
                    print(
                        "[TX] "
                        f"torque={torque:5d} gain={dev_gain:3d} "
                        f"pkt={tx_pkt.hex(' ')} "
                        f"effects_force={ {k: round(v, 4) for k, v in eff_force.items()} } "
                        f"active={eff_active} "
                        f"motion={ {k: round(v, 4) for k, v in motion.items()} }",
                        flush=True,
                    )
                    last_tx_debug = now

            if args.verbose and now - last_print >= 0.2:
                print(
                    f"axes=({last_axes[0]:6d},{last_axes[1]:6d},{last_axes[2]:6d}) "
                    f"torque={torque:5d}",
                    flush=True,
                )
                last_print = now

            time.sleep(args.loop_sleep)

    finally:
        try:
            ser.close()
        except Exception:
            pass
        vjoy.stop()
        if logger.path:
            logger.log("[RUN] stop")
        logger.close()
        print("Bridge detenido.")

    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="ESP Driving Simulator serial <-> vJoy FFB bridge")
    p.add_argument("--port", required=True, help="Puerto COM del ESP (ej: COM4)")
    p.add_argument("--baud", type=int, default=115200, help="Baudrate serial")
    p.add_argument("--vjoy-id", type=int, default=1, help="ID del dispositivo vJoy")
    p.add_argument("--vjoy-dll", default=None, help="Ruta a vJoyInterface.dll")
    p.add_argument("--loop-sleep", type=float, default=0.002, help="Sleep del loop principal (s)")
    p.add_argument("--startup-delay", type=float, default=1.2, help="Espera tras abrir COM para evitar resets (s)")
    p.add_argument("--debug-ffb", action="store_true", help="Imprime paquetes/estado FFB que llegan desde vJoy")
    p.add_argument("--debug-tx", action="store_true", help="Imprime paquetes de torque enviados al ESP")
    p.add_argument("--log-file", default=None, help="Archivo log detallado (sin throttling). Ej: log.txt")
    p.add_argument("--debug-print-interval", type=float, default=0.05, help="Intervalo minimo entre prints de debug (s)")
    p.add_argument("--verbose", action="store_true", help="Imprime telemetria/torque cada 200ms")
    p.add_argument("--force-torque", type=int, default=None, help="Si se define, ignora FFB y envia este torque constante (int16) al ESP")
    p.add_argument("--auto-calib", action="store_true", help="Pide al ESP ejecutar calibracion completa de pedales al arrancar")
    return p


if __name__ == "__main__":
    parser = build_arg_parser()
    sys.exit(run(parser.parse_args()))
