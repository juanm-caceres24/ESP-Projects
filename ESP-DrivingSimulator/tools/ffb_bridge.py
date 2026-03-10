#!/usr/bin/env python3
"""ESP wheel bridge: Serial telemetry <-> vJoy axes/buttons <-> FFB torque back to ESP.

Protocol (little-endian):
- ESP -> PC telemetry (14 bytes):
  [0]=0xAB [1]=0x10 [2]=seq [3:5]=x [5:7]=y [7:9]=z [9:13]=buttons_u32 [13]=crc_xor
- PC -> ESP torque command (8 bytes):
  [0]=0xAB [1]=0x20 [2]=seq [3:5]=torque_i16 [5]=gain_u8 [6]=flags [7]=crc_xor
- PC -> ESP heartbeat (4 bytes):
  [0]=0xAB [1]=0x21 [2]=seq [3]=crc_xor
"""

from __future__ import annotations

import argparse
import ctypes
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass

import serial


BRIDGE_START = 0xAB
PKT_TELEMETRY = 0x10
PKT_TORQUE = 0x20
PKT_HEARTBEAT = 0x21

TELEMETRY_LEN = 14
TORQUE_LEN = 8
HEARTBEAT_LEN = 4

HID_USAGE_X = 0x30
HID_USAGE_Y = 0x31
HID_USAGE_Z = 0x32

ERROR_SUCCESS = 0
EFF_STOP = 3


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


class FFB_EFF_REPORT(ctypes.Structure):
    _fields_ = [
        ("EffectBlockIndex", ctypes.c_ubyte),
        ("EffectType", ctypes.c_int),
        ("Duration", ctypes.c_uint16),
        ("TrigerRpt", ctypes.c_uint16),
        ("SamplePrd", ctypes.c_uint16),
        ("Gain", ctypes.c_ubyte),
        ("TrigerBtn", ctypes.c_ubyte),
        ("Polar", ctypes.c_bool),
        ("Direction", ctypes.c_ubyte),
        ("DirY", ctypes.c_ubyte),
    ]


@dataclass
class AxisRange:
    min_val: int
    max_val: int


class TorqueState:
    def __init__(self, max_torque: int) -> None:
        self.max_torque = max_torque
        self.device_gain = 255
        self._effect_x_scale: dict[int, float] = {}
        self._effect_magnitude: dict[int, int] = {}
        self._effect_active: dict[int, bool] = {}
        self._lock = threading.Lock()

    def set_device_gain(self, gain: int) -> None:
        with self._lock:
            self.device_gain = max(0, min(255, int(gain)))

    def set_constant(self, magnitude: int) -> None:
        self.set_constant_for_block(0, magnitude)

    def set_effect_direction_polar(self, block_index: int, dir_byte: int) -> None:
        # vJoy docs: 0..255 -> 0..360 deg. X axis is cosine component.
        angle_deg = (int(dir_byte) & 0xFF) * (360.0 / 255.0)
        x_scale = math.cos(math.radians(angle_deg))
        with self._lock:
            self._effect_x_scale[int(block_index)] = x_scale

    def set_effect_direction_cartesian_x(self, block_index: int, dir_x_byte: int) -> None:
        raw = int(dir_x_byte) & 0xFF
        signed = raw - 256 if raw >= 128 else raw
        x_scale = max(-1.0, min(1.0, signed / 127.0 if signed != -128 else -1.0))
        with self._lock:
            self._effect_x_scale[int(block_index)] = x_scale

    def set_constant_for_block(self, block_index: int, magnitude: int) -> None:
        with self._lock:
            self._effect_magnitude[int(block_index)] = max(-10000, min(10000, int(magnitude)))
            self._effect_active[int(block_index)] = True

    def stop_effect_block(self, block_index: int) -> None:
        with self._lock:
            self._effect_active[int(block_index)] = False

    def snapshot(self) -> tuple[int, dict[int, int], dict[int, float], dict[int, bool]]:
        with self._lock:
            return (
                int(self.device_gain),
                dict(self._effect_magnitude),
                dict(self._effect_x_scale),
                dict(self._effect_active),
            )

    def stop_effects(self) -> None:
        with self._lock:
            self._effect_active.clear()
            self._effect_magnitude.clear()
            self._effect_x_scale.clear()

    def compute_torque(self) -> int:
        with self._lock:
            summed = 0.0
            for block, mag in self._effect_magnitude.items():
                if not self._effect_active.get(block, False):
                    continue
                x_scale = self._effect_x_scale.get(block, 1.0)
                summed += (mag / 10000.0) * x_scale

            # Keep total effect bounded before max_torque scaling.
            summed = max(-1.0, min(1.0, summed))
            scaled = summed * self.max_torque
            scaled *= self.device_gain / 255.0
        torque = int(round(scaled))

        # Invert output sign to match this wheel's motor/driver direction.
        torque = -torque
        return max(-self.max_torque, min(self.max_torque, torque))


class VJoyBridge:
    def __init__(
        self,
        dll_path: str | None,
        vjoy_id: int,
        torque_state: TorqueState,
        debug_ffb: bool = False,
        debug_print_interval: float = 0.05,
    ) -> None:
        self.vjoy_id = vjoy_id
        self.torque_state = torque_state
        self.debug_ffb = debug_ffb
        self.debug_print_interval = max(0.0, float(debug_print_interval))
        self._last_ffb_debug_ts = 0.0
        self.dll = self._load_dll(dll_path)
        self._setup_prototypes()
        self._ffb_cb_ref = None
        self.axis_ranges: dict[int, AxisRange] = {}

    def _log_ffb(self, msg: str) -> None:
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

    @staticmethod
    def _normalize_ffb_magnitude(raw_magnitude: int) -> int:
        mag = int(raw_magnitude)

        # Defensive decode: some environments expose signed values wrapped in 16-bit unsigned form.
        if mag > 32767:
            mag -= 65536

        # Keep vJoy documented range.
        if mag > 10000:
            mag = 10000
        elif mag < -10000:
            mag = -10000
        return mag

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

            gain = ctypes.c_ubyte(0)
            if self.dll.Ffb_h_DevGain(pkt, ctypes.byref(gain)) == ERROR_SUCCESS:
                self.torque_state.set_device_gain(gain.value)
                self._log_ffb(f"DevGain={gain.value}")

            const_eff = FFB_EFF_CONSTANT()
            if self.dll.Ffb_h_Eff_Constant(pkt, ctypes.byref(const_eff)) == ERROR_SUCCESS:
                mag = self._normalize_ffb_magnitude(const_eff.Magnitude)
                self.torque_state.set_constant_for_block(const_eff.EffectBlockIndex, mag)
                self._log_ffb(
                    f"Const block={const_eff.EffectBlockIndex} mag_raw={const_eff.Magnitude} mag={mag}"
                )

            eff_report = FFB_EFF_REPORT()
            eff_report_res = self.dll.Ffb_h_Eff_Report(pkt, ctypes.byref(eff_report))
            if eff_report_res == ERROR_SUCCESS:
                if bool(eff_report.Polar):
                    self.torque_state.set_effect_direction_polar(eff_report.EffectBlockIndex, eff_report.Direction)
                    self._log_ffb(
                        f"EffReport block={eff_report.EffectBlockIndex} polar dirByte={eff_report.Direction} gain={eff_report.Gain}"
                    )
                else:
                    self.torque_state.set_effect_direction_cartesian_x(eff_report.EffectBlockIndex, eff_report.Direction)
                    self._log_ffb(
                        f"EffReport block={eff_report.EffectBlockIndex} cart dirX={eff_report.Direction} gain={eff_report.Gain}"
                    )
            elif self.debug_ffb:
                self._log_ffb(f"EffReport not available (res={eff_report_res})")

            eff_op = FFB_EFF_OP()
            if self.dll.Ffb_h_EffOp(pkt, ctypes.byref(eff_op)) == ERROR_SUCCESS:
                if int(eff_op.EffectOp) == EFF_STOP:
                    self.torque_state.stop_effect_block(eff_op.EffectBlockIndex)
                self._log_ffb(
                    f"EffOp block={eff_op.EffectBlockIndex} op={int(eff_op.EffectOp)} loops={eff_op.LoopCount}"
                )

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


def build_heartbeat_packet(seq: int) -> bytes:
    msg = bytearray(HEARTBEAT_LEN)
    msg[0] = BRIDGE_START
    msg[1] = PKT_HEARTBEAT
    msg[2] = seq & 0xFF
    msg[3] = crc_xor(msg[:-1])
    return bytes(msg)


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
        write_timeout=args.write_timeout,
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
    torque_state = TorqueState(max_torque=args.max_torque)
    vjoy = VJoyBridge(
        args.vjoy_dll,
        args.vjoy_id,
        torque_state,
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
    print(f"Bridge iniciado. Serial={args.port} vJoyID={args.vjoy_id}")

    rx = bytearray()
    tx_seq = 0
    last_hb = time.monotonic()
    last_print = time.monotonic()
    last_axes = (0, 0, 0)
    consecutive_write_failures = 0
    last_tx_debug = 0.0

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
                    vjoy.update_axes_buttons(x, y, z, buttons)
                    last_axes = (x, y, z)
                except ValueError:
                    del rx[0]
                    continue
                del rx[:TELEMETRY_LEN]

            torque = torque_state.compute_torque()
            try:
                tx_pkt = build_torque_packet(tx_seq, torque, 255, 0)
                ser.write(tx_pkt)
                tx_seq = (tx_seq + 1) & 0xFF
                consecutive_write_failures = 0

                if args.debug_tx:
                    now_dbg = time.monotonic()
                    if (now_dbg - last_tx_debug) >= args.debug_print_interval:
                        dev_gain, eff_mag, eff_dir, eff_active = torque_state.snapshot()
                        print(
                            "[TX] "
                            f"torque={torque:5d} gain={dev_gain:3d} "
                            f"pkt={tx_pkt.hex(' ')} "
                            f"effects_mag={eff_mag} effects_dir={ {k: round(v, 3) for k, v in eff_dir.items()} } active={eff_active}",
                            flush=True,
                        )
                        last_tx_debug = now_dbg
            except (serial.SerialTimeoutException, serial.SerialException) as exc:
                consecutive_write_failures += 1
                if args.verbose:
                    print(f"Serial write fallo ({consecutive_write_failures}): {exc}", flush=True)
                if consecutive_write_failures >= args.reopen_after_failures:
                    if args.verbose:
                        print("Reabriendo puerto serial...", flush=True)
                    try:
                        ser.close()
                    except Exception:
                        pass
                    time.sleep(0.3)
                    ser = open_serial(args)
                    consecutive_write_failures = 0
                time.sleep(0.01)
                continue

            now = time.monotonic()
            if now - last_hb >= 0.5:
                try:
                    ser.write(build_heartbeat_packet(tx_seq))
                    tx_seq = (tx_seq + 1) & 0xFF
                    last_hb = now
                except (serial.SerialTimeoutException, serial.SerialException):
                    pass

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
        print("Bridge detenido.")

    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="ESP Driving Simulator serial <-> vJoy FFB bridge")
    p.add_argument("--port", required=True, help="Puerto COM del ESP (ej: COM4)")
    p.add_argument("--baud", type=int, default=115200, help="Baudrate serial")
    p.add_argument("--vjoy-id", type=int, default=1, help="ID del dispositivo vJoy")
    p.add_argument("--vjoy-dll", default=None, help="Ruta a vJoyInterface.dll")
    p.add_argument("--max-torque", type=int, default=1000, help="Torque maximo enviado al ESP")
    p.add_argument("--loop-sleep", type=float, default=0.002, help="Sleep del loop principal (s)")
    p.add_argument("--write-timeout", type=float, default=0.25, help="Timeout de escritura serial (s)")
    p.add_argument("--startup-delay", type=float, default=1.2, help="Espera tras abrir COM para evitar resets (s)")
    p.add_argument("--reopen-after-failures", type=int, default=8, help="Reabrir COM tras N fallos de escritura")
    p.add_argument("--debug-ffb", action="store_true", help="Imprime paquetes/estado FFB que llegan desde vJoy")
    p.add_argument("--debug-tx", action="store_true", help="Imprime paquetes de torque enviados al ESP")
    p.add_argument("--debug-print-interval", type=float, default=0.05, help="Intervalo minimo entre prints de debug (s)")
    p.add_argument("--verbose", action="store_true", help="Imprime telemetria/torque cada 200ms")
    return p


if __name__ == "__main__":
    parser = build_arg_parser()
    sys.exit(run(parser.parse_args()))
