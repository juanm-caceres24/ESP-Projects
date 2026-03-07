# FFB Bridge (Python)

This bridge sends force commands from a PC app/process to the wheel firmware
using HID Output report `0x20`.

## Why this approach

- Keeps the HID gamepad descriptor clean and stable (`joy.cpl` friendly).
- Avoids relying on HID PID/DirectInput FFB recognition in games.
- Lets you iterate FFB logic from PC-side software quickly.

## Prerequisites

1. Flash firmware with stable descriptor mode:
   - `-DUSE_PID_DESCRIPTOR=0`
2. Connect the ESP game USB port to the game PC.
3. Install Python package:

```bash
pip install hidapi
```

## Quick test (sine force)

```bash
python tools/ffb_bridge.py sine
```

If this works, the wheel should oscillate smoothly.

## Assist mode (PID by effect)

This mode reads the wheel axis from the HID input report and generates:

- center spring effect (PID over position)
- damper effect (PID over velocity)
- friction effect (PID over low-speed velocity + breakaway torque)
- optional inertia effect (PID over acceleration)
- endwall effect (PID outside steering range)

It is not game-native FFB, but it is usable for driving tests and now all
constants are edited directly in `tools/ffb_bridge.py`.

```bash
python tools/ffb_bridge.py assist
```

Useful runtime knobs:

```bash
python tools/ffb_bridge.py --gain 170 assist --strength 1.20
python tools/ffb_bridge.py --gain 180 assist --endstop-deg 390 --units-per-deg 115
```

If center is mechanically off, add `--center-offset` (for example `--center-offset -120`).

To debug wheel position decoding before force feedback, use:

```bash
python tools/ffb_bridge.py axis
```

If values are wrong, force the decoding mode:

```bash
python tools/ffb_bridge.py axis --axis-mode rid
python tools/ffb_bridge.py axis --axis-mode legacy
```

Direct code tuning:

- Open `tools/ffb_bridge.py` and edit constants at the top.
- Core PID constants: `K_P_CENTER`, `K_I_CENTER`, `K_D_CENTER`.
- Effect toggles: `ENABLE_EFFECT_SPRING`, `ENABLE_EFFECT_DAMPER`, `ENABLE_EFFECT_FRICTION`, `ENABLE_EFFECT_INERTIA`, `ENABLE_EFFECT_ENDWALLS`.
- Endwall setup: `ENDWALL_LIMIT_DEG`, `ENDWALL_BAND_DEG`, `K_P_ENDWALL`.
- Sensor scaling/filtering: `UNITS_PER_DEG`, `POS_LPF_MS`, `VEL_LPF_MS`, `ACC_LPF_MS`.
- `auto-center-samples`: startup calibration for wheel center. Keep wheel centered and still for ~1 second while it calibrates.

## UDP bridge mode

Run bridge:

```bash
python tools/ffb_bridge.py udp
```

Send packets from your game-side app/mod:

- Plain integer torque:

```text
120
```

- JSON payload:

```json
{"torque": 120, "gain": 220}
```

## Notes

- Torque range is `-1000..1000`.
- `gain` range is `0..255`.
- Bridge continuously updates force; firmware safety timeout stops motor if data stops.
- Start with conservative values (`gain <= 160`) and increase gradually.
