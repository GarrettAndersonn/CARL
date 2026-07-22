# AVL_CARL Firmware

PlatformIO monorepo for the three CAN nodes.

**Phase 1 (differential drive) is implemented:** master + throttle nodes are functional;
the steering node is a **safe-idle stub** (board unpowered, kart turns via differential
drive). See [`../docs/wiring.md`](../docs/wiring.md) for the pin-level wiring and
[`../docs/can-bus.md`](../docs/can-bus.md) for the locked message map.

## Quick start (Phase 1 teleop)

1. Flash the master (ST-LINK) and throttle (Teensy Loader):
   `pio run -e master -t upload` and `pio run -e throttle -t upload`.
2. With wheels **off the ground**, run the keyboard teleop against the master's USB serial:
   ```bash
   pip install pyserial
   python test/teleop.py --port COM5      # use --list to find the port
   ```
   `1`–`4` set speed, `w/a/s/d` drive, `c` center, `x`/SPACE stop, `e`/`r` e-stop/reset.
3. Run host unit tests for the protocol (no hardware): `pio test -e native`.

## Build / flash

```bash
pio run -e throttle      # build/flash ONLY the throttle node
pio run -e master        # build/flash ONLY the master node
pio run -e steering      # build/flash ONLY the steering node
pio run                  # build ALL three nodes
```

> `pio run -e throttle` builds/flashes just that node; `pio run` builds all three.

Add `-t upload` to flash a connected board (e.g. `pio run -e throttle -t upload`),
and `pio run -e throttle -t monitor` (or `pio device monitor`) for the serial console.

## Layout

```
firmware/
├── platformio.ini       # one [env] per node — builds all 3 from one config
├── lib/                 # SHARED code (single source of truth across nodes)
│   ├── carl_protocol/   # CAN IDs, node IDs, bitrate, heartbeat/e-stop constants
│   │   ├── carl_protocol.h
│   │   └── messages.h   # struct <-> CAN frame (de)serialization
│   ├── heartbeat/       # shared heartbeat + watchdog fail-safe logic
│   └── common/          # shared types, utils, fixed-point helpers
├── master/   src/main.cpp   include/   # NUCLEO-F767ZI — VCU / arbiter
├── throttle/ src/main.cpp   include/   # Teensy 4.1 — 4-motor drive + encoders
├── steering/ src/main.cpp   include/   # Teensy 4.1 — CL57T stepper
└── test/                # host-run unit tests (e.g. message pack/unpack)
```

## Notes

- **Shared `lib/` is the point.** The CAN message definitions in `carl_protocol/` are the
  single source of truth — every node depends on them so senders and receivers can't drift.
  Keep these in sync with [`../docs/can-bus.md`](../docs/can-bus.md).
- **Two chip families, one build system.** Master = STM32 (`platform = ststm32`),
  throttle & steering = Teensy 4.1 (`platform = teensy`). PlatformIO builds both from this
  one `platformio.ini`.
- The `framework = arduino` lines are a sensible default for bring-up; the master can be
  switched to the STM32Cube/HAL framework later if needed.
- **CAN libraries:** Teensy nodes use the core-bundled `FlexCAN_T4`; the STM32 master uses
  `pazi88/STM32_CAN` (pulled in via `lib_deps`). The throttle node uses
  `paulstoffregen/Encoder` for the rear wheel quadrature.
- **Boot/flash protocols differ per chip** — Teensy uses the HalfKay bootloader (Teensy
  Loader; tap the button if it doesn't auto-reboot), the NUCLEO uses on-board ST-LINK (SWD).
  PlatformIO handles both via `-t upload`.
