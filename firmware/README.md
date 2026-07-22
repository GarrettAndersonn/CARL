<div align="center">

# AVL_CARL

**A**utonomous **V**ehicle **L**ab — **C**ontrol **A**rchitecture for **R**obotic **L**ocomotion

*A drive-by-wire electric go-kart platform built toward full autonomy.*

![Status](https://img.shields.io/badge/status-early%20development-orange)
![Phase](https://img.shields.io/badge/phase-1%20drive--by--wire-blue)
![Bus](https://img.shields.io/badge/bus-CAN%202.0-informational)
![License](https://img.shields.io/badge/license-TBD-lightgrey)

</div>

---

## Overview

**AVL_CARL** is an electric go-kart developed by the Autonomous Vehicle Lab as a
distributed, drive-by-wire (DBW) robotics platform. The vehicle is built in two phases:

| Phase | Goal | Status |
|-------|------|--------|
| **1 — Drive-by-wire** | Fully electronic throttle and steering control over a CAN bus, no mechanical linkages | 🔧 In progress |
| **2 — Autonomy** | Add perception, localization, and planning via a companion compute stack on the master node | 🗓️ Planned |

The system is architected as a set of independent **CAN nodes**, each a microcontroller
responsible for one subsystem (vehicle control, propulsion, steering). This mirrors
automotive ECU network design and keeps each subsystem testable in isolation.

> ⚠️ **Safety note — no mechanical brakes.** This vehicle has **no brake actuator**.
> The platform decelerates by commanding **zero throttle and holding zero** to the
> traction motors. All operation must occur in a controlled environment with a hardware
> kill switch / power cutoff within reach. See [Safety](#safety).

---

## Table of Contents

- [System Architecture](#system-architecture)
- [Node / Microcontroller Assignment](#node--microcontroller-assignment)
- [Hardware](#hardware)
- [Communication — CAN Bus](#communication--can-bus)
- [Electrical Integration Notes](#electrical-integration-notes)
- [Safety](#safety)
- [Repository Structure](#repository-structure)
- [Toolchain & Getting Started](#toolchain--getting-started)
- [Roadmap](#roadmap)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)

---

## System Architecture

The vehicle is a **distributed control system** built on a single classic-CAN backbone.
A central **Vehicle Control Unit (VCU / Master)** receives operator (and, later, autonomy)
commands and arbitrates them onto the bus. Edge nodes execute those commands and report
feedback.

```
                       ┌───────────────────────────────────┐
                       │   Companion compute (Phase 2)      │
                       │   perception · planning            │
                       └───────────────┬───────────────────┘
                                       │ Ethernet (future)
                       ┌───────────────┴───────────────────┐
                       │        MASTER / VCU                │
                       │      NUCLEO-F767ZI                 │
                       │  command arbiter · heartbeat ·     │
                       │  watchdog · autonomy interface     │
                       └───────────────┬───────────────────┘
                                       │
   ════════════════════════ CAN 2.0 BUS (120 Ω terminated) ════════════════════════
            │                                                   │
   ┌────────┴───────────┐                            ┌──────────┴──────────┐
   │  THROTTLE / DRIVE  │                            │      STEERING       │
   │     Teensy 4.1     │                            │      Teensy 4.1     │
   │  4× IBT_2 (8 PWM)· │                            │  step/dir/enable →  │
   │  wheel-encoder     │                            │  CL57T stepper      │
   │  quadrature decode │                            │  (Ackermann)        │
   └────────┬───────────┘                            └──────────┬──────────┘
            │                                                    │
   ┌────────┴───────────┐                            ┌──────────┴──────────┐
   │ 4× IBT_2 → MY4845  │                            │ Closed-loop stepper │
   │ 2× rear encoders   │                            │ trapezoidal linkage │
   └────────────────────┘                            └─────────────────────┘
```

### Design principles

- **One subsystem per node.** Each microcontroller owns one safety-relevant function.
- **Command/feedback over CAN.** No analog control wiring between subsystems.
- **Fail-safe by default.** Loss of the master heartbeat causes edge nodes to command
  zero (motors → 0, steering → hold), consistent with the no-brake stop strategy.
- **Headroom for autonomy.** The master is the most capable board, reserved for the
  future compute/Ethernet interface.

---

## Node / Microcontroller Assignment

Inventory: **2× Teensy 4.1** and **1× STM32 NUCLEO-F767ZI** — one board per node, **no spare**.
Assignment is driven by each node's I/O and compute needs (see
[`docs/architecture.md`](docs/architecture.md) for the full rationale).

| Node | Board | Why this board |
|------|-------|----------------|
| **Master / VCU** | **NUCLEO-F767ZI** | On-board Ethernet PHY + RJ45 for the future autonomy/companion-computer link; Cortex-M7 + FPU compute headroom; bxCAN; large GPIO breakout; industrial toolchain. |
| **Throttle / Motor-drive** | **Teensy 4.1** | Most I/O-heavy node. Hardware quadrature decoders for the wheel encoders + abundant FlexPWM channels for 4 motor drivers; 600 MHz Cortex-M7 with tightly-coupled memory for deterministic real-time control; 3× FlexCAN. |
| **Steering** | **Teensy 4.1** | Lowest-complexity node: 3 GPIO for CL57T `step/dir/enable` + optional position feedback; clean timer-based step generation; same Teensyduino toolchain as the throttle node. |

> **Note:** Inventory confirmed 2026-06-11 — 1× NUCLEO + 2× Teensy 4.1, all in use. A future
> 4th node (BMS / sensor hub / redundancy) needs an additional board. See
> [Caveats](docs/architecture.md#caveats).

---

## Hardware

Full bill of materials, part numbers, ratings, and pinouts live in
[`docs/hardware.md`](docs/hardware.md). Summary:

### Propulsion
| Item | Part / Spec | Notes |
|------|-------------|-------|
| Traction motors | **MY4845**, 24 V, 150 W, ~2300 rpm | One per wheel (4 total), each coupled to a gearbox. |
| Motor count | 4 (one per wheel) | Independent per-wheel drive. |
| Drivers | **IBT_2** (BTS7960 dual H-bridge), 4× | One per motor. Bidirectional; 2 PWM lines each (8 total) + enable, commanded by the throttle node. |

### Steering
| Item | Part / Spec | Notes |
|------|-------------|-------|
| Geometry | Trapezoidal **Ackermann** | Mechanical linkage. |
| Actuator | Stepper motor via **CL57T** closed-loop driver | `step/dir/enable` interface, 24 V actuator rail. |

### Sensing
| Item | Part / Spec | Notes |
|------|-------------|-------|
| Wheel encoders | **E38S6-600-24G** (NO:1001119) | Incremental rotary, **600 PPR**, quadrature A/B, 5–24 V. Currently on the **2 rear wheels** (more TBD). |
| Encoder wiring | white = OUT A · green = OUT B · red = VCC · black = 0 V | ⚠️ 5–24 V outputs — requires level shifting into 3.3 V MCU pins. |

### Compute
| Item | Part / Spec | Notes |
|------|-------------|-------|
| Microcontrollers | 2× **Teensy 4.1** + 1× **NUCLEO-F767ZI** | See node assignment above. |

### Power
| Item | Part / Spec | Notes |
|------|-------------|-------|
| System voltage | **24 V** | Main propulsion + actuator rail. |
| Actuator rail | 24 V | Powers the CL57T steering driver. |

> ⚠️ **To confirm:** *MY4845* is a 24 V DC **motor**, not the battery. The main **battery
> pack** is a separate item still to be documented. See [`docs/hardware.md`](docs/hardware.md).

---

## Communication — CAN Bus

- **Standard:** Classic **CAN 2.0** (the Teensy FlexCAN and STM32 bxCAN controllers are
  classic-CAN only — no CAN-FD on this hardware).
- **Recommended bitrate:** 500 kbit/s (revisit for final bus length).
- **Topology:** Single linear bus, **120 Ω termination at both physical ends**.
- **Transceivers:** Adafruit **TJA1051T/3** ×3 (in hand) — 3.3 V logic, on-board switchable
  120 Ω termination (enable on the two end nodes only).
- **Message priority:** 11-bit IDs, lower ID = higher priority. E-stop / heartbeat get the
  lowest IDs (highest priority), then steering/throttle commands, then telemetry/feedback.

The full bitrate, node-ID allocation, and message map are maintained in
[`docs/can-bus.md`](docs/can-bus.md).

---

## Electrical Integration Notes

These are the known integration gotchas to resolve during bring-up (details in
[`docs/hardware.md`](docs/hardware.md)):

- **Encoder level shifting (critical).** E38S6 outputs swing 5–24 V; MCU GPIO is 3.3 V.
  Power the encoder at 5 V and use pull-ups / a divider to a clean 3.3 V logic swing
  **before** connecting to the MCU.
- **CL57T input levels.** The CL57T uses optocoupler `step/dir/enable` inputs. Verify the
  driver version / mode tolerates 3.3 V logic, or add series resistors / use 5 V drive.
- **CAN transceiver logic level.** Use a 3.3 V-`VIO` transceiver (e.g. TJA1051T/3) so the
  PHY matches the MCU logic level.
- **Common grounds.** All 24 V, 5 V, and 3.3 V domains must share a common ground reference.

---

## Safety

- 🛑 **No brakes.** Deceleration is "command 0 and hold." Always operate near a hardware
  power cutoff / kill switch.
- 💓 **Heartbeat + watchdog.** Edge nodes must fail to a safe state (motors → 0,
  steering → hold) on loss of the master heartbeat.
- 🔋 **High current at 24 V.** Respect fusing, wire gauge, and connector ratings.
- 🧪 **Bench first.** Validate each node on the bench (powered, wheels off the ground)
  before integrated vehicle tests.

---

## Repository Structure

```
AVL_CARL/
├── README.md            ← you are here (team-facing overview)
├── .gitignore
├── docs/
│   ├── hardware.md      ← BOM, part numbers, wiring, electrical notes
│   ├── architecture.md  ← node topology, MCU assignment rationale, caveats
│   ├── can-bus.md       ← CAN bitrate, node IDs, message/ID map
│   └── roadmap.md       ← phased plan, open questions, TODOs
└── firmware/            ← PlatformIO monorepo (see firmware/README.md)
    ├── platformio.ini   ← one [env] per node, shared lib/
    ├── lib/             ← shared code: carl_protocol, heartbeat, common
    ├── master/          ← NUCLEO-F767ZI (VCU)
    ├── throttle/        ← Teensy 4.1 (motor drive + encoders)
    ├── steering/        ← Teensy 4.1 (CL57T stepper)
    └── test/            ← host-run unit tests
```

> `CLAUDE.md` (AI-assistant project context) and the `.claude/` directory are local-only
> Claude Code files and are intentionally git-ignored.

---

## Toolchain & Getting Started

Firmware is a **PlatformIO monorepo** under [`firmware/`](firmware/) — one build
environment per node, with shared code in `firmware/lib/`. **Phase 1 (differential drive)
is implemented**: master + throttle nodes are functional and the steering node is a
safe-idle stub. See [`firmware/README.md`](firmware/README.md) and
[`docs/wiring.md`](docs/wiring.md) for details.

```bash
git clone https://github.com/calebhylkema/AVL_CARL.git
cd AVL_CARL/firmware

pio run -e throttle      # build/flash ONLY the throttle node
pio run                  # build all three nodes
pio test -e native       # run protocol unit tests on the PC (no hardware)

# drive it (wheels off the ground!):
python test/teleop.py --port COM5   # 1-4 speed · w a s d move · x stop
```

| Node | Board | PlatformIO platform |
|------|-------|---------------------|
| master | NUCLEO-F767ZI | `ststm32` |
| throttle | Teensy 4.1 | `teensy` |
| steering | Teensy 4.1 | `teensy` |

---

## Roadmap

See [`docs/roadmap.md`](docs/roadmap.md) for the living plan. Near-term milestones:

- [ ] Confirm front-wheel encoder count. *(Teensy models confirmed: all 4.1.)*
- [ ] Acquire and bench-test CAN transceivers; bring up a 2-node CAN link.
- [ ] Define the CAN message/ID map in [`docs/can-bus.md`](docs/can-bus.md).
- [ ] Encoder level-shifting circuit + verified 3.3 V quadrature read on the throttle node.
- [ ] Steering node: CL57T `step/dir/enable` bring-up.
- [ ] Throttle node: 4-motor PWM command + heartbeat fail-safe.
- [ ] **Phase 2:** master ↔ companion-computer interface for autonomy.

---

## Documentation

| Document | Contents |
|----------|----------|
| [`docs/hardware.md`](docs/hardware.md) | Bill of materials, part numbers, ratings, wiring, electrical integration |
| [`docs/architecture.md`](docs/architecture.md) | Node topology, MCU-to-node rationale, capability comparison, caveats |
| [`docs/can-bus.md`](docs/can-bus.md) | CAN bitrate, node IDs, message/ID allocation map |
| [`docs/roadmap.md`](docs/roadmap.md) | Phased plan, milestones, open questions |

---

## Contributing

This is an Autonomous Vehicle Lab team project. When adding hardware or changing the
architecture, **update the relevant `docs/` file in the same change** so documentation
stays in lockstep with the vehicle. Keep node firmware under `firmware/<node>/`.

---

## License

License TBD by the AVL team.
