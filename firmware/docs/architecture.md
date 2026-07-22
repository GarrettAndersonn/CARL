# Architecture — AVL_CARL

> Living document. Last updated: 2026-06-11.

This file documents the distributed control architecture, the microcontroller-to-node
assignment, and the engineering rationale behind it.

---

## 1. Topology

A single classic-CAN backbone connects three nodes. The master arbitrates commands;
edge nodes execute and report feedback. Inventory is exactly three boards
(1× NUCLEO-F767ZI + 2× Teensy 4.1) — one per node, no spare.

```
   Companion compute (Phase 2) ──Ethernet──┐
                                           │
                                    ┌──────┴──────┐
                                    │ MASTER / VCU│  NUCLEO-F767ZI
                                    └──────┬──────┘
   ════════════════ CAN 2.0 bus (120 Ω both ends) ════════════════
            │                                         │
   ┌────────┴────────┐                       ┌────────┴────────┐
   │ THROTTLE/DRIVE  │ Teensy 4.1            │   STEERING      │ Teensy 4.1
   │ 4× IBT_2 (8 PWM)│                       │ step/dir/enable │
   │ encoder decode  │                       │   → CL57T       │
   └─────────────────┘                       └─────────────────┘
```

## 2. Node responsibilities

| Node | Board | Responsibilities |
|------|-------|------------------|
| **Master / VCU** | NUCLEO-F767ZI | Source/arbitrate commands, emit heartbeat, run the watchdog/e-stop logic, expose the future autonomy interface (Ethernet to a companion computer), aggregate telemetry. |
| **Throttle / Motor-drive** | Teensy 4.1 | Receive throttle/motion commands over CAN; drive the 4 wheel motors via **4× IBT_2 (BTS7960) H-bridges** (2 PWM lines each → 8 PWM total, bidirectional); read the wheel encoders (quadrature) for speed/feedback; optionally read per-motor current sense; enforce fail-safe (motors → 0) on heartbeat loss. |
| **Steering** | Teensy 4.1 | Receive steering-angle commands over CAN; generate `step/dir/enable` to the CL57T; handle homing/limits; hold position on heartbeat loss. |

## 3. MCU-to-node rationale

The assignment maps each board's strengths to each node's dominant requirement:

- **Master = NUCLEO-F767ZI** — the one NUCLEO, with an on-board Ethernet PHY + RJ45 that is
  the natural Phase-2 autonomy/companion-computer link. Cortex-M7 + FPU and a large GPIO
  breakout give supervisory headroom. bxCAN covers the single backbone. (Both Teensys have
  Ethernet too, via an add-on PHY — but their FlexPWM/quadrature hardware is needed on the
  edge nodes, so the NUCLEO takes the coordinator role.)
- **Throttle = Teensy 4.1** — the most I/O-heavy node. Teensy 4.x has **hardware
  quadrature decoders** (no CPU cost to track encoders) plus abundant **FlexPWM** channels —
  needed here, since 4× IBT_2 H-bridges take **8 PWM lines** (2 per motor for forward/reverse)
  plus enables and optional current-sense ADC. All at 600 MHz with tightly-coupled memory for
  deterministic timing. 3× FlexCAN.
- **Steering = Teensy 4.1** — simplest node; only needs clean timer-based step pulses plus
  a few GPIO. Same Teensyduino toolchain as the throttle node keeps firmware consistent.
- **No spare board.** All three boards are assigned; a future 4th node (BMS / sensor hub /
  redundancy) requires purchasing an additional board.

## 4. Capability comparison

| Capability | Teensy 4.0 | Teensy 4.1 | NUCLEO-F767ZI |
|------------|-----------|-----------|----------------|
| Classic CAN 2.0 controllers | 2 (CAN1/2) | 3 (CAN1/2/3) | 2 (CAN1/2) |
| CAN-FD | No | No | No |
| Hardware quadrature encoders | Yes (ENC module, 4 ch) | Yes (ENC module, 4 ch) | Yes (timer encoder mode, many timers) |
| PWM channels | Many (FlexPWM + QuadTimer) | Many (FlexPWM + QuadTimer) | Many (14 timers × up to 4 ch) |
| Clock | 600 MHz | 600 MHz | 216 MHz |
| Flash / RAM | 2 MB / 1 MB | 8 MB / 1 MB | 2 MB / 512 KB |
| GPIO | 40 | 55 | up to 144 |
| Ethernet | None | Built-in MAC + PHY + RJ45 | Built-in MAC (+ on-board RJ45) |
| Toolchain | Teensyduino / PlatformIO | Teensyduino / PlatformIO | STM32CubeIDE/HAL / PlatformIO |
| Real-time | Bare-metal, DTCM, low jitter | Bare-metal, DTCM, low jitter | FreeRTOS-capable; deterministic with ISR tuning |

> Specs verified against PJRC (teensy techspecs), ST (STM32F767ZI / NUCLEO-F767ZI), and the
> FlexCAN_T4 / STM32 bxCAN documentation. Both MCU families are **classic CAN only**.

## 5. Fail-safe behavior

Consistent with the **no-brake** design (stop = command 0 and hold):

- The master emits a periodic **heartbeat** on the CAN bus.
- Each edge node runs a **watchdog** on that heartbeat. On timeout it enters a safe state:
  - Throttle node → command **0** to all motors and hold.
  - Steering node → **hold** current steering position (do not free-run the stepper).
- An **e-stop** message (highest CAN priority) forces the same safe state immediately.
- A hardware power cutoff / kill switch is the ultimate stop and must always be present.

## 6. Caveats

- **Teensy models confirmed:** both boards are **Teensy 4.1** (the steering node is no longer
  a 4.0). No I/O or Ethernet compromise on either Teensy node.
- **No CAN-FD / no CAN3 on F767.** STM32F767 has only CAN1/CAN2. Payloads are capped at
  8 bytes (classic CAN) — adequate for the command/feedback set.
- **Encoder level shifting** (5–24 V → 3.3 V) and **CL57T input levels** (3.3 V vs opto)
  must be solved in hardware — see [`hardware.md`](hardware.md#7-electrical-integration-notes-gotchas).
- **CAN transceivers in hand:** Adafruit **TJA1051T/3** ×3 (3.3 V logic, on-board 120 Ω).
  Enable termination on the two end nodes only — not the middle one.

## 7. Future (Phase 2 — autonomy)

- Companion computer (e.g. SBC/Jetson-class) connects to the **master over Ethernet**.
- Master translates high-level autonomy commands into CAN command messages.
- A sensor-aggregation node (IMU, range sensors) or redundant/safety controller would be a
  **new 4th board** — there is no spare in the current 3-board inventory.
