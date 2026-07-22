# Roadmap — AVL_CARL

> Living document. Last updated: 2026-06-10.

---

## Phases

### Phase 1 — Drive-by-wire (current)
Fully electronic throttle + steering over CAN, with a fail-safe heartbeat. No autonomy.

### Phase 2 — Autonomy (planned)
Companion compute on the master (Ethernet) for perception/planning. A sensor/redundancy
node would need a new 4th board (no spare in the 3-board inventory).

---

## Milestones / TODO

### Inventory & procurement
- [x] Confirm board inventory — **1× NUCLEO-F767ZI + 2× Teensy 4.1** (2026-06-11). One per
  node, no spare. Master = NUCLEO; throttle & steering = Teensy.
- [ ] Confirm front-wheel encoder count *(user checking 2026-06-11)*.
- [x] **Purchase CAN transceivers** — Adafruit **TJA1051T/3** ×3 (in hand, 2026-06-11).
- [ ] Document the battery pack (chemistry, capacity, BMS, fusing, kill switch).
- [ ] Identify the motor driver model + PWM requirements.
- [ ] Identify the stepper motor model + CL57T microstep setting.

### Electrical bring-up
- [ ] Encoder level-shifting circuit (5–24 V → 3.3 V); verify clean quadrature on a scope.
- [ ] Verify CL57T `step/dir/enable` accepts 3.3 V drive (or add level shifting).
- [ ] Establish common ground + power distribution (24 V / 5 V / 3.3 V rails).

### Communication
- [ ] Wire a 2-node CAN link (master + one edge) with 120 Ω termination.
- [ ] Finalize bitrate after measuring bus length.
- [x] Lock the CAN message/ID map and payload layouts in [`can-bus.md`](can-bus.md).
- [x] Implement heartbeat + watchdog + e-stop across all nodes *(firmware written; pending bench validation)*.

### Node firmware
- [x] Choose toolchain — **PlatformIO monorepo** (one env per node, shared `lib/`).
- [x] Scaffold `firmware/` (master, throttle, steering, shared `lib/`, test) — empty stubs.
- [x] Throttle node: 4-motor PWM command + rear-encoder read + fail-safe.
- [ ] Steering node: CL57T step/dir/enable + homing/limits + hold-on-fault *(Phase 1: safe-idle stub only; kart turns via differential drive)*.
- [x] Master node: command arbitration + heartbeat + telemetry aggregation *(teleop over USB serial)*.

### Integration & test
- [ ] Bench test each node (wheels off the ground).
- [ ] Integrated drive-by-wire test in a controlled area with a hardware kill switch.

### Phase 2 — autonomy
- [ ] Master ↔ companion-computer interface over Ethernet.
- [ ] Add a 4th board for a sensor hub / redundancy node (no spare in current inventory).

---

## Open decisions

- Per-wheel independent throttle vs single throttle value.
- Steering homing strategy (limit switch / current sense / position pot).
- Whether all 4 wheels are driven or only the (encoder-equipped) rear pair.
