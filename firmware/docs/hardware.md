# Hardware Reference — AVL_CARL

> Living document. Update this in the same commit whenever hardware changes.
> Last updated: 2026-06-11.

This file is the single source of truth for the bill of materials, part numbers,
electrical ratings, wiring, and integration notes. `TBD` marks information the team is
still gathering.

---

## 1. Bill of Materials (BOM)

| Subsystem | Item | Part / Model | Qty | Key ratings | Status |
|-----------|------|--------------|-----|-------------|--------|
| Propulsion | Traction motor | **MY4845** | 4 | 24 V, 150 W, ~2300 rpm, w/ gearbox | On vehicle |
| Propulsion | Motor driver | **IBT_2** (BTS7960 dual H-bridge) | 4 | ~6–27 V motor rail, high-current; PWM + EN inputs (5 V logic) | On vehicle |
| Steering | Stepper driver | **CL57T** closed-loop | 1 | 24 V actuator rail; step/dir/enable | On vehicle |
| Steering | Stepper motor | TBD (closed-loop NEMA) | 1 | TBD | On vehicle |
| Sensing | Wheel encoder | **E38S6-600-24G** (NO:1001119) | 2 (rear) | 600 PPR, quadrature, 5–24 V | On vehicle |
| Compute | Microcontroller | **Teensy 4.1** | 2 | 600 MHz Cortex-M7 | In hand |
| Compute | Microcontroller | **NUCLEO-F767ZI** | 1 | 216 MHz Cortex-M7, Ethernet | In hand |
| Comms | CAN transceiver | **Adafruit TJA1051T/3** breakout | 3 | 3–5 V VIO/logic, optional 5 V boost, switchable on-board 120 Ω | In hand |
| Power | Battery pack | **TBD** | 1 | ~24 V (capacity TBD) | TBD |

---

## 2. Propulsion

### Traction motors — MY4845
- 24 V DC motor, 150 W, ~2300 rpm rated, coupled to a **gearbox**.
- **4 units**, one per wheel → independent per-wheel drive.
- ⚠️ **Clarification:** MY4845 is the **motor**, not the battery. The main battery pack is a
  separate item (see Power) still to be documented.

### Motor drivers — IBT_2 (BTS7960)
- **One IBT_2 module per motor → 4 total**, commanded by the **throttle node**.
- Each module is a **dual BTS7960 half-bridge = one full H-bridge**, so each motor can be
  driven **bidirectionally** (forward / reverse) with PWM speed control.
- Motor supply (`B+` / `B-`): rated ~6–27 V — the 24 V traction rail sits inside this range.
  High current capability (board rated ~43 A peak; realistically ≈20 A continuous **only with
  added heatsinking/airflow** — derate hard).
- **Logic interface per module** (6 pins): `RPWM`, `LPWM` (PWM for each direction),
  `R_EN`, `L_EN` (driver enables, commonly tied together to one GPIO), and `R_IS`, `L_IS`
  (analog current-sense outputs, optional → MCU ADC). `VCC` = 5 V logic, plus common `GND`.
- **Pin budget on the throttle Teensy:** 4 motors × 2 PWM = **8 PWM channels** minimum,
  + enables (≥1 GPIO/module), + up to 8 ADC lines if current sense is used. Within the
  Teensy 4.1 FlexPWM/QuadTimer + ADC budget.
- Stop = command 0 PWM on both inputs (coast) per the no-brake design; the H-bridge also
  makes **active braking / reverse** physically possible if the team later chooses to use it.

**Open items**
- PWM frequency/resolution to use (BTS7960 accepts up to ~25 kHz; pick an inaudible,
  efficient point and confirm against motor/driver heating).
- Whether `R_IS`/`L_IS` current sense is wired back to the MCU (per-wheel current limiting).
- Logic-level check: BTS7960 inputs are 5 V parts — verify 3.3 V Teensy drive meets `VIH`,
  else level-shift to 5 V (see Integration Notes).
- Current draw per motor under load (for fusing / wire gauge).
- Whether all 4 wheels are driven or only the rear pair (rear has the encoders).

---

## 3. Steering

- **Geometry:** trapezoidal **Ackermann** linkage.
- **Actuator:** stepper motor driven by a **CL57T** closed-loop stepper driver.
  - Interface: `PUL/step`, `DIR`, `ENA/enable` (optocoupler inputs).
  - Powered from the **24 V actuator rail**.
  - Driver manual: <https://www.omc-stepperonline.com/download/CL57T_V4.0.pdf>
- Driven by the **steering node**.

**Open items**
- Stepper motor model + steps/rev + microstep setting on the CL57T.
- Steering end-stop / home strategy (limit switch vs current sensing vs position pot).
- CL57T input logic level vs 3.3 V MCU drive (see Integration Notes).

---

## 4. Sensing — Wheel encoders (E38S6-600-24G)

| Spec | Value |
|------|-------|
| Type | Incremental rotary, quadrature (A/B) |
| Resolution | **600 PPR** (pulses/rev) → 2400 counts/rev with 4× quadrature decoding |
| Supply / output | 5–24 V |
| Part / serial | E38S6-600-24G · NO:1001119 |
| Location | 2× rear wheels (front/additional TBD) |

**Wiring (color code)**

| Wire color | Signal |
|------------|--------|
| White | OUT A |
| Green | OUT B |
| Red | VCC (5–24 V) |
| Black | 0 V (GND) |

**Open items**
- Confirm total encoder count (front wheels?) — *user checking 2026-06-11*.
- Output type (open-collector vs push-pull): **no longer blocking** — the A/B lines go through
  a BSS138 level shifter whose built-in 10 kΩ pull-ups handle either type. See
  [`wiring.md` §3](wiring.md). (Still worth confirming for documentation.)

---

## 5. Compute

| Board | Role | Notes |
|-------|------|-------|
| NUCLEO-F767ZI | Master / VCU | Ethernet for autonomy link; 2× bxCAN |
| Teensy 4.1 | Throttle / motor-drive | HW quadrature + FlexPWM; 3× FlexCAN |
| Teensy 4.1 | Steering | step/dir/enable to CL57T |

See [`architecture.md`](architecture.md) for the full capability comparison and rationale.

> **Inventory confirmed (2026-06-11): 1× NUCLEO-F767ZI + 2× Teensy 4.1.** All three boards
> are in use — there is **no spare**. A future 4th node (BMS / sensor hub / redundancy) needs
> an additional board.

---

## 6. Power

- **System voltage:** 24 V.
- **Actuator rail:** 24 V (powers CL57T).
- **Logic rails:** 5 V (encoder supply, possibly motor-driver logic) and 3.3 V (MCU logic).
- **Battery pack:** TBD — capacity, chemistry, BMS, fusing all to be documented.

**Open items**
- Battery pack specification + BMS.
- Main fuse / breaker rating and hardware kill switch location.
- DC-DC converters for 5 V / 3.3 V rails.

---

## 7. Electrical Integration Notes (gotchas)

These must be resolved during bring-up:

1. **Encoder level shifting — CRITICAL.**
   E38S6 outputs swing to the supply rail (5–24 V); Teensy GPIO is **3.3 V only, NOT
   5 V-tolerant**.
   - **Resolved:** route A/B through a **BSS138 4-channel bidirectional level shifter**
     (HV ref = 5 V, LV ref = Teensy 3.3 V). Its built-in 10 kΩ pull-ups serve as the
     open-collector pull-up *and* it translates the level — works for open-collector or
     push-pull, so no separate pull-ups/divider are needed. Pinout in
     [`wiring.md` §3](wiring.md).
   - Power the encoder at **5 V** (never 24 V), common ground. Verify clean quadrature on a
     scope before trusting the counts. **Do not** wire 24 V encoder output to a 3.3 V pin.

2. **CL57T step/dir/enable logic level.**
   Optocoupler inputs. Confirm the driver version/mode accepts 3.3 V logic; otherwise use
   a series resistor sized for the opto LED current, or drive at 5 V.

3. **IBT_2 input logic level.**
   The BTS7960 is a 5 V part. 3.3 V Teensy outputs are often *near* the input threshold —
   verify `VIH` on a scope, and if marginal, level-shift the `RPWM`/`LPWM`/`EN` lines to 5 V.
   The `R_IS`/`L_IS` current-sense outputs are referenced to 5 V — clamp/divide before any
   3.3 V ADC pin.

4. **CAN transceivers — Adafruit TJA1051T/3 (×3, in hand).**
   - Power the **logic side at 3.3 V** (VIO) so TX/RX match the Teensy / NUCLEO directly —
     no level shifting. The transceiver's 5 V supply is handled by the board's optional
     on-board 5 V boost (feed 3.3 V) or by feeding it the 5 V rail directly.
   - **Termination: enable the on-board 120 Ω on the TWO end nodes ONLY.** On a 3-node line
     (master + 2 edges) the middle node must have its 120 Ω **disabled** — two terminators
     total, not three. Wrong termination is the most common CAN bring-up failure.
   - Wire TX/RX to the MCU's CAN controller pins, CANH/CANL to the twisted pair, plus a
     shared ground.

5. **Common ground.**
   All 24 V / 5 V / 3.3 V domains must share a common ground reference — including each
   IBT_2 logic `GND` tied to the Teensy ground.

---

## 8. Reference links

- CL57T driver manual — <https://www.omc-stepperonline.com/download/CL57T_V4.0.pdf>
- Teensy tech specs — <https://www.pjrc.com/teensy/techspecs.html>
- NUCLEO-F767ZI — <https://www.st.com/en/evaluation-tools/nucleo-f767zi.html>
- TJA1051 CAN transceiver — <https://www.nxp.com/docs/en/data-sheet/TJA1051.pdf>
