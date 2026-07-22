# Wiring Reference — AVL_CARL (Phase 1, differential drive)

> Living document. Last updated: 2026-06-11.
> This is the pin-level companion to [`hardware.md`](hardware.md) (BOM/ratings) and
> [`can-bus.md`](can-bus.md) (protocol). **The firmware pin maps in `firmware/` must match
> this file** — change both together.

Phase 1 scope: **throttle + master only**. The steering Teensy is **unpowered**; the kart
turns by **differential drive** (left-side vs right-side wheel speed) handled entirely on the
throttle node. ⚠️ All power and grounds are assumed already handled; this doc covers signal
wiring. **Common ground across all 24 V / 5 V / 3.3 V domains is mandatory.**

---

## 0. Pin map at a glance

| Function | Teensy 4.1 (THROTTLE, 0x02) | NUCLEO-F767ZI (MASTER, 0x01) |
|----------|-----------------------------|------------------------------|
| CAN TX | pin **22** (CAN1_TX) | **PD1** (CAN1_TX) |
| CAN RX | pin **23** (CAN1_RX) | **PD0** (CAN1_RX) |
| Teleop serial | — | ST-LINK **VCP** (USART3, PD8/PD9) over the USB cable |
| Status LED | pin 13 (on-board) | on-board LED |

---

## 1. CAN bus (the backbone)

Three transceivers (Adafruit TJA1051T/3), one twisted pair `CAN_H` / `CAN_L` + common GND.

### 1a. Adafruit TJA1051T/3 breakout ↔ MCU

Breakout pads: `VCC` `GND` `RX` `TX` `SLNT` `CANH` `CANL`.

| Breakout pin | Throttle Teensy 4.1 | Master NUCLEO-F767ZI | Notes |
|--------------|---------------------|----------------------|-------|
| **VCC**  | 3V3 | 3V3 | 3.3 V logic; board boosts to the 5 V the bus side needs (see VCC note) |
| **GND**  | GND | GND | common ground — also tie to the bus GND wire |
| **RX**   | pin **23** = **CRX1** | **PD0** (CAN1_RX) | breakout RX = transceiver output → MCU CAN **RX** |
| **TX**   | pin **22** = **CTX1** | **PD1** (CAN1_TX) | breakout TX = transceiver input → MCU CAN **TX** |
| **SLNT** | GND | GND | silent-mode select — **must be LOW** for normal TX/RX |
| **CANH** | bus CANH | bus CANH | twisted pair, daisy-chained to all nodes |
| **CANL** | bus CANL | bus CANL | twisted pair |

**Verified CAN-controller pins.** Teensy 4.1 FlexCAN is hard-tied per controller (these are
the `CTXn/CRXn` pins on the Teensy pinout card — not remappable) — CAN1 = pin 22 (CTX1) /
pin 23 (CRX1) [CAN2 = 1/0, CAN3 = 31/30]. STM32F767 bxCAN CAN1 (AF9) can
map to PA11/PA12 (=USB OTG, avoid), **PD0/PD1**, PB8/PB9, or PI9/PH13; the master firmware
uses **PD0 (RX) / PD1 (TX)** (`STM32_CAN Can1(CAN1, ALT_2)`). To use PB8/PB9 instead (they sit
at the Arduino `D15/D14` pads on CN7), change the firmware to `... ALT`.

> **No crossover.** Wire `TX→TX` and `RX→RX`. The breakout labels are already from the MCU's
> point of view; the transceiver does the internal swap. Crossing them is the #1 mistake.
>
> **SLNT must be grounded.** On the TJA1051 the S pin HIGH/floating = silent (listen-only) —
> the node will receive but never ACK or transmit, which looks like a dead bus. Tie it to GND
> (or to a spare GPIO held LOW if you ever want software listen-only).
>
> **VCC voltage — confirm on your board.** The TJA1051T/3 is the 3.3 V-logic variant, so RX/TX/
> SLNT are native 3.3 V (no level shifting). This breakout takes 3.3 V on VCC and generates the
> 5 V CAN supply with its on-board boost. *If your specific board exposes a separate `VIO` pad*,
> put VIO = 3.3 V and VCC = 5 V instead. Check the silkscreen before powering.
>
> CANH/CANL of all transceivers tie to the same twisted pair.

### 1b. Termination — exactly TWO, at the two physical bus ends

```
  [ end node ]======twisted pair======[ middle node ]======twisted pair======[ end node ]
   120 Ω ON                              120 Ω OFF                              120 Ω ON
```

- Enable the on-board 120 Ω **only on the two nodes at the physical ends** of the cable.
- The node in the **middle** must have its 120 Ω **disabled**. Three terminators is the #1
  CAN bring-up failure.

**Your current setup:** the master is wired as the middle node (termination **OFF**). The
throttle node and the steering node are the two ends (termination **ON** on both). You have
already set the steering transceiver to terminate.

> ⚠️ **Gotcha with the unpowered steering end:** the 120 Ω resistor is passive and works even
> with that Teensy unpowered, but its **transceiver IC will be unpowered**, which can lightly
> load the bus. For a clean 2-node Phase-1 bring-up, prefer **one of these**:
> 1. Power just the steering transceiver breakout (3.3 V + 5 V + GND) even though its Teensy
>    stays unpowered, **or**
> 2. Make the two **powered** nodes the bus ends — i.e. terminate **master + throttle** and
>    leave steering as an unterminated mid/stub tap.
>
> Either gives two clean terminators between two live transceivers. If the bus is short and
> 500 kbit/s is solid on the scope, the as-wired arrangement will usually work too.

---

## 2. Motor drivers — 4× IBT_2 (BTS7960) ↔ throttle Teensy

Each IBT_2 has 8 logic pins: `RPWM`, `LPWM`, `R_EN`, `L_EN`, `R_IS`, `L_IS`, `VCC`, `GND`.
We tie **`R_EN` + `L_EN` of a module together to one Teensy GPIO** (HIGH = enabled).
`R_IS`/`L_IS` (current sense) are **not wired** in Phase 1.

| Motor (wheel) | IBT_2 `RPWM` → Teensy | IBT_2 `LPWM` → Teensy | IBT_2 `R_EN`+`L_EN` → Teensy | `VCC` | `GND` |
|---------------|------------------------|------------------------|------------------------------|-------|-------|
| **FL** front-left  | pin **2** | pin **3** | pin **10** | 5 V | GND |
| **FR** front-right | pin **4** | pin **5** | pin **11** | 5 V | GND |
| **RL** rear-left   | pin **6** | pin **7** | pin **12** | 5 V | GND |
| **RR** rear-right  | pin **8** | pin **9** | pin **24** | 5 V | GND |

Direction convention in firmware: **`RPWM` = forward**, **`LPWM` = reverse**, both 0 = coast.
Power side of each IBT_2: `B+`/`B-` to the 24 V traction rail, `M+`/`M-` to the motor.

> ⚠️ **Logic level:** BTS7960 inputs are 5 V parts. Teensy drives 3.3 V. Verify `VIH` on a
> scope; if the inputs are marginal, level-shift `RPWM`/`LPWM`/`EN` to 5 V (see
> [`hardware.md` §7.3](hardware.md#7-electrical-integration-notes-gotchas)). Each IBT_2 logic
> `GND` **must** tie to the Teensy GND.

PWM: 20 kHz, 12-bit (set in firmware). The two PWM pins of a wheel never drive simultaneously.

---

## 3. Wheel encoders — 2× E38S6-600-24G → BSS138 level shifter → throttle Teensy

Rear wheels only (600 PPR quadrature). Colors per [`hardware.md` §4](hardware.md#4-sensing--wheel-encoders-e38s6-600-24g).
Teensy 4.1 pins are **3.3 V only / NOT 5 V-tolerant**, and the E38S6 outputs swing to their
supply rail — so the A/B lines go through a **BSS138 4-channel bidirectional level shifter**
(the board with `5V GND HV1..HV4` on one side and `3V3 GND LV1..LV4` on the other).

**Decision: level shifter YES, separate pull-up resistors NO.** The BSS138 board has a 10 kΩ
pull-up built into every channel. On the HV side that pull-up doubles as the pull-up an
open-collector encoder needs; the MOSFET does the 5 V↔3.3 V translation. This works for an
open-collector **or** a push-pull encoder, so the output type does not need to be determined.

| Signal | Encoder wire | → Shifter `HV` | Shifter `LV` → | Teensy pin |
|--------|--------------|----------------|----------------|------------|
| Rear-Left  A  | white | **HV1** | **LV1** | **14** |
| Rear-Left  B  | green | **HV2** | **LV2** | **15** |
| Rear-Right A  | white | **HV3** | **LV3** | **16** |
| Rear-Right B  | green | **HV4** | **LV4** | **17** |

Level-shifter power/reference (all required — the pull-ups don't work otherwise):

| Shifter pin | Connect to |
|-------------|------------|
| `5V` (HV ref) | 5 V rail |
| `3V3` (LV ref) | Teensy **3.3 V** |
| `GND` (both sides) | common ground |

Each encoder: **red (VCC) → 5 V**, **black (0 V) → common GND**. **Never wire 24 V** anywhere
near the Teensy or the LV side.

> The built-in 10 kΩ pull-ups are fast enough for the encoder edge rate here (~10–25 kHz even
> at full kart speed). Rare exception: if your specific shifter lacks built-in pull-ups, add a
> 4.7–10 kΩ from each `HVx` to 5 V (the `5V/GND/HV1-4 + 3V3/GND/LV1-4` board is the BSS138 type,
> which has them). Confirm clean quadrature on a scope before trusting the counts.

---

## 4. Master ↔ PC (teleop)

A single USB cable from the PC to the NUCLEO's ST-LINK port does **both**:

1. Flashes the master (ST-LINK / SWD), and
2. Exposes the **Virtual COM Port** the teleop script talks to (115200 8N1).

No extra wiring. Find the port with `python firmware/test/teleop.py --list`.

---

## 5. Steering node (Phase 1: parked)

Unpowered. Reserved CL57T pins for when steering comes online (verify opto logic levels first):
Teensy `STEP` = pin 3, `DIR` = pin 4, `ENA` = pin 5; CAN on pins 22/23 like the throttle node.
The safe-idle firmware keeps the CL57T disabled and never steps.

---

## 6. Power-on / bring-up order (bench, wheels off the ground)

1. Confirm common ground everywhere; confirm CAN termination = exactly two ends.
2. Power logic rails (3.3 V/5 V) **before** the 24 V traction rail.
3. Flash master (ST-LINK) and throttle (Teensy Loader). Boot protocols differ per chip —
   see [`firmware/README.md`](../firmware/README.md).
4. Open `teleop.py`; verify `TLM` telemetry streams and `link=1`.
5. With wheels **off the ground**, test level 1 + `w`/`s`/`a`/`d`, then `x`. Keep the hardware
   kill switch in hand.
