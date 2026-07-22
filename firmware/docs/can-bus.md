# CAN Bus Design — AVL_CARL

> Living document. Last updated: 2026-06-10.
> The message/ID map below is a **starting proposal** — finalize before firmware bring-up.

---

## 1. Physical layer

| Parameter | Value | Notes |
|-----------|-------|-------|
| Standard | Classic CAN 2.0 | Teensy FlexCAN + STM32 bxCAN are classic-only (no CAN-FD). |
| Bitrate | **500 kbit/s** (proposed) | Revisit for final bus length; lower if the bus is long. |
| Frame format | 11-bit standard IDs | Lower ID = higher priority. |
| Termination | **120 Ω at both physical ends** | Two resistors total, not per node. Enable the Adafruit board's on-board 120 Ω on the **two end nodes only**. |
| Transceiver | **Adafruit TJA1051T/3** ×3 (in hand) | 3.3 V logic side; on-board optional 5 V boost + switchable 120 Ω. |
| Wiring | Twisted pair (CAN_H / CAN_L) + common GND | Keep stubs short. |

---

## 2. Node IDs

| Node | Board | Node ID (proposed) |
|------|-------|--------------------|
| Master / VCU | NUCLEO-F767ZI | 0x01 |
| Throttle / Motor-drive | Teensy 4.1 | 0x02 |
| Steering | Teensy 4.1 | 0x03 |
| (Future 4th node — needs a new board) | TBD | 0x04 |

---

## 3. Message / ID map (LOCKED — matches firmware)

Priority is encoded in the ID (lower = higher priority). Reserve the lowest IDs for safety.
These layouts are the single source of truth and are implemented in
`firmware/lib/carl_protocol/` (`carl_protocol.h`, `messages.h`). **All multi-byte fields are
little-endian.** Change the header and this table together.

| CAN ID | Name | Direction | Period | Len | Payload (byte : field) |
|--------|------|-----------|--------|-----|------------------------|
| `0x000` | **E-STOP** | any → all | event | 1 | `0`:reason (1=operator) |
| `0x010` | **Heartbeat** | Master → all | 50 ms | 3 | `0`:counter · `1`:flags · `2`:mode |
| `0x100` | Throttle command | Master → Throttle | 20 ms | 8 | `0-1`:FL · `2-3`:FR · `4-5`:RL · `6-7`:RR (each `int16` per-mille, −1000…+1000) |
| `0x110` | Steering command | Master → Steering | 20 ms | — | *Phase 1: unused (differential drive)* |
| `0x200` | Throttle status | Throttle → Master | 50 ms | 6 | `0`:state · `1`:faults · `2-3`:applied-left · `4-5`:applied-right (`int16` per-mille) |
| `0x210` | Wheel encoder feedback | Throttle → Master | 20 ms | 8 | `0-3`:rear-L count · `4-7`:rear-R count (`int32`, signed quadrature counts) |
| `0x220` | Steering status | Steering → Master | 50 ms | 2 | `0`:state(0=idle) · `1`:homed *(Phase 1: idle stub)* |

**Flag / enum encodings** (see `carl_protocol.h`):
- Heartbeat `flags`: bit0 `OK`, bit1 `ESTOP`, bit2 `TELEOP_OK`. `mode`: 0 idle, 1 teleop, 2 estop.
- Throttle `state`: 0 init, 1 idle, 2 driving, 3 safe-stop, 4 estop. `faults`: bit0 heartbeat-lost, bit1 estop.
- Throttle "per-mille": −1000 = full reverse, 0 = stop, +1000 = full forward.

> Max 8 data bytes per frame (classic CAN). The master mixes teleop `vx`/`wz` into the four
> per-wheel setpoints (left = vx+wz, right = vx−wz) and sends `0x100`; the throttle node maps
> each setpoint to one IBT_2 (sign = direction, magnitude = PWM duty).

---

## 4. Safety conventions

- **Heartbeat watchdog:** if an edge node misses N consecutive heartbeats (e.g. 3 × 50 ms),
  it enters the safe state: throttle → 0 and hold; steering → hold position.
- **E-stop** (`0x000`) is the highest-priority frame and triggers the safe state immediately
  on every node.
- All command messages should carry a rolling counter and/or be idempotent so a stale frame
  cannot cause unintended motion.

---

## 5. Open questions

- Final bitrate after measuring bus length.
- Exact payload encoding (scaling, signedness, endianness) per message.
- Whether per-wheel throttle is independent (4 setpoints) or a single throttle value.
- Encoder feedback as raw counts vs computed speed.
- Need for a separate diagnostic/telemetry ID block for Phase-2 autonomy.
