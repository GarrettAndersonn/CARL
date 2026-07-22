// ─────────────────────────────────────────────────────────────────────────────
// messages.h — typed (de)serialization for every AVL_CARL CAN payload.
//
// These functions operate on a raw 8-byte buffer + length, NOT on any CAN
// driver's frame struct. That keeps this layer 100% platform-agnostic: the
// Teensy nodes (FlexCAN_T4) and the STM32 master (STM32_CAN) use different
// CAN_message_t types, but both expose `.id`, `.len`, and `.buf[8]`. Each node
// copies between its driver struct and these helpers.
//
// Payload layouts are LITTLE-ENDIAN and mirror docs/can-bus.md §3.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <stdint.h>

#include "carl_protocol.h"
#include "common.h"

namespace carl {

// Per-wheel throttle command (CANID_THROTTLE_CMD, 0x100), 8 bytes.
// Four signed int16 per-mille setpoints, order: FL, FR, RL, RR.
struct ThrottleCmd {
  int16_t fl;
  int16_t fr;
  int16_t rl;
  int16_t rr;
};

// Throttle node status (CANID_THROTTLE_STATUS, 0x200), 6 bytes.
struct ThrottleStatus {
  uint8_t state;          // ThrottleState
  uint8_t faults;         // ThrottleFault bitfield
  int16_t applied_left;   // per-mille actually applied to the left side
  int16_t applied_right;  // per-mille actually applied to the right side
};

// Wheel encoder feedback (CANID_ENCODER_FB, 0x210), 8 bytes.
// Signed 32-bit quadrature counts for the two rear wheels.
struct EncoderFb {
  int32_t left_count;
  int32_t right_count;
};

// Master heartbeat (CANID_HEARTBEAT, 0x010), 3 bytes.
struct Heartbeat {
  uint8_t counter;  // rolls over 0..255
  uint8_t flags;    // HeartbeatFlag bitfield
  uint8_t mode;     // VehicleMode
};

// ── Throttle command ────────────────────────────────────────────────────────
inline uint8_t pack_throttle_cmd(uint8_t* buf, const ThrottleCmd& c) {
  put_i16(&buf[0], c.fl);
  put_i16(&buf[2], c.fr);
  put_i16(&buf[4], c.rl);
  put_i16(&buf[6], c.rr);
  return 8;
}

inline bool unpack_throttle_cmd(const uint8_t* buf, uint8_t len, ThrottleCmd* c) {
  if (len < 8) return false;
  c->fl = get_i16(&buf[0]);
  c->fr = get_i16(&buf[2]);
  c->rl = get_i16(&buf[4]);
  c->rr = get_i16(&buf[6]);
  return true;
}

// ── Throttle status ─────────────────────────────────────────────────────────
inline uint8_t pack_throttle_status(uint8_t* buf, const ThrottleStatus& s) {
  buf[0] = s.state;
  buf[1] = s.faults;
  put_i16(&buf[2], s.applied_left);
  put_i16(&buf[4], s.applied_right);
  return 6;
}

inline bool unpack_throttle_status(const uint8_t* buf, uint8_t len, ThrottleStatus* s) {
  if (len < 6) return false;
  s->state = buf[0];
  s->faults = buf[1];
  s->applied_left = get_i16(&buf[2]);
  s->applied_right = get_i16(&buf[4]);
  return true;
}

// ── Encoder feedback ────────────────────────────────────────────────────────
inline uint8_t pack_encoder_fb(uint8_t* buf, const EncoderFb& e) {
  put_i32(&buf[0], e.left_count);
  put_i32(&buf[4], e.right_count);
  return 8;
}

inline bool unpack_encoder_fb(const uint8_t* buf, uint8_t len, EncoderFb* e) {
  if (len < 8) return false;
  e->left_count = get_i32(&buf[0]);
  e->right_count = get_i32(&buf[4]);
  return true;
}

// ── Heartbeat ───────────────────────────────────────────────────────────────
inline uint8_t pack_heartbeat(uint8_t* buf, const Heartbeat& h) {
  buf[0] = h.counter;
  buf[1] = h.flags;
  buf[2] = h.mode;
  return 3;
}

inline bool unpack_heartbeat(const uint8_t* buf, uint8_t len, Heartbeat* h) {
  if (len < 3) return false;
  h->counter = buf[0];
  h->flags = buf[1];
  h->mode = buf[2];
  return true;
}

// ── E-stop ──────────────────────────────────────────────────────────────────
inline uint8_t pack_estop(uint8_t* buf, uint8_t reason) {
  buf[0] = reason;
  return 1;
}

}  // namespace carl
