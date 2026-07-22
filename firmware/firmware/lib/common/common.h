// ─────────────────────────────────────────────────────────────────────────────
// common.h — shared, platform-agnostic helpers (AVL_CARL)
//
// Pure C++ (no Arduino dependency) so it also compiles for the host/native unit
// tests. Keep it that way: no <Arduino.h>, no millis(), no pin I/O here.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <stdint.h>

namespace carl {

// Clamp v into [lo, hi].
template <typename T>
inline T clampv(T v, T lo, T hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Integer linear map (like Arduino map), done in 64-bit to avoid overflow.
inline int32_t map_i32(int32_t x, int32_t in_lo, int32_t in_hi,
                       int32_t out_lo, int32_t out_hi) {
  if (in_hi == in_lo) return out_lo;
  int64_t num = (int64_t)(x - in_lo) * (int64_t)(out_hi - out_lo);
  return (int32_t)(num / (in_hi - in_lo)) + out_lo;
}

inline int16_t abs_i16(int16_t v) { return v < 0 ? (int16_t)(-v) : v; }

// ── Little-endian byte (de)serialization helpers ────────────────────────────
// All multi-byte CAN payload fields use little-endian. Use these everywhere so
// senders and receivers can never disagree on byte order.

inline void put_i16(uint8_t* p, int16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

inline int16_t get_i16(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

inline void put_u16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

inline uint16_t get_u16(const uint8_t* p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

inline void put_i32(uint8_t* p, int32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

inline int32_t get_i32(const uint8_t* p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

}  // namespace carl
