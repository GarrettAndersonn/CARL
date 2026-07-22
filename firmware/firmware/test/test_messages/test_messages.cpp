// Host (native) unit tests for the AVL_CARL CAN message (de)serialization.
//   pio test -e native
//
// These run on the PC — no board required. They guard the single source of
// truth in lib/carl_protocol so a sender and receiver can never drift on layout,
// endianness, or sign handling.
#include <unity.h>

#include "common.h"
#include "messages.h"

using namespace carl;

void setUp() {}
void tearDown() {}

// ── Little-endian primitives ────────────────────────────────────────────────
void test_i16_le_roundtrip() {
  uint8_t b[2];
  put_i16(b, -1234);
  TEST_ASSERT_EQUAL_INT16(-1234, get_i16(b));
  // Confirm byte order is little-endian: -1 -> 0xFF 0xFF, 0x0102 low byte first.
  put_i16(b, 0x0102);
  TEST_ASSERT_EQUAL_UINT8(0x02, b[0]);
  TEST_ASSERT_EQUAL_UINT8(0x01, b[1]);
}

void test_i32_le_roundtrip() {
  uint8_t b[4];
  put_i32(b, -123456789);
  TEST_ASSERT_EQUAL_INT32(-123456789, get_i32(b));
  put_i32(b, 0x04030201);
  TEST_ASSERT_EQUAL_UINT8(0x01, b[0]);
  TEST_ASSERT_EQUAL_UINT8(0x04, b[3]);
}

// ── Throttle command ────────────────────────────────────────────────────────
void test_throttle_cmd_roundtrip() {
  ThrottleCmd in = {1000, -1000, 250, -250};
  uint8_t buf[8] = {0};
  TEST_ASSERT_EQUAL_UINT8(8, pack_throttle_cmd(buf, in));

  ThrottleCmd out;
  TEST_ASSERT_TRUE(unpack_throttle_cmd(buf, 8, &out));
  TEST_ASSERT_EQUAL_INT16(in.fl, out.fl);
  TEST_ASSERT_EQUAL_INT16(in.fr, out.fr);
  TEST_ASSERT_EQUAL_INT16(in.rl, out.rl);
  TEST_ASSERT_EQUAL_INT16(in.rr, out.rr);
}

void test_throttle_cmd_rejects_short() {
  uint8_t buf[8] = {0};
  ThrottleCmd out;
  TEST_ASSERT_FALSE(unpack_throttle_cmd(buf, 7, &out));
}

// ── Throttle status ─────────────────────────────────────────────────────────
void test_throttle_status_roundtrip() {
  ThrottleStatus in = {TST_DRIVING, TF_NONE, 600, -600};
  uint8_t buf[8] = {0};
  TEST_ASSERT_EQUAL_UINT8(6, pack_throttle_status(buf, in));

  ThrottleStatus out;
  TEST_ASSERT_TRUE(unpack_throttle_status(buf, 6, &out));
  TEST_ASSERT_EQUAL_UINT8(in.state, out.state);
  TEST_ASSERT_EQUAL_UINT8(in.faults, out.faults);
  TEST_ASSERT_EQUAL_INT16(in.applied_left, out.applied_left);
  TEST_ASSERT_EQUAL_INT16(in.applied_right, out.applied_right);
}

// ── Encoder feedback ────────────────────────────────────────────────────────
void test_encoder_fb_roundtrip() {
  EncoderFb in = {123456, -987654};
  uint8_t buf[8] = {0};
  TEST_ASSERT_EQUAL_UINT8(8, pack_encoder_fb(buf, in));

  EncoderFb out;
  TEST_ASSERT_TRUE(unpack_encoder_fb(buf, 8, &out));
  TEST_ASSERT_EQUAL_INT32(in.left_count, out.left_count);
  TEST_ASSERT_EQUAL_INT32(in.right_count, out.right_count);
}

// ── Heartbeat ───────────────────────────────────────────────────────────────
void test_heartbeat_roundtrip() {
  Heartbeat in = {42, (uint8_t)(HB_FLAG_OK | HB_FLAG_TELEOP_OK), MODE_TELEOP};
  uint8_t buf[8] = {0};
  TEST_ASSERT_EQUAL_UINT8(3, pack_heartbeat(buf, in));

  Heartbeat out;
  TEST_ASSERT_TRUE(unpack_heartbeat(buf, 3, &out));
  TEST_ASSERT_EQUAL_UINT8(in.counter, out.counter);
  TEST_ASSERT_EQUAL_UINT8(in.flags, out.flags);
  TEST_ASSERT_EQUAL_UINT8(in.mode, out.mode);
}

// ── Differential mix sanity (the master's core math) ────────────────────────
void test_clamp_and_mix() {
  // left = vx + wz, right = vx - wz, clamped to [-1000, 1000].
  int16_t vx = 900, wz = 300;
  TEST_ASSERT_EQUAL_INT16(1000, clampv<int16_t>(vx + wz, THROTTLE_MIN, THROTTLE_MAX));
  TEST_ASSERT_EQUAL_INT16(600, clampv<int16_t>(vx - wz, THROTTLE_MIN, THROTTLE_MAX));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_i16_le_roundtrip);
  RUN_TEST(test_i32_le_roundtrip);
  RUN_TEST(test_throttle_cmd_roundtrip);
  RUN_TEST(test_throttle_cmd_rejects_short);
  RUN_TEST(test_throttle_status_roundtrip);
  RUN_TEST(test_encoder_fb_roundtrip);
  RUN_TEST(test_heartbeat_roundtrip);
  RUN_TEST(test_clamp_and_mix);
  return UNITY_END();
}
