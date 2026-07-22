// ─────────────────────────────────────────────────────────────────────────────
// carl_protocol.h — single source of truth for the AVL_CARL CAN protocol.
//
// Every node (master, throttle, steering) includes this. If a value changes
// here, it changes everywhere — senders and receivers cannot drift.
//
// Keep in sync with docs/can-bus.md. Pure C++ (no Arduino) so the host tests
// can include it too.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <stdint.h>

namespace carl {

// ── Physical layer ──────────────────────────────────────────────────────────
static const uint32_t CAN_BITRATE = 125000;  // 500 kbit/s, classic CAN 2.0

// ── Node IDs (see docs/can-bus.md §2) ───────────────────────────────────────
enum NodeId : uint8_t {
  NODE_MASTER   = 0x01,
  NODE_THROTTLE = 0x02,
  NODE_STEERING = 0x03,
};

// ── CAN message IDs (11-bit standard, lower = higher priority) ───────────────
enum CanId : uint16_t {
  CANID_ESTOP            = 0x000,  // any -> all,        event
  CANID_HEARTBEAT        = 0x010,  // master -> all,     50 ms
  CANID_THROTTLE_CMD     = 0x100,  // master -> throttle, 20 ms
  CANID_STEERING_CMD     = 0x110,  // master -> steering, 20 ms (Phase 1: unused)
  CANID_THROTTLE_STATUS  = 0x200,  // throttle -> master, 50 ms
  CANID_ENCODER_FB       = 0x210,  // throttle -> master, 20 ms
  CANID_STEERING_STATUS  = 0x220,  // steering -> master, 50 ms (Phase 1: unused)
};

// ── Timing (milliseconds) ───────────────────────────────────────────────────
static const uint32_t HEARTBEAT_PERIOD_MS    = 50;
static const uint32_t THROTTLE_CMD_PERIOD_MS = 20;
static const uint32_t STATUS_PERIOD_MS       = 50;
static const uint32_t ENCODER_FB_PERIOD_MS   = 20;

// Edge nodes enter the safe state if the master heartbeat is missed for this
// long (3 missed beats at 50 ms). No-brake design => safe state = motors 0.
static const uint32_t HEARTBEAT_TIMEOUT_MS = 150;

// Master zeroes drive commands if the teleop link (PC over USB serial) goes
// quiet for this long — so a crashed/unplugged teleop script stops the kart.
static const uint32_t TELEOP_TIMEOUT_MS = 300;

// ── Throttle scale ──────────────────────────────────────────────────────────
// Per-wheel throttle is a signed "per-mille": -1000 = full reverse, 0 = stop,
// +1000 = full forward. Keeps everything integer across the bus.
static const int16_t THROTTLE_MIN = -1000;
static const int16_t THROTTLE_MAX =  1000;
// Below this magnitude the throttle node coasts (avoid buzzing the H-bridge).
static const int16_t THROTTLE_DEADBAND = 20;

// ── Heartbeat status flags (byte 1 of CANID_HEARTBEAT) ──────────────────────
enum HeartbeatFlag : uint8_t {
  HB_FLAG_OK        = 0x01,  // master nominal
  HB_FLAG_ESTOP     = 0x02,  // e-stop latched, bus-wide safe state
  HB_FLAG_TELEOP_OK = 0x04,  // teleop link alive (PC sending commands)
};

// ── Vehicle mode (byte 2 of CANID_HEARTBEAT) ────────────────────────────────
enum VehicleMode : uint8_t {
  MODE_IDLE   = 0,  // armed but commanding zero
  MODE_TELEOP = 1,  // driven from the teleop script
  MODE_ESTOP  = 2,  // emergency stop latched
};

// ── Throttle-node state (byte 0 of CANID_THROTTLE_STATUS) ───────────────────
enum ThrottleState : uint8_t {
  TST_INIT      = 0,  // booting, no heartbeat seen yet
  TST_IDLE      = 1,  // heartbeat OK, commanding zero
  TST_DRIVING   = 2,  // applying a nonzero command
  TST_SAFE_STOP = 3,  // heartbeat lost -> motors 0, enables off
  TST_ESTOP     = 4,  // e-stop latched -> motors 0, enables off
};

// ── Throttle-node fault flags (byte 1 of CANID_THROTTLE_STATUS) ─────────────
enum ThrottleFault : uint8_t {
  TF_NONE           = 0x00,
  TF_HEARTBEAT_LOST = 0x01,
  TF_ESTOP          = 0x02,
};

// ── E-stop reason codes (byte 0 of CANID_ESTOP) ─────────────────────────────
enum EstopReason : uint8_t {
  ESTOP_OPERATOR   = 1,  // commanded from teleop
  ESTOP_LINK_LOST  = 2,  // reserved
  ESTOP_NODE_FAULT = 3,  // reserved
};

}  // namespace carl
