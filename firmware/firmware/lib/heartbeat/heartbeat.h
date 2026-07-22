// ─────────────────────────────────────────────────────────────────────────────
// heartbeat.h — shared heartbeat producer + watchdog for the fail-safe scheme.
//
//   HeartbeatTx : master uses this to pace + count the periodic heartbeat.
//   Watchdog    : edge nodes feed this on each received heartbeat; expired()
//                 going true means "lost the master" -> enter the safe state.
//
// Time is passed in as a millis() value so the logic stays testable and both
// MCU families (Teensy / STM32 Arduino) can use it identically.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <stdint.h>

namespace carl {

// Periodic counter for the master heartbeat.
class HeartbeatTx {
 public:
  explicit HeartbeatTx(uint32_t period_ms) : period_ms_(period_ms) {}

  // True once `period_ms` has elapsed since the last send. When true, call
  // next() to advance the rolling counter and record the send time.
  bool due(uint32_t now_ms) const { return (now_ms - last_ms_) >= period_ms_; }

  uint8_t next(uint32_t now_ms) {
    last_ms_ = now_ms;
    return ++counter_;
  }

  uint8_t counter() const { return counter_; }

 private:
  uint32_t period_ms_;
  uint32_t last_ms_ = 0;
  uint8_t counter_ = 0;
};

// Watchdog on an incoming periodic signal (the master heartbeat).
class Watchdog {
 public:
  explicit Watchdog(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}

  // Call when a heartbeat arrives.
  void feed(uint32_t now_ms) {
    last_feed_ms_ = now_ms;
    started_ = true;
  }

  // True until the first feed (we have never heard the master) and any time the
  // timeout has elapsed since the last feed.
  bool expired(uint32_t now_ms) const {
    if (!started_) return true;
    return (now_ms - last_feed_ms_) >= timeout_ms_;
  }

  bool everFed() const { return started_; }

 private:
  uint32_t timeout_ms_;
  uint32_t last_feed_ms_ = 0;
  bool started_ = false;
};

}  // namespace carl
