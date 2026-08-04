#pragma once

#include <stdint.h>

#include "messages.h"

namespace carl {

/**
 * CARL wheel-control abstraction.
 *
 * Current phase:
 *   Pass-through control only. The requested command is returned unchanged.
 *
 * Future phase:
 *   Encoder-derived wheel-speed estimation and independent left/right PI
 *   control will be implemented inside this class.
 *
 * Safety remains owned by main.cpp:
 *   - heartbeat watchdog
 *   - E-stop
 *   - armed state
 *   - motor enable shutdown
 */
class WheelController {
 public:
  WheelController();

  /**
   * Clear all internal state and force the controller output to zero.
   */
  void reset();

  /**
   * Update the controller.
   *
   * desired:
   *   Current four-wheel command from CAN, in per-mille.
   *
   * encoderFeedback:
   *   Current signed cumulative rear-wheel encoder counts.
   *
   * armed:
   *   True only when heartbeat and E-stop checks permit motion.
   *
   * nowMs:
   *   Current system time from millis().
   */
  void update(
      const ThrottleCmd& desired,
      const EncoderFb& encoderFeedback,
      bool armed,
      uint32_t nowMs);

  /**
   * Return the command that should be sent to driveAll().
   */
  const ThrottleCmd& output() const;

  /**
   * Return the most recently received encoder feedback.
   * This will be used by the later speed-control implementation.
   */
  const EncoderFb& encoderFeedback() const;

  /**
   * Return the time of the most recent controller update.
   */
  uint32_t lastUpdateMs() const;

 private:
  ThrottleCmd outputCommand_;
  EncoderFb encoderFeedback_;
  uint32_t lastUpdateMs_;
};

}  // namespace carl
