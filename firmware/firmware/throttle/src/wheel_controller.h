#pragma once

#include <stdint.h>

#include "messages.h"

namespace carl {

/**
 * CARL wheel-control abstraction.
 *
 * Current milestone:
 *   - Embedded rear-wheel speed estimation.
 *   - Desired wheel-speed calculation.
 *   - PI-control calculations in shadow mode.
 *   - Motor output remains pass-through while PI is disabled.
 *
 * Safety remains owned by main.cpp:
 *   - heartbeat watchdog
 *   - E-stop handling
 *   - armed-state calculation
 *   - physical motor disable
 */
class WheelController {
 public:
  WheelController();

  /**
   * Clear all controller state.
   */
  void reset();

  /**
   * Update speed estimation, target calculation, and control state.
   */
  void update(
      const ThrottleCmd& desired,
      const EncoderFb& encoderFeedback,
      bool armed,
      uint32_t nowMs);

  /**
   * Command sent to driveAll().
   *
   * While PI output is disabled, this remains identical to desired.
   */
  const ThrottleCmd& output() const;

  const EncoderFb& encoderFeedback() const;

  float leftCountsPerSecond() const;
  float rightCountsPerSecond() const;

  float leftRpm() const;
  float rightRpm() const;

  float leftTargetRpm() const;
  float rightTargetRpm() const;

  float leftErrorRpm() const;
  float rightErrorRpm() const;

  float leftCorrectionMille() const;
  float rightCorrectionMille() const;

  float speedMismatchPercent() const;

  uint32_t lastUpdateMs() const;

  bool speedEstimateValid() const;

  /**
   * Returns true only when PI corrections are being applied.
   */
  bool piOutputEnabled() const;

 private:
  void updateSpeedEstimate(
      const EncoderFb& encoderFeedback,
      uint32_t nowMs);

  void updateControlState(
      const ThrottleCmd& desired,
      bool armed,
      uint32_t nowMs);

  void clearControlState();

  static int32_t wrappedCountDelta(
      int32_t currentCount,
      int32_t previousCount);

  static int16_t averageSideCommand(
      int16_t frontCommand,
      int16_t rearCommand);

  static float commandToTargetRpm(
      int16_t sideCommand);

  static int16_t clampCommandMille(
      float command);

  ThrottleCmd outputCommand_;

  EncoderFb encoderFeedback_;
  EncoderFb previousEncoderFeedback_;

  uint32_t lastUpdateMs_;
  uint32_t lastSpeedUpdateMs_;
  uint32_t lastControlUpdateMs_;

  float leftCountsPerSecond_;
  float rightCountsPerSecond_;

  float leftRpm_;
  float rightRpm_;

  float leftTargetRpm_;
  float rightTargetRpm_;

  float leftErrorRpm_;
  float rightErrorRpm_;

  float leftIntegral_;
  float rightIntegral_;

  float leftCorrectionMille_;
  float rightCorrectionMille_;

  bool havePreviousEncoderSample_;
  bool speedEstimateValid_;
};

}  // namespace carl
