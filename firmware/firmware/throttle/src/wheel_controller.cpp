#include "wheel_controller.h"

#include <math.h>

namespace carl {

namespace {

/*
 * IMPORTANT:
 *
 * Keep this false for the current milestone.
 *
 * false:
 *   PI calculations run in shadow mode.
 *   Motor commands remain unchanged.
 *
 * true:
 *   PI corrections are added to the motor commands.
 *
 * Do not change this to true until the shadow-mode checklist passes.
 */
static constexpr bool ENABLE_WHEEL_PI_CONTROL = false;

// Encoder calibration measured on CARL.
static constexpr float LEFT_COUNTS_PER_REV = 12031.0f;
static constexpr float RIGHT_COUNTS_PER_REV = 12181.0f;

// Speed estimator and PI controller update rate: 50 Hz.
static constexpr uint32_t CONTROL_PERIOD_MS = 20;

// Reject stale timing intervals.
static constexpr uint32_t MAX_VALID_DT_MS = 250;

// Encoder-speed low-pass filter.
static constexpr float SPEED_FILTER_ALPHA = 0.25f;

// Treat tiny measured speeds as zero.
static constexpr float STOPPED_RPM_THRESHOLD = 0.25f;

/*
 * Initial conservative PI gains.
 *
 * Units:
 *   KP = per-mille correction per RPM of error
 *   KI = per-mille correction per RPM-second of accumulated error
 *
 * These values are not applied while ENABLE_WHEEL_PI_CONTROL is false.
 */
static constexpr float KP = 2.0f;
static constexpr float KI = 0.8f;

// Prevent integral windup.
static constexpr float INTEGRAL_LIMIT = 250.0f;

// Limit how much the controller may modify the base command.
static constexpr float CORRECTION_LIMIT_MILLE = 300.0f;

/*
 * Provisional command-to-speed model based on CARL bench data.
 *
 * Approximately:
 *   25% command -> 32.5 RPM average
 *   50% command -> 87 RPM average
 *
 * The region above 50% is an initial extrapolation and will be validated
 * before PI output is enabled.
 */
static constexpr float RPM_AT_25_PERCENT = 32.5f;
static constexpr float RPM_AT_50_PERCENT = 87.0f;
static constexpr float RPM_AT_100_PERCENT = 175.0f;

}  // namespace

WheelController::WheelController()
    : outputCommand_{0, 0, 0, 0},
      encoderFeedback_{0, 0},
      previousEncoderFeedback_{0, 0},
      lastUpdateMs_(0),
      lastSpeedUpdateMs_(0),
      lastControlUpdateMs_(0),
      leftCountsPerSecond_(0.0f),
      rightCountsPerSecond_(0.0f),
      leftRpm_(0.0f),
      rightRpm_(0.0f),
      leftTargetRpm_(0.0f),
      rightTargetRpm_(0.0f),
      leftErrorRpm_(0.0f),
      rightErrorRpm_(0.0f),
      leftIntegral_(0.0f),
      rightIntegral_(0.0f),
      leftCorrectionMille_(0.0f),
      rightCorrectionMille_(0.0f),
      havePreviousEncoderSample_(false),
      speedEstimateValid_(false) {
}

void WheelController::reset() {
  outputCommand_ = {0, 0, 0, 0};

  encoderFeedback_ = {0, 0};
  previousEncoderFeedback_ = {0, 0};

  lastUpdateMs_ = 0;
  lastSpeedUpdateMs_ = 0;
  lastControlUpdateMs_ = 0;

  leftCountsPerSecond_ = 0.0f;
  rightCountsPerSecond_ = 0.0f;

  leftRpm_ = 0.0f;
  rightRpm_ = 0.0f;

  clearControlState();

  havePreviousEncoderSample_ = false;
  speedEstimateValid_ = false;
}

void WheelController::update(
    const ThrottleCmd& desired,
    const EncoderFb& encoderFeedback,
    bool armed,
    uint32_t nowMs) {
  encoderFeedback_ = encoderFeedback;
  lastUpdateMs_ = nowMs;

  updateSpeedEstimate(
      encoderFeedback,
      nowMs);

  updateControlState(
      desired,
      armed,
      nowMs);
}

void WheelController::updateSpeedEstimate(
    const EncoderFb& encoderFeedback,
    uint32_t nowMs) {
  if (!havePreviousEncoderSample_) {
    previousEncoderFeedback_ = encoderFeedback;
    lastSpeedUpdateMs_ = nowMs;
    havePreviousEncoderSample_ = true;
    speedEstimateValid_ = false;
    return;
  }

  const uint32_t elapsedMs =
      nowMs - lastSpeedUpdateMs_;

  if (elapsedMs < CONTROL_PERIOD_MS) {
    return;
  }

  if (elapsedMs > MAX_VALID_DT_MS) {
    previousEncoderFeedback_ = encoderFeedback;
    lastSpeedUpdateMs_ = nowMs;

    leftCountsPerSecond_ = 0.0f;
    rightCountsPerSecond_ = 0.0f;

    leftRpm_ = 0.0f;
    rightRpm_ = 0.0f;

    speedEstimateValid_ = false;
    return;
  }

  const int32_t leftDelta =
      wrappedCountDelta(
          encoderFeedback.left_count,
          previousEncoderFeedback_.left_count);

  const int32_t rightDelta =
      wrappedCountDelta(
          encoderFeedback.right_count,
          previousEncoderFeedback_.right_count);

  const float elapsedSeconds =
      static_cast<float>(elapsedMs) / 1000.0f;

  const float rawLeftCountsPerSecond =
      static_cast<float>(leftDelta) /
      elapsedSeconds;

  const float rawRightCountsPerSecond =
      static_cast<float>(rightDelta) /
      elapsedSeconds;

  leftCountsPerSecond_ =
      SPEED_FILTER_ALPHA * rawLeftCountsPerSecond +
      (1.0f - SPEED_FILTER_ALPHA) *
          leftCountsPerSecond_;

  rightCountsPerSecond_ =
      SPEED_FILTER_ALPHA * rawRightCountsPerSecond +
      (1.0f - SPEED_FILTER_ALPHA) *
          rightCountsPerSecond_;

  leftRpm_ =
      (leftCountsPerSecond_ /
       LEFT_COUNTS_PER_REV) *
      60.0f;

  rightRpm_ =
      (rightCountsPerSecond_ /
       RIGHT_COUNTS_PER_REV) *
      60.0f;

  if (fabsf(leftRpm_) < STOPPED_RPM_THRESHOLD) {
    leftRpm_ = 0.0f;
    leftCountsPerSecond_ = 0.0f;
  }

  if (fabsf(rightRpm_) < STOPPED_RPM_THRESHOLD) {
    rightRpm_ = 0.0f;
    rightCountsPerSecond_ = 0.0f;
  }

  previousEncoderFeedback_ = encoderFeedback;
  lastSpeedUpdateMs_ = nowMs;
  speedEstimateValid_ = true;
}

void WheelController::updateControlState(
    const ThrottleCmd& desired,
    bool armed,
    uint32_t nowMs) {
  /*
   * Pass-through is always the default.
   *
   * If PI output is disabled, this remains the final motor command.
   */
  outputCommand_ = desired;

  if (!armed) {
    outputCommand_ = {0, 0, 0, 0};
    clearControlState();
    lastControlUpdateMs_ = nowMs;
    return;
  }

  const uint32_t elapsedMs =
      nowMs - lastControlUpdateMs_;

  if (lastControlUpdateMs_ != 0 &&
      elapsedMs < CONTROL_PERIOD_MS) {
    return;
  }

  float elapsedSeconds =
      static_cast<float>(CONTROL_PERIOD_MS) /
      1000.0f;

  if (lastControlUpdateMs_ != 0 &&
      elapsedMs <= MAX_VALID_DT_MS) {
    elapsedSeconds =
        static_cast<float>(elapsedMs) /
        1000.0f;
  }

  lastControlUpdateMs_ = nowMs;

  const int16_t desiredLeftCommand =
      averageSideCommand(
          desired.fl,
          desired.rl);

  const int16_t desiredRightCommand =
      averageSideCommand(
          desired.fr,
          desired.rr);

  leftTargetRpm_ =
      commandToTargetRpm(
          desiredLeftCommand);

  rightTargetRpm_ =
      commandToTargetRpm(
          desiredRightCommand);

  leftErrorRpm_ =
      leftTargetRpm_ - leftRpm_;

  rightErrorRpm_ =
      rightTargetRpm_ - rightRpm_;

  const bool leftRequested =
      abs(desiredLeftCommand) >=
      THROTTLE_DEADBAND;

  const bool rightRequested =
      abs(desiredRightCommand) >=
      THROTTLE_DEADBAND;

  if (!leftRequested) {
    leftTargetRpm_ = 0.0f;
    leftErrorRpm_ = 0.0f;
    leftIntegral_ = 0.0f;
    leftCorrectionMille_ = 0.0f;
  } else if (ENABLE_WHEEL_PI_CONTROL) {
    leftIntegral_ +=
        leftErrorRpm_ *
        elapsedSeconds;

    if (leftIntegral_ > INTEGRAL_LIMIT) {
      leftIntegral_ = INTEGRAL_LIMIT;
    } else if (leftIntegral_ < -INTEGRAL_LIMIT) {
      leftIntegral_ = -INTEGRAL_LIMIT;
    }

    leftCorrectionMille_ =
        KP * leftErrorRpm_ +
        KI * leftIntegral_;
  } else {
    /*
     * Shadow mode:
     *
     * Calculate proportional correction for diagnostics, but do not
     * accumulate integral or apply any correction.
     */
    leftIntegral_ = 0.0f;

    leftCorrectionMille_ =
        KP * leftErrorRpm_;
  }

  if (!rightRequested) {
    rightTargetRpm_ = 0.0f;
    rightErrorRpm_ = 0.0f;
    rightIntegral_ = 0.0f;
    rightCorrectionMille_ = 0.0f;
  } else if (ENABLE_WHEEL_PI_CONTROL) {
    rightIntegral_ +=
        rightErrorRpm_ *
        elapsedSeconds;

    if (rightIntegral_ > INTEGRAL_LIMIT) {
      rightIntegral_ = INTEGRAL_LIMIT;
    } else if (rightIntegral_ < -INTEGRAL_LIMIT) {
      rightIntegral_ = -INTEGRAL_LIMIT;
    }

    rightCorrectionMille_ =
        KP * rightErrorRpm_ +
        KI * rightIntegral_;
  } else {
    rightIntegral_ = 0.0f;

    rightCorrectionMille_ =
        KP * rightErrorRpm_;
  }

  if (leftCorrectionMille_ >
      CORRECTION_LIMIT_MILLE) {
    leftCorrectionMille_ =
        CORRECTION_LIMIT_MILLE;
  } else if (leftCorrectionMille_ <
             -CORRECTION_LIMIT_MILLE) {
    leftCorrectionMille_ =
        -CORRECTION_LIMIT_MILLE;
  }

  if (rightCorrectionMille_ >
      CORRECTION_LIMIT_MILLE) {
    rightCorrectionMille_ =
        CORRECTION_LIMIT_MILLE;
  } else if (rightCorrectionMille_ <
             -CORRECTION_LIMIT_MILLE) {
    rightCorrectionMille_ =
        -CORRECTION_LIMIT_MILLE;
  }

  if (!ENABLE_WHEEL_PI_CONTROL) {
    /*
     * Critical safety behavior for this milestone:
     *
     * Corrections are visible in debug output but are not applied.
     */
    outputCommand_ = desired;
    return;
  }

  outputCommand_.fl =
      clampCommandMille(
          static_cast<float>(desired.fl) +
          leftCorrectionMille_);

  outputCommand_.rl =
      clampCommandMille(
          static_cast<float>(desired.rl) +
          leftCorrectionMille_);

  outputCommand_.fr =
      clampCommandMille(
          static_cast<float>(desired.fr) +
          rightCorrectionMille_);

  outputCommand_.rr =
      clampCommandMille(
          static_cast<float>(desired.rr) +
          rightCorrectionMille_);
}

void WheelController::clearControlState() {
  leftTargetRpm_ = 0.0f;
  rightTargetRpm_ = 0.0f;

  leftErrorRpm_ = 0.0f;
  rightErrorRpm_ = 0.0f;

  leftIntegral_ = 0.0f;
  rightIntegral_ = 0.0f;

  leftCorrectionMille_ = 0.0f;
  rightCorrectionMille_ = 0.0f;
}

int32_t WheelController::wrappedCountDelta(
    int32_t currentCount,
    int32_t previousCount) {
  const uint32_t currentUnsigned =
      static_cast<uint32_t>(currentCount);

  const uint32_t previousUnsigned =
      static_cast<uint32_t>(previousCount);

  return static_cast<int32_t>(
      currentUnsigned -
      previousUnsigned);
}

int16_t WheelController::averageSideCommand(
    int16_t frontCommand,
    int16_t rearCommand) {
  const int32_t total =
      static_cast<int32_t>(frontCommand) +
      static_cast<int32_t>(rearCommand);

  return static_cast<int16_t>(
      total / 2);
}

float WheelController::commandToTargetRpm(
    int16_t sideCommand) {
  const float commandMagnitude =
      fabsf(
          static_cast<float>(
              sideCommand));

  float targetMagnitudeRpm = 0.0f;

  if (commandMagnitude <= 250.0f) {
    targetMagnitudeRpm =
        (commandMagnitude / 250.0f) *
        RPM_AT_25_PERCENT;
  } else if (commandMagnitude <= 500.0f) {
    const float fraction =
        (commandMagnitude - 250.0f) /
        250.0f;

    targetMagnitudeRpm =
        RPM_AT_25_PERCENT +
        fraction *
            (RPM_AT_50_PERCENT -
             RPM_AT_25_PERCENT);
  } else {
    const float limitedMagnitude =
        commandMagnitude > 1000.0f
            ? 1000.0f
            : commandMagnitude;

    const float fraction =
        (limitedMagnitude - 500.0f) /
        500.0f;

    targetMagnitudeRpm =
        RPM_AT_50_PERCENT +
        fraction *
            (RPM_AT_100_PERCENT -
             RPM_AT_50_PERCENT);
  }

  if (sideCommand < 0) {
    return -targetMagnitudeRpm;
  }

  return targetMagnitudeRpm;
}

int16_t WheelController::clampCommandMille(
    float command) {
  if (command >
      static_cast<float>(THROTTLE_MAX)) {
    return THROTTLE_MAX;
  }

  if (command <
      static_cast<float>(THROTTLE_MIN)) {
    return THROTTLE_MIN;
  }

  if (command >= 0.0f) {
    return static_cast<int16_t>(
        command + 0.5f);
  }

  return static_cast<int16_t>(
      command - 0.5f);
}

const ThrottleCmd& WheelController::output() const {
  return outputCommand_;
}

const EncoderFb& WheelController::encoderFeedback() const {
  return encoderFeedback_;
}

float WheelController::leftCountsPerSecond() const {
  return leftCountsPerSecond_;
}

float WheelController::rightCountsPerSecond() const {
  return rightCountsPerSecond_;
}

float WheelController::leftRpm() const {
  return leftRpm_;
}

float WheelController::rightRpm() const {
  return rightRpm_;
}

float WheelController::leftTargetRpm() const {
  return leftTargetRpm_;
}

float WheelController::rightTargetRpm() const {
  return rightTargetRpm_;
}

float WheelController::leftErrorRpm() const {
  return leftErrorRpm_;
}

float WheelController::rightErrorRpm() const {
  return rightErrorRpm_;
}

float WheelController::leftCorrectionMille() const {
  return leftCorrectionMille_;
}

float WheelController::rightCorrectionMille() const {
  return rightCorrectionMille_;
}

float WheelController::speedMismatchPercent() const {
  const float leftMagnitude =
      fabsf(leftRpm_);

  const float rightMagnitude =
      fabsf(rightRpm_);

  const float averageMagnitude =
      (leftMagnitude +
       rightMagnitude) *
      0.5f;

  if (averageMagnitude <
      STOPPED_RPM_THRESHOLD) {
    return 0.0f;
  }

  return
      (fabsf(
           leftMagnitude -
           rightMagnitude) /
       averageMagnitude) *
      100.0f;
}

uint32_t WheelController::lastUpdateMs() const {
  return lastUpdateMs_;
}

bool WheelController::speedEstimateValid() const {
  return speedEstimateValid_;
}

bool WheelController::piOutputEnabled() const {
  return ENABLE_WHEEL_PI_CONTROL;
}

}  // namespace carl
