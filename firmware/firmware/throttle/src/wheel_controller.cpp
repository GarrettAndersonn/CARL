#include "wheel_controller.h"

namespace carl {

WheelController::WheelController()
    : outputCommand_{0, 0, 0, 0},
      encoderFeedback_{0, 0},
      lastUpdateMs_(0) {
}

void WheelController::reset() {
  outputCommand_ = {0, 0, 0, 0};
  encoderFeedback_ = {0, 0};
  lastUpdateMs_ = 0;
}

void WheelController::update(
    const ThrottleCmd& desired,
    const EncoderFb& encoderFeedback,
    bool armed,
    uint32_t nowMs) {
  encoderFeedback_ = encoderFeedback;
  lastUpdateMs_ = nowMs;

  if (!armed) {
    outputCommand_ = {0, 0, 0, 0};
    return;
  }

  /*
   * Pass-through mode.
   *
   * This deliberately preserves the current open-loop behavior. No scaling,
   * balancing, PI control, or encoder-based correction is performed yet.
   */
  outputCommand_ = desired;
}

const ThrottleCmd& WheelController::output() const {
  return outputCommand_;
}

const EncoderFb& WheelController::encoderFeedback() const {
  return encoderFeedback_;
}

uint32_t WheelController::lastUpdateMs() const {
  return lastUpdateMs_;
}

}  // namespace carl
