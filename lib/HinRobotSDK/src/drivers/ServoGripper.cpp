#if defined(ARDUINO)
#include "ServoGripper.h"

#include <Arduino.h>

#include "../common/Math.h"

namespace hin {

namespace {
constexpr uint32_t kFreqHz = 50;
constexpr uint8_t kResBits = 16;
constexpr uint32_t kPeriodUs = 1000000 / kFreqHz;
}  // namespace

bool ServoGripper::begin(int8_t pin, uint8_t ledcChannel, uint16_t minUs, uint16_t maxUs) {
    if (pin < 0) return false;
    pin_ = pin;
    channel_ = ledcChannel;
    minUs_ = minUs;
    maxUs_ = maxUs;
    if (ledcSetup(channel_, kFreqHz, kResBits) == 0) return false;
    ledcAttachPin(pin_, channel_);
    setPosition(0);
    return true;
}

void ServoGripper::setPosition(float percent) {
    if (pin_ < 0) return;
    percent_ = clamp(percent, 0.0f, 100.0f);
    const float us = minUs_ + (maxUs_ - minUs_) * percent_ / 100.0f;
    const uint32_t duty = static_cast<uint32_t>(us * ((1u << kResBits) - 1) / kPeriodUs);
    ledcWrite(channel_, duty);
}

void ServoGripper::release() {
    if (pin_ >= 0) ledcWrite(channel_, 0);
}

}  // namespace hin
#endif
