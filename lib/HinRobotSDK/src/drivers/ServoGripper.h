#pragma once
#include <cstdint>

#include "IGripper.h"

namespace hin {

// Tay kẹp servo RC, LEDC PWM 50 Hz.
class ServoGripper : public IGripper {
public:
    ServoGripper() = default;

    bool begin(int8_t pin, uint8_t ledcChannel, uint16_t minUs = 1000, uint16_t maxUs = 2000);
    void setPosition(float percent) override;
    float position() const override { return percent_; }
    void release();  // ngắt xung — servo thả lỏng

private:
    int8_t pin_ = -1;
    uint8_t channel_ = 0;
    uint16_t minUs_ = 1000, maxUs_ = 2000;
    float percent_ = 0;
};

}  // namespace hin
