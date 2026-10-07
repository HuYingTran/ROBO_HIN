#pragma once
#include "IActuator.h"

namespace hin {

// Driver A4988: STEP/DIR qua StepPulseGen, EN (active-LOW) có thể dùng chung giữa các driver.
// Vi bước đặt bằng jumper MS1–MS3, chỉ khai báo ở đây để đổi đơn vị.
class A4988 : public IActuator {
public:
    A4988() = default;

    // Gọi sau StepPulseGen::begin() hoặc trước đều được.
    bool begin(int8_t stepPin, int8_t dirPin, int8_t enablePin, uint8_t microstep, bool invert);

    void enable(bool on) override;
    void setVelocity(float stepsPerSec) override;
    int32_t position() const override;
    void setPosition(int32_t steps) override;
    float maxStepRate() const override;

    uint8_t microstep() const { return microstep_; }
    int axis() const { return axis_; }

private:
    int axis_ = -1;
    int8_t enPin_ = -1;
    uint8_t microstep_ = 16;
};

}  // namespace hin
