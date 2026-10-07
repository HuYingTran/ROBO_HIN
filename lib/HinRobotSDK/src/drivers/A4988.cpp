#if defined(ARDUINO)
#include "A4988.h"

#include <Arduino.h>

#include "../hal/StepPulseGen.h"

namespace hin {

bool A4988::begin(int8_t stepPin, int8_t dirPin, int8_t enablePin, uint8_t microstep,
                  bool invert) {
    microstep_ = microstep;
    enPin_ = enablePin;
    if (enPin_ >= 0) {
        digitalWrite(enPin_, HIGH);  // tắt trước khi chuyển sang output
        pinMode(enPin_, OUTPUT);
        digitalWrite(enPin_, HIGH);
    }
    axis_ = StepPulseGen::addAxis(stepPin, dirPin, invert);
    return axis_ >= 0;
}

void A4988::enable(bool on) {
    if (!on) setVelocity(0);
    if (enPin_ >= 0) digitalWrite(enPin_, on ? LOW : HIGH);
}

void A4988::setVelocity(float stepsPerSec) { StepPulseGen::setVelocity(axis_, stepsPerSec); }
int32_t A4988::position() const { return StepPulseGen::position(axis_); }
void A4988::setPosition(int32_t steps) { StepPulseGen::setPosition(axis_, steps); }
float A4988::maxStepRate() const { return StepPulseGen::kMaxStepRate; }

}  // namespace hin
#endif
