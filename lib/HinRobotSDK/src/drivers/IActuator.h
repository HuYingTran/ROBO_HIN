#pragma once
#include <cstdint>

namespace hin {

// Cơ cấu chấp hành điều khiển theo bước.
// Xung STEP do StepPulseGen phát trong ISR; lớp trên chỉ đặt vận tốc và đọc vị trí.
class IActuator {
public:
    virtual ~IActuator() = default;
    virtual void enable(bool on) = 0;
    // Vận tốc có dấu (microstep/s). Dương = chiều thuận của khớp.
    virtual void setVelocity(float stepsPerSec) = 0;
    // Vị trí đã phát (microstep).
    virtual int32_t position() const = 0;
    virtual void setPosition(int32_t steps) = 0;
    virtual float maxStepRate() const = 0;
};

}  // namespace hin
