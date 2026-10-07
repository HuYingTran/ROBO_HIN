#if defined(ARDUINO)
#include "MT6701I2c.h"

#include "../common/Math.h"
#include "../hal/I2cBus.h"
#include "Tca9548a.h"

namespace hin {

namespace {
constexpr uint16_t kLostAfterErrors = 3;
}

void MT6701I2c::attach(Tca9548a& mux, uint8_t channel) {
    mux_ = &mux;
    channel_ = channel;
}

void MT6701I2c::setMaxStep(uint16_t ticks) {
    filter_.setMaxStep(ticks == 0 ? mt6701::kHalfRev - 1 : ticks);
}

Status MT6701I2c::readOnce(uint16_t& raw) {
    Status s = mux_->select(channel_);
    if (s != Status::Ok) return s;
    const uint8_t reg = mt6701::kRegAngleHigh;
    uint8_t buf[2];
    s = mux_->bus().writeRead(mt6701::kI2cAddr, &reg, 1, buf, 2);
    if (s != Status::Ok) return s;
    raw = mt6701::decodeAngle(buf[0], buf[1]);
    return Status::Ok;
}

Status MT6701I2c::readRaw(uint16_t& ticks) {
    uint16_t raw = 0;
    Status s = readOnce(raw);
    if (s != Status::Ok) s = readOnce(raw);  // thử lại 1 lần
    if (s == Status::Ok && !filter_.feed(raw)) {
        // Góc nhảy vô lý → bỏ mẫu, đọc lại một lần
        s = readOnce(raw);
        if (s == Status::Ok && !filter_.feed(raw)) s = Status::Implausible;
    }
    if (s != Status::Ok) {
        if (errStreak_ < 0xFFFF) ++errStreak_;
        return s;
    }
    errStreak_ = 0;
    everOk_ = true;
    ticks = raw;
    return Status::Ok;
}

Status MT6701I2c::read(float& angleRad) {
    uint16_t t = 0;
    const Status s = readRaw(t);
    if (s == Status::Ok) angleRad = t * (2.0f * kPi / mt6701::kTicksPerRev);
    return s;
}

FieldStatus MT6701I2c::fieldStatus() const {
    if (errStreak_ >= kLostAfterErrors) return FieldStatus::Lost;
    return everOk_ ? FieldStatus::Ok : FieldStatus::Unknown;
}

}  // namespace hin
#endif
