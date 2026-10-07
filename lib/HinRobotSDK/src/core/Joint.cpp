#include "Joint.h"

#include <cmath>

#include "../common/Math.h"
#include "../drivers/IActuator.h"
#include "../drivers/IEncoder.h"

namespace hin {

void Joint::attach(const JointConfig& cfg, IActuator* act, IEncoder* enc) {
    cfg_ = &cfg;
    act_ = act;
    enc_ = enc;
    const float stepsPerRev = static_cast<float>(kMotorStepsPerRev) * cfg.microstep;
    spu_ = isPrismatic() ? stepsPerRev / cfg.ratio            // bước/mm
                         : stepsPerRev * cfg.ratio / 360.0f;  // bước/độ
    encSign_ = cfg.encoderInvert ? -1.0f : 1.0f;
}

float Joint::rawToUnits() const {
    if (cfg_->encMount == EncoderMount::Screw) {
        // Vòng vít me × bước vít (mm/vòng)
        return turns_.turns() * cfg_->ratio * encSign_ - offset_;
    }
    return wrapDeg(mt6701::ticksToDeg(lastRaw_) * encSign_ - offset_);
}

Status Joint::updateEncoder() {
    if (!enc_) return Status::Unsupported;
    uint16_t raw = 0;
    const Status s = enc_->readRaw(raw);
    if (s != Status::Ok) {
        if (errStreak_ < 0xFFFF) errStreak_ = errStreak_ + 1;
        return s;
    }
    errStreak_ = 0;
    lastRaw_ = raw;
    if (cfg_->encMount == EncoderMount::Screw) turns_.update(raw);
    measured_ = rawToUnits();
    encValid_ = true;
    return Status::Ok;
}

void Joint::invalidateEncoder() {
    encValid_ = false;
    errStreak_ = 0;
    turns_.invalidate();
    if (enc_) enc_->resetFilter();
}

uint16_t Joint::maxEncoderStep(float sampleHz) const {
    // 2× vmax + biên nhiễu 32 tick
    const float unitsPerSample = 2.0f * cfg_->vmax / sampleHz;
    const float revPerSample = isPrismatic() ? unitsPerSample / cfg_->ratio  // rev vít me
                                             : unitsPerSample / 360.0f;       // rev trục ra
    const float ticks = revPerSample * mt6701::kTicksPerRev + 32.0f;
    return static_cast<uint16_t>(clamp(ticks, 64.0f, static_cast<float>(mt6701::kHalfRev - 1)));
}

float Joint::commanded() const { return act_->position() / spu_; }

void Joint::drive(float target, float dt) {
    const float err = target * spu_ - static_cast<float>(act_->position());
    const float vmaxSteps = act_->maxStepRate();
    act_->setVelocity(clamp(err / dt, -vmaxSteps, vmaxSteps));
}

void Joint::hold() { act_->setVelocity(0); }

float Joint::followingError() const {
    const float e = measured_ - commanded();
    return isPrismatic() ? e : wrapDeg(e);
}

void Joint::syncToEncoder() {
    act_->setPosition(static_cast<int32_t>(std::lround(measured_ * spu_)));
}

void Joint::setCurrentAs(float value) {
    // measured = raw − offset  →  offset = raw − value
    offset_ = offset_ + (measured_ - value);
    measured_ = value;
    act_->setPosition(static_cast<int32_t>(std::lround(value * spu_)));
}

}  // namespace hin
