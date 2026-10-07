#include "Homing.h"

#include <cmath>

namespace hin {

void Homing::configure(const HomingConfig& cfg, float amax) {
    cfg_ = cfg;
    amax_ = amax > 0 ? amax : 100.0f;
}

void Homing::start(float currentPos, bool limitActive) {
    elapsed_ = 0;
    vel_ = 0;
    failure_ = Fault::None;
    mark_ = currentPos;
    phase_ = limitActive ? Phase::Backoff : Phase::FastSeek;
}

Homing::Phase Homing::tick(float dt, bool limitActive, float& target, bool& setZero) {
    setZero = false;
    if (!active()) return phase_;

    elapsed_ += dt;
    if (elapsed_ * 1000.0f > static_cast<float>(cfg_.timeoutMs)) {
        fail(Fault::HomingTimeout);
        return phase_;
    }

    const float dir = cfg_.direction < 0 ? -1.0f : 1.0f;
    switch (phase_) {
        case Phase::FastSeek: {
            const float vTarget = dir * cfg_.fastMmS;
            const float dv = amax_ * dt;
            vel_ += std::fmax(-dv, std::fmin(dv, vTarget - vel_));
            target += vel_ * dt;
            if (limitActive) {
                vel_ = 0;  // dừng ngay tại công tắc
                mark_ = target;
                phase_ = Phase::Backoff;
            }
            break;
        }
        case Phase::Backoff:
            target -= dir * cfg_.fastMmS * dt;
            if (std::fabs(target - mark_) >= cfg_.backoffMm) {
                if (limitActive) {
                    fail(Fault::HomingFailed);  // công tắc kẹt
                } else {
                    mark_ = target;
                    phase_ = Phase::SlowSeek;
                }
            }
            break;
        case Phase::SlowSeek:
            target += dir * cfg_.slowMmS * dt;
            if (limitActive) {
                setZero = true;
                target = 0;
                phase_ = Phase::Lift;
            } else if (std::fabs(target - mark_) > 3.0f * cfg_.backoffMm + 1.0f) {
                fail(Fault::HomingFailed);  // đi quá xa mà không chạm lại
            }
            break;
        case Phase::Lift:
            target -= dir * cfg_.fastMmS * dt;
            if (std::fabs(target) >= cfg_.liftMm) {
                target = -dir * cfg_.liftMm;
                phase_ = Phase::Done;
            }
            break;
        default:
            break;
    }
    return phase_;
}

}  // namespace hin
