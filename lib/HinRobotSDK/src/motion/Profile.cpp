#include "Profile.h"

#include <cmath>

namespace hin {

bool TrapezoidProfile::plan(float distance, float vmax, float amax) {
    D_ = 0;
    vp_ = ta_ = tc_ = T_ = 0;
    if (distance < 0 || vmax <= 0 || amax <= 0) return false;
    D_ = distance;
    a_ = amax;
    if (distance == 0) return true;

    const float dAccel = vmax * vmax / amax;  // quãng tăng tốc + giảm tốc khi đạt vmax
    if (dAccel >= distance) {
        // Tam giác
        vp_ = std::sqrt(distance * amax);
        ta_ = vp_ / amax;
        tc_ = 0;
    } else {
        vp_ = vmax;
        ta_ = vmax / amax;
        tc_ = (distance - dAccel) / vmax;
    }
    T_ = 2 * ta_ + tc_;
    return true;
}

float TrapezoidProfile::position(float t) const {
    if (t <= 0) return 0;
    if (t >= T_) return D_;
    if (t < ta_) return 0.5f * a_ * t * t;
    const float dA = 0.5f * a_ * ta_ * ta_;
    if (t < ta_ + tc_) return dA + vp_ * (t - ta_);
    const float td = T_ - t;  // thời gian còn lại
    return D_ - 0.5f * a_ * td * td;
}

float TrapezoidProfile::velocity(float t) const {
    if (t <= 0 || t >= T_) return 0;
    if (t < ta_) return a_ * t;
    if (t < ta_ + tc_) return vp_;
    return a_ * (T_ - t);
}

}  // namespace hin
