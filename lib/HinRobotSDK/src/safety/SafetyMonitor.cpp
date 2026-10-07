#if defined(ARDUINO)
#include "SafetyMonitor.h"

#include <Arduino.h>

#include <cmath>

#include "../core/Joint.h"

namespace hin {

namespace {
constexpr uint8_t kEStopDebounce = 2;   // mẫu liên tiếp (2 ms)
constexpr float kZMoveEps = 1e-4f;      // mm / chu kỳ
constexpr float kHomeZoneMm = 1.0f;     // vùng quanh gốc Z cho phép công tắc kích
}  // namespace

void SafetyMonitor::begin(const RobotConfig& cfg, Joint* joints, int count) {
    cfg_ = &cfg;
    joints_ = joints;
    n_ = count;
    // GPIO 34–39 không có pull-up nội → cần điện trở kéo lên ngoài
    if (cfg.estopPin >= 0) pinMode(cfg.estopPin, INPUT);
    if (cfg.homing.limitPin >= 0) pinMode(cfg.homing.limitPin, INPUT);
    if (cfg.limitZMaxPin >= 0) pinMode(cfg.limitZMaxPin, INPUT);
}

bool SafetyMonitor::readPin(int8_t pin, bool activeHigh) const {
    if (pin < 0) return false;
    return (digitalRead(pin) == HIGH) == activeHigh;
}

bool SafetyMonitor::readZMin() const {
    return readPin(cfg_->homing.limitPin, !cfg_->homing.limitActiveLow);
}

bool SafetyMonitor::readEStop() const { return readPin(cfg_->estopPin, cfg_->estopActiveHigh); }

Fault SafetyMonitor::check(bool driversEnabled, bool homing, bool homed) {
    faultJoint_ = -1;

    // 1. E-stop (lọc nhiễu 2 mẫu)
    if (readEStop()) {
        if (estopCount_ < kEStopDebounce) ++estopCount_;
    } else {
        estopCount_ = 0;
    }
    estop_ = estopCount_ >= kEStopDebounce;
    if (estop_) return Fault::EStop;

    // 2. Công tắc Z — chỉ lỗi khi trục còn đang đi về phía công tắc
    const int jz = cfg_->homing.joint;
    const float zCmd = joints_[jz].commanded();
    const float dz = zCmd - lastZCmd_;
    lastZCmd_ = zCmd;
    zMin_ = readZMin();
    zMax_ = readPin(cfg_->limitZMaxPin, !cfg_->homing.limitActiveLow);
    const float towardMin = cfg_->homing.direction < 0 ? -dz : dz;
    // Homing đặt Z = 0 đúng tại điểm công tắc kích → quanh 0 công tắc có thể đang đóng
    const bool nearHome = homed && std::fabs(zCmd) < kHomeZoneMm;
    if (driversEnabled && !homing) {
        if ((zMin_ && towardMin > kZMoveEps && !nearHome) || (zMax_ && -towardMin > kZMoveEps)) {
            faultJoint_ = jz;
            return Fault::LimitHit;
        }
    }

    if (!driversEnabled) return Fault::None;

    // 3. Encoder + following error
    for (int i = 0; i < n_; ++i) {
        Joint& j = joints_[i];
        if (j.encoderErrorStreak() > kMaxEncoderErrors) {
            faultJoint_ = i;
            return Fault::EncoderError;
        }
        if (!j.encoderValid()) continue;
        const float e = j.followingError();
        const float lim = j.isPrismatic() ? cfg_->followingErrorMm : cfg_->followingErrorDeg;
        if (std::fabs(e) > lim) {
            faultJoint_ = i;
            faultValue_ = e;
            return Fault::StallDetected;
        }
    }
    return Fault::None;
}

}  // namespace hin
#endif
