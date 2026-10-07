#pragma once

namespace hin {

// Profile vận tốc hình thang cho quãng đường D ≥ 0.
// Nếu không đủ quãng đường để đạt vmax → profile tam giác.
// Thuần C++, không phụ thuộc Arduino.
class TrapezoidProfile {
public:
    // Trả false nếu tham số không hợp lệ (vmax/amax ≤ 0, D < 0).
    bool plan(float distance, float vmax, float amax);

    float position(float t) const;
    float velocity(float t) const;
    float duration() const { return T_; }
    float peakVelocity() const { return vp_; }
    float accelTime() const { return ta_; }

private:
    float D_ = 0, a_ = 1, vp_ = 0, ta_ = 0, tc_ = 0, T_ = 0;
};

}  // namespace hin
