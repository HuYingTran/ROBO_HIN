#pragma once
#include <cstdint>

// Giải mã góc MT6701, đếm vòng và lọc bước nhảy — thuần C++, test được trên PC.
namespace hin {
namespace mt6701 {

constexpr uint8_t kI2cAddr = 0x06;
// Thanh ghi góc: 0x03 = angle[13:6], 0x04 bit[7:2] = angle[5:0].
// (Đối chiếu lại datasheet MT6701 nếu dùng revision khác.)
constexpr uint8_t kRegAngleHigh = 0x03;
constexpr uint8_t kRegAngleLow = 0x04;
constexpr int32_t kTicksPerRev = 16384;  // 14 bit
constexpr int32_t kHalfRev = kTicksPerRev / 2;

inline uint16_t decodeAngle(uint8_t reg03, uint8_t reg04) {
    return static_cast<uint16_t>((static_cast<uint16_t>(reg03) << 6) | (reg04 >> 2));
}

inline float ticksToDeg(int32_t ticks) { return ticks * (360.0f / kTicksPerRev); }

// Hiệu ngắn nhất now - prev trên vòng tròn, trong [-8192, 8191].
inline int32_t circularDelta(uint16_t now, uint16_t prev) {
    int32_t d = static_cast<int32_t>(now) - static_cast<int32_t>(prev);
    if (d >= kHalfRev) d -= kTicksPerRev;
    if (d < -kHalfRev) d += kTicksPerRev;
    return d;
}

// Bỏ mẫu có bước nhảy lớn hơn maxStep (≈ vmax × dt + biên nhiễu).
class JumpFilter {
public:
    void setMaxStep(uint16_t ticks) { maxStep_ = ticks; }
    uint16_t maxStep() const { return maxStep_; }
    void reset() { hasLast_ = false; }
    bool hasLast() const { return hasLast_; }
    uint16_t last() const { return last_; }

    // true = chấp nhận (và cập nhật last), false = loại bỏ.
    bool feed(uint16_t raw) {
        if (hasLast_) {
            int32_t d = circularDelta(raw, last_);
            if (d < 0) d = -d;
            if (d > maxStep_) return false;
        }
        last_ = raw;
        hasLast_ = true;
        return true;
    }

private:
    uint16_t maxStep_ = kHalfRev - 1;
    uint16_t last_ = 0;
    bool hasLast_ = false;
};

// Đếm vòng cho encoder gắn trên trục vít (J1).
// Yêu cầu: giữa 2 lần update trục quay < ½ vòng.
class TurnCounter {
public:
    void reset(uint16_t raw) {
        last_ = raw;
        total_ = raw;
        valid_ = true;
    }
    void update(uint16_t raw) {
        if (!valid_) {
            reset(raw);
            return;
        }
        total_ += circularDelta(raw, last_);
        last_ = raw;
    }
    // Đặt lại gốc: tổng tick hiện tại = 0.
    void zero() { total_ = 0; }
    bool valid() const { return valid_; }
    void invalidate() { valid_ = false; }
    int32_t totalTicks() const { return total_; }
    float turns() const { return static_cast<float>(total_) / kTicksPerRev; }

private:
    uint16_t last_ = 0;
    int32_t total_ = 0;
    bool valid_ = false;
};

}  // namespace mt6701
}  // namespace hin
