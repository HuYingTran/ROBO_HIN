#pragma once
#include "../common/Status.h"
#include "../common/Types.h"
#include "../drivers/Mt6701Decode.h"

namespace hin {

class IActuator;
class IEncoder;

// Một khớp: actuator + encoder + bộ truyền + giới hạn.
// Đơn vị khớp: mm (tịnh tiến) hoặc độ (quay).
//
// Luồng dữ liệu:
//   - encoderTask gọi updateEncoder() → measured()
//   - motionTask gọi drive() mỗi 1 ms → vận tốc STEP
//   - safetyTask đọc followingError()
class Joint {
public:
    static constexpr int kMotorStepsPerRev = 200;  // NEMA 17, 1.8°/bước

    Joint() = default;
    void attach(const JointConfig& cfg, IActuator* act, IEncoder* enc);

    const JointConfig& config() const { return *cfg_; }
    bool isPrismatic() const { return cfg_->type == JointType::Prismatic; }
    float stepsPerUnit() const { return spu_; }
    IActuator* actuator() { return act_; }
    IEncoder* encoder() { return enc_; }

    // ---- Encoder ----
    Status updateEncoder();
    bool encoderValid() const { return encValid_; }
    uint16_t encoderErrorStreak() const { return errStreak_; }
    float measured() const { return measured_; }
    void invalidateEncoder();
    // Bước nhảy encoder hợp lệ tối đa giữa 2 mẫu ở tần số lấy mẫu cho trước.
    uint16_t maxEncoderStep(float sampleHz) const;

    // ---- Lệnh ----
    float commanded() const;
    // Đặt vận tốc để vị trí lệnh đạt target sau dt giây.
    void drive(float target, float dt);
    void hold();
    // measured − commanded (độ: đã quy về [-180, 180)).
    float followingError() const;
    // Coi vị trí encoder là vị trí lệnh (khởi động / bù sai số sau khi dừng).
    void syncToEncoder();
    bool inLimits(float v) const { return v >= cfg_->minPos && v <= cfg_->maxPos; }

    // ---- Hiệu chuẩn ----
    // Vị trí đo hiện tại trở thành `value`; actuator đồng bộ theo.
    void setCurrentAs(float value);
    float encoderOffset() const { return offset_; }
    void setEncoderOffset(float off) { offset_ = off; }

private:
    float rawToUnits() const;

    const JointConfig* cfg_ = nullptr;
    IActuator* act_ = nullptr;
    IEncoder* enc_ = nullptr;
    float spu_ = 1;
    float encSign_ = 1;
    volatile float offset_ = 0;  // đơn vị khớp
    volatile float measured_ = 0;
    volatile bool encValid_ = false;
    uint16_t lastRaw_ = 0;
    volatile uint16_t errStreak_ = 0;
    mt6701::TurnCounter turns_;  // chỉ dùng cho EncoderMount::Screw
};

}  // namespace hin
