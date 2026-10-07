#pragma once
#include "../common/Status.h"
#include "../common/Types.h"

namespace hin {

// Homing J1 bằng limit Z: chạy nhanh → lùi → tiếp cận chậm → đặt gốc → nâng lên.
// Không đụng phần cứng: mỗi tick nhận trạng thái công tắc, trả về điểm đặt mới.
class Homing {
public:
    enum class Phase : uint8_t { Idle, FastSeek, Backoff, SlowSeek, Lift, Done, Failed };

    void configure(const HomingConfig& cfg, float amax);
    void start(float currentPos, bool limitActive);
    void abort() { phase_ = Phase::Idle; }

    // target: điểm đặt hiện tại (mm), được cập nhật.
    // setZero: true đúng tick chạm công tắc lần 2 — caller đặt vị trí hiện tại = 0.
    Phase tick(float dt, bool limitActive, float& target, bool& setZero);

    Phase phase() const { return phase_; }
    bool active() const { return phase_ != Phase::Idle && phase_ != Phase::Done && phase_ != Phase::Failed; }
    Fault failure() const { return failure_; }

private:
    void fail(Fault f) {
        phase_ = Phase::Failed;
        failure_ = f;
    }

    HomingConfig cfg_{};
    float amax_ = 100;
    Phase phase_ = Phase::Idle;
    Fault failure_ = Fault::None;
    float elapsed_ = 0;
    float vel_ = 0;
    float mark_ = 0;
};

}  // namespace hin
