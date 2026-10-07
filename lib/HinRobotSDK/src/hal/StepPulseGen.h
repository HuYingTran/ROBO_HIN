#pragma once
#include <cstdint>

#include "../common/Status.h"

namespace hin {

// Phát xung STEP/DIR cho tối đa 4 trục bằng 1 timer phần cứng (ISR trong IRAM).
//
// Mỗi tick (kTickHz) ISR cộng vận tốc (Q0.32 bước/tick) vào bộ tích lũy của từng trục;
// tràn → phát 1 xung. Xung cao đúng 1 tick (20 µs ≥ 1 µs của A4988). Khi đổi chiều,
// DIR được ghi trước ít nhất 1 tick so với cạnh lên STEP kế tiếp (≥ 200 ns).
//
// Gọi begin() từ task chạy trên core 1 để ISR được cấp trên core 1.
class StepPulseGen {
public:
    static constexpr uint32_t kTickHz = 50000;
    static constexpr float kMaxStepRate = kTickHz / 2.0f;  // 25 kHz / trục
    static constexpr int kMaxAxes = 4;

    static Status begin(uint8_t timerIndex = 0);
    static bool running();

    // Trả chỉ số trục hoặc -1 nếu hết chỗ.
    static int addAxis(int8_t stepPin, int8_t dirPin, bool invert);

    // Vận tốc có dấu (bước/s), tự kẹp trong ±kMaxStepRate.
    static void setVelocity(int axis, float stepsPerSec);
    static void stopAll();
    static int32_t position(int axis);
    // Áp dụng trong ISR ở tick kế tiếp (lock-free); hàm chờ tới khi áp dụng xong.
    static void setPosition(int axis, int32_t pos);
};

}  // namespace hin
