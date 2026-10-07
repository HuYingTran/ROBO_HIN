#pragma once
#include <cstdint>

#include "../common/Status.h"
#include "../common/Types.h"

namespace hin {

class Joint;

// Kiểm tra an toàn chạy trong safetyTask (1 kHz):
// E-stop, công tắc hành trình Z, following error, lỗi encoder.
class SafetyMonitor {
public:
    static constexpr uint16_t kMaxEncoderErrors = 5;

    void begin(const RobotConfig& cfg, Joint* joints, int count);

    // Gọi mỗi chu kỳ. Trả Fault::None nếu mọi thứ ổn.
    // driversEnabled: driver đang bật · homing: đang homing J1 (bỏ qua limit Z dưới).
    // homed: J1 đã có gốc → công tắc dưới kích hoạt gần Z = min là bình thường.
    Fault check(bool driversEnabled, bool homing, bool homed);

    bool estopActive() const { return estop_; }
    bool zMinActive() const { return zMin_; }
    bool zMaxActive() const { return zMax_; }
    // Khớp gây lỗi gần nhất (-1 nếu không áp dụng)
    int faultJoint() const { return faultJoint_; }
    float faultValue() const { return faultValue_; }

    // Đọc trực tiếp chân (dùng trong motionTask cho homing)
    bool readZMin() const;
    bool readEStop() const;

private:
    bool readPin(int8_t pin, bool activeHigh) const;

    const RobotConfig* cfg_ = nullptr;
    Joint* joints_ = nullptr;
    int n_ = 0;
    volatile bool estop_ = false, zMin_ = false, zMax_ = false;
    uint8_t estopCount_ = 0;
    float lastZCmd_ = 0;
    volatile int faultJoint_ = -1;
    volatile float faultValue_ = 0;
};

}  // namespace hin
