#pragma once
#include <cstdint>

namespace hin {

// Mã trả về của mọi hàm SDK (không dùng exception).
// Giá trị số được in ra Serial dạng "error:<code>", giữ ổn định giữa các phiên bản.
enum class Status : uint8_t {
    Ok = 0,
    Error = 1,
    InvalidArgument = 2,
    OutOfRange = 3,      // vượt soft limit khớp
    OutOfReach = 4,      // điểm Descartes ngoài tầm với
    JointLimit = 5,      // IK có nghiệm nhưng vi phạm giới hạn khớp
    NotReady = 6,        // trạng thái không cho phép lệnh này
    NotHomed = 7,
    Busy = 8,
    Timeout = 9,
    BusError = 10,       // I2C NACK / timeout
    Implausible = 11,    // giá trị encoder vô lý
    Faulted = 12,
    QueueFull = 13,
    Unsupported = 14,
    StorageError = 15,
};

// Lỗi làm robot chuyển sang trạng thái Fault / EStop.
enum class Fault : uint8_t {
    None = 0,
    EStop,
    StallDetected,     // following error vượt ngưỡng
    EncoderError,      // I2C lỗi liên tiếp / góc nhảy vô lý
    HomingTimeout,
    HomingFailed,
    LimitHit,          // chạm công tắc hành trình ngoài lúc homing
    Watchdog,
    Internal,
};

enum class RobotState : uint8_t {
    Disabled = 0,
    Unhomed,
    Idle,
    Homing,
    Moving,
    Fault,
    EStop,
};

enum class FieldStatus : uint8_t {
    Unknown = 0,
    Ok,
    Lost,   // đọc lỗi liên tục / nghi mất nam châm
};

const char* toString(Status s);
const char* toString(Fault f);
const char* toString(RobotState s);
const char* toString(FieldStatus f);

}  // namespace hin
