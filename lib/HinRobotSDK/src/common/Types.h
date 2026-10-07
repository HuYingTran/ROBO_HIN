#pragma once
#include <cstdint>

namespace hin {

constexpr int kMaxJoints = 4;
constexpr int kJointZ = 0;  // J1 tịnh tiến

// ---------------------------------------------------------------------------
// Tư thế
// ---------------------------------------------------------------------------

// Không gian khớp: { Z (mm), θ2 (°), θ3 (°), θ4 (°) }
struct JointPose {
    float q[kMaxJoints];
    float& operator[](int i) { return q[i]; }
    float operator[](int i) const { return q[i]; }
};

// Không gian Descartes: { x, y, z (mm), φ góc tay kẹp (°) }
struct CartPose {
    float x, y, z, phi;
};

enum class ElbowConfig : uint8_t { Left, Right, Nearest };

enum class JointId : uint8_t { J1 = 0, J2, J3, J4 };

// Tốc độ: % tốc độ tối đa (moveJ) hoặc mm/s (moveL). Mỗi lệnh tự quy đổi khi cần.
struct Speed {
    enum class Kind : uint8_t { Percent, MmPerSec };
    Kind kind;
    float value;
    static constexpr Speed percent(float p) { return {Kind::Percent, p}; }
    static constexpr Speed mmPerSec(float v) { return {Kind::MmPerSec, v}; }
};

// ---------------------------------------------------------------------------
// Cấu hình
// ---------------------------------------------------------------------------

// Dùng namespace + enum thường (thay vì enum class) để `using hin::JointType::Prismatic;`
// hợp lệ trong C++17.
namespace JointType {
enum Value : uint8_t { Prismatic, Revolute };
}
namespace EncoderMount {
enum Value : uint8_t {
    Screw,   // đầu trục vít me — đếm vòng, mất vị trí khi tắt nguồn
    Output,  // trục ra khớp sau bộ truyền — góc tuyệt đối
};
}

struct JointConfig {
    const char* name;
    JointType::Value type;
    int8_t stepPin;
    int8_t dirPin;
    uint8_t encChannel;      // kênh TCA9548A
    uint8_t microstep;       // 1, 2, 4, 8, 16
    float ratio;             // T: mm/vòng vít me · R: tỉ số đai (vòng động cơ / vòng khớp)
    EncoderMount::Value encMount;
    bool invert;             // đảo chiều động cơ
    float minPos, maxPos;    // soft limit (mm | °)
    float vmax;              // (mm | °)/s
    float amax;              // (mm | °)/s²
    bool encoderInvert = false;  // đảo chiều encoder (nếu nam châm gắn ngược)
};

struct ScaraGeometry {
    float a2;  // J2 → J3 (mm)
    float a3;  // J3 → J4 (mm)
    float d4;  // mặt bích J4 → đầu tay kẹp, hướng xuống (mm)
    float z0;  // độ cao cánh tay khi d1 = 0 (mm)
};

struct HomingConfig {
    uint8_t joint;
    int8_t limitPin;
    int8_t direction;      // -1: chạy về phía âm
    float fastMmS;
    float slowMmS;
    float backoffMm;
    uint32_t timeoutMs;
    bool limitActiveLow = true;  // công tắc NO kéo xuống GND
    float liftMm = 5.0f;         // nâng lên sau homing
};

struct EncoderBusConfig {
    uint8_t port;
    int8_t sda;
    int8_t scl;
    uint32_t hz;
    uint8_t muxAddr;
    int8_t muxResetPin;
};

struct RobotConfig {
    const JointConfig* joints;
    uint8_t jointCount;
    ScaraGeometry geometry;
    HomingConfig homing;
    int8_t enablePin;              // active-LOW, chung cho mọi driver
    EncoderBusConfig encoderBus;
    int8_t gripperPin;
    int8_t estopPin;
    float followingErrorMm;        // J1
    float followingErrorDeg;       // J2–J4
    // ---- Tùy chọn (có giá trị mặc định) ----
    int8_t limitZMaxPin = -1;      // công tắc Z trên, -1 = không dùng
    bool estopActiveHigh = true;   // nút NC kéo xuống GND: bình thường LOW, nhấn → HIGH
    float maxCartSpeedMmS = 150.0f;
    float maxCartAccelMmS2 = 600.0f;
    uint16_t gripperMinUs = 1000;  // 0 % = mở
    uint16_t gripperMaxUs = 2000;  // 100 % = kẹp hết
    uint8_t gripperLedcChannel = 0;
    uint16_t gripperSettleMs = 400;
    uint8_t stepTimerIndex = 0;
    float correctionDeadbandMm = 0.01f;
    float correctionDeadbandDeg = 0.05f;
};

}  // namespace hin
