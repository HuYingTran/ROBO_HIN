#pragma once
#include <atomic>
#include <cstdint>

#include "../common/Status.h"
#include "../common/Types.h"
#include "../core/Homing.h"
#include "Profile.h"

namespace hin {

class Joint;
class IGripper;
class ScaraKinematics;

struct MotionCommand {
    enum class Type : uint8_t { MoveJ, MoveL, Gripper, Wait };
    Type type = Type::MoveJ;
    ElbowConfig elbow = ElbowConfig::Nearest;  // MoveL: nghiệm ưu tiên khi bắt đầu ở θ3 ≈ 0
    JointPose joints{};                        // MoveJ: đích khớp
    CartPose pose{};                           // MoveL: đích Descartes
    float speed = 0;                           // MoveJ: hệ số 0..1 · MoveL: mm/s
    float value = 0;                           // Gripper: % · Wait: ms
    uint32_t id = 0;
};

// Bộ lập quỹ đạo chạy trong motionTask (core 1, 1 kHz).
// Mọi hàm request*/enqueue an toàn khi gọi từ task khác.
class Planner {
public:
    static constexpr int kQueueLen = 32;
    static constexpr int kScanSamples = 64;  // số điểm kiểm tra IK dọc đường moveL

    bool begin(Joint* joints, int count, const ScaraKinematics* kin, IGripper* gripper,
               const RobotConfig* cfg);

    // ---- Gọi từ task khác ----
    Status enqueue(const MotionCommand& cmd, uint32_t timeoutMs);
    void requestStop();                 // giảm tốc về 0, xóa hàng đợi
    void requestAbort();                // dừng ngay (fault / E-stop)
    void requestJog(int joint, float vel);
    // Jog Descartes: axis 0..3 = X, Y, Z (mm/s), φ (°/s). vel = 0 → dừng trục đó.
    void requestCartJog(int axis, float vel);
    // Hệ số tốc độ 0.01..1 áp cho moveJ/moveL đang chạy (đổi được giữa chừng).
    void setSpeedOverride(float f);
    float speedOverride() const { return override_; }
    void requestHome();
    // Đặt vị trí hiện tại của khớp = value (hiệu chuẩn), thực hiện trong motionTask khi rảnh.
    void requestSetPosition(int joint, float value);
    void setEnabled(bool on);          // false: giữ vận tốc 0, điểm đặt bám theo vị trí lệnh

    // ---- Gọi từ motionTask ----
    void tick(float dt, bool zLimitActive);

    // ---- Trạng thái ----
    bool busy() const;
    bool homingActive() const { return mode_ == Mode::Homing; }
    bool jogging() const { return mode_ == Mode::Velocity || mode_ == Mode::CartVelocity; }
    bool homed() const { return homed_; }
    void setHomed(bool h) { homed_ = h; }
    Fault takeFault() { return pendingFault_.exchange(Fault::None); }
    Status lastPlanError() const { return lastError_; }
    uint32_t lastErrorId() const { return lastErrorId_; }
    uint32_t lastDoneId() const { return lastDoneId_; }
    JointPose setpoint() const;
    uint32_t setPositionCount() const { return setPosCount_; }

    // Nghiệm khuỷu dùng cho moveL bắt đầu tại q (không đổi nghiệm giữa đường).
    static ElbowConfig lineElbowFor(const JointPose& q, ElbowConfig pref);

private:
    enum class Mode : uint8_t { Idle, Trajectory, Velocity, CartVelocity, Homing, Dwell, Settle };
    enum class CtrlType : uint8_t { Stop, Jog, CartJog, Home, SetPosition };
    struct Ctrl {
        CtrlType type;
        int8_t joint;
        float value;
    };

    void processControl();
    void flushQueue();
    void startCommand(const MotionCommand& c);
    bool planMoveJ(const MotionCommand& c);
    bool planMoveL(const MotionCommand& c);
    void finishMotion();
    void runTrajectory(float dt);
    void runVelocity(float dt);
    void runCartVelocity(float dt);
    void runHoming(float dt, bool zLimit);
    void runSettle(float dt);
    void syncSetpointToCommanded();
    void planError(Status s, uint32_t id);

    Joint* joints_ = nullptr;
    int n_ = 0;
    const ScaraKinematics* kin_ = nullptr;
    IGripper* gripper_ = nullptr;
    const RobotConfig* cfg_ = nullptr;
    void* queue_ = nullptr;  // QueueHandle_t
    void* ctrlQueue_ = nullptr;

    volatile Mode mode_ = Mode::Idle;
    std::atomic<bool> abortReq_{false};
    std::atomic<bool> enabled_{false};
    std::atomic<bool> resyncReq_{false};
    bool wasEnabled_ = false;
    volatile bool homed_ = false;
    std::atomic<Fault> pendingFault_{Fault::None};
    volatile Status lastError_ = Status::Ok;
    volatile uint32_t lastErrorId_ = 0;
    volatile uint32_t lastDoneId_ = 0;
    volatile uint32_t setPosCount_ = 0;

    float setpoint_[kMaxJoints] = {};
    float prevSetpoint_[kMaxJoints] = {};
    float vel_[kMaxJoints] = {};
    float velTarget_[kMaxJoints] = {};

    // Quỹ đạo
    MotionCommand cur_{};
    TrapezoidProfile prof_;
    float t_ = 0;
    float start_[kMaxJoints] = {};
    float delta_[kMaxJoints] = {};
    CartPose cartStart_{};
    CartPose cartDelta_{};
    ElbowConfig lineElbow_ = ElbowConfig::Left;
    float cartVel_[kMaxJoints] = {};
    float cartVelTarget_[kMaxJoints] = {};
    CartPose cartPose_{};
    std::atomic<float> override_{1.0f};
    float dwellLeft_ = 0;
    float settleLeft_ = 0;
    bool stopping_ = false;

    Homing homing_;
};

}  // namespace hin
