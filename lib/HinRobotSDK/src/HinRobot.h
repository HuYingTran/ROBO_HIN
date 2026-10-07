#pragma once
// ROBO HIN SDK — header tổng. Ứng dụng chỉ cần #include <HinRobot.h>.

#include <atomic>
#include <cstdint>

#include "comm/GcodeParser.h"
#include "comm/ProgramRunner.h"
#include "comm/SerialProtocol.h"
#include "comm/WebUi.h"
#include "comm/WifiManager.h"
#include "common/Math.h"
#include "common/Status.h"
#include "common/Types.h"
#include "core/Joint.h"
#include "drivers/A4988.h"
#include "drivers/IGripper.h"
#include "drivers/MT6701I2c.h"
#include "drivers/ServoGripper.h"
#include "drivers/Tca9548a.h"
#include "hal/I2cBus.h"
#include "hal/StepPulseGen.h"
#include "kinematics/ScaraKinematics.h"
#include "motion/Planner.h"
#include "safety/SafetyMonitor.h"
#include "storage/CalibrationStore.h"
#include "storage/Programs.h"

namespace hin {

constexpr const char* kSdkVersion = "1.0.0";

class HinRobot;

// Tay kẹp đi qua hàng đợi chuyển động: lệnh kẹp chỉ chạy khi các lệnh di chuyển trước đã xong.
class QueuedGripper : public IGripper {
public:
    explicit QueuedGripper(HinRobot& r) : robot_(r) {}
    void setPosition(float percent) override;
    float position() const override;

private:
    HinRobot& robot_;
};

class HinRobot {
public:
    explicit HinRobot(const RobotConfig& cfg);
    HinRobot(const HinRobot&) = delete;
    HinRobot& operator=(const HinRobot&) = delete;

    // Gọi từ setup() (core 1): khởi tạo HAL, driver, đọc encoder, nạp hiệu chuẩn, tạo task.
    Status begin();

    // ---- Driver ----
    Status enable();
    void disable();

    // ---- Homing ----
    Status home();        // chặn tới khi xong
    Status startHome();   // không chặn
    bool isHomed() const { return planner_.homed(); }

    // ---- Chuyển động (không chặn — xếp hàng đợi) ----
    Status moveJ(const JointPose& target, Speed speed);
    Status moveL(const CartPose& target, Speed speed);
    // Chạy kiểu khớp (PTP) tới một tư thế Descartes, nghiệm khuỷu theo setElbow().
    Status moveJPose(const CartPose& target, Speed speed);
    Status dwell(uint32_t ms);
    void setElbow(ElbowConfig e) { elbow_ = e; }
    ElbowConfig elbow() const { return elbow_; }
    Status jog(JointId joint, float vel);  // mm/s (J1) hoặc °/s; 0 = dừng
    // Jog Descartes: axis 0..3 = X, Y, Z (mm/s), φ (°/s). Cần đã homing.
    Status jogCartesian(int axis, float vel);
    bool isJogging() const { return planner_.jogging(); }
    // Hệ số tốc độ toàn cục 1..100 %, áp cả cho lệnh đang chạy.
    void setSpeedOverride(float percent) { planner_.setSpeedOverride(percent / 100.0f); }
    float speedOverride() const { return planner_.speedOverride() * 100.0f; }
    void stop();
    void emergencyStop();
    bool isIdle() const;
    Status waitIdle(uint32_t timeoutMs = UINT32_MAX);

    // ---- Trạng thái ----
    JointPose jointPositions() const;     // đo từ encoder
    JointPose commandedPositions() const; // điểm đặt hiện tại
    JointPose queuedTarget() const;       // đích của lệnh cuối trong hàng đợi
    CartPose toolPose() const;
    RobotState state() const;
    Fault lastFault() const { return fault_; }
    const char* lastErrorString() const { return errBuf_; }
    Status clearFault();
    // Theo dõi lệnh: id của lệnh vừa xếp hàng (bởi task gọi gần nhất), id lệnh đã chạy xong,
    // và lệnh bị planner hủy (vd. đường moveL đi qua vùng không với tới).
    uint32_t lastQueuedId() const { return lastPushedId_; }
    uint32_t lastDoneId() const { return planner_.lastDoneId(); }
    uint32_t lastPlanErrorId() const { return planner_.lastErrorId(); }
    Status lastPlanError() const { return planner_.lastPlanError(); }
    // Nghiệm khuỷu hiện tại theo dấu θ3
    ElbowConfig currentElbow() const;

    // ---- Hiệu chuẩn ----
    Status calibrateZero(JointId joint);
    Status saveCalibration();

    // ---- Động học ----
    bool fk(const JointPose& q, CartPose& out) const;
    bool ik(const CartPose& p, JointPose& out) const;

    // ---- Truy cập lớp dưới ----
    IGripper& gripper() { return queuedGripper_; }
    ServoGripper& gripperServo() { return servo_; }
    Joint& joint(int i) { return joints_[i]; }
    int jointCount() const { return n_; }
    SafetyMonitor& safety() { return safety_; }
    const ScaraKinematics& kinematics() const { return kin_; }
    const RobotConfig& config() const { return cfg_; }

    // Dùng nội bộ (QueuedGripper)
    Status queueGripper(float percent);

private:
    enum class Base : uint8_t { Disabled, Enabled, Fault, EStop };

    static void motionTaskFn(void* arg);
    static void encoderTaskFn(void* arg);
    static void safetyTaskFn(void* arg);

    void raiseFault(Fault f);
    Status fail(Status s, const char* fmt = nullptr, ...);
    Status checkCanMove();
    Status push(MotionCommand& c);
    float speedFraction(Speed s) const;
    float speedMmS(Speed s) const;

    const RobotConfig& cfg_;
    int n_;
    I2cBus bus_;
    Tca9548a mux_;
    MT6701I2c enc_[kMaxJoints];
    A4988 act_[kMaxJoints];
    Joint joints_[kMaxJoints];
    ServoGripper servo_;
    QueuedGripper queuedGripper_;
    ScaraKinematics kin_;
    Planner planner_;
    SafetyMonitor safety_;
    CalibrationStore store_;
    CalibrationData cal_;

    std::atomic<Base> base_{Base::Disabled};
    std::atomic<Fault> fault_{Fault::None};
    ElbowConfig elbow_ = ElbowConfig::Nearest;
    JointPose lastQueued_{};
    std::atomic<uint32_t> nextId_{1};
    std::atomic<uint32_t> lastPushedId_{0};
    char errBuf_[96] = "";
    bool begun_ = false;
};

}  // namespace hin
