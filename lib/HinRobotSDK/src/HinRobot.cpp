#if defined(ARDUINO)
#include "HinRobot.h"

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>

#include "common/Log.h"

namespace hin {

namespace {
constexpr BaseType_t kRtCore = 1;
constexpr UBaseType_t kPrioSafety = 22;
constexpr UBaseType_t kPrioMotion = 20;
constexpr UBaseType_t kPrioEncoder = 18;
constexpr uint32_t kEncoderPeriodMs = 4;          // 250 Hz
constexpr float kEncoderHz = 1000.0f / kEncoderPeriodMs;
constexpr uint32_t kBusRecoverMinIntervalMs = 100;
constexpr int kLineCheckSamples = 32;
}  // namespace

// ---------------------------------------------------------------------------
// QueuedGripper
// ---------------------------------------------------------------------------

void QueuedGripper::setPosition(float percent) { robot_.queueGripper(percent); }
float QueuedGripper::position() const { return const_cast<HinRobot&>(robot_).gripperServo().position(); }

// ---------------------------------------------------------------------------
// Khởi tạo
// ---------------------------------------------------------------------------

HinRobot::HinRobot(const RobotConfig& cfg)
    : cfg_(cfg),
      n_(cfg.jointCount > kMaxJoints ? kMaxJoints : cfg.jointCount),
      queuedGripper_(*this),
      kin_(cfg.geometry) {}

Status HinRobot::begin() {
    if (begun_) return Status::Ok;
    if (n_ != kMaxJoints) return fail(Status::InvalidArgument, "SCARA TRRR can 4 khop");

    // EN active-LOW: giữ driver tắt trong suốt quá trình khởi tạo
    if (cfg_.enablePin >= 0) {
        digitalWrite(cfg_.enablePin, HIGH);
        pinMode(cfg_.enablePin, OUTPUT);
        digitalWrite(cfg_.enablePin, HIGH);
    }

    // Bộ phát xung STEP + driver
    for (int i = 0; i < n_; ++i) {
        const JointConfig& jc = cfg_.joints[i];
        if (!act_[i].begin(jc.stepPin, jc.dirPin, cfg_.enablePin, jc.microstep, jc.invert))
            return fail(Status::InvalidArgument, "J%d chan STEP/DIR", i + 1);
        joints_[i].attach(jc, &act_[i], &enc_[i]);
    }
    if (StepPulseGen::begin(cfg_.stepTimerIndex) != Status::Ok)
        return fail(Status::Error, "khong cap duoc timer");

    // Encoder: I2C → TCA9548A → MT6701
    const EncoderBusConfig& eb = cfg_.encoderBus;
    bus_.configure(eb.port, eb.sda, eb.scl, eb.hz);
    if (bus_.begin() != Status::Ok) return fail(Status::BusError, "I2C khong khoi tao duoc");
    mux_.attach(bus_, eb.muxAddr, eb.muxResetPin);
    if (mux_.begin() != Status::Ok) {
        bus_.recover();
        if (mux_.begin() != Status::Ok)
            return fail(Status::BusError, "TCA9548A 0x%02X khong phan hoi", eb.muxAddr);
    }
    for (int i = 0; i < n_; ++i) {
        enc_[i].attach(mux_, cfg_.joints[i].encChannel);
        enc_[i].setMaxStep(0);  // lần đọc đầu: chưa lọc
    }

    // Hiệu chuẩn
    if (store_.load(cal_) == Status::Ok) {
        for (int i = 0; i < n_; ++i)
            if (cfg_.joints[i].encMount == EncoderMount::Output)
                joints_[i].setEncoderOffset(cal_.encoderOffset[i]);
        elbow_ = static_cast<ElbowConfig>(cal_.elbow);
        HIN_LOGI("da nap hieu chuan tu NVS");
    } else {
        HIN_LOGW("chua co hieu chuan trong NVS — dung offset 0");
    }

    for (int i = 0; i < n_; ++i) {
        Status s = Status::Error;
        for (int attempt = 0; attempt < 3 && s != Status::Ok; ++attempt)
            s = joints_[i].updateEncoder();
        if (s != Status::Ok)
            return fail(Status::BusError, "MT6701 J%d (CH%d): %s", i + 1,
                        cfg_.joints[i].encChannel, toString(s));
        enc_[i].setMaxStep(joints_[i].maxEncoderStep(kEncoderHz));
        joints_[i].syncToEncoder();
    }

    // Động học có lọc giới hạn khớp
    float mins[kMaxJoints], maxs[kMaxJoints];
    for (int i = 0; i < n_; ++i) {
        mins[i] = cfg_.joints[i].minPos;
        maxs[i] = cfg_.joints[i].maxPos;
    }
    kin_.setLimits(mins, maxs);

    if (cfg_.gripperPin >= 0)
        servo_.begin(cfg_.gripperPin, cfg_.gripperLedcChannel, cfg_.gripperMinUs, cfg_.gripperMaxUs);

    safety_.begin(cfg_, joints_, n_);
    if (!planner_.begin(joints_, n_, &kin_, &servo_, &cfg_))
        return fail(Status::Error, "khong tao duoc hang doi");
    lastQueued_ = commandedPositions();

    xTaskCreatePinnedToCore(safetyTaskFn, "hinSafety", 3072, this, kPrioSafety, nullptr, kRtCore);
    xTaskCreatePinnedToCore(motionTaskFn, "hinMotion", 4096, this, kPrioMotion, nullptr, kRtCore);
    xTaskCreatePinnedToCore(encoderTaskFn, "hinEncoder", 3072, this, kPrioEncoder, nullptr, kRtCore);

    begun_ = true;
    HIN_LOGI("ROBO HIN SDK %s san sang", kSdkVersion);
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Task thời gian thực (core 1)
// ---------------------------------------------------------------------------

void HinRobot::motionTaskFn(void* arg) {
    auto* self = static_cast<HinRobot*>(arg);
    esp_task_wdt_add(nullptr);
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = 1;  // CONFIG_FREERTOS_HZ = 1000 → 1 ms
    const float dt = period * portTICK_PERIOD_MS / 1000.0f;
    for (;;) {
        vTaskDelayUntil(&last, period);
        self->planner_.tick(dt, self->safety_.readZMin());
        const Fault f = self->planner_.takeFault();
        if (f != Fault::None) self->raiseFault(f);
        esp_task_wdt_reset();
    }
}

void HinRobot::safetyTaskFn(void* arg) {
    auto* self = static_cast<HinRobot*>(arg);
    esp_task_wdt_add(nullptr);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, 1);
        const Base b = self->base_;
        const Fault f = self->safety_.check(b == Base::Enabled, self->planner_.homingActive(),
                                            self->planner_.homed());
        if (f != Fault::None) self->raiseFault(f);
        esp_task_wdt_reset();
    }
}

void HinRobot::encoderTaskFn(void* arg) {
    auto* self = static_cast<HinRobot*>(arg);
    TickType_t last = xTaskGetTickCount();
    uint32_t lastRecover = 0;
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(kEncoderPeriodMs));
        bool busFault = false;
        for (int i = 0; i < self->n_; ++i) {
            const Status s = self->joints_[i].updateEncoder();
            if (s == Status::BusError && self->joints_[i].encoderErrorStreak() >= 2) busFault = true;
        }
        // Bus treo → 9 xung SCL + reset mux (giới hạn tần suất)
        if (busFault && millis() - lastRecover > kBusRecoverMinIntervalMs) {
            lastRecover = millis();
            self->mux_.reset();
            self->bus_.recover();
        }
        // J1 đếm vòng: mất encoder là mất gốc Z
        const int jz = self->cfg_.homing.joint;
        if (self->joints_[jz].encoderErrorStreak() > SafetyMonitor::kMaxEncoderErrors)
            self->planner_.setHomed(false);
    }
}

void HinRobot::raiseFault(Fault f) {
    const Base b = base_;
    if (b == Base::EStop) return;
    if (b == Base::Fault && f != Fault::EStop) return;
    planner_.setEnabled(false);
    if (cfg_.enablePin >= 0) digitalWrite(cfg_.enablePin, HIGH);
    base_ = (f == Fault::EStop) ? Base::EStop : Base::Fault;
    fault_ = f;
    const int j = safety_.faultJoint();
    if (j >= 0)
        snprintf(errBuf_, sizeof(errBuf_), "Fault %s J%d", toString(f), j + 1);
    else
        snprintf(errBuf_, sizeof(errBuf_), "Fault %s", toString(f));
}

// ---------------------------------------------------------------------------
// Trạng thái
// ---------------------------------------------------------------------------

RobotState HinRobot::state() const {
    switch (base_.load()) {
        case Base::Disabled: return RobotState::Disabled;
        case Base::Fault: return RobotState::Fault;
        case Base::EStop: return RobotState::EStop;
        case Base::Enabled: break;
    }
    if (planner_.homingActive()) return RobotState::Homing;
    if (planner_.busy()) return RobotState::Moving;
    return planner_.homed() ? RobotState::Idle : RobotState::Unhomed;
}

Status HinRobot::fail(Status s, const char* fmt, ...) {
    int n = snprintf(errBuf_, sizeof(errBuf_), "%s", toString(s));
    if (fmt && n > 0 && n < static_cast<int>(sizeof(errBuf_)) - 1) {
        errBuf_[n++] = ' ';
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(errBuf_ + n, sizeof(errBuf_) - n, fmt, ap);
        va_end(ap);
    }
    return s;
}

Status HinRobot::enable() {
    if (!begun_) return fail(Status::NotReady, "chua goi begin()");
    const Base b = base_;
    if (b == Base::Fault || b == Base::EStop) return fail(Status::Faulted, "%s — can M999", toString(fault_.load()));
    if (b == Base::Enabled) return Status::Ok;
    if (safety_.readEStop()) return fail(Status::NotReady, "E-stop dang nhan");
    // Vị trí lệnh = vị trí encoder trước khi bật để safety không báo following error giả
    for (int i = 0; i < n_; ++i)
        if (joints_[i].encoderValid()) joints_[i].syncToEncoder();
    lastQueued_ = jointPositions();
    if (cfg_.enablePin >= 0) digitalWrite(cfg_.enablePin, LOW);
    planner_.setEnabled(true);
    base_ = Base::Enabled;
    return Status::Ok;
}

void HinRobot::disable() {
    planner_.setEnabled(false);
    if (cfg_.enablePin >= 0) digitalWrite(cfg_.enablePin, HIGH);
    if (base_ == Base::Enabled) base_ = Base::Disabled;
}

Status HinRobot::clearFault() {
    const Base b = base_;
    if (b != Base::Fault && b != Base::EStop) return Status::Ok;
    if (safety_.readEStop()) return fail(Status::NotReady, "nha nut E-stop truoc");
    for (int i = 0; i < n_; ++i) {
        if (joints_[i].encoderErrorStreak() > 0 || fault_ == Fault::EncoderError) {
            enc_[i].resetFilter();
        }
    }
    fault_ = Fault::None;
    base_ = Base::Disabled;
    errBuf_[0] = '\0';
    return Status::Ok;
}

bool HinRobot::isIdle() const { return !planner_.busy(); }

Status HinRobot::waitIdle(uint32_t timeoutMs) {
    const uint32_t t0 = millis();
    // Lệnh vừa xếp hàng có thể chưa được motionTask lấy ra
    vTaskDelay(pdMS_TO_TICKS(2));
    while (planner_.busy()) {
        if (base_ != Base::Enabled) return fail(Status::Faulted, "%s", toString(fault_.load()));
        if (timeoutMs != UINT32_MAX && millis() - t0 > timeoutMs) return Status::Timeout;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return base_ == Base::Enabled ? Status::Ok : Status::Faulted;
}

JointPose HinRobot::jointPositions() const {
    JointPose p{};
    for (int i = 0; i < n_; ++i) p[i] = joints_[i].measured();
    return p;
}

JointPose HinRobot::commandedPositions() const {
    JointPose p{};
    for (int i = 0; i < n_; ++i) p[i] = const_cast<Joint&>(joints_[i]).commanded();
    return p;
}

CartPose HinRobot::toolPose() const { return kin_.fk(jointPositions()); }

bool HinRobot::fk(const JointPose& q, CartPose& out) const {
    out = kin_.fk(q);
    return true;
}

bool HinRobot::ik(const CartPose& p, JointPose& out) const {
    const JointPose ref = jointPositions();
    return kin_.ik(p, elbow_, &ref, out) == Status::Ok;
}

// ---------------------------------------------------------------------------
// Chuyển động
// ---------------------------------------------------------------------------

Status HinRobot::checkCanMove() {
    const Base b = base_;
    if (b == Base::Fault || b == Base::EStop) return fail(Status::Faulted, "%s", toString(fault_.load()));
    if (b != Base::Enabled) return fail(Status::NotReady, "driver dang tat (M17)");
    if (planner_.homingActive()) return fail(Status::Busy, "dang homing");
    if (!planner_.homed()) return fail(Status::NotHomed, "chay G28 truoc");
    return Status::Ok;
}

JointPose HinRobot::queuedTarget() const {
    return planner_.busy() ? lastQueued_ : planner_.setpoint();
}

float HinRobot::speedFraction(Speed s) const {
    const float f = s.kind == Speed::Kind::Percent ? s.value / 100.0f : s.value / cfg_.maxCartSpeedMmS;
    return clamp(f, 0.01f, 1.0f);
}

float HinRobot::speedMmS(Speed s) const {
    const float v = s.kind == Speed::Kind::MmPerSec ? s.value : s.value / 100.0f * cfg_.maxCartSpeedMmS;
    return clamp(v, 0.5f, cfg_.maxCartSpeedMmS);
}

Status HinRobot::push(MotionCommand& c) {
    c.id = nextId_++;
    lastPushedId_ = c.id;
    // Chờ chỗ trống trong hàng đợi, bỏ cuộc nếu robot lỗi
    for (;;) {
        const Status s = planner_.enqueue(c, 20);
        if (s == Status::Ok) return Status::Ok;
        if (base_ != Base::Enabled) return fail(Status::Faulted, "%s", toString(fault_.load()));
    }
}

Status HinRobot::moveJ(const JointPose& target, Speed speed) {
    Status s = checkCanMove();
    if (s != Status::Ok) return s;
    for (int i = 0; i < n_; ++i) {
        const JointConfig& jc = cfg_.joints[i];
        if (target[i] > jc.maxPos) return fail(Status::OutOfRange, "J%d max=%g", i + 1, jc.maxPos);
        if (target[i] < jc.minPos) return fail(Status::OutOfRange, "J%d min=%g", i + 1, jc.minPos);
    }
    if (!planner_.busy()) lastQueued_ = planner_.setpoint();
    MotionCommand c;
    c.type = MotionCommand::Type::MoveJ;
    c.joints = target;
    c.speed = speedFraction(speed);
    s = push(c);
    if (s == Status::Ok) lastQueued_ = target;
    return s;
}

Status HinRobot::moveJPose(const CartPose& target, Speed speed) {
    Status s = checkCanMove();
    if (s != Status::Ok) return s;
    const JointPose ref = queuedTarget();
    JointPose q;
    s = kin_.ik(target, elbow_, &ref, q);
    if (s == Status::OutOfReach)
        return fail(s, "r=%.0f max=%.0f", std::sqrt(target.x * target.x + target.y * target.y),
                    kin_.maxReach());
    if (s != Status::Ok) return fail(s);
    return moveJ(q, speed);
}

Status HinRobot::moveL(const CartPose& target, Speed speed) {
    Status s = checkCanMove();
    if (s != Status::Ok) return s;
    const JointPose start = queuedTarget();
    const ElbowConfig e = Planner::lineElbowFor(start, elbow_);

    // Kiểm tra cả đường thẳng ngay khi gọi để báo lỗi đồng bộ
    const CartPose c0 = kin_.fk(start);
    const float dphi = angleDiffDeg(target.phi, c0.phi);
    JointPose prev = start, q{};
    for (int k = 1; k <= kLineCheckSamples; ++k) {
        const float u = static_cast<float>(k) / kLineCheckSamples;
        const CartPose p{c0.x + (target.x - c0.x) * u, c0.y + (target.y - c0.y) * u,
                         c0.z + (target.z - c0.z) * u, wrapDeg(c0.phi + dphi * u)};
        s = kin_.ik(p, e, &prev, q);
        if (s == Status::OutOfReach) {
            const float r = std::sqrt(p.x * p.x + p.y * p.y);
            return k == kLineCheckSamples
                       ? fail(s, "r=%.0f max=%.0f", r, kin_.maxReach())
                       : fail(s, "duong thang di qua r=%.0f (min=%.0f)", r, kin_.minReach());
        }
        if (s != Status::Ok) return fail(s, "tai %.0f%% duong di", u * 100.0f);
        prev = q;
    }

    MotionCommand c;
    c.type = MotionCommand::Type::MoveL;
    c.pose = target;
    c.elbow = elbow_;
    c.speed = speedMmS(speed);
    s = push(c);
    if (s == Status::Ok) lastQueued_ = q;
    return s;
}

Status HinRobot::dwell(uint32_t ms) {
    const Base b = base_;
    if (b != Base::Enabled) return fail(Status::NotReady);
    MotionCommand c;
    c.type = MotionCommand::Type::Wait;
    c.value = static_cast<float>(ms);
    return push(c);
}

Status HinRobot::queueGripper(float percent) {
    if (base_ != Base::Enabled) {
        servo_.setPosition(percent);  // driver tắt → không có chuyển động nào để chờ
        return Status::Ok;
    }
    MotionCommand c;
    c.type = MotionCommand::Type::Gripper;
    c.value = percent;
    return push(c);
}

Status HinRobot::jog(JointId joint, float vel) {
    const Base b = base_;
    if (b != Base::Enabled) return fail(Status::NotReady, "driver dang tat");
    if (planner_.homingActive()) return fail(Status::Busy, "dang homing");
    planner_.requestJog(static_cast<int>(joint), vel);
    return Status::Ok;
}

Status HinRobot::jogCartesian(int axis, float vel) {
    const Base b = base_;
    if (b != Base::Enabled) return fail(Status::NotReady, "driver dang tat");
    if (!planner_.homed()) return fail(Status::NotHomed, "jog XYZ can homing truoc");
    if (axis < 0 || axis > 3) return fail(Status::InvalidArgument);
    planner_.requestCartJog(axis, vel);
    return Status::Ok;
}

ElbowConfig HinRobot::currentElbow() const {
    return joints_[2].measured() >= 0 ? ElbowConfig::Left : ElbowConfig::Right;
}

void HinRobot::stop() { planner_.requestStop(); }

void HinRobot::emergencyStop() { raiseFault(Fault::EStop); }

// ---------------------------------------------------------------------------
// Homing & hiệu chuẩn
// ---------------------------------------------------------------------------

Status HinRobot::startHome() {
    const Base b = base_;
    if (b == Base::Fault || b == Base::EStop) return fail(Status::Faulted, "%s", toString(fault_.load()));
    if (b != Base::Enabled) return fail(Status::NotReady, "driver dang tat (M17)");
    if (planner_.busy()) return fail(Status::Busy, "robot dang chay");
    if (cfg_.homing.limitPin < 0) return fail(Status::Unsupported, "chua cau hinh limit Z");
    planner_.requestHome();
    return Status::Ok;
}

Status HinRobot::home() {
    Status s = startHome();
    if (s != Status::Ok) return s;
    // Chờ motionTask nhận lệnh rồi chờ homing kết thúc
    const uint32_t t0 = millis();
    while (!planner_.homingActive() && millis() - t0 < 50) vTaskDelay(1);
    s = waitIdle(cfg_.homing.timeoutMs + 5000);
    if (s != Status::Ok) return s;
    return planner_.homed() ? Status::Ok : fail(Status::Error, "homing khong hoan tat");
}

Status HinRobot::calibrateZero(JointId joint) {
    const int j = static_cast<int>(joint);
    if (j < 0 || j >= n_) return fail(Status::InvalidArgument);
    if (!joints_[j].encoderValid()) return fail(Status::BusError, "encoder J%d chua co du lieu", j + 1);
    if (base_ == Base::Enabled) {
        if (planner_.busy()) return fail(Status::Busy, "robot dang chay");
        const uint32_t before = planner_.setPositionCount();
        planner_.requestSetPosition(j, 0.0f);
        const uint32_t t0 = millis();
        while (planner_.setPositionCount() == before) {
            if (millis() - t0 > 100) return fail(Status::Timeout);
            vTaskDelay(1);
        }
    } else {
        joints_[j].setCurrentAs(0.0f);  // driver tắt: motionTask chỉ bám theo vị trí lệnh
    }
    if (j == cfg_.homing.joint) planner_.setHomed(true);
    cal_.encoderOffset[j] = joints_[j].encoderOffset();
    lastQueued_ = planner_.setpoint();
    return Status::Ok;
}

Status HinRobot::saveCalibration() {
    for (int i = 0; i < n_; ++i) cal_.encoderOffset[i] = joints_[i].encoderOffset();
    cal_.elbow = static_cast<uint8_t>(elbow_);
    cal_.magic = CalibrationData::kMagic;
    const Status s = store_.save(cal_);
    return s == Status::Ok ? s : fail(s, "ghi NVS that bai");
}

}  // namespace hin
#endif
