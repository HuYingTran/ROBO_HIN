#if defined(ARDUINO)
#include "Planner.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <cmath>

#include "../common/Log.h"
#include "../common/Math.h"
#include "../core/Joint.h"
#include "../drivers/IGripper.h"
#include "../kinematics/ScaraKinematics.h"

namespace hin {

namespace {
constexpr float kEps = 1e-4f;
constexpr float kSettleTime = 0.03f;     // s — chờ encoder có mẫu mới sau khi dừng
constexpr float kSingularDeg = 0.5f;     // |θ3| nhỏ hơn → coi như duỗi thẳng
constexpr float kMinCartSpeed = 0.5f;    // mm/s
constexpr int kCtrlQueueLen = 8;

inline QueueHandle_t q(void* h) { return static_cast<QueueHandle_t>(h); }

float lerpAngle(float a, float d, float s) { return wrapDeg(a + d * s); }
}  // namespace

bool Planner::begin(Joint* joints, int count, const ScaraKinematics* kin, IGripper* gripper,
                    const RobotConfig* cfg) {
    joints_ = joints;
    n_ = count;
    kin_ = kin;
    gripper_ = gripper;
    cfg_ = cfg;
    queue_ = xQueueCreate(kQueueLen, sizeof(MotionCommand));
    ctrlQueue_ = xQueueCreate(kCtrlQueueLen, sizeof(Ctrl));
    homing_.configure(cfg->homing, cfg->joints[cfg->homing.joint].amax);
    return queue_ && ctrlQueue_;
}

// ---------------------------------------------------------------------------
// API cho task khác
// ---------------------------------------------------------------------------

Status Planner::enqueue(const MotionCommand& cmd, uint32_t timeoutMs) {
    if (!enabled_) return Status::NotReady;
    const TickType_t ticks = timeoutMs == UINT32_MAX ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
    return xQueueSend(q(queue_), &cmd, ticks) == pdTRUE ? Status::Ok : Status::QueueFull;
}

void Planner::requestStop() {
    const Ctrl c{CtrlType::Stop, 0, 0};
    xQueueSend(q(ctrlQueue_), &c, 0);
}

void Planner::requestAbort() {
    abortReq_ = true;
    for (int i = 0; i < n_; ++i) joints_[i].hold();
}

void Planner::requestJog(int joint, float vel) {
    if (joint < 0 || joint >= n_) return;
    const Ctrl c{CtrlType::Jog, static_cast<int8_t>(joint), vel};
    xQueueSend(q(ctrlQueue_), &c, 0);
}

void Planner::requestHome() {
    const Ctrl c{CtrlType::Home, 0, 0};
    xQueueSend(q(ctrlQueue_), &c, 0);
}

void Planner::requestSetPosition(int joint, float value) {
    if (joint < 0 || joint >= n_) return;
    const Ctrl c{CtrlType::SetPosition, static_cast<int8_t>(joint), value};
    xQueueSend(q(ctrlQueue_), &c, 0);
}

void Planner::requestCartJog(int axis, float vel) {
    if (axis < 0 || axis > 3) return;
    const Ctrl c{CtrlType::CartJog, static_cast<int8_t>(axis), vel};
    xQueueSend(q(ctrlQueue_), &c, 0);
}

void Planner::setSpeedOverride(float f) { override_ = clamp(f, 0.01f, 1.0f); }

ElbowConfig Planner::lineElbowFor(const JointPose& q, ElbowConfig pref) {
    if (std::fabs(q[2]) < kSingularDeg)
        return pref == ElbowConfig::Right ? ElbowConfig::Right : ElbowConfig::Left;
    return q[2] > 0 ? ElbowConfig::Left : ElbowConfig::Right;
}

void Planner::setEnabled(bool on) {
    if (on) {
        abortReq_ = false;  // abort cũ chưa xử lý không được xóa lệnh mới
        resyncReq_ = true;
    } else {
        requestAbort();
        xQueueReset(q(queue_));
    }
    enabled_ = on;
}

bool Planner::busy() const {
    return mode_ != Mode::Idle || uxQueueMessagesWaiting(q(queue_)) > 0;
}

JointPose Planner::setpoint() const {
    JointPose p{};
    for (int i = 0; i < n_; ++i) p[i] = setpoint_[i];
    return p;
}

// ---------------------------------------------------------------------------
// motionTask
// ---------------------------------------------------------------------------

void Planner::syncSetpointToCommanded() {
    for (int i = 0; i < n_; ++i) {
        setpoint_[i] = prevSetpoint_[i] = joints_[i].commanded();
        vel_[i] = velTarget_[i] = 0;
    }
}

void Planner::flushQueue() {
    xQueueReset(q(queue_));
    Ctrl c;
    while (xQueueReceive(q(ctrlQueue_), &c, 0) == pdTRUE) {
    }
}

void Planner::planError(Status s, uint32_t id) {
    lastError_ = s;
    lastErrorId_ = id;
    HIN_LOGW("lenh %u bi huy: %s", static_cast<unsigned>(id), toString(s));
    xQueueReset(q(queue_));  // các lệnh sau phụ thuộc lệnh này → bỏ hết
    mode_ = Mode::Idle;
}

void Planner::tick(float dt, bool zLimitActive) {
    if (abortReq_.exchange(false)) {
        flushQueue();
        homing_.abort();
        mode_ = Mode::Idle;
        for (int i = 0; i < n_; ++i) joints_[i].hold();
        syncSetpointToCommanded();
    }

    if (!enabled_) {
        for (int i = 0; i < n_; ++i) joints_[i].hold();
        if (wasEnabled_) flushQueue();
        wasEnabled_ = false;
        mode_ = Mode::Idle;
        syncSetpointToCommanded();
        return;
    }
    if (resyncReq_.exchange(false) || !wasEnabled_) {
        // Vừa bật: lấy vị trí encoder làm vị trí lệnh để không giật
        homing_.abort();
        mode_ = Mode::Idle;
        for (int i = 0; i < n_; ++i)
            if (joints_[i].encoderValid()) joints_[i].syncToEncoder();
        syncSetpointToCommanded();
        wasEnabled_ = true;
    }

    processControl();

    for (int i = 0; i < n_; ++i) prevSetpoint_[i] = setpoint_[i];

    switch (mode_) {
        case Mode::Idle: {
            MotionCommand c;
            if (xQueueReceive(q(queue_), &c, 0) == pdTRUE) startCommand(c);
            break;
        }
        case Mode::Trajectory: runTrajectory(dt); break;
        case Mode::Velocity: runVelocity(dt); break;
        case Mode::CartVelocity: runCartVelocity(dt); break;
        case Mode::Homing: runHoming(dt, zLimitActive); break;
        case Mode::Dwell:
            dwellLeft_ -= dt;
            if (dwellLeft_ <= 0) {
                lastDoneId_ = cur_.id;
                mode_ = Mode::Idle;
            }
            break;
        case Mode::Settle: runSettle(dt); break;
    }

    for (int i = 0; i < n_; ++i) {
        if (mode_ != Mode::Velocity) vel_[i] = (setpoint_[i] - prevSetpoint_[i]) / dt;
        joints_[i].drive(setpoint_[i], dt);
    }
}

void Planner::processControl() {
    Ctrl c;
    while (xQueueReceive(q(ctrlQueue_), &c, 0) == pdTRUE) {
        switch (c.type) {
            case CtrlType::Stop:
                xQueueReset(q(queue_));
                if (mode_ == Mode::Trajectory || mode_ == Mode::Velocity) {
                    // vel_ đang giữ vận tốc hiện tại của từng khớp → giảm tốc về 0
                    for (int i = 0; i < n_; ++i) velTarget_[i] = 0;
                    stopping_ = true;
                    mode_ = Mode::Velocity;
                } else if (mode_ == Mode::Homing) {
                    homing_.abort();
                    for (int i = 0; i < n_; ++i) velTarget_[i] = 0;
                    stopping_ = true;
                    mode_ = Mode::Velocity;
                } else if (mode_ == Mode::CartVelocity) {
                    for (int i = 0; i < 4; ++i) cartVelTarget_[i] = 0;
                    stopping_ = true;
                } else if (mode_ == Mode::Dwell) {
                    mode_ = Mode::Idle;
                }
                break;
            case CtrlType::CartJog:
                if (!homed_) break;  // Z chưa có gốc → không nội suy Descartes
                if (mode_ == Mode::Idle || mode_ == Mode::Settle ||
                    (mode_ == Mode::CartVelocity && !stopping_)) {
                    if (mode_ != Mode::CartVelocity) {
                        for (int i = 0; i < 4; ++i) cartVel_[i] = cartVelTarget_[i] = 0;
                        const JointPose q0 = setpoint();
                        cartPose_ = kin_->fk(q0);
                        lineElbow_ = lineElbowFor(q0, ElbowConfig::Nearest);
                    }
                    const float vmax = c.joint == 3 ? joints_[3].config().vmax : cfg_->maxCartSpeedMmS;
                    cartVelTarget_[c.joint] = clamp(c.value, -vmax, vmax);
                    stopping_ = false;
                    mode_ = Mode::CartVelocity;
                }
                break;
            case CtrlType::Jog:
                if (mode_ == Mode::Idle || mode_ == Mode::Settle ||
                    (mode_ == Mode::Velocity && !stopping_)) {
                    if (mode_ != Mode::Velocity)
                        for (int i = 0; i < n_; ++i) vel_[i] = velTarget_[i] = 0;
                    const JointConfig& jc = joints_[c.joint].config();
                    velTarget_[c.joint] = clamp(c.value, -jc.vmax, jc.vmax);
                    stopping_ = false;
                    mode_ = Mode::Velocity;
                }
                break;
            case CtrlType::Home:
                if (mode_ == Mode::Idle || mode_ == Mode::Settle) {
                    const int j = cfg_->homing.joint;
                    homing_.start(setpoint_[j], false);
                    homed_ = false;
                    mode_ = Mode::Homing;
                }
                break;
            case CtrlType::SetPosition:
                if (mode_ == Mode::Idle || mode_ == Mode::Settle) {
                    joints_[c.joint].setCurrentAs(c.value);
                    setpoint_[c.joint] = prevSetpoint_[c.joint] = c.value;
                }
                setPosCount_ = setPosCount_ + 1;
                break;
        }
    }
}

void Planner::startCommand(const MotionCommand& c) {
    cur_ = c;
    switch (c.type) {
        case MotionCommand::Type::MoveJ:
            if (planMoveJ(c)) mode_ = Mode::Trajectory;
            break;
        case MotionCommand::Type::MoveL:
            if (planMoveL(c)) mode_ = Mode::Trajectory;
            break;
        case MotionCommand::Type::Gripper:
            if (gripper_) gripper_->setPosition(c.value);
            dwellLeft_ = cfg_->gripperSettleMs / 1000.0f;
            mode_ = Mode::Dwell;
            break;
        case MotionCommand::Type::Wait:
            dwellLeft_ = c.value / 1000.0f;
            mode_ = Mode::Dwell;
            break;
    }
    t_ = 0;
}

// Đồng bộ: chọn vmax/amax chuẩn hóa sao cho MỌI khớp không vượt giới hạn của nó
// → cùng bắt đầu, cùng kết thúc.
bool Planner::planMoveJ(const MotionCommand& c) {
    const float k = clamp(c.speed, 0.01f, 1.0f);
    float vn = INFINITY, an = INFINITY;
    for (int i = 0; i < n_; ++i) {
        start_[i] = setpoint_[i];
        delta_[i] = c.joints[i] - start_[i];
        const float d = std::fabs(delta_[i]);
        if (d < kEps) continue;
        const JointConfig& jc = joints_[i].config();
        vn = std::fmin(vn, jc.vmax * k / d);
        an = std::fmin(an, jc.amax / d);
    }
    if (std::isinf(vn)) {
        lastDoneId_ = c.id;  // đã ở đích
        return false;
    }
    prof_.plan(1.0f, vn, an);
    cur_.type = MotionCommand::Type::MoveJ;
    return true;
}

bool Planner::planMoveL(const MotionCommand& c) {
    JointPose q0 = setpoint();
    cartStart_ = kin_->fk(q0);
    cartDelta_.x = c.pose.x - cartStart_.x;
    cartDelta_.y = c.pose.y - cartStart_.y;
    cartDelta_.z = c.pose.z - cartStart_.z;
    cartDelta_.phi = angleDiffDeg(c.pose.phi, cartStart_.phi);

    // Không đổi nghiệm khuỷu giữa đường (không liên tục được)
    lineElbow_ = lineElbowFor(q0, c.elbow);

    // Quét IK dọc đường: phát hiện điểm không với tới + ước lượng dq/ds lớn nhất
    float maxDqDs[kMaxJoints] = {};
    JointPose prev = q0;
    for (int k = 1; k <= kScanSamples; ++k) {
        const float s = static_cast<float>(k) / kScanSamples;
        const CartPose p{cartStart_.x + cartDelta_.x * s, cartStart_.y + cartDelta_.y * s,
                         cartStart_.z + cartDelta_.z * s, lerpAngle(cartStart_.phi, cartDelta_.phi, s)};
        JointPose qk;
        const Status st = kin_->ik(p, lineElbow_, &prev, qk);
        if (st != Status::Ok) {
            planError(st, c.id);
            return false;
        }
        for (int i = 0; i < n_; ++i)
            maxDqDs[i] = std::fmax(maxDqDs[i], std::fabs(qk[i] - prev[i]) * kScanSamples);
        prev = qk;
    }

    const float len = std::sqrt(cartDelta_.x * cartDelta_.x + cartDelta_.y * cartDelta_.y +
                                cartDelta_.z * cartDelta_.z);
    const float vCart = clamp(c.speed, kMinCartSpeed, cfg_->maxCartSpeedMmS);
    float vs, as;
    if (len > kEps) {
        vs = vCart / len;
        as = cfg_->maxCartAccelMmS2 / len;
    } else if (std::fabs(cartDelta_.phi) > kEps) {
        // Chỉ xoay tay kẹp: dùng tỉ lệ tốc độ theo J4
        const JointConfig& w = joints_[3].config();
        vs = w.vmax * (vCart / cfg_->maxCartSpeedMmS) / std::fabs(cartDelta_.phi);
        as = w.amax / std::fabs(cartDelta_.phi);
    } else {
        lastDoneId_ = c.id;
        return false;
    }
    // Giới hạn theo khớp → tự giảm tốc gần điểm kỳ dị
    for (int i = 0; i < n_; ++i) {
        if (maxDqDs[i] < kEps) continue;
        const JointConfig& jc = joints_[i].config();
        vs = std::fmin(vs, jc.vmax / maxDqDs[i]);
        as = std::fmin(as, jc.amax / maxDqDs[i]);
    }
    prof_.plan(1.0f, vs, as);
    cur_.type = MotionCommand::Type::MoveL;
    return true;
}

void Planner::runTrajectory(float dt) {
    t_ += dt * override_.load();
    const bool done = t_ >= prof_.duration();
    const float s = done ? 1.0f : prof_.position(t_);

    if (cur_.type == MotionCommand::Type::MoveJ) {
        for (int i = 0; i < n_; ++i) setpoint_[i] = start_[i] + delta_[i] * s;
    } else {
        const CartPose p{cartStart_.x + cartDelta_.x * s, cartStart_.y + cartDelta_.y * s,
                         cartStart_.z + cartDelta_.z * s,
                         lerpAngle(cartStart_.phi, cartDelta_.phi, s)};
        JointPose ref = setpoint();
        JointPose qk;
        const Status st = kin_->ik(p, lineElbow_, &ref, qk);
        if (st != Status::Ok) {
            // Không xảy ra nếu bước quét đã qua; vẫn xử lý cho an toàn
            planError(st, cur_.id);
            for (int i = 0; i < n_; ++i) velTarget_[i] = 0;
            stopping_ = true;
            mode_ = Mode::Velocity;
            return;
        }
        for (int i = 0; i < n_; ++i) setpoint_[i] = qk[i];
    }

    if (done) finishMotion();
}

void Planner::finishMotion() {
    settleLeft_ = kSettleTime;
    mode_ = Mode::Settle;
}

void Planner::runSettle(float dt) {
    settleLeft_ -= dt;
    if (settleLeft_ > 0) return;
    // Bù sai số nhỏ: coi vị trí encoder là vị trí đã phát, drive() sẽ đẩy về điểm đặt
    for (int i = 0; i < n_; ++i) {
        Joint& j = joints_[i];
        if (!j.encoderValid()) continue;
        const bool lin = j.isPrismatic();
        const float dead = lin ? cfg_->correctionDeadbandMm : cfg_->correctionDeadbandDeg;
        const float lim = lin ? cfg_->followingErrorMm : cfg_->followingErrorDeg;
        const float e = std::fabs(j.followingError());
        if (e > dead && e < lim) j.syncToEncoder();
    }
    if (cur_.type == MotionCommand::Type::MoveJ || cur_.type == MotionCommand::Type::MoveL)
        lastDoneId_ = cur_.id;
    mode_ = Mode::Idle;
}

void Planner::runVelocity(float dt) {
    bool moving = false;
    for (int i = 0; i < n_; ++i) {
        const JointConfig& jc = joints_[i].config();
        float vt = velTarget_[i];

        // Soft limit: giảm tốc kịp trước giới hạn (J1 chưa homing thì bỏ qua)
        const bool checkLimit = !(i == cfg_->homing.joint && !homed_);
        if (checkLimit && vel_[i] != 0) {
            const float stopDist = vel_[i] * vel_[i] / (2.0f * jc.amax);
            if (vel_[i] > 0 && setpoint_[i] + stopDist >= jc.maxPos) vt = std::fmin(vt, 0.0f);
            if (vel_[i] < 0 && setpoint_[i] - stopDist <= jc.minPos) vt = std::fmax(vt, 0.0f);
        }

        const float dv = jc.amax * dt;
        vel_[i] += clamp(vt - vel_[i], -dv, dv);
        setpoint_[i] += vel_[i] * dt;
        if (checkLimit) setpoint_[i] = clamp(setpoint_[i], jc.minPos, jc.maxPos);
        if (vel_[i] != 0 || velTarget_[i] != 0) moving = true;
    }
    if (!moving) {
        stopping_ = false;
        finishMotion();
    }
}

void Planner::runCartVelocity(float dt) {
    bool moving = false;
    for (int a = 0; a < 4; ++a) {
        const float amax = a == 3 ? joints_[3].config().amax : cfg_->maxCartAccelMmS2;
        const float dv = amax * dt;
        cartVel_[a] += clamp(cartVelTarget_[a] - cartVel_[a], -dv, dv);
        if (cartVel_[a] != 0 || cartVelTarget_[a] != 0) moving = true;
    }
    if (!moving) {
        stopping_ = false;
        finishMotion();
        return;
    }
    const CartPose next{cartPose_.x + cartVel_[0] * dt, cartPose_.y + cartVel_[1] * dt,
                        cartPose_.z + cartVel_[2] * dt, wrapDeg(cartPose_.phi + cartVel_[3] * dt)};
    const JointPose ref = setpoint();
    JointPose q;
    bool ok = kin_->ik(next, lineElbow_, &ref, q) == Status::Ok;
    // Gần điểm kỳ dị / biên: khớp phải quay quá nhanh → dừng tại chỗ
    for (int i = 0; ok && i < n_; ++i)
        if (std::fabs(q[i] - ref[i]) > joints_[i].config().vmax * dt * 1.2f) ok = false;
    if (!ok) {
        for (int a = 0; a < 4; ++a) cartVel_[a] = cartVelTarget_[a] = 0;
        return;
    }
    cartPose_ = next;
    for (int i = 0; i < n_; ++i) setpoint_[i] = q[i];
}

void Planner::runHoming(float dt, bool zLimit) {
    const int j = cfg_->homing.joint;
    bool setZero = false;
    const Homing::Phase ph = homing_.tick(dt, zLimit, setpoint_[j], setZero);
    if (setZero) {
        joints_[j].setCurrentAs(0);
        prevSetpoint_[j] = 0;
    }
    if (ph == Homing::Phase::Done) {
        homed_ = true;
        finishMotion();
    } else if (ph == Homing::Phase::Failed) {
        pendingFault_ = homing_.failure();
        joints_[j].hold();
        setpoint_[j] = joints_[j].commanded();
        mode_ = Mode::Idle;
    }
}

}  // namespace hin
#endif
