#if defined(ARDUINO)
#include "ProgramRunner.h"

#include <cstdarg>
#include <cstdio>

#include "../HinRobot.h"

namespace hin {

void ProgramRunner::begin(HinRobot& robot, LogFn log, void* ctx) {
    robot_ = &robot;
    log_ = log;
    ctx_ = ctx;
}

void ProgramRunner::log(const char* fmt, ...) {
    if (!log_) return;
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    log_(ctx_, buf);
}

Status ProgramRunner::start(const Program& p, uint16_t loops) {
    if (p.count == 0) return Status::InvalidArgument;
    if (running_) stop();
    prog_ = p;
    loops_ = loops;
    loopsDone_ = 0;
    current_ = -1;
    next_ = 0;
    paused_ = false;
    running_ = true;
    log("Chay chuong trinh '%s' (%d buoc, %s)", prog_.name, prog_.count,
        loops == 0 ? "lap vo han" : loops == 1 ? "1 lan" : "nhieu lan");
    return Status::Ok;
}

void ProgramRunner::pause() {
    if (!running_ || paused_) return;
    paused_ = true;
    robot_->stop();
    if (current_ >= 0) next_ = current_;  // tiếp tục: chạy lại bước dang dở
    log("Tam dung tai buoc %d", next_ + 1);
}

void ProgramRunner::resume() {
    if (!running_ || !paused_) return;
    paused_ = false;
    log("Tiep tuc tu buoc %d", next_ + 1);
}

void ProgramRunner::stop() {
    if (!running_) return;
    running_ = false;
    paused_ = false;
    robot_->stop();
    current_ = -1;
    log("Dung chuong trinh '%s'", prog_.name);
}

Status ProgramRunner::issue(int i) {
    const ProgramStep& st = prog_.steps[i];
    const float pct = st.speed > 0 ? st.speed : prog_.speed;
    const CartPose pose{st.v[0], st.v[1], st.v[2], st.v[3]};
    Status s = Status::Ok;
    switch (st.op) {
        case ProgramStep::Op::MoveJ: s = robot_->moveJPose(pose, Speed::percent(pct)); break;
        case ProgramStep::Op::MoveJJoints: {
            const JointPose q{{st.v[0], st.v[1], st.v[2], st.v[3]}};
            s = robot_->moveJ(q, Speed::percent(pct));
            break;
        }
        case ProgramStep::Op::MoveL: s = robot_->moveL(pose, Speed::percent(pct)); break;
        case ProgramStep::Op::Wait: s = robot_->dwell(st.ms); break;
        case ProgramStep::Op::Gripper: s = robot_->queueGripper(st.gripper); break;
    }
    if (s == Status::Ok && st.gripper >= 0 && st.op != ProgramStep::Op::Gripper)
        s = robot_->queueGripper(st.gripper);
    return s;
}

void ProgramRunner::tick() {
    if (!running_ || paused_) return;

    const RobotState rs = robot_->state();
    if (rs == RobotState::Fault || rs == RobotState::EStop || rs == RobotState::Disabled) {
        paused_ = true;
        if (current_ >= 0) next_ = current_;
        log("Tam dung chuong trinh: robot %s", toString(rs));
        return;
    }
    if (!robot_->isIdle()) return;

    if (next_ >= prog_.count) {
        ++loopsDone_;
        if (loops_ != 0 && loopsDone_ >= loops_) {
            running_ = false;
            current_ = -1;
            log("Xong chuong trinh '%s'", prog_.name);
            return;
        }
        next_ = 0;
    }

    const Status s = issue(next_);
    if (s != Status::Ok) {
        log("Loi buoc %d: %s", next_ + 1, robot_->lastErrorString());
        running_ = false;
        current_ = -1;
        return;
    }
    current_ = next_++;
}

}  // namespace hin
#endif
