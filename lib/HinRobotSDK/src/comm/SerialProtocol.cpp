#if defined(ARDUINO)
#include "SerialProtocol.h"

#include <Arduino.h>

#include <cstring>

#include "../HinRobot.h"
#include "GcodeParser.h"

namespace hin {

void SerialProtocol::begin(HinRobot& robot, Stream& io) {
    robot_ = &robot;
    io_ = &io;
}

void SerialProtocol::startTask(uint8_t core, uint8_t priority) {
    xTaskCreatePinnedToCore(taskFn, "hinComm", 6144, this, priority, nullptr, core);
}

void SerialProtocol::taskFn(void* arg) {
    auto* self = static_cast<SerialProtocol*>(arg);
    for (;;) {
        self->poll();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void SerialProtocol::poll() {
    // Báo lỗi bất đồng bộ (following error, E-stop, ...)
    const uint8_t f = static_cast<uint8_t>(robot_->lastFault());
    if (f != lastFault_) {
        lastFault_ = f;
        if (f != 0) io_->printf("!fault:%s %s\n", toString(robot_->lastFault()), robot_->lastErrorString());
    }

    while (io_->available() > 0) {
        const int c = io_->read();
        if (c == '\r') continue;
        if (c == '\n') {
            buf_[len_] = '\0';
            if (overflow_) {
                io_->println("error:2 InvalidArgument dong qua dai");
            } else if (len_ > 0) {
                handleLine(buf_, *io_);
            }
            len_ = 0;
            overflow_ = false;
        } else if (len_ < kLineMax - 1) {
            buf_[len_++] = static_cast<char>(c);
        } else {
            overflow_ = true;
        }
    }
}

void SerialProtocol::handleLine(const char* line, Print& out) {
    GcodeLine g;
    Status s = parseGcode(line, g);
    if (s != Status::Ok) {
        out.printf("error:%d %s cu phap\n", static_cast<int>(s), toString(s));
        return;
    }
    if (g.letter == 0) return;  // dòng chú thích
    s = execute(g, out);
    if (s == Status::Ok)
        out.println("ok");
    else
        reportError(s, out);
}

void SerialProtocol::reportError(Status s, Print& out) {
    // Chỉ dùng mô tả của robot nếu nó thuộc đúng lỗi này (tránh in mô tả cũ)
    const char* msg = robot_->lastErrorString();
    const char* name = toString(s);
    const bool match = msg && strncmp(msg, name, strlen(name)) == 0;
    out.printf("error:%d %s\n", static_cast<int>(s), match ? msg : name);
}

void SerialProtocol::printPosition(Print& out) {
    const JointPose q = robot_->jointPositions();
    const CartPose p = robot_->toolPose();
    out.printf("J:%.2f,%.2f,%.2f,%.2f X:%.1f Y:%.1f Z:%.1f A:%.1f\n", q[0], q[1], q[2], q[3], p.x, p.y,
               p.z, p.phi);
}

void SerialProtocol::printSwitches(Print& out) {
    SafetyMonitor& sm = robot_->safety();
    out.printf("estop:%d zmin:%d zmax:%d enc:", sm.estopActive(), sm.zMinActive(), sm.zMaxActive());
    for (int i = 0; i < robot_->jointCount(); ++i) {
        IEncoder* e = robot_->joint(i).encoder();
        out.printf("%s%s", i ? "," : "", e ? toString(e->fieldStatus()) : "-");
    }
    out.printf(" ferr:");
    for (int i = 0; i < robot_->jointCount(); ++i)
        out.printf("%s%.3f", i ? "," : "", robot_->joint(i).followingError());
    out.printf(" state:%s homed:%d fault:%s\n", toString(robot_->state()), robot_->isHomed(),
               toString(robot_->lastFault()));
}

Status SerialProtocol::execute(const GcodeLine& g, Print& out) {
    HinRobot& r = *robot_;

    if (g.letter == 'G') {
        switch (g.code) {
            case 0: {  // moveJ — khớp bỏ trống giữ nguyên đích trước
                JointPose q = r.queuedTarget();
                for (int i = 0; i < kMaxJoints; ++i)
                    if (g.hasJoint(i)) q[i] = g.joints[i];
                if (g.has(GcodeLine::kF)) lastG0Pct_ = g.f;
                return r.moveJ(q, Speed::percent(lastG0Pct_));
            }
            case 1: {  // moveL
                CartPose p;
                r.fk(r.queuedTarget(), p);
                if (g.has(GcodeLine::kX)) p.x = g.x;
                if (g.has(GcodeLine::kY)) p.y = g.y;
                if (g.has(GcodeLine::kZ)) p.z = g.z;
                if (g.has(GcodeLine::kA)) p.phi = g.a;
                if (g.has(GcodeLine::kF)) lastG1MmS_ = g.f;
                return r.moveL(p, Speed::mmPerSec(lastG1MmS_));
            }
            case 4:  // G4 P<ms>
                return r.dwell(static_cast<uint32_t>(g.has(GcodeLine::kP) ? g.p : 0));
            case 28:
                return r.startHome();
            case 92: {  // G92 J2=0 J3=0 … : lấy vị trí hiện tại làm 0
                if (g.jointMask == 0) return Status::InvalidArgument;
                for (int i = 0; i < kMaxJoints; ++i) {
                    if (!g.hasJoint(i)) continue;
                    if (g.joints[i] != 0) return Status::Unsupported;
                    const Status s = r.calibrateZero(static_cast<JointId>(i));
                    if (s != Status::Ok) return s;
                }
                return Status::Ok;
            }
        }
    } else {
        switch (g.code) {
            case 3:  // đóng tay kẹp
                r.gripper().setPosition(g.has(GcodeLine::kS) ? g.s : 100.0f);
                return Status::Ok;
            case 5:
                r.gripper().open();
                return Status::Ok;
            case 17: return r.enable();
            case 18: r.disable(); return Status::Ok;
            case 50:
                if (g.flag == 'L') r.setElbow(ElbowConfig::Left);
                else if (g.flag == 'R') r.setElbow(ElbowConfig::Right);
                else if (g.flag == 'N') r.setElbow(ElbowConfig::Nearest);
                else return Status::InvalidArgument;
                return Status::Ok;
            case 112: r.emergencyStop(); return Status::Ok;
            case 114: printPosition(out); return Status::Ok;
            case 119: printSwitches(out); return Status::Ok;
            case 400: return r.waitIdle();   // chờ hết hàng đợi
            case 410: r.stop(); return Status::Ok;  // dừng có giảm tốc
            case 500: return r.saveCalibration();
            case 999: return r.clearFault();
        }
    }
    return Status::Unsupported;
}

}  // namespace hin
#endif
