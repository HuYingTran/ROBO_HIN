#if defined(ARDUINO)
#include "StepPulseGen.h"

#include <Arduino.h>
#include <soc/gpio_reg.h>
#include <soc/soc.h>

namespace hin {

namespace {

struct AxisHw {
    uint32_t stepMask, dirMask;
    uint32_t stepSet, stepClr, dirSet, dirClr;  // địa chỉ thanh ghi W1TS / W1TC
    bool invert;
};

struct AxisState {
    volatile int32_t vel;         // Q0.32 bước/tick, có dấu, |vel| < 2^31
    volatile int32_t pos;
    volatile int32_t pendingPos;
    volatile bool hasPending;
    uint32_t acc;
    int8_t curDir;
    bool high;
};

inline void IRAM_ATTR writeReg(uint32_t addr, uint32_t val) {
    *reinterpret_cast<volatile uint32_t*>(addr) = val;
}

DRAM_ATTR AxisHw s_hw[StepPulseGen::kMaxAxes];
DRAM_ATTR AxisState s_st[StepPulseGen::kMaxAxes];
DRAM_ATTR volatile int s_count = 0;
hw_timer_t* s_timer = nullptr;

void IRAM_ATTR onTick() {
    const int n = s_count;
    for (int i = 0; i < n; ++i) {
        AxisState& a = s_st[i];
        const AxisHw& h = s_hw[i];

        if (a.high) {
            writeReg(h.stepClr, h.stepMask);
            a.high = false;
        }
        if (a.hasPending) {
            a.pos = a.pendingPos;
            a.hasPending = false;
        }

        const int32_t v = a.vel;
        if (v == 0) continue;
        const int8_t want = v > 0 ? 1 : -1;
        if (want != a.curDir) {
            const bool level = (want > 0) != h.invert;
            writeReg(level ? h.dirSet : h.dirClr, h.dirMask);
            a.curDir = want;
            continue;  // chờ 1 tick cho thời gian setup DIR
        }
        const uint32_t mag = v > 0 ? static_cast<uint32_t>(v) : static_cast<uint32_t>(-v);
        const uint32_t prev = a.acc;
        a.acc = prev + mag;
        // |vel| < 2^31 → không thể tràn ở 2 tick liên tiếp, nên tick vừa hạ STEP không phát xung.
        if (a.acc < prev) {
            writeReg(h.stepSet, h.stepMask);
            a.high = true;
            a.pos = a.pos + want;
        }
    }
}

void regsFor(int8_t pin, uint32_t& mask, uint32_t& setReg, uint32_t& clrReg) {
    if (pin < 32) {
        mask = 1u << pin;
        setReg = GPIO_OUT_W1TS_REG;
        clrReg = GPIO_OUT_W1TC_REG;
    } else {
        mask = 1u << (pin - 32);
        setReg = GPIO_OUT1_W1TS_REG;
        clrReg = GPIO_OUT1_W1TC_REG;
    }
}

}  // namespace

int StepPulseGen::addAxis(int8_t stepPin, int8_t dirPin, bool invert) {
    if (s_count >= kMaxAxes || stepPin < 0 || dirPin < 0) return -1;
    const int i = s_count;
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    digitalWrite(stepPin, LOW);
    AxisHw& h = s_hw[i];
    regsFor(stepPin, h.stepMask, h.stepSet, h.stepClr);
    regsFor(dirPin, h.dirMask, h.dirSet, h.dirClr);
    h.invert = invert;
    AxisState& a = s_st[i];
    a.vel = 0;
    a.pos = 0;
    a.hasPending = false;
    a.acc = 0;
    a.curDir = 0;  // ép ghi DIR ở xung đầu tiên
    a.high = false;
    s_count = i + 1;
    return i;
}

Status StepPulseGen::begin(uint8_t timerIndex) {
    if (s_timer) return Status::Ok;
    s_timer = timerBegin(timerIndex, 80, true);  // APB 80 MHz / 80 = 1 MHz
    if (!s_timer) return Status::Error;
    timerAttachInterrupt(s_timer, &onTick, false);
    timerAlarmWrite(s_timer, 1000000 / kTickHz, true);
    timerAlarmEnable(s_timer);
    return Status::Ok;
}

bool StepPulseGen::running() { return s_timer != nullptr; }

void StepPulseGen::setVelocity(int axis, float stepsPerSec) {
    if (axis < 0 || axis >= s_count) return;
    constexpr float kScale = 4294967296.0f / kTickHz;  // bước/s → Q0.32 bước/tick
    constexpr float kLimit = 2147483000.0f;
    float q = stepsPerSec * kScale;
    if (q > kLimit) q = kLimit;
    if (q < -kLimit) q = -kLimit;
    s_st[axis].vel = static_cast<int32_t>(q);
}

void StepPulseGen::stopAll() {
    for (int i = 0; i < s_count; ++i) s_st[i].vel = 0;
}

int32_t StepPulseGen::position(int axis) {
    if (axis < 0 || axis >= s_count) return 0;
    return s_st[axis].pos;
}

void StepPulseGen::setPosition(int axis, int32_t pos) {
    if (axis < 0 || axis >= s_count) return;
    AxisState& a = s_st[axis];
    if (!s_timer) {
        a.pos = pos;
        return;
    }
    a.pendingPos = pos;
    a.hasPending = true;
    const uint32_t t0 = micros();
    while (a.hasPending && micros() - t0 < 1000) {
    }
}

}  // namespace hin
#endif
