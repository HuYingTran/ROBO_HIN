// Example 03 — 1 khớp (J2) + encoder, phát hiện mất bước.
// Kiểm chứng: dùng tay chặn động cơ → following error vượt ngưỡng → báo stall, tắt driver.
//   pio run -e ex03 -t upload
#include <Arduino.h>
#include <HinRobot.h>

#include "robot_config.h"

constexpr int kJ = 1;  // J2
const hin::JointConfig& jc = JOINTS[kJ];
const hin::EncoderBusConfig& eb = ROBOT_CONFIG.encoderBus;

hin::I2cBus bus(eb.port, eb.sda, eb.scl, eb.hz);
hin::Tca9548a mux(bus, eb.muxAddr, eb.muxResetPin);
hin::MT6701I2c enc;
hin::A4988 motor;
hin::Joint joint;
hin::TrapezoidProfile profile;
bool stalled = false;

void setup() {
    Serial.begin(115200);
    delay(200);
    hin::StepPulseGen::begin();
    motor.begin(jc.stepPin, jc.dirPin, ROBOT_CONFIG.enablePin, jc.microstep, jc.invert);
    bus.begin();
    mux.begin();
    enc.attach(mux, jc.encChannel);
    joint.attach(jc, &motor, &enc);

    if (joint.updateEncoder() != hin::Status::Ok) {
        Serial.println("Khong doc duoc encoder J2");
        while (true) delay(1000);
    }
    enc.setMaxStep(joint.maxEncoderStep(250));
    joint.syncToEncoder();
    motor.enable(true);
    Serial.printf("J2 bat dau tai %.2f do\n", joint.measured());
}

// Chạy tới target, trả false nếu stall
bool moveTo(float target) {
    const float start = joint.commanded();
    const float dist = target - start;
    const float dir = dist >= 0 ? 1.0f : -1.0f;
    profile.plan(fabsf(dist), jc.vmax * 0.5f, jc.amax);
    TickType_t last = xTaskGetTickCount();
    int n = 0;
    for (float t = 0; t <= profile.duration() + 0.05f; t += 0.001f) {
        joint.drive(start + dir * profile.position(t + 0.001f), 0.001f);
        if (++n % 4 == 0) {  // encoder 250 Hz
            joint.updateEncoder();
            if (fabsf(joint.followingError()) > ROBOT_CONFIG.followingErrorDeg) {
                motor.enable(false);
                Serial.printf("STALL! lenh %.2f do, do %.2f do\n", joint.commanded(), joint.measured());
                return false;
            }
        }
        vTaskDelayUntil(&last, 1);
    }
    joint.hold();
    return true;
}

void loop() {
    if (stalled) {
        delay(1000);
        return;
    }
    for (float target : {60.0f, -60.0f}) {
        if (!moveTo(target)) {
            stalled = true;
            return;
        }
        delay(50);
        joint.updateEncoder();
        Serial.printf("Dich %.1f | lenh %.3f | do %.3f | sai so %.3f do\n", target, joint.commanded(),
                      joint.measured(), joint.followingError());
        delay(400);
    }
}
