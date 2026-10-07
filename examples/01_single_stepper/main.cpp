// Example 01 — quay 1 động cơ (J2), không encoder.
// Kiểm chứng: động cơ quay êm, không trượt bước; xung STEP do ISR phần cứng phát.
//   pio run -e ex01 -t upload
#include <Arduino.h>
#include <HinRobot.h>

#include "robot_config.h"

hin::A4988 motor;
hin::TrapezoidProfile profile;

constexpr int kJoint = 1;  // J2
const hin::JointConfig& jc = JOINTS[kJoint];

// Bước / độ của khớp
const float stepsPerDeg = 200.0f * jc.microstep * jc.ratio / 360.0f;

void setup() {
    Serial.begin(115200);
    delay(200);
    hin::StepPulseGen::begin();
    motor.begin(jc.stepPin, jc.dirPin, ROBOT_CONFIG.enablePin, jc.microstep, jc.invert);
    motor.enable(true);
    Serial.printf("J2: %.1f buoc/do, vmax %.0f do/s = %.0f buoc/s\n", stepsPerDeg, jc.vmax,
                  jc.vmax * stepsPerDeg);
}

void loop() {
    static float target = 90.0f;
    const float start = motor.position() / stepsPerDeg;
    const float dist = target - start;
    profile.plan(fabsf(dist), jc.vmax * 0.5f, jc.amax);
    Serial.printf("Di tu %.1f toi %.1f do trong %.2f s\n", start, target, profile.duration());

    // Bám profile mỗi 1 ms: vận tốc = (điểm đặt kế tiếp − vị trí đã phát) / dt
    TickType_t last = xTaskGetTickCount();
    const float dir = dist >= 0 ? 1.0f : -1.0f;
    for (float t = 0; t <= profile.duration() + 0.01f; t += 0.001f) {
        const float sp = start + dir * profile.position(t + 0.001f);
        motor.setVelocity((sp * stepsPerDeg - motor.position()) / 0.001f);
        vTaskDelayUntil(&last, 1);
    }
    motor.setVelocity(0);
    Serial.printf("Vi tri: %ld buoc (%.2f do)\n", static_cast<long>(motor.position()),
                  motor.position() / stepsPerDeg);
    target = -target;
    delay(500);
}
