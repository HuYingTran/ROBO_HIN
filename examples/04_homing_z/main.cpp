// Example 04 — homing J1 bằng limit Z + đếm vòng vít me.
// Kiểm chứng: homing lặp lại ± 0.02 mm (so vị trí encoder khi về lại Z = 0).
//   pio run -e ex04 -t upload
#include <Arduino.h>
#include <HinRobot.h>

#include "robot_config.h"

hin::HinRobot robot(ROBOT_CONFIG);

void check(hin::Status s) {
    if (s == hin::Status::Ok) return;
    Serial.printf("Loi: %s\n", robot.lastErrorString());
    while (true) delay(1000);
}

void setup() {
    Serial.begin(115200);
    delay(200);
    check(robot.begin());
    check(robot.enable());

    for (int run = 1; run <= 5; ++run) {
        const uint32_t t0 = millis();
        check(robot.home());
        Serial.printf("Lan %d: homing xong sau %lu ms, Z do = %.3f mm\n", run,
                      static_cast<unsigned long>(millis() - t0), robot.jointPositions()[0]);

        // Lên 50 mm rồi về 0: encoder phải báo ≈ 0
        hin::JointPose q = robot.queuedTarget();
        q[0] = 50;
        check(robot.moveJ(q, hin::Speed::percent(80)));
        q[0] = 0;
        check(robot.moveJ(q, hin::Speed::percent(80)));
        check(robot.waitIdle());
        delay(100);
        Serial.printf("        ve Z = 0: do = %.3f mm\n", robot.jointPositions()[0]);
    }
    robot.disable();
}

void loop() { delay(1000); }
