// Example 05 — đồng bộ 4 trục: mọi khớp bắt đầu và kết thúc cùng lúc.
//   pio run -e ex05 -t upload
#include <Arduino.h>
#include <HinRobot.h>

#include "robot_config.h"

hin::HinRobot robot(ROBOT_CONFIG);

void check(hin::Status s) {
    if (s == hin::Status::Ok) return;
    Serial.printf("Loi: %s\n", robot.lastErrorString());
    while (true) delay(1000);
}

void printPose(const char* tag) {
    const hin::JointPose q = robot.jointPositions();
    Serial.printf("%s J: %.2f, %.2f, %.2f, %.2f\n", tag, q[0], q[1], q[2], q[3]);
}

void setup() {
    Serial.begin(115200);
    delay(200);
    check(robot.begin());
    check(robot.enable());
    check(robot.home());
}

void loop() {
    // { Z (mm), θ2 (°), θ3 (°), θ4 (°) }
    const hin::JointPose poses[] = {
        {100, 45, -30, 0},
        {140, -45, 60, 90},
        {60, 90, -90, -45},
    };
    for (const auto& p : poses) {
        const uint32_t t0 = millis();
        check(robot.moveJ(p, hin::Speed::percent(60)));
        check(robot.waitIdle());
        Serial.printf("moveJ %lu ms — ", static_cast<unsigned long>(millis() - t0));
        printPose("do");
        delay(300);
    }
}
