// Example 06 — IK + chuyển động thẳng trong XY: vẽ hình vuông rồi pick & place.
//   pio run -e ex06 -t upload
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
    check(robot.home());
    robot.setElbow(hin::ElbowConfig::Right);
    check(robot.moveJPose({200, 0, 120, 0}, hin::Speed::percent(50)));
    check(robot.waitIdle());
}

void loop() {
    // Hình vuông 60 × 60 mm, đầu bút cách bàn Z = 60
    const float x0 = 170, y0 = -30, s = 60, z = 60;
    const hin::CartPose square[] = {
        {x0, y0, z, 0}, {x0 + s, y0, z, 0}, {x0 + s, y0 + s, z, 0}, {x0, y0 + s, z, 0}, {x0, y0, z, 0},
    };
    for (const auto& p : square) check(robot.moveL(p, hin::Speed::mmPerSec(40)));
    check(robot.waitIdle());
    Serial.println("Xong hinh vuong");

    // Pick & place (README mục 8.1) — tay kẹp đi qua hàng đợi nên kẹp đúng lúc tới nơi
    robot.gripper().open();
    check(robot.moveL({200, 50, 120, 0}, hin::Speed::mmPerSec(60)));   // tới trên vật
    check(robot.moveL({200, 50, 40, 0}, hin::Speed::mmPerSec(20)));    // hạ Z
    robot.gripper().close();
    check(robot.moveL({200, 50, 120, 0}, hin::Speed::mmPerSec(20)));   // nâng Z
    check(robot.moveL({50, 220, 120, 90}, hin::Speed::mmPerSec(60)));  // tới chỗ thả, xoay 90°
    check(robot.moveL({50, 220, 40, 90}, hin::Speed::mmPerSec(20)));
    robot.gripper().open();
    check(robot.moveL({50, 220, 120, 90}, hin::Speed::mmPerSec(20)));
    check(robot.waitIdle());

    const hin::CartPose p = robot.toolPose();
    Serial.printf("Xong pick & place. TCP: X%.1f Y%.1f Z%.1f A%.1f\n", p.x, p.y, p.z, p.phi);
    check(robot.moveL({200, 0, 120, 0}, hin::Speed::mmPerSec(60)));
    check(robot.waitIdle());
    delay(1000);
}
