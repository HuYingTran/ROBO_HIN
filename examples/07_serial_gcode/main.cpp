// Example 07 — điều khiển từ terminal bằng G-code (README mục 10.6).
//   pio run -e ex07 -t upload && pio device monitor
//
// > M17
// > G28
// > G0 J1=100 J2=45 J3=-30 J4=0 F50
// > G1 X200 Y50 Z40 A0 F40
// > M114
#include <Arduino.h>
#include <HinRobot.h>

#include "robot_config.h"

hin::HinRobot robot(ROBOT_CONFIG);
hin::SerialProtocol gcode;

void setup() {
    Serial.begin(115200);
    delay(200);
    if (robot.begin() != hin::Status::Ok) {
        Serial.printf("begin() loi: %s\n", robot.lastErrorString());
        while (true) delay(1000);
    }
    gcode.begin(robot, Serial);
    Serial.println("Lenh: G0 G1 G4 G28 G92 M3 M5 M17 M18 M50 M112 M114 M119 M400 M410 M500 M999");
}

void loop() {
    gcode.poll();  // ví dụ này đọc Serial ngay trong loop() thay vì tạo commTask
    delay(2);
}
