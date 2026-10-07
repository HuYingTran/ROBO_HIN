// ROBO HIN — firmware chính: Serial G-code (README mục 10.6) + web điều khiển (mục 10).
//
// Khởi động ở trạng thái Disabled.
//   Web:    http://robo-hin.local  (hoặc AP "ROBO-HIN-XXXX", mật khẩu "hinrobot" → http://192.168.4.1)
//   Serial: M17 bật driver · G28 homing · G0/G1 di chuyển · M114 vị trí
//
// Nạp giao diện web một lần: pio run -t uploadfs
#include <Arduino.h>
#include <HinRobot.h>

#include "robot_config.h"

hin::HinRobot robot(ROBOT_CONFIG);
hin::SerialProtocol serialProto;
hin::WifiManager wifi;
hin::WebUi web;

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.printf("\nROBO HIN SDK %s\n", hin::kSdkVersion);

    const hin::Status s = robot.begin();
    if (s != hin::Status::Ok) {
        Serial.printf("begin() loi: %s\n", robot.lastErrorString());
        while (true) delay(1000);
    }

    serialProto.begin(robot, Serial);
    serialProto.startTask();  // core 0, prio 5

    wifi.begin();             // STA (NVS) hoặc AP dự phòng
    if (web.begin(robot, &wifi) != hin::Status::Ok) Serial.println("WebUi khong khoi dong duoc");

    Serial.printf("San sang. Web: http://%s.local hoac http://%s\n", wifi.hostname(), wifi.ip());
}

void loop() {
    // Mọi việc chạy trong task FreeRTOS của SDK
    vTaskDelay(pdMS_TO_TICKS(1000));
}
