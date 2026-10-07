// Example 08 — WiFi + web điều khiển đầy đủ (README mục 10).
// Kiểm chứng: jog + teach & play từ điện thoại; tắt WiFi khi đang jog → robot dừng ≤ 300 ms.
//
//   pio run -e ex08 -t upload
//   pio run -e ex08 -t uploadfs      (nạp giao diện web vào LittleFS)
//
// Lần đầu robot phát AP "ROBO-HIN-XXXX" (mật khẩu "hinrobot") → mở http://192.168.4.1,
// vào Cài đặt nhập WiFi nhà → robot khởi động lại và vào http://robo-hin.local
#include <Arduino.h>
#include <HinRobot.h>

#include "robot_config.h"

hin::HinRobot robot(ROBOT_CONFIG);
hin::WifiManager wifi;
hin::WebUi web;

void setup() {
    Serial.begin(115200);
    delay(200);
    if (robot.begin() != hin::Status::Ok) {
        Serial.printf("begin() loi: %s\n", robot.lastErrorString());
        while (true) delay(1000);
    }

    hin::WifiManager::Options wo;
    wo.hostname = "robo-hin";
    wo.apPassword = "hinrobot";
    wifi.begin(wo);

    hin::WebUi::Options opt;
    opt.heartbeatTimeoutMs = 300;   // deadman khi jog
    opt.pauseOnDisconnect = true;   // mất kết nối → tạm dừng chương trình
    web.begin(robot, &wifi, opt);

    Serial.printf("Mo trinh duyet: http://%s.local  (IP %s%s)\n", wifi.hostname(), wifi.ip(),
                  wifi.isAp() ? ", AP" : "");
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
