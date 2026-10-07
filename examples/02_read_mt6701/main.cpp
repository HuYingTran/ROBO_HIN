// Example 02 — quét I2C, đọc góc 4 encoder MT6701 qua TCA9548A.
// Quay tay từng khớp: góc phải đổi mượt, không nhảy, đủ 0–360°.
//   pio run -e ex02 -t upload
#include <Arduino.h>
#include <HinRobot.h>

#include "robot_config.h"

const hin::EncoderBusConfig& eb = ROBOT_CONFIG.encoderBus;
hin::I2cBus bus(eb.port, eb.sda, eb.scl, eb.hz);
hin::Tca9548a mux(bus, eb.muxAddr, eb.muxResetPin);
hin::MT6701I2c enc[4];

void setup() {
    Serial.begin(115200);
    delay(200);
    bus.begin();

    Serial.println("Quet bus chinh:");
    for (uint8_t a = 1; a < 127; ++a)
        if (bus.probe(a)) Serial.printf("  thay 0x%02X\n", a);

    if (mux.begin() != hin::Status::Ok) {
        Serial.printf("Khong thay TCA9548A tai 0x%02X — kiem tra SDA/SCL, VDD 3V3, pull-up\n",
                      eb.muxAddr);
        while (true) delay(1000);
    }
    for (int i = 0; i < 4; ++i) {
        const uint8_t ch = JOINTS[i].encChannel;
        mux.select(ch);
        Serial.printf("CH%d (J%d): MT6701 0x06 %s\n", ch, i + 1,
                      bus.probe(hin::mt6701::kI2cAddr) ? "OK" : "KHONG THAY");
        enc[i].attach(mux, ch);
    }
}

void loop() {
    for (int i = 0; i < 4; ++i) {
        uint16_t raw = 0;
        const uint32_t t0 = micros();
        const hin::Status s = enc[i].readRaw(raw);
        const uint32_t dt = micros() - t0;
        if (s == hin::Status::Ok)
            Serial.printf("J%d %7.2f deg (%5u) %3luus | ", i + 1, hin::mt6701::ticksToDeg(raw), raw,
                          static_cast<unsigned long>(dt));
        else
            Serial.printf("J%d  %-14s | ", i + 1, hin::toString(s));
    }
    Serial.println();
    delay(100);
}
