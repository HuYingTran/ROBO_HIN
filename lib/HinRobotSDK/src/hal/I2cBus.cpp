#if defined(ARDUINO)
#include "I2cBus.h"

#include <Arduino.h>
#include <Wire.h>

namespace hin {

void I2cBus::configure(uint8_t port, int8_t sda, int8_t scl, uint32_t hz) {
    port_ = port;
    sda_ = sda;
    scl_ = scl;
    hz_ = hz;
    wire_ = (port == 0) ? &Wire : &Wire1;
}

Status I2cBus::begin() {
    if (!wire_) configure(port_, sda_, scl_, hz_);
    if (!wire_->begin(sda_, scl_, hz_)) return Status::BusError;
    wire_->setTimeOut(5);  // ms
    return Status::Ok;
}

Status I2cBus::write(uint8_t addr, const uint8_t* data, size_t len) {
    wire_->beginTransmission(addr);
    wire_->write(data, len);
    return wire_->endTransmission(true) == 0 ? Status::Ok : Status::BusError;
}

Status I2cBus::writeRead(uint8_t addr, const uint8_t* w, size_t wlen, uint8_t* r, size_t rlen) {
    wire_->beginTransmission(addr);
    wire_->write(w, wlen);
    if (wire_->endTransmission(false) != 0) return Status::BusError;
    const size_t got = wire_->requestFrom(addr, rlen, true);
    if (got != rlen) return Status::BusError;
    for (size_t i = 0; i < rlen; ++i) r[i] = static_cast<uint8_t>(wire_->read());
    return Status::Ok;
}

bool I2cBus::probe(uint8_t addr) {
    wire_->beginTransmission(addr);
    return wire_->endTransmission(true) == 0;
}

void I2cBus::recover() {
    wire_->end();
    pinMode(sda_, INPUT_PULLUP);
    pinMode(scl_, OUTPUT_OPEN_DRAIN);
    for (int i = 0; i < 9; ++i) {
        digitalWrite(scl_, LOW);
        delayMicroseconds(5);
        digitalWrite(scl_, HIGH);
        delayMicroseconds(5);
    }
    // STOP: SDA lên trong khi SCL cao
    pinMode(sda_, OUTPUT_OPEN_DRAIN);
    digitalWrite(sda_, LOW);
    delayMicroseconds(5);
    digitalWrite(sda_, HIGH);
    delayMicroseconds(5);
    begin();
}

}  // namespace hin
#endif
