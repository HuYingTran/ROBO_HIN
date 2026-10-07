#if defined(ARDUINO)
#include "Tca9548a.h"

#include <Arduino.h>

#include "../hal/I2cBus.h"

namespace hin {

void Tca9548a::attach(I2cBus& bus, uint8_t addr, int8_t rstPin) {
    bus_ = &bus;
    addr_ = addr;
    rstPin_ = rstPin;
}

Status Tca9548a::begin() {
    if (rstPin_ >= 0) {
        pinMode(rstPin_, OUTPUT);
        digitalWrite(rstPin_, HIGH);
    }
    reset();
    return bus_->probe(addr_) ? Status::Ok : Status::BusError;
}

Status Tca9548a::select(uint8_t ch) {
    if (ch > 7) return Status::InvalidArgument;
    if (current_ == ch) return Status::Ok;
    const uint8_t mask = static_cast<uint8_t>(1u << ch);
    const Status s = bus_->write(addr_, &mask, 1);
    current_ = (s == Status::Ok) ? ch : -1;
    return s;
}

Status Tca9548a::disableAll() {
    const uint8_t zero = 0;
    current_ = -1;
    return bus_->write(addr_, &zero, 1);
}

void Tca9548a::reset() {
    current_ = -1;
    if (rstPin_ < 0) return;
    digitalWrite(rstPin_, LOW);
    delayMicroseconds(2);
    digitalWrite(rstPin_, HIGH);
    delayMicroseconds(2);
}

}  // namespace hin
#endif
