#pragma once
#include <cstdint>

#include "../common/Status.h"

namespace hin {

class I2cBus;

// Mux I2C 8 kênh — cần vì mọi MT6701 cùng địa chỉ 0x06.
class Tca9548a {
public:
    Tca9548a() = default;
    Tca9548a(I2cBus& bus, uint8_t addr = 0x70, int8_t rstPin = -1) { attach(bus, addr, rstPin); }

    void attach(I2cBus& bus, uint8_t addr, int8_t rstPin);
    Status begin();
    Status select(uint8_t ch);  // bỏ qua nếu kênh đã chọn
    Status disableAll();
    // Reset cứng qua chân RST (nếu có) — dùng khi gỡ treo bus.
    void reset();
    I2cBus& bus() { return *bus_; }

private:
    I2cBus* bus_ = nullptr;
    uint8_t addr_ = 0x70;
    int8_t rstPin_ = -1;
    int16_t current_ = -1;
};

}  // namespace hin
