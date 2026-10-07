#pragma once
#include <cstddef>
#include <cstdint>

#include "../common/Status.h"

class TwoWire;

namespace hin {

// Bọc TwoWire: đọc/ghi có mã lỗi, gỡ treo bus.
// Chỉ dùng từ MỘT task (encoderTask) — không tự khóa.
class I2cBus {
public:
    I2cBus() = default;
    I2cBus(uint8_t port, int8_t sda, int8_t scl, uint32_t hz) { configure(port, sda, scl, hz); }

    void configure(uint8_t port, int8_t sda, int8_t scl, uint32_t hz);
    Status begin();
    Status write(uint8_t addr, const uint8_t* data, size_t len);
    // Ghi (thường là địa chỉ thanh ghi) rồi đọc với repeated start.
    Status writeRead(uint8_t addr, const uint8_t* w, size_t wlen, uint8_t* r, size_t rlen);
    bool probe(uint8_t addr);
    // Phát 9 xung SCL để slave nhả SDA rồi khởi tạo lại bus.
    void recover();

private:
    TwoWire* wire_ = nullptr;
    uint8_t port_ = 0;
    int8_t sda_ = 21, scl_ = 22;
    uint32_t hz_ = 400000;
};

}  // namespace hin
