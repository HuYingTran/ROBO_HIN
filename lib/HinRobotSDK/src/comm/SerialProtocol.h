#pragma once
#include <cstddef>
#include <cstdint>

#include "../common/Status.h"

class Stream;
class Print;

namespace hin {

class HinRobot;
struct GcodeLine;

// Điều khiển qua Serial bằng G-code (115200 baud, kết thúc '\n').
// Mỗi dòng trả "ok" hoặc "error:<mã> <mô tả>". Lỗi bất đồng bộ in dạng "!fault:...".
class SerialProtocol {
public:
    static constexpr size_t kLineMax = 96;

    void begin(HinRobot& robot, Stream& io);
    // Tạo commTask trên core 0 (prio 5). Hoặc tự gọi poll() trong loop().
    void startTask(uint8_t core = 0, uint8_t priority = 5);
    void poll();
    // Xử lý 1 dòng, in kết quả ra `out` (dùng chung cho console web).
    void handleLine(const char* line, Print& out);

private:
    static void taskFn(void* arg);
    Status execute(const GcodeLine& g, Print& out);
    void reportError(Status s, Print& out);
    void printPosition(Print& out);
    void printSwitches(Print& out);

    HinRobot* robot_ = nullptr;
    Stream* io_ = nullptr;
    char buf_[kLineMax];
    size_t len_ = 0;
    bool overflow_ = false;
    float lastG0Pct_ = 50.0f;
    float lastG1MmS_ = 50.0f;
    uint8_t lastFault_ = 0;
};

}  // namespace hin
