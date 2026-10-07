#pragma once
#include <cstdint>

#include "../common/Status.h"
#include "../storage/Programs.h"

namespace hin {

class HinRobot;

// Chạy chương trình teach: đẩy từng bước vào hàng đợi robot, chờ xong mới đẩy bước kế.
// Gọi tick() định kỳ (~10 ms) từ một task không thời gian thực (webTask).
class ProgramRunner {
public:
    using LogFn = void (*)(void* ctx, const char* msg);

    void begin(HinRobot& robot, LogFn log = nullptr, void* ctx = nullptr);

    // loops: số lần chạy, 0 = lặp vô hạn
    Status start(const Program& p, uint16_t loops);
    void pause();   // dừng có giảm tốc, nhớ bước đang chạy
    void resume();
    void stop();
    void tick();

    bool running() const { return running_; }
    bool paused() const { return paused_; }
    int currentStep() const { return current_; }
    int stepCount() const { return prog_.count; }
    uint16_t loopsDone() const { return loopsDone_; }
    const char* name() const { return prog_.name; }

private:
    Status issue(int i);
    void log(const char* fmt, ...);

    HinRobot* robot_ = nullptr;
    LogFn log_ = nullptr;
    void* ctx_ = nullptr;
    Program prog_;
    volatile bool running_ = false;
    volatile bool paused_ = false;
    int current_ = -1;
    int next_ = 0;
    uint16_t loops_ = 1;
    uint16_t loopsDone_ = 0;
};

}  // namespace hin
