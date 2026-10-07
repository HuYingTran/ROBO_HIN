#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "../common/Status.h"
#include "ProgramRunner.h"
#include "SerialProtocol.h"
#include "../storage/Programs.h"

class AsyncWebServer;
class AsyncWebSocket;
class AsyncWebServerRequest;

namespace hin {

class HinRobot;
class WifiManager;

// Giao diện web điều khiển (README mục 10):
//   - file tĩnh từ LittleFS /www (tự dùng bản .gz)
//   - WebSocket JSON /ws: lệnh từ trình duyệt, state 20 Hz, diag 2 Hz, ack, fault, log
//   - REST /api/*: info, login, programs, wifi, password; POST /update (OTA)
//   - an toàn: 1 client giữ quyền điều khiển, deadman heartbeat 300 ms khi jog,
//     mất kết nối → dừng jog + tạm dừng chương trình, E-stop xử lý ngay trong callback.
//
// Callback mạng chạy trong task async_tcp; mọi lệnh robot (có thể chặn) được chuyển sang
// webTask (core 0) qua hàng đợi.
class WebUi {
public:
    struct Options {
        uint16_t port = 80;
        uint32_t heartbeatTimeoutMs = 300;
        bool pauseOnDisconnect = true;
        uint8_t core = 0;
        uint8_t priority = 5;
        uint16_t stateIntervalMs = 50;   // 20 Hz
        uint16_t diagIntervalMs = 500;   // 2 Hz
    };

    // LittleFS phải mount được (tự format nếu phân vùng trống).
    Status begin(HinRobot& robot, WifiManager* wifi);
    Status begin(HinRobot& robot, WifiManager* wifi, const Options& opt);

    ProgramRunner& runner() { return runner_; }
    size_t clientCount() const;

    // Dùng nội bộ bởi callback
    void onWsConnect(uint32_t id);
    void onWsDisconnect(uint32_t id);
    void onWsText(uint32_t id, const char* data, size_t len);
    void broadcastLog(const char* msg);

private:
    static constexpr int kMaxClients = 8;
    static constexpr int kMaxPending = 16;
    struct Client {
        uint32_t id;
        bool authed;
    };
    struct Pending {
        uint32_t client, msgId, robotId;
    };

    static void taskFn(void* arg);
    void loop();
    void setupRoutes();
    void handleMessage(uint32_t client, const char* json, size_t len);
    void sendState();
    void sendDiag();
    void sendControl();
    void sendAck(uint32_t client, uint32_t msgId, const char* status, Status code, const char* msg);
    void queueAck(uint32_t client, uint32_t msgId, Status s);
    void checkPending();
    void sendTo(uint32_t client, const char* text);
    bool isAuthed(uint32_t client);
    bool authorized(AsyncWebServerRequest* req) const;
    void loadPassword();
    Status setPassword(const char* pw);
    void newToken();

    HinRobot* robot_ = nullptr;
    WifiManager* wifi_ = nullptr;
    Options opt_;
    AsyncWebServer* server_ = nullptr;
    AsyncWebSocket* ws_ = nullptr;
    void* queue_ = nullptr;  // QueueHandle_t
    void* lock_ = nullptr;   // SemaphoreHandle_t cho bảng client
    SerialProtocol console_;
    ProgramRunner runner_;
    Programs programs_;

    Client clients_[kMaxClients] = {};
    std::atomic<uint32_t> controller_{0};
    std::atomic<uint32_t> lastHb_{0};
    Pending pending_[kMaxPending] = {};
    int nPending_ = 0;
    uint8_t lastFault_ = 0;
    uint32_t restartAt_ = 0;
    bool authRequired_ = false;
    char token_[33] = "";
    char pwHash_[65] = "";
};

}  // namespace hin
