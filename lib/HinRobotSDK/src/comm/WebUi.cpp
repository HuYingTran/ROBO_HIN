#if defined(ARDUINO)
#include "WebUi.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <Update.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <mbedtls/sha256.h>

#include <cstdio>
#include <cstring>

#include "../HinRobot.h"
#include "../common/Log.h"
#include "WifiManager.h"

namespace hin {

namespace {

constexpr size_t kMsgMax = 240;
constexpr const char* kPwNs = "hinweb";

struct WsMsg {
    uint32_t client;  // 0 = sự kiện nội bộ
    uint16_t len;
    char text[kMsgMax];
};

// Gom output của console G-code để gửi lại trình duyệt
class BufPrint : public Print {
public:
    size_t write(uint8_t c) override {
        if (n_ < sizeof(buf_) - 1) buf_[n_++] = static_cast<char>(c);
        return 1;
    }
    const char* str() {
        buf_[n_] = '\0';
        return buf_;
    }

private:
    char buf_[384];
    size_t n_ = 0;
};

// Trang tạm khi chưa nạp LittleFS
const char kFallbackHtml[] PROGMEM = R"HTML(<!doctype html><html lang="vi"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>ROBO HIN</title>
<style>body{font-family:system-ui;background:#111;color:#eee;max-width:640px;margin:40px auto;padding:0 16px}
code{background:#222;padding:2px 6px;border-radius:4px}</style></head><body>
<h1>ROBO HIN</h1><p>Firmware đang chạy nhưng chưa có giao diện web trong LittleFS.</p>
<p>Nạp bằng: <code>pio run -t uploadfs</code></p>
<p>Hoặc nạp file <code>littlefs.bin</code> qua <code>POST /update?target=fs</code>.</p></body></html>)HTML";

Program s_loadBuf;  // bộ đệm nạp chương trình (2 KB, tránh dùng stack)

QueueHandle_t q(void* h) { return static_cast<QueueHandle_t>(h); }

void sha256Hex(const char* s, char out[65]) {
    uint8_t h[32];
    mbedtls_sha256_ret(reinterpret_cast<const unsigned char*>(s), strlen(s), h, 0);
    for (int i = 0; i < 32; ++i) sprintf(out + 2 * i, "%02x", h[i]);
    out[64] = '\0';
}

void collectBody(AsyncWebServerRequest* r, uint8_t* data, size_t len, size_t index, size_t total) {
    if (total > Programs::kMaxFileSize) return;
    if (index == 0 && !r->_tempObject) r->_tempObject = calloc(total + 1, 1);
    if (r->_tempObject) memcpy(static_cast<uint8_t*>(r->_tempObject) + index, data, len);
}

const char* bodyOf(AsyncWebServerRequest* r) {
    return r->_tempObject ? static_cast<const char*>(r->_tempObject) : "";
}

void sendJson(AsyncWebServerRequest* r, int code, const char* json) {
    r->send(code, "application/json", json);
}

void sendErr(AsyncWebServerRequest* r, int code, const char* msg) {
    char buf[160];
    snprintf(buf, sizeof(buf), "{\"error\":\"%s\"}", msg);
    sendJson(r, code, buf);
}

int axisIndex(const char* a, bool& cart) {
    cart = false;
    if (!a) return -1;
    if ((a[0] == 'J' || a[0] == 'j') && a[1] >= '1' && a[1] <= '4' && !a[2]) return a[1] - '1';
    cart = true;
    switch (a[0]) {
        case 'X': case 'x': return 0;
        case 'Y': case 'y': return 1;
        case 'Z': case 'z': return 2;
        case 'A': case 'a': return 3;
    }
    return -1;
}

const char* elbowStr(ElbowConfig e) {
    return e == ElbowConfig::Left ? "L" : e == ElbowConfig::Right ? "R" : "N";
}

}  // namespace

// ---------------------------------------------------------------------------
// Khởi tạo
// ---------------------------------------------------------------------------

Status WebUi::begin(HinRobot& robot, WifiManager* wifi) { return begin(robot, wifi, Options{}); }

Status WebUi::begin(HinRobot& robot, WifiManager* wifi, const Options& opt) {
    robot_ = &robot;
    wifi_ = wifi;
    opt_ = opt;

    if (!LittleFS.begin(true)) {
        HIN_LOGE("khong mount duoc LittleFS");
        return Status::StorageError;
    }
    loadPassword();

    queue_ = xQueueCreate(16, sizeof(WsMsg));
    lock_ = xSemaphoreCreateMutex();
    if (!queue_ || !lock_) return Status::Error;

    console_.begin(robot, Serial);  // chỉ dùng handleLine()
    runner_.begin(robot, [](void* ctx, const char* m) { static_cast<WebUi*>(ctx)->broadcastLog(m); }, this);

    server_ = new AsyncWebServer(opt_.port);
    ws_ = new AsyncWebSocket("/ws");
    ws_->onEvent([this](AsyncWebSocket*, AsyncWebSocketClient* c, AwsEventType type, void* arg,
                        uint8_t* data, size_t len) {
        switch (type) {
            case WS_EVT_CONNECT: onWsConnect(c->id()); break;
            case WS_EVT_DISCONNECT: onWsDisconnect(c->id()); break;
            case WS_EVT_DATA: {
                auto* info = static_cast<AwsFrameInfo*>(arg);
                if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
                    onWsText(c->id(), reinterpret_cast<const char*>(data), len);
                break;
            }
            default: break;
        }
    });
    server_->addHandler(ws_);
    setupRoutes();
    server_->begin();

    xTaskCreatePinnedToCore(taskFn, "hinWeb", 8192, this, opt_.priority, nullptr, opt_.core);
    HIN_LOGI("WebUi san sang (cong %u, xac thuc %s)", opt_.port, authRequired_ ? "bat" : "tat");
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Xác thực
// ---------------------------------------------------------------------------

void WebUi::newToken() {
    for (int i = 0; i < 4; ++i) sprintf(token_ + 8 * i, "%08lx", static_cast<unsigned long>(esp_random()));
}

void WebUi::loadPassword() {
    Preferences p;
    pwHash_[0] = '\0';
    if (p.begin(kPwNs, true)) {
        p.getString("pw", pwHash_, sizeof(pwHash_));
        p.end();
    }
    authRequired_ = pwHash_[0] != '\0';
    newToken();
}

Status WebUi::setPassword(const char* pw) {
    Preferences p;
    if (!p.begin(kPwNs, false)) return Status::StorageError;
    if (pw && pw[0]) {
        sha256Hex(pw, pwHash_);
        p.putString("pw", pwHash_);
    } else {
        pwHash_[0] = '\0';
        p.remove("pw");
    }
    p.end();
    authRequired_ = pwHash_[0] != '\0';
    newToken();  // đăng xuất mọi phiên cũ
    return Status::Ok;
}

bool WebUi::authorized(AsyncWebServerRequest* r) const {
    if (!authRequired_) return true;
    const AsyncWebHeader* h = r->getHeader("X-Token");
    if (h && h->value() == token_) return true;
    if (r->hasParam("token")) return r->getParam("token")->value() == token_;
    return false;
}

bool WebUi::isAuthed(uint32_t client) {
    bool ok = false;
    xSemaphoreTake(static_cast<SemaphoreHandle_t>(lock_), portMAX_DELAY);
    for (auto& c : clients_)
        if (c.id == client) ok = c.authed;
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(lock_));
    return ok;
}

size_t WebUi::clientCount() const { return ws_ ? ws_->count() : 0; }

// ---------------------------------------------------------------------------
// WebSocket callback (task async_tcp — không được chặn)
// ---------------------------------------------------------------------------

void WebUi::onWsConnect(uint32_t id) {
    xSemaphoreTake(static_cast<SemaphoreHandle_t>(lock_), portMAX_DELAY);
    for (auto& c : clients_) {
        if (c.id == 0) {
            c.id = id;
            c.authed = !authRequired_;
            break;
        }
    }
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(lock_));
}

void WebUi::onWsDisconnect(uint32_t id) {
    xSemaphoreTake(static_cast<SemaphoreHandle_t>(lock_), portMAX_DELAY);
    for (auto& c : clients_)
        if (c.id == id) c = Client{};
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(lock_));

    if (controller_ == id) {
        controller_ = 0;
        // Báo webTask xử lý: dừng jog, tạm dừng chương trình
        WsMsg m{};
        m.client = 0;
        m.len = static_cast<uint16_t>(snprintf(m.text, sizeof(m.text), "{\"cmd\":\"_lost\",\"id\":%lu}",
                                               static_cast<unsigned long>(id)));
        xQueueSend(q(queue_), &m, 0);
    }
}

void WebUi::onWsText(uint32_t id, const char* data, size_t len) {
    if (len >= kMsgMax) return;
    // Đường tắt: heartbeat và E-stop không qua hàng đợi
    if (len < 32 && memmem(data, len, "\"hb\"", 4)) {
        if (controller_ == id) lastHb_ = millis();
        return;
    }
    if (memmem(data, len, "\"estop\"", 7) && isAuthed(id)) robot_->emergencyStop();

    WsMsg m;
    m.client = id;
    m.len = static_cast<uint16_t>(len);
    memcpy(m.text, data, len);
    m.text[len] = '\0';
    if (xQueueSend(q(queue_), &m, 0) != pdTRUE) HIN_LOGW("hang doi web day — bo lenh");
}

// ---------------------------------------------------------------------------
// Gửi
// ---------------------------------------------------------------------------

void WebUi::sendTo(uint32_t client, const char* text) {
    if (client) ws_->text(client, text);
    else ws_->textAll(text);
}

void WebUi::broadcastLog(const char* msg) {
    JsonDocument d;
    d["type"] = "log";
    d["msg"] = msg;
    char buf[200];
    serializeJson(d, buf, sizeof(buf));
    ws_->textAll(buf);
}

void WebUi::sendControl() {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"type\":\"control\",\"owner\":%lu}",
             static_cast<unsigned long>(controller_.load()));
    ws_->textAll(buf);
}

void WebUi::sendAck(uint32_t client, uint32_t msgId, const char* status, Status code, const char* msg) {
    JsonDocument d;
    d["type"] = "ack";
    d["id"] = msgId;
    d["status"] = status;
    d["code"] = static_cast<int>(code);
    if (msg && msg[0]) d["msg"] = msg;
    char buf[200];
    serializeJson(d, buf, sizeof(buf));
    ws_->text(client, buf);
}

// Lệnh xếp hàng xong → ack "queued" ngay, "done"/"error" khi planner chạy xong
void WebUi::queueAck(uint32_t client, uint32_t msgId, Status s) {
    if (s != Status::Ok) {
        sendAck(client, msgId, "error", s, robot_->lastErrorString());
        return;
    }
    sendAck(client, msgId, "queued", s, nullptr);
    if (msgId == 0) return;
    if (nPending_ == kMaxPending) {  // bỏ cái cũ nhất
        memmove(pending_, pending_ + 1, sizeof(Pending) * (kMaxPending - 1));
        --nPending_;
    }
    pending_[nPending_++] = Pending{client, msgId, robot_->lastQueuedId()};
}

void WebUi::checkPending() {
    if (nPending_ == 0) return;
    const uint32_t done = robot_->lastDoneId();
    const uint32_t errId = robot_->lastPlanErrorId();
    const bool idle = robot_->isIdle();
    int w = 0;
    for (int i = 0; i < nPending_; ++i) {
        const Pending& p = pending_[i];
        if (p.robotId == errId) {
            sendAck(p.client, p.msgId, "error", robot_->lastPlanError(), "planner huy lenh");
        } else if (done >= p.robotId) {
            sendAck(p.client, p.msgId, "done", Status::Ok, nullptr);
        } else if (idle) {
            sendAck(p.client, p.msgId, "error", Status::Faulted, "lenh bi huy (stop / loi)");
        } else {
            pending_[w++] = p;
        }
    }
    nPending_ = w;
}

void WebUi::sendState() {
    HinRobot& r = *robot_;
    const JointPose q = r.jointPositions();
    const CartPose p = r.toolPose();
    char buf[420];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"state\",\"state\":\"%s\",\"homed\":%d,\"joints\":[%.2f,%.2f,%.2f,%.2f],"
             "\"pose\":[%.2f,%.2f,%.2f,%.2f],\"elbow\":\"%s\",\"elbowPref\":\"%s\",\"speed\":%.0f,"
             "\"gripper\":%.0f,\"jog\":%d,\"owner\":%lu,"
             "\"prog\":{\"name\":\"%s\",\"step\":%d,\"count\":%d,\"running\":%d,\"paused\":%d,\"loops\":%u}}",
             toString(r.state()), r.isHomed(), q[0], q[1], q[2], q[3], p.x, p.y, p.z, p.phi,
             elbowStr(r.currentElbow()), elbowStr(r.elbow()), r.speedOverride(),
             r.gripperServo().position(), r.isJogging(), static_cast<unsigned long>(controller_.load()),
             runner_.name(), runner_.currentStep(), runner_.stepCount(), runner_.running(),
             runner_.paused(), runner_.loopsDone());
    ws_->textAll(buf);
}

void WebUi::sendDiag() {
    HinRobot& r = *robot_;
    SafetyMonitor& sm = r.safety();
    JsonDocument d;
    d["type"] = "diag";
    JsonArray enc = d["enc"].to<JsonArray>();
    JsonArray ferr = d["ferr"].to<JsonArray>();
    JsonArray errs = d["encErr"].to<JsonArray>();
    for (int i = 0; i < r.jointCount(); ++i) {
        Joint& j = r.joint(i);
        enc.add(j.encoder() ? toString(j.encoder()->fieldStatus()) : "-");
        ferr.add(serialized(String(j.followingError(), 3)));
        errs.add(j.encoderErrorStreak());
    }
    d["zmin"] = sm.zMinActive();
    d["zmax"] = sm.zMaxActive();
    d["estop"] = sm.estopActive();
    d["fault"] = toString(r.lastFault());
    d["err"] = r.lastErrorString();
    d["heap"] = ESP.getFreeHeap();
    d["clients"] = ws_->count();
    if (wifi_) {
        d["rssi"] = wifi_->rssi();
        d["ip"] = wifi_->ip();
        d["ap"] = wifi_->isAp();
        d["ssid"] = wifi_->ssid();
    }
    char buf[512];
    serializeJson(d, buf, sizeof(buf));
    ws_->textAll(buf);
}

// ---------------------------------------------------------------------------
// webTask
// ---------------------------------------------------------------------------

void WebUi::taskFn(void* arg) { static_cast<WebUi*>(arg)->loop(); }

void WebUi::loop() {
    uint32_t lastState = 0, lastDiag = 0, lastClean = 0;
    WsMsg m;
    for (;;) {
        // Chờ lệnh tối đa 5 ms rồi làm việc định kỳ
        if (xQueueReceive(q(queue_), &m, pdMS_TO_TICKS(5)) == pdTRUE) {
            handleMessage(m.client, m.text, m.len);
            while (xQueueReceive(q(queue_), &m, 0) == pdTRUE) handleMessage(m.client, m.text, m.len);
        }
        const uint32_t now = millis();

        // Deadman: đang jog mà mất heartbeat → dừng có giảm tốc
        if (robot_->isJogging() && now - lastHb_ > opt_.heartbeatTimeoutMs) {
            robot_->stop();
            lastHb_ = now;
            broadcastLog("jog dung do mat heartbeat");
        }

        runner_.tick();
        checkPending();

        const uint8_t f = static_cast<uint8_t>(robot_->lastFault());
        if (f != lastFault_) {
            lastFault_ = f;
            if (f) {
                JsonDocument d;
                d["type"] = "fault";
                d["fault"] = toString(robot_->lastFault());
                d["msg"] = robot_->lastErrorString();
                char buf[200];
                serializeJson(d, buf, sizeof(buf));
                ws_->textAll(buf);
            }
        }

        if (ws_->count() > 0) {
            if (now - lastState >= opt_.stateIntervalMs && ws_->availableForWriteAll()) {
                lastState = now;
                sendState();
            }
            if (now - lastDiag >= opt_.diagIntervalMs) {
                lastDiag = now;
                sendDiag();
            }
        }
        if (now - lastClean > 1000) {
            lastClean = now;
            ws_->cleanupClients();
        }
        if (restartAt_ && static_cast<int32_t>(now - restartAt_) >= 0) ESP.restart();
    }
}

void WebUi::handleMessage(uint32_t client, const char* json, size_t len) {
    JsonDocument d;
    if (deserializeJson(d, json, len)) return;
    const char* cmd = d["cmd"] | "";
    const uint32_t msgId = d["id"] | 0u;
    HinRobot& r = *robot_;

    // ---- Sự kiện nội bộ ----
    if (client == 0) {
        if (!strcmp(cmd, "_lost")) {
            if (r.isJogging()) r.stop();
            if (opt_.pauseOnDisconnect) runner_.pause();
            broadcastLog("client dieu khien mat ket noi");
            sendControl();
        }
        return;
    }

    // ---- Xác thực ----
    if (!strcmp(cmd, "hello")) {
        const char* tok = d["token"] | "";
        const bool ok = !authRequired_ || strcmp(tok, token_) == 0;
        xSemaphoreTake(static_cast<SemaphoreHandle_t>(lock_), portMAX_DELAY);
        for (auto& c : clients_)
            if (c.id == client) c.authed = ok;
        xSemaphoreGive(static_cast<SemaphoreHandle_t>(lock_));
        char buf[96];
        snprintf(buf, sizeof(buf), "{\"type\":\"hello\",\"ok\":%s,\"you\":%lu,\"auth\":%s}",
                 ok ? "true" : "false", static_cast<unsigned long>(client), authRequired_ ? "true" : "false");
        sendTo(client, buf);
        if (ok) sendControl();
        return;
    }
    if (!isAuthed(client)) {
        sendTo(client, "{\"type\":\"hello\",\"ok\":false}");
        return;
    }

    // ---- Lệnh ai cũng gửi được ----
    if (!strcmp(cmd, "estop")) {
        r.emergencyStop();  // đã gọi ở callback, gọi lại vô hại
        runner_.stop();
        broadcastLog("DUNG KHAN tu web");
        return;
    }
    if (!strcmp(cmd, "takeControl")) {
        const uint32_t prev = controller_.exchange(client);
        if (prev && prev != client && r.isJogging()) r.stop();
        lastHb_ = millis();
        sendControl();
        return;
    }
    if (!strcmp(cmd, "releaseControl")) {
        uint32_t expected = client;
        controller_.compare_exchange_strong(expected, 0);
        sendControl();
        return;
    }

    // ---- Còn lại cần quyền điều khiển ----
    if (controller_ != client) {
        sendAck(client, msgId, "error", Status::NotReady, "chua giu quyen dieu khien");
        return;
    }

    Status s = Status::Ok;
    if (!strcmp(cmd, "jog")) {
        bool cart;
        const int ax = axisIndex(d["axis"] | "", cart);
        const float dir = (d["dir"] | 1) < 0 ? -1.0f : 1.0f;
        const float k = clamp(d["speed"] | 30.0f, 1.0f, 100.0f) / 100.0f;
        if (ax < 0) return;
        lastHb_ = millis();
        if (cart) {
            const float vmax = ax == 3 ? r.config().joints[3].vmax : r.config().maxCartSpeedMmS;
            s = r.jogCartesian(ax, dir * k * vmax);
        } else {
            s = r.jog(static_cast<JointId>(ax), dir * k * r.config().joints[ax].vmax);
        }
        if (s != Status::Ok) sendAck(client, msgId, "error", s, r.lastErrorString());
    } else if (!strcmp(cmd, "jogStop")) {
        bool cart;
        const int ax = axisIndex(d["axis"] | "", cart);
        if (ax < 0) r.stop();
        else if (cart) r.jogCartesian(ax, 0);
        else r.jog(static_cast<JointId>(ax), 0);
    } else if (!strcmp(cmd, "step")) {
        bool cart;
        const int ax = axisIndex(d["axis"] | "", cart);
        const float delta = d["delta"] | 0.0f;
        if (ax < 0 || delta == 0) return;
        if (cart) {
            CartPose p;
            r.fk(r.queuedTarget(), p);
            float* v[4] = {&p.x, &p.y, &p.z, &p.phi};
            *v[ax] += delta;
            s = r.moveL(p, Speed::mmPerSec(d["speed"] | 20.0f));
        } else {
            JointPose q = r.queuedTarget();
            q[ax] += delta;
            s = r.moveJ(q, Speed::percent(d["speed"] | 30.0f));
        }
        queueAck(client, msgId, s);
    } else if (!strcmp(cmd, "moveJ")) {
        JsonArrayConst a = d["joints"].as<JsonArrayConst>();
        if (a.size() != kMaxJoints) return sendAck(client, msgId, "error", Status::InvalidArgument, "joints[4]");
        const JointPose q{{a[0], a[1], a[2], a[3]}};
        queueAck(client, msgId, r.moveJ(q, Speed::percent(d["speed"] | 50.0f)));
    } else if (!strcmp(cmd, "moveL") || !strcmp(cmd, "moveP")) {
        JsonArrayConst a = d["pose"].as<JsonArrayConst>();
        if (a.size() != kMaxJoints) return sendAck(client, msgId, "error", Status::InvalidArgument, "pose[4]");
        const CartPose p{a[0], a[1], a[2], a[3]};
        if (cmd[4] == 'L')
            s = r.moveL(p, Speed::mmPerSec(d["speed"] | 40.0f));
        else  // moveP: PTP tới tư thế Descartes
            s = r.moveJPose(p, Speed::percent(d["speed"] | 50.0f));
        queueAck(client, msgId, s);
    } else if (!strcmp(cmd, "gripper")) {
        r.gripper().setPosition(d["pct"] | 0.0f);
    } else if (!strcmp(cmd, "speed")) {
        r.setSpeedOverride(clamp(d["pct"] | 100.0f, 1.0f, 100.0f));
    } else if (!strcmp(cmd, "elbow")) {
        const char* e = d["value"] | "N";
        r.setElbow(e[0] == 'L' ? ElbowConfig::Left : e[0] == 'R' ? ElbowConfig::Right : ElbowConfig::Nearest);
    } else if (!strcmp(cmd, "home")) {
        s = r.startHome();
        if (s != Status::Ok) sendAck(client, msgId, "error", s, r.lastErrorString());
    } else if (!strcmp(cmd, "enable")) {
        s = r.enable();
        if (s != Status::Ok) sendAck(client, msgId, "error", s, r.lastErrorString());
    } else if (!strcmp(cmd, "disable")) {
        runner_.stop();
        r.disable();
    } else if (!strcmp(cmd, "stop")) {
        runner_.stop();
        r.stop();
    } else if (!strcmp(cmd, "clearFault")) {
        s = r.clearFault();
        if (s != Status::Ok) sendAck(client, msgId, "error", s, r.lastErrorString());
    } else if (!strcmp(cmd, "gcode")) {
        BufPrint out;
        console_.handleLine(d["line"] | "", out);
        JsonDocument resp;
        resp["type"] = "log";
        resp["msg"] = out.str();
        resp["console"] = true;
        char buf[480];
        serializeJson(resp, buf, sizeof(buf));
        sendTo(client, buf);
    } else if (!strcmp(cmd, "prog")) {
        const char* op = d["op"] | "";
        if (!strcmp(op, "run")) {
            const char* name = d["name"] | "";
            s = programs_.load(name, s_loadBuf);
            if (s == Status::Ok) {
                if (!s_loadBuf.name[0]) strncpy(s_loadBuf.name, name, Program::kNameMax - 1);
                s = runner_.start(s_loadBuf, d["loop"] | 1);
            }
            if (s != Status::Ok) sendAck(client, msgId, "error", s, "khong nap duoc chuong trinh");
        } else if (!strcmp(op, "pause")) {
            runner_.pause();
        } else if (!strcmp(op, "resume")) {
            runner_.resume();
        } else if (!strcmp(op, "stop")) {
            runner_.stop();
        }
    }
}

// ---------------------------------------------------------------------------
// HTTP
// ---------------------------------------------------------------------------

void WebUi::setupRoutes() {
    // Thông tin robot để web tự vẽ đúng tỉ lệ (không cần đăng nhập)
    server_->on("/api/info", HTTP_GET, [this](AsyncWebServerRequest* req) {
        const RobotConfig& c = robot_->config();
        JsonDocument d;
        d["name"] = "ROBO HIN";
        d["fw"] = kSdkVersion;
        d["auth"] = authRequired_;
        JsonObject g = d["geometry"].to<JsonObject>();
        g["a2"] = c.geometry.a2;
        g["a3"] = c.geometry.a3;
        g["d4"] = c.geometry.d4;
        g["z0"] = c.geometry.z0;
        JsonArray js = d["joints"].to<JsonArray>();
        for (int i = 0; i < c.jointCount; ++i) {
            JsonObject j = js.add<JsonObject>();
            j["name"] = c.joints[i].name;
            j["type"] = c.joints[i].type == JointType::Prismatic ? "T" : "R";
            j["min"] = c.joints[i].minPos;
            j["max"] = c.joints[i].maxPos;
            j["vmax"] = c.joints[i].vmax;
        }
        d["maxCart"] = c.maxCartSpeedMmS;
        d["hbMs"] = opt_.heartbeatTimeoutMs;
        if (wifi_) {
            d["ap"] = wifi_->isAp();
            d["ssid"] = wifi_->ssid();
            d["ip"] = wifi_->ip();
            d["host"] = wifi_->hostname();
        }
        String out;
        serializeJson(d, out);
        req->send(200, "application/json", out);
    });

    server_->on(
        "/api/login", HTTP_POST,
        [this](AsyncWebServerRequest* req) {
            JsonDocument d;
            deserializeJson(d, bodyOf(req));
            const char* pw = d["password"] | "";
            char h[65];
            sha256Hex(pw, h);
            if (authRequired_ && strcmp(h, pwHash_) != 0) {
                delay(300);  // làm chậm dò mật khẩu
                return sendErr(req, 401, "sai mat khau");
            }
            char buf[64];
            snprintf(buf, sizeof(buf), "{\"token\":\"%s\"}", token_);
            sendJson(req, 200, buf);
        },
        nullptr, collectBody);

    server_->on(
        "/api/password", HTTP_POST,
        [this](AsyncWebServerRequest* req) {
            if (!authorized(req)) return sendErr(req, 401, "can dang nhap");
            JsonDocument d;
            deserializeJson(d, bodyOf(req));
            const char* pw = d["password"] | "";
            if (pw[0] && strlen(pw) < 4) return sendErr(req, 400, "mat khau toi thieu 4 ky tu");
            if (setPassword(pw) != Status::Ok) return sendErr(req, 500, "ghi NVS that bai");
            char buf[64];
            snprintf(buf, sizeof(buf), "{\"token\":\"%s\"}", token_);
            sendJson(req, 200, buf);
        },
        nullptr, collectBody);

    server_->on(
        "/api/wifi", HTTP_POST,
        [this](AsyncWebServerRequest* req) {
            if (!authorized(req)) return sendErr(req, 401, "can dang nhap");
            if (!wifi_) return sendErr(req, 501, "khong co WifiManager");
            JsonDocument d;
            deserializeJson(d, bodyOf(req));
            const Status s = wifi_->saveCredentials(d["ssid"] | "", d["pass"] | "");
            if (s != Status::Ok) return sendErr(req, 400, "SSID / mat khau khong hop le");
            sendJson(req, 200, "{\"ok\":true,\"restart\":true}");
            restartAt_ = millis() + 800;
        },
        nullptr, collectBody);

    // Chương trình teach
    server_->on("/api/programs", HTTP_GET, [this](AsyncWebServerRequest* req) {
        if (!authorized(req)) return sendErr(req, 401, "can dang nhap");
        String out;
        programs_.listJson(out);
        req->send(200, "application/json", out);
    });

    server_->on(
        "/api/programs/*", HTTP_GET | HTTP_PUT | HTTP_DELETE,
        [this](AsyncWebServerRequest* req) {
            if (!authorized(req)) return sendErr(req, 401, "can dang nhap");
            const String name = req->url().substring(strlen("/api/programs/"));
            if (!Programs::validName(name.c_str())) return sendErr(req, 400, "ten khong hop le");
            if (req->method() == HTTP_GET) {
                String body;
                if (programs_.read(name.c_str(), body) != Status::Ok) return sendErr(req, 404, "khong co");
                req->send(200, "application/json", body);
            } else if (req->method() == HTTP_DELETE) {
                if (programs_.remove(name.c_str()) != Status::Ok) return sendErr(req, 404, "khong co");
                sendJson(req, 200, "{\"ok\":true}");
            } else {
                const char* body = bodyOf(req);
                char err[64] = "";
                if (Programs::parse(body, strlen(body), s_loadBuf, err, sizeof(err)) != Status::Ok)
                    return sendErr(req, 400, err);
                if (programs_.write(name.c_str(), body, strlen(body)) != Status::Ok)
                    return sendErr(req, 500, "ghi LittleFS that bai");
                sendJson(req, 200, "{\"ok\":true}");
            }
        },
        nullptr, collectBody);

    // OTA: POST /update (multipart, field "firmware"); ?target=fs để nạp littlefs.bin
    server_->on(
        "/update", HTTP_POST,
        [this](AsyncWebServerRequest* req) {
            if (!authorized(req)) return sendErr(req, 401, "can dang nhap");
            if (Update.hasError() || !Update.isFinished()) {
                char buf[96];
                snprintf(buf, sizeof(buf), "cap nhat that bai: %s", Update.errorString());
                return sendErr(req, 500, buf);
            }
            sendJson(req, 200, "{\"ok\":true,\"restart\":true}");
            restartAt_ = millis() + 1000;
        },
        [this](AsyncWebServerRequest* req, const String& filename, size_t index, uint8_t* data, size_t len,
               bool final) {
            if (index == 0) {
                if (!authorized(req)) return;
                if (robot_->state() != RobotState::Disabled) {
                    HIN_LOGW("OTA bi tu choi: robot phai o trang thai Disabled");
                    return;
                }
                const bool fs = req->hasParam("target") && req->getParam("target")->value() == "fs";
                HIN_LOGI("OTA %s: %s", fs ? "LittleFS" : "firmware", filename.c_str());
                if (fs) LittleFS.end();
                Update.begin(UPDATE_SIZE_UNKNOWN, fs ? U_SPIFFS : U_FLASH);
            }
            if (Update.isRunning()) {
                Update.write(data, len);
                if (final) Update.end(true);
            }
        });

    // File tĩnh (tự ưu tiên .gz)
    server_->serveStatic("/", LittleFS, "/www/").setDefaultFile("index.html").setCacheControl("no-cache");

    server_->onNotFound([](AsyncWebServerRequest* req) {
        if (req->url().startsWith("/api/")) return sendErr(req, 404, "khong co API nay");
        if (req->url() == "/" || req->url() == "/index.html") {
            req->send(200, "text/html", kFallbackHtml);
            return;
        }
        req->send(404, "text/plain", "404");
    });
}

}  // namespace hin
#endif
