#if defined(ARDUINO)
#include "WifiManager.h"

#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>

#include <cstring>

#include "../common/Log.h"

namespace hin {

namespace {
constexpr const char* kNs = "hinwifi";
}

Status WifiManager::begin() { return begin(Options{}); }

Status WifiManager::begin(const Options& opt) {
    opt_ = opt;
    WiFi.persistent(false);  // tự quản lý thông tin WiFi trong NVS
    WiFi.setHostname(opt_.hostname);

    char ssid[33] = "", pass[65] = "";
    Preferences p;
    if (p.begin(kNs, true)) {
        p.getString("ssid", ssid, sizeof(ssid));
        p.getString("pass", pass, sizeof(pass));
        p.end();
    }

    if (!(ssid[0] && startSta(ssid, pass))) startAp();

    if (MDNS.begin(opt_.hostname)) MDNS.addService("http", "tcp", 80);
    HIN_LOGI("WiFi %s: %s  IP %s  http://%s.local", ap_ ? "AP" : "STA", ssid_, ip_, opt_.hostname);
    return Status::Ok;
}

bool WifiManager::startSta(const char* ssid, const char* pass) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // giảm độ trễ WebSocket
    WiFi.begin(ssid, pass);
    const uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - t0 > opt_.staTimeoutMs) {
            HIN_LOGW("khong ket noi duoc WiFi '%s' — chuyen sang AP", ssid);
            WiFi.disconnect(true);
            return false;
        }
        delay(100);
    }
    ap_ = false;
    strncpy(ssid_, ssid, sizeof(ssid_) - 1);
    strncpy(ip_, WiFi.localIP().toString().c_str(), sizeof(ip_) - 1);
    return true;
}

void WifiManager::startAp() {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(ssid_, sizeof(ssid_), "ROBO-HIN-%02X%02X", mac[4], mac[5]);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid_, opt_.apPassword);
    ap_ = true;
    strncpy(ip_, WiFi.softAPIP().toString().c_str(), sizeof(ip_) - 1);
}

Status WifiManager::saveCredentials(const char* ssid, const char* pass) {
    if (!ssid || !ssid[0] || strlen(ssid) > 32 || (pass && strlen(pass) > 64))
        return Status::InvalidArgument;
    Preferences p;
    if (!p.begin(kNs, false)) return Status::StorageError;
    p.putString("ssid", ssid);
    p.putString("pass", pass ? pass : "");
    p.end();
    return Status::Ok;
}

Status WifiManager::clearCredentials() {
    Preferences p;
    if (!p.begin(kNs, false)) return Status::StorageError;
    p.clear();
    p.end();
    return Status::Ok;
}

int WifiManager::rssi() const { return ap_ ? 0 : WiFi.RSSI(); }

}  // namespace hin
#endif
