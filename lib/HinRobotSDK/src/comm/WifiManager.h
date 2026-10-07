#pragma once
#include <cstdint>

#include "../common/Status.h"

namespace hin {

// Kết nối WiFi: thử STA bằng SSID lưu trong NVS (15 s), không được → phát AP riêng
// "ROBO-HIN-XXXX" (WPA2) tại 192.168.4.1. Luôn bật mDNS: http://<hostname>.local
class WifiManager {
public:
    struct Options {
        const char* hostname = "robo-hin";
        const char* apPassword = "hinrobot";  // ≥ 8 ký tự (WPA2)
        uint32_t staTimeoutMs = 15000;
    };

    Status begin();
    Status begin(const Options& opt);
    Status saveCredentials(const char* ssid, const char* pass);
    Status clearCredentials();

    bool isAp() const { return ap_; }
    const char* ssid() const { return ssid_; }
    const char* ip() const { return ip_; }
    const char* hostname() const { return opt_.hostname; }
    int rssi() const;

private:
    bool startSta(const char* ssid, const char* pass);
    void startAp();

    Options opt_;
    bool ap_ = false;
    char ssid_[33] = "";
    char ip_[16] = "";
};

}  // namespace hin
