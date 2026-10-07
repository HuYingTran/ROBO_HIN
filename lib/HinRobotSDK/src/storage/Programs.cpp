#if defined(ARDUINO)
#include "Programs.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

#include <cstdio>
#include <cstring>

namespace hin {

namespace {
constexpr const char* kDir = "/programs";

void pathFor(const char* name, char* buf, size_t len) { snprintf(buf, len, "%s/%s.json", kDir, name); }

void setErr(char* err, size_t len, const char* fmt, int a = 0) {
    if (err && len) snprintf(err, len, fmt, a);
}

bool readVec(JsonVariantConst v, float* out) {
    JsonArrayConst a = v.as<JsonArrayConst>();
    if (a.isNull() || a.size() != kMaxJoints) return false;
    for (int i = 0; i < kMaxJoints; ++i) {
        if (!a[i].is<float>()) return false;
        out[i] = a[i].as<float>();
    }
    return true;
}
}  // namespace

bool Programs::validName(const char* name) {
    const size_t n = name ? strlen(name) : 0;
    if (n == 0 || n >= Program::kNameMax) return false;
    for (size_t i = 0; i < n; ++i) {
        const char c = name[i];
        if (!(isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-')) return false;
    }
    return true;
}

Status Programs::parse(const char* json, size_t len, Program& out, char* err, size_t errLen) {
    JsonDocument doc;
    if (deserializeJson(doc, json, len)) {
        setErr(err, errLen, "JSON khong hop le");
        return Status::InvalidArgument;
    }
    out = Program{};
    const char* name = doc["name"] | "";
    strncpy(out.name, name, Program::kNameMax - 1);
    out.speed = doc["speed"] | 50.0f;
    JsonArrayConst steps = doc["steps"].as<JsonArrayConst>();
    if (steps.isNull()) {
        setErr(err, errLen, "thieu \"steps\"");
        return Status::InvalidArgument;
    }
    if (steps.size() > static_cast<size_t>(Program::kMaxSteps)) {
        setErr(err, errLen, "toi da %d buoc", Program::kMaxSteps);
        return Status::InvalidArgument;
    }
    int i = 0;
    for (JsonObjectConst s : steps) {
        ProgramStep& st = out.steps[i];
        const char* op = s["op"] | "";
        st.gripper = s["gripper"] | -1.0f;
        st.speed = s["speed"] | -1.0f;
        bool ok = true;
        if (!strcmp(op, "moveJ")) {
            if (s["joints"].is<JsonArrayConst>()) {
                st.op = ProgramStep::Op::MoveJJoints;
                ok = readVec(s["joints"], st.v);
            } else {
                st.op = ProgramStep::Op::MoveJ;
                ok = readVec(s["pose"], st.v);
            }
        } else if (!strcmp(op, "moveL")) {
            st.op = ProgramStep::Op::MoveL;
            ok = readVec(s["pose"], st.v);
        } else if (!strcmp(op, "wait")) {
            st.op = ProgramStep::Op::Wait;
            st.ms = s["ms"] | 0u;
        } else if (!strcmp(op, "gripper")) {
            st.op = ProgramStep::Op::Gripper;
            st.gripper = s["pct"] | 0.0f;
        } else {
            ok = false;
        }
        if (!ok) {
            setErr(err, errLen, "buoc %d sai dinh dang", i + 1);
            return Status::InvalidArgument;
        }
        ++i;
    }
    out.count = static_cast<uint16_t>(i);
    return Status::Ok;
}

Status Programs::listJson(String& out) {
    out = "[";
    File dir = LittleFS.open(kDir);
    if (dir && dir.isDirectory()) {
        bool first = true;
        for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
            String n = f.name();
            const int slash = n.lastIndexOf('/');
            if (slash >= 0) n = n.substring(slash + 1);
            if (!n.endsWith(".json")) continue;
            n.remove(n.length() - 5);
            if (!first) out += ',';
            out += '"';
            out += n;
            out += '"';
            first = false;
        }
    }
    out += "]";
    return Status::Ok;
}

Status Programs::read(const char* name, String& out) {
    if (!validName(name)) return Status::InvalidArgument;
    char path[64];
    pathFor(name, path, sizeof(path));
    File f = LittleFS.open(path, "r");
    if (!f) return Status::OutOfRange;
    out = f.readString();
    return Status::Ok;
}

Status Programs::write(const char* name, const char* body, size_t len) {
    if (!validName(name) || len > kMaxFileSize) return Status::InvalidArgument;
    if (!LittleFS.exists(kDir)) LittleFS.mkdir(kDir);
    char path[64];
    pathFor(name, path, sizeof(path));
    File f = LittleFS.open(path, "w");
    if (!f) return Status::StorageError;
    const size_t n = f.write(reinterpret_cast<const uint8_t*>(body), len);
    f.close();
    return n == len ? Status::Ok : Status::StorageError;
}

Status Programs::remove(const char* name) {
    if (!validName(name)) return Status::InvalidArgument;
    char path[64];
    pathFor(name, path, sizeof(path));
    return LittleFS.remove(path) ? Status::Ok : Status::OutOfRange;
}

Status Programs::load(const char* name, Program& out) {
    String body;
    const Status s = read(name, body);
    if (s != Status::Ok) return s;
    return parse(body.c_str(), body.length(), out);
}

}  // namespace hin
#endif
