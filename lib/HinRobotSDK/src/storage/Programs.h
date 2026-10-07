#pragma once
#include <cstddef>
#include <cstdint>

#include "../common/Status.h"
#include "../common/Types.h"

class String;

namespace hin {

// Một bước của chương trình teach (/programs/<name>.json):
//   { "op": "moveJ", "pose": [x, y, z, φ], "gripper": 0 }       PTP tới tư thế Descartes
//   { "op": "moveJ", "joints": [z, θ2, θ3, θ4] }                 PTP theo khớp
//   { "op": "moveL", "pose": [x, y, z, φ], "speed": 30 }         đường thẳng
//   { "op": "wait", "ms": 500 }
//   { "op": "gripper", "pct": 100 }
// "gripper" trên bước di chuyển được áp dụng SAU khi tới nơi. "speed" (%) ghi đè tốc độ chung.
struct ProgramStep {
    enum class Op : uint8_t { MoveJ, MoveJJoints, MoveL, Wait, Gripper };
    Op op = Op::MoveJ;
    float v[kMaxJoints] = {};  // pose hoặc joints
    float gripper = -1;        // < 0: không đổi
    float speed = -1;          // < 0: dùng tốc độ chương trình
    uint32_t ms = 0;
};

struct Program {
    static constexpr int kMaxSteps = 64;
    static constexpr size_t kNameMax = 32;
    char name[kNameMax] = "";
    float speed = 50;  // %
    uint16_t count = 0;
    ProgramStep steps[kMaxSteps];
};

// Đọc / ghi chương trình trên LittleFS. Tên hợp lệ: [A-Za-z0-9_-]{1,31}.
class Programs {
public:
    static constexpr size_t kMaxFileSize = 8192;

    static bool validName(const char* name);
    // Phân tích & kiểm tra JSON (không cần LittleFS) — dùng cả khi PUT.
    static Status parse(const char* json, size_t len, Program& out, char* err = nullptr,
                        size_t errLen = 0);

    Status listJson(String& out);  // ["a","b"]
    Status read(const char* name, String& out);
    Status write(const char* name, const char* body, size_t len);
    Status remove(const char* name);
    Status load(const char* name, Program& out);
};

}  // namespace hin
