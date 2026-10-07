#include "GcodeParser.h"

#include <cctype>
#include <cstdlib>

namespace hin {

namespace {

bool parseNumber(const char*& p, float& out) {
    char* end = nullptr;
    out = std::strtof(p, &end);
    if (end == p) return false;
    p = end;
    return true;
}

bool atTokenEnd(const char* p) { return *p == '\0' || std::isspace(static_cast<unsigned char>(*p)); }

}  // namespace

Status parseGcode(const char* line, GcodeLine& out) {
    out = GcodeLine{};
    const char* p = line;
    bool first = true;

    while (*p) {
        while (*p && std::isspace(static_cast<unsigned char>(*p))) ++p;
        if (!*p || *p == ';' || *p == '(') break;  // chú thích

        const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(*p++)));

        if (first) {
            if (c != 'G' && c != 'M') return Status::InvalidArgument;
            float code;
            if (!parseNumber(p, code) || !atTokenEnd(p)) return Status::InvalidArgument;
            out.letter = c;
            out.code = static_cast<int>(code);
            first = false;
            continue;
        }

        // Cờ chữ đứng riêng: "M50 L"
        if (atTokenEnd(p) && std::isalpha(static_cast<unsigned char>(c))) {
            out.flag = c;
            continue;
        }

        if (c == 'J') {
            float idx;
            if (!parseNumber(p, idx) || *p != '=') return Status::InvalidArgument;
            ++p;
            const int i = static_cast<int>(idx) - 1;
            float v;
            if (i < 0 || i >= kMaxJoints || !parseNumber(p, v)) return Status::InvalidArgument;
            out.joints[i] = v;
            out.jointMask |= static_cast<uint8_t>(1u << i);
        } else {
            float v;
            if (!parseNumber(p, v)) return Status::InvalidArgument;
            switch (c) {
                case 'X': out.x = v; out.words |= GcodeLine::kX; break;
                case 'Y': out.y = v; out.words |= GcodeLine::kY; break;
                case 'Z': out.z = v; out.words |= GcodeLine::kZ; break;
                case 'A': out.a = v; out.words |= GcodeLine::kA; break;
                case 'F': out.f = v; out.words |= GcodeLine::kF; break;
                case 'S': out.s = v; out.words |= GcodeLine::kS; break;
                case 'P': out.p = v; out.words |= GcodeLine::kP; break;
                default: return Status::InvalidArgument;
            }
        }
        if (!atTokenEnd(p)) return Status::InvalidArgument;
    }
    return Status::Ok;
}

}  // namespace hin
