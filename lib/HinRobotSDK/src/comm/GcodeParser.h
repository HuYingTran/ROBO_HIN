#pragma once
#include <cstdint>

#include "../common/Status.h"
#include "../common/Types.h"

namespace hin {

// Một dòng G-code đã phân tích. Thuần C++.
//   G0 J1=100 J2=45 J3=-30 J4=0 F50
//   G1 X200 Y50 Z40 A0 F40
//   M50 L · M3 S80 · G4 P500
struct GcodeLine {
    enum Word : uint16_t {
        kX = 1 << 0, kY = 1 << 1, kZ = 1 << 2, kA = 1 << 3,
        kF = 1 << 4, kS = 1 << 5, kP = 1 << 6,
    };
    char letter = 0;  // 'G' hoặc 'M'
    int code = -1;
    uint16_t words = 0;
    uint8_t jointMask = 0;  // bit i = có Ji+1
    float joints[kMaxJoints] = {};
    float x = 0, y = 0, z = 0, a = 0, f = 0, s = 0, p = 0;
    char flag = 0;  // tham số chữ đứng riêng, vd. 'L' / 'R' trong M50

    bool has(Word w) const { return (words & w) != 0; }
    bool hasJoint(int i) const { return (jointMask >> i) & 1u; }
};

// Trả InvalidArgument nếu dòng sai cú pháp; dòng rỗng / chỉ chú thích → letter = 0.
Status parseGcode(const char* line, GcodeLine& out);

}  // namespace hin
