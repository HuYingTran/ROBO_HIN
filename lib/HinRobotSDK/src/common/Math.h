#pragma once
#include <cmath>

namespace hin {

constexpr float kPi = 3.14159265358979323846f;

constexpr float deg2rad(float d) { return d * (kPi / 180.0f); }
constexpr float rad2deg(float r) { return r * (180.0f / kPi); }

template <typename T>
constexpr T clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Đưa góc (độ) về [-180, 180).
inline float wrapDeg(float d) {
    d = std::fmod(d + 180.0f, 360.0f);
    if (d < 0) d += 360.0f;
    return d - 180.0f;
}

// Hiệu góc ngắn nhất a - b (độ), kết quả trong [-180, 180).
inline float angleDiffDeg(float a, float b) { return wrapDeg(a - b); }

}  // namespace hin
