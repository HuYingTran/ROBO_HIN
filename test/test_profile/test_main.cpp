// pio test -e native -f test_profile
#include <unity.h>

#include <cmath>

#include "motion/Profile.h"

using namespace hin;

void setUp() {}
void tearDown() {}

void test_trapezoid_reaches_vmax() {
    TrapezoidProfile p;
    TEST_ASSERT_TRUE(p.plan(100, 20, 40));  // ta = 0.5 s, d_acc = 5 mm
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 20.0f, p.peakVelocity());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, p.accelTime());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f + 4.5f + 0.5f, p.duration());
}

void test_triangle_when_short() {
    TrapezoidProfile p;
    TEST_ASSERT_TRUE(p.plan(4, 20, 40));
    TEST_ASSERT_TRUE(p.peakVelocity() < 20.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, std::sqrt(4.0f * 40.0f), p.peakVelocity());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.0f, p.position(p.duration()));
}

void test_endpoints_and_monotonic() {
    TrapezoidProfile p;
    p.plan(1.0f, 0.7f, 2.3f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.position(0));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, p.position(p.duration()));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, p.position(p.duration() + 1));
    float prev = 0;
    for (float t = 0; t <= p.duration(); t += 0.001f) {
        const float s = p.position(t);
        TEST_ASSERT_TRUE(s >= prev - 1e-6f);
        TEST_ASSERT_TRUE(p.velocity(t) <= 0.7f + 1e-5f);
        prev = s;
    }
}

// Vận tốc là đạo hàm của vị trí; liên tục tại các điểm chuyển pha
void test_velocity_matches_derivative() {
    TrapezoidProfile p;
    p.plan(50, 10, 25);
    const float h = 1e-3f;
    for (float t = h; t < p.duration() - h; t += 0.05f) {
        const float num = (p.position(t + h) - p.position(t - h)) / (2 * h);
        TEST_ASSERT_FLOAT_WITHIN(0.05f, p.velocity(t), num);
    }
}

void test_zero_and_invalid() {
    TrapezoidProfile p;
    TEST_ASSERT_TRUE(p.plan(0, 10, 10));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.duration());
    TEST_ASSERT_FALSE(p.plan(10, 0, 10));
    TEST_ASSERT_FALSE(p.plan(10, 10, -1));
    TEST_ASSERT_FALSE(p.plan(-1, 10, 10));
}

// Đồng bộ đa trục: chuẩn hóa theo khớp chậm nhất → không khớp nào vượt vmax của nó
void test_normalized_sync_respects_limits() {
    const float delta[4] = {40, 90, -30, 15};
    const float vmax[4] = {40, 90, 120, 180};
    const float amax[4] = {200, 180, 240, 360};
    float vn = INFINITY, an = INFINITY;
    for (int i = 0; i < 4; ++i) {
        vn = std::fmin(vn, vmax[i] / std::fabs(delta[i]));
        an = std::fmin(an, amax[i] / std::fabs(delta[i]));
    }
    TrapezoidProfile p;
    p.plan(1.0f, vn, an);
    for (float t = 0; t <= p.duration(); t += 0.002f)
        for (int i = 0; i < 4; ++i)
            TEST_ASSERT_TRUE(std::fabs(delta[i]) * p.velocity(t) <= vmax[i] + 1e-3f);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_trapezoid_reaches_vmax);
    RUN_TEST(test_triangle_when_short);
    RUN_TEST(test_endpoints_and_monotonic);
    RUN_TEST(test_velocity_matches_derivative);
    RUN_TEST(test_zero_and_invalid);
    RUN_TEST(test_normalized_sync_respects_limits);
    return UNITY_END();
}
