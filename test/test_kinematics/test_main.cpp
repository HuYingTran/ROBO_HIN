// pio test -e native -f test_kinematics
#include <unity.h>

#include <cmath>

#include "kinematics/ScaraKinematics.h"

using namespace hin;

namespace {
const ScaraGeometry kGeo{150.0f, 120.0f, 60.0f, 80.0f};
const float kMin[4] = {0, -135, -145, -180};
const float kMax[4] = {200, 135, 145, 180};

ScaraKinematics makeKin(bool limits = true) {
    ScaraKinematics k(kGeo);
    if (limits) k.setLimits(kMin, kMax);
    return k;
}
}  // namespace

void setUp() {}
void tearDown() {}

void test_fk_stretched_along_x() {
    const ScaraKinematics k = makeKin();
    const CartPose p = k.fk({{100, 0, 0, 0}});
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 270.0f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, p.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 120.0f, p.z);  // z0 + d1 − d4
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, p.phi);
}

void test_fk_right_angle() {
    const ScaraKinematics k = makeKin();
    const CartPose p = k.fk({{0, 90, -90, 0}});
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 120.0f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 150.0f, p.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, p.phi);
}

// IK(FK(q)) == q cho cả 2 nghiệm khuỷu trên một lưới tư thế
void test_ik_fk_roundtrip_both_elbows() {
    const ScaraKinematics k = makeKin();
    int checked = 0;
    for (float z = 0; z <= 200; z += 50)
        for (float t2 = -120; t2 <= 120; t2 += 20)
            for (float t3 = -140; t3 <= 140; t3 += 20) {
                if (std::fabs(t3) < 5) continue;  // tránh điểm kỳ dị
                for (float t4 = -150; t4 <= 150; t4 += 50) {
                    const JointPose q{{z, t2, t3, t4}};
                    const CartPose p = k.fk(q);
                    const ElbowConfig e = t3 > 0 ? ElbowConfig::Left : ElbowConfig::Right;
                    JointPose out;
                    TEST_ASSERT_EQUAL(static_cast<int>(Status::Ok),
                                      static_cast<int>(k.ik(p, e, &q, out)));
                    TEST_ASSERT_FLOAT_WITHIN(1e-2f, z, out[0]);
                    TEST_ASSERT_FLOAT_WITHIN(2e-2f, t2, out[1]);
                    TEST_ASSERT_FLOAT_WITHIN(2e-2f, t3, out[2]);
                    TEST_ASSERT_FLOAT_WITHIN(5e-2f, t4, out[3]);
                    ++checked;
                }
            }
    TEST_ASSERT_GREATER_THAN(500, checked);
}

void test_ik_nearest_picks_closest() {
    const ScaraKinematics k = makeKin();
    const JointPose ref{{50, 30, -60, 0}};
    JointPose out;
    TEST_ASSERT_EQUAL(static_cast<int>(Status::Ok),
                      static_cast<int>(k.ik(k.fk(ref), ElbowConfig::Nearest, &ref, out)));
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, -60.0f, out[2]);
}

void test_ik_out_of_reach() {
    const ScaraKinematics k = makeKin();
    JointPose out;
    TEST_ASSERT_EQUAL(static_cast<int>(Status::OutOfReach),
                      static_cast<int>(k.ik({300, 0, 50, 0}, ElbowConfig::Nearest, nullptr, out)));
    TEST_ASSERT_EQUAL(static_cast<int>(Status::OutOfReach),
                      static_cast<int>(k.ik({10, 0, 50, 0}, ElbowConfig::Nearest, nullptr, out)));
}

void test_ik_joint_limit() {
    const ScaraKinematics k = makeKin();
    JointPose out;
    // Z dưới giới hạn: z = 0 → d1 = −20
    TEST_ASSERT_EQUAL(static_cast<int>(Status::JointLimit),
                      static_cast<int>(k.ik({200, 0, 0, 0}, ElbowConfig::Nearest, nullptr, out)));
    // Sau lưng robot (θ2 ≈ 180°) vượt ±135°
    TEST_ASSERT_EQUAL(static_cast<int>(Status::JointLimit),
                      static_cast<int>(k.ik({-250, 0, 100, 0}, ElbowConfig::Nearest, nullptr, out)));
}

void test_ik_boundary_reach() {
    const ScaraKinematics k = makeKin();
    JointPose out;
    TEST_ASSERT_EQUAL(static_cast<int>(Status::Ok),
                      static_cast<int>(k.ik({270, 0, 100, 0}, ElbowConfig::Nearest, nullptr, out)));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, out[2]);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_fk_stretched_along_x);
    RUN_TEST(test_fk_right_angle);
    RUN_TEST(test_ik_fk_roundtrip_both_elbows);
    RUN_TEST(test_ik_nearest_picks_closest);
    RUN_TEST(test_ik_out_of_reach);
    RUN_TEST(test_ik_joint_limit);
    RUN_TEST(test_ik_boundary_reach);
    return UNITY_END();
}
