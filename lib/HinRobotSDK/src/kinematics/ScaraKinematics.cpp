#include "ScaraKinematics.h"

#include <cmath>
#include <initializer_list>

#include "../common/Math.h"

namespace hin {

namespace {
constexpr float kReachEps = 1e-3f;  // mm
constexpr float kLimitEps = 1e-4f;
}  // namespace

void ScaraKinematics::setLimits(const float* minPos, const float* maxPos) {
    for (int i = 0; i < kMaxJoints; ++i) {
        min_[i] = minPos[i];
        max_[i] = maxPos[i];
    }
    hasLimits_ = true;
}

bool ScaraKinematics::withinLimits(int j, float v) const {
    return !hasLimits_ || (v >= min_[j] - kLimitEps && v <= max_[j] + kLimitEps);
}

float ScaraKinematics::maxReach() const { return g_.a2 + g_.a3; }
float ScaraKinematics::minReach() const { return std::fabs(g_.a2 - g_.a3); }

float ScaraKinematics::singularityMeasure(const JointPose& q) {
    return std::fabs(std::sin(deg2rad(q[2])));
}

CartPose ScaraKinematics::fk(const JointPose& q) const {
    const float t2 = deg2rad(q[1]);
    const float t23 = deg2rad(q[1] + q[2]);
    CartPose p;
    p.x = g_.a2 * std::cos(t2) + g_.a3 * std::cos(t23);
    p.y = g_.a2 * std::sin(t2) + g_.a3 * std::sin(t23);
    p.z = g_.z0 + q[0] - g_.d4;
    p.phi = wrapDeg(q[1] + q[2] + q[3]);
    return p;
}

// Chọn θ4 trong {t, t ± 360} nằm trong giới hạn, gần ref nhất.
bool ScaraKinematics::solveWrist(float t4, const JointPose* ref, float& out) const {
    t4 = wrapDeg(t4);
    bool found = false;
    float best = 0, bestCost = 0;
    for (float c : {t4, t4 - 360.0f, t4 + 360.0f}) {
        if (!withinLimits(3, c)) continue;
        const float cost = ref ? std::fabs(c - (*ref)[3]) : std::fabs(c);
        if (!found || cost < bestCost) {
            best = c;
            bestCost = cost;
            found = true;
        }
    }
    out = best;
    return found;
}

Status ScaraKinematics::ik(const CartPose& p, ElbowConfig elbow, const JointPose* ref,
                           JointPose& out) const {
    const float a2 = g_.a2, a3 = g_.a3;
    const float r2 = p.x * p.x + p.y * p.y;
    const float r = std::sqrt(r2);
    if (r > maxReach() + kReachEps || r < minReach() - kReachEps) return Status::OutOfReach;

    const float c3 = clamp((r2 - a2 * a2 - a3 * a3) / (2.0f * a2 * a3), -1.0f, 1.0f);
    const float s3 = std::sqrt(std::fmax(0.0f, 1.0f - c3 * c3));
    const float d1 = p.z - g_.z0 + g_.d4;
    if (!withinLimits(0, d1)) return Status::JointLimit;

    // Nghiệm 0: khuỷu TRÁI (θ3 > 0) · nghiệm 1: khuỷu PHẢI (θ3 < 0)
    JointPose cand[2];
    bool valid[2] = {false, false};
    for (int k = 0; k < 2; ++k) {
        const float t3 = std::atan2(k == 0 ? s3 : -s3, c3);
        const float t2 = std::atan2(p.y, p.x) -
                         std::atan2(a3 * std::sin(t3), a2 + a3 * std::cos(t3));
        JointPose& q = cand[k];
        q[0] = d1;
        q[1] = wrapDeg(rad2deg(t2));
        q[2] = rad2deg(t3);
        valid[k] = withinLimits(1, q[1]) && withinLimits(2, q[2]) &&
                   solveWrist(p.phi - q[1] - q[2], ref, q[3]);
    }

    int pick = -1;
    switch (elbow) {
        case ElbowConfig::Left: pick = valid[0] ? 0 : -1; break;
        case ElbowConfig::Right: pick = valid[1] ? 1 : -1; break;
        case ElbowConfig::Nearest:
            if (valid[0] && valid[1]) {
                if (ref) {
                    auto dist = [&](const JointPose& q) {
                        return std::fabs(q[1] - (*ref)[1]) + std::fabs(q[2] - (*ref)[2]);
                    };
                    pick = dist(cand[0]) <= dist(cand[1]) ? 0 : 1;
                } else {
                    pick = 0;
                }
            } else if (valid[0]) {
                pick = 0;
            } else if (valid[1]) {
                pick = 1;
            }
            break;
    }
    if (pick < 0) return Status::JointLimit;
    out = cand[pick];
    return Status::Ok;
}

}  // namespace hin
