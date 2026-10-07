#pragma once
#include "../common/Status.h"
#include "../common/Types.h"

namespace hin {

// Động học giải tích SCARA TRRR (J1 tịnh tiến Z + 2 khâu phẳng + xoay tay kẹp).
// Thuần C++, không phụ thuộc Arduino. API dùng mm / độ, bên trong tính bằng rad.
class ScaraKinematics {
public:
    ScaraKinematics() = default;
    explicit ScaraKinematics(const ScaraGeometry& g) : g_(g) {}

    void setGeometry(const ScaraGeometry& g) { g_ = g; }
    const ScaraGeometry& geometry() const { return g_; }

    // Đặt soft limit khớp để IK lọc nghiệm. Không gọi → không lọc.
    void setLimits(const float* minPos, const float* maxPos);

    CartPose fk(const JointPose& q) const;

    // ref: tư thế tham chiếu (vd. vị trí hiện tại) để chọn nghiệm Nearest và chọn θ4 ± 360°
    // gần nhất. Có thể nullptr.
    Status ik(const CartPose& p, ElbowConfig elbow, const JointPose* ref, JointPose& out) const;

    float maxReach() const;
    float minReach() const;

    // Khoảng cách tới điểm kỳ dị (|sin θ3|): 0 = duỗi thẳng / gập hẳn.
    static float singularityMeasure(const JointPose& q);

private:
    bool withinLimits(int j, float v) const;
    bool solveWrist(float t4, const JointPose* ref, float& out) const;

    ScaraGeometry g_{150.0f, 120.0f, 60.0f, 80.0f};
    bool hasLimits_ = false;
    float min_[kMaxJoints] = {};
    float max_[kMaxJoints] = {};
};

}  // namespace hin
