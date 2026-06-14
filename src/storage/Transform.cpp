#include "storage/Transform.h"

#include <cmath>

namespace avb {

namespace {
constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
}

Eigen::Matrix3f Transform::rotationMatrix() const {
    const Eigen::Vector3f r = rotationEulerDeg * kDegToRad;
    // Intrinsic X (pitch) -> Y (yaw) -> Z (roll).
    const Eigen::AngleAxisf rx(r.x(), Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf ry(r.y(), Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf rz(r.z(), Eigen::Vector3f::UnitZ());
    return (rz * ry * rx).toRotationMatrix();
}

Eigen::Matrix4f Transform::toMatrix() const {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<3, 3>(0, 0) = rotationMatrix() * scale;
    m.block<3, 1>(0, 3) = translation;
    return m;
}

bool Transform::isIdentity() const {
    return translation.isZero() && rotationEulerDeg.isZero() && scale == 1.0f;
}

} // namespace avb
