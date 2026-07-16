#include "storage/Transform.h"

#include <algorithm>
#include <cmath>

namespace avb {

namespace {
constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;
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
    m.block<3, 3>(0, 0) = rotationMatrix() * scale.asDiagonal();
    m.block<3, 1>(0, 3) = translation;
    return m;
}

bool Transform::isIdentity() const {
    return translation.isZero() && rotationEulerDeg.isZero() && scale.isOnes();
}

Transform Transform::fromMatrix(const Eigen::Matrix4f& m) {
    Transform t;
    t.translation = m.block<3, 1>(0, 3);

    Eigen::Matrix3f rs = m.block<3, 3>(0, 0);
    const float sx = rs.col(0).norm();
    const float sy = rs.col(1).norm();
    const float sz = rs.col(2).norm();
    t.scale = Eigen::Vector3f(std::max(sx, 1e-4f), std::max(sy, 1e-4f),
                              std::max(sz, 1e-4f));
    if (sx > 1e-6f) rs.col(0) /= sx;
    if (sy > 1e-6f) rs.col(1) /= sy;
    if (sz > 1e-6f) rs.col(2) /= sz;

    // eulerAngles(2,1,0) yields (a,b,c) with R = Rz(a) * Ry(b) * Rx(c) -
    // exactly the composition rotationMatrix() builds.
    const Eigen::Vector3f zyx = rs.eulerAngles(2, 1, 0);
    t.rotationEulerDeg = Eigen::Vector3f(
        zyx.z() * kRadToDeg, zyx.y() * kRadToDeg, zyx.x() * kRadToDeg);
    return t;
}

} // namespace avb
