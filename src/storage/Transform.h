#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace avb {

/// Rigid pose of an FBX model relative to its tracked image.
///
/// Per the spec, translation and rotation **default to zero** (identity pose).
/// Scale defaults to 1 so a freshly assigned model is visible at its native size.
struct Transform {
    /// Translation in image-space units (x, y, z). Defaults to (0, 0, 0).
    Eigen::Vector3f translation{Eigen::Vector3f::Zero()};

    /// Rotation as Euler angles in **degrees** (pitch=X, yaw=Y, roll=Z).
    /// Stored as Euler angles because that is what the Configure UI edits;
    /// use rotationMatrix()/toMatrix() to obtain the matrix form.
    /// Defaults to (0, 0, 0).
    Eigen::Vector3f rotationEulerDeg{Eigen::Vector3f::Zero()};

    /// Uniform scale. Defaults to 1.
    float scale{1.0f};

    /// 3x3 rotation matrix built from the Euler angles.
    Eigen::Matrix3f rotationMatrix() const;

    /// Full 4x4 model matrix: translation * rotation * scale.
    Eigen::Matrix4f toMatrix() const;

    /// True when this is the default (identity) pose.
    bool isIdentity() const;
};

} // namespace avb
