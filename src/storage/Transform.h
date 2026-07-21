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

    /// Per-axis scale (x, y, z). Defaults to (1, 1, 1).
    Eigen::Vector3f scale{Eigen::Vector3f::Ones()};

    /// 3x3 rotation matrix built from the Euler angles.
    Eigen::Matrix3f rotationMatrix() const;

    /// Full 4x4 model matrix: translation * rotation * scale.
    Eigen::Matrix4f toMatrix() const;

    /// True when this is the default (identity) pose.
    bool isIdentity() const;

    /// Decomposes a translation * rotation * scale matrix (no shear) back
    /// into a Transform - the inverse of toMatrix(). Euler angles are chosen
    /// to reproduce the same rotation matrix (the decomposition is not unique,
    /// but toMatrix(fromMatrix(m)) == m). Used by the Configure window's
    /// gizmo to read a manipulated matrix back into pose values.
    static Transform fromMatrix(const Eigen::Matrix4f& m);
};

} // namespace avb
