#include "TestMain.h"

#include <cmath>

#include "storage/Transform.h"

using namespace avb;

static void test_default_is_identity() {
    Transform t;
    CHECK(t.isIdentity());
    CHECK(t.toMatrix().isApprox(Eigen::Matrix4f::Identity()));
}

static void test_translation_in_matrix() {
    Transform t;
    t.translation = Eigen::Vector3f(4.0f, 5.0f, 6.0f);
    const Eigen::Matrix4f m = t.toMatrix();
    CHECK(m.block<3, 1>(0, 3).isApprox(Eigen::Vector3f(4.0f, 5.0f, 6.0f)));
}

static void test_rotation_90_about_y() {
    Transform t;
    t.rotationEulerDeg = Eigen::Vector3f(0.0f, 90.0f, 0.0f);
    const Eigen::Matrix3f r = t.rotationMatrix();
    // Rotating +X by 90deg about Y should give approximately -Z.
    const Eigen::Vector3f x = r * Eigen::Vector3f::UnitX();
    CHECK(x.isApprox(Eigen::Vector3f(0.0f, 0.0f, -1.0f), 1e-4f));
}

static void test_scale_applied() {
    Transform t;
    t.scale = Eigen::Vector3f(2.0f, 3.0f, 4.0f); // per-axis
    const Eigen::Matrix4f m = t.toMatrix();
    CHECK(std::abs(m(0, 0) - 2.0f) < 1e-5f);
    CHECK(std::abs(m(1, 1) - 3.0f) < 1e-5f);
    CHECK(std::abs(m(2, 2) - 4.0f) < 1e-5f);
}

static void test_from_matrix_round_trip() {
    Transform t;
    t.translation = Eigen::Vector3f(0.4f, -0.2f, 1.3f);
    t.rotationEulerDeg = Eigen::Vector3f(25.0f, -40.0f, 130.0f);
    t.scale = Eigen::Vector3f(2.0f, 0.5f, 1.25f);

    // The Euler decomposition is not unique, but the reconstructed matrix
    // must reproduce the original exactly.
    const Transform back = Transform::fromMatrix(t.toMatrix());
    CHECK(back.toMatrix().isApprox(t.toMatrix(), 1e-4f));
    CHECK(back.translation.isApprox(t.translation, 1e-4f));
    CHECK(back.scale.isApprox(t.scale, 1e-4f));
}

static void test_from_matrix_of_composed_rigid_poses() {
    // Composing two rigid (translation+rotation) poses stays rigid, so
    // fromMatrix must recover it with scale 1 - the "Set origin here" fold.
    Transform a;
    a.translation = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    a.rotationEulerDeg = Eigen::Vector3f(0.0f, 90.0f, 0.0f);
    Transform b;
    b.translation = Eigen::Vector3f(-0.5f, 0.25f, 0.0f);
    b.rotationEulerDeg = Eigen::Vector3f(30.0f, 0.0f, -45.0f);

    const Eigen::Matrix4f composed = a.toMatrix() * b.toMatrix();
    const Transform c = Transform::fromMatrix(composed);
    CHECK(c.toMatrix().isApprox(composed, 1e-4f));
    CHECK(c.scale.isApprox(Eigen::Vector3f::Ones(), 1e-3f));
}

void run_transform_tests() {
    test_default_is_identity();
    test_translation_in_matrix();
    test_rotation_90_about_y();
    test_scale_applied();
    test_from_matrix_round_trip();
    test_from_matrix_of_composed_rigid_poses();
}
