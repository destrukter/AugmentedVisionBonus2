#include "TestMain.h"

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
    t.scale = 2.0f;
    const Eigen::Matrix4f m = t.toMatrix();
    CHECK(std::abs(m(0, 0) - 2.0f) < 1e-5f);
}

void run_transform_tests() {
    test_default_is_identity();
    test_translation_in_matrix();
    test_rotation_90_about_y();
    test_scale_applied();
}
