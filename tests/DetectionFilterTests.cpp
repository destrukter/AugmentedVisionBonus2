#include "TestMain.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "vision/DetectionFilter.h"

using namespace avb;

namespace {

Detection makeDetection(Id id, float x, float y, float z) {
    Detection d;
    d.imageId = id;
    d.poseInCamera = Eigen::Matrix4f::Identity();
    d.poseInCamera(0, 3) = x;
    d.poseInCamera(1, 3) = y;
    d.poseInCamera(2, 3) = z;
    d.confidence = 1.0f;
    return d;
}

Eigen::Vector3f position(const Detection& d) {
    return d.poseInCamera.block<3, 1>(0, 3);
}

} // namespace

static void test_first_sighting_passes_through() {
    DetectionFilter filter;
    const auto out = filter.apply({makeDetection(1, 0.1f, 0.2f, -3.0f)}, 0.0);
    CHECK(out.size() == 1);
    if (out.empty()) {
        return;
    }
    CHECK((position(out[0]) - Eigen::Vector3f(0.1f, 0.2f, -3.0f)).norm() < 1e-6f);
}

static void test_small_movement_is_smoothed() {
    DetectionFilter filter(DetectionFilter::Params{/*smoothing=*/0.5f,
                                                   /*holdMs=*/250.0,
                                                   /*snapDistance=*/0.5f});
    filter.apply({makeDetection(1, 0.0f, 0.0f, -3.0f)}, 0.0);
    const auto out = filter.apply({makeDetection(1, 0.2f, 0.0f, -3.0f)}, 33.0);
    CHECK(out.size() == 1);
    if (out.empty()) {
        return;
    }
    // With alpha 0.5 the smoothed x must land halfway between 0 and 0.2.
    CHECK(std::abs(position(out[0]).x() - 0.1f) < 1e-4f);
}

static void test_large_jump_snaps() {
    DetectionFilter filter(DetectionFilter::Params{/*smoothing=*/0.5f,
                                                   /*holdMs=*/250.0,
                                                   /*snapDistance=*/0.5f});
    filter.apply({makeDetection(1, 0.0f, 0.0f, -3.0f)}, 0.0);
    const auto out = filter.apply({makeDetection(1, 2.0f, 0.0f, -3.0f)}, 33.0);
    CHECK(out.size() == 1);
    if (out.empty()) {
        return;
    }
    // A 2-unit jump exceeds snapDistance: take the new pose outright instead
    // of gliding across the scene.
    CHECK(std::abs(position(out[0]).x() - 2.0f) < 1e-6f);
}

static void test_lost_target_is_held_then_dropped() {
    DetectionFilter filter(DetectionFilter::Params{/*smoothing=*/0.5f,
                                                   /*holdMs=*/250.0,
                                                   /*snapDistance=*/0.5f});
    filter.apply({makeDetection(1, 0.0f, 0.0f, -3.0f)}, 0.0);

    // Missed for 100 ms: still reported (hold).
    auto out = filter.apply({}, 100.0);
    CHECK(out.size() == 1);

    // Missed beyond the 250 ms hold: dropped.
    out = filter.apply({}, 400.0);
    CHECK(out.empty());
}

static void test_reset_drops_state() {
    DetectionFilter filter;
    filter.apply({makeDetection(1, 0.0f, 0.0f, -3.0f)}, 0.0);
    filter.reset();
    const auto out = filter.apply({}, 1.0);
    CHECK(out.empty());
}

static void test_rotation_blend_stays_valid() {
    DetectionFilter filter(DetectionFilter::Params{/*smoothing=*/0.5f,
                                                   /*holdMs=*/250.0,
                                                   /*snapDistance=*/0.5f});
    Detection a = makeDetection(1, 0.0f, 0.0f, -3.0f);
    Detection b = makeDetection(1, 0.0f, 0.0f, -3.0f);
    // 90 degrees about Z.
    b.poseInCamera.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(static_cast<float>(M_PI / 2), Eigen::Vector3f::UnitZ())
            .toRotationMatrix();

    filter.apply({a}, 0.0);
    const auto out = filter.apply({b}, 33.0);
    CHECK(out.size() == 1);
    if (out.empty()) {
        return;
    }
    const Eigen::Matrix3f r = out[0].poseInCamera.block<3, 3>(0, 0);
    // Slerp midpoint of 0 and 90 degrees is 45 degrees; the blend must remain
    // a proper rotation (orthonormal, det 1).
    CHECK(std::abs(r.determinant() - 1.0f) < 1e-4f);
    const float angle = std::acos(std::clamp((r.trace() - 1.0f) * 0.5f, -1.0f, 1.0f));
    CHECK(std::abs(angle - static_cast<float>(M_PI / 4)) < 1e-3f);
}

void run_detectionfilter_tests() {
    test_first_sighting_passes_through();
    test_small_movement_is_smoothed();
    test_large_jump_snaps();
    test_lost_target_is_held_then_dropped();
    test_reset_drops_state();
    test_rotation_blend_stays_valid();
}
