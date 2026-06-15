#include "TestMain.h"

#include <cmath>

#include <opencv2/imgproc.hpp>

#include "vision/ImageTracker.h"

using namespace avb;

namespace {

// Builds a feature-rich marker image (shapes + text give ORB plenty of corners).
cv::Mat makeMarker() {
    cv::Mat m(300, 400, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::rectangle(m, {20, 20}, {180, 160}, {0, 0, 0}, 3);
    cv::rectangle(m, {220, 40}, {360, 200}, {40, 40, 200}, cv::FILLED);
    cv::circle(m, {120, 220}, 50, {0, 160, 0}, cv::FILLED);
    cv::circle(m, {300, 240}, 35, {200, 120, 0}, 4);
    cv::line(m, {0, 0}, {399, 299}, {120, 0, 120}, 2);
    cv::putText(m, "AVB-MARKER", {30, 290}, cv::FONT_HERSHEY_SIMPLEX, 1.0,
                {0, 0, 0}, 2);
    return m;
}

bool isFinite(const Eigen::Matrix4f& m) {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (!std::isfinite(m(r, c))) {
                return false;
            }
        }
    }
    return true;
}

} // namespace

static void test_featureless_target_is_ignored() {
    ImageTracker tracker;
    cv::Mat blank(200, 200, CV_8UC3, cv::Scalar(128, 128, 128));
    tracker.addTarget(1, blank);
    // A flat grey image has no ORB features, so nothing should be tracked.
    cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
    CHECK(tracker.detect(frame).empty());
}

static void test_detects_registered_marker() {
    ImageTracker tracker;
    const cv::Mat marker = makeMarker();
    tracker.addTarget(42, marker);

    // Place the marker into a larger black canvas (a clean, undistorted view).
    cv::Mat frame(600, 800, CV_8UC3, cv::Scalar(0, 0, 0));
    marker.copyTo(frame(cv::Rect(200, 150, marker.cols, marker.rows)));

    const std::vector<Detection> dets = tracker.detect(frame);
    CHECK(dets.size() == 1);
    if (!dets.empty()) {
        CHECK(dets[0].imageId == 42);
        CHECK(dets[0].confidence > 0.0f);
        CHECK(isFinite(dets[0].poseInCamera));
    }
}

static void test_clear_and_remove_targets() {
    ImageTracker tracker;
    const cv::Mat marker = makeMarker();
    tracker.addTarget(1, marker);
    tracker.addTarget(2, marker);
    tracker.removeTarget(1);

    cv::Mat frame(600, 800, CV_8UC3, cv::Scalar(0, 0, 0));
    marker.copyTo(frame(cv::Rect(200, 150, marker.cols, marker.rows)));
    // Only id 2 remains registered.
    const std::vector<Detection> dets = tracker.detect(frame);
    for (const auto& d : dets) {
        CHECK(d.imageId == 2);
    }

    tracker.clearTargets();
    CHECK(tracker.detect(frame).empty());
}

void run_imagetracker_tests() {
    test_featureless_target_is_ignored();
    test_detects_registered_marker();
    test_clear_and_remove_targets();
}
