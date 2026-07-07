#include "TestMain.h"

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>

#include "vision/ImageTracker.h"

using namespace avb;

namespace {

// Dense per-pixel noise gives ORB's FAST detector plenty of distinct corners
// to find, unlike a flat or smoothly-varying image. cv::RNG is deterministic
// for a fixed seed, so the same pattern is reproduced on every test run.
cv::Mat makeNoiseMarker(int size, uint64_t seed) {
    cv::Mat img(size, size, CV_8UC1);
    cv::RNG rng(seed);
    rng.fill(img, cv::RNG::UNIFORM, 0, 256);
    return img;
}

// Pastes `marker` unmodified into a plain, featureless canvas at (x, y) so the
// tracker has to actually locate it rather than matching a frame that's
// nothing but the template.
cv::Mat makeFrameWithMarker(const cv::Mat& marker, int canvasSize, int x, int y) {
    cv::Mat frame(canvasSize, canvasSize, CV_8UC1, cv::Scalar(128));
    marker.copyTo(frame(cv::Rect(x, y, marker.cols, marker.rows)));
    return frame;
}

} // namespace

static void test_detects_synthetic_marker() {
    ImageTracker tracker;
    const cv::Mat marker = makeNoiseMarker(160, 1);
    tracker.addTarget(1, marker);

    const cv::Mat frame = makeFrameWithMarker(marker, 400, 100, 100);
    const std::vector<Detection> detections = tracker.detect(frame);

    CHECK(detections.size() == 1);
    if (detections.empty()) {
        return;
    }
    CHECK(detections[0].imageId == 1);
    CHECK(detections[0].confidence > 0.3f);
    // Corners are clockwise from the top-left; the pasted region is at
    // (100,100)-(260,260) with no rotation/scale, so they should land close.
    CHECK(std::abs(detections[0].corners[0].x - 100.0f) < 5.0f);
    CHECK(std::abs(detections[0].corners[0].y - 100.0f) < 5.0f);
    CHECK(std::abs(detections[0].corners[2].x - 260.0f) < 5.0f);
    CHECK(std::abs(detections[0].corners[2].y - 260.0f) < 5.0f);
}

static void test_ignores_featureless_target() {
    ImageTracker tracker;
    const cv::Mat blank(160, 160, CV_8UC1, cv::Scalar(200)); // solid colour, no corners
    tracker.addTarget(2, blank);

    // Even pasted directly into the frame, a target with no extractable
    // features should never have been registered, so nothing is detected.
    const cv::Mat frame = makeFrameWithMarker(blank, 400, 100, 100);
    const std::vector<Detection> detections = tracker.detect(frame);
    CHECK(detections.empty());
}

static void test_add_remove_clear_targets() {
    ImageTracker tracker;
    const cv::Mat markerA = makeNoiseMarker(160, 2);
    const cv::Mat markerB = makeNoiseMarker(160, 3);

    tracker.addTarget(10, markerA);
    const cv::Mat frameA = makeFrameWithMarker(markerA, 400, 50, 50);
    CHECK(tracker.detect(frameA).size() == 1);

    tracker.removeTarget(10);
    CHECK(tracker.detect(frameA).empty());

    tracker.addTarget(11, markerA);
    tracker.addTarget(12, markerB);
    cv::Mat frameBoth = makeFrameWithMarker(markerA, 400, 20, 20);
    markerB.copyTo(frameBoth(cv::Rect(220, 220, markerB.cols, markerB.rows)));

    const std::vector<Detection> both = tracker.detect(frameBoth);
    CHECK(both.size() == 2);
    const bool has11 = std::any_of(both.begin(), both.end(),
                                   [](const Detection& d) { return d.imageId == 11; });
    const bool has12 = std::any_of(both.begin(), both.end(),
                                   [](const Detection& d) { return d.imageId == 12; });
    CHECK(has11);
    CHECK(has12);

    tracker.clearTargets();
    CHECK(tracker.detect(frameBoth).empty());
}

void run_imagetracker_tests() {
    test_detects_synthetic_marker();
    test_ignores_featureless_target();
    test_add_remove_clear_targets();
}
