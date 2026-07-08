#include "TestMain.h"

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

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

// Like makeNoiseMarker, but with `blockPx`-sized cells instead of per-pixel
// noise. Per-pixel noise has no structure that survives downscaling (averaging
// turns it into flat gray), so tests that exercise the tracker's internal
// frame downscale need features that exist across scales - as in real photos.
cv::Mat makeBlockNoiseMarker(int size, int blockPx, uint64_t seed) {
    const int cells = std::max(1, size / blockPx);
    cv::Mat coarse(cells, cells, CV_8UC1);
    cv::RNG rng(seed);
    rng.fill(coarse, cv::RNG::UNIFORM, 0, 256);
    cv::Mat img;
    cv::resize(coarse, img, cv::Size(size, size), 0, 0, cv::INTER_NEAREST);
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

static void test_detects_marker_under_poor_lighting() {
    ImageTracker tracker;
    const cv::Mat marker = makeNoiseMarker(160, 7);
    tracker.addTarget(5, marker);

    cv::Mat frame = makeFrameWithMarker(marker, 400, 100, 100);
    // Simulate a dim, low-contrast scene: squash the dynamic range hard
    // (roughly "marker in a poorly lit room"). The CLAHE preprocessing and
    // adaptive FAST threshold must still find and place it.
    cv::Mat dim;
    frame.convertTo(dim, -1, /*alpha=*/0.25, /*beta=*/12.0);

    const std::vector<Detection> detections = tracker.detect(dim);
    CHECK(detections.size() == 1);
    if (detections.empty()) {
        return;
    }
    CHECK(detections[0].imageId == 5);
    CHECK(std::abs(detections[0].corners[0].x - 100.0f) < 8.0f);
    CHECK(std::abs(detections[0].corners[0].y - 100.0f) < 8.0f);
    CHECK(std::abs(detections[0].corners[2].x - 260.0f) < 8.0f);
    CHECK(std::abs(detections[0].corners[2].y - 260.0f) < 8.0f);
}

static void test_downscaled_detection_keeps_fullres_coordinates() {
    // A frame larger than the tracker's internal detection resolution: the
    // reported corners must still be expressed in full-resolution pixels.
    ImageTracker tracker;
    const cv::Mat marker = makeBlockNoiseMarker(240, 8, 9);
    tracker.addTarget(6, marker);

    const int canvas = 1280; // > internal cap, forces the downscale path
    cv::Mat frame(canvas, canvas, CV_8UC1, cv::Scalar(128));
    marker.copyTo(frame(cv::Rect(500, 400, marker.cols, marker.rows)));

    const std::vector<Detection> detections = tracker.detect(frame);
    CHECK(detections.size() == 1);
    if (detections.empty()) {
        return;
    }
    // Loose tolerance on purpose: this asserts the *coordinate space* (a
    // missing scale-back would land corners around (250,200), hundreds of px
    // off), not the sub-pixel precision of the estimate.
    CHECK(std::abs(detections[0].corners[0].x - 500.0f) < 40.0f);
    CHECK(std::abs(detections[0].corners[0].y - 400.0f) < 40.0f);
    CHECK(std::abs(detections[0].corners[2].x - 740.0f) < 40.0f);
    CHECK(std::abs(detections[0].corners[2].y - 640.0f) < 40.0f);
}

// Renders `marker` as if printed on a plane tilted `tiltDeg` about the
// vertical axis, photographed by a pinhole camera looking straight at it.
cv::Mat makeTiltedFrame(const cv::Mat& marker, int canvas, double tiltDeg,
                        std::array<cv::Point2f, 4>& outCorners) {
    const double f = canvas * 1.2;
    const double c = canvas * 0.5;
    const double th = tiltDeg * CV_PI / 180.0;
    const double dist = 1.6;
    // Marker spans 0.9x0.9 units; OpenCV camera: X right, Y down, Z forward.
    const std::array<cv::Point3d, 4> obj = {cv::Point3d{-0.45, -0.45, 0},
                                            {0.45, -0.45, 0},
                                            {0.45, 0.45, 0},
                                            {-0.45, 0.45, 0}};
    std::array<cv::Point2f, 4> dst;
    for (int i = 0; i < 4; ++i) {
        const double xr = obj[i].x * std::cos(th); // rotate about Y
        const double zr = dist - obj[i].x * std::sin(th);
        dst[i] = cv::Point2f(static_cast<float>(f * xr / zr + c),
                             static_cast<float>(f * obj[i].y / zr + c));
    }
    const float w = static_cast<float>(marker.cols);
    const float h = static_cast<float>(marker.rows);
    const std::array<cv::Point2f, 4> src = {cv::Point2f{0, 0}, {w, 0}, {w, h},
                                            {0, h}};
    const cv::Mat H = cv::getPerspectiveTransform(src.data(), dst.data());
    cv::Mat frame(canvas, canvas, CV_8UC1, cv::Scalar(128));
    cv::warpPerspective(marker, frame, H, frame.size(), cv::INTER_LINEAR,
                        cv::BORDER_TRANSPARENT);
    outCorners = dst;
    return frame;
}

// Out-of-plane robustness: a target tilted away from the camera must still be
// detected and localized. Empirically the ORB pipeline holds up to ~40-45
// degrees; 30 degrees is asserted here as the guaranteed envelope.
static void test_detects_tilted_marker() {
    ImageTracker tracker;
    const cv::Mat marker = makeBlockNoiseMarker(240, 8, 21);
    tracker.addTarget(7, marker);

    std::array<cv::Point2f, 4> expected;
    const cv::Mat frame = makeTiltedFrame(marker, 640, 30.0, expected);
    const std::vector<Detection> detections = tracker.detect(frame);
    CHECK(detections.size() == 1);
    if (detections.empty()) {
        return;
    }
    CHECK(detections[0].imageId == 7);
    for (int i = 0; i < 4; ++i) {
        CHECK(cv::norm(detections[0].corners[i] - expected[i]) < 30.0);
    }
}

static void test_feature_count_reports_trackability() {
    const cv::Mat rich = makeNoiseMarker(160, 11);
    const cv::Mat blank(160, 160, CV_8UC1, cv::Scalar(200));
    CHECK(ImageTracker::countTrackableFeatures(rich) > 100);
    CHECK(ImageTracker::countTrackableFeatures(blank) == 0);
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
    test_detects_marker_under_poor_lighting();
    test_detects_tilted_marker();
    test_downscaled_detection_keeps_fullres_coordinates();
    test_feature_count_reports_trackability();
    test_add_remove_clear_targets();
}
