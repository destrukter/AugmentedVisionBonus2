#pragma once

#include <array>
#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "storage/Types.h"

namespace cv {
class Feature2D;  // full definition in <opencv2/features2d.hpp> (cpp only)
class CLAHE;      // full definition in <opencv2/imgproc.hpp> (cpp only)
} // namespace cv

namespace avb {

/// Result of detecting one registered image in a camera frame.
struct Detection {
    Id imageId{kInvalidId};
    /// 4x4 pose of the detected image plane in camera space. Combined with each
    /// assignment's Transform to place its model.
    Eigen::Matrix4f poseInCamera{Eigen::Matrix4f::Identity()};
    float confidence{0.0f};
    /// The template's boundary projected into frame pixel coordinates (the four
    /// corners, clockwise from the top-left). Useful for drawing a debug outline
    /// around the tracked target.
    std::array<cv::Point2f, 4> corners{};
};

/// Detects which uploaded images are visible in the camera feed and estimates
/// their pose, using ORB feature matching + homography + planar PnP.
///
/// Robustness measures:
///  * CLAHE contrast normalisation on both templates and frames, so matching
///    keeps working under dim / uneven / changing lighting.
///  * If a frame yields few features (low light, low contrast), detection is
///    retried once with a more permissive FAST threshold.
///  * Frames are downscaled for feature detection (poses stay in full-frame
///    coordinates), cutting per-frame cost several-fold.
///  * The homography is sanity-checked (convex, plausibly sized quad) before a
///    pose is accepted, rejecting degenerate fits that made the overlay jump.
///
/// Thread safety: all public methods are internally synchronized, so targets
/// can be added/removed from the UI thread while a background thread runs
/// detect().
class ImageTracker {
public:
    /// Vertical field of view (degrees) assumed when no calibrated intrinsics
    /// are provided. The renderer must use the same value for its virtual
    /// camera or rendered models drift off the tracked image.
    static constexpr float kDefaultFovYDeg = 45.0f;

    ImageTracker();
    ~ImageTracker();

    /// Registers an image template under its store Id. Precomputes features.
    void addTarget(Id imageId, const cv::Mat& image);
    void removeTarget(Id imageId);
    void clearTargets();

    /// Detects all registered targets present in `frame`.
    std::vector<Detection> detect(const cv::Mat& frame);

    /// Sets the camera intrinsics used for pose estimation.
    void setCameraIntrinsics(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

    /// Counts the trackable (ORB) features in an image, after the same
    /// preprocessing detect() uses. Intended for upload-time feedback: images
    /// below roughly 50 features are unlikely to ever be detected.
    static int countTrackableFeatures(const cv::Mat& image);

private:
    struct Target;                  // feature data per registered image
    std::vector<Target> targets_;   // pimpl-style to keep the header light
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    cv::Ptr<cv::Feature2D> orb_;    // shared detector (guarded by mutex_)
    cv::Ptr<cv::CLAHE> clahe_;      // contrast normalisation (guarded by mutex_)
    mutable std::mutex mutex_;
};

} // namespace avb
