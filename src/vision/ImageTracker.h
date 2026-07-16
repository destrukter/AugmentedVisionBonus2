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
    /// True when the pose came from frame-to-frame optical flow, false when it
    /// came from a fresh ORB feature-match acquisition.
    bool viaOpticalFlow{false};
    /// The template's boundary projected into frame pixel coordinates (the four
    /// corners, clockwise from the top-left). Useful for drawing a debug outline
    /// around the tracked target.
    std::array<cv::Point2f, 4> corners{};
};

/// Detects which uploaded images are visible in the camera feed and estimates
/// their pose. All registered targets are searched independently, so any
/// number of different images can be tracked simultaneously in one frame.
///
/// Two-phase detect-then-track design:
///  * Acquisition: ORB feature matching + homography + planar PnP finds a
///    target that is not currently tracked.
///  * Tracking: once acquired, the matched feature points are carried from
///    frame to frame with pyramidal Lucas-Kanade optical flow (with a
///    forward-backward consistency check), and the pose is re-estimated from
///    the flowed correspondences. Optical flow is far more stable than
///    per-frame matching - no jitter from re-matched features - and keeps
///    tracking through steeper viewing angles and greater distances than
///    descriptor matching survives.
///  * Recovery: when too many flow points are lost (occlusion, motion blur,
///    target leaving the frame) or the tracked pose turns implausible, the
///    target drops back to ORB acquisition automatically.
///
/// Robustness measures:
///  * CLAHE contrast normalisation on both templates and frames, so matching
///    keeps working under dim / uneven / changing lighting.
///  * If a frame yields few features (low light, low contrast), detection is
///    retried once with a more permissive FAST threshold.
///  * Frames are downscaled for feature detection (poses stay in full-frame
///    coordinates), cutting per-frame cost several-fold. ORB is skipped
///    entirely on frames where every target is being tracked by optical flow.
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
    struct Target;                  // feature + flow-tracking data per image

    /// Advances `target`'s flow points from `prevGray` to `gray` (both at
    /// detection scale) with pyramidal LK + a forward-backward consistency
    /// check, pruning lost points. Returns false when too few points survive
    /// to keep tracking (the caller then falls back to ORB acquisition).
    bool advanceFlow(const cv::Mat& prevGray, const cv::Mat& gray, Target& target);

    std::vector<Target> targets_;   // pimpl-style to keep the header light
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    cv::Ptr<cv::Feature2D> orb_;    // shared detector (guarded by mutex_)
    cv::Ptr<cv::CLAHE> clahe_;      // contrast normalisation (guarded by mutex_)
    cv::Mat prevGray_;              // previous detection-scale frame, for optical flow
    mutable std::mutex mutex_;
};

} // namespace avb
