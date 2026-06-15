#pragma once

#include <array>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "storage/Types.h"

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
/// their pose, using OpenCV feature matching (e.g. ORB + homography + PnP).
class ImageTracker {
public:
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

private:
    struct Target;                  // feature data per registered image
    std::vector<Target> targets_;   // pimpl-style to keep the header light
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
};

} // namespace avb
