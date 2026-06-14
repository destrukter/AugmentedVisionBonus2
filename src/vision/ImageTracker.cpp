#include "vision/ImageTracker.h"

// #include <opencv2/features2d.hpp>
// #include <opencv2/calib3d.hpp>
// #include <opencv2/imgproc.hpp>

namespace avb {

/// Per-target feature data (kept out of the header).
struct ImageTracker::Target {
    Id imageId{kInvalidId};
    cv::Mat grayTemplate;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Size sizePx;
};

ImageTracker::ImageTracker() = default;
ImageTracker::~ImageTracker() = default;

void ImageTracker::addTarget(Id imageId, const cv::Mat& /*image*/) {
    // TODO:
    //   Target t; t.imageId = imageId;
    //   cv::cvtColor(image, t.grayTemplate, cv::COLOR_BGR2GRAY);
    //   auto orb = cv::ORB::create();
    //   orb->detectAndCompute(t.grayTemplate, {}, t.keypoints, t.descriptors);
    //   t.sizePx = image.size();
    //   targets_.push_back(std::move(t));
    (void)imageId;
}

void ImageTracker::removeTarget(Id imageId) {
    for (auto it = targets_.begin(); it != targets_.end(); ++it) {
        if (it->imageId == imageId) {
            targets_.erase(it);
            return;
        }
    }
}

void ImageTracker::clearTargets() {
    targets_.clear();
}

std::vector<Detection> ImageTracker::detect(const cv::Mat& /*frame*/) {
    // TODO for each target:
    //   detect+match features against the frame (BFMatcher + ratio test),
    //   findHomography(RANSAC); if enough inliers, recover the plane pose via
    //   solvePnP using cameraMatrix_/distCoeffs_, fill a Detection.
    return {};
}

void ImageTracker::setCameraIntrinsics(const cv::Mat& cameraMatrix,
                                       const cv::Mat& distCoeffs) {
    cameraMatrix_ = cameraMatrix.clone();
    distCoeffs_ = distCoeffs.clone();
}

} // namespace avb
