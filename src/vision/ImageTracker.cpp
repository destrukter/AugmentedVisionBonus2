#include "vision/ImageTracker.h"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

namespace avb {

namespace {

// Feature/matching tuning. These are deliberately conservative so the tracker
// favours precision (no false placements) over recall.
constexpr int kMaxFeatures = 1000;
constexpr float kLoweRatio = 0.75f;      // Lowe ratio test threshold
constexpr int kMinGoodMatches = 12;      // before homography
constexpr int kMinInliers = 10;          // homography RANSAC inliers
constexpr double kRansacReprojErr = 3.0; // px

// The registered image is mapped onto a planar quad whose width spans one
// world unit, centred at the origin, lying in the XY plane with +Y up and +Z
// pointing out of the image toward the viewer. Configured model transforms are
// expressed relative to this frame.
constexpr float kPlaneWidth = 1.0f;

cv::Ptr<cv::ORB>& orb() {
    static cv::Ptr<cv::ORB> instance = cv::ORB::create(kMaxFeatures);
    return instance;
}

} // namespace

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

void ImageTracker::addTarget(Id imageId, const cv::Mat& image) {
    if (image.empty()) {
        return;
    }
    // Replace any existing target for this id so re-uploads stay in sync.
    removeTarget(imageId);

    Target t;
    t.imageId = imageId;
    if (image.channels() == 1) {
        t.grayTemplate = image.clone();
    } else {
        cv::cvtColor(image, t.grayTemplate, cv::COLOR_BGR2GRAY);
    }
    t.sizePx = t.grayTemplate.size();
    orb()->detectAndCompute(t.grayTemplate, cv::noArray(), t.keypoints,
                            t.descriptors);
    if (t.descriptors.empty()) {
        return; // featureless image -> not trackable, drop it
    }
    targets_.push_back(std::move(t));
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

std::vector<Detection> ImageTracker::detect(const cv::Mat& frame) {
    std::vector<Detection> detections;
    if (frame.empty() || targets_.empty()) {
        return detections;
    }

    cv::Mat gray;
    if (frame.channels() == 1) {
        gray = frame;
    } else {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }

    std::vector<cv::KeyPoint> frameKeypoints;
    cv::Mat frameDescriptors;
    orb()->detectAndCompute(gray, cv::noArray(), frameKeypoints, frameDescriptors);
    if (frameDescriptors.empty()) {
        return detections;
    }

    // Intrinsics: use the configured matrix, else a sensible default derived
    // from the frame size (focal chosen to match OGRE's default 45deg vertical
    // FOV so the overlay roughly aligns with the rendered camera).
    cv::Mat K = cameraMatrix_;
    cv::Mat dist = distCoeffs_;
    if (K.empty()) {
        const double fovY = 45.0 * CV_PI / 180.0;
        const double f = (frame.rows * 0.5) / std::tan(fovY * 0.5);
        K = (cv::Mat_<double>(3, 3) << f, 0, frame.cols * 0.5, 0, f,
             frame.rows * 0.5, 0, 0, 1);
    }
    if (dist.empty()) {
        dist = cv::Mat::zeros(1, 5, CV_64F);
    }

    cv::BFMatcher matcher(cv::NORM_HAMMING);

    for (const Target& target : targets_) {
        std::vector<std::vector<cv::DMatch>> knn;
        matcher.knnMatch(target.descriptors, frameDescriptors, knn, 2);

        std::vector<cv::Point2f> templatePts;
        std::vector<cv::Point2f> framePts;
        for (const auto& pair : knn) {
            if (pair.size() < 2) {
                continue;
            }
            if (pair[0].distance < kLoweRatio * pair[1].distance) {
                templatePts.push_back(target.keypoints[pair[0].queryIdx].pt);
                framePts.push_back(frameKeypoints[pair[0].trainIdx].pt);
            }
        }
        if (static_cast<int>(templatePts.size()) < kMinGoodMatches) {
            continue;
        }

        std::vector<unsigned char> inlierMask;
        const cv::Mat H = cv::findHomography(templatePts, framePts, cv::RANSAC,
                                             kRansacReprojErr, inlierMask);
        if (H.empty()) {
            continue;
        }
        const int inliers =
            static_cast<int>(std::count(inlierMask.begin(), inlierMask.end(), 1));
        if (inliers < kMinInliers) {
            continue;
        }

        // Build coplanar object/image correspondences from the inliers and
        // recover the image-plane pose with a planar PnP solver.
        const float w = static_cast<float>(target.sizePx.width);
        const float h = static_cast<float>(target.sizePx.height);
        const float planeW = kPlaneWidth;
        const float planeH = w > 0.0f ? kPlaneWidth * (h / w) : kPlaneWidth;

        std::vector<cv::Point3f> objectPts;
        std::vector<cv::Point2f> imagePts;
        objectPts.reserve(inliers);
        imagePts.reserve(inliers);
        for (std::size_t i = 0; i < inlierMask.size(); ++i) {
            if (!inlierMask[i]) {
                continue;
            }
            const cv::Point2f& tp = templatePts[i];
            const float u = w > 0.0f ? tp.x / w : 0.0f;     // 0..1 across width
            const float v = h > 0.0f ? tp.y / h : 0.0f;     // 0..1 down height
            objectPts.emplace_back((u - 0.5f) * planeW,     // +X right
                                   (0.5f - v) * planeH,     // +Y up
                                   0.0f);                   // image plane
            imagePts.push_back(framePts[i]);
        }

        cv::Mat rvec;
        cv::Mat tvec;
        const bool ok = cv::solvePnP(objectPts, imagePts, K, dist, rvec, tvec,
                                     false, cv::SOLVEPNP_IPPE);
        if (!ok) {
            continue;
        }

        cv::Mat Rcv;
        cv::Rodrigues(rvec, Rcv); // object -> OpenCV camera (X right, Y down, Z fwd)

        // Convert from the OpenCV camera frame (Y down, Z into the scene) to the
        // OGRE/OpenGL camera frame (Y up, Z out of the scene) used by the
        // renderer: negate the Y and Z axes.
        Detection d;
        d.imageId = target.imageId;
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        for (int r = 0; r < 3; ++r) {
            const float s = (r == 0) ? 1.0f : -1.0f; // flip rows 1 (Y) and 2 (Z)
            for (int c = 0; c < 3; ++c) {
                pose(r, c) = s * static_cast<float>(Rcv.at<double>(r, c));
            }
            pose(r, 3) = s * static_cast<float>(tvec.at<double>(r));
        }
        d.poseInCamera = pose;
        d.confidence =
            static_cast<float>(inliers) / static_cast<float>(templatePts.size());
        detections.push_back(d);
    }

    return detections;
}

void ImageTracker::setCameraIntrinsics(const cv::Mat& cameraMatrix,
                                       const cv::Mat& distCoeffs) {
    cameraMatrix_ = cameraMatrix.clone();
    distCoeffs_ = distCoeffs.clone();
}

} // namespace avb
