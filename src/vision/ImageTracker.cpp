#include "vision/ImageTracker.h"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

namespace avb {

namespace {

// Matching/geometry thresholds.
constexpr int kMaxFeatures = 1000;
constexpr float kLoweRatio = 0.75f;
constexpr size_t kMinGoodMatches = 12;
constexpr int kMinInliers = 10;
constexpr double kRansacReprojThresh = 3.0;

// Physical width assigned to a marker image; the marker plane is centered at the
// origin and spans [-0.5, 0.5] * kMarkerWidth in X (Y scaled by aspect). Pose
// translations and the configured Transform are therefore in marker-width units.
constexpr double kMarkerWidth = 1.0;

// Builds a reasonable pinhole intrinsic from the frame size when no calibration
// has been supplied (focal ~ image width, principal point at the centre).
cv::Mat defaultCameraMatrix(const cv::Size& size) {
    const double f = static_cast<double>(size.width);
    cv::Mat k = cv::Mat::eye(3, 3, CV_64F);
    k.at<double>(0, 0) = f;
    k.at<double>(1, 1) = f;
    k.at<double>(0, 2) = size.width * 0.5;
    k.at<double>(1, 2) = size.height * 0.5;
    return k;
}

// Maps a target-image pixel to its 3D coordinate on the marker plane (z = 0).
cv::Point3f planePoint(const cv::Point2f& px, const cv::Size& imgSize) {
    const double aspect =
        static_cast<double>(imgSize.height) / std::max(1, imgSize.width);
    const double x = (px.x / imgSize.width - 0.5) * kMarkerWidth;
    const double y = (0.5 - px.y / imgSize.height) * kMarkerWidth * aspect;
    return cv::Point3f(static_cast<float>(x), static_cast<float>(y), 0.0f);
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
    Target t;
    t.imageId = imageId;
    t.sizePx = image.size();
    if (image.channels() == 1) {
        t.grayTemplate = image.clone();
    } else {
        cv::cvtColor(image, t.grayTemplate, cv::COLOR_BGR2GRAY);
    }
    cv::Ptr<cv::ORB> orb = cv::ORB::create(kMaxFeatures);
    orb->detectAndCompute(t.grayTemplate, cv::noArray(), t.keypoints, t.descriptors);
    if (t.descriptors.empty()) {
        return; // featureless image; nothing to track
    }
    removeTarget(imageId); // replace any previous template for this id
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

    cv::Ptr<cv::ORB> orb = cv::ORB::create(kMaxFeatures);
    std::vector<cv::KeyPoint> frameKeypoints;
    cv::Mat frameDesc;
    orb->detectAndCompute(gray, cv::noArray(), frameKeypoints, frameDesc);
    if (frameDesc.empty()) {
        return detections;
    }

    const cv::Mat camMatrix =
        cameraMatrix_.empty() ? defaultCameraMatrix(frame.size()) : cameraMatrix_;
    const cv::Mat distCoeffs =
        distCoeffs_.empty() ? cv::Mat::zeros(1, 5, CV_64F) : distCoeffs_;

    cv::BFMatcher matcher(cv::NORM_HAMMING);

    for (const Target& target : targets_) {
        std::vector<std::vector<cv::DMatch>> knn;
        matcher.knnMatch(target.descriptors, frameDesc, knn, 2);

        // Lowe ratio test.
        std::vector<cv::Point2f> srcPts; // target image pixels
        std::vector<cv::Point2f> dstPts; // frame pixels
        std::vector<int> targetIdx;
        for (const auto& m : knn) {
            if (m.size() == 2 && m[0].distance < kLoweRatio * m[1].distance) {
                srcPts.push_back(target.keypoints[m[0].queryIdx].pt);
                dstPts.push_back(frameKeypoints[m[0].trainIdx].pt);
                targetIdx.push_back(m[0].queryIdx);
            }
        }
        if (srcPts.size() < kMinGoodMatches) {
            continue;
        }

        cv::Mat inlierMask;
        const cv::Mat H = cv::findHomography(srcPts, dstPts, cv::RANSAC,
                                             kRansacReprojThresh, inlierMask);
        if (H.empty()) {
            continue;
        }

        // Gather inliers as 3D (marker plane) <-> 2D (frame) correspondences.
        std::vector<cv::Point3f> objectPts;
        std::vector<cv::Point2f> imagePts;
        for (int i = 0; i < inlierMask.rows; ++i) {
            if (inlierMask.at<unsigned char>(i)) {
                objectPts.push_back(
                    planePoint(target.keypoints[targetIdx[i]].pt, target.sizePx));
                imagePts.push_back(dstPts[i]);
            }
        }
        if (static_cast<int>(objectPts.size()) < kMinInliers) {
            continue;
        }

        cv::Mat rvec, tvec;
        if (!cv::solvePnP(objectPts, imagePts, camMatrix, distCoeffs, rvec, tvec,
                          false, cv::SOLVEPNP_ITERATIVE)) {
            continue;
        }

        cv::Mat R;
        cv::Rodrigues(rvec, R);

        // OpenCV camera: +X right, +Y down, +Z forward.
        // OGRE camera:   +X right, +Y up,   +Z backward.
        // Convert by negating the Y and Z rows of [R|t].
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        for (int r = 0; r < 3; ++r) {
            const float sign = (r == 0) ? 1.0f : -1.0f;
            for (int c = 0; c < 3; ++c) {
                pose(r, c) = sign * static_cast<float>(R.at<double>(r, c));
            }
            pose(r, 3) = sign * static_cast<float>(tvec.at<double>(r));
        }

        Detection d;
        d.imageId = target.imageId;
        d.poseInCamera = pose;
        d.confidence = static_cast<float>(objectPts.size()) /
                       static_cast<float>(srcPts.size());
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
