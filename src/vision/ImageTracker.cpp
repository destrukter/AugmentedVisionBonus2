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

// Frames are downscaled to at most this many pixels on their longest side
// before feature detection; matching quality barely changes but per-frame cost
// drops several-fold. All reported coordinates/poses stay in full-frame pixels.
constexpr int kMaxDetectDim = 640;

// Templates are likewise capped: a multi-megapixel upload viewed from across a
// room appears far smaller in the frame than its file resolution, which would
// exceed ORB's pyramid scale range and produce zero matches.
constexpr int kMaxTemplateDim = 640;

// If the frame yields fewer features than this, detection is retried once with
// a lower FAST threshold, which recovers corners in dim / low-contrast scenes.
constexpr int kMinFrameFeatures = 300;
constexpr int kFallbackFastThreshold = 7;

// A detected quad smaller than this (in full-frame px^2) is treated as a
// degenerate homography rather than a real sighting.
constexpr double kMinQuadAreaPx = 300.0;

// The registered image is mapped onto a planar quad whose width spans one
// world unit, centred at the origin, lying in the XY plane with +Y up and +Z
// pointing out of the image toward the viewer. Configured model transforms are
// expressed relative to this frame.
constexpr float kPlaneWidth = 1.0f;

// Downscales `src` so its longest side is at most `maxDim`. Returns the scale
// applied (1.0 when no resize was needed).
double capSize(const cv::Mat& src, cv::Mat& dst, int maxDim) {
    const int longest = std::max(src.cols, src.rows);
    if (longest <= maxDim) {
        dst = src;
        return 1.0;
    }
    const double scale = static_cast<double>(maxDim) / longest;
    cv::resize(src, dst, cv::Size(), scale, scale, cv::INTER_AREA);
    return scale;
}

// True when the four projected template corners form a plausible sighting:
// finite, convex and covering a non-trivial area. Homographies fitted to
// borderline match sets can pass RANSAC yet map the template to a degenerate
// (bow-tie / near-line / microscopic) quad; rejecting those here removes most
// of the overlay's "jumping" misdetections.
bool isPlausibleQuad(const std::vector<cv::Point2f>& corners) {
    for (const cv::Point2f& c : corners) {
        if (!std::isfinite(c.x) || !std::isfinite(c.y)) {
            return false;
        }
    }
    if (!cv::isContourConvex(corners)) {
        return false;
    }
    return std::abs(cv::contourArea(corners)) >= kMinQuadAreaPx;
}

} // namespace

/// Per-target feature data (kept out of the header).
struct ImageTracker::Target {
    Id imageId{kInvalidId};
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Size sizePx;  // template size the keypoints are expressed in
};

ImageTracker::ImageTracker()
    : orb_(cv::ORB::create(kMaxFeatures)),
      clahe_(cv::createCLAHE(/*clipLimit=*/3.0, /*tileGridSize=*/{8, 8})) {}

ImageTracker::~ImageTracker() = default;

void ImageTracker::addTarget(Id imageId, const cv::Mat& image) {
    if (image.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);

    // Replace any existing target for this id so re-uploads stay in sync.
    targets_.erase(std::remove_if(targets_.begin(), targets_.end(),
                                  [imageId](const Target& t) {
                                      return t.imageId == imageId;
                                  }),
                   targets_.end());

    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    cv::Mat capped;
    capSize(gray, capped, kMaxTemplateDim);
    // Normalise contrast the same way frames are normalised in detect() so
    // descriptors stay comparable regardless of the upload's exposure.
    cv::Mat equalized;
    clahe_->apply(capped, equalized);

    Target t;
    t.imageId = imageId;
    t.sizePx = equalized.size();
    orb_->detectAndCompute(equalized, cv::noArray(), t.keypoints, t.descriptors);
    if (t.descriptors.empty()) {
        return; // featureless image -> not trackable, drop it
    }
    targets_.push_back(std::move(t));
}

void ImageTracker::removeTarget(Id imageId) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = targets_.begin(); it != targets_.end(); ++it) {
        if (it->imageId == imageId) {
            targets_.erase(it);
            return;
        }
    }
}

void ImageTracker::clearTargets() {
    std::lock_guard<std::mutex> lock(mutex_);
    targets_.clear();
}

std::vector<Detection> ImageTracker::detect(const cv::Mat& frame) {
    std::vector<Detection> detections;
    if (frame.empty()) {
        return detections;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (targets_.empty()) {
        return detections;
    }

    cv::Mat gray;
    if (frame.channels() == 1) {
        gray = frame;
    } else {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }

    // Detect on a downscaled, contrast-normalised copy; keypoint coordinates
    // are mapped back to full-frame pixels for homography/PnP below.
    cv::Mat small;
    const double detectScale = capSize(gray, small, kMaxDetectDim);
    cv::Mat equalized;
    clahe_->apply(small, equalized);

    std::vector<cv::KeyPoint> frameKeypoints;
    cv::Mat frameDescriptors;
    orb_->detectAndCompute(equalized, cv::noArray(), frameKeypoints,
                           frameDescriptors);
    if (static_cast<int>(frameKeypoints.size()) < kMinFrameFeatures) {
        // Dim / low-contrast frame: retry once with a permissive corner
        // threshold before giving up, then restore the default.
        if (auto orb = orb_.dynamicCast<cv::ORB>()) {
            const int previous = orb->getFastThreshold();
            orb->setFastThreshold(kFallbackFastThreshold);
            orb_->detectAndCompute(equalized, cv::noArray(), frameKeypoints,
                                   frameDescriptors);
            orb->setFastThreshold(previous);
        }
    }
    if (frameDescriptors.empty()) {
        return detections;
    }
    const float invScale = static_cast<float>(1.0 / detectScale);

    // Intrinsics: use the configured matrix, else a sensible default derived
    // from the frame size (focal chosen to match the renderer's kDefaultFovYDeg
    // vertical FOV so the overlay aligns with the rendered camera).
    cv::Mat K = cameraMatrix_;
    cv::Mat dist = distCoeffs_;
    if (K.empty()) {
        const double fovY = kDefaultFovYDeg * CV_PI / 180.0;
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
                framePts.push_back(frameKeypoints[pair[0].trainIdx].pt * invScale);
            }
        }
        if (static_cast<int>(templatePts.size()) < kMinGoodMatches) {
            continue;
        }

        // The tolerance is defined in detection pixels; framePts were scaled
        // back to full-frame coordinates, so scale the threshold with them.
        const double ransacErr = kRansacReprojErr / detectScale;
        std::vector<unsigned char> inlierMask;
        const cv::Mat H = cv::findHomography(templatePts, framePts, cv::RANSAC,
                                             ransacErr, inlierMask);
        if (H.empty()) {
            continue;
        }
        const int inliers =
            static_cast<int>(std::count(inlierMask.begin(), inlierMask.end(), 1));
        if (inliers < kMinInliers) {
            continue;
        }

        const float w = static_cast<float>(target.sizePx.width);
        const float h = static_cast<float>(target.sizePx.height);

        // Project the template boundary into the frame; used both as the debug
        // outline and to reject degenerate homographies.
        const std::vector<cv::Point2f> templateCorners = {
            {0.0f, 0.0f}, {w, 0.0f}, {w, h}, {0.0f, h}};
        std::vector<cv::Point2f> frameCorners;
        cv::perspectiveTransform(templateCorners, frameCorners, H);
        if (!isPlausibleQuad(frameCorners)) {
            continue;
        }
        const float planeW = kPlaneWidth;
        const float planeH = w > 0.0f ? kPlaneWidth * (h / w) : kPlaneWidth;

        // Build coplanar object/image correspondences from the inliers and
        // recover the image-plane pose with a planar PnP solver.
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
        // Polish the closed-form IPPE solution; a few LM iterations noticeably
        // reduce frame-to-frame pose jitter for near-planar viewing angles.
        cv::solvePnPRefineLM(objectPts, imagePts, K, dist, rvec, tvec);

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
        for (std::size_t i = 0; i < d.corners.size() && i < frameCorners.size();
             ++i) {
            d.corners[i] = frameCorners[i];
        }
        detections.push_back(d);
    }

    return detections;
}

void ImageTracker::setCameraIntrinsics(const cv::Mat& cameraMatrix,
                                       const cv::Mat& distCoeffs) {
    std::lock_guard<std::mutex> lock(mutex_);
    cameraMatrix_ = cameraMatrix.clone();
    distCoeffs_ = distCoeffs.clone();
}

int ImageTracker::countTrackableFeatures(const cv::Mat& image) {
    if (image.empty()) {
        return 0;
    }
    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    cv::Mat capped;
    capSize(gray, capped, kMaxTemplateDim);
    cv::Mat equalized;
    cv::createCLAHE(3.0, {8, 8})->apply(capped, equalized);

    std::vector<cv::KeyPoint> keypoints;
    cv::ORB::create(kMaxFeatures)->detect(equalized, keypoints);
    return static_cast<int>(keypoints.size());
}

} // namespace avb
