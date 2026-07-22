#include "vision/ImageTracker.h"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

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

// Optical-flow tracking. Once a target is acquired by ORB matching, its inlier
// points are carried frame-to-frame with pyramidal Lucas-Kanade flow; when the
// surviving set shrinks below kMinFlowPoints the target falls back to ORB
// re-acquisition.
constexpr int kMinFlowPoints = 12;
// A flowed point whose backward-flowed position lands farther than this from
// its origin (detection-scale px) is inconsistent and dropped.
constexpr float kFlowFbMaxErrPx = 1.5f;
constexpr int kFlowWinSize = 21;
constexpr int kFlowPyramidLevels = 3;

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

/// Estimates the camera-space pose of a planar template from template<->frame
/// point correspondences (frame points in full-frame pixels): homography
/// RANSAC, quad plausibility check, then planar PnP with LM refinement. On
/// success fills `out` (id, pose, corners) and `inlierMask` (parallel to the
/// input correspondences) and returns the inlier count; returns 0 when the
/// correspondences are rejected.
int estimatePlanarPose(const std::vector<cv::Point2f>& templatePts,
                       const std::vector<cv::Point2f>& framePts,
                       cv::Size templateSize, Id imageId, double ransacErr,
                       const cv::Mat& K, const cv::Mat& dist, Detection& out,
                       std::vector<unsigned char>& inlierMask) {
    if (static_cast<int>(templatePts.size()) < kMinGoodMatches) {
        return 0;
    }
    const cv::Mat H =
        cv::findHomography(templatePts, framePts, cv::RANSAC, ransacErr, inlierMask);
    if (H.empty()) {
        return 0;
    }
    const int inliers =
        static_cast<int>(std::count(inlierMask.begin(), inlierMask.end(), 1));
    if (inliers < kMinInliers) {
        return 0;
    }

    const float w = static_cast<float>(templateSize.width);
    const float h = static_cast<float>(templateSize.height);

    // Project the template boundary into the frame; used both as the debug
    // outline and to reject degenerate homographies.
    const std::vector<cv::Point2f> templateCorners = {
        {0.0f, 0.0f}, {w, 0.0f}, {w, h}, {0.0f, h}};
    std::vector<cv::Point2f> frameCorners;
    cv::perspectiveTransform(templateCorners, frameCorners, H);
    if (!isPlausibleQuad(frameCorners)) {
        return 0;
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
        return 0;
    }
    // Polish the closed-form IPPE solution; a few LM iterations noticeably
    // reduce frame-to-frame pose jitter for near-planar viewing angles.
    cv::solvePnPRefineLM(objectPts, imagePts, K, dist, rvec, tvec);

    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv); // object -> OpenCV camera (X right, Y down, Z fwd)

    // Convert from the OpenCV camera frame (Y down, Z into the scene) to the
    // OGRE/OpenGL camera frame (Y up, Z out of the scene) used by the
    // renderer: negate the Y and Z axes.
    out.imageId = imageId;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    for (int r = 0; r < 3; ++r) {
        const float s = (r == 0) ? 1.0f : -1.0f; // flip rows 1 (Y) and 2 (Z)
        for (int c = 0; c < 3; ++c) {
            pose(r, c) = s * static_cast<float>(Rcv.at<double>(r, c));
        }
        pose(r, 3) = s * static_cast<float>(tvec.at<double>(r));
    }
    out.poseInCamera = pose;
    for (std::size_t i = 0; i < out.corners.size() && i < frameCorners.size();
         ++i) {
        out.corners[i] = frameCorners[i];
    }
    return inliers;
}

} // namespace

/// Per-target feature data (kept out of the header).
struct ImageTracker::Target {
    Id imageId{kInvalidId};
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Size sizePx;  // template size the keypoints are expressed in

    // Optical-flow tracking state; meaningful while flowTracked is true.
    // flowFramePts are in detection-scale frame pixels (the resolution flow
    // runs at); flowTemplatePts are template pixels, index-parallel.
    bool flowTracked{false};
    std::vector<cv::Point2f> flowTemplatePts;
    std::vector<cv::Point2f> flowFramePts;
    int flowInitialPts{0};
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

bool ImageTracker::advanceFlow(const cv::Mat& prevGray, const cv::Mat& gray,
                               Target& target) {
    if (static_cast<int>(target.flowFramePts.size()) < kMinFlowPoints) {
        return false;
    }
    const cv::Size win(kFlowWinSize, kFlowWinSize);
    const cv::TermCriteria term(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                30, 0.01);

    std::vector<cv::Point2f> next;
    std::vector<unsigned char> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prevGray, gray, target.flowFramePts, next, status,
                             err, win, kFlowPyramidLevels, term);

    // Forward-backward consistency: flow the results back to the previous
    // frame and keep only points that land where they started. This discards
    // points that latched onto occluders, specular highlights or the
    // background, which otherwise silently corrupt the pose.
    std::vector<cv::Point2f> back;
    std::vector<unsigned char> backStatus;
    cv::calcOpticalFlowPyrLK(gray, prevGray, next, back, backStatus, err, win,
                             kFlowPyramidLevels, term);

    std::vector<cv::Point2f> keptTemplate;
    std::vector<cv::Point2f> keptFrame;
    keptTemplate.reserve(next.size());
    keptFrame.reserve(next.size());
    for (std::size_t i = 0; i < next.size(); ++i) {
        if (!status[i] || !backStatus[i]) {
            continue;
        }
        if (cv::norm(back[i] - target.flowFramePts[i]) > kFlowFbMaxErrPx) {
            continue;
        }
        // Points flowed out of the frame can't be tracked further.
        if (next[i].x < 0.0f || next[i].y < 0.0f ||
            next[i].x >= static_cast<float>(gray.cols) ||
            next[i].y >= static_cast<float>(gray.rows)) {
            continue;
        }
        keptTemplate.push_back(target.flowTemplatePts[i]);
        keptFrame.push_back(next[i]);
    }
    target.flowTemplatePts.swap(keptTemplate);
    target.flowFramePts.swap(keptFrame);
    return static_cast<int>(target.flowFramePts.size()) >= kMinFlowPoints;
}

std::vector<Detection> ImageTracker::detect(const cv::Mat& frame) {
    std::vector<Detection> detections;
    if (frame.empty()) {
        return detections;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (targets_.empty()) {
        prevGray_.release(); // no flow state to continue from
        return detections;
    }

    cv::Mat gray;
    if (frame.channels() == 1) {
        gray = frame;
    } else {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }

    // Both acquisition and flow run on a downscaled copy; all coordinates are
    // mapped back to full-frame pixels for homography/PnP below.
    cv::Mat small;
    const double detectScale = capSize(gray, small, kMaxDetectDim);
    const float invScale = static_cast<float>(1.0 / detectScale);
    // The tolerance is defined in detection pixels; frame points are scaled
    // back to full-frame coordinates, so scale the threshold with them.
    const double ransacErr = kRansacReprojErr / detectScale;

    // A changed frame size (camera swap / mode change) invalidates flow state:
    // the stored points are in the old detection scale.
    if (!prevGray_.empty() && prevGray_.size() != small.size()) {
        for (Target& t : targets_) {
            t.flowTracked = false;
        }
        prevGray_.release();
    }

    // Phase 1: carry tracked targets forward with Lucas-Kanade optical flow.
    // Targets that lose too many points fall back to ORB acquisition below.
    for (Target& target : targets_) {
        if (target.flowTracked &&
            (prevGray_.empty() || !advanceFlow(prevGray_, small, target))) {
            target.flowTracked = false;
        }
    }

    // Phase 2: run ORB only when some target needs (re)acquisition. On frames
    // where every target is flow-tracked this skips the most expensive part
    // of the pipeline entirely.
    std::vector<cv::KeyPoint> frameKeypoints;
    cv::Mat frameDescriptors;
    const bool needAcquisition =
        std::any_of(targets_.begin(), targets_.end(),
                    [](const Target& t) { return !t.flowTracked; });
    if (needAcquisition) {
        cv::Mat equalized;
        clahe_->apply(small, equalized);
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
    }

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

    // Phase 3: estimate each target's pose - from its flowed points when
    // tracked, else by matching against the frame's ORB features.
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    for (Target& target : targets_) {
        if (target.flowTracked) {
            std::vector<cv::Point2f> fullFramePts;
            fullFramePts.reserve(target.flowFramePts.size());
            for (const cv::Point2f& p : target.flowFramePts) {
                fullFramePts.push_back(p * invScale);
            }
            Detection d;
            std::vector<unsigned char> inlierMask;
            const int inliers = estimatePlanarPose(
                target.flowTemplatePts, fullFramePts, target.sizePx,
                target.imageId, ransacErr, K, dist, d, inlierMask);
            if (inliers > 0) {
                // Drop RANSAC outliers so drifted flow points don't accumulate
                // and slowly bend the pose.
                std::vector<cv::Point2f> keptTemplate;
                std::vector<cv::Point2f> keptFrame;
                keptTemplate.reserve(inliers);
                keptFrame.reserve(inliers);
                for (std::size_t i = 0; i < inlierMask.size(); ++i) {
                    if (inlierMask[i]) {
                        keptTemplate.push_back(target.flowTemplatePts[i]);
                        keptFrame.push_back(target.flowFramePts[i]);
                    }
                }
                target.flowTemplatePts.swap(keptTemplate);
                target.flowFramePts.swap(keptFrame);
                if (static_cast<int>(target.flowFramePts.size()) <
                    kMinFlowPoints) {
                    target.flowTracked = false; // re-acquire next frame
                }
                d.viaOpticalFlow = true;
                d.confidence =
                    static_cast<float>(target.flowFramePts.size()) /
                    static_cast<float>(std::max(target.flowInitialPts, 1));
                detections.push_back(d);
                continue;
            }
            // Implausible pose from flowed points: treat the target as lost.
            // ORB may not have run this frame; the next frame re-acquires it.
            target.flowTracked = false;
        }

        if (frameDescriptors.empty()) {
            continue; // nothing to match against (or ORB was skipped)
        }
        std::vector<std::vector<cv::DMatch>> knn;
        matcher.knnMatch(target.descriptors, frameDescriptors, knn, 2);

        std::vector<cv::Point2f> templatePts;
        std::vector<cv::Point2f> framePtsSmall; // detection-scale, seeds flow
        std::vector<cv::Point2f> framePts;      // full-frame, for the pose
        for (const auto& pair : knn) {
            if (pair.size() < 2) {
                continue;
            }
            if (pair[0].distance < kLoweRatio * pair[1].distance) {
                templatePts.push_back(target.keypoints[pair[0].queryIdx].pt);
                const cv::Point2f& fp = frameKeypoints[pair[0].trainIdx].pt;
                framePtsSmall.push_back(fp);
                framePts.push_back(fp * invScale);
            }
        }

        Detection d;
        std::vector<unsigned char> inlierMask;
        const int inliers =
            estimatePlanarPose(templatePts, framePts, target.sizePx,
                               target.imageId, ransacErr, K, dist, d, inlierMask);
        if (inliers == 0) {
            continue;
        }
        d.viaOpticalFlow = false;
        d.confidence =
            static_cast<float>(inliers) / static_cast<float>(templatePts.size());

        // Seed the optical-flow state from the inlier correspondences; from
        // the next frame on this target is tracked, not re-detected.
        target.flowTemplatePts.clear();
        target.flowFramePts.clear();
        target.flowTemplatePts.reserve(inliers);
        target.flowFramePts.reserve(inliers);
        for (std::size_t i = 0; i < inlierMask.size(); ++i) {
            if (inlierMask[i]) {
                target.flowTemplatePts.push_back(templatePts[i]);
                target.flowFramePts.push_back(framePtsSmall[i]);
            }
        }
        target.flowInitialPts = inliers;
        target.flowTracked =
            static_cast<int>(target.flowFramePts.size()) >= kMinFlowPoints;

        detections.push_back(d);
    }

    // Keep this frame for the next call's flow step. copyTo reuses the
    // existing buffer, so this doesn't allocate every frame.
    small.copyTo(prevGray_);
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
