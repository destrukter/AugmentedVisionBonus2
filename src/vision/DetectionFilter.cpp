#include "vision/DetectionFilter.h"

#include <algorithm>

#include <Eigen/Geometry>

namespace avb {

namespace {

// Blends `next` into `prev` with weight `alpha` (1 = take next entirely).
// Positions/corners lerp; the rotation goes through a quaternion slerp so the
// blend stays a valid rotation.
Detection blend(const Detection& prev, const Detection& next, float alpha) {
    Detection out = next;

    const Eigen::Vector3f p0 = prev.poseInCamera.block<3, 1>(0, 3);
    const Eigen::Vector3f p1 = next.poseInCamera.block<3, 1>(0, 3);
    const Eigen::Vector3f p = p0 + alpha * (p1 - p0);

    const Eigen::Quaternionf q0(prev.poseInCamera.block<3, 3>(0, 0));
    const Eigen::Quaternionf q1(next.poseInCamera.block<3, 3>(0, 0));
    const Eigen::Quaternionf q = q0.slerp(alpha, q1).normalized();

    out.poseInCamera = Eigen::Matrix4f::Identity();
    out.poseInCamera.block<3, 3>(0, 0) = q.toRotationMatrix();
    out.poseInCamera.block<3, 1>(0, 3) = p;

    for (std::size_t i = 0; i < out.corners.size(); ++i) {
        out.corners[i] = prev.corners[i] + alpha * (next.corners[i] - prev.corners[i]);
    }
    out.confidence = prev.confidence + alpha * (next.confidence - prev.confidence);
    return out;
}

} // namespace

std::vector<Detection> DetectionFilter::apply(const std::vector<Detection>& raw,
                                              double nowMs) {
    const float alpha = std::clamp(params_.smoothing, 0.01f, 1.0f);

    for (const Detection& d : raw) {
        auto it = tracks_.find(d.imageId);
        if (it == tracks_.end()) {
            tracks_.emplace(d.imageId, TrackState{d, nowMs});
            continue;
        }
        TrackState& track = it->second;
        const Eigen::Vector3f prevPos =
            track.smoothed.poseInCamera.block<3, 1>(0, 3);
        const Eigen::Vector3f newPos = d.poseInCamera.block<3, 1>(0, 3);
        if ((newPos - prevPos).norm() > params_.snapDistance) {
            track.smoothed = d; // real relocation, don't glide across the scene
        } else {
            track.smoothed = blend(track.smoothed, d, alpha);
        }
        track.lastSeenMs = nowMs;
    }

    // Emit all live tracks; expire the ones missing longer than the hold.
    std::vector<Detection> out;
    out.reserve(tracks_.size());
    for (auto it = tracks_.begin(); it != tracks_.end();) {
        if (nowMs - it->second.lastSeenMs > params_.holdMs) {
            it = tracks_.erase(it);
        } else {
            out.push_back(it->second.smoothed);
            ++it;
        }
    }
    return out;
}

void DetectionFilter::reset() {
    tracks_.clear();
}

} // namespace avb
