#pragma once

#include <unordered_map>
#include <vector>

#include "vision/ImageTracker.h"

namespace avb {

/// Temporal filter over per-frame detection results.
///
/// Raw per-frame detections are noisy in two ways that make the AR overlay
/// look unstable:
///  1. Pose jitter: the estimated pose wobbles a little every frame even for a
///     perfectly still target.
///  2. Dropouts: a target missed for one or two frames (motion blur, lighting
///     flicker) makes its model blink out and reappear.
///
/// The filter fixes both: poses are exponentially smoothed (lerp for the
/// position, slerp for the rotation) and a target that disappears keeps
/// reporting its last pose for a short hold period before it is dropped.
///
/// Time is passed in explicitly (milliseconds, any monotonically increasing
/// origin) so the filter is deterministic and unit-testable.
class DetectionFilter {
public:
    struct Params {
        /// Blend weight of the newest measurement (0..1]. 1 disables smoothing.
        float smoothing{0.4f};
        /// How long (ms) a lost target keeps its last pose before dropping out.
        double holdMs{250.0};
        /// A position jump larger than this (world units, image plane width is
        /// 1) snaps directly to the new pose instead of gliding toward it.
        float snapDistance{0.5f};
    };

    DetectionFilter() = default;
    explicit DetectionFilter(Params params) : params_(params) {}

    /// Folds this frame's raw detections into the filter state and returns the
    /// smoothed set, including recently-lost targets still within the hold
    /// period. `nowMs` must be monotonically non-decreasing across calls.
    std::vector<Detection> apply(const std::vector<Detection>& raw, double nowMs);

    /// Drops all state (e.g. when the target set changes).
    void reset();

private:
    struct TrackState {
        Detection smoothed;
        double lastSeenMs{0.0};
    };

    Params params_{};
    std::unordered_map<Id, TrackState> tracks_;
};

} // namespace avb
