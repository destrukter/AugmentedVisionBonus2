#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "vision/DetectionFilter.h"
#include "vision/ImageTracker.h"

namespace avb {

class CaptureWorker;

/// Runs image detection on its own thread so the render loop never waits for
/// feature matching (the most expensive step of the frame pipeline).
///
/// Each iteration takes the newest captured frame (skipping any it was too
/// slow for), runs ImageTracker::detect, feeds the result through a
/// DetectionFilter (pose smoothing + short hold on dropouts) and publishes the
/// filtered detections. The UI thread pairs the newest camera frame with the
/// newest published detections; poses may trail the displayed frame by one
/// detection interval, which the smoothing makes visually unnoticeable.
class TrackingWorker {
public:
    TrackingWorker(std::shared_ptr<CaptureWorker> capture,
                   std::shared_ptr<ImageTracker> tracker,
                   DetectionFilter::Params filterParams = {});
    ~TrackingWorker();

    TrackingWorker(const TrackingWorker&) = delete;
    TrackingWorker& operator=(const TrackingWorker&) = delete;

    void start();
    void stop();

    /// The most recent smoothed detections.
    std::vector<Detection> latestDetections() const;

    /// Measured detection rate (exponential moving average), 0 until measured.
    double fps() const { return fps_.load(); }

    /// Drops the filter's temporal state; call when the target set changes.
    void resetFilter() { resetRequested_.store(true); }

private:
    void run();

    std::shared_ptr<CaptureWorker> capture_;
    std::shared_ptr<ImageTracker> tracker_;
    DetectionFilter filter_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> resetRequested_{false};
    std::atomic<double> fps_{0.0};

    mutable std::mutex mutex_;  // guards detections_
    std::vector<Detection> detections_;
};

} // namespace avb
