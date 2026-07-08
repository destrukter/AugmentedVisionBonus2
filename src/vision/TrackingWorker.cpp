#include "vision/TrackingWorker.h"

#include <chrono>
#include <utility>

#include "vision/CaptureWorker.h"

namespace avb {

namespace {
// How long one wait for a fresh frame may block; short enough that stop() and
// filter hold-expiry stay responsive while no camera is delivering frames.
constexpr int kFrameWaitMs = 100;
} // namespace

TrackingWorker::TrackingWorker(std::shared_ptr<CaptureWorker> capture,
                               std::shared_ptr<ImageTracker> tracker,
                               DetectionFilter::Params filterParams)
    : capture_(std::move(capture)),
      tracker_(std::move(tracker)),
      filter_(filterParams) {}

TrackingWorker::~TrackingWorker() {
    stop();
}

void TrackingWorker::start() {
    if (running_.exchange(true)) {
        return;
    }
    thread_ = std::thread(&TrackingWorker::run, this);
}

void TrackingWorker::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (thread_.joinable()) {
        thread_.join();
    }
}

std::vector<Detection> TrackingWorker::latestDetections() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return detections_;
}

void TrackingWorker::run() {
    using clock = std::chrono::steady_clock;
    const auto epoch = clock::now();
    const auto nowMs = [&epoch] {
        return std::chrono::duration<double, std::milli>(clock::now() - epoch)
            .count();
    };

    cv::Mat frame;
    std::uint64_t lastSeq = 0;
    auto lastDetectTime = clock::now();

    while (running_.load()) {
        if (resetRequested_.exchange(false)) {
            filter_.reset();
        }

        const std::uint64_t seq =
            capture_->waitForFrame(lastSeq, frame, kFrameWaitMs);

        std::vector<Detection> raw;
        if (seq != 0) {
            lastSeq = seq;
            raw = tracker_->detect(frame);

            const auto now = clock::now();
            const double dt =
                std::chrono::duration<double>(now - lastDetectTime).count();
            lastDetectTime = now;
            if (dt > 0.0) {
                const double instant = 1.0 / dt;
                const double previous = fps_.load();
                fps_.store(previous <= 0.0
                               ? instant
                               : previous + 0.1 * (instant - previous));
            }
        }
        // Run the filter even without a new frame so held targets expire on
        // time instead of freezing on screen when the camera stalls.
        std::vector<Detection> filtered = filter_.apply(raw, nowMs());

        {
            std::lock_guard<std::mutex> lock(mutex_);
            detections_ = std::move(filtered);
        }
    }
}

} // namespace avb
