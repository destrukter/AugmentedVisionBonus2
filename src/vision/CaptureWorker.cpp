#include "vision/CaptureWorker.h"

#include <chrono>
#include <utility>

#include "vision/CameraCapture.h"

namespace avb {

namespace {
// Reopen attempts while no camera is available.
constexpr int kRetryDelayMs = 2000;
// Consecutive read failures after which the device is considered gone (e.g.
// unplugged) and the open/retry cycle starts over.
constexpr int kMaxConsecutiveReadFailures = 30;
} // namespace

CaptureWorker::CaptureWorker(std::shared_ptr<CameraCapture> capture,
                             int deviceIndex)
    : capture_(std::move(capture)), deviceIndex_(deviceIndex) {}

CaptureWorker::~CaptureWorker() {
    stop();
}

void CaptureWorker::start() {
    if (running_.exchange(true)) {
        return;
    }
    thread_ = std::thread(&CaptureWorker::run, this);
}

void CaptureWorker::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    wake_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
}

std::uint64_t CaptureWorker::latestFrame(cv::Mat& out) const {
    std::lock_guard<std::mutex> lock(frameMutex_);
    if (seq_ == 0) {
        return 0;
    }
    latest_.copyTo(out);
    return seq_;
}

std::uint64_t CaptureWorker::waitForFrame(std::uint64_t lastSeen, cv::Mat& out,
                                          int timeoutMs) const {
    std::unique_lock<std::mutex> lock(frameMutex_);
    frameCv_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                      [&] { return seq_ > lastSeen; });
    if (seq_ == 0 || seq_ <= lastSeen) {
        return 0;
    }
    latest_.copyTo(out);
    return seq_;
}

bool CaptureWorker::sleepFor(int ms) {
    std::unique_lock<std::mutex> lock(wakeMutex_);
    wake_.wait_for(lock, std::chrono::milliseconds(ms),
                   [&] { return !running_.load() || reconnect_.load(); });
    return running_.load();
}

void CaptureWorker::run() {
    using clock = std::chrono::steady_clock;

    cv::Mat scratch;
    int consecutiveFailures = 0;
    auto lastFrameTime = clock::now();

    while (running_.load()) {
        if (reconnect_.exchange(false)) {
            capture_->close();
            cameraOpen_.store(false);
        }

        if (!capture_->isOpen()) {
            cameraOpen_.store(capture_->open(deviceIndex_.load()));
            if (!cameraOpen_.load()) {
                if (!sleepFor(kRetryDelayMs)) {
                    break;
                }
                continue;
            }
            consecutiveFailures = 0;
        }

        if (!capture_->grab(scratch)) {
            if (++consecutiveFailures >= kMaxConsecutiveReadFailures) {
                capture_->close();
                cameraOpen_.store(false);
            }
            continue;
        }
        consecutiveFailures = 0;

        const auto now = clock::now();
        const double dt =
            std::chrono::duration<double>(now - lastFrameTime).count();
        lastFrameTime = now;
        if (dt > 0.0) {
            const double instant = 1.0 / dt;
            const double previous = fps_.load();
            fps_.store(previous <= 0.0 ? instant
                                       : previous + 0.1 * (instant - previous));
        }

        {
            std::lock_guard<std::mutex> lock(frameMutex_);
            scratch.copyTo(latest_);
            ++seq_;
        }
        frameCv_.notify_all();
    }
}

} // namespace avb
