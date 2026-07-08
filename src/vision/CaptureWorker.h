#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include <opencv2/core.hpp>

namespace avb {

class CameraCapture;

/// Grabs camera frames on a dedicated thread and keeps only the newest one.
///
/// This decouples the ~30 fps blocking camera reads from the render loop (the
/// UI never waits on the camera) and guarantees consumers always see the
/// latest frame: the driver's queue can never back up behind a slow consumer,
/// which is what made the feed lag by whole seconds when capture ran inline
/// with tracking and rendering.
///
/// The worker also owns device lifecycle: it opens the camera on start, keeps
/// retrying while no device is available (so plugging a camera in later just
/// works) and reopens it on request.
class CaptureWorker {
public:
    explicit CaptureWorker(std::shared_ptr<CameraCapture> capture,
                           int deviceIndex = 0);
    ~CaptureWorker();

    CaptureWorker(const CaptureWorker&) = delete;
    CaptureWorker& operator=(const CaptureWorker&) = delete;

    void start();
    void stop();

    /// Copies the newest frame into `out` and returns its sequence number
    /// (increases by 1 per captured frame). Returns 0 while no frame has ever
    /// been captured.
    std::uint64_t latestFrame(cv::Mat& out) const;

    /// Blocks until a frame newer than `lastSeen` arrives (or `timeoutMs`
    /// expires), then behaves like latestFrame(). Lets the tracking thread
    /// sleep between frames instead of polling.
    std::uint64_t waitForFrame(std::uint64_t lastSeen, cv::Mat& out,
                               int timeoutMs) const;

    bool cameraOpen() const { return cameraOpen_.load(); }
    /// Measured capture rate (exponential moving average), 0 until measured.
    double fps() const { return fps_.load(); }

    /// Asks the worker to close and reopen the device (e.g. after replugging).
    void requestReconnect() { reconnect_.store(true); wake_.notify_all(); }

    /// Switches to a different capture device; the worker reopens it in the
    /// background (no-op when `index` is already the active device).
    void setDevice(int index) {
        if (deviceIndex_.exchange(index) != index) {
            requestReconnect();
        }
    }
    int device() const { return deviceIndex_.load(); }

private:
    void run();
    /// Interruptible sleep; returns false when stop() was requested.
    bool sleepFor(int ms);

    std::shared_ptr<CameraCapture> capture_;
    std::atomic<int> deviceIndex_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> reconnect_{false};
    std::atomic<bool> cameraOpen_{false};
    std::atomic<double> fps_{0.0};

    mutable std::mutex frameMutex_;             // guards latest_ + seq_
    mutable std::condition_variable frameCv_;   // signalled per new frame
    cv::Mat latest_;
    std::uint64_t seq_{0};

    mutable std::mutex wakeMutex_;              // for interruptible sleeps
    mutable std::condition_variable wake_;
};

} // namespace avb
