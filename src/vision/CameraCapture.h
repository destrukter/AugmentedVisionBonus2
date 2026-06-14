#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace avb {

/// Thin wrapper around an OpenCV camera/video source.
class CameraCapture {
public:
    CameraCapture() = default;

    /// Opens a camera by device index (default 0).
    bool open(int deviceIndex = 0);
    bool isOpen() const;
    void close();

    /// Grabs the latest frame (BGR). Returns false if no frame is available.
    bool grab(cv::Mat& outFrame);

private:
    cv::VideoCapture capture_;
};

} // namespace avb
