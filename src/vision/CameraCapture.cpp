#include "vision/CameraCapture.h"

namespace avb {

bool CameraCapture::open(int deviceIndex) {
    // On Linux, OpenCV's default backend for a camera index is GStreamer, which
    // emits a noisy one-time "Cannot query video position" warning: a live
    // camera feed has no seekable duration to query. Prefer the V4L2 backend,
    // which talks to the device directly and avoids that warning, and fall back
    // to OpenCV's auto-selected backend if V4L2 is unavailable.
#if defined(__linux__)
    if (capture_.open(deviceIndex, cv::CAP_V4L2)) {
        return true;
    }
#endif
    return capture_.open(deviceIndex, cv::CAP_ANY);
}

bool CameraCapture::isOpen() const {
    return capture_.isOpened();
}

void CameraCapture::close() {
    capture_.release();
}

bool CameraCapture::grab(cv::Mat& outFrame) {
    if (!capture_.isOpened()) {
        return false;
    }
    return capture_.read(outFrame) && !outFrame.empty();
}

} // namespace avb
