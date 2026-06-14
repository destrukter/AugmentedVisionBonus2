#include "vision/CameraCapture.h"

#include <opencv2/core/utils/logger.hpp>

namespace avb {

bool CameraCapture::open(int deviceIndex) {
    // On Linux, OpenCV's default backend (GStreamer) can open a UVC webcam and
    // report frames as read successfully while handing back all-black pixels
    // (format-negotiation / MJPEG-decode mismatch). The V4L2 backend reads the
    // same cameras correctly, so prefer it and fall back to the default backend
    // only if V4L2 is unavailable or fails to open the device.
    //
    // Opening also makes the default backend log a harmless one-time "Cannot
    // query video position" warning (a live feed has no seekable duration), so we
    // raise OpenCV's log level to ERROR while opening and restore it afterwards
    // so real errors still surface.
    namespace logging = cv::utils::logging;
    const logging::LogLevel previous = logging::getLogLevel();
    logging::setLogLevel(logging::LOG_LEVEL_ERROR);
    bool opened = capture_.open(deviceIndex, cv::CAP_V4L2);
    if (!opened) {
        opened = capture_.open(deviceIndex, cv::CAP_ANY);
    }
    logging::setLogLevel(previous);
    return opened;
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
