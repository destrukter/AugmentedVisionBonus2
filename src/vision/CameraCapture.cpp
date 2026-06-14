#include "vision/CameraCapture.h"

#include <opencv2/core/utils/logger.hpp>

namespace avb {

bool CameraCapture::open(int deviceIndex) {
    // Opening a live camera makes OpenCV's default backend (GStreamer on Linux)
    // log a harmless one-time "Cannot query video position" warning, because a
    // live feed has no seekable duration. We keep that default backend -- it is
    // the one that actually delivers frames reliably across setups -- and only
    // silence the noise by raising OpenCV's log level to ERROR while opening,
    // restoring the previous level afterwards so real errors still surface.
    namespace logging = cv::utils::logging;
    const logging::LogLevel previous = logging::getLogLevel();
    logging::setLogLevel(logging::LOG_LEVEL_ERROR);
    const bool opened = capture_.open(deviceIndex);
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
