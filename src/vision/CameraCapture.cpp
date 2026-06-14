#include "vision/CameraCapture.h"

namespace avb {

bool CameraCapture::open(int deviceIndex) {
    return capture_.open(deviceIndex);
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
