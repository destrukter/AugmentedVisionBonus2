#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace avb {

/// A camera device that can be passed to CameraCapture::open().
struct CameraDeviceInfo {
    int index{0};       ///< OpenCV device index.
    std::string name;   ///< Human-readable device name.
};

/// Thin wrapper around an OpenCV camera/video source.
class CameraCapture {
public:
    CameraCapture() = default;

    /// Enumerates the attached capture devices. On Linux this scans
    /// /dev/video* and reads the driver-reported names from sysfs (note that
    /// one physical camera may expose several nodes; only some are capture
    /// streams). On other platforms, where no cheap enumeration exists, the
    /// first few indices are offered generically.
    static std::vector<CameraDeviceInfo> listDevices();

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
