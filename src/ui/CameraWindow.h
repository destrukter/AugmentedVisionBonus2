#pragma once

#include <memory>

#include "ui/Window.h"

namespace avb {

class DataStore;
class CameraCapture;
class ImageTracker;
class SceneRenderer;

/// Camera window (window 3 of 3).
///
/// Each frame it:
///   1. Grabs a camera frame (CameraCapture / OpenCV).
///   2. Detects which uploaded images are visible and their pose (ImageTracker).
///   3. For every detected image, renders the FBX models assigned to it at their
///      configured transform (SceneRenderer / OGRE3D), composited over the feed.
class CameraWindow : public Window {
public:
    CameraWindow(std::shared_ptr<DataStore> store,
                 std::shared_ptr<CameraCapture> capture,
                 std::shared_ptr<ImageTracker> tracker,
                 std::shared_ptr<SceneRenderer> renderer);
    ~CameraWindow() override;

    /// (Re)builds tracker templates from the images currently in the store.
    /// Call after images are uploaded/removed.
    void refreshTrackedImages();

protected:
    void drawUi() override;

private:
    void updateTrackingAndRender();

    std::shared_ptr<DataStore> store_;
    std::shared_ptr<CameraCapture> capture_;
    std::shared_ptr<ImageTracker> tracker_;
    std::shared_ptr<SceneRenderer> renderer_;
};

} // namespace avb
