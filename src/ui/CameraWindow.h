#pragma once

#include <cstddef>
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

    /// When true (default), if no image is tracked but assignments exist, the
    /// first assigned model is rendered at a fixed pose in front of the camera
    /// so the 3D pipeline is visible without a working tracker/camera.
    void setPreviewWhenUntracked(bool enabled) { previewWhenUntracked_ = enabled; }

protected:
    void drawUi() override;

private:
    void updateTrackingAndRender();
    void uploadCompositedToTexture();  // composited image -> this window's GL texture

    std::shared_ptr<DataStore> store_;
    std::shared_ptr<CameraCapture> capture_;
    std::shared_ptr<ImageTracker> tracker_;
    std::shared_ptr<SceneRenderer> renderer_;

    unsigned int glTexture_{0};     // GL texture id owned by this window's context
    int texWidth_{0};
    int texHeight_{0};
    std::size_t lastImageCount_{0}; // triggers tracker refresh on change
    bool previewWhenUntracked_{true};
    bool showTrackingOutline_{true}; // debug rectangle around tracked targets
};

} // namespace avb
