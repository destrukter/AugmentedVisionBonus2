#pragma once

#include <cstdint>
#include <memory>

#include "ui/Window.h"

namespace avb {

class DataStore;
class CaptureWorker;
class TrackingWorker;
class ImageTracker;
class SceneRenderer;

/// Camera window (window 3 of 3).
///
/// Capture and tracking run on background threads (CaptureWorker /
/// TrackingWorker); each frame this window only pairs the newest camera frame
/// with the newest tracking result and drives the OGRE composite:
///   1. Take the latest captured frame (never blocks on the camera).
///   2. Take the latest smoothed detections.
///   3. For every detected image, render the FBX models assigned to it at
///      their configured transform (SceneRenderer / OGRE3D), composited over
///      the feed, and display the result letterboxed.
class CameraWindow : public Window {
public:
    CameraWindow(std::shared_ptr<DataStore> store,
                 std::shared_ptr<CaptureWorker> capture,
                 std::shared_ptr<TrackingWorker> tracking,
                 std::shared_ptr<ImageTracker> tracker,
                 std::shared_ptr<SceneRenderer> renderer);
    ~CameraWindow() override;

    /// (Re)builds tracker templates from the images currently in the store.
    /// Runs automatically when DataStore::imageRevision() changes.
    void refreshTrackedImages();

    /// Consumes the latest frame + detections and drives OGRE's off-screen
    /// render for this frame. This makes OGRE's own GL context current, so the
    /// Application must call it before this window's renderFrame() rather than
    /// from drawUi(): switching away from and back to this window's GL context
    /// mid-render-pass (as opposed to once, before the pass starts) left this
    /// window's existing GL objects (e.g. its ImGui shader program) intact
    /// only intermittently on at least one driver (Mesa llvmpipe), which
    /// silently broke every draw call and left the composited feed blank.
    void updateTrackingAndRender();

    /// When true (default), if no image is tracked but assignments exist, the
    /// first assigned model is rendered at a fixed pose in front of the camera
    /// so the 3D pipeline is visible without a working tracker/camera.
    void setPreviewWhenUntracked(bool enabled) { previewWhenUntracked_ = enabled; }

protected:
    void drawUi() override;

private:
    void uploadCompositedToTexture();  // composited image -> this window's GL texture

    std::shared_ptr<DataStore> store_;
    std::shared_ptr<CaptureWorker> capture_;
    std::shared_ptr<TrackingWorker> tracking_;
    std::shared_ptr<ImageTracker> tracker_;
    std::shared_ptr<SceneRenderer> renderer_;

    unsigned int glTexture_{0};     // GL texture id owned by this window's context
    int texWidth_{0};
    int texHeight_{0};
    std::uint64_t lastImageRevision_{0};  // triggers tracker refresh on change
    int detectionCount_{0};               // detections used for the last render
    bool previewWhenUntracked_{true};
    bool showTrackingOutline_{true}; // debug rectangle around tracked targets
};

} // namespace avb
