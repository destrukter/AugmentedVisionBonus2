#pragma once

#include <memory>
#include <string>
#include <vector>

namespace avb {

class DataStore;
class OgreContext;
class ModelLoader;
class SceneRenderer;
class ConfigurePreview;
class CameraCapture;
class CaptureWorker;
class ImageTracker;
class TrackingWorker;
class Window;
class UploadWindow;
class ConfigureWindow;
class CameraWindow;

/// Top-level application object.
///
/// Owns the shared backend (DataStore) and the three frontend windows, wires
/// them together, and runs the single-threaded main loop that keeps all three
/// OS windows open and rendering at the same time.
class Application {
public:
    Application();
    ~Application();

    /// Initializes SDL, the OGRE context, the vision pipeline and the three
    /// windows. Returns false on failure.
    bool initialize();

    /// Runs the main loop until all windows are closed. Returns process code.
    int run();

    void shutdown();

private:
    void pumpEvents();   ///< Dispatch SDL events to the right window.
    void renderAll();    ///< Render one frame of every open window.
    /// Copies external assets into the library and writes all assignments +
    /// poses to assignments.cfg (the Upload window's "Save session" button).
    void saveSessionToLibrary();

    // Backend (shared).
    std::shared_ptr<DataStore> store_;
    std::string libraryDir_;  ///< Asset-library root (poses persist here).

    // Rendering / vision (shared with the Camera window).
    std::shared_ptr<OgreContext> ogre_;
    std::shared_ptr<ModelLoader> modelLoader_;
    std::shared_ptr<SceneRenderer> renderer_;
    std::shared_ptr<ConfigurePreview> configurePreview_;  // Configure viewport
    std::shared_ptr<CameraCapture> capture_;
    std::shared_ptr<ImageTracker> tracker_;
    std::shared_ptr<CaptureWorker> captureWorker_;    // camera thread
    std::shared_ptr<TrackingWorker> trackingWorker_;  // detection thread

    // Frontend windows.
    std::unique_ptr<UploadWindow> uploadWindow_;
    std::unique_ptr<ConfigureWindow> configureWindow_;
    std::unique_ptr<CameraWindow> cameraWindow_;

    bool running_{false};
    bool sdlInitialized_{false};
    bool nfdInitialized_{false};
};

} // namespace avb
