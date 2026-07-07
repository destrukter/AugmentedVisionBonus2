#include "core/Application.h"

#include "render/ModelLoader.h"
#include "render/OgreContext.h"
#include "render/SceneRenderer.h"
#include "storage/DataStore.h"
#include "ui/CameraWindow.h"
#include "ui/ConfigureWindow.h"
#include "ui/UploadWindow.h"
#include "vision/CameraCapture.h"
#include "vision/CaptureWorker.h"
#include "vision/ImageTracker.h"
#include "vision/TrackingWorker.h"

#include <SDL.h>
#include <nfd.h>

namespace avb {

Application::Application() = default;
Application::~Application() { shutdown(); }

bool Application::initialize() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return false;
    }
    sdlInitialized_ = true;

    if (NFD_Init() != NFD_OKAY) {
        SDL_Log("NFD_Init failed: %s", NFD_GetError());
        return false;
    }
    nfdInitialized_ = true;

    // Request an OpenGL 3.0 context (matches ImGui's "#version 130"). Profile
    // masks (core/compatibility) are only meaningful for GL >= 3.2 - requesting
    // the core profile together with version 3.0 is an invalid combination that
    // drivers resolve inconsistently (observed: Mesa silently substitutes a 4.5
    // compatibility context instead), which left the GL state ImGui depends on
    // (VAO/profile-mask-derived behaviour) mismatched with what it had detected,
    // causing draw calls to silently fail with GL_INVALID_OPERATION. Requesting
    // compatibility explicitly keeps the context consistent with the GLSL 130
    // shaders across drivers.
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    // Backend store shared by every window.
    store_ = std::make_shared<DataStore>();

    // Rendering + vision pipeline.
    ogre_ = std::make_shared<OgreContext>();
    if (!ogre_->initialize()) {
        return false;
    }
    modelLoader_ = std::make_shared<ModelLoader>(*ogre_);
    renderer_ = std::make_shared<SceneRenderer>(ogre_, modelLoader_, store_);
    if (!renderer_->initialize(1280, 720)) {
        return false;
    }
    // Camera frames and detection run on background threads so the UI never
    // blocks on the camera or on feature matching; the capture worker also
    // opens the device (and keeps retrying if none is attached yet).
    capture_ = std::make_shared<CameraCapture>();
    tracker_ = std::make_shared<ImageTracker>();
    captureWorker_ = std::make_shared<CaptureWorker>(capture_, /*deviceIndex=*/0);
    trackingWorker_ = std::make_shared<TrackingWorker>(captureWorker_, tracker_);
    captureWorker_->start();
    trackingWorker_->start();

    // Frontend windows. The Upload window's "Configure" button routes the
    // chosen assignment into the Configure window.
    configureWindow_ = std::make_unique<ConfigureWindow>(store_);
    uploadWindow_ = std::make_unique<UploadWindow>(
        store_, [this](Id assignmentId) {
            configureWindow_->openAssignment(assignmentId);
        });
    cameraWindow_ = std::make_unique<CameraWindow>(
        store_, captureWorker_, trackingWorker_, tracker_, renderer_);

    if (!uploadWindow_->initialize() || !configureWindow_->initialize() ||
        !cameraWindow_->initialize()) {
        return false;
    }

    running_ = true;
    return true;
}

int Application::run() {
    // Pace the loop to ~60 fps by sleeping only for the time the frame left
    // over. The previous fixed SDL_Delay(16) came *on top of* the frame's own
    // cost, capping the UI well below 60 fps and adding a frame of latency.
    constexpr double kTargetFrameMs = 1000.0 / 60.0;
    const double msPerCount = 1000.0 / SDL_GetPerformanceFrequency();

    while (running_) {
        const Uint64 frameStart = SDL_GetPerformanceCounter();

        pumpEvents();
        renderAll();

        // The app exits once the user has closed all three windows.
        running_ = uploadWindow_->isOpen() || configureWindow_->isOpen() ||
                   cameraWindow_->isOpen();

        const double elapsedMs =
            (SDL_GetPerformanceCounter() - frameStart) * msPerCount;
        if (elapsedMs < kTargetFrameMs) {
            SDL_Delay(static_cast<Uint32>(kTargetFrameMs - elapsedMs));
        }
    }
    return 0;
}

void Application::pumpEvents() {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        if (e.type == SDL_QUIT) {
            uploadWindow_->requestClose();
            configureWindow_->requestClose();
            cameraWindow_->requestClose();
            continue;
        }
        // Each window ignores events that target a different window id.
        uploadWindow_->handleEvent(e);
        configureWindow_->handleEvent(e);
        cameraWindow_->handleEvent(e);
    }
}

void Application::renderAll() {
    // Drives OGRE's off-screen render (its own GL context) before any window's
    // own render pass acquires its GL context, rather than from within
    // CameraWindow::drawUi() mid-pass; see updateTrackingAndRender()'s comment.
    cameraWindow_->updateTrackingAndRender();
    uploadWindow_->renderFrame();
    configureWindow_->renderFrame();
    cameraWindow_->renderFrame();
}

void Application::shutdown() {
    // Stop the background threads before tearing down anything they touch.
    if (trackingWorker_) {
        trackingWorker_->stop();
    }
    if (captureWorker_) {
        captureWorker_->stop();
    }
    // Destroy windows (and their GL/ImGui contexts) before tearing down SDL.
    cameraWindow_.reset();
    configureWindow_.reset();
    uploadWindow_.reset();
    renderer_.reset();
    trackingWorker_.reset();
    captureWorker_.reset();
    if (nfdInitialized_) {
        NFD_Quit();
        nfdInitialized_ = false;
    }
    if (sdlInitialized_) {
        SDL_Quit();
        sdlInitialized_ = false;
    }
    running_ = false;
}

} // namespace avb
