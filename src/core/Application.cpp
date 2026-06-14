#include "core/Application.h"

#include "render/ModelLoader.h"
#include "render/OgreContext.h"
#include "render/SceneRenderer.h"
#include "storage/DataStore.h"
#include "ui/CameraWindow.h"
#include "ui/ConfigureWindow.h"
#include "ui/UploadWindow.h"
#include "vision/CameraCapture.h"
#include "vision/ImageTracker.h"

// #include <SDL.h>

namespace avb {

Application::Application() = default;
Application::~Application() { shutdown(); }

bool Application::initialize() {
    // TODO: SDL_Init(SDL_INIT_VIDEO); configure GL attributes.

    // Backend store shared by every window.
    store_ = std::make_shared<DataStore>();

    // Rendering + vision pipeline.
    ogre_ = std::make_shared<OgreContext>();
    if (!ogre_->initialize()) {
        return false;
    }
    modelLoader_ = std::make_shared<ModelLoader>(*ogre_);
    renderer_ = std::make_shared<SceneRenderer>(ogre_, modelLoader_, store_);
    if (!renderer_->initialize()) {
        return false;
    }
    capture_ = std::make_shared<CameraCapture>();
    capture_->open(0);
    tracker_ = std::make_shared<ImageTracker>();

    // Frontend windows. The Upload window's "Configure" button routes the
    // chosen assignment into the Configure window.
    configureWindow_ = std::make_unique<ConfigureWindow>(store_);
    uploadWindow_ = std::make_unique<UploadWindow>(
        store_, [this](Id assignmentId) {
            configureWindow_->openAssignment(assignmentId);
        });
    cameraWindow_ =
        std::make_unique<CameraWindow>(store_, capture_, tracker_, renderer_);

    if (!uploadWindow_->initialize() || !configureWindow_->initialize() ||
        !cameraWindow_->initialize()) {
        return false;
    }

    running_ = true;
    return true;
}

int Application::run() {
    while (running_) {
        pumpEvents();
        renderAll();

        // The app exits once the user has closed all three windows.
        running_ = uploadWindow_->isOpen() || configureWindow_->isOpen() ||
                   cameraWindow_->isOpen();
    }
    return 0;
}

void Application::pumpEvents() {
    // TODO:
    //   SDL_Event e;
    //   while (SDL_PollEvent(&e)) {
    //       uploadWindow_->handleEvent(e);
    //       configureWindow_->handleEvent(e);
    //       cameraWindow_->handleEvent(e);
    //   }
}

void Application::renderAll() {
    uploadWindow_->renderFrame();
    configureWindow_->renderFrame();
    cameraWindow_->renderFrame();
}

void Application::shutdown() {
    cameraWindow_.reset();
    configureWindow_.reset();
    uploadWindow_.reset();
    renderer_.reset();
    // TODO: SDL_Quit();
    running_ = false;
}

} // namespace avb
