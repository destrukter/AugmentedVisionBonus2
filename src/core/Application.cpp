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

#include <SDL.h>

namespace avb {

Application::Application() = default;
Application::~Application() { shutdown(); }

bool Application::initialize() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return false;
    }
    sdlInitialized_ = true;

    // Request an OpenGL 3.0 core context (matches ImGui's "#version 130").
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
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

        // Cap the loop (~60 fps) so idle windows don't busy-spin the CPU.
        SDL_Delay(16);
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
    uploadWindow_->renderFrame();
    configureWindow_->renderFrame();
    cameraWindow_->renderFrame();
}

void Application::shutdown() {
    // Destroy windows (and their GL/ImGui contexts) before tearing down SDL.
    cameraWindow_.reset();
    configureWindow_.reset();
    uploadWindow_.reset();
    renderer_.reset();
    if (sdlInitialized_) {
        SDL_Quit();
        sdlInitialized_ = false;
    }
    running_ = false;
}

} // namespace avb
