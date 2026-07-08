#include "core/Application.h"

#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "render/ConfigurePreview.h"
#include "render/ModelLoader.h"
#include "render/OgreContext.h"
#include "render/SceneRenderer.h"
#include "storage/AssetLibrary.h"
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

namespace {

// Resolves the asset-library root without depending on the process working
// directory (launching from an IDE or the build folder must find the same
// library as launching from the repo root). Candidates, first existing wins:
// the AVB_LIBRARY_DIR override, cwd-relative, relative to the executable
// (which build trees place a few levels below the repo), and the source
// assets directory recorded at configure time. Returns "" when none exists;
// `tried` lists every candidate for the not-found diagnostic.
std::string resolveLibraryDir(std::vector<std::string>& tried) {
    namespace fs = std::filesystem;
    std::vector<fs::path> candidates;
    if (const char* env = std::getenv("AVB_LIBRARY_DIR")) {
        candidates.emplace_back(env);
    }
    candidates.emplace_back("assets/library");
    if (char* base = SDL_GetBasePath()) {
        const fs::path exeDir(base);
        SDL_free(base);
        candidates.push_back(exeDir / "assets/library");
        candidates.push_back(exeDir / "../assets/library");
        candidates.push_back(exeDir / "../../assets/library");
    }
#ifdef AVB_SOURCE_ASSETS_DIR
    candidates.emplace_back(fs::path(AVB_SOURCE_ASSETS_DIR) / "library");
#endif

    for (const fs::path& candidate : candidates) {
        std::error_code ec;
        const fs::path normalized = fs::weakly_canonical(candidate, ec);
        const fs::path& path = ec ? candidate : normalized;
        tried.push_back(path.string());
        if (fs::is_directory(path, ec)) {
            return path.string();
        }
    }
    return "";
}

} // namespace

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

    // Auto-upload the default asset library (assets/library, or the folder
    // AVB_LIBRARY_DIR points to) so recurring images/models, their
    // assignments and their saved poses are available without manual uploads.
    // The summary is shown in the Upload window once it exists (below).
    std::vector<std::string> triedLibraryDirs;
    libraryDir_ = resolveLibraryDir(triedLibraryDirs);
    AssetLibrary library(store_, &ModelLoader::validateModelFile);
    AssetLibrary::Report libraryReport;
    if (libraryDir_.empty()) {
        std::string tried;
        for (const std::string& dir : triedLibraryDirs) {
            tried += (tried.empty() ? "" : "; ") + dir;
        }
        libraryReport.warnings.push_back(
            "asset library folder not found - looked in: " + tried);
    } else {
        libraryReport = library.load(libraryDir_);
    }
    for (const std::string& warning : libraryReport.warnings) {
        SDL_Log("Asset library: %s", warning.c_str());
    }
    SDL_Log("Asset library '%s': %d image(s), %d model(s), %d assignment(s)",
            libraryDir_.c_str(), libraryReport.imagesAdded,
            libraryReport.modelsAdded, libraryReport.assignmentsCreated);

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

    // Off-screen renderer for the Configure window's viewport (the actual
    // image + model behind the gizmo).
    configurePreview_ =
        std::make_shared<ConfigurePreview>(ogre_, modelLoader_, store_);

    // Frontend windows. The Upload window's "Configure" button routes the
    // chosen assignment into the Configure window. Saved poses of library
    // assets are persisted back into assignments.cfg so they survive
    // restarts.
    configureWindow_ = std::make_unique<ConfigureWindow>(
        store_, configurePreview_, [this](Id assignmentId) {
            AssetLibrary lib(store_, &ModelLoader::validateModelFile);
            if (lib.persistAssignment(libraryDir_, assignmentId)) {
                SDL_Log("Asset library: pose of assignment #%llu saved to "
                        "%s/assignments.cfg",
                        static_cast<unsigned long long>(assignmentId),
                        libraryDir_.c_str());
            }
        });
    uploadWindow_ = std::make_unique<UploadWindow>(
        store_,
        [this](Id assignmentId) {
            configureWindow_->openAssignment(assignmentId);
        },
        [this]() { saveSessionToLibrary(); });
    cameraWindow_ = std::make_unique<CameraWindow>(
        store_, captureWorker_, trackingWorker_, tracker_, renderer_);

    if (!uploadWindow_->initialize() || !configureWindow_->initialize() ||
        !cameraWindow_->initialize()) {
        return false;
    }

    // Surface the asset-library result where uploads are managed.
    if (libraryDir_.empty()) {
        uploadWindow_->setStatus(
            UploadWindow::StatusKind::Warning,
            "Asset library folder not found (see the log for the paths "
            "searched). Create assets/library/{images,models} or set "
            "AVB_LIBRARY_DIR.");
    } else if (libraryReport.imagesAdded + libraryReport.modelsAdded > 0 ||
               !libraryReport.warnings.empty()) {
        const std::string summary =
            "Library: " + std::to_string(libraryReport.imagesAdded) +
            " image(s), " + std::to_string(libraryReport.modelsAdded) +
            " model(s), " + std::to_string(libraryReport.assignmentsCreated) +
            " assignment(s) loaded from '" + libraryDir_ + "'" +
            (libraryReport.warnings.empty()
                 ? ""
                 : " - " + std::to_string(libraryReport.warnings.size()) +
                       " warning(s), see log");
        uploadWindow_->setStatus(libraryReport.warnings.empty()
                                     ? UploadWindow::StatusKind::Success
                                     : UploadWindow::StatusKind::Warning,
                                 summary);
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

void Application::saveSessionToLibrary() {
    // When no library existed at startup, create one at the default location
    // so the session has somewhere to go.
    if (libraryDir_.empty()) {
#ifdef AVB_SOURCE_ASSETS_DIR
        libraryDir_ = std::string(AVB_SOURCE_ASSETS_DIR) + "/library";
#else
        libraryDir_ = "assets/library";
#endif
    }
    AssetLibrary library(store_, &ModelLoader::validateModelFile);
    const AssetLibrary::SessionSaveResult result =
        library.saveSession(libraryDir_);
    for (const std::string& warning : result.warnings) {
        SDL_Log("Save session: %s", warning.c_str());
    }
    const std::string summary =
        "Session saved to '" + libraryDir_ + "': " +
        std::to_string(result.filesCopied) + " file(s) copied, " +
        std::to_string(result.assignmentsSaved) +
        " assignment(s) written to assignments.cfg" +
        (result.warnings.empty()
             ? ""
             : " - " + std::to_string(result.warnings.size()) +
                   " warning(s), see log");
    uploadWindow_->setStatus(result.warnings.empty()
                                 ? UploadWindow::StatusKind::Success
                                 : UploadWindow::StatusKind::Warning,
                             summary);
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
    // Drive both OGRE off-screen renders (OGRE's own GL context) before any
    // window's render pass acquires its GL context, rather than from within
    // drawUi() mid-pass; see updateTrackingAndRender()'s comment.
    cameraWindow_->updateTrackingAndRender();
    configureWindow_->updatePreviewRender();
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
    configurePreview_.reset();
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
