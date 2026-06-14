#include "ui/CameraWindow.h"

#include <utility>

#include <imgui.h>

#include "render/SceneRenderer.h"
#include "storage/DataStore.h"
#include "vision/CameraCapture.h"
#include "vision/ImageTracker.h"

namespace avb {

namespace {

void beginFullWindow(const char* name) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::Begin(name, nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);
}

} // namespace

CameraWindow::CameraWindow(std::shared_ptr<DataStore> store,
                           std::shared_ptr<CameraCapture> capture,
                           std::shared_ptr<ImageTracker> tracker,
                           std::shared_ptr<SceneRenderer> renderer)
    : Window("Camera", 1280, 720),
      store_(std::move(store)),
      capture_(std::move(capture)),
      tracker_(std::move(tracker)),
      renderer_(std::move(renderer)) {}

CameraWindow::~CameraWindow() = default;

void CameraWindow::refreshTrackedImages() {
    // For each uploaded image, (re)register a tracking template keyed by its Id.
    //   tracker_->clearTargets();
    //   for (Id id : store_->imageIds()) {
    //       const ImageAsset* img = store_->image(id);
    //       if (img && !img->pixels.empty()) tracker_->addTarget(id, img->pixels);
    //   }
}

void CameraWindow::drawUi() {
    updateTrackingAndRender();

    beginFullWindow("Camera");

    const bool camOpen = capture_ && capture_->isOpen();
    ImGui::Text("Camera: %s", camOpen ? "connected" : "not available");
    ImGui::Text("Tracked images: %zu", store_->imageIds().size());
    ImGui::Separator();

    void* tex = renderer_ ? renderer_->compositedTexture() : nullptr;
    if (tex) {
        // Composited feed + rendered models (filled in by SceneRenderer).
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        ImGui::Image(reinterpret_cast<ImTextureID>(tex), avail);
    } else {
        ImGui::TextDisabled(
            "Live AR view will appear here once the OGRE SceneRenderer is "
            "wired up.");
    }

    ImGui::End();
}

void CameraWindow::updateTrackingAndRender() {
    // Pseudocode for the per-frame AR pipeline:
    //
    //   cv::Mat frame;
    //   if (!capture_->grab(frame)) return;
    //
    //   renderer_->beginFrame(frame);                  // draw feed as background
    //   for (const Detection& d : tracker_->detect(frame)) {
    //       for (Id aid : store_->assignmentsForImage(d.imageId)) {
    //           const Assignment* a = store_->assignment(aid);
    //           // Final pose = image pose in camera space * configured offset.
    //           Eigen::Matrix4f pose = d.poseInCamera * a->transform.toMatrix();
    //           renderer_->drawModel(a->modelId, pose);
    //       }
    //   }
    //   renderer_->endFrame();
}

} // namespace avb
