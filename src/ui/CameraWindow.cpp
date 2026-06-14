#include "ui/CameraWindow.h"

#include <cstdint>
#include <utility>
#include <vector>

#include <SDL_opengl.h>
#include <imgui.h>
#include <opencv2/core.hpp>

#include "render/SceneRenderer.h"
#include "storage/Assets.h"
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

// A fixed pose placing a model a little in front of the camera (looking down
// -Z), used for the preview fallback when nothing is tracked yet.
Eigen::Matrix4f previewPose() {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m(2, 3) = -3.0f; // 3 units in front of the camera
    return m;
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

CameraWindow::~CameraWindow() {
    if (glTexture_ != 0) {
        GLuint id = glTexture_;
        glDeleteTextures(1, &id);
        glTexture_ = 0;
    }
}

void CameraWindow::refreshTrackedImages() {
    if (!tracker_) {
        return;
    }
    tracker_->clearTargets();
    for (const Id id : store_->imageIds()) {
        const ImageAsset* img = store_->image(id);
        if (img && !img->pixels.empty()) {
            tracker_->addTarget(id, img->pixels);
        }
    }
}

void CameraWindow::drawUi() {
    updateTrackingAndRender();
    // updateTrackingAndRender() drives OGRE's off-screen render, which makes
    // OGRE's own GL context current and leaves it so. Restore this window's GL
    // context before uploading the camera texture and letting ImGui draw, or the
    // texture is created in the wrong context and the feed shows up black.
    makeContextCurrent();
    uploadCompositedToTexture();

    beginFullWindow("Camera");

    const bool camOpen = capture_ && capture_->isOpen();
    ImGui::Text("Camera: %s", camOpen ? "connected" : "no device");
    ImGui::SameLine();
    ImGui::Text("| Tracked images: %zu", store_->imageIds().size());
    ImGui::SameLine();
    ImGui::Text("| Frame: %s (avg %.1f)",
                lastGrabHadFrame_ ? "yes" : "none", lastFrameBrightness_);

    bool preview = previewWhenUntracked_;
    if (ImGui::Checkbox("Preview model when untracked", &preview)) {
        previewWhenUntracked_ = preview;
    }
    ImGui::Separator();

    if (glTexture_ != 0) {
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        ImGui::Image(reinterpret_cast<ImTextureID>(
                         static_cast<std::uintptr_t>(glTexture_)),
                     avail);
    } else {
        ImGui::TextDisabled("No rendered frame yet.");
    }

    ImGui::End();
}

void CameraWindow::updateTrackingAndRender() {
    if (!renderer_) {
        return;
    }

    // Keep tracker templates in sync with the uploaded image set.
    const std::size_t count = store_->imageIds().size();
    if (count != lastImageCount_) {
        refreshTrackedImages();
        lastImageCount_ = count;
    }

    cv::Mat frame;
    const bool haveFrame = capture_ && capture_->grab(frame);

    // Diagnostic: record the grabbed frame's average brightness so a black feed
    // can be attributed to the capture (avg ~0) vs the display path.
    lastGrabHadFrame_ = haveFrame;
    if (haveFrame) {
        const cv::Scalar m = cv::mean(frame);
        lastFrameBrightness_ = (m[0] + m[1] + m[2]) / 3.0;
    }

    renderer_->beginFrame(haveFrame ? frame : cv::Mat());

    int rendered = 0;
    if (tracker_ && haveFrame) {
        for (const Detection& d : tracker_->detect(frame)) {
            for (const Id aid : store_->assignmentsForImage(d.imageId)) {
                const Assignment* a = store_->assignment(aid);
                if (!a) {
                    continue;
                }
                // Final pose = detected image pose * configured model offset.
                const Eigen::Matrix4f pose = d.poseInCamera * a->transform.toMatrix();
                renderer_->drawModel(a->modelId, pose);
                ++rendered;
            }
        }
    }

    // Fallback so the 3D pipeline is visible before tracking is implemented.
    if (rendered == 0 && previewWhenUntracked_) {
        const std::vector<Id> assignments = store_->assignmentIds();
        if (!assignments.empty()) {
            const Assignment* a = store_->assignment(assignments.front());
            if (a) {
                renderer_->drawModel(a->modelId,
                                     previewPose() * a->transform.toMatrix());
            }
        }
    }

    renderer_->endFrame();
}

void CameraWindow::uploadCompositedToTexture() {
    if (!renderer_) {
        return;
    }
    const cv::Mat& img = renderer_->compositedImage();
    if (img.empty()) {
        return;
    }

    if (glTexture_ == 0) {
        GLuint id = 0;
        glGenTextures(1, &id);
        glTexture_ = id;
        glBindTexture(GL_TEXTURE_2D, glTexture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    } else {
        glBindTexture(GL_TEXTURE_2D, glTexture_);
    }

    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    if (img.cols != texWidth_ || img.rows != texHeight_) {
        texWidth_ = img.cols;
        texHeight_ = img.rows;
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texWidth_, texHeight_, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, img.data);
    } else {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texWidth_, texHeight_, GL_RGBA,
                        GL_UNSIGNED_BYTE, img.data);
    }
}

} // namespace avb
