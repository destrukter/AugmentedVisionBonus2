#include "ui/CameraWindow.h"

#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

#include <SDL_opengl.h>
#include <imgui.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

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

// Draws the detected target's quad (and a confidence label) onto a BGR frame.
void drawTrackingOutline(cv::Mat& frame, const Detection& d) {
    std::vector<cv::Point> pts;
    pts.reserve(d.corners.size());
    for (const cv::Point2f& c : d.corners) {
        pts.emplace_back(cvRound(c.x), cvRound(c.y));
    }
    const cv::Scalar green(0, 255, 0); // BGR
    cv::polylines(frame, pts, /*isClosed=*/true, green, 2, cv::LINE_AA);

    char label[64];
    std::snprintf(label, sizeof(label), "#%llu (%.0f%%)",
                  static_cast<unsigned long long>(d.imageId),
                  d.confidence * 100.0f);
    cv::putText(frame, label, pts.front() + cv::Point(0, -6),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, green, 1, cv::LINE_AA);
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

    bool preview = previewWhenUntracked_;
    if (ImGui::Checkbox("Preview model when untracked", &preview)) {
        previewWhenUntracked_ = preview;
    }
    ImGui::SameLine();
    bool outline = showTrackingOutline_;
    if (ImGui::Checkbox("Show tracking outline", &outline)) {
        showTrackingOutline_ = outline;
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

    renderer_->beginFrame(haveFrame ? frame : cv::Mat());

    int rendered = 0;
    if (tracker_ && haveFrame) {
        const std::vector<Detection> detections = tracker_->detect(frame);
        for (const Detection& d : detections) {
            // Debug: outline the tracked target on the feed so its detection can
            // be verified visually. Drawn on `frame`, which SceneRenderer holds a
            // shallow reference to, so it appears in the composited background.
            if (showTrackingOutline_) {
                drawTrackingOutline(frame, d);
            }
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
