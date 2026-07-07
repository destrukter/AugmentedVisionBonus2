#include "ui/CameraWindow.h"

#include <algorithm>
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
#include "vision/CaptureWorker.h"
#include "vision/ImageTracker.h"
#include "vision/TrackingWorker.h"

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
                           std::shared_ptr<CaptureWorker> capture,
                           std::shared_ptr<TrackingWorker> tracking,
                           std::shared_ptr<ImageTracker> tracker,
                           std::shared_ptr<SceneRenderer> renderer)
    : Window("Camera", 1280, 720),
      store_(std::move(store)),
      capture_(std::move(capture)),
      tracking_(std::move(tracking)),
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
    if (tracking_) {
        tracking_->resetFilter();
    }
}

void CameraWindow::drawUi() {
    // updateTrackingAndRender() already ran (see Application::renderAll()) and
    // drove OGRE's off-screen render before this window's own GL context was
    // reacquired for the frame; nothing here needs to touch OGRE's context.
    uploadCompositedToTexture();

    beginFullWindow("Camera");

    const bool camOpen = capture_ && capture_->cameraOpen();
    if (camOpen) {
        ImGui::Text("Camera: connected  |  feed %.0f fps  |  tracking %.0f fps",
                    capture_->fps(), tracking_ ? tracking_->fps() : 0.0);
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.3f, 1.0f),
                           "Camera: no device (retrying...)");
        ImGui::SameLine();
        if (ImGui::Button("Reconnect") && capture_) {
            capture_->requestReconnect();
        }
    }
    ImGui::SameLine();
    ImGui::Text("| Images: %zu | Tracked now: %d", store_->imageIds().size(),
                detectionCount_);

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

    if (glTexture_ != 0 && texWidth_ > 0 && texHeight_ > 0) {
        // Letterbox: scale the feed to fit the available region while keeping
        // its aspect ratio, instead of stretching it to the window. The feed
        // is only ever scaled uniformly for display; tracking always runs on
        // the raw camera frames, so the window size never affects detection.
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        if (avail.x >= 1.0f && avail.y >= 1.0f) {
            const float scale =
                std::min(avail.x / static_cast<float>(texWidth_),
                         avail.y / static_cast<float>(texHeight_));
            const ImVec2 size(texWidth_ * scale, texHeight_ * scale);
            const ImVec2 cursor = ImGui::GetCursorPos();
            ImGui::SetCursorPos(ImVec2(cursor.x + (avail.x - size.x) * 0.5f,
                                       cursor.y + (avail.y - size.y) * 0.5f));
            ImGui::Image(reinterpret_cast<ImTextureID>(
                             static_cast<std::uintptr_t>(glTexture_)),
                         size);
        }
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
    const std::uint64_t revision = store_->imageRevision();
    if (revision != lastImageRevision_) {
        refreshTrackedImages();
        lastImageRevision_ = revision;
    }

    // Newest frame (our own copy) + newest smoothed detections; neither call
    // blocks on the camera or on feature matching.
    cv::Mat frame;
    const bool haveFrame = capture_ && capture_->latestFrame(frame) != 0;

    renderer_->beginFrame(haveFrame ? frame : cv::Mat());

    int rendered = 0;
    detectionCount_ = 0;
    if (tracking_ && haveFrame) {
        const std::vector<Detection> detections = tracking_->latestDetections();
        detectionCount_ = static_cast<int>(detections.size());
        for (const Detection& d : detections) {
            // Outline the tracked target on the feed so its detection can be
            // verified visually. Drawn on `frame`, which SceneRenderer holds a
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

    // Fallback so the 3D pipeline is visible without a tracked image.
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
