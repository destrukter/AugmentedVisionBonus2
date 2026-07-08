#include "ui/ConfigureWindow.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

#include <SDL_opengl.h>
#include <imgui.h>
#include <ImGuizmo.h>

#include <Eigen/Geometry>

#include "render/ConfigurePreview.h"
#include "storage/Assets.h"
#include "storage/DataStore.h"
#include "ui/Panels.h"

namespace avb {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;

// Vertical FOV of the viewport; must be identical for the gizmo projection
// and the ConfigurePreview render or handles drift off the rendered pixels.
constexpr float kViewportFovYDeg = 45.0f;

// Column-major (OpenGL-style) right-handed perspective projection, the layout
// ImGuizmo expects. Eigen matrices are column-major by default, so .data()
// can be handed to ImGuizmo directly.
Eigen::Matrix4f perspective(float fovyRad, float aspect, float zNear, float zFar) {
    const float f = 1.0f / std::tan(fovyRad * 0.5f);
    Eigen::Matrix4f m = Eigen::Matrix4f::Zero();
    m(0, 0) = f / aspect;
    m(1, 1) = f;
    m(2, 2) = (zFar + zNear) / (zNear - zFar);
    m(2, 3) = 2.0f * zFar * zNear / (zNear - zFar);
    m(3, 2) = -1.0f;
    return m;
}

Eigen::Matrix4f lookAt(const Eigen::Vector3f& eye, const Eigen::Vector3f& center,
                       const Eigen::Vector3f& up) {
    const Eigen::Vector3f f = (center - eye).normalized();
    const Eigen::Vector3f s = f.cross(up).normalized();
    const Eigen::Vector3f u = s.cross(f);
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<1, 3>(0, 0) = s.transpose();
    m.block<1, 3>(1, 0) = u.transpose();
    m.block<1, 3>(2, 0) = (-f).transpose();
    m(0, 3) = -s.dot(eye);
    m(1, 3) = -u.dot(eye);
    m(2, 3) = f.dot(eye);
    return m;
}

/// Extracts translation / rotation (Euler degrees, matching
/// Transform::rotationMatrix's Rz*Ry*Rx convention) / per-axis scale from a
/// manipulated 4x4 matrix back into a Transform.
Transform decomposeToTransform(const Eigen::Matrix4f& m) {
    Transform t;
    t.translation = m.block<3, 1>(0, 3);

    Eigen::Matrix3f rs = m.block<3, 3>(0, 0);
    const float sx = rs.col(0).norm();
    const float sy = rs.col(1).norm();
    const float sz = rs.col(2).norm();
    t.scale = Eigen::Vector3f(std::max(sx, 1e-4f), std::max(sy, 1e-4f),
                              std::max(sz, 1e-4f));
    if (sx > 1e-6f) rs.col(0) /= sx;
    if (sy > 1e-6f) rs.col(1) /= sy;
    if (sz > 1e-6f) rs.col(2) /= sz;

    // eulerAngles(2,1,0) yields (a,b,c) with R = Rz(a) * Ry(b) * Rx(c) -
    // exactly the composition Transform::rotationMatrix builds.
    const Eigen::Vector3f zyx = rs.eulerAngles(2, 1, 0);
    t.rotationEulerDeg =
        Eigen::Vector3f(zyx.z() * kRadToDeg, zyx.y() * kRadToDeg,
                        zyx.x() * kRadToDeg);
    return t;
}

/// Projects a world-space point into the viewport rect. Returns false when the
/// point is behind the camera.
bool projectToScreen(const Eigen::Matrix4f& viewProj, const Eigen::Vector3f& p,
                     const ImVec2& rectPos, const ImVec2& rectSize, ImVec2& out) {
    const Eigen::Vector4f clip = viewProj * Eigen::Vector4f(p.x(), p.y(), p.z(), 1.0f);
    if (clip.w() <= 1e-6f) {
        return false;
    }
    const float ndcX = clip.x() / clip.w();
    const float ndcY = clip.y() / clip.w();
    out.x = rectPos.x + (ndcX * 0.5f + 0.5f) * rectSize.x;
    out.y = rectPos.y + (1.0f - (ndcY * 0.5f + 0.5f)) * rectSize.y;
    return true;
}

// Draws a wireframe cube proxy for the model at `model` (its full transform,
// including scale) so every pose edit - translation, rotation and notably
// scale - is visible in the viewport even though the real mesh isn't loaded
// here.
void drawModelProxy(ImDrawList* drawList, const Eigen::Matrix4f& viewProj,
                    const Eigen::Matrix4f& model, const ImVec2& rectPos,
                    const ImVec2& rectSize) {
    constexpr float h = 0.25f; // half edge length: a 0.5-unit cube at scale 1
    const Eigen::Vector3f corners[8] = {
        {-h, -h, -h}, {h, -h, -h}, {h, h, -h}, {-h, h, -h},
        {-h, -h, h},  {h, -h, h},  {h, h, h},  {-h, h, h}};
    static constexpr int kEdges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0},
                                          {4, 5}, {5, 6}, {6, 7}, {7, 4},
                                          {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    ImVec2 screen[8];
    bool visible[8];
    for (int i = 0; i < 8; ++i) {
        const Eigen::Vector4f world =
            model * Eigen::Vector4f(corners[i].x(), corners[i].y(),
                                    corners[i].z(), 1.0f);
        visible[i] = projectToScreen(viewProj, world.head<3>(), rectPos,
                                     rectSize, screen[i]);
    }
    const ImU32 color = IM_COL32(250, 200, 90, 220);
    for (const auto& e : kEdges) {
        if (visible[e[0]] && visible[e[1]]) {
            drawList->AddLine(screen[e[0]], screen[e[1]], color, 1.5f);
        }
    }
}

} // namespace

ConfigureWindow::ConfigureWindow(std::shared_ptr<DataStore> store,
                                 std::shared_ptr<ConfigurePreview> preview,
                                 SaveCallback onSaved)
    : Window("Configure", 560, 680),
      store_(std::move(store)),
      preview_(std::move(preview)),
      onSaved_(std::move(onSaved)) {}

ConfigureWindow::~ConfigureWindow() {
    if (previewTexture_ != 0) {
        GLuint id = previewTexture_;
        glDeleteTextures(1, &id);
        previewTexture_ = 0;
    }
}

Eigen::Vector3f ConfigureWindow::eyePosition() const {
    const float yaw = orbitYawDeg_ * kDegToRad;
    const float pitch = orbitPitchDeg_ * kDegToRad;
    return orbitDistance_ *
           Eigen::Vector3f(std::cos(pitch) * std::sin(yaw), std::sin(pitch),
                           std::cos(pitch) * std::cos(yaw));
}

void ConfigureWindow::updatePreviewRender() {
    previewValid_ = false;
    if (!preview_ || !isOpen() || activeAssignment_ == kInvalidId ||
        canvasW_ < 16 || canvasH_ < 16) {
        return;
    }
    previewValid_ =
        preview_->render(activeAssignment_, working_.toMatrix(), eyePosition(),
                         kViewportFovYDeg, canvasW_, canvasH_);
}

void ConfigureWindow::openAssignment(Id assignmentId) {
    activeAssignment_ = assignmentId;
    if (const auto t = store_->transform(assignmentId)) {
        working_ = *t;
    } else {
        activeAssignment_ = kInvalidId;
        working_ = Transform{};
    }
    dirty_ = false;
}

float ConfigureWindow::imagePlaneAspect() const {
    const Assignment* a = store_->assignment(activeAssignment_);
    if (!a) {
        return 1.0f;
    }
    const ImageAsset* img = store_->image(a->imageId);
    if (!img || img->pixels.empty() || img->pixels.cols <= 0) {
        return 1.0f;
    }
    return static_cast<float>(img->pixels.rows) /
           static_cast<float>(img->pixels.cols);
}

void ConfigureWindow::drawUi() {
    ImGuizmo::BeginFrame();

    // No scrolling: the gizmo viewport below fills whatever space remains, so
    // the panel's content always fits exactly (and an avail-sized canvas
    // inside a scrolling window would grow with every scroll, pushing these
    // controls permanently out of view).
    beginFullWindow("Configure", /*allowScroll=*/false);

    // The assignment may vanish behind us (reverted, or its image/model was
    // removed in the Upload window).
    if (activeAssignment_ != kInvalidId &&
        store_->assignment(activeAssignment_) == nullptr) {
        activeAssignment_ = kInvalidId;
    }
    if (activeAssignment_ == kInvalidId) {
        ImGui::TextDisabled(
            "Click 'Configure' on an assignment in the Upload window.");
        ImGui::End();
        return;
    }

    ImGui::Text("Editing assignment #%llu",
                static_cast<unsigned long long>(activeAssignment_));
    if (dirty_) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "(unsaved)");
    }
    ImGui::Separator();

    // Numeric controls bound to the working copy; the gizmo below edits the
    // same values interactively. Any change marks the pose dirty.
    bool changed = false;
    changed |= ImGui::DragFloat3("Translation (x,y,z)",
                                 working_.translation.data(), 0.01f);
    changed |= ImGui::DragFloat3("Rotation (deg)",
                                 working_.rotationEulerDeg.data(), 0.5f);
    changed |= ImGui::DragFloat3("Scale (x,y,z)", working_.scale.data(), 0.01f,
                                 0.001f, 1000.0f);
    dirty_ = dirty_ || changed;

    ImGui::TextDisabled("Model matrix preview");
    const Eigen::Matrix4f preview = working_.toMatrix();
    for (int r = 0; r < 4; ++r) {
        ImGui::Text("% .3f  % .3f  % .3f  % .3f", preview(r, 0), preview(r, 1),
                    preview(r, 2), preview(r, 3));
    }

    if (ImGui::Button("Save")) {
        save();
    }
    ImGui::SameLine();
    if (ImGui::Button("Revert")) {
        revert();
    }
    ImGui::SameLine();
    // The ##op suffixes keep these IDs distinct from the identically-labelled
    // widgets above ("Scale" collides with the Scale drag field otherwise,
    // and ImGui routes all clicks on duplicate IDs to whichever item was
    // submitted first - leaving one of the two dead).
    ImGui::RadioButton("Move##op", &gizmoOperation_, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Rotate##op", &gizmoOperation_, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Scale##op", &gizmoOperation_, 2);
    ImGui::SameLine();
    ImGui::TextDisabled("(right-drag orbits, wheel zooms)");

    drawGizmoViewport();

    ImGui::End();
}

void ConfigureWindow::drawGizmoViewport() {
    // The viewport takes all remaining panel space below the controls. Its
    // size is recorded for the next updatePreviewRender() call (1-frame lag).
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 canvasSize(std::max(avail.x, 120.0f), std::max(avail.y, 120.0f));
    canvasW_ = static_cast<int>(canvasSize.x);
    canvasH_ = static_cast<int>(canvasSize.y);
    const ImVec2 canvasPos = ImGui::GetCursorScreenPos();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    // Background: the off-screen render of the actual image + actual model
    // (ConfigurePreview), when available. Displayed 1:1 - it was rendered at
    // this canvas size with the same camera the gizmo uses below. Falls back
    // to schematic drawing (plane outline + proxy cube) when the preview
    // can't render (image pixels missing, model failed to load, first frame).
    //
    // Both ImGui::Image and Dummy reserve the layout space without ever
    // registering as the hovered *item*: ImGuizmo only lets a handle be
    // grabbed while no item is hovered (CanActivate checks IsAnyItemHovered),
    // which is also why this must not be an InvisibleButton. IsItemHovered()
    // still works on both for the orbit/zoom handling.
    bool showedPreview = false;
    if (previewValid_ && preview_ && !preview_->image().empty()) {
        const cv::Mat& img = preview_->image();
        if (previewTexture_ == 0) {
            GLuint id = 0;
            glGenTextures(1, &id);
            previewTexture_ = id;
            glBindTexture(GL_TEXTURE_2D, previewTexture_);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        } else {
            glBindTexture(GL_TEXTURE_2D, previewTexture_);
        }
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        if (img.cols != previewTexW_ || img.rows != previewTexH_) {
            previewTexW_ = img.cols;
            previewTexH_ = img.rows;
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, previewTexW_, previewTexH_,
                         0, GL_RGBA, GL_UNSIGNED_BYTE, img.data);
        } else {
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, previewTexW_, previewTexH_,
                            GL_RGBA, GL_UNSIGNED_BYTE, img.data);
        }
        ImGui::Image(reinterpret_cast<ImTextureID>(
                         static_cast<std::uintptr_t>(previewTexture_)),
                     canvasSize);
        showedPreview = true;
    } else {
        drawList->AddRectFilled(canvasPos,
                                ImVec2(canvasPos.x + canvasSize.x,
                                       canvasPos.y + canvasSize.y),
                                IM_COL32(26, 28, 33, 255), 4.0f);
        ImGui::Dummy(canvasSize);
    }

    // Orbit / zoom the viewport camera.
    if (ImGui::IsItemHovered()) {
        const ImGuiIO& io = ImGui::GetIO();
        orbitDistance_ = std::clamp(orbitDistance_ - io.MouseWheel * 0.3f,
                                    0.5f, 20.0f);
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
            orbitYawDeg_ += io.MouseDelta.x * 0.4f;
            orbitPitchDeg_ =
                std::clamp(orbitPitchDeg_ + io.MouseDelta.y * 0.4f, -85.0f, 85.0f);
        }
    }

    const Eigen::Vector3f eye = eyePosition();
    const Eigen::Matrix4f view =
        lookAt(eye, Eigen::Vector3f::Zero(), Eigen::Vector3f::UnitY());
    const Eigen::Matrix4f proj = perspective(
        kViewportFovYDeg * kDegToRad, canvasSize.x / canvasSize.y, 0.05f, 100.0f);
    const Eigen::Matrix4f viewProj = proj * view;

    // Keep manipulating one cached matrix for the whole drag instead of
    // rebuilding it from the decomposed Euler angles every frame: Euler
    // decomposition is not unique, and feeding a re-decomposed matrix back
    // into an active drag makes the gizmo snap at representation boundaries.
    if (!ImGuizmo::IsUsing()) {
        gizmoMatrix_ = working_.toMatrix();
    }

    if (!showedPreview) {
        // Schematic fallback: image-plane outline, axes and a proxy cube.
        const float halfH = 0.5f * imagePlaneAspect();
        const Eigen::Vector3f corners[4] = {{-0.5f, halfH, 0.0f},
                                            {0.5f, halfH, 0.0f},
                                            {0.5f, -halfH, 0.0f},
                                            {-0.5f, -halfH, 0.0f}};
        ImVec2 screen[4];
        bool visible = true;
        for (int i = 0; i < 4; ++i) {
            visible &= projectToScreen(viewProj, corners[i], canvasPos,
                                       canvasSize, screen[i]);
        }
        if (visible) {
            drawList->AddQuadFilled(screen[0], screen[1], screen[2], screen[3],
                                    IM_COL32(90, 140, 200, 40));
            drawList->AddQuad(screen[0], screen[1], screen[2], screen[3],
                              IM_COL32(120, 170, 230, 180), 1.5f);
        }
        // Image-plane axes: X red, Y green (matches the tracker's frame).
        ImVec2 origin;
        ImVec2 axisEnd;
        if (projectToScreen(viewProj, Eigen::Vector3f::Zero(), canvasPos,
                            canvasSize, origin)) {
            if (projectToScreen(viewProj, {0.25f, 0.0f, 0.0f}, canvasPos,
                                canvasSize, axisEnd)) {
                drawList->AddLine(origin, axisEnd, IM_COL32(230, 90, 90, 200),
                                  2.0f);
            }
            if (projectToScreen(viewProj, {0.0f, 0.25f, 0.0f}, canvasPos,
                                canvasSize, axisEnd)) {
                drawList->AddLine(origin, axisEnd, IM_COL32(90, 210, 90, 200),
                                  2.0f);
            }
        }
        // The model proxy reflects the live matrix, so scale edits show.
        drawModelProxy(drawList, viewProj, gizmoMatrix_, canvasPos, canvasSize);
    }

    ImGuizmo::SetOrthographic(false);
    ImGuizmo::SetDrawlist(drawList);
    ImGuizmo::SetRect(canvasPos.x, canvasPos.y, canvasSize.x, canvasSize.y);

    ImGuizmo::OPERATION op = ImGuizmo::TRANSLATE;
    // Translation happens along the image plane's axes (the parent frame);
    // rotation/scale act in the model's local frame.
    ImGuizmo::MODE mode = ImGuizmo::WORLD;
    if (gizmoOperation_ == 1) {
        op = ImGuizmo::ROTATE;
        mode = ImGuizmo::LOCAL;
    } else if (gizmoOperation_ == 2) {
        op = ImGuizmo::SCALE;
        mode = ImGuizmo::LOCAL; // ImGuizmo scales locally regardless
    }

    if (ImGuizmo::Manipulate(view.data(), proj.data(), op, mode,
                             gizmoMatrix_.data())) {
        working_ = decomposeToTransform(gizmoMatrix_);
        dirty_ = true;
    }
}

void ConfigureWindow::save() {
    if (activeAssignment_ != kInvalidId &&
        store_->setTransform(activeAssignment_, working_)) {
        dirty_ = false;
        if (onSaved_) {
            onSaved_(activeAssignment_);
        }
    }
}

void ConfigureWindow::revert() {
    openAssignment(activeAssignment_);
}

} // namespace avb
