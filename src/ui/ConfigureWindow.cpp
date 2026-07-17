#include "ui/ConfigureWindow.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
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
    if (!preview_ || !isOpen() || activeImage_ == kInvalidId || canvasW_ < 16 ||
        canvasH_ < 16) {
        return;
    }
    // Every model assigned to the image renders at its working pose, so
    // relative placement of multiple models is visible while editing any one
    // of them.
    std::vector<ConfigurePreview::ModelPose> models;
    for (const Id aid : store_->assignmentsForImage(activeImage_)) {
        const Assignment* a = store_->assignment(aid);
        if (!a) {
            continue;
        }
        const WorkingState& w = workingFor(aid);
        models.push_back(
            {a->modelId, w.origin.toMatrix() * w.transform.toMatrix()});
    }
    previewValid_ = preview_->render(activeImage_, models, eyePosition(),
                                     kViewportFovYDeg, canvasW_, canvasH_);
}

void ConfigureWindow::openImage(Id imageId) {
    activeImage_ = store_->image(imageId) ? imageId : kInvalidId;
    selectedAssignment_ = kInvalidId;
    working_.clear();
    refreshAssignments();
}

std::vector<Id> ConfigureWindow::refreshAssignments() {
    // The image (or single assignments) may vanish behind us - removed or
    // reverted in the Upload window.
    if (activeImage_ != kInvalidId && store_->image(activeImage_) == nullptr) {
        activeImage_ = kInvalidId;
    }
    std::vector<Id> assignments;
    if (activeImage_ != kInvalidId) {
        assignments = store_->assignmentsForImage(activeImage_);
    }
    for (auto it = working_.begin(); it != working_.end();) {
        if (std::find(assignments.begin(), assignments.end(), it->first) ==
            assignments.end()) {
            it = working_.erase(it); // assignment gone: drop its working copy
        } else {
            ++it;
        }
    }
    if (std::find(assignments.begin(), assignments.end(),
                  selectedAssignment_) == assignments.end()) {
        selectedAssignment_ =
            assignments.empty() ? kInvalidId : assignments.front();
    }
    return assignments;
}

ConfigureWindow::WorkingState& ConfigureWindow::workingFor(Id assignmentId) {
    if (const auto it = working_.find(assignmentId); it != working_.end()) {
        return it->second;
    }
    WorkingState w;
    if (const auto t = store_->transform(assignmentId)) {
        w.transform = *t;
    }
    if (const auto o = store_->origin(assignmentId)) {
        w.origin = *o;
    }
    return working_.emplace(assignmentId, w).first->second;
}

ConfigureWindow::WorkingState* ConfigureWindow::selectedWorking() {
    if (selectedAssignment_ == kInvalidId ||
        store_->assignment(selectedAssignment_) == nullptr) {
        return nullptr;
    }
    return &workingFor(selectedAssignment_);
}

float ConfigureWindow::imagePlaneAspect() const {
    const ImageAsset* img = store_->image(activeImage_);
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

    const std::vector<Id> assignments = refreshAssignments();
    if (activeImage_ == kInvalidId) {
        ImGui::TextDisabled(
            "Click 'Configure' on an image in the Upload window.");
        ImGui::End();
        return;
    }

    const ImageAsset* img = store_->image(activeImage_);
    ImGui::Text("Editing image '%s'", img ? img->name.c_str() : "<missing>");
    int dirtyCount = 0;
    for (const Id aid : assignments) {
        const auto it = working_.find(aid);
        if (it != working_.end() && it->second.dirty) {
            ++dirtyCount;
        }
    }
    if (dirtyCount > 0) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "(%d unsaved)",
                           dirtyCount);
    }

    if (assignments.empty()) {
        ImGui::TextDisabled(
            "No models assigned to this image - assign one in the Upload "
            "window.");
        ImGui::Separator();
        drawGizmoViewport(nullptr); // still show the picture itself
        ImGui::End();
        return;
    }

    // Model selector: which of the image's models the fields/gizmo edit.
    // Every model stays visible in the viewport; unsaved ones are marked *.
    // Labels carry an instance number when the same model is assigned to the
    // image more than once (matching the Upload window's assignment list).
    const auto labels = assignmentDisplayLabels(*store_, assignments);
    const auto selectedLabel = labels.find(selectedAssignment_);
    ImGui::SetNextItemWidth(280.0f);
    if (ImGui::BeginCombo("Model", selectedLabel != labels.end()
                                       ? selectedLabel->second.c_str()
                                       : "<missing>")) {
        for (const Id aid : assignments) {
            const auto it = working_.find(aid);
            const bool entryDirty = it != working_.end() && it->second.dirty;
            const auto name = labels.find(aid);
            const std::string label =
                (name != labels.end() ? name->second : "<missing>") +
                (entryDirty ? " *" : "") + "##" + std::to_string(aid);
            if (ImGui::Selectable(label.c_str(), aid == selectedAssignment_)) {
                selectedAssignment_ = aid;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::Separator();

    WorkingState* w = selectedWorking();
    if (!w) { // selection raced a removal this frame; recover next frame
        ImGui::End();
        return;
    }

    // Numeric controls bound to the selected model's working copy; the gizmo
    // below edits the same values interactively.
    bool changed = false;
    changed |= ImGui::DragFloat3("Translation (x,y,z)",
                                 w->transform.translation.data(), 0.01f);
    changed |= ImGui::DragFloat3("Rotation (deg)",
                                 w->transform.rotationEulerDeg.data(), 0.5f);
    changed |= ImGui::DragFloat3("Scale (x,y,z)", w->transform.scale.data(),
                                 0.01f, 0.001f, 1000.0f);
    w->dirty = w->dirty || changed;

    if (!w->origin.isIdentity()) {
        ImGui::TextDisabled("Origin: t=(%.3f, %.3f, %.3f)  r=(%.1f, %.1f, %.1f)",
                            w->origin.translation.x(),
                            w->origin.translation.y(),
                            w->origin.translation.z(),
                            w->origin.rotationEulerDeg.x(),
                            w->origin.rotationEulerDeg.y(),
                            w->origin.rotationEulerDeg.z());
        ImGui::SameLine();
        if (ImGui::SmallButton("fold back")) {
            // Undo of "Set origin here": move the origin back into the
            // editable values. The model does not move.
            w->transform = Transform::fromMatrix(w->origin.toMatrix() *
                                                 w->transform.toMatrix());
            w->origin = Transform{};
            w->dirty = true;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "Moves the origin back into the translation/rotation fields "
                "(the model stays where it is).");
        }
    }

    ImGui::TextDisabled("Model matrix preview");
    const Eigen::Matrix4f preview =
        w->origin.toMatrix() * w->transform.toMatrix();
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
        w = selectedWorking(); // the working copies were reloaded
        if (!w) {
            ImGui::End();
            return;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Set origin here")) {
        setOriginToCurrent();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Makes the model's current position/rotation its new origin: the "
            "model stays put, translation and rotation reset to 0, and "
            "further edits are relative to this origin.");
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

    drawGizmoViewport(w);

    ImGui::End();
}

void ConfigureWindow::drawGizmoViewport(WorkingState* selected) {
    // The viewport takes all remaining panel space below the controls. Its
    // size is recorded for the next updatePreviewRender() call (1-frame lag).
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 canvasSize(std::max(avail.x, 120.0f), std::max(avail.y, 120.0f));
    canvasW_ = static_cast<int>(canvasSize.x);
    canvasH_ = static_cast<int>(canvasSize.y);
    const ImVec2 canvasPos = ImGui::GetCursorScreenPos();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    // Background: the off-screen render of the actual image + all its models
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

    // The gizmo manipulates the selected model's editable transform in the
    // *origin's* frame: the origin matrix is folded into the view matrix
    // passed to ImGuizmo below, so after "Set origin here" the handle sits on
    // the model (which IS the new origin at that moment), the translate
    // arrows align with the origin's axes, and dragging one arrow changes
    // exactly one translation field. Keep manipulating one cached matrix for
    // the whole drag instead of rebuilding it from the decomposed Euler
    // angles every frame: Euler decomposition is not unique, and feeding a
    // re-decomposed matrix back into an active drag makes the handles snap
    // at representation boundaries.
    const Eigen::Matrix4f originMatrix =
        selected ? selected->origin.toMatrix() : Eigen::Matrix4f::Identity();
    if (!ImGuizmo::IsUsing()) {
        if (selected) {
            gizmoMatrix_ = selected->transform.toMatrix();
        } else {
            gizmoMatrix_ = Eigen::Matrix4f::Identity();
        }
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
        if (selected) {
            drawModelProxy(drawList, viewProj, originMatrix * gizmoMatrix_,
                           canvasPos, canvasSize);
        }
    }

    // Mark the origin while one is set: an axes triad (X red, Y green,
    // Z blue) at the origin pose, so the reference point "Set origin here"
    // created stays visible when the model is later moved away from it. The
    // gizmo handle itself always follows the model.
    if (selected && !selected->origin.isIdentity()) {
        constexpr float kAxisLen = 0.15f;
        const Eigen::Vector3f o = originMatrix.block<3, 1>(0, 3);
        const Eigen::Vector3f axes[3] = {
            (originMatrix * Eigen::Vector4f(kAxisLen, 0, 0, 1)).head<3>(),
            (originMatrix * Eigen::Vector4f(0, kAxisLen, 0, 1)).head<3>(),
            (originMatrix * Eigen::Vector4f(0, 0, kAxisLen, 1)).head<3>()};
        static constexpr ImU32 kAxisColors[3] = {
            IM_COL32(230, 90, 90, 230), IM_COL32(90, 210, 90, 230),
            IM_COL32(90, 130, 230, 230)};
        ImVec2 screenOrigin;
        if (projectToScreen(viewProj, o, canvasPos, canvasSize, screenOrigin)) {
            for (int i = 0; i < 3; ++i) {
                ImVec2 screenEnd;
                if (projectToScreen(viewProj, axes[i], canvasPos, canvasSize,
                                    screenEnd)) {
                    drawList->AddLine(screenOrigin, screenEnd, kAxisColors[i],
                                      2.0f);
                }
            }
            drawList->AddCircleFilled(screenOrigin, 3.5f,
                                      IM_COL32(255, 255, 255, 230));
            drawList->AddText(ImVec2(screenOrigin.x + 6.0f, screenOrigin.y - 16.0f),
                              IM_COL32(255, 255, 255, 200), "origin");
        }
    }

    if (!selected) {
        return; // nothing to manipulate; the viewport still shows the image
    }

    ImGuizmo::SetOrthographic(false);
    ImGuizmo::SetDrawlist(drawList);
    ImGuizmo::SetRect(canvasPos.x, canvasPos.y, canvasSize.x, canvasSize.y);

    ImGuizmo::OPERATION op = ImGuizmo::TRANSLATE;
    // Translation happens along the parent frame's axes - the assignment's
    // origin frame, which is the image plane until "Set origin here" is used;
    // rotation/scale act in the model's local frame.
    ImGuizmo::MODE mode = ImGuizmo::WORLD;
    if (gizmoOperation_ == 1) {
        op = ImGuizmo::ROTATE;
        mode = ImGuizmo::LOCAL;
    } else if (gizmoOperation_ == 2) {
        op = ImGuizmo::SCALE;
        mode = ImGuizmo::LOCAL; // ImGuizmo scales locally regardless
    }

    // Composing the (rigid) origin into the view matrix makes ImGuizmo
    // operate entirely in the origin's frame while the handles still render
    // at the model's on-screen position (view * origin * transform is the
    // same clip-space pose as before).
    const Eigen::Matrix4f gizmoView = view * originMatrix;
    if (ImGuizmo::Manipulate(gizmoView.data(), proj.data(), op, mode,
                             gizmoMatrix_.data())) {
        selected->transform = Transform::fromMatrix(gizmoMatrix_);
        selected->dirty = true;
    }
}

void ConfigureWindow::save() {
    for (const Id aid : store_->assignmentsForImage(activeImage_)) {
        const auto it = working_.find(aid);
        if (it == working_.end() || !it->second.dirty) {
            continue;
        }
        if (store_->setTransform(aid, it->second.transform) &&
            store_->setOrigin(aid, it->second.origin)) {
            it->second.dirty = false;
            if (onSaved_) {
                onSaved_(aid);
            }
        }
    }
}

void ConfigureWindow::revert() {
    working_.clear(); // reloaded lazily from the store
}

void ConfigureWindow::setOriginToCurrent() {
    WorkingState* w = selectedWorking();
    if (!w) {
        return;
    }
    Transform rigid;
    rigid.translation = w->transform.translation;
    rigid.rotationEulerDeg = w->transform.rotationEulerDeg;
    if (rigid.isIdentity()) {
        return; // nothing to fold
    }
    // Fold the current translation/rotation into the origin; the model's full
    // pose (origin * transform) is unchanged, but the editable values now
    // read zero. Scale stays in the editable transform.
    w->origin = Transform::fromMatrix(w->origin.toMatrix() * rigid.toMatrix());
    w->origin.scale = Eigen::Vector3f::Ones(); // rigid by construction
    w->transform.translation.setZero();
    w->transform.rotationEulerDeg.setZero();
    w->dirty = true;
}

} // namespace avb
