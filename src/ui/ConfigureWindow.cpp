#include "ui/ConfigureWindow.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <imgui.h>
#include <ImGuizmo.h>

#include <Eigen/Geometry>

#include "storage/Assets.h"
#include "storage/DataStore.h"

namespace avb {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;

void beginFullWindow(const char* name) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::Begin(name, nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);
}

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
/// Transform::rotationMatrix's Rz*Ry*Rx convention) / uniform scale from a
/// manipulated 4x4 matrix back into a Transform.
Transform decomposeToTransform(const Eigen::Matrix4f& m) {
    Transform t;
    t.translation = m.block<3, 1>(0, 3);

    Eigen::Matrix3f rs = m.block<3, 3>(0, 0);
    const float sx = rs.col(0).norm();
    const float sy = rs.col(1).norm();
    const float sz = rs.col(2).norm();
    t.scale = std::max((sx + sy + sz) / 3.0f, 1e-4f); // uniform scale by design
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

} // namespace

ConfigureWindow::ConfigureWindow(std::shared_ptr<DataStore> store)
    : Window("Configure", 560, 680), store_(std::move(store)) {}

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

    beginFullWindow("Configure");

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

    ImGui::RadioButton("Move", &gizmoOperation_, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Rotate", &gizmoOperation_, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Scale", &gizmoOperation_, 2);
    ImGui::SameLine();
    ImGui::TextDisabled("(drag handles; right-drag orbits, wheel zooms)");

    drawGizmoViewport();

    ImGui::Separator();

    // Numeric fallback bound to the same working copy; any change marks it
    // dirty. Kept alongside the gizmo for exact values.
    bool changed = false;
    changed |= ImGui::DragFloat3("Translation (x,y,z)",
                                 working_.translation.data(), 0.01f);
    changed |= ImGui::DragFloat3("Rotation (deg)",
                                 working_.rotationEulerDeg.data(), 0.5f);
    changed |= ImGui::DragFloat("Scale", &working_.scale, 0.01f, 0.001f, 1000.0f);
    dirty_ = dirty_ || changed;

    if (ImGui::TreeNode("Model matrix")) {
        const Eigen::Matrix4f m = working_.toMatrix();
        for (int r = 0; r < 4; ++r) {
            ImGui::Text("% .3f  % .3f  % .3f  % .3f", m(r, 0), m(r, 1), m(r, 2),
                        m(r, 3));
        }
        ImGui::TreePop();
    }

    ImGui::Spacing();
    if (ImGui::Button("Save")) {
        save();
    }
    ImGui::SameLine();
    if (ImGui::Button("Revert")) {
        revert();
    }

    ImGui::End();
}

void ConfigureWindow::drawGizmoViewport() {
    // Reserve the viewport, leaving room for the numeric controls below.
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 canvasSize(std::max(avail.x, 120.0f),
                            std::max(200.0f, avail.y - 190.0f));
    const ImVec2 canvasPos = ImGui::GetCursorScreenPos();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    drawList->AddRectFilled(
        canvasPos, ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y),
        IM_COL32(26, 28, 33, 255), 4.0f);

    ImGui::InvisibleButton("##gizmo_viewport", canvasSize);

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

    const float yaw = orbitYawDeg_ * kDegToRad;
    const float pitch = orbitPitchDeg_ * kDegToRad;
    const Eigen::Vector3f eye =
        orbitDistance_ * Eigen::Vector3f(std::cos(pitch) * std::sin(yaw),
                                         std::sin(pitch),
                                         std::cos(pitch) * std::cos(yaw));
    const Eigen::Matrix4f view =
        lookAt(eye, Eigen::Vector3f::Zero(), Eigen::Vector3f::UnitY());
    const Eigen::Matrix4f proj = perspective(
        45.0f * kDegToRad, canvasSize.x / canvasSize.y, 0.05f, 100.0f);
    const Eigen::Matrix4f viewProj = proj * view;

    // Reference: the tracked image's plane (width 1, matching the tracker's
    // convention; models are placed relative to this quad).
    const float halfH = 0.5f * imagePlaneAspect();
    const Eigen::Vector3f corners[4] = {{-0.5f, halfH, 0.0f},
                                        {0.5f, halfH, 0.0f},
                                        {0.5f, -halfH, 0.0f},
                                        {-0.5f, -halfH, 0.0f}};
    ImVec2 screen[4];
    bool visible = true;
    for (int i = 0; i < 4; ++i) {
        visible &= projectToScreen(viewProj, corners[i], canvasPos, canvasSize,
                                   screen[i]);
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
    if (projectToScreen(viewProj, Eigen::Vector3f::Zero(), canvasPos, canvasSize,
                        origin)) {
        if (projectToScreen(viewProj, {0.25f, 0.0f, 0.0f}, canvasPos, canvasSize,
                            axisEnd)) {
            drawList->AddLine(origin, axisEnd, IM_COL32(230, 90, 90, 200), 2.0f);
        }
        if (projectToScreen(viewProj, {0.0f, 0.25f, 0.0f}, canvasPos, canvasSize,
                            axisEnd)) {
            drawList->AddLine(origin, axisEnd, IM_COL32(90, 210, 90, 200), 2.0f);
        }
    }

    // The gizmo itself, manipulating the working transform in place.
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

    Eigen::Matrix4f model = working_.toMatrix();
    if (ImGuizmo::Manipulate(view.data(), proj.data(), op, mode, model.data())) {
        working_ = decomposeToTransform(model);
        dirty_ = true;
    }
}

void ConfigureWindow::save() {
    if (activeAssignment_ != kInvalidId &&
        store_->setTransform(activeAssignment_, working_)) {
        dirty_ = false;
    }
}

void ConfigureWindow::revert() {
    openAssignment(activeAssignment_);
}

} // namespace avb
