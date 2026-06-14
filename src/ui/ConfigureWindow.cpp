#include "ui/ConfigureWindow.h"

#include <utility>

#include <imgui.h>

#include "storage/DataStore.h"

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

ConfigureWindow::ConfigureWindow(std::shared_ptr<DataStore> store)
    : Window("Configure", 480, 520), store_(std::move(store)) {}

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

void ConfigureWindow::drawUi() {
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
    ImGui::Separator();

    // Editable controls bound to the working copy. Any change marks it dirty.
    bool changed = false;
    changed |= ImGui::DragFloat3("Translation (x,y,z)",
                                 working_.translation.data(), 0.01f);
    changed |= ImGui::DragFloat3("Rotation (deg)",
                                 working_.rotationEulerDeg.data(), 0.5f);
    changed |= ImGui::DragFloat("Scale", &working_.scale, 0.01f, 0.001f, 1000.0f);
    dirty_ = dirty_ || changed;

    ImGui::Spacing();
    ImGui::TextDisabled("Model matrix preview");
    const Eigen::Matrix4f m = working_.toMatrix();
    for (int r = 0; r < 4; ++r) {
        ImGui::Text("% .3f  % .3f  % .3f  % .3f", m(r, 0), m(r, 1), m(r, 2),
                    m(r, 3));
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
