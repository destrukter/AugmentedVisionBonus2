#include "ui/ConfigureWindow.h"

#include <utility>

#include "storage/DataStore.h"

// #include <imgui.h>

namespace avb {

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
    // ImGui::Begin("Configure");
    if (activeAssignment_ == kInvalidId) {
        // ImGui::TextDisabled("Select 'Configure' on an assignment in the Upload window.");
        // ImGui::End();
        return;
    }

    // Editable controls bound to the working copy. Any change sets dirty_.
    //
    //   dirty_ |= ImGui::DragFloat3("Translation (x,y,z)", working_.translation.data(), 0.01f);
    //   dirty_ |= ImGui::DragFloat3("Rotation (deg)",      working_.rotationEulerDeg.data(), 0.5f);
    //   dirty_ |= ImGui::DragFloat("Scale",                &working_.scale, 0.01f, 0.001f, 1000.0f);
    //
    //   // Live preview of the resulting 4x4 matrix:
    //   Eigen::Matrix4f m = working_.toMatrix();
    //   ImGui::Text("Model matrix preview:");
    //   for (int r = 0; r < 4; ++r)
    //       ImGui::Text("% .3f % .3f % .3f % .3f", m(r,0), m(r,1), m(r,2), m(r,3));
    //
    //   if (ImGui::Button("Save")) save();
    //   ImGui::SameLine();
    //   if (ImGui::Button("Revert")) revert();
    //
    // ImGui::End();
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
