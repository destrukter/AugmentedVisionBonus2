#include "ui/UploadWindow.h"

#include <utility>

#include "storage/DataStore.h"

// #include <imgui.h>

namespace avb {

UploadWindow::UploadWindow(std::shared_ptr<DataStore> store,
                           ConfigureCallback onConfigure)
    : Window("Upload", 900, 600),
      store_(std::move(store)),
      onConfigure_(std::move(onConfigure)) {}

void UploadWindow::drawUi() {
    // ImGui::Begin("Upload");
    drawUploadSection();
    // ImGui::Separator();
    drawAssignmentSection();
    // ImGui::End();
}

void UploadWindow::drawUploadSection() {
    // Pseudocode for the intended ImGui interactions:
    //
    // if (ImGui::Button("Upload image...")) {
    //     std::string path = openFileDialog({"png", "jpg", "jpeg", "bmp"});
    //     if (!path.empty()) store_->addImage(path);
    // }
    // ImGui::SameLine();
    // if (ImGui::Button("Upload FBX model...")) {
    //     std::string path = openFileDialog({"fbx"});
    //     if (!path.empty()) store_->addModel(path);
    // }
    //
    // Then list store_->imageIds() and store_->modelIds() as selectable rows,
    // updating selectedImage_ / selectedModel_.
}

void UploadWindow::drawAssignmentSection() {
    // Pseudocode for the assignment interactions:
    //
    // Assign the currently selected model to the currently selected image:
    //   if (ImGui::Button("Assign selected model -> selected image")) {
    //       store_->assign(selectedModel_, selectedImage_);
    //   }
    //
    // For the selected image, list its assignments. Each row offers Configure
    // and Revert; one model can appear under several images at once:
    //   for (Id aid : store_->assignmentsForImage(selectedImage_)) {
    //       const Assignment* a = store_->assignment(aid);
    //       const ModelAsset* m = store_->model(a->modelId);
    //       ImGui::Text("%s", m ? m->name.c_str() : "<missing>");
    //       ImGui::SameLine();
    //       if (ImGui::Button("Configure")) onConfigure_(aid);  // -> Configure window
    //       ImGui::SameLine();
    //       if (ImGui::Button("Revert"))   store_->unassign(aid);
    //   }
}

} // namespace avb
