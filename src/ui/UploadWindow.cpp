#include "ui/UploadWindow.h"

#include <string>
#include <utility>

#include <SDL.h>
#include <imgui.h>
#include <nfd.h>

#include "render/ModelLoader.h"
#include "storage/DataStore.h"
#include "ui/Panels.h"
#include "vision/ImageTracker.h"

namespace avb {

namespace {

// Below roughly this many ORB features an image is unlikely to ever be
// detected in the feed; uploads under it succeed but get a warning.
constexpr int kLowFeatureThreshold = 60;

std::string fileNameOf(const std::string& path) {
    const auto slash = path.find_last_of("/\\");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

// Opens a native "Open File" dialog restricted to `filter` (e.g. "png,jpg").
// Returns the chosen path, or empty if the user cancelled or the dialog
// failed (logged via SDL so failures aren't silent).
std::string pickFile(const char* filterName, const char* filterExtensions) {
    nfdchar_t* path = nullptr;
    const nfdfilteritem_t filter[1] = {{filterName, filterExtensions}};
    const nfdresult_t result = NFD_OpenDialog(&path, filter, 1, nullptr);
    if (result == NFD_OKAY) {
        std::string picked = path;
        NFD_FreePath(path);
        return picked;
    }
    if (result == NFD_ERROR) {
        SDL_Log("NFD_OpenDialog failed: %s", NFD_GetError());
    }
    return {}; // NFD_CANCEL, or NFD_ERROR already logged above
}

} // namespace

UploadWindow::UploadWindow(std::shared_ptr<DataStore> store,
                           ConfigureCallback onConfigure,
                           SaveSessionCallback onSaveSession)
    : Window("Upload", 900, 600),
      store_(std::move(store)),
      onConfigure_(std::move(onConfigure)),
      onSaveSession_(std::move(onSaveSession)) {}

void UploadWindow::drawUi() {
    // Scrolling allowed: the asset/assignment lists have fixed-size rows and
    // may legitimately overflow a small window.
    beginFullWindow("Upload", /*allowScroll=*/true);
    drawUploadSection();
    ImGui::Separator();
    drawAssignmentSection();
    ImGui::End();
}

void UploadWindow::drawUploadSection() {
    ImGui::TextUnformatted("Upload assets");

    ImGui::InputTextWithHint("##imgpath", "optional: paste a path, or leave empty to browse",
                             imagePathBuf_, sizeof(imagePathBuf_));
    ImGui::SameLine();
    if (ImGui::Button("Add image...")) {
        std::string path = imagePathBuf_;
        if (path.empty()) {
            path = pickFile("Images", "png,jpg,jpeg,bmp");
        }
        if (!path.empty()) {
            uploadImage(path);
            imagePathBuf_[0] = '\0';
        }
    }

    ImGui::InputTextWithHint("##modelpath", "optional: paste a path, or leave empty to browse",
                             modelPathBuf_, sizeof(modelPathBuf_));
    ImGui::SameLine();
    if (ImGui::Button("Add 3D model...")) {
        std::string path = modelPathBuf_;
        if (path.empty()) {
            // NFD takes a comma-separated extension list for one filter entry.
            path = pickFile("3D models", "fbx,obj");
        }
        if (!path.empty()) {
            uploadModel(path);
            modelPathBuf_[0] = '\0';
        }
    }

    if (onSaveSession_) {
        if (ImGui::Button("Save session to library")) {
            onSaveSession_();
        }
        ImGui::SameLine();
        ImGui::TextDisabled(
            "(copies external files into the library and writes all "
            "assignments + poses to assignments.cfg)");
    }

    if (!statusMessage_.empty()) {
        ImVec4 color;
        switch (statusKind_) {
            case StatusKind::Success: color = ImVec4(0.4f, 0.9f, 0.4f, 1.0f); break;
            case StatusKind::Warning: color = ImVec4(1.0f, 0.8f, 0.2f, 1.0f); break;
            case StatusKind::Error:   color = ImVec4(1.0f, 0.4f, 0.4f, 1.0f); break;
        }
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::TextWrapped("%s", statusMessage_.c_str());
        ImGui::PopStyleColor();
    }

    ImGui::Spacing();
    ImGui::Columns(2, "assets");

    ImGui::TextDisabled("Images");
    for (const Id id : store_->imageIds()) {
        const ImageAsset* img = store_->image(id);
        if (!img) {
            continue;
        }
        const std::string name = img->name;
        ImGui::PushID(static_cast<int>(id));
        if (ImGui::SmallButton("x")) {
            // Removes the image and (cascading) its assignments; "Save
            // session to library" syncs the removal to disk.
            if (selectedImage_ == id) {
                selectedImage_ = kInvalidId;
            }
            store_->removeImage(id);
            setStatus(StatusKind::Success, "Removed image '" + name + "'.");
            ImGui::PopID();
            continue;
        }
        ImGui::SameLine();
        if (ImGui::Selectable(name.c_str(), selectedImage_ == id)) {
            selectedImage_ = id;
        }
        ImGui::PopID();
    }

    ImGui::NextColumn();

    ImGui::TextDisabled("3D models");
    for (const Id id : store_->modelIds()) {
        const ModelAsset* model = store_->model(id);
        if (!model) {
            continue;
        }
        const std::string name = model->name;
        ImGui::PushID(static_cast<int>(id));
        if (ImGui::SmallButton("x")) {
            if (selectedModel_ == id) {
                selectedModel_ = kInvalidId;
            }
            store_->removeModel(id);
            setStatus(StatusKind::Success, "Removed model '" + name + "'.");
            ImGui::PopID();
            continue;
        }
        ImGui::SameLine();
        if (ImGui::Selectable(name.c_str(), selectedModel_ == id)) {
            selectedModel_ = id;
        }
        ImGui::PopID();
    }

    ImGui::Columns(1);
}

void UploadWindow::uploadImage(const std::string& path) {
    const Id id = store_->addImage(path);
    // Decode now so the tracker has a template to match against - and so a
    // broken upload is caught here, with feedback, instead of silently never
    // tracking.
    if (!store_->loadImagePixels(id)) {
        store_->removeImage(id);
        setStatus(StatusKind::Error,
                  "Could not load image '" + fileNameOf(path) +
                      "': the file is missing, unreadable or not a supported "
                      "image format.");
        return;
    }
    const ImageAsset* img = store_->image(id);
    const int features =
        img ? ImageTracker::countTrackableFeatures(img->pixels) : 0;
    if (features < kLowFeatureThreshold) {
        setStatus(StatusKind::Warning,
                  "Image '" + fileNameOf(path) + "' uploaded, but it has few "
                  "distinctive features (" + std::to_string(features) +
                  ") and may not track reliably. Prefer detailed, high-contrast "
                  "images.");
    } else {
        setStatus(StatusKind::Success,
                  "Image '" + fileNameOf(path) + "' uploaded (" +
                      std::to_string(features) + " trackable features).");
    }
}

void UploadWindow::uploadModel(const std::string& path) {
    std::string error;
    if (!ModelLoader::validateModelFile(path, &error)) {
        setStatus(StatusKind::Error, "Could not load model '" +
                                         fileNameOf(path) + "': " + error);
        return;
    }
    store_->addModel(path);
    setStatus(StatusKind::Success,
              "Model '" + fileNameOf(path) + "' uploaded.");
}

void UploadWindow::setStatus(StatusKind kind, std::string message) {
    statusKind_ = kind;
    statusMessage_ = std::move(message);
}

void UploadWindow::drawAssignmentSection() {
    ImGui::TextUnformatted("Assignments");

    const bool canAssign =
        selectedImage_ != kInvalidId && selectedModel_ != kInvalidId;
    ImGui::BeginDisabled(!canAssign);
    if (ImGui::Button("Assign selected model -> selected image")) {
        // One model can be assigned to many images - and to the same image
        // several times: every click adds another independent copy with its
        // own pose.
        store_->assign(selectedModel_, selectedImage_);
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("(assigning again adds another copy)");

    if (selectedImage_ == kInvalidId) {
        ImGui::TextDisabled("Select an image to see its assigned models.");
        return;
    }

    const ImageAsset* img = store_->image(selectedImage_);
    ImGui::Text("Models on '%s':", img ? img->name.c_str() : "<image>");
    // One Configure button per picture: the Configure window shows all of the
    // image's models and switches between them with its own dropdown.
    const std::vector<Id> assignments =
        store_->assignmentsForImage(selectedImage_);
    ImGui::SameLine();
    ImGui::BeginDisabled(assignments.empty());
    if (ImGui::Button("Configure")) {
        onConfigure_(selectedImage_); // hand the image to the Configure window
    }
    ImGui::EndDisabled();
    if (assignments.empty()) {
        ImGui::TextDisabled("(no models assigned yet)");
    }

    // Labels carry an instance number when the same model is assigned to the
    // image more than once (matching the Configure window's dropdown).
    const auto labels = assignmentDisplayLabels(*store_, assignments);
    for (const Id aid : assignments) {
        const auto label = labels.find(aid);
        if (label == labels.end()) {
            continue;
        }
        ImGui::PushID(static_cast<int>(aid));
        ImGui::BulletText("%s", label->second.c_str());
        ImGui::SameLine();
        if (ImGui::Button("Revert")) {
            store_->unassign(aid);
        }
        ImGui::PopID();
    }
}

} // namespace avb
