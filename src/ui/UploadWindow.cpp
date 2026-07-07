#include "ui/UploadWindow.h"

#include <string>
#include <utility>

#include <SDL.h>
#include <imgui.h>
#include <nfd.h>

#include "render/ModelLoader.h"
#include "storage/DataStore.h"
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

// Makes the next ImGui window fill the whole OS window (each OS window hosts a
// single top-level panel).
void beginFullWindow(const char* name) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::Begin(name, nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);
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
                           ConfigureCallback onConfigure)
    : Window("Upload", 900, 600),
      store_(std::move(store)),
      onConfigure_(std::move(onConfigure)) {}

void UploadWindow::drawUi() {
    beginFullWindow("Upload");
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

    ImGui::InputTextWithHint("##fbxpath", "optional: paste a path, or leave empty to browse",
                             modelPathBuf_, sizeof(modelPathBuf_));
    ImGui::SameLine();
    if (ImGui::Button("Add FBX model...")) {
        std::string path = modelPathBuf_;
        if (path.empty()) {
            path = pickFile("FBX models", "fbx");
        }
        if (!path.empty()) {
            uploadModel(path);
            modelPathBuf_[0] = '\0';
        }
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
        if (ImGui::Selectable(img->name.c_str(), selectedImage_ == id)) {
            selectedImage_ = id;
        }
    }

    ImGui::NextColumn();

    ImGui::TextDisabled("FBX models");
    for (const Id id : store_->modelIds()) {
        const ModelAsset* model = store_->model(id);
        if (!model) {
            continue;
        }
        if (ImGui::Selectable(model->name.c_str(), selectedModel_ == id)) {
            selectedModel_ = id;
        }
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
        // One model can be assigned to many images; assign() is idempotent per
        // (model, image) pair.
        store_->assign(selectedModel_, selectedImage_);
    }
    ImGui::EndDisabled();

    if (selectedImage_ == kInvalidId) {
        ImGui::TextDisabled("Select an image to see its assigned models.");
        return;
    }

    const ImageAsset* img = store_->image(selectedImage_);
    ImGui::Text("Models on '%s':", img ? img->name.c_str() : "<image>");

    for (const Id aid : store_->assignmentsForImage(selectedImage_)) {
        const Assignment* a = store_->assignment(aid);
        if (!a) {
            continue;
        }
        const ModelAsset* model = store_->model(a->modelId);
        ImGui::PushID(static_cast<int>(aid));
        ImGui::BulletText("%s", model ? model->name.c_str() : "<missing>");
        ImGui::SameLine();
        if (ImGui::Button("Configure")) {
            onConfigure_(aid); // hand off to the Configure window
        }
        ImGui::SameLine();
        if (ImGui::Button("Revert")) {
            store_->unassign(aid);
        }
        ImGui::PopID();
    }
}

} // namespace avb
