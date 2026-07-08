#include "storage/AssetLibrary.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <utility>

#include "storage/Assets.h"
#include "storage/DataStore.h"

namespace avb {

namespace {

namespace fs = std::filesystem;

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return s;
}

std::string trim(const std::string& s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

/// Files in `dir` whose (lowercased) extension is in `extensions`, sorted by
/// name so load order - and thus asset ids - is deterministic.
std::vector<fs::path> listFiles(const fs::path& dir,
                                const std::vector<std::string>& extensions) {
    std::vector<fs::path> files;
    std::error_code ec;
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end;
         it.increment(ec)) {
        std::error_code fec;
        if (!it->is_regular_file(fec)) {
            continue;
        }
        const std::string ext = toLower(it->path().extension().string());
        if (std::find(extensions.begin(), extensions.end(), ext) !=
            extensions.end()) {
            files.push_back(it->path());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

Id findImageByName(const DataStore& store, const std::string& name) {
    const std::string wanted = toLower(name);
    for (const Id id : store.imageIds()) {
        if (const ImageAsset* img = store.image(id)) {
            if (toLower(img->name) == wanted) {
                return id;
            }
        }
    }
    return kInvalidId;
}

Id findModelByName(const DataStore& store, const std::string& name) {
    const std::string wanted = toLower(name);
    for (const Id id : store.modelIds()) {
        if (const ModelAsset* model = store.model(id)) {
            if (toLower(model->name) == wanted) {
                return id;
            }
        }
    }
    return kInvalidId;
}

} // namespace

AssetLibrary::AssetLibrary(std::shared_ptr<DataStore> store,
                           ModelValidator modelValidator)
    : store_(std::move(store)), validateModel_(std::move(modelValidator)) {}

AssetLibrary::Report AssetLibrary::load(const std::string& rootDir) {
    Report report;
    std::error_code ec;
    const fs::path root(rootDir);
    if (!fs::is_directory(root, ec)) {
        return report; // no library on disk: nothing to load
    }

    // Images: validated by decoding, exactly like a manual upload.
    for (const fs::path& file :
         listFiles(root / "images", {".png", ".jpg", ".jpeg", ".bmp"})) {
        const Id id = store_->addImage(file.string());
        if (!store_->loadImagePixels(id)) {
            store_->removeImage(id);
            report.warnings.push_back("image '" + file.filename().string() +
                                      "' could not be decoded, skipped");
            continue;
        }
        ++report.imagesAdded;
    }

    // Models: validated through the injected checker (Assimp in the app).
    for (const fs::path& file : listFiles(root / "models", {".fbx"})) {
        std::string error;
        if (validateModel_ && !validateModel_(file.string(), &error)) {
            report.warnings.push_back("model '" + file.filename().string() +
                                      "' failed to load (" +
                                      (error.empty() ? "unknown error" : error) +
                                      "), skipped");
            continue;
        }
        store_->addModel(file.string());
        ++report.modelsAdded;
    }

    // Explicit name pairs from assignments.cfg.
    const fs::path cfgPath = root / "assignments.cfg";
    if (fs::exists(cfgPath, ec)) {
        std::ifstream cfg(cfgPath);
        std::string line;
        int lineNo = 0;
        while (std::getline(cfg, line)) {
            ++lineNo;
            const std::string stripped = trim(line);
            if (stripped.empty() || stripped[0] == '#') {
                continue;
            }
            const auto eq = stripped.find('=');
            if (eq == std::string::npos) {
                report.warnings.push_back(
                    "assignments.cfg line " + std::to_string(lineNo) +
                    ": expected 'model-file = image-file', skipped");
                continue;
            }
            const std::string modelName = trim(stripped.substr(0, eq));
            const std::string imageName = trim(stripped.substr(eq + 1));
            const Id modelId = findModelByName(*store_, modelName);
            const Id imageId = findImageByName(*store_, imageName);
            if (modelId == kInvalidId || imageId == kInvalidId) {
                report.warnings.push_back(
                    "assignments.cfg line " + std::to_string(lineNo) + ": " +
                    (modelId == kInvalidId ? "model '" + modelName + "'"
                                           : "image '" + imageName + "'") +
                    " not found in the library, skipped");
                continue;
            }
            if (!store_->findAssignment(modelId, imageId)) {
                store_->assign(modelId, imageId);
                ++report.assignmentsCreated;
            }
        }
    }

    // Automatic pairing: same base name (stem) links a model to an image,
    // e.g. dragon.fbx <-> dragon.png.
    for (const Id modelId : store_->modelIds()) {
        const ModelAsset* model = store_->model(modelId);
        if (!model) {
            continue;
        }
        const std::string stem = toLower(fs::path(model->name).stem().string());
        for (const Id imageId : store_->imageIds()) {
            const ImageAsset* img = store_->image(imageId);
            if (!img ||
                toLower(fs::path(img->name).stem().string()) != stem) {
                continue;
            }
            if (!store_->findAssignment(modelId, imageId)) {
                store_->assign(modelId, imageId);
                ++report.assignmentsCreated;
            }
        }
    }

    return report;
}

} // namespace avb
