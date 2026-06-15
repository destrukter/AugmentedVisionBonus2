#include "storage/DataStore.h"

#include <algorithm>

#include <opencv2/imgcodecs.hpp>

namespace avb {

namespace {
std::string fileNameOf(const std::string& path) {
    const auto slash = path.find_last_of("/\\");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}
} // namespace

// ---- Images ---------------------------------------------------------------

Id DataStore::addImage(const std::string& filePath, const std::string& name) {
    ImageAsset asset;
    asset.id = nextId();
    asset.filePath = filePath;
    asset.name = name.empty() ? fileNameOf(filePath) : name;
    const Id id = asset.id;
    images_.emplace(id, std::move(asset));
    return id;
}

bool DataStore::loadImagePixels(Id imageId) {
    const auto it = images_.find(imageId);
    if (it == images_.end()) {
        return false;
    }
    if (!it->second.pixels.empty()) {
        return true; // already decoded
    }
    cv::Mat pixels = cv::imread(it->second.filePath, cv::IMREAD_COLOR);
    if (pixels.empty()) {
        return false; // missing file or unsupported format
    }
    it->second.pixels = std::move(pixels);
    return true;
}

bool DataStore::removeImage(Id imageId) {
    if (images_.erase(imageId) == 0) {
        return false;
    }
    // Drop any assignments that referenced this image.
    for (auto it = assignments_.begin(); it != assignments_.end();) {
        it = (it->second.imageId == imageId) ? assignments_.erase(it) : std::next(it);
    }
    return true;
}

const ImageAsset* DataStore::image(Id imageId) const {
    const auto it = images_.find(imageId);
    return it == images_.end() ? nullptr : &it->second;
}

std::vector<Id> DataStore::imageIds() const {
    std::vector<Id> ids;
    ids.reserve(images_.size());
    for (const auto& [id, _] : images_) {
        ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

// ---- Models ---------------------------------------------------------------

Id DataStore::addModel(const std::string& filePath, const std::string& name) {
    ModelAsset asset;
    asset.id = nextId();
    asset.filePath = filePath;
    asset.name = name.empty() ? fileNameOf(filePath) : name;
    const Id id = asset.id;
    models_.emplace(id, std::move(asset));
    return id;
}

bool DataStore::removeModel(Id modelId) {
    if (models_.erase(modelId) == 0) {
        return false;
    }
    for (auto it = assignments_.begin(); it != assignments_.end();) {
        it = (it->second.modelId == modelId) ? assignments_.erase(it) : std::next(it);
    }
    return true;
}

const ModelAsset* DataStore::model(Id modelId) const {
    const auto it = models_.find(modelId);
    return it == models_.end() ? nullptr : &it->second;
}

std::vector<Id> DataStore::modelIds() const {
    std::vector<Id> ids;
    ids.reserve(models_.size());
    for (const auto& [id, _] : models_) {
        ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

// ---- Assignments ----------------------------------------------------------

Id DataStore::assign(Id modelId, Id imageId) {
    if (models_.find(modelId) == models_.end() ||
        images_.find(imageId) == images_.end()) {
        return kInvalidId;
    }
    if (const auto existing = findAssignment(modelId, imageId)) {
        return *existing;
    }
    Assignment a;
    a.id = nextId();
    a.modelId = modelId;
    a.imageId = imageId;
    // a.transform is default-constructed -> identity (zero) pose.
    const Id id = a.id;
    assignments_.emplace(id, a);
    return id;
}

bool DataStore::unassign(Id assignmentId) {
    return assignments_.erase(assignmentId) > 0;
}

bool DataStore::unassign(Id modelId, Id imageId) {
    if (const auto id = findAssignment(modelId, imageId)) {
        return unassign(*id);
    }
    return false;
}

bool DataStore::reassignImage(Id assignmentId, Id newImageId) {
    const auto it = assignments_.find(assignmentId);
    if (it == assignments_.end() || images_.find(newImageId) == images_.end()) {
        return false;
    }
    if (findAssignment(it->second.modelId, newImageId)) {
        return false; // pair already exists
    }
    it->second.imageId = newImageId;
    return true;
}

const Assignment* DataStore::assignment(Id assignmentId) const {
    const auto it = assignments_.find(assignmentId);
    return it == assignments_.end() ? nullptr : &it->second;
}

std::optional<Id> DataStore::findAssignment(Id modelId, Id imageId) const {
    for (const auto& [id, a] : assignments_) {
        if (a.modelId == modelId && a.imageId == imageId) {
            return id;
        }
    }
    return std::nullopt;
}

std::vector<Id> DataStore::assignmentsForImage(Id imageId) const {
    std::vector<Id> ids;
    for (const auto& [id, a] : assignments_) {
        if (a.imageId == imageId) {
            ids.push_back(id);
        }
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<Id> DataStore::assignmentsForModel(Id modelId) const {
    std::vector<Id> ids;
    for (const auto& [id, a] : assignments_) {
        if (a.modelId == modelId) {
            ids.push_back(id);
        }
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<Id> DataStore::assignmentIds() const {
    std::vector<Id> ids;
    ids.reserve(assignments_.size());
    for (const auto& [id, _] : assignments_) {
        ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

// ---- Transforms -----------------------------------------------------------

std::optional<Transform> DataStore::transform(Id assignmentId) const {
    const auto it = assignments_.find(assignmentId);
    if (it == assignments_.end()) {
        return std::nullopt;
    }
    return it->second.transform;
}

bool DataStore::setTransform(Id assignmentId, const Transform& transform) {
    const auto it = assignments_.find(assignmentId);
    if (it == assignments_.end()) {
        return false;
    }
    it->second.transform = transform;
    return true;
}

// ---- Bulk -----------------------------------------------------------------

void DataStore::clear() {
    images_.clear();
    models_.clear();
    assignments_.clear();
    idCounter_ = kInvalidId;
}

} // namespace avb
