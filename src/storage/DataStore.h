#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "storage/Assets.h"
#include "storage/Transform.h"
#include "storage/Types.h"

namespace avb {

/// Central in-memory backend store shared by all three windows.
///
/// Owns images, FBX models and the assignments that bind a model to an image
/// together with a relative `Transform`. The class is intentionally free of any
/// UI / rendering / vision dependency so it can be unit-tested in isolation.
///
/// Threading: not internally synchronized. The application drives all windows
/// from a single thread; if background capture/tracking is added it must
/// marshal store access back to that thread.
class DataStore {
public:
    DataStore() = default;

    // ---- Images -----------------------------------------------------------
    /// Registers an image file. `name` defaults to the file name when empty.
    /// Pixel data is not decoded here (see loadImagePixels()).
    Id addImage(const std::string& filePath, const std::string& name = "");
    bool removeImage(Id imageId);
    const ImageAsset* image(Id imageId) const;
    std::vector<Id> imageIds() const;

    // ---- Models -----------------------------------------------------------
    /// Registers an FBX model file. `name` defaults to the file name when empty.
    Id addModel(const std::string& filePath, const std::string& name = "");
    bool removeModel(Id modelId);
    const ModelAsset* model(Id modelId) const;
    std::vector<Id> modelIds() const;

    // ---- Assignments ------------------------------------------------------
    /// Assigns `modelId` to `imageId` with a default (identity) transform.
    /// If the pair already exists the existing assignment id is returned.
    /// Returns kInvalidId when either id is unknown.
    Id assign(Id modelId, Id imageId);

    /// Reverts an assignment. Both overloads are no-ops for unknown inputs.
    bool unassign(Id assignmentId);
    bool unassign(Id modelId, Id imageId);

    /// Reassigns an existing assignment to a different image, preserving its
    /// transform. Returns false if the target pair already exists or ids are
    /// unknown.
    bool reassignImage(Id assignmentId, Id newImageId);

    const Assignment* assignment(Id assignmentId) const;
    std::optional<Id> findAssignment(Id modelId, Id imageId) const;

    /// All assignments for a given image (used by the Camera window to know
    /// what to render when that image is tracked).
    std::vector<Id> assignmentsForImage(Id imageId) const;
    /// All assignments referencing a given model.
    std::vector<Id> assignmentsForModel(Id modelId) const;
    std::vector<Id> assignmentIds() const;

    // ---- Transforms (Configure window) ------------------------------------
    /// Reads the current pose of an assignment, if it exists.
    std::optional<Transform> transform(Id assignmentId) const;
    /// Writes a new pose for an assignment (the Configure "Save" action).
    bool setTransform(Id assignmentId, const Transform& transform);

    // ---- Bulk -------------------------------------------------------------
    void clear();

private:
    Id nextId() { return ++idCounter_; }

    Id idCounter_{kInvalidId};
    std::unordered_map<Id, ImageAsset> images_;
    std::unordered_map<Id, ModelAsset> models_;
    std::unordered_map<Id, Assignment> assignments_;
};

} // namespace avb
