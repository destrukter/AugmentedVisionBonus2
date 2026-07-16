#pragma once

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "storage/Transform.h"
#include "storage/Types.h"
#include "ui/Window.h"

namespace avb {

class DataStore;
class ConfigurePreview;

/// Configure window (window 2 of 3).
///
/// Edits the poses of the models assigned to one image. The viewport shows
/// the image and **all** its assigned models (rendered off-screen by
/// ConfigurePreview); a dropdown selects which model is being edited - the
/// numeric fields, matrix preview and gizmo handles all follow the selection.
/// Edits live in per-model working copies until the user clicks "Save", which
/// writes every modified pose back into the DataStore, so switching models in
/// the dropdown never loses unsaved changes.
///
/// "Set origin here" folds the selected model's current translation/rotation
/// into the assignment's persistent origin: the model stays where it is, the
/// editable translation/rotation read zero again, and further edits are
/// relative to that origin.
class ConfigureWindow : public Window {
public:
    /// Invoked after a pose was successfully saved to the store (used by the
    /// Application to persist library poses into assignments.cfg).
    using SaveCallback = std::function<void(Id assignmentId)>;

    explicit ConfigureWindow(std::shared_ptr<DataStore> store,
                             std::shared_ptr<ConfigurePreview> preview = {},
                             SaveCallback onSaved = {});
    ~ConfigureWindow() override;

    /// Opens an image for editing (called when "Configure" is clicked in the
    /// Upload window). All models assigned to the image are shown; the first
    /// is selected for editing. Discards any unsaved working state.
    void openImage(Id imageId);

    /// Drives the off-screen OGRE render of the viewport content (real image
    /// + real models). Must be called from the application loop before this
    /// window's renderFrame() - it switches to OGRE's GL context, which must
    /// never happen mid-ImGui-pass (see CameraWindow::updateTrackingAndRender
    /// for the same constraint).
    void updatePreviewRender();

protected:
    void drawUi() override;

private:
    /// Per-assignment editable state (working copy of the stored pose).
    struct WorkingState {
        Transform origin;     ///< Rigid base pose ("Set origin here").
        Transform transform;  ///< Editable pose, relative to origin.
        bool dirty{false};    ///< Differs from the stored pose.
    };

    void drawGizmoViewport(WorkingState* selected);
    void save();    ///< Commit every dirty working copy back into the store.
    void revert();  ///< Reload all working copies from the store.
    /// Folds the selected model's translation/rotation into its origin (the
    /// model stays put; the editable values read zero afterwards).
    void setOriginToCurrent();

    /// The open image's assignments, in store order; also drops stale working
    /// state and keeps selectedAssignment_ valid.
    std::vector<Id> refreshAssignments();
    /// The working copy for an assignment, lazily loaded from the store.
    WorkingState& workingFor(Id assignmentId);
    WorkingState* selectedWorking();

    /// Aspect ratio (height/width) of the image's tracking plane, falling
    /// back to 1 when the image pixels aren't available.
    float imagePlaneAspect() const;

    /// Orbit camera position derived from yaw/pitch/distance; feeds both the
    /// gizmo view matrix and the ConfigurePreview render so they stay aligned.
    Eigen::Vector3f eyePosition() const;

    std::shared_ptr<DataStore> store_;
    std::shared_ptr<ConfigurePreview> preview_;
    SaveCallback onSaved_;
    Id activeImage_{kInvalidId};
    Id selectedAssignment_{kInvalidId};
    std::unordered_map<Id, WorkingState> working_;

    int gizmoOperation_{0}; ///< 0 translate, 1 rotate, 2 scale.
    float orbitYawDeg_{40.0f};    ///< Viewport camera orbit around the plane.
    float orbitPitchDeg_{30.0f};
    float orbitDistance_{3.0f};

    // Off-screen preview display state. The canvas size is recorded during
    // drawUi and consumed by the next updatePreviewRender (1-frame lag).
    unsigned int previewTexture_{0};  // GL texture in this window's context
    int previewTexW_{0};
    int previewTexH_{0};
    int canvasW_{0};
    int canvasH_{0};
    bool previewValid_{false};
    /// Matrix the gizmo manipulates (the selected model's full pose,
    /// origin * transform). Kept across the frames of one drag (and only
    /// rebuilt while the gizmo is idle) because rebuilding it from the
    /// decomposed Euler angles mid-drag makes the handles snap at
    /// representation boundaries.
    Eigen::Matrix4f gizmoMatrix_{Eigen::Matrix4f::Identity()};
};

} // namespace avb
