#pragma once

#include <functional>
#include <memory>

#include <Eigen/Core>

#include "storage/Transform.h"
#include "storage/Types.h"
#include "ui/Window.h"

namespace avb {

class DataStore;
class ConfigurePreview;

/// Configure window (window 2 of 3).
///
/// Edits the pose of a model **relative to its image** for a single
/// assignment interactively: the viewport shows the assignment's actual
/// image and actual model (rendered off-screen by ConfigurePreview) with
/// translate / rotate / scale gizmo handles (ImGuizmo) drawn on top; numeric
/// fields above give exact control over the same values. The edits live in a
/// working copy until the user clicks "Save", which writes them back into
/// the DataStore.
class ConfigureWindow : public Window {
public:
    /// Invoked after a pose was successfully saved to the store (used by the
    /// Application to persist library poses into assignments.cfg).
    using SaveCallback = std::function<void(Id assignmentId)>;

    explicit ConfigureWindow(std::shared_ptr<DataStore> store,
                             std::shared_ptr<ConfigurePreview> preview = {},
                             SaveCallback onSaved = {});
    ~ConfigureWindow() override;

    /// Loads an assignment for editing (called when "Configure" is clicked in
    /// the Upload window). Pulls the stored Transform into the working copy.
    void openAssignment(Id assignmentId);

    /// Drives the off-screen OGRE render of the viewport content (real image
    /// + real model). Must be called from the application loop before this
    /// window's renderFrame() - it switches to OGRE's GL context, which must
    /// never happen mid-ImGui-pass (see CameraWindow::updateTrackingAndRender
    /// for the same constraint).
    void updatePreviewRender();

protected:
    void drawUi() override;

private:
    void drawGizmoViewport();  ///< Interactive 3D manipulation of working_.
    void save();    ///< Commit working_ back into the store.
    void revert();  ///< Reload working_ from the store.

    /// Aspect ratio (height/width) of the assigned image's tracking plane,
    /// falling back to 1 when the image pixels aren't available.
    float imagePlaneAspect() const;

    /// Orbit camera position derived from yaw/pitch/distance; feeds both the
    /// gizmo view matrix and the ConfigurePreview render so they stay aligned.
    Eigen::Vector3f eyePosition() const;

    std::shared_ptr<DataStore> store_;
    std::shared_ptr<ConfigurePreview> preview_;
    SaveCallback onSaved_;
    Id activeAssignment_{kInvalidId};
    Transform working_{};   ///< Editable copy; defaults to identity.
    bool dirty_{false};     ///< True when working_ differs from the stored pose.

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
    /// Matrix the gizmo manipulates. Kept across the frames of one drag (and
    /// only rebuilt from working_ while the gizmo is idle) because rebuilding
    /// it from the decomposed Euler angles mid-drag makes the handles snap at
    /// representation boundaries.
    Eigen::Matrix4f gizmoMatrix_{Eigen::Matrix4f::Identity()};
};

} // namespace avb
