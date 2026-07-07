#pragma once

#include <memory>

#include "storage/Transform.h"
#include "storage/Types.h"
#include "ui/Window.h"

namespace avb {

class DataStore;

/// Configure window (window 2 of 3).
///
/// Edits the pose of a model **relative to its image** for a single
/// assignment, either interactively - a 3D viewport with translate / rotate /
/// scale gizmo handles (ImGuizmo) drawn over the image plane - or through the
/// numeric fields below it. The edits live in a working copy until the user
/// clicks "Save", which writes them back into the DataStore.
class ConfigureWindow : public Window {
public:
    explicit ConfigureWindow(std::shared_ptr<DataStore> store);

    /// Loads an assignment for editing (called when "Configure" is clicked in
    /// the Upload window). Pulls the stored Transform into the working copy.
    void openAssignment(Id assignmentId);

protected:
    void drawUi() override;

private:
    void drawGizmoViewport();  ///< Interactive 3D manipulation of working_.
    void save();    ///< Commit working_ back into the store.
    void revert();  ///< Reload working_ from the store.

    /// Aspect ratio (height/width) of the assigned image's tracking plane,
    /// falling back to 1 when the image pixels aren't available.
    float imagePlaneAspect() const;

    std::shared_ptr<DataStore> store_;
    Id activeAssignment_{kInvalidId};
    Transform working_{};   ///< Editable copy; defaults to identity.
    bool dirty_{false};     ///< True when working_ differs from the stored pose.

    int gizmoOperation_{0}; ///< 0 translate, 1 rotate, 2 scale.
    float orbitYawDeg_{40.0f};    ///< Viewport camera orbit around the plane.
    float orbitPitchDeg_{30.0f};
    float orbitDistance_{3.0f};
};

} // namespace avb
