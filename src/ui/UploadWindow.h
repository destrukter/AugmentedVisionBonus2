#pragma once

#include <functional>
#include <memory>

#include "storage/Types.h"
#include "ui/Window.h"

namespace avb {

class DataStore;

/// Upload window (window 1 of 3).
///
/// Responsibilities:
///   * Upload images and FBX models (file pickers).
///   * Assign a model to an image; one model may be assigned to many images.
///   * Revert (unassign) and re-assign differently.
///   * Offer a "Configure" button per (model, image) assignment that hands the
///     assignment off to the Configure window via the onConfigure callback.
class UploadWindow : public Window {
public:
    /// Invoked when the user clicks "Configure" on an assignment.
    using ConfigureCallback = std::function<void(Id assignmentId)>;

    UploadWindow(std::shared_ptr<DataStore> store, ConfigureCallback onConfigure);

    /// Sets the status line shown under the upload buttons. Also used by the
    /// Application to surface the startup asset-library summary.
    enum class StatusKind { Success, Warning, Error };
    void setStatus(StatusKind kind, std::string message);

protected:
    void drawUi() override;

private:
    void drawUploadSection();      ///< Buttons to import images / FBX models.
    void drawAssignmentSection();  ///< Image+model matrix with assign/revert.

    void uploadImage(const std::string& path);
    void uploadModel(const std::string& path);

    std::shared_ptr<DataStore> store_;
    ConfigureCallback onConfigure_;

    // Transient UI selection state.
    Id selectedImage_{kInvalidId};
    Id selectedModel_{kInvalidId};

    // Upload feedback (validation success/failure of the last upload).
    std::string statusMessage_;
    StatusKind statusKind_{StatusKind::Success};

    // Optional pasted-path entry; left empty, "Add..." opens a native file
    // dialog instead.
    char imagePathBuf_[512]{};
    char modelPathBuf_[512]{};
};

} // namespace avb
