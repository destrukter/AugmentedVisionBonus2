#pragma once

#include <memory>

#include "storage/Transform.h"
#include "storage/Types.h"
#include "ui/Window.h"

namespace avb {

class DataStore;

/// Configure window (window 2 of 3).
///
/// Edits the translation and rotation of a model **relative to its image** for a
/// single assignment. The edits live in a working copy until the user clicks
/// "Save", which writes them back into the DataStore.
class ConfigureWindow : public Window {
public:
    explicit ConfigureWindow(std::shared_ptr<DataStore> store);

    /// Loads an assignment for editing (called when "Configure" is clicked in
    /// the Upload window). Pulls the stored Transform into the working copy.
    void openAssignment(Id assignmentId);

protected:
    void drawUi() override;

private:
    void save();    ///< Commit working_ back into the store.
    void revert();  ///< Reload working_ from the store.

    std::shared_ptr<DataStore> store_;
    Id activeAssignment_{kInvalidId};
    Transform working_{};   ///< Editable copy; defaults to identity.
    bool dirty_{false};     ///< True when working_ differs from the stored pose.
};

} // namespace avb
