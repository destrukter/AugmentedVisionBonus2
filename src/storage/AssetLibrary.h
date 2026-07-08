#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace avb {

class DataStore;

/// Loads a default asset library from disk into the DataStore at startup, so
/// recurring images/models don't have to be uploaded by hand on every run.
///
/// Expected layout under the library root (default `assets/library`, override
/// with the AVB_LIBRARY_DIR environment variable):
///
///     <root>/images/          tracked images (*.png *.jpg *.jpeg *.bmp)
///     <root>/models/          FBX models (*.fbx)
///     <root>/assignments.cfg  optional model->image pairs (see below)
///
/// Assignments are resolved **by file name**, two ways:
///  1. Explicit pairs in assignments.cfg, one per line:
///         model-file.fbx = image-file.png
///     (`#`-prefixed lines are comments; names are matched case-insensitively
///     against the files found in the two folders.)
///  2. Automatically: a model and an image sharing the same base name (stem)
///     are paired, e.g. `dragon.fbx` + `dragon.png`.
///
/// Every file is validated the same way a manual upload is (images must
/// decode; models must pass the injected validator); failures are reported as
/// warnings instead of silently loading broken assets. The class lives in the
/// storage layer, so the FBX check is injected as a callback rather than
/// depending on the render module.
class AssetLibrary {
public:
    /// Returns true when `path` is a loadable model; on failure may write a
    /// reason into the string pointer (which can be null).
    using ModelValidator = std::function<bool(const std::string&, std::string*)>;

    struct Report {
        int imagesAdded{0};
        int modelsAdded{0};
        int assignmentsCreated{0};
        std::vector<std::string> warnings;
    };

    AssetLibrary(std::shared_ptr<DataStore> store, ModelValidator modelValidator);

    /// Scans `rootDir` and populates the store. A missing root (or missing
    /// subfolders) is not an error - the report just stays empty.
    Report load(const std::string& rootDir);

private:
    std::shared_ptr<DataStore> store_;
    ModelValidator validateModel_;
};

} // namespace avb
