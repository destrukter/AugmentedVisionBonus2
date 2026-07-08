#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "storage/Types.h"

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
/// A pair line may carry optional pose columns after a `|`:
///
///     model-file.fbx = image-file.png | t=x,y,z r=x,y,z s=v
///
/// `t` is the translation, `r` the rotation in Euler degrees, `s` the uniform
/// scale. Each may be omitted (in any order); missing components default to
/// the identity pose (translation 0, rotation 0, scale 1). Poses configured
/// in the app are written back into these columns via persistAssignment(), so
/// they survive restarts.
///
/// A line starting with `!` is an exclusion:
///
///     ! model-file.fbx = image-file.png
///
/// It suppresses the automatic stem pairing for exactly that pair - written
/// by saveSession() when a name-matching pair was reverted in the app, so the
/// removal survives restarts too.
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

    /// Writes the current pose of `assignmentId` back into
    /// `<rootDir>/assignments.cfg` so it survives restarts. The edit is
    /// surgical: the matching pair line is rewritten (or appended when the
    /// pair - e.g. one auto-created by stem matching - has no line yet) and
    /// every other line, including comments, is preserved. Identity poses
    /// write a bare pair with no pose columns.
    ///
    /// Returns false when the assignment is unknown or when its model/image
    /// files do not live in the library folders - such names could not be
    /// resolved at the next startup, so persisting them would be misleading.
    bool persistAssignment(const std::string& rootDir, Id assignmentId);

    struct SessionSaveResult {
        int filesCopied{0};
        int filesRemoved{0};       ///< moved into <root>/removed/
        int assignmentsSaved{0};
        std::vector<std::string> warnings;
    };

    /// Saves the whole current session into the library so it is restored on
    /// the next startup - a full sync in both directions:
    ///  * every image/model whose file lives outside the library folders is
    ///    copied in (and the store re-pointed at the copy);
    ///  * library files whose asset was removed from the session are moved
    ///    into `<root>/removed/` (never deleted outright);
    ///  * assignments.cfg is rewritten to hold exactly the current
    ///    assignments with their poses: stale pair lines are dropped, and a
    ///    `!` exclusion line is written for every name-matching (stem) pair
    ///    that is currently unassigned, so reverting an auto-paired
    ///    assignment survives restarts. Comments are preserved.
    /// Missing library folders are created.
    SessionSaveResult saveSession(const std::string& rootDir);

private:
    std::shared_ptr<DataStore> store_;
    ModelValidator validateModel_;
};

} // namespace avb
