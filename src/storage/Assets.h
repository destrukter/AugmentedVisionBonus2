#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "storage/Transform.h"
#include "storage/Types.h"

namespace avb {

/// An uploaded image that can be tracked in the camera feed.
struct ImageAsset {
    Id id{kInvalidId};
    std::string name;       ///< Display name (defaults to the file name).
    std::string filePath;   ///< Absolute path on disk.
    cv::Mat pixels;         ///< Decoded image (lazily loaded; may be empty).
};

/// An uploaded FBX 3D model that can be rendered on top of images.
struct ModelAsset {
    Id id{kInvalidId};
    std::string name;       ///< Display name (defaults to the file name).
    std::string filePath;   ///< Absolute path to the .fbx file.
};

/// Links one model to one image with a configurable relative pose.
///
/// One model may appear in many assignments - across different images, and
/// also multiple times on the *same* image (each placed copy is its own
/// assignment with its own pose). Both poses default to identity.
///
/// The model's full pose relative to the image is `origin * transform`.
/// `origin` is a rigid (translation + rotation, scale 1) base pose set by the
/// Configure window's "Set origin here" action: it folds the current
/// translation/rotation into the origin so `transform` reads zero while the
/// model stays put, and further edits are relative to that origin.
struct Assignment {
    Id id{kInvalidId};
    Id modelId{kInvalidId};
    Id imageId{kInvalidId};
    Transform origin{};     ///< Rigid base pose (see above); usually identity.
    Transform transform{};  ///< Editable pose of the model, relative to origin.
};

} // namespace avb
