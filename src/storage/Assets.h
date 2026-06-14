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
/// One model may appear in many assignments (one per image it is assigned to);
/// the (modelId, imageId) pair is unique. `transform` defaults to identity.
struct Assignment {
    Id id{kInvalidId};
    Id modelId{kInvalidId};
    Id imageId{kInvalidId};
    Transform transform{};  ///< Pose of the model relative to the image.
};

} // namespace avb
