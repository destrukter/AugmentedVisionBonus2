#pragma once

#include <string>
#include <unordered_map>

namespace avb {

/// Loads FBX files with Assimp and converts them into OGRE meshes.
///
/// OGRE has no native FBX importer, so Assimp parses the file and the resulting
/// vertex/index data is fed into an Ogre::ManualObject which is converted into a
/// cached Ogre::Mesh that the SceneRenderer can instantiate. The file's
/// materials come along: per-submesh diffuse/specular/emissive colors,
/// shininess, two-sidedness, vertex colors and diffuse textures (embedded or
/// external, decoded through OpenCV so no OGRE codec plugin is needed).
/// Submeshes without a usable material fall back to a shared default.
///
/// Animations come along too: when the file contains animations, the whole
/// node hierarchy is mirrored into an Ogre::Skeleton (one bone per node) and
/// every animation becomes a skeletal animation on it. Skinned meshes keep
/// their per-vertex bone weights; meshes without weights are bound rigidly to
/// their node's bone, so plain node/transform animations play as well.
/// Entities instantiated from the mesh then expose the animations as OGRE
/// AnimationStates (the SceneRenderer enables and advances them per frame).
///
/// The loader only touches OGRE's process-wide resource managers (materials,
/// meshes, skeletons, textures), so it has no construction dependencies.
class ModelLoader {
public:
    ModelLoader() = default;

    /// Imports an FBX file and registers an Ogre::Mesh named `meshName`.
    /// Returns the mesh resource name, or "" on failure. Repeated calls for the
    /// same file path return the cached mesh name.
    std::string loadFbx(const std::string& filePath, const std::string& meshName);

    /// Checks (via Assimp, no OGRE/GPU needed) whether `filePath` is a loadable
    /// model containing at least one non-empty mesh. Intended for upload-time
    /// validation so a broken file is rejected with feedback instead of
    /// silently failing to render later. On failure `error` (if non-null)
    /// receives a human-readable reason.
    static bool validateModelFile(const std::string& filePath, std::string* error);

private:
    /// Lazily creates the shared default lit material and returns its name.
    std::string ensureDefaultMaterial();

    std::unordered_map<std::string, std::string> cache_; // filePath -> meshName
    bool defaultMaterialCreated_{false};
};

} // namespace avb
