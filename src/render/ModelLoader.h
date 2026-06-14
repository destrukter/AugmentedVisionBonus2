#pragma once

#include <string>
#include <unordered_map>

namespace avb {

class OgreContext;

/// Loads FBX files with Assimp and converts them into OGRE meshes.
///
/// OGRE has no native FBX importer, so Assimp parses the file and the resulting
/// vertex/index data is fed into an Ogre::ManualObject which is converted into a
/// cached Ogre::Mesh that the SceneRenderer can instantiate.
class ModelLoader {
public:
    explicit ModelLoader(OgreContext& context);

    /// Imports an FBX file and registers an Ogre::Mesh named `meshName`.
    /// Returns the mesh resource name, or "" on failure. Repeated calls for the
    /// same file path return the cached mesh name.
    std::string loadFbx(const std::string& filePath, const std::string& meshName);

private:
    /// Lazily creates the shared default lit material and returns its name.
    std::string ensureDefaultMaterial();

    OgreContext& context_;
    std::unordered_map<std::string, std::string> cache_; // filePath -> meshName
    bool defaultMaterialCreated_{false};
};

} // namespace avb
