#pragma once

#include <string>

namespace avb {

class OgreContext;

/// Loads FBX files with Assimp and converts them into OGRE meshes.
///
/// OGRE has no native FBX importer, so Assimp parses the file and the resulting
/// vertex/index/material data is uploaded into an Ogre::Mesh that the
/// SceneRenderer can instantiate. Loaded meshes are cached by file path.
class ModelLoader {
public:
    explicit ModelLoader(OgreContext& context);

    /// Imports an FBX file and registers an Ogre::Mesh under `meshName`.
    /// Returns the resource name of the created mesh, or "" on failure.
    /// Subsequent calls with the same path return the cached mesh name.
    std::string loadFbx(const std::string& filePath, const std::string& meshName);

private:
    OgreContext& context_;
};

} // namespace avb
