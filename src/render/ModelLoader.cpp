#include "render/ModelLoader.h"

#include "render/OgreContext.h"

// #include <assimp/Importer.hpp>
// #include <assimp/scene.h>
// #include <assimp/postprocess.h>
// #include <OgreMeshManager.h>
// #include <OgreSubMesh.h>

namespace avb {

ModelLoader::ModelLoader(OgreContext& context) : context_(context) {}

std::string ModelLoader::loadFbx(const std::string& /*filePath*/,
                                 const std::string& meshName) {
    // TODO:
    //   Assimp::Importer importer;
    //   const aiScene* scene = importer.ReadFile(filePath,
    //       aiProcess_Triangulate | aiProcess_GenSmoothNormals |
    //       aiProcess_FlipUVs | aiProcess_CalcTangentSpace);
    //   if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) return "";
    //
    //   Create an Ogre::ManualObject / Mesh: for each aiMesh copy positions,
    //   normals, uvs and indices into a SubMesh; map aiMaterial -> Ogre material.
    //   MeshManager::getSingleton().createManual(meshName, group) ...
    //
    //   Cache and return meshName.
    return meshName;
}

} // namespace avb
