#include "render/ModelLoader.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <OgreManualObject.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreMesh.h>
#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>

#include "render/OgreContext.h"

namespace avb {

namespace {
constexpr const char* kDefaultMaterial = "avb/DefaultLit";
// Read lazily inside functions to avoid static-init-order issues with OGRE.
const Ogre::String& defaultGroup() {
    return Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
}
} // namespace

ModelLoader::ModelLoader(OgreContext& context) : context_(context) {}

std::string ModelLoader::ensureDefaultMaterial() {
    if (defaultMaterialCreated_) {
        return kDefaultMaterial;
    }
    auto& mm = Ogre::MaterialManager::getSingleton();
    if (!mm.getByName(kDefaultMaterial, defaultGroup())) {
        Ogre::MaterialPtr mat = mm.create(kDefaultMaterial, defaultGroup());
        Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
        pass->setLightingEnabled(true);
        pass->setDiffuse(0.8f, 0.8f, 0.82f, 1.0f);
        pass->setSpecular(0.2f, 0.2f, 0.2f, 1.0f);
        pass->setShininess(20.0f);
        // RTSS (set up by OgreContext) generates the actual shaders.
    }
    defaultMaterialCreated_ = true;
    return kDefaultMaterial;
}

std::string ModelLoader::loadFbx(const std::string& filePath,
                                 const std::string& meshName) {
    if (const auto it = cache_.find(filePath); it != cache_.end()) {
        return it->second;
    }

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        filePath, aiProcess_Triangulate | aiProcess_GenSmoothNormals |
                      aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices |
                      aiProcess_PreTransformVertices);
    if (!scene || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) ||
        !scene->mRootNode || scene->mNumMeshes == 0) {
        return "";
    }

    const std::string material = ensureDefaultMaterial();
    Ogre::SceneManager* sm = context_.sceneManager();
    Ogre::ManualObject* manual = sm->createManualObject();
    manual->setDynamic(false);

    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
        if (mesh->mNumVertices == 0) {
            continue;
        }
        manual->begin(material, Ogre::RenderOperation::OT_TRIANGLE_LIST, defaultGroup());

        const bool hasUv = mesh->HasTextureCoords(0);
        for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
            const aiVector3D& p = mesh->mVertices[v];
            manual->position(p.x, p.y, p.z);
            if (mesh->HasNormals()) {
                const aiVector3D& n = mesh->mNormals[v];
                manual->normal(n.x, n.y, n.z);
            }
            if (hasUv) {
                const aiVector3D& t = mesh->mTextureCoords[0][v];
                manual->textureCoord(t.x, t.y);
            }
        }
        // Indices are local to this section (one section per aiMesh).
        for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            if (face.mNumIndices != 3) {
                continue; // triangulated above, skip any stray primitives
            }
            manual->triangle(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
        }
        manual->end();
    }

    Ogre::MeshPtr ogreMesh = manual->convertToMesh(meshName, defaultGroup());
    sm->destroyManualObject(manual);
    if (!ogreMesh) {
        return "";
    }

    cache_.emplace(filePath, meshName);
    return meshName;
}

} // namespace avb
