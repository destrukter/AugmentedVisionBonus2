#include "TestMain.h"

#include <cstdio>
#include <filesystem>
#include <string>

#include <assimp/Exporter.hpp>
#include <assimp/anim.h>
#include <assimp/mesh.h>
#include <assimp/scene.h>

#include <OgreAnimation.h>
#include <OgreDefaultHardwareBufferManager.h>
#include <OgreLodStrategyManager.h>
#include <OgreLogManager.h>
#include <OgreMaterialManager.h>
#include <OgreMath.h>
#include <OgreMesh.h>
#include <OgreMeshManager.h>
#include <OgreResourceGroupManager.h>
#include <OgreSkeleton.h>
#include <OgreSkeletonManager.h>
#include <OgreSubMesh.h>

#include "render/ModelLoader.h"

using namespace avb;

namespace {

namespace fs = std::filesystem;

/// The CPU-side slice of OGRE that mesh/skeleton building needs - the same
/// managers OGRE's own offline mesh tools instantiate. No window, GL context
/// or render system is required; DefaultHardwareBufferManager keeps "hardware"
/// buffers in system memory.
struct HeadlessOgre {
    HeadlessOgre() {
        logMgr.createLog("ModelLoaderTests.ogre.log", /*defaultLog=*/true,
                         /*debuggerOutput=*/false, /*suppressFileOutput=*/false);
        matMgr.initialise();
    }

    Ogre::LogManager logMgr;
    Ogre::ResourceGroupManager resGroupMgr;
    Ogre::Math math;
    Ogre::LodStrategyManager lodMgr;
    // Declared before the resource managers so it is destroyed after them:
    // meshes must release their (software) buffers before the manager goes.
    Ogre::DefaultHardwareBufferManager bufferMgr;
    Ogre::MaterialManager matMgr;
    Ogre::SkeletonManager skelMgr;
    Ogre::MeshManager meshMgr;
};

/// Builds a one-triangle scene under a node named "Spinner". When `animated`,
/// a 1-second rotation animation targets that node - the shape of a simple
/// rigid FBX animation. All allocations are handed to the aiScene, whose
/// destructor frees them.
aiScene* buildScene(bool animated) {
    auto* scene = new aiScene();
    scene->mRootNode = new aiNode("Scene");

    auto* spinner = new aiNode("Spinner");
    // Static variant: offset the node so baking node transforms is observable.
    if (!animated) {
        spinner->mTransformation = aiMatrix4x4(); // identity...
        spinner->mTransformation.a4 = 2.0f;       // ...plus translate +2 on X
    }
    spinner->mParent = scene->mRootNode;
    spinner->mNumMeshes = 1;
    spinner->mMeshes = new unsigned int[1]{0};
    scene->mRootNode->mNumChildren = 1;
    scene->mRootNode->mChildren = new aiNode*[1]{spinner};

    auto* mesh = new aiMesh();
    mesh->mName = aiString("tri");
    mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
    mesh->mMaterialIndex = 0;
    mesh->mNumVertices = 3;
    mesh->mVertices = new aiVector3D[3]{{0.0f, 0.0f, 0.0f},
                                        {1.0f, 0.0f, 0.0f},
                                        {0.0f, 1.0f, 0.0f}};
    mesh->mNormals = new aiVector3D[3]{{0.0f, 0.0f, 1.0f},
                                       {0.0f, 0.0f, 1.0f},
                                       {0.0f, 0.0f, 1.0f}};
    mesh->mNumFaces = 1;
    mesh->mFaces = new aiFace[1];
    mesh->mFaces[0].mNumIndices = 3;
    mesh->mFaces[0].mIndices = new unsigned int[3]{0, 1, 2};
    scene->mNumMeshes = 1;
    scene->mMeshes = new aiMesh*[1]{mesh};

    scene->mNumMaterials = 1;
    scene->mMaterials = new aiMaterial*[1]{new aiMaterial()};

    if (animated) {
        auto* channel = new aiNodeAnim();
        channel->mNodeName = aiString("Spinner");
        channel->mNumRotationKeys = 3;
        channel->mRotationKeys = new aiQuatKey[3];
        channel->mRotationKeys[0] =
            aiQuatKey(0.0, aiQuaternion(aiVector3D(0, 0, 1), 0.0f));
        channel->mRotationKeys[1] =
            aiQuatKey(1.0, aiQuaternion(aiVector3D(0, 0, 1), 1.5708f));
        channel->mRotationKeys[2] =
            aiQuatKey(2.0, aiQuaternion(aiVector3D(0, 0, 1), 3.1416f));
        channel->mNumPositionKeys = 1;
        channel->mPositionKeys =
            new aiVectorKey[1]{aiVectorKey(0.0, aiVector3D(0, 0, 0))};
        channel->mNumScalingKeys = 1;
        channel->mScalingKeys =
            new aiVectorKey[1]{aiVectorKey(0.0, aiVector3D(1, 1, 1))};

        auto* anim = new aiAnimation();
        anim->mName = aiString("spin");
        anim->mDuration = 2.0;       // ticks
        anim->mTicksPerSecond = 2.0; // -> 1 second long
        anim->mNumChannels = 1;
        anim->mChannels = new aiNodeAnim*[1]{channel};
        scene->mNumAnimations = 1;
        scene->mAnimations = new aiAnimation*[1]{anim};
    }
    return scene;
}

/// Exports `scene` as a real binary FBX file and returns its path, or "" on
/// failure - so the tests exercise exactly the format users upload. (Assimp's
/// FBX exporter may resample animation keys, but preserves targets/length.)
std::string writeFixture(bool animated, const std::string& name) {
    aiScene* scene = buildScene(animated);
    const fs::path path = fs::temp_directory_path() / (name + ".fbx");
    Assimp::Exporter exporter;
    const aiReturn rc = exporter.Export(scene, "fbx", path.string());
    delete scene;
    if (rc != aiReturn_SUCCESS) {
        std::printf("FAIL: could not export test fixture %s: %s\n",
                    path.string().c_str(), exporter.GetErrorString());
        return "";
    }
    return path.string();
}

} // namespace

static void test_animated_model_gets_skeleton_and_animation() {
    const std::string file = writeFixture(/*animated=*/true, "avb_anim_fixture");
    CHECK(!file.empty());
    if (file.empty()) {
        return;
    }

    ModelLoader loader;
    const std::string meshName = loader.loadFbx(file, "test/animated");
    CHECK(meshName == "test/animated");
    if (meshName.empty()) {
        return;
    }

    const Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().getByName(
        meshName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    CHECK(mesh != nullptr);
    if (!mesh) {
        return;
    }
    CHECK(mesh->getNumSubMeshes() == 1);

    // The animation must arrive as a skeleton with a ~1s animation whose
    // track targets the animated node's bone.
    CHECK(mesh->hasSkeleton());
    const Ogre::SkeletonPtr skeleton = mesh->getSkeleton();
    CHECK(skeleton != nullptr);
    if (!skeleton) {
        return;
    }
    CHECK(skeleton->getNumAnimations() == 1);
    if (skeleton->getNumAnimations() == 0) {
        return;
    }
    Ogre::Animation* animation = skeleton->getAnimation(0);
    CHECK(animation->getLength() > 0.9f && animation->getLength() < 1.1f);
    CHECK(animation->getNumNodeTracks() >= 1);

    // The (unskinned) mesh must ride its node's bone: one weight per vertex.
    CHECK(!mesh->getSubMesh(0)->getBoneAssignments().empty());
}

static void test_static_model_loads_without_skeleton() {
    const std::string file = writeFixture(/*animated=*/false, "avb_static_fixture");
    CHECK(!file.empty());
    if (file.empty()) {
        return;
    }

    ModelLoader loader;
    const std::string meshName = loader.loadFbx(file, "test/static");
    CHECK(meshName == "test/static");
    if (meshName.empty()) {
        return;
    }

    const Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().getByName(
        meshName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    CHECK(mesh != nullptr);
    if (!mesh) {
        return;
    }
    CHECK(!mesh->hasSkeleton());
    CHECK(mesh->getSubMesh(0)->getBoneAssignments().empty());

    // The node's +2 X offset must be baked into the geometry (the job
    // aiProcess_PreTransformVertices did before animations were supported).
    const Ogre::AxisAlignedBox& box = mesh->getBounds();
    CHECK(!box.isNull());
    CHECK(box.getMinimum().x > 1.5f);
    CHECK(box.getMaximum().x < 3.5f);
}

void run_modelloader_tests() {
    HeadlessOgre ogre;
    test_animated_model_gets_skeleton_and_animation();
    test_static_model_loads_without_skeleton();
}
