#include "render/SceneRenderer.h"

#include <utility>

#include "render/ModelLoader.h"
#include "render/OgreContext.h"
#include "storage/DataStore.h"

// #include <OgreSceneManager.h>
// #include <OgreSceneNode.h>
// #include <OgreEntity.h>
// #include <OgreTextureManager.h>

namespace avb {

SceneRenderer::SceneRenderer(std::shared_ptr<OgreContext> context,
                             std::shared_ptr<ModelLoader> loader,
                             std::shared_ptr<DataStore> store)
    : context_(std::move(context)),
      loader_(std::move(loader)),
      store_(std::move(store)) {}

SceneRenderer::~SceneRenderer() = default;

bool SceneRenderer::initialize() {
    // TODO: create an off-screen render target (RTT) + camera, a background
    //       rectangle (Ogre::Rectangle2D) for the feed, and a dynamic texture
    //       that is exposed as compositedTexture_ for the ImGui::Image call.
    return true;
}

void SceneRenderer::beginFrame(const cv::Mat& /*cameraImage*/) {
    // TODO: upload cameraImage (BGR -> RGBA) into the background texture and
    //       hide all model nodes; visible ones are re-shown by drawModel().
}

void SceneRenderer::drawModel(Id modelId, const Eigen::Matrix4f& /*pose*/) {
    // TODO:
    //   auto it = nodes_.find(modelId);
    //   if (it == nodes_.end()) {
    //       const ModelAsset* m = store_->model(modelId);
    //       std::string mesh = loader_->loadFbx(m->filePath, "mesh#" + std::to_string(modelId));
    //       Ogre::Entity* e = sceneManager->createEntity(mesh);
    //       Ogre::SceneNode* node = rootNode->createChildSceneNode();
    //       node->attachObject(e);
    //       it = nodes_.emplace(modelId, node).first;
    //   }
    //   apply `pose` to the node (decompose into position + quaternion + scale)
    //   and set it visible.
}

void SceneRenderer::endFrame() {
    // TODO: render the off-screen target; compositedTexture_ now holds the
    //       feed with models composited on top.
}

} // namespace avb
