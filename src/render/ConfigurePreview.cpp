#include "render/ConfigurePreview.h"

#include <utility>

#include <opencv2/imgproc.hpp>

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreRenderTexture.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>
#include <OgreViewport.h>

#include <RTShaderSystem/OgreShaderGenerator.h>

#include "render/ModelLoader.h"
#include "render/OgreContext.h"
#include "storage/DataStore.h"

namespace avb {

namespace {

// Applies a full 4x4 pose (translation * rotation * per-axis scale) to a node;
// same decomposition as SceneRenderer::drawModel.
void applyPose(Ogre::SceneNode* node, const Eigen::Matrix4f& pose) {
    const Eigen::Vector3f t = pose.block<3, 1>(0, 3);
    Eigen::Matrix3f rs = pose.block<3, 3>(0, 0);
    const float sx = rs.col(0).norm();
    const float sy = rs.col(1).norm();
    const float sz = rs.col(2).norm();
    if (sx > 1e-6f) rs.col(0) /= sx;
    if (sy > 1e-6f) rs.col(1) /= sy;
    if (sz > 1e-6f) rs.col(2) /= sz;
    const Ogre::Matrix3 r(rs(0, 0), rs(0, 1), rs(0, 2), rs(1, 0), rs(1, 1),
                          rs(1, 2), rs(2, 0), rs(2, 1), rs(2, 2));
    node->setPosition(t.x(), t.y(), t.z());
    node->setOrientation(Ogre::Quaternion(r));
    node->setScale(sx, sy, sz);
}

} // namespace

ConfigurePreview::ConfigurePreview(std::shared_ptr<OgreContext> context,
                                   std::shared_ptr<ModelLoader> loader,
                                   std::shared_ptr<DataStore> store)
    : context_(std::move(context)),
      loader_(std::move(loader)),
      store_(std::move(store)) {}

ConfigurePreview::~ConfigurePreview() {
    destroyTarget();
}

bool ConfigurePreview::ensureInitialized() {
    if (initialized_) {
        return true;
    }
    Ogre::SceneManager* sm = context_->sceneManager();
    root_ = sm->getRootSceneNode()->createChildSceneNode("avb_cfg_root");

    camera_ = sm->createCamera("avb_cfg_camera");
    camera_->setNearClipDistance(0.05f);
    camera_->setAutoAspectRatio(true);
    cameraNode_ = sm->getRootSceneNode()->createChildSceneNode("avb_cfg_camNode");
    cameraNode_->attachObject(camera_);
    // Roll-free orbiting, matching the lookAt(up=+Y) the window's gizmo uses.
    cameraNode_->setFixedYawAxis(true, Ogre::Vector3::UNIT_Y);

    initialized_ = true;
    return true;
}

bool ConfigurePreview::ensureTarget(int width, int height) {
    if (renderTarget_ && width == width_ && height == height_) {
        return true;
    }
    destroyTarget();

    rttName_ = "avb_cfg_rtt";
    Ogre::TexturePtr rtt = Ogre::TextureManager::getSingleton().createManual(
        rttName_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, static_cast<unsigned int>(width),
        static_cast<unsigned int>(height), 0, Ogre::PF_R8G8B8A8,
        Ogre::TU_RENDERTARGET);
    if (!rtt) {
        return false;
    }
    width_ = width;
    height_ = height;
    renderTarget_ = rtt->getBuffer()->getRenderTarget();
    Ogre::Viewport* vp = renderTarget_->addViewport(camera_);
    // Matches the ImGui viewport's dark background.
    vp->setBackgroundColour(Ogre::ColourValue(26 / 255.0f, 28 / 255.0f,
                                              33 / 255.0f, 1.0f));
    vp->setClearEveryFrame(true);
    vp->setOverlaysEnabled(false);
    vp->setMaterialScheme(Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
    // Show only the preview objects, not the AR scene's model nodes.
    vp->setVisibilityMask(kConfigPreviewVisibilityMask);
    renderTarget_->setAutoUpdated(false);

    readback_.assign(static_cast<size_t>(width_) * height_ * 4, 0);
    return true;
}

void ConfigurePreview::destroyTarget() {
    if (rttName_.empty()) {
        return;
    }
    Ogre::TextureManager::getSingleton().remove(
        rttName_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    renderTarget_ = nullptr;
    rttName_.clear();
}

bool ConfigurePreview::ensureImagePlane(Id imageId) {
    if (imageId == currentImageId_ && plane_) {
        return true;
    }
    const ImageAsset* img = store_->image(imageId);
    if (!img || img->pixels.empty()) {
        return false;
    }
    Ogre::SceneManager* sm = context_->sceneManager();

    // Texture with the image pixels (created once per image id; ids are never
    // reused, so the cache key is stable).
    const std::string texName = "avb/cfgtex/" + std::to_string(imageId);
    const std::string matName = "avb/cfgmat/" + std::to_string(imageId);
    auto& tm = Ogre::TextureManager::getSingleton();
    if (!tm.getByName(texName,
                      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME)) {
        cv::Mat rgba;
        cv::cvtColor(img->pixels, rgba, cv::COLOR_BGR2RGBA);
        Ogre::TexturePtr tex = tm.createManual(
            texName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            Ogre::TEX_TYPE_2D, static_cast<unsigned int>(rgba.cols),
            static_cast<unsigned int>(rgba.rows), 0, Ogre::PF_BYTE_RGBA,
            Ogre::TU_DEFAULT);
        const Ogre::PixelBox src(static_cast<unsigned int>(rgba.cols),
                                 static_cast<unsigned int>(rgba.rows), 1,
                                 Ogre::PF_BYTE_RGBA, rgba.data);
        tex->getBuffer()->blitFromMemory(src);
    }
    auto& mm = Ogre::MaterialManager::getSingleton();
    if (!mm.getByName(matName,
                      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME)) {
        Ogre::MaterialPtr mat = mm.create(
            matName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
        pass->setLightingEnabled(false);   // show the picture at full brightness
        pass->setCullingMode(Ogre::CULL_NONE);  // visible from behind too
        pass->createTextureUnitState(texName);
    }

    // Quad in the tracker's image-plane convention: width 1, centred at the
    // origin, +Y up, +Z toward the viewer; the image's top edge is at +Y.
    const float halfH =
        0.5f * static_cast<float>(img->pixels.rows) / img->pixels.cols;
    if (!plane_) {
        plane_ = sm->createManualObject("avb_cfg_plane");
        plane_->setDynamic(true);
        planeNode_ = root_->createChildSceneNode("avb_cfg_planeNode");
        planeNode_->attachObject(plane_);
    } else {
        plane_->clear();
    }
    plane_->begin(matName, Ogre::RenderOperation::OT_TRIANGLE_LIST,
                  Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    plane_->position(-0.5f, halfH, 0.0f);
    plane_->textureCoord(0.0f, 0.0f);
    plane_->position(0.5f, halfH, 0.0f);
    plane_->textureCoord(1.0f, 0.0f);
    plane_->position(0.5f, -halfH, 0.0f);
    plane_->textureCoord(1.0f, 1.0f);
    plane_->position(-0.5f, -halfH, 0.0f);
    plane_->textureCoord(0.0f, 1.0f);
    plane_->triangle(0, 1, 2);
    plane_->triangle(0, 2, 3);
    plane_->end();
    plane_->setVisibilityFlags(kConfigPreviewVisibilityMask);

    currentImageId_ = imageId;
    return true;
}

Ogre::SceneNode* ConfigurePreview::ensureModelNode(Id modelId,
                                                   std::size_t index) {
    std::vector<Ogre::SceneNode*>& pool = modelNodes_[modelId];
    if (index < pool.size()) {
        return pool[index];
    }
    const ModelAsset* model = store_->model(modelId);
    if (!model) {
        return nullptr;
    }
    // The mesh is shared with SceneRenderer through ModelLoader's cache; only
    // the entity instance (and its visibility flag) is preview-specific.
    const std::string meshName = "avb/mesh/" + std::to_string(modelId);
    const std::string mesh = loader_->loadModel(model->filePath, meshName);
    if (mesh.empty()) {
        return nullptr;
    }
    Ogre::SceneManager* sm = context_->sceneManager();
    Ogre::Entity* entity = sm->createEntity(
        "avb/cfg/ent/" + std::to_string(modelId) + "/" +
            std::to_string(pool.size()),
        mesh);
    entity->setVisibilityFlags(kConfigPreviewVisibilityMask);
    Ogre::SceneNode* node = root_->createChildSceneNode();
    node->attachObject(entity);
    node->setVisible(false);
    pool.push_back(node);
    return node;
}

bool ConfigurePreview::render(Id imageId, const std::vector<ModelPose>& models,
                              const Eigen::Vector3f& eye, float fovYDeg,
                              int width, int height) {
    if (width < 16 || height < 16) {
        return false;
    }

    // All GPU work below must happen in OGRE's own GL context (the ImGui
    // windows made theirs current since the last OGRE call).
    context_->makeRenderContextCurrent();

    if (!ensureInitialized() || !ensureTarget(width, height) ||
        !ensureImagePlane(imageId)) {
        return false;
    }

    // Show exactly the requested models: everything else (models of other
    // images, models/instances unassigned since the last render) is hidden.
    for (auto& [id, pool] : modelNodes_) {
        for (Ogre::SceneNode* node : pool) {
            node->setVisible(false);
        }
    }
    // The same model may appear several times (assigned to the image more
    // than once); each occurrence gets its own node instance.
    std::unordered_map<Id, std::size_t> instancesUsed;
    for (const ModelPose& m : models) {
        Ogre::SceneNode* modelNode =
            ensureModelNode(m.modelId, instancesUsed[m.modelId]);
        if (!modelNode) {
            continue; // model failed to load; still render the others
        }
        ++instancesUsed[m.modelId];
        applyPose(modelNode, m.pose);
        modelNode->setVisible(true);
    }

    cameraNode_->setPosition(eye.x(), eye.y(), eye.z());
    cameraNode_->lookAt(Ogre::Vector3::ZERO, Ogre::Node::TS_WORLD);
    camera_->setFOVy(Ogre::Degree(fovYDeg));

    renderTarget_->update();

    // PF_BYTE_RGBA = byte-order R,G,B,A, matching the cv::Mat view (see the
    // same readback in SceneRenderer::endFrame).
    Ogre::PixelBox pb(static_cast<unsigned int>(width_),
                      static_cast<unsigned int>(height_), 1,
                      Ogre::PF_BYTE_RGBA, readback_.data());
    renderTarget_->copyContentsToMemory(
        Ogre::Box(0, 0, static_cast<unsigned int>(width_),
                  static_cast<unsigned int>(height_)),
        pb, Ogre::RenderTarget::FB_AUTO);
    cv::Mat(height_, width_, CV_8UC4, readback_.data()).copyTo(output_);
    return true;
}

} // namespace avb
