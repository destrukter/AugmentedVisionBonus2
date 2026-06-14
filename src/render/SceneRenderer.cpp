#include "render/SceneRenderer.h"

#include <utility>

#include <opencv2/imgproc.hpp>

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreLight.h>
#include <OgreRenderTexture.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

#include "render/ModelLoader.h"
#include "render/OgreContext.h"
#include "storage/DataStore.h"

namespace avb {

SceneRenderer::SceneRenderer(std::shared_ptr<OgreContext> context,
                             std::shared_ptr<ModelLoader> loader,
                             std::shared_ptr<DataStore> store)
    : context_(std::move(context)),
      loader_(std::move(loader)),
      store_(std::move(store)) {}

SceneRenderer::~SceneRenderer() {
    if (!rttName_.empty()) {
        Ogre::TextureManager::getSingleton().remove(
            rttName_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    }
}

bool SceneRenderer::initialize(int width, int height) {
    if (width <= 0 || height <= 0) {
        return false;
    }
    width_ = width;
    height_ = height;

    Ogre::SceneManager* sm = context_->sceneManager();
    sm->setAmbientLight(Ogre::ColourValue(0.3f, 0.3f, 0.3f));

    worldRoot_ = sm->getRootSceneNode()->createChildSceneNode("avb_world");

    camera_ = sm->createCamera("avb_camera");
    camera_->setNearClipDistance(0.05f);
    camera_->setAutoAspectRatio(true);
    cameraNode_ = sm->getRootSceneNode()->createChildSceneNode("avb_cameraNode");
    cameraNode_->attachObject(camera_);
    // Camera looks down -Z (OpenGL/OGRE convention); tracker poses are expressed
    // in this camera space.

    light_ = sm->createLight("avb_light");
    light_->setType(Ogre::Light::LT_DIRECTIONAL);
    Ogre::SceneNode* lightNode =
        sm->getRootSceneNode()->createChildSceneNode("avb_lightNode");
    lightNode->attachObject(light_);
    lightNode->setDirection(Ogre::Vector3(-0.5f, -1.0f, -0.5f).normalisedCopy());

    // Off-screen render target with alpha so models composite over the feed.
    rttName_ = "avb_rtt";
    Ogre::TexturePtr rtt = Ogre::TextureManager::getSingleton().createManual(
        rttName_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, static_cast<unsigned int>(width_),
        static_cast<unsigned int>(height_), 0, Ogre::PF_R8G8B8A8,
        Ogre::TU_RENDERTARGET);
    if (!rtt) {
        return false;
    }
    renderTarget_ = rtt->getBuffer()->getRenderTarget();
    Ogre::Viewport* vp = renderTarget_->addViewport(camera_);
    vp->setBackgroundColour(Ogre::ColourValue(0, 0, 0, 0)); // transparent
    vp->setClearEveryFrame(true);
    vp->setOverlaysEnabled(false);
    renderTarget_->setAutoUpdated(false);

    readback_.assign(static_cast<size_t>(width_) * height_ * 4, 0);
    initialized_ = true;
    return true;
}

void SceneRenderer::beginFrame(const cv::Mat& cameraFrame) {
    cameraFrame_ = cameraFrame; // shallow ref; copied during compositing
    for (auto& [id, node] : nodes_) {
        node->setVisible(false);
    }
}

Ogre::SceneNode* SceneRenderer::ensureNode(Id modelId) {
    if (const auto it = nodes_.find(modelId); it != nodes_.end()) {
        return it->second;
    }
    const ModelAsset* model = store_->model(modelId);
    if (!model) {
        return nullptr;
    }
    const std::string meshName = "avb/mesh/" + std::to_string(modelId);
    const std::string mesh = loader_->loadFbx(model->filePath, meshName);
    if (mesh.empty()) {
        return nullptr;
    }
    Ogre::SceneManager* sm = context_->sceneManager();
    Ogre::Entity* entity = sm->createEntity("avb/ent/" + std::to_string(modelId), mesh);
    Ogre::SceneNode* node = worldRoot_->createChildSceneNode();
    node->attachObject(entity);
    nodes_.emplace(modelId, node);
    return node;
}

void SceneRenderer::drawModel(Id modelId, const Eigen::Matrix4f& pose) {
    Ogre::SceneNode* node = ensureNode(modelId);
    if (!node) {
        return;
    }
    // Decompose the 4x4 pose into translation, rotation and per-axis scale.
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
    node->setVisible(true);
}

void SceneRenderer::endFrame() {
    if (!initialized_) {
        return;
    }
    renderTarget_->update();

    // Read the rendered RGBA overlay back to the CPU.
    Ogre::PixelBox pb(static_cast<unsigned int>(width_),
                      static_cast<unsigned int>(height_), 1, Ogre::PF_R8G8B8A8,
                      readback_.data());
    renderTarget_->copyContentsToMemory(
        Ogre::Box(0, 0, static_cast<unsigned int>(width_),
                  static_cast<unsigned int>(height_)),
        pb, Ogre::RenderTarget::FB_AUTO);

    cv::Mat overlay(height_, width_, CV_8UC4, readback_.data()); // RGBA

    // Build the RGBA background from the camera frame (BGR -> RGBA), or black.
    if (composited_.empty() || composited_.rows != height_ ||
        composited_.cols != width_) {
        composited_.create(height_, width_, CV_8UC4);
    }
    if (cameraFrame_.empty()) {
        composited_.setTo(cv::Scalar(0, 0, 0, 255));
    } else {
        cv::Mat resized;
        if (cameraFrame_.cols != width_ || cameraFrame_.rows != height_) {
            cv::resize(cameraFrame_, resized, cv::Size(width_, height_));
        } else {
            resized = cameraFrame_;
        }
        cv::cvtColor(resized, composited_, cv::COLOR_BGR2RGBA);
    }

    // Alpha-composite the overlay over the background (straight alpha).
    for (int y = 0; y < height_; ++y) {
        const cv::Vec4b* o = overlay.ptr<cv::Vec4b>(y);
        cv::Vec4b* d = composited_.ptr<cv::Vec4b>(y);
        for (int x = 0; x < width_; ++x) {
            const int a = o[x][3];
            if (a == 0) {
                continue;
            }
            const int ia = 255 - a;
            d[x][0] = static_cast<uchar>((o[x][0] * a + d[x][0] * ia) / 255);
            d[x][1] = static_cast<uchar>((o[x][1] * a + d[x][1] * ia) / 255);
            d[x][2] = static_cast<uchar>((o[x][2] * a + d[x][2] * ia) / 255);
            d[x][3] = 255;
        }
    }
}

} // namespace avb
