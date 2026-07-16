#include "render/SceneRenderer.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>

#include <OgreAnimationState.h>
#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreLight.h>
#include <OgreRenderTexture.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

#include <RTShaderSystem/OgreShaderGenerator.h>

#include "render/ModelLoader.h"
#include "render/OgreContext.h"
#include "storage/DataStore.h"
#include "vision/ImageTracker.h"

namespace avb {

namespace {

/// Enables the entity's first animation (looping) and returns its state, or
/// null when the mesh carries no animations. Playing the first animation
/// automatically means an animated FBX moves without any configuration.
Ogre::AnimationState* enableFirstAnimation(Ogre::Entity* entity) {
    Ogre::AnimationStateSet* states = entity->getAllAnimationStates();
    if (!states) {
        return nullptr;
    }
    auto it = states->getAnimationStateIterator();
    if (!it.hasMoreElements()) {
        return nullptr;
    }
    Ogre::AnimationState* state = it.getNext();
    state->setEnabled(true);
    state->setLoop(true);
    return state;
}

} // namespace

SceneRenderer::SceneRenderer(std::shared_ptr<OgreContext> context,
                             std::shared_ptr<ModelLoader> loader,
                             std::shared_ptr<DataStore> store)
    : context_(std::move(context)),
      loader_(std::move(loader)),
      store_(std::move(store)) {}

SceneRenderer::~SceneRenderer() {
    destroyRenderTarget();
}

bool SceneRenderer::initialize(int width, int height) {
    if (width <= 0 || height <= 0) {
        return false;
    }

    Ogre::SceneManager* sm = context_->sceneManager();
    sm->setAmbientLight(Ogre::ColourValue(0.3f, 0.3f, 0.3f));

    worldRoot_ = sm->getRootSceneNode()->createChildSceneNode("avb_world");

    camera_ = sm->createCamera("avb_camera");
    camera_->setNearClipDistance(0.05f);
    camera_->setAutoAspectRatio(true);
    // Must match the vertical FOV the tracker assumes for pose estimation when
    // no calibrated intrinsics are provided, or rendered models drift off their
    // tracked image as it moves toward the frame edges.
    camera_->setFOVy(Ogre::Degree(ImageTracker::kDefaultFovYDeg));
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

    if (!createRenderTarget(width, height)) {
        return false;
    }
    initialized_ = true;
    return true;
}

bool SceneRenderer::createRenderTarget(int width, int height) {
    destroyRenderTarget();

    rttName_ = "avb_rtt";
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
    // Clear to the exact chroma-key color the compositor tests for (bytes
    // R=1, G=0, B=255) - a color lit geometry essentially never produces, so
    // even pure-black materials survive compositing. See endFrame().
    vp->setBackgroundColour(Ogre::ColourValue(1.0f / 255.0f, 0.0f, 1.0f, 0.0f));
    vp->setClearEveryFrame(true);
    vp->setOverlaysEnabled(false);
    // Route this viewport's material lookups through the RTSS scheme so
    // OgreContext's scheme-not-found listener generates real GLSL techniques
    // for our materials. Without this, the render-system-agnostic
    // fixed-function-style passes (lighting/diffuse/specular, no shader
    // programs) never get a technique the GL3Plus render system - which has
    // no fixed-function pipeline - can actually draw, so geometry renders
    // invisibly.
    vp->setMaterialScheme(Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
    // Render only the AR scene; the Configure window's preview objects share
    // the scene manager but carry a different visibility flag.
    vp->setVisibilityMask(kMainSceneVisibilityMask);
    renderTarget_->setAutoUpdated(false);

    readback_.assign(static_cast<size_t>(width_) * height_ * 4, 0);
    return true;
}

void SceneRenderer::destroyRenderTarget() {
    if (rttName_.empty()) {
        return;
    }
    Ogre::TextureManager::getSingleton().remove(
        rttName_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    renderTarget_ = nullptr;
    rttName_.clear();
}

void SceneRenderer::beginFrame(const cv::Mat& cameraFrame) {
    // The ImGui windows made their own GL contexts current since the last
    // frame; re-bind OGRE's before any mesh upload / render / readback this
    // frame, or those calls land in whichever context is bound and the RTT
    // reads back stale garbage. (Previously this happened as a side effect of
    // the since-removed debug window's update bouncing OGRE's context.)
    context_->makeRenderContextCurrent();
    cameraFrame_ = cameraFrame; // shallow ref; copied during compositing
    // Track at the camera's native resolution: sizing the render target to the
    // frame keeps the OGRE projection aligned with the tracker's intrinsics
    // (which are derived from the frame size) and removes the per-frame
    // cv::resize of the feed. Camera resolutions change at most once, on the
    // first delivered frame.
    if (initialized_ && !cameraFrame.empty() &&
        (cameraFrame.cols != width_ || cameraFrame.rows != height_)) {
        createRenderTarget(cameraFrame.cols, cameraFrame.rows);
    }
    for (auto& [id, pool] : pools_) {
        for (ModelInstance& instance : pool.instances) {
            instance.node->setVisible(false);
        }
        pool.used = 0;
    }
    visibleModels_ = 0;
}

SceneRenderer::ModelInstance* SceneRenderer::ensureFirstInstance(Id modelId) {
    InstancePool& pool = pools_[modelId];
    if (!pool.instances.empty()) {
        return &pool.instances.front();
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
    ModelInstance instance;
    instance.entity =
        sm->createEntity("avb/ent/" + std::to_string(modelId) + "/0", mesh);
    instance.entity->setVisibilityFlags(kMainSceneVisibilityMask);
    instance.animation = enableFirstAnimation(instance.entity);
    instance.node = worldRoot_->createChildSceneNode();
    instance.node->attachObject(instance.entity);
    pool.instances.push_back(instance);
    return &pool.instances.front();
}

SceneRenderer::ModelInstance* SceneRenderer::acquireInstance(Id modelId) {
    // Creates the pool (and validates the model loads) via the first instance.
    if (!ensureFirstInstance(modelId)) {
        return nullptr;
    }
    InstancePool& pool = pools_[modelId];
    if (pool.used < pool.instances.size()) {
        return &pool.instances[pool.used++];
    }
    // Same model drawn again this frame (e.g. assigned to a second tracked
    // image): clone another entity of the shared mesh.
    Ogre::SceneManager* sm = context_->sceneManager();
    ModelInstance instance;
    instance.entity = sm->createEntity(
        "avb/ent/" + std::to_string(modelId) + "/" +
            std::to_string(pool.instances.size()),
        pool.instances.front().entity->getMesh());
    instance.entity->setVisibilityFlags(kMainSceneVisibilityMask);
    instance.animation = enableFirstAnimation(instance.entity);
    instance.node = worldRoot_->createChildSceneNode();
    instance.node->attachObject(instance.entity);
    pool.instances.push_back(instance);
    ++pool.used;
    return &pool.instances.back();
}

void SceneRenderer::drawModel(Id modelId, const Eigen::Matrix4f& pose) {
    ModelInstance* instance = acquireInstance(modelId);
    if (!instance) {
        return;
    }
    Ogre::SceneNode* node = instance->node;
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
    ++visibleModels_;
}

bool SceneRenderer::previewFramingPose(Id modelId,
                                       const Eigen::Matrix4f& configured,
                                       float yawDeg, Eigen::Matrix4f& outPose) {
    ModelInstance* instance = ensureFirstInstance(modelId);
    if (!instance || !instance->entity) {
        return false;
    }
    const Ogre::AxisAlignedBox& box = instance->entity->getBoundingBox();
    if (box.isNull() || box.isInfinite()) {
        return false;
    }

    // Bounding sphere of the model with the configured transform applied
    // (scale in `configured` must influence the framing distance).
    const Ogre::Vector3 mn = box.getMinimum();
    const Ogre::Vector3 mx = box.getMaximum();
    Eigen::Vector3f corners[8];
    int n = 0;
    for (int ix = 0; ix < 2; ++ix) {
        for (int iy = 0; iy < 2; ++iy) {
            for (int iz = 0; iz < 2; ++iz) {
                const Eigen::Vector4f c(ix ? mx.x : mn.x, iy ? mx.y : mn.y,
                                        iz ? mx.z : mn.z, 1.0f);
                corners[n++] = (configured * c).head<3>();
            }
        }
    }
    Eigen::Vector3f lo = corners[0];
    Eigen::Vector3f hi = corners[0];
    for (int i = 1; i < 8; ++i) {
        lo = lo.cwiseMin(corners[i]);
        hi = hi.cwiseMax(corners[i]);
    }
    const Eigen::Vector3f center = 0.5f * (lo + hi);
    float radius = 1e-3f;
    for (const Eigen::Vector3f& c : corners) {
        radius = std::max(radius, (c - center).norm());
    }

    // Distance at which the bounding sphere fits the tighter half-FOV
    // (sin(halfFov) = radius / distance), with some margin.
    constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
    const float fovY = ImageTracker::kDefaultFovYDeg * kDegToRad;
    const float aspect =
        height_ > 0 ? static_cast<float>(width_) / static_cast<float>(height_)
                    : 1.0f;
    const float fovX = 2.0f * std::atan(std::tan(fovY * 0.5f) * aspect);
    const float halfFov = 0.5f * std::min(fovX, fovY);
    const float distance =
        std::max(1.15f * radius / std::sin(halfFov), 0.1f + 1.2f * radius);

    // Slight fixed tilt plus the caller-provided turntable yaw, rotating about
    // the model's (transformed) bound center so it stays centered in view.
    const Eigen::Matrix3f rot =
        (Eigen::AngleAxisf(-20.0f * kDegToRad, Eigen::Vector3f::UnitX()) *
         Eigen::AngleAxisf(yawDeg * kDegToRad, Eigen::Vector3f::UnitY()))
            .toRotationMatrix();

    // outPose * x = rot * (x - center) + (0, 0, -distance)
    outPose = Eigen::Matrix4f::Identity();
    outPose.block<3, 3>(0, 0) = rot;
    outPose.block<3, 1>(0, 3) =
        Eigen::Vector3f(0.0f, 0.0f, -distance) - rot * center;
    return true;
}

void SceneRenderer::endFrame() {
    if (!initialized_) {
        return;
    }

    // Real time elapsed since the previous frame, for animation playback.
    // Clamped so a stall (window drag, camera hiccup) doesn't make animations
    // leap; measured every frame so playback resumes smoothly regardless of
    // how long no model was visible.
    const auto now = std::chrono::steady_clock::now();
    float animDt = 0.0f;
    if (haveAnimTick_) {
        animDt = std::chrono::duration<float>(now - lastAnimTick_).count();
    }
    lastAnimTick_ = now;
    haveAnimTick_ = true;
    animDt = std::min(animDt, 0.1f);

    // Build the RGBA background from the camera frame (BGR -> RGBA), or black.
    // beginFrame() sized the render target to the frame, so no resize is needed.
    if (composited_.empty() || composited_.rows != height_ ||
        composited_.cols != width_) {
        composited_.create(height_, width_, CV_8UC4);
    }
    if (cameraFrame_.empty()) {
        composited_.setTo(cv::Scalar(0, 0, 0, 255));
    } else {
        cv::cvtColor(cameraFrame_, composited_, cv::COLOR_BGR2RGBA);
    }

    if (visibleModels_ == 0 || !renderTarget_) {
        return; // nothing rendered: skip the OGRE pass and GPU readback entirely
    }

    // Advance the animations of every instance drawn this frame.
    if (animDt > 0.0f) {
        for (auto& [id, pool] : pools_) {
            for (std::size_t i = 0; i < pool.used; ++i) {
                if (pool.instances[i].animation) {
                    pool.instances[i].animation->addTime(animDt);
                }
            }
        }
    }

    renderTarget_->update();

    // Read the rendered overlay back to the CPU. PF_BYTE_RGBA requests
    // byte-order R,G,B,A - which is what the cv::Mat view below assumes.
    // (PF_R8G8B8A8 is an int-packed format whose little-endian memory layout
    // is A,B,G,R; using it here put the FBO's alpha byte into the red channel,
    // tinting models red on drivers that return alpha=255 and cyan-blue on
    // drivers that return alpha=0.)
    Ogre::PixelBox pb(static_cast<unsigned int>(width_),
                      static_cast<unsigned int>(height_), 1, Ogre::PF_BYTE_RGBA,
                      readback_.data());
    renderTarget_->copyContentsToMemory(
        Ogre::Box(0, 0, static_cast<unsigned int>(width_),
                  static_cast<unsigned int>(height_)),
        pb, Ogre::RenderTarget::FB_AUTO);

    const cv::Mat overlay(height_, width_, CV_8UC4, readback_.data()); // RGBA

    // Composite the rendered overlay over the camera background.
    //
    // We key on the overlay's RGB (the exact clear colour set in
    // createRenderTarget) rather than its alpha channel: many Linux GL
    // drivers hand the FBO's alpha back as constant through
    // copyContentsToMemory, which would either blank the feed or the models.
    // The key colour (1, 0, 255) is one lit geometry essentially never
    // produces exactly, so even pure-black materials composite correctly.
    // Models are opaque, so a masked copy replaces a per-pixel alpha blend
    // (SIMD via OpenCV instead of a scalar loop over every pixel).
    cv::Mat clearMask; // 255 where the overlay is the key colour (any alpha)
    cv::inRange(overlay, cv::Scalar(1, 0, 255, 0), cv::Scalar(1, 0, 255, 255),
                clearMask);
    cv::bitwise_not(clearMask, drawnMask_);
    overlay.copyTo(composited_, drawnMask_);
    // The overlay's alpha is driver-dependent; force the output opaque so the
    // display texture never blends against the UI background.
    if (solidAlpha_.size() != composited_.size()) {
        solidAlpha_.create(composited_.size(), CV_8UC1);
        solidAlpha_.setTo(255);
    }
    cv::insertChannel(solidAlpha_, composited_, 3);
}

} // namespace avb
