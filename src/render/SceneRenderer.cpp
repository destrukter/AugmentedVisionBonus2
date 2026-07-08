#include "render/SceneRenderer.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>

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
    vp->setBackgroundColour(Ogre::ColourValue(0, 0, 0, 0)); // transparent
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
    for (auto& [id, node] : nodes_) {
        node->setVisible(false);
    }
    visibleModels_ = 0;
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
    ++visibleModels_;
}

bool SceneRenderer::previewFramingPose(Id modelId,
                                       const Eigen::Matrix4f& configured,
                                       float yawDeg, Eigen::Matrix4f& outPose) {
    Ogre::SceneNode* node = ensureNode(modelId);
    if (!node || node->numAttachedObjects() == 0) {
        return false;
    }
    const Ogre::AxisAlignedBox& box = node->getAttachedObject(0)->getBoundingBox();
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
    // We key on the overlay's RGB (the clear colour) rather than its alpha
    // channel. The off-screen render target is cleared to transparent black, but
    // many Linux GL drivers hand the FBO's alpha channel back as fully opaque
    // (255) everywhere through copyContentsToMemory. With a straight-alpha
    // composite that would blend the overlay's black clear colour over the whole
    // camera frame and blank the feed entirely. Treating pure-black overlay
    // pixels as "nothing drawn here" keeps the camera visible wherever no model
    // was rendered, regardless of how the driver reports alpha; rendered model
    // pixels are lit and therefore non-black. Models are opaque, so a masked
    // copy replaces the old per-pixel alpha blend (SIMD via OpenCV instead of
    // a scalar loop over every pixel).
    cv::Mat clearMask; // 255 where the overlay is pure black in RGB (any alpha)
    cv::inRange(overlay, cv::Scalar(0, 0, 0, 0), cv::Scalar(0, 0, 0, 255),
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
