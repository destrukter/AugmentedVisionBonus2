#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "storage/Types.h"

namespace Ogre {
class Camera;
class Entity;
class SceneNode;
class Light;
class RenderTexture;
} // namespace Ogre

namespace avb {

class OgreContext;
class ModelLoader;
class DataStore;

/// Renders assigned FBX models over the live camera frame for the Camera window.
///
/// To avoid sharing GL contexts between OGRE and the window's SDL/GL context,
/// OGRE renders the models (over a transparent background) into an off-screen
/// render texture; the result is read back to the CPU and alpha-composited over
/// the camera frame. The Camera window then uploads compositedImage() to its own
/// GL texture for display.
///
/// Per-frame usage:
///   beginFrame(cameraFrame)  - cache the frame, hide all model nodes
///   drawModel(modelId, pose) - place/show the model at a camera-space pose
///   endFrame()               - render, read back, composite
class SceneRenderer {
public:
    SceneRenderer(std::shared_ptr<OgreContext> context,
                  std::shared_ptr<ModelLoader> loader,
                  std::shared_ptr<DataStore> store);
    ~SceneRenderer();

    /// Creates the camera, light and off-screen render target of the given size.
    bool initialize(int width, int height);

    void beginFrame(const cv::Mat& cameraFrame);
    /// Places an instance of the model for `modelId` at `pose` (a 4x4
    /// camera-space matrix) and makes it visible. Lazily loads/instantiates
    /// the mesh on first use. May be called several times per frame for the
    /// same model (e.g. the same model assigned to two tracked images); each
    /// call places a separate instance.
    void drawModel(Id modelId, const Eigen::Matrix4f& pose);
    void endFrame();

    /// Computes a camera-space framing pose for previewing `modelId` with
    /// `configured` (the assignment transform) applied: the model is centered,
    /// tilted (plus `yawDeg` for a turntable spin) and pushed back far enough
    /// that its whole bounding sphere fits the view - a fixed distance would
    /// put the camera inside large models or show flat ones edge-on as a bare
    /// sliver. Pass the result to drawModel as `pose * configured`. Returns
    /// false when the model cannot be loaded.
    bool previewFramingPose(Id modelId, const Eigen::Matrix4f& configured,
                            float yawDeg, Eigen::Matrix4f& outPose);

    /// Composited RGBA image (camera frame + rendered models). Empty until the
    /// first endFrame(). CV_8UC4.
    const cv::Mat& compositedImage() const { return composited_; }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    /// One placed copy of a model. Several instances of the same model exist
    /// when it is drawn more than once per frame (same model assigned to
    /// multiple tracked images).
    struct ModelInstance {
        Ogre::SceneNode* node{nullptr};
        Ogre::Entity* entity{nullptr};
    };
    struct InstancePool {
        std::vector<ModelInstance> instances;
        std::size_t used{0};  // instances handed out since beginFrame()
    };

    /// Returns the next unused instance of `modelId` for this frame, creating
    /// node+entity (sharing the cached mesh) when the pool is exhausted.
    /// Returns nullptr when the model cannot be loaded.
    ModelInstance* acquireInstance(Id modelId);
    /// Ensures at least one instance of `modelId` exists (without consuming it
    /// from this frame's pool) - used for bounding-box queries.
    ModelInstance* ensureFirstInstance(Id modelId);
    /// (Re)creates the off-screen render target at the given size. Called from
    /// initialize() and again whenever the camera frame size changes.
    bool createRenderTarget(int width, int height);
    void destroyRenderTarget();

    std::shared_ptr<OgreContext> context_;
    std::shared_ptr<ModelLoader> loader_;
    std::shared_ptr<DataStore> store_;

    Ogre::Camera* camera_{nullptr};
    Ogre::SceneNode* cameraNode_{nullptr};
    Ogre::SceneNode* worldRoot_{nullptr};
    Ogre::Light* light_{nullptr};
    Ogre::RenderTexture* renderTarget_{nullptr};
    std::string rttName_;

    // modelId -> instance pool (instances created on demand).
    std::unordered_map<Id, InstancePool> pools_;

    cv::Mat cameraFrame_;            // latest BGR frame (may be empty)
    cv::Mat composited_;             // RGBA output
    cv::Mat drawnMask_;              // compositing scratch (see endFrame)
    cv::Mat solidAlpha_;             // cached all-255 plane for opaque output
    std::vector<unsigned char> readback_;  // RGBA scratch for RTT readback

    int width_{0};
    int height_{0};
    int visibleModels_{0};           // models drawn since beginFrame()
    bool initialized_{false};
};

} // namespace avb
