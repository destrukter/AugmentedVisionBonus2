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
    /// Places the model for `modelId` at `pose` (a 4x4 camera-space matrix) and
    /// makes it visible. Lazily loads/instantiates the mesh on first use.
    void drawModel(Id modelId, const Eigen::Matrix4f& pose);
    void endFrame();

    /// Composited RGBA image (camera frame + rendered models). Empty until the
    /// first endFrame(). CV_8UC4.
    const cv::Mat& compositedImage() const { return composited_; }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    Ogre::SceneNode* ensureNode(Id modelId);

    std::shared_ptr<OgreContext> context_;
    std::shared_ptr<ModelLoader> loader_;
    std::shared_ptr<DataStore> store_;

    Ogre::Camera* camera_{nullptr};
    Ogre::SceneNode* cameraNode_{nullptr};
    Ogre::SceneNode* worldRoot_{nullptr};
    Ogre::Light* light_{nullptr};
    Ogre::RenderTexture* renderTarget_{nullptr};
    std::string rttName_;

    // modelId -> scene node (created on demand).
    std::unordered_map<Id, Ogre::SceneNode*> nodes_;

    cv::Mat cameraFrame_;            // latest BGR frame (may be empty)
    cv::Mat composited_;             // RGBA output
    std::vector<unsigned char> readback_;  // RGBA scratch for RTT readback

    int width_{0};
    int height_{0};
    bool initialized_{false};
};

} // namespace avb
