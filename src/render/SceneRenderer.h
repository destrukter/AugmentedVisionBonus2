#pragma once

#include <memory>
#include <unordered_map>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "storage/Types.h"

namespace avb {

class OgreContext;
class ModelLoader;
class DataStore;

/// Renders assigned FBX models over the live camera background for the Camera
/// window, using OGRE3D.
///
/// Workflow per frame:
///   beginFrame(cameraImage)  - upload the feed as the scene background
///   drawModel(modelId, pose) - place/update the model's scene node at `pose`
///   endFrame()               - render and expose compositedTexture()
class SceneRenderer {
public:
    SceneRenderer(std::shared_ptr<OgreContext> context,
                  std::shared_ptr<ModelLoader> loader,
                  std::shared_ptr<DataStore> store);
    ~SceneRenderer();

    bool initialize();

    void beginFrame(const cv::Mat& cameraImage);
    /// Places the model for `modelId` at `pose` (a 4x4 camera-space matrix).
    /// Lazily loads/instantiates the model's mesh on first use.
    void drawModel(Id modelId, const Eigen::Matrix4f& pose);
    void endFrame();

    /// Native handle of the composited texture (feed + models) for ImGui::Image.
    void* compositedTexture() const { return compositedTexture_; }

private:
    std::shared_ptr<OgreContext> context_;
    std::shared_ptr<ModelLoader> loader_;
    std::shared_ptr<DataStore> store_;

    // modelId -> OGRE scene node handle (created on demand).
    std::unordered_map<Id, void*> nodes_;
    void* compositedTexture_{nullptr};
};

} // namespace avb
