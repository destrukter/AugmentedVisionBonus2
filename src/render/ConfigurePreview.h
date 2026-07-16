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
class ManualObject;
class RenderTexture;
class SceneNode;
} // namespace Ogre

namespace avb {

class OgreContext;
class ModelLoader;
class DataStore;

/// Renders the Configure window's viewport content - the image as a textured
/// plane (width 1, matching the tracker's convention) plus every model
/// assigned to it at its working pose - through OGRE into an off-screen
/// target. The window draws the result as the background of the gizmo
/// viewport, so the user manipulates the real models over the real picture
/// instead of proxy shapes.
///
/// Shares SceneRenderer's scene manager; the two renders are isolated with
/// visibility masks (kConfigPreviewVisibilityMask vs kMainSceneVisibilityMask)
/// so neither shows the other's objects. The scene's light and ambient level
/// are owned by SceneRenderer::initialize(), which must have run first. Like
/// SceneRenderer, render() must be driven from the application loop outside
/// any ImGui window's GL pass; it re-binds OGRE's GL context itself.
class ConfigurePreview {
public:
    /// One model to draw, at a full pose (origin * working transform)
    /// relative to the image plane.
    struct ModelPose {
        Id modelId{kInvalidId};
        Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};
    };

    ConfigurePreview(std::shared_ptr<OgreContext> context,
                     std::shared_ptr<ModelLoader> loader,
                     std::shared_ptr<DataStore> store);
    ~ConfigurePreview();

    ConfigurePreview(const ConfigurePreview&) = delete;
    ConfigurePreview& operator=(const ConfigurePreview&) = delete;

    /// Renders `imageId`'s picture plane plus all `models` as seen from `eye`
    /// looking at the image-plane origin (up = +Y), with vertical FOV
    /// `fovYDeg`, into a `width` x `height` target. The caller must use the
    /// same eye/FOV/aspect for its gizmo matrices so handles line up with the
    /// rendered pixels. Models whose mesh cannot be loaded are skipped.
    /// Returns false (rendering nothing) when the image pixels or the render
    /// target are unavailable.
    bool render(Id imageId, const std::vector<ModelPose>& models,
                const Eigen::Vector3f& eye, float fovYDeg, int width,
                int height);

    /// The last successful render (RGBA); empty before the first success.
    const cv::Mat& image() const { return output_; }

private:
    bool ensureInitialized();
    bool ensureTarget(int width, int height);
    void destroyTarget();
    bool ensureImagePlane(Id imageId);
    Ogre::SceneNode* ensureModelNode(Id modelId);

    std::shared_ptr<OgreContext> context_;
    std::shared_ptr<ModelLoader> loader_;
    std::shared_ptr<DataStore> store_;

    Ogre::Camera* camera_{nullptr};
    Ogre::SceneNode* cameraNode_{nullptr};
    Ogre::SceneNode* root_{nullptr};        // parent of plane + model nodes
    Ogre::SceneNode* planeNode_{nullptr};
    Ogre::ManualObject* plane_{nullptr};
    Ogre::RenderTexture* renderTarget_{nullptr};
    std::string rttName_;

    Id currentImageId_{kInvalidId};
    std::unordered_map<Id, Ogre::SceneNode*> modelNodes_;

    cv::Mat output_;                        // RGBA result
    std::vector<unsigned char> readback_;
    int width_{0};
    int height_{0};
    bool initialized_{false};
};

} // namespace avb
