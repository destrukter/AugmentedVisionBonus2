#pragma once

#include <memory>
#include <string>

namespace Ogre {
class Root;
class SceneManager;
class RenderWindow;
namespace RTShader {
class ShaderGenerator;
}
} // namespace Ogre

namespace avb {

/// Owns the OGRE3D Root, render system, an off-screen GL context and the shared
/// scene manager + RTSS shader generator.
///
/// A single OgreContext is created by the Application and shared with the
/// SceneRenderer used by the Camera window. It loads the render-system plugin
/// programmatically (no fragile plugins.cfg path required) and creates a hidden
/// 1x1 render window so a GL context exists for GPU resources without showing an
/// extra OS window.
class OgreContext {
public:
    OgreContext();
    ~OgreContext();

    OgreContext(const OgreContext&) = delete;
    OgreContext& operator=(const OgreContext&) = delete;

    /// Initializes Root, the render system, the hidden context window, the scene
    /// manager and the RTSS shader generator. Returns false on failure.
    bool initialize();

    Ogre::Root* root() const { return root_.get(); }
    Ogre::SceneManager* sceneManager() const { return sceneManager_; }
    Ogre::RTShader::ShaderGenerator* shaderGenerator() const { return shaderGen_; }

private:
    /// Loads the first available GL render-system plugin from common locations
    /// (override with the OGRE_PLUGIN_DIR environment variable).
    bool loadRenderSystem();
    /// Registers the OGRE RTShaderLib media (so the RTSS can find its shader
    /// includes) plus any locations listed in assets/resources.cfg.
    void loadResources();
    bool initShaderSystem();

    std::unique_ptr<Ogre::Root> root_;
    Ogre::RenderWindow* hiddenWindow_{nullptr};         // owns the GL context
    Ogre::SceneManager* sceneManager_{nullptr};         // owned by root_
    Ogre::RTShader::ShaderGenerator* shaderGen_{nullptr};

    // RTSS scheme-not-found resolver (generates shaders for default materials).
    class MaterialResolver;
    std::unique_ptr<MaterialResolver> materialResolver_;
};

} // namespace avb
