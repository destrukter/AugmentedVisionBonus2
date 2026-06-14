#pragma once

#include <memory>
#include <string>

namespace Ogre {
class Root;
class SceneManager;
class RenderWindow;
} // namespace Ogre

namespace avb {

/// Owns the OGRE3D Root and shared rendering resources.
///
/// A single OgreContext is created by the Application and shared with the
/// rendering pieces used by the Camera window. It loads plugins/resources from
/// the config files under assets/.
class OgreContext {
public:
    OgreContext();
    ~OgreContext();

    OgreContext(const OgreContext&) = delete;
    OgreContext& operator=(const OgreContext&) = delete;

    /// Initializes OGRE Root, RenderSystem and resource groups. Returns false on
    /// failure. `pluginsCfg` / `resourcesCfg` default to the files in assets/.
    bool initialize(const std::string& pluginsCfg = "assets/plugins.cfg",
                    const std::string& resourcesCfg = "assets/resources.cfg");

    Ogre::Root* root() const { return root_.get(); }
    Ogre::SceneManager* sceneManager() const { return sceneManager_; }

private:
    std::unique_ptr<Ogre::Root> root_;
    Ogre::SceneManager* sceneManager_{nullptr};  // owned by root_
};

} // namespace avb
