#include "render/OgreContext.h"

// #include <OgreRoot.h>
// #include <OgreSceneManager.h>
// #include <OgreConfigFile.h>

namespace avb {

OgreContext::OgreContext() = default;
OgreContext::~OgreContext() = default;

bool OgreContext::initialize(const std::string& /*pluginsCfg*/,
                             const std::string& /*resourcesCfg*/) {
    // TODO:
    //   root_ = std::make_unique<Ogre::Root>(pluginsCfg);
    //   parse resourcesCfg, addResourceLocation(...) per section
    //   select/configure RenderSystem (GL3+), root_->initialise(false)
    //   sceneManager_ = root_->createSceneManager();
    //   ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    return true;
}

} // namespace avb
