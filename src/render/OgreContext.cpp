#include "render/OgreContext.h"

#include <cstdlib>
#include <vector>

#include <OgreLogManager.h>
#include <OgreMaterialManager.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreResourceGroupManager.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>

#include <RTShaderSystem/OgreShaderGenerator.h>

namespace avb {

// RTSS resolver: when a material is requested under the RTSS scheme but has no
// such technique, generate one on the fly so default materials render under the
// GL3+ (shader-only) render system.
class OgreContext::MaterialResolver : public Ogre::MaterialManager::Listener {
public:
    explicit MaterialResolver(Ogre::RTShader::ShaderGenerator* gen) : gen_(gen) {}

    Ogre::Technique* handleSchemeNotFound(unsigned short /*schemeIndex*/,
                                          const Ogre::String& schemeName,
                                          Ogre::Material* originalMaterial,
                                          unsigned short /*lodIndex*/,
                                          const Ogre::Renderable* /*rend*/) override {
        if (schemeName != Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME) {
            return nullptr;
        }
        if (!gen_->createShaderBasedTechnique(
                *originalMaterial, Ogre::MaterialManager::DEFAULT_SCHEME_NAME,
                schemeName)) {
            return nullptr;
        }
        gen_->validateMaterial(schemeName, originalMaterial->getName(),
                               originalMaterial->getGroup());
        for (Ogre::Technique* tech : originalMaterial->getTechniques()) {
            if (tech->getSchemeName() == schemeName) {
                return tech;
            }
        }
        return nullptr;
    }

private:
    Ogre::RTShader::ShaderGenerator* gen_;
};

OgreContext::OgreContext() = default;

OgreContext::~OgreContext() {
    if (materialResolver_) {
        Ogre::MaterialManager::getSingleton().removeListener(materialResolver_.get());
    }
    if (shaderGen_) {
        Ogre::RTShader::ShaderGenerator::destroy();
        shaderGen_ = nullptr;
    }
    // root_ destruction tears down the scene manager, hidden window and plugins.
}

bool OgreContext::loadRenderSystem() {
    std::vector<std::string> dirs;
    if (const char* env = std::getenv("OGRE_PLUGIN_DIR")) {
        dirs.emplace_back(env);
    }
    dirs.emplace_back("/usr/lib/x86_64-linux-gnu/OGRE");
    dirs.emplace_back("/usr/lib/OGRE");
    dirs.emplace_back("/usr/local/lib/OGRE");

    // Prefer GL3Plus, fall back to legacy GL.
    for (const char* plugin : {"RenderSystem_GL3Plus", "RenderSystem_GL"}) {
        for (const std::string& dir : dirs) {
            try {
                root_->loadPlugin(dir + "/" + plugin);
            } catch (const Ogre::Exception&) {
                continue; // not in this dir
            }
            if (!root_->getAvailableRenderers().empty()) {
                root_->setRenderSystem(root_->getAvailableRenderers().front());
                return true;
            }
        }
    }
    return false;
}

bool OgreContext::initShaderSystem() {
    if (!Ogre::RTShader::ShaderGenerator::initialize()) {
        return false;
    }
    shaderGen_ = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
    shaderGen_->addSceneManager(sceneManager_);
    materialResolver_ = std::make_unique<MaterialResolver>(shaderGen_);
    Ogre::MaterialManager::getSingleton().addListener(materialResolver_.get());
    return true;
}

bool OgreContext::initialize() {
    root_ = std::make_unique<Ogre::Root>(/*plugins*/ "", /*config*/ "",
                                         /*log*/ "Ogre.log");
    if (!loadRenderSystem()) {
        Ogre::LogManager::getSingleton().logMessage(
            "OgreContext: no GL render system plugin found");
        return false;
    }

    // Initialise without an automatic window, then create a hidden one so a GL
    // context exists for textures/meshes/RTTs.
    root_->initialise(false);
    Ogre::NameValuePairList params;
    params["hidden"] = "true";
    hiddenWindow_ = root_->createRenderWindow("avb_context", 1, 1, false, &params);
    if (hiddenWindow_) {
        hiddenWindow_->setHidden(true);
    }

    sceneManager_ = root_->createSceneManager();

    // A generic resource group is enough for procedurally built meshes and the
    // default material; no on-disk resources are required for the core pipeline.
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    if (!initShaderSystem()) {
        Ogre::LogManager::getSingleton().logMessage(
            "OgreContext: RTSS init failed; materials may not render under GL3+");
    }
    return true;
}

} // namespace avb
