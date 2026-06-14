#include "render/OgreContext.h"

#include <cstdlib>
#include <filesystem>
#include <system_error>
#include <vector>

#include <OgreConfigFile.h>
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

namespace {

namespace fs = std::filesystem;

// Locate the OGRE "RTShaderLib" media directory that ships the RTSS shader
// includes (OgreUnifiedShader.h and friends). The shader generator cannot
// compile the GPU programs it builds for our materials without these, so the
// directory must be registered as a resource location up front. Returns the
// path to the RTShaderLib dir, or an empty string if none was found.
std::string findRtShaderLib() {
    std::vector<fs::path> candidates;
    const auto addMedia = [&](const fs::path& media) {
        candidates.emplace_back(media / "RTShaderLib");
    };

    if (const char* env = std::getenv("OGRE_MEDIA_DIR")) {
        addMedia(env);
    }
#ifdef AVB_OGRE_MEDIA_DIR
    addMedia(AVB_OGRE_MEDIA_DIR);
#endif
    addMedia("/usr/share/OGRE/Media");
    addMedia("/usr/local/share/OGRE/Media");

    // Distros install versioned media dirs, e.g. /usr/share/OGRE-13.6/Media.
    for (const char* root : {"/usr/share", "/usr/local/share"}) {
        std::error_code ec;
        if (!fs::is_directory(root, ec)) {
            continue;
        }
        for (const auto& entry : fs::directory_iterator(root, ec)) {
            if (entry.is_directory(ec) &&
                entry.path().filename().string().rfind("OGRE", 0) == 0) {
                addMedia(entry.path() / "Media");
            }
        }
    }

    std::string fallback;
    for (const fs::path& dir : candidates) {
        std::error_code ec;
        // Prefer a directory that demonstrably contains the RTSS includes.
        if (fs::exists(dir / "GLSL" / "OgreUnifiedShader.h", ec)) {
            return dir.string();
        }
        if (fallback.empty() && fs::is_directory(dir, ec)) {
            fallback = dir.string();
        }
    }
    return fallback;
}

} // namespace

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

void OgreContext::loadResources() {
    auto& rgm = Ogre::ResourceGroupManager::getSingleton();

    // RTSS shader library: required for the shader generator to compile the
    // techniques it builds for our materials (e.g. avb/DefaultLit). Registered
    // recursively into the internal group OGRE itself looks these includes up
    // in, so the layout works across OGRE versions.
    const std::string rtShaderLib = findRtShaderLib();
    if (!rtShaderLib.empty()) {
        rgm.addResourceLocation(
            rtShaderLib, "FileSystem",
            Ogre::ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME,
            /*recursive*/ true);
    } else {
        Ogre::LogManager::getSingleton().logMessage(
            "OgreContext: could not locate the OGRE RTShaderLib media; set the "
            "OGRE_MEDIA_DIR environment variable. RTSS shader generation will "
            "fail (missing OgreUnifiedShader.h).",
            Ogre::LML_CRITICAL);
    }

    // Optional project resources (materials/textures/meshes) listed in
    // assets/resources.cfg, resolved relative to the working directory.
    constexpr const char* kResourcesCfg = "assets/resources.cfg";
    std::error_code ec;
    if (fs::exists(kResourcesCfg, ec)) {
        Ogre::ConfigFile cfg;
        cfg.load(kResourcesCfg);
        for (const auto& [section, settings] : cfg.getSettingsBySection()) {
            const Ogre::String& group =
                section.empty()
                    ? Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME
                    : section;
            for (const auto& [type, location] : settings) {
                rgm.addResourceLocation(location, type, group);
            }
        }
    }
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

    // Register the RTSS shader library (and any project resources) before the
    // groups are initialised so the shader generator can find its includes.
    loadResources();
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    if (!initShaderSystem()) {
        Ogre::LogManager::getSingleton().logMessage(
            "OgreContext: RTSS init failed; materials may not render under GL3+");
    }
    return true;
}

} // namespace avb
