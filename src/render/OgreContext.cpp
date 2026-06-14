#include "render/OgreContext.h"

#include <cstdlib>
#include <filesystem>
#include <set>
#include <string>
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

// Candidate OGRE media roots to search for the RTSS shader library, in
// priority order: an explicit OGRE_MEDIA_DIR, the directory recorded at build
// time, then the usual system install locations (including versioned dirs such
// as /usr/share/OGRE-13.6/Media).
std::vector<fs::path> mediaRoots() {
    std::vector<fs::path> roots;
    if (const char* env = std::getenv("OGRE_MEDIA_DIR")) {
        roots.emplace_back(env);
    }
#ifdef AVB_OGRE_MEDIA_DIR
    roots.emplace_back(AVB_OGRE_MEDIA_DIR);
#endif
    roots.emplace_back("/usr/share/OGRE/Media");
    roots.emplace_back("/usr/local/share/OGRE/Media");

    for (const char* base : {"/usr/share", "/usr/local/share"}) {
        std::error_code ec;
        if (!fs::is_directory(base, ec)) {
            continue;
        }
        for (const auto& entry : fs::directory_iterator(base, ec)) {
            if (entry.is_directory(ec) &&
                entry.path().filename().string().rfind("OGRE", 0) == 0) {
                roots.emplace_back(entry.path() / "Media");
                roots.emplace_back(entry.path());
            }
        }
    }
    return roots;
}

// Returns true for the RTSS shader library files the generator pulls in by bare
// name: the shared header OgreUnifiedShader.h and the SGXLib_*/RTSLib_*/FFPLib_*
// sources (.glsl/.glsles/.cg/.hlsl). Matching on these names rather than on a
// bare extension keeps us from registering unrelated sample-shader directories.
bool isRtShaderLibFile(const std::string& name) {
    static const char* kPrefixes[] = {"OgreUnifiedShader", "SGXLib", "RTSLib",
                                      "FFPLib"};
    for (const char* prefix : kPrefixes) {
        if (name.rfind(prefix, 0) == 0) {
            return true;
        }
    }
    return false;
}

// Find the directories that must be registered so the RTSS can resolve the
// shader includes it pulls in by bare name. Each media root is walked
// recursively and we collect every directory that holds an RTSS shader library
// file (see isRtShaderLibFile). The layout varies across OGRE versions -- the
// shared header may sit next to the SGXLib sources or in a sibling directory
// (e.g. OGRE 14's Media/Main/OgreUnifiedShader.h alongside Media/RTShaderLib),
// and on older releases the sources live under a GLSL/ subdir -- so a recursive
// search is the only layout-agnostic way to locate them. We register these leaf
// directories directly (non-recursively) so the RTSS resolves the includes by
// bare name. Returns the directories found under the first media root that
// actually ships OgreUnifiedShader.h.
std::vector<std::string> findShaderIncludeDirs() {
    std::set<fs::path> best;
    for (const fs::path& root : mediaRoots()) {
        std::error_code ec;
        if (!fs::is_directory(root, ec)) {
            continue;
        }
        std::set<fs::path> dirs;
        bool haveHeader = false;
        for (auto it = fs::recursive_directory_iterator(
                 root, fs::directory_options::skip_permission_denied, ec);
             !ec && it != fs::recursive_directory_iterator();
             it.increment(ec)) {
            std::error_code fec;
            if (!it->is_regular_file(fec)) {
                continue;
            }
            const std::string name = it->path().filename().string();
            if (!isRtShaderLibFile(name)) {
                continue;
            }
            dirs.insert(it->path().parent_path());
            if (name == "OgreUnifiedShader.h") {
                haveHeader = true;
            }
        }
        if (haveHeader) {
            // First root that actually ships the header wins.
            best = std::move(dirs);
            break;
        }
        if (best.empty()) {
            best = std::move(dirs); // remember partial matches as a fallback
        }
    }

    std::vector<std::string> out;
    out.reserve(best.size());
    for (const fs::path& dir : best) {
        out.push_back(dir.string());
    }
    return out;
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
    // techniques it builds for our materials (e.g. avb/DefaultLit). Register the
    // leaf directories that hold the shader includes (see findShaderIncludeDirs)
    // non-recursively so the RTSS resolves them by bare name, without dragging
    // in OGRE's shipped sample materials.
    const std::vector<std::string> shaderDirs = findShaderIncludeDirs();
    if (!shaderDirs.empty()) {
        for (const std::string& dir : shaderDirs) {
            rgm.addResourceLocation(
                dir, "FileSystem",
                Ogre::ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME,
                /*recursive*/ false);
        }
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

    // Register the RTSS shader library (and any project resources) first so the
    // shader generator can find its includes. Then initialise the RTSS *before*
    // parsing resource scripts: the shader library ships sample materials that
    // use the rtshader_system keyword, which only becomes a recognised script
    // token once ShaderGenerator::initialize() has registered its translator.
    // Parsing those scripts beforehand floods the log with "unexpected token
    // rtshader_system" errors.
    loadResources();
    if (!initShaderSystem()) {
        Ogre::LogManager::getSingleton().logMessage(
            "OgreContext: RTSS init failed; materials may not render under GL3+");
    }
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    // Surface a clear diagnostic if the RTSS shader includes are still not
    // resolvable: shader generation for any material will otherwise abort with
    // a FileNotFoundException for OgreUnifiedShader.h the moment a model renders.
    if (!Ogre::ResourceGroupManager::getSingleton().resourceExists(
            Ogre::ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME,
            "OgreUnifiedShader.h")) {
        Ogre::LogManager::getSingleton().logMessage(
            "OgreContext: OgreUnifiedShader.h is not resolvable in the internal "
            "resource group; the RTShaderLib media was not found or is "
            "incomplete. Set OGRE_MEDIA_DIR to the OGRE Media directory that "
            "contains RTShaderLib. RTSS shader generation will fail.",
            Ogre::LML_CRITICAL);
    }
    return true;
}

} // namespace avb
