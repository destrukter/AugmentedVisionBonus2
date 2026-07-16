#include "render/ModelLoader.h"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <OgreAnimation.h>
#include <OgreBone.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreKeyFrame.h>
#include <OgreLogManager.h>
#include <OgreManualObject.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreMesh.h>
#include <OgrePass.h>
#include <OgreResourceGroupManager.h>
#include <OgreSkeleton.h>
#include <OgreSkeletonManager.h>
#include <OgreSubMesh.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>

namespace avb {

namespace {

namespace fs = std::filesystem;

constexpr const char* kDefaultMaterial = "avb/DefaultLit";
// Read lazily inside functions to avoid static-init-order issues with OGRE.
const Ogre::String& defaultGroup() {
    return Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
}

/// Creates an OGRE texture named `name` from OpenCV pixels (any of gray /
/// BGR / BGRA, 8- or 16-bit). Decoding through OpenCV means no OGRE image
/// codec plugin is required. Returns false when the pixels are unusable.
bool createTextureFromMat(const std::string& name, cv::Mat img) {
    auto& tm = Ogre::TextureManager::getSingleton();
    if (tm.getByName(name, defaultGroup())) {
        return true;
    }
    if (img.empty()) {
        return false;
    }
    if (img.depth() != CV_8U) {
        const double scale = img.depth() == CV_16U ? 255.0 / 65535.0 : 255.0;
        img.convertTo(img, CV_8U, scale);
    }
    cv::Mat rgba;
    switch (img.channels()) {
        case 1: cv::cvtColor(img, rgba, cv::COLOR_GRAY2RGBA); break;
        case 3: cv::cvtColor(img, rgba, cv::COLOR_BGR2RGBA); break;
        case 4: cv::cvtColor(img, rgba, cv::COLOR_BGRA2RGBA); break;
        default: return false;
    }
    Ogre::TexturePtr tex = tm.createManual(
        name, defaultGroup(), Ogre::TEX_TYPE_2D,
        static_cast<unsigned int>(rgba.cols), static_cast<unsigned int>(rgba.rows),
        0, Ogre::PF_BYTE_RGBA, Ogre::TU_DEFAULT);
    if (!tex) {
        return false;
    }
    const Ogre::PixelBox src(static_cast<unsigned int>(rgba.cols),
                             static_cast<unsigned int>(rgba.rows), 1,
                             Ogre::PF_BYTE_RGBA, rgba.data);
    tex->getBuffer()->blitFromMemory(src);
    return true;
}

/// Loads the diffuse texture referenced by `material` - either embedded in
/// the file (common for FBX) or an external image resolved against the
/// model's folder - into an OGRE texture named `textureName`. Returns false
/// when the material has no usable diffuse texture.
bool loadDiffuseTexture(const aiScene* scene, const aiMaterial* material,
                        const std::string& modelPath,
                        const std::string& textureName) {
    aiString ref;
    if (material->GetTexture(aiTextureType_DIFFUSE, 0, &ref) != AI_SUCCESS ||
        ref.length == 0) {
        return false;
    }

    if (const aiTexture* embedded = scene->GetEmbeddedTexture(ref.C_Str())) {
        cv::Mat img;
        if (embedded->mHeight == 0) {
            // Compressed blob (png/jpg/...) stored inside the model file.
            const cv::Mat blob(1, static_cast<int>(embedded->mWidth), CV_8UC1,
                               const_cast<aiTexel*>(embedded->pcData));
            img = cv::imdecode(blob, cv::IMREAD_UNCHANGED);
        } else {
            // Raw texels; aiTexel's byte order is b,g,r,a.
            img = cv::Mat(static_cast<int>(embedded->mHeight),
                          static_cast<int>(embedded->mWidth), CV_8UC4,
                          const_cast<aiTexel*>(embedded->pcData))
                      .clone();
        }
        if (img.empty()) {
            Ogre::LogManager::getSingleton().logMessage(
                "ModelLoader: could not decode embedded texture '" +
                std::string(ref.C_Str()) + "' of " + modelPath);
            return false;
        }
        return createTextureFromMat(textureName, img);
    }

    // External file. Exporters frequently store absolute paths from the
    // authoring machine, so fall back to the bare file name next to the model.
    const fs::path modelDir = fs::path(modelPath).parent_path();
    const fs::path given(ref.C_Str());
    std::error_code ec;
    fs::path resolved;
    if (given.is_absolute() && fs::exists(given, ec)) {
        resolved = given;
    } else if (fs::exists(modelDir / given, ec)) {
        resolved = modelDir / given;
    } else if (fs::exists(modelDir / given.filename(), ec)) {
        resolved = modelDir / given.filename();
    }
    if (resolved.empty()) {
        Ogre::LogManager::getSingleton().logMessage(
            "ModelLoader: texture '" + std::string(ref.C_Str()) + "' of " +
            modelPath + " not found (also tried next to the model file)");
        return false;
    }
    cv::Mat img = cv::imread(resolved.string(), cv::IMREAD_UNCHANGED);
    if (img.empty()) {
        Ogre::LogManager::getSingleton().logMessage(
            "ModelLoader: could not decode texture file " + resolved.string());
        return false;
    }
    return createTextureFromMat(textureName, img);
}

/// Builds an OGRE material for `materialIndex` of the imported scene,
/// carrying over the source material's colors, shininess, sidedness and
/// diffuse texture. Named uniquely per (mesh, material index) and cached in
/// the MaterialManager.
std::string buildMaterial(const aiScene* scene, unsigned int materialIndex,
                          const std::string& meshName,
                          const std::string& modelPath, bool hasVertexColours) {
    const std::string name =
        meshName + "/mat/" + std::to_string(materialIndex);
    auto& mm = Ogre::MaterialManager::getSingleton();
    if (mm.getByName(name, defaultGroup())) {
        return name;
    }
    const aiMaterial* src = scene->mMaterials[materialIndex];

    Ogre::MaterialPtr mat = mm.create(name, defaultGroup());
    Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(true);

    aiColor3D diffuse(0.8f, 0.8f, 0.82f);
    src->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse);
    pass->setDiffuse(diffuse.r, diffuse.g, diffuse.b, 1.0f);
    // Tie the ambient term to the diffuse color so the scene's ambient light
    // tints the object its own color instead of washing it gray.
    pass->setAmbient(diffuse.r, diffuse.g, diffuse.b);

    aiColor3D specular(0.2f, 0.2f, 0.2f);
    src->Get(AI_MATKEY_COLOR_SPECULAR, specular);
    float shininess = 20.0f;
    src->Get(AI_MATKEY_SHININESS, shininess);
    pass->setSpecular(specular.r, specular.g, specular.b, 1.0f);
    pass->setShininess(std::clamp(shininess, 1.0f, 128.0f));

    aiColor3D emissive(0.0f, 0.0f, 0.0f);
    if (src->Get(AI_MATKEY_COLOR_EMISSIVE, emissive) == AI_SUCCESS) {
        pass->setSelfIllumination(emissive.r, emissive.g, emissive.b);
    }

    int twoSided = 0;
    if (src->Get(AI_MATKEY_TWOSIDED, twoSided) == AI_SUCCESS && twoSided) {
        pass->setCullingMode(Ogre::CULL_NONE);
    }

    if (hasVertexColours) {
        pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    }

    if (loadDiffuseTexture(scene, src, modelPath, name + "/diffuse")) {
        pass->createTextureUnitState(name + "/diffuse");
    }
    // The RTSS (set up by OgreContext) generates the actual shaders.
    return name;
}

// ---------------------------------------------------------------------------
// Animation import: the imported node hierarchy is mirrored into an OGRE
// skeleton (one bone per node) and the file's animations become skeletal
// animations on it. Meshes with real skinning weights keep them; meshes
// without are bound rigidly (weight 1) to their node's bone, so plain
// node-transform animations play too.
// ---------------------------------------------------------------------------

struct SkeletonBuild {
    Ogre::SkeletonPtr skeleton;
    std::unordered_map<const aiNode*, Ogre::Bone*> byNode;
    // aiBone / animation channels reference nodes by name. First one wins on
    // (rare) duplicate names.
    std::unordered_map<std::string, Ogre::Bone*> byName;
};

/// Creates one bone per node, mirroring the hierarchy and each node's local
/// bind transform. Returns false when the file has more nodes than a v1
/// skeleton can address (the caller then falls back to static geometry).
bool createBonesRecursive(SkeletonBuild& build, const aiNode* node,
                          Ogre::Bone* parent) {
    if (build.skeleton->getNumBones() >= OGRE_MAX_NUM_BONES) {
        return false;
    }
    std::string name = node->mName.C_Str();
    if (name.empty() || build.skeleton->hasBone(name)) {
        name += "#" + std::to_string(build.skeleton->getNumBones());
    }
    Ogre::Bone* bone = build.skeleton->createBone(name);
    if (parent) {
        parent->addChild(bone);
    }
    aiVector3D scale;
    aiQuaternion rot;
    aiVector3D pos;
    node->mTransformation.Decompose(scale, rot, pos);
    bone->setPosition(pos.x, pos.y, pos.z);
    bone->setOrientation(Ogre::Quaternion(rot.w, rot.x, rot.y, rot.z));
    bone->setScale(scale.x, scale.y, scale.z);
    build.byNode.emplace(node, bone);
    build.byName.emplace(node->mName.C_Str(), bone);
    for (unsigned int i = 0; i < node->mNumChildren; ++i) {
        if (!createBonesRecursive(build, node->mChildren[i], bone)) {
            return false;
        }
    }
    return true;
}

/// Samples a key array at `ticks` with linear interpolation (assimp's keys are
/// sorted by time). `fallback` is the node's bind value, used when the channel
/// has no keys for this component.
aiVector3D sampleVectorKeys(const aiVectorKey* keys, unsigned int count,
                            double ticks, const aiVector3D& fallback) {
    if (count == 0) {
        return fallback;
    }
    if (ticks <= keys[0].mTime || count == 1) {
        return keys[0].mValue;
    }
    for (unsigned int i = 1; i < count; ++i) {
        if (ticks < keys[i].mTime) {
            const double span = keys[i].mTime - keys[i - 1].mTime;
            const float f =
                span > 0.0
                    ? static_cast<float>((ticks - keys[i - 1].mTime) / span)
                    : 0.0f;
            return keys[i - 1].mValue +
                   (keys[i].mValue - keys[i - 1].mValue) * f;
        }
    }
    return keys[count - 1].mValue;
}

aiQuaternion sampleQuaternionKeys(const aiQuatKey* keys, unsigned int count,
                                  double ticks, const aiQuaternion& fallback) {
    if (count == 0) {
        return fallback;
    }
    if (ticks <= keys[0].mTime || count == 1) {
        return keys[0].mValue;
    }
    for (unsigned int i = 1; i < count; ++i) {
        if (ticks < keys[i].mTime) {
            const double span = keys[i].mTime - keys[i - 1].mTime;
            const float f =
                span > 0.0
                    ? static_cast<float>((ticks - keys[i - 1].mTime) / span)
                    : 0.0f;
            aiQuaternion out;
            aiQuaternion::Interpolate(out, keys[i - 1].mValue, keys[i].mValue, f);
            return out;
        }
    }
    return keys[count - 1].mValue;
}

/// Converts every aiAnimation into an OGRE skeletal animation. Assimp keys
/// replace a node's local transform outright, while OGRE keyframes are offsets
/// from the binding pose - each sampled key is rebased accordingly.
void buildAnimations(const aiScene* scene, SkeletonBuild& build) {
    for (unsigned int a = 0; a < scene->mNumAnimations; ++a) {
        const aiAnimation* src = scene->mAnimations[a];
        const double tps =
            src->mTicksPerSecond > 0.0 ? src->mTicksPerSecond : 25.0;
        std::string name = src->mName.C_Str();
        if (name.empty() || build.skeleton->hasAnimation(name)) {
            name = "animation_" + std::to_string(a);
        }
        Ogre::Animation* anim = build.skeleton->createAnimation(
            name, static_cast<Ogre::Real>(src->mDuration / tps));

        for (unsigned int c = 0; c < src->mNumChannels; ++c) {
            const aiNodeAnim* channel = src->mChannels[c];
            const auto it = build.byName.find(channel->mNodeName.C_Str());
            if (it == build.byName.end()) {
                continue; // channel for a node that didn't become a bone
            }
            Ogre::Bone* bone = it->second;
            const Ogre::Vector3 bindP = bone->getPosition();
            const Ogre::Quaternion bindQ = bone->getOrientation();
            const Ogre::Vector3 bindS = bone->getScale();
            const aiVector3D bindPos(bindP.x, bindP.y, bindP.z);
            const aiQuaternion bindRot(bindQ.w, bindQ.x, bindQ.y, bindQ.z);
            const aiVector3D bindScale(bindS.x, bindS.y, bindS.z);

            // One keyframe per distinct key time across the three components;
            // the other two components are interpolated at that time.
            std::set<double> ticks;
            for (unsigned int k = 0; k < channel->mNumPositionKeys; ++k) {
                ticks.insert(channel->mPositionKeys[k].mTime);
            }
            for (unsigned int k = 0; k < channel->mNumRotationKeys; ++k) {
                ticks.insert(channel->mRotationKeys[k].mTime);
            }
            for (unsigned int k = 0; k < channel->mNumScalingKeys; ++k) {
                ticks.insert(channel->mScalingKeys[k].mTime);
            }
            if (ticks.empty()) {
                continue;
            }

            Ogre::NodeAnimationTrack* track =
                anim->createNodeTrack(bone->getHandle(), bone);
            for (const double t : ticks) {
                const aiVector3D p = sampleVectorKeys(
                    channel->mPositionKeys, channel->mNumPositionKeys, t, bindPos);
                const aiQuaternion r =
                    sampleQuaternionKeys(channel->mRotationKeys,
                                         channel->mNumRotationKeys, t, bindRot);
                const aiVector3D s = sampleVectorKeys(
                    channel->mScalingKeys, channel->mNumScalingKeys, t, bindScale);

                Ogre::TransformKeyFrame* kf =
                    track->createNodeKeyFrame(static_cast<Ogre::Real>(t / tps));
                kf->setTranslate(Ogre::Vector3(p.x, p.y, p.z) - bindP);
                kf->setRotation(bindQ.Inverse() *
                                Ogre::Quaternion(r.w, r.x, r.y, r.z));
                kf->setScale(Ogre::Vector3(
                    bindS.x != 0.0f ? s.x / bindS.x : 1.0f,
                    bindS.y != 0.0f ? s.y / bindS.y : 1.0f,
                    bindS.z != 0.0f ? s.z / bindS.z : 1.0f));
            }
        }
    }
}

} // namespace

bool ModelLoader::validateModelFile(const std::string& filePath,
                                    std::string* error) {
    Assimp::Importer importer;
    // Triangulate mirrors loadFbx() closely enough to predict whether it will
    // succeed, while skipping the heavier post-processing steps.
    const aiScene* scene =
        importer.ReadFile(filePath, aiProcess_Triangulate);
    if (!scene || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) ||
        !scene->mRootNode) {
        if (error) {
            const char* reason = importer.GetErrorString();
            *error = (reason && reason[0]) ? reason
                                           : "file could not be parsed as a model";
        }
        return false;
    }
    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
        if (scene->mMeshes[m]->mNumVertices > 0) {
            return true;
        }
    }
    if (error) {
        *error = "model contains no mesh geometry";
    }
    return false;
}

std::string ModelLoader::ensureDefaultMaterial() {
    if (defaultMaterialCreated_) {
        return kDefaultMaterial;
    }
    auto& mm = Ogre::MaterialManager::getSingleton();
    if (!mm.getByName(kDefaultMaterial, defaultGroup())) {
        Ogre::MaterialPtr mat = mm.create(kDefaultMaterial, defaultGroup());
        Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
        pass->setLightingEnabled(true);
        pass->setDiffuse(0.8f, 0.8f, 0.82f, 1.0f);
        pass->setSpecular(0.2f, 0.2f, 0.2f, 1.0f);
        pass->setShininess(20.0f);
        // RTSS (set up by OgreContext) generates the actual shaders.
    }
    defaultMaterialCreated_ = true;
    return kDefaultMaterial;
}

namespace {

/// Emits one ManualObject section per (node, mesh) reference, depth-first.
/// Each node's global (bind-pose) transform is baked into its vertices - the
/// job aiProcess_PreTransformVertices used to do, done by hand here because
/// that step would also strip the animations. Per section, the matching bone
/// assignments are collected into `sectionWeights` (empty when unskinned).
void emitNodeRecursive(
    const aiScene* scene, const aiNode* node, const aiMatrix4x4& parentGlobal,
    Ogre::ManualObject& manual, const std::string& meshName,
    const std::string& modelPath, const std::string& fallbackMaterial,
    const SkeletonBuild& skel,
    std::vector<std::vector<Ogre::VertexBoneAssignment>>& sectionWeights) {
    const aiMatrix4x4 global = parentGlobal * node->mTransformation;
    aiMatrix3x3 normalMat(global);
    normalMat.Inverse().Transpose(); // survives non-uniform scale

    for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
        const aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        if (mesh->mNumVertices == 0) {
            continue;
        }
        const bool hasColours = mesh->HasVertexColors(0);
        // One section per referenced aiMesh, carrying its own material from
        // the file (colors + diffuse texture); fall back to the shared
        // default when the file has no material for it.
        std::string material;
        if (mesh->mMaterialIndex < scene->mNumMaterials) {
            material = buildMaterial(scene, mesh->mMaterialIndex, meshName,
                                     modelPath, hasColours);
        }
        if (material.empty()) {
            material = fallbackMaterial;
        }
        manual.begin(material, Ogre::RenderOperation::OT_TRIANGLE_LIST,
                     defaultGroup());

        const bool hasUv = mesh->HasTextureCoords(0);
        for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
            const aiVector3D p = global * mesh->mVertices[v];
            manual.position(p.x, p.y, p.z);
            if (mesh->HasNormals()) {
                aiVector3D n = normalMat * mesh->mNormals[v];
                n.NormalizeSafe();
                manual.normal(n.x, n.y, n.z);
            }
            if (hasUv) {
                const aiVector3D& t = mesh->mTextureCoords[0][v];
                manual.textureCoord(t.x, t.y);
            }
            if (hasColours) {
                const aiColor4D& c = mesh->mColors[0][v];
                manual.colour(c.r, c.g, c.b, c.a);
            }
        }
        // Indices are local to this section (one section per aiMesh).
        for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            if (face.mNumIndices != 3) {
                continue; // triangulated above, skip any stray primitives
            }
            manual.triangle(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
        }
        manual.end();

        // Bone weights for this section: the mesh's own skinning weights when
        // present, else the whole mesh rides rigidly on its node's bone.
        std::vector<Ogre::VertexBoneAssignment> weights;
        if (skel.skeleton) {
            if (mesh->HasBones()) {
                for (unsigned int b = 0; b < mesh->mNumBones; ++b) {
                    const aiBone* bone = mesh->mBones[b];
                    const auto it = skel.byName.find(bone->mName.C_Str());
                    if (it == skel.byName.end()) {
                        continue;
                    }
                    const unsigned short handle = it->second->getHandle();
                    for (unsigned int w = 0; w < bone->mNumWeights; ++w) {
                        Ogre::VertexBoneAssignment vba;
                        vba.vertexIndex = bone->mWeights[w].mVertexId;
                        vba.boneIndex = handle;
                        vba.weight = bone->mWeights[w].mWeight;
                        weights.push_back(vba);
                    }
                }
            } else if (const auto it = skel.byNode.find(node);
                       it != skel.byNode.end()) {
                const unsigned short handle = it->second->getHandle();
                weights.reserve(mesh->mNumVertices);
                for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
                    Ogre::VertexBoneAssignment vba;
                    vba.vertexIndex = v;
                    vba.boneIndex = handle;
                    vba.weight = 1.0f;
                    weights.push_back(vba);
                }
            }
        }
        sectionWeights.push_back(std::move(weights));
    }

    for (unsigned int c = 0; c < node->mNumChildren; ++c) {
        emitNodeRecursive(scene, node->mChildren[c], global, manual, meshName,
                          modelPath, fallbackMaterial, skel, sectionWeights);
    }
}

} // namespace

std::string ModelLoader::loadFbx(const std::string& filePath,
                                 const std::string& meshName) {
    if (const auto it = cache_.find(filePath); it != cache_.end()) {
        return it->second;
    }

    Assimp::Importer importer;
    // FBX pivot preservation inserts chains of helper nodes per pivot, which
    // would multiply the skeleton's bone count for nothing this renderer uses.
    importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
    const aiScene* scene = importer.ReadFile(
        filePath, aiProcess_Triangulate | aiProcess_GenSmoothNormals |
                      aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices |
                      aiProcess_TransformUVCoords | aiProcess_LimitBoneWeights);
    if (!scene || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) ||
        !scene->mRootNode || scene->mNumMeshes == 0) {
        return "";
    }

    // Mirror the node hierarchy into a skeleton when the file animates.
    SkeletonBuild skel;
    if (scene->mNumAnimations > 0) {
        const std::string skelName = meshName + "/skeleton";
        auto& skelMgr = Ogre::SkeletonManager::getSingleton();
        if (auto existing = skelMgr.getByName(skelName, defaultGroup())) {
            skelMgr.remove(existing);
        }
        skel.skeleton = skelMgr.create(skelName, defaultGroup(), /*isManual=*/true);
        if (createBonesRecursive(skel, scene->mRootNode, nullptr)) {
            skel.skeleton->setBindingPose();
            buildAnimations(scene, skel);
        } else {
            // More nodes than a skeleton can address: render statically.
            skelMgr.remove(skel.skeleton);
            skel = SkeletonBuild{};
        }
    }

    Ogre::ManualObject manual(meshName + "/import");
    manual.setDynamic(false);
    std::vector<std::vector<Ogre::VertexBoneAssignment>> sectionWeights;
    emitNodeRecursive(scene, scene->mRootNode, aiMatrix4x4(), manual, meshName,
                      filePath, ensureDefaultMaterial(), skel, sectionWeights);
    if (manual.getNumSections() == 0) {
        if (skel.skeleton) {
            Ogre::SkeletonManager::getSingleton().remove(skel.skeleton);
        }
        return ""; // no non-empty mesh anywhere in the file
    }

    Ogre::MeshPtr ogreMesh = manual.convertToMesh(meshName, defaultGroup());
    if (!ogreMesh) {
        if (skel.skeleton) {
            Ogre::SkeletonManager::getSingleton().remove(skel.skeleton);
        }
        return "";
    }

    if (skel.skeleton && skel.skeleton->getNumAnimations() > 0) {
        ogreMesh->setSkeletonName(skel.skeleton->getName());
        for (std::size_t s = 0; s < sectionWeights.size(); ++s) {
            Ogre::SubMesh* sub = ogreMesh->getSubMesh(s);
            for (const Ogre::VertexBoneAssignment& vba : sectionWeights[s]) {
                sub->addBoneAssignment(vba);
            }
        }
        ogreMesh->_updateCompiledBoneAssignments();
        // The converted bounds cover the bind pose only; grow them so an
        // animation swinging geometry outward doesn't get frustum-culled.
        Ogre::AxisAlignedBox box = ogreMesh->getBounds();
        if (!box.isNull() && !box.isInfinite()) {
            const Ogre::Vector3 grow = box.getHalfSize() * 0.5f;
            box.setExtents(box.getMinimum() - grow, box.getMaximum() + grow);
            ogreMesh->_setBounds(box, false);
            ogreMesh->_setBoundingSphereRadius(
                ogreMesh->getBoundingSphereRadius() * 1.5f);
        }
    } else if (skel.skeleton) {
        // Animations existed but none produced a usable track: no skeleton.
        Ogre::SkeletonManager::getSingleton().remove(skel.skeleton);
    }

    cache_.emplace(filePath, meshName);
    return meshName;
}

} // namespace avb
