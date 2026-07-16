#include "storage/AssetLibrary.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <utility>

#include "storage/Assets.h"
#include "storage/DataStore.h"
#include "storage/Transform.h"

namespace avb {

namespace {

namespace fs = std::filesystem;

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return s;
}

std::string trim(const std::string& s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

/// Files in `dir` whose (lowercased) extension is in `extensions`, sorted by
/// name so load order - and thus asset ids - is deterministic.
std::vector<fs::path> listFiles(const fs::path& dir,
                                const std::vector<std::string>& extensions) {
    std::vector<fs::path> files;
    std::error_code ec;
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end;
         it.increment(ec)) {
        std::error_code fec;
        if (!it->is_regular_file(fec)) {
            continue;
        }
        const std::string ext = toLower(it->path().extension().string());
        if (std::find(extensions.begin(), extensions.end(), ext) !=
            extensions.end()) {
            files.push_back(it->path());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

Id findImageByName(const DataStore& store, const std::string& name) {
    const std::string wanted = toLower(name);
    for (const Id id : store.imageIds()) {
        if (const ImageAsset* img = store.image(id)) {
            if (toLower(img->name) == wanted) {
                return id;
            }
        }
    }
    return kInvalidId;
}

Id findModelByName(const DataStore& store, const std::string& name) {
    const std::string wanted = toLower(name);
    for (const Id id : store.modelIds()) {
        if (const ModelAsset* model = store.model(id)) {
            if (toLower(model->name) == wanted) {
                return id;
            }
        }
    }
    return kInvalidId;
}

/// Splits a cfg pair line (comments/blank already skipped) into its parts:
/// `model = image [| pose]`. Returns false when there is no '='.
bool splitPairLine(const std::string& stripped, std::string& modelName,
                   std::string& imageName, std::string& poseSpec) {
    const auto eq = stripped.find('=');
    if (eq == std::string::npos) {
        return false;
    }
    modelName = trim(stripped.substr(0, eq));
    std::string rhs = trim(stripped.substr(eq + 1));
    poseSpec.clear();
    if (const auto bar = rhs.find('|'); bar != std::string::npos) {
        poseSpec = trim(rhs.substr(bar + 1));
        rhs = trim(rhs.substr(0, bar));
    }
    imageName = rhs;
    return !modelName.empty() && !imageName.empty();
}

/// Parses `count` comma-separated floats out of `values`. Returns false on
/// malformed input or wrong arity.
bool parseFloats(const std::string& values, float* out, int count) {
    std::stringstream in(values);
    std::string item;
    int parsed = 0;
    while (std::getline(in, item, ',')) {
        if (parsed >= count) {
            return false;
        }
        const std::string token = trim(item);
        char* end = nullptr;
        out[parsed] = std::strtof(token.c_str(), &end);
        if (token.empty() || end != token.c_str() + token.size()) {
            return false;
        }
        ++parsed;
    }
    return parsed == count;
}

/// A parsed pose column set: the editable transform plus the rigid origin it
/// is relative to (see Assignment).
struct PoseSpec {
    Transform origin;
    Transform transform;
};

/// Parses the optional pose columns (`t=x,y,z r=x,y,z s=v ot=x,y,z or=x,y,z`,
/// any subset, any order; `ot`/`or` are the origin's translation/rotation).
/// Malformed tokens are reported and fall back to that component's identity
/// default.
PoseSpec parsePose(const std::string& spec, int lineNo,
                   std::vector<std::string>& warnings) {
    PoseSpec pose;
    Transform& t = pose.transform;
    std::istringstream in(spec);
    std::string token;
    while (in >> token) {
        const auto warn = [&](const char* reason) {
            warnings.push_back("assignments.cfg line " + std::to_string(lineNo) +
                               ": pose token '" + token + "' " + reason +
                               ", using the default");
        };
        const auto eq = token.find('=');
        if (eq == std::string::npos || eq == 0 || eq + 1 >= token.size()) {
            warn("is not of the form t=x,y,z / r=x,y,z / s=x,y,z (or s=v) / "
                 "ot=x,y,z / or=x,y,z");
            continue;
        }
        const std::string key = toLower(token.substr(0, eq));
        const std::string values = token.substr(eq + 1);
        float nums[3] = {0.0f, 0.0f, 0.0f};
        if (key == "t" && parseFloats(values, nums, 3)) {
            t.translation = Eigen::Vector3f(nums[0], nums[1], nums[2]);
        } else if (key == "r" && parseFloats(values, nums, 3)) {
            t.rotationEulerDeg = Eigen::Vector3f(nums[0], nums[1], nums[2]);
        } else if (key == "ot" && parseFloats(values, nums, 3)) {
            pose.origin.translation = Eigen::Vector3f(nums[0], nums[1], nums[2]);
        } else if (key == "or" && parseFloats(values, nums, 3)) {
            pose.origin.rotationEulerDeg =
                Eigen::Vector3f(nums[0], nums[1], nums[2]);
        } else if (key == "s") {
            // Either three per-axis values or one uniform value.
            bool ok = parseFloats(values, nums, 3);
            if (!ok && parseFloats(values, nums, 1)) {
                nums[1] = nums[2] = nums[0];
                ok = true;
            }
            if (!ok) {
                warn("could not be parsed");
            } else if (nums[0] <= 0.0f || nums[1] <= 0.0f || nums[2] <= 0.0f) {
                warn("has a non-positive scale");
            } else {
                t.scale = Eigen::Vector3f(nums[0], nums[1], nums[2]);
            }
        } else {
            warn("could not be parsed");
        }
    }
    return pose;
}

std::string formatFloat(float v) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%g", static_cast<double>(v));
    return buffer;
}

/// Key identifying a (model, image) pair case-insensitively by file name.
std::string pairKey(const std::string& modelName, const std::string& imageName) {
    return toLower(modelName) + "\n" + toLower(imageName);
}

std::string formatVec3(const Eigen::Vector3f& v) {
    return formatFloat(v.x()) + "," + formatFloat(v.y()) + "," +
           formatFloat(v.z());
}

/// Formats the pose columns written after `|`; empty when both the transform
/// and origin are identity (a bare pair line already means "identity"). A
/// uniform scale is written as a single value, per-axis scales as x,y,z. The
/// origin's translation/rotation are written as ot=/or= only when set.
std::string formatPose(const Transform& origin, const Transform& t) {
    if (t.isIdentity() && origin.isIdentity()) {
        return "";
    }
    std::string scale = formatFloat(t.scale.x());
    if (t.scale.x() != t.scale.y() || t.scale.y() != t.scale.z()) {
        scale += "," + formatFloat(t.scale.y()) + "," + formatFloat(t.scale.z());
    }
    std::string pose = "t=" + formatVec3(t.translation) + " r=" +
                       formatVec3(t.rotationEulerDeg) + " s=" + scale;
    if (!origin.isIdentity()) {
        pose += " ot=" + formatVec3(origin.translation) + " or=" +
                formatVec3(origin.rotationEulerDeg);
    }
    return pose;
}

/// One complete cfg pair line: `model = image [| pose]`.
std::string formatPairLine(const std::string& modelName,
                           const std::string& imageName, const Transform& origin,
                           const Transform& t) {
    std::string line = modelName + " = " + imageName;
    if (const std::string pose = formatPose(origin, t); !pose.empty()) {
        line += " | " + pose;
    }
    return line;
}

/// True when `filePath` is a direct child of `<root>/<subdir>` - i.e. the
/// asset actually lives in the library and its name will resolve on the next
/// startup scan.
bool isInLibraryFolder(const std::string& filePath, const fs::path& root,
                       const char* subdir) {
    std::error_code ec;
    const fs::path parent =
        fs::weakly_canonical(fs::path(filePath), ec).parent_path();
    if (ec) {
        return false;
    }
    const fs::path wanted = fs::weakly_canonical(root / subdir, ec);
    return !ec && parent == wanted;
}

} // namespace

AssetLibrary::AssetLibrary(std::shared_ptr<DataStore> store,
                           ModelValidator modelValidator)
    : store_(std::move(store)), validateModel_(std::move(modelValidator)) {}

AssetLibrary::Report AssetLibrary::load(const std::string& rootDir) {
    Report report;
    std::error_code ec;
    const fs::path root(rootDir);
    if (!fs::is_directory(root, ec)) {
        return report; // no library on disk: nothing to load
    }

    // Images: validated by decoding, exactly like a manual upload.
    for (const fs::path& file :
         listFiles(root / "images", {".png", ".jpg", ".jpeg", ".bmp"})) {
        const Id id = store_->addImage(file.string());
        if (!store_->loadImagePixels(id)) {
            store_->removeImage(id);
            report.warnings.push_back("image '" + file.filename().string() +
                                      "' could not be decoded, skipped");
            continue;
        }
        ++report.imagesAdded;
    }

    // Models: validated through the injected checker (Assimp in the app).
    for (const fs::path& file : listFiles(root / "models", {".fbx"})) {
        std::string error;
        if (validateModel_ && !validateModel_(file.string(), &error)) {
            report.warnings.push_back("model '" + file.filename().string() +
                                      "' failed to load (" +
                                      (error.empty() ? "unknown error" : error) +
                                      "), skipped");
            continue;
        }
        store_->addModel(file.string());
        ++report.modelsAdded;
    }

    // Explicit name pairs from assignments.cfg. `!`-prefixed lines are
    // exclusions: they suppress the automatic stem pairing below (written by
    // saveSession when a name-matching pair was reverted in the app).
    std::set<std::string> excludedPairs;
    const fs::path cfgPath = root / "assignments.cfg";
    if (fs::exists(cfgPath, ec)) {
        std::ifstream cfg(cfgPath);
        std::string line;
        int lineNo = 0;
        while (std::getline(cfg, line)) {
            ++lineNo;
            const std::string stripped = trim(line);
            if (stripped.empty() || stripped[0] == '#') {
                continue;
            }
            if (stripped[0] == '!') {
                std::string exModel;
                std::string exImage;
                std::string exPose;
                if (splitPairLine(trim(stripped.substr(1)), exModel, exImage,
                                  exPose)) {
                    excludedPairs.insert(pairKey(exModel, exImage));
                } else {
                    report.warnings.push_back(
                        "assignments.cfg line " + std::to_string(lineNo) +
                        ": malformed exclusion, expected "
                        "'! model-file = image-file', skipped");
                }
                continue;
            }
            std::string modelName;
            std::string imageName;
            std::string poseSpec;
            if (!splitPairLine(stripped, modelName, imageName, poseSpec)) {
                report.warnings.push_back(
                    "assignments.cfg line " + std::to_string(lineNo) +
                    ": expected 'model-file = image-file [| pose]', skipped");
                continue;
            }
            const Id modelId = findModelByName(*store_, modelName);
            const Id imageId = findImageByName(*store_, imageName);
            if (modelId == kInvalidId || imageId == kInvalidId) {
                report.warnings.push_back(
                    "assignments.cfg line " + std::to_string(lineNo) + ": " +
                    (modelId == kInvalidId ? "model '" + modelName + "'"
                                           : "image '" + imageName + "'") +
                    " not found in the library, skipped");
                continue;
            }
            Id assignmentId;
            if (const auto existing = store_->findAssignment(modelId, imageId)) {
                assignmentId = *existing;
            } else {
                assignmentId = store_->assign(modelId, imageId);
                ++report.assignmentsCreated;
            }
            if (!poseSpec.empty()) {
                const PoseSpec pose =
                    parsePose(poseSpec, lineNo, report.warnings);
                store_->setTransform(assignmentId, pose.transform);
                store_->setOrigin(assignmentId, pose.origin);
            }
        }
    }

    // Automatic pairing: same base name (stem) links a model to an image,
    // e.g. dragon.fbx <-> dragon.png - unless the pair is excluded.
    for (const Id modelId : store_->modelIds()) {
        const ModelAsset* model = store_->model(modelId);
        if (!model) {
            continue;
        }
        const std::string stem = toLower(fs::path(model->name).stem().string());
        for (const Id imageId : store_->imageIds()) {
            const ImageAsset* img = store_->image(imageId);
            if (!img ||
                toLower(fs::path(img->name).stem().string()) != stem) {
                continue;
            }
            if (excludedPairs.count(pairKey(model->name, img->name))) {
                continue; // reverted in a saved session, keep it unassigned
            }
            if (!store_->findAssignment(modelId, imageId)) {
                store_->assign(modelId, imageId);
                ++report.assignmentsCreated;
            }
        }
    }

    return report;
}

bool AssetLibrary::persistAssignment(const std::string& rootDir,
                                     Id assignmentId) {
    const Assignment* assignment = store_->assignment(assignmentId);
    if (!assignment) {
        return false;
    }
    const ModelAsset* model = store_->model(assignment->modelId);
    const ImageAsset* image = store_->image(assignment->imageId);
    if (!model || !image) {
        return false;
    }
    const fs::path root(rootDir);
    if (!isInLibraryFolder(model->filePath, root, "models") ||
        !isInLibraryFolder(image->filePath, root, "images")) {
        return false; // not library assets: names would not resolve on restart
    }

    const std::string newLine = formatPairLine(
        model->name, image->name, assignment->origin, assignment->transform);

    // Rewrite only the line(s) for this pair; keep everything else - other
    // pairs, comments, blank lines - byte-for-byte. Duplicate lines for the
    // pair are collapsed into one (load applies them in order, so a stale
    // duplicate would win over the update otherwise), and an exclusion line
    // for the pair is dropped since the pair is assigned again.
    const fs::path cfgPath = root / "assignments.cfg";
    std::vector<std::string> lines;
    {
        std::ifstream in(cfgPath);
        std::string line;
        while (std::getline(in, line)) {
            lines.push_back(line);
        }
    }
    const std::string wanted = pairKey(model->name, image->name);
    bool replaced = false;
    std::vector<std::string> out;
    out.reserve(lines.size() + 1);
    for (const std::string& line : lines) {
        std::string stripped = trim(line);
        const bool exclusion = !stripped.empty() && stripped[0] == '!';
        if (exclusion) {
            stripped = trim(stripped.substr(1));
        }
        std::string modelName;
        std::string imageName;
        std::string poseSpec;
        const bool matchesPair =
            !stripped.empty() && stripped[0] != '#' &&
            splitPairLine(stripped, modelName, imageName, poseSpec) &&
            pairKey(modelName, imageName) == wanted;
        if (!matchesPair) {
            out.push_back(line);
        } else if (!exclusion && !replaced) {
            out.push_back(newLine);
            replaced = true;
        } // duplicates and now-stale exclusions of the pair are dropped
    }
    if (!replaced) {
        out.push_back(newLine);
    }

    std::ofstream file(cfgPath, std::ios::trunc);
    if (!file) {
        return false;
    }
    for (const std::string& line : out) {
        file << line << '\n';
    }
    return file.good();
}

AssetLibrary::SessionSaveResult AssetLibrary::saveSession(
    const std::string& rootDir) {
    SessionSaveResult result;
    const fs::path root(rootDir);
    std::error_code ec;
    fs::create_directories(root / "images", ec);
    fs::create_directories(root / "models", ec);

    // ---- 1. Move library files whose asset was removed from the session
    // into <root>/removed/ (never deleted outright, so nothing is lost).
    // Done before the copy-in below so that re-uploading a different file
    // under the same name replaces the library copy instead of hitting the
    // name-collision path.
    std::set<std::string> imageNames;
    for (const Id id : store_->imageIds()) {
        if (const ImageAsset* img = store_->image(id)) {
            imageNames.insert(toLower(img->name));
        }
    }
    std::set<std::string> modelNames;
    for (const Id id : store_->modelIds()) {
        if (const ModelAsset* model = store_->model(id)) {
            modelNames.insert(toLower(model->name));
        }
    }
    const auto moveRemoved = [&](const char* subdir,
                                 const std::vector<std::string>& extensions,
                                 const std::set<std::string>& inSession) {
        for (const fs::path& file : listFiles(root / subdir, extensions)) {
            const std::string name = file.filename().string();
            if (inSession.count(toLower(name))) {
                continue;
            }
            std::error_code mec;
            fs::create_directories(root / "removed", mec);
            fs::path dst = root / "removed" / name;
            for (int i = 1; fs::exists(dst, mec); ++i) {
                dst = root / "removed" / (std::to_string(i) + "_" + name);
            }
            fs::rename(file, dst, mec);
            if (mec) {
                result.warnings.push_back("could not move removed '" + name +
                                          "' out of the library: " +
                                          mec.message());
            } else {
                ++result.filesRemoved;
            }
        }
    };
    moveRemoved("images", {".png", ".jpg", ".jpeg", ".bmp"}, imageNames);
    moveRemoved("models", {".fbx"}, modelNames);

    // ---- 2. Copy external files into the library.
    // Copies `filePath` (named `name`) into the library subfolder unless a
    // file of that name is already there; returns the library path to use, or
    // empty on failure.
    const auto bringIntoLibrary = [&](const std::string& filePath,
                                      const std::string& name,
                                      const char* subdir) -> std::string {
        const fs::path dst = root / subdir / name;
        std::error_code fec;
        if (fs::exists(dst, fec)) {
            // A file of this name is already in the library; it wins (names
            // are the identity the cfg resolves by). Flag likely mismatches.
            std::error_code sec;
            const auto srcSize = fs::file_size(filePath, sec);
            const auto dstSize = fs::file_size(dst, fec);
            if (!sec && !fec && srcSize != dstSize) {
                result.warnings.push_back(
                    "'" + name + "' already exists in the library with "
                    "different content; the library version is kept");
            }
            return dst.string();
        }
        if (!fs::copy_file(filePath, dst, fec) || fec) {
            result.warnings.push_back("could not copy '" + name +
                                      "' into the library: " + fec.message());
            return "";
        }
        ++result.filesCopied;
        return dst.string();
    };

    for (const Id id : store_->imageIds()) {
        const ImageAsset* img = store_->image(id);
        if (!img || isInLibraryFolder(img->filePath, root, "images")) {
            continue;
        }
        const std::string libraryPath =
            bringIntoLibrary(img->filePath, img->name, "images");
        if (!libraryPath.empty()) {
            store_->setImageFilePath(id, libraryPath);
        }
    }
    for (const Id id : store_->modelIds()) {
        const ModelAsset* model = store_->model(id);
        if (!model || isInLibraryFolder(model->filePath, root, "models")) {
            continue;
        }
        const std::string libraryPath =
            bringIntoLibrary(model->filePath, model->name, "models");
        if (!libraryPath.empty()) {
            store_->setModelFilePath(id, libraryPath);
        }
    }

    // ---- 3. Rewrite assignments.cfg to hold exactly the current session:
    // fresh pair lines for every current assignment, `!` exclusions for
    // stem-matching pairs that are currently unassigned (so reverting an
    // auto-paired assignment sticks), stale lines dropped, comments kept.
    std::map<std::string, std::string> pairLines;       // key -> line
    std::map<std::string, std::string> exclusionLines;  // key -> line
    for (const Id assignmentId : store_->assignmentIds()) {
        const Assignment* a = store_->assignment(assignmentId);
        const ModelAsset* model = a ? store_->model(a->modelId) : nullptr;
        const ImageAsset* image = a ? store_->image(a->imageId) : nullptr;
        if (!model || !image) {
            continue;
        }
        if (!isInLibraryFolder(model->filePath, root, "models") ||
            !isInLibraryFolder(image->filePath, root, "images")) {
            result.warnings.push_back(
                "assignment '" + model->name + " = " + image->name +
                "' could not be saved (its files are not in the library)");
            continue;
        }
        pairLines[pairKey(model->name, image->name)] =
            formatPairLine(model->name, image->name, a->origin, a->transform);
    }
    for (const Id modelId : store_->modelIds()) {
        const ModelAsset* model = store_->model(modelId);
        if (!model || !isInLibraryFolder(model->filePath, root, "models")) {
            continue;
        }
        const std::string stem = toLower(fs::path(model->name).stem().string());
        for (const Id imageId : store_->imageIds()) {
            const ImageAsset* img = store_->image(imageId);
            if (!img || !isInLibraryFolder(img->filePath, root, "images") ||
                toLower(fs::path(img->name).stem().string()) != stem ||
                store_->findAssignment(modelId, imageId)) {
                continue;
            }
            exclusionLines[pairKey(model->name, img->name)] =
                "! " + model->name + " = " + img->name;
        }
    }
    result.assignmentsSaved = static_cast<int>(pairLines.size());

    const fs::path cfgPath = root / "assignments.cfg";
    std::vector<std::string> lines;
    {
        std::ifstream in(cfgPath);
        std::string line;
        while (std::getline(in, line)) {
            lines.push_back(line);
        }
    }
    std::vector<std::string> out;
    out.reserve(lines.size() + pairLines.size() + exclusionLines.size());
    for (const std::string& line : lines) {
        std::string stripped = trim(line);
        if (stripped.empty() || stripped[0] == '#') {
            out.push_back(line); // comments and spacing survive the sync
            continue;
        }
        const bool exclusion = stripped[0] == '!';
        if (exclusion) {
            stripped = trim(stripped.substr(1));
        }
        std::string modelName;
        std::string imageName;
        std::string poseSpec;
        if (!splitPairLine(stripped, modelName, imageName, poseSpec)) {
            out.push_back(line); // unparseable: keep, load() will warn
            continue;
        }
        const std::string key = pairKey(modelName, imageName);
        auto& source = exclusion ? exclusionLines : pairLines;
        if (const auto it = source.find(key); it != source.end()) {
            out.push_back(it->second); // refresh in place
            source.erase(it);
        }
        // else: stale (pair reverted / asset removed) -> dropped
    }
    for (const auto& [key, line] : pairLines) {
        out.push_back(line); // new assignments without an existing line
    }
    for (const auto& [key, line] : exclusionLines) {
        out.push_back(line);
    }

    std::ofstream file(cfgPath, std::ios::trunc);
    if (!file) {
        result.warnings.push_back("could not write " + cfgPath.string());
        return result;
    }
    for (const std::string& line : out) {
        file << line << '\n';
    }
    return result;
}

} // namespace avb
