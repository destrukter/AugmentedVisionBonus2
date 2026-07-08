#include "storage/AssetLibrary.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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

/// Parses the optional pose columns (`t=x,y,z r=x,y,z s=v`, any subset, any
/// order). Malformed tokens are reported and fall back to that component's
/// identity default.
Transform parsePose(const std::string& spec, int lineNo,
                    std::vector<std::string>& warnings) {
    Transform t;
    std::istringstream in(spec);
    std::string token;
    while (in >> token) {
        const auto warn = [&](const char* reason) {
            warnings.push_back("assignments.cfg line " + std::to_string(lineNo) +
                               ": pose token '" + token + "' " + reason +
                               ", using the default");
        };
        if (token.size() < 3 || token[1] != '=') {
            warn("is not of the form t=x,y,z / r=x,y,z / s=x,y,z (or s=v)");
            continue;
        }
        const char key = static_cast<char>(std::tolower(token[0]));
        const std::string values = token.substr(2);
        float nums[3] = {0.0f, 0.0f, 0.0f};
        if (key == 't' && parseFloats(values, nums, 3)) {
            t.translation = Eigen::Vector3f(nums[0], nums[1], nums[2]);
        } else if (key == 'r' && parseFloats(values, nums, 3)) {
            t.rotationEulerDeg = Eigen::Vector3f(nums[0], nums[1], nums[2]);
        } else if (key == 's') {
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
    return t;
}

std::string formatFloat(float v) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%g", static_cast<double>(v));
    return buffer;
}

/// Formats the pose columns written after `|`; empty for the identity pose
/// (a bare pair line already means "identity"). A uniform scale is written as
/// a single value, per-axis scales as x,y,z.
std::string formatPose(const Transform& t) {
    if (t.isIdentity()) {
        return "";
    }
    std::string scale = formatFloat(t.scale.x());
    if (t.scale.x() != t.scale.y() || t.scale.y() != t.scale.z()) {
        scale += "," + formatFloat(t.scale.y()) + "," + formatFloat(t.scale.z());
    }
    return "t=" + formatFloat(t.translation.x()) + "," +
           formatFloat(t.translation.y()) + "," +
           formatFloat(t.translation.z()) + " r=" +
           formatFloat(t.rotationEulerDeg.x()) + "," +
           formatFloat(t.rotationEulerDeg.y()) + "," +
           formatFloat(t.rotationEulerDeg.z()) + " s=" + scale;
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

    // Explicit name pairs from assignments.cfg.
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
                store_->setTransform(
                    assignmentId, parsePose(poseSpec, lineNo, report.warnings));
            }
        }
    }

    // Automatic pairing: same base name (stem) links a model to an image,
    // e.g. dragon.fbx <-> dragon.png.
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

    std::string newLine = model->name + " = " + image->name;
    if (const std::string pose = formatPose(assignment->transform);
        !pose.empty()) {
        newLine += " | " + pose;
    }

    // Rewrite only the line(s) for this pair; keep everything else - other
    // pairs, comments, blank lines - byte-for-byte. Duplicate lines for the
    // pair are collapsed into one (load applies them in order, so a stale
    // duplicate would win over the update otherwise).
    const fs::path cfgPath = root / "assignments.cfg";
    std::vector<std::string> lines;
    {
        std::ifstream in(cfgPath);
        std::string line;
        while (std::getline(in, line)) {
            lines.push_back(line);
        }
    }
    const std::string wantedModel = toLower(model->name);
    const std::string wantedImage = toLower(image->name);
    bool replaced = false;
    std::vector<std::string> out;
    out.reserve(lines.size() + 1);
    for (const std::string& line : lines) {
        const std::string stripped = trim(line);
        std::string modelName;
        std::string imageName;
        std::string poseSpec;
        const bool matchesPair =
            !stripped.empty() && stripped[0] != '#' &&
            splitPairLine(stripped, modelName, imageName, poseSpec) &&
            toLower(modelName) == wantedModel && toLower(imageName) == wantedImage;
        if (!matchesPair) {
            out.push_back(line);
        } else if (!replaced) {
            out.push_back(newLine);
            replaced = true;
        } // further duplicates of the pair are dropped
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

} // namespace avb
