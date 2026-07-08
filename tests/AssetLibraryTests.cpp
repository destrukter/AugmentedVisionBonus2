#include "TestMain.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "storage/AssetLibrary.h"
#include "storage/DataStore.h"

using namespace avb;

namespace {

namespace fs = std::filesystem;

// Stub validator standing in for the Assimp check: any file whose name
// contains "bad" is rejected.
bool stubValidator(const std::string& path, std::string* error) {
    if (path.find("bad") != std::string::npos) {
        if (error) {
            *error = "stub rejection";
        }
        return false;
    }
    return true;
}

/// Creates a disposable library folder with images/, models/ and a cfg.
struct TempLibrary {
    fs::path root;

    TempLibrary() {
        root = fs::temp_directory_path() /
               ("avb_library_test_" + std::to_string(std::rand()));
        fs::remove_all(root);
        fs::create_directories(root / "images");
        fs::create_directories(root / "models");
    }
    ~TempLibrary() { fs::remove_all(root); }

    void addImage(const std::string& name) {
        cv::Mat img(64, 64, CV_8UC3);
        cv::RNG rng(42);
        rng.fill(img, cv::RNG::UNIFORM, 0, 256);
        cv::imwrite((root / "images" / name).string(), img);
    }
    void addBrokenImage(const std::string& name) {
        std::ofstream((root / "images" / name).string()) << "not an image";
    }
    void addModel(const std::string& name) {
        std::ofstream((root / "models" / name).string()) << "fake fbx";
    }
    void writeCfg(const std::string& contents) {
        std::ofstream((root / "assignments.cfg").string()) << contents;
    }
};

Id imageByName(const DataStore& store, const std::string& name) {
    for (const Id id : store.imageIds()) {
        if (store.image(id)->name == name) {
            return id;
        }
    }
    return kInvalidId;
}

Id modelByName(const DataStore& store, const std::string& name) {
    for (const Id id : store.modelIds()) {
        if (store.model(id)->name == name) {
            return id;
        }
    }
    return kInvalidId;
}

} // namespace

static void test_loads_and_validates_assets() {
    TempLibrary lib;
    lib.addImage("marker.png");
    lib.addBrokenImage("broken.png");
    lib.addModel("dragon.fbx");
    lib.addModel("bad-model.fbx");

    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    const AssetLibrary::Report report = loader.load(lib.root.string());

    CHECK(report.imagesAdded == 1);
    CHECK(report.modelsAdded == 1);
    CHECK(report.warnings.size() == 2); // broken image + rejected model
    CHECK(store->imageIds().size() == 1);
    CHECK(store->modelIds().size() == 1);
}

static void test_explicit_pairs_from_cfg() {
    TempLibrary lib;
    lib.addImage("marker.png");
    lib.addModel("statue.fbx");
    lib.writeCfg("# comment\n"
                 "statue.fbx = marker.png\n"
                 "ghost.fbx = marker.png\n"   // unknown model -> warning
                 "not a pair line\n");        // malformed -> warning

    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    const AssetLibrary::Report report = loader.load(lib.root.string());

    CHECK(report.assignmentsCreated == 1);
    CHECK(report.warnings.size() == 2);
    const Id model = modelByName(*store, "statue.fbx");
    const Id image = imageByName(*store, "marker.png");
    CHECK(store->findAssignment(model, image).has_value());
}

static void test_auto_pairs_by_stem() {
    TempLibrary lib;
    lib.addImage("Dragon.png");   // stem match is case-insensitive
    lib.addImage("other.png");
    lib.addModel("dragon.fbx");

    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    const AssetLibrary::Report report = loader.load(lib.root.string());

    CHECK(report.assignmentsCreated == 1);
    const Id model = modelByName(*store, "dragon.fbx");
    const Id image = imageByName(*store, "Dragon.png");
    CHECK(store->findAssignment(model, image).has_value());
    CHECK(!store->findAssignment(model, imageByName(*store, "other.png")));
}

static void test_cfg_and_stem_pair_do_not_duplicate() {
    TempLibrary lib;
    lib.addImage("dragon.png");
    lib.addModel("dragon.fbx");
    lib.writeCfg("dragon.fbx = dragon.png\n");

    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    const AssetLibrary::Report report = loader.load(lib.root.string());

    // The cfg pair and the stem auto-pair resolve to the same assignment.
    CHECK(report.assignmentsCreated == 1);
    CHECK(store->assignmentIds().size() == 1);
}

static void test_cfg_pose_columns_are_applied() {
    TempLibrary lib;
    lib.addImage("marker.png");
    lib.addModel("statue.fbx");
    lib.writeCfg("statue.fbx = marker.png | t=0.5,-1,0.25 r=0,90,45 s=2\n");

    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    const AssetLibrary::Report report = loader.load(lib.root.string());
    CHECK(report.warnings.empty());

    const auto aid = store->findAssignment(modelByName(*store, "statue.fbx"),
                                           imageByName(*store, "marker.png"));
    CHECK(aid.has_value());
    if (!aid) {
        return;
    }
    const auto t = store->transform(*aid);
    CHECK(t.has_value());
    CHECK(t->translation.isApprox(Eigen::Vector3f(0.5f, -1.0f, 0.25f)));
    CHECK(t->rotationEulerDeg.isApprox(Eigen::Vector3f(0.0f, 90.0f, 45.0f)));
    // A single s value scales all axes uniformly.
    CHECK(t->scale.isApprox(Eigen::Vector3f(2.0f, 2.0f, 2.0f)));
}

static void test_cfg_per_axis_scale() {
    TempLibrary lib;
    lib.addImage("marker.png");
    lib.addModel("statue.fbx");
    lib.writeCfg("statue.fbx = marker.png | s=1,2,3\n");

    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    const AssetLibrary::Report report = loader.load(lib.root.string());
    CHECK(report.warnings.empty());

    const auto aid = store->findAssignment(modelByName(*store, "statue.fbx"),
                                           imageByName(*store, "marker.png"));
    const auto t = store->transform(*aid);
    CHECK(t->scale.isApprox(Eigen::Vector3f(1.0f, 2.0f, 3.0f)));
}

static void test_cfg_pose_partial_and_missing_defaults() {
    TempLibrary lib;
    lib.addImage("marker.png");
    lib.addModel("statue.fbx");
    // Only the scale is given; translation/rotation must stay at identity.
    lib.writeCfg("statue.fbx = marker.png | s=3\n");

    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    loader.load(lib.root.string());

    const auto aid = store->findAssignment(modelByName(*store, "statue.fbx"),
                                           imageByName(*store, "marker.png"));
    const auto t = store->transform(*aid);
    CHECK(t->translation.isZero());
    CHECK(t->rotationEulerDeg.isZero());
    CHECK(t->scale.isApprox(Eigen::Vector3f(3.0f, 3.0f, 3.0f)));
}

static void test_cfg_pose_malformed_tokens_warn_and_default() {
    TempLibrary lib;
    lib.addImage("marker.png");
    lib.addModel("statue.fbx");
    lib.writeCfg("statue.fbx = marker.png | t=1,2 s=abc r=0,10,0\n");

    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    const AssetLibrary::Report report = loader.load(lib.root.string());
    CHECK(report.warnings.size() == 2); // bad arity t=, bad float s=

    const auto aid = store->findAssignment(modelByName(*store, "statue.fbx"),
                                           imageByName(*store, "marker.png"));
    const auto t = store->transform(*aid);
    CHECK(t->translation.isZero());  // malformed -> default
    CHECK(t->scale.isOnes());
    CHECK(t->rotationEulerDeg.isApprox(Eigen::Vector3f(0.0f, 10.0f, 0.0f)));
}

static void test_persisted_pose_survives_reload() {
    TempLibrary lib;
    lib.addImage("dragon.png");
    lib.addModel("dragon.fbx");
    lib.writeCfg("# user comment stays intact\n");

    // First run: auto-paired by stem, user configures + saves a pose.
    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    loader.load(lib.root.string());
    const auto aid = store->findAssignment(modelByName(*store, "dragon.fbx"),
                                           imageByName(*store, "dragon.png"));
    CHECK(aid.has_value());
    Transform pose;
    pose.translation = Eigen::Vector3f(0.1f, 0.2f, 0.3f);
    pose.rotationEulerDeg = Eigen::Vector3f(10.0f, 20.0f, 30.0f);
    pose.scale = Eigen::Vector3f(1.5f, 2.5f, 0.75f); // per-axis round trip
    store->setTransform(*aid, pose);
    CHECK(loader.persistAssignment(lib.root.string(), *aid));

    // The comment survived the surgical rewrite.
    std::ifstream cfg((lib.root / "assignments.cfg").string());
    std::string contents((std::istreambuf_iterator<char>(cfg)),
                         std::istreambuf_iterator<char>());
    CHECK(contents.find("# user comment stays intact") != std::string::npos);
    CHECK(contents.find("dragon.fbx = dragon.png |") != std::string::npos);

    // Second run (fresh store): the pose is restored.
    auto store2 = std::make_shared<DataStore>();
    AssetLibrary loader2(store2, stubValidator);
    const AssetLibrary::Report report2 = loader2.load(lib.root.string());
    CHECK(report2.warnings.empty());
    const auto aid2 =
        store2->findAssignment(modelByName(*store2, "dragon.fbx"),
                               imageByName(*store2, "dragon.png"));
    CHECK(aid2.has_value());
    const auto restored = store2->transform(*aid2);
    CHECK(restored->translation.isApprox(pose.translation, 1e-4f));
    CHECK(restored->rotationEulerDeg.isApprox(pose.rotationEulerDeg, 1e-4f));
    CHECK(restored->scale.isApprox(pose.scale, 1e-4f));
}

static void test_persist_identity_writes_bare_pair() {
    TempLibrary lib;
    lib.addImage("dragon.png");
    lib.addModel("dragon.fbx");

    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    loader.load(lib.root.string());
    const auto aid = store->findAssignment(modelByName(*store, "dragon.fbx"),
                                           imageByName(*store, "dragon.png"));
    CHECK(loader.persistAssignment(lib.root.string(), *aid));

    std::ifstream cfg((lib.root / "assignments.cfg").string());
    std::string contents((std::istreambuf_iterator<char>(cfg)),
                         std::istreambuf_iterator<char>());
    CHECK(contents.find("dragon.fbx = dragon.png") != std::string::npos);
    CHECK(contents.find('|') == std::string::npos); // identity: no pose columns
}

static void test_persist_rejects_non_library_assets() {
    TempLibrary lib;
    auto store = std::make_shared<DataStore>();
    const Id img = store->addImage("/somewhere/else/marker.png");
    const Id mdl = store->addModel("/somewhere/else/model.fbx");
    const Id aid = store->assign(mdl, img);

    AssetLibrary loader(store, stubValidator);
    CHECK(!loader.persistAssignment(lib.root.string(), aid));
}

static void test_missing_root_is_not_an_error() {
    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    const AssetLibrary::Report report =
        loader.load("/nonexistent/avb-library-path");
    CHECK(report.imagesAdded == 0);
    CHECK(report.modelsAdded == 0);
    CHECK(report.warnings.empty());
}

void run_assetlibrary_tests() {
    test_loads_and_validates_assets();
    test_explicit_pairs_from_cfg();
    test_auto_pairs_by_stem();
    test_cfg_and_stem_pair_do_not_duplicate();
    test_cfg_pose_columns_are_applied();
    test_cfg_per_axis_scale();
    test_cfg_pose_partial_and_missing_defaults();
    test_cfg_pose_malformed_tokens_warn_and_default();
    test_persisted_pose_survives_reload();
    test_persist_identity_writes_bare_pair();
    test_persist_rejects_non_library_assets();
    test_missing_root_is_not_an_error();
}
