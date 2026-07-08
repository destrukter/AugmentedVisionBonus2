#include "TestMain.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
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
    test_missing_root_is_not_an_error();
}
