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

static void test_no_automatic_name_based_pairing() {
    TempLibrary lib;
    // A model and images sharing its base name: without a cfg line they must
    // stay unassigned (automatic stem pairing is intentionally not a thing).
    lib.addImage("Dragon.png");
    lib.addImage("dragon.jpg");
    lib.addModel("dragon.fbx");

    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    const AssetLibrary::Report report = loader.load(lib.root.string());

    CHECK(report.warnings.empty());
    CHECK(report.assignmentsCreated == 0);
    CHECK(store->assignmentIds().empty());

    // Legacy '!' exclusion lines (from when auto-pairing existed) are
    // tolerated silently.
    lib.writeCfg("! dragon.fbx = Dragon.png\n");
    auto store2 = std::make_shared<DataStore>();
    AssetLibrary loader2(store2, stubValidator);
    const AssetLibrary::Report report2 = loader2.load(lib.root.string());
    CHECK(report2.warnings.empty());
    CHECK(store2->assignmentIds().empty());
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
    lib.writeCfg("# user comment stays intact\ndragon.fbx = dragon.png\n");

    // First run: paired via the cfg, user configures + saves a pose.
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

static void test_duplicate_pair_lines_create_instances() {
    TempLibrary lib;
    lib.addImage("dragon.png");
    lib.addModel("dragon.fbx");
    // Two lines for the same pair = the model placed on the image twice,
    // each instance with its own pose (in file order).
    lib.writeCfg(
        "dragon.fbx = dragon.png | t=0.5,0,0\n"
        "dragon.fbx = dragon.png | t=-0.5,0,0 s=2\n");

    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    const AssetLibrary::Report report = loader.load(lib.root.string());
    CHECK(report.warnings.empty());
    CHECK(report.assignmentsCreated == 2);

    const Id image = imageByName(*store, "dragon.png");
    const std::vector<Id> instances = store->assignmentsForImage(image);
    CHECK(instances.size() == 2);
    if (instances.size() != 2) {
        return;
    }
    CHECK(store->transform(instances[0])->translation.x() > 0.4f);
    CHECK(store->transform(instances[1])->translation.x() < -0.4f);
    CHECK(store->transform(instances[1])->scale.x() > 1.9f);

    // Persisting one instance rewrites the pair's lines from the store and
    // must not lose its sibling.
    Transform moved = *store->transform(instances[0]);
    moved.translation.y() = 0.25f;
    store->setTransform(instances[0], moved);
    CHECK(loader.persistAssignment(lib.root.string(), instances[0]));

    auto store2 = std::make_shared<DataStore>();
    AssetLibrary loader2(store2, stubValidator);
    const AssetLibrary::Report report2 = loader2.load(lib.root.string());
    CHECK(report2.warnings.empty());
    const std::vector<Id> restored =
        store2->assignmentsForImage(imageByName(*store2, "dragon.png"));
    CHECK(restored.size() == 2);
    if (restored.size() != 2) {
        return;
    }
    CHECK(std::abs(store2->transform(restored[0])->translation.y() - 0.25f) <
          1e-4f);
    CHECK(store2->transform(restored[1])->scale.x() > 1.9f);

    // A full session save also keeps one line per instance...
    const AssetLibrary::SessionSaveResult saved =
        loader2.saveSession(lib.root.string());
    CHECK(saved.warnings.empty());
    CHECK(saved.assignmentsSaved == 2);

    // ...so a third run still restores both instances.
    auto store3 = std::make_shared<DataStore>();
    AssetLibrary loader3(store3, stubValidator);
    loader3.load(lib.root.string());
    CHECK(store3->assignmentsForImage(imageByName(*store3, "dragon.png"))
              .size() == 2);
}

static void test_persist_identity_writes_bare_pair() {
    TempLibrary lib;
    lib.addImage("dragon.png");
    lib.addModel("dragon.fbx");

    // Assigned in the app (no cfg line yet); persisting appends one.
    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    loader.load(lib.root.string());
    const Id aid = store->assign(modelByName(*store, "dragon.fbx"),
                                 imageByName(*store, "dragon.png"));
    CHECK(loader.persistAssignment(lib.root.string(), aid));

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

static void test_save_session_copies_external_files_and_persists() {
    TempLibrary lib;
    // External assets living outside the library folders.
    const fs::path ext =
        fs::temp_directory_path() /
        ("avb_session_ext_" + std::to_string(std::rand()));
    fs::create_directories(ext);
    {
        cv::Mat img(64, 64, CV_8UC3);
        cv::RNG rng(5);
        rng.fill(img, cv::RNG::UNIFORM, 0, 256);
        cv::imwrite((ext / "poster.png").string(), img);
        std::ofstream((ext / "robot.fbx").string()) << "fake fbx";
    }

    auto store = std::make_shared<DataStore>();
    const Id img = store->addImage((ext / "poster.png").string());
    store->loadImagePixels(img);
    const Id mdl = store->addModel((ext / "robot.fbx").string());
    const Id aid = store->assign(mdl, img);
    Transform pose;
    pose.translation = Eigen::Vector3f(0.0f, 0.3f, 0.0f);
    pose.scale = Eigen::Vector3f(2.0f, 2.0f, 2.0f);
    store->setTransform(aid, pose);

    AssetLibrary loader(store, stubValidator);
    const AssetLibrary::SessionSaveResult saved =
        loader.saveSession(lib.root.string());
    CHECK(saved.filesCopied == 2);
    CHECK(saved.assignmentsSaved == 1);
    CHECK(saved.warnings.empty());
    CHECK(fs::exists(lib.root / "images" / "poster.png"));
    CHECK(fs::exists(lib.root / "models" / "robot.fbx"));
    // The store now points at the library copies, so subsequent per-save
    // persistence works too.
    CHECK(store->image(img)->filePath.find(lib.root.string()) == 0);
    CHECK(store->model(mdl)->filePath.find(lib.root.string()) == 0);

    // A fresh start restores the whole session, pose included.
    auto store2 = std::make_shared<DataStore>();
    AssetLibrary loader2(store2, stubValidator);
    const AssetLibrary::Report report = loader2.load(lib.root.string());
    CHECK(report.imagesAdded == 1);
    CHECK(report.modelsAdded == 1);
    const auto aid2 = store2->findAssignment(modelByName(*store2, "robot.fbx"),
                                             imageByName(*store2, "poster.png"));
    CHECK(aid2.has_value());
    if (aid2) {
        const auto restored = store2->transform(*aid2);
        CHECK(restored->translation.isApprox(pose.translation, 1e-4f));
        CHECK(restored->scale.isApprox(pose.scale, 1e-4f));
    }

    fs::remove_all(ext);
}

static void test_save_session_drops_reverted_assignments() {
    TempLibrary lib;
    lib.addImage("dragon.png");
    lib.addModel("dragon.fbx");
    // A pair line plus a legacy '!' exclusion line (from the era of
    // automatic name-based pairing).
    lib.writeCfg("dragon.fbx = dragon.png\n! dragon.fbx = dragon.png\n");

    // First run: the user reverts the pair and saves the session.
    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    loader.load(lib.root.string());
    const Id model = modelByName(*store, "dragon.fbx");
    const Id image = imageByName(*store, "dragon.png");
    CHECK(store->findAssignment(model, image).has_value());
    store->unassign(model, image);
    const AssetLibrary::SessionSaveResult saved =
        loader.saveSession(lib.root.string());
    CHECK(saved.assignmentsSaved == 0);

    // The pair line is gone and the legacy exclusion line was cleaned up.
    std::ifstream cfg((lib.root / "assignments.cfg").string());
    std::string contents((std::istreambuf_iterator<char>(cfg)),
                         std::istreambuf_iterator<char>());
    CHECK(contents.find("dragon.fbx") == std::string::npos);

    // Second run: nothing is assigned (and no auto-pairing resurrects it).
    auto store2 = std::make_shared<DataStore>();
    AssetLibrary loader2(store2, stubValidator);
    const AssetLibrary::Report report = loader2.load(lib.root.string());
    CHECK(report.warnings.empty());
    CHECK(report.assignmentsCreated == 0);
    CHECK(store2->assignmentIds().empty());

    // Re-assigning and saving again restores the pair line.
    store2->assign(modelByName(*store2, "dragon.fbx"),
                   imageByName(*store2, "dragon.png"));
    loader2.saveSession(lib.root.string());
    std::ifstream cfg2((lib.root / "assignments.cfg").string());
    std::string contents2((std::istreambuf_iterator<char>(cfg2)),
                          std::istreambuf_iterator<char>());
    CHECK(contents2.find("dragon.fbx = dragon.png") != std::string::npos);
}

static void test_save_session_moves_removed_assets_and_drops_stale_lines() {
    TempLibrary lib;
    lib.addImage("marker.png");
    lib.addModel("statue.fbx");
    lib.writeCfg("# keep this comment\nstatue.fbx = marker.png | s=2\n");

    auto store = std::make_shared<DataStore>();
    AssetLibrary loader(store, stubValidator);
    loader.load(lib.root.string());
    // The user removes the model entirely (cascades the assignment).
    store->removeModel(modelByName(*store, "statue.fbx"));

    const AssetLibrary::SessionSaveResult saved =
        loader.saveSession(lib.root.string());
    CHECK(saved.filesRemoved == 1);
    CHECK(saved.assignmentsSaved == 0);
    CHECK(!fs::exists(lib.root / "models" / "statue.fbx"));
    CHECK(fs::exists(lib.root / "removed" / "statue.fbx"));

    std::ifstream cfg((lib.root / "assignments.cfg").string());
    std::string contents((std::istreambuf_iterator<char>(cfg)),
                         std::istreambuf_iterator<char>());
    CHECK(contents.find("# keep this comment") != std::string::npos);
    CHECK(contents.find("statue.fbx = marker.png") == std::string::npos);

    // Next start: only the image remains, nothing is assigned.
    auto store2 = std::make_shared<DataStore>();
    AssetLibrary loader2(store2, stubValidator);
    const AssetLibrary::Report report = loader2.load(lib.root.string());
    CHECK(report.imagesAdded == 1);
    CHECK(report.modelsAdded == 0);
    CHECK(store2->assignmentIds().empty());
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
    test_no_automatic_name_based_pairing();
    test_cfg_pose_columns_are_applied();
    test_cfg_per_axis_scale();
    test_cfg_pose_partial_and_missing_defaults();
    test_cfg_pose_malformed_tokens_warn_and_default();
    test_persisted_pose_survives_reload();
    test_duplicate_pair_lines_create_instances();
    test_persist_identity_writes_bare_pair();
    test_persist_rejects_non_library_assets();
    test_save_session_copies_external_files_and_persists();
    test_save_session_drops_reverted_assignments();
    test_save_session_moves_removed_assets_and_drops_stale_lines();
    test_missing_root_is_not_an_error();
}
