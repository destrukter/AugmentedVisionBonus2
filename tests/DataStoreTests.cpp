#include "TestMain.h"

#include "storage/DataStore.h"

using namespace avb;

static void test_add_and_lookup() {
    DataStore store;
    const Id img = store.addImage("/tmp/marker.png");
    const Id model = store.addModel("/tmp/teapot.fbx");

    CHECK(img != kInvalidId);
    CHECK(model != kInvalidId);
    CHECK(store.image(img) != nullptr);
    CHECK(store.image(img)->name == "marker.png");
    CHECK(store.model(model)->name == "teapot.fbx");
    CHECK(store.imageIds().size() == 1);
    CHECK(store.modelIds().size() == 1);
}

static void test_assignment_defaults_to_identity() {
    DataStore store;
    const Id img = store.addImage("a.png");
    const Id model = store.addModel("m.fbx");
    const Id a = store.assign(model, img);

    CHECK(a != kInvalidId);
    const auto t = store.transform(a);
    CHECK(t.has_value());
    CHECK(t->isIdentity());            // rotation + translation default to 0
}

static void test_one_model_many_images() {
    DataStore store;
    const Id model = store.addModel("m.fbx");
    const Id img1 = store.addImage("1.png");
    const Id img2 = store.addImage("2.png");

    const Id a1 = store.assign(model, img1);
    const Id a2 = store.assign(model, img2);
    CHECK(a1 != a2);
    CHECK(store.assignmentsForModel(model).size() == 2);
    CHECK(store.assignmentsForImage(img1).size() == 1);
}

static void test_assign_is_idempotent_per_pair() {
    DataStore store;
    const Id model = store.addModel("m.fbx");
    const Id img = store.addImage("1.png");
    const Id a1 = store.assign(model, img);
    const Id a2 = store.assign(model, img);  // same pair
    CHECK(a1 == a2);
    CHECK(store.assignmentIds().size() == 1);
}

static void test_revert_and_reassign() {
    DataStore store;
    const Id model = store.addModel("m.fbx");
    const Id img1 = store.addImage("1.png");
    const Id img2 = store.addImage("2.png");
    const Id a = store.assign(model, img1);

    CHECK(store.unassign(a));
    CHECK(store.assignment(a) == nullptr);
    CHECK(store.assignmentsForModel(model).empty());

    // Re-assign differently.
    const Id b = store.assign(model, img2);
    CHECK(b != kInvalidId);
    CHECK(store.assignmentsForImage(img2).size() == 1);
}

static void test_set_transform_persists() {
    DataStore store;
    const Id model = store.addModel("m.fbx");
    const Id img = store.addImage("1.png");
    const Id a = store.assign(model, img);

    Transform t;
    t.translation = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    t.rotationEulerDeg = Eigen::Vector3f(0.0f, 90.0f, 0.0f);
    CHECK(store.setTransform(a, t));

    const auto read = store.transform(a);
    CHECK(read.has_value());
    CHECK(read->translation.isApprox(Eigen::Vector3f(1.0f, 2.0f, 3.0f)));
    CHECK(!read->isIdentity());
}

static void test_removing_image_drops_assignments() {
    DataStore store;
    const Id model = store.addModel("m.fbx");
    const Id img = store.addImage("1.png");
    store.assign(model, img);
    CHECK(store.removeImage(img));
    CHECK(store.assignmentsForModel(model).empty());
}

static void test_assign_unknown_ids_fails() {
    DataStore store;
    const Id img = store.addImage("1.png");
    CHECK(store.assign(/*model*/ 999, img) == kInvalidId);
    CHECK(store.assign(/*model*/ kInvalidId, img) == kInvalidId);
}

void run_datastore_tests() {
    test_add_and_lookup();
    test_assignment_defaults_to_identity();
    test_one_model_many_images();
    test_assign_is_idempotent_per_pair();
    test_revert_and_reassign();
    test_set_transform_persists();
    test_removing_image_drops_assignments();
    test_assign_unknown_ids_fails();
}
