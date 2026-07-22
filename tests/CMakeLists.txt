# Unit tests for the backend storage layer. These need no GPU / camera and run
# headless in CI.
add_executable(avb_storage_tests
    TestRunner.cpp
    AssetLibraryTests.cpp
    DataStoreTests.cpp
    TransformTests.cpp
)
target_link_libraries(avb_storage_tests PRIVATE avb_storage)

add_test(NAME avb_storage_tests COMMAND avb_storage_tests)

# Unit tests for the vision layer (image tracking). Headless: exercised with
# synthetic in-memory images only, no camera/GPU required.
add_executable(avb_vision_tests
    VisionTestRunner.cpp
    ImageTrackerTests.cpp
    DetectionFilterTests.cpp
)
target_link_libraries(avb_vision_tests PRIVATE avb_vision)

add_test(NAME avb_vision_tests COMMAND avb_vision_tests)

# Unit tests for the model import (FBX -> OGRE mesh/skeleton/animations).
# Headless: they bootstrap OGRE's CPU-side resource managers only (no window,
# GL context or render system) and generate their model fixtures with Assimp's
# exporter, so they run in CI like the others.
add_executable(avb_model_tests
    ModelTestRunner.cpp
    ModelLoaderTests.cpp
)
target_link_libraries(avb_model_tests PRIVATE avb_model)

add_test(NAME avb_model_tests COMMAND avb_model_tests)
