# Unit tests for the backend storage layer. These need no GPU / camera and run
# headless in CI.
add_executable(avb_storage_tests
    TestRunner.cpp
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
)
target_link_libraries(avb_vision_tests PRIVATE avb_vision)

add_test(NAME avb_vision_tests COMMAND avb_vision_tests)
