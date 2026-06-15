# Unit tests for the backend storage and vision layers. These need no GPU /
# camera and run headless in CI.
add_executable(avb_storage_tests
    TestRunner.cpp
    DataStoreTests.cpp
    TransformTests.cpp
    ImageTrackerTests.cpp
)
target_link_libraries(avb_storage_tests PRIVATE avb_storage avb_vision)

add_test(NAME avb_storage_tests COMMAND avb_storage_tests)
