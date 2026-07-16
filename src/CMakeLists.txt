# Backend storage is a standalone library so it can be unit-tested without any
# UI / rendering / vision dependency.
add_library(avb_storage
    storage/AssetLibrary.cpp
    storage/DataStore.cpp
    storage/Transform.cpp
)
target_include_directories(avb_storage PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(avb_storage
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
)
target_include_directories(avb_storage PUBLIC ${OpenCV_INCLUDE_DIRS})

# Vision (camera capture + image tracking) is a standalone library so
# ImageTracker can be unit-tested without any UI / rendering dependency.
add_library(avb_vision
    vision/CameraCapture.cpp
    vision/CaptureWorker.cpp
    vision/DetectionFilter.cpp
    vision/ImageTracker.cpp
    vision/TrackingWorker.cpp
)
target_include_directories(avb_vision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(avb_vision
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
)
target_include_directories(avb_vision PUBLIC ${OpenCV_INCLUDE_DIRS})

# Model import (FBX -> OGRE mesh/skeleton/animations) is a standalone library
# so it can be unit-tested headless: it only uses OGRE's CPU-side resource
# managers, which tests can bootstrap without a window or GPU.
add_library(avb_model
    render/ModelLoader.cpp
)
target_include_directories(avb_model PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
)
target_link_libraries(avb_model
    PUBLIC
        ${OpenCV_LIBS}
        assimp::assimp
        OgreMain
)

# Main application executable.
add_executable(AugmentedVisionBonus2
    main.cpp
    core/Application.cpp
    ui/Window.cpp
    ui/UploadWindow.cpp
    ui/ConfigureWindow.cpp
    ui/CameraWindow.cpp
    render/ConfigurePreview.cpp
    render/OgreContext.cpp
    render/SceneRenderer.cpp
)

target_include_directories(AugmentedVisionBonus2 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
)

# Hand the build-time OGRE media directory to the app so it can locate the RTSS
# shader library (RTShaderLib) without relying on OGRE_MEDIA_DIR being set at
# runtime. OGRE's CMake config exports OGRE_MEDIA_DIR.
if(OGRE_MEDIA_DIR)
    target_compile_definitions(AugmentedVisionBonus2 PRIVATE
        AVB_OGRE_MEDIA_DIR="${OGRE_MEDIA_DIR}")
endif()

# Record the source assets directory so the app can find the default asset
# library regardless of the working directory it is launched from (IDE, build
# folder, ...). Used as the last candidate after cwd/executable-relative paths.
target_compile_definitions(AugmentedVisionBonus2 PRIVATE
    AVB_SOURCE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

target_link_libraries(AugmentedVisionBonus2 PRIVATE
    avb_storage
    avb_vision
    avb_model
    Eigen3::Eigen
    ${OpenCV_LIBS}
    assimp::assimp
    OgreMain
    OgreRTShaderSystem
    SDL2::SDL2
    OpenGL::GL
    imgui
    imguizmo
    nfd
)
