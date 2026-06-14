# Backend storage is a standalone library so it can be unit-tested without any
# UI / rendering / vision dependency.
add_library(avb_storage
    storage/Transform.cpp
    storage/DataStore.cpp
)
target_include_directories(avb_storage PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(avb_storage
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
)
target_include_directories(avb_storage PUBLIC ${OpenCV_INCLUDE_DIRS})

# Main application executable.
add_executable(AugmentedVisionBonus2
    main.cpp
    core/Application.cpp
    ui/Window.cpp
    ui/UploadWindow.cpp
    ui/ConfigureWindow.cpp
    ui/CameraWindow.cpp
    render/OgreContext.cpp
    render/ModelLoader.cpp
    render/SceneRenderer.cpp
    vision/CameraCapture.cpp
    vision/ImageTracker.cpp
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

target_link_libraries(AugmentedVisionBonus2 PRIVATE
    avb_storage
    Eigen3::Eigen
    ${OpenCV_LIBS}
    assimp::assimp
    OgreMain
    OgreRTShaderSystem
    SDL2::SDL2
    OpenGL::GL
    imgui
)
