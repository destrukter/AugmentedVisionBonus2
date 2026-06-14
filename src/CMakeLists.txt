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
)

target_link_libraries(AugmentedVisionBonus2 PRIVATE
    avb_storage
    Eigen3::Eigen
    ${OpenCV_LIBS}
    assimp::assimp
    OgreMain
    SDL2::SDL2
    imgui
)
