cmake_minimum_required(VERSION 3.16)

project(AugmentedVisionBonus2
    VERSION 0.1.0
    DESCRIPTION "AR app: track uploaded images and render assigned FBX models"
    LANGUAGES CXX)

# ----------------------------------------------------------------------------
# Global settings
# ----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type" FORCE)
endif()

# Export compile_commands.json for tooling / clangd.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------
# These are resolved from the system / package manager (vcpkg, conan, apt, ...).
find_package(OpenCV REQUIRED)        # camera capture + image tracking
find_package(Eigen3 REQUIRED)        # math (poses, matrices)
find_package(assimp REQUIRED)        # FBX model loading
find_package(OGRE REQUIRED)          # 3D rendering
find_package(SDL2 REQUIRED)          # OS windows + input + GL context

# Dear ImGui is header/source only and not always packaged: fetch it.
include(FetchContent)
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.90.9
)
FetchContent_MakeAvailable(imgui)

# Build a small static lib for ImGui core + the SDL2/OpenGL3 backends.
add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC SDL2::SDL2)

# ----------------------------------------------------------------------------
# Subdirectories
# ----------------------------------------------------------------------------
add_subdirectory(src)

# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
option(AVB_BUILD_TESTS "Build unit tests" ON)
if(AVB_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
