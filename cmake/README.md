# cmake/

Helper CMake modules and `Find*.cmake` overrides go here. The directory is on
`CMAKE_MODULE_PATH` (see the top-level `CMakeLists.txt`).

Most dependencies (OpenCV, Eigen3, Assimp, OGRE, SDL2) ship their own package
config files, so no custom finders are required by default. Add modules here if
you need to support a platform whose package lacks a config file.
