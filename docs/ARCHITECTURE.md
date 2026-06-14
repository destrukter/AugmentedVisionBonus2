# Architecture

This document describes how AugmentedVisionBonus2 is structured and how data
flows between the three windows and the backend.

## High-level diagram

```
                         +-----------------------------+
                         |        DataStore            |
                         |  images / models /          |
                         |  assignments (+Transform)   |
                         +--------------+--------------+
                                        ^
                 reads/writes           |          reads
        +-----------------+-------------+-------------+-----------------+
        |                 |                           |                 |
+-------v-------+ +-------v--------+         +--------v--------+        |
| Upload window | | Configure win. |         |  Camera window  |        |
|  (ImGui)      | |   (ImGui)      |         |  (ImGui+OGRE)   |        |
| upload + assign| | edit pose,    |         | track + render  |        |
| + Configure btn| | Save          |         |                 |        |
+-------+-------+ +-------^--------+         +--------+--------+        |
        |                 |                           |                 |
        | onConfigure(id) |                           |  uses           |
        +-----------------+                  +--------+--------+         |
                                             | CameraCapture   | OpenCV  |
                                             | ImageTracker    |---------+
                                             | SceneRenderer   | OGRE+Assimp
                                             +-----------------+
```

## Layers

### `src/storage` — backend (no UI/GPU/camera dependencies)

The single source of truth, shared by all windows as a `std::shared_ptr`.

- `Types.h` — `Id` (uint64) handle type, `kInvalidId`.
- `Transform` — translation (Vec3) + rotation (Euler degrees) + scale; converts
  to a 4x4 Eigen matrix. **Translation and rotation default to zero** per spec.
- `Assets.h` — `ImageAsset`, `ModelAsset`, `Assignment`.
- `DataStore` — CRUD for images/models and the assignment graph:
  - `assign(model, image)` creates an assignment with an identity transform; the
    `(model, image)` pair is unique, so one model maps to many images via many
    assignments.
  - `unassign(...)` reverts; `reassignImage(...)` moves an assignment.
  - `setTransform(assignment, t)` is what the Configure window's Save calls.

This layer is fully unit-tested in `tests/` and builds as the `avb_storage`
library with no UI dependencies, so CI can run it headless.

### `src/ui` — frontend (Dear ImGui, three separate OS windows)

- `Window` — base class; each window owns its own SDL2 window, GL context and
  ImGui context, so all three stay open and interactive simultaneously.
- `UploadWindow` — file import + assignment management; emits `onConfigure(id)`.
- `ConfigureWindow` — edits a working copy of an assignment's `Transform` and
  commits it on Save.
- `CameraWindow` — drives the per-frame AR pipeline and displays the result.

### `src/render` — OGRE3D + Assimp

- `OgreContext` — owns `Ogre::Root` and the scene manager.
- `ModelLoader` — imports FBX via Assimp and builds `Ogre::Mesh` objects (OGRE
  has no native FBX importer).
- `SceneRenderer` — composites the camera feed with assigned models at their
  poses into a texture the Camera window shows via `ImGui::Image`.

### `src/vision` — OpenCV

- `CameraCapture` — wraps `cv::VideoCapture`.
- `ImageTracker` — registers uploaded images as templates and detects them in
  the feed (ORB features + homography + PnP), returning `Detection{imageId,
  poseInCamera}`.

### `src/core`

- `Application` — owns the backend + shared pipeline + the three windows, wires
  the `onConfigure` callback, and runs the single-threaded event/render loop
  until every window is closed.

## Key data-flow scenarios

1. **Upload & assign**: Upload window calls `store->addImage/addModel`, then
   `store->assign(model, image)`. The new assignment starts at identity pose.
2. **Configure**: clicking *Configure* calls `onConfigure(assignmentId)` →
   `ConfigureWindow::openAssignment`. Editing changes a working copy; *Save*
   calls `store->setTransform`.
3. **Render**: each frame the Camera window grabs a frame, asks `ImageTracker`
   which images are visible, and for each detected image looks up
   `store->assignmentsForImage(imageId)` and renders each model at
   `detection.poseInCamera * assignment.transform.toMatrix()`.

## Build notes

- System packages: OpenCV, Eigen3, Assimp, OGRE, SDL2 (`find_package`).
- Dear ImGui is fetched at configure time via `FetchContent`.
- `AVB_BUILD_TESTS=ON` (default) builds and registers the storage tests with
  CTest.
