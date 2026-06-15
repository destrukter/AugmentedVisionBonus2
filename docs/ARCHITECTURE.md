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

- `OgreContext` — owns `Ogre::Root`, loads the GL render-system plugin
  programmatically, creates a hidden 1x1 render window (so a GL context exists
  without an extra OS window), owns the scene manager, and initialises the RTSS
  shader generator (with a scheme-not-found resolver) so default materials
  render under the GL3+ shader-only pipeline.
- `ModelLoader` — imports FBX via Assimp and builds a cached `Ogre::Mesh` from an
  `Ogre::ManualObject` (positions/normals/UVs/indices), assigning a shared
  default lit material. OGRE has no native FBX importer.
- `SceneRenderer` — renders the assigned models (over a transparent background)
  into an off-screen render texture, reads the RGBA result back to the CPU, and
  alpha-composites it over the camera frame. This avoids sharing GL contexts
  between OGRE and the window's SDL/GL context.

#### Why CPU read-back?

OGRE owns its own GL context; the Camera window's texture lives in the SDL GL
context. Rather than set up shared GL contexts (fragile, driver-dependent), the
renderer reads the RTT back to a `cv::Mat` and the Camera window uploads that to
a GL texture it owns (`glTexSubImage2D`) for `ImGui::Image`. Camera frames are
already CPU-side from OpenCV, so this keeps the pipeline simple and robust.

### `src/vision` — OpenCV (built as the `avb_vision` library, unit-tested)

- `CameraCapture` — wraps `cv::VideoCapture`.
- `ImageTracker` — registers uploaded images as ORB templates and detects them
  in the feed: ORB features → BFMatcher + Lowe ratio test → `findHomography`
  (RANSAC) → `solvePnP` on the inliers against the marker plane, returning
  `Detection{imageId, poseInCamera, confidence}`. The pose is converted from
  OpenCV camera axes (+Y down, +Z forward) to OGRE axes (+Y up, +Z backward).
  If no calibration is supplied via `setCameraIntrinsics`, a default pinhole
  intrinsic (focal ≈ image width, centred principal point) is used. The marker
  plane spans one "marker width" unit, so pose translation and the configured
  `Transform` are in those units.

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
   `detection.poseInCamera * assignment.transform.toMatrix()`. When nothing is
   tracked yet (the tracker is still a stub) but assignments exist, the window
   renders the first assigned model at a fixed preview pose so the 3D pipeline is
   visible; toggle this with the "Preview model when untracked" checkbox.

## Build notes

- System packages: OpenCV, Eigen3, Assimp, OGRE, SDL2 (`find_package`).
- Dear ImGui is fetched at configure time via `FetchContent`.
- `AVB_BUILD_TESTS=ON` (default) builds and registers the storage + vision tests
  (`DataStore`, `Transform`, `ImageTracker`) with CTest. They are GPU/camera-free
  and run headless in CI.
