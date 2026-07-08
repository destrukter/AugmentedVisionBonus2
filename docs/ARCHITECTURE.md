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
|  (ImGui)      | | (ImGui+gizmo)  |         |  (ImGui+OGRE)   |        |
| upload+validate| | drag handles / |         | compose feed +  |        |
| + assign      | | numeric, Save  |         | tracked models  |        |
+-------+-------+ +-------^--------+         +--------+--------+        |
        |                 |                           | consumes        |
        | onConfigure(id) |               +-----------+-----------+     |
        +-----------------+               |   background threads  |     |
                                          | CaptureWorker (camera)|     |
                                          | TrackingWorker (ORB + |-----+
                                          |  DetectionFilter)     | OpenCV
                                          +-----------------------+
                                          | SceneRenderer (main   | OGRE+Assimp
                                          |  thread, GL)          |
                                          +-----------------------+
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
  - `imageRevision()` is a monotonic counter bumped on any image-set change;
    the Camera window uses it to know exactly when to rebuild tracker targets
    (comparing counts would miss a remove+add between two frames).

- `AssetLibrary` — loads the default asset folder (`assets/library`, or
  `AVB_LIBRARY_DIR`) into the store at startup: images and FBX models are
  validated like manual uploads, and assignments are resolved by file name -
  explicit `model.fbx = image.png` pairs from `assignments.cfg` plus automatic
  pairing of files sharing a base name (`dragon.fbx` + `dragon.png`). Pair
  lines carry optional pose columns (`| t=x,y,z r=x,y,z s=v`, each part
  defaulting to identity when omitted); `persistAssignment()` writes a saved
  pose back into the cfg surgically (other lines and comments are preserved),
  which the Configure window's Save triggers for library assets - poses
  therefore survive restarts. The FBX check is injected as a callback so the
  storage layer stays free of render dependencies (the app passes
  `ModelLoader::validateModelFile`).

This layer is fully unit-tested in `tests/` and builds as the `avb_storage`
library with no UI dependencies, so CI can run it headless.

### `src/ui` — frontend (Dear ImGui, three separate OS windows)

- `Window` — base class; each window owns its own SDL2 window, GL context and
  ImGui context, so all three stay open and interactive simultaneously.
- `UploadWindow` — file import + assignment management; emits `onConfigure(id)`.
  Uploads are **validated eagerly** and the outcome is reported in the UI:
  - images must decode (otherwise the asset is removed again and an error is
    shown), and a warning appears when an image has too few ORB features to
    ever track reliably (`ImageTracker::countTrackableFeatures`);
  - models must pass `ModelLoader::validateModelFile` (an Assimp parse that
    requires at least one non-empty mesh) before they are added.
- `ConfigureWindow` — edits a working copy of an assignment's `Transform` and
  commits it on Save. Editing is interactive first: an ImGuizmo viewport shows
  the image plane (correct aspect ratio, axes) and the model's pose as a
  translate/rotate/scale gizmo; right-drag orbits, the wheel zooms. Numeric
  drag fields remain below for exact values, plus a collapsible matrix preview.
- `CameraWindow` — pairs the newest captured frame with the newest tracking
  result, drives the OGRE composite and displays it **letterboxed** (uniform
  scale, never stretched). Shows capture/tracking FPS, tracked-target count
  and a Reconnect button when no camera is available.

### `src/vision` — OpenCV (all UI-free, built as `avb_vision`)

- `CameraCapture` — wraps `cv::VideoCapture`; configures the device for low
  latency (MJPG, 720p@30, driver queue of 1 frame).
- `CaptureWorker` — **capture thread**. Continuously grabs frames and keeps
  only the newest (sequence-numbered) one, so consumers never see a backlog
  and the UI never blocks on the camera. Owns device lifecycle: retries while
  no camera is present and reopens on request.
- `ImageTracker` — registers uploaded images as templates and detects them in
  a frame (ORB features + Lowe ratio + RANSAC homography + planar IPPE PnP with
  LM refinement), returning `Detection{imageId, poseInCamera, corners,
  confidence}`. Robustness measures:
  - CLAHE contrast normalisation of templates *and* frames (lighting
    invariance);
  - a second detection pass with a permissive FAST threshold when a frame
    yields few features (dim scenes);
  - frame features are found on a downscaled copy (≤640 px) and mapped back to
    full-frame coordinates — several-fold cheaper with near-identical results;
  - templates are capped to 640 px so multi-megapixel uploads stay within
    ORB's scale-pyramid range;
  - homographies must map the template to a convex, plausibly sized quad or
    the sighting is rejected (kills the "jumping overlay" misdetections).
  All public methods are mutex-guarded so the UI thread can add/remove targets
  while the tracking thread detects.
- `DetectionFilter` — temporal smoothing: exponential lerp/slerp of poses and
  a ~250 ms hold for briefly lost targets, so models neither jitter nor blink.
  Time is injected, making it deterministic and unit-testable.
- `TrackingWorker` — **tracking thread**. Waits for the newest frame (skipping
  any it was too slow for), runs `ImageTracker::detect`, applies the
  `DetectionFilter` and publishes the result for the UI to consume.

### `src/render` — OGRE3D + Assimp (main thread only)

- `OgreContext` — owns `Ogre::Root`, loads the GL render-system plugin
  programmatically, creates a hidden 1x1 render window (so a GL context exists
  without an extra OS window), owns the scene manager, and initialises the RTSS
  shader generator (with a scheme-not-found resolver) so default materials
  render under the GL3+ shader-only pipeline.
- `ModelLoader` — imports FBX via Assimp and builds a cached `Ogre::Mesh` from an
  `Ogre::ManualObject` (positions/normals/UVs/indices), assigning a shared
  default lit material. OGRE has no native FBX importer. Also provides the
  static `validateModelFile()` used by the Upload window.
- `SceneRenderer` — renders the assigned models (over a transparent background)
  into an off-screen render texture, reads the RGBA result back to the CPU, and
  composites it over the camera frame. Notes:
  - every frame starts by re-binding OGRE's own GL context
    (`OgreContext::makeRenderContextCurrent`): the ImGui windows make their
    SDL contexts current in between, and OGRE only tracks context switches it
    performed itself - without the re-bind its rendering silently lands in
    whichever context happens to be bound;
  - the untracked-model preview uses `previewFramingPose`, which fits the
    model's (transformed) bounding sphere into the view with a slight tilt and
    turntable spin - a fixed pose showed huge models from inside and flat ones
    edge-on;
  - the CPU readback requests `PF_BYTE_RGBA` (byte-order R,G,B,A); the
    int-packed `PF_R8G8B8A8` layout put the alpha byte in the red channel;
  - the render target is **resized to the camera's native frame size** on the
    first delivered frame, keeping the OGRE projection aligned with the
    tracker's intrinsics and avoiding any per-frame feed resize;
  - the virtual camera's vertical FOV is `ImageTracker::kDefaultFovYDeg`, the
    same value the tracker assumes for pose estimation — the two must match or
    overlays drift near the frame edges;
  - when no model is visible in a frame, the OGRE pass and GPU read-back are
    skipped entirely and the feed is passed through;
  - compositing keys on the clear colour with vectorized OpenCV ops (masked
    copy) instead of a per-pixel loop.

#### Why CPU read-back?

OGRE owns its own GL context; the Camera window's texture lives in the SDL GL
context. Rather than set up shared GL contexts (fragile, driver-dependent), the
renderer reads the RTT back to a `cv::Mat` and the Camera window uploads that to
a GL texture it owns (`glTexSubImage2D`) for `ImGui::Image`. Camera frames are
already CPU-side from OpenCV, so this keeps the pipeline simple and robust.

### `src/core`

- `Application` — owns the backend, the worker threads and the three windows,
  wires the `onConfigure` callback, and runs the main loop until every window
  is closed. The loop is paced adaptively to ~60 fps (sleeping only the time
  each frame left over, instead of a fixed delay on top of the frame cost).

## Threading model

Three threads:

| Thread          | Work                                            | Rate            |
| --------------- | ----------------------------------------------- | --------------- |
| main (UI)       | SDL events, ImGui, OGRE render + composite      | ~60 fps         |
| CaptureWorker   | blocking camera reads, newest-frame slot        | camera (~30fps) |
| TrackingWorker  | ORB detect + homography/PnP + DetectionFilter   | as fast as able |

Hand-off points are small and lock-scoped: the newest frame (copied out under
a mutex), the newest filtered detection list, and the mutex-guarded target set
inside `ImageTracker`. The `DataStore` itself is only ever touched from the
main thread.

**Does the Camera window size affect tracking?** No. Detection always runs on
the raw camera frames from the capture thread; the window only receives the
finished composite, drawn letterboxed at uniform scale. Resizing the window
cannot warp what the tracker sees.

## Key data-flow scenarios

1. **Upload & assign**: Upload window validates the file (decode / Assimp
   parse), then calls `store->addImage/addModel` and reports success, warning
   (low-feature image) or error in the UI. `store->assign(model, image)`
   creates the pairing at identity pose.
2. **Configure**: clicking *Configure* calls `onConfigure(assignmentId)` →
   `ConfigureWindow::openAssignment`. Dragging the gizmo (or the numeric
   fields) changes a working copy; *Save* calls `store->setTransform`.
3. **Render**: each frame the Camera window takes the newest captured frame and
   the newest smoothed detections, and for each detected image looks up
   `store->assignmentsForImage(imageId)` and renders each model at
   `detection.poseInCamera * assignment.transform.toMatrix()`. When nothing is
   tracked but assignments exist, the window renders the first assigned model
   at a fixed preview pose so the 3D pipeline is visible; toggle this with the
   "Preview model when untracked" checkbox.

## Build notes

- System packages: OpenCV, Eigen3, Assimp, OGRE, SDL2 (`find_package`).
- Dear ImGui, ImGuizmo and nativefiledialog-extended are fetched at configure
  time via `FetchContent`.
- `AVB_BUILD_TESTS=ON` (default) builds and registers the storage + vision
  tests with CTest.
