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
                                          | TrackingWorker (ORB/LK|-----+
                                          |  + DetectionFilter)   | OpenCV
                                          +-----------------------+
                                          | SceneRenderer (main   | OGRE+Assimp
                                          |  thread, GL)          |
                                          +-----------------------+
```

## Layers

### `src/storage` — backend (no UI/GPU/camera dependencies)

The single source of truth, shared by all windows as a `std::shared_ptr`.

- `Types.h` — `Id` (uint64) handle type, `kInvalidId`.
- `Transform` — translation (Vec3) + rotation (Euler degrees) + per-axis scale; converts
  to a 4x4 Eigen matrix. **Translation and rotation default to zero** per spec.
- `Assets.h` — `ImageAsset`, `ModelAsset`, `Assignment`.
- `DataStore` — CRUD for images/models and the assignment graph:
  - `assign(model, image)` creates an assignment with an identity transform; the
    `(model, image)` pair is unique, so one model maps to many images via many
    assignments.
  - `unassign(...)` reverts; `reassignImage(...)` moves an assignment.
  - `setTransform(assignment, t)` is what the Configure window's Save calls.
  - `setOrigin(assignment, o)` stores the rigid base pose written by the
    Configure window's "Set origin here"; a model's full pose relative to its
    image is `origin * transform`.
  - `imageRevision()` is a monotonic counter bumped on any image-set change;
    the Camera window uses it to know exactly when to rebuild tracker targets
    (comparing counts would miss a remove+add between two frames).

- `AssetLibrary` — loads the default asset folder (`assets/library`, or
  `AVB_LIBRARY_DIR`) into the store at startup: images and FBX models are
  validated like manual uploads, and assignments are resolved by file name -
  explicit `model.fbx = image.png` pairs from `assignments.cfg` plus automatic
  pairing of files sharing a base name (`dragon.fbx` + `dragon.png`). Pair
  lines carry optional pose columns (`| t=x,y,z r=x,y,z s=v` or `s=x,y,z`,
  plus `ot=x,y,z or=x,y,z` for the pose's origin; each part defaulting to
  identity when omitted); `persistAssignment()` writes a saved
  pose back into the cfg surgically (other lines and comments are preserved),
  which the Configure window's Save triggers for library assets - poses
  therefore survive restarts. `saveSession()` (the Upload window's "Save
  session to library" button) fully syncs the library with the session:
  externally-uploaded files are copied in (store re-pointed at the copies),
  files of removed assets are moved to `<root>/removed/`, and the cfg is
  rewritten to exactly the current assignments - stale lines dropped and `!`
  exclusion lines emitted for reverted stem-matching pairs (load() honours
  them by skipping auto-pairing), so removals survive restarts too. The FBX
  check is injected as a callback so the storage layer stays free of render
  dependencies (the app passes `ModelLoader::validateModelFile`).

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
- `ConfigureWindow` — edits the poses of all models assigned to one image
  (opened per image from the Upload window). A dropdown selects which model
  the numeric fields, matrix preview and gizmo edit; each model keeps its own
  working copy (dirty ones marked `*`), so switching loses nothing, and Save
  commits every modified pose. The viewport shows the image and **all** its
  models (ConfigurePreview's off-screen render as the background) with the
  ImGuizmo translate/rotate/scale gizmo on the selected model, sharing one
  camera so the handles line up with the rendered pixels; right-drag orbits,
  the wheel zooms. "Set origin here" folds the selected model's current
  translation/rotation into the assignment's persistent origin (the model
  stays put, the editable values read zero; "fold back" undoes it). Falls
  back to a schematic plane + proxy cube when the render is unavailable.
- `CameraWindow` — pairs the newest captured frame with the newest tracking
  result, drives the OGRE composite and displays it **letterboxed** (uniform
  scale, never stretched). Shows capture/tracking FPS, tracked-target count
  a device-selector dropdown (switching reopens the camera on the capture
  thread) and a Reconnect button.

### `src/vision` — OpenCV (all UI-free, built as `avb_vision`)

- `CameraCapture` — wraps `cv::VideoCapture`; configures the device for low
  latency (MJPG, 720p@30, driver queue of 1 frame). `listDevices()` enumerates
  attached cameras (Linux: /dev/video* + sysfs names) for the Camera window's
  device selector.
- `CaptureWorker` — **capture thread**. Continuously grabs frames and keeps
  only the newest (sequence-numbered) one, so consumers never see a backlog
  and the UI never blocks on the camera. Owns device lifecycle: retries while
  no camera is present and reopens on request.
- `ImageTracker` — registers uploaded images as templates and tracks them
  per frame with a **detect-then-track** design, returning
  `Detection{imageId, poseInCamera, corners, confidence, viaOpticalFlow}`.
  Every registered target is handled independently, so any number of
  different images track simultaneously in one frame.
  - *Acquisition*: ORB features + Lowe ratio + RANSAC homography + planar
    IPPE PnP with LM refinement.
  - *Tracking*: the acquisition's inlier points are carried frame-to-frame
    with pyramidal Lucas-Kanade optical flow (forward-backward consistency
    checked); the pose is re-estimated from the flowed correspondences and
    RANSAC outliers are pruned from the tracked set so drift can't
    accumulate. No per-frame re-matching means no matching jitter, and flow
    keeps tracking through steeper tilt / greater distance than descriptor
    matching survives. ORB is skipped entirely on frames where every target
    is flow-tracked.
  - *Recovery*: when the surviving point set shrinks too far (occlusion,
    leaving the frame) or the pose turns implausible, the target falls back
    to ORB re-acquisition automatically.
  Robustness measures:
  - CLAHE contrast normalisation of templates *and* frames (lighting
    invariance);
  - a second detection pass with a permissive FAST threshold when a frame
    yields few features (dim scenes);
  - frame features and flow run on a downscaled copy (≤640 px) and map back to
    full-frame coordinates — several-fold cheaper with near-identical results;
  - templates are capped to 640 px so multi-megapixel uploads stay within
    ORB's scale-pyramid range;
  - homographies must map the template to a convex, plausibly sized quad or
    the sighting is rejected (kills the "jumping overlay" misdetections).
  Out-of-plane robustness: ORB descriptors are rotation- and scale-invariant
  but not perspective-invariant, so acquisition degrades with tilt; the
  guaranteed envelope (regression-tested) is 30 degrees, and ~40-45 degrees
  works in practice — after which optical flow extends the envelope while
  locked on. The recovered pose (IPPE PnP) reflects the tilt.
  All public methods are mutex-guarded so the UI thread can add/remove targets
  while the tracking thread detects.
- `DetectionFilter` — temporal smoothing: exponential lerp/slerp of poses and
  a ~250 ms hold for briefly lost targets, so models neither jitter nor blink.
  Time is injected, making it deterministic and unit-testable.
- `TrackingWorker` — **tracking thread**. Waits for the newest frame (skipping
  any it was too slow for), runs `ImageTracker::detect`, applies the
  `DetectionFilter` and publishes the result for the UI to consume.

### `src/render` — OGRE3D + Assimp (main thread only; `ModelLoader` is built as `avb_model` for headless testing)

- `OgreContext` — owns `Ogre::Root`, loads the GL render-system plugin
  programmatically, creates a hidden 1x1 render window (so a GL context exists
  without an extra OS window), owns the scene manager, and initialises the RTSS
  shader generator (with a scheme-not-found resolver) so default materials
  render under the GL3+ shader-only pipeline.
- `ConfigurePreview` — renders the Configure window's viewport content (the
  assignment's image as a textured plane plus its model at the working
  transform) into an off-screen target. Shares the scene manager with
  SceneRenderer; the two renders are isolated per viewport with visibility
  masks (`kMainSceneVisibilityMask` / `kConfigPreviewVisibilityMask`).
- `ModelLoader` — imports FBX via Assimp and builds a cached `Ogre::Mesh` from
  an `Ogre::ManualObject` (positions/normals/UVs/vertex colors/indices),
  carrying the file's materials along: per-submesh diffuse/specular/emissive
  colors, shininess, two-sidedness and diffuse textures - both embedded (FBX)
  and external image files (resolved next to the model), decoded through
  OpenCV so no OGRE codec plugin is needed. Submeshes without a usable
  material fall back to a shared default. OGRE has no native FBX importer.
  **Animations** come along too: when the file animates, the whole node
  hierarchy is mirrored into an `Ogre::Skeleton` (one bone per node, node
  transforms baked into the vertices as bind pose) and every `aiAnimation`
  becomes a skeletal animation (assimp keys replace a node's local transform;
  OGRE keyframes are offsets from the binding pose, so keys are rebased).
  Skinned meshes keep their per-vertex bone weights; meshes without weights
  bind rigidly (weight 1) to their node's bone, so plain node-transform
  animations play as well. Only OGRE's CPU-side resource managers are
  touched, so the import is unit-tested headless (`avb_model_tests`
  round-trips a generated animated FBX). Also provides the static
  `validateModelFile()` used by the Upload window.
- `SceneRenderer` — renders the assigned models (over a transparent background)
  into an off-screen render texture, reads the RGBA result back to the CPU, and
  composites it over the camera frame. Models are kept in per-model instance
  pools: every `drawModel` call in a frame places its own entity (sharing the
  cached mesh), so the same model assigned to two simultaneously tracked
  images renders at both poses. Each instance's first animation (when the
  mesh has one) is enabled, looping, and advanced by wall-clock time every
  frame the instance is drawn. Notes:
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
  - compositing keys on an exact chroma-key clear colour (1, 0, 255) with
    vectorized OpenCV ops (masked copy) - a colour lit geometry essentially
    never produces, so even pure-black materials composite correctly.

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
| TrackingWorker  | ORB acquire / LK flow + PnP + DetectionFilter   | as fast as able |

Hand-off points are small and lock-scoped: the newest frame (copied out under
a mutex), the newest filtered detection list, and the mutex-guarded target set
inside `ImageTracker`. The `DataStore` itself is only ever touched from the
main thread.

GL context discipline (all on the main thread): OGRE and SDL each cache which
GL context they believe is current and skip "redundant" switches, but neither
sees switches the other performs. Every OGRE render section therefore starts
with `OgreContext::makeRenderContextCurrent()` (a forced GLContext bind), and
every `Window` render pass re-binds its SDL context through a clear-then-bind
pair so SDL's cache can't turn the switch into a no-op.

**Does the Camera window size affect tracking?** No. Detection always runs on
the raw camera frames from the capture thread; the window only receives the
finished composite, drawn letterboxed at uniform scale. Resizing the window
cannot warp what the tracker sees.

## Key data-flow scenarios

1. **Upload & assign**: Upload window validates the file (decode / Assimp
   parse), then calls `store->addImage/addModel` and reports success, warning
   (low-feature image) or error in the UI. `store->assign(model, image)`
   creates the pairing at identity pose.
2. **Configure**: clicking a picture's *Configure* button calls
   `onConfigure(imageId)` → `ConfigureWindow::openImage`. The dropdown picks
   one of the image's models; dragging the gizmo (or the numeric fields)
   changes that model's working copy; *Save* calls `store->setTransform` +
   `store->setOrigin` for every modified model.
3. **Render**: each frame the Camera window takes the newest captured frame and
   the newest smoothed detections, and for each detected image looks up
   `store->assignmentsForImage(imageId)` and renders each model at
   `detection.poseInCamera * origin.toMatrix() * transform.toMatrix()`. When
   nothing is tracked but assignments exist, the window renders the first
   assigned model at a fixed preview pose so the 3D pipeline is visible;
   toggle this with the "Preview model when untracked" checkbox.

## Build notes

- System packages: OpenCV, Eigen3, Assimp, OGRE, SDL2 (`find_package`).
- Dear ImGui, ImGuizmo and nativefiledialog-extended are fetched at configure
  time via `FetchContent`.
- `AVB_BUILD_TESTS=ON` (default) builds and registers the storage + vision
  tests with CTest.
