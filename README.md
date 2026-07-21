# AugmentedVisionBonus2

An augmented-reality desktop application that tracks uploaded images via a camera
feed and renders user-assigned 3D (FBX) models on top of them.

## Technology stack

| Concern            | Library            |
| ------------------ | ------------------ |
| Language           | C++17              |
| 3D rendering       | OGRE3D             |
| Math               | Eigen              |
| Model loading      | Assimp (FBX)       |
| Computer vision    | OpenCV             |
| GUI                | Dear ImGui         |
| Pose gizmo         | ImGuizmo           |
| Windowing / input  | SDL2               |
| Native file dialogs | nativefiledialog-extended |
| Build              | CMake              |

## The three windows

The application opens **three independent OS windows at the same time**, all backed
by one shared in-memory data store (see `src/storage`).

1. **Upload window** (`src/ui/UploadWindow`)
   - Upload images and FBX models. Uploads are validated immediately and the
     outcome (success / warning / error) is shown in the window — a broken
     image or model file is rejected with a reason, and images with too few
     trackable features get a warning.
   - Assign an FBX model to an image. One model may be assigned to many
     images - and to the same image several times: each click of Assign adds
     another independent copy with its own pose.
   - Revert (unassign) and re-assign freely.
   - Remove images/models with the `x` next to each entry (assignments to
     them are removed along).
   - **Save session to library**: a full sync — assets uploaded from outside
     the library are copied into `assets/library/`, every assignment is
     written with its pose to `assignments.cfg`, stale entries for reverted
     assignments are dropped, and files of removed assets are moved to
     `assets/library/removed/`. The next start restores exactly the saved
     session.
   - Each picture has one **Configure** button that opens the image (with all
     its assigned models) in the Configure window.

2. **Configure window** (`src/ui/ConfigureWindow`)
   - Edits the poses of the models assigned to one image **relative to that
     image**: the viewport shows the actual picture and **all** of its
     assigned models, rendered live, with translate / rotate / scale gizmo
     handles on the selected model (right-drag orbits the view, wheel zooms).
     Scale is per-axis (with a uniform handle at the gizmo center).
   - A **Model dropdown** switches which model is being edited; the numeric
     fields, matrix preview and gizmo follow the selection. Edits are kept
     per model, so switching never loses unsaved changes (marked `*`).
   - Numeric fields give exact control over the same values.
   - **Save** writes every modified pose back into the data store.

3. **Camera window** (`src/ui/CameraWindow`)
   - Streams the camera feed and tracks the uploaded images with a
     detect-then-track pipeline: OpenCV ORB feature matching finds a target
     once, then Lucas-Kanade optical flow carries its points from frame to
     frame (far steadier than re-matching every frame); when too many points
     are lost, ORB re-acquires the target automatically. Contrast-normalised
     so tracking survives lighting changes, temporally smoothed so poses
     don't jitter or flicker. Targets don't need to face the camera straight
     on: acquisition holds up to roughly 40 degrees of out-of-plane tilt
     (and any in-plane rotation), and optical flow keeps tracking through
     steeper angles and greater distances once locked on. Any number of
     different images can be tracked at the same time, each with its own
     models.
   - A dropdown selects the capture device (on Linux, enumerated from
     /dev/video* with driver names); Reconnect reopens it after replugging.
   - Capture and tracking run on background threads, so the feed stays smooth
     regardless of detection cost, and the newest frame is always shown.
   - When a tracked image is detected, the FBX models assigned to it are rendered
     (OGRE3D) at their configured pose — with their own materials: colors,
     shininess, vertex colors and diffuse textures (embedded in the FBX or
     referenced image files next to it). Animated FBX files play their
     animation (skeletal or plain node animation) while rendered. The feed is
     displayed letterboxed — resizing the window never warps the image or
     affects tracking.

## Default asset library (auto-upload at startup)

Files placed under `assets/library/` are uploaded automatically when the app
starts, with the same validation as manual uploads:

```
assets/library/
├── images/           # tracked images (*.png *.jpg *.jpeg *.bmp)
├── models/           # FBX models (*.fbx)
└── assignments.cfg   # optional model -> image pairs
```

Assignments between them are resolved **by file name** from explicit pairs in
`assignments.cfg`, one per line (case-insensitive):
`model-file.fbx = image-file.png`. Repeating a line places the same model on
the image several times (one copy per line, each with its own pose). There is
no automatic pairing: files sharing a base name (`dragon.fbx` + `dragon.png`)
are not assigned to each other unless the cfg says so.

A pair line may carry optional pose columns after a `|`; each part can be
omitted and defaults to the identity pose (translation 0, rotation 0, scale 1):

```
model-file.fbx = image-file.png | t=0,0.5,0 r=0,90,0 s=2
model-file.fbx = image-file.png | s=1,2,0.5
```

`s` takes one uniform value or per-axis `x,y,z`.

Poses saved in the Configure window are written back into these columns
automatically (for library assets), so configured poses **survive restarts**.

The load summary (and any validation warnings) appears in the Upload window
and the log. Point the app at a different folder with the `AVB_LIBRARY_DIR`
environment variable.

## Backend storage model

See `src/storage`. The store keeps:

- **Images** — uploaded image assets.
- **Models** — uploaded FBX model assets.
- **Assignments** — each links one model to one image and carries a `Transform`
  (translation + rotation) that **defaults to zero** (identity pose). One model can
  participate in many assignments (one per image).

## Repository layout

```
.
├── CMakeLists.txt          # Top-level build
├── cmake/                  # Helper CMake modules
├── assets/                 # OGRE config + runtime resources
├── docs/ARCHITECTURE.md    # Design notes & data flow
├── src/
│   ├── main.cpp            # Entry point
│   ├── core/               # Application: owns windows + main loop
│   ├── storage/            # Backend: DataStore, assets, assignments, Transform
│   ├── ui/                 # Frontend: Upload / Configure / Camera windows (ImGui)
│   ├── render/             # OGRE context, Assimp->OGRE model loading, scene render
│   └── vision/             # OpenCV camera capture + image tracking
└── tests/                  # Unit tests
```

## Building

```bash
cmake -S . -B build
cmake --build build -j
./build/src/AugmentedVisionBonus2
```

Dependencies (OpenCV, OGRE, Eigen3, Assimp, SDL2) are resolved with `find_package`.
Dear ImGui is fetched automatically via CMake `FetchContent`. See
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for details.

The RTSS (auto-shader) pipeline needs OGRE's `RTShaderLib` media (e.g.
`OgreUnifiedShader.h`). The build records OGRE's media directory automatically;
if it lives somewhere unusual, point the app at it at runtime:

```bash
OGRE_MEDIA_DIR=/path/to/OGRE/Media ./build/src/AugmentedVisionBonus2
```
