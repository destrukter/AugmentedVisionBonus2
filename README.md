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
   - Assign an FBX model to an image. One model may be assigned to many images.
   - Revert (unassign) and re-assign freely.
   - For every model assigned to an image, a **Configure** button opens that pairing
     in the Configure window.

2. **Configure window** (`src/ui/ConfigureWindow`)
   - Edit the pose of an FBX model **relative to its image** interactively:
     a 3D viewport shows the image plane and a translate / rotate / scale
     gizmo (drag the handles; right-drag orbits the view, wheel zooms).
   - Numeric fields underneath give exact control over the same values.
   - **Save** writes the pose back into the data store.

3. **Camera window** (`src/ui/CameraWindow`)
   - Streams the camera feed and tracks the uploaded images (OpenCV ORB
     features; contrast-normalised so tracking survives lighting changes,
     temporally smoothed so poses don't jitter or flicker).
   - Capture and tracking run on background threads, so the feed stays smooth
     regardless of detection cost, and the newest frame is always shown.
   - When a tracked image is detected, the FBX models assigned to it are rendered
     (OGRE3D) at their configured pose. The feed is displayed letterboxed —
     resizing the window never warps the image or affects tracking.

## Default asset library (auto-upload at startup)

Files placed under `assets/library/` are uploaded automatically when the app
starts, with the same validation as manual uploads:

```
assets/library/
├── images/           # tracked images (*.png *.jpg *.jpeg *.bmp)
├── models/           # FBX models (*.fbx)
└── assignments.cfg   # optional model -> image pairs
```

Assignments between them are resolved **by file name**, two ways:

1. Explicit pairs in `assignments.cfg`, one per line (case-insensitive):
   `model-file.fbx = image-file.png`
2. Automatically by base name: `dragon.fbx` + `dragon.png` are paired without
   any config entry.

A pair line may carry optional pose columns after a `|`; each part can be
omitted and defaults to the identity pose (translation 0, rotation 0, scale 1):

```
model-file.fbx = image-file.png | t=0,0.5,0 r=0,90,0 s=2
```

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
./build/AugmentedVisionBonus2
```

Dependencies (OpenCV, OGRE, Eigen3, Assimp, SDL2) are resolved with `find_package`.
Dear ImGui is fetched automatically via CMake `FetchContent`. See
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for details.

The RTSS (auto-shader) pipeline needs OGRE's `RTShaderLib` media (e.g.
`OgreUnifiedShader.h`). The build records OGRE's media directory automatically;
if it lives somewhere unusual, point the app at it at runtime:

```bash
OGRE_MEDIA_DIR=/path/to/OGRE/Media ./build/AugmentedVisionBonus2
```
