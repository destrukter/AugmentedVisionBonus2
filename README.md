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
| Windowing / input  | SDL2               |
| Build              | CMake              |

## The three windows

The application opens **three independent OS windows at the same time**, all backed
by one shared in-memory data store (see `src/storage`).

1. **Upload window** (`src/ui/UploadWindow`)
   - Upload images and FBX models.
   - Assign an FBX model to an image. One model may be assigned to many images.
   - Revert (unassign) and re-assign freely.
   - For every model assigned to an image, a **Configure** button opens that pairing
     in the Configure window.

2. **Configure window** (`src/ui/ConfigureWindow`)
   - Edit the translation and rotation of an FBX model **relative to its image**
     through UI controls.
   - **Save** writes the pose back into the data store.

3. **Camera window** (`src/ui/CameraWindow`)
   - Streams the camera feed, tracks the uploaded images (OpenCV).
   - When a tracked image is detected, the FBX models assigned to it are rendered
     (OGRE3D) at their configured pose.

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
