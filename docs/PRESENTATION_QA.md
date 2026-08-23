# Final presentation — supervisor question drill

Questions a supervisor can reach for after the *Modular Image Tracking Software*
talk, answered from what the code actually does. Every constant quoted here is
the value in the source as committed — re-check them if you tune anything before
the presentation.

## Numbers to have on the tip of your tongue

| Value | Meaning | Constant |
| --- | --- | --- |
| 1000 | ORB features per image / frame | `kMaxFeatures` |
| 0.75 | Lowe ratio test threshold | `kLoweRatio` |
| 12 / 10 | Min good matches / RANSAC inliers | `kMinGoodMatches`, `kMinInliers` |
| 3.0 px | RANSAC reprojection tolerance (detection scale) | `kRansacReprojErr` |
| 640 px | Longest side, frames **and** templates | `kMaxDetectDim`, `kMaxTemplateDim` |
| 3.0 / 8x8 | CLAHE clip limit / tile grid | `createCLAHE` |
| 300 -> 7 | Feature floor -> fallback FAST threshold (default is 20) | `kMinFrameFeatures`, `kFallbackFastThreshold` |
| 300 px² | Minimum plausible quad area | `kMinQuadAreaPx` |
| 12 | Min surviving flow points before re-acquisition | `kMinFlowPoints` |
| 1.5 px | Forward-backward flow tolerance | `kFlowFbMaxErrPx` |
| 21 / 3 | LK window size / pyramid levels | `kFlowWinSize`, `kFlowPyramidLevels` |
| 0.4 | Pose smoothing weight of the new measurement | `DetectionFilter::Params::smoothing` |
| 250 ms | Hold before a lost target disappears | `holdMs` |
| 0.5 | Snap distance, in image widths | `snapDistance` |
| 45° | Assumed vertical FOV — tracker *and* renderer | `kDefaultFovYDeg` |
| 1.0 | World units across the tracked image plane | `kPlaneWidth` |
| 30° | Regression-tested tilt envelope (~40-45° in practice) | `ImageTrackerTests` |

## The eight questions that can actually hurt

These probe genuine weak points. Each has a defensible answer, but only if you
concede the limitation first and show you know the fix.

1. **"You assume a 45° field of view. What is your pose actually worth?"**
   Without calibration the focal length is a guess, so absolute depth is a guess
   with it, and unmodelled radial distortion bends the overlay near the frame
   edges. The system is *self-consistent* — the renderer uses the identical 45°,
   so overlays sit correctly — but it is not metrically accurate. That is why
   calibration is first on the future-work slide.

2. **"Your smoothing uses a fixed blend weight. What happens when the frame rate
   changes?"** A constant alpha = 0.4 per *frame* means the effective time
   constant moves with detection rate. The fix is one line:
   `alpha = 1 - exp(-dt/tau)`. Time is already passed into the filter explicitly,
   so the plumbing exists.

3. **"Doesn't Lucas-Kanade drift over a long track?"** Not the usual way. Points
   are never re-anchored to the previous frame's pose: each point keeps its
   original *template* coordinate for its whole life, so the pose is always
   solved template<->frame and per-frame flow error does not integrate into it.
   RANSAC prunes drifted points every frame; the forward-backward check kills
   them earlier.

4. **"What exactly is that confidence number?"** Two different things. In
   acquisition it is inliers / ratio-test matches; in flow it is surviving points
   / points at acquisition. Both are in [0,1] and both fall as tracking degrades,
   but they are not the same scale — it is a health indicator, not a probability.
   Unifying both on inlier ratio is the honest fix.

5. **"How does this scale to fifty images?"** Acquisition is brute-force Hamming
   matching, one pass per un-tracked target: O(targets x 1000 x 1000). Linear in
   the library. Mitigations already present: ORB runs once per frame and is
   shared across all targets needing acquisition, and is skipped entirely on
   frames where everything is flow-tracked. Real fix: an LSH / vocabulary-tree
   index over all templates.

6. **"Your flow point set only ever shrinks — then what?"** Correct: no new
   points are added during a track, so it thins until it drops below 12 and falls
   back to ORB. That is designed re-acquisition, not failure, but accuracy does
   decay near the end of a long track. Re-seeding fresh corners inside the
   tracked quad every N frames is the standard remedy.

7. **"Planar PnP has two solutions. Which one do you take?"** IPPE computes both
   and returns the lower-reprojection-error branch; LM refinement polishes it.
   Near fronto-parallel the two become nearly equally good, so the recovered tilt
   can flip between frames — a slight rocking that the smoothing masks. A proper
   fix keeps both hypotheses and picks by temporal consistency.

8. **"How do you know the tracking is good? Show me a number."** There are
   functional regression tests but no quantitative pose-accuracy benchmark. Say
   that plainly, then say what you would measure: RMS reprojection error against
   a printed marker at known distances, and pose jitter as the standard deviation
   of translation and rotation over a static 10-second capture.

## Architecture and the word "modular"

**Q1 — What does "modular" mean concretely?** Four layers with enforced
dependency direction: `storage` has no UI, GPU or camera dependencies and builds
as its own library; `vision` is UI-free; `render` is main-thread only; `ui`
depends on all three and none depend on it. The proof is the build — storage and
vision tests link and run headless in CI. Sharpest example: `AssetLibrary` needs
to validate model files (Assimp), so model validation is *injected as a callback*
(`ModelLoader::validateModelFile`) rather than letting storage depend on the
renderer.

**Q2 — Could you swap ORB for SIFT, or for AprilTags, without touching the UI?**
For a different feature type, yes: everything above `ImageTracker` consumes only
the `Detection` struct (image id, 4x4 camera-space pose, confidence, corners, and
a flag for which path produced it). Caveat to volunteer: `ImageTracker` is a
concrete class, not an abstract interface, so "swap" today means editing that
class. Extracting an `ITracker` interface is mechanical — the seam is in the
right place, it just isn't formalised.

**Q3 — Why three separate OS windows?** It was the specified interface, and it
maps to three independent tasks: content management, per-image authoring, live
view. Each `Window` owns its own SDL2 window, GL context and ImGui context, so
all three stay interactive at once — you can change a pose and watch it update in
the camera feed.

**Q4 — Walk me through your threading model.** Main thread (~60 fps, adaptively
paced): SDL events, ImGui, OGRE render, compositing. Capture thread: blocking
camera reads into a single newest-frame slot with a sequence number, so consumers
never see a backlog. Tracking thread: takes the newest frame (skipping any it was
too slow for), detects, filters, publishes. Three small lock-scoped hand-offs;
the `DataStore` is main-thread-only by design, so it needs no locking.

**Q5 — What latency does that introduce?** Poses can trail the displayed frame by
up to one detection interval, since the UI pairs the newest frame with the newest
detections. Acceptable because detection usually runs at or above capture rate
(flow-tracked frames skip ORB) and the smoothing blurs one interval below
noticeability. Blocking the render loop on detection would drop the feed to the
tracker's rate, which looks far worse.

**Q6 — Why CPU readback instead of GPU compositing?** OGRE owns its own GL
context; the camera window's texture lives in the SDL context, and sharing
contexts across those is fragile and driver-dependent. The overlay is rendered
off-screen, read back to a `cv::Mat`, composited with vectorized OpenCV ops, and
uploaded by the window to its own texture. Camera frames are already CPU-side.
Cost: a full-frame readback per rendered frame — which is why the OGRE pass and
readback are skipped entirely when no model is visible.

**Q7 — Why re-bind the GL context every frame?** OGRE and SDL each cache which
context they believe is current and skip "redundant" switches, but neither sees
the other's switches. ImGui windows make their SDL contexts current between OGRE
passes, so without a forced re-bind OGRE's rendering silently lands in whatever
context is bound. Every OGRE section now forces a bind; every window pass
re-binds through a clear-then-bind pair so SDL's cache cannot no-op it.

## Feature extraction

**Q8 — Why ORB rather than SIFT/SURF/AKAZE?** Binary descriptors matched by
Hamming distance (XOR + popcount) — roughly an order of magnitude cheaper than
SIFT's 128-float L2, and this runs every frame alongside rendering. ORB is
rotation-invariant (intensity centroid) and scale-invariant (image pyramid),
which covers what a hand-held camera does to a poster. Patent-free, in OpenCV
main. Trade accepted: SIFT is more robust to viewpoint change and would extend
the tilt envelope, at a frame-rate cost.

**Q9 — Why 1000 features?** Matching cost is quadratic in the product of the two
feature sets, so this is the biggest single lever on cost. Below a few hundred
you stop clearing the 12-match / 10-inlier floor on textured-but-not-rich images;
well above a thousand ORB returns progressively weaker corners that mostly
produce ambiguous matches the ratio test discards — paid for twice, little
returned.

**Q10 — Why CLAHE and not plain histogram equalization?** Global equalization is
driven by the whole-image histogram, so a lamp or a window skews the mapping for
every pixel and the poster in the dim corner stays flat. CLAHE equalizes in 8x8
tiles with bilinear interpolation between them. The clip limit of 3.0 is the
noise brake — without clipping, flat regions amplify into noise and ORB detects
hundreds of spurious corners. Key point: CLAHE is applied identically to
*templates and frames*; descriptors are only comparable if both sides had the
same preprocessing.

**Q11 — Why downscale frames to 640 px?** Detection cost is linear in pixel
count; 720p -> 640 px on the long side cuts it several-fold with barely any
quality change, since ORB's own pyramid would have found most of these corners at
a coarser level anyway. All coordinates map back to full-frame pixels before
homography and PnP, and the RANSAC tolerance is scaled by the same factor.

**Q12 — Why cap the *template* at 640 too?** The subtlest fix in the pipeline. A
4000 px upload viewed from across a room occupies ~200 px in the frame — a 20x
scale ratio. ORB's pyramid spans ~8 levels at 1.2x, roughly 3.6x. The template's
features simply do not exist at the frame's scale and you get *zero* matches from
a perfectly good image. Capping both sides puts the pair inside ORB's scale
range.

**Q13 — What is the fallback FAST threshold for?** Under 300 keypoints, the frame
is re-run once with the FAST threshold dropped from 20 to 7 — a dim scene has
real corners with shallow intensity steps. A retry rather than a permanent
setting, because 7 on a normally lit frame floods the detector with noise-driven
corners. The threshold is restored immediately.

**Q14 — What makes a good target image?** High-frequency, aperiodic texture. Bad:
flat colour, smooth gradients, and *repeating patterns* — every feature has a
near-identical twin, so the ratio test rejects almost everything. Handled at
upload: `countTrackableFeatures` runs the same preprocessing chain and warns
below roughly 50 features; an image with no descriptors is never registered.

**Q15 — You throw away colour.** Deliberate. ORB is defined on intensity, and
three channels would triple detection cost for features that mostly coincide with
the luminance ones. Real failure case: two targets identical in structure and
different only in colour are indistinguishable — rare enough not to pay for.

## Matching, homography and pose

**Q16 — Walk me through acquisition.**
1. Brute-force Hamming kNN (k=2), template descriptors vs frame descriptors.
2. Lowe ratio test at 0.75.
3. Need >= 12 surviving matches, else abandon this target this frame.
4. RANSAC homography at 3 px; need >= 10 inliers.
5. Project the template's four corners through H, check the quad is finite,
   convex and >= 300 px².
6. Build coplanar 3D points on the image plane from the inliers; solve with
   planar IPPE PnP; refine with Levenberg-Marquardt.
7. Convert OpenCV's camera frame to OGRE's by negating the Y and Z rows.
8. Seed the flow tracker from the inlier points — from the next frame this target
   is tracked, not detected.

**Q17 — Why 0.75 for the ratio test?** Lowe's paper suggests 0.7-0.8. The
pipeline favours precision over recall: a false placement puts a model somewhere
absurd, far more damaging in a demo than a target taking an extra frame to
acquire. Strictness is affordable — RANSAC and the quad check sit downstream, and
once acquired, flow carries the target regardless.

**Q18 — Brute force? Why not FLANN with LSH?** At 1000x1000 binary descriptors,
brute-force Hamming is a cache- and SIMD-friendly popcount-bound loop. LSH is
approximate and its index build is paid per frame on the frame side, so it only
wins once the template side is large — many targets. That is exactly the point
where you would switch; it is on the list, not in the build.

**Q19 — Why homography first and then PnP?** Different jobs. The homography is
*outlier rejection*: RANSAC on an 8-DOF planar model is cheap, well-conditioned,
and needs no intrinsics. PnP is the *metric* stage: correspondences plus
intrinsics give a rotation and translation. Decomposing the homography directly
is more noise-sensitive and offers no refinement path; PnP on clean inliers with
LM polish is visibly steadier.

**Q20 — Why `SOLVEPNP_IPPE`?** The points are coplanar by construction (Z=0). The
default iterative solver and EPnP are general 3D solvers that degrade on planar
configurations; IPPE is built for this case, is closed-form, and handles the
planar two-solution ambiguity by returning the lower-reprojection-error branch.
The LM refinement afterwards is what noticeably reduces jitter at near-planar
angles.

**Q21 — What is the quad plausibility check for?** A concrete bug fixed in three
lines. A borderline match set can pass RANSAC and still map the template to a
bow-tie, a near-degenerate line or a microscopic quad — those were the "jumping
overlay" misdetections. Requiring finite, convex, >= 300 px² removes essentially
all of them; it is a geometric consistency check the inlier count cannot express.

**Q22 — How are the 3D object points constructed?** Each inlier's template pixel
is normalised to (u,v) in [0,1], then mapped onto a plane one world unit wide,
centred at the origin, in the XY plane: x = (u - 0.5), y = (0.5 - v) * aspect,
z = 0. So the image spans exactly 1.0 world units in width and its height follows
the aspect ratio. Every configured model transform is relative to that frame —
which is why translating a model by 1.0 moves it one image-width.

**Q23 — Where do your intrinsics come from?** Synthesised: f = (rows/2) /
tan(45°/2), principal point at the frame centre, zero distortion. A
`setCameraIntrinsics` entry point exists; nothing calls it yet. See danger item 1.

**Q24 — Why negate the Y and Z rows?** Handedness. OpenCV's camera frame is X
right, Y *down*, Z *into* the scene; OGRE/OpenGL is X right, Y up, Z out toward
the viewer. Negating rows 1 and 2 of the rotation and translation converts
between them. Getting it wrong renders the model mirrored and behind the camera.

**Q25 — What if two uploaded images are very similar?** Each target is matched
independently, so both can fire on the same physical picture and both sets of
models overlay. There is no cross-target competition. On a distinct library it
never comes up; the fix is a global assignment or winner-by-inlier-count
arbitration.

## Lucas-Kanade tracking

**Q26 — Why track at all?** Three wins, name all three. *Stability*: per-frame
matching returns a different inlier set every frame, so the pose wobbles even on a
static camera; flow carries the same points forward and that jitter source
disappears. *Cost*: on frames where every target is flow-tracked, ORB does not run
at all. *Envelope*: ORB descriptors are rotation- and scale-invariant but not
perspective-invariant, so matching fails at steep tilt, while LK only needs the
local patch to look similar between consecutive frames — which holds well past
the acquisition angle.

**Q27 — What assumptions does LK make?** Brightness constancy, spatial coherence
(hence the 21 px window), and small motion. The last breaks first, which is what
the pyramid is for: 3 levels estimate motion coarse-to-fine, extending tractable
displacement roughly eight-fold. It still breaks under motion blur, hard lighting
changes, or a specular highlight sweeping the target — all handled by falling back
to ORB rather than trying harder.

**Q28 — Explain the forward-backward check.** Flow every point forward, then flow
the results back. A point that genuinely tracked lands where it started; one that
latched onto an occluder edge, a highlight or the background does not. Points
returning more than 1.5 px from their origin are dropped. LK's own status flag
reports convergence, and a point can converge confidently onto the wrong thing —
without this check those points silently corrupt the pose. 1.5 px at detection
scale is strict relative to the 3 px RANSAC tolerance: cheaper to lose a good
point than keep a bad one, because RANSAC is the second line of defence.

**Q29 — When does a target fall back to detection?** Surviving flow points drop
below 12 (from the FB check, points leaving the frame, or LK failure); the pose
from flowed points is rejected (too few inliers or an implausible quad); or the
frame size changes, invalidating all stored point coordinates. Recovery is
automatic on the very next frame, since ORB runs whenever any target needs
acquisition.

**Q30 — Why re-run RANSAC on flowed points every frame?** It produces the pose
*and* identifies points that have wandered off the plane's motion model, which are
pruned immediately so a drifting point cannot gradually bend the pose. It is the
mechanism that keeps a long track honest.

**Q31 — Two images at once — does it cost twice as much?** Less. Targets are fully
independent: own descriptors, own flow set, own pose, and each can be in a
different state. The expensive shared stage, ORB on the frame, runs *once per
frame* and its keypoints are matched against every target needing acquisition.
There is no special case for two — the loop is over all registered targets.

**Q32 — What happens under partial occlusion?** Tracking degrades gracefully:
occluded points fail the FB check and are dropped, the rest still solve a valid
pose, and tracking continues until the count falls under 12. What it does *not* do
is occlude the rendered model — the hand passes behind it, because the renderer
has no depth information about the real scene. Distinguish the two meanings
explicitly; the second is the future-work item.

## Temporal smoothing

**Q33 — What does the filter do?** An exponential moving average over pose plus a
dropout hold. Position and confidence lerp toward the new measurement at
alpha = 0.4; rotation goes through a quaternion *slerp*. A target missing from a
frame keeps reporting its last smoothed pose for 250 ms before being dropped, so a
one- or two-frame miss no longer makes the model blink out.

**Q34 — Why slerp and not a matrix blend?** The element-wise average of two
rotation matrices is not a rotation matrix — it loses orthonormality and
introduces shear and scale, visibly distorting the model. Quaternion slerp plus
normalisation guarantees a valid rotation and interpolates along the shortest arc
at constant angular velocity.

**Q35 — What is the snap distance for?** Smoothing assumes the new measurement is
near the old one. If a target genuinely relocates, gliding across the scene looks
wrong and takes many frames. A jump beyond 0.5 world units (half an image width)
bypasses the blend and snaps — cleanly separating noise to suppress from real
motion to follow.

**Q36 — What does smoothing cost?** Lag. alpha = 0.4 has a time constant of
roughly two frames, so fast movement is followed a couple of frames late on top of
pipeline latency. Deliberate: visible jitter on a static target is more
objectionable than slight sluggishness during motion.

**Q37 — Why not a Kalman filter?** An EMA has no motion model, so it can only lag.
A constant-velocity Kalman filter would *predict* forward, cancelling both the
smoothing lag and the tracking thread's one-interval latency, and its covariance
would weight measurements by actual quality instead of a fixed alpha. Scoped out
as more machinery than the demo needed — but it is the right answer to "how would
you improve this".

**Q38 — Why is time passed in rather than read from a clock?** Testability. The
hold-and-expire behaviour is deterministic and unit-tested with synthetic
timestamps and no sleeping. Anything that reads a wall clock internally can only
be tested by waiting.

## Rendering and model placement

**Q39 — How is a model's final position computed?** One matrix product:
`detection.poseInCamera * assignment.transform.toMatrix()`. The tracker gives the
image plane's pose in camera space; the assignment carries the model's pose
relative to that plane. That factorisation is why a configured pose is independent
of where the picture is in the room.

**Q40 — What are the units in the Configure window?** Image widths. The tracked
plane is one world unit across, so 0.5 moves the model half an image-width and a
pose authored for a small photo behaves identically on a large poster. There is no
metric scale in the system — without calibration there could not be. The AR is
scale-relative to the target by design.

**Q41 — What is your Euler convention, and why Euler at all?** Intrinsic X then Y
then Z; the matrix is composed as Rz * Ry * Rx. Euler because that is what the UI
edits — nobody authors a quaternion by hand. `fromMatrix` is documented as
non-unique: it picks angles that reproduce the same rotation matrix, guaranteeing
`toMatrix(fromMatrix(m)) == m` even if the angles differ from what you typed. On
gimbal lock: yes, at +/-90° pitch the decomposition degenerates and the numeric
fields can jump; the stored rotation is still correct, only its Euler
*representation* is ambiguous. Storing a quaternion and showing Euler for display
only would remove the artefact.

**Q42 — How do you composite the render over the feed?** Chroma key. The off-screen
viewport is cleared to exactly (1, 0, 255) and the compositor copies every pixel
that is *not* that colour, with a vectorized masked copy. The odd value is
deliberate: lit geometry essentially never produces it exactly, so even pure-black
materials composite correctly. Expect the follow-up "why not alpha?" — keying is
simple and robust across the readback path; the cost is that anti-aliased
silhouette pixels blended toward the key colour are treated as opaque, a faint
edge fringe. Premultiplied-alpha compositing is the better answer and a small
change.

**Q43 — The same model on two tracked images at once?** Per-model instance pools.
Each `drawModel` call takes its own entity from the pool, all sharing one cached
mesh, so geometry is loaded once and placed at as many poses as there are
assignments. Unused instances are hidden rather than destroyed, so the pool
stabilises and there is no per-frame allocation.

**Q44 — Why write your own Assimp-to-OGRE importer?** OGRE has no native importer
for FBX or OBJ. The loader builds an `Ogre::ManualObject` from Assimp's positions,
normals, UVs, vertex colours and indices, and carries materials per submesh:
diffuse/specular/emissive colours, shininess, two-sidedness, and diffuse textures
both embedded (FBX) and external (an OBJ's .mtl). Textures are decoded through
OpenCV, which is already linked, so no OGRE codec plugin is needed.

**Q45 — How do FBX animations end up playing?** The whole Assimp node hierarchy is
mirrored into an `Ogre::Skeleton`, one bone per node, with node transforms baked
into the vertices as the bind pose. Each `aiAnimation` becomes an OGRE skeletal
animation, with a rebasing step: Assimp keys *replace* a node's local transform
while OGRE keyframes are *offsets from the binding pose*. Skinned meshes keep
their per-vertex weights; unskinned meshes bind rigidly with weight 1 to their own
node's bone, which is what makes plain node-transform animations play through the
same path. Only CPU-side resource managers are touched, so the importer is
unit-tested headless.

**Q46 — Does resizing the camera window affect tracking?** No. Detection always
runs on the raw frames from the capture thread; the window receives only the
finished composite and draws it letterboxed at uniform scale, never stretched and
never re-sampled into the tracker. The render target is sized to the camera's
native frame size on the first delivered frame, keeping the OGRE projection
aligned with the tracker's intrinsics.

## UI, storage and persistence

**Q47 — Why is an assignment its own object?** The relationship is many-to-many
*with its own state*. One model can sit on many images, and the same model can be
placed on the same image several times — each placement its own assignment with
its own transform. That is only expressible if the pose lives on the relationship.

**Q48 — How does a configured pose survive a restart?** Assignments live in
`assignments.cfg` as `model.fbx = image.png | t=... r=... s=...`, each pose column
optional and defaulting to identity. Configure's Save rewrites that line
*surgically*, preserving other lines and comments. "Save session to library" does
the fuller sync: externally-uploaded files are copied in and the store re-pointed
at the copies, removed assets' files are moved to `removed/`, and the cfg is
rewritten to exactly the current assignment set so reverts and removals persist.

**Q49 — Why a revision counter on the image set?** Comparing counts is wrong: a
remove plus an add between two frames leaves the count unchanged while the
tracker's targets are stale. A monotonic counter bumped on any image-set change
makes the rebuild condition exact.

**Q50 — How does the gizmo stay aligned with the rendered preview?** They share one
camera. `ConfigurePreview` renders the image plane and all its models off-screen,
and ImGuizmo is driven with the same view and projection matrices, so the handles
land on the pixels they manipulate. The two off-screen renders share a scene
manager and are isolated by visibility masks. If the render is unavailable the
window falls back to a schematic plane and proxy cube.

## Testing and validation

**Q51 — What is tested, and how do you test CV deterministically?** Storage is
fully unit-tested; vision is tested against synthetic markers from a seeded RNG,
so every run sees identical pixels. Coverage: detecting a marker pasted into a
plain canvas at a known position, rejecting a featureless target, detecting under
a hard contrast squash (dim-room case), preserving full-resolution coordinates
through the internal downscale, a 30° tilted marker, flow tracking a moving
marker, re-acquisition after loss, and two markers tracked simultaneously. The
model importer round-trips headless against a generated animated FBX and a static
OBJ. Detail worth volunteering: the downscale test needs *block* noise, because
per-pixel noise averages to flat grey when downscaled and has no features at
detection scale — real photographs do, and the test had to reproduce that.

**Q52 — What are you not testing?** End-to-end with a real camera, the renderer
(it needs a GL context), and anything quantitative about pose accuracy. Concede
it, then give the protocol: a printed target at measured distances and angles, RMS
reprojection error of the projected corners against hand-labelled ground truth,
and translation/rotation standard deviation over a static capture for jitter.

## Learnings and future work

**Q53 — "Architecture first" — what would you do differently?** Be specific.
Define the layer boundaries and the data contract before writing the first window:
the places where that discipline held (storage with no UI/GPU dependencies, the
injected model-validation callback) were easy to test and easy to change; the
places where it was retrofitted are where the pain was.

**Q54 — "Backend problems" — which ones?** Two concrete debugging stories: the two
GL-context caches invalidating each other silently, so OGRE rendered into whatever
context ImGui had left bound; and the readback pixel format, where int-packed
`PF_R8G8B8A8` put alpha in the red channel and byte-order `PF_BYTE_RGBA` was the
correct request. Same lesson both times — a layer that caches state you don't own
will lie to you.

**Q55 — "Utilize libraries" — where did you draw the line?** Libraries for the
solved problems (ORB, LK, RANSAC, PnP, mesh import, GUI); own code for what is
specific to this system: the acquisition-to-tracking state machine, the
plausibility checks, the temporal filter, the Assimp-to-OGRE animation rebasing,
the compositing path. OpenCV gives you a pose estimator but not a tracking policy,
and the policy is where the engineering is.

**Q56 — How would you implement camera calibration?** Standard Zhang: 10-20 views
of a checkerboard, `findChessboardCorners` with sub-pixel refinement,
`calibrateCamera` for intrinsics and distortion, stored per device.
`setCameraIntrinsics` is already the hook, and the renderer's FOV would derive
from the calibrated focal length instead of both sides sharing a hard-coded 45°.
What improves: absolute depth becomes meaningful, and edge-of-frame drift from
unmodelled radial distortion goes away.

**Q57 — How would you extend the tilt envelope?** The limit is that ORB
descriptors are not perspective-invariant. Ascending cost: register several
pre-warped views of each template so acquisition has a match at steep angles
(ASIFT-style, cheap and effective); switch to a learned descriptor such as
SuperPoint + LightGlue, far more viewpoint-robust but needs a GPU budget; or
attempt acquisition on a perspective-rectified crop once a rough plane hypothesis
exists. Note the *tracking* envelope is already much wider — the target is
specifically the lock-on angle.

**Q58 — Occlusion and lighting estimation, concretely?** Occlusion needs scene
depth: a depth sensor, monocular depth estimation, or a hand/arm segmentation mask
rendered into the depth buffer before the model so real geometry can win the depth
test. Lighting estimation would sample the frame around the tracked quad for a
dominant light direction and colour temperature and drive the OGRE light — the
plane pose already gives a known surface normal to reason from, a real advantage
over an arbitrary scene.

**Q59 — What breaks on a phone or headset?** Vision and storage are portable C++
and OpenCV. What does not travel: SDL2 desktop windowing and the three-window UI
model; the desktop GL render path (mobile is GL ES, and per-frame CPU readback is
far more expensive on a tile-based GPU); V4L2 camera enumeration; and on standalone
headsets, camera access is often restricted or unavailable to third-party apps
entirely. The realistic port is the tracker as a library with a thin platform layer
per target.

**Q60 — A Vuforia alternative — how far off are you?** The core loop is there:
image targets, pose estimation, stable tracking, authored model placement,
persistence — and the architecture would expose cleanly to Unity as a native
plugin (a C API taking a frame and returning pose matrices per target id). What
Vuforia has that this does not: per-device calibration profiles, cloud recognition
and large indexed target databases, model and cylinder targets, extended tracking
fused with IMU and SLAM, and years of device tuning. Position it as "a working,
transparent, dependency-free implementation of the core technique" rather than
claiming parity.

**Q61 — "Did you achieve what you wanted?"** Do not answer with a flat yes. Answer
against the given requirements — upload, configure, extract, detect, track,
render, position — confirm each is functional, then name the one thing you would
fix first (calibration) and the one you are most pleased with technically (the
detect-then-track split, and the measured reasons it was worth building). A candid
answer with a ranked "what next" reads as engineering judgement; an unqualified
yes invites them to find a counterexample in your own demo.

## Answering tactics

| Situation | What to do |
| --- | --- |
| Asked to justify a constant | Name the value, name what goes wrong in each direction, then how you landed on it. "Twelve, because below that RANSAC fits noise and above it we reject usable acquisitions" beats any amount of theory. |
| You hit a genuine limitation | Concede in one sentence, then spend three on the fix. Conceding first is what makes the fix credible. |
| You don't know | Say what you would measure to find out. Never invent a number. |
| The demo misbehaves live | Narrate it as pipeline state: "it's lost flow and is re-acquiring — you can see it come back within a frame." A failure you can explain in real time demonstrates understanding better than a clean run. |
| A question about a library's internals | Answer what it does and why you chose it. It is fair to treat a well-tested implementation as a black box with known properties — that is the point of using it. |
| Asked for a performance number | Measure both FPS counters on the actual demo machine before the talk and memorise them. The camera window already displays capture and tracking rates. |
