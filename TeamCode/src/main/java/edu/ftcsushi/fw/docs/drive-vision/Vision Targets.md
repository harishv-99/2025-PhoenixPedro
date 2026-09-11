---
tags:
  - Advanced
---

# Locate a vision target

**Outcome:** choose a useful target location without making robot strategy depend on camera pixels.
**Before this page:** [Read a switch](<../build/Read a Switch.md>) explains a source, and
[field-relative drive](<../examples/Field-relative Drive.md#what-up-means>) explains robot and
field coordinates. No image-processing knowledge is assumed. Reading does not require a camera.

## A colored patch is not yet a ball location

A camera sees a flat image. A **color blob** is a connected region of pixels that passed a color
test; a yellow shirt can pass the same test as a yellow ball. The region's position in the image
tells us a direction, not its distance from the robot.

To estimate distance, this capability follows that direction until it reaches an assumed height
above the floor. A **ray** is that direction starting at the camera. A **height plane** is an
imaginary flat surface at the chosen height. Their intersection gives an estimated target point.

This path uses a **fixed mount**: the camera stays in the same position and orientation relative
to the robot. The robot itself may drive and turn; the pose-history step below accounts for that
motion. A camera that turns with a turret does not have a fixed mount and is not supported by this
shared floor-object path. The height plane stays parallel to the robot's horizontal floor plane;
it does not describe a tilted surface.

![Side view: a camera 10 inches high sees along a downward ray that meets the modeled target-height plane at 2 inches; the plane intersection estimates a point, not a proven ball center.](<../assets/diagrams/floor-target-ray.svg>)

In the drawing, the camera is 10 inches above the floor and the modeled point is 2 inches high.
Changing either height changes the estimated distance. These are illustrative values, not a robot
profile. The webcam uses the detected box's center; Limelight uses its configured targeting point.
Neither is automatically the physical center of a sphere, particularly when partly hidden.

!!! info "New concept: camera calibration"

    Calibration describes how a camera maps directions to image positions, including lens bending
    called distortion. The webcam needs valid SDK calibration for its actual image size. Its
    measured mount describes the lens position and rotation on the robot. Missing calibration is
    unavailable evidence, not permission to guess a field of view.

[`FloorTargetModel`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/vision/FloorTargetModel.html>)
names the assumed target height. [`FloorTargetProjection`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/vision/FloorTargetProjection.html>)
does the geometry. A ray parallel to the plane, pointing away from it, or producing invalid
geometry is rejected. A valid intersection still does not establish that the object is reachable.
`atHeightInches(...)` has no physical range cap; use `withMaxRangeInches(...)` to add an explicit
camera-to-target line-of-sight limit established for your camera and target model.
For resting balls, validate the chosen aim point and height with real measurements at several
distances; do not silently label this estimate an exact ball center.

## One camera, separate borrowed views

[`FtcWebcamVisionLane`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/FtcWebcamVisionLane.html>)
and [`FtcLimelightVisionLane`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/FtcLimelightVisionLane.html>)
own physical cameras. A **capability view** lets another component read a particular kind of
result without acquiring or closing the camera. Configure `aprilTags`, `floorObjects`, or both
before constructing the owner; a null setting means that capability is absent. The mount is one
shared camera fact, not separate tag and ball guesses.

For a webcam, the essential construction is below. `measuredMount` is the measured camera pose;
`validatedAimPointHeightInches` is the height assumption you have checked for your chosen target.
They are deliberately not supplied as physical defaults here.

```java
FtcWebcamVisionLane.Config config = FtcWebcamVisionLane.Config.defaults();
config.cameraMount = measuredMount;
config.aprilTags = FtcWebcamVisionLane.AprilTagConfig.defaults();
config.floorObjects = FtcFloorObjectVision.Config.defaults();
config.floorObjects.targetModel = FloorTargetModel.atHeightInches(validatedAimPointHeightInches);
FtcWebcamVisionLane camera = new FtcWebcamVisionLane(hardwareMap, config);
AprilTagVision tags = camera.aprilTags();
Source<TargetObservations2d> objects = camera.floorObjects();
```

`new` constructs one owner during setup, not every loop.
The unchanged webcam defaults request hardware name `Webcam 1` and a `640 × 480` image. Set
`config.webcamName` and `config.cameraResolution` to the adopted device and calibrated image size.
[`AprilTagVision`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/AprilTagVision.html>)
is non-closeable. `Source<TargetObservations2d>` means “an object that returns one frame of located
targets when asked with `get(clock)`.” The angle brackets identify that returned value's type.
The robot's owning service reads the sources in the managed loop and closes `camera` during STOP;
selectors, guidance, and presenters borrow results and never close it. The
[camera ownership guide](<AprilTag Localization & Fixed Layouts.md>) supplies the complete managed
lifecycle and tester wiring. This excerpt alone is not a runnable OpMode.

[`FtcFloorObjectVision.Config`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/vision/FtcFloorObjectVision.Config.html>)
starts with a yellow software threshold: Y from 32–255, Cr from 128–170, and Cb from 0–120.
**YCrCb** separates brightness (Y) from two color-difference values (Cr and Cb); each interval is
the range of pixel values accepted. Change `minY`/`maxY`, `minCr`/`maxCr`, and `minCb`/`maxCb`
after checking your lighting. These defaults do not prove yellow-ball recognition. There is no
area filter by default (`minContourAreaPixels = 0`) and no blur (`blurSizePixels = 0`).
The default maximum frame age is `0.25` seconds and the maximum candidate count is `16`;
an oversized frame is unavailable, not silently reduced to one ball.

| Camera fact | Webcam | Limelight |
|---|---|---|
| Tags and color | Both configured processors can run together | One configured pipeline at a time |
| Color configuration | The configuration above supplies thresholds | Configure a color pipeline on the device |
| Switching | Owner enables/disables its fixed processor set | Owner explicitly calls `requestPipeline(index)` |
| Location input | Calibrated box-center ray | Per-candidate no-crosshair direction angles |

For Limelight, set `config.floorObjects.limelightPipelineIndex` to the actual color pipeline
(software default `1`), and `config.aprilTags.pipelineIndex` to the tag pipeline (default `0`).
`config.pipelineIndex` is the initially requested pipeline, default `0`. Setting a capability does
not request its pipeline. Reading `floorObjects()` never switches it. After an explicit switch,
wait for a fresh confirmed result; the inactive capability remains unavailable. Limelight's
no-crosshair angles are measured right/up-positive, and the boundary converts them to Sushi's
left/up convention. A malformed or missing candidate angle is not a zero-angle detection.

## Choose one useful location

An **observation** records what was seen at capture time. A **selection** chooses which observation
matters now; it does not assign a permanent identity to an unlabeled ball. The following complete
choice constructs a selector but does not yet sample the camera:

```java
TargetSelectionSource selected = TargetSelections.fromVisibleObjects(objects)
        .freshWithinSec(0.20)
        .choose(TargetSelectionPolicies.nearestToRobot());
```

The chained calls form a **builder**: answer how old the sighting may be, then how to choose.
`TargetSelectionPolicies.nearestToRobot()` creates a reusable selection rule; it does not read
the camera. `choose(...)` completes construction. `TargetSelectionSource` is a source whose
`get(clock)` returns a `TargetSelectionResult`: one choice from an actual captured frame.
It continuously reconsiders the candidates; anonymous objects do not have a held-identity mode.
Here `0.20` seconds is an explicit illustrative freshness limit, stricter than the camera's default
`0.25`. `nearestToRobot()` compares distance from the robot center **at capture**, not lens range
and not the robot's later position. The owning service samples once in its managed `update(clock)`:

```java
TargetSelectionResult result = selected.get(clock);
// Store this immutable result in the service's status for the presenter to read.
```

The presenter formats that stored result without sampling again: when `result.isUsable(clock)`,
`result.observation().forwardInches` and `.leftInches` supply the two robot-at-capture coordinates.
For a point 18 inches forward and 4 inches left, expect `18` and `4`. If the original capture
becomes too old, no selection is usable—even if the camera returns that same image again.
Unavailable data includes a reason. A confirmed empty frame is different: the camera delivered a
usable frame with no matching candidates. Neither case means a target at `(0, 0)`.

Other [`TargetSelectionPolicies`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/observation/TargetSelectionPolicies.html>)
choices replace only the rule passed to `choose(...)`. Each table entry is called on
`TargetSelectionPolicies`, just like `nearestToRobot()` above:

| Want to choose… | Policy |
|---|---|
| nearest to the intake's capture-time origin | `nearestToControlFrame(robotToIntakeFrame)` |
| nearest to a field location, inside a limit | `nearFieldPoint(fieldX, fieldY, radiusInches)` |
| nearest a requested capture-frame direction | `nearestBearingRad(bearingSource)` |
| a candidate with the most nearby candidates | `mostNeighborsWithinInches(radiusInches)` |
| the lowest robot-specific numerical cost | `lowestCost(costFunction)` |

Equal costs use deterministic geometric ordering. The neighbor count counts observations in one
frame, not verified balls or a prediction of intake yield. A new frame may select another ball.
Tag-ID selection remains [`TagSelections`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/vision/apriltag/TagSelections.html>):
its real IDs permit policies that anonymous color regions cannot honestly promise.

## Put the point on the field

If the robot moves between capture and reading, combining old image geometry with its current
pose puts the target in the wrong place. **Pose history** stores past robot poses, so field
conversion can ask where the robot was at the original capture time.

```java
Source<TargetObservations2d> fieldObjects = ObservationSources.inField(
        camera.floorObjects(), poseHistory.lookupSource());
TargetSelectionSource selected = TargetSelections.fromVisibleObjects(fieldObjects)
        .freshWithinSec(0.20)
        .choose(TargetSelectionPolicies.nearFieldPoint(48.0, 24.0, 12.0));
```

Here the illustrated field location is `(48, 24)` inches and the allowed radius is `12` inches.
Only candidates with valid field coordinates qualify. `poseHistory` is the owner's existing
[`PlanarPoseHistory`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/localization/PlanarPoseHistory.html>),
recorded after localization updates in the Services phase. The source only looks up history;
it does not update localization. A missing lookup leaves robot geometry usable but field geometry
unavailable, with the lookup reason retained. It never falls back to today's pose for yesterday's
image. See the [timestamped history example](<../examples/Timestamped Adaptive Collection.md>)
for complete owner wiring and the interpolation/reset limits.

Tags can enter the same geometric path through `ObservationSources.aprilTags(tags.tagSensor(),
camera.cameraMountConfig())`, followed by `inField(...)` if needed. Tag identity/orientation and
the separate fixed-tag localization rules remain intact; a detected colored patch never becomes
a localization landmark merely because it has a field position. Quality is unknown (`NaN`) when
the producer supplies no meaningful score; no fictitious confidence is added.

## Select a tag: observed or inferred

A tag has an ID, so an aiming attempt can keep the same target even when another candidate moves
closer to the image center. A **preview** is the candidate that would win now. A **held selection**
is the ID already chosen for the attempt. Keeping that ID does not mean the tag remains visible.

These alternatives use the same ranking and hold policy. `scoringIds` is the nonempty set chosen
for the alliance during setup; the IDs and field geometry belong to the adopting robot's config.
`attemptActive` is a Boolean source that stays true through the entire aim-and-shoot attempt.
The code constructs sources; the owning service samples the chosen source in its managed loop.

**Bearing** is a signed horizontal direction angle in radians from a frame's forward axis;
`smallestAbsCameraBearing()` ranks nearest camera-forward. For the pose-derived alternative,
first read the [AprilTag localization model](<AprilTag Localization & Fixed Layouts.md>).
`localization` is an `AbsolutePoseEstimator` already updated by its owning service, and
`fixedLayout` records trusted tag positions on the field. Update localization before selection;
the selector only reads its estimate and never advances it.

```java
TagSelectionSource visibleChoice = TagSelections
        .fromVisibleTags(tags.tagSensor(), camera.cameraMountConfig())
        .among(scoringIds).freshWithinSec(0.25)
        .choose(TagSelectionPolicies.smallestAbsCameraBearing())
        .holdWhile(attemptActive);

TagSelectionSource poseChoice = TagSelections
        .fromFieldPose(localization, fixedLayout, camera.cameraMountConfig())
        .among(scoringIds).freshWithinSec(0.50).minQuality(0.10)
        .choose(TagSelectionPolicies.smallestAbsCameraBearing())
        .holdWhile(attemptActive);
```

Choose **one** source for the behavior; there is no implicit switching between them.
Field-pose selection calculates where each fixed tag should be relative to the camera; it needs usable
position **and** heading, not heading alone. It can select a never-seen tag while the camera is
blocked. That is an inferred direction, not a report of what the camera sees.

The shown age limits are illustrative. Pose quality must be at least `0.10`; that producer score
is not a probability of successful aiming. The no-argument bearing policy has no angular cutoff.
Use `smallestAbsCameraBearing(Math.toRadians(30))` for an explicit inclusive 30-degree acquisition
limit, or `smallestAbsRobotBearing(...)` to rank from robot-forward instead. The latter gets its
mount from the source, not a second argument. Neither policy models occlusion or a full optical
field of view. Equal built-in metrics choose the lowest tag ID. `closestRange()` ranks 3D lens-to-tag
distance; `priorityOrder(...)` follows an authored ID preference.

For a complete attempt, keep selection enabled through aiming **and** feeding, then make its
release observable. If release and the next request happen between samples, the owning service
must explicitly call `selected.reset()` before starting the next attempt. That reset is local and
never resets the camera or localization. `holdUntilReset()` is the alternative
when the owner already has an explicit attempt reset boundary; `continuous()` is for live preview
or deliberately changing targets. Ordinary loss does not release a held ID; the explicit
`holdWhileReacquiringAfterLossSec(attemptActive, seconds)` or
`holdUntilResetReacquiringAfterLossSec(seconds)` alternative authorizes choosing again after the
selected tag has lacked usable evidence for the stated duration. These terminal calls return the
source directly: there is no further `build()` or second lifetime answer.

### Reuse the selected identity for aim, distance, and approach

Let `selected` be the one selected source above. A tag-relative **offset** describes the target
in the tag's axes, not the robot's axes. The same offset can apply to every candidate:

```java
ReferencePoint2d scoringPoint = References.relativeToSelectedTagPoint(selected, 6.0, -1.5);
ReferenceFrame2d approach = References.relativeToSelectedTagFrame(selected, 12.0, 0.0, Math.PI);
```

These illustrative inches/radians are not a field specification. When different tags need different
definitions, supply one map entry for every candidate ID:

```java
Map<Integer, References.TagPointOffset> points = new HashMap<>();
points.put(20, References.pointOffset(6.0, -1.5));
points.put(24, References.pointOffset(5.0, 2.0));
ReferencePoint2d scoringPoint = References.relativeToSelectedTagPoint(selected, points);

Map<Integer, References.TagFrameOffset> poses = new HashMap<>();
poses.put(20, References.frameOffset(12.0, 0.0, Math.PI));
poses.put(24, References.frameOffset(10.0, 2.0, Math.PI));
ReferenceFrame2d approach = References.relativeToSelectedTagFrame(selected, poses);
```

A `Map` pairs each ID with its definition. The factories copy and validate the entries, including
coverage of all candidate IDs. These examples assume exactly `{20, 24}`; use your configured set.
Offsets may describe different scoring destinations, or the same physical destination from several
tags. The author must verify that physical relationship; the framework does not infer it.

Pass `scoringPoint` to both a facing plan and a translation-only spatial query for distance.
Pass `References.framePoint(approach)` and the frame heading to a full approach plan. Both
`absolutePose(...)` and `relativeAprilTags(...)` can use these references. The reference carries
identity, not a cached camera answer: the chosen solve source must supply its own usable pose or
fresh selected-tag observation. For example, a selector using camera A cannot lend its observation
to a guidance solve explicitly configured with camera B.

Choose the distance convention deliberately: a translation solution's `frameDistanceInches()` is
planar distance from the configured control frame to the target point. It is not automatically
3D camera-to-tag-center distance. Use the same selected ID and evidence policy for aim and range;
do not build a second independently choosing selector for the shooter.

### Read status without inventing visibility

`TagSelectionResult.previewChoice` describes the current ranking winner. `selectionDecision`
retains the acquisition choice while held; `currentSelectedCandidate` supplies the selected ID's
current usable geometry or is null. Each candidate labels its evidence `OBSERVED` or `FIELD_POSE`
and retains its original evidence timestamp. Only `OBSERVED` can supply a real observation.

`hasFreshSelectedObservation` is therefore false for pose-derived selection.
`visibleCandidateIds` and `visibilityTimestamp` report only actual fresh camera evidence;
an unavailable visibility timestamp means **unknown**, not a confirmed empty image.
`ObservationSources.aprilTag(selected)` projects only current actual observations into generic
robot-at-capture geometry. It takes no second mount and never projects an old held decision or
an inferred field candidate as a sighting. `TagSelectionResult.forTagId(id)` is an authored
identity-only value, not a fabricated selection decision or camera observation.

## Software checkpoint and next action

The software checks ask whether a known image direction reaches the expected target location.
They keep the real calibration math and projection, but replace the camera with authored pixels,
calibration values, and mount geometry. Image-center and four-corner cases check the axes and
offsets; an off-center ray with a rolled camera checks rotation about its forward axis.
The expected positions are calculated independently of the production rotation code.
**Complete source:** [calibrated pixel checks](<../../../../../../../test/java/edu/ftcsushi/fw/ftc/vision/WebcamColorCalibrationTest.java>)
and [projection checks](<../../../../../../../test/java/edu/ftcsushi/fw/sensing/vision/FloorTargetProjectionTest.java>).

The [bounded pickup checkpoint](<../examples/One Bounded Vision Pickup.md#software-checkpoint-and-hardware-gate>)
also feeds a projected ray through real pose history, selection, and pickup policy. Authored pose
samples put the robot somewhere different when the image arrives. The expected field target and
approach stay tied to capture time; missing capture-time history prevents pickup instead of using
the current pose. **Complete source:** [pickup scenarios](<../../../../../../../test/java/edu/ftcsushi/robots/examples/visionpickup/VisionPickupSoftwareScenarioTest.java>).
These are supplied software experiments, not a lens, native color processor, or moving robot.
The existing [selection](<../../../../../../../test/java/edu/ftcsushi/fw/sensing/observation/TargetSelectionsTest.java>)
and [shared guidance](<../../../../../../../test/java/edu/ftcsushi/fw/spatial/ObservedTargetGuidanceTest.java>)
checks additionally cover stale/empty data, deterministic selection, and tag/object parity
(**Complete source**).
The [maintainer verification command](<../maintainers/Maintainer Notes.md#16-automated-framework-verification>)
runs them as part of the software suite.

Before physical use, separately verify stream resolution/calibration, mount signs, aim-point
height, color thresholds, latency, pipeline readiness, and location error against measured
distances. No pickup is physically enabled by this guide. Next, use the chosen point for
[shared aim and approach guidance](<Drive Guidance.md#use-an-observed-object-or-a-computed-approach>).
