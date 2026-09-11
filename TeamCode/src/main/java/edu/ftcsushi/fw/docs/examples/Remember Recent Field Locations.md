---
tags:
  - Advanced
---

# Remember recently seen ball locations

**Outcome:** explain why three sightings remain at fixed field locations while the robot turns,
then expire independently when they are not seen again. This is a read-only software example,
not a pickup routine or runnable FTC OpMode. No camera or robot is needed to read it.

**Before this page:** read [Locate a vision target](<../drive-vision/Vision Targets.md>) for
observations, source builders, and capture-time field projection; [Spatial Queries](<../drive-vision/Spatial Queries.md>)
for read-only geometry; and [Loop Structure](<../core-concepts/Loop Structure.md>) for managed
service ordering. These are required concepts; the later pickup example is an optional next step.

## Seen now, remembered, or expired

A camera can lose sight of a ball when the robot turns. **Remembering a location** means retaining
the last field position and the time of that sighting. It does not mean seeing through an obstacle,
proving that the ball stayed there, or predicting where a moving ball went.

In this synthetic example, the robot sees A at `(20, 0)`, B at `(20, 10)`, and C at `(40, 0)` inches.
The letters label this explanation; the camera has not identified individual balls. Every sighting
may remain eligible for at most `1.0` second:

![A and B are observed at 0.00 and 0.60 seconds, remembered at 1.10, and expired at 1.70. C is observed only at 0.00, remembered at 0.60, and expired at 1.10.](<../assets/diagrams/recent-field-locations.svg>)

In words: at `0.60 s`, a newer image refreshes A and B, but C retains its original position and
`0.00 s` sighting time. At `1.10 s`, a new empty image arrives: C is older than the bound and expires;
A and B are only `0.50 s` old and remain remembered. At `1.70 s`, they have expired too. No image
between those sample instants or continued physical presence of an unseen ball is asserted.

[`FieldTargetMemory`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/observation/FieldTargetMemory.html>)
owns this bounded collection. A **snapshot** is one immutable publication of
that owner's current entries and diagnostics. Entries can come from different images, so their
individual capture times matter; the snapshot's publication time is not another camera exposure.
Its entry count is a count of possible locations, not an independently verified ball count.

## Supply field locations, not the current robot pose for an old image

`fieldObjects` is an ordinary variable containing a source of **one image's located objects in
field coordinates**. It is created during setup with the existing
[`ObservationSources`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/observation/ObservationSources.html>)
projection API:

```java
Source<TargetObservations2d> fieldObjects = ObservationSources.inField(
        camera.floorObjects(), poseHistory.lookupSource());
```

`camera` is the fixed-mount webcam or Limelight owner configured in the prerequisite vision guide.
Its `floorObjects()` source supplies positions relative to the robot at image capture.
`poseHistory` is the existing localization owner's
[`PlanarPoseHistory`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/localization/PlanarPoseHistory.html>):
it answers where the robot
was **at that capture time**. The source's angle brackets name the returned frame type;
construction retains these inputs without polling them. A missing history lookup prevents field
admission; it never substitutes the robot's later pose.

For example, at field pose `(10, 0)` facing left by `Math.PI / 2` radians (90 degrees), an observed
point `0` inches forward and `10` inches right is still field point `(20, 0)`. Rotation changes the
robot-relative coordinates, not that field location. The supplied scenario authors these values
independently and uses the real projection to check them.

Use one fixed producer, target model, camera geometry, and field coordinate system per memory
owner. AprilTags with genuine IDs keep their separate tag path; this memory accepts anonymous,
positioned objects with valid capture-time field evidence. It does not merge cameras.

## Construct one memory, selection, and read-only query

The maintained
[`RecentFieldLocations`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/visionmemory/RecentFieldLocations.html>)
service privately owns these objects. Its constructor's exact memory/selection excerpt is:

```java
memory = FieldTargetMemory.fromFieldObjects(fieldObjects)
        .retainingForSec(1.0)
        .matchingWithinInches(2.0)
        .maxEntries(4);
selected = TargetSelections.fromRecentFieldLocations(memory.source())
        .choose(FieldTargetSelectionPolicies.nearestToRobot(localization, 0.20, 0.10));
ReferencePoint2d point = References.selectedFieldTargetPoint(selected);
```

The final `maxEntries(4)` constructs memory; there is no additional `build()`. These are explicit
synthetic fixture settings, **not physical defaults**. When adapting the example, change the
constructor's `retainingForSec`, `matchingWithinInches`, and `maxEntries` answers after reviewing
your robot's evidence. The `2.0 in` radius permits a nearby unambiguous sighting to refresh an entry;
the capacity bounds storage and matching work to four retained locations.

Selection chooses a useful entry; it does not advance memory. `memory.source()` is one stable
read-only view, so the selector cannot reset or stop its owner. It inherits the `1.0 s` usable
sighting bound. The nearest-to-robot policy reads an already-published localization pose, at most
`0.20 s` old and quality at least `0.10`. That quality is the pose producer's score, not a ball
confidence or pickup probability. A fresh pose cannot refresh the ball's last sighting.

The alternative [`FieldTargetSelectionPolicies.nearFieldPoint(x, y, radiusInches)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/sensing/observation/FieldTargetSelectionPolicies.html>) ranks locations
near an authored field point without requiring a current robot pose. It still cannot solve a
robot-relative answer without localization. For a stricter sighting-age policy, put
`.freshWithinSec(0.50)` before `choose(...)`; the bound must not exceed memory retention. These
policies do not rank intake sweeps, available inventory slots, or expected cluster yield.

The same selected point feeds translation and facing:

```java
query = SpatialQuery.builder()
        .translateTo(SpatialTargets.point(point))
        .andFaceTo(SpatialTargets.point(point))
        .solveWith(SpatialSolveSet.builder()
                .absolutePose(localization, 0.20, 0.10).build())
        .build();
```

This constructs a read-only query, not a drive command. The default control frame is the robot
center. `absolutePose(...)` uses the same authoritative localization owner and explicit pose gates
as ranking. A remembered field point cannot use `observedPoints()` or direct tag-relative solving:
its old robot-at-capture coordinates do not describe today's robot frame.

Selection, reference, and consumption remain separate, as with AprilTags. Here the reference's
selection details have kind `REMEMBERED_TARGET`; `rememberedTarget()` exposes the exact chosen
entry, snapshot, age limit, ranking metric/reason, and optional ranking pose. The query's solution
retains the last-sighting timestamp separately from the solving-pose timestamp. Its general
evidence time is the older required one, not automatically today's pose time.

## Advance once, then publish status

Register this service during `configure(program)`, **after** the already configured camera and
localization/history services:

```java
RecentFieldLocations recent = program.service(
        new RecentFieldLocations(fieldObjects, localization));
```

Registration saves an owner for later callbacks; it does not execute its update or launch a
thread. `RobotProgram` calls its `update(clock)` synchronously in the Services phase, in declaration
order. The existing upstream owner first updates localization and calls
`poseHistory.recordCurrent(clock)`; this service then updates memory and samples its private query;
the Presenters phase reads `recent.status()` without polling either one. Use the same authoritative
trajectory for history, nearest-to-robot ranking, and the query's absolute pose.

The status wraps the exact immutable memory/query publications. `status.geometry` is null before
an update and after reset/STOP; after an update, inspect `geometry.laneResult(0).hasTranslation()`
and `.hasFacing()` rather than assuming a selected location has a usable solve. A retained status
is historical evidence, not a current control decision. No drivetrain or intake is connected.

The scenario's history configuration is explicit: retain `2.0 s`, at most `8` pose samples, with
interpolation time/translation/yaw limits all `0.0`. Every fixture image has an exact recorded pose,
so the test does not invent movement between samples. Production interpolation and camera latency
need their own reviewed history settings; see the prerequisite history discussion.

### Reset before changing what coordinates mean

A **reset fence** rejects images captured at or before a reset boundary, even if the camera returns
them later. Before rebasing field coordinates, resetting history, or changing this camera's
configuration/pipeline, the robot owner first calls:

```java
recent.resetBeforeTransition(clock);
```

Then it performs the external transition. Invalidation belongs before the operation, including
one that might fail after partly changing hardware. The helper clears memory first, then the
locally owned selector and query; it does not perform the camera change or reset borrowed history.
The next eligible sighting must be strictly newer than the fence.

The service also compares the authoritative estimator's `trajectorySegmentId()` before sampling.
A changed segment means the estimator reported a coordinate discontinuity, so old memory is
invalidated first. Normal motion, accepted corrections, temporarily unavailable pose, or an empty
image do not themselves clear memory. Generic projected frames cannot reveal an out-of-band
camera change: all such changes must pass through the owning service's reset wiring. Construct
new owners for a different producer/field graph.

Complete transitions before downstream consumers sample. Resetting memory does not retroactively
rewrite a spatial result already returned earlier in the loop or stop another owner's drive Task.
An active behavior would need its own explicit withdrawal and local runtime reset. During managed
STOP, `RecentFieldLocations.stop()` permanently invalidates its memory and publishes empty status;
the camera/localization owners clean up their own resources separately. Repeated STOP and STOP
before START are harmless.

## What matching can and cannot establish

Memory compares field positions, not image appearance. It refreshes only a one-to-one nearby pair:
the old entry must have exactly one nearby new candidate, and that candidate exactly one nearby
old entry. This removes a short trail from small unambiguous position changes without pretending
to identify a physical ball.

Ambiguous candidates neither refresh nor create entries. Exact duplicate positions in one image
remain ambiguous; distinct nearby positions can still create separate initial entries, so an
initial cluster is not collapsed. A jump farther than the matching radius can create a new possible
location while the old one ages out. Crossing objects can remain ambiguous. There is no velocity
estimate, appearance matching, prediction, or guaranteed identity.

Unmatched entries keep their exact last-sighting position/time. Empty or unavailable images do not
prove a ball was removed; expiry still runs. A repeated or out-of-order capture cannot refresh a
location. A new capture is considered only once even if its field projection is unavailable; a
later history lookup does not re-admit that same image. At capacity, oldest last sightings are
evicted first, with stable owner-issued order breaking ties. Reset, STOP, expiry, and eviction
invalidate retained entry keys; a key labels one owner's location lifetime, not a target ID.

## Software checkpoint and next gate

**Question:** when the robot turns and later images omit targets, do the remembered field
locations retain their own sighting times and expire independently?

**Keep real:** the maintained service, its exact constructor settings, the real clock/history,
field projection, memory, selector, reference, and spatial query.
**Replace:** physical camera results with authored robot-relative observations, localization with
authored trajectory poses, and elapsed wall time with the supplied test clock. No ball physics or
sensor noise is modeled.
**Observe:** the scenario's entry counts `3 -> 3 -> 2 -> 0`, unchanged C at `(40, 0)`, A's solved
robot-relative point `(0, -10)` after turning, and C's original timestamp until expiry.
**Cannot conclude:** a ball remained still, the camera recognized a ball, or a pickup is safe.

**Read the causal chain:** publish pose and record history; inject the corresponding image; run
the service heartbeat; inspect its published status. At `0.60 s` only A/B get a newer timestamp.
The empty `1.10 s` frame therefore cannot keep C eligible.
**Proves:** the software respects field geometry, independent sighting ages, and the tested reset,
localization-loss, and STOP boundaries.
**Does not prove:** physical localization/camera accuracy, a useful radius or retention period,
ball persistence, collision clearance, capture, or improved match performance.
**Next gate:** review the software trace, then separately validate camera geometry and timing
against independently measured points on the adopting robot. This example enables no motion.

Exact maintained files, supplied as **Complete source**:

- [`RecentFieldLocations.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/visionmemory/RecentFieldLocations.java>)
- [`RecentFieldLocationsSoftwareScenarioTest.java`](<../../../../../../../test/java/edu/ftcsushi/robots/examples/visionmemory/RecentFieldLocationsSoftwareScenarioTest.java>)

Optional: run the supplied scenario from the repository root. Passing it tests this maintained
owner, not a separately adapted robot implementation.

=== "Windows"

    ```powershell
    .\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests 'edu.ftcsushi.robots.examples.visionmemory.RecentFieldLocationsSoftwareScenarioTest'
    ```

=== "macOS"

    ```bash
    ./gradlew --console=plain :TeamCode:testDebugUnitTest --tests 'edu.ftcsushi.robots.examples.visionmemory.RecentFieldLocationsSoftwareScenarioTest'
    ```

Next, compare [one bounded vision pickup](<One Bounded Vision Pickup.md>). That separate example
requires a fresh newer whole-image recheck before its final maneuver. Remembered entries cannot
satisfy that recheck, and arrival or an intake command still does not confirm capture.
