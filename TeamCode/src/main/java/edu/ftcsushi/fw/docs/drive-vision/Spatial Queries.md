---
tags:
  - Advanced
---

# Spatial Queries

**Before this page:** understand [robot-relative drive](<../build/First Drive.md>) and the
[field-relative frame distinction](<../examples/Field-relative Drive.md#what-up-means>). This optional
reference answers a geometry question; it does not teach another motor-control path.

A **pose** combines position and facing direction. A **coordinate frame** names the origin and axes
used to express those numbers. A **spatial query** asks where a target is relative to the robot or
one of its tools. A **solve lane** is one way to obtain that answer, such as an estimated field pose
or a camera observation. These are alternative sources of evidence, not separate robot loops.

`SpatialQuery` is the shared framework layer for **field/robot geometry**. It answers a position or
facing question:

> Given a target, a controlled robot frame, and one or more solve lanes, what translation or facing relationship can each lane solve this loop?

It does **not** decide which lane wins, and it does **not** command a drivetrain or mechanism. That policy belongs to consumers such as Drive Guidance or Mechanism Target Planning.

## When to use `SpatialQuery`

Use `SpatialQuery` directly when you need raw geometry:

- compare live AprilTag solving against localization fallback
- run a simple PID on a facing error
- inspect translation/facing solutions for telemetry
- build a robot-specific mechanism planner that needs field-relative context

Do not use `SpatialQuery` when the target is already a plant-unit value. A lift preset such as
`1420 ticks` should go straight to a `ScalarTarget`; a periodic mechanism can wrap that command with
`PlantTargets.equivalentPositionsOf(...)`; advanced alternative sets use
`PlantTargets.plan(request)`.

## Main vocabulary

- `TranslationTarget2d`: the point a controlled frame should move toward.
- `FacingTarget2d`: the point or heading a controlled frame should face.
- `SpatialControlFrames`: robot-relative frame providers for translation and facing.
- `SpatialSolveLane`: one strategy for solving the relationship, such as absolute pose or live AprilTags.
- `SpatialQueryResult`: ordered per-lane results from one loop.
- `TranslationSolution`: solved target point in robot and controlled-frame coordinates, with the
  capture timestamp retaining the underlying measurement's clock and reset identity.
- `FacingSolution`: signed facing error in radians, with the same kind of measurement timestamp.

The key naming rule is:

```text
Query -> Result
Planner/Guidance -> Plan or Status
PlantTargetResolver -> PlantTargetResolution
```

`SpatialQuery` produces a `SpatialQueryResult`. `DriveGuidanceQuery` produces drive status, and a
resolver built with `PlantTargets.plan(request)` produces a `PlantTargetResolution` because it
applies policy and chooses a requested target.

## Builder shape

`SpatialQuery` follows the same guided-builder rule as Drive Guidance and Mechanism Target
Planning: answer the required conceptual questions explicitly, and do not expose `build()` before the
query has both a target and solve lanes.

The common path builds a runtime query directly. `SpatialSolveSet.builder()` is also staged: it asks for at least one solve lane before `build()` is visible, because an empty solve set cannot answer a spatial query.

```java
SpatialQuery query = SpatialQuery.builder()
        .faceTo(SpatialTargets.point(References.relativeToTagPoint(20, 6.0, -1.5)))
        .controlFrames(
                SpatialControlFrames.robotCenter()
                        .withFacingFrame(robotToShooterFrame)
        )
        .solveWith(
                SpatialSolveSet.builder()
                        .aprilTags(tagSensor, cameraMount, 0.25)
                        .absolutePose(globalPoseEstimator, 0.50, 0.10)
                        .build()
        )
        .fixedAprilTagLayout(tagLayout)
        .build();
```

The first question is the target relationship:

```java
SpatialQuery facingOnly = SpatialQuery.builder()
        .faceTo(SpatialTargets.fieldHeading(Math.PI))
        .solveWith(solveSet)
        .build();

SpatialQuery translationOnly = SpatialQuery.builder()
        .translateTo(SpatialTargets.fieldPoint(48.0, 24.0))
        .solveWith(solveSet)
        .build();

SpatialQuery both = SpatialQuery.builder()
        .translateTo(SpatialTargets.fieldPoint(48.0, 24.0))
        .andFaceTo(SpatialTargets.fieldHeading(Math.PI))
        .controlFrames(frames)
        .solveWith(solveSet)
        .build();
```

`controlFrames(...)` defaults to `SpatialControlFrames.robotCenter()`. `fixedAprilTagLayout(...)` is
optional and should be supplied only when a lane or target reference needs trusted field-tag
geometry.

Use `SpatialQuerySpec.builder()` only when you need a reusable immutable description and separate
runtime query instances:

```java
SpatialQuerySpec spec = SpatialQuerySpec.builder()
        .faceTo(SpatialTargets.fieldHeading(Math.PI))
        .controlFrames(SpatialControlFrames.robotCenter())
        .solveWith(solveSet)
        .build();

SpatialQuery driveSide = SpatialQuery.from(spec);
SpatialQuery telemetrySide = SpatialQuery.from(spec);
```

The staged builders and `SpatialQuery.from(spec)` are the public construction paths. Named
factories such as `References.relativeToTagPoint(...)`, `SpatialTargets.fieldHeading(...)`, and
`RobotFrames.rigid(...)` validate finite authored coordinates, offsets, and angles where
their meaning is known. Generic `Pose2d` and `Pose3d` values remain permissive because they also
carry runtime observations and derived math; do not use `NaN` or infinity as a configuration
sentinel.

Create one stateful `SpatialQuery` per independent consumer and share the immutable
`SpatialQuerySpec`. Each runtime owns its per-cycle result cache and explicit reset boundary. A
repeated read in one cycle returns the same completed result; the cache is committed only after all
frame and lane sampling succeeds.

Frame providers, solve lanes, sensors, estimators, and selected-tag policies supplied through the
spec remain borrowed dependencies. An optional fixed `TagLayout` is different: the completed spec
retains an immutable semantic snapshot, so later edits to a mutable `SimpleTagLayout` cannot change
trusted field facts inside an existing query. Build a new spec/runtime when the field layout should
change.

`SpatialQuery.reset()` clears only that runtime query's cached result. It does not reset the
borrowed collaborators, because they may also serve another query or a robot-owned targeting
service. Reset those collaborators only through their actual composition-root owner when that
owner's lifecycle requires it.

`LoopClock.reset(...)` is separate again: it advances the cycle identity, which prevents any
pre-reset result from being reused, but it does not call `SpatialQuery.reset()` or clear component
state.

## Control frame vs camera frame

### Observed points and computed approaches

An observed target and a desired robot destination answer different questions. After
[locating and selecting a target](<Vision Targets.md>), use
[`References.observedPoint(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/spatial/References.html>)
for the actual observed point. The stateless
[`ObservedTargetSpatialSolveLane`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/spatial/ObservedTargetSpatialSolveLane.html>)
solves it in robot coordinates at capture, without pretending the robot has been localized.
An absolute-pose lane can instead solve the same point in current robot coordinates when the
observation has a valid capture-time field projection. The camera mount is not a tool frame.

Use `References.approachFrame(source)` when the source returns an
[`ApproachResult2d`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/spatial/ApproachResult2d.html>):
a desired **robot-center field pose**, already accounting for the chosen tool offset and heading.
Translate to `References.framePoint(frame)` and face its heading using robot-center control frames.
Both tags and located objects can support this destination; tag-relative points/frames still
retain their specialized identity and orientation semantics.

Solutions retain `targetObservationTimestamp` separately from `robotPoseTimestamp`. Direct
observed-point solving has no robot-pose estimate, so the latter is unavailable. For a live
observed field target, the ordinary `timestamp` is the older required evidence: updating the robot
pose cannot make an old ball sighting fresh. A bounded committed approach retains the original
sighting for diagnostics but uses fresh robot-pose evidence to solve the frozen destination.
Commitment means “continue toward this resting-target destination until this deadline,” not
“the target is still visible.” A field-coordinate rebase must explicitly discard retained goals.

Quality is a lane-specific score, not the probability of collecting a ball. `NaN` means that no
score was supplied. `SpatialSolutionGate.defaults()` accepts unknown quality but still requires
a valid same-clock timestamp; it imposes no finite age cap. Set `maxAgeSec(...)` for a bounded age.
Calling `minQuality(...)` explicitly requires a known score—even `minQuality(0)` rejects unknown
quality. A pose lane's score describes its pose evidence, not a fabricated combined target score.

### The tool and sensor have different jobs

A **control frame** is attached to the part whose position or facing matters, such as a shooter's
exit. A **camera frame** is attached to the sensor that observed the target. They need not have the
same origin or direction. A frame transform states one frame's position and orientation relative
to another; it does not move either object.

![Top-down robot axes with a shooter frame 8 inches forward, 2 inches left, and 3 degrees left of robot-forward; a separate illustrative camera frame observes the target.](<../assets/diagrams/control-camera-frames.svg>)

The robot origin is the reference for `robotToShooterFrame`: the shooter is `8 in` forward and
`2 in` left, and its `+X` direction points `3°` left of robot-forward. The camera's drawn placement
is illustrative only; its measured mount configuration answers where the observation came from.
The shooter frame answers which part should face the target. Neither frame is an extra hardware owner.

For a shooter:

```java
Pose2d robotToShooterFrame = new Pose2d(8.0, 2.0, Math.toRadians(3.0));
CameraMountConfig robotToCamera = profile.vision.webcam.cameraMount;
```

`robotToShooterFrame` belongs in `SpatialControlFrames.withFacingFrame(...)`. `robotToCamera` belongs in `SpatialSolveSet.aprilTags(...)`.

For a turret-mounted camera, those frames are still separate:

```java
Pose2d robotToTurretToolZero = new Pose2d(7.0, 2.5, Math.toRadians(10.0));
TimeAwareSource<CameraMountConfig> turretCameraMount = turretCameraMountHistory;

SpatialSolveSet solveSet = SpatialSolveSet.builder()
        .aprilTags(turretCameraTags, turretCameraMount, 0.15)
        .absolutePose(globalPoseEstimator, 0.50, 0.10)
        .build();
```

The query controls the turret tool frame. The AprilTag lane uses the dynamic camera frame to understand what the camera saw.

## Timestamp-aware frames

Fast moving mechanisms need more than “current pose.” A camera frame may be old by the time the loop reads it. If a turret moved during that delay, the AprilTag lane should interpret the tag using the turret camera mount from the frame timestamp.

Sushi therefore supports `TimeAwareSource<T>` for dynamic frames and camera mounts. Its
historical lookup receives one `LoopTimestamp`, not a raw timestamp plus a reset epoch. Fixed frames
use `RobotFrames.rigid(...)` or `TimeAwareSources.fixed(...)`. Current-only dynamic frames can use
`RobotFrames.currentOnly(...)`, but moving sensors should eventually use a history-backed source.

Fixed-frame factories validate their authored pose immediately. A live `Source` or
`TimeAwareSource` is runtime evidence and is not sampled during construction. Its owner must publish
truthful finite poses; a non-finite dynamic frame does not currently have a general framework-wide
"unavailable" interpretation.

A source owner that receives age instead of an absolute Sushi timestamp anchors the measurement
once:

```java
LoopTimestamp frameTimestamp = clock.timestampSecondsAgo(frameAgeSec);
```

For AprilTags, the webcam or Limelight acquisition owner performs that anchoring once and
`AprilTagDetections.fromFrame(...)` attaches the one frame timestamp to every geometry-only tag
observation. A consumer that converts a selected tag into generic robot-relative geometry uses
`CameraMountLogic.robotObservation2d(observation, mount, clock)`; the conversion forwards the exact
timestamp and fails closed if it is unavailable, from a prior reset, or materially in the future.
It does not invent a new capture time.

That one value can be retained across a deliberate `LoopClock.reset()` without losing its origin,
but it becomes invalid as current-epoch evidence: `ageSec(clock)` returns `NaN` and
`isFresh(clock, ...)` returns false. Consumers never read or compare a clock epoch. They derive
current age only when needed:

```java
double ageSec = facing.timestamp().ageSec(clock);
boolean stillFresh = facing.timestamp().isFresh(clock, 0.20);
```

Rule of thumb:

> Anything derived from a delayed sensor frame should carry or derive a timestamp. Any moving frame used to interpret that sensor should be sampled at that timestamp when possible.

The optional [Timestamped adaptive collection](<../examples/Timestamped Adaptive Collection.md>)
case study uses `PlanarPoseHistory.lookupSource()` to query the authoritative field-to-robot pose at
the detector frame timestamp. Exact and bounded interpolated lookups therefore support a moving
robot without pretending that its current pose existed at exposure. A reset, unavailable sample,
eviction, out-of-range request, or excessive time/translation/yaw bracket produces a typed fallback
instead of current-pose substitution. That planar robot-pose history does not solve an articulated
camera mount: a moving turret or arm still needs its own timestamp-aware frame source.

## Selecting lane results

The base query returns every lane result. A **gate** accepts or rejects evidence using declared
limits; here those limits are age (seconds since capture) and quality (the producer's confidence
score, not a probability of success). A **selector** chooses among the accepted results in declared
priority order. Use selectors when you want priority behavior:

```java
SpatialSolutionGate gate = SpatialSolutionGate.builder()
        .maxAgeSec(0.20)
        .minQuality(0.45)
        .build();

SpatialQueryResult result = query.get(clock);
SpatialFacingSelection facing = SpatialQuerySelectors.firstValidFacing(result, gate);
```

The selector compares each measurement timestamp with the query result's one coherent
`sampleTimestamp`, so robot code does not pass a second time or epoch argument. A retained result is
a snapshot of that query cycle; call `query.get(clock)` again when you need a current decision.

The same selector/gate concepts should be used by Drive Guidance and mechanism target request builders so “valid enough” means the same thing across the framework.

## Bounded query consumer: coordinated turret

The optional
[`ReferenceCoordinatedShotService`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/targeting/ReferenceCoordinatedShotService.html>)
is a complete robot-owned translation-query consumer. It selects one gated translation, preserves
that solution's observation timestamp, and maps robot-forward/robot-left geometry into one observed
periodic-equivalent turret request plus flywheel and hood intent. The separate
[`ReferenceTurretMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/targeting/ReferenceTurretMechanism.html>)
owns the live planner, cable bounds, unavailable hold, PositionPlant, hardware update, and stop.

```java
ReferenceCoordinatedShotService shot = program.service(
        new ReferenceCoordinatedShotService(
                localization,
                ReferenceCoordinatedShotService.Config.defaults()));

program.output(new ReferenceTurretMechanism(
        hardwareMap,
        ReferenceTurretMechanism.Config.defaults(),
        shot));
```

This keeps field geometry and robot kinematics upstream of one final bounded Plant path. It does not
write motor power from the query loop, infer that planner selection means physical arrival, or hide
target-loss behavior in an imperative `else` branch. See
[`Mechanism Target Planning.md`](<Mechanism Target Planning.md>) for the request/planner boundary and
the [hardware-free Reference scenario](<../examples/Hardware-free Reference Scenarios.md>) for the
software evidence and explicit physical limits of this illustrative, uncalibrated example. Neither
owner is wired into the ordinary Reference robot, and no default establishes physical accuracy,
safety, cable limits, or a trustworthy turret zero.
