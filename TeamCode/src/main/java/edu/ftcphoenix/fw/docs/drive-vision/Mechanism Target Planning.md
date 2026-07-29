# Mechanism Target Planning

`PlantTargets` is the framework's target-generation system for `Plant`s. It turns simple numbers,
command targets, queued pulses, behavior overlays, equivalent positions, and advanced candidate
requests into one requested Plant target each loop.

The Plant still owns low-level hardware/control work. Target planning answers:

```text
what target should this Plant request this loop?
```

The Plant answers:

```text
can this hardware safely apply that request, and is the mechanism at that target?
```

```text
controller / service / autonomous policy
    decides which behavior is active
        ↓
command target (optional ScalarTarget)
    the stable request that robot policy and PlantTasks may change
        ↓
PlantTargets graph
    exact target, overlay, equivalent-position transform, candidate planner, fallback/hold policy
        ↓
requested target
    one finite target value in the Plant's public units
        ↓
Plant target guards
    bounds, homing/reference state, interlocks, fallback targets, target rate limits
        ↓
applied target
    one safe target after bounds and guards, still in the Plant's public units
        ↓
actuator command
    the native position/velocity command or normalized regulated power sent to hardware
```

## Unit rule

Plant targets are unit-agnostic, but each Plant has one public coordinate. These must all use the
same public Plant units:

- `PlantTargetRequest` values
- request period, when an explicit period is supplied
- Plant measurement
- Plant legal target range
- Plant tolerance
- command target, when present
- requested target
- applied target

For a ticks-based turret, values may be ticks. For an inches-based extension, values may be inches.
For a servo claw built with `rangeMapsToNative(0.30, 0.80)`, robot code and target planning can use
logical `0.0..1.0` while the Plant maps those values to raw servo fractions internally.

## The one target rule

For anything intended to become a Plant target, use `PlantTargets`.

```text
ScalarSource
    A number stream. Useful as a primitive.

ScalarTarget
    A writable number stream. When it is the exact source or overlay base, it is the command target.

PlantTargetSource
    A Plant-aware source that resolves to the requested target for this Plant.

PlantTargets
    The factory/builder family that creates exact, overlay, equivalent, and planned sources.
```

The Plant builder accepts friendly forms and normalizes them internally:

```java
.targetedBy(ScalarTarget target)         // exact source and command target
.targetedBy(ScalarSource source)         // exact source; command-capable if it is a ScalarTarget
.targetedBy(PlantTargetSource source)    // full Plant-aware target graph
.targetedByCommand(initialTarget)        // builder-created command target and exact source
```

The final graph owns this relationship; the builder never asks robot code to register a second,
possibly disconnected target. An exact source carries a command target when its runtime object is a
`ScalarTarget`. An overlay carries only the command target from its base graph. `ScalarTarget`s used
as conditional layers never become the command target, because a task writing a layer does not own
that layer's activation and an overlay may contain several such layers.

Put the stable robot request in the base when tasks should be able to command a composed Plant:

```java
ScalarTarget armCommand = ScalarTarget.held(STOWED);

PlantTargetSource finalArmTarget = PlantTargets.overlay(armCommand)
        .add("autoStow", autoStowRequested, STOWED)
        .add("manual", manualActive, manualTarget)
        .build();

PositionPlant arm = FtcActuators.plant(hardwareMap)
        .motor("arm", Direction.FORWARD)
        .position()
        .deviceManagedWithDefaults()
        .linear()
            .bounded(0.0, 4200.0)
            .nativeUnits()
            .alreadyReferenced()
        .positionTolerance(20.0)
        .targetedBy(finalArmTarget)
        .build();
```

```java
Task raiseArm = PlantTasks.move(arm)
        .to(HIGH)
        .cancelTo(STOWED)
        .build();
```

This writes the graph-owned `armCommand`, then waits until that command path wins and the Plant is
physically at the target selected from it. If `autoStow` or `manual` wins the overlay—even with the
same numeric value—the task does not complete from the wrong behavior path. If the active move is
cancelled, it changes `armCommand` to the explicit Plant-unit `STOWED` request once.

The resulting vocabulary stays consistent across every Plant:

```text
command target → requested target → applied target → actuator command
```

The command target is optional. Constants, ordinary read-only sources, planners, measured holds,
and custom target graphs without a command base remain read-only to `PlantTasks`. `hasCommandTarget()`
reports that capability, while `commandTarget()` returns it for framework helpers and compact
testers. Ordinary robot services should usually retain their named `ScalarTarget` directly rather
than rediscovering it from the Plant.

## Exact targets

Use a `ScalarTarget` when robot code or tasks should write a persistent command target.

```java
ScalarTarget liftTarget = ScalarTarget.held(0.0);

PositionPlant lift = FtcActuators.plant(hardwareMap)
        .motor("lift", Direction.FORWARD)
        .position()
        .deviceManagedWithDefaults()
        .linear()
            .bounded(0.0, 4200.0)
            .nativeUnits()
            .needsReference("lift not homed")
        .positionTolerance(20.0)
        .targetedBy(liftTarget)
        .build();

liftTarget.set(BASKET_TICKS);
lift.update(clock);
```

When a target is controlled only through tasks, the builder can create the command target:

```java
PositionPlant lift = FtcActuators.plant(hardwareMap)
        .motor("lift", Direction.FORWARD)
        .position()
        .deviceManagedWithDefaults()
        .linear()
            .bounded(0.0, 4200.0)
            .nativeUnits()
            .needsReference("lift not homed")
        .positionTolerance(20.0)
        .targetedByCommand(0.0)
        .build();

Task raiseLift = PlantTasks.move(lift)
        .to(BASKET_TICKS)
        .cancelTo(0.0)
        .build();
```

Every feedback move must choose `.cancelTo(value)` or `.leaveTargetOnCancel()` immediately after
`.to(...)`. The latter deliberately leaves the move request in place, so motion may continue.
Neither option imperatively stops hardware: `cancelTo(...)` changes the command target, while
the final target still flows through overlays, bounds, references, and guards on the next Plant
update. An owner-level shutdown must coordinate every related request and behavior layer.

## Same command, equivalent periodic positions

Most mechanisms request one logical number. A lift position, a bounded turret angle, and a freely
rotating turret angle should therefore use the same robot-facing capability and Task:

```java
turretCommand.set(GOAL_ANGLE_DEG);

Task aim = PlantTasks.move(turret)
        .to(GOAL_ANGLE_DEG)
        .leaveTargetOnCancel()
        .timeout(1.0)
        .build();
```

Only mechanism realization changes. An exact, unwrapped turret uses:

```java
PlantTargetSource finalTurretTarget = PlantTargets.exact(turretCommand);
```

A turret that may use any whole-turn equivalent wraps the same command:

```java
PlantTargetSource finalTurretTarget =
        PlantTargets.equivalentPositionsOf(turretCommand)
                .nearestToMeasurement()
                .whenUnavailable().holdMeasuredTargetOnEntry(0.0);
```

If `GOAL_ANGLE_DEG` is `20` and the turret is near `350`, a 360-degree periodic Plant may request
physical position `380`. `PlantTasks.move(...)` still takes logical value `20`: it completes only
when the `turretCommand` path won and the Plant reports physical arrival at `380`. A same-valued
overlay, fallback, hold, clamp, bound, or guard cannot make the move report success.

Periodicity does not silently change exact commands. A Plant may have periodic topology while some
behavior deliberately requests an exact unwrapped turn; omit `equivalentPositionsOf(...)` for that
behavior. The focused transform always uses the consuming Plant's declared period and rejects a
family with no legal representative instead of clamping it to a non-equivalent answer.

The preference stage asks one real motion-policy question:

- `nearestToMeasurement()` minimizes travel.
- `preferIncreasing()` chooses the nearest legal representative at or above the measurement when
  one exists; `preferDecreasing()` is symmetric.
- `preferRangeCenter()` favors the center of a finite range and otherwise behaves like nearest.

Each path then requires `whenUnavailable()`. Its fallback and initial hold numbers are physical
Plant-unit safety answers, not alternate logical commands.

Apply overlays before the equivalent-position transform so every final logical winner receives the
same physical interpretation:

```java
PlantTargetSource logicalTurretTarget = PlantTargets.overlay(turretCommand)
        .add("vision", visionAimEnabled, visionAngleDeg)
        .add("stow", stowRequested, STOW_ANGLE_DEG)
        .build();

PlantTargetSource finalTurretTarget =
        PlantTargets.equivalentPositionsOf(logicalTurretTarget)
                .nearestToMeasurement()
                .whenUnavailable().holdMeasuredTargetOnEntry(0.0);
```

The overlay decides which logical behavior wins. The outer transform then chooses that winner's
physical equivalent. The base `turretCommand` remains the graph-owned Task command; conditional
layers do not become competing writers.

## Overlays: behavior priority in target space

Use `PlantTargets.overlay(...)` when several behaviors can influence the same Plant. The base target
must be total. Later enabled layers have higher priority.

```java
PlantTargetSource feederTarget = PlantTargets.overlay(0.0)
        .add("stage", stageRequested, 0.20)
        .add("feedPulse", feedPulseQueue.activeSource(), feedPulseQueue)
        .add("eject", ejectRequested, -1.0)
        .build();

Plant feeder = FtcActuators.plant(hardwareMap)
        .motor("feeder", Direction.FORWARD)
        .power()
        .targetedBy(feederTarget)
        .build();
```

The Boolean on each layer means “this behavior is requested.” Overlay resolution has two passes:

1. Sample every layer's activation gate exactly once for the loop. This keeps stateful gates current
   even while another layer has priority.
2. Starting with the last-added, highest-priority enabled layer, resolve target producers only until
   the result is determined.

An available target from either layer form wins. If an enabled `add(...)` layer is unavailable, the
overlay reports it as unavailable and does not consult lower-priority targets. Disabled and shadowed
target producers are not resolved, and the total base target is resolved only after every layer is
disabled or explicitly falls through. Robot code does not need to notify sources about selection
changes or reset them when priorities change; the overlay call above is the complete declaration.

Do not hide target validity inside the Boolean unless that is truly the behavior you want. It is
usually easier to debug when the layer is enabled and its target source reports why it used a fallback,
hold target, or explicit unavailable result.

When the desired behavior really is “try this layer, but keep the lower-priority target if it cannot
produce a value,” say that explicitly:

```java
PlantTargetSource turretTarget = PlantTargets.overlay(PlantTargets.holdMeasuredTargetOnEntry(0.0))
        .addIfAvailable("visionAim", visionAimRequested, visionAimPlanner)
        .add("manual", manualActive, manualAngleTarget)
        .build();
```

`addIfAvailable(...)` is intentionally named differently from `add(...)` because it changes the
meaning of an enabled-but-unavailable layer. Debug output records that the layer was enabled and fell
through to the next enabled lower-priority layer, or ultimately the base, so this is not a hidden
Boolean filter. Disabled and shadowed target producers still are not resolved.

## Target-plan diagnostics

Plants now report two different target diagnostics:

```java
plant.getTargetPlan();    // how PlantTargets selected the requested target
plant.getTargetStatus();  // how the Plant turned requested target into applied target
```

For example, a turret planner may report `PLANNED_CANDIDATE` with candidate `"slot-2-purple"`,
while the Plant status reports `RATE_LIMITED` because the applied target is still walking toward the
requested target. This separation keeps behavior target generation separate from hardware protection
while making telemetry easier to read.

When the focused periodic transform successfully resolves the selected logical intent, it reports
`EQUIVALENT_POSITION`. Its target is the selected physical representative; the logical command
remains in the graph-owned `ScalarTarget` rather than becoming a second public target field. If the
logical child produced a fallback or hold target, the transformed plan retains that fallback/hold
kind so diagnostics do not mislabel it as satisfied intent.

For a `PLANNED_CANDIDATE`, `selectedQuality()` reports the chosen candidate's quality. When that
candidate came from an observation, `selectedTimestamp()` retains its epoch-safe `LoopTimestamp`
and `selectedAgeSec()` reports the age derived when this plan was resolved. A timeless candidate
instead reports quality `1.0`, an unavailable timestamp, and `NaN` age. These are selection facts,
not a second hardware-arrival signal.

## Advanced planning: candidate sets and observation metadata

Use `PlantTargets.plan()` when one request contains several named candidates, relative targets,
explicit periods, observation freshness/quality, or an intentional clamp policy. For one ordinary
logical command with whole-turn equivalents, use `equivalentPositionsOf(...)` above; students do
not need a request or candidate object for that case. The advanced planner does not receive a Plant
object. During `plant.update(clock)`, the Plant supplies:

- feedback availability and measurement
- legal target range
- linear/periodic topology and period
- previous requested/applied targets

The advanced planner builder intentionally asks one required question at a time: `request(...)`, then one
candidate preference, then one unreachable-candidate policy, then `whenUnavailable()`. Optional
observation-age/quality tuning lives in `accept()...doneAccept()` after the required motion-semantics
choices have been made.

### Candidate requests

A tray service can convert robot-specific inventory into Plant-unit candidates. The planner chooses
the best reachable representative.

```java
Source<PlantTargetRequest> purpleToOutput = clock -> {
    ArrayList<PlantTargetCandidate> candidates = new ArrayList<>();

    for (int slot = 0; slot < 3; slot++) {
        if (inventory.colorAt(slot) == ArtifactColor.PURPLE) {
            double alignDeg = trayModel.alignSlotToOutputDegrees(slot);
            candidates.add(PlantTargetCandidate.equivalentPosition(
                    "slot-" + slot + "-purple",
                    alignDeg
            ));
        }
    }

    return candidates.isEmpty()
            ? PlantTargetRequest.none("no purple artifact")
            : PlantTargetRequest.oneOf(candidates);
};

PlantTargetSource trayTarget = PlantTargets.plan()
        .request(purpleToOutput)
        .nearestToMeasurement()
        .rejectUnreachable()
        .whenUnavailable().holdLastTarget(0.0);
```

The planner does not know what “purple” means. It only sees Plant-unit candidates. Notice that
`nearestToMeasurement()` and `rejectUnreachable()` are staged single-answer questions: after one
choice, the returned type exposes only the next question. There is no later-replacement model. Branches that may set several independent tuning values, such as
`accept()`, still end with an explicit `doneAccept()`.

### How periodic candidates are selected

A periodic candidate describes an entire family of equivalent positions. The focused transform and
advanced planner share the same fixed, bounded selector; work does not grow when a wider range or
smaller period contains more equivalents. This is an internal guarantee, so robot code does not set
a search window or candidate limit.

Selection follows these rules:

- Plant range bounds are inclusive.
- `nearestToMeasurement()` chooses the closest legal position. An exact tie between two equivalent
  positions in the same periodic family goes to the lower value.
- `preferIncreasing()` first chooses the closest legal target at or above the measurement across the
  complete request. It considers targets below the measurement only when none is legal on the
  preferred side. `preferDecreasing()` applies the same rule in the opposite direction.
- `preferRangeCenter()` uses the center when both range bounds are finite. On a one-sided or
  unbounded range, it behaves like `nearestToMeasurement()`. An exact tie within one periodic family
  goes to the lower value.
- If distinct request candidates are otherwise exactly tied, the candidate declared first wins.

### Current intent versus observations

Use the ordinary factories for current robot intent: presets, inventory choices, or other requests
that do not become stale with time. Use the explicitly named `observed...` factories when a target
was calculated from a camera, localization estimate, or another sampled observation. An observed
factory takes one quality value and one stable observation timestamp in the consuming `LoopClock`
timebase; it does not also ask robot code to supply a potentially contradictory age.

`PlantTargetRequest` provides the concise one-candidate form, while `PlantTargetCandidate` provides
the same families for `oneOf(...)` lists:

| Target meaning | Current/timeless factory | Observation-derived factory |
| --- | --- | --- |
| Absolute exact | `exact(...)` | `observedExact(...)` |
| Absolute, using the Plant's period | `equivalentPosition(...)` | `observedEquivalentPosition(...)` |
| Absolute, with an explicit period | `periodic(...)` | `observedPeriodic(...)` |
| Measurement-relative exact | `relative(...)` | `observedRelative(...)` |
| Measurement-relative, using the Plant's period | `relativeEquivalentPosition(...)` | `observedRelativeEquivalentPosition(...)` |
| Measurement-relative, with an explicit period | `relativePeriodic(...)` | `observedRelativePeriodic(...)` |

The reusable source or adapter that publishes an observation-derived target also owns timestamp
anchoring. If its upstream API reports an age, that owner calls
`clock.timestampSecondsAgo(sampledAgeSec)` once before publishing the immutable snapshot used for
target construction and retains that `LoopTimestamp`. A target-request source must not repeat the
conversion whenever a cached observation or request is sampled, because that would make it appear
fresh. This
responsibility stays at or before the target's source boundary; ordinary mechanism code only
forwards the snapshot's quality and timestamp.

The planner derives age at resolution. `.maxObservationAgeSec(...)` applies only to observed
candidates; timeless intent is not rejected by that gate. A non-finite or out-of-range quality, an
invalid timestamp, or a timestamp too far in the future makes that observed candidate unavailable.
The planner may still choose a later valid candidate, and if none remains, its existing
`whenUnavailable()` policy supplies the one declared fallback, hold, or unavailable result.

## `whenUnavailable()` versus overlay

These two tools are intentionally parallel but not interchangeable.

```text
PlantTargets.overlay(...)
    Which behavior layer wins?

whenUnavailable()
    If this selected source cannot produce a valid target, what explicit target should it emit?
```

For a smart planner used directly as the Plant target, choose a total unavailable policy:

```java
PlantTargetSource turretTarget = PlantTargets.plan()
        .request(autoAimRequest)
        .nearestToMeasurement()
        .rejectUnreachable()
        .whenUnavailable().holdMeasuredTargetOnEntry(0.0);
```

For a smart planner inside an overlay, you can still make the layer total:

```java
PlantTargetSource turretTarget = PlantTargets.overlay(PlantTargets.holdMeasuredTargetOnEntry(0.0))
        .add("autoAim", autoAimRequested,
                PlantTargets.plan()
                        .request(autoAimRequest)
                        .nearestToMeasurement()
                        .rejectUnreachable()
                        .whenUnavailable().holdLastTarget(0.0))
        .add("manual", manualActive, manualAngleTarget)
        .add("stow", stowRequested, 0.0)
        .build();
```

`holdMeasuredTargetOnEntry(...)` is a total source whenever it is resolved. It latches the current
measurement when the hold source is entered, then keeps emitting that captured value. It does not
mean “skip commanding the Plant this loop.” Inside an overlay, “entered” means consecutive
resolution cycles in which that hold source is actually resolved. If it is disabled or shadowed for
a complete resolution cycle, its next resolution is a new entry and captures the then-current
measurement. Robot code does not call a selection reset or lifecycle hook to make that happen.

## Spatial-derived requests

Spatial queries can still feed a mechanism target. The spatial solve chooses geometry; robot-specific
kinematics map that geometry into Plant units; `PlantTargets.plan()` resolves the Plant target.

```java
Source<PlantTargetRequest> turretFacingRequest = clock -> {
    SpatialQueryResult facing = turretFacingQuery.get(clock);
    if (!facing.hasSolution()) {
        return PlantTargetRequest.none(facing.reason());
    }

    double targetDeg = Math.toDegrees(facing.solution().facingErrorRad());
    return PlantTargetRequest.observedEquivalentPosition(
            facing.sourceId(),
            targetDeg,
            facing.quality(),
            facing.timestamp()
    );
};

PlantTargetSource turretTarget = PlantTargets.plan()
        .request(turretFacingRequest)
        .nearestToMeasurement()
        .rejectUnreachable()
        .accept().maxObservationAgeSec(0.20).minQuality(0.45).doneAccept()
        .whenUnavailable().holdMeasuredTargetOnEntry(0.0);
```

The `observedEquivalentPosition(...)` name makes the freshness contract visible at the call site,
while the source-owned snapshot keeps student code to one quality value and one timestamp.

The Plant reports physical arrival with `atTarget()` and literal physical
`atTarget(value)`. `atTarget(value)` never applies modulo arithmetic. Target planning reports
selection status with `PlantTargetPlan`; it does not claim the mechanism has physically arrived.
`PlantTasks.move(...)` combines the plan's logical-command evidence with physical arrival at the
selected requested target.

## Hardware guards are separate

Plant target generation is behavior policy. Plant target guards are hardware protection.

```java
PositionPlant lift = FtcActuators.plant(hardwareMap)
        .motor("lift", Direction.FORWARD)
        .position()
        .deviceManagedWithDefaults()
        .linear()
            .bounded(0.0, 4200.0)
            .nativeUnits()
            .needsReference("lift not homed")
        .positionTolerance(20.0)
        .targetGuards()
            .maxTargetRate(1200.0)
            .holdLastTargetUnless("wrist clear", wristClear)
            .doneTargetGuards()
        .targetedBy(liftTarget)
        .build();
```

`bounded(...)` defines the static legal target range. `targetGuards()` handles dynamic protection
such as interlocks, fallback targets, and maximum target rate. If behavior asks for an impossible
or temporarily unsafe target, telemetry can show both the requested target and the applied target.
A Plant with a fixed range rejects a configured fallback outside that range at build time and
rechecks the final dynamic-guard result before sending it to hardware. Direct power Plants use the
fixed normalized range `[-1.0, +1.0]` without adding a builder question.

## Calibration belongs next to the mechanism

Target planning assumes the mechanism coordinate is meaningful, or that the PositionPlant will
publish an invalid target range until it becomes meaningful. A turret or lift service owns
homing/indexing tasks and semantic goals.

```java
Task homeLift = PositionCalibrationTasks.search(lift)
        .withPower(-0.20)
        .until(bottomSwitch)
        .establishReferenceAt(0.0)
        .holdAfterReference(0.0)
        .failAfterSec(3.0)
        .build();
```

The planner should not decide when zero is trustworthy. Homing, indexing, manual zeroing, and
semantic presets belong in the robot mechanism/service layer.
