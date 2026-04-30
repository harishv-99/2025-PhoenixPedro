# Mechanism Target Planning

`PlantTargets` is the framework's target-generation system for `Plant`s. It turns simple numbers,
writable command targets, queued pulses, behavior overlays, and smart equivalent-target requests into
one requested Plant target each loop.

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
PlantTargets graph
    exact target, overlay, queued pulse, candidate planner, fallback/hold policy
        ↓
requested target
    one finite target value in the Plant's public units
        ↓
Plant target guards
    bounds, homing/reference state, interlocks, fallback targets, target rate limits
        ↓
applied target
    low-level output or regulation in the Plant's public units
```

## Unit rule

Plant targets are unit-agnostic, but each Plant has one public coordinate. These must all use the
same public Plant units:

- `PlantTargetRequest` values
- request period, when an explicit period is supplied
- Plant measurement
- Plant legal target range
- Plant tolerance
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
    A writable number stream. Useful for commands and tasks.

PlantTargetSource
    A Plant-aware source that resolves to the requested target for this Plant.

PlantTargets
    The factory/builder family that creates PlantTargetSource objects.
```

The Plant builder accepts friendly forms and normalizes them internally:

```java
.targetedBy(ScalarTarget target)                 // writable exact target; auto-registered
.targetedBy(ScalarSource source)         // read-only exact source
.targetedBy(PlantTargetSource source)            // full plant-aware target source
.targetedByDefaultWritable(initialTarget)        // builder-created writable exact target
```

If a read-only or composed source still has an underlying command variable that tasks should write,
register it explicitly:

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
            .writableTarget(armCommand)
        .build();
```

`PlantTasks.moveTo(arm, HIGH)` writes the registered `armCommand`, then waits for
`arm.atTarget(HIGH)`. If `autoStow` or `manual` wins the overlay, the task does not complete early,
because the Plant is not actually at the requested `HIGH` target.

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

When a target is only controlled through tasks, the builder can create the writable target:

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
        .targetedByDefaultWritable(0.0)
        .build();

Task raiseLift = PlantTasks.moveTo(lift, BASKET_TICKS);
```

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

The Boolean on each layer means “this behavior is requested.” If an enabled `add(...)` layer cannot
produce a target, the overlay reports that layer as unavailable instead of silently falling through.
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
through, so this is not a hidden Boolean filter.

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

## Smart planning: equivalent and candidate targets

Use `PlantTargets.plan()` when a request can be satisfied by more than one scalar value, or when the
request should be resolved using the consuming Plant's context. The planner does not receive a Plant
object. During `plant.update(clock)`, the Plant supplies:

- feedback availability and measurement
- legal target range
- linear/periodic topology and period
- previous requested/applied targets

The planner builder intentionally asks one required question at a time: `request(...)`, then one
candidate preference, then one unreachable-candidate policy, then `whenUnavailable()`. Optional
request-age/quality tuning lives in `accept()...doneAccept()` after the required motion-semantics
choices have been made.

A free spinner or tray can declare its own period in Plant units:

```java
PositionPlant tray = FtcActuators.plant(hardwareMap)
        .motor("tray", Direction.FORWARD)
        .position()
        .deviceManagedWithDefaults()
        .periodic(360.0)
            .unbounded()
            .scaleToNative(TRAY_TICKS_PER_DEGREE)
            .needsReference("tray index mark not found")
        .positionTolerance(2.0)
        .targetedBy(
                PlantTargets.plan()
                        .request(clock -> PlantTargetRequest.equivalentPosition("slot-2", 240.0))
                        .nearestToMeasurement()
                        .rejectUnreachable()
                        .whenUnavailable().holdMeasuredTargetOnEntry(0.0)
        )
        .build();
```

The request does not repeat the period. The Plant owns that topology. If the Plant were linear,
`equivalentPosition(...)` would be unavailable unless a fallback/hold policy produces a target.

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

`holdMeasuredTargetOnEntry(...)` still provides a target every cycle. It latches the current
measurement when the hold source is entered, then keeps emitting that captured value. It does not
mean “skip commanding the Plant this loop.”

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
    return PlantTargetRequest.equivalentPosition(
            facing.sourceId(),
            targetDeg,
            facing.quality(),
            facing.ageSec(),
            facing.timestampSec()
    );
};

PlantTargetSource turretTarget = PlantTargets.plan()
        .request(turretFacingRequest)
        .nearestToMeasurement()
        .rejectUnreachable()
        .accept().maxRequestAgeSec(0.20).minQuality(0.45).doneAccept()
        .whenUnavailable().holdMeasuredTargetOnEntry(0.0);
```

The Plant reports physical arrival with `atTarget()` and `atTarget(value)`. Target planning reports
selection status with `PlantTargetPlan`; it does not claim the mechanism has physically arrived.

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
