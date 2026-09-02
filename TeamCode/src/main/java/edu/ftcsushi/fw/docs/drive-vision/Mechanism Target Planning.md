# Mechanism Target Planning

`PlantTargets` is the framework's target-generation system for `Plant`s. It turns simple numbers,
command targets, queued pulses, behavior overlays, equivalent positions, and advanced alternative
requests into one requested Plant target each loop.

The Plant still owns low-level hardware/control work. Target planning answers:

```text
what target should this Plant request this loop?
```

The Plant answers:

```text
can this hardware safely apply that request, and is the mechanism at that target?
```

## How to read the construction examples

In an ordinary FTC robot, a mechanism constructor receives `HardwareMap` plus its robot-owned,
data-only config, defensively copies that config, and constructs its private final target graph and
Plant there. The composition root constructs the mechanism; it does not normally construct a Plant
and pass that raw Plant across the boundary. The mechanism also owns Plant update order, stop, and
semantic methods such as `collect()` or `aimAt(...)`.

The short resolver and `FtcActuators` fragments in this guide are therefore construction-time
excerpts from that owner unless a section explicitly labels a lower-level custom-adapter or test
seam. Resolver graphs are built once, not rebuilt in `update(...)`. The complete beginner reference
is the [`Basic Mechanisms Robot`](<../getting-started/Basic Mechanisms Robot.md#complete-source-and-owner-map>).

```text
controller / service / autonomous policy
    decides which behavior is active
        ↓
command target (optional ScalarTarget)
    the stable request that robot policy and ScalarTasks may change
        ↓
PlantTargets graph
    exact target, overlay, equivalent-position transform, request resolver, fallback/hold policy
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

Target planning stops at the applied Plant target; it must not pre-clamp or duplicate the later
native mapping. The realization pipeline is `Plant target -> shared Plant-to-native map -> optional
per-child scale/bias -> output adapter`. Core Plants reject non-finite mapping arithmetic, while an
FTC construction adds its actual Servo, normalized-power, velocity, or integer-position domain.
Power and velocity child maps are scale-only so exact zero remains zero; additive child alignment is
position-only. Invalid configuration is reported before hardware effects when the complete static
map is known. At runtime, the core Plant-to-native conversion is checked before applied state or
output; the later FTC layer precomputes every child/domain result before writing the first child.

## Advanced target-range protocol

`ScalarRange` is the advanced value exchanged by a `PositionPlant`, `PlantTargetContext`, and the
planner. It describes which finite target values are legal; it is not another ordinary Plant
builder. A valid range has exactly one explicit shape:

- `ScalarRange.bounded(finiteMin, finiteMax)` — both inclusive bounds with
  `finiteMin <= finiteMax` (equality is a valid singleton);
- `ScalarRange.boundedFrom(finiteMin)` — an inclusive lower bound and no upper bound;
- `ScalarRange.boundedTo(finiteMax)` — no lower bound and an inclusive upper bound; or
- `ScalarRange.unbounded()` — no software target bound.

Every supplied endpoint must be finite. Do not pass `NaN` or an infinity to a bounded factory as a
sentinel for an absent side. A non-finite value is never a legal target, including for an unbounded
range. `ScalarRange.invalid(reason)` is separate: it publishes temporary runtime unavailability,
such as a position coordinate that has not been referenced, so the resolver can follow its explicit
`whenUnavailable()` policy.

Ordinary `Plants.fromOutputs()` and `FtcActuators` construction stays narrower: its range question
offers only `bounded(min, max)` with two finite endpoints or `unbounded()`, and a standard-servo
position path is bounded-only. The one-sided shapes above are for advanced custom Plant/context
producers; they are not hidden builder answers.

## The one target rule

For anything intended to become a Plant target, use `PlantTargets`.

```text
ScalarSource
    A number stream. Useful as a primitive.

ScalarTarget
    A writable number stream. When it is the exact source or overlay base, it is the command target.

PlantTargetResolver
    A Plant-aware graph that uses Plant context to resolve the requested target.

PlantTargetResolution
    The immutable result of resolving that graph for one Plant update.

PlantTargets
    The factory/builder family that creates exact, overlay, equivalent, and planned resolvers.
```

The Plant builder has one ordinary binding and one advanced binding:

```java
.targetFromNewCommand(double initialValue)         // create the ordinary exact command
.targetFromResolver(PlantTargetResolver resolver)  // bind the supplied final resolver
```

For a simple exact Plant, keep one Plant variable and create its command inline with
`targetFromNewCommand(initialValue)`. Immediate methods and Tasks retrieve that generated, stable
command with `plant.commandTarget()`. Retrieval is side-effect-free, so ordinary robot code
may do that at the point of use instead of retaining a second field beside the Plant.

Keep a named `ScalarTarget` when it is useful before the Plant exists: for a standalone or shared
request, a target-only policy object, or the base of an overlay, equivalent-position transform, or
advanced graph. Bind a named target with `targetFromResolver(PlantTargets.exact(target))`. If a Plant
should follow a read-only `ScalarSource`, use
`targetFromResolver(PlantTargets.exact(source))`. `targetFromResolver(...)` describes the source of
resolution, not whether the Plant is commandable: a supplied resolver may carry a recognized stable
command target, or it may carry none. An arbitrary custom resolver has no way to claim framework
command-target provenance; compose it through the recognized `PlantTargets` graph when that
provenance matters.

The final graph owns this relationship; the builder never asks robot code to register a second,
possibly disconnected target. An exact source carries a command target when its runtime object is a
`ScalarTarget`. An overlay carries only the command target from its base graph. `ScalarTarget`s used
as conditional layers never become the command target, because a task writing a layer does not own
that layer's activation and an overlay may contain several such layers.

Put the stable robot request in the base when tasks should be able to command a composed Plant. This
is a construction-time excerpt from the mechanism that owns `arm`:

```java
private final ScalarTarget armCommand = ScalarTarget.create(STOWED);
private final PositionPlant arm;

// In the ArmMechanism constructor:
PlantTargetResolver finalArmTarget = PlantTargets.overlay(armCommand)
        .add("autoStow", autoStowRequested, STOWED)
        .add("manual", manualActive, manualTarget)
        .build();

this.arm = FtcActuators.plant(hardwareMap)
        .motor("arm", Direction.FORWARD)
        .position()
        .deviceManaged()
        .nonPeriodic()
            .bounded(0.0, 4200.0)
            .nativeUnits()
            .alreadyReferenced()
        .positionTolerance(20.0)
        .targetFromResolver(finalArmTarget)
        .build();
```

```java
Task raiseArm = ScalarTasks.set(armCommand, HIGH)
        .untilReachedBy(arm)
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

The command target is optional. Constants, ordinary read-only sources, planned resolvers, measured holds,
and custom target graphs without a command base cannot support a feedback-aware `ScalarTasks`
move. `hasCommandTarget()` reports that capability, while `commandTarget()` remains available for
framework validation, calibration, compact testers, and the mechanism-owner pattern described
below.

### Keep one Plant variable for a simple exact mechanism

A simple exact Plant creates its command inline, and the mechanism owner retains only the completed
Plant. `commandTarget()` returns the same stable `ScalarTarget` for the Plant's lifetime and has no
sampling or hardware side effects, so commands can retrieve it where they write it. Following the
starter pattern, the ordinary ownership shape is (`IntakeConfig` is illustrative robot-owned data,
not a framework type):

```java
final class IntakeMechanism {
    private static final double STOPPED_POWER = 0.0;

    private final Plant intake;
    private final double collectPower;

    IntakeMechanism(HardwareMap hardwareMap, IntakeConfig config) {
        IntakeConfig snapshot = Objects.requireNonNull(config, "config").copy();
        this.collectPower = snapshot.collectPower;
        this.intake = FtcActuators.plant(
                        Objects.requireNonNull(hardwareMap, "hardwareMap"))
                .motor(snapshot.motorName, snapshot.direction)
                .power()
                .targetFromNewCommand(STOPPED_POWER)
                .build();
    }

    public void collect() {
        intake.commandTarget().set(collectPower);
    }

    void update(LoopClock clock) {
        intake.update(clock);
    }

    void stop() {
        intake.stop();
    }
}
```

The composition root passes FTC resources and checked-in configuration to that owner:

```java
intake = new IntakeMechanism(hardwareMap, profile.intake);
```

An overlay follows the related composed-graph rule inside the same owner: retain its named base while
constructing the graph, let the final Plant carry that base as its command identity, then retain the
completed Plant. A read-only or planned mechanism also constructs and retains only its Plant but
does not require or invent a command target. A policy object that only writes a standalone or
deliberately shared request and does not own Plant lifecycle receives only the `ScalarTarget` or a
robot-owned semantic capability.

### Lifecycle stop is not another target

`Plant.stop()` permanently ends that Plant instance. It claims the terminal state before invoking
the realization's natural stop, and every later `update(clock)` returns before sampling feedback,
resolving an exact/overlay/equivalent/planned graph, advancing guards or controllers, or commanding
hardware. Repeated calls are harmless, and a cleanup failure still leaves the Plant terminal.

The resolver graph and optional command target are deliberately left unchanged. A planned or
read-only Plant therefore needs no plan-specific disable operation, while a shared target is not
silently changed for another consumer. Use a normal zero-power/zero-velocity command or position
hold when a mechanism should pause and later resume during the active match. A new lifetime uses a
new Plant rather than a reset or restart method.

The natural final output action depends on the realization, not merely the public target domain.
Power and velocity paths submit zero; a standard-servo position path retains its last position
command; motor and framework-regulated position paths may remove power. `PositionPlant` inherits
the same terminal `stop()` contract rather than defining a second meaning.

Injecting a completed Plant is an explicit exception for a package-private hardware-neutral test
seam, a custom hardware adapter, or another deliberately portable host. Label that constructor as
such. At that seam, pass the Plant alone—never a Plant and its target as peer dependencies—and
validate `hasCommandTarget()` only when the injected owner needs a writable command.

Do not replace this rule with `Plant.set(...)`, a Plant-root Task facade, a target-to-Plant backlink,
or a public binding wrapper. Those add a second command path or another noun. The one intentional
place both objects remain explicit is a feedback-aware Task:

```java
ScalarTasks.set(intake.commandTarget(), GOAL)
        .untilReachedBy(intake)
        .cancelTo(IDLE)
        .build();
```

Here the target identifies the persistent request being written, while the Plant selects the
resolution provenance and physical feedback used for completion. A target may feed more than one
Plant, so the observer cannot be inferred from the target.

## Exact targets

For an ordinary exact Plant, create the command inline in the owning mechanism constructor and
retrieve it from the Plant when a semantic method or Task writes it:

```java
private final PositionPlant lift;

// In the LiftMechanism constructor:
this.lift = FtcActuators.plant(hardwareMap)
        .motor("lift", Direction.FORWARD)
        .position()
        .deviceManaged()
        .nonPeriodic()
            .bounded(0.0, 4200.0)
            .nativeUnits()
            .needsReference("lift not homed")
        .positionTolerance(20.0)
        .targetFromNewCommand(0.0)
        .build();

// In semantic command/update methods on LiftMechanism:
void selectBasket() {
    lift.commandTarget().set(BASKET_TICKS);
}

void update(LoopClock clock) {
    lift.update(clock);
}
```

The same Plant-owned target is also the Task entry point:

```java
Task raiseLift = ScalarTasks.set(lift.commandTarget(), BASKET_TICKS)
        .untilReachedBy(lift)
        .cancelTo(0.0)
        .build();
```

Every feedback move must choose `.cancelTo(value)` or `.leaveTargetOnCancel()` immediately after
`.untilReachedBy(plant)`. The latter deliberately leaves the move request in place, so motion may
continue.
Neither option imperatively stops hardware: `cancelTo(...)` changes the command target, while
the final target still flows through overlays, bounds, references, and guards on the next Plant
update. At final owner shutdown, cancel independently owned Tasks and queues, then terminally stop
the Plant; there is no need to neutralize its resolver graph solely to prevent later actuation.

## Same command, equivalent periodic positions

Most mechanisms request one logical number. A lift position, a bounded turret angle, and a freely
rotating turret angle should therefore use the same robot-facing capability and Task:

```java
turretCommand.set(GOAL_ANGLE_DEG);

Task aim = ScalarTasks.set(turretCommand, GOAL_ANGLE_DEG)
        .untilReachedBy(turret)
        .leaveTargetOnCancel()
        .timeout(1.0)
        .build();
```

Only mechanism realization changes. An exact, unwrapped turret uses:

```java
PlantTargetResolver finalTurretTarget = PlantTargets.exact(turretCommand);
```

A turret that may use any whole-turn equivalent wraps the same command:

```java
PlantTargetResolver finalTurretTarget =
        PlantTargets.equivalentPositionsOf(turretCommand)
                .nearestToMeasurement()
                .whenUnavailable().holdMeasuredTargetOnEntry(0.0);
```

If `GOAL_ANGLE_DEG` is `20` and the turret is near `350`, a 360-degree periodic Plant may request
physical position `380`. The same `ScalarTasks.set(turretCommand, 20)` still expresses logical
value `20`; its feedback branch completes only
when the `turretCommand` path won and the Plant reports physical arrival at `380`. A same-valued
overlay, fallback, hold, clamp, bound, or guard cannot make the move report success.

Periodicity does not silently change exact commands. A Plant may have periodic equivalence while some
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
PlantTargetResolver logicalTurretTarget = PlantTargets.overlay(turretCommand)
        .add("vision", visionAimEnabled, visionAngleDeg)
        .add("stow", stowRequested, STOW_ANGLE_DEG)
        .build();

PlantTargetResolver finalTurretTarget =
        PlantTargets.equivalentPositionsOf(logicalTurretTarget)
                .nearestToMeasurement()
                .whenUnavailable().holdMeasuredTargetOnEntry(0.0);
```

The overlay decides which logical behavior wins. The outer transform then chooses that winner's
physical equivalent. The base `turretCommand` remains the graph-owned Task command; conditional
layers do not become competing writers.

## Overlays: behavior priority in target space

Use `PlantTargets.overlay(...)` when several behaviors can influence the same Plant. The base target
must be total. Later enabled layers have higher priority. Build this graph and its Plant once in the
mechanism constructor; `update(...)` advances queue/task inputs and then updates the Plant.

```java
private final Plant feeder;

// In the FeederMechanism constructor:
PlantTargetResolver feederTarget = PlantTargets.overlay(0.0)
        .add("stage", stageRequested, 0.20)
        .add("feedPulse", feedPulseQueue.activeSource(), feedPulseQueue)
        .add("eject", ejectRequested, -1.0)
        .build();

this.feeder = FtcActuators.plant(hardwareMap)
        .motor("feeder", Direction.FORWARD)
        .power()
        .targetFromResolver(feederTarget)
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
usually easier to debug when the layer is enabled and its target resolver reports why it used a fallback,
hold target, or explicit unavailable result.

When the desired behavior really is “try this layer, but keep the lower-priority target if it cannot
produce a value,” say that explicitly:

```java
PlantTargetResolver turretTarget = PlantTargets.overlay(PlantTargets.holdMeasuredTargetOnEntry(0.0))
        .addIfAvailable("visionAim", visionAimRequested, visionAimPlanner)
        .add("manual", manualActive, manualAngleTarget)
        .build();
```

`addIfAvailable(...)` is intentionally named differently from `add(...)` because it changes the
meaning of an enabled-but-unavailable layer. Debug output records that the layer was enabled and fell
through to the next enabled lower-priority layer, or ultimately the base, so this is not a hidden
Boolean filter. Disabled and shadowed target producers still are not resolved.

## Target-resolution diagnostics

Plants report two different target diagnostics:

```java
plant.getTargetResolution(); // how PlantTargets selected the requested target
plant.getTargetStatus();     // how the Plant turned requested target into applied target
```

For example, a turret resolution may report `PLANNED_CANDIDATE` with candidate `"slot-2-purple"`,
while the Plant status reports `RATE_LIMITED` because the applied target is still walking toward the
requested target. This separation keeps behavior target generation separate from hardware protection
while making telemetry easier to read.

When the focused periodic transform successfully resolves the selected logical intent, it reports
`EQUIVALENT_POSITION`. Its target is the selected physical representative; the logical command
remains in the graph-owned `ScalarTarget` rather than becoming a second public target field. If the
logical child produced a fallback or hold target, the transformed resolution retains that fallback/hold
kind so diagnostics do not mislabel it as satisfied intent.

For a `PLANNED_CANDIDATE`, `selectedQuality()` reports the chosen candidate's quality. When that
candidate came from an observation, `selectedTimestamp()` retains its epoch-safe `LoopTimestamp`
and `selectedAgeSec()` reports the age derived when the resolver produced this resolution. A
timeless candidate instead reports quality `1.0`, an unavailable timestamp, and `NaN` age. These are selection facts,
not a second hardware-arrival signal.

`satisfiesIntent()` says whether the selected resolution satisfies the active logical intent. Exact,
equivalent, and accepted planned choices satisfy intent; fallback, hold, clamp, and unavailable
results do not. It is not a physical-arrival signal.

## Advanced planning: alternative requests and observation metadata

Use `PlantTargets.plan(request)` when one request contains several named alternatives, relative
targets, explicit periods, observation freshness/quality, or an intentional clamp policy. For one ordinary
logical command with whole-turn equivalents, use `equivalentPositionsOf(...)` above; students do
not need a request object for that case. The advanced resolver does not receive a Plant
object. During `plant.update(clock)`, the Plant supplies:

- feedback availability and measurement
- legal target range
- non-periodic/periodic equivalence and period
- previous requested/applied targets

The factory takes either one fixed `PlantTargetRequest` or a live `Source<PlantTargetRequest>`.
The returned staged builder then asks one required question at a time: one alternative preference,
one unreachable-alternative policy, then `whenUnavailable()`. Optional observation-age/quality
tuning lives in `accept()...doneAccept()` after the required motion-semantics choices have been
made.

The built equivalent-position and advanced planning resolvers publish their selection, fallback or
hold state, diagnostics, result, and cycle atomically after resolution succeeds. A request,
measurement, range, policy, timestamp, or fallback failure therefore cannot partially advance a
hold or overwrite the prior successful resolution. Recursive resolution fails clearly; a later
nonrecursive call in the same cycle may retry a failed value attempt.

### Fixed and dynamic alternatives

When every listed target genuinely satisfies the same goal, compose fixed alternatives directly:

```java
PlantTargetRequest fixedGoals = PlantTargetRequest.oneOf(
        PlantTargetRequest.equivalentPosition("left-goal", leftAngleDeg),
        PlantTargetRequest.equivalentPosition("right-goal", rightAngleDeg));

PlantTargetResolver goalTarget = PlantTargets.plan(fixedGoals)
        .nearestToMeasurement()
        .rejectUnreachable()
        .whenUnavailable().holdLastTarget(0.0);
```

Do not put two operator-selected goals in `oneOf(...)` merely because either could be commanded.
If the operator selected exactly one goal, publish that request alone—or use the ordinary
`ScalarTarget`. `oneOf(...)` authorizes the resolver to choose any listed alternative.

A tray service can convert robot-specific inventory into Plant-unit alternatives. The resolver
chooses the best reachable representative.

```java
Source<PlantTargetRequest> purpleToOutput = Source.of(clock -> {
    List<PlantTargetRequest> alternatives = new ArrayList<>();

    for (int slot = 0; slot < 3; slot++) {
        if (inventory.colorAt(slot) == ArtifactColor.PURPLE) {
            double alignDeg = trayModel.alignSlotToOutputDegrees(slot);
            alternatives.add(PlantTargetRequest.equivalentPosition(
                    "slot-" + slot + "-purple",
                    alignDeg
            ));
        }
    }

    return PlantTargetRequest.oneOf(alternatives);
});

PlantTargetResolver trayTarget = PlantTargets.plan(purpleToOutput)
        .nearestToMeasurement()
        .rejectUnreachable()
        .whenUnavailable().holdLastTarget(0.0);
```

The resolver does not know what “purple” means. It only sees Plant-unit alternatives. An empty list
is an unavailable request automatically; return `PlantTargetRequest.none("no purple artifact")`
explicitly instead when that domain-specific reason matters in diagnostics. The varargs and `List`
forms both preserve declaration order, flatten nested requests, skip unavailable members when some
alternative remains, and defensively retain their inputs. If none remains, composition retains the
first specific absence reason and otherwise reports the generic empty-alternatives reason.

`nearestToMeasurement()` and `rejectUnreachable()` are staged single-answer questions: after one
choice, the returned type exposes only the next question. There is no later-replacement model.
Branches that may set several independent tuning values, such as `accept()`, still end with an
explicit `doneAccept()`.

### How periodic alternatives are selected

A periodic request alternative describes an entire family of equivalent positions. The focused
transform and advanced resolver share the same fixed, bounded selector; work does not grow when a
wider range or smaller period contains more equivalents. This is an internal guarantee, so robot code does not set
a search window or alternative limit.

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
- If distinct request alternatives are otherwise exactly tied, the alternative declared first wins.

### Current intent versus observations

Use the ordinary factories for current robot intent: presets, inventory choices, or other requests
that do not become stale with time. Use the explicitly named `observed...` factories when a target
was calculated from a camera, localization estimate, or another sampled observation. An observed
factory takes one quality value and one stable observation timestamp in the consuming `LoopClock`
timebase; it does not also ask robot code to supply a potentially contradictory age.

`PlantTargetRequest` provides all leaf factories. Use one directly for one acceptable target, or
compose several requests with `oneOf(...)`:

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

The resolver derives age at resolution. `.maxObservationAgeSec(...)` applies only to observed
alternatives; timeless intent is not rejected by that gate. A non-finite or out-of-range quality, an
invalid timestamp, or a timestamp too far in the future makes that observed alternative unavailable.
The resolver may still choose a later valid alternative, and if none remains, its existing
`whenUnavailable()` policy supplies the one declared fallback, hold, or unavailable result.

## `whenUnavailable()` versus overlay

These two tools are intentionally parallel but not interchangeable.

```text
PlantTargets.overlay(...)
    Which behavior layer wins?

whenUnavailable()
    If this selected resolver cannot produce a valid target, what explicit target should it emit?
```

For a planned resolver used directly as the Plant target, choose a total unavailable policy:

```java
PlantTargetResolver turretTarget = PlantTargets.plan(autoAimRequest)
        .nearestToMeasurement()
        .rejectUnreachable()
        .whenUnavailable().holdMeasuredTargetOnEntry(0.0);
```

For a planned resolver inside an overlay, you can still make the layer total:

```java
PlantTargetResolver turretTarget = PlantTargets.overlay(PlantTargets.holdMeasuredTargetOnEntry(0.0))
        .add("autoAim", autoAimRequested,
                PlantTargets.plan(autoAimRequest)
                        .nearestToMeasurement()
                        .rejectUnreachable()
                        .whenUnavailable().holdLastTarget(0.0))
        .add("manual", manualActive, manualAngleTarget)
        .add("stow", stowRequested, 0.0)
        .build();
```

`holdMeasuredTargetOnEntry(...)` is a total resolver whenever it runs. It latches the current
measurement when the hold resolver is entered, then keeps emitting that captured value. It does not
mean “skip commanding the Plant this loop.” Inside an overlay, “entered” means consecutive
resolution cycles in which that hold resolver actually runs. If it is disabled or shadowed for
a complete resolution cycle, its next resolution is a new entry and captures the then-current
measurement. Robot code does not call a selection reset or lifecycle hook to make that happen.

## Spatial-derived requests

Spatial queries can feed a mechanism target, but the conversion remains robot policy. The spatial
solve chooses geometry; robot-specific kinematics map that geometry into one absolute Plant-unit
request; `PlantTargets.plan(request)` resolves the final reachable Plant target.

The optional
[`ReferenceCoordinatedShotService`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/targeting/ReferenceCoordinatedShotService.html>)
uses the translation channel because turret, flywheel, and hood must derive from one vector. These
are the calculation's relevant statements, with remaining validation, model interpolation, and
additional unavailable returns omitted:

```java
SpatialTranslationSelection selection =
        SpatialQuerySelectors.firstValidTranslation(targetQuery.get(clock), spatialGate);
if (selection == null) {
    return unavailable(Reason.SPATIAL_UNAVAILABLE, LoopTimestamp.unavailable());
}
LoopTimestamp timestamp = selection.timestamp();
double observedForward = selection.solution.robotForwardInches();
double observedLeft = selection.solution.robotLeftInches();

MotionAssessment motion = assessMotion(predictor.getLatestMotionDelta(), timestamp, clock);
double effectiveForward = observedForward;
double effectiveLeft = observedLeft;
double quality = selection.quality();
if (motion.usable) {
    effectiveForward -= motion.forwardInchesPerSec * illustrativeFlightTimeSec;
    effectiveLeft -= motion.leftInchesPerSec * illustrativeFlightTimeSec;
    quality = Math.min(quality, motion.quality);
}

double turretAngleRad = Math.atan2(effectiveLeft, effectiveForward);
PlantTargetRequest turretRequest = PlantTargetRequest.observedEquivalentPosition(
        TURRET_REQUEST_ID, turretAngleRad, quality, timestamp);
```

`turretAngleRad` is absolute in the example's declared turret coordinate: zero faces robot
`+X`, and positive angles turn toward robot `+Y` left. It is not a delayed facing error added to a
current turret measurement. The complete service retains the accepted translation's exact
observation timestamp, permits `p - v * t` compensation only from matching timestamped motion, and
publishes one immutable turret/flywheel/hood solution. Its defaults are illustrative software data,
not calibrated projectile physics.

The separate
[`ReferenceTurretMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/targeting/ReferenceTurretMechanism.html>)
privately projects that cached request and owns the entire planner/Plant realization:

```java
Source<PlantTargetRequest> requestSource = Source.of(
        clock -> service.solution().turretRequest);

PlantTargetResolver finalTarget = PlantTargets.plan(requestSource)
        .nearestToMeasurement()
        .rejectUnreachable()
        .accept()
        .maxObservationAgeSec(service.plannerObservationMaxAgeSec())
        .minQuality(service.plannerMinimumObservationQuality())
        .doneAccept()
        .whenUnavailable()
        .holdMeasuredTargetOnEntry(c.initialHoldAngleRad);
```

The `observedEquivalentPosition(...)` name makes the freshness contract visible at the call site,
while the service-owned snapshot keeps every mechanism consumer on one quality value and one
timestamp. The no-reset Source projection owns no state and cannot reset or resample localization.
The mechanism binds `finalTarget` to one full-turn-periodic PositionPlant with finite cable bounds;
it does not write the motor through a second imperative path. See the
[hardware-free Reference scenario](<../examples/Hardware-free Reference Scenarios.md>) for the
managed composition, software evidence, and physical limits of this illustrative example. It is
unwired from the ordinary Reference robot, and its defaults establish no shot accuracy, mechanism
safety, cable limits, or trustworthy encoder zero.

The Plant reports physical arrival with `atTarget()` and literal physical
`atTarget(value)`. `atTarget(value)` never applies modulo arithmetic. Target planning reports
selection status with `PlantTargetResolution`; it does not claim the mechanism has physically arrived.
`ScalarTasks.set(command, value).untilReachedBy(plant)` combines the resolution's logical-command
evidence with physical arrival at the selected requested target. It validates at construction that
the supplied Plant has feedback and that `command` is the exact command target owned by its graph.

## Hardware guards are separate

Plant target generation is behavior policy. Plant target guards are hardware protection.

The builder below is an excerpt from the lift mechanism constructor; the mechanism's loop later
calls only `lift.update(clock)` after its resolver inputs are ready.

```java
private final PositionPlant lift;

// In the LiftMechanism constructor:
this.lift = FtcActuators.plant(hardwareMap)
        .motor("lift", Direction.FORWARD)
        .position()
        .deviceManaged()
        .nonPeriodic()
            .bounded(0.0, 4200.0)
            .nativeUnits()
            .needsReference("lift not homed")
        .positionTolerance(20.0)
        .targetGuards()
            .maxTargetRate(1200.0)
            .holdLastTargetUnless("wrist clear", wristClear)
            .doneTargetGuards()
        .targetFromNewCommand(0.0)
        .build();
```

The ordinary finite `bounded(...)` answer defines the static legal target range. `targetGuards()`
handles dynamic protection such as interlocks, fallback targets, and maximum target rate. If
behavior asks for an impossible or temporarily unsafe target, telemetry can show both the requested
target and the applied target.
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

Search power must be finite and inside the inclusive normalized range `[-1.0, +1.0]`; the recipe
rejects `NaN`, infinities, and overshoot at `.withPower(...)` instead of clamping them. That
structural check does not select a safe magnitude, direction, cue, or mechanical setup—the lift
service still owns those robot-specific decisions.

The reference and post-success hold are separate plant-unit answers and must also be finite. The
recipe rejects `NaN` or infinity at each answer without clamping. A reference is a coordinate anchor
and need not lie inside the Plant's target range. A finite hold is a logical command, so the normal
resolver, range, overlays, and target guards may still transform or clamp it; choose a deliberately
safe in-range hold when exact predictable holding is intended.

The Task runner advances this recipe before the mechanism's downstream update. The search Task
owns the cue, reference, timeout, and handoff decisions, but it never calls `lift.update(clock)`;
the lift mechanism remains the sole Plant heartbeat owner. `holdAfterReference(0.0)` changes the
Plant's graph-owned command before that same downstream Plant phase, which still evaluates the
complete resolver and may select an enabled overlay instead. Use `resumeTargeting()` when success
should preserve the existing persistent command and resume the unchanged resolver. Timeout and
active cancellation also preserve that command while requesting a temporary-output stop and
releasing the search.

That calibration stop is an internal, nonterminal output handoff, not public `Plant.stop()`.
Calibration can therefore resume normal targeting after it releases search ownership without
introducing a second public Plant lifecycle method.

The resolver should not decide when zero is trustworthy. Homing, indexing, manual zeroing, and
semantic presets belong in the robot mechanism/service layer. Finite software values cannot prove
the physical cue, pose, scale, reference, or hold is correct; the adopting mechanism must verify
those facts. When a periodic re-reference has a finite current plant estimate, the Plant rejects a
non-finite final nearest-equivalent result before commit. It also validates the complete candidate
bounded affine map before reference or public-measurement state changes. A later realized command
must still pass any native-domain check defined by its concrete output; an unbounded core
Plant-to-native conversion is checked one at a time before applied-target commit or output.
