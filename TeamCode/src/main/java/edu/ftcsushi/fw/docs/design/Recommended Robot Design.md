# Recommended Robot Design

## Recommended top-level split

For larger robots, use this ownership pattern:

- **direct framework owners** when one object already owns the complete runtime capability
  - example: `MecanumDrivebase`, created at the FTC boundary by `FtcDrives.mecanum(...)`
- **framework lanes** for stable FTC-side systems that recur year to year
  - example: `FtcWebcamAprilTagVisionLane` or `FtcLimelightAprilTagVisionLane` (behind the `AprilTagVisionLane` seam)
  - example: `FtcWebcamVisionPortalLane` or `FtcLimelightVisionLane` behind a robot-owned typed
    vision interface when the season needs multiple semantic modes
  - example: `FtcOdometryAprilTagLocalizationLane`
- **shared field facts** for layouts and landmarks used by several systems
  - example: `TagLayout` and `FtcGameTagLayout.currentGameFieldFixed()`
- **robot-owned capability families** for the shared TeleOp/Auto vocabulary
  - example: `MyCapabilities.gamePiece()`
  - example: `MyCapabilities.targeting()`
- **robot-owned controls** for all TeleOp input semantics
  - driver sticks
  - slow mode
  - auto-aim enable/override
  - scoring signals and edge/change/toggle behavior
- **robot-owned policy/services** for game-specific logic
  - targeting
  - supervisors
  - shot models
  - telemetry presenters

A good composition root wires those owners together, but does not absorb their responsibilities.
In ordinary FTC robot code, it declares those completed owners through one framework-created
`RobotProgram`; the framework host owns FTC callback forwarding, the one clock, phase advancement,
telemetry commit, and fail-stop cleanup.
For custom vision, keep camera plumbing and trustworthy acquisition in the concrete framework
owner, map robot meanings to processor enablement or pipeline transitions in one robot realization,
and expose immutable timestamped robot snapshots to strategy. Do not make Auto or TeleOp interpret FTC/vendor
results, and do not pretend that webcam processors and Limelight pipelines have the same lifecycle.

Treat vision Configs as data-only drafts. The selected direct owner, or a deliberately deferred
vision-lane factory, validates and captures its complete active configuration before hardware
acquisition. A custom FTC tag library stays borrowed while it is only an inactive profile value;
the active webcam boundary deep-snapshots its mutable metadata. Runtime consumers use focused lane
capabilities and diagnostics rather than recovering a construction Config from the lane.

Likewise, author fixed-tag solve tuning in `FixedTagFieldPoseSolver.Config`, but construct a
`FixedTagFieldPoseSolver` before passing that completed policy into spatial or guidance graphs.
`AprilTagPoseEstimator.Config` composes its own solver Config with freshness and camera-mount data;
it is not a specialized solver Config.

### Why this matters

This keeps stable framework code reusable across seasons without turning the lifecycle host into a
giant robot capability superclass. It also keeps each robot's control scheme and game logic easy to
find, instead of scattering it across convenience helpers and subsystem constructors.

This is an architecture guide for readers who already understand
[`Robot roles`](<../getting-started/learn-sushi/Robot Roles.md>). Read
[`Framework Lanes & Robot Controls`](<Framework Lanes & Robot Controls.md>) when you need the full
ownership vocabulary used here.

This document describes Sushi's recommended design for robot code that will be shared between
TeleOp and Auto.

The short version is:

- **TeleOp bindings and Auto routines should talk to the robot through the same small intent API.**
- **Subsystems should own their Plant target resolvers and update order.**
- **Supervisors should own policy, timing, requests, and queueing.**
- **Status snapshots should be the normal way to observe a mechanism from the outside.**
- **Use the behavior pattern that matches the problem instead of forcing everything into one abstraction.**

If you keep those rules, a robot can stay understandable even as it grows.

## Related guides

- [`Loop Structure`](<../core-concepts/Loop Structure.md>)
- [`Supervisors & Pipelines`](<Supervisors & Pipelines.md>)
- [`Tasks and Macros`](<Tasks & Macros Quickstart.md>)
- [`Framework Principles`](<../../Framework Principles.md>)

Most examples in this document use real Sushi types. A few route-library snippets are
intentionally conceptual, because the exact adapter API depends on the library you choose to wrap.

For ordinary FTC mechanisms, the examples follow the same construction boundary as the
[`Basic Mechanisms Robot`](<../getting-started/Basic Mechanisms Robot.md#complete-source-and-owner-map>):
the composition root checks the
robot-level permissions and cross-owner relationships it owns, then passes the selected active
Config to `new Mechanism(hardwareMap, profile.mechanism)`. The mechanism defensively captures and
validates that data-only configuration before its own hardware effects, constructs and privately
owns its Plants and local sensors, and owns their update and stop lifecycle. A short-lived aggregate
profile need not be copied or retained by the composition root; one fresh `current()` graph can be
synchronously distributed to the owners selected by the mode.
If a proven FTC device-managed velocity Plant also needs live tuning, keep this exact production
owner and declare the framework workflow described in
[`Declare one framework tuner`](<../testing-calibration/Control Tuning Workflow.md#choose-the-workflow-that-matches-production>).
The tuner receives a fresh Plant from the production owner's canonical recipe; it does not require
a second robot-specific mechanism or tuning-session object.
Do not make live-tuning ceremony mandatory for ordinary checked-in configuration.

The illustrative `WristConfig`, `LiftConfig`, and `IntakeConfig` types below are data-only profile
slices with a `copy()` method. Their owning mechanism should validate its copied names, directions,
finite ranges, and gains before that owner's first hardware effect. A software-valid default is only
an authoring baseline; it is not proof about installed hardware or safe motion. These proven
owner-local copies do not imply that the outer robot profile also needs a public `copy()`.

All Java snippets in this document use Java 8-compatible syntax so they match the FTC project
environment. That means examples avoid newer language features such as records and switch
expressions.

---

## The main recommendation in one picture

Think of a robot as four layers:

1. **Program declaration layer**
   - one `FtcRobotOpMode.configure(RobotProgram)` composition root
   - TeleOp binding declarations
   - one optional Auto root Task / route graph

2. **Intent / capabilities layer**
   - robot-owned capability families such as `gamePiece()` or `targeting()`
   - small public methods like `setTargetHeightIn(...)`, `setIntakeEnabled(...)`,
     `requestSingleShot()`
   - generic actuator snapshots like `PositionPlantSnapshot` and custom policy snapshots like
     `ShooterSupervisor.Status`

3. **Mechanism layer**
   - subsystems own plants, sensors, and final target calculation
   - supervisors own policy, timing, state machines, and queueing

4. **Behavior-pattern internals**
   - local targets
   - scalar regulation
   - event/classification supervision
   - spatial relation guidance
   - external route integration

The key simplification is this:

> **TeleOp and Auto do not need to share the same top-level code. They should share the same mechanism vocabulary.**

That vocabulary is usually:

- persistent goals
- momentary requests
- mode/enable flags
- cancel/reset methods
- status snapshots

---

## The five behavior patterns

Sushi stays simplest when you choose the pattern that matches the problem instead of forcing every
mechanism into one abstraction.

Here **pattern** means a problem-solving approach, not a framework resource owner. Spatial APIs also
use the domain-specific term `SpatialSolveLane` for one ordered query alternative; that name does
not imply a generic framework-lane lifecycle.

Use this quick decision rule:

1. **Am I commanding a local actuator target directly?** Use a `Plant`.
2. **Am I regulating one measured scalar toward a target?** Use a scalar controller source.
3. **Am I reacting to an event or classification?** Use `BooleanSource` / `Source<T>` with a
   supervisor or task.
4. **Am I solving a 2D frame-to-target relation?** Use drive guidance or a similar spatial layer.
5. **Am I following an external route package?** Adapt it into Sushi tasks and status; do not
   duplicate the planner.

### Pattern 1: local target

Examples:

- wrist servo pose
- motor position target in ticks
- flywheel velocity target
- raw motor or CR servo power

This is the simplest pattern. The mechanism already has a meaningful local command mode. Do not add a
controller loop or a queue unless you really need one.

### Pattern 2: scalar regulation

Examples:

- lift height from a potentiometer or distance sensor
- arm angle from an analog sensor
- local turret hold from an encoder

This is still a **local scalar** problem. One measured variable is driven toward one target.
Sushi's `ScalarControllers` helper packages the common pattern:

```java
ScalarSource liftPower = ScalarControllers.pid(desiredHeightIn, measuredHeightIn, pid);
```

The controller source stages target and measurement reads before invoking the stateful controller.
Those value reads may be retried after failure. Once controller invocation begins, a thrown
exception is retained and rethrown for that cycle rather than invoking potentially mutated
controller state twice; a later cycle may try again.

### Pattern 3: event / classification supervision

Examples:

- touch sensor for homing
- beam break for game-piece present
- color sensor classifying an object
- robot-classified current spike based on `FtcSensors.motorCurrentAmps(...)` indicating a jam or
  hard stop

These sensors are telling you that something happened, not "how far away" you are. Treat them as
signals that a supervisor or task reacts to.

If the signal needs memory across an explicit window (for example, keep a slot classification until
an encoder boundary pulse), keep that memory as a `Source` using `accumulateUntil(...)` rather than
teaching the FTC boundary adapter about your mechanism. If a supervisor owns that lifecycle
explicitly (mode change, task restart, tester clear, or a state-machine transition), keep
`reset()` on the composed source and call it from the owner instead of forcing every boundary into
a synthetic `BooleanSource`.

For an ordinary clock-aware object value, adapt the calculation with `Source.of(...)` and compose
the decorators it needs:

```java
Source<Classification> classification =
        Source.of(this::classifyCurrentSample).memoized();
```

Do not implement an anonymous `Source` or duplicate a `lastCycle` cache in robot code. Direct
implementation is reserved for a named framework/integration extension with an additional domain
contract; fixed object values use `Source.constant(...)`, and clockless primitive leaves use
`ScalarSource.of(...)` or `BooleanSource.of(...)`.

### Pattern 4: spatial relation guidance

Examples:

- drivetrain goes to a field point
- drivetrain aims at a target while translating
- turret faces a tag
- end effector lines up to a tag-relative point

Today the mature public framework consumer for this pattern is
[`Drive Guidance`](<../drive-vision/Drive Guidance.md>). The shared idea is a frame-to-target
relation, but Sushi only publishes the drive-shaped version until a second real consumer proves a
cleaner general API.

### Pattern 5: external route integration

Examples:

- Road Runner
- Pedro Pathing
- a custom route follower you already trust

Sushi should not reimplement those planners. Wrap them in tasks, let Sushi own mechanism
supervision around them, and use Sushi cancellation seams to interrupt them cleanly.

Separate behavior selection from lifecycle ownership. If the vendor follower owns pose, callbacks,
hold, or drive realization outside an active route execution, the robot composition root must
advance its adapter every Auto loop. Route and guidance Tasks may call the same hook while active,
so the adapter must deduplicate by `LoopClock.cycle()`. Its `stop()` must apply a physical stopped state
immediately; storing zero for a future heartbeat is not enough.

Separate route identity from mutable follower state too. `RouteFollower.follow(route)` returns a
`RouteExecution` whose `status()` and `cancel()` belong only to that run. The adapter classifies the
vendor transition once; `RouteTask.getRouteStatus()` then reports one exact terminal status:
`COMPLETED`, `FOLLOWER_TIMEOUT_OR_STALL`, `INTERRUPTED`, `REPLACED`, `TASK_TIMEOUT`, `CANCELLED`,
`FAILED`, or `UNKNOWN_TERMINAL`. This complexity belongs at the route boundary, not at every Auto
call site.

Keep fixed geometry on the ordinary eager `RouteTasks.follow(...)` path. When a later route must
start from the robot's live pose or a current vision choice, use
`RouteTasks.followBuiltAtStart(...)` with one quick robot-owned path-factory lambda. It resolves
exactly once when that Task starts; the integration still receives only a concrete route and no
vendor or game-strategy type enters the generic Task API.

In a single-module codebase, keep that pattern-5 code at the edges:

- framework-owned bridges in `fw/integrations/<library>/`
- robot-specific autos/examples in `.../autonomous/<library>/`
- core framework and robot packages depending only on `RouteFollower<RouteT>`, `RouteExecution`,
  `RouteStatus`, and `RouteTask<RouteT>`

---

## What each layer should own

### Robot container / Robot class

The robot container wires everything together and declares each owner in a clear order. In the
ordinary path it has no clock, Task runner, mode flags, FTC callback forwarding, or stop shell;
`FtcRobotOpMode` owns those reusable mechanics through `RobotProgram`.

It should usually own:

- construction of stable FTC lanes and robot-owned mechanisms
- the stable `HardwareMap` resource and synchronous routing of selected active profile slices into
  their owners
- robot-level permissions and relationships that cross owner boundaries
- supervisor construction
- gamepad binding declarations
- service/output declaration order
- shared debug/telemetry formatting

It should usually **not** own:

- low-level plant math
- repeated policy logic
- sensor gating rules
- queue management details for one mechanism

A robot container is the place where it should be obvious what exists on the robot. It is not the
place where each mechanism's Plant graph or detailed behavior should live.

For a mode-asymmetric robot, keep the root constructor resource-only and put mode inputs at the
declaration boundary. Construct the root with `HardwareMap`, then pass one fresh profile plus only
the selected mode's inputs to `declareTeleOp(...)` or `declareAuto(...)`. Perform any centralized
cross-owner hardware collision preflight before effects. Neither mode should retain or broadly copy
the aggregate profile, and Auto should not receive unused Gamepads. A custom caller supplying an
opaque drive sink must enforce its own exclusive hardware ownership.

### Coordinated cleanup is automatic for declared program owners

An ordinary program immediately owns each registered service, output, drive sink, and root Task.
Its terminal cleanup order is fixed: cancel Tasks, clear bindings, stop outputs in declaration
order, then stop services in reverse declaration order. If later configuration or a runtime phase
fails, already registered siblings are still cleaned and the exact primary failure is retained.

Coordinated cleanup remains explicit in a deliberate custom host or inside one owner with several
private resources. Such an owner marks itself terminal or detaches its references first, then lists
its real actions in their required safety order:

```java
public void stop() {
    if (stopped) {
        return;
    }
    stopped = true;

    CleanupActions.attemptAll(
            runner::cancelAndClear,
            mechanism::stop,
            drive::stop
    );
}
```

Every later action is attempted if an earlier one throws a `RuntimeException`. The first cleanup
failure remains primary and later failures are suppressed onto it; an `Error` propagates
immediately. If an update or construction operation already failed, keep that more useful failure
primary:

```java
catch (RuntimeException failure) {
    throw CleanupActions.attemptAllAfterFailure(
            failure,
            () -> { if (mechanism != null) mechanism.stop(); },
            () -> { if (drive != null) drive.stop(); }
    );
}
```

The second method returns the same supplied exception after attaching cleanup failures. The caller
may rethrow it, wrap it as an actionable cause, or retain it for telemetry. `CleanupActions` does
not choose eligibility, ownership, idempotence, rollback order, retry, or replacement policy. The
construction-failure form uses null-guarded lambdas because a bound `mechanism::stop` reference
would dereference an owner before `attemptAllAfterFailure(...)` can protect the primary failure.
This helper is not a normal command sequencer. Continuing later drive, feed, or mechanism commands
after a failed prerequisite can be unsafe.

### Subsystem

A subsystem owns the target resolvers and Plant update order for one mechanism or one tightly-coupled hardware group.

A subsystem should usually own:

- plants and direct hardware-facing objects
- the sensor sources most closely tied to that mechanism
- the mechanism's long-lived desired state
- output queues used as temporary overrides
- the final target resolver that each Plant invokes during its update
- a small `status()` snapshot

A subsystem should usually **not** own:

- cross-subsystem policy
- match strategy
- driver button semantics
- large mode-selection logic spread across several mechanisms

### Supervisor

A supervisor is the policy/orchestration layer above one subsystem or a small group of closely
related mechanisms.

A supervisor should usually own:

- request counters
- cooldowns
- mode enables
- task queueing
- "do X only when Y is true" rules
- a compact status/debug snapshot for higher-level code

A supervisor should usually **not** command Plants directly. It should decide what should happen,
then let the subsystem remain the owner of the mechanism target resolvers and Plant update order.

### TeleOp and Auto

TeleOp and Auto are both clients of the robot. They should normally call the same public intent
methods and read the same status snapshots.

That is the main recommended reuse boundary.

### Capability families

For larger robots, expose that shared vocabulary through one robot-owned capabilities aggregate with
a few cohesive families.

Example shape:

```java
public interface MyCapabilities {
    GamePiece gamePiece();
    Targeting targeting();

    interface GamePiece {
        void setIntakeEnabled(boolean enabled);
        void requestSingleShot();
        void cancelTransientActions();
        GamePieceStatus status();
    }

    interface Targeting {
        TargetingStatus status();
        Task aimTask(DriveCommandSink driveSink, DriveGuidanceTask.Config cfg);
    }
}
```

The split philosophy is:

- group one coherent public story per family
- keep TeleOp and Auto on the same vocabulary
- do not mirror every internal class mechanically
- do not mirror TeleOp button semantics
- keep the common path easy instead of over-splitting for SOLID mechanically

Those families are a **robot-owned** pattern, not usually a framework lane. The exact names should
change when the robot's public story changes from season to season. For the full decision rules, see
[`Robot Capabilities & Mode Clients`](<Robot Capabilities & Mode Clients.md>).

---

## The common API shape

For most shared mechanisms, one capability family should expose a public API that looks something like this:

```java
public interface LiftApi {
    void setTargetHeightIn(double heightIn);
    void home();
    void cancelTransientActions();
    PositionPlantSnapshot status();
}
```

That is only an example, but the shape matters:

- **persistent goal** — `setTargetHeightIn(...)`
- **momentary request** — `home()`
- **cancel/reset** — `cancelTransientActions()`
- **read-side snapshot** — `status()`

This style keeps the number of things the rest of the robot needs to know very small.

### Prefer these public method categories

Use these as the default vocabulary:

- **Held values / selections**
  - `setPose(...)`
  - `setTargetHeightIn(...)`
  - `setFlywheelEnabled(boolean)`
  - `selectPreset(...)`

- **Frame commands**
  - `commandManualPower(...)`
  - `commandJog(...)`

- **Pending requests**
  - `requestSingleShot()`
  - `home()`
  - `captureCurrentHeading()`

- **Mode / enable flags**
  - `setIntakeEnabled(boolean)`
  - `setAssistEnabled(boolean)`
  - `setContinuousShootEnabled(boolean)`

- **Cancel / reset**
  - `cancelTransientActions()`
  - `resetController()`

- **Read-side status**
  - `status()`

### Avoid making the public API too TeleOp-shaped

Methods like these are sometimes fine as private helpers or binding adapters:

- `toggleFlywheel()`
- `setShootHeld(boolean)`
- `whileRightTriggerHeld()`

But they are usually **not** the best shared interface for both TeleOp and Auto.

Prefer:

- `setFlywheelEnabled(boolean)` instead of only `toggleFlywheel()`
- `requestSingleShot()` instead of exposing a button-shaped queue detail
- `commandManualPower(double)` for per-loop manual control instead of writing raw stick values directly to a plant
- `beginShooting()` / `endShooting()` instead of only `setShootHeld(...)`

TeleOp bindings can always build toggles, hold behavior, or `copyEachCycle(...)` mappings on top of a cleaner public API.

---

## Detailed example 1: wrist poses (pattern 1: local target)

This is the simplest pattern and the one beginners should reach for first.

### The problem

A wrist servo has three valid poses:

- `STOW`
- `INTAKE`
- `SCORE`

The wrist is just a local target problem. It does not need a controller loop or an output queue.

### Recommended subsystem shape

```java
public final class Wrist {
    public enum Pose { STOW, INTAKE, SCORE }

    public static final class Status {
        private final SemanticScalarSnapshot<Pose, PositionPlantSnapshot> actuator;

        private Status(SemanticScalarSnapshot<Pose, PositionPlantSnapshot> actuator) {
            this.actuator = actuator;
        }

        public Pose requestedPose() {
            return actuator.request().semantic();
        }

        public double requestedPosition() {
            return actuator.request().commandTarget();
        }

        public double appliedPosition() {
            return actuator.plant().appliedTarget();
        }

        public PositionPlantSnapshot plantSnapshot() {
            return actuator.plant();
        }
    }

    private final SemanticScalarCommand<Pose> poseCommand;
    private final PositionPlant plant;
    private final double stowTarget;
    private final double intakeTarget;
    private final double scoreTarget;

    public Wrist(HardwareMap hardwareMap, WristConfig config) {
        WristConfig snapshot = Objects.requireNonNull(config, "config").copy();
        stowTarget = snapshot.stowPosition;
        intakeTarget = snapshot.intakePosition;
        scoreTarget = snapshot.scorePosition;
        poseCommand = SemanticScalarCommand.create(Pose.STOW, this::targetFor);

        plant = FtcActuators.plant(Objects.requireNonNull(hardwareMap, "hardwareMap"))
                .servo(snapshot.servoName, snapshot.direction)
                .position()
                .nonPeriodic()
                    .bounded(0.0, 1.0)
                    .nativeUnits()
                .targetFromResolver(PlantTargets.exact(poseCommand))
                .build();
    }

    public void setPose(Pose pose) {
        poseCommand.set(Objects.requireNonNull(pose, "pose"));
    }

    public Status status() {
        return new Status(poseCommand.snapshot(plant.snapshot()));
    }

    public void update(LoopClock clock) {
        plant.update(clock);
    }

    public void stop() {
        plant.stop();
    }

    private double targetFor(Pose pose) {
        switch (pose) {
            case STOW:
                return stowTarget;
            case INTAKE:
                return intakeTarget;
            case SCORE:
                return scoreTarget;
            default:
                throw new IllegalStateException("Unhandled Wrist.Pose: " + pose);
        }
    }
}
```

After selecting the active profile slice, the composition root has one ordinary construction step:

```java
wrist = new Wrist(hardwareMap, profile.wrist);
```

The mechanism copies and validates that slice before its own hardware lookup, constructs the Plant,
keeps it private, and composes `poseCommand.snapshot(plant.snapshot())` instead of rebuilding common
Plant status fields. `SemanticScalarCommand` maps and validates before publishing one immutable
semantic/numeric request. The one-field `Status` view gives ordinary clients wrist vocabulary while
retaining `plantSnapshot()` for advanced diagnostics. After each `plant.update(clock)`, the backing
Plant snapshot reflects the
bounds and guards actually applied. A failed mapping cannot expose a pose whose matching command was
never accepted, and direct calls and Tasks share that same setter. This open-loop servo can prove
that the request selected its target, but not physical arrival; add authoritative feedback before
using `currentRequestAtTarget()` as readiness.

### TeleOp interaction

```java
bindings.onRise(pads.p1().a(), () -> wrist.setPose(Wrist.Pose.INTAKE));
bindings.onRise(pads.p1().b(), () -> wrist.setPose(Wrist.Pose.SCORE));
bindings.onRise(pads.p1().y(), () -> wrist.setPose(Wrist.Pose.STOW));
```

### Auto interaction

```java
Task wristToScore = Tasks.runOnce(() -> wrist.setPose(Wrist.Pose.SCORE));
```

There is no need for a task just to represent the normal wrist state. The task is only a convenient
way for Auto to invoke the same intent.

### Why this is the recommended design

- the public API is tiny
- the subsystem is the only writer
- TeleOp and Auto use the same method
- the normal state is stored as state, not as a queue entry

---

## Detailed example 2: lift with external height sensor (pattern 2: scalar regulation)

### The problem

A lift is driven by motor power, but the desired quantity is **height in inches**. A distance sensor
or potentiometer gives the measured height.

This is not a spatial problem. It is one scalar measured variable driven toward a scalar target.

The clean implementation is a **regulated position plant**: a raw actuator command (motor power),
a controller/regulator, and a feedback source packaged into one Plant owned by the subsystem.

### Recommended subsystem shape

The mechanism receives only FTC resources and its data-only profile slice. It copies and validates
that slice before its own hardware effects, then constructs the feedback source and regulated Plant
together so they cannot be wired to different owners:

```java
public final class Lift {
    private final PositionPlant liftPlant;
    private final double minimumHeightIn;
    private final double maximumHeightIn;

    public Lift(HardwareMap hardwareMap, LiftConfig config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        LiftConfig snapshot = Objects.requireNonNull(config, "config").copy();
        minimumHeightIn = snapshot.minimumHeightIn;
        maximumHeightIn = snapshot.maximumHeightIn;

        ScalarSource measuredHeightIn =
                FtcSensors.distanceIn(hardwareMap, snapshot.heightSensorName);
        liftPlant = FtcActuators.plant(hardwareMap)
                .motor(snapshot.motorName, snapshot.direction)
                .position()
                .regulated()
                    .nativeFeedback(measuredHeightIn)
                .nonPeriodic()
                    .bounded(minimumHeightIn, maximumHeightIn)
                    .nativeUnits()
                    .alreadyReferenced()
                .positionTolerance(snapshot.positionToleranceIn)
                .setpointFromAppliedTarget()
                .feedbackFromPid(snapshot.kP, snapshot.kI, snapshot.kD)
                .feedforwardFromLift(snapshot.kG)
                .outputPowerLimitedTo(snapshot.maximumPower)
                .targetFromNewCommand(minimumHeightIn)
                .build();
    }

    public void setTargetHeightIn(double heightIn) {
        if (!Double.isFinite(heightIn)) {
            throw new IllegalArgumentException("heightIn must be finite");
        }
        double boundedHeightIn = Math.max(minimumHeightIn, Math.min(heightIn, maximumHeightIn));
        liftPlant.commandTarget().set(boundedHeightIn);
    }

    public PositionPlantSnapshot status() {
        return liftPlant.snapshot();
    }

    public void update(LoopClock clock) {
        liftPlant.update(clock);
    }

    public void stop() {
        liftPlant.stop();
    }
}
```

For the preset values below, this profile declares a range that includes them, such as
`minimumHeightIn = 0.0` and `maximumHeightIn = 30.0`. The composition root constructs the owner,
not the Plant:

```java
lift = new Lift(hardwareMap, profile.lift);
```

### TeleOp interaction

```java
bindings.onRise(pads.p2().dpadUp(), () -> lift.setTargetHeightIn(24.0));
bindings.onRise(pads.p2().dpadRight(), () -> lift.setTargetHeightIn(16.0));
bindings.onRise(pads.p2().dpadDown(), () -> lift.setTargetHeightIn(0.0));
```

### Auto interaction

```java
Task liftToHigh = Tasks.sequence(
        Tasks.runOnce(() -> lift.setTargetHeightIn(24.0)),
        Tasks.waitUntil(() -> lift.status().atCommandTarget(), 2.0)
);
```

### Why this is the recommended design

- Auto and TeleOp share the same intent method
- the subsystem constructs and owns one authoritative Plant and its feedback source
- the profile's declared bounds, the public clamp, and the shown presets describe the same range
- `status()` reuses the complete position-Plant facts without a parallel field list
- `atCommandTarget()` rejects stale arrival evidence if the command changed before the next heartbeat
- the rest of the robot never needs to know about the PID internals

### What not to do

Do **not** make the public API expose raw PID pieces like:

```java
liftPid.update(...)
liftMotor.setPower(...)
```

Those are subsystem internals, not robot-level vocabulary.

---

## Detailed example 3: intake with beam break and feed pulses (pattern 3: event / classification supervision)

This example shows the main reason Sushi separates subsystems from supervisors.

### The problem

An intake mechanism has:

- a motor that runs continuously while intake is enabled
- a beam break that reports whether a game piece is present
- a feeder servo that occasionally needs a timed or sensor-gated pulse

The intake motor is a simple base output. The feeder pulse is a temporary override. The beam break
is an event/classification signal, not a continuous measured variable.

### Recommended split

- **Intake subsystem**
  - owns the motor plant, feeder plant, beam break source, and feed queue
  - computes the final targets each loop

- **Intake supervisor**
  - owns intake-enable state, feed requests, and policy like "only queue one feed pulse at a time"
  - exposes a small status snapshot

### Recommended subsystem shape

```java
public final class Intake {
    public static final class Status {
        private final boolean piecePresent;
        private final boolean feedActive;
        private final boolean feedPending;

        public Status(boolean piecePresent, boolean feedActive, boolean feedPending) {
            this.piecePresent = piecePresent;
            this.feedActive = feedActive;
            this.feedPending = feedPending;
        }

        public boolean piecePresent() {
            return piecePresent;
        }

        public boolean feedActive() {
            return feedActive;
        }

        public boolean feedPending() {
            return feedPending;
        }
    }

    private final Plant intakePlant;
    private final Plant feederPlant;
    private final BooleanSource piecePresent;
    private final OutputTaskRunner feedQueue = Tasks.outputQueue(0.0);
    private final double collectPower;
    private final double feedPosition;
    private final double feedDurationSec;

    private boolean lastPiecePresent = false;

    public Intake(HardwareMap hardwareMap, IntakeConfig config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        IntakeConfig snapshot = Objects.requireNonNull(config, "config").copy();
        collectPower = snapshot.collectPower;
        feedPosition = snapshot.feedPosition;
        feedDurationSec = snapshot.feedDurationSec;

        piecePresent = FtcSensors.digitalLow(hardwareMap, snapshot.beamBreakName);

        intakePlant = FtcActuators.plant(hardwareMap)
                .motor(snapshot.intakeMotorName, snapshot.intakeDirection)
                .power()
                .targetFromNewCommand(0.0)
                .build();

        PlantTargetResolver finalFeederTarget =
                PlantTargets.overlay(snapshot.feederIdlePosition)
                        .add("feedPulse", feedQueue.activeSource(), feedQueue)
                        .build();
        feederPlant = FtcActuators.plant(hardwareMap)
                .servo(snapshot.feederServoName, snapshot.feederDirection)
                .position()
                .nonPeriodic()
                    .bounded(0.0, 1.0)
                    .nativeUnits()
                .targetFromResolver(finalFeederTarget)
                .build();
    }

    public void setIntakeEnabled(boolean enabled) {
        intakePlant.commandTarget().set(enabled ? collectPower : 0.0);
    }

    public void requestFeedPulse() {
        feedQueue.enqueue(Tasks.outputForSeconds(
                "feedOne",
                feedPosition,
                feedDurationSec));
    }

    public void cancelTransientActions() {
        feedQueue.cancelAndClear();
    }

    public Status status() {
        return new Status(
                lastPiecePresent,
                feedQueue.hasActiveTask(),
                feedQueue.backlogCount() > 0);
    }

    public void update(LoopClock clock) {
        lastPiecePresent = piecePresent.getAsBoolean(clock);
        feedQueue.update(clock);

        intakePlant.update(clock);
        feederPlant.update(clock);
    }

    public void stop() {
        CleanupActions.attemptAll(
                feedQueue::cancelAndClear,
                intakePlant::stop,
                feederPlant::stop);
    }
}
```

The final feeder overlay is constructed once, in the mechanism constructor. Each loop only advances
the queue and then the Plants. The queue, resolver, sensor source, and Plants remain private; callers
see semantic requests and a small status snapshot.

This example deliberately treats holding either feeder-servo position at FTC STOP as mechanically
safe. A standard-servo Plant's terminal `stop()` re-commands its last applied position, while the
power Plant submits zero. Neither stop rewrites its resolver, and neither Plant will realize another
target afterward. Cancelling the independently owned queue remains necessary for truthful Task
cleanup, not to disable the stopped feeder Plant. If a real mechanism must retract before shutdown,
make retraction a bounded cooperative Task before STOP or use a hardware adapter with an explicit
safe stop behavior. Do not imply that merely setting an idle target inside `stop()` applies it.

```java
intake = new Intake(hardwareMap, profile.intake);
```

### Recommended supervisor shape

```java
public final class IntakeSupervisor {
    public static final class Status {
        private final boolean intakeEnabled;
        private final boolean piecePresent;
        private final boolean feedPending;

        public Status(boolean intakeEnabled, boolean piecePresent, boolean feedPending) {
            this.intakeEnabled = intakeEnabled;
            this.piecePresent = piecePresent;
            this.feedPending = feedPending;
        }

        public boolean intakeEnabled() {
            return intakeEnabled;
        }

        public boolean piecePresent() {
            return piecePresent;
        }

        public boolean feedPending() {
            return feedPending;
        }
    }

    private final Intake intake;
    private boolean intakeEnabled = false;
    private Status lastStatus = new Status(false, false, false);

    public IntakeSupervisor(Intake intake) {
        this.intake = Objects.requireNonNull(intake, "intake");
    }

    public void setIntakeEnabled(boolean enabled) {
        intakeEnabled = enabled;
        intake.setIntakeEnabled(enabled);
    }

    public void requestFeedOne() {
        if (!intake.status().feedPending()) {
            intake.requestFeedPulse();
        }
    }

    public void cancelTransientActions() {
        intake.cancelTransientActions();
    }

    public void update(LoopClock clock) {
        Intake.Status intakeStatus = intake.status();
        lastStatus = new Status(
                intakeEnabled,
                intakeStatus.piecePresent(),
                intakeStatus.feedPending()
        );
    }

    public Status status() {
        return lastStatus;
    }
}
```

The important point of this example is the boundary:

- supervisor owns requests and queueing policy
- subsystem exposes semantic feed/cancel operations but keeps the queue, feeder target resolver, and
  Plant update order private

### TeleOp interaction

```java
bindings.mirrorOnChange(pads.p1().rightBumper(), intakeSupervisor::setIntakeEnabled);
bindings.onRise(pads.p1().a(), intakeSupervisor::requestFeedOne);
```

### Auto interaction

```java
Task acquirePiece = Tasks.sequenceOnCompletion(
        Tasks.runOnce(() -> intakeSupervisor.setIntakeEnabled(true)),
        Tasks.waitUntil(() -> intakeSupervisor.status().piecePresent(), 1.5),
        Tasks.runOnce(() -> intakeSupervisor.setIntakeEnabled(false))
);
```

This graph intentionally disables intake after either successful detection or a natural timeout,
so it uses the explicitly named completion-continuing sequence and retains the timeout outcome.
Direct cancellation still skips the later disable Task. The active operation's cancellation path or
the intake owner's managed cleanup must therefore clear any persistent request that must be safe on
abort or STOP; `sequenceOnCompletion(...)` is not a `finally` block.

### Why this is the recommended design

- event logic stays out of the subsystem's final target calculation
- the queue stays encapsulated; callers request feed actions instead of manipulating plant targets
- Auto and TeleOp both reuse the same supervisor methods and status
- temporary overrides do not break the target-resolver ownership rule

---

## Detailed example 4: shooter or scorer API design

Mechanisms with more policy usually benefit from a supervisor-facing API rather than exposing raw
queue details.

### Recommended public shape

```java
public interface ShooterApi {
    void setFlywheelEnabled(boolean enabled);
    void setIntakeEnabled(boolean enabled);
    void requestSingleShot();
    void beginShooting();
    void endShooting();
    void cancelTransientActions();
    ShooterStatus status();
}
```

This is better than exposing only methods like:

- `toggleFlywheel()`
- `setShootHeld(boolean)`
- `feedQueue().enqueue(...)`

because Auto and TeleOp can both understand the API without knowing button semantics or queue
internals.

### TeleOp interaction

```java
bindings.onRise(pads.p2().rightBumper(), () -> shooter.setFlywheelEnabled(true));
bindings.onRise(pads.p2().leftBumper(), () -> shooter.setFlywheelEnabled(false));
bindings.onRise(pads.p2().a(), shooter::requestSingleShot);
bindings.onRise(pads.p2().b(), shooter::beginShooting);
bindings.onFall(pads.p2().b(), shooter::endShooting);
```

### Auto interaction

```java
Task waitUntilReady = Tasks.waitUntil(
        () -> shooter.status().readyToShoot(),
        1.5
);

Task fireOne = Tasks.sequence(
        Tasks.runOnce(() -> shooter.setFlywheelEnabled(true)),
        Tasks.branchOnOutcome(
                waitUntilReady,
                Tasks.sequence(
                        Tasks.runOnce(shooter::requestSingleShot),
                        Tasks.waitUntil(() -> shooter.status().shotComplete(), 1.0)
                ),
                Tasks.runOnce(() -> shooter.setFlywheelEnabled(false))
        )
);
```

The shot is requested only after readiness succeeds. If readiness times out, the explicit fallback
turns the flywheel request off instead. Direct cancellation does not start either branch, so the
owner of a larger routine must still release any persistent shooter request that routine owns.

### Why this is the recommended design

- the robot code reuses intent names, not button shapes
- Auto does not need to mimic a driver's hold semantics
- status snapshots provide clean, inspectable wait conditions

### Keep standard software control inside realization

A power-based flywheel declares one complete control model inside its private Plant recipe. Robot
capabilities expose shooter intent and status, never controller pieces:

```java
flywheel = FtcActuators.plant(hardwareMap)
        .motor(config.motorName, config.direction)
        .velocity()
        .regulated()
            .internalEncoder()
        .bounded(0.0, config.maximumRpm)
        .scaleToNative(config.ticksPerRpm)
        .velocityTolerance(config.toleranceRpm)
        .setpointFromAccelerationLimitedProfile(config.maximumRpmPerSec)
        .feedbackFromPid(config.kP, config.kI, config.kD)
        .feedbackIntegralLimitedTo(-0.15, 0.15)
        .feedforwardFromMotion(config.kS, config.kV, config.kA)
        .outputPowerLimitedTo(0.0, config.maximumPower)
        .targetFromNewCommand(0.0)
        .build();
```

The Plant owns the setpoint profile, PID, feedforward, final output policy, state reset, completion,
and one hardware write. Match TeleOp and Auto still request shooter intent and read status. The
ready-made `FtcPanelsTuners.velocityControl(...)` workflow derives either this exact standard
controller or an FTC device-managed controller from a fresh Plant built by the same canonical
recipe. See the [`control tuning workflow`](<../testing-calibration/Control Tuning Workflow.md>) for
the tuning order, experiment range, metrics, and evidence contract.

For a genuinely nonlinear or table-driven complete law, realization may use the explicit advanced
`controlFromCustomRegulator(...)` seam. That custom owner must provide its own typed tuning contract;
the obsolete peer PIDF/setpoint-feedforward factories are not a parallel construction path.

---

## Detailed example 5: drive is special

Drive is the one place where TeleOp and Auto usually do not use the exact same API shape. That is
okay.

### TeleOp drive

TeleOp drive usually wants a continuous manual `DriveSource`:

```java
DriveSource manual = new GamepadDriveSource(
        pads.p1().leftX(),
        pads.p1().leftY(),
        pads.p1().rightX(),
        GamepadDriveSource.Config.defaults()
).scaledWhen(pads.p1().rightBumper(), 0.35, 0.20);
```

That manual source may then be wrapped with overlays or assist logic.

When those overlays depend on robot state, put them in a robot-owned **drive-assist service** rather
than hiding small latches or helper booleans in the composition root. For example, a `shootBrace`
pose-hold that depends on both scoring state and stick idleness is not just "drive wiring." It is
robot policy, so it should live beside targeting/services/supervisors, not inside `MyRobot`.

A typical split looks like:

- controls owner: maps sticks/buttons into intent sources
- drive-assist service: combines manual drive + robot-state-driven overlays
- `MecanumDrivebase`: owns the coordinated drive outputs and executes the final command

That keeps the control script out of the robot container and makes the drive path reusable and easier
to debug.

### Auto drive

Auto usually wants either:

- a Sushi `DriveGuidanceTask`, or
- a `RouteTask<RouteT>` built around a project-specific external follower adapter (Road Runner / Pedro / other)

When you do use `DriveGuidanceTask`, prefer semantic references over tag-specific public targets.
For example, define “slot 4 face” or “speaker aim point” once, then let guidance solve that
reference from field pose, live AprilTags, or both.

Conceptually:

```java
Task goToBackstage = drivePlan.task(drivebase, cfg);
Task followCyclePath =
        RouteTasks.follow("cycle", roadRunnerAdapter, cyclePath, routeTimeoutSec);
```

The adapter is project-specific, while Sushi provides the generic `RouteFollower<RouteT>` /
`RouteTask<RouteT>` seam so the rest of your Auto can stay inside the normal Task vocabulary.

Each `RouteTask` retains the `RouteExecution` returned when it starts. Ordinary code still uses the
same one-line `RouteTasks.follow(...)` call. Code that genuinely needs the terminal reason can keep
the returned `RouteTask<RouteT>` and read `getRouteStatus()`; it should not inspect a vendor busy
flag or call raw follower lifecycle methods.

Route construction has one public owner: `RouteTasks`. Give every route a nonblank diagnostic name
and pass its finite, positive Task timeout directly. If robot policy deliberately assigns its outer
Task-level budget elsewhere, use `followWithoutTaskTimeout(...)` or its parallel start-time variant
instead of encoding that choice with zero, a negative number, a non-finite value, or another
sentinel. “Without Task timeout” does not disable a follower's own timeout/stall result.

Build fixed routes eagerly. If a fallback or return route needs the current pose or current vision
selection at the moment its phase begins, pass a quick lambda or method reference to the explicitly
named `RouteTasks.followBuiltAtStart(...)` factory. Keep geometry and sensor interpretation in the
robot path factory; do not move them into the follower adapter or write a custom Task at each call
site.

For a stateful adapter such as Pedro, register one robot-owned service during program
configuration. Author one data-only Config, then cross one production effect boundary:

```java
PedroPathingRuntime.Config pedroConfig = PedroPathingRuntime.Config.defaults();
pedroConfig.predictor = robotPinpointConfig;
pedroConfig.followerConstants = robotFollowerConstants;
pedroConfig.mecanumConstants = robotMecanumConstants;
pedroConfig.pathConstraints = robotPathConstraints;
pedroConfig.fieldTransform = robotFieldTransform;

PedroPathingRuntime pedro = PedroPathingRuntime.create(hardwareMap, pedroConfig);
```

The runtime snapshots and validates the complete Config before hardware lookup, Pinpoint reset,
motor output, Follower construction, or Pedro-global mutation. It then privately owns the captured
Pinpoint, Follower, Mecanum, constraint, and transform graph. Do not retain a Config or nested Pedro
constants as a live-tuning channel; edit checked-in data and reconstruct the OpMode instead.

The checked-in basic reference instead keeps its complete example configuration in one local,
fresh profile and gives that short-lived value to its sole composition-root construction path:

```java
BasicPedroAutoRobot robot = new BasicPedroAutoRobot(
        program,
        hardwareMap,
        BasicPedroProfile.current());
```

`BasicPedroProfile.current()` returns a fresh `PedroPathingRuntime.Config`, a fresh owner-local
intake Config, and `allowRobotMotion = false`. The root checks that permission and the intake-versus-
drive motor ownership collision before effects, creates and immediately registers the runtime, then
constructs and registers the intake. Each long-lived owner snapshots only its own active Config, so
the root retains no mutable profile and a later intake failure receives managed drive cleanup.

The profile's explicit Mecanum `maxPower = 0.25` is initial software data, not a durable route cap:
Pedro 2.1.2 restores the Follower's separate `globalMaxPower` to `1.0` when `followPath(...)`
starts, and ordinary managed Sushi route callers cannot currently set that persistent limit. The
false permission blocks route motion in the checked-in construction; `@Disabled` separately hides
its FTC entry. Physical route qualification remains blocked pending a focused integration
improvement. After that control exists, every runtime, intake, route, placement, and STOP fact still
requires physical review before enabling.

The registered service owns localization first and the recurring adapter heartbeat second. It owns
`pedro.motionPredictor().update(clock)` followed by `pedro.driveAdapter().update(clock)` on every
Auto loop, then its final drive stop. The routine still uses
`RouteTasks.follow("cycle", pedro.driveAdapter(), route, routeTimeoutSec)` and guidance Tasks
normally; the adapter makes their same-cycle update calls harmless. The runtime's passive Pedro
localizer reads the same current-cycle predictor that Sushi localization updates, so this also
avoids a second odometry owner without adding another scheduler to student Auto code.

Pinpoint construction requests its one reset without sleeping. Until a completed `READY` poll
publishes current finite pose and velocity, the Pedro heartbeat throws through its fail-stop owner
instead of driving on a commanded rebase or stale sample. Keep the robot stationary during that
reset/calibration interval.

The adapter also classifies each route's terminal state during that owned heartbeat, while the
evidence still exists. Robot code must use the adapter and Sushi Task cancellation seams for
start, replacement, interruption, and cancellation; raw Pedro Follower lifecycle calls are
unsupported because they bypass the execution status. The runtime exports no raw Follower: use
`pathBuilder()` for route construction and `currentPedroPose()` for one defensive, no-poll snapshot
when start-time geometry needs the Follower's cached Pedro pose.

The public `PedroPathingDriveAdapter(completedFollower)` constructor is an advanced seam for a
custom or portable host that has already constructed a complete Follower and will route its
lifecycle through the adapter; it does not acquire Pinpoint or drivetrain hardware. Pedro's
generated tuning OpModes use a different package-local factory for a native Follower because those
exclusive tools require their own `PinpointLocalizer` and raw
Follower heartbeat. Neither seam is a parallel production construction path.

After a valid Config crosses the effect boundary, SDK/vendor construction can still fail after
partial effects. The runtime best-effort breaks a successfully constructed drivetrain when a later
step fails, but it cannot stop a Mecanum constructor that never returned a handle, close/rollback
Pinpoint, or undo vendor statics already touched by a failing Follower. Treat such a failure as
terminal for that OpMode. Software validation also cannot prove motor identity/direction, pod
placement/readiness, tuning stability, field alignment, route clearance, stopping distance, or
physical STOP; those remain adopting-robot evidence.

### What still stays common

Even though drive is special, the rest of the robot should still follow the same rules:

- mechanisms are exposed through intent + status APIs
- Auto routes call the same mechanism intents as TeleOp
- route interruption should use Sushi cancellation seams
- exact-success prerequisite gating comes from ordinary `Tasks.sequence(...)`, while
  continue/fallback/abort decisions remain visible as robot-owned strategy
- direct routine cancellation never starts a fallback
- failure cleanup clears only capability requests owned by that routine phase

Truthful route status deliberately does not choose that strategy. Ordinary `Tasks.sequence(...)`
stops after a non-success Task outcome, but it cannot distinguish every precise route status or
select a fallback. Each robot routine still applies its own explicit route-failure policy.

### Route-policy inputs with shared mechanism intents

Construct fresh status-bearing route Tasks and semantic capability Tasks, then give them to one
robot-owned routine helper/coordinator. A generic exact-success sequence can prevent the lift and
shot from starting after a non-success route outcome, but it does not choose a timeout fallback or
distinguish route statuses that need different strategy.

```java
RouteTask<YourRoute> preloadRoute = RouteTasks.follow(
        "preload",
        routeAdapter,
        preloadPath,
        routeTimeoutSec
);
```

Build the numeric lift move separately, including its timeout outcome:

```java
Task raiseLift = Tasks.sequence(
        Tasks.runOnce(() -> lift.setTargetHeightIn(24.0)),
        Tasks.waitUntil(() -> lift.status().atCommandTarget(), 1.5)
);

Task scorePreload = Tasks.branchOnOutcome(
        raiseLift,
        Tasks.runOnce(shooter::requestSingleShot),
        Tasks.runOnce(() -> lift.setTargetHeightIn(0.0))
);
```

The robot policy selects `scorePreload` only after `preloadRoute.getRouteStatus()` is `COMPLETED`.
Within that scoring Task, the shot starts only after the lift reaches its target; a lift timeout
selects the example's explicit return-to-low request instead. An adopting robot must choose the
fallback that is physically safe for its mechanism. Cancellation-like results abort, and direct
cancellation never launches recovery. If the route or scoring phase owns a transient request, its
cancellation path clears that request through the robot capability; it does not reset unrelated
mechanism intent. Drive may be special internally, but the routine still uses the same small Task and
capability vocabulary.

---

## How to decide whether to use a task, a queue, or plain state

This is one of the most important design decisions in Sushi.

### Use plain subsystem state when the command is long-lived

Good examples:

- wrist pose
- lift target height
- flywheel enabled
- intake enabled

These are usually last-request-wins state variables.

### Use an output queue when the command is a temporary override

Good examples:

- feeder pulse for 0.12 s
- eject for 0.30 s
- open gripper briefly
- run until beam break clears

These are usually time-shaped or condition-shaped actions layered on top of a base state.

### Use a task when you need a reusable routine over time

Good examples:

- set lift height, then wait until at target
- run intake until piece is seen
- follow a route, apply its explicit result policy, and score only after confirmed completion

A task is often the right thing for Auto, or for a reusable TeleOp macro. It is usually **not** the
right place to store the normal steady-state target of a mechanism.

### Use deadline composition for bounded work beside an owner

When several Tasks must all finish, use `Tasks.parallelAll(...)`. It waits for every child and
succeeds only if every child succeeds; matching abnormal outcomes remain exact and mixed abnormal
outcomes report `UNKNOWN`. When one Task owns the useful lifetime of bounded companion work, use
`Tasks.parallelDeadline(deadline, companions...)`. For example, a route can own a cancellation-safe
collection macro: route completion and outcome end the group, while collection finishing early
does not end the route.

The composite does not invent mechanism cleanup; it only calls each companion's `cancel()`. Do not
use `sequence(enable, wait, disable)` as a companion because cancellation skips the future disable
step. Put persistent intake/flywheel/aim requests in capability or service state. Use a bounded
companion only when its own active cancellation restores the intended caller-selected state.

### A simple rule of thumb

- **Normal steady state** lives as subsystem state.
- **Temporary override** lives in an output queue.
- **Reusable routine over time** lives in a task.

That rule keeps most mechanisms simple.

---

## What status snapshots should contain

A status snapshot should help callers answer two questions quickly:

1. **What did I ask for?**
2. **What is happening right now?**

Good status fields are things like:

- desired pose / target height / mode
- measured position or velocity
- whether the mechanism is at target
- whether a piece is present
- whether a transient action is active
- whether an assist is ready or blocked

Do not copy common scalar-Plant facts into every capability. `Plant.snapshot()` already captures
the current command value (when one exists), requested/applied targets, resolution/status,
feedback/measurement, errors, and arrival. `PositionPlant.snapshot()` adds range, periodicity, and
reference facts. A numeric position or velocity capability can return that snapshot directly.

For named points or modes, the mechanism composes them with Plant facts through
`SemanticScalarSnapshot<S, P>`, then exposes a small capability-shaped status view. The semantic
request stays authoritative and is never inferred from a number. The view projects ordinary domain
facts without copying mutable state and may retain `plantSnapshot()` for advanced diagnostics. A
capability with extra evidence—piece presence, per-wheel balance, transient state, debounced
readiness, or a failure reason—adds those facts there. Status snapshots should still **not** expose
mutable internal owners.

Bad external status design:

- return raw `OutputTaskRunner`
- return raw `Plant`
- make callers inspect several booleans spread across many objects

Good external status use:

```java
BasicLift.Status status = lift.status();
telemetry.addData("height", status.requestedHeight());
telemetry.addData("requestedIn", status.requestedPositionIn());
telemetry.addData("appliedIn", status.appliedPositionIn());
telemetry.addData("measuredIn", status.measuredPositionIn());
telemetry.addData("referenced", status.referenced());
telemetry.addData("atTarget", status.atTarget());
```

The mechanism still uses `SemanticScalarCommand` and `SemanticScalarSnapshot` to keep request
identity, numeric mapping, and Plant evidence coherent. Ordinary callers get one-hop capability
names; `status.plantSnapshot()` deliberately exposes the immutable generic facts only when a
diagnostic needs them. The enum is the named position, `setHeight(...)` is a persistent non-waiting
request, and `moveTo(...)` returns a fresh single-use feedback Task. Config and the mechanism mapper
own the numeric coordinates; do not add a second `NamedPosition` abstraction or parallel `toHigh`
alias. Named velocities use the same pattern. A grouped Plant's `atTarget()` remains aggregate;
per-wheel readiness belongs in capability status.

---

## Recommended managed update order

A `RobotProgram` applies this fixed active order:

1. advance `LoopClock`
2. update declared services such as sensors, acquisition/localization lanes, and required vendor heartbeats
3. update input bindings
4. update the private Task runner so it publishes this cycle's intent
5. update declared outputs and the optional source-driven drive in declaration order
6. invoke additive presenters
7. commit telemetry once

The important thing is that declarations make this order easy to inspect. In particular, do not
update a Plant and then write the target that you expected it to apply in that same cycle. A
third-party follower whose supported lifecycle requires an earlier heartbeat belongs in a service;
its adapter deduplicates a same-cycle Task call rather than creating a second timebase.

If a robot grows, use declaration helper methods so `configure(program)` still reads like a
composition root instead of becoming a 500-line control script.

---

## Anti-patterns to avoid

### 1. OpMode writes plant targets directly

Bad:

```java
if (gamepad1.a) {
    liftPlant.commandTarget().set(0.7); // still wrong: OpMode bypasses mechanism policy
}
```

Why it hurts:

- TeleOp and Auto cannot share the same vocabulary
- debugging gets harder because plant ownership is unclear
- another subsystem/supervisor may already think it owns that plant

### 2. Tasks hold the normal steady-state target of a mechanism

Bad idea:

- enqueue a task just to represent the wrist's normal pose forever

Why it hurts:

- queues are best for temporary overrides and routines
- the normal state becomes harder to inspect and replace

### 3. Public API is only button-shaped

Bad idea:

- `toggleFlywheel()` is the only flywheel method
- `setShootHeld(...)` is the only shooting method

Why it hurts:

- Auto has to imitate a driver instead of expressing intent directly

### 4. Exposing raw queue internals as the normal API

Bad idea:

```java
robot.shooter().feedQueue().enqueue(...)
```

Why it hurts:

- callers need to know queue semantics
- subsystem invariants become easier to break
- debugging spreads across too many objects

---

## The checklist for a new mechanism

When adding a mechanism, ask these questions in order.

### 1. Which behavior pattern does the mechanism use internally?

- local target?
- scalar regulation?
- event/classification supervision?
- spatial guidance?
- route integration?

### 2. What is the smallest useful intent vocabulary?

Try to phrase it as:

- persistent goals
- momentary requests
- mode/enable flags
- cancel/reset

### 3. What small status snapshot will outside code need?

Usually that includes:

- desired target or mode
- current measured value or readiness
- whether the mechanism is done / ready / blocked

### 4. Who owns the target resolvers and Plant updates?

Make sure exactly one place computes the final plant target.

### 5. Should this be plain state, an output queue, or a reusable task?

Use the rules from the previous section.

If you can answer those five questions clearly, the mechanism will usually fit the framework well.

---

## Final recommendation

If you are unsure how to structure a new robot, start here:

- expose **intent methods + status snapshots**
- keep **subsystems as the owners of target resolvers and Plant updates**
- keep **supervisors as the policy layer**
- use the **behavior patterns** to choose the internals
- let **TeleOp bindings and Auto routines call the same public mechanism vocabulary**

That design keeps the number of objects small, makes reuse between TeleOp and Auto natural, and
keeps debugging focused on the right level of abstraction.
