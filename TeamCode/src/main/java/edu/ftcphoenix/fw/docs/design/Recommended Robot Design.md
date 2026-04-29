# Recommended Robot Design

## Recommended top-level split

For larger robots, use this ownership pattern:

- **framework lanes** for stable FTC-side systems that recur year to year
  - example: `FtcMecanumDriveLane`
  - example: `FtcWebcamAprilTagVisionLane` or `FtcLimelightAprilTagVisionLane` (behind the `AprilTagVisionLane` seam)
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

### Why this matters

This keeps stable framework code reusable across seasons without turning the framework into a giant base robot class. It also keeps each robot's control scheme and game logic easy to find, instead of scattering it across convenience helpers and subsystem constructors.

Before reading the rest of this file, read [`Framework Lanes & Robot Controls`](<Framework Lanes & Robot Controls.md>). That document defines the ownership vocabulary used throughout the framework and shows how to build a robot from scratch without copying an older robot.

This document describes Phoenix's recommended design for robot code that will be shared between
TeleOp and Auto.

The short version is:

- **TeleOp bindings and Auto routines should talk to the robot through the same small intent API.**
- **Subsystems should own their Plant target sources and update order.**
- **Supervisors should own policy, timing, requests, and queueing.**
- **Status snapshots should be the normal way to observe a mechanism from the outside.**
- **Use the behavior lane that matches the problem instead of forcing everything into one pattern.**

If you keep those rules, a robot can stay understandable even as it grows.

## Read this with

- [`Framework Overview`](<../getting-started/Framework Overview.md>)
- [`Beginner's Guide`](<../getting-started/Beginner's Guide.md>)
- [`Loop Structure`](<../core-concepts/Loop Structure.md>)
- [`Supervisors & Pipelines`](<Supervisors & Pipelines.md>)
- [`Tasks & Macros Quickstart`](<Tasks & Macros Quickstart.md>)
- [`Framework Principles`](<../../Framework Principles.md>)

Most examples in this document use real Phoenix types. A few route-library snippets are
intentionally conceptual, because the exact adapter API depends on the library you choose to wrap.

All Java snippets in this document use Java 8-compatible syntax so they match the FTC project
environment. That means examples avoid newer language features such as records and switch
expressions.

---

## The main recommendation in one picture

Think of a robot as four layers:

1. **OpMode layer**
   - TeleOp bindings
   - Auto routines / route tasks

2. **Intent / capabilities layer**
   - robot-owned capability families such as `gamePiece()` or `targeting()`
   - small public methods like `setTargetHeightIn(...)`, `setIntakeEnabled(...)`,
     `requestSingleShot()`
   - small status snapshots like `Lift.Status`, `ShooterSupervisor.Status`

3. **Mechanism layer**
   - subsystems own plants, sensors, and final target calculation
   - supervisors own policy, timing, state machines, and queueing

4. **Framework lane internals**
   - local setpoints
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

## The five behavior lanes

Phoenix stays simplest when you choose the lane that matches the problem instead of forcing every
mechanism into one abstraction.

Use this quick decision rule:

1. **Am I commanding a local actuator setpoint directly?** Use a `Plant`.
2. **Am I regulating one measured scalar toward a setpoint?** Use a scalar controller source.
3. **Am I reacting to an event or classification?** Use `BooleanSource` / `Source<T>` with a
   supervisor or task.
4. **Am I solving a 2D frame-to-target relation?** Use drive guidance or a similar spatial layer.
5. **Am I following an external route package?** Adapt it into Phoenix tasks and status; do not
   duplicate the planner.

### Lane 1: local setpoint

Examples:

- wrist servo pose
- motor position target in ticks
- flywheel velocity target
- raw motor or CR servo power

This is the simplest lane. The mechanism already has a meaningful local command mode. Do not add a
controller loop or a queue unless you really need one.

### Lane 2: scalar regulation

Examples:

- lift height from a potentiometer or distance sensor
- arm angle from an analog sensor
- local turret hold from an encoder

This is still a **local scalar** problem. One measured variable is driven toward one setpoint.
Phoenix's `ScalarControllers` helper packages the common pattern:

```java
ScalarSource liftPower = ScalarControllers.pid(desiredHeightIn, measuredHeightIn, pid);
```

### Lane 3: event / classification supervision

Examples:

- touch sensor for homing
- beam break for game-piece present
- color sensor classifying an object
- current spike indicating a jam or hard stop

These sensors are telling you that something happened, not "how far away" you are. Treat them as
signals that a supervisor or task reacts to.

If the signal needs memory across an explicit window (for example, keep a slot classification until
an encoder boundary pulse), keep that memory as a `Source` using `accumulateUntil(...)` rather than
teaching the FTC boundary adapter about your mechanism. If a supervisor owns that lifecycle
explicitly (mode change, task restart, tester clear, or a state-machine transition), keep
`reset()` on the composed source and call it from the owner instead of forcing every boundary into
a synthetic `BooleanSource`.

### Lane 4: spatial relation guidance

Examples:

- drivetrain goes to a field point
- drivetrain aims at a target while translating
- turret faces a tag
- end effector lines up to a tag-relative point

Today the mature public framework consumer for this lane is
[`Drive Guidance`](<../drive-vision/Drive Guidance.md>). The shared idea is a frame-to-target
relation, but Phoenix only publishes the drive-shaped version until a second real consumer proves a
cleaner general API.

### Lane 5: external route integration

Examples:

- Road Runner
- Pedro Pathing
- a custom route follower you already trust

Phoenix should not reimplement those planners. Wrap them in tasks, let Phoenix own mechanism
supervision around them, and use Phoenix cancellation seams to interrupt them cleanly.

In a single-module codebase, keep that lane-5 code at the edges:

- framework-owned bridges in `fw/integrations/<library>/`
- robot-specific autos/examples in `.../autonomous/<library>/`
- core framework and robot packages depending only on `RouteFollower<RouteT>` / `RouteTask<RouteT>`

---

## What each layer should own

### Robot container / Robot class

The robot container wires everything together and advances the loop in a clear order.

It should usually own:

- hardware construction
- subsystem construction
- supervisor construction
- gamepad bindings
- top-level update order
- shared debug/telemetry formatting

It should usually **not** own:

- low-level plant math
- repeated policy logic
- sensor gating rules
- queue management details for one mechanism

A robot container is the place where it should be obvious what exists on the robot. It is not the
place where each mechanism's detailed behavior should live.

### Subsystem

A subsystem owns the target sources and Plant update order for one mechanism or one tightly-coupled hardware group.

A subsystem should usually own:

- plants and direct hardware-facing objects
- the sensor sources most closely tied to that mechanism
- the mechanism's long-lived desired state
- output queues used as temporary overrides
- the final target source that each Plant follows
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

A supervisor should usually **not** write plants directly. It should decide what should happen,
then let the subsystem remain the owner of the mechanism target sources and Plant update order.

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
        TargetingStatus status(LoopClock clock);
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
    LiftStatus status();
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

## Detailed example 1: wrist poses (lane 1: local setpoint)

This is the simplest pattern and the one beginners should reach for first.

### The problem

A wrist servo has three valid poses:

- `STOW`
- `INTAKE`
- `SCORE`

The wrist is just a local setpoint problem. It does not need a controller loop or an output queue.

### Recommended subsystem shape

```java
public final class Wrist {
    public enum Pose { STOW, INTAKE, SCORE }

    public static final class Status {
        private final Pose desiredPose;
        private final double appliedTarget;

        public Status(Pose desiredPose, double appliedTarget) {
            this.desiredPose = desiredPose;
            this.appliedTarget = appliedTarget;
        }

        public Pose desiredPose() {
            return desiredPose;
        }

        public double appliedTarget() {
            return appliedTarget;
        }
    }

    private final Plant plant;
    private Pose desiredPose = Pose.STOW;
    private double lastAppliedTarget = 0.10;

    public Wrist(Plant plant) {
        this.plant = plant;
    }

    public void setPose(Pose pose) {
        desiredPose = pose;
    }

    public Status status() {
        return new Status(desiredPose, lastAppliedTarget);
    }

    public void update(LoopClock clock) {
        double target;
        switch (desiredPose) {
            case STOW:
                target = 0.10;
                break;
            case INTAKE:
                target = 0.45;
                break;
            case SCORE:
                target = 0.80;
                break;
            default:
                target = 0.10;
                break;
        }

        lastAppliedTarget = target;
        plant.writableTarget().set(target);
        plant.update(clock);
    }
}
```

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

## Detailed example 2: lift with external height sensor (lane 2: scalar regulation)

### The problem

A lift is driven by motor power, but the desired quantity is **height in inches**. A distance sensor
or potentiometer gives the measured height.

This is not a spatial problem. It is one scalar measured variable driven toward a scalar setpoint.

With the current plant design, the clean implementation is a **regulated position plant**: a raw actuator command (motor power), a controller/regulator, and a feedback source packaged into one plant owned by the subsystem.

### Recommended subsystem shape

A typical plant construction would look like this:

```java
PositionPlant liftPlant = FtcActuators.plant(hardwareMap)
        .motor("liftMotor", Direction.FORWARD)
        .position()
        .regulated()
            .nativeFeedback(FtcActuators.PositionFeedback.fromSource(measuredHeightIn))
            .regulator(ScalarRegulators.pid(Pid.withGains(0.12, 0.0, 0.0).setOutputLimits(-0.55, 0.55)))
        .linear()
            .bounded(0.0, 15.0)
            .nativeUnits()
            .alreadyReferenced()
        .positionTolerance(0.50)
        .targetedByDefaultWritable(0.0)
        .build();
```

Then the subsystem can own that plant and expose a robot-level API:

```java
public final class Lift {
    public static final class Status {
        private final double targetHeightIn;
        private final double measuredHeightIn;
        private final boolean atTarget;

        public Status(double targetHeightIn, double measuredHeightIn, boolean atTarget) {
            this.targetHeightIn = targetHeightIn;
            this.measuredHeightIn = measuredHeightIn;
            this.atTarget = atTarget;
        }

        public double targetHeightIn() {
            return targetHeightIn;
        }

        public double measuredHeightIn() {
            return measuredHeightIn;
        }

        public boolean atTarget() {
            return atTarget;
        }
    }

    private final Plant liftPlant;
    private double targetHeightIn = 0.0;

    public Lift(Plant liftPlant) {
        this.liftPlant = liftPlant;
    }

    public void setTargetHeightIn(double heightIn) {
        targetHeightIn = Math.max(0.0, Math.min(heightIn, 30.0));
        liftPlant.writableTarget().set(targetHeightIn);
    }

    public Status status() {
        double measured = liftPlant.getMeasurement();
        boolean atTarget = liftPlant.atTarget();
        return new Status(targetHeightIn, measured, atTarget);
    }

    public void update(LoopClock clock) {
        liftPlant.update(clock);
    }
}
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
        Tasks.waitUntil(() -> lift.status().atTarget(), 2.0)
);
```

### Why this is the recommended design

- Auto and TeleOp share the same intent method
- the subsystem still owns the command target, source composition, and Plant update order
- `status()` gives Auto a clean wait condition
- the rest of the robot never needs to know about the PID internals

### What not to do

Do **not** make the public API expose raw PID pieces like:

```java
liftPid.update(...)
liftMotor.setPower(...)
```

Those are subsystem internals, not robot-level vocabulary.

---

## Detailed example 3: intake with beam break and feed pulses (lane 3: event / classification supervision)

This example shows the main reason Phoenix separates subsystems from supervisors.

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

        public Status(boolean piecePresent, boolean feedActive) {
            this.piecePresent = piecePresent;
            this.feedActive = feedActive;
        }

        public boolean piecePresent() {
            return piecePresent;
        }

        public boolean feedActive() {
            return feedActive;
        }
    }

    private final Plant intakePlant;
    private final Plant feederPlant;
    private final BooleanSource piecePresent;
    private final OutputTaskRunner feedQueue = new OutputTaskRunner(0.0);

    private boolean intakeEnabled = false;
    private boolean lastPiecePresent = false;

    public Intake(Plant intakePlant, Plant feederPlant, BooleanSource piecePresent) {
        this.intakePlant = intakePlant;
        this.feederPlant = feederPlant;
        this.piecePresent = piecePresent;
    }

    public void setIntakeEnabled(boolean enabled) {
        intakeEnabled = enabled;
    }

    public BooleanSource piecePresentSource() {
        return piecePresent;
    }

    public OutputTaskRunner feedQueue() {
        return feedQueue;
    }

    public Status status() {
        return new Status(lastPiecePresent, feedQueue.hasActiveTask());
    }

    public void update(LoopClock clock) {
        lastPiecePresent = piecePresent.getAsBoolean(clock);
        feedQueue.update(clock);

        intakePlant.writableTarget().set(intakeEnabled ? 1.0 : 0.0);
        intakePlant.update(clock);

        double feederCmd = feedQueue.activeSource().choose(feedQueue, ScalarSource.constant(0.0))
                .getAsDouble(clock);
        feederPlant.writableTarget().set(feederCmd);
        feederPlant.update(clock);
    }
}
```

The exact `status()` implementation can vary. In real code you would usually cache whatever values
outside callers need while the subsystem is updating. The important point is the shape: small
external status, target-source ownership inside the subsystem.

### Recommended supervisor shape

```java
public final class IntakeSupervisor {
    public static final class Status {
        private final boolean intakeEnabled;
        private final boolean piecePresent;
        private final boolean feedQueued;

        public Status(boolean intakeEnabled, boolean piecePresent, boolean feedQueued) {
            this.intakeEnabled = intakeEnabled;
            this.piecePresent = piecePresent;
            this.feedQueued = feedQueued;
        }

        public boolean intakeEnabled() {
            return intakeEnabled;
        }

        public boolean piecePresent() {
            return piecePresent;
        }

        public boolean feedQueued() {
            return feedQueued;
        }
    }

    private final Intake intake;
    private boolean intakeEnabled = false;
    private Status lastStatus = new Status(false, false, false);

    public IntakeSupervisor(Intake intake) {
        this.intake = intake;
    }

    public void setIntakeEnabled(boolean enabled) {
        intakeEnabled = enabled;
        intake.setIntakeEnabled(enabled);
    }

    public void requestFeedOne() {
        intake.feedQueue().enqueue(
                Tasks.outputForSeconds("feedOne", 0.90, 0.12)
        );
    }

    public void cancelTransientActions() {
        intake.feedQueue().cancelAndClear();
    }

    public void update(LoopClock clock) {
        lastStatus = new Status(
                intakeEnabled,
                intake.status().piecePresent(),
                intake.feedQueue().backlogCount() > 0
        );
    }

    public Status status() {
        return lastStatus;
    }
}
```

The important point of this example is the boundary:

- supervisor owns requests and queueing policy
- subsystem remains the single owner of the feeder target source and Plant update order

### TeleOp interaction

```java
bindings.mirrorOnChange(pads.p1().rightBumper(), intakeSupervisor::setIntakeEnabled);
bindings.onRise(pads.p1().a(), intakeSupervisor::requestFeedOne);
```

### Auto interaction

```java
Task acquirePiece = Tasks.sequence(
        Tasks.runOnce(() -> intakeSupervisor.setIntakeEnabled(true)),
        Tasks.waitUntil(() -> intakeSupervisor.status().piecePresent(), 1.5),
        Tasks.runOnce(() -> intakeSupervisor.setIntakeEnabled(false))
);
```

### Why this is the recommended design

- event logic stays out of the subsystem's final target calculation
- the queue stays encapsulated; callers request feed actions instead of manipulating plant targets
- Auto and TeleOp both reuse the same supervisor methods and status
- temporary overrides do not break the single-writer rule

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
Task fireOne = Tasks.sequence(
        Tasks.runOnce(() -> shooter.setFlywheelEnabled(true)),
        Tasks.waitUntil(() -> shooter.status().readyToShoot(), 1.5),
        Tasks.runOnce(shooter::requestSingleShot),
        Tasks.waitUntil(() -> shooter.status().shotComplete(), 1.0)
);
```

### Why this is the recommended design

- the robot code reuses intent names, not button shapes
- Auto does not need to mimic a driver's hold semantics
- status snapshots provide clean, inspectable wait conditions

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
- drive lane: owns the drivebase hardware and executes the final command

That keeps the control script out of the robot container and makes the drive path reusable and easier
to debug.

### Auto drive

Auto usually wants either:

- a Phoenix `DriveGuidanceTask`, or
- a `RouteTask<RouteT>` built around a project-specific external follower adapter (Road Runner / Pedro / other)

When you do use `DriveGuidanceTask`, prefer semantic references over tag-specific public targets.
For example, define “slot 4 face” or “speaker aim point” once, then let guidance solve that
reference from field pose, live AprilTags, or both.

Conceptually:

```java
Task goToBackstage = drivePlan.task(drivebase, cfg);
Task followCyclePath = RouteTasks.follow(roadRunnerAdapter, cyclePath, new RouteTask.Config());
```

The adapter is still project-specific, but Phoenix now provides the generic `RouteFollower<RouteT>` / `RouteTask<RouteT>` seam so the rest of your Auto can stay inside the normal task vocabulary.

### What still stays common

Even though drive is special, the rest of the robot should still follow the same rules:

- mechanisms are exposed through intent + status APIs
- Auto routes call the same mechanism intents as TeleOp
- route interruption should use Phoenix cancellation seams

### Example Auto sequence with shared mechanism intents

```java
Task scoreCycle = Tasks.sequence(
        roadRunnerAdapter.follow(preloadPath),
        Tasks.runOnce(() -> lift.setTargetHeightIn(24.0)),
        Tasks.waitUntil(() -> lift.status().atTarget(), 1.5),
        Tasks.runOnce(shooter::requestSingleShot)
);
```

Drive may be special internally, but the rest of the robot still benefits from the same small
vocabulary.

---

## How to decide whether to use a task, a queue, or plain state

This is one of the most important design decisions in Phoenix.

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
- follow a route and then score

A task is often the right thing for Auto, or for a reusable TeleOp macro. It is usually **not** the
right place to store the normal steady-state target of a mechanism.

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

Status snapshots should usually **not** expose every internal object.

Bad external status design:

- return raw `OutputTaskRunner`
- return raw `Plant`
- make callers inspect several booleans spread across many objects

Good external status design:

```java
public final class LiftStatus {
    private final double targetHeightIn;
    private final double measuredHeightIn;
    private final boolean atTarget;
    private final boolean homed;

    public LiftStatus(double targetHeightIn, double measuredHeightIn, boolean atTarget, boolean homed) {
        this.targetHeightIn = targetHeightIn;
        this.measuredHeightIn = measuredHeightIn;
        this.atTarget = atTarget;
        this.homed = homed;
    }

    public double targetHeightIn() {
        return targetHeightIn;
    }

    public double measuredHeightIn() {
        return measuredHeightIn;
    }

    public boolean atTarget() {
        return atTarget;
    }

    public boolean homed() {
        return homed;
    }
}
```

That keeps debugging readable without forcing the rest of the robot to know the internals.

---

## Recommended update order inside the robot container

A typical loop order looks like this:

1. advance `LoopClock`
2. refresh bindings / request intents
3. update supervisors
4. update subsystems
5. update drive / route tasks
6. publish telemetry/debug

The important thing is not the exact wording. The important thing is that the order stays consistent
and easy to read.

If a robot grows, use helper methods so the container still reads like a composition root instead of
becoming a 500-line control script.

---

## Anti-patterns to avoid

### 1. OpMode writes plant targets directly

Bad:

```java
if (gamepad1.a) {
    liftPlant.writableTarget().set(0.7); // still wrong place: OpMode is bypassing mechanism policy
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

### 1. Which lane is the mechanism internally?

- local setpoint?
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

### 4. Who owns the target sources and Plant updates?

Make sure exactly one place computes the final plant target.

### 5. Should this be plain state, an output queue, or a reusable task?

Use the rules from the previous section.

If you can answer those five questions clearly, the mechanism will usually fit the framework well.

---

## Final recommendation

If you are unsure how to structure a new robot, start here:

- expose **intent methods + status snapshots**
- keep **subsystems as the owners of target sources and Plant updates**
- keep **supervisors as the policy layer**
- use the **behavior lanes** to choose the internals
- let **TeleOp bindings and Auto routines call the same public mechanism vocabulary**

That design keeps the number of objects small, makes reuse between TeleOp and Auto natural, and
keeps debugging focused on the right level of abstraction.
