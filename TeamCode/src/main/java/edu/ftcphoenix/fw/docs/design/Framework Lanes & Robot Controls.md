# Architecture Roles, Framework Lanes, and Robot Build Guide

This document is the framework's main guide for **how to structure a robot**.

Read this if you want to answer questions like:

- What is the difference between a primitive and a lane?
- What should live in the framework versus in robot code?
- Where should controls, strategy, capability families, field facts, and telemetry go?
- How should TeleOp and Auto share code without building a giant base robot class?
- How can I build a clean robot from scratch without copying an old robot verbatim?

The goal is not just to explain the current Phoenix robot. The goal is to make the architecture explicit enough that a future robot can be designed from first principles.

---

## The core philosophy

Phoenix uses a few simple ownership rules.

1. **Framework primitives own one narrow reusable capability.**
2. **Framework lanes own one stable multi-object capability graph.**
3. **Field facts stay separate from both sensor rigs and strategy.**
4. **Robot capabilities expose the shared mode-neutral robot vocabulary.**
5. **Robot controls own operator semantics.**
6. **Subsystems are the single writers to mechanisms.**
7. **Supervisors own policy and orchestration.**
8. **Services own shared robot-specific computation.**
9. **Presenters own human-facing output.**
10. **The composition root wires everything together and owns loop order.**
11. **Profiles hold data, not behavior.**

A good rule of thumb is:

> Put an object in the layer where its primary reason to change lives.

Examples:

- camera mount changes because the **camera rig** changed
- drive wiring changes because the **drivetrain hardware** changed
- slow mode changes because the **driver controls** changed
- scoring target offsets change because the **game strategy** changed
- flywheel feed timing changes because the **mechanism policy** changed

---

## The architecture glossary

These names are not just style. They communicate what an object is allowed to own.

### Primitive

A **primitive** is a focused reusable building block with one clear job.

A primitive should usually:

- do one thing well
- be reusable across many robots
- avoid season-specific strategy
- avoid button semantics
- avoid hiding a larger ownership graph than necessary

Examples:

- `GamepadDriveSource`
- `MecanumDrivebase`
- `AprilTagSensor`
- `PinpointPoseEstimator`
- `FixedTagFieldPoseSolver`

A primitive answers:

> What is the smallest reusable capability here?

### Lane

A **lane** is a reusable owner of a stable multi-object capability graph.

A lane usually:

- owns several collaborating primitives
- owns stable FTC resources or lifecycle concerns
- has a clear `update(...)` lifecycle
- may have cleanup like `stop()` or `close()`
- contains little or no game-specific strategy

Examples:

- `FtcMecanumDriveLane`
- `AprilTagVisionLane` / `FtcWebcamAprilTagVisionLane`
- `FtcOdometryAprilTagLocalizationLane`

A lane answers:

> What recurring system do we wire almost the same way every year?

### Field facts

**Field facts** are shared environment data, not hardware ownership and not game strategy.

Examples:

- `TagLayout`
- `FtcGameTagLayout.currentGameFieldFixed()`
- `FtcFieldRegions`

Field facts answer:

> What stable landmarks or regions describe the field?

Keep field facts separate from both the vision lane and the localization lane. The camera rig is a physical sensor concern. Localization is a pose-estimation concern. Field facts describe the environment both of them operate in.

### Controls owner

A **controls owner** defines what operator inputs mean on this robot.

It should own:

- stick mapping
- slow mode
- button edges, toggles, and holds
- enable and override semantics
- conversion from gamepad input into robot intents and reusable sources

Example:

- `PhoenixTeleOpControls`

A controls owner answers:

> What do these sticks and buttons mean on this robot?

### Capability family

A **capability family** is a robot-owned, mode-neutral public façade used by both TeleOp and Auto.

It should usually own:

- intent-level methods and status snapshots
- one cohesive public vocabulary
- as little knowledge of internal class boundaries as possible

It should usually **not** own:

- button semantics
- specific autonomous route sequencing
- final actuator writes

Examples:

- `PhoenixCapabilities.scoring()`
- `PhoenixCapabilities.targeting()`

A capability family answers:

> What shared robot vocabulary should all mode clients use?

Capability families are a normal yearly robot pattern, but they are usually **not** framework lanes.
The public nouns and the best splits vary too much by game and robot design. For the full split
philosophy, read [`Robot Capabilities & Mode Clients`](<Robot Capabilities & Mode Clients.md>).

### Subsystem

A **subsystem** is a robot-owned mechanism owner and usually the single writer to a mechanism.

It should own:

- plants
- mechanism-local sensors or feedback sources
- the mechanism's output queue or pipeline
- final actuator writes
- a small status snapshot

Example:

- `Shooter`

A subsystem answers:

> Who is the final writer for this mechanism?

### Supervisor

A **supervisor** is a robot-owned policy and orchestration object.

It should own:

- request counters
- cooldowns
- small state machines
- "do X only when Y is true" rules
- coordination around one subsystem or a small mechanism cluster

Example:

- `ShooterSupervisor`

A supervisor answers:

> What decides what should happen next?

### Service

A **service** is a robot-owned shared logic object that computes decisions or shared status.

It is usually not the final writer and is usually not primarily about buttons.

Examples:

- `ScoringTargeting`
- `PhoenixDriveAssistService`
- a shot planner
- a target selector
- a pose-based evaluator

A service answers:

> What shared computation or robot-state-dependent policy do multiple robot parts need?

### Presenter

A **presenter** is a read-only object that formats status for humans.

It should consume snapshots and avoid hidden control logic.

Example:

- `PhoenixTelemetryPresenter`

A presenter answers:

> How do we explain the robot's current state to drivers or debuggers?

### Composition root

The **composition root** is the top-level runtime/container object.

It should:

- build the object graph
- wire dependencies together
- choose update order
- own mode lifecycle

It should not quietly absorb policy that belongs elsewhere.

Example:

- `PhoenixRobot`

A composition root answers:

> Where is the system assembled?

### Profile

A **profile** is a data-only configuration aggregate for one robot.

It should contain:

- lane configs
- field facts or field overrides
- controls tuning
- subsystem tuning
- strategy tuning
- calibration acknowledgements

It should not contain runtime behavior.

Example:

- `PhoenixProfile`

A profile answers:

> What data tunes this robot without changing the architecture?

### Low-level runtime nouns

Phoenix also uses some lower-level runtime terms:

- **Source**: a time-varying value sampled with `LoopClock`
- **Plant**: a low-level actuation sink written with a scalar target
- **Task**: a non-blocking behavior that advances over time
- **Overlay**: a way to modify a base output stream, such as drive assistance layered on top of manual drive

Those are framework runtime nouns. The higher-level robot-architecture roles above tell you **who should own them**.

---

## What depends on what

A clean dependency graph matters as much as the object names.

A good top-level shape looks like this:

```text
framework primitives
    ↑
framework lanes
    ↑
robot services / subsystems / supervisors / controls / presenters
    ↑
composition root
```

Field facts sit beside the lanes and can be shared by lanes and robot services:

```text
field facts ───────────────┐
                           ├─> localization service / lane
vision lane ───────────────┤
                           └─> targeting service
```

### Allowed dependency direction

- primitives may depend on other small framework utilities
- lanes may depend on primitives and FTC-boundary adapters
- robot services may depend on lanes, field facts, and other narrow robot APIs
- supervisors may depend on subsystems and services
- presenters may depend on status snapshots
- the composition root may depend on everything it wires together

### Dependency direction to avoid

- a primitive depending on robot controls
- a lane depending on game-specific scoring strategy
- a subsystem depending on button choices
- a presenter pulling hidden live state from everywhere
- a composition root implementing mechanism policy directly

---

## Naming rules

Good names reveal the role.

Good:

- `FtcMecanumDriveLane`
- `AprilTagVisionLane` / `FtcWebcamAprilTagVisionLane`
- `ShooterSupervisor`
- `PhoenixTeleOpControls`
- `ScoringTargeting`
- `PhoenixTelemetryPresenter`

Bad:

- `DriveManager`
- `RobotHelper`
- `MechanismModule`
- `ControllerThing`
- `CommonSubsystemUtils`

The name should help a reader predict the object's ownership role before opening the file.

---

## How to decide what a new object should be

Ask these in order.

### Is it one narrow reusable behavior with no broad lifecycle?
Then it is probably a **primitive**.

### Is it a stable reusable multi-object resource graph with update/cleanup?
Then it is probably a **lane**.

### Is it the single writer to one mechanism?
Then it is probably a **subsystem**.

### Does it decide what should happen without doing final writes?
Then it is probably a **supervisor**.

### Is it mapping sticks and buttons into intent?
Then it is **controls**.

### Is it shared robot-specific computation?
Then it is probably a **service**.

### Is it only formatting output for humans?
Then it is a **presenter**.

### Is it only wiring the system together?
Then it is the **composition root**.

### Is it only data?
Then it is **profile/config**.

---

## Build a robot from scratch

This section is intentionally concrete. The goal is that you can design a new robot without opening an old robot first.

Not every robot needs every role shown below. For example, a simple TeleOp-only robot may have no vision lane, no localization lane, and no targeting service. The point of the taxonomy is not to force every robot into the same shape. The point is to give each concern a principled home when that concern exists.

## Step 1: design the profile first

Start by sketching the robot's **profile shape**. That forces the architectural boundaries to be explicit.

A strong starting pattern is:

```java
public final class MyRobotProfile {

    public FtcMecanumDriveLane.Config drive = FtcMecanumDriveLane.Config.defaults();
    public FtcWebcamAprilTagVisionLane.Config vision = FtcWebcamAprilTagVisionLane.Config.defaults();
    public FtcOdometryAprilTagLocalizationLane.Config localization =
            FtcOdometryAprilTagLocalizationLane.Config.defaults();

    public FieldConfig field = new FieldConfig();
    public ControlsConfig controls = new ControlsConfig();
    public DriveAssistConfig driveAssist = new DriveAssistConfig();
    public MechanismConfig mechanism = new MechanismConfig();
    public StrategyConfig strategy = new StrategyConfig();
    public CalibrationConfig calibration = new CalibrationConfig();

    public MyRobotProfile copy() {
        MyRobotProfile c = new MyRobotProfile();
        c.drive = this.drive.copy();
        c.vision = this.vision.copy();
        c.localization = this.localization.copy();
        c.field = this.field.copy();
        c.controls = this.controls.copy();
        c.driveAssist = this.driveAssist.copy();
        c.mechanism = this.mechanism.copy();
        c.strategy = this.strategy.copy();
        c.calibration = this.calibration.copy();
        return c;
    }
}
```


For the simplest robot, keeping `vision` concrete as `FtcWebcamAprilTagVisionLane.Config` is still a good default.
If you expect to swap between webcam and smart-camera backends later, keep a robot-owned `VisionConfig`
wrapper in the profile and let the composition root hold the backend-neutral `AprilTagVisionLane` interface.

Why this order works:

- lane configs stay grouped by stable ownership
- field facts stay separate from sensor rigs and strategy
- controls tuning stays with controls
- drive-assist tuning stays with the service that owns robot-specific overlays and assist latches
- mechanism tuning stays with the subsystem
- strategy tuning stays with services and supervisors

## Step 2: create stable framework lanes

Create framework lanes for systems that are wired almost the same way every year.

### Drive lane example

```java
FtcMecanumDriveLane drive = new FtcMecanumDriveLane(hardwareMap, profile.drive);
```

### Vision lane example

```java
AprilTagVisionLane vision = new FtcWebcamAprilTagVisionLane(hardwareMap, profile.vision);
```

The important split is: construct a concrete lane at the FTC boundary, but store and pass around the
backend-neutral `AprilTagVisionLane` seam above that boundary. That lets localization and targeting
consume tag observations without caring whether the implementation is webcam-backed today or smart-camera-backed later.

### Localization lane example

```java
FtcOdometryAprilTagLocalizationLane localization =
        new FtcOdometryAprilTagLocalizationLane(
                hardwareMap,
                vision,
                profile.field.fixedAprilTagLayout,
                profile.localization
        );
```

This split is intentional:

- `vision` owns the camera rig
- `localization` owns pose estimation
- `field` owns field facts

Do not push all three into one giant lane unless they truly have the same reason to change.

## Step 3: create one controls owner

All TeleOp input semantics should live in one robot-owned controls object.

```java
public final class MyTeleOpControls {

    private final Bindings bindings = new Bindings();
    private final DriveSource manualDrive;
    private final ScalarSource manualTranslateMagnitude;
    private final BooleanSource assistEnabled;
    private final BooleanSource override;
    private final GamepadDevice driver;
    private final GamepadDevice operator;

    public MyTeleOpControls(Gamepads gamepads, MyRobotProfile.ControlsConfig cfg) {
        this.driver = gamepads.p1();
        this.operator = gamepads.p2();
        this.manualTranslateMagnitude = driver.leftStickMagnitude().memoized();

        this.manualDrive = new GamepadDriveSource(
                driver.leftX(),
                driver.leftY(),
                driver.rightX(),
                cfg.drive.manualDrive
        ).scaledWhen(driver.rightBumper(), cfg.drive.slowTranslateScale, cfg.drive.slowOmegaScale);

        this.assistEnabled = operator.leftBumper().memoized();
        this.override = operator.y().memoized();
    }

    public void bind(MyCapabilities capabilities) {
        MyCapabilities.GamePiece gamePiece = capabilities.gamePiece();

        bindings.onToggle(operator.a(), gamePiece::setIntakeEnabled);
        bindings.onRiseAndFall(
                operator.b(),
                new Runnable() {
                    @Override
                    public void run() {
                        gamePiece.setActionEnabled(true);
                    }
                },
                new Runnable() {
                    @Override
                    public void run() {
                        gamePiece.setActionEnabled(false);
                    }
                }
        );
    }

    public DriveSource manualDriveSource() {
        return manualDrive;
    }

    public ScalarSource manualTranslateMagnitudeSource() {
        return manualTranslateMagnitude;
    }

    public BooleanSource assistEnabledSource() {
        return assistEnabled;
    }

    public BooleanSource overrideSource() {
        return override;
    }

    public void update(LoopClock clock) {
        bindings.update(clock);
    }
}
```

Why this matters:

- stick mapping lives with button semantics
- slow mode is not hidden in a low-level primitive
- higher-level services can depend on input semantics through narrow sources instead of peeking at raw gamepads
- mode clients can bind against shared capability families instead of raw internals
- the composition root does not need to know specific button identities


## Step 3.5: add one shared capability aggregate

Before you wire TeleOp or Auto, decide what shared robot-facing capability families exist this year.

A small example:

```java
public interface MyCapabilities {
    GamePiece gamePiece();
    Targeting targeting();

    interface GamePiece {
        void setIntakeEnabled(boolean enabled);
        void setFlywheelEnabled(boolean enabled);
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

Why this layer exists:

- TeleOp and Auto should share the same vocabulary even when they do not share the same top-level code
- the controls owner should not depend directly on supervisors/subsystems/services
- the robot container should expose a small public façade instead of leaking all internals

Do **not** assume every year will use the same family names. Choose the split that matches the
robot's cohesive public story this season. For the full decision rules, read
[`Robot Capabilities & Mode Clients`](<Robot Capabilities & Mode Clients.md>).

### Optional: add a robot-specific drive-assist service

If your TeleOp drive path needs robot-specific overlays or latches, create a service for them instead
of putting that logic in the composition root.

Good examples:

- shoot-brace / pose hold while scoring
- heading hold that depends on a robot mode
- temporary precision translation hold
- auto-align overlays that combine manual drive with targeting state

Bad example:

- a `shootBraceEnabled()` helper plus a hysteresis latch stored directly inside `MyRobot`

That is not the composition root's job. The composition root should wire the service, not own the
assist policy itself.

```java
public final class MyDriveAssistService {

    private final DriveSource driveSource;
    private final ScalarSource manualTranslateMagnitude;
    private final BooleanSource assistRequested;
    private final HysteresisBoolean braceLatch;
    private DriveAssistStatus lastStatus = new DriveAssistStatus(false, false, false, 0.0);

    public MyDriveAssistService(MyRobotProfile.DriveAssistConfig cfg,
                                DriveSource manualDrive,
                                ScalarSource manualTranslateMagnitude,
                                BooleanSource assistRequested,
                                PoseEstimator globalPose,
                                DriveOverlay assistOverlay) {
        this.manualTranslateMagnitude = manualTranslateMagnitude;
        this.assistRequested = assistRequested;
        this.braceLatch = HysteresisBoolean.onWhenBelowOffWhenAbove(
                cfg.shootBrace.enterTranslateMagnitude,
                cfg.shootBrace.exitTranslateMagnitude
        );

        this.driveSource = DriveOverlayStack.on(manualDrive)
                .add(
                        "shootBrace",
                        BooleanSource.of(braceLatch::get),
                        DriveGuidance.poseLock(
                                globalPose,
                                DriveGuidancePlan.Tuning.defaults()
                                        .withTranslateKp(cfg.shootBrace.translateKp)
                                        .withMaxTranslateCmd(cfg.shootBrace.maxTranslateCmd)
                        ),
                        DriveOverlayMask.TRANSLATION_ONLY
                )
                .add("assist", assistRequested, assistOverlay, DriveOverlayMask.OMEGA_ONLY)
                .build();
    }

    public DriveSource driveSource() {
        return driveSource;
    }

    public void update(LoopClock clock, MechanismSupervisorStatus supervisorStatus) {
        boolean braceEligible = supervisorStatus != null && supervisorStatus.actionActive();
        double translateMag = manualTranslateMagnitude.getAsDouble(clock);

        boolean braceEnabled;
        if (!braceEligible) {
            braceLatch.reset(false);
            braceEnabled = false;
        } else {
            braceEnabled = braceLatch.update(translateMag);
        }

        lastStatus = new DriveAssistStatus(
                assistRequested.getAsBoolean(clock),
                braceEligible,
                braceEnabled,
                translateMag
        );
    }

    public DriveAssistStatus status() {
        return lastStatus;
    }
}
```

Why this boundary works:

- controls still own what the sticks and buttons mean
- the service owns how robot policy reshapes drive behavior
- the subsystem and supervisor stay focused on the mechanism
- the composition root stays boring

When in doubt, ask:

> Is this logic deciding how manual drive should be reshaped based on robot state?

If yes, that usually belongs in a robot service like `MyDriveAssistService`, not in the controls owner
and not in the composition root.

## Step 4: make each mechanism a subsystem

The subsystem is the single writer.

```java
public final class IntakeShooterSubsystem {

    private final Plant intake;
    private final Plant feeder;
    private final Plant flywheel;

    public IntakeShooterSubsystem(HardwareMap hardwareMap, MyRobotProfile.MechanismConfig cfg) {
        // build plants here
    }

    public void setIntakeEnabled(boolean enabled) {
        // update desired state
    }

    public void setFlywheelTarget(double velocityNative) {
        // update desired state
    }

    public void requestSingleFeedPulse() {
        // enqueue feed request
    }

    public void update(LoopClock clock) {
        // compute final outputs and write plants here
    }

    public Status status(LoopClock clock) {
        return new Status(/* snapshot fields */);
    }
}
```

If multiple objects are directly writing the same actuator targets, the subsystem boundary is probably wrong.

## Step 5: move policy into a supervisor

Supervisors translate intent and status into requests to the subsystem.

```java
public final class IntakeShooterSupervisor {

    private final IntakeShooterSubsystem subsystem;
    private boolean shootingRequested;
    private boolean intakeEnabled;

    public IntakeShooterSupervisor(IntakeShooterSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    public void setIntakeEnabled(boolean enabled) {
        this.intakeEnabled = enabled;
    }

    public void beginShooting() {
        this.shootingRequested = true;
    }

    public void endShooting() {
        this.shootingRequested = false;
    }

    public void update(LoopClock clock) {
        subsystem.setIntakeEnabled(intakeEnabled);
        if (shootingRequested) {
            subsystem.requestSingleFeedPulse();
        }
    }

    public SupervisorStatus status() {
        return new SupervisorStatus(intakeEnabled, shootingRequested);
    }
}
```

In real robot code, make `SupervisorStatus` an immutable snapshot class just like subsystem and service
status objects. The exact fields will vary by mechanism; the important point is that higher-level
consumers such as drive-assist services or telemetry should read a narrow status object instead of
peeking into supervisor internals.

A supervisor should usually not be the final plant writer.

## Step 6: put shared reasoning into a service

A service is ideal for targeting, shot planning, or pose-based decisions.

```java
public final class ScoringTargeting {

    public ScoringTargeting(MyRobotProfile.StrategyConfig cfg,
                            AprilTagSensor tagSensor,
                            CameraMountConfig cameraMount,
                            PoseEstimator globalPose,
                            TagLayout fieldTags,
                            BooleanSource aimEnabled,
                            BooleanSource aimOverride) {
        // build shared targeting logic here
    }

    public void update(LoopClock clock) {
        // cache one status snapshot per loop
    }

    public TargetingStatus status(LoopClock clock) {
        // return the cached snapshot
    }

    public BooleanSource aimOkToShootSource() {
        // expose a narrow reusable signal
    }
}
```

A service is often the right place to centralize anything that several parts of the robot need to agree on.

## Step 7: keep presentation separate

```java
public final class MyTelemetryPresenter {

    public void emitTeleOp(MechanismStatus mechanism,
                           SupervisorStatus supervisor,
                           TargetingStatus targeting,
                           PoseEstimate globalPose) {
        // read snapshots, write telemetry
    }
}
```

Do not hide robot control logic in the presenter.

## Step 8: make the composition root boring

The composition root should mostly assemble the graph and run the loop in a clear order.

```java
public final class MyRobot {

    private final LoopClock clock = new LoopClock();
    private final MyRobotProfile profile;

    private FtcMecanumDriveLane drive;
    private AprilTagVisionLane vision;
    private FtcOdometryAprilTagLocalizationLane localization;

    private MyCapabilities capabilities;
    private MyTeleOpControls controls;
    private ScoringTargeting targeting;
    private IntakeShooterSubsystem shooter;
    private IntakeShooterSupervisor scoring;
    private MyTelemetryPresenter telemetry;
    private DriveSource driveSource;

    public void initTeleOp() {
        drive = new FtcMecanumDriveLane(hardwareMap, profile.drive);
        vision = new FtcWebcamAprilTagVisionLane(hardwareMap, profile.vision);
        localization = new FtcOdometryAprilTagLocalizationLane(
                hardwareMap,
                vision,
                profile.field.fixedAprilTagLayout,
                profile.localization
        );

        controls = new MyTeleOpControls(gamepads, profile.controls);
        shooter = new IntakeShooterSubsystem(hardwareMap, profile.mechanism);
        targeting = new ScoringTargeting(
                profile.strategy,
                vision.tagSensor(),
                vision.cameraMountConfig(),
                localization.globalEstimator(),
                profile.field.fixedAprilTagLayout,
                controls.assistEnabledSource(),
                controls.overrideSource()
        );
        scoring = new IntakeShooterSupervisor(shooter);
        telemetry = new MyTelemetryPresenter();
        capabilities = new MyRobotCapabilities(shooter, scoring, targeting);

        controls.bind(capabilities);
        driveSource = controls.manualDriveSource();
    }

    public void updateTeleOp() {
        localization.update(clock);
        targeting.update(clock);
        controls.update(clock);
        scoring.update(clock);
        drive.update(clock);
        drive.drive(driveSource.get(clock).clamped());
        shooter.update(clock);
        telemetry.emitTeleOp(
                shooter.status(clock),
                scoring.status(),
                targeting.status(clock),
                localization.globalPose()
        );
    }
}
```

If the composition root starts containing lots of button semantics, aim thresholds, or mechanism timing rules, stop and move that behavior down into the right owner.

---

## Example dependency graph for a modern robot

```text
MyRobotProfile
  ├─ drive      -> FtcMecanumDriveLane.Config
  ├─ vision     -> FtcWebcamAprilTagVisionLane.Config (or a robot-owned backend wrapper)
  ├─ localization -> FtcOdometryAprilTagLocalizationLane.Config
  ├─ field      -> TagLayout / field facts
  ├─ controls   -> MyTeleOpControls.Config
  ├─ mechanism  -> MySubsystem.Config
  └─ strategy   -> MyService / MySupervisor tuning

MyRobot
  ├─ FtcMecanumDriveLane
  ├─ AprilTagVisionLane (commonly `FtcWebcamAprilTagVisionLane`)
  ├─ FtcOdometryAprilTagLocalizationLane
  ├─ MyCapabilities
  ├─ MyTeleOpControls
  ├─ MySubsystem
  ├─ MySupervisor
  ├─ MyService
  └─ MyTelemetryPresenter
```

This structure scales well because each owner has one obvious reason to change.

---

## What should go into config versus code

### Put this in config

- hardware names
- directions
- open-loop tuning
- controller gains
- tolerances
- calibration values
- target catalogs and offsets
- field overrides when needed

### Keep this in code

- which button means slow mode
- which button toggles intake
- the exact sequencing logic for a mechanism
- how multiple conditions combine into one policy decision
- the update order of the runtime

A useful rule is:

> Put data that describes the robot in config. Put decisions that describe behavior in code.

---

## Common anti-patterns

### Anti-pattern: giant base robot class

```text
BaseCompetitionRobot
  with dozens of hooks and convenience methods
```

Why it is a problem:

- hidden assumptions accumulate quickly
- every season adds more hooks
- boundaries blur between framework and robot strategy

Prefer composition and explicit owners.

### Anti-pattern: a primitive that smuggles button policy

Bad in spirit:

```text
GamepadDriveSource.teleOpMecanumSlowRb(...)
```

Why it is a problem:

- a low-level primitive suddenly chooses a button
- drive controls become split across different files
- future robots inherit one specific control scheme by accident

Instead, create the primitive with explicit axes and apply slow mode in the controls owner.

### Anti-pattern: localization owning the camera rig

Bad in spirit:

```text
one localization lane owns camera identity, camera mount, and every AprilTag user
```

Why it is a problem:

- the camera is not only for localization
- aiming and other vision features may need the same resource
- camera mount is a vision-rig concern, not a localization-strategy concern

Instead, split:

- `AprilTagVisionLane` / `FtcWebcamAprilTagVisionLane` for the camera rig
- `FtcOdometryAprilTagLocalizationLane` for pose production
- field facts for shared landmarks

### Anti-pattern: presenter reaches back into live mutable state

Why it is a problem:

- telemetry code starts changing behavior indirectly
- refactors become harder to audit
- loop order becomes ambiguous

Instead, presenters should consume snapshots already computed by the rest of the robot.

---

## Mapping the roles onto Phoenix

Phoenix is the reference example for this split:

- primitive: `GamepadDriveSource`
- primitive: `MecanumDrivebase`
- primitive: `AprilTagSensor`
- lane: `FtcMecanumDriveLane`
- lane: `FtcWebcamAprilTagVisionLane` (through the `AprilTagVisionLane` seam)
- lane: `FtcOdometryAprilTagLocalizationLane`
- field facts: `PhoenixProfile.field.fixedAprilTagLayout`
- capability family: `PhoenixCapabilities.scoring()` / `PhoenixCapabilities.targeting()`
- controls owner: `PhoenixTeleOpControls`
- subsystem: `Shooter`
- supervisor: `ShooterSupervisor`
- service: `ScoringTargeting`
- presenter: `PhoenixTelemetryPresenter`
- composition root: `PhoenixRobot`
- profile: `PhoenixProfile`

That is the intended pattern for future robots too. Copy the structure, not the season-specific behavior.

---

## Read next

- [`Robot Capabilities & Mode Clients`](<Robot Capabilities & Mode Clients.md>)
- [`Recommended Robot Design`](<Recommended Robot Design.md>)
- [`Supervisors & Pipelines`](<Supervisors & Pipelines.md>)
- [`Tasks & Macros Quickstart`](<Tasks & Macros Quickstart.md>)
- [`Framework Overview`](<../getting-started/Framework Overview.md>)
- [`AprilTag Localization & Fixed Layouts`](<../drive-vision/AprilTag Localization & Fixed Layouts.md>)
