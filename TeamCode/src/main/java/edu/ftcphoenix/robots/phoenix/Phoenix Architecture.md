# Phoenix Architecture

This document explains Phoenix's current object graph and how it maps onto the framework's
architecture vocabulary.

The governing rule is:

> Stable hardware/resource ownership lives in framework lanes. Shared public robot vocabulary lives
> in robot-owned capability families. Operator semantics live in TeleOp controls. Game-specific
> reasoning lives in cohesive robot services and subsystems.

That split keeps Phoenix reusable as a template without pushing one season's strategy into the
framework.

Useful framework references:

- [`Framework Lanes & Robot Controls`](<../../fw/docs/design/Framework Lanes & Robot Controls.md>)
- [`Robot Capabilities & Mode Clients`](<../../fw/docs/design/Robot Capabilities & Mode Clients.md>)
- [`Recommended Robot Design`](<../../fw/docs/design/Recommended Robot Design.md>)

## Big-picture structure

```text
PhoenixRobot
  PhoenixProfile

  PhoenixCapabilities capabilities

  FtcMecanumDriveLane drive
  AprilTagVisionLane vision   (selected by PhoenixVisionFactory from the active profile backend)
  FtcOdometryAprilTagLocalizationLane localization
    (constructs TeleOp predictor or retains the predictor supplied for Auto)

  PhoenixTeleOpControls controls
  PhoenixDriveAssistService driveAssists
  ScoringTargeting targeting
  ScoringPath scoring
  DriveCommandSink autoDrive   (Auto only; supplied by the Auto mode client)
  AutoRoutineLifecycle autoRoutineLifecycle
    TaskRunner runner          (private lifecycle driver for one installed root)
    Task installedRoutine      (retained for terminal telemetry)
  PhoenixTelemetryPresenter telemetry
```

Phoenix keeps backend choice in a small robot-owned wrapper: `PhoenixProfile.VisionConfig`
plus `PhoenixVisionFactory`. That is why the rest of the graph can depend on
`AprilTagVisionLane` instead of a backend-specific owner even though the FTC-boundary
implementation may be a webcam lane or a Limelight lane.

The shared mode clients are:

```text
TeleOp client:
  PhoenixTeleOpControls -> PhoenixCapabilities -> Phoenix internals

Auto client:
  PhoenixAutoSpec / PhoenixPedroAutoRoutineFactory -> PhoenixCapabilities -> Phoenix internals
```

That is the intended parallelism. TeleOp and Auto are parallel **clients** of Phoenix, not
different APIs layered directly onto internals.

`PhoenixCapabilities` is now only a tiny aggregate of robot-owned capability families. It no longer has a separate delegating implementation class; `ScoringPath` implements the scoring family directly and `ScoringTargeting` implements the targeting family directly.

## Role map

### Framework primitives used underneath the lanes

- `GamepadDriveSource`
- `MecanumDrivebase`
- `AprilTagSensor`
- `PinpointOdometryPredictor`
- `FixedTagFieldPoseSolver`

### Framework lanes

- `FtcMecanumDriveLane`: owns mecanum hardware wiring, brake behavior, drivebase construction, and drive lifecycle
- `AprilTagVisionLane` plus a concrete FTC-boundary implementation such as `FtcWebcamAprilTagVisionLane` or `FtcLimelightAprilTagVisionLane`: Phoenix consumes the backend-neutral seam while `PhoenixVisionFactory` chooses the active backend
- `FtcOdometryAprilTagLocalizationLane`: owns predictor wiring + AprilTag localization strategy,
  correction-source selection, corrected-estimator selection, and pose production; it constructs
  Pinpoint for ordinary TeleOp or consumes an explicitly supplied Auto predictor

### Shared field facts

- `PhoenixProfile.field.fixedAprilTagLayout`

This layout is consumed by localization and targeting. It is intentionally not hidden inside the
vision lane or the localization lane.

### Robot-owned capability families

- `PhoenixCapabilities.scoring()`
- `PhoenixCapabilities.targeting()`

These are Phoenix's shared mode-neutral public API families. TeleOp controls and Auto plans should
use them instead of touching `ScoringPath` or `ScoringTargeting` directly.

### Robot-owned objects

- `PhoenixTeleOpControls`: all TeleOp input semantics, including stick mapping and scoring bindings
- `PhoenixDriveAssistService`: robot-specific drive assists that combine manual drive, scoring state, localization, and overlays
- `ScoringTargeting`: selected-tag policy, auto-aim guidance, cached targeting status, and shot suggestions
- `ScoringPath`: scoring-path mechanism owner, source-driven scoring Plants, internally layered as inputs → execution → realization
- `DriveCommandSink autoDrive`: required Auto drive lifecycle owner supplied through a
  backend-neutral seam; Phoenix advances and stops it without owning route-library strategy
- private `AutoRoutineLifecycle autoRoutineLifecycle`: owns both the private `TaskRunner` lifecycle
  driver and Phoenix's one installed Auto root
- retained installed Auto root: the single routine selected before FTC START and the source of
  active or terminal routine telemetry; Phoenix does not expose queue scheduling or runner access
- `PhoenixTelemetryPresenter`: driver-facing presentation from snapshots
- `PhoenixRobot`: composition root and loop owner

## Why Phoenix uses capability families

Phoenix now has an explicit `PhoenixCapabilities` layer because TeleOp and Auto both need the same
robot vocabulary, but they should not depend on raw internals.

Phoenix currently splits that vocabulary into two families because that is the clearest public split
for Decode.

### `scoring()`

Owns public actions that change scoring-path state:

- intake enable
- flywheel enable
- continuous shooting enable
- eject enable
- single-shot requests
- selected-velocity adjustments
- scoring status

### `targeting()`

Owns public targeting/aiming behavior and read-side status:

- selected target / aim status snapshots
- autonomous aim task creation

### Why this split makes sense for Decode

`scoring()` is command-heavy and mutates mechanism state.

`targeting()` is mostly about selection, aim readiness, and aim execution. Auto and telemetry often
need to read targeting status without also caring about intake/eject/flywheel requests.

That makes the split cohesive without forcing callers to depend on one giant flat robot API.

### What future robots should copy

Copy the **pattern**, not these exact family names.

Another season may use families like:

- `gamePiece()`
- `endgame()`
- `awareness()`
- `drive()`

The rule is to split by cohesive public vocabulary, not by this year's internal class names.

## Dependency graph

```text
PhoenixProfile
  ├─ drive        -> FtcMecanumDriveLane.Config
  ├─ vision       -> PhoenixProfile.VisionConfig
  ├─ localization -> FtcOdometryAprilTagLocalizationLane.Config
  ├─ field        -> TagLayout
  ├─ controls     -> PhoenixTeleOpControls tuning
  ├─ driveAssist  -> PhoenixDriveAssistService tuning
  ├─ scoring      -> ScoringPath config
  ├─ autoAim      -> ScoringTargeting / scoring policy config
  └─ auto         -> Auto route/aim/wait timing

PhoenixRobot
  ├─ drive lane
  ├─ vision lane
  ├─ localization lane (retains the predictor supplied for Auto)
  ├─ capabilities aggregate
  ├─ controls owner
  ├─ drive-assist service
  ├─ targeting service
  ├─ scoring path
  ├─ retained Auto DriveCommandSink lifecycle owner
  └─ telemetry presenter
```

## Why the vision/localization split matters

Older FTC code often lumps camera ownership and localization together. Phoenix avoids that because
the camera rig is not only for localization.

### `AprilTagVisionLane` plus its concrete backend owner own

- backend-specific device identity (for example webcam or Limelight)
- backend-specific processor / pipeline setup
- camera mount
- AprilTag sensor lifetime
- vision-device cleanup

### `FtcOdometryAprilTagLocalizationLane` owns

- the motion-predictor seam and its correction integration
- ordinary FTC/TeleOp construction of Pinpoint, or an explicitly injected Auto predictor without
  constructing a competing hardware owner
- raw AprilTag field solver
- optional direct Limelight field pose
- correction-source and corrected/global estimator selection
- localization update order
- corrected and predictor pose outputs

### `ScoringTargeting` consumes

- the shared AprilTag sensor from the vision lane
- the camera mount from the vision lane
- the corrected/global pose from the localization lane
- the fixed field tag layout from `PhoenixProfile.field`

That dependency structure is deliberate and matches the role vocabulary in the framework docs.

## TeleOp controls live in one place

Phoenix no longer spreads drive stick setup in one class and scoring bindings in another.

`PhoenixTeleOpControls` owns all of these:

- manual drive source creation
- translation-magnitude semantics shared with higher-level services
- slow mode
- auto-aim enable source
- aim override source
- intake/flywheel/shoot/eject button semantics
- selected-velocity nudge semantics

The important boundary is that TeleOp binds against `PhoenixCapabilities`, not against raw
`ScoringPath` or `ScoringTargeting` objects.

Phoenix still keeps drivetrain control on the dedicated drive-source path because manual drive is a
continuous stream that benefits from source composition and overlays. Non-drive continuous mechanism
controls should generally go through capabilities as frame-valued commands and be registered with
`Bindings.copyEachCycle(...)` when Phoenix adds those commands.

`PhoenixDriveAssistService` then consumes those control-layer sources without taking ownership of the
button layout itself. That keeps input semantics and drive-assist policy separate while still
letting them collaborate cleanly.

`PhoenixReadiness.teleOpPoseAssists(...)` evaluates the two checked-in Pinpoint calibration
acknowledgements before those pose-dependent overlays are wired. If axes or pod offsets are not
acknowledged, manual drive and all mechanism capabilities remain available, while auto-aim and
shoot-brace are both gated off. The INIT and normal TeleOp frames print the exact calibration step
needed; lack of a currently visible AprilTag is not an initialization blocker.

## ScoringPath internal layering

`ScoringPath` now keeps its mutable state split into the same three roles Phoenix wants future
mechanisms to follow:

- **Inputs**: held caller-owned selections and pending request counters written by capability methods
- **Execution**: priority rules, queue admission, cancellation rules, and transient shot behavior
- **Realization**: plant ownership, final actuator targets, and flywheel readback used for readiness

That internal split matters more than file count. `ScoringPath` is still one public mechanism owner,
but the code now makes it much harder to accidentally mix caller-owned intent with robot-owned queue
state or plant readback.

The realization layer no longer imperatively chooses and writes plant targets every loop. Instead,
each scoring Plant is built with a final `PlantTargetSource`. Continuous baseline feed behavior is a
source, the behavior-owned feed pulse queue is another source, and `PlantTargets.overlay(...)` expresses
the priority rule. The Plant samples the final target source during `update(clock)` and then applies
its own hardware guards. Telemetry/debug output can now separate `getTargetPlan()` (“which behavior
target won?”) from `getTargetStatus()` (“how did hardware guards apply it?”).

That distinction is important:

```text
behavior guard: source shaping and overlay priority
Plant guard: hardware protection after the final target source is sampled
```

`ScoringPath` uses the feed pulse queue as a behavior pattern, not as a Plant wrapper. One logical
shot pulse can therefore fan out into the intake motor, intake transfer, and shooter transfer through
separate scaled overlay layers while still preserving one final target source per Plant.

## Autonomous structure

Phoenix Auto reuses the same targeting, scoring path, and telemetry stack as TeleOp. Route-library
configuration, route geometry, and strategy remain outside the robot container, while the robot
composition root retains the supplied backend-neutral Auto `DriveCommandSink` as the one recurring
heartbeat and shutdown owner. The robot-owned spec/strategy objects make the selected match setup
explicit before any route or task sequence is built.

The core Auto types are:

```text
PhoenixAutoSpec
  alliance
  startPosition
  partnerPlan
  strategy

PhoenixAutoProfiles
  spec + base profile -> Auto-specific profile snapshot, using profile-owned red/blue Auto tag ids

PhoenixReadiness
  profile + exact spec + route availability + OpMode purpose -> immutable warnings/blockers

PhoenixAutoTasks
  reusable scoring/targeting task snippets over PhoenixCapabilities, with truthful attempt outcomes

PhoenixPedroPathFactory
  spec -> fixed Pedro PathChain set
  current Pedro pose / current robot facts -> start-time Pedro PathChain

PhoenixPedroAutoRoutineFactory
  spec.strategy + context -> route-status-aware Task routine
```

`PhoenixRobot.initAuto(autoDrive, motionPredictor)` does four things only:

1. retain the required backend-neutral Auto drive lifecycle owner supplied by the mode client
2. build shared Auto localization around the explicitly supplied backend-neutral predictor
3. create `PhoenixCapabilities`
4. create the private `TaskRunner` that will start and own one pre-START-installed Auto root

Those four successful construction steps mean the Phoenix-owned runtime services exist; they do
not mean that a selected match Auto is armed. `PhoenixPedroAutoOpModeBase` owns that higher-level
decision because it also knows the exact spec, route maturity, calibration acknowledgements, and
Driver Station entry purpose.

Each `PhoenixRobot` accepts one mode initialization for its lifetime. A repeated Auto init or a
TeleOp/Auto cross-init fails before replacing any retained owner; a different mode/runtime uses a
new robot container.

The Auto mode client installs exactly one root with `installAutoRoutine(...)` before it starts the
robot. Normal static entries and a confirmed selector do this during INIT. If the driver presses
START without confirming the selector, the selector makes one guarded last-chance call through the
same readiness and construction path; any blocker still returns before hardware or a root is
constructed. After `startAny(...)` resets the shared clock at FTC START, `startAuto()` immediately
starts that root through the private runner, so a match budget is measured from the real start
boundary rather than the first later loop. Phoenix's private pre-park coordinator arms at that
boundary but defers its outbound child until the first normal Auto Task phase, after localization,
targeting, and the Pedro heartbeat. A second install, an install after `startAny(...)`, start without
a root, or repeated start fails fast.

For a simple open-loop Auto/test interval, `DriveTasks.driveExclusivelyForSeconds(...)` may command
the retained sink only when that Task is the sole behavior-command writer. It refreshes the sink and
writes the requested signal each active cycle, then stops on completion or active cancellation.
Phoenix's retained Auto owner still provides any independent composition-root heartbeat the sink
requires, with same-cycle adapter deduplication. Production Pedro movement normally uses
`RouteTasks` or guidance Tasks instead of timed open-loop drive.

For Pedro, `PhoenixPedroAutoOpModeBase` asks the project constants factory for one
`PedroPathingRuntime`. That runtime contains one configured Pinpoint predictor, one passive Pedro
localizer view, one Follower/native mecanum drivetrain, and one `PedroPathingDriveAdapter`. The mode
client calls `robot.initAuto(runtime.driveAdapter(), runtime.motionPredictor())`, then applies the
selected Pedro start pose through the runtime before the first heartbeat. The adapter remains
available to route and guidance Tasks for behavior requests, but Phoenix owns its recurring update
and final stop. The adapter uses `LoopClock.cycle()` to make the root update and later same-cycle
generic Task hooks one physical Pedro heartbeat.

Before constructing that graph, the base obtains one path-factory-owned `RouteAvailability` and
evaluates `PhoenixReadiness`. Match Auto requires verified Pinpoint axes, calibrated pod offsets,
the selected alliance tag in both the scoring catalog and fixed layout, and deliberately
`MATCH_READY` geometry. The inactive alliance target is irrelevant. Any blocker is rendered during
INIT and returns before `PhoenixRobot`, Pedro hardware, paths, or the Auto root are constructed;
START checks the retained result again. The explicitly named Pedro test entry is the only client
that accepts `INTEGRATION_ONLY` geometry. It still blocks unverified axes, permits uncalibrated pod
offsets only as a persistent warning, and displays `TEST` on every frame.

The fixed path's first translation and effective wrapped heading must structurally match its
declared start pose. The runtime verifies that the Pinpoint predictor publishes the requested
software rebase during INIT, and the mode client reapplies that same pose at FTC START before
resetting the shared clock or allowing the first heartbeat. This establishes one software
coordinate authority. It does not measure physical placement, so INIT telemetry prominently shows
the expected Pedro-field x/y/heading (inches/degrees) and the drive team must place the robot there.

Each route start returns a backend-neutral `RouteExecution` whose status and cancellation remain
bound to that run. During its owned heartbeat, the Pedro adapter classifies natural completion,
follower timeout/stall, interruption, replacement, failure, or an unknown terminal transition
before Pedro clears the evidence. `RouteTask.getRouteStatus()` also distinguishes Task timeout and
active cancellation. This keeps the routine call short while preventing an old Task from completing
from, or cancelling, a replacement route.

The adapter reports those facts but does not choose strategy. The checked-in Phoenix routine calls
all outbound and scoring behavior **pre-park work**. It gates scoring on outbound `COMPLETED` and
wraps that complete pre-park policy in `Tasks.withTimeout(...)` using
`profile.auto.parkTakeoverElapsedSec`. An outbound follower/Task timeout or scoring timeout remains
truthful and permits the one live-pose park. At the match threshold, the wrapper actively cancels
the current pre-park phase, waits for its safety cleanup, and then permits that same park. If
pre-park finishes early, the park starts early and is no longer inside the timer.

Because the persistent Pedro heartbeat precedes the Task phase, a route may become terminal on the
exact cutoff cycle before its `RouteTask` update. `RouteTask.getRouteStatus()` observes that exact
execution before pre-park cancellation is classified. The same boundary check validates a terminal
scoring outcome. Success or a local timeout remains park-eligible; cancellation-like or malformed
evidence fails closed and suppresses the park.

Interruption, replacement, cancellation, failure, or an unknown pre-park result aborts instead of
starting another route. Direct cancellation of the routine also aborts without launching fallback.
The scoring attempt reports unavailable-target, aim, and shot-drain timeouts truthfully so the
routine can make the same explicit decision. A successful park after any allowed local or match
timeout retains the routine's `TIMEOUT`; a cancellation-like park result takes precedence as
`CANCELLED`, and no park result starts a second fallback.

Fixed geometry is built eagerly while the selected runtime is constructed before robot start and
uses the named `RouteTasks.follow(...)` factory with `profile.auto.routeTimeoutSec` passed directly.
Geometry that must start from a live pose or a current robot selection stays in
`PhoenixPedroPathFactory` and uses the parallel named `RouteTasks.followBuiltAtStart(...)` factory
with the same explicit timeout. Phoenix deliberately does not select the
`WithoutTaskTimeout` variants because its route policy retains `RouteStatus.TASK_TIMEOUT`; those
variants disable only the Task deadline and would not disable follower timeout/stall detection.
The path factory is retained in `PhoenixPedroAutoContext`; its quick supplier runs once when that
Task starts, reads only supported current snapshots, and gives the adapter one concrete route.
Neither `PhoenixRobot` nor the generic route API gains Pedro, vision, alliance, or game-strategy
types.

When future strategy needs a bounded mechanism Task only while a route remains active, the routine
may use `Tasks.parallelDeadline(routeTask, companionTask)`. The route then owns the group's terminal
outcome and every start-attempted companion is asked to cancel when it ends; only still-active
companions perform cleanup. Each companion must already implement safe active cancellation;
persistent scoring, intake, flywheel, or aim requests remain `PhoenixCapabilities`/service state.
The checked-in placeholder routines do not yet need this composition, so Phoenix does not fabricate
a production caller for it.

Raw Follower lifecycle calls are unsupported in Phoenix robot code because they bypass that
ownership and terminal truth. Paths are built through the runtime, route/guidance commands use the
adapter, the initial pose uses the runtime, and shutdown uses `PhoenixRobot.stop()`.

It intentionally does **not**:

- construct or configure a Pedro `Follower` (the project-specific runtime factory does that)
- choose alliance color
- choose a starting side
- choose a partner-coordination strategy
- build Pedro paths
- choose a specific autonomous routine
- expose route-script helpers from the robot container

Annotated Phoenix Driver Station entries live under:

```java
edu.ftcphoenix.robots.phoenix.opmode
```

Implementation code stays by role under packages such as:

```java
edu.ftcphoenix.robots.phoenix.autonomous
edu.ftcphoenix.robots.phoenix.autonomous.pedro
edu.ftcphoenix.robots.phoenix.tester
```

That means OpModes are easy to find from the Driver Station perspective, while path factories,
routine factories, tester implementations, and robot services remain grouped by responsibility.

The checked-in Pedro routes are explicitly `INTEGRATION_ONLY` placeholder geometry, so no current
static or selected match entry can arm. Real alliance/start/partner paths should be added in
`PhoenixPedroPathFactory` and deliberately classified `MATCH_READY` only after validation;
high-level strategy decisions should stay in
`PhoenixPedroAutoRoutineFactory`; reusable aim/shoot snippets should stay in `PhoenixAutoTasks`.
The existing private pre-park and final routine coordinators centralize Phoenix's conservative
route-status mapping and match-time park boundary; new strategies should change that robot-owned
mapping deliberately rather than changing the Pedro adapter or generic Task semantics.

## Driver Station setup UI

Phoenix should use the framework FTC UI helpers for future pre-start selectors rather than building
selection mechanics into `PhoenixRobot` or individual OpModes. The intended split is:

- `edu.ftcphoenix.fw.ftc.ui.SelectionMenu` for one visible list of choices
- `MenuNavigator` for nested setup flows, breadcrumbs, back/home behavior, and level display
- `SelectionMenus` for compact enum-backed setup screens
- `ConfirmationScreen` for final review before building a selected Auto routine
- `SummaryScreen` for locked summaries after a selected runtime has already been initialized
- `HardwareNamePicker` for tester hardware selection, with `X` as refresh so `B` can remain
  available for back/cancel in richer flows
- Phoenix robot code for the meaning of selected values, such as alliance, start position, partner
  plan, or autonomous strategy

`PhoenixPedroAutoSelectorOpMode` is the first robot-specific user of this split. It produces a
`PhoenixAutoSpec` during INIT, delegates to the same Pedro/Phoenix lifecycle glue used by static
Auto entries, and then replaces the wizard with a locked summary so the menu cannot drift away from
the installed routine. If START bypasses confirmation, it delegates once through that same
fail-closed path before starting. OpMode classes should still stay thin: they choose or collect the
spec, construct `PhoenixRobot`, and install the selected routine before starting it. They should not
become route scripts or hardware selection screens.

The strategy screen disables every selection whose exact path-factory fact is not match-ready and
never offers `PEDRO_INTEGRATION_TEST` as a match strategy. The confirmation preview uses the same
readiness policy as the shared base, and pressing START without confirmation reaches the same
fail-closed check rather than bypassing the selector.

`PhoenixPedroAutoOpModeBase` also treats INIT failures as retryable setup failures, not as
half-built robot states. Before each new build attempt it clears any previous error and asks the
active `PhoenixRobot` to stop its retained Auto drive owner and other partial runtime owners; after a
successful build it preserves the active spec, profile, paths, robot, and adapter as one consistent
set. A cleanup failure prevents a competing graph from being created. Construction/start errors
retain their original message, and any later cleanup error is attached and displayed separately
instead of being replaced by a generic drive/Pinpoint suggestion.

Pedro debug telemetry is composed into the same frame as Phoenix Auto telemetry. The OpMode adds
`auto.readiness`, route maturity, `auto.expectedPhysicalStartPedro` in explicit Pedro field
coordinates, actionable issues, `auto.spec`,
`auto.paths`, Pedro pose/busy rows, and the backend-neutral `route.status` before
`PhoenixRobot.updateAuto()` calls the Auto telemetry presenter, so the Driver Station sees one
coherent Auto status page per loop. The installed root is retained after completion, so its dynamic
Task name and outcome continue to identify normal pre-park work, local degradation, match-time
cutoff, suppressed park, active park, or the terminal park result even after the adapter's mutable
latest-route row changes.

Truthful route status deliberately does not decide what Phoenix should do next.
Phoenix now applies its explicit conservative continue/fallback/abort mapping in
`PhoenixPedroAutoRoutineFactory`; generic Task composition keeps its existing semantics. Natural
PHX-03 endings retain their narrow ownership cleanup: the scoring attempt cancels only a transient
shot it owns and pre-park disables the flywheel request enabled by this routine. When the outer
match budget actively cancels pre-park, that private owner additionally best-effort cancels its
active child, clears the Auto root's transient/feed and held intake/shoot/eject/flywheel requests
through `PhoenixCapabilities`, and applies immediate drive zero through the retained sink. It
attempts every safety action and does not release the park continuation after failed cleanup. This
is robot-owned match policy, not a callback added to generic `Tasks` or an imperative Plant write.


## Loop order

Phoenix keeps loop order explicit inside `PhoenixRobot.updateTeleOp()`:

```text
1. localization.update(clock)
2. targeting.update(clock)
3. controls.update(clock)
4. scoringPath.update(clock)
5. driveAssists.update(clock, scoringStatus)
6. drive.update(clock)
7. drive.drive(...)
8. telemetryPresenter.emitTeleOp(...snapshots...)
```

That order reflects ownership:

- lanes produce stable shared state first
- controls update operator intent
- the scoring path translates intent and writes scoring hardware
- services reshape drive behavior
- presenters explain the result

Phoenix keeps `updateAuto()` just as explicit:

```text
1. localization.update(clock)
2. targeting.update(clock)
3. autoDrive.update(clock)
4. autoRoutineLifecycle.update(clock)
5. scoringPath.update(clock)
6. telemetryPresenter.emitAuto(...with Auto task and scoring snapshots...)
```

Auto uses the same scoring path and targeting service, but swaps TeleOp drive-assist policy for the
retained Auto drive heartbeat plus the private lifecycle driver for one installed root. The
heartbeat runs after current
targeting state is available and before Tasks observe their exact `RouteExecution` status or select
their next drive behavior. A cycle-aware Pedro adapter makes the later generic `RouteTask` or
guidance update hook—or the update made by an exclusive timed-drive Task—a no-op in that same cycle,
while continuing hold, pose, and stopped-state updates during mechanism or wait Tasks. Its passive
localizer verifies that the Pinpoint snapshot belongs to the current Phoenix cycle before Pedro
computes or writes drive output.

A `followBuiltAtStart(...)` supplier therefore sees localization, targeting, and the follower pose
from that current cycle when the Task runner starts its phase. The new concrete route begins
advancing on the next owned heartbeat; no hidden update is added during path construction.

## Shutdown ownership

`PhoenixRobot.stop()` is the one public shutdown operation for both TeleOp and Auto. It is
idempotent and owns the complete Phoenix sequence: cancel behavior producers, stop scoring and drive
outputs, reset targeting, release vision, and clear the composition-root references. Initialization
failures use the same operation so every successfully constructed Phoenix owner is cleaned before
the original error is reported.

In Auto, task cancellation happens before the retained drive owner's final `stop()`. Pedro task
cancellation may already stop the adapter immediately; the later owner-level stop is intentionally
idempotent and leaves one stable motionless state. The OpMode therefore stops `PhoenixRobot`, not the
adapter separately. Mode clients still stop only resources that they did not supply to and transfer
into the robot lifecycle.

Phoenix Pedro production uses one physical pose authority. The selected profile configures the
single `PinpointOdometryPredictor`; an explicit `PedroFieldTransform.decodeInvertedFtc()` converts
pose origin/axes/heading and velocity axes at the integration boundary; and Pedro reads that
predictor passively after Phoenix localization. The default corrected estimator pushes accepted
AprilTag corrections back into the shared predictor, so targeting and path following see one pose
in the same downstream heartbeat. Disabling correction push is the explicit targeting-only policy;
telemetry keeps raw predictor pose, corrected global pose, and their translation/heading drift
visible in either mode.

## Recommended profile shape

```text
PhoenixProfile
  drive         -> FtcMecanumDriveLane.Config
  vision        -> PhoenixProfile.VisionConfig
  localization  -> FtcOdometryAprilTagLocalizationLane.Config
  field         -> fixed AprilTag layout
  controls      -> TeleOp control tuning
  driveAssist   -> shoot-brace / drive-assist tuning
  scoring       -> scoring-path mechanism config
  autoAim       -> scoring target catalog + shot table + aim tuning
  auto          -> Auto route/aim/wait timing + pre-park takeover time + red/blue Auto scoring tag ids
  calibration   -> human acknowledgements
```

This profile shape is the template future robots should copy.

## Anti-patterns this architecture avoids

### Letting the robot container become a control script

`PhoenixRobot` is allowed to wire objects together, expose `PhoenixCapabilities`, and choose loop
order. It should not quietly absorb detailed scoring, targeting, or route-specific strategy.

### Letting TeleOp or Auto touch raw internals directly

`PhoenixTeleOpControls` and auto plans should use `PhoenixCapabilities`, not `ScoringPath`
or `ScoringTargeting`.

### Treating capability families like framework lanes

The capability pattern is expected every year, but the family names and splits belong to the robot,
not to the framework.

## If you are building a future robot

Copy the ownership pattern, not Phoenix's season-specific behavior.

A good starting point is:

```text
MyRobot
  MyRobotProfile

  framework lanes
  shared field facts

  MyCapabilities
  MyTeleOpControls
  MyAutoPlan / MyAutoRoutine

  MyDriveAssistService
  MyTargetingService
  MyScoringPath / MySubsystem
  MyTelemetryPresenter
```

Then read the framework docs for the full split philosophy and role glossary.

## Spatial guidance and mechanism target planning notes

Phoenix currently uses the framework's Drive Guidance path for drivetrain-facing behaviors such as scoring aim. The same framework layer now separates three concepts that robot code should keep distinct:

1. **`SpatialQuery`** solves field/robot geometry: target points, facing errors, translation errors, and alternate solve lanes such as live AprilTags vs global localization.
2. **`DriveGuidancePlan` / `DriveGuidanceQuery`** consume spatial results and produce drivetrain omega/translation commands.
3. **`PlantTargets.plan()`** consumes plant-unit target requests and produces feasible requested targets for `Plant`s.

Phoenix scoring aim follows the drive path because the drivetrain turns the whole robot. A future turret, tray, arm, or extension should generally follow the Plant target path: robot-owned calibration/reference setup converts semantic intent or spatial geometry into the plant units exposed by the mechanism, then `PlantTargets.plan()` handles candidates, periodic equivalents, travel range, and fallback/hold policy. Physical readiness remains `Plant.atTarget(...)`.

For example, a future turret with its own camera would use one spatial query with two solve lanes: a direct turret-camera AprilTag lane and a global-pose fallback lane. The turret service would map the selected facing solution into a `PlantTargetRequest`, and `PlantTargets.plan()` would choose a reachable requested target under cable limits as part of the turret Plant's target source.

Calibration remains robot-owned. Homing switches, encoder zero offsets, ticks-per-turn constants, and cable-limit ranges should be established by the mechanism service and exposed through `PositionPlant`/Plant target context measurements and `ScalarRange`s.
