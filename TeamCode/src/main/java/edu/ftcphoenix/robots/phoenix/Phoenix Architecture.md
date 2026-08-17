# Phoenix Architecture

This document describes the production framework-based robot under
`edu.ftcphoenix.robots.phoenix`. It is the complete reference design for applying the framework's
ownership, lifecycle, capability, targeting, and autonomous contracts to one competition robot.

The repository-wide [`Framework Principles`](<../../fw/Framework Principles.md>) remain the primary
design authority. Use the package [`README`](<README.md>) for a shorter map of what to read and
where to make common robot changes.

## Big picture

Phoenix separates five concerns:

```text
mode client
  -> robot capability vocabulary
  -> robot policy/services
  -> mechanism realization
  -> framework/FTC/vendor boundaries
```

- `PhoenixRobot` is the composition root. It constructs and privately retains the complete
  mechanism graph for one mode and declares its managed lifecycle roles.
- `PhoenixCapabilities` is the shared, mode-neutral intent/status vocabulary used by TeleOp and
  Auto.
- `PhoenixTeleOpControls` owns driver/operator meanings.
- `PhoenixAlliance` is the mode-neutral match-alliance fact shared by TeleOp and Auto.
- `PhoenixAutoSpec`, path factories, and routine factories own the remaining autonomous selection
  and strategy.
- `.scoring.PhoenixTargeting` publishes targeting facts and guidance.
- `.scoring.PhoenixScoring` is the one scoring mechanism owner. It owns requests, feed policy, the
  shot queue, all four private scoring Plants, readiness evidence, update order, status production,
  and terminal stop.
- `RobotProgram` owns FTC lifecycle forwarding, one `LoopClock`, bindings, Tasks, outputs,
  presenters, telemetry commit, and fail-stop cleanup.

Reusable FTC and Android adapters stay in `edu.ftcphoenix.fw.ftc`; vendor bridges stay in the
narrow `edu.ftcphoenix.fw.integrations` edge; FTC-bound examples and diagnostics stay in
`edu.ftcphoenix.fw.tools`. Phoenix is the robot application edge: its composition and hardware
owners may construct those adapters from FTC resources, and its explicit path edge may build Pedro
geometry. Capability code and policy outside those construction edges depend on Phoenix contracts
rather than raw FTC devices or Pedro Followers. Dependencies point from these explicit edges into
the protected framework core, never from the core back into an edge.

## One managed OpMode grammar

TeleOp and Auto are parallel clients of `FtcRobotOpMode`/`RobotProgram`. Annotated OpModes do not
manually forward FTC callbacks.

TeleOp has one override:

```java
@Override
protected void configure(RobotProgram program) {
    new PhoenixTeleOpProgram(this, program, PhoenixAlliance.RED);
}
```

The entry makes its standalone default explicit. The package-private program owner copies
`PhoenixProfile`, installs one visible `PhoenixTeleOpPrestart`, constructs `PhoenixRobot`, and calls
`declareTeleOp(program, prestart.eligibleScoringTagIds())`. It then consumes the optional match
snapshot after localization exists, using its alliance only to seed the still-editable INIT menu.
The prestart's value at FTC START is the sole frozen targeting-eligibility authority.

Every production or diagnostic Phoenix-season Auto extends `PhoenixAutoOpMode` and returns only a
setup:

```java
@Override
protected PhoenixAutoSetup autoSetup() {
    return PhoenixAutoSetup.fromFixedSpec(
            PhoenixAutoSpec.audienceSafe(PhoenixAlliance.RED),
            PhoenixReadiness.AutoPurpose.MATCH_AUTO
    );
}
```

The parallel selector factory is
`PhoenixAutoSetup.fromInitSelection(defaultSpec, purpose)`. Both feed the same internal
`PhoenixAutoProgram` declaration. There is no raw Phoenix Auto OpMode base, private Auto clock,
private Auto runner, manual telemetry commit, or public start/update/stop alias on `PhoenixRobot`.
The disabled `robots.examples.pedro.BasicPedroAutoExample` is a generic new-robot reference using
the same managed `FtcRobotOpMode` grammar; it is not a Phoenix-season entry or alternate Phoenix
hardware graph.

## Construction and ownership

An ordinary Phoenix mechanism constructor receives `HardwareMap` plus data-only configuration,
defensively snapshots that configuration, constructs and privately owns its final resolver/Plant
graph, and owns update/stop. The OpMode does not prebuild or inject Plants.

`PhoenixTeleOpControls` constructs its stable driver and operator sources without registering
behavior. After `PhoenixCapabilities` exists, `PhoenixRobot` calls
`controls.bind(program.callbackBindings(), capabilities)` exactly once. This explicit boundary keeps
callback-graph mutation out of constructors while the managed program retains the only binding
heartbeat.

`PhoenixRobot` is constructed once for one mode. `declareTeleOp(...)` or `declareAuto(...)` may be
called once. Both modes retain the complete defensive `PhoenixProfile`; neither constructs an
alliance-filtered hardware profile. Their prestart owners instead supply a source that becomes one
singleton scoring-tag set after their `PhoenixAlliance` freezes. The ordinary Phoenix Auto host
supplies always-enabled aim and no override, while the same single `declareAuto(...)` boundary
retains clock-aware enable and override sources for an advanced direct assembly; neither choice
owns a second lifecycle.

Auto construction happens once during FTC INIT:

1. invalidate an old Phoenix match handoff and snapshot `PhoenixProfile`;
2. register one data-only `PhoenixAutoPrestart`;
3. construct one `PedroPathingRuntime`;
4. register the final-pose-plus-frozen-alliance match handoff transaction when applicable;
5. transfer the Pedro drive owner into Phoenix's managed service and construct vision,
   localization, targeting, and scoring once;
6. declare one root Task;
7. register additive presenters.

A construction/configuration failure is terminal for that OpMode instance. Correct the problem and
restart the OpMode. The selector never retries or replaces a hardware graph.

## Prestart selection and readiness

`RobotProgram` accepts at most one aggregated `Prestart` role:

```java
public interface Prestart {
    void update(LoopClock clock);
    StartDisposition freezeForStart();
}
```

Phoenix has two package-private prestart owners. TeleOp shows one RED/BLUE screen with the entry's
visible RED default. Gamepad 1's D-pad directly edits the highlighted draft during INIT—there is no
`A` confirmation—and a fresh Auto handoff may seed—but never freeze—the
same visible draft. Auto selects only alliance, start position, and strategy, then shows a read-only
summary. It has no unused partner-plan field and no second confirmation state; FTC START is the sole
freeze boundary for both modes.

These owners contain only menu bindings, selected data, previews, and readiness. They own no camera,
motor, Plant, Pedro heartbeat, or stop callback.

During INIT the order is clock, prestart, presenters, commit. At START, prestart freezes once before
the clock reset:

- `READY` starts services, root Task, and outputs normally.
- `BLOCKED` starts none of those and continues only clock plus presenters.

`PhoenixReadiness.allianceScoringTarget(...)` gives both modes the same selected-alliance catalog
and fixed-layout validation. `PhoenixReadiness.pedroAuto(...)` adds Auto purpose, calibration,
starting-contract, and route-maturity checks. Target visibility and game-piece uncertainty are not
readiness. Current competition routes are explicitly `INTEGRATION_ONLY`, so match entries remain
fail-closed; the named Pedro test entry may run them with a persistent warning.

## Root and route construction

Fixed route geometry is built eagerly when its spec is known. The INIT selector declares one lazy
root:

```java
program.rootTask(Tasks.buildAtStart(
        "phoenix.selectedAuto",
        () -> buildRoutine(prestart.frozenSpec())
));
```

`Tasks.buildAtStart(...)` defers only a fresh single-use Task graph. It is not a hardware or
resource factory. It invokes its supplier once at its own start boundary, retains the child before
starting it, and preserves child failures/cancellation/outcome.

Later route phases whose geometry depends on current pose use
`RouteTasks.followBuiltAtStart(...)`. That narrower factory samples live geometry once when that
route Task starts. `PhoenixPedroPathFactory` remains the one owner of Pedro geometry, route labels,
declared starting poses, and route maturity.

Each route start has one retained execution identity. The integration classifies endpoint success,
follower timeout/stall, interruption, replacement, failure, and unknown terminal state. `RouteTask`
retains that integration classification and adds Task-owned `TASK_TIMEOUT`, `CANCELLED`,
or fail-closed `FAILED` when applicable; `FAILED` can originate at either boundary. Robot routine
policy—not a generic Task sequence—decides continue, fallback, or abort.

## Pedro and localization

`PedroPathingRuntime` owns one `PinpointOdometryPredictor` and exposes a passive Pedro localizer
view of its snapshots. No second Pinpoint owner polls or resets the device.

Pinpoint construction issues one non-blocking reset. Pedro remains fail-stopped until a completed
`READY` poll supplies current finite pose and velocity. Managed TeleOp similarly holds its final
drive sink at physical zero until its owned predictor publishes the first current finite measured
pose after construction reset. That startup gate latches once; a later localization fault disables
localization-dependent assists, but ordinary robot-centric manual drive remains available.

At a permitted START:

```text
freeze spec
-> reset shared LoopClock
-> apply frozen Pedro starting pose
-> vision readiness
-> localization
-> targeting
-> Pedro heartbeat
-> root Task start + first update
-> scoring output
```

The adapter keeps one stable heartbeat throughout the active OpMode, even while a mechanism/wait
Task runs. Route and guidance Tasks may call the same hook; the adapter deduplicates every call by
`clock.cycle()`, including vendor-hidden updates.

The adapter's `stop()` applies physical zero immediately, including after reentrant callbacks. It
does not merely stage zero for a future follower update.

## Targeting eligibility

Phoenix keeps the complete configured scoring-target catalog and fixed tag layout. This allows all
appropriate fixed tags to support pose solving and field localization.

`PhoenixTargeting` additionally receives one robot-owned
`Source<Set<Integer>> eligibleScoringTagIds`:

- TeleOp supplies the singleton mapped from its START-frozen alliance.
- Auto supplies the same singleton shape mapped from its START-frozen alliance.

At the first targeting update of a session, the owner validates and defensively freezes that set,
then builds one exact selector, fixed-layout subset, offset map, and guidance plan before reading the
sensor. Null, empty, null-member, unknown-ID, or missing fixed-layout entries fail fast. Inactive
catalog entries therefore cannot enter the current mode's plan. Eligibility is applied before
candidate preview and sticky selection, so an opposite-alliance tag cannot become either mode's
scoring target. This does not filter localization: its solver still owns the complete fixed tag
layout. `reset()` begins a new targeting session and requires a fresh drive graph.

The profile retains data-only `FixedTagFieldPoseSolver.Config` tuning. `PhoenixTargeting` validates
and captures it once as a configured `FixedTagFieldPoseSolver`, then passes that completed policy
into each guidance plan. Localization independently captures the nested solver Config in its
`AprilTagPoseEstimator`. Later edits to either profile draft cannot change a running solve policy.

Target visibility is handled inside the routine. The scoring attempt waits for its bounded
`waitForTargetSec`; timeout is retained truthfully, and `PhoenixPedroPreParkTask` selects the
explicit return/park fallback. No pre-reset INIT timestamp crosses the START clock epoch.

`PhoenixTargeting.aimTask(...)` accepts `DriveGuidanceTask.Config` as a mutable construction input,
but freezes those values when the aim Task is requested. The returned start-time wrapper therefore
cannot drift if its caller later edits the same Config. The wrapper may start later in the root
graph, after an earlier route or Task has already used the drive sink. When the wrapper starts after
targeting has published the selected-target guidance plan, it constructs the inner
`DriveGuidanceTask`; that framework boundary performs the authoritative numeric validation before
that inner aim Task invokes the sink. An invalid deferred aim configuration is consequently
reported at that inner-task start boundary rather than being silently treated as an unbounded
timeout. Request a fresh aim Task to use later configuration edits.

This snapshot and validation establish software consistency only. They do not prove safe physical
timeout or tolerance choices, successful aiming, drivetrain tuning, or target visibility.

## Capabilities and scoring

`PhoenixCapabilities` contains the common vocabulary used by both mode clients. Controls and Auto
routines request intent through capability families; neither reaches into `PhoenixScoring`,
`PhoenixTargeting`, or raw Plants. `PhoenixCapabilities.ScoringStatus` and
`PhoenixCapabilities.TargetingStatus` belong to that public vocabulary, so clients do not depend
on status types owned by either implementation class.

`PhoenixScoring` is the single owner for:

- intake/eject/flywheel/shoot intent;
- the feed queue and active feed pulse;
- target overlays and flywheel target selection;
- flywheel readiness prediction;
- all four final Plant resolvers and their update order;
- transient rollback and terminal stop.

Requests, execution policy, and realization remain visible as private field/method sections inside
that one class; they are not separate layer or flywheel-owner objects.
This keeps the conceptual layers without making a student assemble or navigate an object per layer.
`PhoenixScoring` receives `HardwareMap` plus a defensively copied `ScoringConfig`, builds its private
Plants, and exposes only the `PhoenixCapabilities.Scoring` vocabulary.

Feedback-aware Tasks name both command target and Plant because they write the target while the
Plant supplies completion feedback/provenance. Ordinary exact mechanisms retain only their Plant
and use its stable `commandTarget()` when constructing a command.

## Dedicated Panels tuning

**Phoenix: Tuning (Panels)** is a dedicated direct tester host, not another production mode,
scoring owner, or tester menu. Its `createTester()` method returns the framework workflow
`FtcPanelsTuners.velocityControl(...)`. Robot code declares only the tester name, finite allowed
test-target range, and a function that creates a fresh flywheel Plant. The motor name and FTC
device-managed controller are answered only inside that canonical Plant recipe; the framework
derives the exact controller handle from the completed Plant and cannot independently select
another motor.

That Plant comes from `PhoenixScoring.createFlywheelPlantForTuning(...)`, an explicitly advanced
assembly seam that calls the same canonical private flywheel recipe as production. The tuner owns
the returned Plant, its heartbeat, the narrow FTC controller-configuration handle, candidate
capture, evidence, and cleanup for that exclusive OpMode. It does not construct all of scoring,
share a production Plant, or write the raw motor beside a Plant. The shared item is the **recipe**,
not another flywheel owner object.

This is the opt-in pattern for another supported standard or FTC controller: keep the ordinary
production owner unchanged, expose one clearly named fresh-Plant recipe seam only for an exclusive
diagnostic, and declare the generic velocity or position workflow in a thin OpMode. There is no
generic tunable-regulator interface or registry, and mechanisms without a proven live-tuning need
add no tuning API.

Production TeleOp and Auto copy `PhoenixProfile` and never read Configurables. The tuning entry
starts from the checked-in scoring configuration, then captures the motor controller's resulting
session baseline. Panels **Update All** changes only mutable draft fields. After Update All finishes
and the operator verifies the displayed values, an A press causes the single OpMode heartbeat to
read all fields after a short best-effort quiescence interval and validate the resulting complete
tuple before any effect. Configurables provides no atomic batch marker, so the interval filters
ordinary in-flight changes but is not described as transport atomicity.

Each accepted A becomes an immutable numbered segment. Its transition label states whether the
controller, target, both, or neither changed from the prior accepted request. A first or post-B
velocity segment waits non-blockingly for finite feedback and truthful `atTarget(0.0)` before it
starts. A target-only hot change does not rewrite the controller. A controller change applies and
reads back the complete tuple before the new target is committed; B requests zero through the Plant
graph but leaves the Plant available for another cold start.

Exactly zero `autoStopAfterSec` disables automatic stop; a positive finite value starts at the new
segment boundary. Invalid or unstable drafts leave a running segment and its timer unchanged.
Telemetry keeps draft, captured request, controller readback, current segment, full session-local
history, and response metrics distinct. The displayed accepted readback values are what students
copy into checked-in configuration. Graph uses stable `tune.velocityControl.*` keys for segment,
target, measurement, error, settling, overshoot, droop, and disturbance facts. It does not invent
FTC controller output or internal terms that the SDK does not expose. Shot distance and make/miss
outcomes remain an external lab-sheet concern correlated by the displayed session and segment IDs.

Powered tuning requires exactly one Panels client. BACK, OpMode stop, disconnect, controller
failure, or another terminal failure stops the Plant and best-effort restores the captured session
baseline. The pinned FTC setter does not intentionally change target, mode, power, or direction,
but firmware controller-history behavior and physically bumpless hot transitions are not promised.
The complete student runbook is the
[`control tuning workflow`](<../../fw/docs/testing-calibration/Control Tuning Workflow.md>).

## Loop and telemetry order

The active Auto order is:

```text
LoopClock
-> vision/localization/targeting/Pedro service
-> bindings
-> Tasks
-> scoring output
-> presenters
-> one telemetry commit
```

TeleOp uses the same framework phases with its TeleOp services, controls, scoring output, and final
source-driven drive declaration. Services publish upstream facts. Tasks/controls change intent.
Outputs perform final physical realization. Presenters read retained snapshots only and never
clear or commit telemetry.

Optional `LoopPhaseProfiler` markers are observational. They do not add a heartbeat or include FTC
telemetry commit latency in a claimed robot phase.

## Cleanup and match handoff

`RobotProgram` first marks the program terminal, then performs:

```text
cancel Tasks
-> clear bindings
-> stop outputs in declaration order
-> stop services in reverse declaration order
```

For Auto, scoring therefore stops before the managed service writes Pedro zero, resets targeting,
and closes vision. Repeated/reentrant STOP is inert. Lifecycle `RuntimeException` preserves the
first failure and suppresses later cleanup failures; `Error` is not caught.

Match Auto declares one typed `stopHandoff(capture, publish, invalidate)` transaction. It
invalidates immediately. Only normal STOP from `ACTIVE` captures the cached final pose together
with the `PhoenixAlliance` frozen by that Auto. Publication occurs only after all cleanup succeeds.
Blocked, pre-start, failed, test, cleanup-failure, and publication-failure paths publish nothing and
invalidate.

TeleOp consumes a fresh snapshot once during INIT after localization exists. Its pose restores
localization and its alliance seeds the still-editable TeleOp draft. The operator may change that
draft before START. Missing, stale, or already-consumed snapshots leave the normally initialized
pose and visible RED draft unchanged.

## Profile shape

`PhoenixProfile` is data-only and copied by long-lived owners. Major sections include:

- `drive`: drivetrain hardware and scaling;
- `vision`: selected backend and camera rig;
- `localization`: predictor, AprilTag solve, and corrected-estimator policy;
- `field`: fixed field facts and tag layout;
- `autoAim`: alliance-to-scoring-tag mapping, complete target catalog, and targeting tolerances;
- `auto`: route/scoring budgets and fallback timing;
- `scoring`: mechanism hardware, Plants, readiness, and feed timing;
- `controls` and `driveAssist`: operator mapping and assist policy;
- `calibration`: explicit calibration acknowledgements.

Alliance, route choice, and strategy are not mutable profile filters. Auto carries all three in its
frozen `PhoenixAutoSpec`; TeleOp freezes its own `PhoenixAlliance` through
`PhoenixTeleOpPrestart`.

## Anti-patterns

Do not:

- put control scripts or route selection inside `PhoenixRobot`;
- construct a second Phoenix/Pedro hardware graph after INIT selection;
- override raw FTC callbacks in a Phoenix Auto entry;
- keep a private Auto clock, runner, telemetry commit, or cleanup path;
- make route Tasks the only Pedro heartbeat owner;
- filter the complete profile to choose an alliance target;
- let opposite-alliance observations enter preview/sticky target selection;
- make sensor visibility a structural START-readiness requirement;
- infer route success from a vendor idle flag;
- let presenters advance state;
- let Auto or TeleOp reach raw Plants or vendor Followers.
- read Panels Configurables from production TeleOp or Auto;
- duplicate the flywheel Plant recipe or write its raw motor from a tuning entry; or
- claim that a requested PIDF tuple, controller restore, or hot transition proves physical truth.

## Coordinate contract

Phoenix robot frame is right-handed: `+X` forward, `+Y` left, `+Z` up. Yaw and omega are
counter-clockwise positive. Distances are inches and angles are radians unless a name explicitly
states otherwise. Pedro/FTC conversions belong at the integration boundary.
