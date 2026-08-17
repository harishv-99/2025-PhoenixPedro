# Pedro Pathing integration

The Pedro integration is a narrow vendor boundary. Core Phoenix Tasks and robot capabilities do
not depend on Pedro types; the adapter owns conversion, follower lifecycle, route status, and the
final drivetrain stop.

## Production ownership

Author one data-only `PedroPathingRuntime.Config`, then construct one `PedroPathingRuntime` during
`FtcRobotOpMode.configure(program)`. `Config.defaults()` is a valid software baseline and a useful
place to start; replace its five robot facts with the adopting robot's reviewed Pinpoint,
Follower, Mecanum, path-constraint, and field-transform values:

```java
PedroPathingRuntime.Config pedroConfig = PedroPathingRuntime.Config.defaults();
pedroConfig.predictor = robotPinpointConfig;
pedroConfig.followerConstants = robotFollowerConstants;
pedroConfig.mecanumConstants = robotMecanumConstants;
pedroConfig.pathConstraints = robotPathConstraints;
pedroConfig.fieldTransform = robotFieldTransform;

PedroPathingRuntime pedro = PedroPathingRuntime.create(hardwareMap, pedroConfig);
```

`create(...)` is the sole production hardware-acquisition boundary. It raw-copies the complete
draft, validates the captured graph without consulting hardware or Pedro process globals, and then
constructs the graph from owner-local copies. Invalid authored configuration therefore fails
before hardware lookup, the Pinpoint reset request, motor output, Follower construction, or changes
to Pedro's mutable static state. Later edits to the caller's Config or any nested Pedro constants do
not retune the running owner.

`Config.copy()` is the raw deep-copy operation: it deliberately preserves null or invalid draft
facts and performs no validation. `Config.validatedCopy(context)` is the advanced data-only
preflight used by the separate native-tool factory; a null or blank context uses the canonical
`PedroPathingRuntime.Config` diagnostic path. Ordinary robot code calls `create(...)` and lets the
owner perform that same authoritative validation before effects.

The runtime's `MotionPredictor` is the only Pinpoint hardware owner, and its
`PedroPathingDriveAdapter` is the only Follower heartbeat and drivetrain writer. The runtime does
not export its mutable Follower. Use `pathBuilder()` for routes, `driveAdapter()` for lifecycle, and
`currentPedroPose()` when start-time route geometry needs one defensive snapshot of the Follower's
already-cached Pedro pose. That pose read does not poll hardware or advance the Follower.

Pinpoint construction requests one reset without sleeping. Until a completed poll reports `READY`
and publishes current finite pose and velocity, the passive localizer rejects the heartbeat and the
adapter fail-stops drive. Its diagnostic includes the predictor's cached last device status; it
never polls the device through a second owner.

```java
PedroPathingRuntime pedro =
        PedroPathingRuntime.create(
                hardwareMap,
                Constants.phoenixAutoRuntimeConfig(profile));

robot.declareAuto(
        program,
        pedro.driveAdapter(),
        pedro.motionPredictor(),
        frozenEligibleTagIds,
        BooleanSource.constant(true),
        BooleanSource.constant(false),
        () -> pedro.setStartingPose(frozenPedroStartPose())
);
program.rootTask(rootRoutine);
```

Phoenix's `Constants.phoenixAutoRuntimeConfig(profile)` is only a pure project mapper. It snapshots
the profile's Pinpoint and drivetrain slice, combines it with fresh checked-in Pedro tuning, and
returns an independent runtime Config. It constructs no hardware and is not a second runtime
factory. Unrelated vision, scoring, targeting, or Auto profile sections are outside that mapping;
the runtime remains the authoritative whole-Config validation boundary.

Those boolean sources keep ordinary Auto aim enabled with no manual override. Advanced direct
assembly can provide other clock-aware targeting policies through the same managed declaration;
the adapter heartbeat and cleanup still belong to `RobotProgram`.

The managed service calls the adapter once at START and once per active loop. Route and guidance
Tasks may call the same adapter hook; `PedroPathingDriveAdapter` deduplicates by
`LoopClock.cycle()`, including updates hidden inside vendor calls.

At START, the program has already frozen any `RobotProgram.Prestart` policy and reset the shared
clock. Phoenix then applies the frozen start pose, updates vision readiness, localization,
targeting, and the Pedro heartbeat, starts/first-updates the root Task, and finally realizes robot
outputs. There is no separate Pedro or Auto clock.

## Routes

Use `RouteTasks.follow(...)` when fixed geometry is already known during construction. Use
`RouteTasks.followBuiltAtStart(...)` when geometry genuinely depends on the current pose or another
fact that should be sampled exactly once when that route phase begins.

If an entire selected root depends on data frozen at FTC START, wrap only the Task construction:

```java
program.rootTask(Tasks.buildAtStart(
        "selectedAuto",
        () -> buildRoutine(frozenSpec())
));
```

`Tasks.buildAtStart(...)` is not a hardware factory. Hardware, resource owners, and stable services
must already be registered during `configure(program)`.

Each route start returns a per-start execution. Completion remains attached to that exact start and
is classified as endpoint success, timeout/stall, interruption, replacement, failure, or unknown.
Never infer success solely from Pedro becoming idle.

## Cleanup

`RobotProgram` cancels the root first, stops downstream mechanism outputs, then stops services in
reverse declaration order. Phoenix's managed Pedro service makes `stop()` write physical zero
immediately, including after reentrant callbacks, before targeting reset and vision close finish.
The adapter's stop and same-cycle deduplication are idempotent.

After validation succeeds, an SDK or vendor constructor can still fail after partial hardware
effects. Construction creates the stoppable Mecanum drivetrain first and best-effort breaks it if a
later Pinpoint, passive-localizer, or Follower step fails; a distinct stop failure is suppressed on
the original failure. A Mecanum constructor that itself throws supplies no handle to stop, Pinpoint
has no close/rollback operation, and a failing Follower may already have touched Pedro's static
controller switches. Treat construction failure as terminal for that OpMode and restart only after
correcting the cause.

## Advanced and tool-only seams

`new PedroPathingDriveAdapter(completedFollower)` is the advanced custom/portable-host seam for a
caller that has already constructed a Follower. It acquires no hardware and creates no Pinpoint
owner. After adaptation, that host must route heartbeat and stop through the adapter instead of
creating peer lifecycle writers. Assembly code already holds the supplied vendor reference, so the
adapter intentionally adds neither a Follower getter nor a parallel pose-observation API.

Generated Pedro tuning OpModes and `PedroTest` instead use the project package's package-local
native-Follower factory. Their native `PinpointLocalizer` and raw `Follower.update()` lifecycle are
required by Pedro's stock tools and are exclusive to those OpModes. That tool graph is neither the
completed-Follower adapter seam nor a second production option, and it must never coexist with a
production runtime.

## What validation does not prove

Config validation proves that the captured software graph is complete, finite, internally
coherent, and representable by the pinned Pedro implementation. It does not prove that motor names
identify the intended ports, directions match the chassis, Pinpoint pods are correctly placed or
ready, follower tuning is stable, the field transform matches physical placement, constraints leave
safe stopping distance, a route is clear, or STOP produces the expected physical result. Verify
those facts on the adopting robot with conservative limits, clear space, and an operator ready to
stop it.
