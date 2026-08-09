# Pedro Pathing integration

The Pedro integration is a narrow vendor boundary. Core Phoenix Tasks and robot capabilities do
not depend on Pedro types; the adapter owns conversion, follower lifecycle, route status, and the
final drivetrain stop.

## Production ownership

Construct one `PedroPathingRuntime` during `FtcRobotOpMode.configure(program)`. Its
`MotionPredictor` is the only Pinpoint hardware owner, and its `PedroPathingDriveAdapter` is the
only Follower heartbeat and drivetrain writer.

```java
PedroPathingRuntime pedro =
        Constants.createPhoenixAutoRuntime(hardwareMap, profile);

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

Generated Pedro tuning OpModes may use their native Follower/Pinpoint graph. That graph is tool-only
and must never be combined with the production runtime, because doing so creates competing Pinpoint
owners and drivetrain heartbeats.
