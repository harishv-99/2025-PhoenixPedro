# Your first Pedro Auto

**Learning mode:** Architecture reference

This optional module presents the maintained fixed
route slice, composition, outcome policy, and software verification without claiming robot safety.

## Goal

Understand the checked-in managed Pedro reference in software: one fixed route, one action after
confirmed completion, and one safe timeout fallback. This optional track does not require enabling
the route or moving a robot.

**Time:** 45–60 minutes for the software walkthrough. Installing Pedro and validating drivetrain,
localization, follower tuning, constraints, and field placement are separate projects.

**Prerequisites:**

- the Task vocabulary from
  [`Tasks and autonomous`](<learn-sushi/Tasks and Autonomous.md>).

No physical Pedro setup is required for this walkthrough. Before any later robot-owned physical
test, complete Pedro's official [`tuning`](https://pedropathing.com/docs/pathing/tuning) and
[`localization`](https://pedropathing.com/docs/pathing/tuning/localization) workflows, understand
Pedro's official [`coordinate system`](https://pedropathing.com/docs/pathing/reference/coordinates),
and complete the adopting robot's calibration guide.

## Files you will create

Copy the six maintained files below into one robot-owned package bubble. Keep the host disabled;
the software checkpoint compiles and tests this graph without authorizing motion.

- [`BasicPedroProfile.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/robot/BasicPedroProfile.java>) — fresh local
  Pedro and intake configuration plus the false-by-default motion permission;
- [`BasicPedroAutoPaths.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/autonomous/BasicPedroAutoPaths.java>) — physical
  start and fixed route;
- [`BasicPedroAutoRoutine.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/autonomous/BasicPedroAutoRoutine.java>) —
  route Task and outcome policy;
- [`BasicPedroAutoRobot.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/robot/BasicPedroAutoRobot.java>) — managed
  service, output, and root declarations;
- [`BasicPedroAutoMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/capability/intake/BasicPedroAutoMechanism.java>) —
  the reference action capability;
- [`BasicPedroAutoExample.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/opmode/BasicPedroAutoExample.java>) —
  disabled FTC host that selects the local profile; the composition root owns status and presenter
  registration.

## Safety

- The checked-in path is a test route, not a match routine.
- A declared start pose is a software coordinate fact. It does not prove where the robot was placed.
- Keep the example `@Disabled` throughout this software walkthrough.
- `BasicPedroProfile.current()` is a complete software baseline, not reviewed configuration for a
  physical robot. Its `allowRobotMotion` permission remains false.
- The checked-in Mecanum `maxPower = 0.25` is only initial drivetrain data. Pedro 2.1.2 restores
  its Follower's separate `globalMaxPower` to `1.0` when `followPath(...)` starts, so `0.25` is not
  an autonomous-route power cap. Only the false permission blocks checked-in route construction
  and motion; `@Disabled` separately keeps the entry off the Driver Station list.
- A runtime Config passing software validation does not prove motor direction, Pinpoint placement
  or readiness, follower tuning, field alignment, route clearance, stopping distance, or physical
  STOP.
- A later physical test needs its own reviewed start, heading, route, reduced first-test limits,
  odometry, stopping distance, clear space, and operator on STOP.
- Never enable the host on another robot merely because the project compiles.

## 1. Start with the geometry

### Critical code

`BasicPedroAutoPaths` declares a 12-inch line in Pedro field coordinates:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/autonomous/BasicPedroAutoPaths.java -->
```java
private static final double START_X_INCHES = 24.0;
private static final double START_Y_INCHES = 24.0;
private static final double START_HEADING_RAD = 0.0;
private static final double END_X_INCHES = 36.0;
private static final double END_Y_INCHES = 24.0;
private static final double END_HEADING_RAD = 0.0;
```

Its constructor builds that fixed route through the configured runtime:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/autonomous/BasicPedroAutoPaths.java -->
```java
practiceRoute = requiredRuntime.pathBuilder()
        .addPath(new BezierLine(pedroStartPose, pedroEndPose))
        .setLinearHeadingInterpolation(
                pedroStartPose.getHeading(),
                pedroEndPose.getHeading()
        )
        .build();
```

**What to notice**

- Fixed geometry is constructed eagerly, before the Task starts.
- Inches, radians, and the robot footprint remain visible physical facts.

**Key APIs:** Pedro `Pose` names field geometry; `PedroRuntime.pathBuilder()` creates the robot-owned
fixed route.

Fixed geometry is built during configuration. Route geometry that genuinely depends on a live
fact uses the separately documented start-time route factory; this first route does not need it.
Such a factory reads `runtime.currentPedroPose()` once for a defensive snapshot of Pedro's cached
pose. It never obtains the mutable Follower or advances localization while building geometry.

Before changing any number, draw the start, end, heading, robot's relevant physical boundary in its
route-running mechanism state, and clear stopping area. Names ending in `INCHES` or `RAD` make the
units visible. Use Pedro's
[`coordinate reference`](https://pedropathing.com/docs/pathing/reference/coordinates) to interpret
the field origin, axes, and heading; a pose applies to the adopting localizer's tracked robot
reference point, not automatically to a bumper corner.

If a later parking lesson models that boundary as a conservative rectangle, use
`RobotFrameRectangle2d.fromRobotFrameBoundsInches(...)` when the tracked origin is off-center.
That rectangle can answer an exact rectangle-versus-box question at one pose; it does not prove a
clear route, collision clearance, physical support, or an official season score. See the
known-clear-box composition in [`Drive Guidance`](<../drive-vision/Drive Guidance.md#known-clear-rectangular-parking-assist>).

## 2. Follow the route as a Task

### Critical code

`BasicPedroAutoRoutine.build(...)` creates the route Task with a diagnostic name and a finite Task
timeout:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/autonomous/BasicPedroAutoRoutine.java -->
```java
Task followPracticeRoute = RouteTasks.follow(
        "BasicPracticeRoute",
        Objects.requireNonNull(routes, "routes"),
        Objects.requireNonNull(practiceRoute, "practiceRoute"),
        ROUTE_TIMEOUT_SEC
);
```

**What to notice**

- One Task owns one route start and a finite timeout boundary.
- Robot code does not drive the raw vendor lifecycle beside the adapter.

**Key APIs:** `RouteTasks.follow(...)` wraps one per-start execution; its returned `Task` is
single-use cooperative work.

Robot code does not call the raw Pedro follower's start, update, or break methods. The route adapter
owns that boundary and reports the result for this exact run.

## 3. Make the result policy explicit

### Critical code

The route ending determines which mechanism behavior is allowed:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/autonomous/BasicPedroAutoRoutine.java -->
```java
return Tasks.branchOnOutcome(
        followPracticeRoute,
        requiredMechanism.collectTask(COLLECT_DURATION_SEC),
        requiredMechanism.idleTask()
);
```

**What to notice**

- Confirmed success, recoverable timeout, and cancellation-like failure lead to different actions.
- Robot-owned Auto policy interprets retained evidence; the integration does not choose strategy.

**Key APIs:** `Tasks.branchOnOutcome(...)` selects success and timeout continuations while failing
closed for cancellation-like or failed endings.

For this routine:

| Route result | Routine decision |
|---|---|
| Confirmed completion | Run one fresh 0.50-second collection Task. |
| Follower or Task timeout | Start the explicit idle fallback. |
| Cancellation-like or failed ending | Abort without starting either branch. |

“The follower is no longer busy” is not enough evidence for the success action. The route Task
retains a truthful status; the robot routine chooses what that status means. A successful timeout
fallback makes the outer root outcome `SUCCESS` without rewriting the route fact. For example, a
Task deadline remains `routeStatus=TASK_TIMEOUT` while the completed wrapper reports
`rootOutcome=SUCCESS`.

## 4. Let the managed program own Pedro's lifecycle

`BasicPedroAutoRobot` declares:

1. one service that owns localization, the Pedro heartbeat, exact-start pose application, and drive
   stop;
2. one Plant-backed mechanism output; and
3. one root routine Task.

Copy its Pedro service heartbeat, START ordering, and cleanup ownership as the lifecycle template.
The lesson does not require a separate clock, runner, manual follower update, or FTC callback. The
managed program updates the service before the route Task and stops the output and drive owner
during cleanup. This single-purpose Auto root also selects its one routine; a multi-client robot
keeps reusable capabilities in its root and Auto strategy in an Auto-owned routine instead.

## 5. Review the local profile and use the one root boundary

### Critical code

`BasicPedroAutoExample` selects one fresh, local profile and gives the complete active
configuration to the composition root:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/opmode/BasicPedroAutoExample.java -->
```java
robot = testRobotFactory == null
        ? new BasicPedroAutoRobot(
                program,
                hardwareMap,
                BasicPedroProfile.current()
        )
        : Objects.requireNonNull(
                testRobotFactory.apply(program),
                "testRobotFactory.apply(program)"
        );
```

**What to notice**

- A fresh profile keeps route, follower, mechanism, and permission facts together.
- The composition root declares one managed service/output/root graph.

**Key APIs:** `RobotProgram` owns the managed lifecycle; `BasicPedroAutoRobot` composes Pedro and the
mechanism once.

## Complete working slice

<details>
<summary>Complete working slice: BasicPedroAutoExample.java</summary>

The complete disabled FTC host shows the package, imports, annotations, ordinary construction, and
test-only seam. The other five complete owners are presented in the linked
[Pedro autonomous reference](<../examples/Pedro Autonomous Reference.md>).

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/opmode/BasicPedroAutoExample.java -->
```java
package edu.ftcsushi.robots.examples.pedro.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Objects;
import java.util.function.Function;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.robots.examples.pedro.robot.BasicPedroAutoRobot;
import edu.ftcsushi.robots.examples.pedro.robot.BasicPedroProfile;

/**
 * Disabled, compiling FTC host for the independent basic Pedro Auto reference.
 *
 * <p>The adjacent {@link BasicPedroProfile} keeps the example's local software baseline and
 * false-by-default motion permission visible without borrowing an adopting robot's hardware
 * configuration or Pedro constants. This generic example demonstrates the complete underlying
 * {@link FtcRobotOpMode}/{@link RobotProgram} grammar without constructing another robot.</p>
 */
@Autonomous(name = "FW Pedro Auto: Basic Reference", group = "Framework Examples")
@Disabled
public final class BasicPedroAutoExample extends FtcRobotOpMode {

    private final Function<RobotProgram, BasicPedroAutoRobot> testRobotFactory;
    private BasicPedroAutoRobot robot;

    /** Creates the disabled Driver Station entry using the independent local example profile. */
    public BasicPedroAutoExample() {
        testRobotFactory = null;
    }

    /** Test-only construction seam; it is deliberately not a public extension API. */
    BasicPedroAutoExample(
            Function<RobotProgram, BasicPedroAutoRobot> testRobotFactory
    ) {
        this.testRobotFactory = Objects.requireNonNull(testRobotFactory, "testRobotFactory");
    }

    /** Construct robot-specific owners and declare their managed roles during FTC INIT. */
    @Override
    protected void configure(RobotProgram program) {
        robot = testRobotFactory == null
                ? new BasicPedroAutoRobot(
                        program,
                        hardwareMap,
                        BasicPedroProfile.current()
                )
                : Objects.requireNonNull(
                        testRobotFactory.apply(program),
                        "testRobotFactory.apply(program)"
                );
    }
}
```

</details>

`BasicPedroProfile.current()` returns three fresh authored answers: the complete
`PedroPathingRuntime.Config` in `pedro`, the example mechanism's Config in `intake`, and
`allowRobotMotion = false`. The two long-lived owners, not the profile, defensively snapshot their
own active Config. Later edits to the short-lived profile cannot retune either running owner.

The checked-in values are software-valid examples. They include the framework Pinpoint baseline,
fresh Pedro follower/constraint/field-transform data, named Mecanum motors, the Pedro baseline
left-REVERSE/right-FORWARD directions, initial `maxPower = 0.25`, brake mode enabled, and one
`"intakeMotor"` at `Direction.FORWARD` with collection power `0.20`. Replace and physically review
every active fact for the adopting robot. Pedro's route-time `globalMaxPower` reset means the
initial `0.25` value does not limit `followPath(...)`; review and tune Pedro's actual route behavior
before any motion test. The `useBrakeModeInTeleOp = true` baseline likewise does not define Auto
route braking.

The root first requires the false-by-default permission and rejects an intake name that resolves to
one of the four drive motor names. It then crosses the sole
`PedroPathingRuntime.create(hardwareMap, profile.pedro)` hardware boundary. The runtime snapshots
and validates its complete graph before hardware lookup, the non-blocking Pinpoint reset request,
motor output, Follower construction, or Pedro-global mutation. It is registered with the managed
program before path or intake construction, so a later intake failure receives best-effort managed
drive cleanup. The intake owner separately snapshots and validates `profile.intake` before its own
lookup.

For another robot, keep the path/routine/declaration pattern and edit the local profile's Pinpoint,
Follower, Mecanum, path-constraint, field-transform, and intake facts. Keep
`allowRobotMotion = false` until the complete active graph has been reviewed for a supervised test.
Do not create another runtime factory, import season hardware values, or create a second mechanism
owner merely to fit the reference.

## Verify the slice

Keep the example disabled while adapting configuration and geometry:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest
```

Confirm that the host, path, routine, and capability compile together. In source, trace how the
presenter would report the expected physical start during a later enabled integration test.

## 7. Stop at the software checkpoint

Leave `BasicPedroAutoExample` marked `@Disabled`. Trace the route from its declared Pedro pose to its
endpoint, identify the result policy, and confirm the host, path, routine, capability, and tests all
compile. No Driver Station entry or robot motion is expected for this lesson.

A team may later create a robot-owned, supervised integration test after completing the official
Pedro tuning/localization work and that robot's local calibration checklist. That separate test
must define reviewed reduced first-test limits, the localizer's physical reference point, start
placement, clear stopping area, and immediate STOP plan before enabling any route host. Do not turn
this generic reference into a motion test merely by removing `@Disabled`.

## Expected checkpoint

- You can identify the start pose, route geometry, route timeout, success action, and timeout
  fallback without reading Pedro internals.
- You can find every active runtime and intake configuration answer in `BasicPedroProfile.current()` and
  explain why `allowRobotMotion` remains false.
- The project compiles with one managed `FtcRobotOpMode` path.
- Robot code does not manually update or break the raw follower.
- The generic example remains disabled and no physical run is required.
- You can name the additional tuning, localization, coordinate, constraint, placement, and STOP
  evidence a later robot-owned integration test must supply.

## Common problems

**The OpMode is missing.**

That is the expected source-only result: the example is checked in with `@Disabled`. A later
robot-owned integration test, not this walkthrough, owns any decision to expose a Driver Station
entry after its complete readiness review.

**INIT says `BasicPedroProfile.allowRobotMotion` must be true.**

That is the expected checked-in result. Leave the permission false for this software walkthrough.
A later robot-owned test may set it true only after replacing and reviewing the complete active
runtime and intake configuration, route placement and clearance, and physical STOP plan.

**A later integration test starts in the wrong place in software.**

Confirm the declared Pedro start and exact-start pose application use the same coordinates and
heading. A software pose assignment does not correct physical placement.

**A later integration test's physical motion does not match the drawn route.**

Press STOP. Recheck motor directions, odometry directions and offsets, coordinate convention,
Follower constraints and route-time `globalMaxPower`, and physical placement before changing
route-policy code. The profile's initial Mecanum `maxPower = 0.25` is not a `followPath(...)` cap.

Passing runtime Config validation is not evidence that those physical facts are correct. It proves
only that the captured software values are complete, finite, mutually coherent, and representable
by this pinned Pedro integration.

**The route ends but the mechanism action does not start.**

Inspect the retained route status in telemetry. Timeout, interruption, replacement, cancellation,
failure, and unknown endings are not success and intentionally do not run the position-dependent
action.

**The mechanism idles after a timeout.**

That is the reference's explicit safe fallback. Change timeout strategy only in the robot-owned
routine, with a clear reason and a physically validated replacement action.

**Can I copy only the short routine method?**

No. A safe Pedro program also needs the verified runtime, starting pose, stable managed service,
mechanism owner, root declaration, telemetry, and cleanup represented by the linked files.

**Full lifecycle reference:** [`Pedro autonomous reference`](<../examples/Pedro Autonomous Reference.md>)

**Return to:** [`Build a robot step by step`](<Build a Robot Step by Step.md>) or the
[`Sushi documentation hub`](<../README.md>)
