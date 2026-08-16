# Your first Pedro Auto

## Goal

Understand the checked-in managed Pedro reference in software: one fixed route, one action after
confirmed completion, and one safe timeout fallback. This optional track does not require enabling
the route or moving a robot.

**Time:** 45–60 minutes for the software walkthrough. Installing Pedro and validating drivetrain,
localization, follower tuning, constraints, and field placement are separate projects.

**Prerequisites:**

- the Task and Auto checkpoint from [`Your first Task and Auto`](<First Task and Auto.md>); and
- the ability to compile and read the checked-in Pedro reference files.

No physical Pedro setup is required for this walkthrough. Before any later robot-owned physical
test, complete Pedro's official [`tuning`](https://pedropathing.com/docs/pathing/tuning) and
[`localization`](https://pedropathing.com/docs/pathing/tuning/localization) workflows, understand
Pedro's official [`coordinate system`](https://pedropathing.com/docs/pathing/reference/coordinates),
and complete the adopting robot's calibration guide. For this repository's Phoenix robot, that is
the [`Phoenix Calibration Guide`](<../../../robots/phoenix/Phoenix Calibration Guide.md>).

**Files for this lesson:**

- [`BasicPedroAutoPaths.java`](<../../../robots/examples/pedro/BasicPedroAutoPaths.java>) — physical
  start and fixed route;
- [`BasicPedroAutoRoutine.java`](<../../../robots/examples/pedro/BasicPedroAutoRoutine.java>) —
  route Task and outcome policy;
- [`BasicPedroAutoRobot.java`](<../../../robots/examples/pedro/BasicPedroAutoRobot.java>) — managed
  service, output, and root declarations;
- [`BasicPedroAutoMechanism.java`](<../../../robots/examples/pedro/BasicPedroAutoMechanism.java>) —
  the reference action capability;
- [`BasicPedroAutoExample.java`](<../../../robots/examples/pedro/BasicPedroAutoExample.java>) —
  disabled FTC host for this repository's Phoenix hardware configuration.

## Safety

- The checked-in path is a test route, not a match routine.
- A declared start pose is a software coordinate fact. It does not prove where the robot was placed.
- Keep the example `@Disabled` throughout this software walkthrough.
- The checked-in host inherits the Phoenix robot's configured Pedro constraints; this lesson does
  not claim that those values are conservative for another robot or first motion.
- A later physical test needs its own reviewed start, heading, route, reduced first-test limits,
  odometry, stopping distance, clear space, and operator on STOP.
- Never enable the host on another robot merely because the project compiles.

## 1. Start with the geometry

`BasicPedroAutoPaths` declares a 12-inch line in Pedro field coordinates:

```java
private static final double START_X_INCHES = 24.0;
private static final double START_Y_INCHES = 24.0;
private static final double START_HEADING_RAD = 0.0;
private static final double END_X_INCHES = 36.0;
private static final double END_Y_INCHES = 24.0;
private static final double END_HEADING_RAD = 0.0;
```

Its constructor builds that fixed route through the configured runtime:

```java
practiceRoute = requiredRuntime.pathBuilder()
        .addPath(new BezierLine(pedroStartPose, pedroEndPose))
        .setLinearHeadingInterpolation(
                pedroStartPose.getHeading(),
                pedroEndPose.getHeading()
        )
        .build();
```

Fixed geometry is built during configuration. Route geometry that genuinely depends on a live
fact uses the separately documented start-time route factory; this first route does not need it.

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

`BasicPedroAutoRoutine.build(...)` creates the route Task with a diagnostic name and a finite Task
timeout:

```java
Task followPracticeRoute = RouteTasks.follow(
        "BasicPracticeRoute",
        routes,
        practiceRoute,
        ROUTE_TIMEOUT_SEC
);
```

Robot code does not call the raw Pedro follower's start, update, or break methods. The route adapter
owns that boundary and reports the result for this exact run.

## 3. Make the result policy explicit

The route ending determines which mechanism behavior is allowed:

```java
return Tasks.branchOnOutcome(
        followPracticeRoute,
        requiredMechanism.collectTask(COLLECT_DURATION_SEC),
        requiredMechanism.idleTask()
);
```

For this routine:

| Route result | Routine decision |
|---|---|
| Confirmed completion | Run one fresh 0.50-second collection Task. |
| Follower or Task timeout | Start the explicit idle fallback. |
| Cancellation-like or failed ending | Abort without starting either branch. |

“The follower is no longer busy” is not enough evidence for the success action. The route Task
retains a truthful status; the robot routine chooses what that status means.

## 4. Let the managed program own Pedro's lifecycle

`BasicPedroAutoRobot` declares:

1. one service that owns localization, the Pedro heartbeat, exact-start pose application, and drive
   stop;
2. one Plant-backed mechanism output; and
3. one root routine Task.

Treat that class as the lifecycle template. The lesson does not require a separate clock, runner,
manual follower update, or FTC callback. The managed program updates the service before the route
Task and stops the output and drive owner during cleanup.

## 5. Use the correct host boundary

`BasicPedroAutoExample` is the only reference file tied to the Phoenix robot configuration in this
repository. It builds the runtime with:

```java
PedroPathingRuntime builtRuntime = Constants.createPhoenixAutoRuntime(
        hardwareMap,
        profile
);
```

If a later robot-owned integration test uses that configuration-owning Phoenix robot, it must review
the profile, runtime, placement, limits, and mechanism facts before enabling its host.

For another robot, keep the path/routine/declaration pattern but replace this host with that robot's
verified Pedro runtime factory and existing mechanism capability. Do not import Phoenix hardware
values or create a second mechanism owner merely to fit the reference.

## 6. Compile while the host remains disabled

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
- The project compiles with one managed `FtcRobotOpMode` path.
- Robot code does not manually update or break the raw follower.
- The generic example remains disabled and no physical run is required.
- You can name the additional tuning, localization, coordinate, constraint, placement, and STOP
  evidence a later robot-owned integration test must supply.

## Common problems

**The OpMode is missing.**

That is the expected software-course result: the example is checked in with `@Disabled`. A later
robot-owned integration test, not this walkthrough, owns any decision to expose a Driver Station
entry after its complete readiness review.

**A later integration test starts in the wrong place in software.**

Confirm the declared Pedro start and exact-start pose application use the same coordinates and
heading. A software pose assignment does not correct physical placement.

**A later integration test's physical motion does not match the drawn route.**

Press STOP. Recheck motor directions, odometry directions and offsets, coordinate convention,
follower constraints, and physical placement before changing route-policy code.

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

**Next:** [`Phoenix documentation hub`](<../README.md>)
