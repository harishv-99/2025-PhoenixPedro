---
tags:
  - Advanced
---

# Inspect one Pedro route's software outcome

**Outcome:** compile one fixed-route Auto, verify its classified software outcome, and keep the
retained route attempt's status distinct from its Task outcome.

**Optional integration lesson. Knowledge before this page:** understand fresh Tasks and exact
outcomes from [the timed Auto](<Run One Timed Auto.md>) and
[Task reference](<../getting-started/learn-sushi/Tasks and Autonomous.md>). No installation, test
run, or hardware is needed to read the boundary below; no physical motion is authorized.

**Learning scope — blocked software-boundary checkpoint:** this page teaches fixed route creation
and the exact retained attempt's software status. It is not yet a reconstruction-grade Pedro
hardware recipe: use the [advanced Pedro integration guide](<../../integrations/pedro/README.md>)
for runtime wiring, and keep motion blocked for the power-limit reason stated below.

## Critical production idea

A **route** describes where the robot should travel. Pedro's **follower** uses position estimates
to produce drive commands along that route. **Localization** supplies those estimates; it is not
proof that the robot is actually at the reported location.

A **pose** contains position and facing direction, or **heading**. A **field frame** fixes the axes
to the field instead of the robot; a **transform** converts coordinates between two named frames.
Angles here use radians: a full turn is `2 * Math.PI`, and zero points along the frame's positive X.
The maintained checkpoint authors one straight route and one Task-level time budget in named units:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/basic/BasicPedroAuto.java -->
```java
private static final double ROUTE_TIMEOUT_SEC = 4.0;

private static final double START_X_INCHES = 24.0;
private static final double START_Y_INCHES = 24.0;
private static final double END_X_INCHES = 36.0;
private static final double END_Y_INCHES = 24.0;
private static final double HEADING_RAD = 0.0;
```

These `Pose` values are in Pedro's field frame: start at `(24 in, 24 in, 0 rad)`, end at
`(36 in, 24 in, 0 rad)`, and keep heading constant. That is a 12-inch change in Pedro `+X`.
Do not silently paste Sushi-frame coordinates into this route. The runtime's configured
`PedroFieldTransform` converts localization facts between Sushi's FTC field convention and Pedro;
the advanced integration guide owns that runtime choice.

The setup helper `registerServiceOrStop(...)` makes sure the acquired Pedro heartbeat has a
cleanup owner, or stops it if registration fails. That heartbeat remains active outside the route
Task because the follower has lifecycle work of its own.

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/basic/BasicPedroAuto.java -->
```java
Pose startPose = new Pose(START_X_INCHES, START_Y_INCHES, HEADING_RAD);

// Register lifecycle ownership before later route construction can fail.
registerServiceOrStop(program, new PedroHeartbeat(runtime, startPose));
```

The fixed route then uses that same authored start pose and the visible end coordinates.
`BezierLine` is Pedro's straight-line path piece; equal heading-interpolation endpoints keep the
heading constant. `PathChain` stores the built route. In `RouteTask<PathChain>`, the type inside
angle brackets tells Java which route representation the Task uses:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/basic/BasicPedroAuto.java -->
```java
PathChain route = runtime.pathBuilder()
        .addPath(new BezierLine(
                startPose,
                new Pose(END_X_INCHES, END_Y_INCHES, HEADING_RAD)
        ))
        .setLinearHeadingInterpolation(HEADING_RAD, HEADING_RAD)
        .build();
RouteTask<PathChain> routeTask = routeTask(runtime.driveAdapter(), route);
program.rootTask(routeTask);
```

The small factory attaches the exact four-second Task timeout to this eagerly built route:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/basic/BasicPedroAuto.java -->
```java
return RouteTasks.follow(
        "basicPedro.oneRoute",
        Objects.requireNonNull(follower, "follower"),
        Objects.requireNonNull(route, "route"),
        ROUTE_TIMEOUT_SEC
);
```

Notice:

- Fixed geometry is built eagerly; live-pose or vision-dependent geometry needs a clearly named
  built-at-start factory.
- [`RouteTask`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/drive/route/RouteTask.html>)
  owns one attempt and preserves its classified result.
- The OpMode registers one stable Pedro service heartbeat outside the route Task.

The presenter must also observe that retained Task. Reading an adapter-wide "latest" result could
drift to a different attempt in a larger robot:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/basic/BasicPedroAuto.java -->
```java
telemetry.addData(
        "route.status",
        routeTask.getRouteStatus()
);
telemetry.addData("route.outcome", routeTask.getOutcome());
```

## Files in this checkpoint

**Main:**

- [`BasicPedroAuto`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/pedro/basic/BasicPedroAuto.html>) — API reference.
- [Complete source: `BasicPedroAuto.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/basic/BasicPedroAuto.java>)

**Test:**

- [Complete source: `BasicPedroRouteSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/pedro/basic/BasicPedroRouteSoftwareScenarioTest.java>)

## Software checkpoint: completion needs endpoint evidence

**Expected observations:** retained endpoint completion maps to route `COMPLETED` and Task
`SUCCESS`. If that evidence does not arrive before the four-second Task limit, the result is
`TASK_TIMEOUT` and `TIMEOUT`, with one cancellation of that same execution. These are software
boundary expectations, not observations of a moving robot.

- **Question:** Does endpoint evidence from the exact retained execution become Task success?
- **Keep real:** `RouteTask` and its route-status mapping.
- **Replace:** Pedro's external follower/execution boundary.
- **Observe:** the exact `RouteStatus` and `TaskOutcome` after one update.
- **Cannot conclude:** drivetrain motion, localization, path accuracy, clearance, or physical stop.

The recording execution is a supplied boundary probe, not a drivetrain simulator. It publishes only
the external completion fact this question needs. Arrangement keeps the real `RouteTask` and
replaces Pedro's external follower/execution boundary; the empty `PathChain` is only an identity
token, so this checkpoint does not test the production geometry or runtime wiring:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/pedro/basic/BasicPedroRouteSoftwareScenarioTest.java -->
```java
// ARRANGE: the follower records which route started; it invents no motion or completion.
PathChain authoredRoute = new PathChain();
RecordingExecution execution = new RecordingExecution();
RecordingFollower follower = new RecordingFollower(execution);
ManualLoopClock time = new ManualLoopClock();
RouteTask<PathChain> routeTask = BasicPedroAuto.routeTask(follower, authoredRoute);
```

Starting is the route request. The immediate observations prove that the exact route was handed to
the follower once and that no completion was invented:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/pedro/basic/BasicPedroRouteSoftwareScenarioTest.java -->
```java
// REQUEST: starting the Task must start this exact, eagerly authored route once.
routeTask.start(time.clock());
assertSame(authoredRoute, follower.followedRoute);
assertEquals(1, follower.followCount);
assertEquals(RouteStatus.ACTIVE, routeTask.getRouteStatus());
assertEquals(TaskOutcome.NOT_DONE, routeTask.getOutcome());
```

The test then injects one completion fact. Only the following Task heartbeat can make the retained
status and outcome observations terminal:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/pedro/basic/BasicPedroRouteSoftwareScenarioTest.java -->
```java
// INJECT EVIDENCE: only the retained execution may say that its endpoint was reached.
execution.integrationStatus = RouteStatus.COMPLETED;

// HEARTBEAT: the real Route Task observes that external fact on the next test cycle.
routeTask.update(time.nextCycle(0.02));

// ASSERT: exact endpoint evidence maps to exact route status and Task success.
assertEquals(RouteStatus.COMPLETED, routeTask.getRouteStatus());
assertEquals(TaskOutcome.SUCCESS, routeTask.getOutcome());
```

Optionally run the maintained scenario after [software setup](<../getting-started/Build and Run.md>):

=== "Windows"

    ```powershell
    .\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.pedro.basic.BasicPedroRouteSoftwareScenarioTest
    ```

=== "macOS"

    ```bash
    ./gradlew --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.pedro.basic.BasicPedroRouteSoftwareScenarioTest
    ```

**Read the causal chain:** the Task starts one authored route; the test supplies endpoint evidence
to that retained execution; the next Task heartbeat classifies it as `COMPLETED` and `SUCCESS`.

The second scenario arranges a fresh execution, follower, clock, and single-use route Task but
supplies no endpoint evidence. Its heartbeat reaches the same factory's `4.0 s` limit:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/pedro/basic/BasicPedroRouteSoftwareScenarioTest.java -->
```java
// ARRANGE: this execution stays ACTIVE unless the test supplies another fact.
RecordingExecution execution = new RecordingExecution();
RecordingFollower follower = new RecordingFollower(execution);
ManualLoopClock time = new ManualLoopClock();
RouteTask<PathChain> routeTask =
        BasicPedroAuto.routeTask(follower, new PathChain());
```

That fresh Task starts once, receives no endpoint evidence, and times out on its own boundary:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/pedro/basic/BasicPedroRouteSoftwareScenarioTest.java -->
```java
// REQUEST: begin one route attempt at this Task's own time boundary.
routeTask.start(time.clock());

// HEARTBEAT: no endpoint evidence arrives before the lesson's four-second Task limit.
routeTask.update(time.nextCycle(4.0));

// ASSERT: timeout remains distinct from success and cleans up this execution exactly once.
assertEquals(RouteStatus.TASK_TIMEOUT, routeTask.getRouteStatus());
assertEquals(TaskOutcome.TIMEOUT, routeTask.getOutcome());
assertEquals(1, execution.cancelCount);
```

**Proves:** endpoint completion from the retained execution maps to exact `COMPLETED` route status
and `SUCCESS` Task outcome for that start; missing completion reaches exact `TASK_TIMEOUT` /
`TIMEOUT` and cancels that same retained execution once.

Separate source inspection above shows that the production presenter reads both displayed facts
from the retained `routeTask`; this focused boundary scenario does not instantiate that presenter
or assert telemetry output.

**Does not prove:** the robot can follow this geometry accurately or safely.

**Reading checkpoint:** explain why an idle follower alone cannot establish endpoint completion,
why the two result vocabularies are distinct, and why both displayed facts must belong to the
same retained route attempt. Those answers complete this software-boundary lesson; the physical
gate remains blocked even if you optionally run the supplied test successfully.

## Isolated hardware gate — currently blocked { #isolated-hardware-gate-currently-blocked }

Keep the example `@Disabled` and `ROBOT_MOTION_REVIEWED` false. Pedro 2.1.2 creates its Follower
with a separate `globalMaxPower` default of `1.0`; when following begins, that current value
overwrites the drivetrain scaling initialized from Mecanum `maxPower = 0.25`. The follow call does
not reset `globalMaxPower`, but the ordinary managed Sushi route API does not yet expose its
persistent setting. The team therefore cannot authorize the “low-power run” this first physical
gate would require. Do not bypass the managed boundary to obtain a raw Follower.

The software checkpoint remains useful: review motor names/directions, Pinpoint installation,
follower tuning, start pose, route clearance, and the STOP plan without enabling motion. Physical
route qualification remains blocked until the supported integration can retain a reviewed power
limit for the route attempt.

**Next gate:** complete the
[Pedro integration lifecycle review](<../../integrations/pedro/README.md#what-validation-does-not-prove>)
and leave this example disabled while that page still marks physical route qualification blocked.
