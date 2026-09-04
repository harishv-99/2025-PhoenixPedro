---
tags:
  - Build
---

# Inspect one Pedro route's software outcome

**Outcome:** compile one fixed-route Auto, verify its classified software outcome, and keep the
retained route attempt's status distinct from its Task outcome.

**Prerequisites:** the project software checks pass; you understand fresh Tasks; no physical motion
is authorized by this lesson.

**Learning scope — blocked software-boundary checkpoint:** this page teaches fixed route creation
and the exact retained attempt's software status. It is not yet a reconstruction-grade Pedro
hardware recipe: use the [advanced Pedro integration guide](<../../integrations/pedro/README.md>)
for runtime wiring, and keep motion blocked for the power-limit reason stated below.

## Critical production idea

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

- **Question:** Does endpoint evidence from the exact retained execution become Task success?
- **Keep real:** `RouteTask` and its route-status mapping.
- **Replace:** Pedro's external follower/execution boundary.
- **Observe:** the exact `RouteStatus` and `TaskOutcome` after one update.
- **Cannot conclude:** drivetrain motion, localization, path accuracy, clearance, or physical stop.

The recording execution is a supplied boundary probe, not a drivetrain simulator. It publishes only
the external completion fact this question needs.

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

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.pedro.basic.BasicPedroRouteSoftwareScenarioTest
```

**Read the causal chain:** the Task starts one authored route; the test supplies endpoint evidence
to that retained execution; the next Task heartbeat classifies it as `COMPLETED` and `SUCCESS`.

**Proves:** endpoint completion from the retained execution maps to exact `COMPLETED` route status
and `SUCCESS` Task outcome for that start.

Separate source inspection above shows that the production presenter reads both displayed facts
from the retained `routeTask`; this focused boundary scenario does not instantiate that presenter
or assert telemetry output.

**Does not prove:** the robot can follow this geometry accurately or safely.

## Isolated hardware gate — currently blocked

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
