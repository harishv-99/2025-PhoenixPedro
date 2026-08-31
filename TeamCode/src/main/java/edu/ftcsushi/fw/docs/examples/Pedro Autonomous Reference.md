# Pedro autonomous reference

**Learning mode:** Architecture reference

**Audience:** students and mentors adapting Sushi's managed runtime to a robot that uses Pedro
Pathing.

This is the advanced, compiling reference for the complete supported Pedro lifecycle. It shows one
fixed practice route, one Plant-backed mechanism capability, explicit route-outcome policy, one
stable follower heartbeat, and deterministic cleanup. It is not the first-route tutorial; start
with [`Your first Pedro Auto`](<../getting-started/First Pedro Auto.md>) for the guided source tour.

Start with the compiling [`BasicPedroAutoExample.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/opmode/BasicPedroAutoExample.java>)
entry. Its six-file example uses the ordinary `FtcRobotOpMode`/`RobotProgram` grammar, one local
data-only profile, and the project's pinned Pedro Pathing dependency. It is independent of every
production application package and keeps all example configuration beside the example.

## The six reference roles

| File | Role | Adapt for another robot |
|---|---|---|
| [`BasicPedroProfile.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/robot/BasicPedroProfile.java>) | Fresh local Pedro/runtime and intake configuration plus the false-by-default motion permission. | Replace and physically review every active fact, then permit only the complete supervised run. |
| [`BasicPedroAutoExample.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/opmode/BasicPedroAutoExample.java>) | Thin FTC host: selects the local profile and constructs the composition root. | Keep the host declarative; choose the adopting robot's profile here. |
| [`BasicPedroAutoRobot.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/robot/BasicPedroAutoRobot.java>) | Composition root: gates configuration, constructs/registers the runtime and mechanism, declares one fresh root Task, and wires additive presenters in safety-significant order. | Keep the managed role shape; add an owner only when it has a distinct lifecycle job. |
| [`BasicPedroAutoPaths.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/autonomous/BasicPedroAutoPaths.java>) | Geometry owner: declares the physical start pose and eagerly builds one fixed Pedro route. | Replace start/end poses and route geometry in Pedro field inches and radians. |
| [`BasicPedroAutoRoutine.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/autonomous/BasicPedroAutoRoutine.java>) | Strategy owner: maps route success, timeout, and cancellation-like outcomes to explicit Task behavior. | Compose the robot's capability Tasks and state every non-success policy. |
| [`BasicPedroAutoMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/capability/intake/BasicPedroAutoMechanism.java>) | Mechanism output: snapshots data-only configuration, privately owns one Plant, and creates fresh cancellation-safe Tasks. | Prefer the robot's existing capability; otherwise build the real mechanism with the same ownership boundary. |

These roles are robot code. The short routine expression is strategy, while construction,
lifecycle, capability realization, and physical configuration stay with their actual owners.

## Managed lifecycle and safety contract

### INIT

`BasicPedroAutoExample.configure(...)` selects one fresh local profile and invokes the sole
ordinary composition-root construction path:

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

- The FTC host chooses one fresh profile and delegates the complete graph to the composition root.
- The checked-in motion permission is false; construction validity is not physical authorization.
- `RobotProgram` owns later lifecycle callbacks, cleanup, and the shared heartbeat.

**Key APIs**

- `FtcRobotOpMode.configure(...)`: the ordinary thin FTC host seam.
- `BasicPedroProfile.current()`: returns fresh data-only example configuration.
- `BasicPedroAutoRobot`: declares owners and managed order; it is not the Auto strategy script.
- `RobotProgram`: owns lifecycle, Tasks, outputs, presenters, and cleanup.

`BasicPedroProfile.current()` returns a fresh outer value, a fresh
`PedroPathingRuntime.Config pedro` graph, a fresh `BasicPedroAutoMechanism.Config intake`, and
`allowRobotMotion = false`. The checked-in data is a valid software baseline, not reviewed physical
configuration. It includes explicit motor names/directions, an initial Mecanum `maxPower = 0.25`,
brake mode enabled, and a `0.20` intake command. Pedro 2.1.2 restores its Follower's separate
`globalMaxPower` to `1.0` when `followPath(...)` begins, so the initial `0.25` is not an autonomous
route cap and is not a first-motion safety claim. The `useBrakeModeInTeleOp = true` baseline also
does not define Auto route braking. Only the false permission blocks construction and route motion
in the checked-in program; `@Disabled` separately hides the Driver Station entry.

Before hardware effects, the root requires that explicit permission, requires both active Configs,
and rejects an intake name that resolves to one of the four drive motor names. It then calls the
sole `PedroPathingRuntime.create(hardwareMap, profile.pedro)` hardware boundary. The runtime
deep-snapshots and validates the complete Config before lookup, the non-blocking Pinpoint reset
request, motor output, Follower construction, or Pedro-static mutation. The intake mechanism later
snapshots and validates only `profile.intake`. Later caller edits cannot retune either running
owner.

The root registers the Pedro service before building paths or constructing the mechanism, then
registers the mechanism immediately after construction and declares one root Task. Construction
does not start a route or actuate the mechanism. If path or intake construction fails after runtime
registration, managed failure cleanup best-effort stops the already-owned Pedro drive; a mechanism
that completed its Plant before failing stops that Plant before ownership transfer.

The framework advances only the clock and presenters during INIT. Placement and test warnings stay
visible without creating a second hardware graph or selector retry path.

### START and active loops

At START, `RobotProgram` resets its one clock and runs the registered phases in this order:

```text
apply Pedro starting pose
-> localization
-> Pedro drive heartbeat
-> root Task start + first update
-> mechanism output
```

Each active loop uses:

```text
LoopClock -> Pedro service -> Bindings -> root Task -> mechanism output
          -> presenters -> one telemetry commit
```

The service updates localization before the drive adapter and keeps that adapter's heartbeat alive
even when no route Task is active. A Route Task may reach the same adapter in the same cycle; the
adapter deduplicates by `clock.cycle()`. The mechanism remains the only owner that advances its
Plant.

Applying the declared start pose rebases software coordinates. It does not prove where the chassis
was physically placed.

### STOP and failures

`RobotProgram` marks the graph terminal, cancels active Task work, clears bindings, stops outputs,
and stops services in reverse declaration order. The mechanism terminally stops its Plant without
rewriting the retained request; that power Plant submits zero immediately and never updates again.
The Pedro service also applies physical zero immediately. Later cleanup is still attempted after a
`RuntimeException`, with additional cleanup failures suppressed onto the first failure.

The same fail-stop path handles configuration, START, loop, presenter, or telemetry failures.
Owners registered before a later construction failure are already covered by program cleanup, and
constructors clean resources that fail before ownership transfer completes.

That cleanup is best effort rather than transactional hardware rollback. Once a valid Config has
entered SDK/vendor construction, a Mecanum constructor can throw without returning a stoppable
handle, Pinpoint has no close/rollback operation, and a failed Follower may already have touched
Pedro's static controller switches. The runtime breaks a successfully returned drivetrain if a
later construction step fails and preserves any stop failure as suppressed evidence. Correct the
cause and restart the OpMode rather than retrying construction in place.

## Route-result policy

### Critical code

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/autonomous/BasicPedroAutoRoutine.java -->
```java
return Tasks.branchOnOutcome(
        followPracticeRoute,
        requiredMechanism.collectTask(COLLECT_DURATION_SEC),
        requiredMechanism.idleTask()
);
```

**What to notice**

- `branchOnOutcome(...)` uses the exact route Task outcome; the retained route status supplies finer diagnostics.
- Endpoint success runs collection; timeout runs an explicit idle fallback; cancellation-like endings abort.
- A successful fallback may make the root successful while the route fact remains `TASK_TIMEOUT`.
- Vendor idle/not-busy is never promoted to endpoint success.

**Key APIs**

- `RouteTasks.follow(...)`: adapts one route start into a single-use timed `RouteTask`.
- `RouteTask.getRouteStatus()`: retains the integration/Task boundary classification for that start.
- `RouteStatus`: distinguishes completion, follower stall, Task timeout, cancellation, failure, and unknown endings.
- `TaskOutcome`: describes the selected aggregate strategy branch, not the route geometry result.

`BasicPedroAutoRoutine` follows one route with a four-second Task timeout and branches on the
retained result from that exact start:

| Retained route status | Route Task outcome | Reference policy and root result |
|---|---|---|
| `COMPLETED` | `SUCCESS` | Run a fresh collection Task; the root is `SUCCESS` if that selected Task succeeds. |
| `FOLLOWER_TIMEOUT_OR_STALL` | `TIMEOUT` | Run the explicit idle fallback; the root is `SUCCESS` if that fallback succeeds. |
| `TASK_TIMEOUT` | `TIMEOUT` | Run the same idle fallback; retain `TASK_TIMEOUT` as the route fact even though successful fallback makes the root `SUCCESS`. |
| interruption, replacement, cancellation, failure, or unknown ending | `CANCELLED` | Abort without starting either branch; the root is `CANCELLED`. |

The robot routine owns this policy. Vendor idle or not-busy state alone is never success, and
direct cancellation never manufactures a fallback action. The route status describes why that
route attempt ended; the outer/root outcome describes whether the selected strategy branch later
completed. Telemetry intentionally presents both instead of letting successful cleanup erase a
route timeout.

## Adaptation checklist

1. Extend `FtcRobotOpMode` and override only `configure(RobotProgram)`.
2. Start from `BasicPedroProfile.current()`. Replace and physically review its Pinpoint, Follower,
   Mecanum, path-constraint, field-transform, and intake facts. Keep `allowRobotMotion = false`
   until the complete active graph is ready for one supervised test.
3. Construct the reference only through
   `new BasicPedroAutoRobot(program, hardwareMap, profile)`. Do not add a profile mapper or second
   runtime factory; the runtime and intake owners snapshot and validate their own active Configs.
4. Replace `BasicPedroAutoPaths` geometry and display the exact required physical start pose.
5. Keep fixed routes eager. Use `RouteTasks.followBuiltAtStart(...)` only when geometry genuinely
   depends on a live fact at that route's start.
6. Replace the example mechanism with existing mode-neutral capability Tasks when the robot has
   them. Otherwise, pass `HardwareMap` plus data-only configuration to a mechanism that privately
   owns its Plant and implements `RobotProgram.Output`.
7. Build a fresh root Task graph and state the strategy for success, timeout, and every
   cancellation-like route result.
8. Register the Pedro service before dependent work, register outputs immediately after
   construction, and keep presenters read-only and additive.
9. Keep raw `Follower` calls, lifecycle callbacks, private clocks/runners, and manual telemetry
   commits out of the OpMode. Build paths with `runtime.pathBuilder()` and use
   `runtime.currentPedroPose()` only when start-time geometry needs a defensive snapshot of the
   already-cached Pedro pose.

The public `PedroPathingDriveAdapter(completedFollower)` constructor is reserved for an advanced
custom/portable host that has already constructed the completed vendor graph and will route its
lifecycle through the adapter. Pedro's generated native tuning tools use a separate package-local
factory and exclusive raw-Follower/Pinpoint lifecycle. Neither is a second ordinary runtime
construction answer.

For a route whose geometry genuinely depends on a delayed detector frame and live start pose, see
the optional [Timestamped adaptive collection](<Timestamped Adaptive Collection.md>) case study. It
keeps the basic six-role reference unchanged while demonstrating typed unavailable fallback,
semantic native callbacks, inventory-gated early exit, and separate exact statuses for collection
and return.

## Files you will create

Create the six files in the role table. The complete host and strategy files below show both public
ends of the graph: `configure(program)` declares the robot, and `build(...)` creates fresh route
behavior from the capability. The profile, root, path owner, and mechanism supply the exact owners
described in the table; do not collapse those responsibilities into the host.

## Complete working slice

<details>
<summary>Complete working slice: FTC Auto host</summary>

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

<details>
<summary>Complete working slice: route outcome policy</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/autonomous/BasicPedroAutoRoutine.java -->
```java
package edu.ftcsushi.robots.examples.pedro.autonomous;

import com.pedropathing.paths.PathChain;

import java.util.Objects;

import edu.ftcsushi.fw.drive.route.RouteFollower;
import edu.ftcsushi.fw.drive.route.RouteTasks;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.robots.examples.pedro.capability.intake.BasicPedroAutoMechanism;

/** Composes the basic route, success action, and timeout fallback with ordinary Task factories. */
public final class BasicPedroAutoRoutine {

    private static final double ROUTE_TIMEOUT_SEC = 4.0;
    private static final double COLLECT_DURATION_SEC = 0.50;

    private BasicPedroAutoRoutine() {
        // Robot-owned Task factory only.
    }

    /**
     * Builds one fresh routine.
     *
     * <p>Confirmed route completion starts collection. A follower or Task timeout restores the
     * mechanism's idle request. Cancellation-like route endings abort without either branch and
     * deliberately leave any unrelated prior request unchanged. The basic host enters at idle;
     * another caller must establish the entry state its policy requires.</p>
     */
    public static Task build(RouteFollower<PathChain> routes,
                             PathChain practiceRoute,
                             BasicPedroAutoMechanism mechanism) {
        Task followPracticeRoute = RouteTasks.follow(
                "BasicPracticeRoute",
                Objects.requireNonNull(routes, "routes"),
                Objects.requireNonNull(practiceRoute, "practiceRoute"),
                ROUTE_TIMEOUT_SEC
        );

        BasicPedroAutoMechanism requiredMechanism = Objects.requireNonNull(
                mechanism,
                "mechanism"
        );
        return Tasks.branchOnOutcome(
                followPracticeRoute,
                requiredMechanism.collectTask(COLLECT_DURATION_SEC),
                requiredMechanism.idleTask()
        );
    }
}
```

</details>

## Verify the slice

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac `
  :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.pedro.autonomous.* `
  --tests edu.ftcsushi.robots.examples.pedro.robot.BasicPedroAutoRobotTest
```

Expected checkpoint: the example compiles and route-policy/root tests pass. It remains `@Disabled`;
software verification does not validate localization, path clearance, or drivetrain motion.

## Expected result

The checked-in host remains `@Disabled` and compiles as a reference. After an adopting team
deliberately verifies the configuration and enables its adapted host, the reference behavior is:

1. display and apply the Pedro start pose;
2. follow the 12-inch practice line from `(24, 24, 0)` to `(36, 24, 0)`;
3. collect for 0.50 seconds only after confirmed endpoint success;
4. restore idle after a route timeout; and
5. stop the mechanism and Pedro drivetrain on STOP or failure.

Telemetry reports the expected physical start, latest route status, root completion, and root
outcome without advancing those owners. For example, a Task deadline followed by successful idle
fallback ends with `example.routeStatus=TASK_TIMEOUT` and `example.rootOutcome=SUCCESS`; those are
different truthful boundaries, not a contradiction.

## Physical validation before enabling

- Verify all four drive motor names and directions with wheels safely raised.
- Verify Pinpoint offsets, pod directions, resolution, yaw scalar, and field convention.
- Review Pedro's Follower `globalMaxPower`, speed, acceleration, and completion constraints with
  clear space beyond the endpoint. The profile's initial Mecanum `maxPower = 0.25` is overwritten
  by the default route-time `globalMaxPower = 1.0` and is not a route limit.
- Verify the mechanism direction, collection magnitude, and safe idle request.
- Place the robot at the displayed physical start and keep an operator ready to press STOP.

Compilation and fake tests cannot prove wiring, calibration, traction, stopping distance, or
physical placement. Runtime Config validation likewise cannot prove motor direction, Pinpoint
placement/readiness, follower stability, field alignment, route clearance, or physical STOP.

## Related reading

- [`Your first Pedro Auto`](<../getting-started/First Pedro Auto.md>)
- [`Timestamped adaptive collection`](<Timestamped Adaptive Collection.md>)
- [`Pedro Pathing integration contract`](<../../integrations/pedro/README.md>)
- [`Tasks and Macros`](<../design/Tasks & Macros Quickstart.md>)
- [`Robot Capabilities and Mode Clients`](<../design/Robot Capabilities & Mode Clients.md>)
- [`Loop Structure`](<../core-concepts/Loop Structure.md>)
- [`Framework Principles`](<../../Framework Principles.md>)
