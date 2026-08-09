# Pedro autonomous reference

Use this example when you want the smallest checked-in Phoenix Auto that shows the complete
supported Pedro Pathing lifecycle. It follows one fixed practice route, starts a mechanism Task
only after confirmed route completion, chooses a safe mechanism fallback after a truthful timeout,
and stops every owner deterministically. The code compiles against the project's pinned Pedro
Pathing 2.1.2 dependency. It uses the ordinary `FtcRobotOpMode`/`RobotProgram` lifecycle and
deliberately does not introduce an Auto DSL, a robot superclass, or another scheduler.

The four Phoenix-season-independent reference classes and their disabled physical host are under
[`edu.ftcphoenix.robots.examples.pedro`](<../../../robots/examples/pedro/>). Everything there is
**robot code**, including the composition root and host OpMode; the short routine method is not the
whole cost of the example. The host uses this repository's Phoenix hardware configuration only as a
concrete adapter for the generic example. It is not a second production Phoenix-season Auto path:
production Phoenix entries use `PhoenixAutoOpMode`, while a new robot uses the framework's base
`FtcRobotOpMode` grammar shown here.

## Read the five files

Read them in this order:

| File | Source lines | Job | What a new robot normally changes |
|---|---:|---|---|
| [`BasicPedroAutoMechanism.java`](<../../../robots/examples/pedro/BasicPedroAutoMechanism.java>) | 195 | Ordinary `(HardwareMap, Config)` Plant-backed intake output that creates fresh, cancellation-safe Tasks | Replace it with an existing robot capability when one already owns the mechanism; otherwise change its data-only configuration, Plant, and semantic actions here. |
| [`BasicPedroAutoPaths.java`](<../../../robots/examples/pedro/BasicPedroAutoPaths.java>) | 52 | Declared physical start pose and one eagerly built fixed Pedro route | Change the start/end coordinates and path geometry here. Keep all coordinates explicitly in Pedro field inches and radians. |
| [`BasicPedroAutoRoutine.java`](<../../../robots/examples/pedro/BasicPedroAutoRoutine.java>) | 50 | Route, success action, and timeout fallback composed with framework Task factories | Change semantic order, route timeout, capability actions, and the policy for each route result here. |
| [`BasicPedroAutoRobot.java`](<../../../robots/examples/pedro/BasicPedroAutoRobot.java>) | 200 | Declaration-only composition root for the Pedro service, mechanism output, and one root routine | Retain the managed role shape. Add a service or output only when a real owner has that distinct job. |
| [`BasicPedroAutoExample.java`](<../../../robots/examples/pedro/BasicPedroAutoExample.java>) | 114 | Disabled generic `FtcRobotOpMode` host adapted to this repository's physical configuration/runtime wiring and presenters | Replace this host boundary with the new robot's verified Pedro runtime and mechanism configuration. Do not copy Phoenix hardware values into another robot. |

`BasicPedroAutoMechanism` now demonstrates the ordinary construction rule directly: its public
constructor receives `HardwareMap` plus one data-only `Config`, snapshots and validates that
configuration, and privately builds its final Plant. Its completed-Plant constructor is
package-private and exists only for the explicitly hardware-neutral test/portable seam.

The five files total **611 source lines**, including comments, Javadocs, imports, and blank lines.
The four season-independent reference classes total 497 lines; the concrete hardware-adapter host
is another 114.
These counts describe the current checked-in reference and should be updated when its documented
ownership or safety seams change.
That full count matters. A student maintaining or adapting the reference encounters about 12
concrete concepts, not merely the few calls in `BasicPedroAutoRoutine`:

1. verified hardware/profile/runtime wiring,
2. one `FtcRobotOpMode.configure(RobotProgram)` declaration boundary,
3. a robot composition root,
4. Pedro-coordinate poses and path construction,
5. one service owning localization before the recurring Pedro heartbeat,
6. retained route execution, status, and timeout,
7. one single-use root Task graph,
8. explicit outcome branching,
9. a privately owned source-driven Plant and data-only mechanism configuration,
10. fresh, cancellation-safe mechanism capability Tasks,
11. additive telemetry and physical-start readiness,
12. and framework-managed phase order plus idempotent, best-effort fail-stop cleanup.

This is intentionally much smaller than the Phoenix season Auto graph, but it is still a complete
ownership example rather than a claim that all robot code is one line.

## What happens through the FTC lifecycle

### INIT

`BasicPedroAutoExample.configure(...)` creates one real `PedroPathingRuntime` and passes a
mechanism factory to `BasicPedroAutoRobot`. That declaration root immediately registers the Pedro
service before it builds paths, invokes the factory once, immediately registers the resulting
mechanism output, and declares one fresh root routine. Construction does not start the route or
move the mechanism. The final framework-owned `init()` and `init_loop()` callbacks advance only the
shared clock and presenters, keeping the expected physical placement and test warning visible.

Only the host file imports `edu.ftcphoenix.robots.phoenix.PhoenixProfile`. It reuses this
repository's already-owned hardware names and directions rather than publishing plausible-looking
placeholder values. The four independent reference classes do not depend on `PhoenixRobot`, Phoenix
readiness, scoring, targeting, season paths, or season strategy.

### START

The final framework host resets its one clock at the exact FTC START boundary, then performs these
steps once:

1. the Pedro service applies the declared starting pose,
2. that service updates localization and gives the Pedro drive adapter its boundary heartbeat,
3. the private runner starts and first-updates the one fresh root Task,
4. the mechanism output realizes the resulting request once.

This exact-start output pass preserves every positive-duration Task request even if the first FTC
loop arrives after its time boundary. The route still sees a current localization snapshot and
Pedro heartbeat before it starts; later same-cycle adapter calls deduplicate on the shared clock
cycle. Applying the start pose is a software coordinate rebase; it does **not** prove that the
robot was physically placed at that pose.

### Each loop

`RobotProgram` applies one fixed ownership order:

```text
LoopClock -> Pedro service (localization -> drive heartbeat) -> Bindings
          -> root/input Tasks -> mechanism output -> presenters -> telemetry commit
```

The framework advances the clock exactly once. The registered service gives Pedro its one stable
heartbeat even when no route Task is active. The active `RouteTask` may reach the same adapter later
in that cycle, but the adapter deduplicates by `clock.cycle()` instead of updating the vendor
follower twice. Finally, the registered mechanism output resolves and applies the capability's
retained source-driven request, and the host commits one complete telemetry frame.

The OpMode declares only robot-specific construction and presenters. It does not forward lifecycle
steps or call raw Pedro
`Follower.update()`, `followPath(...)`, `breakFollowing()`, pose-reset, or drivetrain methods.

### STOP and failures

The final framework host marks the program terminal first and tries every cleanup owner in this
order:

1. cancel the active Task and clear pending work,
2. clear input bindings,
3. restore the mechanism's idle request and stop its Plant,
4. stop the Pedro drive adapter immediately through the service's reverse-order cleanup phase.

If an earlier cleanup throws a `RuntimeException`, later cleanup is still attempted and additional
failures are suppressed onto the first one; an `Error` propagates immediately. A configuration,
START, active-loop, presenter, or telemetry `RuntimeException` enters the same fail-stop path and
retains that exact original failure. Active cancellation of the collection Task restores the
intake's idle request; queued Tasks are never treated as though they had started. Because owners are
registered immediately, a later configuration failure also stops every already-transferred owner.
The mechanism constructor and declaration helper separately stop an owner that fails before its
ownership transfer completes.

## The route-result policy

The example builds one route Task with `RouteTasks.follow(...)`, passing its four-second Task
timeout directly in that factory call. The routine stores it through the ordinary `Task` interface
because this policy needs only the mapped outcome; code that needs the precise terminal reason can
retain the factory's `RouteTask` result. `RouteTask` converts the integration's detailed terminal
fact into the outcome understood by `Tasks.branchOnOutcome(...)`:

| Route terminal fact | Task outcome | Example policy |
|---|---|---|
| Original route reaches its confirmed endpoint | `SUCCESS` | Run a fresh 0.50-second collection Task, then return the Plant request to idle. |
| Pedro reports follower timeout/stall, or the Route Task reaches its own timeout | `TIMEOUT` | Run the explicit `idleTask()` fallback. No position-dependent mechanism action starts. |
| Interruption, replacement, direct cancellation, integration failure, or unknown terminal state | `CANCELLED` | Abort. Neither the success action nor timeout fallback starts. |

This policy is robot strategy, not Pedro behavior. Another robot may choose a different timeout
action, but it should make that choice visible in its routine and should never infer success merely
because the follower is no longer busy. `getLatestRouteStatus()` is useful for telemetry; decisions
about one run belong to that run's retained `RouteTask` or `RouteExecution`.

The timeout branch handles the timeout, so the composed Task ultimately reports the chosen
fallback Task's outcome. Direct cancellation remains cancellation and does not manufacture a
fallback action. The cancellation-like route branch also deliberately does not change mechanism
requests. This one-shot reference enters the routine with the intake target constructed at idle; a
robot that can enter from another state must restore its own safe capability requests before the
route or include that policy explicitly in its owning routine.

## Adapting the reference to another robot

The normal edit path is:

1. **Sync the supported framework baseline.** Bring over the current Pedro runtime/adapter, route
   execution/status, route Task, Task lifecycle, and their pinned dependencies before copying the
   example shape. An older generated Pedro OpMode is not an equivalent lifecycle owner.
2. **Replace the host wiring file.** Extend `FtcRobotOpMode` and override only
   `configure(RobotProgram)`. Construct one runtime from that robot's verified motor names,
   directions, Pinpoint setup, field transform, follower tuning, and path constraints. Supply the
   real mechanism's `HardwareMap` plus data-only configuration. The host is the intentionally
   robot-specific composition boundary.
3. **Edit `BasicPedroAutoPaths`.** Declare the required physical start and fixed path geometry. Build
   fixed routes eagerly through `runtime.pathBuilder()`. Use the separately documented start-time
   route factory only when geometry genuinely depends on a live fact.
4. **Edit `BasicPedroAutoRoutine`.** Name the route, select its timeout, and state exactly what
   success, timeout, and cancellation-like endings mean. Return fresh Task graphs each time a
   routine can run.
5. **Reuse or replace the capability pattern.** A robot with an existing shooter/intake capability
   should call those Task factories rather than copy `BasicPedroAutoMechanism`. Otherwise, give the
   mechanism `HardwareMap` plus its data-only configuration, let it privately own its final Plant,
   implement `RobotProgram.Output`, and make active Task cancellation safe. The checked-in routine
   names the concrete example capability, so adapt its signature rather than inventing a generic
   plug-in interface.
6. **Adapt the declaration root only as needed.** Register the Pedro service first, each mechanism
   output immediately after construction, and one composed root Task. Let `RobotProgram` retain the
   one clock, runner, fixed phase order, telemetry commit, and cleanup.
7. **Keep the OpMode thin.** Construct and declare roles in `configure(...)`, then add read-only
   presenters for placement/readiness facts. Do not override lifecycle callbacks or move routine
   policy into the OpMode.

The Pedro runtime checks its four mecanum motor names using FTC's trimmed, case-sensitive lookup
identity. Blank or trim-equivalent duplicate names fail before fresh drivetrain hardware resolution
or configuration. That catches a configuration-string mistake, but it does not prove that four
differently named Robot Configuration entries are wired to four different physical motors.

## Portability boundary

The four reference classes have no dependency on another season robot or its hardware constants.
The mechanism intentionally imports FTC `HardwareMap` because ordinary FTC mechanisms own their
own Plant construction; only its package-private completed-Plant test seam is hardware-neutral. An
adopting robot should keep its existing capability Tasks, sync the supported managed-runtime and
PEDRO/ROUTE/TASK framework baseline, and adapt the configuration, path, routine, declaration root,
and host to its own owners. Because `BasicPedroAutoRoutine` and `BasicPedroAutoRobot` name the
concrete example mechanism type, they are a readable pattern rather than a drop-in generic base.

The adopting robot must provide and verify its own motor names and directions, localization
calibration, field convention, Pedro constraints, and follower tuning. Replace
`BasicPedroAutoExample` with a host that uses exactly one runtime factory and the robot's
existing mechanisms; do not retain the Phoenix profile import. Android Studio compilation in the
adopting project and deliberate on-robot validation remain required steps.

## Hardware walkthrough

`BasicPedroAutoExample` is `@Disabled` intentionally. Its path is a real 12-inch practice
line in Pedro coordinates, from `(24, 24, 0)` to `(36, 24, 0)` in inches/radians; it is not a match
route and it is not safe merely because it compiles.

Before deliberately enabling it on the configuration-owning robot:

- verify all four drive motor names and directions with the wheels safely raised,
- verify Pinpoint offsets, pod directions, resolution, yaw scalar, and pose convention,
- verify follower/path constraints are appropriate for that hardware,
- place the robot at the displayed physical Pedro start and leave clear space beyond the endpoint,
- verify the intake direction and safe idle target,
- keep an operator ready to press STOP immediately,
- then run at deliberately conservative speed/acceleration limits before raising them.

Compilation and fake tests cannot verify wiring, calibration, traction, available stopping distance,
or physical placement. Treat those as explicit readiness checks owned by the robot team.

## Deliberately outside this beginner example

This item contains one fixed route and one timeout fallback. It does not teach alliance transforms,
vision-selected geometry, live-pose return paths, progress callbacks, parallel deadline companions,
match-time park takeover, or a complete competition scoring strategy. Add those only when a real
routine needs them, using the corresponding route/Task helpers and robot-owned policy; do not turn
this reference into a second season framework.

## Related reading

- [`../../integrations/pedro/README.md`](<../../integrations/pedro/README.md>)
- [`../design/Tasks & Macros Quickstart.md`](<../design/Tasks & Macros Quickstart.md>)
- [`../design/Robot Capabilities & Mode Clients.md`](<../design/Robot Capabilities & Mode Clients.md>)
- [`../core-concepts/Loop Structure.md`](<../core-concepts/Loop Structure.md>)
- [`../../Framework Principles.md`](<../../Framework Principles.md>)
