# Pedro autonomous reference

Use this example when you want the smallest checked-in Phoenix Auto that shows the complete
supported Pedro Pathing lifecycle. It follows one fixed practice route, starts a mechanism Task
only after confirmed route completion, chooses a safe mechanism fallback after a truthful timeout,
and stops every owner deterministically. The code compiles against the project's pinned Pedro
Pathing 2.1.2 dependency. It deliberately does not introduce an Auto DSL, a base OpMode, or another
scheduler.

The four Phoenix-season-independent reference classes are under
[`edu.ftcphoenix.robots.examples.pedro`](<../../../robots/examples/pedro/>). The disabled physical
host is under [`edu.ftcphoenix.robots.phoenix.opmode`](<../../../robots/phoenix/opmode/>). Everything
in both locations is **robot code**, including the composition root and host OpMode; the short
routine method is not the whole cost of the example.

## Read the five files

Read them in this order:

| File | Source lines | Job | What a new robot normally changes |
|---|---:|---|---|
| [`BasicPedroAutoMechanism.java`](<../../../robots/examples/pedro/BasicPedroAutoMechanism.java>) | 98 | Plant-backed intake capability that creates fresh, cancellation-safe Tasks | Replace it with an existing robot capability when one already owns the mechanism; otherwise change the Plant and action names/targets here. |
| [`BasicPedroAutoPaths.java`](<../../../robots/examples/pedro/BasicPedroAutoPaths.java>) | 52 | Declared physical start pose and one eagerly built fixed Pedro route | Change the start/end coordinates and path geometry here. Keep all coordinates explicitly in Pedro field inches and radians. |
| [`BasicPedroAutoRoutine.java`](<../../../robots/examples/pedro/BasicPedroAutoRoutine.java>) | 50 | Route, success action, and timeout fallback composed with framework Task factories | Change semantic order, route timeout, capability actions, and the policy for each route result here. |
| [`BasicPedroAutoRobot.java`](<../../../robots/examples/pedro/BasicPedroAutoRobot.java>) | 209 | Composition root for the shared clock, localization, recurring Pedro heartbeat, Task runner, Plant realization, and shutdown | Usually retain this shape. Add robot-owned sensor/service/capability updates only when the robot actually has them, preserving one explicit loop order. |
| [`PhoenixBasicPedroAutoExample.java`](<../../../robots/phoenix/opmode/PhoenixBasicPedroAutoExample.java>) | 217 | Disabled FTC lifecycle host and this repository's physical hardware/runtime wiring | Replace this entire host boundary with the new robot's verified Pedro runtime and mechanism construction. Do not copy Phoenix hardware values into another robot. |

The five files total **626 source lines**, including comments, Javadocs, imports, and blank lines.
The four independent reference classes total 409 lines; the Phoenix-specific host is another 217.
That full count matters. A student maintaining or adapting the reference encounters about 15
concrete concepts, not merely the few calls in `BasicPedroAutoRoutine`:

1. verified hardware/profile/runtime wiring,
2. the FTC INIT/START/loop/STOP lifecycle,
3. a robot composition root,
4. Pedro-coordinate poses and path construction,
5. one shared `LoopClock`,
6. predictor/localization update ownership,
7. one recurring drive-adapter/follower heartbeat,
8. retained route execution, status, and timeout,
9. a `TaskRunner` and single-use Task graph,
10. explicit outcome branching,
11. a writable, source-driven Plant target,
12. fresh mechanism capability Tasks,
13. ordered mechanism realization,
14. operator telemetry and physical-start readiness,
15. and idempotent, best-effort cancellation/fail-stop cleanup.

This is intentionally much smaller than the Phoenix season Auto graph, but it is still a complete
ownership example rather than a claim that all robot code is one line.

## What happens through the FTC lifecycle

### INIT

`PhoenixBasicPedroAutoExample.init()` creates one real `PedroPathingRuntime`, builds the fixed path,
constructs one writable intake Plant and capability, and wires `BasicPedroAutoRobot`. Construction
does not start the route or move the mechanism. `init_loop()` keeps the expected physical placement
and test warning visible.

Only the host file imports `edu.ftcphoenix.robots.phoenix.PhoenixProfile`. It reuses this
repository's already-owned hardware names and directions rather than publishing plausible-looking
placeholder values. The four independent reference classes do not depend on `PhoenixRobot`, Phoenix
readiness, scoring, targeting, season paths, or season strategy.

### START

The composition root performs these steps once:

1. apply the declared Pedro starting pose through `PedroPathingRuntime.setStartingPose(...)`,
2. reset the shared `LoopClock` at the exact FTC START runtime,
3. enqueue the one fresh root Task.

The route is not advanced during `start()`. Its first update happens in the regular loop after the
current localization snapshot and Pedro heartbeat exist. Applying the start pose is a software
coordinate rebase; it does **not** prove that the robot was physically placed at that pose.

### Each loop

`BasicPedroAutoRobot.update(...)` has one visible ownership order:

```text
LoopClock -> localization -> Pedro drive heartbeat -> TaskRunner -> mechanism Plant
```

The composition root advances the clock exactly once. The drive adapter then gives Pedro its one
stable heartbeat even when no route Task is active. The active `RouteTask` may reach the same
adapter later in that cycle, but the adapter deduplicates by `clock.cycle()` instead of updating the
vendor follower twice. Finally, the mechanism Plant resolves and applies the capability's retained
source-driven request.

The OpMode owns only lifecycle forwarding and telemetry. It does not call raw Pedro
`Follower.update()`, `followPath(...)`, `breakFollowing()`, pose-reset, or drivetrain methods.

### STOP and failures

`BasicPedroAutoRobot.stop()` is idempotent and tries every cleanup owner in this order:

1. cancel the active Task and clear pending work,
2. restore the mechanism's idle request and stop its Plant,
3. stop the Pedro drive adapter immediately.

If an earlier cleanup throws, later cleanup is still attempted and additional failures are
suppressed onto the first one. A START or loop failure also enters this fail-stop path. Active
cancellation of the collection Task restores the intake's idle request; queued Tasks are never
treated as though they had started. Partial FTC INIT construction also stops any already-created
mechanism and drive owners before rethrowing the construction error.

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
2. **Replace the host wiring file.** Construct one runtime from that robot's verified motor names,
   directions, Pinpoint setup, field transform, follower tuning, and path constraints. Construct or
   obtain its real capability owners. The host is the one intentionally robot-specific dependency
   boundary.
3. **Edit `BasicPedroAutoPaths`.** Declare the required physical start and fixed path geometry. Build
   fixed routes eagerly through `runtime.pathBuilder()`. Use the separately documented start-time
   route factory only when geometry genuinely depends on a live fact.
4. **Edit `BasicPedroAutoRoutine`.** Name the route, select its timeout, and state exactly what
   success, timeout, and cancellation-like endings mean. Return fresh Task graphs each time a
   routine can run.
5. **Reuse or replace the capability pattern.** A robot with existing shooter/intake capabilities
   should call those factories rather than copy `BasicPedroAutoMechanism`. The checked-in routine
   and composition root intentionally name that concrete example capability, so adapt their
   signatures when using another robot's capability; they are a readable pattern, not a generic
   plug-in interface. Keep the Plant as the one final actuator owner and make active cancellation
   safe.
6. **Adapt the composition-root shape only as needed.** Keep one clock and one recurring Pedro
   heartbeat, then place additional sensor, service, Task, drive, Plant, and telemetry phases
   explicitly.
7. **Keep the OpMode thin.** Construct during INIT, forward START/loop/STOP, and display required
   placement/readiness facts. Do not move the routine into the OpMode.

## Portability boundary

The four independent reference classes have no dependency on another robot project or its hardware
constants. An adopting robot should keep its existing capability Tasks, sync the supported
PEDRO/ROUTE/TASK framework baseline, and adapt the path, routine, composition-root, and host pattern
to its own owners. Because `BasicPedroAutoRoutine` and `BasicPedroAutoRobot` name the concrete
example mechanism type, they are a readable pattern rather than a drop-in generic base.

The adopting robot must provide and verify its own motor names and directions, localization
calibration, field convention, Pedro constraints, and follower tuning. Replace
`PhoenixBasicPedroAutoExample` with a host that uses exactly one runtime factory and the robot's
existing mechanisms; do not retain the Phoenix profile import. Android Studio compilation in the
adopting project and deliberate on-robot validation remain required steps.

## Hardware walkthrough

`PhoenixBasicPedroAutoExample` is `@Disabled` intentionally. Its path is a real 12-inch practice
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
