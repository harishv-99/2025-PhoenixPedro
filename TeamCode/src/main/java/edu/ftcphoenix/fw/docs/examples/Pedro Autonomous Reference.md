# Pedro autonomous reference

**Audience:** students and mentors adapting Phoenix's managed runtime to a robot that uses Pedro
Pathing.

This is the advanced, compiling reference for the complete supported Pedro lifecycle. It shows one
fixed practice route, one Plant-backed mechanism capability, explicit route-outcome policy, one
stable follower heartbeat, and deterministic cleanup. It is not the first-route tutorial; start at
the canonical [`Phoenix docs hub`](<../README.md>) for the guided learning path.

Start with the compiling [`BasicPedroAutoExample.java`](<../../../robots/examples/pedro/BasicPedroAutoExample.java>)
entry. Its five-file example uses the ordinary `FtcRobotOpMode`/`RobotProgram` grammar and the
project's pinned Pedro Pathing dependency. The disabled host borrows this repository's Phoenix
hardware configuration only to make the example concrete. Production Phoenix Auto uses
`PhoenixAutoOpMode`; this package is the independent reference for another robot.

## The five owner roles

| File | Owner role | Adapt for another robot |
|---|---|---|
| [`BasicPedroAutoExample.java`](<../../../robots/examples/pedro/BasicPedroAutoExample.java>) | Thin FTC host: constructs verified robot-specific configuration and adds read-only presenters. | Replace Phoenix profile wiring with that robot's runtime and mechanism configuration. |
| [`BasicPedroAutoRobot.java`](<../../../robots/examples/pedro/BasicPedroAutoRobot.java>) | Declaration root: registers the Pedro service, mechanism output, and one fresh root Task in safety-significant order. | Keep the managed role shape; add an owner only when it has a distinct lifecycle job. |
| [`BasicPedroAutoPaths.java`](<../../../robots/examples/pedro/BasicPedroAutoPaths.java>) | Geometry owner: declares the physical start pose and eagerly builds one fixed Pedro route. | Replace start/end poses and route geometry in Pedro field inches and radians. |
| [`BasicPedroAutoRoutine.java`](<../../../robots/examples/pedro/BasicPedroAutoRoutine.java>) | Strategy owner: maps route success, timeout, and cancellation-like outcomes to explicit Task behavior. | Compose the robot's capability Tasks and state every non-success policy. |
| [`BasicPedroAutoMechanism.java`](<../../../robots/examples/pedro/BasicPedroAutoMechanism.java>) | Mechanism output: snapshots data-only configuration, privately owns one Plant, and creates fresh cancellation-safe Tasks. | Prefer the robot's existing capability; otherwise build the real mechanism with the same ownership boundary. |

These roles are robot code. The short routine expression is strategy, while construction,
lifecycle, capability realization, and physical configuration stay with their actual owners.

## Managed lifecycle and safety contract

### INIT

`BasicPedroAutoExample.configure(...)` creates one `PedroPathingRuntime` and one declaration root.
The root registers the Pedro service before building paths or constructing the mechanism, then
registers the mechanism immediately after construction and declares one root Task. Construction
does not start a route or actuate the mechanism.

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
and stops services in reverse declaration order. The mechanism restores its idle request and stops
its Plant; the Pedro service applies physical zero immediately. Later cleanup is still attempted
after a `RuntimeException`, with additional cleanup failures suppressed onto the first failure.

The same fail-stop path handles configuration, START, loop, presenter, or telemetry failures.
Owners registered before a later construction failure are already covered by program cleanup, and
constructors clean resources that fail before ownership transfer completes.

## Route-result policy

`BasicPedroAutoRoutine` follows one route with a four-second Task timeout and branches on the
retained result from that exact start:

| Terminal result | Task outcome | Reference policy |
|---|---|---|
| Confirmed endpoint completion | `SUCCESS` | Run a fresh 0.50-second collection Task, then return the intake request to idle. |
| Follower timeout/stall or Task timeout | `TIMEOUT` | Run the explicit idle fallback; do not start the position-dependent collection action. |
| Interruption, replacement, direct cancellation, integration failure, or unknown ending | `CANCELLED` | Abort without starting either branch. |

The robot routine owns this policy. Vendor idle or not-busy state alone is never success, and
direct cancellation never manufactures a fallback action.

## Adaptation checklist

1. Extend `FtcRobotOpMode` and override only `configure(RobotProgram)`.
2. Construct one Pedro runtime from verified motor names, directions, localization configuration,
   field transforms, follower tuning, and path constraints.
3. Replace `BasicPedroAutoPaths` geometry and display the exact required physical start pose.
4. Keep fixed routes eager. Use `RouteTasks.followBuiltAtStart(...)` only when geometry genuinely
   depends on a live fact at that route's start.
5. Replace the example mechanism with existing mode-neutral capability Tasks when the robot has
   them. Otherwise, pass `HardwareMap` plus data-only configuration to a mechanism that privately
   owns its Plant and implements `RobotProgram.Output`.
6. Build a fresh root Task graph and state the strategy for success, timeout, and every
   cancellation-like route result.
7. Register the Pedro service before dependent work, register outputs immediately after
   construction, and keep presenters read-only and additive.
8. Keep raw `Follower` calls, lifecycle callbacks, private clocks/runners, and manual telemetry
   commits out of the OpMode.

## Expected result

The checked-in host remains `@Disabled` and compiles as a reference. After an adopting team
deliberately verifies the configuration and enables its adapted host, the reference behavior is:

1. display and apply the Pedro start pose;
2. follow the 12-inch practice line from `(24, 24, 0)` to `(36, 24, 0)`;
3. collect for 0.50 seconds only after confirmed endpoint success;
4. restore idle after a route timeout; and
5. stop the mechanism and Pedro drivetrain on STOP or failure.

Telemetry reports the expected physical start, latest route status, root completion, and root
outcome without advancing those owners.

## Physical validation before enabling

- Verify all four drive motor names and directions with wheels safely raised.
- Verify Pinpoint offsets, pod directions, resolution, yaw scalar, and field convention.
- Use conservative follower speed and acceleration limits with clear space beyond the endpoint.
- Verify the mechanism direction, collection magnitude, and safe idle request.
- Place the robot at the displayed physical start and keep an operator ready to press STOP.

Compilation and fake tests cannot prove wiring, calibration, traction, stopping distance, or
physical placement.

## Related reading

- [`Pedro Pathing integration contract`](<../../integrations/pedro/README.md>)
- [`Tasks and Macros`](<../design/Tasks & Macros Quickstart.md>)
- [`Robot Capabilities and Mode Clients`](<../design/Robot Capabilities & Mode Clients.md>)
- [`Loop Structure`](<../core-concepts/Loop Structure.md>)
- [`Framework Principles`](<../../Framework Principles.md>)
