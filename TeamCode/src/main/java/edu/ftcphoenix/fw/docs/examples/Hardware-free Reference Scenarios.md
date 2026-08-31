# Hardware-free Reference scenarios

**Audience:** Students whose mechanism needs feedback beyond the Starter intake

**Purpose:** Show how the same test-only hardware registry scales to sensors, encoders, and paired
velocity motors without changing production robot construction.

Complete [Test a mechanism without hardware](<../getting-started/Test a Mechanism Without Hardware.md>)
first. These are optional Reference case studies, not a second beginner path and not code to copy
wholesale.

## The scenario boundary

Each scenario constructs the real production mechanism with its ordinary `HardwareMap + Config`
constructor. `FtcTestHardware` supplies software device probes at that existing FTC boundary:

- a scenario **injects inputs** such as encoder ticks, measured velocity in ticks per second, or an
  electrical HIGH/LOW level;
- the production mechanism and Plants compute normally; and
- the scenario **records outputs** such as motor power, target ticks, commanded velocity, CR-servo
  power, or servo position.

Inputs never change merely because an output was commanded. These direct-setter scenarios contain
no motor, mechanism, battery, response-time, or game-piece physics. Call them software device
scenarios, not simulations.

## React to an observed command

A passive probe does not require a time-indexed observation schedule. The typed Java scenario can
request an action, run its fresh Task and the production output phase, assert the command that was
actually recorded, and only then inject the next named observation. The first Task and mechanism
updates use the unchanged start clock, so no pre-command interval is charged. For later cycles, the
fixture's `advance(...)` helper advances the shared `ManualLoopClock`, then explicitly updates the
Task before the mechanism. After that output phase, the scenario asserts status or the Task outcome.

This order avoids predicting a future command. It also keeps the boundary reviewable: a command is
software output, while a switch level, encoder count, or measured velocity is an independently
authored external fact. The private fixtures only remove repeated setup; the command and observation
steps remain visible in each test.

## Referenced lift: position and active-low input

The
[`ReferenceLiftSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcphoenix/robots/examples/reference/capability/lift/ReferenceLiftSoftwareScenarioTest.java>)
adds one motor and one digital input using the names from `ReferenceLiftMechanism.Config.defaults()`.
It then constructs the unchanged `ReferenceLiftMechanism`.

The motor's injected encoder ticks are measurement input. The bottom switch's injected electrical
level is a separate input: HIGH means the active-low semantic source is false; LOW means it is
true. The successful case starts with HIGH and explicit encoder evidence, requests a fresh homing
Task, and asserts the recorded homing power before injecting LOW. It then advances enough explicit
`ManualLoopClock` cycles for debounce, checks `SUCCESS` and reference establishment, and separately
injects encoder ticks to check the published position status. Recorded power and target ticks remain
outputs; neither one automatically moves the encoder or presses the switch.

The timeout case leaves the switch HIGH. After first proving that the homing command was issued, it
advances to the configured deadline and checks `TIMEOUT`, no reference, and release of the temporary
search command. That is Task and command-ownership cleanup in software; it is not evidence that a
physical lift stopped safely.

This is useful for proving software questions such as:

- Does LOW, not HIGH, satisfy this chosen bottom-switch polarity?
- Does the homing Task retain success only after conditioned switch evidence?
- Does a never-pressed switch produce `TIMEOUT` and release the search request?
- Does the position request scale inches into the expected target ticks?

It cannot prove that the switch is actually wired active-low, the lift reaches bottom safely, or
the encoder scale is physically correct.

## Launcher: independent wheel measurements

The
[`ReferenceLauncherSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcphoenix/robots/examples/reference/capability/launcher/ReferenceLauncherSoftwareScenarioTest.java>)
registers both flywheel motors plus the transfer CR servo, release servo, and object input before
constructing the unchanged `ReferenceLauncherMechanism`.

One velocity request is recorded on both motors, but left and right measured velocities are
injected independently. The successful case first asserts that the configured velocity reached
both probes. It then injects asymmetric measurements and proves readiness remains false until each
finite wheel measurement is inside its configured tolerance. The velocity command therefore does
not manufacture matching measurements or claim that either wheel spun.

The successful full-flow case starts a fresh `launchOne()` Task, observes both flywheel commands,
then injects ready measurements. Later clock advances expose the release and transfer commands in
their documented phases before the Task reaches `SUCCESS`; the next output update proves that
flywheel, release, and transfer requests returned to idle. The stalled-wheel case keeps one
measurement outside tolerance until the spin-up deadline, then checks `TIMEOUT`, no feed command,
and the same final idle cleanup.

These scenarios expose a one-sided or averaged readiness error and the Task phases that depend on
readiness. The same pattern can support separate focused cases for swapped names, a missing wheel,
non-finite measurements, or an incorrect inclusive tolerance. It does not predict spin-up time,
voltage sag, load recovery, vibration, launch result, or safe wheel speed.

## Optional inventory service: three observations, one snapshot

[`ReferenceInventoryStatusService.java`](<../../../robots/examples/reference/capability/inventory/ReferenceInventoryStatusService.java>)
is an optional robot-owned example, not a framework inventory type and not part of the ordinary
Reference robot. A custom OpMode can declare it before consumers:

```java
ReferenceInventoryStatusService inventory = program.service(
        new ReferenceInventoryStatusService(
                hardwareMap, ReferenceInventoryStatusService.Config.defaults()));

program.presenter((clock, telemetry) -> {
    ReferenceInventoryStatusService.Status status = inventory.status();
    telemetry.addData("inventory.count", status.conditionedOccupiedPositionCount);
    telemetry.addData("inventory.orderIssue", status.orderIssue);
});

program.rootTask(Tasks.waitUntil(inventory.fullSource(), 1.0));
```

The service phase samples first, second, then third once per cycle and publishes only after all
three conditioned reads succeed. `status()` and `fullSource()` read that same cache; neither
resamples hardware. If manual bulk caching is deliberately enabled, declare its cache owner first
and this inventory service second.

The ordered patterns are `000`, `100`, `110`, and `111`. The eight possible conditioned bit
patterns remain visible: count is the number of occupied sensor positions, while `OrderIssue`
describes `SECOND_WITHOUT_FIRST` or `THIRD_WITHOUT_SECOND`. Only ordered `111` is `full`. Before the
first successful START observation and after STOP, `observed` is false; that is different from a
successful conditioned `000` observation.

The hardware-free scenario begins with explicit HIGH levels, injects a LOW pulse shorter than the
debounce delay, then supplies stable LOW observations for the first, second, and third positions.
It later clears earlier inputs to expose both order issues. Those authored changes can represent an
ejection or shot in a test story, but the probe does not manufacture that physical event.

### Inventory scenario proves

For the authored cycles, the production/example owner applies active-low polarity and independent
debounce state, publishes an atomic immutable truth-table result, and gives presenters and Tasks
the same cached `full` fact.

### Inventory scenario does not prove

Software observations do not establish wiring or pullups, active-low meaning, sensor placement or
coverage, position order, physical object count, debounce under vibration, acquisition/ejection/
shot reliability, safety, timing, or game benefit.

### Inventory next gate

An adopting team must review the three configured inputs, observe raw and conditioned states on
the robot, deliberately move objects through every position, and record physical outcomes. Keep
the service robot-owned when the team's capacity, ordering, and policy differ from this example.

## Optional coordinated shot: one timestamped solution, one bounded turret

[`ReferenceCoordinatedShotService.java`](<../../../robots/examples/reference/capability/targeting/ReferenceCoordinatedShotService.java>)
and
[`ReferenceTurretMechanism.java`](<../../../robots/examples/reference/capability/targeting/ReferenceTurretMechanism.java>)
form an optional, example-only pair. They are not framework shooter types and the ordinary
Reference robot does not wire either owner. A custom managed OpMode can register its localization
owner first, then the calculation service, then the hardware output:

```java
ReferenceCoordinatedShotService shot = program.service(
        new ReferenceCoordinatedShotService(
                localization,
                ReferenceCoordinatedShotService.Config.defaults()));

ReferenceTurretMechanism turret = program.output(
        new ReferenceTurretMechanism(
                hardwareMap,
                ReferenceTurretMechanism.Config.defaults(),
                shot));
```

`localization` is the custom robot's already-owned `MotionPredictor`; the shot service borrows it
and never updates, resets, or stops it. The managed service phase publishes before the output phase,
so `shot.solution()` gives the turret, flywheel consumer, hood consumer, and presenter the same
clockless immutable snapshot for that cycle. Reading it does not resample localization. The turret
mechanism alone owns its motor, final resolver, periodic PositionPlant update, and stop.

The service asks one translation-only `SpatialQuery` for a stationary field point. One accepted
translation supplies the robot-forward/robot-left vector, quality, and observation timestamp used
by all three outputs. A finite, fresh `MotionDelta` may contribute only when its end timestamp is
exactly that observation timestamp. The illustrative moving calculation rotates the measured
displacement into current robot axes and applies `effectiveTarget = observedTarget - velocity *
flightTime`. Turret intent is `atan2(effectiveLeft, effectiveForward)`; the flywheel and hood tables
use the distance from that same effective vector.

The published mode makes degradation explicit:

- `MOVING_COMPENSATED` means spatial and co-temporal motion evidence passed their software gates;
- `STATIONARY_FALLBACK` means spatial geometry remained usable but same-clock motion was missing,
  stale, reset-invalidated, timestamp-mismatched, or otherwise unusable;
- `UNAVAILABLE` means the owner is not started/stopped or geometry/model output is unusable, so
  there is no turret request and the published flywheel/hood values are configured finite fallback
  intents for adopting consumers; and
- a timestamp from a different `LoopClock` is a wiring error that throws instead of silently
  degrading either mode.

The turret converts the cached request through
`PlantTargets.plan(...) -> nearest-to-measurement -> reject-unreachable -> observation acceptance ->
hold-measured-on-entry` and one bounded full-turn-periodic PositionPlant. An equivalent angle outside
the configured cable range is not silently clamped into a different meaning. Planner selection and
fallback are requested-target facts; neither one claims physical arrival.

The focused
[`ReferenceCoordinatedShotServiceTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcphoenix/robots/examples/reference/capability/targeting/ReferenceCoordinatedShotServiceTest.java>)
and
[`ReferenceTurretMechanismTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcphoenix/robots/examples/reference/capability/targeting/ReferenceTurretMechanismTest.java>)
contracts plus the
[`ReferenceCoordinatedShotSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcphoenix/robots/examples/reference/capability/targeting/ReferenceCoordinatedShotSoftwareScenarioTest.java>)
story supply authored timestamped poses and motion, then record the ordinary FTC-boundary turret
command. They also read flywheel and hood intents from that exact published solution rather than
recalculating them. The cases cover moving compensation, stationary fallback, unavailable geometry,
stale/reset/mismatched evidence, same-cycle snapshot coherence, reachable and unreachable periodic
alternatives, and service/output start/stop ownership.

### Coordinated-shot scenario proves

For the authored software observations, one service calculation publishes a coherent immutable
tuple, preserves its observation timestamp, names degraded modes truthfully, and supplies the live
observed request consumed by one bounded periodic Plant owner. It also proves that repeated reads in
one cycle do not independently resample localization and that stopping the example owners cannot
restart or reset the borrowed localization owner.

### Coordinated-shot scenario does not prove

The fixed flight time, interpolation tables, field point, ranges, fallback intents, and all default
numbers are uncalibrated software data. The scenario does not model projectile drop, spin, sensor
latency, articulated or off-center shooter velocity, motor response, cable routing, or collisions.
It does not establish shot accuracy, readiness, mechanism safety, a trustworthy encoder zero, full-
turn equivalence, physical bounds, or that any fallback is safe on a real robot.

### Coordinated-shot next gate

An adopting team must replace and validate the field/model data, establish the turret's reference
and cable limits, supervise actuator bring-up, and measure mechanism and shot outcomes on its own
robot. If the encoder zero is not physically trustworthy at START, use a reviewed reference or
homing design instead of copying the example's `alreadyReferenced()` assumption.

## Adapt the pattern to one team subsystem

1. Start from the subsystem's data-only production configuration.
2. Register only the software devices named by that configuration.
3. Construct the same production mechanism used by the robot; inject initial observations before
   construction or, when construction is passive, before its first sample.
4. Request one semantic action with a fresh Task when the action takes time.
5. Run Task before mechanism output, then assert the actual recorded command.
6. Inject the next named observation, advance the shared `ManualLoopClock` exactly once, and again
   update Task before mechanism output.
7. Assert the smallest status, Task-outcome, and recorded-output facts needed for the question.

Keep the setup behind a small test-local fixture when it distracts from the question, but leave
requests, command assertions, and observation injection in the test body. Prefer explicit units
such as `currentPositionTicks` and `measuredVelocityTicksPerSec`. Keep the scenario in plain Java
instead of asking students to predict observations in a per-cycle data file. A recorded run or a
physics model is a different experiment and needs its own stated evidence and assumptions.

### Proves

The real production mechanism interprets explicitly supplied device observations and realizes its
documented software outputs for the tested cycles.

### Does not prove

The probes do not model physics or establish physical wiring, polarity, calibration, timing under
load, safety, reliability, or game performance.

### Next gate

Use supervised [actuator bring-up](<../testing-calibration/Actuator Bring-up.md>) for the physical
device, then create a team-owned [subsystem experiment](<Subsystem Experiments.md>) with explicit
success criteria for the facts that only the robot can establish.

[Back to the examples index](<README.md>)
