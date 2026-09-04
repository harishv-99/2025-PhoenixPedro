---
tags:
  - Build
---

# Reach one flywheel velocity and observe arrival

**Outcome:** command one motor in encoder ticks per second, observe requested, applied, and measured
velocity separately, and create a non-blocking Task that succeeds only from feedback.

**Prerequisites:** complete the [continuous-intake lesson](<Continuous Intake.md>) for mechanism
ownership and [move a referenced lift](<Move a Referenced Lift.md>) for feedback-aware Tasks. No
robot is needed until the isolated hardware gate below.

**Builds on:** one capability, one mechanism-owned Plant, one managed output heartbeat, and fresh
single-use Tasks.

**New here:** velocity itself is the public capability intent. There is no named mode to map, so
direct behavior writes the Plant-owned numeric command and deferred behavior uses `ScalarTasks`.

## Critical production idea

The mechanism constructs one feedback-capable FTC velocity Plant:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelMechanism.java -->
```java
flywheel = FtcActuators.plant(map)
        .motor(c.motorName, c.direction)
        .velocity()
        .deviceManaged()
        .bounded(STOPPED_VELOCITY_TICKS_PER_SEC, c.maximumVelocityTicksPerSec)
        .nativeUnits()
        .velocityTolerance(c.velocityToleranceTicksPerSec)
        .targetFromNewCommand(STOPPED_VELOCITY_TICKS_PER_SEC)
        .build();
```

Read the stages as questions:

| Stage | Question answered |
|---|---|
| `motor(name, direction)` | Which FTC motor realizes positive flywheel motion? |
| `velocity()` | Is the requested quantity power, position, or velocity? |
| `deviceManaged()` | Does the FTC motor controller own the velocity loop? |
| `bounded(0.0, maximum)` | Which velocity requests may reach this hardware, including zero? |
| `nativeUnits()` | Are public values already encoder ticks per second? |
| `velocityTolerance(...)` | How close must measured velocity be for software arrival? |
| `targetFromNewCommand(0.0)` | Which persistent request owns intent, and what is its initial value? |
| `build()` | Is the complete recipe ready to become one Plant? |

`build()` resolves and configures the recipe; it does not submit a motion command. The initial
command is zero, and the mechanism's first normal `update(clock)` is what submits it. Ordinary
`deviceManaged()` also leaves the FTC controller's existing PIDF coefficients unchanged. Software
construction therefore proves neither that those coefficients are tuned nor that a numeric value
is safe on a particular mechanism. The example rejects a tolerance equal to or larger than its
maximum before hardware lookup, so a zero measurement is not inside the completion band at the
maximum request.

Immediate behavior retrieves the Plant's stable command where it is used:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelMechanism.java -->
```java
public void setVelocityTicksPerSec(double velocityTicksPerSec) {
    requireVelocityInRange(velocityTicksPerSec);
    flywheel.commandTarget().set(velocityTicksPerSec);
}
```

The feedback-aware path uses that exact command and names the same Plant as its observer:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelMechanism.java -->
```java
return ScalarTasks.set(flywheel.commandTarget(), velocityTicksPerSec)
        .untilReachedBy(flywheel)
        .cancelTo(STOPPED_VELOCITY_TICKS_PER_SEC)
        .timeout(spinUpTimeoutSec)
        .build();
```

The Task writes only when it starts. Success requires command-correlated Plant arrival. Timeout
leaves the persistent request unchanged; active cancellation requests zero exactly once, and the
next mechanism heartbeat realizes that request through the same Plant graph.

Status is a view over one immutable Plant capture, not a second cache:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelMechanism.java -->
```java
@Override
public Status status() {
    return new Status(flywheel.snapshot());
}
```

[`BasicFlywheel.Status`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheel.Status.html>)
keeps four different claims visible: current requested velocity, cached applied velocity, cached
encoder measurement, and whether that captured request has command-correlated arrival evidence.

Notice:

- [`Plant.commandTarget()`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/Plant.html>)
  returns the same side-effect-free request object for this exact Plant.
- [`ScalarTasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/ScalarTasks.html>)
  supplies the ordinary feedback lifetime and requires an explicit active-cancellation choice.
- [`BasicFlywheelMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelMechanism.html>)
  privately owns the Plant, `update`, and terminal `stop`; controls never touch FTC hardware.

## Files in this checkpoint

**Main:**

- [`BasicFlywheel`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheel.html>) — numeric capability and thin Status.
  [Complete source: `BasicFlywheel.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheel.java>)
- [`BasicFlywheelMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelMechanism.html>) — configuration, Plant, Tasks, and lifecycle owner.
  [Complete source: `BasicFlywheelMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelMechanism.java>)
- `BasicFlywheelControls` — A/B direct fixture meanings and one-time binding owner.
  [Complete source: `BasicFlywheelControls.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelControls.java>)
- [`BasicFlywheelTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelTeleOp.html>) — disabled, fail-closed mechanism-only host.
  [Complete source: `BasicFlywheelTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelTeleOp.java>)

**Test:**

- [Complete source: `BasicFlywheelSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelSoftwareScenarioTest.java>)
- [Complete source: `BasicFlywheelConfigurationContractTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelConfigurationContractTest.java>) — supplied maintainer boundary coverage.

## Software checkpoint: fresh feedback completes the selected request

- **Question:** Does a new 300-ticks/sec Task reject cached success for the older 200-ticks/sec
  request, then finish only after a later heartbeat publishes matching feedback?
- **Keep real:** the numeric capability, production mechanism, staged Plant, `ScalarTasks` Task,
  status snapshot, and heartbeat order.
- **Replace:** only the FTC motor with the supplied recording software device.
- **Observe:** the prior ready snapshot, incomplete new Task, later measured velocity, and exact
  Task outcome.
- **Cannot conclude:** physical direction, encoder scale, PIDF tuning, loaded speed, coast-down, or
  emergency-stop distance.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelSoftwareScenarioTest.java -->
```java
// OLD SUCCESS: make the prior request's cached evidence genuinely successful.
scenario.motor.setMeasuredVelocityTicksPerSec(200.0);
scenario.flywheel.update(scenario.time.nextCycle(0.02));
assertTrue(scenario.flywheel.status().atRequestedVelocity());

// REQUEST: the Task writes a different numeric intent but cannot reuse that old success.
Task reachVelocity = scenario.flywheel.setVelocityTask(300.0);
reachVelocity.start(scenario.time.nextCycle(0.02));
reachVelocity.update(scenario.time.clock());
assertFalse(reachVelocity.isComplete());
```

Only the explicitly authored later observation can complete it:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicflywheel/BasicFlywheelSoftwareScenarioTest.java -->
```java
// INJECT EVIDENCE + HEARTBEAT: a later owner update observes the requested velocity.
scenario.motor.setMeasuredVelocityTicksPerSec(300.0);
scenario.flywheel.update(scenario.time.clock());

// ASSERT: the following Task phase consumes that cached feedback and succeeds.
reachVelocity.update(scenario.time.nextCycle(0.02));
BasicFlywheel.Status reached = scenario.flywheel.status();
assertEquals(TaskOutcome.SUCCESS, reachVelocity.getOutcome());
assertEquals(300.0, reached.measuredVelocityTicksPerSec(), EPSILON);
assertTrue(reached.atRequestedVelocity());
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicflywheel.BasicFlywheelSoftwareScenarioTest
```

**Read the causal chain:** 200 is first supported by matching cached feedback; the new Task publishes
300 but rejects that older success; a later output heartbeat caches the explicitly injected 300
measurement; the next Task phase reports exact `SUCCESS`. The second scenario in the same focused
file also verifies `TIMEOUT` leaves the request unchanged and active cancellation requests zero
before the normal heartbeat realizes it.

**Proves:** numeric request timing, one realization path, requested/applied/measured separation,
command-correlated feedback success, timeout, and cancellation-to-zero semantics in software.

**Does not prove:** the candidate values produce safe, stable, or useful physical flywheel motion.

## Isolated hardware gate

The checked-in `BasicFlywheelTeleOp` has both `@Disabled` and `MOTION_REVIEWED = false`. Leave both
locks in place while you review `Config.defaults()`: `flywheelMotor`, `FORWARD`, the `500` ticks/sec
maximum, `25` ticks/sec tolerance, and two-second timeout are software-valid candidates, not
reviewed facts for your motor.

Secure or remove the flywheel load, disconnect other motion owners, verify the motor name and
positive direction, confirm the encoder's ticks-per-revolution and expected sign, choose a backed-
off range for that exact assembly, and appoint an immediate STOP operator. Use
[Actuator bring-up](<../testing-calibration/Actuator Bring-up.md>) before relying on the checked-in
`250` ticks/sec test candidate. Only after that review should you set the gate true, remove
`@Disabled`, and try A for the candidate request and B for a persistent zero request. The fixture
validates that its candidate is above the zero measurement's tolerance band and no greater than the
configured maximum before constructing hardware. Feedback-aware Tasks are exercised in the
software scenario and belong in Auto or another coordinator with explicit abort policy, rather
than in a TeleOp button queue. Measure actual coast-down; a submitted zero command is not proof
that the mechanism has physically stopped.

**Next gate:** after one motor has proven direction, range, feedback, and stop behavior, study the
[advanced paired-flywheel example](<../advanced/Paired Flywheel Velocity.md>) only if two wheels
share a command but require independent readiness evidence.
