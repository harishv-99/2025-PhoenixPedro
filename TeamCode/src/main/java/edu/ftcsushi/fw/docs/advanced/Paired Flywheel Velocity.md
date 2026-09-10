---
tags:
  - Advanced
---

# Command paired flywheel velocity without hiding readiness

Use this focused example when two wheels share one velocity request but each wheel supplies evidence
that must be true before the mechanism is ready. A grouped mean alone can hide one fast wheel and
one slow wheel.

**Before this page:** read [one motor velocity](<../build/Single Flywheel Velocity.md>) for encoder
speed, tolerance, and feedback-based Tasks. No paired hardware is needed to understand this example.
The new idea is **readiness for a group**: an average can match the request even when neither wheel
does. Here, both individual speeds must be close enough to the request for wheel readiness. Deciding
whether an object may be fed also needs the [feeding policy](<Feedback-confirmed Feeding.md>).

## One command owner, two observations

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheelMechanism.java -->
```java
return FtcActuators.plant(map)
        .motor(c.leftMotorName, c.leftMotorDirection)
        .andMotor(c.rightMotorName, c.rightMotorDirection)
        .velocity()
        .deviceManaged()
        .bounded(IDLE_VELOCITY_TICKS_PER_SEC, c.maximumVelocityTicksPerSec)
        .nativeUnits()
        .velocityTolerance(c.velocityToleranceTicksPerSec)
        .targetFromNewCommand(IDLE_VELOCITY_TICKS_PER_SEC)
        .build();
```

Notice:

- [`ReferenceFlywheelMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheelMechanism.html>)
  owns the one grouped Plant, persistent request, update, and stop.
- [`ReferenceFlywheels.Status`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheels.Status.html>)
  composes the generic Plant snapshot with two independently sampled velocities.
- `setVelocityTask(...)` succeeds only after a new publication proves both wheel measurements are
  within tolerance; timeout leaves the request, while active cancellation requests zero.

Wheel status also records which successful request occurrence was sampled and when the software
sampled it. A same-valued request is still new work; a repeated update in the same clock cycle
cannot create another eligible sample. These are software observations, not proof of independent
new motor-controller frames or uninterrupted physical readiness between samples.

## Complete example files

- [Complete source: `ReferenceFlywheels.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheels.java>)
- [Complete source: `ReferenceFlywheelMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheelMechanism.java>)
- [Complete source: `ReferenceFlywheelMechanismOpMode.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/opmode/ReferenceFlywheelMechanismOpMode.java>)
- [Complete source: `ReferenceFlywheelSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheelSoftwareScenarioTest.java>)

## Software checkpoint

- **Question:** Can opposite member errors cancel in the grouped mean without producing readiness?
- **Keep real:** the mechanism, grouped Plant, readiness status, Task, and output heartbeat.
- **Replace:** only the two FTC motors.
- **Observe:** both recorded commands, grouped arrival, independent measurements, readiness, and Task outcome.
- **Cannot conclude:** physical direction, tuning, balance under load, release, or scoring.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheelSoftwareScenarioTest.java -->
```java
// HEARTBEAT: the production owner writes both motors and publishes one complete status.
scenario.flywheels.update(scenario.time.clock());
ReferenceFlywheels.Status unbalanced = scenario.flywheels.status();
assertEquals(1000.0, scenario.left.commandedVelocityTicksPerSec(), EPSILON);
assertTrue("the grouped mean is at target", unbalanced.plantSnapshot().atCommandTarget());
assertFalse("independent member evidence prevents a false ready claim", unbalanced.ready());
```

**Read the causal chain:** one request writes both motors; independent `800` and `1200` samples
average to the `1000` request; grouped arrival is true, but per-wheel readiness stays false.

**Reading checkpoint:** explain why the average `1000` cannot, by itself, authorize feeding.

**Proves:** paired readiness does not substitute an aggregate mean for member evidence.

**Does not prove:** the configured velocity is physically safe, stable, or effective.

**Next gate:** use [Actuator bring-up](<../testing-calibration/Actuator Bring-up.md>) on each motor,
then define and run a restrained spin-up experiment with an explicit settling criterion.

For the separate decision to feed an object, continue to
[Feedback-confirmed feeding](<Feedback-confirmed Feeding.md>). It combines the wheel evidence with
staging and later departure observations; the spin-up check alone does not authorize feeding.
