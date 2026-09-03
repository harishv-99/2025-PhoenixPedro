---
tags:
  - Advanced
---

# Command paired flywheel velocity without hiding readiness

Use this focused example when two wheels share one velocity request but each wheel supplies evidence
that must be true before the mechanism is ready. A grouped mean alone can hide one fast wheel and
one slow wheel.

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

**Proves:** paired readiness does not substitute an aggregate mean for member evidence.

**Does not prove:** the configured velocity is physically safe, stable, or effective.

**Next gate:** use [Actuator bring-up](<../testing-calibration/Actuator Bring-up.md>) on each motor,
then define and run a restrained spin-up experiment with an explicit settling criterion.
