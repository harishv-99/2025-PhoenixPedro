---
tags:
  - Advanced
---

# Select the nearest legal periodic turret position

Use periodic position when one logical angle has physically interchangeable full-turn
representatives. Do not use semantic names with hidden periodic behavior: the focused capability
accepts numeric radians and makes the physical representative visible in status.

## Resolve before the Plant applies bounds

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/targeting/ReferencePeriodicTurretMechanism.java -->
```java
angleCommand = ScalarTarget.create(c.initialAngleRad);
PlantTargetResolver nearestEquivalent = PlantTargets
        .equivalentPositionsOf(angleCommand)
        .nearestToMeasurement()
        .whenUnavailable()
        .holdMeasuredTargetOnEntry(c.initialAngleRad);
```

The Plant then declares periodicity, legal physical bounds, measurement conversion, tolerance, and
that resolver in one complete construction chain:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/targeting/ReferencePeriodicTurretMechanism.java -->
```java
turret = FtcActuators.plant(map)
        .motor(c.motorName, c.direction)
        .position()
        .deviceManaged()
        .periodic(FULL_TURN_RAD)
        .bounded(c.minimumAngleRad, c.maximumAngleRad)
        .scaleToNative(c.ticksPerRad)
        .alreadyReferenced()
        .positionTolerance(c.positionToleranceRad)
        .targetFromResolver(nearestEquivalent)
        .build();
```

Notice:

- [`PlantTargets.equivalentPositionsOf(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/PlantTargets.html>)
  preserves the logical command while choosing one bounded physical representative.
- [`ReferencePeriodicTurretMechanism.Status`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/targeting/ReferencePeriodicTurretMechanism.Status.html>)
  keeps requested, selected, applied, measured, and arrived facts separate.
- Timeout and active cancellation leave the persistent position request unchanged so the
  controller continues holding it; a caller requests a different hold point explicitly.

## Complete example files

- [Complete source: `ReferencePeriodicTurretMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/targeting/ReferencePeriodicTurretMechanism.java>)
- [Complete source: `ReferencePeriodicTurretOpMode.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/opmode/ReferencePeriodicTurretOpMode.java>)
- [Complete source: `ReferencePeriodicTurretSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/reference/capability/targeting/ReferencePeriodicTurretSoftwareScenarioTest.java>)

## Software checkpoint

- **Question:** Does a logical `0.1π` request choose the nearest legal full-turn representative
  when the measured turret begins near `2.9π`?
- **Keep real:** the command, periodic resolver, Plant, Task status, and heartbeat.
- **Replace:** only the FTC motor and its encoder sample.
- **Observe:** requested, selected, applied, measured, and arrival facts.
- **Cannot conclude:** encoder zero/scale, cable clearance, motor direction, tuning, or safety.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/reference/capability/targeting/ReferencePeriodicTurretSoftwareScenarioTest.java -->
```java
// HEARTBEAT: equivalentPositionsOf chooses the nearest in-range full-turn representative.
scenario.turret.update(scenario.time.clock());
ReferencePeriodicTurretMechanism.Status moving = scenario.turret.status();
assertEquals(logicalRequestRad, moving.requestedAngleRad(), EPSILON);
assertEquals(expectedPhysicalRad, moving.selectedAngleRad(), EPSILON);
assertEquals(expectedPhysicalRad, moving.appliedAngleRad(), EPSILON);
```

**Read the causal chain:** the numeric request stays `0.1π`; the current measurement makes `2.1π`
the nearest legal equivalent; only a later injected encoder sample can establish arrival.

**Proves:** periodic selection stays distinct from final bounds and from feedback arrival.

**Does not prove:** any configured turn is physically reachable or safe for wiring.

**Next gate:** establish zero, encoder scale, direction, hard cable/collision limits, backed-off
software bounds, and a safe hold power with the turret isolated.
