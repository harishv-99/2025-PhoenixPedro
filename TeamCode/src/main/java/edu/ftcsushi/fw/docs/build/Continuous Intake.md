---
tags:
  - Build
---

# Run a continuous intake by name

**Outcome:** request `COLLECT`, `EJECT`, or `STOPPED` without spreading motor-power numbers through
controls and autonomous code.

**Prerequisites:** the project software checks pass; only the configured intake motor will be
connected during its isolated hardware gate.

## Critical production idea

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
modeCommand = SemanticScalarCommand.forEnum(Mode.STOPPED)
        .map(Mode.STOPPED, 0.0)
        .map(Mode.COLLECT, copiedCollectPower)
        .map(Mode.EJECT, copiedEjectPower)
        .build();
```

Notice:

- [`SemanticScalarCommand`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/SemanticScalarCommand.html>)
  keeps the named request and its numeric target in one coherent snapshot.
- [`StarterIntakeMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.html>)
  privately owns the Plant, update, and stop; callers use `StarterIntake` intent only.
- A request changes immediately, while the next managed output heartbeat applies it to hardware.

## Files in this checkpoint

**Main:**

- [`StarterIntake`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.html>) — capability API.
  [Complete source: `StarterIntake.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.java>)
- [`StarterIntakeMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.html>) — Plant owner.
  [Complete source: `StarterIntakeMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java>)
- [`StarterProfile`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/robot/StarterProfile.html>) — configuration and motion gate.
  [Complete source: `StarterProfile.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterProfile.java>)
- `StarterTeleOpControls` — A/B/X button meanings.
  [Complete source: `StarterTeleOpControls.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterTeleOpControls.java>)
- [`StarterRobot`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.html>) — focused declaration.
  [Complete source: `StarterRobot.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java>)
- [`StarterIntakeTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/opmode/StarterIntakeTeleOp.html>) — mechanism-only host.
  [Complete source: `StarterIntakeTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterIntakeTeleOp.java>)

**Test:**

- [Complete source: `StarterMechanismLessonTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterMechanismLessonTest.java>)

## Software checkpoint: request first, apply on heartbeat

- **Question:** Does one semantic request reach the configured motor power through the production
  Plant update?
- **Keep real:** the production capability, mechanism, command mapping, and Plant.
- **Replace:** only the FTC motor with a recording software device.
- **Observe:** semantic status, cached applied power, and the recorded device write.
- **Cannot conclude:** physical direction, safe ingestion, current draw, jamming, or stopping time.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterMechanismLessonTest.java -->
```java
// REQUEST: semantic intent changes immediately; hardware has not been updated yet.
intake.setMode(StarterIntake.Mode.COLLECT);
assertEquals(StarterIntake.Mode.COLLECT, intake.status().mode());
assertEquals(0.0, intake.status().appliedPower(), 0.0);
assertEquals(0, motor.powerWrites());

// HEARTBEAT: the same production update path maps the request and writes the motor.
intake.update(time.clock());
assertEquals(config.collectPower, intake.status().appliedPower(), 0.0);
assertEquals(config.collectPower, motor.power(), 0.0);
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.robot.StarterMechanismLessonTest
```

**Read the causal chain:** named intent changes the paired semantic/numeric request; the normal
output heartbeat resolves that request once; the fake motor records the boundary write.

**Proves:** semantic mapping, update order, cached status, and the submitted software command.

**Does not prove:** the mechanism collects, ejects, or stops safely on the physical robot.

## Isolated hardware gate

Keep the intake-only OpMode disabled and `allowIntakeMotion` false while reviewing configuration.
Restrain loose material, disconnect other motion owners, verify the motor name and direction, and
appoint an immediate STOP operator. Only then enable the OpMode and set the gate true for a
supervised low-power run that begins from `STOPPED`.

**Next gate:** after isolated direction and stop behavior are established, bind the same
`StarterIntake` capability into the full Starter TeleOp.
