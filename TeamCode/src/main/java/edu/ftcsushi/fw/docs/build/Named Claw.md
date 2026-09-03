---
tags:
  - Build
---

# Move a claw through named safe positions

**Outcome:** use `CLOSED`, `HALF`, and `OPEN` everywhere while one mechanism maps normalized
coordinates `0.0`, `0.5`, and `1.0` into reviewed native servo endpoints.

**Prerequisites:** the project software checks pass. No servo or linkage is required for the
software checkpoint; physical prerequisites begin only at the isolated hardware gate below.

## Critical production idea

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanism.java -->
```java
claw = FtcActuators.plant(map)
        .servo(c.servoName, c.direction)
        .position()
        .nonPeriodic()
        .bounded(CLOSED_TARGET, OPEN_TARGET)
        .rangeMapsToNative(c.closedNativePosition, c.openNativePosition)
        .targetExactlyFrom(stateCommand)
        .build();
```

Notice:

- [`BasicClaw.State`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.State.html>)
  gives robot code names; the mechanism owns their normalized coordinates.
- [`FtcActuators`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcActuators.html>)
  maps the bounded logical range to native endpoints `0.25` and `0.70`; `HALF` derives `0.475`.
- A standard servo provides no position feedback here, so status says **applied coordinate**, never
  physical arrival.

## Files in this checkpoint

**Main:**

- [`BasicClaw`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.html>) — capability and status.
  [Complete source: `BasicClaw.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.java>)
- [`BasicClawMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanism.html>) — mapping and Plant owner.
  [Complete source: `BasicClawMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanism.java>)
- [`BasicClawProfile`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawProfile.html>) — reviewed facts and motion gate.
  [Complete source: `BasicClawProfile.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawProfile.java>)
- `BasicClawControls` — rising-edge A/Y/B meanings.
  [Complete source: `BasicClawControls.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawControls.java>)
- [`BasicClawTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawTeleOp.html>) — mechanism-only host.
  [Complete source: `BasicClawTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawTeleOp.java>)

**Test:**

- [Complete source: `BasicClawSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawSoftwareScenarioTest.java>)

## Software checkpoint: normalized half derives from two endpoints

- **Question:** Does `HALF` remain a named request while its normalized midpoint maps through the
  same reviewed native range?
- **Keep real:** the production semantic command, servo Plant, bounds, and range mapping.
- **Replace:** only the FTC servo with a recording software device.
- **Observe:** requested state/coordinate, applied coordinate, and recorded native command.
- **Cannot conclude:** endpoint clearance, linkage geometry, interpolation, or physical arrival.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawSoftwareScenarioTest.java -->
```java
// HALF: semantic intent changes now; the normal heartbeat derives the native midpoint.
claw.setState(BasicClaw.State.HALF);
assertEquals(BasicClaw.State.HALF, claw.status().requestedState());
assertEquals(0.5, claw.status().requestedCoordinate(), 0.0);
assertEquals(1, servo.positionWrites());
claw.update(time.nextCycle(0.02));
assertEquals(0.5, claw.status().appliedCoordinate(), 0.0);
assertEquals(0.475, servo.position(), 1e-12);
assertEquals(2, servo.positionWrites());
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicmechanisms.BasicClawSoftwareScenarioTest
```

**Read the causal chain:** `HALF` selects normalized `0.5`; the managed Plant heartbeat applies the
bounded target; the range map derives the native midpoint; the software servo records that command.

**Proves:** the named request and submitted command stay coherent through the configured mapping.

**Does not prove:** the claw is halfway open, reached the command, or avoided a mechanical stop.

## Isolated hardware gate

Keep `BasicClawTeleOp` disabled and `allowClawMotion` false while reviewing configuration. Confirm
that the horn is removable or the linkage can be tested without entering a pinch or hard-stop
region. Begin with the horn or linkage disconnected when practical, establish backed-off endpoints
incrementally, and reinstall the linkage. Only then enable the OpMode and motion gate for a supervised
`CLOSED`–`HALF`–`OPEN` run while watching clearance and pinch zones.

**Next gate:** after recording that isolated run, declare the proven claw capability, output, and
control binding in the team's TeleOp composition root; keep the
[robot-role boundaries](<../getting-started/learn-sushi/Robot Roles.md>) unchanged.
