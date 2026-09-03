---
tags:
  - Build
---

# Home a lift, then move to a named height

**Outcome:** establish an encoder reference from a bottom switch, request a named height, and finish
a cooperative move Task only when fresh feedback says that request arrived.

**Prerequisites:** the project software checks pass. No lift, switch, or encoder is required for the
software checkpoint; physical prerequisites begin only at the isolated hardware gate below, and
passing software tests does not authorize motion.

## Critical production idea

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java -->
```java
return SemanticScalarTasks.set(heightCommand, Objects.requireNonNull(height, "height"))
        .untilReachedBy(lift)
        .leaveRequestOnCancel()
        .timeout(moveTimeoutSec)
        .build();
```

Notice:

- [`SemanticScalarTasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/SemanticScalarTasks.html>)
  publishes the named and numeric request together and uses the selected Plant for feedback.
- [`PositionCalibrationTasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/PositionCalibrationTasks.html>)
  performs the non-blocking search; only success establishes the zero reference and selects `STOWED`.
- Timeout or cancellation leaves the persistent height request explicit; neither path pretends the
  lift arrived.

## Files in this checkpoint

**Main:**

- [`BasicLift`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLift.html>) — semantic capability and evidence.
  [Complete source: `BasicLift.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLift.java>)
- [`BasicLiftMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.html>) — reference, mapping, and Plant owner.
  [Complete source: `BasicLiftMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java>)
- [`BasicLiftProfile`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftProfile.html>) — mechanism facts and motion gate.
  [Complete source: `BasicLiftProfile.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftProfile.java>)
- `BasicLiftControls` — rising-edge height and home meanings.
  [Complete source: `BasicLiftControls.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftControls.java>)
- [`BasicLiftTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftTeleOp.html>) — mechanism-only host.
  [Complete source: `BasicLiftTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftTeleOp.java>)

**Test:**

- [Complete source: `BasicLiftSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftSoftwareScenarioTest.java>)

## Software checkpoint: authored switch evidence controls homing

- **Question:** Does the authored active-low switch observation establish reference only after its
  debounce interval?
- **Keep real:** the production lift, homing Task, switch interpretation, Plant, and loop order.
- **Replace:** only the motor and digital channel with recording software devices.
- **Observe:** the homing Task outcome and the mechanism's reference status.
- **Cannot conclude:** switch placement, wiring, motor polarity, conversion accuracy, limits, or
  safe motion.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftSoftwareScenarioTest.java -->
```java
// INJECT EVIDENCE: LOW must remain observed long enough to pass the configured debouncer.
scenario.bottomSwitch.setHigh(false);
scenario.advance(0.01);
assertFalse(scenario.task.isComplete());
scenario.advance(0.01);
assertEquals(TaskOutcome.SUCCESS, scenario.task.getOutcome());
assertTrue(scenario.lift.status().referenced());
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicmechanisms.BasicLiftSoftwareScenarioTest
```

**Read the causal chain:** the test explicitly supplies switch evidence across real clock cycles;
the first observation is shorter than the debounce interval; the second observation lets the
production homing Task finish and publish referenced status.

**Proves:** software polarity interpretation, debounce timing, and the reference transition for the
authored observations.

**Does not prove:** the real switch changes safely, the lift travels in the intended direction, or
the encoder scale and controller are tuned.

## Isolated hardware gate

Keep `BasicLiftTeleOp` disabled and `allowLiftMotion` false while reviewing configuration. First write
the switch-polarity, motor-direction, travel-envelope, and emergency-stop check plan. Mechanically
support the lift, start away from hard stops, confirm released/pressed switch telemetry, and verify
a tiny manual-direction test. Only then enable the OpMode and motion gate for low-power homing with
an immediate STOP operator. Measure travel before accepting ticks-per-inch, bounds, named heights,
or tolerance.

**Next gate:** after recording repeatable homing and one low-height move, declare the proven lift
capability, output, and control binding in the team's TeleOp composition root; keep the
[robot-role boundaries](<../getting-started/learn-sushi/Robot Roles.md>) unchanged.
