---
tags:
  - Build
---

# Move a referenced lift and wait for feedback

**Outcome:** request a named lift height and let a cooperative Task finish only after fresh encoder
evidence says that exact request reached its target.

**Prerequisites:** complete [the lift-reference lesson](<Referenced Lift.md>) through its software
checkpoint. On hardware, establish the reference safely and repeatably before attempting this
page's first low-height move.

**Builds on:** the lift's bounded inch coordinate, ticks-per-inch conversion, active-low switch,
non-blocking home Task, private position Plant, and managed `Tasks -> Outputs` order.

**New here:** one semantic height command feeds both a direct persistent request and a fresh
feedback-aware Task. Status separates requested, applied, measured, and arrived facts, and the Task
states exactly what active cancellation does to the persistent request.

## Critical production idea

### Map named heights into the established coordinate

TeleOp and Auto use `STOWED`, `LOW`, and `HIGH`; only the mechanism owns their positions in inches:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java -->
```java
heightCommand = SemanticScalarCommand.forEnum(Height.STOWED)
        .map(Height.STOWED, c.stowedHeightIn)
        .map(Height.LOW, c.lowHeightIn)
        .map(Height.HIGH, c.highHeightIn)
        .build();
```

The same `heightCommand` is already bound to the private Plant from the reference lesson. Calling
`setHeight(LOW)` changes the persistent request immediately; the next mechanism update resolves it,
converts inches to ticks, writes the controller target, and refreshes cached feedback. A direct
setter does not wait and does not claim arrival. Configuration validation also requires the
position tolerance to be smaller than half the closest adjacent named-height gap, so the arrival
bands for two names cannot overlap merely because the tolerance was too wide.

### Build fresh work when a caller must wait

Auto can ask for a fresh single-use Task through the same semantic command:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java -->
```java
@Override
public Task moveTo(Height height) {
    return SemanticScalarTasks.set(heightCommand, Objects.requireNonNull(height, "height"))
            .untilReachedBy(lift)
            .leaveRequestOnCancel()
            .timeout(moveTimeoutSec)
            .build();
}
```

Read each stage as a behavior decision:

| Stage | Decision |
|---|---|
| `set(heightCommand, height)` | Publish the name and mapped inches together when this fresh Task starts. |
| `untilReachedBy(lift)` | Use this Plant's command-correlated cached feedback as completion evidence. |
| `leaveRequestOnCancel()` | Active cancellation ends the Task but deliberately leaves the latest persistent height request unchanged. |
| `timeout(moveTimeoutSec)` | Report `TIMEOUT` when arrival evidence is missing; do not pretend success or choose recovery. |
| `build()` | Return one fresh, single-use Task without publishing the request yet. |

This cancellation choice suits a position mechanism that should continue holding its requested
height. A caller that needs another hold point must request it explicitly. The Task never writes
the motor directly and never calls `lift.update(clock)`.

Reference state belongs to the current Plant instance, so a new OpMode run starts unreferenced even
if an earlier run homed successfully. In `BasicLiftTeleOp`, press X to start a fresh home Task in
that same run; wait until telemetry reports `lift.referenced = true`; then use D-pad down, left, or
up for direct `STOWED`, `LOW`, or `HIGH` requests:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftControls.java -->
```java
requiredCallbacks.onRise(
        driver.dpadDown(),
        () -> requiredLift.setHeight(BasicLift.Height.STOWED));
requiredCallbacks.onRise(
        driver.dpadLeft(),
        () -> requiredLift.setHeight(BasicLift.Height.LOW));
requiredCallbacks.onRise(
        driver.dpadUp(),
        () -> requiredLift.setHeight(BasicLift.Height.HIGH));

// A Supplier creates a fresh single-use homing Task on every X-button rise.
requiredTasks.onRise(driver.x(), requiredLift::home);
```

Those D-pad callbacks are direct setters and do not wait. `moveTo(...)` is the parallel capability
path for Auto or another coordinator that must wait for the exact Task outcome.

### Read evidence without collapsing its meaning

[`BasicLift.Status`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLift.Status.html>)
keeps four distinct questions visible:

| Status fact | What it answers |
|---|---|
| `requestedHeight()` / `requestedPositionIn()` | Which semantic request and mapped number are held now? |
| `appliedPositionIn()` | Which final bounded target did the Plant cache? |
| `measuredPositionIn()` | Which encoder-derived position did the Plant cache? |
| `atTarget()` | Is this exact semantic request still selected and supported by arrival evidence? |

`atTarget()` is controller evidence inside the configured tolerance. It cannot prove that the
encoder scale, load response, physical height, or mechanism safety is correct.

Notice:

- Direct TeleOp requests and feedback-aware Auto Tasks publish through one semantic command owner.
- A Task starts the request; the downstream Plant heartbeat performs the write and samples feedback.
- Success, timeout, and cancellation report software outcomes without inventing physical motion.

## Files in this checkpoint

**Main:**

- [`BasicLift`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLift.html>) — direct and Task capability methods plus status.
  [Complete source: `BasicLift.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLift.java>)
- [`BasicLiftMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.html>) — one semantic command and feedback Plant owner.
  [Complete source: `BasicLiftMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java>)
- `BasicLiftControls` — direct height meanings plus a fresh home binding.
  [Complete source: `BasicLiftControls.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftControls.java>)
- [`BasicLiftTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftTeleOp.html>) — mechanism-only managed host and evidence presenter.
  [Complete source: `BasicLiftTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftTeleOp.java>)

**Test:**

- [Complete source: `BasicLiftMoveSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMoveSoftwareScenarioTest.java>)

## Software checkpoint: fresh encoder evidence completes the move

- **Question:** Does a started `moveTo(LOW)` wait until a later output publication contains fresh
  encoder evidence for that selected request?
- **Keep real:** the production semantic command, feedback-aware Task, referenced Plant, and managed
  Task-before-output order.
- **Replace:** only the motor and bottom switch with recording software devices.
- **Observe:** target writes, semantic status, cached arrival, and exact Task outcome.
- **Cannot conclude:** physical direction, encoder scale, controller tuning, loaded stability,
  clearance, or safe travel.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMoveSoftwareScenarioTest.java -->
```java
// INJECT EVIDENCE: the fake motor moves only because this test supplies a new observation.
scenario.motor.setCurrentPositionTicks(lowTicks);
scenario.advance(0.02);
assertTrue(scenario.lift.status().atTarget());
assertFalse("Tasks run before outputs, so completion waits for the next cycle",
        scenario.task.isComplete());

// NEXT TASK PHASE: the Task now sees the fresh cached arrival and reports exact success.
scenario.advance(0.02);
assertEquals(TaskOutcome.SUCCESS, scenario.task.getOutcome());
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicmechanisms.BasicLiftMoveSoftwareScenarioTest
```

**Read the causal chain:** starting the Task publishes `LOW`; the next output heartbeat writes its
mapped encoder target but sees old feedback; the test supplies a later encoder observation; the
following Task phase can then observe that fresh cached arrival and report exact success.

**Proves:** semantic request correlation, managed update order, fresh-feedback completion, and the
software Task outcome for authored encoder evidence.

**Does not prove:** the physical lift moved to LOW, held it safely, or can repeat the motion under
load.

## Isolated hardware gate

Repeat the prior lesson's bottom-reference procedure in this same OpMode run; its recorded result
qualifies the procedure, not reference state in a future Plant. Mechanically support the lift, keep
a clear travel envelope and immediate STOP operator, and request only the lowest backed-off height
at conservative power. Compare measured inches with independent physical measurement before
trusting ticks per inch, tolerance, higher presets, timeout, or holding behavior.

**Next gate:** after one repeatable low-height move and physical STOP are established, continue to
[single-flywheel velocity](<Single Flywheel Velocity.md>). It keeps one motor and one Plant but
uses a numeric command because velocity itself is the complete capability request.
