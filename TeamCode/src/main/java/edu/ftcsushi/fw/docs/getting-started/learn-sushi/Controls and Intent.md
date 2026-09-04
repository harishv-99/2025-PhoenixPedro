---
tags:
  - Learn
---

# Controls and intent

**Learning mode:** Architecture reference

This page explains controls ownership and APIs. The
Starter buildable module supplies the complete controls file and focused test.

**Prerequisite:** take the [First software tour](<../First Software Tour.md>) first if `() ->` or
“run once when pressed” is new. It explains, with familiar `if` code, why setup can save a function
without running it and when Sushi calls that function later.

**Question:** How does a human action become robot intent without putting gamepad policy inside a
mechanism?

Controls are the code where a team decides what each stick or button means. Sushi calls this
decision **intent**. Controls do not construct hardware or update motors. You can follow this
source-only lesson without a gamepad or robot.

## A button becomes a capability request

### Critical code

[`GamepadDevice`](<../../../ftc/input/GamepadDevice.java>) adapts the FTC gamepad to Sushi sources.
`gamepad.a()` is a `BooleanSource` that is `true` while A is pressed. That meaning is not an
electrical HIGH or LOW signal. A trigger is instead a `ScalarSource` from `0.0` to `1.0`; code can
derive a Boolean meaning with, for example, `rightTrigger().above(0.2)`.

The complete Starter intake mapping lives in
[`StarterTeleOpControls`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterTeleOpControls.java>):

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
// ...
requiredCallbacks.onRise(driver.a(),
        () -> requiredIntake.setMode(StarterIntake.Mode.COLLECT));
requiredCallbacks.onRise(driver.b(),
        () -> requiredIntake.setMode(StarterIntake.Mode.EJECT));
requiredCallbacks.onRise(driver.x(),
        () -> requiredIntake.setMode(StarterIntake.Mode.STOPPED));
// ...
```

**What to notice**

- Buttons map to semantic modes, never directly to motor power.
- The callback owner and capability dependency are explicit in each registration.

**Key APIs:** `CallbackBindings.onRise(...)` declares an edge meaning; `BooleanSource` provides the
cycle-aware input fact.

```text
A pressed -> BooleanSource true -> rising edge -> setMode(COLLECT)
          -> mechanism records a persistent semantic request
```

The binding owns what A means. The
[`StarterIntake`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.html>)
capability names what the robot should do. Its mechanism owns how hardware realizes that request.
Holding A for twenty cycles produces one rising edge; COLLECT persists because `setMode` replaced
the held request.

## Drive follows a separate path

The Starter controls also expose a robot-centric `DriveSource`. Stick and slow-mode sources are
sampled when `RobotProgram` reaches the **output/drive phase**; they do not pass through a callback
binding on every cycle.

```text
sticks + bumper -> controls-owned DriveSource -> DriveSignal
                 -> program.drive(source, sink) -> one mecanum hardware write
```

Controls never call motor setters. Robot-centric forward follows the robot. Field-relative “up”
requires an explicitly chosen field frame and is demonstrated only in the optional
[`Field-relative Drive`](<../../examples/Field-relative Drive.md>) example.

## How the pattern scales: callback or Task?

### Critical code

Use a callback when an action completes synchronously by replacing intent. Use a Task binding when
non-blocking behavior unfolds over several managed cycles:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/opmode/ReferenceFlywheelMechanismOpMode.java -->
```java
program.callbackBindings().onRise(
        operator.a(),
        () -> flywheels.setVelocityTicksPerSec(TEST_VELOCITY_TICKS_PER_SEC));
program.taskBindings().onRise(
        operator.y(),
        () -> flywheels.setVelocityTask(
                TEST_VELOCITY_TICKS_PER_SEC,
                WAIT_TIMEOUT_SEC));
```

**What to notice**

- A synchronous persistent velocity request is a callback; waiting for feedback is a Task.
- The lambda calls `setVelocityTask(...)` on each eligible edge, so every run receives fresh work.

**Key APIs:** `TaskBindings.onRise(...)` accepts a Task supplier; `CallbackBindings.onRise(...)`
accepts an immediate semantic callback.

`setVelocityTask(...)` is a factory method. Each eligible press returns a **fresh, single-use
Task**; controls never run it in a private loop. The focused
[`ReferenceFlywheelMechanismOpMode`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/opmode/ReferenceFlywheelMechanismOpMode.html>)
shows the direct and deferred forms side by side; its
[Complete source: `ReferenceFlywheelMechanismOpMode.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/opmode/ReferenceFlywheelMechanismOpMode.java>)
is the compiling authority. Declaration order remains observable when multiple buttons rise in one
cycle.

## Check your understanding

**B should request COLLECT instead of EJECT. What changes?** The controls mapping, not the
capability or mechanism.

**COLLECT should use a different reviewed motor power. What changes?** Intake configuration, never
the button binding.

**A fully pressed trigger is digital HIGH. True or false?** False. It is a scalar operator value;
a threshold may derive a semantic Boolean, but not an electrical pin state.

## Go deeper when needed

- [Tasks and autonomous](<Tasks and Autonomous.md>) — Task lifetime, timing, and outcomes
- [Sources and signals](<../../core-concepts/Sources and Signals.md>) — source caching and mapping
- [Evidence and experiments](<Evidence and Experiments.md>) — electrical switch polarity and debounce
- [Learn Sushi topic guide](<../Beginner's Guide.md>) — choose another topic
