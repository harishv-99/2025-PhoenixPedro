# Controls and intent

**Question:** How does a human action become robot intent without putting gamepad policy inside a
mechanism?

Controls own operator meanings. They turn stable input sources into capability calls or drive
intent; they do not construct hardware or update Plants. You can follow this source-only lesson
without a gamepad or robot.

## A button becomes a capability request

[`GamepadDevice`](<../../../ftc/input/GamepadDevice.java>) adapts the FTC gamepad to Sushi sources.
`gamepad.a()` is a `BooleanSource` that is `true` while A is pressed. That meaning is not an
electrical HIGH or LOW signal. A trigger is instead a `ScalarSource` from `0.0` to `1.0`; code can
derive a Boolean meaning with, for example, `rightTrigger().above(0.2)`.

The complete Starter intake mapping lives in
[`StarterTeleOpControls`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterTeleOpControls.java>):

```java
requiredCallbacks.onRise(driver.a(),
        () -> requiredIntake.setMode(StarterIntake.Mode.COLLECT));
requiredCallbacks.onRise(driver.b(),
        () -> requiredIntake.setMode(StarterIntake.Mode.EJECT));
requiredCallbacks.onRise(driver.x(),
        () -> requiredIntake.setMode(StarterIntake.Mode.STOPPED));
```

```text
A pressed -> BooleanSource true -> rising edge -> setMode(COLLECT)
          -> mechanism records a persistent semantic request
```

The binding owns what A means. The
[`StarterIntake`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.java>)
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

Use a callback when an action completes synchronously by replacing intent. Use a Task binding when
non-blocking behavior unfolds over several managed cycles:

```java
requiredCallbacks.onRise(gamepad.dpadUp(),
        () -> requiredLift.setHeight(ReferenceLift.Height.HIGH));
requiredTasks.onRise(gamepad.x(), requiredLift::home);
```

`lift::home` is a factory. Each eligible press returns a **fresh, single-use Task**; controls never
run it in a private loop. The
[`ReferenceTeleOpControls`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/robot/ReferenceTeleOpControls.java>)
also uses this rule for launcher spin-up/feed behavior. Declaration order remains observable when
multiple buttons rise in one cycle.

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
