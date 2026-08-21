# Controls and intent

**Question this chapter answers:** How does a human action become robot intent without putting
gamepad policy inside a mechanism?

**Reading time:** about 15 minutes

Controls own meanings. They turn stable input sources into capability calls and drive intent; they
do not construct hardware or update Plants. This chapter is source-only—you do not need a gamepad
or robot to follow the data flow.

## Source map

| Read | Look for |
| --- | --- |
| [`StarterTeleOpControls.java`](<../../../../robots/examples/starter/robot/StarterTeleOpControls.java>) | Stable input sources, callback bindings, and robot-centric drive intent |
| [`ReferenceTeleOpControls.java`](<../../../../robots/examples/reference/robot/ReferenceTeleOpControls.java>) | The distinction between synchronous callbacks and fresh Tasks |
| [`StarterIntake.java`](<../../../../robots/examples/starter/capability/intake/StarterIntake.java>) | Semantic requests that contain no button names or motor powers |
| [`GamepadDevice.java`](<../../../ftc/input/GamepadDevice.java>) | The FTC boundary that exposes buttons and axes as Phoenix sources |

## A button is a semantic source

`GamepadDevice` wraps the FTC SDK `Gamepad`. A button source is `true` while that named button is
pressed. That is already a human-facing meaning; it is not a statement about an electrical HIGH or
LOW pin inside the controller.

The Starter controls constructor retains the boundary object and creates its stable drive source:

```java
StarterTeleOpControls(GamepadDevice driver) {
    this.driver = Objects.requireNonNull(driver, "driver");

    driveSource = new GamepadDriveSource(
            this.driver.leftX(),
            this.driver.leftY(),
            this.driver.rightX(),
            GamepadDriveSource.Config.defaults()
    ).scaledWhen(this.driver.rightBumper(), SLOW_TRANSLATE_SCALE, SLOW_OMEGA_SCALE);
}
```

Construction does not register behavior. The root later calls `bind(...)` exactly once for the
program's callback graph.

Here is the Starter's complete intake mapping:

```java
requiredCallbacks.onRise(
        driver.a(),
        () -> requiredIntake.setMode(StarterIntake.Mode.COLLECT));
requiredCallbacks.onRise(
        driver.b(),
        () -> requiredIntake.setMode(StarterIntake.Mode.EJECT));
requiredCallbacks.onRise(
        driver.x(),
        () -> requiredIntake.setMode(StarterIntake.Mode.STOPPED));
```

Trace the A button:

```text
FTC Gamepad.a: pressed
        |
        v
GamepadDevice.a(): BooleanSource true
        |
        v rising edge
CallbackBindings.onRise(...)
        |
        v
StarterIntake.setMode(COLLECT)
        |
        v
mechanism records a persistent semantic request
```

The binding says *what A means*. The capability says *what COLLECT means to the robot*. The
mechanism decides how hardware realizes COLLECT. Moving any of those decisions into another layer
would make the same fact have two owners.

## Buttons and triggers carry different kinds of intent

Keep these source meanings separate:

| Source | Type | What `true` or the value means |
| --- | --- | --- |
| `gamepad.a()` | `BooleanSource` | The FTC gamepad reports A pressed. |
| `gamepad.rightTrigger()` | `ScalarSource` | Calibrated trigger travel from `0.0` to `1.0`; it is not a Boolean button. |
| `gamepad.rightTrigger().above(0.2)` | `BooleanSource` | Trigger travel is above the program's semantic threshold. |

These are operator meanings, not statements about an electrical HIGH or LOW pin. Hardware digital
polarity, semantic switch meaning, and debounce are introduced together in
[`Evidence and experiments`](<Evidence and Experiments.md>).

## Callback bindings versus Task bindings

A callback is appropriate for a synchronous request such as replacing a held target. A Task
binding is appropriate for non-blocking behavior that unfolds over several loop cycles.

The Reference controls show both choices next to each other:

```java
callbacks.onRise(gamepad.dpadDown(),
        () -> lift.setHeight(ReferenceLift.Height.STOWED));
callbacks.onRise(gamepad.dpadLeft(),
        () -> lift.setHeight(ReferenceLift.Height.LOW));
callbacks.onRise(gamepad.dpadUp(),
        () -> lift.setHeight(ReferenceLift.Height.HIGH));
tasks.onRise(gamepad.x(), lift::home);

tasks.onRise(gamepad.y(), launcher::launchOne);
callbacks.onRise(gamepad.b(), launcher::abortLaunches);
```

`lift::home` and `launcher::launchOne` are factories. Each eligible rising edge calls the method to
obtain a **fresh, single-use Task** for the managed runner. The controls do not update that Task in
a private loop.

Use this decision rule:

```text
Does the action complete synchronously by replacing intent?
    yes -> program.callbackBindings()
    no  -> program.taskBindings() with a fresh Task supplier
```

Examples:

- `setHeight(HIGH)` replaces a persistent target, so it is a callback.
- `home()` searches over time and can time out, so it is a Task.
- `abortLaunches()` synchronously invalidates older launch requests and restores active-match
  launcher requests, so it is a callback.
- `launchOne()` spins up, feeds, and retracts over time, so it is a Task.

Declaration order is also observable policy. The Reference controls declare Y before B. If both
rise in one cycle, the fresh launch request is seen first and the later abort invalidates it, so B
wins without a priority API.

## Drive is intent plus one final sink

`GamepadDriveSource` combines three axes into a robot-centric `DriveSignal`. It obeys the Phoenix
robot frame: `+X` forward, `+Y` left, and counter-clockwise omega positive. `GamepadDevice.leftY()`
already converts FTC's stick sign so pushing the stick up is positive before the drive source maps
the axes.

The Starter adds one controls-owned meaning: while the right bumper is pressed, scale translation
and rotation. The source still produces the same robot-centric `DriveSignal` type.

```text
left stick + right stick + right bumper
                    |
                    v
        StarterTeleOpControls
                    |
                    v
              DriveSource
          robot-centric intent
                    |
                    v
        program.drive(source, sink)
                    |
                    v
       one final mecanum hardware write
```

The controls never call motor setters. `StarterRobot` pairs the source with the one mecanum sink,
and `RobotProgram` samples and realizes that pair in the managed output phase.

Robot-centric “forward” follows the robot. Driver-station or field “up” is a separate choice. The
optional [`Field-relative Drive`](<../../examples/Field-relative Drive.md>) example shows how to
author that direction and transform manual intent before the same normal mecanum sink; field-
relative behavior is not built into the Phoenix robot.

## Trace it

### 1. The team wants B to request COLLECT instead of EJECT. What changes?

**Answer:** the mapping in the controls owner. `StarterIntake.Mode.COLLECT` and its hardware
realization remain unchanged.

### 2. The team changes COLLECT from `0.20` to `0.30` power. What changes?

**Answer:** the intake configuration, after the team's physical review process. A control binding
should never contain that motor value.

### 3. Two autonomous routines also need COLLECT. Should either mention gamepad A?

**Answer:** no. They call `StarterIntake.setMode(COLLECT)` or request a fresh capability Task. The
gamepad mapping belongs only to TeleOp controls.

## Predict it

### The driver holds A for twenty loops. How many rising-edge callbacks run?

**Prediction:** one, when the source changes from false to true.

**Answer:** one. COLLECT persists because the callback replaced the mechanism's held request; the
binding does not need to repeat it every loop.

### The driver presses Y, releases it, and presses it again after the first launch completes. Is the same Task reused?

**Prediction:** no.

**Answer:** `tasks.onRise(gamepad.y(), launcher::launchOne)` calls the factory again. Task instances
are single-use, so repeatable behavior must return a fresh instance.

### Is a fully pressed gamepad trigger “digital HIGH”?

**Prediction:** no.

**Answer:** it is a scalar operator value near `1.0`. A program may turn it into a semantic Boolean
with a threshold, but that Boolean does not describe an electrical digital input pin.

## Copy, adapt, and leave behind

**Copy this structure:** create stable sources in a controls constructor, bind meanings once,
depend on capabilities, use callbacks for synchronous requests, use Task suppliers for behavior
over time, and expose a `DriveSource` instead of writing motors.

**Adapt these meanings:** buttons, axes, slow-mode scales, capability calls, trigger thresholds, and
which actions deserve Tasks.

**Do not copy:** a particular button layout as a universal standard, raw motor writes into controls,
one prebuilt Task reused by several presses, or field-relative behavior when the team has not chosen
and documented a field control frame.

**Previous:** [Robot roles](<Robot Roles.md>)

**Next:** [Plants and hardware](<Plants and Hardware.md>)

For complete source composition rules, see
[`Sources and signals`](<../../core-concepts/Sources and Signals.md>).
