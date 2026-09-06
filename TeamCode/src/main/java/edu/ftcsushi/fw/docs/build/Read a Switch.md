---
tags:
  - Build
---

# Read one switch without moving anything

**Outcome:** explain how one electrical switch reading becomes a named `pressed` fact and appears
in Driver Station telemetry. Complete the lesson by predicting the observations below; optionally
build that same path in your own robot package afterward.

**Prerequisites:** basic Java booleans and methods and the
[FTC-loop bridge](<../getting-started/Framework Overview.md>). Reading the complete lesson needs no
hardware, project setup, or test runner. [Project setup](<../getting-started/Build and Run.md>) is
needed only for the optional authoring or software run. The optional physical check uses one
reviewed active-low digital switch; this fixture constructs no motors or servos.

## First pass: observations every loop

Imagine asking “is the switch pressed?” once in each active FTC loop. The maintained owner does
three things in that loop: read the chosen meaning, condition it, and save the resulting facts:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchService.java -->
```java
boolean rawPressed = rawPressedSource.getAsBoolean(clock);
boolean pressed = pressedSource.getAsBoolean(clock);
status = new Status(true, rawPressed, pressed);
```

A **source** is a retained way to obtain a value when asked. These two sources share the same
electrical observation: the first returns the LOW-as-pressed meaning, and the second applies
**debounce**, which delays changes using the observations and elapsed loop intervals. `status`
stores the last complete result so displaying it does not read the switch again.

Read this authored software timeline. Its `0.02`-second debounce delays are the actual lesson
configuration, and its times are seconds after START:

| When | Authored electrical input | `observed` | `rawPressed` | `pressed` |
| --- | --- | --- | --- | --- |
| INIT | HIGH, not sampled | false | unknown | unknown |
| START, 0.000 | HIGH | true | false | false |
| Loop, 0.010 | LOW | true | true | false |
| Loop, 0.015 | HIGH | true | false | false |
| Loop, 0.026 | LOW | true | true | false |
| Loop, 0.037 | LOW | true | true | true |
| Loop, 0.048 | HIGH | true | false | true |
| Loop, 0.059 | HIGH | true | false | false |

Notice:

- A short observed LOW followed by HIGH does not become debounced `pressed`.
- On the later LOW samples, two `0.011`-second loop intervals contribute `0.022` seconds; that
  exceeds the configured `0.02` delay. Release uses the same sampled rule.
- `observed=false` means there is no current observation. The two stored false values then mean
  unknown, not proof that a physical switch is released.

Sushi observes the input at loop boundaries; it cannot see transitions between samples. This
debouncer accumulates the elapsed intervals of samples that differ from its current state. The
table describes the expected software interpretation of authored levels, not the real switch's voltage or
continuous physical behavior. START has zero elapsed time, so it cannot spend the time from INIT
on debounce.

**First-pass checkpoint:** at `0.048`, explain why `rawPressed` is false while `pressed` is still
true. No code or test run is required. On the initial tour, continue to
[one function per button press](<Continuous Intake.md#first-pass-run-a-function-once-per-press>).
Continue below to understand the complete switch program; writing it is optional.

## Critical production idea

One owner reads the input and saves the result; every display uses that saved result. Read the
walkthrough in order to connect the small first-pass excerpt to the complete production program.

### Optional: author the switch owner in your package

Create `edu.ftcsushi.robots.myrobot.basicsensing` under both `TeamCode/src/main/java/` and
`TeamCode/src/test/java/`. These are matching package locations: robot code belongs in `main`,
and optional software experiments belong in `test`. Keep the class names below as you build, and
use `package edu.ftcsushi.robots.myrobot.basicsensing;` in each file. Import Sushi's `fw` types;
your robot must not import `edu.ftcsushi.robots.examples.*`.

Start `BasicSwitchService` as a class implementing
[`RobotProgram.Service`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/RobotProgram.Service.html>).
A service is an object Sushi calls near the start of each active loop to refresh observations.
It owns this switch's source graph, status, and reset behavior. There is no actuator to command.

### Choose the input and its meaning

Inside that class, `Config` holds only data: the FTC digital-channel name and two delays in
seconds. Its constructor is private; `Config.defaults()` returns a fresh complete configuration.
These are the assignments to edit in your copy:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchService.java -->
```java
Config config = new Config();
config.switchName = "lessonSwitch";
config.pressedDebounceSec = 0.02;
config.releasedDebounceSec = 0.02;
return config;
```

The owner constructor takes `HardwareMap` and that `Config`. It copies a nonblank trimmed name
and finite, nonnegative delays into local values before hardware lookup. Editing the original
configuration afterward does not reconfigure a running owner. Keep validation and these two
retained source fields inside the owner; do not pass independently constructed sources from the
OpMode:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchService.java -->
```java
private final BooleanSource rawPressedSource;
private final BooleanSource pressedSource;
private Status status = NOT_OBSERVED;
```

After the constructor validates its local `switchName`, `pressedDelay`, and `releasedDelay`, it
constructs the graph once:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchService.java -->
```java
rawPressedSource = FtcSensors.digitalLow(map, switchName);
pressedSource = rawPressedSource.debouncedOnOff(pressedDelay, releasedDelay);
```

[`FtcSensors.digitalLow(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcSensors.html>)
resolves that FTC device, configures it as an input, and returns a source. Construction does not
sample its electrical level. The adapter makes `true` mean LOW; we call that meaning `rawPressed`
because this fixture explicitly assumes reviewed active-low switch wiring. It does not discover
the wiring. `rawPressed` is already a semantic boolean, not the FTC pin's HIGH value.

[`BooleanSource.debouncedOnOff(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/core/source/BooleanSource.html>)
wraps that same source with the two sampled delays. The adapter caches one successful read for the
shared loop cycle, so asking both sources in `update(clock)` does not cause two hardware reads.

### Save one complete result

Add a nested `Status` class with public final booleans `observed`, `rawPressed`, and `pressed`.
Its private constructor assigns those three values. Final fields make a saved status immutable:
the next loop publishes a new object instead of changing an object that another reader retained.

The initial and stopped value is explicit:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchService.java -->
```java
private static final Status NOT_OBSERVED = new Status(false, false, false);
```

Write `status()` to return the retained `status` field. Write `update(LoopClock clock)` with the
three-line first-pass excerpt: it samples both sources before assigning the new status. A read
failure therefore cannot publish a partial result. `clock` is the shared loop's time and cycle
identity; the owner reads it but never advances another clock.

START resets the source graph and samples once. Add this body to `start(LoopClock clock)`:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchService.java -->
```java
Objects.requireNonNull(clock, "clock is required");
pressedSource.reset();
status = NOT_OBSERVED;
update(clock);
```

Resetting the conditioned source also resets its raw input source. The initial debounced value
is false; START's zero elapsed interval leaves it false even if the first raw reading is pressed.
On STOP, clear the evidence and reset the same graph without sampling:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchService.java -->
```java
status = NOT_OBSERVED;
pressedSource.reset();
```

This cleanup works even if STOP occurs before START. The managed host owns the terminal boundary
and prevents later loops from calling the service again.

## Wire the loop and save the display function

Create `BasicSwitchTeleOp extends FtcRobotOpMode`, annotated `@TeleOp` and `@Disabled`. Its
`configure(RobotProgram program)` runs once at INIT. It reads the single config edit point and
calls a package-private `declare(program, hardwareMap, config)` method containing the graph below.
That small declaration method is also used by the supplied software experiment, so both hosts
exercise the same production wiring.

First, register the owner:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchTeleOp.java -->
```java
BasicSwitchService limitSwitch = program.service(
        new BasicSwitchService(hardwareMap, config));
```

`new BasicSwitchService(...)` constructs the owner now. `program.service(...)` retains it for
START, later active-loop updates, and STOP; registration does not call `update` now.

Next, save a display function. Calling `limitSwitch.status()` by itself reads the saved value
immediately. Putting that call inside `(clock, telemetry) -> { ... }` creates a Java **lambda**:
a function whose two arguments will be supplied when Sushi invokes it later. Registration saves
the function and does not execute its body. A **presenter** is this read-only display function;
it runs synchronously in the current FTC callback, not on a new thread:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchTeleOp.java -->
```java
program.presenter((clock, telemetry) -> {
    BasicSwitchService.Status status = limitSwitch.status();
    telemetry.addData("switch.observed", status.observed);
    telemetry.addData("switch.rawPressed", status.rawPressed);
    telemetry.addData("switch.pressed", status.pressed);
});
```

Finish `declare(...)` by returning `limitSwitch`. In an active loop Sushi updates the service
first, then the presenter reads its complete saved status, and Sushi commits telemetry once. INIT
also presents, but the service has not sampled: `switch.observed` is false. START samples the
service; the next active loop presents its refreshed facts. STOP or an input exception ends the
managed lifetime and clears the owner's status. You write neither an FTC loop nor a telemetry
commit in these two robot classes.

## Files in this checkpoint

The maintained files are the completed answer; use them for imports and validation details after
authoring the active graph above. In your copy, the exact main files are
`TeamCode/src/main/java/edu/ftcsushi/robots/myrobot/basicsensing/BasicSwitchService.java` and
`TeamCode/src/main/java/edu/ftcsushi/robots/myrobot/basicsensing/BasicSwitchTeleOp.java`.

**Main:**

- [`BasicSwitchService` API](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchService.html>) — input owner, configuration, and immutable status.
  [Complete source: `BasicSwitchService.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchService.java>)
- [`BasicSwitchTeleOp` API](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchTeleOp.html>) — disabled managed host and presenter.
  [Complete source: `BasicSwitchTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchTeleOp.java>)

The optional test files use the matching
`TeamCode/src/test/java/edu/ftcsushi/robots/myrobot/basicsensing/` directory and the same package:

**Test:**

- [Complete source: `BasicSwitchSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchSoftwareScenarioTest.java>) — two small experiments to read or adapt.
- [Complete source: `BasicSwitchTestRig.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchTestRig.java>) — supplied input, runtime, and telemetry plumbing; you do not need to write its proxy implementation to understand this lesson.
- [Complete source: `BasicSwitchServiceTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchServiceTest.java>) — supplied maintainer regression coverage.

When adapting the optional tests, copy their package-local support into your matching test package
and change package declarations together. Their Sushi test-helper imports remain `fw.testing`;
the tests should exercise your owner and your `declare(...)`, not import the maintained example.

## Software checkpoint: author the outside world

- **Question:** do the authored LOW/HIGH samples produce the debounced status in the timeline?
- **Keep real:** the maintained configuration, owner, source graph, declaration, and managed loop.
- **Replace:** only the FTC digital input, runtime readings, and telemetry destination with
  recording software boundaries.
- **Observe:** the cached raw/conditioned booleans and displayed status.
- **Cannot conclude:** wiring, voltage, placement, real switch bounce, or physical detection.

The supplied rig starts the real managed graph. After the earlier brief LOW/HIGH bounce, these
two explicit input observations explain the transition; `observeAt(...)` sets the input and
runtime, then calls the real managed `loop()` once:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchSoftwareScenarioTest.java -->
```java
// HEARTBEAT: two LOW samples contribute 0.022 seconds to the 0.02-second debounce.
rig.observeAt(0.026, false);
assertFalse(rig.status().pressed);
rig.observeAt(0.037, false);
assertTrue(rig.status().pressed);
assertEquals(true, rig.rows.get("switch.pressed"));
```

**Read the causal chain:** the authored `false` electrical HIGH flag means LOW. The real adapter
maps it to `rawPressed=true`; the sampled delay becomes sufficient on the second loop; the owner
publishes `pressed=true`; the managed presenter prints that same fact. The assertion checks a
recorded observation, not a simulated physical press.

**Proves:** software polarity, sampled debounce, and the service-to-presenter path for these inputs.
**Does not prove:** the switch circuit has the assumed polarity or detects an intended object.
The second small experiment separately checks unobserved INIT and terminal STOP.

**Expected observations:** after the two later LOW samples, both cached `pressed` and the
`switch.pressed` telemetry row are true. After one HIGH sample, `rawPressed` is false while
`pressed` remains true; the next HIGH sample clears it. INIT and STOP leave the status unobserved.

**Reading checkpoint:** predict those rows, explain why displaying them performs no extra input
read, and distinguish a software LOW level from verified physical wiring. That completes the
lesson without running or writing code.

Optionally run the maintained experiment from the repository root:

=== "Windows"

    ```powershell
    .\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicsensing.BasicSwitchSoftwareScenarioTest
    ```

=== "macOS"

    ```bash
    ./gradlew --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicsensing.BasicSwitchSoftwareScenarioTest
    ```

Those commands target the maintained answer. After adapting the tests, substitute
`edu.ftcsushi.robots.myrobot.basicsensing.BasicSwitchSoftwareScenarioTest` as the test class to
check your own implementation. A passing test establishes the software claims above.

## Isolated hardware gate

This physical check is optional and follows the completed reading lesson.

**Next gate:** with robot power off, review the exact input device, supported voltage, wiring,
configuration name, and the assumption that LOW means pressed. Keep all mechanisms inactive;
this OpMode only owns the switch. Edit the assignments in your `Config.defaults()` and confirm
that the FTC Robot Configuration uses that digital-channel name. If that review cannot establish
compatible wiring and polarity, this physical check is blocked until it can.

After that review, enable only your switch OpMode by removing its `@Disabled` annotation for the
supervised check. INIT should show `switch.observed=false`. After START, watch the rows while
manually pressing, holding, and releasing the switch. Confirm the raw and conditioned meanings
against what you actually observe; Driver Station refresh speed may hide a 20 ms delay. Use STOP
to end sampling, and power off before altering wiring. Record the real name, polarity, and
behavior separately from the software timeline. This check does not establish a game-piece
detector or a motion interlock.

Continue to [requesting one named action from a button](<Continuous Intake.md>). You will keep
the distinction between a value read every loop and a function invoked when an event occurs.
