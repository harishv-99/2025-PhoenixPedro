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

A digital switch input reports one of two electrical levels, **HIGH** or **LOW**. This wiring uses
LOW for pressed, called **active-low polarity**. LOW is not automatically Java `false`: our reader
maps LOW to `rawPressed=true`.

A physical switch's contacts can briefly flicker while being pressed or released. **Debouncing**
filters those brief changes before accepting a new state. `rawPressed` is the latest interpreted
reading; `pressed` is the filtered result.

A **source** is a retained way to obtain a value when asked. These sources share one electrical
observation. Each active loop reads both and saves their results in **status**:

Here `clock` reports the shared loop's time and cycle; these readers never advance it.

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/BasicSwitchService.java -->
```java
boolean rawPressed = rawPressedSource.getAsBoolean(clock);
boolean pressed = pressedSource.getAsBoolean(clock);
status = new Status(true, rawPressed, pressed);
```

`new Status(...)` creates that saved result. Its first argument, `true`, sets `observed`: a reading
has occurred. Saving the result is **caching**; displaying it does not read the switch again.

This software timeline uses the lesson's `0.02`-second press/release delays. Times are seconds after START:

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

![Raw pressed flickers at 0.010 seconds; accepted pressed changes only at 0.037 and 0.059.](<../assets/diagrams/debounce-samples.svg>)

The chart connects saved **software values**, not unseen electrical transitions. The table supplies
the same meaning in text.

Notice:

- A short observed LOW followed by HIGH does not become debounced `pressed`.
- Later LOW samples contribute two `0.011`-second loop intervals: `0.022` exceeds the `0.02` delay.
  Release uses the same rule.
- `observed=false` means no observation. The stored booleans then mean unknown, not physically released.

This debouncer counts each differing sample's preceding loop interval. One sample after a long
interval can satisfy the delay; this does not prove continuous physical stability. START's zero
elapsed interval cannot spend INIT time on debounce.

**First-pass checkpoint:** why is `rawPressed` false but `pressed` true at `0.048`? Continue the tour to
[one function per button press](<Continuous Intake.md#first-pass-run-a-function-once-per-press>).
Read below for the complete program; writing or running it is optional.

## Critical production idea

One owner reads the input and saves the result; every display uses that saved result. Read the
walkthrough in order to connect the small first-pass excerpt to the complete production program.

### Optional: author the switch owner in your package

Create `edu.ftcsushi.robots.myrobot.basicsensing` under both `TeamCode/src/main/java/` and
`TeamCode/src/test/java/`. These are matching package locations: robot code belongs in `main`,
and optional software experiments belong in `test`. Keep the class names below as you build, and
use `package edu.ftcsushi.robots.myrobot.basicsensing;` in each file. Import Sushi's `fw` types;
your robot must not import `edu.ftcsushi.robots.examples.*`.

An **object** groups saved values and the methods that use them; a **class** defines that object's
shape. `new BasicSwitchService(...)` constructs one object, running its constructor once. Keep that
same object across loops so its saved readings survive between method calls.

Start `BasicSwitchService` as a class implementing
[`RobotProgram.Service`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/RobotProgram.Service.html>).
A service is an object Sushi calls near the start of each active loop to refresh observations.
The Java `interface` names the methods Sushi can call; `implements RobotProgram.Service` promises
that this class supplies those methods. It owns the connected sources, status, and reset behavior.
There is no actuator to command.

### Choose the input and its meaning

Inside that class, the nested class `Config` holds only data: the FTC digital-channel name and two
delays in seconds. “Nested” means declared inside the owner class. Its constructor is `private`,
so callers cannot construct an incomplete configuration. Instead, `Config.defaults()` is a
**factory method**: it returns a fresh complete configuration. A `static` method belongs to the
class and can be called without first constructing an object of that class.
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

`private` keeps these fields inside the owner. `final` prevents assigning a different source to
that field after construction; it does not prevent the source from returning changing readings.

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
wraps that same source with the two sampled delays. Processing observations into a more useful
value is called **signal conditioning**. The adapter caches one successful read for the
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

START resets the source graph and samples once. Add this body to `start(LoopClock clock)`.
`Objects.requireNonNull(...)` rejects a missing clock with the displayed error message:

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

Create `BasicSwitchTeleOp extends FtcRobotOpMode`: `extends` reuses the framework's FTC lifecycle.
The annotations `@TeleOp` and `@Disabled` identify the OpMode and keep it off the selectable list
until a deliberate hardware review. `@Override` marks your implementation of an inherited method. Its
`configure(RobotProgram program)` runs once at INIT. It reads the single config edit point and
calls a package-private `declare(program, hardwareMap, config)` method containing the graph below.
Package-private means no `public` or `private` modifier: other classes in this same package can call
the declaration. That small declaration method is also used by the supplied software experiment, so both hosts
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
runtime, then calls the real managed `loop()` once. An **assertion** checks an expected observation:
`assertFalse(...)` expects false, `assertTrue(...)` expects true, and `assertEquals(expected, actual)`
compares two values. A mismatch fails the test. `rig.rows.get("switch.pressed")` retrieves the
recorded telemetry row by its name:

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
