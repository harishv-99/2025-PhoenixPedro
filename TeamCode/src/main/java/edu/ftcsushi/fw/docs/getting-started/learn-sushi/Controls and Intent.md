---
tags:
  - Learn
---

# Controls and intent { #intent }

**Question:** Which operator input requests which robot action?

This is a concept reference. The complete button path is taught in
[Continuous Intake](<../../build/Continuous Intake.md>); continuous stick values are taught in
[First Drive](<../../build/First Drive.md>). Reading either explanation requires no installation,
code edit, test run, or hardware.

!!! info "New concept: intent"

    **Intent** is the action the robot is being asked to perform, such as collect or stop. Controls
    translate operator input into that request. The mechanism decides how its hardware realizes it.

## Give a button a robot meaning

`GamepadDevice` adapts the FTC gamepad into reusable
[Sources](<../Framework Overview.md#source>). Its `a()` source means “A is pressed”; it is already
a true/false robot meaning, not an electrical pin level. A trigger is a **scalar**, one number, from `0.0` released to
`1.0` fully pressed. A comparison such as `rightTrigger().above(0.2)` derives a Boolean meaning
from that number.

Calling `setMode(...)` directly runs that setter now. The
[no-argument lambda](<../Framework Overview.md#saved-callback>) `() -> ...` packages
the call so `onRise(...)` can save it during configuration. It does not execute at registration.
An accepted released-to-pressed transition invokes it synchronously during a later bindings phase;
there is no background thread. `Mode` is an enum: a fixed set of names for requests.
The focused controls owner assigns the three intake buttons:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterIntakeControls.java -->
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

Holding or releasing A does not repeat the callback, and the named
`COLLECT` request persists until B selects `EJECT` or X selects `STOPPED`.

The owner is **package-private**, so only code in its package can name it; its
[Complete source: `StarterIntakeControls.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterIntakeControls.java>)
contains its constructor and one-time bind check. The
[intake Build lesson](<../../build/Continuous Intake.md>) shows construction, managed registration,
and the output that realizes the request.

## Choose the execution shape

| Need | Ordinary shape | When it runs |
| --- | --- | --- |
| Read current drive sticks | controls-owned `DriveSource` connected with `program.drive(...)` | every active output/drive phase |
| Replace a persistent request on a press | `program.callbackBindings().onRise(...)` | synchronously on an accepted rise |
| Begin work that takes several loops | `program.taskBindings().onRise(...)` with a fresh-Task factory | admitted by the managed Task runner |

A **method reference** such as `lift::home` is shorthand here for `() -> lift.home()`:
it saves the call to `home()`; when invoked, that method
must return a fresh Task. It does not save a reusable Task instance. The
[Task reference](<Tasks and Autonomous.md>) explains lifetime and cancellation.

Drive is sampled continuously rather than through callbacks on every cycle. The controls own axis
meanings; a single **sink**, the receiver of the drive request, owns the final drivetrain write. Holding a bumper to scale the current
drive request also belongs in that continuously sampled source. See the complete
[combined TeleOp](<../../build/Combine Drive and Intake.md>).

## Locate the change

Changing B from eject to collect changes the controls mapping. Changing collection power changes
mechanism configuration. Changing hardware realization belongs to the mechanism. The same
mode-neutral capability remains available to TeleOp and Auto.

**Key APIs:** [`GamepadDevice`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/input/GamepadDevice.html>)
supplies current gamepad readers;
[`CallbackBindings`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/input/binding/CallbackBindings.html>)
registers synchronous meanings;
[`TaskBindings`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/TaskBindings.html>)
registers fresh work.

For electrical polarity and debounce, read [one switch](<../../build/Read a Switch.md>).
For detailed sampling contracts, use [Sources and signals](<../../core-concepts/Sources and Signals.md>).
Return to [the concept index](<../Beginner's Guide.md>) for another question.
