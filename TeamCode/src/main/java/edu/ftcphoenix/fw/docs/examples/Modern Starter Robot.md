# Modern starter robot

Use this example as the smallest compiling Phoenix structure shared by TeleOp and Auto. It has one
direct mecanum drive, one intake mechanism, one mode-neutral intake capability, one controls owner,
and one declaration-only composition root. FTC lifecycle ceremony is supplied by
`FtcRobotOpMode` and its framework-created `RobotProgram`; robot meanings and hardware ownership
remain in the seven files under
[`edu.ftcphoenix.robots.examples.starter`](<../../../robots/examples/starter/>).

## Read the seven files

| File | Job | What a student normally edits |
|---|---|---|
| [`StarterProfile.java`](<../../../robots/examples/starter/StarterProfile.java>) | Data-only, defensively copied hardware/tuning configuration | Fill and physically review `current()`. |
| [`StarterIntake.java`](<../../../robots/examples/starter/StarterIntake.java>) | Shared mode-neutral capability vocabulary | Rename the capability and modes for the real robot. |
| [`StarterIntakeMechanism.java`](<../../../robots/examples/starter/StarterIntakeMechanism.java>) | Privately owns the intake Plant and implements `RobotProgram.Output` | Change hardware realization and safe stop behavior. |
| [`StarterTeleOpControls.java`](<../../../robots/examples/starter/StarterTeleOpControls.java>) | Declares button meanings and exposes the final manual `DriveSource` | Change driver meanings here. |
| [`StarterRobot.java`](<../../../robots/examples/starter/StarterRobot.java>) | Validates, constructs, and declares the mode-specific graph | Add real owners and their declaration order. |
| [`StarterTeleOp.java`](<../../../robots/examples/starter/StarterTeleOp.java>) | Disabled one-method FTC TeleOp | Normally change only the name/profile selection. |
| [`StarterAuto.java`](<../../../robots/examples/starter/StarterAuto.java>) | Disabled one-method Auto with one fresh root Task | Replace the routine expression. |

The split is intentional. `FtcRobotOpMode` is not a season robot superclass and knows no intake,
scoring, route, or strategy vocabulary. It owns only the reusable FTC callback/lifecycle contract.

## Configure before constructing hardware

`StarterProfile.current()` is deliberately not runnable as checked in. Before enabling either
OpMode, fill and review:

- drive motor names/directions for TeleOp,
- intake motor name/direction,
- distinct finite nonzero collect/eject powers in `[-1, +1]`, and
- `hardwareConfigurationReviewed = true`.

`StarterRobot.declareTeleOp(...)` validates the complete TeleOp profile before its first hardware
lookup. `declareAuto(...)` validates only the shared intake facts, so an unused drive configuration
does not block the tiny Auto. The acknowledgement records human review; it does not prove wiring,
polarity, traction, or safe physical motion.

## The entire FTC hosts

TeleOp has one ordinary override:

```java
public final class StarterTeleOp extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        new StarterRobot(hardwareMap, StarterProfile.current())
                .declareTeleOp(program, gamepad1);
    }
}
```

Auto is parallel:

```java
public final class StarterAuto extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        new StarterRobot(hardwareMap, StarterProfile.current())
                .declareAuto(program, 0.75);
    }
}
```

Neither file stores a clock, runner, lifecycle flags, robot field, or STOP method. The framework
retains the program before configuration begins, freezes declarations when the method returns, and
owns all later callbacks.

## Declaration-only composition

The TeleOp root constructs and immediately transfers each lifecycle owner:

```java
StarterIntakeMechanism intake = program.output(
        new StarterIntakeMechanism(hardwareMap, profile.intake));

StarterTeleOpControls controls = new StarterTeleOpControls(
        program.bindings(),
        new GamepadDevice(gamepad1),
        intake);

program.drive(
        controls.driveSource(),
        FtcDrives.mecanum(hardwareMap, profile.drive));

program.presenter((clock, telemetry) -> presentIntake(telemetry, intake));
```

Registration retains the same object immediately. If a later construction or declaration fails,
the program still stops every already registered sibling. The source-driven drive joins output
declaration order; it calls the sink heartbeat, samples the source, rejects non-finite components,
clamps finite components, writes once, and stops the sink during cleanup.

Auto declares the same capability realization plus one fresh root:

```java
StarterIntakeMechanism intake = program.output(
        new StarterIntakeMechanism(hardwareMap, profile.intake));
Task root = intake.collectForSeconds(collectDurationSec);
program.rootTask(root);
```

There is no one-member capability aggregate. `StarterIntake` is already the one cohesive family
both modes need.

## Mechanism and controls ownership

`StarterIntakeMechanism` receives `HardwareMap` plus a data-only config, defensively copies it,
constructs its final command-backed Plant, and implements the downstream role:

```java
final class StarterIntakeMechanism
        implements StarterIntake, RobotProgram.Output {
    @Override
    public void update(LoopClock clock) {
        plant.update(clock);
    }

    @Override
    public void stop() {
        // request zero, then stop the private Plant/output
    }
}
```

The completed-Plant constructor remains package-private and explicitly hardware-neutral for tests.
It is not a second ordinary construction path.

Controls receive only `BindingRegistrar`. They declare mappings but cannot update or clear the
program-owned graph:

```java
requiredBindings.onRise(driver.a(),
        () -> intake.setMode(StarterIntake.Mode.COLLECT));
requiredBindings.onRise(driver.b(),
        () -> intake.setMode(StarterIntake.Mode.EJECT));
requiredBindings.onRise(driver.x(),
        () -> intake.setMode(StarterIntake.Mode.STOPPED));
```

## Managed lifecycle and order

The framework applies:

```text
INIT       reset clock -> configure/freeze graph -> optional prestart -> presenters -> one commit
INIT loop  clock -> optional prestart -> presenters -> one commit
START      freeze prestart -> reset clock -> service starts -> root start/first update -> exact-start outputs
loop       clock -> services -> bindings -> Tasks -> outputs/drive -> presenters -> one commit
STOP       cancel Tasks -> clear bindings -> outputs in order -> services in reverse order
```

INIT never advances services, bindings, Tasks, drive, or mechanisms. An optional prestart owner is
data-only and may block START while presenters continue. Exact-start output realization
makes the Auto's positive-duration collect request observable even if the first regular loop is
late. On a lifecycle `RuntimeException`, cleanup follows the same terminal path, retains the exact
primary failure, and suppresses later cleanup failures. Repeated/reentrant STOP is inert; `Error`
is not caught.

The starter's TeleOp output order is intake then drive because the intake is registered immediately
after construction. That preserves failure cleanup without holding an unregistered hardware owner.
Robots that require a different downstream order should arrange construction/declaration so every
completed owner transfers immediately, or encapsulate tightly coupled outputs behind one truthful
owner rather than creating an unsafe registration gap.

## Adapting the example

For a new robot:

1. complete and physically review the profile;
2. rename the capability and modes to real robot meanings;
3. keep Plant construction/update/stop in each mechanism output;
4. pass `program.bindings()` into the controls owner;
5. let TeleOp and Auto call the same capabilities;
6. declare upstream computation as services and final realization as outputs; and
7. keep `configure(program)` declarative rather than adding FTC lifecycle methods.

The OpModes remain `@Disabled` until names, directions, power, clear space, wheel direction, and
immediate STOP behavior are checked on the actual robot. Software tests prove declaration order,
cancellation, and output-seam zero commands—not physical motion.

## Related reading

- [`../getting-started/Beginner's Guide.md`](<../getting-started/Beginner's Guide.md>)
- [`../design/Framework Lanes & Robot Controls.md`](<../design/Framework Lanes & Robot Controls.md>)
- [`../design/Robot Capabilities & Mode Clients.md`](<../design/Robot Capabilities & Mode Clients.md>)
- [`../core-concepts/Loop Structure.md`](<../core-concepts/Loop Structure.md>)
- [`../../Framework Principles.md`](<../../Framework Principles.md>)
