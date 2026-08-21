# Modern starter robot

**Classification:** Copyable starter

**Audience:** Students following the [`Learn Phoenix`](<../getting-started/Beginner's Guide.md>)
common path
**Source entry:** [`StarterTeleOp.java`](<../../../robots/examples/starter/opmode/StarterTeleOp.java>)

Use this example as the smallest compiling Phoenix structure shared by TeleOp and Auto. It has one
direct mecanum drive, one intake mechanism, one mode-neutral intake capability, one controls owner,
and one declaration-only composition root. FTC lifecycle ceremony is supplied by
`FtcRobotOpMode` and its framework-created `RobotProgram`; robot meanings and hardware ownership
remain in the seven source files listed below.

For a progressive source explanation, begin with
[`Learn Phoenix`](<../getting-started/Beginner's Guide.md>). This page remains the complete Starter
source reference; project setup is separate in
[`Build and run Phoenix`](<../getting-started/Build and Run.md>).

## Read the seven files

| File | Job | What a student normally edits |
|---|---|---|
| [`StarterProfile.java`](<../../../robots/examples/starter/robot/StarterProfile.java>) | Fresh four-field intake/drive configuration and separate motion permissions | Replace and review the example facts in `current()`, then permit only the motion being tested. |
| [`StarterIntake.java`](<../../../robots/examples/starter/capability/intake/StarterIntake.java>) | Shared mode-neutral capability vocabulary | Rename the capability and modes for the real robot. |
| [`StarterIntakeMechanism.java`](<../../../robots/examples/starter/capability/intake/StarterIntakeMechanism.java>) | Privately owns the intake Plant and implements `RobotProgram.Output` | Change hardware realization and safe stop behavior. |
| [`StarterTeleOpControls.java`](<../../../robots/examples/starter/robot/StarterTeleOpControls.java>) | Declares button meanings and exposes the final manual `DriveSource` | Change driver meanings here. |
| [`StarterRobot.java`](<../../../robots/examples/starter/robot/StarterRobot.java>) | Checks robot-level permissions/relationships, then constructs and declares the mode-specific graph | Add real owners and their declaration order. |
| [`StarterTeleOp.java`](<../../../robots/examples/starter/opmode/StarterTeleOp.java>) | Disabled one-method FTC TeleOp | Normally change only the name/profile selection. |
| [`StarterAuto.java`](<../../../robots/examples/starter/opmode/StarterAuto.java>) | Disabled one-method Auto with one fresh root Task | Replace the routine expression. |

The split is intentional. `FtcRobotOpMode` is not a season robot superclass and knows no intake,
scoring, route, or strategy vocabulary. It owns only the reusable FTC callback/lifecycle contract.

## Configure only the active slices

Every `StarterProfile.current()` call returns a fresh, complete, software-valid graph with exactly
four top-level fields: `intake`, `allowIntakeMotion`, `drive`, and `allowDriveMotion`. The checked-in
names, directions, powers, brake choice, and conservative drive scales are examples, not facts about
your robot. Both permissions are false, so the examples cannot run until a human reviews the motion
being tested.

The baseline intake is `"intakeMotor"`, `FORWARD`, `+0.20` collect, and `-0.20` eject. The drive uses
`"frontLeftMotor"`, `"frontRightMotor"`, `"backLeftMotor"`, and `"backRightMotor"`, directions
F/R/F/R, `enableZeroPowerBrake = true`, and axial/lateral/omega scales `0.25/0.25/0.20`.

Before the intake-only Auto, replace and review the intake motor name/direction and distinct finite
nonzero collect/eject powers in `[-1, +1]`, then set only `allowIntakeMotion = true`. Leave
`allowDriveMotion = false`; Auto never reads the inactive drive slice or its permission.

Before TeleOp, also replace and review all four drive names/directions, the explicit
`enableZeroPowerBrake` choice, and conservative `maxAxial`, `maxLateral`, and `maxOmega` scales. Then
set `allowDriveMotion = true`; TeleOp requires both permissions. Clear the corresponding permission
again whenever you edit either slice.

The root rejects missing mode permissions and TeleOp's trimmed intake-versus-drive name collision
before any lookup. Each active hardware owner then copies and validates only its own configuration
before that owner's lookup. These checks prove software shape and explicit human acknowledgement;
they do not prove FTC device identity, wiring, polarity, braking behavior, traction, safe power, or
physical motion.

## The entire FTC hosts

TeleOp has one ordinary override:

```java
public final class StarterTeleOp extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        StarterProfile profile = StarterProfile.current();
        new StarterRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
    }
}
```

Auto is parallel:

```java
public final class StarterAuto extends FtcRobotOpMode {
    private static final double COLLECT_DURATION_SEC = 0.75;

    @Override
    protected void configure(RobotProgram program) {
        StarterProfile profile = StarterProfile.current();
        StarterIntake intake =
                new StarterRobot(hardwareMap).declareAuto(program, profile);
        program.rootTask(intake.collectForSeconds(COLLECT_DURATION_SEC));
    }
}
```

Neither file stores a clock, runner, lifecycle flags, robot field, profile field, or STOP method.
The declaration consumes the fresh profile synchronously; each constructed owner retains only its
own copied slice. The framework retains the program before configuration begins, freezes
declarations when the method returns, and owns all later callbacks.

## Declaration-only composition

The TeleOp root constructs and immediately transfers each lifecycle owner:

```java
import edu.ftcphoenix.fw.ftc.input.GamepadDevice;

StarterIntakeMechanism intake = program.output(
        new StarterIntakeMechanism(hardwareMap, profile.intake));

StarterTeleOpControls controls =
        new StarterTeleOpControls(new GamepadDevice(gamepad1));
controls.bind(program.callbackBindings(), intake);

program.drive(
        controls.driveSource(),
        FtcDrives.mecanum(hardwareMap, profile.drive));

program.presenter((clock, telemetry) -> presentIntake(telemetry, intake));
```

The import identifies `GamepadDevice` as the FTC input edge. It converts the SDK gamepad into
Phoenix sources through the starter's ordinary direct construction.

Registration retains the same object immediately. A malformed drive slice can therefore be found
after the intake is registered and controls/bindings exist. If that later construction or
declaration throws a `RuntimeException`, managed cleanup clears the bindings, stops the registered
intake, and rethrows the exact primary failure; construction is not a transactional rollback of SDK
configuration effects. The source-driven drive joins output declaration order; it calls the sink
heartbeat, samples the source, rejects non-finite components, clamps finite components, writes once,
and stops the sink during cleanup.

The Auto declaration method constructs the same capability realization and returns the capability
to its mode client:

```java
StarterIntakeMechanism intake = program.output(
        new StarterIntakeMechanism(hardwareMap, profile.intake));
program.presenter((clock, telemetry) -> presentIntake(telemetry, intake));
return intake;
```

`StarterAuto` then chooses and declares
`intake.collectForSeconds(COLLECT_DURATION_SEC)`. Strategy therefore stays with the Auto client
rather than inside the composition root. There is no one-member capability aggregate:
`StarterIntake` is already the one cohesive family both modes need.

## Mechanism and controls ownership

`StarterIntakeMechanism` receives `HardwareMap` plus a data-only config, defensively copies and
validates the complete snapshot before its first hardware lookup, constructs its final
command-backed Plant, and implements the downstream role:

```java
public final class StarterIntakeMechanism
        implements StarterIntake, RobotProgram.Output {
    @Override
    public void update(LoopClock clock) {
        plant.update(clock);
    }

    @Override
    public void stop() {
        plant.stop();
    }
}
```

For advanced hardware-neutral tests only, one package-private constructor accepts a completed Plant
alone; ordinary robot code has only the `HardwareMap`-plus-`Config` path.

The controls constructor creates stable input sources without changing the callback graph. Its
explicit, one-shot `bind(...)` call receives `CallbackBindings` first and the capability second. It
can declare mappings but cannot update or clear the program-owned graph:

```java
requiredCallbacks.onRise(driver.a(),
        () -> intake.setMode(StarterIntake.Mode.COLLECT));
requiredCallbacks.onRise(driver.b(),
        () -> intake.setMode(StarterIntake.Mode.EJECT));
requiredCallbacks.onRise(driver.x(),
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

1. replace and review each active configuration slice, including the drive brake choice, then set
   only that slice's `allow...Motion` permission;
2. rename the capability and modes to real robot meanings;
3. keep Plant construction/update/stop in each mechanism output;
4. construct the controls from inputs, then bind them once with
   `controls.bind(program.callbackBindings(), capability)`;
5. let TeleOp and Auto call the same capabilities;
6. declare upstream computation as services and final realization as outputs; and
7. keep `configure(program)` declarative rather than adding FTC lifecycle methods.

The OpModes remain `@Disabled` until names, intended directions and power signs, the brake choice,
reduced first-test limits, clear space, and an operator-on-STOP plan are reviewed. The first enabled
test is supervised with wheels or mechanisms safely unloaded; it verifies physical direction,
braking response, and immediate STOP before the robot is lowered or limits are increased. Release
sticks and triggers before INIT so `GamepadDevice` calibrates their physical neutral positions.
Software tests prove declaration order, cancellation, configuration domains, and output-seam zero
commands—not physical motion or that the review happened.

## Related reading

- [`Phoenix Cheat Sheet`](<../reference/Phoenix Cheat Sheet.md>)
- [`Common Problems`](<../troubleshooting/Common Problems.md>)
- [`Learn Phoenix`](<../getting-started/Beginner's Guide.md>)
- [`Architecture Roles, Framework Lanes, and Robot Controls`](<../design/Framework Lanes & Robot Controls.md>)
- [`Robot Capabilities and Mode Clients`](<../design/Robot Capabilities & Mode Clients.md>)
- [`Loop Structure`](<../core-concepts/Loop Structure.md>)
- [`Framework Principles`](<../../Framework Principles.md>)
