# Modern starter robot

**Learning mode:** Architecture reference

**Audience:** Students moving from
[`Get your first robot driving`](<../getting-started/First Sushi Robot Code.md>) into the
[`Build a robot step by step`](<../getting-started/Build a Robot Step by Step.md>) course. Use
[`Sushi in one picture`](<../getting-started/Framework Overview.md>) only when you need lifecycle
context.
**Source entry:** [`StarterTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.java>)

Use this example as the smallest compiling Sushi structure shared by TeleOp and Auto. It has one
direct mecanum drive, one intake mechanism, one mode-neutral intake capability, one controls owner,
and one declaration-only composition root. FTC lifecycle ceremony is supplied by
`FtcRobotOpMode` and its framework-created `RobotProgram`; robot meanings and hardware ownership
remain in the focused owners mapped below.

**Buildable promise:** the critical excerpts on this page explain the ownership decisions; the
collapsed complete files below contain the package declarations, imports, enclosing types, and
configuration needed to recreate the maintained Starter without opening a Java source link.

Begin with [`Get your first robot driving`](<../getting-started/First Sushi Robot Code.md>) for the
one-file physical milestone. Copy this Starter into a team-owned package when
[`Build a robot step by step`](<../getting-started/Build a Robot Step by Step.md>) introduces the
first subsystem. Use
[`Test a mechanism without hardware`](<../getting-started/Test a Mechanism Without Hardware.md>) to
exercise the same production mechanism and Plant with a test-only hardware registry. Use
[`Choose a Sushi topic`](<../getting-started/Beginner's Guide.md>) when one ownership boundary
needs more explanation. This page remains the complete Starter source reference.

## Open one owner at a time

Do not read seven files before making a change. Start with the question in front of you, open its
owner, and return to this map only when the next boundary matters.

| Question in front of you | Open this source | Boundary it owns |
|---|---|---|
| What should a button or drive stick mean? | [`StarterTeleOpControls.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterTeleOpControls.java>) | Driver meanings and the final manual `DriveSource` |
| Which robot words should TeleOp and Auto share? | [`StarterIntake.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.java>) | Mode-neutral intake intent, fresh Tasks, and status |
| How does an intake mode become one final motor command? | [`StarterIntakeMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java>) | Private Plant construction, realization, update, and stop |
| Where do names, directions, powers, limits, and permissions belong? | [`StarterProfile.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterProfile.java>) | Fresh data-only intake/drive configuration and separate motion permissions |
| Who constructs and declares the active robot graph? | [`StarterRobot.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java>) | Robot-level checks, construction, declaration order, and presentation |
| What is the smallest ordinary FTC TeleOp host? | [`StarterTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.java>) | Disabled profile selection and TeleOp declaration |
| How does Auto compose timed behavior from the same capability? | [`StarterAuto.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.java>) | Disabled Auto strategy and one fresh root Task |

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

Before the intake-only Auto, replace and review the intake motor name/direction and finite nonzero
collect/eject powers in `[-1, +1]`, then set only `allowIntakeMotion = true`. Leave
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

### Critical code

TeleOp has one ordinary override:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.java -->
```java
@Override
protected void configure(RobotProgram program) {
    StarterProfile profile = StarterProfile.current();
    new StarterRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
}
```

Auto is parallel:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.java -->
```java
@Override
protected void configure(RobotProgram program) {
    StarterProfile profile = StarterProfile.current();
    StarterIntake intake = new StarterRobot(hardwareMap).declareAuto(program, profile);
    program.rootTask(intake.collectForSeconds(COLLECT_DURATION_SEC));
}
```

**What to notice**

- Both FTC hosts override only `configure(...)`; the framework owns every later callback.
- TeleOp declares bindings/drive, while Auto declares one fresh root Task from the same capability.
- Each host selects a fresh profile synchronously and stores no lifecycle state.

**Key APIs**

- `FtcRobotOpMode`: ordinary managed FTC host.
- `RobotProgram`: declaration surface for lifecycle owners and behavior.
- `StarterProfile.current()`: fresh data-only robot configuration.
- `RobotProgram.rootTask(...)`: declares one managed Auto root.

Neither file stores a clock, runner, lifecycle flags, robot field, profile field, or STOP method.
The declaration consumes the fresh profile synchronously; each constructed owner retains only its
own copied slice. The framework retains the program before configuration begins, freezes
declarations when the method returns, and owns all later callbacks.

## Declaration-only composition

### Critical code

The TeleOp root constructs and immediately transfers each lifecycle owner:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java -->
```java
StarterIntakeMechanism intake = program.output(
        new StarterIntakeMechanism(hardwareMap, activeProfile.intake));
StarterTeleOpControls controls = new StarterTeleOpControls(
        new GamepadDevice(requiredGamepad));
controls.bind(program.callbackBindings(), intake);

program.drive(
        controls.driveSource(),
        FtcDrives.mecanum(hardwareMap, activeProfile.drive));
program.presenter((clock, telemetry) -> presentIntake(telemetry, intake));
```

The import identifies `GamepadDevice` as the FTC input edge. It converts the SDK gamepad into
Sushi sources through the starter's ordinary direct construction.

Registration retains the same object immediately. A malformed drive slice can therefore be found
after the intake is registered and controls/bindings exist. If that later construction or
declaration throws a `RuntimeException`, managed cleanup clears the bindings, stops the registered
intake, and rethrows the exact primary failure; construction is not a transactional rollback of SDK
configuration effects. The source-driven drive joins output declaration order; it calls the sink
heartbeat, samples the source, rejects non-finite components, clamps finite components, writes once,
and stops the sink during cleanup.

The Auto declaration method constructs the same capability realization and returns the capability
to its mode client:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java -->
```java
StarterIntakeMechanism intake = program.output(
        new StarterIntakeMechanism(hardwareMap, activeProfile.intake));
program.presenter((clock, telemetry) -> presentIntake(telemetry, intake));
return intake;
```

**What to notice**

- The root wires owners and order; it does not contain TeleOp meanings or Auto strategy.
- Controls call the capability, and one drive sink owns final drivetrain writes.
- Auto receives the same mode-neutral capability rather than a parallel mechanism API.

**Key APIs**

- `RobotProgram.output(...)`: registers mechanism lifecycle immediately.
- `controls.bind(...)`: maps stable operator meanings to capability calls once.
- `RobotProgram.drive(...)`: connects one `DriveSource` to one final sink.
- `RobotProgram.presenter(...)`: observes snapshots without owning behavior.

`StarterAuto` then chooses and declares
`intake.collectForSeconds(COLLECT_DURATION_SEC)`. Strategy therefore stays with the Auto client
rather than inside the composition root. There is no one-member capability aggregate:
`StarterIntake` is already the one cohesive family both modes need.

## Mechanism and controls ownership

### Critical code

`StarterIntakeMechanism` receives `HardwareMap` plus a data-only config, defensively copies and
validates the complete snapshot before its first hardware lookup, constructs its final
command-backed Plant, and implements the downstream role. Its central rule stays small:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
@Override
public void setMode(Mode mode) {
    Mode requested = Objects.requireNonNull(mode, "mode");
    plant.commandTarget().set(powerFor(requested));
    requestedMode = requested;
}
```

The named mode is the request. The mechanism maps it forward to configured power and never tries to
recover it from a number. Collect and eject powers may therefore be equal without making status
ambiguous; each action power must still be finite, nonzero, and in `[-1, +1]`. Equal values are a
software-valid regression case, not a suggested physical intake configuration. The team must verify
that its chosen values produce the intended collect and eject actions on its hardware.

The same owner applies the downstream lifecycle:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
@Override
public Status status() {
    return new Status(requestedMode, plant.getAppliedTarget());
}

@Override
public void update(LoopClock clock) {
    plant.update(clock);
}

@Override
public void stop() {
    plant.stop();
}
```

The controls constructor creates stable input sources without changing the callback graph. Its
explicit, one-shot `bind(...)` call receives `CallbackBindings` first and the capability second. It
can declare mappings but cannot update or clear the program-owned graph:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterTeleOpControls.java -->
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

**What to notice**

- The mechanism maps semantic intent into its private Plant and publishes requested/applied facts separately.
- The same owner advances and terminally stops the Plant; Tasks and controls never write hardware directly.
- Controls register meanings once against the capability, not the concrete motor or Plant.

**Key APIs**

- `Plant.commandTarget()`: stable source-graph command seam owned by the mechanism.
- `RobotProgram.Output`: managed update/stop role for final realization.
- `CallbackBindings.onRise(...)`: synchronous semantic edge binding.
- `StarterIntake.Status`: immutable requested/applied evidence for presenters and tests.

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

The Starter mechanism scenario uses the ordinary production constructor and private Plant while a
test-only `HardwareMap` records the motor command. Its result is software evidence, not a substitute
for the supervised bring-up above.

## Files you will create

Create the seven files in the owner map at the top of this page. The two complete hosts and the
shared capability below establish the public shape; the complete mechanism, profile, controls, and
composition-root files follow the same package layout in the maintained working slice on this page.

## Complete working slice

<details>
<summary>Complete working slice: Starter TeleOp host</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.java -->
```java
package edu.ftcsushi.robots.examples.starter.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.robots.examples.starter.robot.StarterProfile;
import edu.ftcsushi.robots.examples.starter.robot.StarterRobot;

/** Thin declarative FTC host for the modern starter TeleOp reference. */
@TeleOp(name = "FW Starter: TeleOp", group = "FW Examples")
@Disabled
public final class StarterTeleOp extends FtcRobotOpMode {

    /** FTC construction path using the checked-in starter profile. */
    public StarterTeleOp() {
        // FTC constructs OpModes through their public no-argument constructor.
    }

    @Override
    protected void configure(RobotProgram program) {
        StarterProfile profile = StarterProfile.current();
        new StarterRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
    }
}
```

</details>

<details>
<summary>Complete working slice: Starter Auto host</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.java -->
```java
package edu.ftcsushi.robots.examples.starter.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;
import edu.ftcsushi.robots.examples.starter.robot.StarterProfile;
import edu.ftcsushi.robots.examples.starter.robot.StarterRobot;

/** Tiny declarative Auto that uses the same intake capability as the starter TeleOp. */
@Autonomous(name = "FW Starter: Auto", group = "FW Examples")
@Disabled
public final class StarterAuto extends FtcRobotOpMode {

    private static final double COLLECT_DURATION_SEC = 0.75;

    /** FTC construction path using the checked-in starter profile. */
    public StarterAuto() {
        // FTC constructs OpModes through their public no-argument constructor.
    }

    @Override
    protected void configure(RobotProgram program) {
        StarterProfile profile = StarterProfile.current();
        StarterIntake intake = new StarterRobot(hardwareMap).declareAuto(program, profile);
        program.rootTask(intake.collectForSeconds(COLLECT_DURATION_SEC));
    }
}
```

</details>

<details>
<summary>Complete working slice: shared intake capability</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.java -->
```java
package edu.ftcsushi.robots.examples.starter.capability.intake;

import edu.ftcsushi.fw.task.Task;

/** Mode-neutral intake capability shared by the starter TeleOp and Auto. */
public interface StarterIntake {

    /** Semantic intake requests; callers do not need to know or distinguish motor power values. */
    enum Mode {
        STOPPED,
        COLLECT,
        EJECT
    }

    /**
     * Small immutable status snapshot for telemetry and higher-level decisions.
     * The mode is the held semantic request; it is not reconstructed from motor power. The applied
     * target is the Plant's cached final target after guards, not hardware readback.
     */
    final class Status {
        private final Mode mode;
        private final double appliedTargetPower;

        public Status(Mode mode, double appliedTargetPower) {
            this.mode = mode;
            this.appliedTargetPower = appliedTargetPower;
        }

        /** Returns the held semantic request, independent of its configured numeric realization. */
        public Mode mode() {
            return mode;
        }

        /** Returns the Plant's cached applied target, not measured motor motion. */
        public double appliedTargetPower() {
            return appliedTargetPower;
        }

        @Override
        public String toString() {
            return "Status{mode=" + mode
                    + ", appliedTargetPower=" + appliedTargetPower + '}';
        }
    }

    /** Replaces the held semantic request; the mechanism maps it to configured motor power. */
    void setMode(Mode mode);

    /**
     * Creates a fresh single-use Task that requests collect for the requested duration, then
     * requests stopped. Active cancellation also restores the stopped semantic request.
     *
     * @param durationSec finite duration greater than zero, in seconds
     * @throws IllegalArgumentException if {@code durationSec} is non-finite or not greater than zero
     */
    Task collectForSeconds(double durationSec);

    /** Returns the held semantic request and the Plant's cached applied target. */
    Status status();
}
```

</details>

## Verify the slice

Run the exact checkpoint from the repository root:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac `
  :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.*
```

Expected checkpoint: compilation succeeds and every Starter-focused test is green. The files stay
`@Disabled`; software success does not authorize motion.

## Related reading

- [`Sushi Cheat Sheet`](<../reference/Sushi Cheat Sheet.md>)
- [`Common Problems`](<../troubleshooting/Common Problems.md>)
- [`Choose a Sushi topic`](<../getting-started/Beginner's Guide.md>)
- [`Test a mechanism without hardware`](<../getting-started/Test a Mechanism Without Hardware.md>)
- [`Architecture Roles, Framework Lanes, and Robot Controls`](<../design/Framework Lanes & Robot Controls.md>)
- [`Robot Capabilities and Mode Clients`](<../design/Robot Capabilities & Mode Clients.md>)
- [`Loop Structure`](<../core-concepts/Loop Structure.md>)
- [`Framework Principles`](<../../Framework Principles.md>)
