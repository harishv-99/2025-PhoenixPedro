# Robot roles

**Question this chapter answers:** Where does each part of robot code belong?

**Reading time:** about 15 minutes

Phoenix is easiest to learn as a set of owners. Start with the small Starter robot, then notice that
the larger Reference robot uses the same roles. You do not need either robot's hardware, and this
chapter does not ask you to enable or run an OpMode.

By the end, you should be able to look at a new requirement and choose its likely owner before
writing code.

One term appears in the source map before its full lesson: a **Plant** is the mechanism-owned object
that resolves one requested actuator target and performs the final hardware write. Chapter 3 traces
that path in detail.

## Source map

| Read | What it proves |
| --- | --- |
| [`StarterTeleOp.java`](<../../../../robots/examples/starter/opmode/StarterTeleOp.java>) | An ordinary FTC OpMode selects configuration and delegates one declaration. |
| [`StarterProfile.java`](<../../../../robots/examples/starter/robot/StarterProfile.java>) | Hardware names, directions, limits, and permissions are data. |
| [`StarterRobot.java`](<../../../../robots/examples/starter/robot/StarterRobot.java>) | The composition root constructs owners and declares their lifecycle roles. |
| [`StarterIntake.java`](<../../../../robots/examples/starter/capability/intake/StarterIntake.java>) | TeleOp and Auto share mode-neutral robot vocabulary. |
| [`StarterIntakeMechanism.java`](<../../../../robots/examples/starter/capability/intake/StarterIntakeMechanism.java>) | A mechanism owns its Plant, update, and terminal stop. |
| [`ReferenceRobot.java`](<../../../../robots/examples/reference/robot/ReferenceRobot.java>) | The same structure scales to multiple capability families. |
| [`ReferenceCapabilities.java`](<../../../../robots/examples/reference/robot/ReferenceCapabilities.java>) | A real multi-family Auto client receives one small mode-neutral handoff. |
| [`ReferenceAutoRoutines.java`](<../../../../robots/examples/reference/autonomous/ReferenceAutoRoutines.java>) | Autonomous strategy composes capabilities outside the composition root. |

## First trace: the smallest complete robot

The Starter TeleOp contains one framework callback. This is the entire method from the checked-in
source:

```java
@Override
protected void configure(RobotProgram program) {
    StarterProfile profile = StarterProfile.current();
    new StarterRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
}
```

The OpMode chooses the program's facts and mode. It does not own a clock, forward FTC lifecycle
callbacks, update hardware, or translate buttons. `FtcRobotOpMode` creates the `RobotProgram`, and
the program owns the managed lifecycle.

Follow the objects declared by `StarterRobot`:

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

Read that block as declarations, not as a loop:

```text
StarterTeleOp
    chooses StarterProfile.current()
              |
              v
StarterRobot composition root
    |-- output ------> StarterIntakeMechanism ------> private Plant
    |-- bindings ----> StarterTeleOpControls -------> StarterIntake capability
    |-- drive -------> DriveSource + mecanum sink
    `-- presenter ---> cached intake status

RobotProgram owns the one heartbeat and later advances those declared roles.
```

The important distinction is between **wiring** and **work**. `StarterRobot` wires the graph once.
The managed program performs recurring work in its defined loop phases.

## The roles in ordinary robot code

| Role | Owns | Does not own |
| --- | --- | --- |
| OpMode | Mode selection and one `configure(program)` entry | A private loop, clock, or STOP sequence |
| Profile/configuration | Data such as names, directions, bounds, and tuning | Hardware resources or runtime decisions |
| Composition root | Construction, relationships, and declaration order | Button meanings or a robot control script |
| Capability | Mode-neutral intent and status vocabulary | FTC device details |
| Controls | What operator inputs mean | Plant construction or autonomous strategy |
| Auto routine/strategy | Which fresh capability Tasks run, in what order, and what outcomes allow next | Hardware construction or final writes |
| Mechanism/output | Private Plants, final target resolution, update order, and stop | Gamepad meanings |
| Presenter | Formatting already-computed snapshots | Behavior decisions or telemetry commits |
| `RobotProgram` | One clock, lifecycle phases, bindings, Tasks, outputs, presentation, and cleanup | Season-specific robot meanings |

These are ownership boundaries, not a requirement to create the maximum number of classes. The
Starter has one cohesive `StarterIntake` capability, so it does not add a one-member aggregate just
for symmetry.

## What the profile means

`StarterProfile.current()` returns a fresh data graph. Its values are compiling examples, not facts
about an adopting robot:

```java
profile.intake = StarterIntakeMechanism.Config.defaults();
profile.intake.motorName = "intakeMotor";
profile.intake.direction = Direction.FORWARD;
profile.intake.collectPower = 0.20;
profile.intake.ejectPower = -0.20;
profile.allowIntakeMotion = false;

profile.drive = FtcDrives.MecanumConfig.defaults();
```

The profile says *what this robot is configured to use*. The mechanism that understands a slice
copies and validates that slice before retaining it. The composition root passes data to owners; it
does not build Plants for them.

For your own robot, configuration may have different fields, but keep the same separation:

```text
data about the robot                 behavior and ownership
--------------------                 ----------------------
motor name, direction, limits   -->  mechanism
button choices                  -->  controls code
task sequence                   -->  capability macro / Auto routine
update and stop order           -->  owning mechanism + RobotProgram
```

## How Reference scales the pattern

The Reference robot adds independent lift and launcher families. Its root still only constructs and
declares owners. It validates the active gamepad, permissions, and cross-owner motor-name
relationships before the first hardware lookup, then wires the graph:

```java
Gamepad requiredGamepad = Objects.requireNonNull(gamepad1, "gamepad1");
ReferenceLiftMechanism lift = program.output(
        new ReferenceLiftMechanism(hardwareMap, p.lift));
ReferenceLauncherMechanism launcher = program.output(
        new ReferenceLauncherMechanism(hardwareMap, p.launcher));
ReferenceTeleOpControls controls = new ReferenceTeleOpControls(
        new GamepadDevice(requiredGamepad));
controls.bind(program.callbackBindings(), program.taskBindings(), lift, launcher);
program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, p.drive));
program.presenter((clock, telemetry) -> present(telemetry, lift, launcher));
```

`ReferenceLiftMechanism` implements `ReferenceLift`; `ReferenceLauncherMechanism` implements
`ReferenceLauncher`. The controls depend on those capability interfaces, not on raw Plants. The
Reference Auto calls the same vocabulary, so moving from TeleOp to Auto does not create a second
mechanism API.

Reference Auto receives both families in a small `ReferenceCapabilities` handoff because its
routine needs both. That bundle is not a required registry pattern: Starter returns its one
`StarterIntake` capability directly rather than wrapping one family merely for symmetry. Add an
aggregate only when it makes a real multi-family client simpler.

The difference between Starter and Reference is scale:

```text
Starter:   controls --> StarterIntake --> one mechanism

Reference: controls/Auto --> ReferenceLift ----> lift mechanism
                        `-> ReferenceLauncher -> launcher mechanism

Same managed host, same owner rules, more cohesive capability families.
```

## Trace it

Try each question before opening the answer.

### 1. A team changes which button requests the LOW lift height. Which owner changes?

**Answer:** `ReferenceTeleOpControls`. LOW is already a mode-neutral lift meaning; only its operator
mapping changed. Neither the lift mechanism nor the composition root should learn which button was
chosen.

### 2. A team changes the lift motor name and its maximum travel. Which owner changes?

**Answer:** the lift configuration in the team profile. `ReferenceLiftMechanism` should continue to
copy, validate, and realize that data unless the hardware capability itself changes.

### 3. A mechanism needs two private Plants updated in a particular order. Where does that order live?

**Answer:** inside the mechanism/output that owns both Plants. Registering two peer writers in the
composition root would obscure final actuator ownership.

## Predict it

### If `configure(program)` returns, what calls `lift.update(clock)` every active loop?

**Prediction:** not the OpMode and not `ReferenceRobot`.

**Answer:** `RobotProgram` advances every declared output in its managed output phase. The Reference
root transferred the lift to that role with `program.output(...)`.

### If Auto and TeleOp both need `launchOne()`, should Auto call the launcher Plant directly?

**Prediction:** no.

**Answer:** both modes call the shared `ReferenceLauncher` capability. Plant details stay private to
the launcher mechanism.

## Copy, adapt, and leave behind

**Copy this structure:** thin managed OpModes, a data-only profile, a declaration-only composition
root, mode-neutral capability families, owner-local Plants, controls, and presenters.

**Adapt these meanings:** capability names, configuration fields, operator mappings, autonomous
routines, status evidence, and the set of owners your robot actually needs.

**Do not copy as physical facts:** checked-in hardware names, directions, powers, bounds, or motion
permissions. Also do not copy the Reference robot wholesale when your robot needs fewer owners.

**Back:** [Learn Phoenix](<../Beginner's Guide.md>)

**Next:** [Controls and intent](<Controls and Intent.md>)

For the complete example inventory, see
[`Framework components through examples`](<../../examples/Framework Components Through Examples.md>).
