# Your first Phoenix robot code

## Goal

Copy the compiled Starter into a team-owned package, change COLLECT from the A button to Y, and
prove the new meaning with one hardware-free test.

**Time:** 45–60 minutes.

**Prerequisite:** Finish [Build and run Phoenix](<Build and Run.md>) and begin with its compile and
software-test checkpoints green.

**Safety boundary:** This lesson is software-only. It needs no Robot Controller or matching robot.
Keep both copied OpModes `@Disabled`, and keep `allowIntakeMotion` and `allowDriveMotion` false.

## 1. Copy the Starter into your package

Choose one lowercase package name for the robot. The steps below use `myrobot`; replace it with the
team's name everywhere.

In Android Studio's Project view, find
[`edu.ftcphoenix.robots.examples.starter`](<../../../robots/examples/starter/opmode/StarterTeleOp.java>).
Select the `starter` package and choose **Refactor > Copy** (`F5`). Set **New name** to `myrobot`
and the destination directory to `TeamCode/src/main/java/edu/ftcphoenix/robots`. The result must be
`edu.ftcphoenix.robots.myrobot`. Let Android Studio update package declarations and imports. Leave
the seven class names unchanged for now; rename them only when the real robot has better vocabulary.

The focused test lives under a different source root, so copy it separately. Copy
[`StarterFirstLessonTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcphoenix/robots/examples/starter/robot/StarterFirstLessonTest.java>)
with **Refactor > Copy**. Keep its name, select the `TeamCode/src/test/java` destination source
root, and set **Destination package** to `edu.ftcphoenix.robots.myrobot.robot`. Its `StarterIntake`
import must point to the copied capability. Keeping the test beside the package-private controls
owner is intentional. You only edit the indicated gamepad field later; the rest is provided test
scaffolding.

Use **Find in Files** over both copied packages for
`edu.ftcphoenix.robots.examples.starter`; a correct copy has no matches.

Before continuing, confirm the copied `StarterTeleOp.java` and `StarterAuto.java` still contain
`@Disabled`. Confirm the copied `StarterProfile.current()` still sets both motion permissions to
false.

## 2. Establish a green copied checkpoint

Compile before reading all of the copied files:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac
```

Then run only the copied lesson test:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcphoenix.robots.myrobot.robot.StarterFirstLessonTest
```

Both commands should end with `BUILD SUCCESSFUL`. This proves the package refactor is coherent and
the original A-button meaning is green before you change it. If either command fails, correct the
copied package or test; do not edit the canonical example.

## 3. Change one operator meaning, then prove it

Open only the copied `robot/StarterTeleOpControls.java`. In `bind(...)`, change `driver.a()` to
`driver.y()` for COLLECT. The final registration is:

```java
requiredCallbacks.onRise(
        driver.y(),
        () -> requiredIntake.setMode(StarterIntake.Mode.COLLECT));
```

Run the focused test again **without changing the test**. It should fail because the test still
presses A and expects COLLECT. That red result is useful evidence: the test noticed the real control
change.

Now open only the copied `StarterFirstLessonTest.java`. Change the COLLECT pulse from
`driver.a = true` / `driver.a = false` to `driver.y = true` / `driver.y = false`, and update its two
A comments to Y. Leave the registration-count, B, X, rising-edge, and zero-Task assertions
unchanged. Run the same test again; it should be green.

The test records the semantic request `COLLECT`. It never constructs a Plant, looks up FTC hardware,
or claims that a motor moved.

## 4. Reveal the next owner only when you need it

**Where does FTC enter the robot?** Open the copied `opmode/StarterTeleOp.java`. Its complete
ordinary declaration is only:

```java
@Override
protected void configure(RobotProgram program) {
    StarterProfile profile = StarterProfile.current();
    new StarterRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
}
```

`FtcRobotOpMode` owns the later FTC lifecycle. The method chooses configuration and asks the
composition root to declare the robot; it is not a loop or control script.

**What did the controls request?** Open
[`StarterIntake.java`](<../../../robots/examples/starter/capability/intake/StarterIntake.java>).
TeleOp and Auto share these robot words:

```java
enum Mode {
    STOPPED,
    COLLECT,
    EJECT
}
```

Controls call the capability, not a motor or numeric power.

**How does COLLECT reach one output?** Open the copied
`capability/intake/StarterIntakeMechanism.java`. Its request path is:

```java
@Override
public void setMode(Mode mode) {
    Mode requested = Objects.requireNonNull(mode, "mode");
    plant.commandTarget().set(powerFor(requested));
    requestedMode = requested;
}
```

The mechanism privately owns the Plant, maps the mode to configured power, updates it in the
managed output phase, and stops it during cleanup. Your button edit did not create a second hardware
writer.

**How does Auto reuse the same meaning?** Open the copied `opmode/StarterAuto.java` last:

```java
StarterProfile profile = StarterProfile.current();
StarterIntake intake = new StarterRobot(hardwareMap).declareAuto(program, profile);
program.rootTask(intake.collectForSeconds(COLLECT_DURATION_SEC));
```

`collectForSeconds(...)` creates a fresh, single-use Task. The managed loop advances it without
sleeping or blocking, while the same mechanism remains the final output owner.

## Checkpoint and next route

You are finished when the copied package compiles, its focused test passes with Y → COLLECT, and the
OpModes and permissions remain disabled. The complete source map is in
[Modern starter robot](<../examples/Modern Starter Robot.md>).

Continue in software with [Test a mechanism without hardware](<Test a Mechanism Without Hardware.md>)
to follow the same COLLECT request through the production mechanism and Plant to a recorded motor
command. When the team actually has a supervised robot, return to [Build and run Phoenix](<Build and Run.md>),
then use the
[hardware question selector](<../testing-calibration/README.md#when-hardware-is-available-choose-one-question>)
to review names, directions, powers, brake behavior, clear space, and physical STOP before enabling
any motion.
