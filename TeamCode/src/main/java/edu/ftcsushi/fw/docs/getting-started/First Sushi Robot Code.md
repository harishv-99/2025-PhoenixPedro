# Your first Sushi robot code

**Learning mode:** Architecture reference

The files, edit, test, and verification command
needed for this lesson are included below.

## Goal

Copy the compiled Starter into a team-owned package, change COLLECT from the A button to Y, and
prove the new meaning with one hardware-free test.

**Time:** 45–60 minutes.

**Prerequisite:** Finish [Build and run Sushi](<Build and Run.md>) and begin with its compile and
software-test checkpoints green.

**Safety boundary:** This lesson is software-only. It needs no Robot Controller or matching robot.
Keep both copied OpModes `@Disabled`, and keep `allowIntakeMotion` and `allowDriveMotion` false.

## Files you will create

Copy the seven production files in `edu.ftcsushi.robots.examples.starter` into
`edu.ftcsushi.robots.myrobot`, plus this focused test:

- `capability/intake/StarterIntake.java` and `StarterIntakeMechanism.java`;
- `robot/StarterProfile.java`, `StarterRobot.java`, and `StarterTeleOpControls.java`;
- `opmode/StarterTeleOp.java` and `StarterAuto.java`; and
- test file `robot/StarterFirstLessonTest.java` under `TeamCode/src/test/java`.

The two files you edit in this lesson are reproduced completely below. The other five production
files are copied unchanged; [Modern starter robot](<../examples/Modern Starter Robot.md>) keeps the
complete maintained graph together for later subsystem work.

## 1. Copy the Starter into your package

Choose one lowercase package name for the robot. The steps below use `myrobot`; replace it with the
team's name everywhere.

In Android Studio's Project view, find
[`edu.ftcsushi.robots.examples.starter`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.java>).
Select the `starter` package and choose **Refactor > Copy** (`F5`). Set **New name** to `myrobot`
and the destination directory to `TeamCode/src/main/java/edu/ftcsushi/robots`. The result must be
`edu.ftcsushi.robots.myrobot`. Let Android Studio update package declarations and imports. Leave
the seven class names unchanged for now; rename them only when the real robot has better vocabulary.

The focused test lives under a different source root, so copy it separately. Copy
[`StarterFirstLessonTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterFirstLessonTest.java>)
with **Refactor > Copy**. Keep its name, select the `TeamCode/src/test/java` destination source
root, and set **Destination package** to `edu.ftcsushi.robots.myrobot.robot`. Its `StarterIntake`
import must point to the copied capability. Keeping the test beside the package-private controls
owner is intentional. You only edit the indicated gamepad field later; the rest is provided test
scaffolding.

Use **Find in Files** over both copied packages for
`edu.ftcsushi.robots.examples.starter`; a correct copy has no matches.

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
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.myrobot.robot.StarterFirstLessonTest
```

Both commands should end with `BUILD SUCCESSFUL`. This proves the package refactor is coherent and
the original A-button meaning is green before you change it. If either command fails, correct the
copied package or test; do not edit the canonical example.

## 3. Change one operator meaning, then prove it

### Critical code

Open only the copied `robot/StarterTeleOpControls.java`. In `bind(...)`, change `driver.a()` to
`driver.y()` for COLLECT. The final registration is:

Change only the first binding's input from `driver.a()` to `driver.y()`; leave the B and X
bindings unchanged. The complete checked-in file below shows the original A/B/X baseline, so you
can restore the lesson before continuing.

**What to notice**

- The edit changes an operator meaning without changing the capability or mechanism.
- The unchanged test first turns red, proving it observes the control contract.

**Key APIs:** `CallbackBindings.onRise(...)` declares the edge meaning; `StarterIntake.setMode(...)`
is the semantic capability request.

Run the focused test again **without changing the test**. It should fail because the test still
presses A and expects COLLECT. That red result is useful evidence: the test noticed the real control
change.

Now open only the copied `StarterFirstLessonTest.java`. Change the COLLECT pulse from
`driver.a = true` / `driver.a = false` to `driver.y = true` / `driver.y = false`, and update its two
A comments to Y. Leave the registration-count, B, X, rising-edge, and zero-Task assertions
unchanged. Run the same test again; it should be green.

## Complete working slice

<details>
<summary>Complete working edit slice: StarterTeleOpControls.java</summary>

This is the complete maintained file before the lesson's A-to-Y edit. After copying it into
`myrobot`, change only the first `driver.a()` call to `driver.y()`.

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterTeleOpControls.java -->
```java
package edu.ftcsushi.robots.examples.starter.robot;

import java.util.Objects;

import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;

/** Owns every gamepad meaning used by the starter TeleOp. */
final class StarterTeleOpControls {

    private static final double SLOW_TRANSLATE_SCALE = 0.35;
    private static final double SLOW_OMEGA_SCALE = 0.20;

    private final GamepadDevice driver;
    private final DriveSource driveSource;
    private boolean bindAttempted;

    StarterTeleOpControls(GamepadDevice driver) {
        this.driver = Objects.requireNonNull(driver, "driver");

        driveSource = new GamepadDriveSource(
                this.driver.leftX(),
                this.driver.leftY(),
                this.driver.rightX(),
                GamepadDriveSource.Config.defaults()
        ).scaledWhen(this.driver.rightBumper(), SLOW_TRANSLATE_SCALE, SLOW_OMEGA_SCALE);
    }

    /**
     * Declare this controls owner's callback mappings exactly once.
     *
     * @param callbackBindings managed callback surface; validated before the bind is claimed
     * @param intake semantic intake capability; validated before the bind is claimed
     * @throws NullPointerException if either argument is {@code null}; this does not consume the
     *                              bind opportunity
     * @throws IllegalStateException if a bind was already attempted, including one whose callback
     *                               registration failed partway through
     */
    void bind(CallbackBindings callbackBindings, StarterIntake intake) {
        CallbackBindings requiredCallbacks = Objects.requireNonNull(
                callbackBindings,
                "callbackBindings"
        );
        StarterIntake requiredIntake = Objects.requireNonNull(intake, "intake");
        claimBind();

        requiredCallbacks.onRise(
                driver.a(),
                () -> requiredIntake.setMode(StarterIntake.Mode.COLLECT));
        requiredCallbacks.onRise(
                driver.b(),
                () -> requiredIntake.setMode(StarterIntake.Mode.EJECT));
        requiredCallbacks.onRise(
                driver.x(),
                () -> requiredIntake.setMode(StarterIntake.Mode.STOPPED));
    }

    private void claimBind() {
        if (bindAttempted) {
            throw new IllegalStateException(
                    "StarterTeleOpControls.bind(...) may be called only once; "
                            + "create a fresh controls owner for another callback graph"
            );
        }
        bindAttempted = true;
    }

    DriveSource driveSource() {
        return driveSource;
    }
}
```

</details>

<details>
<summary>Complete working edit slice: StarterFirstLessonTest.java</summary>

This is the complete maintained test before the lesson edit. After copying it, change the two
`driver.a` assignments to `driver.y` and update the two nearby A comments to Y.

<!-- source-file: TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterFirstLessonTest.java -->
```java
package edu.ftcsushi.robots.examples.starter.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.RecordingCallbackBindings;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;

import static org.junit.Assert.assertEquals;

/** Hardware-free proof of the Starter's first operator meanings. */
public final class StarterFirstLessonTest {

    @Test
    public void buttonsRequestSemanticModesOnRisingEdgesWithoutCreatingTasks() {
        Gamepad driver = new Gamepad();
        RecordingIntake intake = new RecordingIntake();
        RecordingCallbackBindings callbacks = new RecordingCallbackBindings();
        StarterTeleOpControls controls =
                new StarterTeleOpControls(new GamepadDevice(driver));
        controls.bind(callbacks, intake);
        assertEquals(3, callbacks.successfulRegistrations());

        Bindings bindings = callbacks.root();
        ManualLoopClock time = new ManualLoopClock();
        bindings.update(time.clock());

        driver.a = true;
        bindings.update(time.nextCycle(0.02));
        assertEquals(Arrays.asList(StarterIntake.Mode.COLLECT), intake.modeRequests);

        bindings.update(time.nextCycle(0.02)); // Holding A is not another rising edge.
        driver.a = false;
        bindings.update(time.nextCycle(0.02)); // Releasing A is not a rising edge.
        assertEquals(Arrays.asList(StarterIntake.Mode.COLLECT), intake.modeRequests);

        pulseB(driver, bindings, time);
        pulseX(driver, bindings, time);
        assertEquals(
                Arrays.asList(
                        StarterIntake.Mode.COLLECT,
                        StarterIntake.Mode.EJECT,
                        StarterIntake.Mode.STOPPED),
                intake.modeRequests);
        assertEquals(0, intake.taskRequests);
    }

    private static void pulseB(Gamepad driver, Bindings bindings, ManualLoopClock time) {
        driver.b = true;
        bindings.update(time.nextCycle(0.02));
        driver.b = false;
        bindings.update(time.nextCycle(0.02));
    }

    private static void pulseX(Gamepad driver, Bindings bindings, ManualLoopClock time) {
        driver.x = true;
        bindings.update(time.nextCycle(0.02));
        driver.x = false;
        bindings.update(time.nextCycle(0.02));
    }

    private static final class RecordingIntake implements StarterIntake {
        private final List<Mode> modeRequests = new ArrayList<Mode>();
        private int taskRequests;

        @Override
        public void setMode(Mode mode) {
            modeRequests.add(mode);
        }

        @Override
        public Task collectForSeconds(double durationSec) {
            taskRequests++;
            return Tasks.noop();
        }

        @Override
        public Status status() {
            Mode mode = modeRequests.isEmpty()
                    ? Mode.STOPPED
                    : modeRequests.get(modeRequests.size() - 1);
            return new Status(mode, 0.0);
        }
    }
}
```

</details>

The test records the semantic request `COLLECT`. It never constructs a Plant, looks up FTC hardware,
or claims that a motor moved.

## 4. Reveal the next owner only when you need it

### Critical code

**Where does FTC enter the robot?** Open the copied `opmode/StarterTeleOp.java`. Its complete
ordinary declaration is only:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.java -->
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
[`StarterIntake.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.java>).
TeleOp and Auto share these robot words:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.java -->
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

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
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

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.java -->
```java
StarterProfile profile = StarterProfile.current();
StarterIntake intake = new StarterRobot(hardwareMap).declareAuto(program, profile);
program.rootTask(intake.collectForSeconds(COLLECT_DURATION_SEC));
```

`collectForSeconds(...)` creates a fresh, single-use Task. The managed loop advances it without
sleeping or blocking, while the same mechanism remains the final output owner.

**What to notice**

- OpMode, capability, mechanism, and Auto each own one different decision.
- Both modes share `StarterIntake.Mode`; only the mechanism maps it to output power.
- Auto creates fresh temporal work but never becomes a second hardware writer.

**Key APIs**

- `FtcRobotOpMode.configure(...)` — one managed composition entry.
- `Plant.commandTarget()` — the mechanism's persistent request within one realization graph.
- `program.rootTask(...)` — one fresh managed Auto root.

## Verify the slice

You are finished when the copied package compiles, its focused test passes with Y → COLLECT, and the
OpModes and permissions remain disabled. The complete source map is in
[Modern starter robot](<../examples/Modern Starter Robot.md>).

Continue in software with [Test a mechanism without hardware](<Test a Mechanism Without Hardware.md>)
to follow the same COLLECT request through the production mechanism and Plant to a recorded motor
command. When the team actually has a supervised robot, return to [Build and run Sushi](<Build and Run.md>),
then use the
[hardware question selector](<../testing-calibration/README.md#when-hardware-is-available-choose-one-question>)
to review names, directions, powers, brake behavior, clear space, and physical STOP before enabling
any motion.
