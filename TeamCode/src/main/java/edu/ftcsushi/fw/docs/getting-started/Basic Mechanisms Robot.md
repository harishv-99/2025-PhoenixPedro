# Build the Basic Mechanisms robot

**Learning mode:** Guided course

<!-- buildable-files: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutines.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawControls.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanism.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawProfile.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawTeleOp.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveAuto.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveControls.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveProfile.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveStopOwner.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicHardwareOwnership.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLift.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftControls.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftProfile.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftTeleOp.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicMechanismsAuto.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotAuto.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotAutoRoutines.java | TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotTeleOp.java | TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanismTest.java | TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanismTest.java | TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicTeleOpControlsTest.java | TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutinesTest.java | TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotScenarioTest.java -->

Use this one course from first drivetrain motion through a referenced lift, claw, integrated TeleOp,
and small Auto. Work one gate at a time. The checked-in examples remain `@Disabled` and all profile
motion permissions remain false until their supervised gate passes.

## Files you will create

The course supplies 21 production files and five tests. Each complete file is collapsed beside the
gate that first needs it, so open only the current file. Copy each needed file to
`TeamCode/src/main/java/edu/ftcsushi/robots/myrobot/`, change its package to
`edu.ftcsushi.robots.myrobot`, and keep filename/public class aligned. Do not edit or enable the
maintained examples.

## Before the season: get first drive moving

Start with [`FirstDriveTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java>). Review these example assumptions:

| Wheel | Configuration name | Starting direction |
|---|---|---|
| Front left | `frontLeftMotor` | `FORWARD` |
| Front right | `frontRightMotor` | `REVERSE` |
| Back left | `backLeftMotor` | `FORWARD` |
| Back right | `backRightMotor` | `REVERSE` |

### Critical code

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
        FirstDriveControls controls = new FirstDriveControls(new GamepadDevice(gamepad1));
        FtcDrives.MecanumConfig drive = FtcDrives.MecanumConfig.defaults();
```

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
        program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, drive));
```

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
            driveSource = new GamepadDriveSource(
                    driver.leftX(),   // Left/right translation.
                    driver.leftY(),   // Forward/back translation.
                    driver.rightX(),  // Clockwise/counter-clockwise turn.
                    GamepadDriveSource.Config.defaults())
                    .scaled(FIRST_RUN_TRANSLATION_SCALE, FIRST_RUN_TURN_SCALE);
```

**What to notice**

- `GamepadDevice` adapts FTC input; `GamepadDriveSource` turns three axes into drive intent.
- `MecanumConfig` owns reviewed robot facts; `program.drive(...)` declares one final sink.

**Key APIs**

- [`FtcRobotOpMode`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcRobotOpMode.html>) — managed FTC host.
- [`GamepadDevice`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/input/GamepadDevice.html>) — FTC input adapter.
- [`GamepadDriveSource`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/drive/source/GamepadDriveSource.html>) — robot-centric drive intent.
- [`FtcDrives`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcDrives.html>) — configured mecanum construction.
- [`RobotProgram.drive(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/RobotProgram.html#drive(edu.ftcsushi.fw.drive.DriveSource,T)>) — one managed drive path.

### Compile checkpoint

Keep `@Disabled` and run:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac
```

Expect `BUILD SUCCESSFUL` and no Driver Station entry. Fix the first compiler error before touching
framework code.

### Raised-wheel pattern and diagnosis

Use [HW: Actuator Bring-up](<../testing-calibration/Actuator Bring-up.md>) on one motor at low power,
then secure the robot raised. Remove `@Disabled` only after the four isolated checks pass. Enter
INIT with controls neutral, START, try one axis at a time, release, and press STOP.

| Driver command | Front left | Front right | Back left | Back right |
|---|---|---|---|---|
| Left stick forward | forward | forward | forward | forward |
| Left stick right | forward | reverse | reverse | forward |
| Right stick right | forward | reverse | forward | reverse |
| Sticks released | stop | stop | stop | stop |

If one corner is wrong, correct that hardware name or `Direction`; never reverse a joystick axis to
hide it. If forward works but strafe or turn does not, recheck wheel placement and configuration mapping.
STOP on the first mismatch and repeat every row twice.

### Bounded floor-drive acceptance

On a clear floor, retain the 0.25 translation / 0.20 turn limits and staff STOP. Without combining
axes, command a brief forward move, right strafe, and clockwise turn, stopping between them. Accept
the gate only when all three directions and the observed stopping distance repeat. Increase scales
later only in small reviewed steps. Software evidence is not physical evidence.

<details>
<summary>Complete FirstDriveTeleOp.java</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
package edu.ftcsushi.robots.examples.firstdrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

@TeleOp(name = "FW First Drive", group = "FW Examples")
@Disabled
public final class FirstDriveTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        FirstDriveControls controls = new FirstDriveControls(new GamepadDevice(gamepad1));
        FtcDrives.MecanumConfig drive = FtcDrives.MecanumConfig.defaults();

        drive.wiring.frontLeftName = "frontLeftMotor";
        drive.wiring.frontRightName = "frontRightMotor";
        drive.wiring.backLeftName = "backLeftMotor";
        drive.wiring.backRightName = "backRightMotor";
        drive.wiring.frontLeftDirection = Direction.FORWARD;
        drive.wiring.frontRightDirection = Direction.REVERSE;
        drive.wiring.backLeftDirection = Direction.FORWARD;
        drive.wiring.backRightDirection = Direction.REVERSE;
        drive.enableZeroPowerBrake = true; // Review BRAKE versus FLOAT for this drivetrain.

        program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, drive));
    }

    private static final class FirstDriveControls {
        private static final double FIRST_RUN_TRANSLATION_SCALE = 0.25;
        private static final double FIRST_RUN_TURN_SCALE = 0.20;

        private final DriveSource driveSource;

        FirstDriveControls(GamepadDevice driver) {
            driveSource = new GamepadDriveSource(
                    driver.leftX(),   // Left/right translation.
                    driver.leftY(),   // Forward/back translation.
                    driver.rightX(),  // Clockwise/counter-clockwise turn.
                    GamepadDriveSource.Config.defaults())
                    .scaled(FIRST_RUN_TRANSLATION_SCALE, FIRST_RUN_TURN_SCALE);
        }

        DriveSource driveSource() {
            return driveSource;
        }
    }
}
```

</details>

## 1. Define lift and claw capability interfaces

**Outcome:** TeleOp and Auto share named behavior while mechanisms privately own sensors, Plants,
configuration, and realization.

**Files:** Implement the lift and claw interfaces/mechanisms, then the three small Profiles, three
focused Controls owners, and `BasicHardwareOwnership.java`. Compile after each file.

### Critical code

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLift.java -->
```java
// docs: Persistent intent, feedback-aware work, homing, and evidence are different promises.
    /** Selects a persistent semantic height without waiting for physical arrival. */
    void setHeight(Height height);

    /**
     * Builds a fresh feedback-aware move to one semantic height.
     *
     * <p>Success requires the selected semantic request still to be current and its cached feedback
     * to report arrival. Timeout and active cancellation do not overwrite the latest persistent
     * request, which may have been superseded while this Task was active.</p>
     *
     * @param height non-null destination
     * @return fresh single-use move Task
     */
    Task moveTo(Height height);

    /**
     * Builds a fresh non-blocking search for the bottom reference switch.
     *
     * <p>A successful search establishes zero, then selects {@link Height#STOWED} before the
     * mechanism's downstream output phase. Timeout and active cancellation retain their truthful
     * outcomes and preserve the latest coherent semantic and numeric height request.</p>
     *
     * @return fresh single-use homing Task
     */
    Task home();

    /** Returns cached request and feedback evidence without polling hardware. */
    Status status();
```

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.java -->
```java
// docs: The claw exposes named state and truthful command status, not raw positions.
    /** Replaces the persistent semantic request. */
    void setState(State state);

    /** Returns the semantic request and cached applied command without polling hardware. */
    Status status();
```

### Implement the foundation before testing

1. Define the semantic interfaces.
2. Give each mechanism a data-only `Config`, defensively snapshot it, and privately build its final
   Plant. The lift also resolves its bottom switch.
3. Implement fresh lift move/home Task factories; keep `update` as the only hardware heartbeat.
4. Put each resource's facts and false-by-default permission in its own Drive/Lift/Claw Profile.
5. Give drive, lift, and claw separate Controls owners; map only semantic operator meanings there.
6. Add the cross-resource motor collision check, then compile before any test or OpMode.

The implementation excerpts below are exact and annotated. Expand only the file you are building.

<details>
<summary>BasicLift.java — semantic API</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLift.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import edu.ftcsushi.fw.task.Task;

/**
 * Small semantic lift API shared by the teaching TeleOps and Autos.
 *
 * <p>Callers choose a named height and inspect evidence; only the mechanism knows encoder units,
 * motor power, or how the bottom switch establishes a reference.</p>
 */
public interface BasicLift {

    /** Named robot positions used everywhere instead of repeating encoder values. */
    enum Height {
        STOWED,
        LOW,
        HIGH
    }

    /** Immutable snapshot of the current request and cached feedback evidence. */
    final class Status {
        /** Most recent semantic request, updated synchronously by command methods or Task start. */
        public final Height requestedHeight;

        /** Numeric position selected by {@link #requestedHeight}, in mechanism inches. */
        public final double requestedPositionIn;

        /**
         * Last encoder measurement published by a successful output heartbeat, in mechanism
         * inches, or {@link Double#NaN} while measurement evidence is unavailable.
         */
        public final double measuredPositionIn;

        /** Whether the last successful output heartbeat had an established position reference. */
        public final boolean referenced;

        /**
         * Whether the last successful output heartbeat proved arrival for the current request.
         * A new request and terminal stop both invalidate this evidence immediately.
         */
        public final boolean atTarget;

        public Status(Height requestedHeight,
                      double requestedPositionIn,
                      double measuredPositionIn,
                      boolean referenced,
                      boolean atTarget) {
            this.requestedHeight = requestedHeight;
            this.requestedPositionIn = requestedPositionIn;
            this.measuredPositionIn = measuredPositionIn;
            this.referenced = referenced;
            this.atTarget = atTarget;
        }
    }

    /** Selects a persistent semantic height without waiting for physical arrival. */
    void setHeight(Height height);

    /**
     * Builds a fresh feedback-aware move to one semantic height.
     *
     * <p>Success requires the selected semantic request still to be current and its cached feedback
     * to report arrival. Timeout and active cancellation do not overwrite the latest persistent
     * request, which may have been superseded while this Task was active.</p>
     *
     * @param height non-null destination
     * @return fresh single-use move Task
     */
    Task moveTo(Height height);

    /**
     * Builds a fresh non-blocking search for the bottom reference switch.
     *
     * <p>A successful search establishes zero, then selects {@link Height#STOWED} before the
     * mechanism's downstream output phase. Timeout and active cancellation retain their truthful
     * outcomes and preserve the latest coherent semantic and numeric height request.</p>
     *
     * @return fresh single-use homing Task
     */
    Task home();

    /** Returns cached request and feedback evidence without polling hardware. */
    Status status();
}

```

</details>
<details>
<summary>BasicLiftMechanism.java — sensor, Plant, move, and home</summary>

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java -->
```java
// docs: Resolve the active-low reference input, then privately build one bounded position Plant.
        bottomSwitch = FtcSensors.digitalLow(map, c.bottomSwitchName)
                .debouncedOnOff(0.02, 0.02);
        lift = FtcActuators.plant(map)
                .motor(c.motorName, c.direction)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .bounded(0.0, c.maximumHeightIn)
                .scaleToNative(c.ticksPerIn)
                .needsReference("basic lift has not been homed")
                .positionTolerance(c.toleranceIn)
                .outputPowerLimitedTo(c.maximumPower)
                .targetFromNewCommand(c.stowedHeightIn)
                .build();
```

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java -->
```java
// docs: The Task uses the same semantic setter, then waits on one paired status snapshot.
        Height selectedHeight = Objects.requireNonNull(height, "height");
        BooleanSource selectedRequestReached = BooleanSource.of(() -> {
            Status snapshot = status();
            return snapshot.requestedHeight == selectedHeight && snapshot.atTarget;
        });

        // Every semantic Task routes through the same owner setter as direct controls.
        return Tasks.sequence(
                Tasks.runOnce(() -> setHeight(selectedHeight)),
                Tasks.waitUntil(selectedRequestReached, moveTimeoutSec));
```

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java -->
```java
// docs: Search preserves the request; only exact success selects the semantic STOWED request.
        Task search = PositionCalibrationTasks.search(lift)
                .withPower(homingPower)
                .until(bottomSwitch)
                .establishReferenceAt(0.0)
                .failAfterSec(homingTimeoutSec)
                .build();

        return Tasks.sequence(
                search,
                Tasks.runOnce(() -> setHeight(Height.STOWED)));
```

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.PositionCalibrationTasks;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.ftc.FtcSensors;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;

/** Owns one bounded referenced-position Plant and realizes {@link BasicLift} intent. */
public final class BasicLiftMechanism implements BasicLift, RobotProgram.Output {

    /** One owner-published semantic request and its forward-mapped Plant command. */
    private static final class RequestSnapshot {
        private final Height height;
        private final double positionIn;

        private RequestSnapshot(Height height, double positionIn) {
            this.height = height;
            this.positionIn = positionIn;
        }
    }

    /** Data-only lift wiring, coordinate, limits, and timeout choices. */
    public static final class Config {
        /** Nonblank FTC Robot Configuration name for the lift motor. */
        public String motorName;

        /** Logical motor direction; verify that positive position moves away from the bottom. */
        public Direction direction;

        /**
         * Nonblank FTC digital-channel name for the bottom switch. The electrical input is
         * active-low: LOW means pressed/at the bottom reference, while HIGH means released.
         */
        public String bottomSwitchName;

        /** Maximum legal lift target in mechanism inches; finite and greater than zero. */
        public double maximumHeightIn;

        /** Positive encoder conversion in native ticks per mechanism inch. */
        public double ticksPerIn;

        /** Symmetric feedback completion tolerance in mechanism inches; finite and positive. */
        public double toleranceIn;

        /** Maximum device-managed motor-power magnitude in range {@code (0, 1]}. */
        public double maximumPower;

        /** Named {@link Height#STOWED} position in mechanism inches. */
        public double stowedHeightIn;

        /** Named {@link Height#LOW} position in mechanism inches. */
        public double lowHeightIn;

        /** Named {@link Height#HIGH} position in mechanism inches. */
        public double highHeightIn;

        /**
         * Normalized search power toward the bottom switch, finite in {@code [-1, 0)}. This
         * polarity is a software convention that must be verified on the physical mechanism.
         */
        public double homingPower;

        /** Positive homing-search timeout in seconds. */
        public double homingTimeoutSec;

        /** Positive feedback-move timeout in seconds. */
        public double moveTimeoutSec;

        private Config() {
            // Start from defaults() so the whole software recipe is present.
        }

        /** Returns a complete software baseline, not reviewed physical facts. */
        public static Config defaults() {
            Config config = new Config();
            config.motorName = "liftMotor";
            config.direction = Direction.FORWARD;
            config.bottomSwitchName = "liftBottom";
            config.maximumHeightIn = 18.0;
            config.ticksPerIn = 100.0;
            config.toleranceIn = 0.20;
            config.maximumPower = 0.30;
            config.stowedHeightIn = 0.0;
            config.lowHeightIn = 4.0;
            config.highHeightIn = 14.0;
            config.homingPower = -0.15;
            config.homingTimeoutSec = 3.0;
            config.moveTimeoutSec = 2.0;
            return config;
        }
    }

    private final PositionPlant lift;
    private final BooleanSource bottomSwitch;
    private final double stowedHeightIn;
    private final double lowHeightIn;
    private final double highHeightIn;
    private final double homingPower;
    private final double homingTimeoutSec;
    private final double moveTimeoutSec;

    private RequestSnapshot request;
    private Status lastStatus;

    /**
     * Validates a defensive configuration snapshot, then constructs and privately owns the Plant.
     *
     * @param hardwareMap FTC hardware registry
     * @param config complete data-only lift configuration
     */
    public BasicLiftMechanism(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config c = copyAndValidate(config);

        bottomSwitch = FtcSensors.digitalLow(map, c.bottomSwitchName)
                .debouncedOnOff(0.02, 0.02);
        lift = FtcActuators.plant(map)
                .motor(c.motorName, c.direction)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .bounded(0.0, c.maximumHeightIn)
                .scaleToNative(c.ticksPerIn)
                .needsReference("basic lift has not been homed")
                .positionTolerance(c.toleranceIn)
                .outputPowerLimitedTo(c.maximumPower)
                .targetFromNewCommand(c.stowedHeightIn)
                .build();

        stowedHeightIn = c.stowedHeightIn;
        lowHeightIn = c.lowHeightIn;
        highHeightIn = c.highHeightIn;
        homingPower = c.homingPower;
        homingTimeoutSec = c.homingTimeoutSec;
        moveTimeoutSec = c.moveTimeoutSec;
        request = requestFor(Height.STOWED);
        lastStatus = new Status(
                request.height, request.positionIn, Double.NaN, false, false);
    }

    @Override
    public void setHeight(Height height) {
        RequestSnapshot next = requestFor(height);

        // Publish no semantic request unless its matching numeric command was accepted.
        lift.commandTarget().set(next.positionIn);
        request = next;
        lastStatus = new Status(
                next.height,
                next.positionIn,
                lastStatus.measuredPositionIn,
                lastStatus.referenced,
                false);
    }

    @Override
    public Task moveTo(Height height) {
        Height selectedHeight = Objects.requireNonNull(height, "height");
        BooleanSource selectedRequestReached = BooleanSource.of(() -> {
            Status snapshot = status();
            return snapshot.requestedHeight == selectedHeight && snapshot.atTarget;
        });

        // Every semantic Task routes through the same owner setter as direct controls.
        return Tasks.sequence(
                Tasks.runOnce(() -> setHeight(selectedHeight)),
                Tasks.waitUntil(selectedRequestReached, moveTimeoutSec));
    }

    @Override
    public Task home() {
        Task search = PositionCalibrationTasks.search(lift)
                .withPower(homingPower)
                .until(bottomSwitch)
                .establishReferenceAt(0.0)
                .failAfterSec(homingTimeoutSec)
                .build();

        return Tasks.sequence(
                search,
                Tasks.runOnce(() -> setHeight(Height.STOWED)));
    }

    @Override
    public Status status() {
        return lastStatus;
    }

    /** Applies the current request once, then publishes one cached evidence snapshot. */
    @Override
    public void update(LoopClock clock) {
        lift.update(clock);
        RequestSnapshot current = request;
        lastStatus = new Status(
                current.height,
                current.positionIn,
                lift.getMeasurement(),
                lift.isReferenced(),
                lift.atTarget());
    }

    /** Terminally stops the one privately owned Plant. */
    @Override
    public void stop() {
        try {
            lift.stop();
        } finally {
            // Terminal stop makes prior arrival evidence inapplicable even if cleanup throws.
            RequestSnapshot current = request;
            lastStatus = new Status(
                    current.height,
                    current.positionIn,
                    lastStatus.measuredPositionIn,
                    lastStatus.referenced,
                    false);
        }
    }

    private RequestSnapshot requestFor(Height height) {
        Height required = Objects.requireNonNull(height, "height");
        return new RequestSnapshot(required, positionFor(required));
    }

    private double positionFor(Height height) {
        switch (height) {
            case HIGH:
                return highHeightIn;
            case LOW:
                return lowHeightIn;
            case STOWED:
            default:
                return stowedHeightIn;
        }
    }

    private static Config copyAndValidate(Config source) {
        Config s = Objects.requireNonNull(source, "BasicLiftMechanism.Config is required");
        Config c = new Config();
        c.motorName = hardwareName(s.motorName, "motorName");
        c.direction = Objects.requireNonNull(s.direction, "direction is required");
        c.bottomSwitchName = hardwareName(s.bottomSwitchName, "bottomSwitchName");
        c.maximumHeightIn = positive(s.maximumHeightIn, "maximumHeightIn");
        c.ticksPerIn = positive(s.ticksPerIn, "ticksPerIn");
        c.toleranceIn = positive(s.toleranceIn, "toleranceIn");
        c.maximumPower = positive(s.maximumPower, "maximumPower");
        if (c.maximumPower > 1.0) {
            throw new IllegalArgumentException("maximumPower must be <= 1.0");
        }
        c.stowedHeightIn = within(s.stowedHeightIn, c.maximumHeightIn, "stowedHeightIn");
        c.lowHeightIn = within(s.lowHeightIn, c.maximumHeightIn, "lowHeightIn");
        c.highHeightIn = within(s.highHeightIn, c.maximumHeightIn, "highHeightIn");
        if (!(c.stowedHeightIn < c.lowHeightIn && c.lowHeightIn < c.highHeightIn)) {
            throw new IllegalArgumentException(
                    "lift heights must satisfy stowedHeightIn < lowHeightIn < highHeightIn "
                            + "<= maximumHeightIn, got " + c.stowedHeightIn + ", "
                            + c.lowHeightIn + ", " + c.highHeightIn + ", "
                            + c.maximumHeightIn);
        }
        c.homingPower = s.homingPower;
        if (!Double.isFinite(c.homingPower) || c.homingPower < -1.0 || c.homingPower >= 0.0) {
            throw new IllegalArgumentException("homingPower must be finite and in [-1, 0)");
        }
        c.homingTimeoutSec = positive(s.homingTimeoutSec, "homingTimeoutSec");
        c.moveTimeoutSec = positive(s.moveTimeoutSec, "moveTimeoutSec");
        return c;
    }

    private static String hardwareName(String value, String field) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "BasicLiftMechanism.Config." + field
                            + " must be a non-blank FTC hardware name");
        }
        return value;
    }

    private static double positive(double value, String field) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(field + " must be finite and > 0, got " + value);
        }
        return value;
    }

    private static double within(double value, double maximum, String field) {
        if (!Double.isFinite(value) || value < 0.0 || value > maximum) {
            throw new IllegalArgumentException(
                    field + " must be finite and in [0, " + maximum + "], got " + value);
        }
        return value;
    }
}

```

</details>
<details>
<summary>BasicClaw.java — semantic API</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

/** Semantic open/closed claw capability shared by TeleOp and Auto. */
public interface BasicClaw {

    /** Named claw requests; callers never repeat raw servo positions. */
    enum State {
        CLOSED,
        OPEN
    }

    /**
     * Immutable command snapshot.
     *
     * <p>A standard servo has no position feedback here, so {@code appliedPosition} is the Plant's
     * final submitted target after bounds, not proof that the claw physically arrived.</p>
     */
    final class Status {
        /** Most recent semantic request, updated synchronously by {@link #setState(State)}. */
        public final State requestedState;

        /**
         * Last position successfully submitted by an output heartbeat, in native FTC Servo units
         * {@code [0.0, 1.0]}, or {@link Double#NaN} before the first successful heartbeat.
         * This is command evidence, not physical-position feedback.
         */
        public final double appliedPosition;

        public Status(State requestedState, double appliedPosition) {
            this.requestedState = requestedState;
            this.appliedPosition = appliedPosition;
        }
    }

    /** Replaces the persistent semantic request. */
    void setState(State state);

    /** Returns the semantic request and cached applied command without polling hardware. */
    Status status();
}
```

</details>
<details>
<summary>BasicClawMechanism.java — bounded servo Plant</summary>

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanism.java -->
```java
// docs: The claw owns one bounded standard-servo Plant initialized from a named semantic state.
        claw = FtcActuators.plant(map)
                .servo(c.servoName, c.direction)
                .position()
                .nonPeriodic()
                .bounded(0.0, 1.0)
                .nativeUnits()
                .targetFromNewCommand(positionFor(c.initialState))
                .build();
```

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanism.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.ftc.RobotProgram;

/** Owns the claw's semantic request, final resolver, standard-servo Plant, update, and stop. */
public final class BasicClawMechanism implements BasicClaw, RobotProgram.Output {

    /** Data-only standard-servo wiring and named-state positions. */
    public static final class Config {
        /** Nonblank FTC Robot Configuration name for the standard positional Servo. */
        public String servoName;

        /** Logical FTC Servo direction applied once during hardware construction. */
        public Direction direction;

        /** Native Servo command for {@link State#CLOSED}, in inclusive range {@code [0, 1]}. */
        public double closedPosition;

        /** Native Servo command for {@link State#OPEN}, in inclusive range {@code [0, 1]}. */
        public double openPosition;

        /**
         * Semantic request held from construction until a caller selects another state. With the
         * default {@link State#CLOSED} value, the first successful output heartbeat submits
         * {@link #closedPosition}; that initial Servo motion must be reviewed physically.
         */
        public State initialState;

        private Config() {
            // Start from defaults() so the whole software recipe is present.
        }

        /** Returns a complete software baseline, not reviewed physical endpoints. */
        public static Config defaults() {
            Config config = new Config();
            config.servoName = "clawServo";
            config.direction = Direction.FORWARD;
            config.closedPosition = 0.25;
            config.openPosition = 0.70;
            config.initialState = State.CLOSED;
            return config;
        }
    }

    private final Plant claw;
    private final double closedPosition;
    private final double openPosition;
    private State requestedState;
    private double lastAppliedPosition = Double.NaN;
    private boolean stopped;

    /**
     * Validates a defensive configuration snapshot, then privately constructs the servo Plant.
     *
     * @param hardwareMap FTC hardware registry
     * @param config complete data-only claw configuration
     */
    public BasicClawMechanism(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config c = copyAndValidate(config);

        closedPosition = c.closedPosition;
        openPosition = c.openPosition;
        requestedState = c.initialState;
        claw = FtcActuators.plant(map)
                .servo(c.servoName, c.direction)
                .position()
                .nonPeriodic()
                .bounded(0.0, 1.0)
                .nativeUnits()
                .targetFromNewCommand(positionFor(c.initialState))
                .build();
    }

    @Override
    public void setState(State state) {
        State requested = Objects.requireNonNull(state, "state");
        claw.commandTarget().set(positionFor(requested));
        requestedState = requested;
    }

    @Override
    public Status status() {
        return new Status(requestedState, lastAppliedPosition);
    }

    /** Applies the held command through the one final Plant path. */
    @Override
    public void update(LoopClock clock) {
        if (stopped) {
            return;
        }
        claw.update(clock);
        // Publish command evidence only after the complete Plant heartbeat succeeds.
        lastAppliedPosition = claw.getAppliedTarget();
    }

    /**
     * Stops the Plant lifecycle without inventing a new OPEN or CLOSED request.
     * The standard servo therefore retains its last physically submitted position command.
     */
    @Override
    public void stop() {
        try {
            claw.stop();
        } finally {
            // A terminally stopped Plant cannot publish new applied-command evidence.
            stopped = true;
        }
    }

    private double positionFor(State state) {
        return state == State.OPEN ? openPosition : closedPosition;
    }

    private static Config copyAndValidate(Config source) {
        Config s = Objects.requireNonNull(source, "BasicClawMechanism.Config is required");
        Config c = new Config();
        c.servoName = hardwareName(s.servoName);
        c.direction = Objects.requireNonNull(s.direction, "direction is required");
        c.closedPosition = servoPosition(s.closedPosition, "closedPosition");
        c.openPosition = servoPosition(s.openPosition, "openPosition");
        c.initialState = Objects.requireNonNull(s.initialState, "initialState is required");
        if (Double.compare(c.closedPosition, c.openPosition) == 0) {
            throw new IllegalArgumentException(
                    "closedPosition and openPosition must be distinct so the named states "
                            + "produce different commands");
        }
        return c;
    }

    private static String hardwareName(String value) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "BasicClawMechanism.Config.servoName must be a non-blank FTC hardware name");
        }
        return value;
    }

    private static double servoPosition(double value, String field) {
        if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
            throw new IllegalArgumentException(
                    field + " must be finite and in [0, 1], got " + value);
        }
        return value;
    }
}
```

</details>
<details>
<summary>BasicDriveProfile.java — drivetrain facts and false-by-default gate</summary>

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveProfile.java -->
```java
// docs: A fresh profile is a software baseline; supervised review authorizes motion later.
        profile.allowDriveMotion = false;
        return profile;
```

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveProfile.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.ftc.FtcDrives;

/** Data-only drivetrain answers and the explicit motion permission used by each drive lesson. */
public final class BasicDriveProfile {

    /**
     * Complete direct-mecanum wiring, direction, BRAKE/FLOAT, and normalized mixer-scale
     * configuration. The configured {@code maxAxial}, {@code maxLateral}, and {@code maxOmega}
     * values are dimensionless multipliers in {@code [0, 1]} applied to robot-centric requests.
     */
    public FtcDrives.MecanumConfig drive;

    /**
     * Whether supervised drivetrain motion is permitted after names, directions, scaling, and
     * physical STOP behavior have been reviewed. {@link #current()} always returns {@code false}.
     */
    public boolean allowDriveMotion;

    private BasicDriveProfile() {
        // Start from current() so every required drivetrain answer is populated together.
    }

    /** Returns a fresh conservative software baseline, not reviewed physical robot facts. */
    public static BasicDriveProfile current() {
        BasicDriveProfile profile = new BasicDriveProfile();
        profile.drive = FtcDrives.MecanumConfig.defaults();
        profile.drive.wiring.frontLeftName = "frontLeftMotor";
        profile.drive.wiring.frontRightName = "frontRightMotor";
        profile.drive.wiring.backLeftName = "backLeftMotor";
        profile.drive.wiring.backRightName = "backRightMotor";
        profile.drive.wiring.frontLeftDirection = Direction.FORWARD;
        profile.drive.wiring.frontRightDirection = Direction.REVERSE;
        profile.drive.wiring.backLeftDirection = Direction.FORWARD;
        profile.drive.wiring.backRightDirection = Direction.REVERSE;
        profile.drive.enableZeroPowerBrake = true;
        profile.drive.drivebase.maxAxial = 0.25;
        profile.drive.drivebase.maxLateral = 0.25;
        profile.drive.drivebase.maxOmega = 0.20;
        profile.allowDriveMotion = false;
        return profile;
    }

    /** Rejects unchecked drivetrain motion before a host performs any hardware lookup. */
    static void requireMotionAllowed(BasicDriveProfile profile, String mode) {
        BasicDriveProfile p = Objects.requireNonNull(profile, "driveProfile");
        if (!p.allowDriveMotion) {
            throw new IllegalStateException(
                    "BasicDriveProfile.allowDriveMotion must be true before " + mode
                            + " may construct a motion-capable drivetrain. Review motor names, "
                            + "directions, scales, and small supervised motion first; then verify "
                            + "physical STOP.");
        }
    }
}
```

</details>
<details>
<summary>BasicLiftProfile.java — lift facts and false-by-default gate</summary>

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftProfile.java -->
```java
// docs: Keep the lift's complete config and its permission in one focused profile.
        profile.lift = BasicLiftMechanism.Config.defaults();
        profile.allowLiftMotion = false;
        return profile;
```

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftProfile.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

/** Data-only lift answers and the explicit motion permission used by each lift lesson. */
public final class BasicLiftProfile {

    /**
     * Complete lift wiring, mechanism-inch coordinate, legal range, named heights, power limit,
     * reference search, and Task timeouts. The owned config documents each field's exact units and
     * polarity.
     */
    public BasicLiftMechanism.Config lift;

    /**
     * Whether supervised lift motion and homing are permitted after wiring, polarity, limits,
     * clearance, and physical STOP behavior have been reviewed. {@link #current()} returns false.
     */
    public boolean allowLiftMotion;

    private BasicLiftProfile() {
        // Start from current() so every required lift answer is populated together.
    }

    /** Returns a fresh complete software baseline, not reviewed physical robot facts. */
    public static BasicLiftProfile current() {
        BasicLiftProfile profile = new BasicLiftProfile();
        profile.lift = BasicLiftMechanism.Config.defaults();
        profile.allowLiftMotion = false;
        return profile;
    }

    /** Rejects unchecked lift motion before a host performs any hardware lookup. */
    static void requireMotionAllowed(BasicLiftProfile profile, String mode) {
        BasicLiftProfile p = Objects.requireNonNull(profile, "liftProfile");
        if (!p.allowLiftMotion) {
            throw new IllegalStateException(
                    "BasicLiftProfile.allowLiftMotion must be true before " + mode
                            + " may construct a motion-capable lift. Review the motor and active-low "
                            + "switch names, direction, limits, and small supervised motion first; "
                            + "then verify physical STOP.");
        }
    }
}
```

</details>
<details>
<summary>BasicClawProfile.java — claw facts and false-by-default gate</summary>

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawProfile.java -->
```java
// docs: Servo endpoints are candidate commands until their supervised gate passes.
        profile.claw = BasicClawMechanism.Config.defaults();
        profile.allowClawMotion = false;
        return profile;
```

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawProfile.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

/** Data-only claw answers and the explicit motion permission used by each claw lesson. */
public final class BasicClawProfile {

    /**
     * Complete standard-servo wiring, direction, initial semantic request, and CLOSED/OPEN native
     * Servo positions in {@code [0, 1]}. These command positions are not feedback measurements.
     */
    public BasicClawMechanism.Config claw;

    /**
     * Whether supervised Servo motion is permitted after direction, endpoints, clearance, and
     * initial CLOSED motion have been reviewed. {@link #current()} always returns {@code false}.
     */
    public boolean allowClawMotion;

    private BasicClawProfile() {
        // Start from current() so every required claw answer is populated together.
    }

    /** Returns a fresh complete software baseline, not reviewed physical robot facts. */
    public static BasicClawProfile current() {
        BasicClawProfile profile = new BasicClawProfile();
        profile.claw = BasicClawMechanism.Config.defaults();
        profile.allowClawMotion = false;
        return profile;
    }

    /** Rejects unchecked claw motion before a host performs any hardware lookup. */
    static void requireMotionAllowed(BasicClawProfile profile, String mode) {
        BasicClawProfile p = Objects.requireNonNull(profile, "clawProfile");
        if (!p.allowClawMotion) {
            throw new IllegalStateException(
                    "BasicClawProfile.allowClawMotion must be true before " + mode
                            + " may construct a motion-capable claw. Review the Servo name, "
                            + "direction, endpoints, initial CLOSED motion, and physical STOP "
                            + "behavior first.");
        }
    }
}
```

</details>
<details>
<summary>BasicDriveControls.java — stable drive meaning</summary>

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveControls.java -->
```java
// docs: Controls select robot-centric intent; the final sink remains elsewhere.
    BasicDriveControls(GamepadDevice driver) {
        GamepadDevice requiredDriver = Objects.requireNonNull(driver, "driver");
        driveSource = new GamepadDriveSource(
                requiredDriver.leftX(),
                requiredDriver.leftY(),
                requiredDriver.rightX(),
                GamepadDriveSource.Config.defaults());
    }
```

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveControls.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Owns the stable robot-centric drive source shared by the focused and complete TeleOps. */
final class BasicDriveControls {

    private final DriveSource driveSource;

    BasicDriveControls(GamepadDevice driver) {
        GamepadDevice requiredDriver = Objects.requireNonNull(driver, "driver");
        driveSource = new GamepadDriveSource(
                requiredDriver.leftX(),
                requiredDriver.leftY(),
                requiredDriver.rightX(),
                GamepadDriveSource.Config.defaults());
    }

    /** Returns the stable robot-centric source used by the managed TeleOp drive phase. */
    DriveSource driveSource() {
        return driveSource;
    }
}
```

</details>
<details>
<summary>BasicLiftControls.java — named lift buttons and fresh homing Tasks</summary>

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftControls.java -->
```java
// docs: Callbacks hold named requests; TaskBindings calls the Supplier for every X rise.
        requiredCallbacks.onRise(
                driver.dpadDown(),
                () -> requiredLift.setHeight(BasicLift.Height.STOWED));
        requiredCallbacks.onRise(
                driver.dpadLeft(),
                () -> requiredLift.setHeight(BasicLift.Height.LOW));
        requiredCallbacks.onRise(
                driver.dpadUp(),
                () -> requiredLift.setHeight(BasicLift.Height.HIGH));

        // A Supplier creates a fresh single-use homing Task on every X-button rise.
        requiredTasks.onRise(driver.x(), requiredLift::home);
```

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftControls.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.fw.task.TaskBindings;

/** Owns the driver's named lift meanings, independent of drivetrain and claw controls. */
final class BasicLiftControls {

    private final GamepadDevice driver;
    private boolean bindAttempted;

    BasicLiftControls(GamepadDevice driver) {
        this.driver = Objects.requireNonNull(driver, "driver");
    }

    /** Declares the lift controls exactly once for one callback and Task graph. */
    void bind(CallbackBindings callbacks, TaskBindings tasks, BasicLift lift) {
        CallbackBindings requiredCallbacks = Objects.requireNonNull(callbacks, "callbacks");
        TaskBindings requiredTasks = Objects.requireNonNull(tasks, "tasks");
        BasicLift requiredLift = Objects.requireNonNull(lift, "lift");
        claimBind();

        requiredCallbacks.onRise(
                driver.dpadDown(),
                () -> requiredLift.setHeight(BasicLift.Height.STOWED));
        requiredCallbacks.onRise(
                driver.dpadLeft(),
                () -> requiredLift.setHeight(BasicLift.Height.LOW));
        requiredCallbacks.onRise(
                driver.dpadUp(),
                () -> requiredLift.setHeight(BasicLift.Height.HIGH));

        // A Supplier creates a fresh single-use homing Task on every X-button rise.
        requiredTasks.onRise(driver.x(), requiredLift::home);
    }

    private void claimBind() {
        if (bindAttempted) {
            throw new IllegalStateException(
                    "BasicLiftControls may be bound only once; create a fresh controls owner for "
                            + "another callback graph");
        }
        bindAttempted = true;
    }
}
```

</details>
<details>
<summary>BasicClawControls.java — named claw buttons</summary>

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawControls.java -->
```java
// docs: Buttons request semantic claw states, never Servo numbers.
        requiredCallbacks.onRise(
                driver.a(),
                () -> requiredClaw.setState(BasicClaw.State.CLOSED));
        requiredCallbacks.onRise(
                driver.b(),
                () -> requiredClaw.setState(BasicClaw.State.OPEN));
```

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawControls.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.CallbackBindings;

/** Owns the driver's named claw meanings, independent of drivetrain and lift controls. */
final class BasicClawControls {

    private final GamepadDevice driver;
    private boolean bindAttempted;

    BasicClawControls(GamepadDevice driver) {
        this.driver = Objects.requireNonNull(driver, "driver");
    }

    /** Declares the claw controls exactly once for one callback graph. */
    void bind(CallbackBindings callbacks, BasicClaw claw) {
        CallbackBindings requiredCallbacks = Objects.requireNonNull(callbacks, "callbacks");
        BasicClaw requiredClaw = Objects.requireNonNull(claw, "claw");
        claimBind();

        requiredCallbacks.onRise(
                driver.a(),
                () -> requiredClaw.setState(BasicClaw.State.CLOSED));
        requiredCallbacks.onRise(
                driver.b(),
                () -> requiredClaw.setState(BasicClaw.State.OPEN));
    }

    private void claimBind() {
        if (bindAttempted) {
            throw new IllegalStateException(
                    "BasicClawControls may be bound only once; create a fresh controls owner for "
                            + "another callback graph");
        }
        bindAttempted = true;
    }
}
```

</details>
<details>
<summary>BasicHardwareOwnership.java — cross-resource motor collision check</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicHardwareOwnership.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.FtcDrives;

/** Package-local cross-capability checks used only by composition roots that own both resources. */
final class BasicHardwareOwnership {

    private BasicHardwareOwnership() {
        // Static validation utility.
    }

    /** Rejects a lift motor key that would also be owned by the direct drivetrain. */
    static void requireDistinctDriveAndLiftMotors(FtcDrives.MecanumConfig drive,
                                                  BasicLiftMechanism.Config lift,
                                                  String mode) {
        FtcDrives.MecanumConfig requiredDrive = Objects.requireNonNull(drive, "drive config");
        BasicLiftMechanism.Config requiredLift = Objects.requireNonNull(lift, "lift config");
        if (requiredDrive.wiring == null) {
            return; // FtcDrives reports the missing nested config before hardware lookup.
        }

        String liftName = requiredLift.motorName;
        requireDistinct(
                mode, liftName, "BasicLiftProfile.lift.motorName",
                requiredDrive.wiring.frontLeftName,
                "BasicDriveProfile.drive.wiring.frontLeftName");
        requireDistinct(
                mode, liftName, "BasicLiftProfile.lift.motorName",
                requiredDrive.wiring.frontRightName,
                "BasicDriveProfile.drive.wiring.frontRightName");
        requireDistinct(
                mode, liftName, "BasicLiftProfile.lift.motorName",
                requiredDrive.wiring.backLeftName,
                "BasicDriveProfile.drive.wiring.backLeftName");
        requireDistinct(
                mode, liftName, "BasicLiftProfile.lift.motorName",
                requiredDrive.wiring.backRightName,
                "BasicDriveProfile.drive.wiring.backRightName");
    }

    private static void requireDistinct(String mode,
                                        String firstName,
                                        String firstPath,
                                        String secondName,
                                        String secondPath) {
        if (isBlank(firstName) || isBlank(secondName)) {
            return;
        }
        String firstKey = firstName.trim();
        String secondKey = secondName.trim();
        if (firstKey.equals(secondKey)) {
            throw new IllegalStateException(
                    mode + " motor ownership collision: " + firstPath + " and " + secondPath
                            + " both resolve to FTC hardware key \"" + firstKey
                            + "\". Configure distinct motor names.");
        }
    }

    private static boolean isBlank(String value) {
        return value == null || value.trim().isEmpty();
    }
}
```

</details>

**Run:** Compile after each foundation file is added:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac
```

**Expect:** Each compile stays green; clients repeat no device name or numeric target; all three
motion permissions remain false.

**Software checkpoint:** `BUILD SUCCESSFUL`; a teammate can describe TeleOp and Auto using only
named heights, claw states, fresh Tasks, and status.

**Physical gate:** No motion is authorized here. Keep `@Disabled` and all three permissions false;
if any device moves during review, press STOP and investigate before continuing.

**What to notice**

- `FtcSensors.digitalLow(...)` makes the active-low switch meaning explicit.
- The Plant builder declares units, bounds, reference, feedback tolerance, command source, and
  output limit before `build()`.
- The lift's direct commands and Tasks share one setter; `PositionCalibrationTasks` searches
  without changing that paired request.

**Key APIs**

- [`BasicLift`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLift.html>) / [`BasicClaw`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.html>) — shared mode-neutral robot vocabulary.
- [`BasicDriveProfile`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveProfile.html>) / [`BasicLiftProfile`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftProfile.html>) / [`BasicClawProfile`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawProfile.html>) — one resource's facts and permission per file.
- [`BasicDriveControls`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveControls.java>) / [`BasicLiftControls`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftControls.java>) / [`BasicClawControls`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawControls.java>) — one operator-meaning owner per capability.
- [`FtcSensors.digitalLow(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcSensors.html>) / [`FtcActuators.plant(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcActuators.html>) — explicit FTC input and staged Plant construction boundaries.
- [`PositionCalibrationTasks.search(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/PositionCalibrationTasks.html>) / [`Tasks.sequence(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/Tasks.html>) — command-preserving reference search plus exact-success semantic continuation.
- [`Tasks.waitUntil(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/Tasks.html>) — bounded wait on the mechanism's coherent semantic request and arrival evidence.

**If it fails:** Fix the first configuration or ownership error. Do not expose a raw Plant at the
composition root or add hardware words to the capability.

**Advance when:** The foundation compiles, both clients use only semantic APIs, and each
status field supports a real criterion.

## 2. Test the interfaces without hardware

**Outcome:** Red/green software-device scenarios prove request → heartbeat → recorded output,
feedback truth, fresh Tasks, and button meanings before hardware is enabled.

**Files:** Add `BasicLiftMechanismTest.java`, `BasicClawMechanismTest.java`, and
`BasicTeleOpControlsTest.java` one at a time.

### Critical code

<!-- annotated-source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanismTest.java -->
```java
// docs: setHeight above changed semantic intent; no motor write occurs until update.
        assertEquals(targetWritesBeforeRequest, f.motor.targetPositionWrites());

        f.lift.update(time.nextCycle(0.02));
        assertEquals(BasicLift.Height.LOW, f.lift.status().requestedHeight);
        assertEquals(f.originalLowHeightIn, f.lift.status().requestedPositionIn, 0.0);
        assertEquals(
                (int) Math.round(f.originalLowHeightIn * f.originalTicksPerIn),
                f.motor.targetPositionTicks());
```

**Run:** Add and run one focused test at a time with no hardware attached:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests "edu.ftcsushi.robots.examples.basicmechanisms.BasicLiftMechanismTest"
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests "edu.ftcsushi.robots.examples.basicmechanisms.BasicClawMechanismTest"
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests "edu.ftcsushi.robots.examples.basicmechanisms.BasicTeleOpControlsTest"
```

### Software checkpoint: request, heartbeat, recorded output

Deliberately make one expectation wrong, see a focused red failure, restore it, and see green.

<details>
<summary>BasicLiftMechanismTest.java — reference, feedback, timeout</summary>

<!-- source-file: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanismTest.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import org.junit.Test;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Teaches the lift's semantic requests, evidence, configuration, and Task lifecycle. */
public final class BasicLiftMechanismTest {

    @Test
    public void semanticRequestReachesHardwareOnTheOutputHeartbeat() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(BasicLift.Height.STOWED, f.lift.status().requestedHeight);
        assertFalse(f.lift.status().referenced);
        assertFalse(f.lift.status().atTarget);
        assertTrue(Double.isNaN(f.lift.status().measuredPositionIn));

        homeSuccessfully(f, time);
        BasicLift.Status beforeRequest = f.lift.status();
        int targetWritesBeforeRequest = f.motor.targetPositionWrites();
        f.lift.setHeight(BasicLift.Height.LOW);

        // Semantic intent is synchronous; hardware and feedback evidence wait for the heartbeat.
        assertEquals(BasicLift.Height.LOW, f.lift.status().requestedHeight);
        assertEquals(f.originalLowHeightIn, f.lift.status().requestedPositionIn, 0.0);
        assertEquals(beforeRequest.measuredPositionIn, f.lift.status().measuredPositionIn, 0.0);
        assertEquals(beforeRequest.referenced, f.lift.status().referenced);
        assertFalse(f.lift.status().atTarget);
        assertEquals(targetWritesBeforeRequest, f.motor.targetPositionWrites());

        f.lift.update(time.nextCycle(0.02));
        assertEquals(BasicLift.Height.LOW, f.lift.status().requestedHeight);
        assertEquals(f.originalLowHeightIn, f.lift.status().requestedPositionIn, 0.0);
        assertEquals(
                (int) Math.round(f.originalLowHeightIn * f.originalTicksPerIn),
                f.motor.targetPositionTicks());

        f.motor.setCurrentPositionTicks(25);
        f.lift.update(time.nextCycle(0.02));
        assertEquals(2.5, f.lift.status().measuredPositionIn, 0.0);
    }

    @Test
    public void constructorDefensivelySnapshotsConfiguration() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        f.config.lowHeightIn = 9.0;
        f.config.ticksPerIn = 50.0;
        f.config.motorName = "replacementMotor";

        f.lift.setHeight(BasicLift.Height.LOW);
        f.lift.update(time.nextCycle(0.02));

        assertEquals(f.originalLowHeightIn, f.lift.status().requestedPositionIn, 0.0);
        assertEquals(
                (int) Math.round(f.originalLowHeightIn * f.originalTicksPerIn),
                f.motor.targetPositionTicks());
    }

    @Test
    public void feedbackMovesAreFreshSingleUseAndCompleteOnlyFromEvidence() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        Task first = f.lift.moveTo(BasicLift.Height.HIGH);
        Task second = f.lift.moveTo(BasicLift.Height.HIGH);
        assertNotSame(first, second);

        f.motor.setCurrentPositionTicks(
                (int) Math.round(f.config.highHeightIn * f.originalTicksPerIn));
        first.start(time.nextCycle(0.02));
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight);
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn, 0.0);
        assertFalse(f.lift.status().atTarget);
        f.lift.update(time.clock());
        assertFalse(first.isComplete());

        first.update(time.nextCycle(0.02));
        f.lift.update(time.clock());
        assertEquals(TaskOutcome.SUCCESS, first.getOutcome());
        assertTrue(f.lift.status().atTarget);

        expectSingleUse(() -> first.start(time.nextCycle(0.02)));
    }

    @Test
    public void unreferencedHighRequestFailsClosedWithoutInventingFeedback() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        f.lift.setHeight(BasicLift.Height.HIGH);
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight);
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn, 0.0);
        assertTrue(Double.isNaN(f.lift.status().measuredPositionIn));
        assertFalse(f.lift.status().referenced);
        assertFalse(f.lift.status().atTarget);
        assertEquals(0, f.motor.targetPositionWrites());

        f.lift.update(time.clock());

        assertEquals(0, f.motor.targetPositionWrites());
        assertEquals(0.0, f.motor.power(), 0.0);
        assertFalse(f.lift.status().referenced);
        assertFalse(f.lift.status().atTarget);
    }

    @Test
    public void feedbackMoveTimesOutAndLeavesItsPersistentRequest() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        Task move = f.lift.moveTo(BasicLift.Height.HIGH);
        move.start(time.nextCycle(0.02));
        f.lift.update(time.clock());
        update(move, f.lift, time, f.config.moveTimeoutSec + 0.01);

        assertEquals(TaskOutcome.TIMEOUT, move.getOutcome());
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight);
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn, 0.0);
        assertEquals(
                (int) Math.round(f.config.highHeightIn * f.originalTicksPerIn),
                f.motor.targetPositionTicks());
        assertFalse(f.lift.status().atTarget);
    }

    @Test
    public void activeMoveCancellationLeavesTheSelectedTargetAndInvalidatesArrival() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        Task move = f.lift.moveTo(BasicLift.Height.LOW);
        move.start(time.nextCycle(0.02));
        f.lift.update(time.clock());
        int selectedTicks = (int) Math.round(
                f.config.lowHeightIn * f.originalTicksPerIn);
        assertEquals(selectedTicks, f.motor.targetPositionTicks());

        move.cancel();
        move.cancel();
        f.lift.update(time.nextCycle(0.02));

        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
        assertEquals(BasicLift.Height.LOW, f.lift.status().requestedHeight);
        assertEquals(f.config.lowHeightIn, f.lift.status().requestedPositionIn, 0.0);
        assertEquals(selectedTicks, f.motor.targetPositionTicks());
        assertFalse(f.lift.status().atTarget);
    }

    @Test
    public void feedbackMoveCannotSucceedFromSupersededSemanticRequest() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        Task move = f.lift.moveTo(BasicLift.Height.LOW);
        move.start(time.nextCycle(0.02));
        f.motor.setCurrentPositionTicks(
                (int) Math.round(f.config.lowHeightIn * f.originalTicksPerIn));
        f.lift.update(time.clock());
        assertTrue(f.lift.status().atTarget);

        // A newer semantic request invalidates the old arrival before the move samples it.
        f.lift.setHeight(BasicLift.Height.HIGH);
        move.update(time.nextCycle(0.01));
        assertFalse(move.isComplete());
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight);
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn, 0.0);

        move.update(time.nextCycle(f.config.moveTimeoutSec));
        assertEquals(TaskOutcome.TIMEOUT, move.getOutcome());
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight);
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn, 0.0);
    }

    @Test
    public void newRequestAndTerminalStopInvalidatePriorArrivalEvidence() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        f.motor.setCurrentPositionTicks(
                (int) Math.round(f.config.lowHeightIn * f.originalTicksPerIn));
        f.lift.setHeight(BasicLift.Height.LOW);
        f.lift.update(time.nextCycle(0.02));
        assertTrue(f.lift.status().atTarget);
        double lowMeasurement = f.lift.status().measuredPositionIn;

        f.lift.setHeight(BasicLift.Height.HIGH);
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight);
        assertEquals(lowMeasurement, f.lift.status().measuredPositionIn, 0.0);
        assertTrue(f.lift.status().referenced);
        assertFalse(f.lift.status().atTarget);

        f.motor.setCurrentPositionTicks(
                (int) Math.round(f.config.highHeightIn * f.originalTicksPerIn));
        f.lift.update(time.nextCycle(0.02));
        assertTrue(f.lift.status().atTarget);
        double highMeasurement = f.lift.status().measuredPositionIn;

        f.lift.stop();
        assertEquals(highMeasurement, f.lift.status().measuredPositionIn, 0.0);
        assertTrue(f.lift.status().referenced);
        assertFalse(f.lift.status().atTarget);
    }

    @Test
    public void homingSuccessSelectsStowedButTimeoutPreservesTheLatestRequest() {
        Fixture success = fixture();
        Task firstHome = success.lift.home();
        Task secondHome = success.lift.home();
        assertNotSame(firstHome, secondHome);

        ManualLoopClock successTime = new ManualLoopClock();
        success.lift.setHeight(BasicLift.Height.HIGH);
        firstHome.start(successTime.clock());
        assertEquals(BasicLift.Height.HIGH, success.lift.status().requestedHeight);
        assertEquals(success.config.highHeightIn,
                success.lift.status().requestedPositionIn, 0.0);
        success.lift.update(successTime.clock());
        assertEquals(success.config.homingPower, success.motor.power(), 0.0);

        success.bottomSwitch.setHigh(false);
        update(firstHome, success.lift, successTime, 0.01);
        int targetWritesBeforeSuccess = success.motor.targetPositionWrites();
        firstHome.update(successTime.nextCycle(0.03));

        assertEquals(TaskOutcome.SUCCESS, firstHome.getOutcome());
        assertEquals(BasicLift.Height.STOWED, success.lift.status().requestedHeight);
        assertEquals(success.config.stowedHeightIn,
                success.lift.status().requestedPositionIn, 0.0);
        assertEquals(targetWritesBeforeSuccess, success.motor.targetPositionWrites());

        success.lift.update(successTime.clock());
        assertTrue(success.lift.status().referenced);
        assertEquals(
                (int) Math.round(success.config.stowedHeightIn * success.originalTicksPerIn),
                success.motor.targetPositionTicks());

        Fixture timeout = fixture();
        ManualLoopClock timeoutTime = new ManualLoopClock();
        Task timedOutHome = timeout.lift.home();
        timedOutHome.start(timeoutTime.clock());
        timeout.lift.setHeight(BasicLift.Height.HIGH);
        assertEquals(BasicLift.Height.HIGH, timeout.lift.status().requestedHeight);
        timeout.lift.update(timeoutTime.clock());
        update(timedOutHome, timeout.lift, timeoutTime, timeout.config.homingTimeoutSec);

        assertEquals(TaskOutcome.TIMEOUT, timedOutHome.getOutcome());
        assertFalse(timeout.lift.status().referenced);
        assertEquals(BasicLift.Height.HIGH, timeout.lift.status().requestedHeight);
        assertEquals(timeout.config.highHeightIn,
                timeout.lift.status().requestedPositionIn, 0.0);
        assertEquals(0.0, timeout.motor.power(), 0.0);
    }

    @Test
    public void activeHomingCancellationZerosPowerAndDoesNotEstablishAReference() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        f.lift.setHeight(BasicLift.Height.HIGH);
        Task home = f.lift.home();

        home.start(time.clock());
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight);
        f.lift.update(time.clock());
        assertEquals(f.config.homingPower, f.motor.power(), 0.0);

        home.cancel();
        home.cancel();
        f.lift.update(time.nextCycle(0.02));

        assertEquals(TaskOutcome.CANCELLED, home.getOutcome());
        assertEquals(0.0, f.motor.power(), 0.0);
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight);
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn, 0.0);
        assertFalse(f.lift.status().referenced);
        assertFalse(f.lift.status().atTarget);
    }

    @Test
    public void invalidConfigurationFailsBeforeAnyHardwareLookup() {
        BasicLiftMechanism.Config timeout = config();
        timeout.moveTimeoutSec = Double.NaN;
        assertConfigFailureBeforeLookup(timeout, "moveTimeoutSec");

        BasicLiftMechanism.Config unordered = config();
        unordered.lowHeightIn = unordered.stowedHeightIn;
        assertConfigFailureBeforeLookup(unordered, "stowedHeightIn < lowHeightIn");

        BasicLiftMechanism.Config unnamed = config();
        unnamed.motorName = "  ";
        assertConfigFailureBeforeLookup(unnamed, "motorName");
    }

    private static void homeSuccessfully(Fixture f, ManualLoopClock time) {
        Task home = f.lift.home();
        home.start(time.clock());
        f.lift.update(time.clock());
        f.bottomSwitch.setHigh(false);
        update(home, f.lift, time, 0.01);
        update(home, f.lift, time, 0.03);
        assertEquals(TaskOutcome.SUCCESS, home.getOutcome());
    }

    private static void update(Task task,
                               BasicLiftMechanism lift,
                               ManualLoopClock time,
                               double dtSec) {
        task.update(time.nextCycle(dtSec));
        lift.update(time.clock());
    }

    private static Fixture fixture() {
        BasicLiftMechanism.Config config = config();
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        FtcTestHardware.DigitalProbe bottomSwitch =
                hardware.addDigitalInput(config.bottomSwitchName);
        return new Fixture(config, motor, bottomSwitch,
                new BasicLiftMechanism(hardware, config));
    }

    private static BasicLiftMechanism.Config config() {
        BasicLiftMechanism.Config config = BasicLiftMechanism.Config.defaults();
        config.motorName = "lift";
        config.bottomSwitchName = "bottom";
        config.maximumHeightIn = 10.0;
        config.ticksPerIn = 10.0;
        config.toleranceIn = 0.10;
        config.stowedHeightIn = 1.0;
        config.lowHeightIn = 4.0;
        config.highHeightIn = 8.0;
        config.homingTimeoutSec = 0.10;
        config.moveTimeoutSec = 0.10;
        return config;
    }

    private static void assertConfigFailureBeforeLookup(
            BasicLiftMechanism.Config config,
            String expectedMessage) {
        FtcTestHardware hardware = new FtcTestHardware();
        try {
            new BasicLiftMechanism(hardware, config);
            fail("Expected invalid Basic lift configuration");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains(expectedMessage));
        }
        assertEquals(0, hardware.lookupCalls());
    }

    private static void expectSingleUse(Runnable action) {
        try {
            action.run();
            fail("Expected a second Task start to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().toLowerCase().contains("single-use"));
        }
    }

    private static final class Fixture {
        private final BasicLiftMechanism.Config config;
        private final double originalLowHeightIn;
        private final double originalTicksPerIn;
        private final FtcTestHardware.MotorProbe motor;
        private final FtcTestHardware.DigitalProbe bottomSwitch;
        private final BasicLiftMechanism lift;

        private Fixture(BasicLiftMechanism.Config config,
                        FtcTestHardware.MotorProbe motor,
                        FtcTestHardware.DigitalProbe bottomSwitch,
                        BasicLiftMechanism lift) {
            this.config = config;
            this.originalLowHeightIn = config.lowHeightIn;
            this.originalTicksPerIn = config.ticksPerIn;
            this.motor = motor;
            this.bottomSwitch = bottomSwitch;
            this.lift = lift;
        }
    }
}

```

</details>
<details>
<summary>BasicClawMechanismTest.java — held command and truthful status</summary>

<!-- source-file: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanismTest.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import org.junit.Test;

import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Teaches semantic servo requests without pretending a standard servo has feedback. */
public final class BasicClawMechanismTest {

    @Test
    public void firstHeartbeatAppliesTheConfiguredInitialClosedRequest() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        assertStatus(f.claw, BasicClaw.State.CLOSED, Double.NaN);
        assertEquals(0, f.servo.positionWrites());

        f.claw.update(time.clock());

        assertStatus(f.claw, BasicClaw.State.CLOSED, f.originalClosedPosition);
        assertEquals(f.originalClosedPosition, f.servo.position(), 0.0);
        assertEquals(1, f.servo.positionWrites());
    }

    @Test
    public void requestIsImmediateButAppliedCommandWaitsForTheOutputHeartbeat() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        assertStatus(f.claw, BasicClaw.State.CLOSED, Double.NaN);
        assertEquals(0, f.servo.positionWrites());

        f.claw.setState(BasicClaw.State.OPEN);
        assertStatus(f.claw, BasicClaw.State.OPEN, Double.NaN);
        assertEquals(0, f.servo.positionWrites());

        f.claw.update(time.clock());

        assertStatus(f.claw, BasicClaw.State.OPEN, f.originalOpenPosition);
        assertEquals(f.originalOpenPosition, f.servo.position(), 0.0);
    }

    @Test
    public void stopBeforeFirstHeartbeatKeepsAppliedCommandUnknownAndWritesNothing() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        f.claw.setState(BasicClaw.State.OPEN);
        f.claw.stop();
        assertStatus(f.claw, BasicClaw.State.OPEN, Double.NaN);
        assertEquals(0, f.servo.positionWrites());

        f.claw.setState(BasicClaw.State.CLOSED);
        f.claw.update(time.clock());
        assertStatus(f.claw, BasicClaw.State.CLOSED, Double.NaN);
        assertEquals(0, f.servo.positionWrites());
    }

    @Test
    public void standardServoStopRetainsTheLastCommandWithoutClaimingArrival() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        f.claw.setState(BasicClaw.State.OPEN);
        f.claw.update(time.clock());
        int writesBeforeStop = f.servo.positionWrites();

        f.claw.stop();
        assertStatus(f.claw, BasicClaw.State.OPEN, f.originalOpenPosition);
        assertEquals(f.originalOpenPosition, f.servo.position(), 0.0);
        int writesAfterStop = f.servo.positionWrites();
        assertTrue(writesAfterStop >= writesBeforeStop);

        f.claw.setState(BasicClaw.State.CLOSED);
        f.claw.update(time.nextCycle(0.02));
        assertStatus(f.claw, BasicClaw.State.CLOSED, f.originalOpenPosition);
        assertEquals(writesAfterStop, f.servo.positionWrites());
        assertEquals(f.originalOpenPosition, f.servo.position(), 0.0);
    }

    @Test
    public void constructorDefensivelySnapshotsNamedPositions() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        f.config.openPosition = 0.95;
        f.config.closedPosition = 0.05;
        f.config.servoName = "replacementServo";

        f.claw.setState(BasicClaw.State.OPEN);
        f.claw.update(time.clock());

        assertStatus(f.claw, BasicClaw.State.OPEN, f.originalOpenPosition);
        assertEquals(f.originalOpenPosition, f.servo.position(), 0.0);
    }

    @Test
    public void invalidConfigurationFailsBeforeAnyHardwareLookup() {
        BasicClawMechanism.Config unnamed = config();
        unnamed.servoName = "  ";
        assertConfigFailureBeforeLookup(unnamed, "servoName");

        BasicClawMechanism.Config nonFinite = config();
        nonFinite.openPosition = Double.NaN;
        assertConfigFailureBeforeLookup(nonFinite, "openPosition");

        BasicClawMechanism.Config indistinguishable = config();
        indistinguishable.openPosition = indistinguishable.closedPosition;
        assertConfigFailureBeforeLookup(indistinguishable, "must be distinct");
    }

    private static Fixture fixture() {
        BasicClawMechanism.Config config = config();
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.ServoProbe servo = hardware.addServo(config.servoName);
        return new Fixture(config, servo, new BasicClawMechanism(hardware, config));
    }

    private static BasicClawMechanism.Config config() {
        BasicClawMechanism.Config config = BasicClawMechanism.Config.defaults();
        config.servoName = "claw";
        config.closedPosition = 0.20;
        config.openPosition = 0.80;
        return config;
    }

    private static void assertStatus(BasicClaw claw,
                                     BasicClaw.State expectedState,
                                     double expectedAppliedPosition) {
        BasicClaw.Status status = claw.status();
        assertEquals(expectedState, status.requestedState);
        assertEquals(expectedAppliedPosition, status.appliedPosition, 0.0);
    }

    private static void assertConfigFailureBeforeLookup(
            BasicClawMechanism.Config config,
            String expectedMessage) {
        FtcTestHardware hardware = new FtcTestHardware();
        try {
            new BasicClawMechanism(hardware, config);
            fail("Expected invalid Basic claw configuration");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains(expectedMessage));
        }
        assertEquals(0, hardware.lookupCalls());
    }

    private static final class Fixture {
        private final BasicClawMechanism.Config config;
        private final double originalClosedPosition;
        private final double originalOpenPosition;
        private final FtcTestHardware.ServoProbe servo;
        private final BasicClawMechanism claw;

        private Fixture(BasicClawMechanism.Config config,
                        FtcTestHardware.ServoProbe servo,
                        BasicClawMechanism claw) {
            this.config = config;
            this.originalClosedPosition = config.closedPosition;
            this.originalOpenPosition = config.openPosition;
            this.servo = servo;
            this.claw = claw;
        }
    }
}
```

</details>
<details>
<summary>BasicTeleOpControlsTest.java — buttons and fresh homing Tasks</summary>

<!-- source-file: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicTeleOpControlsTest.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.RecordingCallbackBindings;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the complete TeleOp's student-facing button vocabulary and declaration lifecycle. */
public final class BasicTeleOpControlsTest {

    @Test
    public void buttonsMapToNamedLiftAndClawIntent() {
        Gamepad driver = new Gamepad();
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        RecordingCallbackBindings callbacks = new RecordingCallbackBindings();
        TaskRunner runner = new TaskRunner();
        GamepadDevice driverDevice = new GamepadDevice(driver);
        BasicDriveControls driveControls = new BasicDriveControls(driverDevice);
        BasicLiftControls liftControls = new BasicLiftControls(driverDevice);
        BasicClawControls clawControls = new BasicClawControls(driverDevice);
        liftControls.bind(callbacks, TaskBindings.of(callbacks, runner), lift);
        clawControls.bind(callbacks, claw);
        Bindings bindings = callbacks.root();
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(6, callbacks.successfulRegistrations());
        assertSame(driveControls.driveSource(), driveControls.driveSource());
        bindings.update(time.clock());

        pulse(driver, bindings, time, Button.DPAD_DOWN);
        pulse(driver, bindings, time, Button.DPAD_LEFT);
        pulse(driver, bindings, time, Button.DPAD_UP);
        pulse(driver, bindings, time, Button.A);
        pulse(driver, bindings, time, Button.B);

        assertEquals(
                Arrays.asList(
                        BasicLift.Height.STOWED,
                        BasicLift.Height.LOW,
                        BasicLift.Height.HIGH),
                lift.heightRequests);
        assertEquals(
                Arrays.asList(BasicClaw.State.CLOSED, BasicClaw.State.OPEN),
                claw.stateRequests);
    }

    @Test
    public void everyXButtonRiseBuildsAndQueuesAFreshHomeTask() {
        Gamepad driver = new Gamepad();
        RecordingLift lift = new RecordingLift();
        RecordingCallbackBindings callbacks = new RecordingCallbackBindings();
        TaskRunner runner = new TaskRunner();
        BasicLiftControls controls = new BasicLiftControls(new GamepadDevice(driver));
        controls.bind(callbacks, TaskBindings.of(callbacks, runner), lift);
        Bindings bindings = callbacks.root();
        ManualLoopClock time = new ManualLoopClock();

        bindings.update(time.clock());
        pulse(driver, bindings, time, Button.X);
        pulse(driver, bindings, time, Button.X);

        assertEquals(2, lift.homeTasks.size());
        assertNotSame(lift.homeTasks.get(0), lift.homeTasks.get(1));
        assertEquals(2, runner.queuedCount());
    }

    @Test
    public void eachFocusedBindingValidatesFirstAndIsOneShot() {
        GamepadDevice driver = new GamepadDevice(new Gamepad());
        BasicLiftControls liftControls = new BasicLiftControls(driver);
        BasicClawControls clawControls = new BasicClawControls(driver);
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        RecordingCallbackBindings liftCallbacks = new RecordingCallbackBindings();
        TaskBindings liftTasks = TaskBindings.of(liftCallbacks, new TaskRunner());

        expectNullPointer(() -> liftControls.bind(null, liftTasks, lift));
        expectNullPointer(() -> liftControls.bind(liftCallbacks, null, lift));
        expectNullPointer(() -> liftControls.bind(liftCallbacks, liftTasks, null));
        assertEquals(0, liftCallbacks.registrationAttempts());

        liftControls.bind(liftCallbacks, liftTasks, lift);
        assertEquals(4, liftCallbacks.successfulRegistrations());

        RecordingCallbackBindings liftRetry = new RecordingCallbackBindings();
        expectAlreadyBound(() -> liftControls.bind(
                liftRetry, TaskBindings.of(liftRetry, new TaskRunner()), lift));
        assertEquals(0, liftRetry.registrationAttempts());

        RecordingCallbackBindings clawCallbacks = new RecordingCallbackBindings();
        expectNullPointer(() -> clawControls.bind(null, claw));
        expectNullPointer(() -> clawControls.bind(clawCallbacks, null));
        assertEquals(0, clawCallbacks.registrationAttempts());

        clawControls.bind(clawCallbacks, claw);
        assertEquals(2, clawCallbacks.successfulRegistrations());

        RecordingCallbackBindings clawRetry = new RecordingCallbackBindings();
        expectAlreadyBound(() -> clawControls.bind(clawRetry, claw));
        assertEquals(0, clawRetry.registrationAttempts());
    }

    private enum Button {
        A,
        B,
        X,
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT
    }

    private static void pulse(Gamepad gamepad,
                              Bindings bindings,
                              ManualLoopClock time,
                              Button button) {
        setButton(gamepad, button, true);
        bindings.update(time.nextCycle(0.02));
        setButton(gamepad, button, false);
        bindings.update(time.nextCycle(0.02));
    }

    private static void setButton(Gamepad gamepad, Button button, boolean value) {
        switch (button) {
            case A:
                gamepad.a = value;
                return;
            case B:
                gamepad.b = value;
                return;
            case X:
                gamepad.x = value;
                return;
            case DPAD_UP:
                gamepad.dpad_up = value;
                return;
            case DPAD_DOWN:
                gamepad.dpad_down = value;
                return;
            case DPAD_LEFT:
                gamepad.dpad_left = value;
                return;
            default:
                throw new AssertionError("Unsupported button " + button);
        }
    }

    private static void expectNullPointer(Runnable action) {
        try {
            action.run();
            fail("Expected null argument to fail");
        } catch (NullPointerException expected) {
            // Expected.
        }
    }

    private static void expectAlreadyBound(Runnable action) {
        try {
            action.run();
            fail("Expected a repeated bind to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().toLowerCase().contains("bound"));
        }
    }

    private static final class RecordingLift implements BasicLift {
        private final List<Height> heightRequests = new ArrayList<Height>();
        private final List<Task> homeTasks = new ArrayList<Task>();

        @Override
        public void setHeight(Height height) {
            heightRequests.add(height);
        }

        @Override
        public Task moveTo(Height height) {
            return Tasks.runOnce(() -> setHeight(height));
        }

        @Override
        public Task home() {
            Task task = Tasks.runOnce(() -> heightRequests.add(Height.STOWED));
            homeTasks.add(task);
            return task;
        }

        @Override
        public Status status() {
            return new Status(Height.STOWED, 0.0, 0.0, false, false);
        }
    }

    private static final class RecordingClaw implements BasicClaw {
        private final List<State> stateRequests = new ArrayList<State>();

        @Override
        public void setState(State state) {
            stateRequests.add(state);
        }

        @Override
        public Status status() {
            return new Status(State.CLOSED, 0.0);
        }
    }
}
```

</details>

**Expect:** `FtcTestHardware` records output independently from injected encoder/switch inputs, and
one `ManualLoopClock` supplies every tested cycle.

**Software checkpoint:** Each named suite reports green after one deliberate red/green cycle; the
lift trace visibly separates request, heartbeat, and recorded output.

**Physical gate:** None is authorized. Keep the teaching hosts `@Disabled`, keep all profile
permissions false, and press STOP if connected hardware moves unexpectedly.

**What to notice**

- A request alone does not impersonate the output heartbeat.
- Configuration failures occur before hardware lookup.
- These tests model no physics and prove no physical direction, loading, clearance, or tuning.

Repository-example invariants are intentional: `BasicRobotScenarioTest` requires every maintained
host to stay `@Disabled` and all three Profile `current()` factories to return all permissions false.
Do not weaken those assertions. Make team-owned hosts/profiles and record each approved transition
in team-owned tests or a signed bring-up checklist.

**Key APIs**

- [`BasicLiftMechanismTest`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanismTest.java>) — complete lift configuration, reference, feedback, and timeout scenario.
- [`BasicClawMechanismTest`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanismTest.java>) — complete standard-servo command scenario.
- [`BasicTeleOpControlsTest`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicTeleOpControlsTest.java>) — semantic button and fresh-Task scenario.
- [`FtcTestHardware`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/fw/testing/ftc/FtcTestHardware.java>) — test-only FTC registry and probes.
- [`ManualLoopClock`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/fw/testing/ManualLoopClock.java>) — deterministic cycle and time identity.

**If it fails:** Fix the earliest semantic, configuration, or evidence mismatch. Never mirror a
command into feedback to make the test pass.

**Advance when:** All three focused suites pass and the team can state what each does and does not
prove.

## 3. Connect hardware and run focused TeleOp

**Outcome:** Drive plus one mechanism at a time passes configuration review, isolated bring-up,
neutral INIT, controlled motion, telemetry review, and STOP.

**Files:** Add `BasicLiftTeleOp.java` and `BasicClawTeleOp.java`.

### Critical code

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftTeleOp.java -->
```java
// docs: The mechanism is one managed output owner; drive remains one managed final path.
        BasicLiftMechanism lift = program.output(
                new BasicLiftMechanism(hardwareMap, liftProfile.lift));
        GamepadDevice driver = new GamepadDevice(gamepad1);
        BasicDriveControls driveControls = new BasicDriveControls(driver);
        BasicLiftControls liftControls = new BasicLiftControls(driver);
        liftControls.bind(program.callbackBindings(), program.taskBindings(), lift);
        program.drive(
                driveControls.driveSource(),
                FtcDrives.mecanum(hardwareMap, driveProfile.drive));
```
<details>
<summary>BasicLiftTeleOp.java — drive plus lift</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftTeleOp.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Frozen drive-plus-lift fixture with focused lift controls and truthful feedback evidence. */
@TeleOp(name = "FW Basic 2: Lift", group = "FW Examples")
@Disabled
public final class BasicLiftTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicDriveProfile driveProfile = BasicDriveProfile.current();
        BasicLiftProfile liftProfile = BasicLiftProfile.current();
        BasicDriveProfile.requireMotionAllowed(driveProfile, "Basic Lift TeleOp");
        BasicLiftProfile.requireMotionAllowed(liftProfile, "Basic Lift TeleOp");
        BasicHardwareOwnership.requireDistinctDriveAndLiftMotors(
                driveProfile.drive, liftProfile.lift, "Basic Lift TeleOp");

        BasicLiftMechanism lift = program.output(
                new BasicLiftMechanism(hardwareMap, liftProfile.lift));
        GamepadDevice driver = new GamepadDevice(gamepad1);
        BasicDriveControls driveControls = new BasicDriveControls(driver);
        BasicLiftControls liftControls = new BasicLiftControls(driver);
        liftControls.bind(program.callbackBindings(), program.taskBindings(), lift);
        program.drive(
                driveControls.driveSource(),
                FtcDrives.mecanum(hardwareMap, driveProfile.drive));

        program.presenter((clock, telemetry) -> {
            BasicLift.Status status = lift.status();
            telemetry.addData("lift.request", status.requestedHeight);
            telemetry.addData("lift.positionIn", "%.2f / %.2f",
                    status.measuredPositionIn, status.requestedPositionIn);
            telemetry.addData("lift.referenced", status.referenced);
            telemetry.addData("lift.atTarget", status.atTarget);
        });
    }
}
```

</details>
<details>
<summary>BasicClawTeleOp.java — drive plus claw</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawTeleOp.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Frozen drive-plus-claw fixture that never claims unmeasured servo arrival. */
@TeleOp(name = "FW Basic 3: Claw", group = "FW Examples")
@Disabled
public final class BasicClawTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicDriveProfile driveProfile = BasicDriveProfile.current();
        BasicClawProfile clawProfile = BasicClawProfile.current();
        BasicDriveProfile.requireMotionAllowed(driveProfile, "Basic Claw TeleOp");
        BasicClawProfile.requireMotionAllowed(clawProfile, "Basic Claw TeleOp");

        BasicClawMechanism claw = program.output(
                new BasicClawMechanism(hardwareMap, clawProfile.claw));
        GamepadDevice driver = new GamepadDevice(gamepad1);
        BasicDriveControls driveControls = new BasicDriveControls(driver);
        BasicClawControls clawControls = new BasicClawControls(driver);
        clawControls.bind(program.callbackBindings(), claw);
        program.drive(
                driveControls.driveSource(),
                FtcDrives.mecanum(hardwareMap, driveProfile.drive));

        program.presenter((clock, telemetry) -> {
            BasicClaw.Status status = claw.status();
            telemetry.addData("claw.request", status.requestedState);
            telemetry.addData("claw.appliedCommand", "%.2f", status.appliedPosition);
            telemetry.addLine("claw position is a command, not servo feedback");
        });
    }
}
```

</details>

**Run:** First compile both focused hosts:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac
```

Replace each profile fact with a reviewed fact and use HW: Actuator Bring-up. In a team-owned copy,
enable only drive plus one mechanism, remove `@Disabled` from that host, enter INIT with controls
neutral, and test secured at low power with STOP staffed.

**Expect:**

- Lift host: D-pad selects heights, X starts a fresh homing Task, and request/reference/arrival
  telemetry changes only with the corresponding evidence.
- Claw host: A/B select closed/open and telemetry reports a submitted command, never measured
  arrival.

**Software checkpoint:** `BUILD SUCCESSFUL`; both hosts remain absent from the Driver Station while
the repository copies retain `@Disabled`.

**Physical gate:** Test lift and claw separately. For lift, accept only reviewed direction, range,
switch polarity, telemetry, and STOP. For claw, clear the mechanism first: its initial `CLOSED` target
moves on the first active heartbeat, before A/B is pressed. Accept only reviewed endpoints,
clearance, and STOP before enabling the other host.

**What to notice**

- `program.output(...)` registers the sole mechanism update/stop owner.
- Each host rejects motion before hardware lookup until its gates are true.
- Servo applied position is a submitted command, not measured arrival.

**Key APIs**

- [`BasicLiftMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftMechanism.html>) / [`BasicClawMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawMechanism.html>) — final mechanism ownership and status.
- [`BasicLiftTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftTeleOp.html>) / [`BasicClawTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawTeleOp.html>) — focused drive-plus-mechanism hosts.
- [`RobotProgram.output(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/RobotProgram.html#output(T)>) — managed output registration.
- [`FtcActuators`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcActuators.html>) — FTC Plant construction boundary.

**If it fails:** Press STOP, restore the permission to false, and return to the earliest invalid
profile fact, software scenario, or isolated actuator check.

**Advance when:** Drive+lift and drive+claw each pass neutral INIT, expected range/direction,
telemetry truth, and physical STOP independently.

## 4. Run a bounded subsystem experiment

**Outcome:** Repeated trials answer one measurable mechanism question before integration.

**Files:** Keep production code fixed. Use focused-host status and the
[Subsystem Experiments](<../examples/Subsystem Experiments.md>) lab card.

### Critical code

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftTeleOp.java -->
```java
// docs: The presenter formats cached evidence; it never becomes another behavior owner.
        program.presenter((clock, telemetry) -> {
            BasicLift.Status status = lift.status();
            telemetry.addData("lift.request", status.requestedHeight);
            telemetry.addData("lift.positionIn", "%.2f / %.2f",
                    status.measuredPositionIn, status.requestedPositionIn);
            telemetry.addData("lift.referenced", status.referenced);
            telemetry.addData("lift.atTarget", status.atTarget);
        });
```

**Run:** Reconfirm the lift scenario before every physical trial set:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests "edu.ftcsushi.robots.examples.basicmechanisms.BasicLiftMechanismTest"
```

Freeze starting conditions and run at least three trials: does HIGH reach target within its timeout
without crossing the reviewed limit? Record status and observation separately.

**Expect:** The evidence supports accept, revise, or reject. “Looked okay” is not a criterion.

**Software checkpoint:** `BasicLiftMechanismTest` is green and the recorded test trace still shows
request, heartbeat, feedback, timeout, and cancellation as separate facts.

**Physical gate:** Staff STOP. Accept only when three bounded HIGH trials meet the written time,
range, reference, and repeatability criterion; STOP on unexpected motion.

**What to notice**

- Computed status and observation are different evidence.
- Change one factor per trial and retain failures.
- Servo command does not prove claw arrival.

**Key APIs**

- [`BasicLift`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLift.html>) / [`BasicClaw`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClaw.html>) — cached evidence owned by each capability.
- [`RobotProgram.presenter(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/RobotProgram.html#presenter(edu.ftcsushi.fw.ftc.RobotProgram.Presenter)>) — snapshot formatting after outputs update.

**If it fails:** Set the relevant permission false, revise one fact, rerun software checks, and
repeat the bounded trial.

**Advance when:** The criterion passes repeatedly and the safe range and STOP response are recorded.

## 5. Integrate the complete TeleOp

**Outcome:** One controls owner, two mechanism owners, and one drive sink work together without
competing writers.

**Files:** Add `BasicRobotTeleOp.java`.

### Critical code

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotTeleOp.java -->
```java
// docs: Declare each owner once, then connect semantic controls and the final drive path.
        BasicLiftMechanism lift = program.output(
                new BasicLiftMechanism(hardwareMap, liftProfile.lift));
        BasicClawMechanism claw = program.output(
                new BasicClawMechanism(hardwareMap, clawProfile.claw));

        GamepadDevice driver = new GamepadDevice(gamepad1);
        BasicDriveControls driveControls = new BasicDriveControls(driver);
        new BasicLiftControls(driver).bind(
                program.callbackBindings(), program.taskBindings(), lift);
        new BasicClawControls(driver).bind(program.callbackBindings(), claw);

        // RobotProgram owns the one source-driven final drivetrain write in TeleOp.
        program.drive(
                driveControls.driveSource(),
                FtcDrives.mecanum(hardwareMap, driveProfile.drive));
```
<details>
<summary>BasicRobotTeleOp.java — complete TeleOp</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotTeleOp.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Complete basic TeleOp: one gamepad, drivetrain, referenced lift, and standard-servo claw. */
@TeleOp(name = "FW Basic 5: Robot TeleOp", group = "FW Examples")
@Disabled
public final class BasicRobotTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicDriveProfile driveProfile = BasicDriveProfile.current();
        BasicLiftProfile liftProfile = BasicLiftProfile.current();
        BasicClawProfile clawProfile = BasicClawProfile.current();
        BasicDriveProfile.requireMotionAllowed(driveProfile, "Basic Robot TeleOp");
        BasicLiftProfile.requireMotionAllowed(liftProfile, "Basic Robot TeleOp");
        BasicClawProfile.requireMotionAllowed(clawProfile, "Basic Robot TeleOp");
        BasicHardwareOwnership.requireDistinctDriveAndLiftMotors(
                driveProfile.drive, liftProfile.lift, "Basic Robot TeleOp");

        BasicLiftMechanism lift = program.output(
                new BasicLiftMechanism(hardwareMap, liftProfile.lift));
        BasicClawMechanism claw = program.output(
                new BasicClawMechanism(hardwareMap, clawProfile.claw));

        GamepadDevice driver = new GamepadDevice(gamepad1);
        BasicDriveControls driveControls = new BasicDriveControls(driver);
        new BasicLiftControls(driver).bind(
                program.callbackBindings(), program.taskBindings(), lift);
        new BasicClawControls(driver).bind(program.callbackBindings(), claw);

        // RobotProgram owns the one source-driven final drivetrain write in TeleOp.
        program.drive(
                driveControls.driveSource(),
                FtcDrives.mecanum(hardwareMap, driveProfile.drive));
        program.presenter((clock, telemetry) -> {
            BasicLift.Status liftStatus = lift.status();
            BasicClaw.Status clawStatus = claw.status();
            telemetry.addData("lift", "%s %.2f/%.2f in referenced=%s atTarget=%s",
                    liftStatus.requestedHeight,
                    liftStatus.measuredPositionIn,
                    liftStatus.requestedPositionIn,
                    liftStatus.referenced,
                    liftStatus.atTarget);
            telemetry.addData("claw", "%s command=%.2f",
                    clawStatus.requestedState, clawStatus.appliedPosition);
        });
    }
}
```

</details>

**Run:** Re-run the semantic controls suite:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests "edu.ftcsushi.robots.examples.basicmechanisms.BasicTeleOpControlsTest"
```

Verify lift and drive keys are distinct. In the team-owned host, enable all three permissions and
remove `@Disabled`; enter INIT neutral, then test one function before combined motion.

**Expect:** Focused behavior and telemetry remain truthful. STOP ends drive and both output
lifecycles.

**Software checkpoint:** `BasicTeleOpControlsTest` is green, including fresh homing Tasks and the
complete semantic button map.

**Physical gate:** Accept only after neutral INIT, every isolated control, one bounded combined
movement, truthful telemetry, and STOP all repeat without a motor-owner collision.

**What to notice**

- Controls own meanings; mechanisms own numbers.
- Callback bindings hold requests; Task bindings create fresh homing work.
- `RobotProgram` supplies lifecycle and loop order.

**Key APIs**

- [`BasicRobotTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotTeleOp.html>) — complete managed TeleOp root.
- [`BasicLiftControls.bind(...)`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftControls.java>) / [`BasicClawControls.bind(...)`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawControls.java>) — separate semantic control groups.
- [`RobotProgram.drive(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/RobotProgram.html#drive(edu.ftcsushi.fw.drive.DriveSource,T)>) — one source-to-sink drive path.
- [`FtcDrives.mecanum(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcDrives.html#mecanum(com.qualcomm.robotcore.hardware.HardwareMap,edu.ftcsushi.fw.ftc.FtcDrives.MecanumConfig)>) — configured drivetrain boundary.

**If it fails:** Re-disable the host and reproduce the issue in the smallest focused host or test.
Never share a motor key.

**Advance when:** Neutral INIT, isolated/combined controls, status, and STOP all pass without
regression.

## 6. Test individual Auto behaviors

**Outcome:** Mechanism groups and timed drive prove order, prerequisite outcomes, cancellation, and
physical readiness before full Auto.

**Files:** Add `BasicAutoRoutines.java`, `BasicMechanismsAuto.java`, `BasicDriveAuto.java`,
`BasicDriveStopOwner.java`, and
`BasicAutoRoutinesTest.java`.

### Critical code

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutines.java -->
```java
// docs: Side-effect-free capability Task factories build eagerly; exact success alone starts the next child.
        return Tasks.sequence(
                requiredLift.home(),
                requiredLift.moveTo(BasicLift.Height.HIGH),
                // The lift is the deadline, while the one-cycle claw request starts concurrently
                // and persists through the mechanism after that companion Task completes.
                Tasks.parallelDeadline(
                        requiredLift.moveTo(BasicLift.Height.LOW),
                        requestClaw(requiredClaw, BasicClaw.State.CLOSED)),
                Tasks.waitForSeconds(GUIDE_HOLD_SEC),
                Tasks.parallelDeadline(
                        requiredLift.moveTo(BasicLift.Height.STOWED),
                        requestClaw(requiredClaw, BasicClaw.State.OPEN)));
```

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveAuto.java -->
```java
// docs: Register STOP ownership before constructing the exclusive bounded root.
        DriveCommandSink autoDrive = FtcDrives.mecanum(hardwareMap, driveProfile.drive);
        BasicDriveStopOwner driveStopOwner = new BasicDriveStopOwner(autoDrive);
        try {
            // A stop-only owner is registered before the root Task can fail to be constructed.
            program.output(driveStopOwner);
            Task auto = DriveTasks.driveExclusivelyForSeconds(
                    autoDrive,
                    new DriveSignal(FORWARD_REQUEST, 0.0, 0.0),
                    FORWARD_DURATION_SEC);
            program.rootTask(auto);
```

### Semantic controls and Auto tests

<details>
<summary>BasicAutoRoutines.java — fresh mechanism and complete Auto factories</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutines.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;

/** Auto policy composed entirely from the same semantic lift and claw APIs used in TeleOp. */
public final class BasicAutoRoutines {

    private static final double GUIDE_HOLD_SEC = 0.50;

    private BasicAutoRoutines() {
        // Static robot-owned routine factories.
    }

    /**
     * Builds the course's mechanism-only command-group lesson.
     *
     * <p>The exact sequence is home, HIGH, LOW and CLOSED in parallel, hold for 0.5 seconds, then
     * OPEN and STOWED in parallel. The fixed graph calls these side-effect-free capability Task
     * factories eagerly, but each feedback prerequisite must succeed before the next Task starts.
     * A timeout stays visible and does not quietly continue the routine.</p>
     *
     * @return fresh single-use mechanism-only root Task
     */
    public static Task guide(BasicLift lift, BasicClaw claw) {
        BasicLift requiredLift = Objects.requireNonNull(lift, "lift");
        BasicClaw requiredClaw = Objects.requireNonNull(claw, "claw");

        return Tasks.sequence(
                requiredLift.home(),
                requiredLift.moveTo(BasicLift.Height.HIGH),
                // The lift is the deadline, while the one-cycle claw request starts concurrently
                // and persists through the mechanism after that companion Task completes.
                Tasks.parallelDeadline(
                        requiredLift.moveTo(BasicLift.Height.LOW),
                        requestClaw(requiredClaw, BasicClaw.State.CLOSED)),
                Tasks.waitForSeconds(GUIDE_HOLD_SEC),
                Tasks.parallelDeadline(
                        requiredLift.moveTo(BasicLift.Height.STOWED),
                        requestClaw(requiredClaw, BasicClaw.State.OPEN)));
    }

    /** Creates a fresh one-cycle semantic command Task. */
    private static Task requestClaw(BasicClaw claw, BasicClaw.State state) {
        return Tasks.runOnce(() -> claw.setState(state));
    }

}
```

</details>
<details>
<summary>BasicMechanismsAuto.java — mechanism-only managed host</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicMechanismsAuto.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;

/** Mechanism-only Auto host used to learn sequencing, parallel work, and success gates. */
@Autonomous(name = "FW Basic 4: Mechanisms Auto", group = "FW Examples")
@Disabled
public final class BasicMechanismsAuto extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicLiftProfile liftProfile = BasicLiftProfile.current();
        BasicClawProfile clawProfile = BasicClawProfile.current();
        BasicLiftProfile.requireMotionAllowed(liftProfile, "Basic Mechanisms Auto");
        BasicClawProfile.requireMotionAllowed(clawProfile, "Basic Mechanisms Auto");

        // Register each owner immediately so a later construction failure still receives STOP.
        BasicLiftMechanism lift = program.output(
                new BasicLiftMechanism(hardwareMap, liftProfile.lift));
        BasicClawMechanism claw = program.output(
                new BasicClawMechanism(hardwareMap, clawProfile.claw));

        Task auto = BasicAutoRoutines.guide(lift, claw);
        program.rootTask(auto);
        program.presenter((clock, telemetry) -> {
            BasicLift.Status liftStatus = lift.status();
            BasicClaw.Status clawStatus = claw.status();
            telemetry.addData("lift", "%s %.2f/%.2f in referenced=%s atTarget=%s",
                    liftStatus.requestedHeight,
                    liftStatus.measuredPositionIn,
                    liftStatus.requestedPositionIn,
                    liftStatus.referenced,
                    liftStatus.atTarget);
            telemetry.addData("claw", "%s command=%.2f",
                    clawStatus.requestedState, clawStatus.appliedPosition);
            telemetry.addData("auto.complete", auto.isComplete());
            telemetry.addData("auto.outcome", auto.getOutcome());
        });
    }
}
```

</details>
<details>
<summary>BasicDriveAuto.java — bounded drive-only fixture</summary>

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveAuto.java -->
```java
// docs: The Task exclusively owns the sink and the presenter exposes its terminal truth.
            Task auto = DriveTasks.driveExclusivelyForSeconds(
                    autoDrive,
                    new DriveSignal(FORWARD_REQUEST, 0.0, 0.0),
                    FORWARD_DURATION_SEC);
            program.rootTask(auto);
            program.presenter((clock, telemetry) -> {
                telemetry.addData("auto.complete", auto.isComplete());
                telemetry.addData("auto.outcome", auto.getOutcome());
            });
```

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveAuto.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveTasks;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;

/**
 * Bounded drive-only Auto with one exclusive Task command path and a stop-only lifecycle owner.
 */
@Autonomous(name = "FW Basic 6: Drive Auto", group = "FW Examples")
@Disabled
public final class BasicDriveAuto extends FtcRobotOpMode {

    /** Robot-centric axial request before the profile's {@code maxAxial} mixer scaling. */
    private static final double FORWARD_REQUEST = 0.20;

    /** Positive duration for which the request remains observable, in seconds. */
    private static final double FORWARD_DURATION_SEC = 0.75;

    @Override
    protected void configure(RobotProgram program) {
        BasicDriveProfile driveProfile = BasicDriveProfile.current();
        BasicDriveProfile.requireMotionAllowed(driveProfile, "Basic Drive Auto");

        // This sink belongs only to DriveTasks; do not also pass it to program.drive(...).
        DriveCommandSink autoDrive = FtcDrives.mecanum(hardwareMap, driveProfile.drive);
        BasicDriveStopOwner driveStopOwner = new BasicDriveStopOwner(autoDrive);
        try {
            // A stop-only owner is registered before the root Task can fail to be constructed.
            program.output(driveStopOwner);
            Task auto = DriveTasks.driveExclusivelyForSeconds(
                    autoDrive,
                    new DriveSignal(FORWARD_REQUEST, 0.0, 0.0),
                    FORWARD_DURATION_SEC);
            program.rootTask(auto);
            program.presenter((clock, telemetry) -> {
                telemetry.addData("auto.complete", auto.isComplete());
                telemetry.addData("auto.outcome", auto.getOutcome());
            });
        } catch (RuntimeException failure) {
            throw CleanupActions.attemptAllAfterFailure(failure, driveStopOwner::stop);
        }
    }
}
```

</details>
<details>
<summary>BasicDriveStopOwner.java — STOP-before-START software-zero owner</summary>

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveStopOwner.java -->
```java
// docs: STOP is idempotent and reaches the sink boundary even when no drive Task ever starts.
    @Override
    public void stop() {
        if (stopped) {
            return;
        }
        stopped = true;
        driveSink.stop();
    }
```

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveStopOwner.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.ftc.RobotProgram;

/**
 * Stable lifecycle owner that makes one terminal software-zero attempt for an Auto drive sink.
 *
 * <p>Its output heartbeat intentionally performs no sink update or drive command; the active
 * {@code DriveTasks} leaf remains the only command owner. Register this owner immediately after
 * acquiring the sink so STOP-before-START and an Auto that ends before its drive phase still call
 * the sink's immediate {@link DriveCommandSink#stop()} boundary.</p>
 */
final class BasicDriveStopOwner implements RobotProgram.Output {

    private final DriveCommandSink driveSink;
    private boolean stopped;

    BasicDriveStopOwner(DriveCommandSink driveSink) {
        this.driveSink = Objects.requireNonNull(driveSink, "driveSink");
    }

    /** No-op by design: this lifecycle owner never competes with the exclusive drive Task. */
    @Override
    public void update(LoopClock clock) {
        // DriveTasks owns every active update and command.
    }

    /** Requests terminal software zero at most once across failure cleanup and managed STOP. */
    @Override
    public void stop() {
        if (stopped) {
            return;
        }
        stopped = true;
        driveSink.stop();
    }
}
```

</details>
<details>
<summary>BasicAutoRoutinesTest.java — order, outcomes, drive stop, cancellation</summary>

<!-- source-file: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutinesTest.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;

/** Verifies that the two teaching Autos compose semantic capabilities without hiding outcomes. */
public final class BasicAutoRoutinesTest {

    @Test
    public void guideRunsTheDocumentedCommandGroupsInOrder() {
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        Task root = BasicAutoRoutines.guide(lift, claw);
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(1, lift.homeRequests);
        assertEquals(
                Arrays.asList(
                        BasicLift.Height.HIGH,
                        BasicLift.Height.LOW,
                        BasicLift.Height.STOWED),
                lift.moveHeights);
        assertTrue(lift.startedHeights.isEmpty());
        root.start(time.clock());

        finishSuccessfullyAndStartNext(lift.lastHome, root, time);
        assertEquals(Arrays.asList(BasicLift.Height.HIGH), lift.startedHeights);
        assertTrue(claw.states.isEmpty());

        finishSuccessfullyAndStartNext(lift.moveTask(0), root, time);
        assertEquals(
                Arrays.asList(BasicLift.Height.HIGH, BasicLift.Height.LOW),
                lift.startedHeights);
        assertEquals(Arrays.asList(BasicClaw.State.CLOSED), claw.states);

        finishSuccessfullyAndStartNext(lift.moveTask(1), root, time);
        root.update(time.nextCycle(0.25));
        assertEquals(2, lift.startedHeights.size());
        assertEquals(1, claw.states.size());

        root.update(time.nextCycle(0.26));
        assertEquals(
                Arrays.asList(
                        BasicLift.Height.HIGH,
                        BasicLift.Height.LOW,
                        BasicLift.Height.STOWED),
                lift.startedHeights);
        assertEquals(
                Arrays.asList(BasicClaw.State.CLOSED, BasicClaw.State.OPEN),
                claw.states);

        finishAndUpdate(lift.moveTask(2), TaskOutcome.SUCCESS, root, time, 0.02);
        assertEquals(TaskOutcome.SUCCESS, root.getOutcome());
    }

    @Test
    public void guidePreservesEveryNonSuccessPrerequisiteWithoutLaterMotion() {
        for (TaskOutcome outcome : Arrays.asList(
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN)) {
            RecordingLift lift = new RecordingLift();
            RecordingClaw claw = new RecordingClaw();
            Task root = BasicAutoRoutines.guide(lift, claw);
            ManualLoopClock time = new ManualLoopClock();
            root.start(time.clock());

            finishAndUpdate(lift.lastHome, outcome, root, time, 0.02);

            assertEquals(outcome, root.getOutcome());
            assertEquals(
                    Arrays.asList(
                            BasicLift.Height.HIGH,
                            BasicLift.Height.LOW,
                            BasicLift.Height.STOWED),
                    lift.moveHeights);
            assertTrue(lift.startedHeights.isEmpty());
            assertTrue(claw.states.isEmpty());
        }
    }

    @Test
    public void guidePreservesAbnormalHighAndLowDeadlineOutcomes() {
        for (TaskOutcome outcome : Arrays.asList(
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN)) {
            RecordingLift highLift = new RecordingLift();
            RecordingClaw highClaw = new RecordingClaw();
            Task highRoot = BasicAutoRoutines.guide(highLift, highClaw);
            ManualLoopClock highTime = new ManualLoopClock();
            highRoot.start(highTime.clock());
            finishSuccessfullyAndStartNext(highLift.lastHome, highRoot, highTime);

            finishAndUpdate(highLift.moveTask(0), outcome, highRoot, highTime, 0.02);

            assertEquals(outcome, highRoot.getOutcome());
            assertEquals(Arrays.asList(BasicLift.Height.HIGH), highLift.startedHeights);
            assertTrue(highClaw.states.isEmpty());

            RecordingLift lowLift = new RecordingLift();
            RecordingClaw lowClaw = new RecordingClaw();
            Task lowRoot = BasicAutoRoutines.guide(lowLift, lowClaw);
            ManualLoopClock lowTime = new ManualLoopClock();
            lowRoot.start(lowTime.clock());
            finishSuccessfullyAndStartNext(lowLift.lastHome, lowRoot, lowTime);
            finishSuccessfullyAndStartNext(lowLift.moveTask(0), lowRoot, lowTime);

            finishAndUpdate(lowLift.moveTask(1), outcome, lowRoot, lowTime, 0.02);

            assertEquals(outcome, lowRoot.getOutcome());
            assertEquals(
                    Arrays.asList(BasicLift.Height.HIGH, BasicLift.Height.LOW),
                    lowLift.startedHeights);
            assertEquals(Arrays.asList(BasicClaw.State.CLOSED), lowClaw.states);
        }
    }

    @Test
    public void guidePreservesAbnormalFinalStowDeadlineOutcomes() {
        for (TaskOutcome outcome : Arrays.asList(
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN)) {
            RecordingLift lift = new RecordingLift();
            RecordingClaw claw = new RecordingClaw();
            Task root = BasicAutoRoutines.guide(lift, claw);
            ManualLoopClock time = new ManualLoopClock();
            root.start(time.clock());
            finishSuccessfullyAndStartNext(lift.lastHome, root, time);
            finishSuccessfullyAndStartNext(lift.moveTask(0), root, time);
            finishSuccessfullyAndStartNext(lift.moveTask(1), root, time);
            root.update(time.nextCycle(0.51));

            assertEquals(BasicLift.Height.STOWED, lift.startedHeights.get(2));
            assertEquals(BasicClaw.State.OPEN, claw.states.get(1));
            finishAndUpdate(lift.moveTask(2), outcome, root, time, 0.02);

            assertEquals(outcome, root.getOutcome());
        }
    }

    @Test
    public void directCancellationStopsTheActiveGuidePrerequisiteWithoutLaterMotion() {

        RecordingLift cancelledLift = new RecordingLift();
        RecordingClaw cancelledClaw = new RecordingClaw();
        Task cancelled = BasicAutoRoutines.guide(cancelledLift, cancelledClaw);
        ManualLoopClock cancelledTime = new ManualLoopClock();
        cancelled.start(cancelledTime.clock());
        finishSuccessfullyAndStartNext(cancelledLift.lastHome, cancelled, cancelledTime);

        cancelled.cancel();
        cancelled.cancel();
        assertEquals(TaskOutcome.CANCELLED, cancelled.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, cancelledLift.moveTask(0).getOutcome());
        assertEquals(Arrays.asList(BasicLift.Height.HIGH), cancelledLift.startedHeights);
        assertTrue(cancelledClaw.states.isEmpty());
    }

    @Test
    public void completeAutoCommandsAndStopsItsExclusiveTimedDriveBeforeRelease() {
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        RecordingDriveSink drive = new RecordingDriveSink();
        Task root = BasicRobotAutoRoutines.complete(lift, claw, drive);
        ManualLoopClock time = new ManualLoopClock();

        root.start(time.clock());
        finishSuccessfullyAndStartNext(lift.lastHome, root, time);
        assertEquals(Arrays.asList(BasicLift.Height.HIGH), lift.startedHeights);
        assertEquals(Arrays.asList(BasicClaw.State.CLOSED), claw.states);

        finishSuccessfullyAndStartNext(lift.moveTask(0), root, time);
        assertEquals(Arrays.asList("update", "drive"), drive.events);
        assertEquals(0.20, drive.lastCommand.axial, 0.0);
        assertEquals(0.0, drive.lastCommand.lateral, 0.0);
        assertEquals(0.0, drive.lastCommand.omega, 0.0);
        assertEquals(0, drive.stopCount);

        root.update(time.nextCycle(0.75));
        assertEquals(1, drive.stopCount);
        assertEquals("stop", drive.events.get(drive.events.size() - 1));
        assertEquals(
                Arrays.asList(BasicLift.Height.HIGH, BasicLift.Height.STOWED),
                lift.startedHeights);
        assertEquals(
                Arrays.asList(BasicClaw.State.CLOSED, BasicClaw.State.OPEN),
                claw.states);

        finishAndUpdate(lift.moveTask(1), TaskOutcome.SUCCESS, root, time, 0.02);
        assertEquals(TaskOutcome.SUCCESS, root.getOutcome());
        assertEquals(1, drive.stopCount);
    }

    @Test
    public void stopOwnerZerosOnceEvenWhenTheDriveTaskNeverStarts() {
        RecordingDriveSink drive = new RecordingDriveSink();
        BasicDriveStopOwner owner = new BasicDriveStopOwner(drive);
        ManualLoopClock time = new ManualLoopClock();

        owner.update(time.clock());
        assertTrue(drive.events.isEmpty());

        owner.stop();
        owner.stop();

        assertEquals(Arrays.asList("stop"), drive.events);
        assertEquals(0, drive.driveCount);
        assertEquals(1, drive.stopCount);
    }

    @Test
    public void carryFailureBeforeDrivePreservesOutcomeAndLifecycleStop() {
        for (TaskOutcome outcome : Arrays.asList(
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN)) {
            RecordingLift lift = new RecordingLift();
            RecordingClaw claw = new RecordingClaw();
            RecordingDriveSink drive = new RecordingDriveSink();
            BasicDriveStopOwner owner = new BasicDriveStopOwner(drive);
            Task root = BasicRobotAutoRoutines.complete(lift, claw, drive);
            ManualLoopClock time = new ManualLoopClock();
            root.start(time.clock());
            finishSuccessfullyAndStartNext(lift.lastHome, root, time);

            finishAndUpdate(lift.moveTask(0), outcome, root, time, 0.02);

            assertEquals(outcome, root.getOutcome());
            assertEquals(0, drive.driveCount);
            assertEquals(0, drive.stopCount);
            assertEquals(Arrays.asList(BasicClaw.State.CLOSED), claw.states);

            // This is the managed STOP path after Auto ended before its drive phase.
            owner.stop();
            owner.stop();
            assertEquals(1, drive.stopCount);
        }
    }

    @Test
    public void completeAutoPreservesAbnormalFinalStowDeadlineOutcomes() {
        for (TaskOutcome outcome : Arrays.asList(
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN)) {
            RecordingLift lift = new RecordingLift();
            RecordingClaw claw = new RecordingClaw();
            RecordingDriveSink drive = new RecordingDriveSink();
            Task root = BasicRobotAutoRoutines.complete(lift, claw, drive);
            ManualLoopClock time = new ManualLoopClock();
            root.start(time.clock());
            finishSuccessfullyAndStartNext(lift.lastHome, root, time);
            finishSuccessfullyAndStartNext(lift.moveTask(0), root, time);
            root.update(time.nextCycle(0.75));

            assertEquals(BasicLift.Height.STOWED, lift.startedHeights.get(1));
            assertEquals(BasicClaw.State.OPEN, claw.states.get(1));
            finishAndUpdate(lift.moveTask(1), outcome, root, time, 0.02);

            assertEquals(outcome, root.getOutcome());
            assertEquals(1, drive.stopCount);
        }
    }

    @Test
    public void cancellingCompleteAutoStopsActiveDriveAndStartsNoReleasePhase() {
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        RecordingDriveSink drive = new RecordingDriveSink();
        Task root = BasicRobotAutoRoutines.complete(lift, claw, drive);
        ManualLoopClock time = new ManualLoopClock();

        root.start(time.clock());
        finishSuccessfullyAndStartNext(lift.lastHome, root, time);
        finishSuccessfullyAndStartNext(lift.moveTask(0), root, time);
        assertEquals(1, drive.driveCount);

        root.cancel();
        root.cancel();

        assertEquals(TaskOutcome.CANCELLED, root.getOutcome());
        assertEquals(1, drive.stopCount);
        assertEquals(Arrays.asList(BasicLift.Height.HIGH), lift.startedHeights);
        assertEquals(Arrays.asList(BasicClaw.State.CLOSED), claw.states);
    }

    @Test
    public void eachFactoryCallBuildsFreshRootAndHomeTasks() {
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        RecordingDriveSink drive = new RecordingDriveSink();

        Task first = BasicRobotAutoRoutines.complete(lift, claw, drive);
        ControllableTask firstHome = lift.lastHome;
        Task second = BasicRobotAutoRoutines.complete(lift, claw, drive);

        assertNotSame(first, second);
        assertNotSame(firstHome, lift.lastHome);
        assertEquals(2, lift.homeRequests);
    }

    private static void finishAndUpdate(ControllableTask task,
                                        TaskOutcome outcome,
                                        Task root,
                                        ManualLoopClock time,
                                        double dtSec) {
        task.finish(outcome);
        root.update(time.nextCycle(dtSec));
    }

    /** Exact-success sequence handoff starts the next fixed child in the same lifecycle call. */
    private static void finishSuccessfullyAndStartNext(ControllableTask task,
                                                        Task root,
                                                        ManualLoopClock time) {
        finishAndUpdate(task, TaskOutcome.SUCCESS, root, time, 0.02);
    }

    private static final class RecordingLift implements BasicLift {
        private final List<Height> moveHeights = new ArrayList<Height>();
        private final List<Height> startedHeights = new ArrayList<Height>();
        private final List<ControllableTask> moveTasks = new ArrayList<ControllableTask>();
        private int homeRequests;
        private ControllableTask lastHome;

        @Override
        public void setHeight(Height height) {
            throw new AssertionError("Basic Auto should use moveTo(Height)");
        }

        @Override
        public Task moveTo(Height height) {
            moveHeights.add(height);
            ControllableTask move = new ControllableTask(
                    "move-" + height,
                    () -> startedHeights.add(height));
            moveTasks.add(move);
            return move;
        }

        @Override
        public Task home() {
            homeRequests++;
            lastHome = new ControllableTask("home", null);
            return lastHome;
        }

        @Override
        public Status status() {
            return new Status(Height.STOWED, 0.0, 0.0, true, true);
        }

        private ControllableTask moveTask(int index) {
            return moveTasks.get(index);
        }
    }

    private static final class RecordingClaw implements BasicClaw {
        private final List<State> states = new ArrayList<State>();

        @Override
        public void setState(State state) {
            states.add(state);
        }

        @Override
        public Status status() {
            State state = states.isEmpty() ? State.CLOSED : states.get(states.size() - 1);
            return new Status(state, 0.0);
        }
    }

    private static final class RecordingDriveSink implements DriveCommandSink {
        private final List<String> events = new ArrayList<String>();
        private int driveCount;
        private int stopCount;
        private DriveSignal lastCommand;

        @Override
        public void update(LoopClock clock) {
            events.add("update");
        }

        @Override
        public void drive(DriveSignal signal) {
            driveCount++;
            lastCommand = signal;
            events.add("drive");
        }

        @Override
        public void stop() {
            stopCount++;
            lastCommand = null;
            events.add("stop");
        }
    }

    private static final class ControllableTask implements Task {
        private final String name;
        private final Runnable onStart;
        private boolean startAttempted;
        private boolean started;
        private boolean complete;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private ControllableTask(String name, Runnable onStart) {
            this.name = name;
            this.onStart = onStart;
        }

        @Override
        public void start(LoopClock clock) {
            if (startAttempted) {
                throw new IllegalStateException(name + " is single-use");
            }
            startAttempted = true;
            started = true;
            if (onStart != null) {
                onStart.run();
            }
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException(name + " updated before start");
            }
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            outcome = TaskOutcome.CANCELLED;
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? outcome : TaskOutcome.NOT_DONE;
        }

        @Override
        public String getDebugName() {
            return name;
        }

        private void finish(TaskOutcome terminalOutcome) {
            if (!started || complete) {
                throw new IllegalStateException(name + " cannot finish now");
            }
            complete = true;
            outcome = terminalOutcome;
        }
    }
}
```

</details>

**Run:** Prove the individual behavior factories first:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests "edu.ftcsushi.robots.examples.basicmechanisms.BasicAutoRoutinesTest"
```

**Expect:** Home and HIGH gate later work; lift-authoritative LOW+CLOSED runs together; the hold has
an exact success gate; timed drive stops before release; cancellation starts no later phase.

**Software checkpoint:** `BasicAutoRoutinesTest` is green for exact SUCCESS, TIMEOUT, CANCELLED, and
UNKNOWN propagation, STOP-before-START, prerequisite failure, and drive zero. In each Auto host,
inspect root `complete` and `outcome`; mechanism observations do not replace the root outcome.

**Physical gate:** Run the mechanism-only host only after gates 3–5. Qualify `BasicDriveAuto`
separately: keep lift/claw absent, verify its `@Disabled` and drive-only permission gate, check the
wheel pattern raised, then mark a clear floor and run its 0.20 source request for 0.75 seconds with
STOP staffed. The 0.25 profile scale makes the exact straight wheel command 0.05. At natural
terminal, accept only when direction repeats, physical stopping is observed, and root telemetry
matches the outcome. In a separate operator-STOP check, require prompt physical stopping; the
lifecycle does not promise a final post-STOP telemetry frame. `BasicDriveStopOwner` only attempts
the sink's software STOP boundary; it does not prove physical zero.

**What to notice**

- Fixed child Tasks are constructed eagerly, but `Tasks.sequence(...)` starts each later child only
  after the prior child reports exact `SUCCESS`.
- `parallelDeadline(...)` makes the lift outcome authoritative while one-cycle claw intent persists.
- The stop-only owner never competes with the active drive Task.
- Fixed-time drive teaches exclusive ownership and stop, not a field path.

**Key APIs**

- [`BasicAutoRoutines.guide(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutines.html>) — mechanism-only command-group factory.
- [`BasicMechanismsAuto`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicMechanismsAuto.html>) / [`BasicDriveAuto`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveAuto.html>) / [`BasicDriveStopOwner`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicDriveStopOwner.java>) — separate fixtures and STOP lifecycle owner.
- [`Tasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/Tasks.html>) — exact-success sequence, parallel deadline, waits, and explicit start-time construction.
- [`TaskOutcome`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/TaskOutcome.html>) — exact SUCCESS, TIMEOUT, CANCELLED, and UNKNOWN terminal outcomes.

**If it fails:** Keep full Auto disabled and repair the smallest failed phase. Never continue after
a failed prerequisite.

**Advance when:** Every phase succeeds and cancels safely alone, including drive zero before later
work.

## 7. Test end-to-end Auto

**Outcome:** The complete host runs home → carry-ready → short exclusive drive → release/stow, with
terminal cleanup and no hidden cancellation continuation.

**Files:** Add `BasicRobotAutoRoutines.java`, `BasicRobotAuto.java`, and
`BasicRobotScenarioTest.java`.

### Critical code

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotAuto.java -->
```java
// docs: Give the direct sink to the full-robot factory, then register its retained root once.
        // This sink belongs only to DriveTasks; do not also pass it to program.drive(...).
        DriveCommandSink autoDrive = FtcDrives.mecanum(hardwareMap, driveProfile.drive);
        BasicDriveStopOwner driveStopOwner = new BasicDriveStopOwner(autoDrive);
        try {
            // A stop-only owner covers STOP even if a lift prerequisite ends before drive starts.
            program.output(driveStopOwner);
            Task auto = BasicRobotAutoRoutines.complete(lift, claw, autoDrive);
            program.rootTask(auto);
```

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotAuto.java -->
```java
// docs: Read both fields; complete alone does not distinguish success, timeout, or cancellation.
                telemetry.addData("auto.complete", auto.isComplete());
                telemetry.addData("auto.outcome", auto.getOutcome());
```
<details>
<summary>BasicRobotAutoRoutines.java — complete robot Auto policy</summary>

<!-- annotated-source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotAutoRoutines.java -->
```java
// docs: Side-effect-free capability Task factories build eagerly; exact success starts later motion.
        return Tasks.sequence(
                requiredLift.home(),
                // The lift deadline preserves its exact outcome; CLOSED starts concurrently and
                // persists after its one-cycle Task completes.
                Tasks.parallelDeadline(
                        requiredLift.moveTo(BasicLift.Height.HIGH),
                        requestClaw(requiredClaw, BasicClaw.State.CLOSED)),
                DriveTasks.driveExclusivelyForSeconds(
                        requiredDrive,
                        new DriveSignal(FORWARD_REQUEST, 0.0, 0.0),
                        FORWARD_DURATION_SEC),
                Tasks.parallelDeadline(
                        requiredLift.moveTo(BasicLift.Height.STOWED),
                        requestClaw(requiredClaw, BasicClaw.State.OPEN)));
```

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotAutoRoutines.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveTasks;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;

/** Full-robot Auto policy composed from semantic mechanisms and one exclusive drive sink. */
public final class BasicRobotAutoRoutines {

    /** Robot-centric axial request before {@link BasicDriveProfile} mixer scaling. */
    private static final double FORWARD_REQUEST = 0.20;

    /** Positive duration for which the axial request remains observable, in seconds. */
    private static final double FORWARD_DURATION_SEC = 0.75;

    private BasicRobotAutoRoutines() {
        // Static robot-owned routine factories.
    }

    /**
     * Builds a complete basic Auto that homes, carries, drives forward briefly, releases, and
     * stows.
     *
     * <p>The direct drive sink is exclusive to the timed {@link DriveTasks} phase; callers must
     * not also register it through {@code program.drive(...)}. Lift feedback gates every later
     * motion. The fixed graph calls these side-effect-free capability Task factories eagerly, but
     * normal cancellation stops the active task graph without starting a later phase.</p>
     *
     * @return fresh single-use full-robot root Task
     */
    public static Task complete(BasicLift lift,
                                BasicClaw claw,
                                DriveCommandSink driveSink) {
        BasicLift requiredLift = Objects.requireNonNull(lift, "lift");
        BasicClaw requiredClaw = Objects.requireNonNull(claw, "claw");
        DriveCommandSink requiredDrive = Objects.requireNonNull(driveSink, "driveSink");

        return Tasks.sequence(
                requiredLift.home(),
                // The lift deadline preserves its exact outcome; CLOSED starts concurrently and
                // persists after its one-cycle Task completes.
                Tasks.parallelDeadline(
                        requiredLift.moveTo(BasicLift.Height.HIGH),
                        requestClaw(requiredClaw, BasicClaw.State.CLOSED)),
                DriveTasks.driveExclusivelyForSeconds(
                        requiredDrive,
                        new DriveSignal(FORWARD_REQUEST, 0.0, 0.0),
                        FORWARD_DURATION_SEC),
                Tasks.parallelDeadline(
                        requiredLift.moveTo(BasicLift.Height.STOWED),
                        requestClaw(requiredClaw, BasicClaw.State.OPEN)));
    }

    /** Creates a fresh one-cycle semantic command Task. */
    private static Task requestClaw(BasicClaw claw, BasicClaw.State state) {
        return Tasks.runOnce(() -> claw.setState(state));
    }
}
```

</details>
<details>
<summary>BasicRobotAuto.java — complete disabled Auto</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotAuto.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;

/** Complete basic Auto host with one exclusive fixed-time drive phase. */
@Autonomous(name = "FW Basic 7: Robot Auto", group = "FW Examples")
@Disabled
public final class BasicRobotAuto extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicDriveProfile driveProfile = BasicDriveProfile.current();
        BasicLiftProfile liftProfile = BasicLiftProfile.current();
        BasicClawProfile clawProfile = BasicClawProfile.current();
        BasicDriveProfile.requireMotionAllowed(driveProfile, "Basic Robot Auto");
        BasicLiftProfile.requireMotionAllowed(liftProfile, "Basic Robot Auto");
        BasicClawProfile.requireMotionAllowed(clawProfile, "Basic Robot Auto");
        BasicHardwareOwnership.requireDistinctDriveAndLiftMotors(
                driveProfile.drive, liftProfile.lift, "Basic Robot Auto");

        BasicLiftMechanism lift = program.output(
                new BasicLiftMechanism(hardwareMap, liftProfile.lift));
        BasicClawMechanism claw = program.output(
                new BasicClawMechanism(hardwareMap, clawProfile.claw));

        // This sink belongs only to DriveTasks; do not also pass it to program.drive(...).
        DriveCommandSink autoDrive = FtcDrives.mecanum(hardwareMap, driveProfile.drive);
        BasicDriveStopOwner driveStopOwner = new BasicDriveStopOwner(autoDrive);
        try {
            // A stop-only owner covers STOP even if a lift prerequisite ends before drive starts.
            program.output(driveStopOwner);
            Task auto = BasicRobotAutoRoutines.complete(lift, claw, autoDrive);
            program.rootTask(auto);
            program.presenter((clock, telemetry) -> {
                BasicLift.Status liftStatus = lift.status();
                BasicClaw.Status clawStatus = claw.status();
                telemetry.addData("lift", "%s %.2f/%.2f in referenced=%s atTarget=%s",
                        liftStatus.requestedHeight,
                        liftStatus.measuredPositionIn,
                        liftStatus.requestedPositionIn,
                        liftStatus.referenced,
                        liftStatus.atTarget);
                telemetry.addData("claw", "%s command=%.2f",
                        clawStatus.requestedState, clawStatus.appliedPosition);
                telemetry.addData("auto.complete", auto.isComplete());
                telemetry.addData("auto.outcome", auto.getOutcome());
            });
        } catch (RuntimeException failure) {
            // Preserve the failure while still attempting the acquired sink's software STOP.
            throw CleanupActions.attemptAllAfterFailure(failure, driveStopOwner::stop);
        }
    }
}
```

</details>
<details>
<summary>BasicRobotScenarioTest.java — real Plants and configured drive without a Control Hub</summary>

<!-- source-file: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotScenarioTest.java -->
```java
package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.nio.charset.StandardCharsets;

import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Runs the complete basic robot graph without a Control Hub and checks its public safety gates. */
public final class BasicRobotScenarioTest {

    @Test
    public void capabilityProfilesAreFreshIndependentAndBlockEveryKindOfMotion() {
        BasicDriveProfile firstDrive = BasicDriveProfile.current();
        BasicDriveProfile secondDrive = BasicDriveProfile.current();
        BasicLiftProfile firstLift = BasicLiftProfile.current();
        BasicLiftProfile secondLift = BasicLiftProfile.current();
        BasicClawProfile firstClaw = BasicClawProfile.current();
        BasicClawProfile secondClaw = BasicClawProfile.current();

        assertNotSame(firstDrive, secondDrive);
        assertNotSame(firstDrive.drive, secondDrive.drive);
        assertNotSame(firstDrive.drive.wiring, secondDrive.drive.wiring);
        assertNotSame(firstDrive.drive.drivebase, secondDrive.drive.drivebase);
        assertNotSame(firstLift, secondLift);
        assertNotSame(firstLift.lift, secondLift.lift);
        assertNotSame(firstClaw, secondClaw);
        assertNotSame(firstClaw.claw, secondClaw.claw);

        assertFalse(firstDrive.allowDriveMotion);
        assertFalse(firstLift.allowLiftMotion);
        assertFalse(firstClaw.allowClawMotion);

        firstDrive.drive.wiring.frontLeftName = "changed";
        firstLift.lift.lowHeightIn = 9.0;
        firstClaw.claw.openPosition = 0.95;
        assertEquals("frontLeftMotor", secondDrive.drive.wiring.frontLeftName);
        assertEquals(4.0, secondLift.lift.lowHeightIn, 0.0);
        assertEquals(0.70, secondClaw.claw.openPosition, 0.0);
    }

    @Test
    public void publicTeachingHostsStayDisabledAndMotionGatesRunBeforeLookup() {
        Class<?>[] hostTypes = {
                BasicLiftTeleOp.class,
                BasicClawTeleOp.class,
                BasicMechanismsAuto.class,
                BasicRobotTeleOp.class,
                BasicDriveAuto.class,
                BasicRobotAuto.class
        };
        String[] firstPermissions = {
                "BasicDriveProfile.allowDriveMotion",
                "BasicDriveProfile.allowDriveMotion",
                "BasicLiftProfile.allowLiftMotion",
                "BasicDriveProfile.allowDriveMotion",
                "BasicDriveProfile.allowDriveMotion",
                "BasicDriveProfile.allowDriveMotion"
        };

        for (int index = 0; index < hostTypes.length; index++) {
            Class<?> type = hostTypes[index];
            assertTrue(Modifier.isPublic(type.getModifiers()));
            assertTrue(Modifier.isFinal(type.getModifiers()));
            assertNotNull(type.getAnnotation(Disabled.class));

            Constructor<?>[] constructors = type.getDeclaredConstructors();
            assertEquals(1, constructors.length);
            assertTrue(Modifier.isPublic(constructors[0].getModifiers()));
            assertEquals(0, constructors[0].getParameterTypes().length);

            FtcTestHardware hardware = new FtcTestHardware();
            FtcRobotOpMode mode = newHost(type);
            prepare(mode, hardware);
            IllegalStateException failure = expectIllegalState(mode::init);
            assertTrue(failure.getMessage().contains(firstPermissions[index]));
            assertEquals(0, hardware.lookupCalls());
        }
    }

    @Test
    public void liftAndDriveCannotSilentlyOwnTheSameMotorKey() {
        BasicDriveProfile driveProfile = BasicDriveProfile.current();
        BasicLiftProfile liftProfile = BasicLiftProfile.current();
        liftProfile.lift.motorName = "  sharedMotor  ";
        driveProfile.drive.wiring.backRightName = "sharedMotor";

        IllegalStateException failure = expectIllegalState(() ->
                BasicHardwareOwnership.requireDistinctDriveAndLiftMotors(
                        driveProfile.drive, liftProfile.lift, "test mode"));

        assertTrue(failure.getMessage().contains("BasicLiftProfile.lift.motorName"));
        assertTrue(failure.getMessage().contains(
                "BasicDriveProfile.drive.wiring.backRightName"));
        assertTrue(failure.getMessage().contains("\"sharedMotor\""));
    }

    @Test
    public void focusedSourceClosuresStayIndependentAndAutosExposeRootOutcomeTelemetry() {
        assertClassDoesNotReference(
                BasicLiftTeleOp.class, "BasicClaw", "BasicClawProfile", "BasicClawControls");
        assertClassDoesNotReference(
                BasicClawTeleOp.class, "BasicLift", "BasicLiftProfile", "BasicLiftControls");
        assertClassDoesNotReference(
                BasicMechanismsAuto.class, "BasicDrive", "FtcDrives", "DriveCommandSink");
        assertClassDoesNotReference(
                BasicDriveAuto.class, "BasicLift", "BasicClaw", "BasicRobotAutoRoutines");
        assertClassDoesNotReference(
                BasicAutoRoutines.class, "BasicDrive", "DriveCommandSink", "DriveTasks");

        assertClassDoesNotReference(
                BasicDriveProfile.class, "BasicLiftProfile", "BasicClawProfile");
        assertClassDoesNotReference(
                BasicLiftProfile.class, "BasicDriveProfile", "BasicClawProfile");
        assertClassDoesNotReference(
                BasicClawProfile.class, "BasicDriveProfile", "BasicLiftProfile");
        assertClassDoesNotReference(
                BasicDriveControls.class, "BasicLiftControls", "BasicClawControls");
        assertClassDoesNotReference(
                BasicLiftControls.class, "BasicDriveControls", "BasicClawControls");
        assertClassDoesNotReference(
                BasicClawControls.class, "BasicDriveControls", "BasicLiftControls");

        assertClassReferences(BasicMechanismsAuto.class, "auto.complete", "auto.outcome");
        assertClassReferences(
                BasicDriveAuto.class,
                "BasicDriveStopOwner",
                "auto.complete",
                "auto.outcome");
        assertClassReferences(
                BasicRobotAuto.class,
                "BasicDriveStopOwner",
                "auto.complete",
                "auto.outcome");
    }

    @Test
    public void completeAutoRunsAgainstRealPlantsAndStopsDriveBeforeSuccessfulFinish() {
        Scenario f = new Scenario();
        Task root = BasicRobotAutoRoutines.complete(f.lift, f.claw, f.drive);

        beginAndReachTimedDrive(root, f);
        assertConservativeForwardRequestWasScaledByTheRealDrivebase(f);

        heartbeat(root, f, 0.75);
        assertAllDriveMotorsStopped(f);
        heartbeat(root, f, 0.02);
        assertEquals(BasicClaw.State.OPEN, f.claw.status().requestedState);
        assertEquals(BasicLift.Height.STOWED, f.lift.status().requestedHeight);

        f.liftMotor.setCurrentPositionTicks(0);
        heartbeat(root, f, 0.02);
        heartbeat(root, f, 0.02);

        assertEquals(TaskOutcome.SUCCESS, root.getOutcome());
        assertAllDriveMotorsStopped(f);
        assertEquals(f.clawProfile.claw.openPosition, f.clawServo.position(), 0.0);
    }

    @Test
    public void cancellingTheCompleteAutoStopsDriveAndStartsNoReleaseOrStow() {
        Scenario f = new Scenario();
        Task root = BasicRobotAutoRoutines.complete(f.lift, f.claw, f.drive);

        beginAndReachTimedDrive(root, f);
        assertConservativeForwardRequestWasScaledByTheRealDrivebase(f);

        root.cancel();
        root.cancel();

        assertEquals(TaskOutcome.CANCELLED, root.getOutcome());
        assertAllDriveMotorsStopped(f);
        assertEquals(BasicClaw.State.CLOSED, f.claw.status().requestedState);
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight);
    }

    private static void beginAndReachTimedDrive(Task root, Scenario f) {
        root.start(f.time.clock());
        f.lift.update(f.time.clock());
        f.claw.update(f.time.clock());

        f.bottomSwitch.setHigh(false);
        heartbeat(root, f, 0.01);
        heartbeat(root, f, 0.03);
        heartbeat(root, f, 0.02);
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight);
        assertEquals(BasicClaw.State.CLOSED, f.claw.status().requestedState);

        f.liftMotor.setCurrentPositionTicks(
                (int) Math.round(
                        f.liftProfile.lift.highHeightIn
                                * f.liftProfile.lift.ticksPerIn));
        heartbeat(root, f, 0.02);
        heartbeat(root, f, 0.02);
        heartbeat(root, f, 0.02);
        assertFalse(root.isComplete());
    }

    /** Mimics the managed ACTIVE order: Task graph first, then mechanism output owners. */
    private static void heartbeat(Task root, Scenario f, double dtSec) {
        f.time.nextCycle(dtSec);
        root.update(f.time.clock());
        f.lift.update(f.time.clock());
        f.claw.update(f.time.clock());
    }

    private static void assertConservativeForwardRequestWasScaledByTheRealDrivebase(Scenario f) {
        // The routine requests 0.20; the reviewed profile separately limits max axial to 0.25.
        double expectedWheelCommand = 0.20 * f.driveProfile.drive.drivebase.maxAxial;
        assertEquals(0.05, expectedWheelCommand, 0.0);
        assertEquals(expectedWheelCommand, f.frontLeft.power(), 0.0);
        assertEquals(expectedWheelCommand, f.frontRight.power(), 0.0);
        assertEquals(expectedWheelCommand, f.backLeft.power(), 0.0);
        assertEquals(expectedWheelCommand, f.backRight.power(), 0.0);
    }

    private static void assertAllDriveMotorsStopped(Scenario f) {
        assertEquals(0.0, f.frontLeft.power(), 0.0);
        assertEquals(0.0, f.frontRight.power(), 0.0);
        assertEquals(0.0, f.backLeft.power(), 0.0);
        assertEquals(0.0, f.backRight.power(), 0.0);
    }

    private static FtcRobotOpMode newHost(Class<?> type) {
        try {
            return (FtcRobotOpMode) type.getDeclaredConstructor().newInstance();
        } catch (ReflectiveOperationException failure) {
            throw new AssertionError("Could not construct " + type.getName(), failure);
        }
    }

    private static void prepare(FtcRobotOpMode mode, FtcTestHardware hardware) {
        mode.hardwareMap = hardware;
        mode.telemetry = telemetryProxy();
        mode.gamepad1 = new Gamepad();
        mode.gamepad2 = new Gamepad();
        mode.resetRuntime();
    }

    private static Telemetry telemetryProxy() {
        InvocationHandler handler = new InvocationHandler() {
            @Override
            public Object invoke(Object proxy, Method method, Object[] args) {
                if (method.getDeclaringClass() == Object.class) {
                    if ("equals".equals(method.getName())) {
                        return proxy == args[0];
                    }
                    if ("hashCode".equals(method.getName())) {
                        return System.identityHashCode(proxy);
                    }
                    if ("toString".equals(method.getName())) {
                        return "BasicRobotTelemetry";
                    }
                }
                if (method.getReturnType() == boolean.class) {
                    return true;
                }
                if (method.getReturnType().isPrimitive()) {
                    return 0;
                }
                return null;
            }
        };
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                handler);
    }

    private static IllegalStateException expectIllegalState(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalStateException");
            throw new AssertionError("unreachable");
        } catch (IllegalStateException expected) {
            return expected;
        }
    }

    private static void assertClassDoesNotReference(Class<?> type, String... forbiddenText) {
        String classFile = classFileText(type);
        for (String text : forbiddenText) {
            assertFalse(type.getSimpleName() + " must not reference " + text,
                    classFile.contains(text));
        }
    }

    private static void assertClassReferences(Class<?> type, String... requiredText) {
        String classFile = classFileText(type);
        for (String text : requiredText) {
            assertTrue(type.getSimpleName() + " must reference " + text,
                    classFile.contains(text));
        }
    }

    private static String classFileText(Class<?> type) {
        String resource = type.getSimpleName() + ".class";
        try (InputStream input = type.getResourceAsStream(resource)) {
            assertNotNull("Missing compiled class resource for " + type.getName(), input);
            ByteArrayOutputStream bytes = new ByteArrayOutputStream();
            byte[] buffer = new byte[1024];
            int count;
            while ((count = input.read(buffer)) >= 0) {
                bytes.write(buffer, 0, count);
            }
            return new String(bytes.toByteArray(), StandardCharsets.ISO_8859_1);
        } catch (IOException failure) {
            throw new AssertionError("Could not inspect " + type.getName(), failure);
        }
    }

    private static final class Scenario {
        private final BasicDriveProfile driveProfile = BasicDriveProfile.current();
        private final BasicLiftProfile liftProfile = BasicLiftProfile.current();
        private final BasicClawProfile clawProfile = BasicClawProfile.current();
        private final FtcTestHardware hardware = new FtcTestHardware();
        private final FtcTestHardware.MotorProbe liftMotor;
        private final FtcTestHardware.DigitalProbe bottomSwitch;
        private final FtcTestHardware.ServoProbe clawServo;
        private final FtcTestHardware.MotorProbe frontLeft;
        private final FtcTestHardware.MotorProbe frontRight;
        private final FtcTestHardware.MotorProbe backLeft;
        private final FtcTestHardware.MotorProbe backRight;
        private final BasicLiftMechanism lift;
        private final BasicClawMechanism claw;
        private final DriveCommandSink drive;
        private final ManualLoopClock time = new ManualLoopClock();

        private Scenario() {
            liftProfile.lift.motorName = "lift";
            liftProfile.lift.bottomSwitchName = "bottom";
            liftProfile.lift.maximumHeightIn = 10.0;
            liftProfile.lift.ticksPerIn = 10.0;
            liftProfile.lift.toleranceIn = 0.10;
            liftProfile.lift.stowedHeightIn = 0.0;
            liftProfile.lift.lowHeightIn = 4.0;
            liftProfile.lift.highHeightIn = 8.0;
            liftProfile.lift.homingTimeoutSec = 0.20;
            liftProfile.lift.moveTimeoutSec = 0.20;
            clawProfile.claw.servoName = "claw";
            driveProfile.drive.wiring.frontLeftName = "frontLeft";
            driveProfile.drive.wiring.frontRightName = "frontRight";
            driveProfile.drive.wiring.backLeftName = "backLeft";
            driveProfile.drive.wiring.backRightName = "backRight";

            liftMotor = hardware.addMotor(liftProfile.lift.motorName);
            bottomSwitch = hardware.addDigitalInput(liftProfile.lift.bottomSwitchName);
            clawServo = hardware.addServo(clawProfile.claw.servoName);
            frontLeft = hardware.addMotor(driveProfile.drive.wiring.frontLeftName);
            frontRight = hardware.addMotor(driveProfile.drive.wiring.frontRightName);
            backLeft = hardware.addMotor(driveProfile.drive.wiring.backLeftName);
            backRight = hardware.addMotor(driveProfile.drive.wiring.backRightName);

            lift = new BasicLiftMechanism(hardware, liftProfile.lift);
            claw = new BasicClawMechanism(hardware, clawProfile.claw);
            drive = FtcDrives.mecanum(hardware, driveProfile.drive);
        }
    }
}
```

</details>

**Run:**

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests "edu.ftcsushi.robots.examples.basicmechanisms.BasicRobotScenarioTest"
```

**Expect:** Software proves drive reaches zero before release, success ends open/stowed, and
cancellation starts no later phase. Physical evidence must match.

**Software checkpoint:** `BasicRobotScenarioTest` is green; its success trace reaches open/stowed
only after drive zero, and cancellation starts no later phase. The host source registers
`BasicDriveStopOwner` before constructing the root Task; `BasicAutoRoutinesTest` separately proves
that owner zeros the sink on STOP-before-start.

**Physical gate:** After all prior gates pass, review the start pose and clearance, enable all three
permissions only in the team-owned profile, remove `@Disabled` only there, and staff STOP. Accept
only when physical order matches the trace. At natural terminal, require drive zero and root
`complete`/`outcome` telemetry that matches the observed result. During a separate operator-STOP
check, require prompt physical stopping; do not expect a new post-STOP telemetry frame.

**What to notice**

- The Auto sink is not also registered through `program.drive(...)`.
- Managed lifecycle owns root update/cancel, output order, and the stop-only sink owner.
- Timed drive cannot promise a field endpoint.

**Key APIs**

- [`BasicRobotAuto`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotAuto.html>) — complete disabled basic Auto host.
- [`BasicRobotAutoRoutines.complete(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicRobotAutoRoutines.html>) — fresh full-robot factory.
- [`DriveTasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/drive/DriveTasks.html>) — exclusive timed drive with final stop.
- [`RobotProgram.rootTask(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/RobotProgram.html#rootTask(edu.ftcsushi.fw.task.Task)>) — one managed Auto root.

**If it fails:** Press STOP, restore permissions or `@Disabled`, retain the first failed phase, and
return to its gate.

**Advance when:** Software and physical order agree and every terminal path leaves drive stopped.

For real field paths, localization, endpoint truth, and route-time limits, continue to
[Your first Pedro Auto](<First Pedro Auto.md>). Pedro owns that later lesson.

## Complete working slice

### Complete source and owner map

| Gate | Owners introduced |
|---|---|
| Warm-up | First drive host and configured drivetrain |
| 1 | Semantic lift/claw, private Plants, focused Profiles/Controls, ownership check |
| 2 | Lift, claw, and controls software scenarios |
| 3–4 | Focused drive+mechanism hosts and physical evidence |
| 5 | Integrated TeleOp |
| 6 | Auto policy, mechanism Auto, bounded drive Auto, policy tests |
| 7 | Integrated Auto and full robot scenario |

Every manifest file appears exactly once in a per-file disclosure at its first gate.

### Integrated TeleOp and Auto

`BasicRobotTeleOp` and `BasicRobotAuto` are the final composition roots. They reuse the same
profile, mechanisms, controls, and Auto factories already accepted in smaller gates.

## Verify the slice

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests "edu.ftcsushi.robots.examples.basicmechanisms.*"
```

### Proves

The Java compiles; tests cover semantic mappings, configuration, reference/feedback, Task outcomes,
exclusive drive stop, cancellation, and disabled-by-default hosts.

### Does not prove

Software cannot prove wiring, direction, switch polarity, servo endpoints, loading, clearance,
traction, tuning, field position, or physical STOP.

### Next gate

Adopt one gate at a time, retain experiment records, and move to Pedro only for truthful field
paths.
