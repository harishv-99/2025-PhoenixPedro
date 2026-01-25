package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.Drives;
import edu.ftcphoenix.fw.drive.MecanumConfig;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.sensing.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.TagAim;
import edu.ftcphoenix.fw.sensing.TagTarget;
import edu.ftcphoenix.fw.sensing.Tags;
import edu.ftcphoenix.fw.task.TaskRunner;
import edu.ftcphoenix.fw.util.LoopClock;

/**
 * Central robot class for Phoenix-based robots.
 *
 * <p>Beginners should mostly edit <b>this file</b>. TeleOp and Auto OpModes are
 * kept very thin and simply delegate into the methods here.</p>
 *
 * <h2>Responsibilities</h2>
 * <ul>
 *   <li>Wire all hardware once (drive, intake, transfer, shooter, pusher, vision).</li>
 *   <li>Define gamepad mappings in one place via {@link Bindings}.</li>
 *   <li>Own shared logic: auto-aim, shooter velocity, macros, autos.</li>
 *   <li>Expose simple entry points for TeleOp and Auto.</li>
 * </ul>
 */
public final class PhoenixRobotBackup {

    // ---------------------------------------------------------------------
    // Public auto modes: TeleOp/Autos choose from this.
    // ---------------------------------------------------------------------

    public enum AutoMode {
        CLOSE,
        FAR,
        TWELVE_BALL
        // Add more as needed
    }

    // ---------------------------------------------------------------------
    // FTC plumbing (injected from OpModes)
    // ---------------------------------------------------------------------

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private Gamepads gamepads;
    private Bindings bindings;

    // Loop timing
    private final LoopClock clock = new LoopClock();

    // ---------------------------------------------------------------------
    // Drive: mecanum + drive sources (manual + auto-aim)
    // ---------------------------------------------------------------------

    private MecanumDrivebase drivebase;

    /**
     * Base TeleOp drive (sticks + slow mode).
     */
    private DriveSource baseDrive;

    /**
     * Base drive wrapped with TagAim (auto-aim while aim button held).
     */
    private DriveSource driveWithAim;

    /**
     * Last commanded drive signal (for telemetry).
     */
    private DriveSignal lastDriveCmd = new DriveSignal(0.0, 0.0, 0.0);

    // ---------------------------------------------------------------------
    // Mechanisms: transfer, shooter, pusher (all Plants)
    // ---------------------------------------------------------------------

    private Plant transfer;  // buffer / indexer
    private Plant shooter;   // single or paired velocity plant
    private Plant pusher;    // positional servo

    // Task runners for macros (TeleOp) and autonomous scripts
    private final TaskRunner macroRunner = new TaskRunner();
    private final TaskRunner autoRunner = new TaskRunner();

    // ---------------------------------------------------------------------
    // Vision + auto-aim + shooter auto-velocity
    // ---------------------------------------------------------------------

    private AprilTagSensor tagSensor;

    private TagTarget scoringTarget;

    // Example tag IDs we care about (tune for your game)
    private static final Set<Integer> SCORING_TAGS = Set.of(1, 2, 3);

    // TODO(team): optionally add an InterpolatingTable1D here for range->velocity.

    // ---------------------------------------------------------------------
    // Construction
    // ---------------------------------------------------------------------

    public PhoenixRobotBackup() {
        // All wiring happens when initHardware(...) is called.
    }

    // ---------------------------------------------------------------------
    // Initialization entrypoint: call from OpMode.init()
    // ---------------------------------------------------------------------

    /**
     * Wire all hardware and set up the robot.
     *
     * <p>Call this once from TeleOp/Auto {@code init()}.</p>
     */
    public void initHardware(HardwareMap hw,
                             Telemetry tel,
                             Gamepad gp1,
                             Gamepad gp2) {
        this.hardwareMap = hw;
        this.telemetry = tel;

        initInputs(gp1, gp2);
        initVision();
        initDrive();
        initMechanisms();
        wireBindings();

        telemetry.addLine("PhoenixRobot: initHardware complete");
        telemetry.update();
    }

    // ---------------------------------------------------------------------
    // TeleOp entrypoints (thin wrappers from TeleOp OpMode)
    // ---------------------------------------------------------------------

    /**
     * Call once from TeleOp {@code start()}.
     */
    public void startTeleOp(double runtimeSec) {
        clock.reset(runtimeSec);
        macroRunner.clear();
        autoRunner.clear();
        // TODO(team): set any TeleOp-specific initial goals (e.g., shooter off)
    }

    /**
     * Call once per loop from TeleOp {@code loop()}.
     */
    public void loopTeleOp(double runtimeSec) {
        clock.update(runtimeSec);
        double dtSec = clock.dtSec();

        updateInputs(dtSec);
        updateTeleOpDrive();
        updateMechanisms(dtSec);
        updateTeleOpVisionAndShooter(dtSec);
        updateTelemetry();
    }

    // ---------------------------------------------------------------------
    // Auto entrypoints (thin wrappers from Auto OpModes)
    // ---------------------------------------------------------------------

    /**
     * Call once from Auto after {@code waitForStart()}.
     */
    public void startAuto(double runtimeSec, AutoMode mode) {
        clock.reset(runtimeSec);
        macroRunner.clear();
        autoRunner.clear();
        // TODO(team): enqueue auto task sequence(s) into autoRunner based on mode.
    }

    /**
     * Call repeatedly while Auto is active.
     */
    public void loopAuto(double runtimeSec, AutoMode mode) {
        clock.update(runtimeSec);
        double dtSec = clock.dtSec();

        // Inputs are usually minimal in auto, but safe to update.
        updateInputs(dtSec);

        updateAutoScript(dtSec, mode);
        updateTelemetry();
    }

    // ---------------------------------------------------------------------
    // Initialization helpers
    // ---------------------------------------------------------------------

    private void initInputs(Gamepad gp1, Gamepad gp2) {
        gamepads = Gamepads.create(gp1, gp2);
        bindings = new Bindings();
    }

    private void initVision() {
        // AprilTag vision using a single webcam.
        // TODO(team): ensure "Webcam 1" matches your config name.
        tagSensor = Tags.aprilTags(hardwareMap, "Webcam 1");

        scoringTarget = new TagTarget(tagSensor, SCORING_TAGS, 0.3);

        // TODO(team): if you want range->velocity mapping, initialize it here.
    }

    private void initDrive() {
        // 1) Configure drive behavior (scaling + optional smoothing).
        //
        // Start from Phoenix defaults and optionally enable lateral rate limiting.
        // Setting maxLateralRatePerSec > 0 makes strafing less “twitchy”
        // without affecting axial or rotational response.
        MecanumConfig driveCfg = MecanumConfig.defaults();
        driveCfg.maxLateralRatePerSec = 4.0;  // try 0.0 to disable smoothing

        // 2) Wire a standard mecanum drivebase.
        //
        // This assumes the standard inversion pattern:
        //   - left side normal,
        //   - right side inverted.
        // TODO(team): ensure motor names match your Robot Configuration.
        drivebase = Drives.mecanum(
                hardwareMap,
                driveCfg
        );

        // 3) Base TeleOp drive: sticks + slow mode on right bumper.
        //
        // StickDriveSource.teleOpMecanumStandard(...) uses:
        //   - P1 left stick X for lateral,
        //   - P1 left stick Y for axial,
        //   - P1 right stick X for omega,
        //   - StickConfig.defaults() for shaping,
        //   - P1 right bumper as slow-mode at 30% speed.
        baseDrive = GamepadDriveSource.teleOpMecanumStandard(gamepads);

        // 4) Wrap base drive with tag-based auto-aim.
        // While left bumper is held, we auto-aim at SCORING_TAGS.
        driveWithAim = TagAim.teleOpAim(
                baseDrive,
                gamepads.p1().leftBumper(),  // aim button
                scoringTarget
        );
    }

    private void initMechanisms() {
        // TODO(team): ensure these hardware names & inversions match your config.

        // Transfer/indexer: another power plant
        transfer = Actuators.plant(hardwareMap)
                .crServoPair("transferLeft", false,
                        "transferRight", false)
                .power()
                .build();

        // Shooter: paired velocity plant (two motors as one mechanism)
        shooter = Actuators.plant(hardwareMap)
                .motorPair("shooterLeft", false,
            "shooterRight", true)
                .velocity(28.0)
                .build();

        // Pusher: positional servo (0..1)
        pusher = Actuators.plant(hardwareMap)
                .servo("pusher", false)
                .position()
                .build();

        // TODO(team): You can add more plants here (e.g., arm, turret) as needed.
    }

    private void wireBindings() {
        // All button mappings for TeleOp live here.
        // This keeps the rest of the code easy to read and easy to change.

        // Example: shooter simple on/off modes.
        // A = SHOOT, B = STOP
        bindings.onPress(
                gamepads.p1().a(),
                new Runnable() {
                    @Override
                    public void run() {
                        // TODO(team): set shooter target velocity for shooting.
                        // e.g., setShooterRpm(3600.0);
                    }
                }
        );
        bindings.onPress(
                gamepads.p1().b(),
                new Runnable() {
                    @Override
                    public void run() {
                        // TODO(team): stop shooter.
                        // setShooterRpm(0.0);
                    }
                }
        );

        // Example: intake macro on X (e.g., timed pulse in via macroRunner)
        bindings.onPress(
                gamepads.p1().x(),
                new Runnable() {
                    @Override
                    public void run() {
                        // TODO(team): enqueue an intake macro task on macroRunner.
                    }
                }
        );

        // Example: fire one ring on Y (pusher + transfer + shooter ready)
        bindings.onPress(
                gamepads.p1().y(),
                new Runnable() {
                    @Override
                    public void run() {
                        // TODO(team): enqueue "fire one ring" macro (use PlantTasks + TaskRunner).
                    }
                }
        );
    }

    // ---------------------------------------------------------------------
    // Per-loop update helpers
    // ---------------------------------------------------------------------

    private void updateInputs(double dtSec) {
        gamepads.update(dtSec);   // currently a no-op; kept for future filters
        bindings.update(dtSec);   // fires press/release/held/toggle callbacks

        scoringTarget.update();
    }

    private void updateTeleOpDrive() {
        // Use driveWithAim, which automatically overrides rotation while
        // the aim button is held; otherwise this is just baseDrive.
        DriveSignal cmd = driveWithAim.get(clock).clamped();
        drivebase.drive(cmd);
        drivebase.update(clock);  // feeds dtSec for smoothing in MecanumDrivebase
        lastDriveCmd = cmd;
    }

    private void updateMechanisms(double dtSec) {
        // TeleOp macros and long-running tasks own the mechanisms when active.
        macroRunner.update(clock);

        // When no macro is active, fall back to simple manual control.
        if (!macroRunner.hasActiveTask()) {
            // Example: intake on right trigger (0..1)
            double intakeTarget = gamepads.p1().rightTrigger().get();
//            intake.setTarget(intakeTarget);

            // TODO(team): you can also add manual controls for transfer/pusher here.
        }

        // Update all plants once per loop.
        transfer.update(dtSec);
        shooter.update(dtSec);
        pusher.update(dtSec);
    }

    private void updateTeleOpVisionAndShooter(double dtSec) {
        // Optional: auto-set shooter velocity based on distance to tag.
        // This keeps the logic in one place and works for both TeleOp and Auto.

        // Example skeleton (disabled by default):
        //
        // AprilTagObservation obs = tagSensor.best(SCORING_TAGS, 0.3);
        // if (obs.hasTarget) {
        //     double rangeInches = obs.rangeInches;
        //     double desiredRpm   = lookupShooterRpm(rangeInches);
        //     setShooterRpm(desiredRpm);
        // }
        //
        // TODO(team): implement lookupShooterRpm(...) and setShooterRpm(...).
    }

    private void updateAutoScript(double dtSec, AutoMode mode) {
        // Auto scripts are built as Tasks and run via autoRunner.
        // This method is the single place Autos call to advance their script.

        // Example skeleton:
        //
        // if (!autoRunner.hasActiveTask()) {
        //     // Build and enqueue tasks for this mode once, then let them run.
        //     Task script = AutoScripts.buildScriptForMode(mode, drivebase, ...);
        //     autoRunner.enqueue(script);
        // }
        //
        // autoRunner.update(clock);
    }

    private void updateTelemetry() {
        // Minimal but useful telemetry focused on key state.
        telemetry.addLine("PhoenixRobot");

        telemetry.addLine("Drive")
                .addData("axial", lastDriveCmd.axial)
                .addData("lateral", lastDriveCmd.lateral)
                .addData("omega", lastDriveCmd.omega);

        telemetry.addLine("Shooter")
                .addData("target", shooter.getTarget())
                .addData("atSetpoint", shooter.atSetpoint());

        // Example vision info (closest scoring tag, if any)
        if (tagSensor != null) {
            AprilTagObservation obs = tagSensor.best(SCORING_TAGS, 0.3);
            if (obs.hasTarget) {
                telemetry.addLine("Tag")
                        .addData("id", obs.id)
                        .addData("rangeIn", "%.1f", obs.rangeInches)
                        .addData("bearingDeg", "%.1f", Math.toDegrees(obs.bearingRad));
            } else {
                telemetry.addLine("Tag: none or stale");
            }
        }

        telemetry.update();
    }

    // ---------------------------------------------------------------------
    // (Optional) helpers you may want to add later
    // ---------------------------------------------------------------------

    // Example: helper to set shooter in RPM instead of raw rad/s when you
    // wire up a proper conversion / lookup:
    //
    // private void setShooterRpm(double rpm) {
    //     double radPerSec = Units.rpmToRadPerSec(rpm);
    //     shooter.setTarget(radPerSec);
    // }
    //
    // private double lookupShooterRpm(double rangeInches) {
    //     // TODO(team): implement using InterpolatingTable1D or simple math.
    // }
}
