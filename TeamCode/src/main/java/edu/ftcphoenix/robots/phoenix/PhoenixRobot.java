package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import edu.ftcphoenix.fw.adapters.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.adapters.ftc.FtcVision;
import edu.ftcphoenix.fw.debug.DebugSink;
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
public final class PhoenixRobot {
    private final LoopClock clock = new LoopClock();
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepads gamepads;
    private final Bindings bindings = new Bindings();
    private final TaskRunner taskRunnerTeleOp = new TaskRunner();
    private final DebugSink dbg;
    private Shooter shooter;
    private MecanumDrivebase drivebase;
    private DriveSource stickDrive;
    private DriveSource driveWithAim;
    private AprilTagSensor tagSensor;
    private TagTarget scoringTarget;
    private TagAim.Config aimConfig;


    // ----------------------------------------------------------------------
    // Tag IDs we care about (example values; adjust per game) – Java 8 style
    // ----------------------------------------------------------------------

    private static final Set<Integer> SCORING_TAG_IDS;

    static {
        HashSet<Integer> ids = new HashSet<Integer>();
        ids.add(20);
        ids.add(24);
        SCORING_TAG_IDS = Collections.unmodifiableSet(ids);
    }

    public PhoenixRobot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.gamepads = Gamepads.create(gamepad1, gamepad2);
        this.telemetry = telemetry;
        this.dbg = new FtcTelemetryDebugSink(telemetry);
    }

    public void initAny() {
    }

    public void initTeleOp() {

        // --- Create mechanisms ---
        MecanumConfig mecanumConfig = MecanumConfig.defaults();
        mecanumConfig.maxLateralRatePerSec = 0.01;
        drivebase = Drives.mecanum(hardwareMap,
                RobotConfig.DriveTrain.invertMotorFrontLeft,
                RobotConfig.DriveTrain.invertMotorFrontRight,
                RobotConfig.DriveTrain.invertMotorBackLeft,
                RobotConfig.DriveTrain.invertMotorBackRight,
                null);
        shooter = new Shooter(hardwareMap, telemetry, gamepads);

        // --- Use the standard TeleOp stick mapping for mecanum.
        stickDrive = GamepadDriveSource.teleOpMecanumStandard(gamepads);

        // ---
        tagSensor = FtcVision.aprilTags(hardwareMap, "Webcam 1");

        // Track scoring tags with a freshness window.
        scoringTarget = new TagTarget(tagSensor, SCORING_TAG_IDS, 0.5);

        // Wrap baseDrive with TagAim: hold left bumper to auto-aim omega.
        aimConfig = TagAim.Config.defaults();
        aimConfig.kp = 2;
        aimConfig.maxOmega = 0.5;
        aimConfig.deadbandRad = Math.toRadians(.5);
        driveWithAim = TagAim.teleOpAim(
                stickDrive,
                gamepads.p2().leftBumper(),
                scoringTarget,
                aimConfig
        );

        telemetry.addLine("Phoenix TeleOp with AutoAim");
        telemetry.addLine("Left stick: drive, Right stick: turn, RB: slow mode");
        telemetry.update();

        // Create bindings
        createBindings();
    }

    private void createBindings() {
        bindings.onPress(gamepads.p2().y(),
                () -> taskRunnerTeleOp.enqueue(shooter.instantSetPusherFront()));

        bindings.onPress(gamepads.p2().a(),
                () -> taskRunnerTeleOp.enqueue(shooter.instantSetPusherBack()));

        bindings.whileHeld(gamepads.p2().b(),
                () -> taskRunnerTeleOp.enqueue(shooter.instantStartTransfer(Shooter.TransferDirection.FORWARD)),
                () -> taskRunnerTeleOp.enqueue(shooter.instantStopTransfer()));

        bindings.whileHeld(gamepads.p2().x(),
                () -> taskRunnerTeleOp.enqueue(shooter.instantStartTransfer(Shooter.TransferDirection.BACKWARD)),
                () -> taskRunnerTeleOp.enqueue(shooter.instantStopTransfer()));

        bindings.whileHeld(gamepads.p2().leftBumper(),
                () -> {
            AprilTagObservation obs = scoringTarget.last();
            if (obs.hasTarget) {
                taskRunnerTeleOp.enqueue(shooter.instantSetVelocityByDist(obs.rangeInches));
            }
                });

        bindings.toggle(gamepads.p2().rightBumper(),
                (isOn) -> {
            if (isOn) {
                taskRunnerTeleOp.enqueue(shooter.instantStartShooter());
            }

            else {
                taskRunnerTeleOp.enqueue(shooter.instantStopShooter());
            }
                });

        bindings.onPress(gamepads.p2().dpadUp(),
                () -> taskRunnerTeleOp.enqueue(shooter.instantIncreaseVelocity()));

        bindings.onPress(gamepads.p2().dpadDown(),
                () -> taskRunnerTeleOp.enqueue(shooter.instantDecreaseVelocity()));
    }

    public void startAny(double runtime) {
        // Initialize loop timing.
        clock.reset(runtime);
    }

    public void startTeleOp() {
    }

    public void updateAny(double runtime) {
        // --- 1) Clock ---
        clock.update(runtime);
    }

    public void updateTeleOp() {
        // --- 2) Inputs + bindings ---
        gamepads.update(clock.dtSec());
        bindings.update(clock.dtSec());

        // Update tracked tag once per loop.
        scoringTarget.update();

        // --- 3) TeleOp Macros ---
        taskRunnerTeleOp.update(clock);

        // When no macro is active, hold a safe default state.
        if (!taskRunnerTeleOp.hasActiveTask()) {
        }

        // --- 4) Drive: TagAim-wrapped drive source (LB may override omega) ---
        DriveSignal cmd = driveWithAim.get(clock);
        drivebase.drive(cmd);
        drivebase.update(clock);

        // --- 4) Other mechanisms ---


        // --- 5) Telemetry / debug ---
        telemetry.addData("shooter velocity", shooter.getVelocity());
//        driveWithAim.debugDump(dbg, "driveWAim");
        AprilTagObservation obs = scoringTarget.last();
        if (obs.hasTarget) {
            if (Math.abs(obs.bearingRad) <= (aimConfig.deadbandRad * 5))
                telemetry.addLine(">>> AIMED <<<");
            telemetry.addData("id", obs.id);
            telemetry.addData("dist", obs.rangeInches);
            telemetry.addData("bearing", Math.toDegrees(obs.bearingRad));
        }

        telemetry.update();
    }

    public void stopAny() {
        drivebase.stop();
    }

    public void stopTeleOp() {
    }
}
