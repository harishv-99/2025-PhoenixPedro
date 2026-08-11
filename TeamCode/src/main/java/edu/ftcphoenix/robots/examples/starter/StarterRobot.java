package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.input.GamepadDevice;
import edu.ftcphoenix.fw.task.Task;

/**
 * Declaration-only composition for the modern starter robot.
 *
 * <p>The framework-owned {@link RobotProgram} supplies the one clock, fixed heartbeat order,
 * Task runner, telemetry commit, and shutdown. This class owns only the starter robot's hardware
 * construction, mode-specific declarations, and presenter content.</p>
 */
public final class StarterRobot {

    private final HardwareMap hardwareMap;
    private final StarterProfile profile;

    /** Retains the FTC boundary and an independent data-only profile snapshot. */
    public StarterRobot(HardwareMap hardwareMap, StarterProfile profile) {
        this.hardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.profile = Objects.requireNonNull(profile, "profile").copy();
    }

    /**
     * Declare the starter TeleOp's controls, intake output, drive output, and telemetry rows.
     *
     * <p>The complete profile is validated before either mechanism or drive hardware is looked
     * up. The intake mechanism, rather than its private Plant, is the declared output owner.</p>
     */
    public StarterIntake declareTeleOp(RobotProgram program, Gamepad gamepad1) {
        Objects.requireNonNull(program, "program");
        Gamepad requiredGamepad = Objects.requireNonNull(gamepad1, "gamepad1");
        profile.requireReadyForTeleOp();

        StarterIntakeMechanism intake = program.output(
                new StarterIntakeMechanism(hardwareMap, profile.intake));
        StarterTeleOpControls controls = new StarterTeleOpControls(
                new GamepadDevice(requiredGamepad));
        controls.bind(program.callbackBindings(), intake);

        program.drive(
                controls.driveSource(),
                FtcDrives.mecanum(hardwareMap, profile.drive));
        program.presenter((clock, telemetry) -> presentIntake(telemetry, intake));
        return intake;
    }

    /**
     * Declare the starter Auto's intake output, one fresh root Task, and telemetry rows.
     *
     * <p>Only the shared intake configuration is required in Auto, so an unused drive
     * configuration cannot prevent this program from initializing.</p>
     */
    public StarterIntake declareAuto(RobotProgram program, double collectDurationSec) {
        Objects.requireNonNull(program, "program");
        profile.requireReadyForAuto();

        StarterIntakeMechanism intake = program.output(
                new StarterIntakeMechanism(hardwareMap, profile.intake));
        Task root = intake.collectForSeconds(collectDurationSec);
        program.rootTask(root);
        program.presenter((clock, telemetry) -> {
            presentIntake(telemetry, intake);
            telemetry.addData("auto.idle", root.isComplete());
        });
        return intake;
    }

    private static void presentIntake(Telemetry telemetry, StarterIntake intake) {
        StarterIntake.Status status = intake.status();
        telemetry.addData("intake.mode", status.mode());
        telemetry.addData("intake.appliedTargetPower", status.appliedTargetPower());
    }
}
