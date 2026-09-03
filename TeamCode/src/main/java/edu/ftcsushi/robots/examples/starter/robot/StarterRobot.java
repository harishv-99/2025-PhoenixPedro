package edu.ftcsushi.robots.examples.starter.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntakeMechanism;

/**
 * Declaration-only composition for the modern starter robot.
 *
 * <p>The framework-owned {@link RobotProgram} supplies the one clock, fixed heartbeat order,
 * Task runner, telemetry commit, and shutdown. This class owns only the starter robot's hardware
 * construction, mode-specific declarations, and presenter content.</p>
 */
public final class StarterRobot {

    private final HardwareMap hardwareMap;

    /** Retains only the FTC registry used by the owner-local construction paths. */
    public StarterRobot(HardwareMap hardwareMap) {
        this.hardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
    }

    /**
     * Declare the starter TeleOp's controls, intake output, drive output, and telemetry rows.
     *
     * <p>The two motion permissions and the only cross-owner motor-name collision are checked
     * before hardware lookup. Each hardware owner then snapshots and validates only its own active
     * configuration. The intake mechanism, rather than its private Plant, is the declared output
     * owner.</p>
     */
    public void declareTeleOp(RobotProgram program,
                              StarterProfile profile,
                              Gamepad gamepad1) {
        Objects.requireNonNull(program, "program");
        StarterProfile activeProfile = Objects.requireNonNull(profile, "profile");
        Gamepad requiredGamepad = Objects.requireNonNull(gamepad1, "gamepad1");
        requireMotionAllowed(
                "TeleOp",
                "StarterProfile.allowIntakeMotion",
                activeProfile.allowIntakeMotion);
        requireMotionAllowed(
                "TeleOp",
                "StarterProfile.allowDriveMotion",
                activeProfile.allowDriveMotion);
        requireDistinctMotorOwners(activeProfile);

        StarterIntakeMechanism intake = declareIntake(program, activeProfile);
        StarterTeleOpControls controls = new StarterTeleOpControls(
                new GamepadDevice(requiredGamepad));
        controls.bind(program.callbackBindings(), intake);

        program.drive(
                controls.driveSource(),
                FtcDrives.mecanum(hardwareMap, activeProfile.drive));
    }

    /**
     * Declare only the intake controls, output, and status used by the focused power lesson.
     *
     * <p>This path deliberately does not inspect or construct the drivetrain. It lets a student
     * prove one continuous-power mechanism before combining it with another hardware owner, while
     * preserving the same capability, controls, Plant, heartbeat, and stop ownership used by the
     * complete Starter TeleOp.</p>
     */
    public void declareIntakeTeleOp(RobotProgram program,
                                    StarterProfile profile,
                                    Gamepad gamepad1) {
        Objects.requireNonNull(program, "program");
        StarterProfile activeProfile = Objects.requireNonNull(profile, "profile");
        Gamepad requiredGamepad = Objects.requireNonNull(gamepad1, "gamepad1");
        requireMotionAllowed(
                "focused intake TeleOp",
                "StarterProfile.allowIntakeMotion",
                activeProfile.allowIntakeMotion);

        StarterIntakeMechanism intake = declareIntake(program, activeProfile);
        StarterTeleOpControls controls = new StarterTeleOpControls(
                new GamepadDevice(requiredGamepad));
        controls.bind(program.callbackBindings(), intake);
    }

    /**
     * Declare the starter Auto's intake output and telemetry rows.
     *
     * <p>Only the intake configuration and its motion permission are read in Auto, so an unused
     * drive configuration or drive permission cannot prevent this program from initializing.
     * The Auto client receives the shared capability and owns its routine choice.</p>
     *
     * @return declared intake capability for the Auto client's selected routine
     */
    public StarterIntake declareAuto(RobotProgram program,
                                     StarterProfile profile) {
        Objects.requireNonNull(program, "program");
        StarterProfile activeProfile = Objects.requireNonNull(profile, "profile");
        requireMotionAllowed(
                "Auto",
                "StarterProfile.allowIntakeMotion",
                activeProfile.allowIntakeMotion);

        return declareIntake(program, activeProfile);
    }

    private static void requireMotionAllowed(String mode,
                                             String permissionPath,
                                             boolean allowed) {
        if (!allowed) {
            throw new IllegalStateException(
                    permissionPath + " must be true before " + mode
                            + " may construct a motion-capable hardware owner. Review the active "
                            + "physical configuration before permitting motion, then verify "
                            + "small supervised motion and physical STOP.");
        }
    }

    private static void requireDistinctMotorOwners(StarterProfile profile) {
        if (profile.intake == null || profile.drive == null || profile.drive.wiring == null) {
            return;
        }
        String intakeName = profile.intake.motorName;
        requireDistinctMotorOwner(
                intakeName,
                "StarterProfile.intake.motorName",
                profile.drive.wiring.frontLeftName,
                "StarterProfile.drive.wiring.frontLeftName");
        requireDistinctMotorOwner(
                intakeName,
                "StarterProfile.intake.motorName",
                profile.drive.wiring.frontRightName,
                "StarterProfile.drive.wiring.frontRightName");
        requireDistinctMotorOwner(
                intakeName,
                "StarterProfile.intake.motorName",
                profile.drive.wiring.backLeftName,
                "StarterProfile.drive.wiring.backLeftName");
        requireDistinctMotorOwner(
                intakeName,
                "StarterProfile.intake.motorName",
                profile.drive.wiring.backRightName,
                "StarterProfile.drive.wiring.backRightName");
    }

    private static void requireDistinctMotorOwner(String firstName,
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
                    "Starter TeleOp motor ownership collision: " + firstPath + " and "
                            + secondPath + " both resolve to FTC hardware key \"" + firstKey
                            + "\". Configure distinct motor names.");
        }
    }

    private static boolean isBlank(String value) {
        return value == null || value.trim().isEmpty();
    }

    private StarterIntakeMechanism declareIntake(RobotProgram program,
                                                  StarterProfile profile) {
        StarterIntakeMechanism intake = program.output(
                new StarterIntakeMechanism(hardwareMap, profile.intake));
        program.presenter((clock, telemetry) -> presentIntake(telemetry, intake));
        return intake;
    }

    private static void presentIntake(Telemetry telemetry, StarterIntake intake) {
        StarterIntake.Status status = intake.status();
        telemetry.addData("intake.mode", status.mode());
        telemetry.addData("intake.appliedTargetPower", status.appliedPower());
    }
}
