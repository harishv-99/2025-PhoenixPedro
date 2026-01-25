package edu.ftcphoenix.fw.tools.tester;

import edu.ftcphoenix.fw.tools.tester.calibration.CameraMountCalibrator;
import edu.ftcphoenix.fw.tools.tester.hardware.CrServoPowerTester;
import edu.ftcphoenix.fw.tools.tester.hardware.DcMotorPositionTester;
import edu.ftcphoenix.fw.tools.tester.hardware.DcMotorPowerTester;
import edu.ftcphoenix.fw.tools.tester.hardware.DcMotorVelocityTester;
import edu.ftcphoenix.fw.tools.tester.hardware.ServoPositionTester;
import edu.ftcphoenix.fw.tools.tester.localization.AprilTagLocalizationTester;

/**
 * Registers the standard Phoenix framework testers into a {@link TesterSuite}.
 *
 * <p>This is preferred over inheritance: create a suite, call {@link #register(TesterSuite)},
 * then optionally add robot-specific testers that use your RobotConfig.</p>
 *
 * <pre>
 * TesterSuite suite = new TesterSuite();
 * StandardTesters.register(suite);
 * suite.add("Intake Motor", () -> new DcMotorPowerTester(cfg.intakeMotorName()));
 * </pre>
 */
public final class StandardTesters {

    private StandardTesters() {
    }

    /**
     * Adds the standard framework testers to the given suite.
     */
    public static void register(TesterSuite suite) {
        if (suite == null) return;

        // Calibration
        suite.add(
                "Calib: Camera Mount",
                "Solve robotToCameraPose using known robot pose + tag layout. Includes camera picker.",
                CameraMountCalibrator::new
        );

        // Localization
        suite.add(
                "Loc: AprilTag Localization",
                "Verify AprilTag detections + compute fieldToRobot pose (includes camera picker).",
                AprilTagLocalizationTester::new
        );

        // Hardware - Motors
        suite.add(
                "HW: DcMotor Power",
                "Open-loop motor power test (enable, invert, step, stick override).",
                DcMotorPowerTester::new
        );

        suite.add(
                "HW: DcMotor Position",
                "RUN_TO_POSITION encoder target test (target + power + stick nudge).",
                DcMotorPositionTester::new
        );

        suite.add(
                "HW: DcMotor Velocity",
                "DcMotorEx velocity closed-loop test (setVelocity + target + stick nudge).",
                DcMotorVelocityTester::new
        );

        // Hardware - Servos
        suite.add(
                "HW: CRServo Power",
                "Continuous rotation servo power test (enable, invert, step, stick override).",
                CrServoPowerTester::new
        );

        suite.add(
                "HW: Servo Position",
                "Standard servo position test (enable=hold, invert, step, stick override).",
                ServoPositionTester::new
        );
    }

    /**
     * Convenience: creates a new suite with standard testers already registered.
     */
    public static TesterSuite createSuite() {
        TesterSuite suite = new TesterSuite();
        register(suite);
        return suite;
    }
}
