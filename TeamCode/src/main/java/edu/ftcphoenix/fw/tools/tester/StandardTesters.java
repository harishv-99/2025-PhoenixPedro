package edu.ftcphoenix.fw.tools.tester;

import edu.ftcphoenix.fw.tools.tester.calibration.CameraMountCalibrator;
import edu.ftcphoenix.fw.tools.tester.hardware.CrServoPowerTester;
import edu.ftcphoenix.fw.tools.tester.hardware.DcMotorPositionTester;
import edu.ftcphoenix.fw.tools.tester.hardware.DcMotorPowerTester;
import edu.ftcphoenix.fw.tools.tester.hardware.DcMotorVelocityTester;
import edu.ftcphoenix.fw.tools.tester.hardware.ServoPositionTester;
import edu.ftcphoenix.fw.tools.tester.localization.AprilTagLocalizationTester;

/**
 * Registers the standard Phoenix framework testers into organized menu groups.
 *
 * <p>Guided walkthrough menus may intentionally duplicate a few entries, but the framework testers
 * themselves should still have one obvious "home" in the menu tree. That keeps the top-level menu
 * compact even as more testers are added over time.</p>
 */
public final class StandardTesters {

    private StandardTesters() {
    }

    /**
     * Adds the framework tester groups to the supplied top-level suite.
     */
    public static void register(TesterSuite suite) {
        if (suite == null) return;

        suite.add(
                "Framework: Calibration & Localization",
                "Framework-owned camera-mount and AprilTag localization tools.",
                StandardTesters::createCalibrationAndLocalizationSuite
        );

        suite.add(
                "Framework: Hardware Testers",
                "Generic DcMotor / Servo / CRServo bring-up tools.",
                StandardTesters::createHardwareSuite
        );
    }

    /**
     * Creates a standalone suite containing the organized framework tester groups.
     */
    public static TesterSuite createSuite() {
        TesterSuite suite = new TesterSuite()
                .setTitle("Framework Tester Menu")
                .setHelp("Choose a tester group. BACK returns to the previous menu.");
        register(suite);
        return suite;
    }

    /**
     * Framework camera-mount and localization tools.
     */
    public static TesterSuite createCalibrationAndLocalizationSuite() {
        TesterSuite suite = new TesterSuite()
                .setTitle("Framework Calibration & Localization")
                .setHelp("Camera mount and AprilTag bring-up tools.")
                .setMaxVisibleItems(8);

        suite.add(
                "Calib: Camera Mount",
                "Solve robotToCameraPose using known robot pose + fixed tag layout. Includes camera picker.",
                CameraMountCalibrator::new
        );

        suite.add(
                "Loc: AprilTag Localization",
                "Verify AprilTag detections and the field pose solve. Includes camera picker.",
                AprilTagLocalizationTester::new
        );

        return suite;
    }

    /**
     * Framework hardware bring-up tools.
     */
    public static TesterSuite createHardwareSuite() {
        TesterSuite suite = new TesterSuite()
                .setTitle("Framework Hardware Testers")
                .setHelp("Generic hardware sanity-check tools.")
                .setMaxVisibleItems(8);

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

        return suite;
    }
}
