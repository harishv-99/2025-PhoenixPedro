package edu.ftcphoenix.robots.phoenix.tester;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.robots.phoenix.RobotConfig;

/**
 * Robot-specific tester for verifying drivetrain motor direction.
 *
 * <p>Hold a button to run one drivetrain motor forward at a fixed power.
 * This helps confirm the wiring + inversion flags in {@link RobotConfig.DriveTrain}.</p>
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li>X: front-left motor</li>
 *   <li>Y: front-right motor</li>
 *   <li>A: back-left motor</li>
 *   <li>B: back-right motor</li>
 * </ul>
 *
 * <p><b>Expected behavior:</b> each motor should spin such that the wheel would drive the robot forward.</p>
 */
public final class DrivetrainMotorDirectionTester extends BaseTeleOpTester {

    private static final double TEST_POWER = 0.5;

    private Plant plantFL;
    private Plant plantFR;
    private Plant plantBL;
    private Plant plantBR;

    /**
     * Register this tester into a suite (robot-side).
     *
     * <p>Typical usage in your robot OpMode:
     * <pre>{@code
     * TesterSuite suite = StandardTesters.createSuite();
     * DrivetrainMotorDirectionTester.register(suite);
     * }</pre>
     */
    public static void register(TesterSuite suite) {
        if (suite == null) return;

        suite.add(
                "HW: Drivetrain Motor Direction (Robot)",
                "Hold X/Y/A/B to run FL/FR/BL/BR. Each should drive robot forward.",
                DrivetrainMotorDirectionTester::new
        );
    }

    @Override
    public String name() {
        return "Drivetrain Motor Direction";
    }

    @Override
    protected void onInit() {
        // Build drivetrain motor plants (open-loop power).
        plantFL = Actuators.plant(ctx.hw)
                .motor(RobotConfig.DriveTrain.nameMotorFrontLeft,
                        RobotConfig.DriveTrain.directionMotorFrontLeft)
                .power()
                .build();

        plantFR = Actuators.plant(ctx.hw)
                .motor(RobotConfig.DriveTrain.nameMotorFrontRight,
                        RobotConfig.DriveTrain.directionMotorFrontRight)
                .power()
                .build();

        plantBL = Actuators.plant(ctx.hw)
                .motor(RobotConfig.DriveTrain.nameMotorBackLeft,
                        RobotConfig.DriveTrain.directionMotorBackLeft)
                .power()
                .build();

        plantBR = Actuators.plant(ctx.hw)
                .motor(RobotConfig.DriveTrain.nameMotorBackRight,
                        RobotConfig.DriveTrain.directionMotorBackRight)
                .power()
                .build();

        // Keep the same button mapping/behavior: hold button -> run that motor, release -> stop.
        bindings.whileHeld(gamepads.p1().x(),
                () -> plantFL.setTarget(TEST_POWER),
                () -> plantFL.setTarget(0.0));

        bindings.whileHeld(gamepads.p1().y(),
                () -> plantFR.setTarget(TEST_POWER),
                () -> plantFR.setTarget(0.0));

        bindings.whileHeld(gamepads.p1().a(),
                () -> plantBL.setTarget(TEST_POWER),
                () -> plantBL.setTarget(0.0));

        bindings.whileHeld(gamepads.p1().b(),
                () -> plantBR.setTarget(TEST_POWER),
                () -> plantBR.setTarget(0.0));

        // Safety default.
        stopAll();
    }

    @Override
    protected void onInitLoop(double dtSec) {
        updateAndRender(dtSec);
    }

    @Override
    protected void onLoop(double dtSec) {
        updateAndRender(dtSec);
    }

    @Override
    protected void onStop() {
        stopAll();

        // One last update to ensure hardware receives stop commands.
        if (plantFL != null) plantFL.update(0.0);
        if (plantFR != null) plantFR.update(0.0);
        if (plantBL != null) plantBL.update(0.0);
        if (plantBR != null) plantBR.update(0.0);
    }

    // ---------------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------------

    private void updateAndRender(double dtSec) {
        // If nothing is held, force a safe default (helps if a binding ever gets skipped).
        if (!anyHeld()) {
            stopAll();
        }

        // Update plants.
        plantFL.update(dtSec);
        plantFR.update(dtSec);
        plantBL.update(dtSec);
        plantBR.update(dtSec);

        // Telemetry / instructions.
        ctx.telemetry.addLine("Test drivetrain motor direction (each should drive robot forward):");
        ctx.telemetry.addLine("  X = front-left");
        ctx.telemetry.addLine("  Y = front-right");
        ctx.telemetry.addLine("  A = back-left");
        ctx.telemetry.addLine("  B = back-right");
        ctx.telemetry.update();
    }

    private boolean anyHeld() {
        return gamepads.p1().x().isHeld()
                || gamepads.p1().y().isHeld()
                || gamepads.p1().a().isHeld()
                || gamepads.p1().b().isHeld();
    }

    private void stopAll() {
        if (plantFL != null) plantFL.setTarget(0.0);
        if (plantFR != null) plantFR.setTarget(0.0);
        if (plantBL != null) plantBL.setTarget(0.0);
        if (plantBR != null) plantBR.setTarget(0.0);
    }
}
