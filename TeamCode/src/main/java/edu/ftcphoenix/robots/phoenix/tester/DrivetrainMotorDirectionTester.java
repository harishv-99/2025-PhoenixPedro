package edu.ftcphoenix.robots.phoenix.tester;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.drive.FtcMecanumDriveLane;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

/**
 * Robot-specific tester for verifying drivetrain motor direction.
 *
 * <p>
 * Hold a button to run one drivetrain motor forward at a fixed power. This helps confirm the
 * wiring + inversion flags in {@link FtcMecanumDriveLane.Config}.
 * </p>
 */
public final class DrivetrainMotorDirectionTester extends BaseTeleOpTester {

    private static final double TEST_POWER = 0.5;

    private Plant plantFL;
    private Plant plantFR;
    private Plant plantBL;
    private Plant plantBR;

    /**
     * Creates the tester instance.
     */
    public DrivetrainMotorDirectionTester() {
    }

    /**
     * Registers this tester in a tester suite.
     *
     * @param suite suite to receive the menu entry; ignored when {@code null}
     */
    public static void register(TesterSuite suite) {
        if (suite == null) {
            return;
        }

        suite.add(
                "HW: Drivetrain Motor Direction (Robot)",
                "Hold X/Y/A/B to run FL/FR/BL/BR. Each should drive robot forward.",
                DrivetrainMotorDirectionTester::new
        );
    }

    /**
     * Returns the display name shown by the tester framework.
     *
     * @return short user-facing tester name
     */
    @Override
    public String name() {
        return "Drivetrain Motor Direction";
    }

    @Override
    protected void onInit() {
        FtcMecanumDriveLane.Config drive = PhoenixProfile.current().drive;

        plantFL = FtcActuators.plant(ctx.hw)
                .motor(drive.wiring.frontLeftName, drive.wiring.frontLeftDirection)
                .power()
                .build();

        plantFR = FtcActuators.plant(ctx.hw)
                .motor(drive.wiring.frontRightName, drive.wiring.frontRightDirection)
                .power()
                .build();

        plantBL = FtcActuators.plant(ctx.hw)
                .motor(drive.wiring.backLeftName, drive.wiring.backLeftDirection)
                .power()
                .build();

        plantBR = FtcActuators.plant(ctx.hw)
                .motor(drive.wiring.backRightName, drive.wiring.backRightDirection)
                .power()
                .build();

        bindings.whileTrue(gamepads.p1().x(),
                () -> plantFL.setTarget(TEST_POWER),
                () -> plantFL.setTarget(0.0));

        bindings.whileTrue(gamepads.p1().y(),
                () -> plantFR.setTarget(TEST_POWER),
                () -> plantFR.setTarget(0.0));

        bindings.whileTrue(gamepads.p1().a(),
                () -> plantBL.setTarget(TEST_POWER),
                () -> plantBL.setTarget(0.0));

        bindings.whileTrue(gamepads.p1().b(),
                () -> plantBR.setTarget(TEST_POWER),
                () -> plantBR.setTarget(0.0));

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
    }

    private void updateAndRender(double dtSec) {
        plantFL.update(clock);
        plantFR.update(clock);
        plantBL.update(clock);
        plantBR.update(clock);

        telemHeader("Drivetrain Motor Direction");
        telemHint("Hold X/Y/A/B to run one drivetrain motor forward.");
        ctx.telemetry.addData("FL", gamepads.p1().x().getAsBoolean(clock) ? TEST_POWER : 0.0);
        ctx.telemetry.addData("FR", gamepads.p1().y().getAsBoolean(clock) ? TEST_POWER : 0.0);
        ctx.telemetry.addData("BL", gamepads.p1().a().getAsBoolean(clock) ? TEST_POWER : 0.0);
        ctx.telemetry.addData("BR", gamepads.p1().b().getAsBoolean(clock) ? TEST_POWER : 0.0);
        telemUpdate();
    }

    private void stopAll() {
        if (plantFL != null) plantFL.stop();
        if (plantFR != null) plantFR.stop();
        if (plantBL != null) plantBL.stop();
        if (plantBR != null) plantBR.stop();
    }
}
