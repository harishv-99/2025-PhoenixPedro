package edu.ftcphoenix.robots.phoenix.tester;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

/**
 * Robot-specific tester for verifying drivetrain motor direction.
 *
 * <p>
 * Hold a button to run one drivetrain motor forward at a fixed power. This helps confirm the
 * wiring + inversion flags in {@link FtcDrives.MecanumConfig}.
 * </p>
 */
public final class DrivetrainMotorDirectionTester extends BaseTeleOpTester {

    private static final double TEST_POWER = 0.5;

    private Plant plantFL;
    private Plant plantFR;
    private Plant plantBL;
    private Plant plantBR;
    private ScalarTarget targetFL;
    private ScalarTarget targetFR;
    private ScalarTarget targetBL;
    private ScalarTarget targetBR;

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
        FtcDrives.MecanumConfig drive = PhoenixProfile.current().drive;

        targetFL = ScalarTarget.create(0.0);
        targetFR = ScalarTarget.create(0.0);
        targetBL = ScalarTarget.create(0.0);
        targetBR = ScalarTarget.create(0.0);

        plantFL = FtcActuators.plant(ctx.hw)
                .motor(drive.wiring.frontLeftName, drive.wiring.frontLeftDirection)
                .power()
                .targetedBy(targetFL)
                .build();

        plantFR = FtcActuators.plant(ctx.hw)
                .motor(drive.wiring.frontRightName, drive.wiring.frontRightDirection)
                .power()
                .targetedBy(targetFR)
                .build();

        plantBL = FtcActuators.plant(ctx.hw)
                .motor(drive.wiring.backLeftName, drive.wiring.backLeftDirection)
                .power()
                .targetedBy(targetBL)
                .build();

        plantBR = FtcActuators.plant(ctx.hw)
                .motor(drive.wiring.backRightName, drive.wiring.backRightDirection)
                .power()
                .targetedBy(targetBR)
                .build();

        Bindings.ControlContext motorControls = bindings.contextWhen(
                BooleanSource.constant(true),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL
        );

        motorControls.mirrorOnChange(gamepads.p1().x(),
                high -> targetFL.set(high ? TEST_POWER : 0.0));

        motorControls.mirrorOnChange(gamepads.p1().y(),
                high -> targetFR.set(high ? TEST_POWER : 0.0));

        motorControls.mirrorOnChange(gamepads.p1().a(),
                high -> targetBL.set(high ? TEST_POWER : 0.0));

        motorControls.mirrorOnChange(gamepads.p1().b(),
                high -> targetBR.set(high ? TEST_POWER : 0.0));

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
        ctx.telemetry.addData("FL target", plantFL.getAppliedTarget());
        ctx.telemetry.addData("FR target", plantFR.getAppliedTarget());
        ctx.telemetry.addData("BL target", plantBL.getAppliedTarget());
        ctx.telemetry.addData("BR target", plantBR.getAppliedTarget());
        telemUpdate();
    }

    private void stopAll() {
        if (plantFL != null) plantFL.stop();
        if (plantFR != null) plantFR.stop();
        if (plantBL != null) plantBL.stop();
        if (plantBR != null) plantBR.stop();
    }
}
