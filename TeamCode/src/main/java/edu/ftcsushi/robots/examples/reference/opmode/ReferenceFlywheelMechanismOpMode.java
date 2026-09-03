package edu.ftcsushi.robots.examples.reference.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.robots.examples.reference.capability.flywheel.ReferenceFlywheelMechanism;
import edu.ftcsushi.robots.examples.reference.capability.flywheel.ReferenceFlywheels;

/** Disabled, flywheel-only host for isolated bring-up after a team's physical safety review. */
@TeleOp(name = "FW Reference: Paired Flywheels", group = "FW Examples")
@Disabled
public final class ReferenceFlywheelMechanismOpMode extends FtcRobotOpMode {
    private static final boolean MOTION_REVIEWED = false;
    private static final double TEST_VELOCITY_TICKS_PER_SEC = 3000.0;
    private static final double WAIT_TIMEOUT_SEC = 2.0;

    @Override
    protected void configure(RobotProgram program) {
        if (!MOTION_REVIEWED) {
            throw new IllegalStateException(
                    "Reference paired-flywheel motion is locked. Verify motor identity, "
                            + "direction, restraint, velocity range, and emergency stop before "
                            + "setting MOTION_REVIEWED=true and removing @Disabled.");
        }

        ReferenceFlywheelMechanism.Config config =
                ReferenceFlywheelMechanism.Config.defaults();
        ReferenceFlywheelMechanism flywheels = program.output(
                new ReferenceFlywheelMechanism(hardwareMap, config));
        GamepadDevice operator = new GamepadDevice(gamepad1);

        // A is the direct persistent request. Y creates a fresh Task and waits for both wheels.
        program.callbackBindings().onRise(
                operator.a(),
                () -> flywheels.setVelocityTicksPerSec(TEST_VELOCITY_TICKS_PER_SEC));
        program.taskBindings().onRise(
                operator.y(),
                () -> flywheels.setVelocityTask(
                        TEST_VELOCITY_TICKS_PER_SEC,
                        WAIT_TIMEOUT_SEC));
        program.callbackBindings().onRise(
                operator.b(),
                () -> flywheels.setVelocityTicksPerSec(0.0));

        program.presenter((clock, telemetry) -> {
            ReferenceFlywheels.Status status = flywheels.status();
            telemetry.addData("flywheels.requested", status.requestedVelocityTicksPerSec());
            telemetry.addData("flywheels.applied", status.appliedVelocityTicksPerSec());
            telemetry.addData("flywheels.left", status.leftMeasuredVelocityTicksPerSec());
            telemetry.addData("flywheels.right", status.rightMeasuredVelocityTicksPerSec());
            telemetry.addData("flywheels.ready", status.ready());
            telemetry.addLine("A: request | Y: request-and-wait | B: idle");
        });
    }
}
