package edu.ftcsushi.robots.examples.reference.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.robots.examples.reference.capability.targeting.ReferencePeriodicTurretMechanism;

/** Disabled, turret-only host for inspecting periodic selection without a complete robot graph. */
@TeleOp(name = "FW Reference: Periodic Turret", group = "FW Examples")
@Disabled
public final class ReferencePeriodicTurretOpMode extends FtcRobotOpMode {
    private static final boolean MOTION_REVIEWED = false;
    private static final double TEST_ANGLE_RAD = 0.25 * Math.PI;
    private static final double WAIT_TIMEOUT_SEC = 2.0;

    @Override
    protected void configure(RobotProgram program) {
        if (!MOTION_REVIEWED) {
            throw new IllegalStateException(
                    "Reference turret motion is locked. Establish encoder zero, motor direction, "
                            + "scale, cable bounds, collision clearance, and emergency stop before "
                            + "setting MOTION_REVIEWED=true and removing @Disabled.");
        }

        ReferencePeriodicTurretMechanism turret = program.output(
                new ReferencePeriodicTurretMechanism(
                        hardwareMap,
                        ReferencePeriodicTurretMechanism.Config.defaults()));
        GamepadDevice operator = new GamepadDevice(gamepad1);

        // A makes a direct periodic request. Y creates a fresh Task that also waits for arrival.
        program.callbackBindings().onRise(
                operator.a(),
                () -> turret.setAngleRad(TEST_ANGLE_RAD));
        program.taskBindings().onRise(
                operator.y(),
                () -> turret.setAngleTask(TEST_ANGLE_RAD, WAIT_TIMEOUT_SEC));
        program.callbackBindings().onRise(operator.b(), () -> turret.setAngleRad(0.0));

        program.presenter((clock, telemetry) -> {
            ReferencePeriodicTurretMechanism.Status status = turret.status();
            telemetry.addData("turret.requestedRad", status.requestedAngleRad());
            telemetry.addData("turret.selectedRad", status.selectedAngleRad());
            telemetry.addData("turret.appliedRad", status.appliedAngleRad());
            telemetry.addData("turret.measuredRad", status.measuredAngleRad());
            telemetry.addData("turret.arrived", status.arrived());
            telemetry.addLine("A: request | Y: request-and-wait | B: zero");
        });
    }
}
