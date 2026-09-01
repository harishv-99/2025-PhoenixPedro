package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Frozen drive-plus-claw fixture that never claims unmeasured servo arrival. */
@TeleOp(name = "FW Basic 3: Claw", group = "FW Examples")
@Disabled
public final class BasicClawTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicDriveProfile driveProfile = BasicDriveProfile.current();
        BasicClawProfile clawProfile = BasicClawProfile.current();
        BasicDriveProfile.requireMotionAllowed(driveProfile, "Basic Claw TeleOp");
        BasicClawProfile.requireMotionAllowed(clawProfile, "Basic Claw TeleOp");

        BasicClawMechanism claw = program.output(
                new BasicClawMechanism(hardwareMap, clawProfile.claw));
        GamepadDevice driver = new GamepadDevice(gamepad1);
        BasicDriveControls driveControls = new BasicDriveControls(driver);
        BasicClawControls clawControls = new BasicClawControls(driver);
        clawControls.bind(program.callbackBindings(), claw);
        program.drive(
                driveControls.driveSource(),
                FtcDrives.mecanum(hardwareMap, driveProfile.drive));

        program.presenter((clock, telemetry) -> {
            BasicClaw.Status status = claw.status();
            telemetry.addData("claw.request", status.requestedState);
            telemetry.addData("claw.appliedCommand", "%.2f", status.appliedPosition);
            telemetry.addLine("claw position is a command, not servo feedback");
        });
    }
}
