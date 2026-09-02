package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Frozen drive-plus-lift fixture with focused lift controls and truthful feedback evidence. */
@TeleOp(name = "FW Basic 2: Lift", group = "FW Examples")
@Disabled
public final class BasicLiftTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicDriveProfile driveProfile = BasicDriveProfile.current();
        BasicLiftProfile liftProfile = BasicLiftProfile.current();
        BasicDriveProfile.requireMotionAllowed(driveProfile, "Basic Lift TeleOp");
        BasicLiftProfile.requireMotionAllowed(liftProfile, "Basic Lift TeleOp");
        BasicHardwareOwnership.requireDistinctDriveAndLiftMotors(
                driveProfile.drive, liftProfile.lift, "Basic Lift TeleOp");

        BasicLiftMechanism lift = program.output(
                new BasicLiftMechanism(hardwareMap, liftProfile.lift));
        GamepadDevice driver = new GamepadDevice(gamepad1);
        BasicDriveControls driveControls = new BasicDriveControls(driver);
        BasicLiftControls liftControls = new BasicLiftControls(driver);
        liftControls.bind(program.callbackBindings(), program.taskBindings(), lift);
        program.drive(
                driveControls.driveSource(),
                FtcDrives.mecanum(hardwareMap, driveProfile.drive));

        program.presenter((clock, telemetry) -> {
            BasicLift.Status status = lift.status();
            telemetry.addData("lift.request", status.requestedHeight());
            telemetry.addData("lift.positionIn", "%.2f / %.2f",
                    status.measuredPositionIn(), status.requestedPositionIn());
            telemetry.addData("lift.referenced", status.referenced());
            telemetry.addData("lift.atTarget", status.atTarget());
        });
    }
}
