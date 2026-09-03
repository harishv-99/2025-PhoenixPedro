package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Complete basic TeleOp: one gamepad, drivetrain, referenced lift, and standard-servo claw. */
@TeleOp(name = "FW Basic 5: Robot TeleOp", group = "FW Examples")
@Disabled
public final class BasicRobotTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicDriveProfile driveProfile = BasicDriveProfile.current();
        BasicLiftProfile liftProfile = BasicLiftProfile.current();
        BasicClawProfile clawProfile = BasicClawProfile.current();
        BasicDriveProfile.requireMotionAllowed(driveProfile, "Basic Robot TeleOp");
        BasicLiftProfile.requireMotionAllowed(liftProfile, "Basic Robot TeleOp");
        BasicClawProfile.requireMotionAllowed(clawProfile, "Basic Robot TeleOp");
        BasicHardwareOwnership.requireDistinctDriveAndLiftMotors(
                driveProfile.drive, liftProfile.lift, "Basic Robot TeleOp");

        BasicLiftMechanism lift = program.output(
                new BasicLiftMechanism(hardwareMap, liftProfile.lift));
        BasicClawMechanism claw = program.output(
                new BasicClawMechanism(hardwareMap, clawProfile.claw));

        GamepadDevice driver = new GamepadDevice(gamepad1);
        BasicDriveControls driveControls = new BasicDriveControls(driver);
        new BasicLiftControls(driver).bind(
                program.callbackBindings(), program.taskBindings(), lift);
        new BasicClawControls(driver).bind(program.callbackBindings(), claw);

        // RobotProgram owns the one source-driven final drivetrain write in TeleOp.
        program.drive(
                driveControls.driveSource(),
                FtcDrives.mecanum(hardwareMap, driveProfile.drive));
        program.presenter((clock, telemetry) -> {
            BasicLift.Status liftStatus = lift.status();
            BasicClaw.Status clawStatus = claw.status();
            telemetry.addData("lift", "%s %.2f/%.2f in referenced=%s atTarget=%s",
                    liftStatus.requestedHeight(),
                    liftStatus.measuredPositionIn(),
                    liftStatus.requestedPositionIn(),
                    liftStatus.referenced(),
                    liftStatus.atTarget());
            telemetry.addData("claw", "%s coordinate=%.2f",
                    clawStatus.requestedState, clawStatus.appliedCoordinate);
        });
    }
}
