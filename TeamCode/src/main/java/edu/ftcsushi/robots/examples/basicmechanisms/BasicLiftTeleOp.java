package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Focused lift-only fixture with named controls and truthful feedback evidence. */
@TeleOp(name = "FW Basic 2: Lift", group = "FW Examples")
@Disabled
public final class BasicLiftTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicLiftProfile liftProfile = BasicLiftProfile.current();
        BasicLiftProfile.requireMotionAllowed(liftProfile, "Basic Lift TeleOp");

        BasicLiftMechanism lift = program.output(
                new BasicLiftMechanism(hardwareMap, liftProfile.lift));
        GamepadDevice driver = new GamepadDevice(gamepad1);
        BasicLiftControls liftControls = new BasicLiftControls(driver);
        liftControls.bind(program.callbackBindings(), program.taskBindings(), lift);

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
