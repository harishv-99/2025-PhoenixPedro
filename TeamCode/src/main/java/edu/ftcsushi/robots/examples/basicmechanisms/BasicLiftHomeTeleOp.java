package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Disabled lift-only host that exposes homing without exposing later named moves. */
@TeleOp(name = "FW Basic: Lift home", group = "FW Examples")
@Disabled
public final class BasicLiftHomeTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicLiftProfile profile = BasicLiftProfile.current();
        BasicLiftProfile.requireMotionAllowed(profile, "Basic Lift Home TeleOp");

        BasicLiftMechanism lift = program.output(
                new BasicLiftMechanism(hardwareMap, profile.lift));
        BasicLiftHomeControls controls = new BasicLiftHomeControls(
                new GamepadDevice(gamepad1));
        controls.bind(program.taskBindings(), lift);

        program.presenter((clock, telemetry) -> {
            BasicLift.Status status = lift.status();
            telemetry.addData("lift.request", status.requestedHeight());
            telemetry.addData("lift.positionIn", "%.2f / %.2f",
                    status.measuredPositionIn(), status.requestedPositionIn());
            telemetry.addData("lift.referenced", status.referenced());
            telemetry.addLine("X: home");
        });
    }
}
