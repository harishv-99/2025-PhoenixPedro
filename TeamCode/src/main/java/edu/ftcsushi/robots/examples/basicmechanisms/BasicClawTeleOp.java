package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Focused claw-only fixture that never claims unmeasured standard-servo arrival. */
@TeleOp(name = "FW Basic 3: Claw", group = "FW Examples")
@Disabled
public final class BasicClawTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicClawProfile clawProfile = BasicClawProfile.current();
        BasicClawProfile.requireMotionAllowed(clawProfile, "Basic Claw TeleOp");

        BasicClawMechanism claw = program.output(
                new BasicClawMechanism(hardwareMap, clawProfile.claw));
        GamepadDevice driver = new GamepadDevice(gamepad1);
        BasicClawControls clawControls = new BasicClawControls(driver);
        clawControls.bind(program.callbackBindings(), claw);

        program.presenter((clock, telemetry) -> {
            BasicClaw.Status status = claw.status();
            telemetry.addData("claw.request", status.requestedState());
            telemetry.addData("claw.appliedCoordinate", "%.2f", status.appliedCoordinate());
            telemetry.addLine("claw coordinate is a normalized target, not servo feedback");
        });
    }
}
