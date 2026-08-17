package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;

/** Thin declarative FTC host for the modern starter TeleOp reference. */
@TeleOp(name = "FW Starter: TeleOp", group = "FW Examples")
@Disabled
public final class StarterTeleOp extends FtcRobotOpMode {

    /** FTC construction path using the checked-in starter profile. */
    public StarterTeleOp() {
        // FTC constructs OpModes through their public no-argument constructor.
    }

    @Override
    protected void configure(RobotProgram program) {
        StarterProfile profile = StarterProfile.current();
        new StarterRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
    }
}
