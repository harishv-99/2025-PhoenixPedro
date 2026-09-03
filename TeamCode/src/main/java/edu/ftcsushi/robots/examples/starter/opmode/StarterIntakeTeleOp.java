package edu.ftcsushi.robots.examples.starter.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.robots.examples.starter.robot.StarterProfile;
import edu.ftcsushi.robots.examples.starter.robot.StarterRobot;

/** Focused intake-only host for learning one continuous-power mechanism before integration. */
@TeleOp(name = "FW Starter: Intake only", group = "FW Examples")
@Disabled
public final class StarterIntakeTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        StarterProfile profile = StarterProfile.current();
        new StarterRobot(hardwareMap).declareIntakeTeleOp(program, profile, gamepad1);
    }
}
