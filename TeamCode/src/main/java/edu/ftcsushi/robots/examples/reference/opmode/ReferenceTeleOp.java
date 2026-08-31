package edu.ftcsushi.robots.examples.reference.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.robots.examples.reference.robot.ReferenceProfile;
import edu.ftcsushi.robots.examples.reference.robot.ReferenceRobot;

/** Thin FTC host for the season-independent reference TeleOp. */
@TeleOp(name = "FW Reference: TeleOp", group = "FW Examples")
@Disabled
public final class ReferenceTeleOp extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        new ReferenceRobot(hardwareMap)
                .declareTeleOp(program, ReferenceProfile.current(), gamepad1);
    }
}
