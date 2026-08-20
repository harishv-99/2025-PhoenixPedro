package edu.ftcphoenix.robots.examples.reference.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.robots.examples.reference.robot.ReferenceProfile;
import edu.ftcphoenix.robots.examples.reference.robot.ReferenceRobot;

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
