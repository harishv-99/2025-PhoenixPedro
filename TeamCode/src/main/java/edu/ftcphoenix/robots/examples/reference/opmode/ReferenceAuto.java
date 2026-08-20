package edu.ftcphoenix.robots.examples.reference.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.robots.examples.reference.robot.ReferenceProfile;
import edu.ftcphoenix.robots.examples.reference.robot.ReferenceRobot;

/** Thin FTC host proving that Auto and TeleOp share the same reference capabilities. */
@Autonomous(name = "FW Reference: Auto", group = "FW Examples")
@Disabled
public final class ReferenceAuto extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        new ReferenceRobot(hardwareMap)
                .declareAuto(program, ReferenceProfile.current());
    }
}
