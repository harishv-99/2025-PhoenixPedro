package edu.ftcsushi.robots.examples.fieldrelative.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.robots.examples.fieldrelative.robot.FieldRelativeExampleProfile;
import edu.ftcsushi.robots.examples.fieldrelative.robot.FieldRelativeExampleRobot;

/** Managed example of explicit station-relative TeleOp drive using the Hub IMU. */
@TeleOp(name = "FW Example: Field-relative drive", group = "FW Examples")
@Disabled
public final class FieldRelativeDriveExample extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        new FieldRelativeExampleRobot(hardwareMap).declareTeleOp(
                program, FieldRelativeExampleProfile.current(), gamepad1);
    }
}
