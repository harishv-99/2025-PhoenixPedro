package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.robots.phoenix.PhoenixRobot;

@TeleOp(name = "Phoenix TeleOp", group = "Phoenix")
public class PhoenixTeleOp extends OpMode {
    private PhoenixRobot robot;

    @Override
    public void init() {

        // Create the season robot and share its bindings.
        robot = new PhoenixRobot(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.initAny();
        robot.initTeleOp();
    }

    @Override
    public void start() {
        robot.startAny(getRuntime());
        robot.startTeleOp();
    }

    @Override
    public void loop() {
        robot.updateAny(getRuntime());
        robot.updateTeleOp();
    }

    @Override
    public void stop() {
        robot.stopTeleOp();
        robot.stopAny();
    }
}