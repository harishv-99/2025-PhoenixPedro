package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Thin FTC host for the modern starter TeleOp reference. */
@TeleOp(name = "FW Starter: TeleOp", group = "FW Examples")
@Disabled
public final class StarterTeleOp extends OpMode {

    private StarterRobot robot;

    @Override
    public void init() {
        robot = new StarterRobot(hardwareMap, telemetry, StarterProfile.current());
        robot.initTeleOp(gamepad1);
    }

    @Override
    public void start() {
        robot.start(getRuntime());
    }

    @Override
    public void loop() {
        robot.update(getRuntime());
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stop();
        }
    }
}
