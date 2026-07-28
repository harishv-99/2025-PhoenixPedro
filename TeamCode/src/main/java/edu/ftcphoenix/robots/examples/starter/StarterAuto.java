package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/** Tiny Auto that uses the same intake capability as the starter TeleOp. */
@Autonomous(name = "FW Starter: Auto", group = "FW Examples")
@Disabled
public final class StarterAuto extends OpMode {

    private static final double COLLECT_DURATION_SEC = 0.75;

    private StarterRobot robot;

    @Override
    public void init() {
        robot = new StarterRobot(hardwareMap, telemetry, StarterProfile.current());
        robot.initAuto();
        robot.installAutoRoutine(robot.intake().collectForSeconds(COLLECT_DURATION_SEC));
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
