package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.robots.phoenix.PhoenixRobot;

/**
 * Main Phoenix TeleOp Driver Station entry.
 *
 * <p>The OpMode is intentionally thin: FTC lifecycle calls are forwarded to {@link PhoenixRobot},
 * while all gamepad meanings live in Phoenix's TeleOp controls object and all subsystem behavior
 * lives inside the robot container.</p>
 */
@TeleOp(name = "Phoenix TeleOp", group = "Phoenix")
public final class PhoenixTeleOp extends OpMode {
    private PhoenixRobot robot;

    /**
     * Build the Phoenix TeleOp runtime.
     */
    @Override
    public void init() {
        robot = new PhoenixRobot(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.initAny();
        robot.initTeleOp();
    }

    /**
     * Reset shared loop timing and start TeleOp-specific state.
     */
    @Override
    public void start() {
        robot.startAny(getRuntime());
        robot.startTeleOp();
    }

    /**
     * Advance one TeleOp loop.
     */
    @Override
    public void loop() {
        robot.updateAny(getRuntime());
        robot.updateTeleOp();
    }

    /**
     * Stop TeleOp-specific and shared Phoenix resources.
     */
    @Override
    public void stop() {
        if (robot != null) {
            robot.stopTeleOp();
            robot.stopAny();
            robot = null;
        }
    }
}
