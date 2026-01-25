package edu.ftcphoenix.fw.robot;

import edu.ftcphoenix.fw.util.LoopClock;

/**
 * Simple lifecycle interface for robot subsystems.
 *
 * <p>Subsystems are owned by a robot class (e.g. PhoenixRobot) and receive
 * lifecycle callbacks from TeleOp and Autonomous code.
 *
 * <p>All time-aware methods receive a {@link LoopClock}, which is the single
 * source of truth for loop timing.
 */
public interface Subsystem {

    /**
     * Called once when TeleOp starts.
     */
    default void onTeleopInit() {
    }

    /**
     * Called every loop during TeleOp.
     */
    default void onTeleopLoop(LoopClock clock) {
    }

    /**
     * Called once when Autonomous starts.
     */
    default void onAutoInit() {
    }

    /**
     * Called every loop during Autonomous.
     */
    default void onAutoLoop(LoopClock clock) {
    }

    /**
     * Called when the OpMode is stopping (TeleOp or Autonomous).
     */
    default void onStop() {
    }
}
