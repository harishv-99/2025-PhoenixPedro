package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.HysteresisLatch;
import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * Hysteresis latch for heading-based predicates.
 *
 * <p>This is the heading analogue of {@link ZoneLatch}:</p>
 * <ul>
 *   <li>{@link ZoneLatch} turns a signed distance-to-region into a stable boolean “in zone?”</li>
 *   <li>{@link HeadingLatch} turns a heading error signal into a stable boolean “facing target?”</li>
 * </ul>
 *
 * <p>It uses {@link HysteresisLatch} to avoid chatter when the heading error hovers around a threshold.</p>
 *
 * <p><b>Thresholds:</b></p>
 * <ul>
 *   <li>Turns ON when {@code |error| <= enterTolRad}</li>
 *   <li>Turns OFF when {@code |error| >= exitTolRad}</li>
 * </ul>
 *
 * <p>This is the common pattern: enter tolerance is tighter than exit tolerance.</p>
 */
public final class HeadingLatch {

    private final RobotHeading2d heading;
    private final HysteresisLatch latch;

    /**
     * Create a latch using the provided heading error rule.
     *
     * @param heading     heading error rule
     * @param enterTolRad tolerance (radians) to enter the “ON” state
     * @param exitTolRad  tolerance (radians) to leave the “ON” state
     */
    public HeadingLatch(RobotHeading2d heading, double enterTolRad, double exitTolRad) {
        this.heading = Objects.requireNonNull(heading, "heading");
        this.latch = HysteresisLatch.onWhenBelowOffWhenAbove(enterTolRad, exitTolRad);
    }

    /**
     * Convenience: create a latch for “robot center faces field heading”.
     */
    public static HeadingLatch robotCenterToFieldHeading(double targetFieldHeadingRad,
                                                         double enterTolRad,
                                                         double exitTolRad) {
        return new HeadingLatch(
                RobotHeadings2d.robotCenter().toFieldHeadingRad(targetFieldHeadingRad),
                enterTolRad,
                exitTolRad
        );
    }

    /**
     * Update the latch using the current robot pose.
     *
     * @param fieldToRobot robot pose in the field frame
     * @return latched state
     */
    public boolean update(Pose2d fieldToRobot) {
        double err = heading.errorRad(fieldToRobot);
        return latch.update(Math.abs(err));
    }

    /**
     * @return current latched state
     */
    public boolean get() {
        return latch.get();
    }

    /**
     * Reset the latch OFF.
     */
    public void resetOff() {
        latch.reset(false);
    }
}
