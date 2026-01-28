package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.HysteresisBoolean;
import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * A convenience wrapper that answers “is the robot in this zone?” with hysteresis.
 *
 * <p>This is a <b>spatial predicate</b>. It does not create motion commands. It simply
 * turns a signed-distance zone test ({@link RobotZone2d}) into a stable boolean suitable for
 * safety gating and state machines.</p>
 */
public final class ZoneLatch {

    private final RobotZone2d zone;
    private final HysteresisBoolean latch;

    /**
     * Creates a zone latch using signed-distance hysteresis.
     *
     * <p>Because {@link RobotZone2d#signedDistanceInches(Pose2d)} is positive inside,
     * you typically use something like:</p>
     * <ul>
     *   <li>{@code enterDistanceInches = +2} (must be at least 2" inside to turn on)</li>
     *   <li>{@code exitDistanceInches = -2} (must be at least 2" outside to turn off)</li>
     * </ul>
     *
     * @param zone                signed-distance robot zone rule
     * @param enterDistanceInches latch turns ON when signedDistance >= enterDistanceInches
     * @param exitDistanceInches  latch turns OFF when signedDistance <= exitDistanceInches
     */
    public ZoneLatch(RobotZone2d zone, double enterDistanceInches, double exitDistanceInches) {
        this.zone = Objects.requireNonNull(zone, "zone");
        this.latch = HysteresisBoolean.onWhenAboveOffWhenBelow(enterDistanceInches, exitDistanceInches);
    }

    /**
     * Convenience: create a latch for a {@link Region2d} using the robot center point.
     */
    public static ZoneLatch forRegion(Region2d region, double enterDistanceInches, double exitDistanceInches) {
        Objects.requireNonNull(region, "region");
        return new ZoneLatch(
                fieldToRobot -> region.signedDistanceInches(fieldToRobot.xInches, fieldToRobot.yInches),
                enterDistanceInches,
                exitDistanceInches
        );
    }

    /**
     * Update the latch with the current robot pose.
     */
    public boolean update(Pose2d fieldToRobot) {
        double s = zone.signedDistanceInches(fieldToRobot);
        return latch.update(s);
    }

    /**
     * @return current latched state
     */
    public boolean get() {
        return latch.get();
    }

    /**
     * Reset to OFF.
     */
    public void resetOff() {
        latch.reset(false);
    }
}
