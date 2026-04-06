package edu.ftcphoenix.fw.spatial;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * Simple 2D geometry model of the robot for spatial predicates (zones).
 *
 * <p>This class is intentionally lightweight. It supports the two most common needs:</p>
 * <ul>
 *   <li>a footprint approximation (rectangle or circle) for "overlap" and "fully inside" tests</li>
 *   <li>named robot-relative points (e.g., "shooter", "intake") for point-in-zone tests</li>
 * </ul>
 */
public final class RobotGeometry2d {

    private final RobotFootprint2d footprint;
    private final Map<String, Pose2d> robotToNamedPoints;

    private RobotGeometry2d(RobotFootprint2d footprint, Map<String, Pose2d> robotToNamedPoints) {
        this.footprint = Objects.requireNonNull(footprint, "footprint");
        this.robotToNamedPoints = Collections.unmodifiableMap(new HashMap<String, Pose2d>(robotToNamedPoints));
    }

    /**
     * Start building a robot geometry.
     */
    public static FootprintStep builder() {
        return new Builder();
    }

    /**
     * @return the configured robot footprint model.
     */
    public RobotFootprint2d footprint() {
        return footprint;
    }

    /**
     * @return an unmodifiable map of point name -> robot→point transform (heading is ignored).
     */
    public Map<String, Pose2d> namedPoints() {
        return robotToNamedPoints;
    }

    /**
     * Get a named robot-relative point.
     *
     * @throws IllegalArgumentException if the name is not present
     */
    public Pose2d robotToPointOrThrow(String name) {
        Pose2d p = robotToNamedPoints.get(name);
        if (p == null) {
            throw new IllegalArgumentException("Unknown robot point: '" + name + "'. Known: " + robotToNamedPoints.keySet());
        }
        return p;
    }

    /**
     * Builder stage: choose the footprint model.
     */
    public interface FootprintStep {

        /**
         * Model the robot as a rectangle centered on the robot origin.
         */
        BuildStep rectangleFootprint(double lengthInches, double widthInches);

        /**
         * Model the robot as a circle centered on the robot origin.
         */
        BuildStep circleFootprint(double radiusInches);
    }

    /**
     * Builder stage: add named points and build.
     */
    public interface BuildStep {

        /**
         * Add a named point in robot coordinates (inches). Heading is assumed 0.
         */
        BuildStep point(String name, double forwardInches, double leftInches);

        /**
         * Add a named point using a robot→point transform. Heading is ignored.
         */
        BuildStep point(String name, Pose2d robotToPoint);

        /**
         * Build the immutable geometry.
         */
        RobotGeometry2d build();
    }

    private static final class Builder implements FootprintStep, BuildStep {

        private RobotFootprint2d footprint;
        private final Map<String, Pose2d> points = new HashMap<String, Pose2d>();

        /**
         * {@inheritDoc}
         */
        @Override
        public BuildStep rectangleFootprint(double lengthInches, double widthInches) {
            this.footprint = new RectangleFootprint2d(lengthInches, widthInches);
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public BuildStep circleFootprint(double radiusInches) {
            this.footprint = new CircleFootprint2d(radiusInches);
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public BuildStep point(String name, double forwardInches, double leftInches) {
            return point(name, new Pose2d(forwardInches, leftInches, 0.0));
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public BuildStep point(String name, Pose2d robotToPoint) {
            Objects.requireNonNull(name, "name");
            Objects.requireNonNull(robotToPoint, "robotToPoint");
            points.put(name, new Pose2d(robotToPoint.xInches, robotToPoint.yInches, 0.0));
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public RobotGeometry2d build() {
            if (footprint == null) {
                throw new IllegalStateException("RobotGeometry2d requires a footprint");
            }
            return new RobotGeometry2d(footprint, points);
        }
    }
}
