package edu.ftcphoenix.fw.spatial;

import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * Factory/builder for robot-aware zone checks.
 *
 * <p>This class answers questions like:</p>
 * <ul>
 *   <li>"Is a specific point on the robot inside a zone?"</li>
 *   <li>"Does the robot footprint overlap a zone at all?" (approximate)</li>
 *   <li>"Is the robot footprint fully inside a zone?"</li>
 * </ul>
 *
 * <p>Most APIs in this class assume the zone is <b>convex</b> (see {@link ConvexRegion2d}).
 * Convexity lets Phoenix implement "fully inside" using only footprint vertices.</p>
 */
public final class RobotZones2d {

    private RobotZones2d() {
        // utility
    }

    /**
     * Start a fluent builder for creating a {@link RobotZone2d} against a specific region.
     */
    public static InRegion in(ConvexRegion2d region) {
        return new InRegion(region);
    }

    /**
     * Signed-distance rule: a named robot point must be inside the region.
     */
    public static RobotZone2d robotPointInside(final ConvexRegion2d region, final Pose2d robotToPoint) {
        Objects.requireNonNull(region, "region");
        Objects.requireNonNull(robotToPoint, "robotToPoint");
        return fieldToRobot -> {
            Pose2d fieldToPoint = fieldToRobot.then(new Pose2d(robotToPoint.xInches, robotToPoint.yInches, 0.0));
            return region.signedDistanceInches(fieldToPoint.xInches, fieldToPoint.yInches);
        };
    }

    /**
     * Signed-distance rule: the robot footprint must be fully inside the region.
     */
    public static RobotZone2d footprintFullyInside(final ConvexRegion2d region, final RobotFootprint2d footprint) {
        Objects.requireNonNull(region, "region");
        Objects.requireNonNull(footprint, "footprint");

        if (footprint instanceof CircleFootprint2d) {
            final double r = ((CircleFootprint2d) footprint).radiusInches;
            return fieldToRobot -> region.signedDistanceInches(fieldToRobot.xInches, fieldToRobot.yInches) - r;
        }
        if (!(footprint instanceof RectangleFootprint2d)) {
            throw new IllegalArgumentException("Unsupported footprint type: " + footprint.getClass().getSimpleName());
        }
        final RectangleFootprint2d rect = (RectangleFootprint2d) footprint;
        final Pose2d[] corners = rect.cornersRobotFrame();

        return fieldToRobot -> {
            double min = Double.POSITIVE_INFINITY;
            for (Pose2d corner : corners) {
                Pose2d fieldToCorner = fieldToRobot.then(corner);
                double d = region.signedDistanceInches(fieldToCorner.xInches, fieldToCorner.yInches);
                if (d < min) {
                    min = d;
                }
            }
            return min;
        };
    }

    /**
     * Signed-distance rule: the robot footprint overlaps the region.
     *
     * <p>For circle footprints, this uses an analytic test. For rectangles, this uses probe points around
     * the boundary (see {@link RectangleFootprint2d#boundaryProbePointsRobotFrame(int)}).</p>
     */
    public static RobotZone2d footprintOverlaps(final ConvexRegion2d region,
                                                final RobotFootprint2d footprint,
                                                final int samplesPerEdge) {
        Objects.requireNonNull(region, "region");
        Objects.requireNonNull(footprint, "footprint");

        if (footprint instanceof CircleFootprint2d) {
            final double r = ((CircleFootprint2d) footprint).radiusInches;
            return fieldToRobot -> region.signedDistanceInches(fieldToRobot.xInches, fieldToRobot.yInches) + r;
        }
        if (!(footprint instanceof RectangleFootprint2d)) {
            throw new IllegalArgumentException("Unsupported footprint type: " + footprint.getClass().getSimpleName());
        }
        final RectangleFootprint2d rect = (RectangleFootprint2d) footprint;
        final List<Pose2d> probes = rect.boundaryProbePointsRobotFrame(samplesPerEdge);

        return fieldToRobot -> {
            double max = Double.NEGATIVE_INFINITY;
            for (Pose2d probe : probes) {
                Pose2d fieldToProbe = fieldToRobot.then(probe);
                double d = region.signedDistanceInches(fieldToProbe.xInches, fieldToProbe.yInches);
                if (d > max) {
                    max = d;
                }
            }
            return max;
        };
    }

    // ---------------------------------------------------------------------
    // Fluent builder
    // ---------------------------------------------------------------------

    /**
     * Builder stage: region fixed, choose robot geometry.
     */
    public static final class InRegion {
        private final ConvexRegion2d region;

        private InRegion(ConvexRegion2d region) {
            this.region = Objects.requireNonNull(region, "region");
        }

        /**
         * Chooses the robot geometry used by the next builder stage.
         */
        public ForRobot robot(RobotGeometry2d robot) {
            return new ForRobot(region, robot);
        }
    }

    /**
     * Builder stage: build specific robot zone rules.
     */
    public static final class ForRobot {
        private final ConvexRegion2d region;
        private final RobotGeometry2d robot;

        private ForRobot(ConvexRegion2d region, RobotGeometry2d robot) {
            this.region = Objects.requireNonNull(region, "region");
            this.robot = Objects.requireNonNull(robot, "robot");
        }

        /**
         * Create a zone rule for a named robot point.
         */
        public RobotZone2d pointInside(String pointName) {
            Pose2d robotToPoint = robot.robotToPointOrThrow(pointName);
            return robotPointInside(region, robotToPoint);
        }

        /**
         * Create a zone rule requiring the full footprint to be inside the region.
         */
        public RobotZone2d footprintFullyInside() {
            return RobotZones2d.footprintFullyInside(region, robot.footprint());
        }

        /**
         * Create a zone rule requiring the footprint to overlap the region.
         *
         * <p>For rectangles, {@code samplesPerEdge} controls the accuracy vs cost tradeoff.</p>
         */
        public RobotZone2d footprintOverlaps(int samplesPerEdge) {
            return RobotZones2d.footprintOverlaps(region, robot.footprint(), samplesPerEdge);
        }
    }
}
