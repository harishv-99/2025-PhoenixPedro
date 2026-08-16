package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * Immutable rectangle rigidly authored in the Phoenix robot frame.
 *
 * <p>Robot-frame {@code +X} is forward and {@code +Y} is left; all bounds are in inches. This
 * value answers only two literal geometric questions against an {@link AxisAlignedBoxRegion2d}:
 * whether all four transformed corners are inside, and whether any transformed corner is inside.
 * It is not a footprint/support model, collision envelope, occupancy test, or season-scoring
 * decision.</p>
 *
 * <p>The adopting robot must author a rectangle that conservatively represents its relevant
 * physical configuration. A mechanism extending beyond this rectangle is not covered by either
 * predicate.</p>
 */
public final class RobotFrameRectangle2d {

    private final double minXInches;
    private final double maxXInches;
    private final double minYInches;
    private final double maxYInches;

    private RobotFrameRectangle2d(double minXInches,
                                  double maxXInches,
                                  double minYInches,
                                  double maxYInches) {
        this.minXInches = minXInches;
        this.maxXInches = maxXInches;
        this.minYInches = minYInches;
        this.maxYInches = maxYInches;
    }

    /**
     * Creates a rectangle centered on the tracked robot origin.
     *
     * <p>{@code lengthInches} spans robot {@code +X/-X} (forward/back), and
     * {@code widthInches} spans robot {@code +Y/-Y} (left/right).</p>
     *
     * @param lengthInches finite, strictly positive front-to-back length in inches
     * @param widthInches finite, strictly positive left-to-right width in inches
     * @return centered immutable rectangle
     * @throws IllegalArgumentException if a dimension is non-finite, not positive, or too small
     *                                  to represent nondegenerate centered bounds
     */
    public static RobotFrameRectangle2d centeredInches(double lengthInches,
                                                       double widthInches) {
        SpatialValidation.requireFinite(
                "RobotFrameRectangle2d.centeredInches lengthInches",
                lengthInches
        );
        SpatialValidation.requireFinite(
                "RobotFrameRectangle2d.centeredInches widthInches",
                widthInches
        );
        requirePositive("lengthInches", lengthInches);
        requirePositive("widthInches", widthInches);

        double halfLengthInches = lengthInches / 2.0;
        double halfWidthInches = widthInches / 2.0;
        if (halfLengthInches == 0.0) {
            throw new IllegalArgumentException(
                    "RobotFrameRectangle2d.centeredInches lengthInches is too small to "
                            + "represent nondegenerate centered bounds, got " + lengthInches
            );
        }
        if (halfWidthInches == 0.0) {
            throw new IllegalArgumentException(
                    "RobotFrameRectangle2d.centeredInches widthInches is too small to "
                            + "represent nondegenerate centered bounds, got " + widthInches
            );
        }
        return new RobotFrameRectangle2d(
                -halfLengthInches,
                halfLengthInches,
                -halfWidthInches,
                halfWidthInches
        );
    }

    /**
     * Creates a rectangle from explicit bounds in the tracked robot frame.
     *
     * <p>The tracked robot origin need not lie inside these bounds. The named ordering is strict:
     * each minimum must be less than its corresponding maximum rather than being silently
     * reordered.</p>
     *
     * @param minXInches finite minimum robot-frame X bound in inches
     * @param maxXInches finite maximum robot-frame X bound in inches, greater than the minimum
     * @param minYInches finite minimum robot-frame Y bound in inches
     * @param maxYInches finite maximum robot-frame Y bound in inches, greater than the minimum
     * @return immutable robot-frame rectangle
     * @throws IllegalArgumentException if a bound is non-finite or an axis has no positive span
     */
    public static RobotFrameRectangle2d fromRobotFrameBoundsInches(double minXInches,
                                                                   double maxXInches,
                                                                   double minYInches,
                                                                   double maxYInches) {
        SpatialValidation.requireFinite(
                "RobotFrameRectangle2d.fromRobotFrameBoundsInches minXInches",
                minXInches
        );
        SpatialValidation.requireFinite(
                "RobotFrameRectangle2d.fromRobotFrameBoundsInches maxXInches",
                maxXInches
        );
        SpatialValidation.requireFinite(
                "RobotFrameRectangle2d.fromRobotFrameBoundsInches minYInches",
                minYInches
        );
        SpatialValidation.requireFinite(
                "RobotFrameRectangle2d.fromRobotFrameBoundsInches maxYInches",
                maxYInches
        );
        if (!(minXInches < maxXInches)) {
            throw new IllegalArgumentException(
                    "RobotFrameRectangle2d.fromRobotFrameBoundsInches requires minXInches "
                            + "< maxXInches, got minXInches=" + minXInches
                            + ", maxXInches=" + maxXInches
            );
        }
        if (!(minYInches < maxYInches)) {
            throw new IllegalArgumentException(
                    "RobotFrameRectangle2d.fromRobotFrameBoundsInches requires minYInches "
                            + "< maxYInches, got minYInches=" + minYInches
                            + ", maxYInches=" + maxYInches
            );
        }
        return new RobotFrameRectangle2d(
                minXInches,
                maxXInches,
                minYInches,
                maxYInches
        );
    }

    /**
     * Returns whether all four rectangle corners are inside or on {@code fieldBox}.
     *
     * <p>The supplied pose is the field-to-robot transform. A non-finite runtime pose, or
     * non-finite derived corner caused by arithmetic overflow, fails closed with {@code false}.
     * This instantaneous result has no hidden hysteresis or history.</p>
     *
     * @param fieldBox axis-aligned box expressed in the same field frame as the robot pose
     * @param fieldToRobot current field-to-robot pose
     * @return {@code true} only when every transformed corner is inside or on the box
     * @throws NullPointerException if {@code fieldBox} or {@code fieldToRobot} is null
     */
    public boolean fullyInside(AxisAlignedBoxRegion2d fieldBox, Pose2d fieldToRobot) {
        return cornersInside(fieldBox, fieldToRobot, true);
    }

    /**
     * Returns whether at least one rectangle corner is inside or on {@code fieldBox}.
     *
     * <p>This method deliberately does not claim general overlap. A box may lie wholly inside the
     * rectangle, or rectangle edges may cross the box, while no rectangle corner is inside. A
     * non-finite runtime pose, or non-finite derived corner caused by arithmetic overflow, fails
     * closed with {@code false}. This instantaneous result has no hidden hysteresis or history.</p>
     *
     * @param fieldBox axis-aligned box expressed in the same field frame as the robot pose
     * @param fieldToRobot current field-to-robot pose
     * @return {@code true} only when at least one transformed corner is inside or on the box
     * @throws NullPointerException if {@code fieldBox} or {@code fieldToRobot} is null
     */
    public boolean hasAnyCornerInside(AxisAlignedBoxRegion2d fieldBox, Pose2d fieldToRobot) {
        return cornersInside(fieldBox, fieldToRobot, false);
    }

    /**
     * Returns a compact diagnostic description of the authored robot-frame bounds.
     */
    @Override
    public String toString() {
        return "RobotFrameRectangle2d{" +
                "minXInches=" + minXInches +
                ", maxXInches=" + maxXInches +
                ", minYInches=" + minYInches +
                ", maxYInches=" + maxYInches +
                '}';
    }

    private boolean cornersInside(AxisAlignedBoxRegion2d fieldBox,
                                  Pose2d fieldToRobot,
                                  boolean requireAll) {
        Objects.requireNonNull(fieldBox, "fieldBox");
        Objects.requireNonNull(fieldToRobot, "fieldToRobot");
        if (!SpatialValidation.isFinite(fieldToRobot)) {
            return false;
        }

        double cos = Math.cos(fieldToRobot.headingRad);
        double sin = Math.sin(fieldToRobot.headingRad);
        double x0 = transformedX(fieldToRobot, cos, sin, minXInches, minYInches);
        double y0 = transformedY(fieldToRobot, cos, sin, minXInches, minYInches);
        double x1 = transformedX(fieldToRobot, cos, sin, minXInches, maxYInches);
        double y1 = transformedY(fieldToRobot, cos, sin, minXInches, maxYInches);
        double x2 = transformedX(fieldToRobot, cos, sin, maxXInches, minYInches);
        double y2 = transformedY(fieldToRobot, cos, sin, maxXInches, minYInches);
        double x3 = transformedX(fieldToRobot, cos, sin, maxXInches, maxYInches);
        double y3 = transformedY(fieldToRobot, cos, sin, maxXInches, maxYInches);

        if (!allFinite(x0, y0, x1, y1, x2, y2, x3, y3)) {
            return false;
        }

        boolean corner0Inside = fieldBox.contains(x0, y0);
        boolean corner1Inside = fieldBox.contains(x1, y1);
        boolean corner2Inside = fieldBox.contains(x2, y2);
        boolean corner3Inside = fieldBox.contains(x3, y3);
        return requireAll
                ? corner0Inside && corner1Inside && corner2Inside && corner3Inside
                : corner0Inside || corner1Inside || corner2Inside || corner3Inside;
    }

    private static void requirePositive(String fieldName, double value) {
        if (value <= 0.0) {
            throw new IllegalArgumentException(
                    "RobotFrameRectangle2d.centeredInches " + fieldName
                            + " must be > 0, got " + value
            );
        }
    }

    private static double transformedX(Pose2d fieldToRobot,
                                       double cos,
                                       double sin,
                                       double robotXInches,
                                       double robotYInches) {
        return fieldToRobot.xInches
                + cos * robotXInches
                - sin * robotYInches;
    }

    private static double transformedY(Pose2d fieldToRobot,
                                       double cos,
                                       double sin,
                                       double robotXInches,
                                       double robotYInches) {
        return fieldToRobot.yInches
                + sin * robotXInches
                + cos * robotYInches;
    }

    private static boolean allFinite(double... values) {
        for (double value : values) {
            if (!Double.isFinite(value)) {
                return false;
            }
        }
        return true;
    }
}
