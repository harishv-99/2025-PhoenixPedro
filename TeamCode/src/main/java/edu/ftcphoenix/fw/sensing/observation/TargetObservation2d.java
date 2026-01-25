package edu.ftcphoenix.fw.sensing.observation;

import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * A lightweight 2D observation of a target relative to the robot.
 *
 * <p>This is intentionally generic: it can represent an AprilTag observation, an object detection
 * observation, or any other sensor-provided target measurement. It describes the target's
 * location in the robot's horizontal plane using Phoenix conventions:</p>
 * <ul>
 *   <li>+X (forwardInches) is forward out the front of the robot.</li>
 *   <li>+Y (leftInches) is left of the robot.</li>
 *   <li>bearingRad is CCW-positive, with 0 meaning straight ahead.</li>
 * </ul>
 *
 * <h2>Bearing-only vs position observations</h2>
 * <p>Some sensors can only provide bearing (direction) and not full position. Phoenix represents
 * this by setting {@link #forwardInches} and {@link #leftInches} to {@link Double#NaN} while still
 * providing {@link #bearingRad}. Use {@link #hasPosition()} to check.</p>
 *
 * <h2>Quality and age</h2>
 * <p>Quality is a unitless score in [0, 1] where 1 is “very confident”. The meaning is sensor-specific
 * but the gating logic in higher-level code typically uses it as a simple threshold.</p>
 */
public final class TargetObservation2d {

    /**
     * Whether the target is currently detected.
     */
    public final boolean hasTarget;

    /**
     * Optional ID for the observed target.
     *
     * <p>For AprilTags, this is the tag ID. For other detectors, it can be a class ID or tracker ID.
     * When unknown or not applicable, this is {@code -1}.</p>
     */
    public final int targetId;

    /**
     * Target X position in robot frame (inches). May be NaN if bearing-only.
     */
    public final double forwardInches;

    /**
     * Target Y position in robot frame (inches). May be NaN if bearing-only.
     */
    public final double leftInches;

    /**
     * Bearing to target in robot frame (radians). CCW-positive.
     */
    public final double bearingRad;

    /**
     * Optional heading of the target's reference frame in the robot frame (radians).
     *
     * <p>This is most relevant for AprilTags where the tag has a well-defined coordinate frame.
     * When available, it represents the yaw/heading of the tag's +X axis as seen from the robot.
     *
     * <p>For generic object detections (balls, cones, etc.), this is often unknown and will be
     * {@link Double#NaN}.
     */
    public final double targetHeadingRad;

    /**
     * Confidence score in [0, 1].
     */
    public final double quality;

    /**
     * Age of this sample in seconds (0 means fresh).
     */
    public final double ageSec;

    private TargetObservation2d(boolean hasTarget,
                                int targetId,
                                double forwardInches,
                                double leftInches,
                                double bearingRad,
                                double targetHeadingRad,
                                double quality,
                                double ageSec) {
        this.hasTarget = hasTarget;
        this.targetId = targetId;
        this.forwardInches = forwardInches;
        this.leftInches = leftInches;
        this.bearingRad = bearingRad;
        this.targetHeadingRad = targetHeadingRad;
        this.quality = MathUtil.clamp(quality, 0.0, 1.0);
        this.ageSec = Math.max(0.0, ageSec);
    }

    /**
     * Return a "no target" observation.
     */
    public static TargetObservation2d none() {
        return new TargetObservation2d(
                false,
                -1,
                Double.NaN,
                Double.NaN,
                0.0,
                Double.NaN,
                0.0,
                Double.POSITIVE_INFINITY
        );
    }

    /**
     * Create an observation with full 2D position.
     */
    public static TargetObservation2d ofRobotRelativePosition(double forwardInches,
                                                              double leftInches,
                                                              double quality,
                                                              double ageSec) {
        return ofRobotRelativePosition(-1, forwardInches, leftInches, quality, ageSec);
    }

    /**
     * Create an observation with full 2D position and a target ID.
     */
    public static TargetObservation2d ofRobotRelativePosition(int targetId,
                                                              double forwardInches,
                                                              double leftInches,
                                                              double quality,
                                                              double ageSec) {
        double bearing = Math.atan2(leftInches, forwardInches);
        return new TargetObservation2d(
                true,
                targetId,
                forwardInches,
                leftInches,
                bearing,
                Double.NaN,
                quality,
                ageSec
        );
    }

    /**
     * Create an observation with full 2D position and an explicit target-frame heading.
     *
     * <p>This is the most complete form of a planar observation: it describes the target origin
     * position and also the orientation of the target's +X axis. For AprilTags, this heading is
     * the tag's yaw as seen in the robot frame.</p>
     */
    public static TargetObservation2d ofRobotRelativePose(int targetId,
                                                          double forwardInches,
                                                          double leftInches,
                                                          double targetHeadingRad,
                                                          double quality,
                                                          double ageSec) {
        double bearing = Math.atan2(leftInches, forwardInches);
        return new TargetObservation2d(
                true,
                targetId,
                forwardInches,
                leftInches,
                bearing,
                targetHeadingRad,
                quality,
                ageSec
        );
    }

    /**
     * Create an observation that only has bearing.
     */
    public static TargetObservation2d ofRobotRelativeBearing(double bearingRad,
                                                             double quality,
                                                             double ageSec) {
        return ofRobotRelativeBearing(-1, bearingRad, quality, ageSec);
    }

    /**
     * Create an observation that only has bearing, with a target ID.
     */
    public static TargetObservation2d ofRobotRelativeBearing(int targetId,
                                                             double bearingRad,
                                                             double quality,
                                                             double ageSec) {
        return new TargetObservation2d(
                true,
                targetId,
                Double.NaN,
                Double.NaN,
                bearingRad,
                Double.NaN,
                quality,
                ageSec
        );
    }

    /**
     * True if this observation includes full position (not just bearing).
     */
    public boolean hasPosition() {
        return hasTarget && Double.isFinite(forwardInches) && Double.isFinite(leftInches);
    }

    /**
     * True if this observation includes a target-frame heading.
     */
    public boolean hasOrientation() {
        return hasTarget && Double.isFinite(targetHeadingRad);
    }

    /**
     * True if this observation includes a non-negative {@link #targetId}.
     */
    public boolean hasTargetId() {
        return hasTarget && targetId >= 0;
    }

    @Override
    public String toString() {
        if (!hasTarget) {
            return "TargetObservation2d{none}";
        }
        String pos = hasPosition()
                ? ("pos=(" + forwardInches + "," + leftInches + ")")
                : "pos=(bearing-only)";
        String id = hasTargetId() ? (", id=" + targetId) : "";
        return "TargetObservation2d{" + pos
                + id
                + ", bearingRad=" + bearingRad
                + (hasOrientation() ? (", targetHeadingRad=" + targetHeadingRad) : "")
                + ", quality=" + quality
                + ", ageSec=" + ageSec
                + "}";
    }
}
