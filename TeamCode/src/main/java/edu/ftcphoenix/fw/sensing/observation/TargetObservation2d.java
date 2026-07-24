package edu.ftcphoenix.fw.sensing.observation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;

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
 * <h2>Quality and capture time</h2>
 * <p>Quality is a unitless score in [0, 1] where 1 is “very confident”. The meaning is sensor-specific
 * but the gating logic in higher-level code typically uses it as a simple threshold. The epoch-safe
 * {@link #timestamp} keeps observation freshness valid across deliberate clock resets without
 * requiring consumers to carry a separate epoch.</p>
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

    /** Epoch-safe timestamp of this measurement. */
    public final LoopTimestamp timestamp;

    private TargetObservation2d(boolean hasTarget,
                                int targetId,
                                double forwardInches,
                                double leftInches,
                                double bearingRad,
                                double targetHeadingRad,
                                double quality,
                                LoopTimestamp timestamp) {
        this.hasTarget = hasTarget;
        this.targetId = targetId;
        this.forwardInches = forwardInches;
        this.leftInches = leftInches;
        this.bearingRad = bearingRad;
        this.targetHeadingRad = targetHeadingRad;
        this.quality = MathUtil.clamp(quality, 0.0, 1.0);
        this.timestamp = Objects.requireNonNull(timestamp, "timestamp");
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
                LoopTimestamp.unavailable()
        );
    }

    /**
     * Create an observation with full 2D position.
     */
    public static TargetObservation2d ofRobotRelativePosition(double forwardInches,
                                                              double leftInches,
                                                              double quality,
                                                              LoopTimestamp timestamp) {
        return ofRobotRelativePosition(-1, forwardInches, leftInches, quality, timestamp);
    }

    /**
     * Create an observation with full 2D position and a target ID.
     */
    public static TargetObservation2d ofRobotRelativePosition(int targetId,
                                                              double forwardInches,
                                                              double leftInches,
                                                              double quality,
                                                              LoopTimestamp timestamp) {
        double bearing = Math.atan2(leftInches, forwardInches);
        return new TargetObservation2d(
                true,
                targetId,
                forwardInches,
                leftInches,
                bearing,
                Double.NaN,
                quality,
                timestamp
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
                                                          LoopTimestamp timestamp) {
        double bearing = Math.atan2(leftInches, forwardInches);
        return new TargetObservation2d(
                true,
                targetId,
                forwardInches,
                leftInches,
                bearing,
                targetHeadingRad,
                quality,
                timestamp
        );
    }

    /**
     * Create an observation that only has bearing.
     */
    public static TargetObservation2d ofRobotRelativeBearing(double bearingRad,
                                                             double quality,
                                                             LoopTimestamp timestamp) {
        return ofRobotRelativeBearing(-1, bearingRad, quality, timestamp);
    }

    /**
     * Create an observation that only has bearing, with a target ID.
     */
    public static TargetObservation2d ofRobotRelativeBearing(int targetId,
                                                             double bearingRad,
                                                             double quality,
                                                             LoopTimestamp timestamp) {
        return new TargetObservation2d(
                true,
                targetId,
                Double.NaN,
                Double.NaN,
                bearingRad,
                Double.NaN,
                quality,
                timestamp
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

    /** Returns this observation's current age, or NaN when its timestamp is no longer valid. */
    public double ageSec(LoopClock clock) {
        return timestamp.ageSec(clock);
    }

    /** Returns whether this observation is valid now and within the inclusive maximum age. */
    public boolean isFresh(LoopClock clock, double maxAgeSec) {
        return timestamp.isFresh(clock, maxAgeSec);
    }

    /**
     * {@inheritDoc}
     */
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
                + ", timestamp=" + timestamp
                + "}";
    }
}
