package edu.ftcsushi.fw.sensing.observation;

import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;

/**
 * An immutable 2D observation relative to the robot at capture, not at a later read.
 *
 * <p>This is intentionally generic: it can represent an AprilTag observation, an object detection
 * observation, or any other sensor-provided target measurement. It describes the target's
 * location in the robot's horizontal plane using Sushi conventions:</p>
 * <ul>
 *   <li>+X (forwardInches) is forward out the front of the robot.</li>
 *   <li>+Y (leftInches) is left of the robot.</li>
 *   <li>bearingRad is CCW-positive, with 0 meaning straight ahead.</li>
 * </ul>
 *
 * <h2>Bearing-only vs position observations</h2>
 * <p>Some sensors can only provide bearing (direction) and not full position. Sushi represents
 * this by setting {@link #forwardInches} and {@link #leftInches} to {@link Double#NaN} while still
 * providing {@link #bearingRad}. Use {@link #hasPosition()} to check.</p>
 *
 * <h2>Quality and capture time</h2>
 * <p>Quality is NaN when unknown, otherwise a score in [0, 1] where 1 is “very confident”. Geometric
 * validity never manufactures confidence. The meaning is sensor-specific
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
     * <p>This requires producer-supported stable identity, such as an AprilTag ID. A blob's list
     * index or object class is not such an identity. Unknown identity is {@code -1}.</p>
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
     * Confidence score in [0, 1], or NaN when unknown.
     */
    public final double quality;

    /** Epoch-safe timestamp of this measurement. */
    public final LoopTimestamp timestamp;

    /** Optional field X in inches; NaN without a usable capture-time pose lookup. */
    public final double fieldXInches;
    /** Optional field Y in inches; NaN without a usable capture-time pose lookup. */
    public final double fieldYInches;
    private final PlanarPoseHistory.Lookup fieldLookup;
    private final String fieldProjectionReason;

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
        if (!Double.isNaN(quality) && (!Double.isFinite(quality) || quality < 0 || quality > 1)) {
            throw new IllegalArgumentException("quality must be NaN (unknown) or within [0, 1]");
        }
        if (targetId < -1) throw new IllegalArgumentException("targetId must be -1 or non-negative");
        this.quality = quality;
        this.timestamp = Objects.requireNonNull(timestamp, "timestamp");
        this.fieldXInches = Double.NaN;
        this.fieldYInches = Double.NaN;
        this.fieldLookup = null;
        this.fieldProjectionReason = "field projection not requested";
    }

    private TargetObservation2d(TargetObservation2d original, double fieldX, double fieldY,
                                PlanarPoseHistory.Lookup lookup, String reason) {
        hasTarget = original.hasTarget;
        targetId = original.targetId;
        forwardInches = original.forwardInches;
        leftInches = original.leftInches;
        bearingRad = original.bearingRad;
        targetHeadingRad = original.targetHeadingRad;
        quality = original.quality;
        timestamp = original.timestamp;
        fieldXInches = fieldX;
        fieldYInches = fieldY;
        fieldLookup = lookup;
        fieldProjectionReason = reason;
    }

    /**
     * Attaches an available or failed capture-time lookup without changing robot geometry,
     * detector confidence, or the original timestamp. Failed lookup/overflow retains its reason
     * and publishes no field position. Does not read or reset history.
     *
     * @throws IllegalArgumentException if the lookup did not retain this exact requested timestamp
     */
    public TargetObservation2d withFieldPoseLookup(PlanarPoseHistory.Lookup lookup) {
        Objects.requireNonNull(lookup, "lookup");
        if (lookup.timestamp() != timestamp) {
            throw new IllegalArgumentException("field lookup must retain the exact observation timestamp");
        }
        double x = Double.NaN;
        double y = Double.NaN;
        String reason;
        if (!lookup.isAvailable()) {
            reason = "field pose unavailable: " + lookup.unavailableReason();
        } else if (!hasPosition()) {
            reason = "observation has no robot-relative position";
        } else {
            Pose2d pose = lookup.fieldToRobotPose();
            double c = Math.cos(pose.headingRad);
            double s = Math.sin(pose.headingRad);
            double candidateX = pose.xInches + c * forwardInches - s * leftInches;
            double candidateY = pose.yInches + s * forwardInches + c * leftInches;
            if (Double.isFinite(candidateX) && Double.isFinite(candidateY)) {
                x = candidateX;
                y = candidateY;
                reason = "available";
            } else {
                reason = "field transform produced non-finite coordinates";
            }
        }
        return new TargetObservation2d(this, x, y, lookup, reason);
    }

    /** Returns whether the producer supplied confidence rather than an unknown NaN value. */
    public boolean hasQuality() { return hasTarget && Double.isFinite(quality); }
    /** Returns whether capture-time projection supplied finite field coordinates. */
    public boolean hasFieldPosition() {
        return hasTarget && Double.isFinite(fieldXInches) && Double.isFinite(fieldYInches);
    }
    /** Returns the available/failed lookup, or null before field projection was requested. */
    public PlanarPoseHistory.Lookup fieldLookup() { return fieldLookup; }
    /** Returns field-position availability/absence without resampling history. */
    public String fieldProjectionReason() { return fieldProjectionReason; }

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
        requireFinite("forwardInches", forwardInches);
        requireFinite("leftInches", leftInches);
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
        requireFinite("forwardInches", forwardInches);
        requireFinite("leftInches", leftInches);
        requireFinite("targetHeadingRad", targetHeadingRad);
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
        requireFinite("bearingRad", bearingRad);
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
        return timestamp.isFresh(clock, maxAgeSec) && hasTarget;
    }

    private static void requireFinite(String name, double value) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException(name + " must be finite");
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
