package edu.ftcphoenix.fw.sensing.vision.apriltag;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;

/**
 * Immutable snapshot of a single AprilTag observation (sensor measurement), expressed in
 * <b>Phoenix framing</b>.
 *
 * <h2>Principle: Phoenix framing for Phoenix-defined frames</h2>
 * <p>
 * All core Phoenix framework types operate in Phoenix's standard right-handed axis convention
 * for <b>Phoenix-defined frames</b> (robot, camera, field, mechanisms, etc.):
 * </p>
 * <ul>
 *   <li><b>+X</b> forward</li>
 *   <li><b>+Y</b> left</li>
 *   <li><b>+Z</b> up</li>
 * </ul>
 *
 * <p>
 * Any FTC-SDK-specific coordinate conventions must be converted inside the FTC adapter layer
 * before constructing this object.
 * </p>
 *
 * <p>
 * Note: AprilTags introduce a <em>third-party tag frame</em> (the tag's own +X/+Y/+Z axes).
 * Phoenix keeps that tag frame consistent with the FTC game database metadata so that
 * {@code fieldToTagPose} (from the layout) and {@code cameraToTagPose} (from the detector)
 * compose cleanly.
 * </p>
 *
 * <h2>Phoenix pose naming convention</h2>
 * <p>
 * Whenever a variable or accessor represents a pose/transform, Phoenix code uses:
 * </p>
 * <pre>
 * fromFrameToToFramePose
 * </pre>
 *
 * <p>Examples:</p>
 * <ul>
 *   <li>{@code robotToCameraPose}: camera pose expressed in the robot frame</li>
 *   <li>{@code cameraToTagPose}: tag pose expressed in the camera frame</li>
 *   <li>{@code fieldToRobotPose}: robot pose expressed in the field frame</li>
 * </ul>
 *
 * <h2>Component naming convention (non-pose values derived from a pose)</h2>
 * <p>
 * For a component derived from a pose's translation, Phoenix uses:
 * </p>
 * <pre>
 * &lt;frame&gt;&lt;Direction&gt;&lt;Unit&gt;()
 * </pre>
 *
 * <p>Examples:</p>
 * <ul>
 *   <li>{@code cameraForwardInches()} is the +X (forward) component of {@code cameraToTagPose}.</li>
 *   <li>{@code robotLeftInches()} would be the +Y (left) component of a {@code robotToSomethingPose} pose.</li>
 * </ul>
 *
 * <h2>Phoenix camera frame (for cameraToTagPose)</h2>
 * <p>
 * Phoenix defines the camera frame using the same axis convention as all Phoenix frames:
 * </p>
 * <ul>
 *   <li><b>+X</b> forward (out of the lens)</li>
 *   <li><b>+Y</b> left</li>
 *   <li><b>+Z</b> up</li>
 * </ul>
 * <h2>AprilTag tag frame (for cameraToTagPose)</h2>
 * <p>
 * The tag's axes are defined by the AprilTag library/FTC SDK (i.e., the tag frame used by
 * {@code AprilTagDetection.rawPose} and by FTC game metadata {@code fieldOrientation}).
 * Do <b>not</b> assume the tag frame is a "robot-style" +X-forward/+Y-left/+Z-up frame.
 * </p>
 *
 * <h2>Derived aiming helpers</h2>
 * <p>
 * Bearing and range are derived from {@link #cameraToTagPose} and are not stored separately to avoid
 * redundancy and drift.
 * </p>
 */
public final class AprilTagObservation {

    private static final AprilTagObservation NO_TARGET = new AprilTagObservation(
            false,
            -1,
            LoopTimestamp.unavailable(),
            null,
            null
    );

    /**
     * True if this observation represents a real detected tag.
     */
    public final boolean hasTarget;

    /**
     * AprilTag numeric ID code.
     *
     * <p>Only meaningful when {@link #hasTarget} is true. When {@link #hasTarget} is false, this is -1.</p>
     */
    public final int id;

    /** Timestamp of the camera frame that produced this observation. */
    private final LoopTimestamp frameTimestamp;

    /**
     * Tag pose expressed in the Phoenix camera frame: {@code cameraToTagPose}.
     *
     * <p>Non-null when {@link #hasTarget} is true.</p>
     */
    public final Pose3d cameraToTagPose;

    /**
     * Optional robot pose expressed in the Phoenix field frame: {@code fieldToRobotPose}.
     *
     * <p>If present, this is a field-centric pose measurement source (6DOF). For example, an FTC
     * adapter may populate this from SDK robotPose after converting into Phoenix framing.</p>
     */
    public final Pose3d fieldToRobotPose;

    private AprilTagObservation(boolean hasTarget,
                                int id,
                                LoopTimestamp frameTimestamp,
                                Pose3d cameraToTagPose,
                                Pose3d fieldToRobotPose) {
        this.hasTarget = hasTarget;
        this.id = id;
        this.frameTimestamp = frameTimestamp;
        this.cameraToTagPose = cameraToTagPose;
        this.fieldToRobotPose = fieldToRobotPose;
    }

    /**
     * Create an observation representing "no target".
     *
     * <p>The sentinel has no camera-frame timestamp and is never fresh.</p>
     */
    public static AprilTagObservation noTarget() {
        return NO_TARGET;
    }

    /**
     * Create an observation representing a detected tag, expressed in Phoenix framing.
     *
     * @param id              AprilTag ID
     * @param cameraToTagPose tag pose in Phoenix camera frame (non-null)
     * @param frameTimestamp timestamp of the underlying camera frame
     */
    public static AprilTagObservation target(int id,
                                             Pose3d cameraToTagPose,
                                             LoopTimestamp frameTimestamp) {
        if (cameraToTagPose == null) {
            throw new IllegalArgumentException("cameraToTagPose must be non-null when hasTarget is true");
        }
        requireAvailableTimestamp(frameTimestamp);
        return new AprilTagObservation(true, id, frameTimestamp, cameraToTagPose, null);
    }

    /**
     * Create an observation representing a detected tag with an additional field-centric robot pose
     * measurement.
     *
     * @param id               AprilTag ID
     * @param cameraToTagPose  tag pose in Phoenix camera frame (non-null)
     * @param fieldToRobotPose robot pose in Phoenix field frame (non-null)
     * @param frameTimestamp   timestamp of the underlying camera frame
     */
    public static AprilTagObservation target(int id,
                                             Pose3d cameraToTagPose,
                                             Pose3d fieldToRobotPose,
                                             LoopTimestamp frameTimestamp) {
        if (cameraToTagPose == null) {
            throw new IllegalArgumentException("cameraToTagPose must be non-null when hasTarget is true");
        }
        if (fieldToRobotPose == null) {
            throw new IllegalArgumentException("fieldToRobotPose must be non-null when provided");
        }
        requireAvailableTimestamp(frameTimestamp);
        return new AprilTagObservation(
                true,
                id,
                frameTimestamp,
                cameraToTagPose,
                fieldToRobotPose
        );
    }

    /** Returns the timestamp of the camera frame, or {@link LoopTimestamp#unavailable()}. */
    public LoopTimestamp frameTimestamp() {
        return frameTimestamp;
    }

    /**
     * Returns the current age of the camera frame.
     *
     * @param clock the same stable loop clock that created the timestamp
     * @return non-negative age in seconds, or {@code NaN} when unavailable or reset-invalidated
     */
    public double frameAgeSec(LoopClock clock) {
        return frameTimestamp.ageSec(clock);
    }

    /**
     * Returns true if this observation contains a {@link #fieldToRobotPose} measurement.
     */
    public boolean hasFieldToRobotPose() {
        return hasTarget && fieldToRobotPose != null;
    }

    /**
     * Convenience helper to test whether this observation is considered "fresh enough".
     *
     * @param clock the same stable loop clock that created the frame timestamp
     * @param maxAgeSec maximum acceptable age in seconds
     * @return true if this observation has a target and its frame is no older than maxAgeSec
     */
    public boolean isFresh(LoopClock clock, double maxAgeSec) {
        return hasTarget && frameTimestamp.isFresh(clock, maxAgeSec);
    }

    private static void requireAvailableTimestamp(LoopTimestamp timestamp) {
        if (timestamp == null || !timestamp.isAvailable()) {
            throw new IllegalArgumentException(
                    "frameTimestamp must be an available LoopTimestamp created by LoopClock");
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Components derived from cameraToTagPose (Phoenix camera frame)
    // ---------------------------------------------------------------------------------------------

    /**
     * Tag forward component in the Phoenix camera frame (+X forward), in inches.
     */
    public double cameraForwardInches() {
        return hasTarget ? cameraToTagPose.xInches : 0.0;
    }

    /**
     * Tag left component in the Phoenix camera frame (+Y left), in inches.
     */
    public double cameraLeftInches() {
        return hasTarget ? cameraToTagPose.yInches : 0.0;
    }

    /**
     * Tag up component in the Phoenix camera frame (+Z up), in inches.
     */
    public double cameraUpInches() {
        return hasTarget ? cameraToTagPose.zInches : 0.0;
    }

    /**
     * Horizontal bearing from camera forward (+X) to the tag center, in radians.
     *
     * <p>Derived from {@link #cameraToTagPose} using Phoenix sign convention: positive = left.</p>
     */
    public double cameraBearingRad() {
        if (!hasTarget) {
            return 0.0;
        }
        return Math.atan2(cameraLeftInches(), cameraForwardInches());
    }

    /**
     * 3D line-of-sight distance from camera origin to the tag center, in inches.
     *
     * <p>Derived from {@link #cameraToTagPose} translation components.</p>
     */
    public double cameraRangeInches() {
        if (!hasTarget) {
            return 0.0;
        }
        double f = cameraForwardInches();
        double l = cameraLeftInches();
        double u = cameraUpInches();
        return Math.sqrt(f * f + l * l + u * u);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        if (!hasTarget) {
            return "AprilTagObservation{no target}";
        }
        return "AprilTagObservation{"
                + "id=" + id
                + ", frameTimestamp=" + frameTimestamp
                + ", cameraBearingRad=" + cameraBearingRad()
                + ", cameraRangeInches=" + cameraRangeInches()
                + ", hasFieldToRobotPose=" + (fieldToRobotPose != null)
                + '}';
    }
}
