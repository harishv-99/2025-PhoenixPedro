package edu.ftcphoenix.fw.localization;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;

/**
 * Immutable snapshot of the robot's estimated pose on the field (6DOF).
 *
 * <p>A {@code PoseEstimate} is produced by a {@link AbsolutePoseEstimator} and consumed by drive
 * controllers, tasks, and debugging tools. It captures:</p>
 *
 * <ul>
 *   <li>The robot's pose in the field coordinate system (if available).</li>
 *   <li>Whether a valid pose is currently available.</li>
 *   <li>A simple quality score (0–1) for selection/fusion/debugging.</li>
 *   <li>The epoch-safe timestamp at which the underlying measurement was taken.</li>
 * </ul>
 *
 * <h2>Field coordinate system (FTC)</h2>
 *
 * <p>Phoenix field-centric poses follow the <b>FTC Field Coordinate System</b> for the current game.
 * Importantly, the <b>meaning of the X/Y axes is season-dependent</b> (square vs diamond vs inverted
 * square). Phoenix therefore avoids hardcoding “+X is …” or “+Y is …” in core pose types.</p>
 *
 * <p>Reference (FTC official docs):</p>
 * <pre>
 * https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html
 * </pre>
 *
 * <p>What is consistent across seasons:</p>
 * <ul>
 *   <li>{@code zInches} is height above the field floor, where <b>+Z is up</b>.</li>
 *   <li>{@code yawRad/pitchRad/rollRad} are rotations about +Z/+Y/+X respectively in the chosen field frame,
 *       using the right-hand rule.</li>
 * </ul>
 *
 * <h2>Time semantics</h2>
 *
 * <p>{@link #timestamp} keeps the originating {@link LoopClock} and reset epoch attached to the
 * measurement time. Consumers derive age with {@link LoopTimestamp#ageSec(LoopClock)} rather than
 * retaining a second age value that can drift from the timestamp.</p>
 */
public final class PoseEstimate {

    /**
     * Estimated robot pose in the FTC field coordinate system for the current game.
     *
     * <p>When {@link #hasPose} is {@code false}, this pose is still defined but should not be used
     * for control. It may be useful for debugging or as a last-known pose.</p>
     */
    public final Pose3d fieldToRobotPose;

    /**
     * True if this estimate contains a valid pose.
     *
     * <p>When {@code hasPose} is {@code false}, the other fields are still defined but represent
     * a "no pose" condition.</p>
     */
    public final boolean hasPose;

    /**
     * Quality score in the range [0.0, 1.0], where 1.0 is "best" and 0.0 is "no confidence".
     *
     * <p>This is intentionally simple. Different {@link AbsolutePoseEstimator} implementations may interpret
     * it differently (e.g., number of tags visible, covariance-like measure, etc.).</p>
     */
    public final double quality;

    /**
     * Epoch-safe time at which the underlying measurement was taken.
     *
     * <p>Use {@link LoopTimestamp#ageSec(LoopClock)} or
     * {@link LoopTimestamp#isFresh(LoopClock, double)} to interpret this value. A no-pose result
     * may still carry the time at which that result was produced; it does not claim an underlying
     * measurement exists.</p>
     */
    public final LoopTimestamp timestamp;

    /**
     * Constructs a new {@code PoseEstimate}.
     *
     * @param fieldToRobotPose robot pose in the FTC field coordinate system (field→robot; 6DOF)
     * @param hasPose      whether this represents a valid pose
     * @param quality      quality score in [0.0, 1.0]
     * @param timestamp    epoch-safe time at which the underlying measurement was taken
     */
    public PoseEstimate(Pose3d fieldToRobotPose,
                        boolean hasPose,
                        double quality,
                        LoopTimestamp timestamp) {
        if (fieldToRobotPose == null) {
            throw new IllegalArgumentException("fieldToRobotPose is required");
        }
        if (timestamp == null) {
            throw new IllegalArgumentException("timestamp is required; use LoopTimestamp.unavailable() when no truthful time exists");
        }
        this.fieldToRobotPose = fieldToRobotPose;
        this.hasPose = hasPose;
        this.quality = quality;
        this.timestamp = timestamp;
    }

    /**
     * Convenience: project this 6DOF pose estimate into a planar {@link Pose2d}.
     *
     * <p>This keeps (x, y) and uses {@link Pose3d#yawRad} as heading, dropping z/pitch/roll.
     * This is the common representation for drivetrain controllers.</p>
     *
     * @return a planar pose projection (x, y, yaw)
     */
    public Pose2d toPose2d() {
        return fieldToRobotPose.toPose2d();
    }

    /**
     * Creates a {@code PoseEstimate} representing "no pose".
     *
     * <ul>
     *   <li>{@link #hasPose} will be {@code false}.</li>
     *   <li>{@link #fieldToRobotPose} will be the 6DOF identity pose by default.</li>
     *   <li>{@link #quality} will be 0.0.</li>
     *   <li>{@link #timestamp} will identify when this no-result snapshot was produced, or be
     *       unavailable when no truthful production time exists.</li>
     * </ul>
     *
     * @param timestamp time at which this no-result snapshot was produced
     * @return a {@code PoseEstimate} with {@link #hasPose} = false
     */
    public static PoseEstimate noPose(LoopTimestamp timestamp) {
        return new PoseEstimate(
                Pose3d.zero(),
                false,
                0.0,
                timestamp
        );
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        if (!hasPose) {
            return "PoseEstimate{no pose, timestamp=" + timestamp + "}";
        }
        return "PoseEstimate{" +
                "fieldToRobotPose=" + fieldToRobotPose +
                ", quality=" + quality +
                ", timestamp=" + timestamp +
                '}';
    }
}
