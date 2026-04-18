package edu.ftcphoenix.fw.localization;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;

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
 *   <li>How "old" the estimate is relative to now.</li>
 *   <li>The absolute timestamp at which the underlying measurement was taken.</li>
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
 * <ul>
 *   <li>{@link #timestampSec} is the time (in seconds, from an arbitrary monotonic epoch)
 *       at which the underlying measurement was taken.</li>
 *   <li>{@link #ageSec} is the age of the estimate (how long ago the measurement was taken)
 *       <em>at the time the {@code PoseEstimate} instance was created</em>.</li>
 * </ul>
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
     * Age of this estimate in seconds at the time it was created.
     *
     * <p>Example: if a camera frame was captured 50 ms before the current loop iteration, an
     * estimator might set {@code ageSec = 0.050} for the pose derived from that frame.</p>
     */
    public final double ageSec;

    /**
     * Timestamp in seconds at which the underlying measurement was taken.
     *
     * <p>This value is in the same timebase as the {@link LoopClock}
     * used by the estimator (typically seconds since OpMode start or another monotonic clock).</p>
     */
    public final double timestampSec;

    /**
     * Constructs a new {@code PoseEstimate}.
     *
     * @param fieldToRobotPose robot pose in the FTC field coordinate system (field→robot; 6DOF)
     * @param hasPose      whether this represents a valid pose
     * @param quality      quality score in [0.0, 1.0]
     * @param ageSec       age of the estimate in seconds
     * @param timestampSec time at which the underlying measurement was taken
     */
    public PoseEstimate(Pose3d fieldToRobotPose,
                        boolean hasPose,
                        double quality,
                        double ageSec,
                        double timestampSec) {
        if (fieldToRobotPose == null) {
            throw new IllegalArgumentException("fieldToRobotPose is required");
        }
        this.fieldToRobotPose = fieldToRobotPose;
        this.hasPose = hasPose;
        this.quality = quality;
        this.ageSec = ageSec;
        this.timestampSec = timestampSec;
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
     *   <li>{@link #ageSec} will be 0.0.</li>
     *   <li>{@link #timestampSec} will be set to {@code nowSec}.</li>
     * </ul>
     *
     * @param nowSec current time in seconds, from the same timebase used by the estimator
     * @return a {@code PoseEstimate} with {@link #hasPose} = false
     */
    public static PoseEstimate noPose(double nowSec) {
        return new PoseEstimate(
                Pose3d.zero(),
                false,
                0.0,
                0.0,
                nowSec
        );
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        if (!hasPose) {
            return "PoseEstimate{no pose, timestampSec=" + timestampSec + "}";
        }
        return "PoseEstimate{" +
                "fieldToRobotPose=" + fieldToRobotPose +
                ", quality=" + quality +
                ", ageSec=" + ageSec +
                ", timestampSec=" + timestampSec +
                '}';
    }
}
