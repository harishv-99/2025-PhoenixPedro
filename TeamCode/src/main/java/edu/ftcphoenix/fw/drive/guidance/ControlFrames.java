package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * Defines which point(s) on the robot are being controlled by a guidance action.
 *
 * <p>Most drive code implicitly controls the robot’s center. In many games, however, you want to
 * align or position an off-center mechanism (shooter, intake, bucket, etc.). This class lets you
 * specify separate “control frames” for translation and aiming:</p>
 * <ul>
 *   <li><b>Translation frame</b>: the point that should be moved to the translation target.</li>
 *   <li><b>Aim frame</b>: the frame whose +X axis should be rotated to point at the aim target.</li>
 * </ul>
 *
 * <h2>Coordinate convention</h2>
 * <p>Frames are expressed as a {@link Pose2d} transform <b>robot → frame</b> using Phoenix
 * conventions (+X forward, +Y left, heading CCW-positive).</p>
 *
 * <h2>Typical usage</h2>
 * <pre>{@code
 * // Default: control the robot center for both translation and aim.
 * ControlFrames frames = ControlFrames.robotCenter();
 *
 * // Shooter is 6" left of center and angled 30° left.
 * Pose2d robotToShooter = new Pose2d(0.0, 6.0, Math.toRadians(30));
 * ControlFrames shooterAim = ControlFrames.robotCenter().withAimFrame(robotToShooter);
 * }</pre>
 */
public final class ControlFrames {

    private final Pose2d robotToTranslationFrame;
    private final Pose2d robotToAimFrame;

    private ControlFrames(Pose2d robotToTranslationFrame, Pose2d robotToAimFrame) {
        this.robotToTranslationFrame = Objects.requireNonNull(robotToTranslationFrame, "robotToTranslationFrame");
        this.robotToAimFrame = Objects.requireNonNull(robotToAimFrame, "robotToAimFrame");
    }

    /**
     * Default control frames: both translation and aiming are performed about the robot center.
     */
    public static ControlFrames robotCenter() {
        return new ControlFrames(Pose2d.zero(), Pose2d.zero());
    }

    /**
     * Create a new {@link ControlFrames} with explicit frames.
     */
    public static ControlFrames of(Pose2d robotToTranslationFrame, Pose2d robotToAimFrame) {
        return new ControlFrames(robotToTranslationFrame, robotToAimFrame);
    }

    /**
     * @return robot → translation-frame transform
     */
    public Pose2d robotToTranslationFrame() {
        return robotToTranslationFrame;
    }

    /**
     * @return robot → aim-frame transform
     */
    public Pose2d robotToAimFrame() {
        return robotToAimFrame;
    }

    /**
     * Return a copy with the translation control frame replaced.
     */
    public ControlFrames withTranslationFrame(Pose2d robotToTranslationFrame) {
        return new ControlFrames(robotToTranslationFrame, this.robotToAimFrame);
    }

    /**
     * Return a copy with the aim control frame replaced.
     */
    public ControlFrames withAimFrame(Pose2d robotToAimFrame) {
        return new ControlFrames(this.robotToTranslationFrame, robotToAimFrame);
    }


    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "ControlFrames{" +
                "robotToTranslationFrame=" + robotToTranslationFrame +
                ", robotToAimFrame=" + robotToAimFrame +
                '}';
    }

}
