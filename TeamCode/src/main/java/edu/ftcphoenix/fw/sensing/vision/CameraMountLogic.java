package edu.ftcphoenix.fw.sensing.vision;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.sensing.observation.TargetObservation2d;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;

/**
 * Math helpers for applying {@link CameraMountConfig} (camera extrinsics) to camera-frame
 * measurements such as {@link AprilTagObservation#cameraToTagPose}.
 *
 * <h2>Why this exists</h2>
 * <p>
 * Many vision measurements are naturally expressed in the camera frame (e.g., {@code cameraToTagPose}).
 * If the camera is offset from the robot center (translation and/or yaw), aiming using the
 * camera bearing will cause the <em>camera</em> to face the target, which may not make the
 * <em>robot center</em> face the target.
 * </p>
 *
 * <p>
 * This class converts camera-frame tag measurements into robot-frame equivalents using
 * the mount transform {@code robotToCameraPose} from {@link CameraMountConfig}.
 * </p>
 *
 * <h2>Conventions</h2>
 * <ul>
 *   <li>Frames use Phoenix convention: +X forward, +Y left, +Z up.</li>
 *   <li>Bearing is computed in the robot frame: {@code atan2(left, forward)}.</li>
 *   <li>Therefore, {@code bearingRad > 0} means the target is to the left (CCW).</li>
 * </ul>
 */
public final class CameraMountLogic {

    private CameraMountLogic() {
        // Static helpers only.
    }

    /**
     * Convert a camera-frame tag pose into a robot-frame tag pose.
     *
     * <p>If {@code mount} is {@code robotToCameraPose} and {@code cameraToTagPose} is camera→tag,
     * then this returns {@code robotToTagPose}.</p>
     *
     * @param mount           camera mount (robot→camera); must not be null
     * @param cameraToTagPose camera→tag pose; must not be null
     * @return robot→tag pose
     */
    public static Pose3d robotToTagPose(CameraMountConfig mount, Pose3d cameraToTagPose) {
        Objects.requireNonNull(mount, "mount");
        Objects.requireNonNull(cameraToTagPose, "cameraToTagPose");
        return mount.robotToCameraPose().then(cameraToTagPose);
    }

    /**
     * Compute a robot-centric bearing to the observed tag, accounting for camera offset.
     *
     * <p>This answers: <em>“What bearing should the robot center turn to, so the robot forward axis
     * points at the tag?”</em></p>
     *
     * @param obs   observation; must not be null
     * @param mount camera mount; must not be null
     * @return bearing in radians (positive = left/CCW). Returns 0 if {@code obs.hasTarget == false}.
     */
    public static double robotBearingRad(AprilTagObservation obs, CameraMountConfig mount) {
        Objects.requireNonNull(obs, "obs");
        Objects.requireNonNull(mount, "mount");

        if (!obs.hasTarget) {
            return 0.0;
        }

        Pose3d robotToTagPose = robotToTagPose(mount, obs.cameraToTagPose);

        // Robot-frame bearing: atan2(left, forward).
        return Math.atan2(robotToTagPose.yInches, robotToTagPose.xInches);
    }

    /**
     * Convert an {@link AprilTagObservation} into a robot-relative planar observation.
     *
     * <p>This method applies the camera extrinsics from {@code mount} to produce a measurement in the
     * robot frame. The returned {@link TargetObservation2d} includes full 2D position (forward/left)
     * and bearing.</p>
     *
     * <p>AprilTag observations currently do not expose a separate “quality” metric, so this returns a
     * default quality of 1.0 and uses {@link AprilTagObservation#ageSec} for {@link TargetObservation2d#ageSec}.</p>
     */
    public static TargetObservation2d robotObservation2d(AprilTagObservation obs, CameraMountConfig mount) {
        Objects.requireNonNull(obs, "obs");
        Objects.requireNonNull(mount, "mount");

        if (!obs.hasTarget) {
            return TargetObservation2d.none();
        }

        Pose3d robotToTag = robotToTagPose(mount, obs.cameraToTagPose);
        // Preserve tag yaw so tag-relative points can be resolved in the tag frame.
        return TargetObservation2d.ofRobotRelativePose(
                obs.id,
                robotToTag.xInches,
                robotToTag.yInches,
                robotToTag.yawRad,
                1.0,
                obs.ageSec
        );
    }
}
