package edu.ftcphoenix.fw.sensing.vision;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose3d;

/**
 * Physical mounting configuration for a camera on the robot (camera extrinsics).
 *
 * <h2>Why this exists</h2>
 * <p>
 * {@link Pose3d} is a generic geometry type that can represent many different poses (robot-in-field,
 * tag-in-camera, camera-in-robot, etc.). {@code CameraMountConfig} is a <b>semantic wrapper</b>
 * that makes it unambiguous that this pose is specifically the <b>camera mounted on the robot</b>.
 * This significantly reduces frame mixups and is also the natural home for camera-specific helpers.
 * </p>
 *
 * <h2>Frame of reference</h2>
 * <p>
 * The stored pose is expressed in the <b>robot frame</b>:
 * </p>
 * <ul>
 *   <li>Origin: robot center of rotation on the floor plane (z = 0).</li>
 *   <li><b>+X</b>: robot forward</li>
 *   <li><b>+Y</b>: robot left</li>
 *   <li><b>+Z</b>: robot up</li>
 * </ul>
 *
 * <h2>Units</h2>
 * <ul>
 *   <li>Distances are in <b>inches</b>.</li>
 *   <li>Angles are in <b>radians</b>.</li>
 * </ul>
 *
 * <h2>Orientation conventions</h2>
 * <p>
 * Orientation is expressed as yaw/pitch/roll using the right-hand rule about the robot axes,
 * consistent with {@link Pose3d}:
 * </p>
 * <ul>
 *   <li><b>yawRad</b>: rotation about <b>+Z</b></li>
 *   <li><b>pitchRad</b>: rotation about <b>+Y</b></li>
 *   <li><b>rollRad</b>: rotation about <b>+X</b></li>
 * </ul>
 *
 * <h2>Immutability</h2>
 * <p>
 * This class is immutable: construct it once during init and pass it to the systems that need it.
 * If you want to “tune” values, create a new instance.
 * </p>
 */
public final class CameraMountConfig {

    /**
     * Robot→camera pose expressed in the robot frame.
     *
     * <p>This is the transform from the robot origin (center of rotation on the floor) to the camera lens.</p>
     */
    private final Pose3d robotToCamera;


    /**
     * A constant identity camera mount (camera at robot origin with no rotation).
     *
     * <p>This is safe to share as a singleton because {@code CameraMountConfig} is immutable.</p>
     */
    private static final CameraMountConfig IDENTITY = new CameraMountConfig(Pose3d.zero());

    private CameraMountConfig(Pose3d robotToCamera) {
        this.robotToCamera = Objects.requireNonNull(robotToCamera, "robotToCamera");
    }

    /**
     * Creates a mount config from a full 6DOF robot→camera pose expressed in the robot frame.
     *
     * @param robotToCameraPose robot→camera pose in the robot frame (non-null)
     * @return a new {@link CameraMountConfig}
     */
    public static CameraMountConfig ofPose(Pose3d robotToCameraPose) {
        return new CameraMountConfig(robotToCameraPose);
    }

    /**
     * Creates a mount config from the six standard values (robot frame).
     *
     * @param xInches  camera position +X forward from robot center (inches)
     * @param yInches  camera position +Y left from robot center (inches)
     * @param zInches  camera position +Z up from floor (inches)
     * @param yawRad   rotation about +Z (radians)
     * @param pitchRad rotation about +Y (radians)
     * @param rollRad  rotation about +X (radians)
     * @return a new {@link CameraMountConfig}
     */
    public static CameraMountConfig of(
            double xInches,
            double yInches,
            double zInches,
            double yawRad,
            double pitchRad,
            double rollRad
    ) {
        return new CameraMountConfig(new Pose3d(xInches, yInches, zInches, yawRad, pitchRad, rollRad));
    }

    /**
     * Creates a mount config from the six standard values, but with angles expressed in degrees.
     *
     * <p>This is a convenience for calibration workflows where humans think in degrees. The config
     * still stores angles internally in <b>radians</b>.</p>
     *
     * @param xInches  camera position +X forward from robot center (inches)
     * @param yInches  camera position +Y left from robot center (inches)
     * @param zInches  camera position +Z up from floor (inches)
     * @param yawDeg   rotation about +Z (degrees)
     * @param pitchDeg rotation about +Y (degrees)
     * @param rollDeg  rotation about +X (degrees)
     * @return a new {@link CameraMountConfig}
     */
    public static CameraMountConfig ofDegrees(
            double xInches,
            double yInches,
            double zInches,
            double yawDeg,
            double pitchDeg,
            double rollDeg
    ) {
        return of(
                xInches,
                yInches,
                zInches,
                Math.toRadians(yawDeg),
                Math.toRadians(pitchDeg),
                Math.toRadians(rollDeg)
        );
    }

    /**
     * Convenience: camera mount at robot origin with no rotation.
     */
    public static CameraMountConfig identity() {
        return IDENTITY;
    }

    /**
     * Returns the robot→camera pose expressed in the robot frame.
     */
    public Pose3d robotToCameraPose() {
        return robotToCamera;
    }

    /**
     * Camera X offset in robot frame (inches). +X is forward.
     */
    public double xInches() {
        return robotToCamera.xInches;
    }

    /**
     * Camera Y offset in robot frame (inches). +Y is left.
     */
    public double yInches() {
        return robotToCamera.yInches;
    }

    /**
     * Camera Z offset in robot frame (inches). +Z is up.
     */
    public double zInches() {
        return robotToCamera.zInches;
    }

    /**
     * Camera yaw in robot frame (radians). Rotation about +Z.
     */
    public double yawRad() {
        return robotToCamera.yawRad;
    }

    /**
     * Camera pitch in robot frame (radians). Rotation about +Y.
     */
    public double pitchRad() {
        return robotToCamera.pitchRad;
    }

    /**
     * Camera roll in robot frame (radians). Rotation about +X.
     */
    public double rollRad() {
        return robotToCamera.rollRad;
    }

    /**
     * Debug-dump this camera mount using the framework {@link DebugSink} pattern.
     *
     * <p>Keys are emitted under {@code prefix} in the form:</p>
     * <pre>
     * prefix.xInches
     * prefix.yInches
     * prefix.zInches
     * prefix.yawRad
     * prefix.pitchRad
     * prefix.rollRad
     * </pre>
     *
     * @param dbg    debug sink (may be {@code null})
     * @param prefix debug key prefix (may be {@code null} or empty)
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "cameraMount" : prefix;
        dbg.addData(p + ".xInches", xInches())
                .addData(p + ".yInches", yInches())
                .addData(p + ".zInches", zInches())
                .addData(p + ".yawRad", yawRad())
                .addData(p + ".pitchRad", pitchRad())
                .addData(p + ".rollRad", rollRad());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "CameraMountConfig{robotToCamera=" + robotToCamera + "}";
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean equals(Object o) {
        if (!(o instanceof CameraMountConfig)) return false;
        CameraMountConfig other = (CameraMountConfig) o;
        return robotToCamera.equals(other.robotToCamera);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int hashCode() {
        return robotToCamera.hashCode();
    }
}
