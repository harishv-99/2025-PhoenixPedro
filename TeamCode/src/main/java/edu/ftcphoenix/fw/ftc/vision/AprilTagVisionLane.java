package edu.ftcphoenix.fw.ftc.vision;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * Backend-neutral FTC-boundary seam for a shared AprilTag vision rig.
 *
 * <p>
 * This interface intentionally exposes only the pieces that higher-level localization and
 * targeting code actually need from an FTC AprilTag camera rig:
 * </p>
 * <ul>
 *   <li>the shared {@link AprilTagSensor},</li>
 *   <li>the fixed {@link CameraMountConfig} used to interpret those observations,</li>
 *   <li>component readiness, kept separate from whether a target is visible, and</li>
 *   <li>lifecycle/debug hooks for the owner that holds the camera resource.</li>
 * </ul>
 *
 * <p>
 * The point of this seam is not to erase the real differences between webcam- and smart-camera
 * backends. It is to keep the rest of the framework dependent on the outputs it consumes, while
 * concrete FTC-boundary owners such as {@link FtcWebcamAprilTagVisionLane} or
 * {@link FtcLimelightAprilTagVisionLane} remain free to own backend-specific wiring, processor
 * setup, and cleanup.
 * </p>
 */
public interface AprilTagVisionLane extends AutoCloseable {

    /**
     * Returns the shared AprilTag sensor resource owned by this lane.
     *
     * @return shared AprilTag sensor
     */
    AprilTagSensor tagSensor();

    /**
     * Returns the fixed camera-mount configuration used by this vision rig.
     *
     * @return camera extrinsics for the active backend/profile
     */
    CameraMountConfig cameraMountConfig();

    /**
     * Returns whether this lane can safely provide AprilTag observations in the current loop.
     *
     * <p>This is component readiness, not target visibility. A ready lane may legitimately report
     * zero detections. Limelight-backed implementations use the clock to keep result sampling
     * cycle-stable; webcam-backed implementations use the same signature so callers remain
     * backend-neutral.</p>
     *
     * @param clock current shared loop clock (required)
     * @return immutable AprilTag-component readiness and operator-facing reason
     */
    VisionReadiness readiness(LoopClock clock);

    /**
     * Releases the shared vision resources owned by this lane.
     *
     * <p>Implementations should make this idempotent so repeated shutdown paths stay safe.</p>
     */
    @Override
    void close();

    /**
     * Dumps the lane's live debug state.
     *
     * @param dbg    debug sink to write to; ignored when {@code null}
     * @param prefix key prefix for all entries; may be {@code null} or empty
     */
    default void debugDump(DebugSink dbg, String prefix) {
        // Default no-op so tiny adapters do not need boilerplate when they have nothing extra
        // to report beyond the shared sensor itself.
    }
}
