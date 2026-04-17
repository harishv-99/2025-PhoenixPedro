package edu.ftcphoenix.fw.ftc.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.ftc.FtcVision;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * FTC-boundary lane owner for a webcam-backed AprilTag vision rig.
 *
 * <p>
 * This lane owns the pieces that are primarily about the camera rig itself: webcam identity,
 * camera extrinsics, FTC AprilTag-processor settings, sensor construction, and portal cleanup.
 * It intentionally does <em>not</em> own field-pose solving or localization strategy. Those are
 * separate concerns that may consume the same shared vision resource through the backend-neutral
 * {@link AprilTagVisionLane} seam.
 * </p>
 *
 * <p>
 * In other words, this lane answers: "how do we own and configure the AprilTag webcam?" It does
 * not answer: "how do we use AprilTags for localization, aiming, or game strategy?"
 * </p>
 */
public final class FtcWebcamAprilTagVisionLane implements AprilTagVisionLane {

    /**
     * Configuration for a webcam-backed FTC AprilTag-vision lane.
     *
     * <p>
     * The config groups stable camera-rig concerns that tend to recur across robots and seasons:
     * webcam identity, streaming resolution, camera mount, FTC SDK pitch adjustment, and optional
     * tag-library overrides. Higher-level behaviors such as localization and targeting consume this
     * lane; they do not own these camera-rig settings directly.
     * </p>
     */
    public static final class Config {

        /**
         * Preferred FTC hardware-map name for the AprilTag webcam.
         */
        public String webcamName = "Webcam 1";

        /**
         * Camera streaming resolution used by the FTC vision portal.
         */
        public Size cameraResolution = FtcVision.Config.defaults().cameraResolution;

        /**
         * Camera extrinsics expressed in the robot frame.
         */
        public CameraMountConfig cameraMount = CameraMountConfig.identity();

        /**
         * Optional FTC SDK pitch offset applied when converting the mount into SDK camera axes.
         */
        public double sdkPitchRadOffset = 0.0;

        /**
         * Optional AprilTag library override; {@code null} uses the current FTC game library.
         */
        public AprilTagLibrary tagLibrary = null;

        private Config() {
            // Defaults assigned in field initializers.
        }

        /**
         * Creates a config populated with framework defaults.
         *
         * @return new mutable config instance
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Creates a deep copy of this config.
         *
         * @return copied config whose fields can be edited independently
         */
        public Config copy() {
            Config c = new Config();
            c.webcamName = this.webcamName;
            c.cameraResolution = this.cameraResolution;
            c.cameraMount = this.cameraMount;
            c.sdkPitchRadOffset = this.sdkPitchRadOffset;
            c.tagLibrary = this.tagLibrary;
            return c;
        }

        /**
         * Converts this lane config into the lower-level {@link FtcVision.Config} used by the FTC
         * AprilTag adapter.
         *
         * @return new {@link FtcVision.Config} snapshot with the equivalent processor settings
         */
        public FtcVision.Config toFtcVisionConfig() {
            FtcVision.Config c = FtcVision.Config.defaults();
            c.cameraResolution = this.cameraResolution;
            c.cameraMount = this.cameraMount;
            c.sdkPitchRadOffset = this.sdkPitchRadOffset;
            c.tagLibrary = this.tagLibrary;
            return c;
        }
    }

    private final Config cfg;
    private final AprilTagSensor tagSensor;
    private boolean closed = false;

    /**
     * Creates the webcam AprilTag vision lane from one FTC hardware map and one config snapshot.
     *
     * @param hardwareMap FTC hardware map used to create the underlying vision portal
     * @param config      lane config; defensively copied for the lifetime of this owner
     */
    public FtcWebcamAprilTagVisionLane(HardwareMap hardwareMap, Config config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.cfg = Objects.requireNonNull(config, "config").copy();
        this.tagSensor = FtcVision.aprilTags(hardwareMap, this.cfg.webcamName, this.cfg.toFtcVisionConfig());
    }

    /**
     * Returns a defensive copy of the lane config owned by this vision rig.
     *
     * @return copied config snapshot describing webcam identity and camera-rig settings
     */
    public Config config() {
        return cfg.copy();
    }

    /**
     * Returns the configured FTC webcam name.
     *
     * @return configured FTC hardware-map webcam name
     */
    public String webcamName() {
        return cfg.webcamName;
    }

    /**
     * Returns the fixed camera-mount configuration used by this vision rig.
     *
     * @return camera extrinsics for the active robot profile
     */
    @Override
    public CameraMountConfig cameraMountConfig() {
        return cfg.cameraMount;
    }

    /**
     * Returns the shared AprilTag sensor resource owned by this lane.
     *
     * @return shared AprilTag sensor
     */
    @Override
    public AprilTagSensor tagSensor() {
        return tagSensor;
    }

    /**
     * Releases the shared vision resources owned by this lane.
     *
     * <p>This method is idempotent so it is safe to call from repeated shutdown paths.</p>
     */
    @Override
    public void close() {
        if (closed) {
            return;
        }
        closed = true;
        tagSensor.close();
    }

    /**
     * Returns whether this vision lane has already released its camera resource.
     *
     * @return {@code true} after {@link #close()} has been called
     */
    public boolean isClosed() {
        return closed;
    }

    /**
     * Dumps the lane's live debug state.
     *
     * @param dbg    debug sink to write to; ignored when {@code null}
     * @param prefix key prefix for all entries; may be {@code null} or empty
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "visionLane" : prefix;
        dbg.addData(p + ".backend", "webcam");
        dbg.addData(p + ".webcamName", cfg.webcamName);
        dbg.addData(p + ".sdkPitchRadOffset", cfg.sdkPitchRadOffset);
        dbg.addData(p + ".closed", closed);
        cfg.cameraMount.debugDump(dbg, p + ".cameraMount");
        dbg.addData(p + ".cameraResolution", cfg.cameraResolution != null ? cfg.cameraResolution.toString() : "null");
        dbg.addData(p + ".tagLibraryOverride", cfg.tagLibrary != null ? cfg.tagLibrary.getClass().getSimpleName() : "currentGame");
    }
}
