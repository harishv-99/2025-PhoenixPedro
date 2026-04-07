package edu.ftcphoenix.fw.ftc.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.FtcVision;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.apriltag.TagOnlyPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagEkfPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagFusionPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.VisionCorrectionPoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * FTC-boundary lane owner for the common "odometry + AprilTag + fused global pose" stack.
 *
 * <p>
 * This class is the reusable owner for a localization lane that many FTC robots will share across
 * seasons. It owns resource construction, the fixed field-tag layout, AprilTag camera ownership,
 * AprilTag-only field solving, fused/global estimator selection, per-loop updates, and cleanup.
 * </p>
 *
 * <p>
 * The lane deliberately stops at <em>pose production</em>. It does not own scoring strategy,
 * target selection, auto-aim thresholds, or other game-specific policy. Those concerns belong in
 * robot code that consumes the lane.
 * </p>
 */
public final class FtcLocalizationLane {

    /**
     * Selects which fused/global estimator implementation the lane should own.
     */
    public enum GlobalEstimatorMode {
        /**
         * Use the simpler gain-based odometry + AprilTag fusion estimator.
         */
        FUSION,
        /**
         * Use the covariance-aware EKF-style odometry + AprilTag estimator.
         */
        EKF
    }

    /**
     * Configuration for the FTC localization lane.
     *
     * <p>
     * The config intentionally groups all stable lane ownership concerns into one framework object:
     * camera identity and mount, fixed field-tag layout, odometry config, AprilTag-only solve
     * config, and fused-estimator config. That lets robot profiles compose the lane directly rather
     * than rebuilding the ownership graph every season.
     * </p>
     */
    public static final class Config {

        /**
         * Preferred FTC hardware-map name for the AprilTag webcam.
         */
        public String webcamName = "Webcam 1";

        /**
         * Fixed camera extrinsics expressed in the robot frame.
         */
        public CameraMountConfig cameraMount = CameraMountConfig.identity();

        /**
         * Fixed field tags that may be trusted for global localization.
         */
        public TagLayout fieldTagLayout = FtcGameTagLayout.currentGameFieldFixed();

        /**
         * Odometry configuration for the Pinpoint-based estimator.
         */
        public PinpointPoseEstimator.Config pinpoint = PinpointPoseEstimator.Config.defaults();

        /**
         * AprilTag-only field-pose solver configuration.
         */
        public TagOnlyPoseEstimator.Config aprilTags = TagOnlyPoseEstimator.Config.defaults();

        /**
         * Gain-based fusion configuration used when {@link #globalEstimatorMode} is {@link GlobalEstimatorMode#FUSION}.
         */
        public OdometryTagFusionPoseEstimator.Config odometryTagFusion =
                OdometryTagFusionPoseEstimator.Config.defaults();

        /**
         * EKF-style fusion configuration used when {@link #globalEstimatorMode} is {@link GlobalEstimatorMode#EKF}.
         */
        public OdometryTagEkfPoseEstimator.Config odometryTagEkf =
                OdometryTagEkfPoseEstimator.Config.defaults();

        /**
         * Which fused/global estimator implementation the lane should construct.
         */
        public GlobalEstimatorMode globalEstimatorMode = GlobalEstimatorMode.FUSION;

        private Config() {
            // Defaults assigned in field initializers.
        }

        /**
         * Creates a config populated with framework defaults for the current FTC season.
         *
         * @return new mutable config instance
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Creates a deep copy of this config.
         *
         * @return copied config whose nested configs can be edited independently
         */
        public Config copy() {
            Config c = new Config();
            c.webcamName = this.webcamName;
            c.cameraMount = this.cameraMount;
            c.fieldTagLayout = this.fieldTagLayout;
            c.pinpoint = this.pinpoint.copy();
            c.aprilTags = this.aprilTags.copy();
            c.odometryTagFusion = this.odometryTagFusion.copy();
            c.odometryTagEkf = this.odometryTagEkf.copy();
            c.globalEstimatorMode = this.globalEstimatorMode;
            return c;
        }
    }

    private final Config cfg;
    private final PinpointPoseEstimator pinpoint;
    private final TagOnlyPoseEstimator aprilTagLocalizer;
    private final VisionCorrectionPoseEstimator globalEstimator;
    private final AprilTagSensor tagSensor;

    private boolean closed = false;

    /**
     * Creates the localization lane from one FTC hardware map and one config snapshot.
     *
     * @param hardwareMap FTC hardware map used to create odometry and vision owners
     * @param config      lane config; defensively copied for the lifetime of this owner
     */
    public FtcLocalizationLane(HardwareMap hardwareMap, Config config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.cfg = Objects.requireNonNull(config, "config").copy();

        this.pinpoint = new PinpointPoseEstimator(hardwareMap, this.cfg.pinpoint.copy());

        FtcVision.Config visionCfg = FtcVision.Config.defaults().withCameraMount(this.cfg.cameraMount);
        this.tagSensor = FtcVision.aprilTags(hardwareMap, this.cfg.webcamName, visionCfg);

        TagOnlyPoseEstimator.Config tagLocalizerCfg = this.cfg.aprilTags.copy().withCameraMount(this.cfg.cameraMount);
        this.aprilTagLocalizer = new TagOnlyPoseEstimator(tagSensor, this.cfg.fieldTagLayout, tagLocalizerCfg);
        this.globalEstimator = createGlobalEstimator(pinpoint, aprilTagLocalizer);
    }

    /**
     * Returns a defensive copy of the lane config owned by this localizer.
     *
     * @return copied config snapshot describing webcam, camera mount, field tags, and estimator tuning
     */
    public Config config() {
        return cfg.copy();
    }

    /**
     * Returns the fixed camera-mount configuration used by the AprilTag vision stack.
     *
     * @return camera extrinsics for the active robot profile
     */
    public CameraMountConfig cameraMountConfig() {
        return cfg.cameraMount;
    }

    /**
     * Returns the fixed field-tag layout trusted by this localization lane.
     *
     * @return field tag layout used by localization and any consumers sharing the lane
     */
    public TagLayout fieldTagLayout() {
        return cfg.fieldTagLayout;
    }

    /**
     * Returns the shared AprilTag sensor resource owned by this lane.
     *
     * @return shared AprilTag sensor
     */
    public AprilTagSensor tagSensor() {
        return tagSensor;
    }

    /**
     * Returns the active fused/global pose estimator.
     *
     * @return global estimator selected by the lane config
     */
    public VisionCorrectionPoseEstimator globalEstimator() {
        return globalEstimator;
    }

    /**
     * Returns the odometry-only estimator.
     *
     * @return Pinpoint odometry estimator owned by this lane
     */
    public PinpointPoseEstimator odometryEstimator() {
        return pinpoint;
    }

    /**
     * Returns the AprilTag-only field-pose estimator.
     *
     * @return AprilTag-only localizer used as the vision input to the fused estimator
     */
    public TagOnlyPoseEstimator aprilTagLocalizer() {
        return aprilTagLocalizer;
    }

    /**
     * Updates localization for one loop cycle.
     *
     * <p>
     * The fused/global estimator is preferred whenever available because it owns the combined pose
     * stack and internally updates the child estimators it depends on. Odometry remains as a small
     * defensive fallback.
     * </p>
     *
     * @param clock shared loop clock for the active OpMode cycle
     */
    public void update(LoopClock clock) {
        if (globalEstimator != null) {
            globalEstimator.update(clock);
        } else if (pinpoint != null) {
            pinpoint.update(clock);
        }
    }

    /**
     * Returns the current fused/global pose estimate.
     *
     * @return current global pose, or {@code null} if the estimator has no pose yet
     */
    public PoseEstimate globalPose() {
        return globalEstimator != null ? globalEstimator.getEstimate() : null;
    }

    /**
     * Returns the current odometry-only pose estimate.
     *
     * @return current odometry pose, or {@code null} if odometry is unavailable
     */
    public PoseEstimate odometryPose() {
        return pinpoint != null ? pinpoint.getEstimate() : null;
    }

    /**
     * Releases the shared camera/vision resources owned by this lane.
     *
     * <p>This method is idempotent so it is safe to call from repeated shutdown paths.</p>
     */
    public void close() {
        if (closed) {
            return;
        }
        closed = true;
        tagSensor.close();
    }

    /**
     * Returns whether this localization lane has already released its camera resource.
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
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "localization" : prefix;
        dbg.addData(p + ".webcamName", cfg.webcamName);
        dbg.addData(p + ".globalEstimatorMode", cfg.globalEstimatorMode);
        dbg.addData(p + ".closed", closed);
        cfg.cameraMount.debugDump(dbg, p + ".cameraMount");
        pinpoint.debugDump(dbg, p + ".odometry");
        aprilTagLocalizer.debugDump(dbg, p + ".apriltags");
        globalEstimator.debugDump(dbg, p + ".global");
    }

    /**
     * Creates the configured fused/global estimator implementation from odometry and AprilTag vision.
     *
     * @param odometry configured Pinpoint odometry estimator
     * @param vision   configured AprilTag-only field-pose estimator
     * @return fused/global estimator selected by {@link Config#globalEstimatorMode}
     */
    private VisionCorrectionPoseEstimator createGlobalEstimator(PinpointPoseEstimator odometry,
                                                                TagOnlyPoseEstimator vision) {
        if (cfg.globalEstimatorMode == GlobalEstimatorMode.EKF) {
            return new OdometryTagEkfPoseEstimator(
                    odometry,
                    vision,
                    cfg.odometryTagEkf.validatedCopy("FtcLocalizationLane.Config.odometryTagEkf")
            );
        }
        return new OdometryTagFusionPoseEstimator(
                odometry,
                vision,
                cfg.odometryTagFusion.validatedCopy("FtcLocalizationLane.Config.odometryTagFusion")
        );
    }
}
