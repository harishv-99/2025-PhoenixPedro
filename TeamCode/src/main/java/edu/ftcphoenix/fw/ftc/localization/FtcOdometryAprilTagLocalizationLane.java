package edu.ftcphoenix.fw.ftc.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcphoenix.fw.localization.apriltag.TagOnlyPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagEkfPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagFusionPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.VisionCorrectionPoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;

/**
 * FTC-boundary lane owner for the common "odometry + AprilTag + fused global pose" stack.
 *
 * <p>
 * This lane consumes three stable inputs:
 * </p>
 * <ul>
 *   <li>a Pinpoint odometry rig,</li>
 *   <li>a shared {@link AprilTagVisionLane}, and</li>
 *   <li>a field-fixed {@link TagLayout} describing which tags are trusted landmarks.</li>
 * </ul>
 *
 * <p>
 * This owner deliberately does <em>not</em> own the camera rig. The vision lane owns device identity, camera mount, and backend cleanup. This localization lane owns the pose-estimation strategy built on top of those resources: odometry, AprilTag-only field solving, fused/global estimator selection, and per-loop updates.
 * </p>
 */
public final class FtcOdometryAprilTagLocalizationLane {

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
     * AprilTag-localization tuning owned by the localization lane.
     *
     * <p>
     * This config intentionally excludes camera-rig concerns such as backend/device identity and camera
     * mount. Those belong in the concrete vision-lane config owned by the active backend. It only contains the
     * AprilTag-specific field-pose solve policy needed by localization itself.
     * </p>
     */
    public static final class AprilTagLocalizationConfig {

        /**
         * Maximum accepted detections-frame age in seconds for AprilTag localization.
         */
        public double maxDetectionAgeSec = 0.50;

        /**
         * Shared fixed-tag field-pose solver tuning used by AprilTag localization.
         */
        public FixedTagFieldPoseSolver.Config fieldPoseSolver = FixedTagFieldPoseSolver.Config.defaults();

        private AprilTagLocalizationConfig() {
            // Defaults assigned in field initializers.
        }

        /**
         * Creates a config populated with framework defaults.
         *
         * @return new mutable config instance
         */
        public static AprilTagLocalizationConfig defaults() {
            return new AprilTagLocalizationConfig();
        }

        /**
         * Creates a deep copy of this config.
         *
         * @return copied config whose nested solver config can be edited independently
         */
        public AprilTagLocalizationConfig copy() {
            AprilTagLocalizationConfig c = new AprilTagLocalizationConfig();
            c.maxDetectionAgeSec = this.maxDetectionAgeSec;
            c.fieldPoseSolver = this.fieldPoseSolver.copy();
            return c;
        }

        /**
         * Builds a {@link TagOnlyPoseEstimator.Config} by combining this localization tuning with a
         * specific camera mount from a shared vision lane.
         *
         * @param cameraMount camera extrinsics provided by the vision lane
         * @return new {@link TagOnlyPoseEstimator.Config} snapshot ready for construction
         */
        public TagOnlyPoseEstimator.Config toTagOnlyPoseEstimatorConfig(CameraMountConfig cameraMount) {
            TagOnlyPoseEstimator.Config c = TagOnlyPoseEstimator.Config.defaults();
            FixedTagFieldPoseSolver.Config solver = FixedTagFieldPoseSolver.Config.copyOf(this.fieldPoseSolver);
            c.maxAbsBearingRad = solver.maxAbsBearingRad;
            c.preferObservationFieldPose = solver.preferObservationFieldPose;
            c.observationFieldPoseMaxDeltaInches = solver.observationFieldPoseMaxDeltaInches;
            c.observationFieldPoseMaxDeltaHeadingRad = solver.observationFieldPoseMaxDeltaHeadingRad;
            c.rangeSoftnessInches = solver.rangeSoftnessInches;
            c.minObservationWeight = solver.minObservationWeight;
            c.outlierPositionGateInches = solver.outlierPositionGateInches;
            c.outlierHeadingGateRad = solver.outlierHeadingGateRad;
            c.consistencyPositionScaleInches = solver.consistencyPositionScaleInches;
            c.consistencyHeadingScaleRad = solver.consistencyHeadingScaleRad;
            c.plausibleFieldRegion = solver.plausibleFieldRegion;
            c.maxOutsidePlausibleFieldRegionInches = solver.maxOutsidePlausibleFieldRegionInches;
            c.maxDetectionAgeSec = this.maxDetectionAgeSec;
            c.cameraMount = cameraMount != null ? cameraMount : CameraMountConfig.identity();
            return c;
        }
    }

    /**
     * Configuration for the FTC odometry + AprilTag localization lane.
     *
     * <p>
     * The config groups the stable pieces of localization strategy: odometry tuning, AprilTag field
     * solve tuning, fused-estimator tuning, and which fused/global estimator implementation to use.
     * The camera rig itself is intentionally separate and belongs to
     * the concrete vision-lane config owned by the active backend.
     * </p>
     */
    public static final class Config {

        /**
         * Odometry configuration for the Pinpoint-based estimator.
         */
        public PinpointPoseEstimator.Config odometry = PinpointPoseEstimator.Config.defaults();

        /**
         * AprilTag field-solve tuning for the AprilTag-only localizer.
         */
        public AprilTagLocalizationConfig aprilTags = AprilTagLocalizationConfig.defaults();

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
         * @return copied config whose nested configs can be edited independently
         */
        public Config copy() {
            Config c = new Config();
            c.odometry = this.odometry.copy();
            c.aprilTags = this.aprilTags.copy();
            c.odometryTagFusion = this.odometryTagFusion.copy();
            c.odometryTagEkf = this.odometryTagEkf.copy();
            c.globalEstimatorMode = this.globalEstimatorMode;
            return c;
        }
    }

    private final Config cfg;
    private final AprilTagVisionLane visionLane;
    private final TagLayout fixedFieldTagLayout;
    private final PinpointPoseEstimator odometryEstimator;
    private final TagOnlyPoseEstimator aprilTagLocalizer;
    private final VisionCorrectionPoseEstimator globalEstimator;

    /**
     * Creates the localization lane from one FTC hardware map, one shared vision lane, one field
     * tag layout, and one config snapshot.
     *
     * @param hardwareMap         FTC hardware map used to create the odometry estimator
     * @param visionLane          shared AprilTag vision lane that owns the camera rig
     * @param fixedFieldTagLayout field-fixed AprilTag layout trusted for localization
     * @param config              lane config; defensively copied for the lifetime of this owner
     */
    public FtcOdometryAprilTagLocalizationLane(HardwareMap hardwareMap,
                                               AprilTagVisionLane visionLane,
                                               TagLayout fixedFieldTagLayout,
                                               Config config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.visionLane = Objects.requireNonNull(visionLane, "visionLane");
        this.fixedFieldTagLayout = Objects.requireNonNull(fixedFieldTagLayout, "fixedFieldTagLayout");
        this.cfg = Objects.requireNonNull(config, "config").copy();

        this.odometryEstimator = new PinpointPoseEstimator(hardwareMap, this.cfg.odometry.copy());

        TagOnlyPoseEstimator.Config aprilTagCfg =
                this.cfg.aprilTags.toTagOnlyPoseEstimatorConfig(this.visionLane.cameraMountConfig());
        this.aprilTagLocalizer = new TagOnlyPoseEstimator(
                this.visionLane.tagSensor(),
                this.fixedFieldTagLayout,
                aprilTagCfg
        );
        this.globalEstimator = createGlobalEstimator(this.odometryEstimator, this.aprilTagLocalizer);
    }

    /**
     * Returns a defensive copy of the lane config owned by this localizer.
     *
     * @return copied config snapshot describing odometry, AprilTag solve, and fusion tuning
     */
    public Config config() {
        return cfg.copy();
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
        return odometryEstimator;
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
        } else if (odometryEstimator != null) {
            odometryEstimator.update(clock);
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
        return odometryEstimator != null ? odometryEstimator.getEstimate() : null;
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
        String p = (prefix == null || prefix.isEmpty()) ? "localizationLane" : prefix;
        dbg.addData(p + ".globalEstimatorMode", cfg.globalEstimatorMode);
        dbg.addData(p + ".fixedFieldTagCount", fixedFieldTagLayout.ids().size());
        odometryEstimator.debugDump(dbg, p + ".odometry");
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
                    cfg.odometryTagEkf.validatedCopy("FtcOdometryAprilTagLocalizationLane.Config.odometryTagEkf")
            );
        }
        return new OdometryTagFusionPoseEstimator(
                odometry,
                vision,
                cfg.odometryTagFusion.validatedCopy("FtcOdometryAprilTagLocalizationLane.Config.odometryTagFusion")
        );
    }
}
