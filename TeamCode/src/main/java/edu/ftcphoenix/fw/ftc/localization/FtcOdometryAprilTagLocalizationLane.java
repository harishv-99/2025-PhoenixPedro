package edu.ftcphoenix.fw.ftc.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.FtcLimelightAprilTagVisionLane;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.apriltag.AprilTagPoseEstimator;
import edu.ftcphoenix.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcphoenix.fw.localization.fusion.CorrectedPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryCorrectionEkfEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryCorrectionFusionEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;

/**
 * FTC-boundary owner for the common "predictor + AprilTag + corrected global pose" stack.
 *
 * <p>This lane consumes three stable inputs:</p>
 * <ul>
 *   <li>a Pinpoint motion predictor,</li>
 *   <li>a shared {@link AprilTagVisionLane}, and</li>
 *   <li>a field-fixed {@link TagLayout} describing which tags are trusted landmarks.</li>
 * </ul>
 *
 * <p>The vision lane owns device identity, camera mount, and backend cleanup. This localization lane
 * owns the estimation strategy built on top of those resources: predictor wiring, AprilTag-only
 * field solving, optional direct Limelight field pose, correction-source selection, corrected/global
 * estimator selection, and per-loop updates.</p>
 */
public final class FtcOdometryAprilTagLocalizationLane {

    /**
     * Which corrected/global estimator implementation the lane should own.
     */
    public enum GlobalEstimatorMode {
        /** Use the simpler gain-based predictor + correction fusion estimator. */
        FUSION,
        /** Use the covariance-aware EKF-style predictor + correction estimator. */
        EKF
    }

    /**
     * Which absolute correction source should feed the corrected/global estimator.
     */
    public enum CorrectionSourceMode {
        /**
         * Use the framework AprilTag pose solve built on raw AprilTag observations.
         */
        APRILTAG_POSE,
        /**
         * Use the Limelight's own direct field pose (botpose / MegaTag) when the backend supports it.
         */
        LIMELIGHT_FIELD_POSE
    }

    /** AprilTag-localization tuning owned by the localization lane. */
    public static final class AprilTagLocalizationConfig {

        /** Maximum accepted detection-frame age in seconds for AprilTag localization. */
        public double maxDetectionAgeSec = 0.50;

        /** Shared fixed-tag field-pose solver tuning used by AprilTag localization. */
        public FixedTagFieldPoseSolver.Config fieldPoseSolver = FixedTagFieldPoseSolver.Config.defaults();

        private AprilTagLocalizationConfig() {
        }

        /** @return new mutable config instance populated with framework defaults. */
        public static AprilTagLocalizationConfig defaults() {
            return new AprilTagLocalizationConfig();
        }

        /** @return deep copy whose nested solver config can be edited independently. */
        public AprilTagLocalizationConfig copy() {
            AprilTagLocalizationConfig c = new AprilTagLocalizationConfig();
            c.maxDetectionAgeSec = this.maxDetectionAgeSec;
            c.fieldPoseSolver = this.fieldPoseSolver.copy();
            return c;
        }

        /**
         * Builds an {@link AprilTagPoseEstimator.Config} by combining this localization tuning with
         * the specific camera mount from a shared vision lane.
         */
        public AprilTagPoseEstimator.Config toAprilTagPoseEstimatorConfig(CameraMountConfig cameraMount) {
            AprilTagPoseEstimator.Config c = AprilTagPoseEstimator.Config.defaults();
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
     * Configuration for how the lane chooses and builds its absolute correction source.
     */
    public static final class CorrectionSourceConfig {

        /** Which absolute correction source should be used for the corrected/global estimator. */
        public CorrectionSourceMode mode = CorrectionSourceMode.APRILTAG_POSE;

        /** Direct Limelight field-pose tuning used when {@link #mode} is {@link CorrectionSourceMode#LIMELIGHT_FIELD_POSE}. */
        public LimelightFieldPoseEstimator.Config limelightFieldPose = LimelightFieldPoseEstimator.Config.defaults();

        private CorrectionSourceConfig() {
        }

        /** @return new mutable correction-source config populated with framework defaults. */
        public static CorrectionSourceConfig defaults() {
            return new CorrectionSourceConfig();
        }

        /** @return deep copy of this correction-source config. */
        public CorrectionSourceConfig copy() {
            CorrectionSourceConfig c = new CorrectionSourceConfig();
            c.mode = this.mode;
            c.limelightFieldPose = this.limelightFieldPose.copy();
            return c;
        }
    }

    /**
     * Configuration for the FTC predictor + AprilTag localization lane.
     *
     * <p>The config groups the stable pieces of localization strategy: predictor tuning, AprilTag
     * field-solve tuning, correction-source selection, corrected-estimator tuning, and which
     * corrected/global estimator implementation to use. The camera rig itself is intentionally
     * separate and belongs to the concrete vision-lane config owned by the active backend.</p>
     */
    public static final class Config {

        /**
         * Motion-predictor configuration for the Pinpoint-based predictor.
         */
        public PinpointOdometryPredictor.Config predictor = PinpointOdometryPredictor.Config.defaults();

        /** AprilTag field-solve tuning for the raw AprilTag pose estimator. */
        public AprilTagLocalizationConfig aprilTags = AprilTagLocalizationConfig.defaults();

        /** Absolute correction source selection and direct-pose tuning. */
        public CorrectionSourceConfig correctionSource = CorrectionSourceConfig.defaults();

        /**
         * Gain-based fusion tuning used when {@link #correctedEstimatorMode} is {@link GlobalEstimatorMode#FUSION}.
         */
        public OdometryCorrectionFusionEstimator.Config correctionFusion =
                OdometryCorrectionFusionEstimator.Config.defaults();

        /**
         * EKF-style tuning used when {@link #correctedEstimatorMode} is {@link GlobalEstimatorMode#EKF}.
         */
        public OdometryCorrectionEkfEstimator.Config correctionEkf =
                OdometryCorrectionEkfEstimator.Config.defaults();

        /** Which corrected/global estimator implementation the lane should construct. */
        public GlobalEstimatorMode correctedEstimatorMode = GlobalEstimatorMode.FUSION;

        private Config() {
        }

        /** @return new mutable config instance populated with framework defaults. */
        public static Config defaults() {
            return new Config();
        }

        /** @return deep copy whose nested configs can be edited independently. */
        public Config copy() {
            Config c = new Config();
            c.predictor = this.predictor.copy();
            c.aprilTags = this.aprilTags.copy();
            c.correctionSource = this.correctionSource.copy();
            c.correctionFusion = this.correctionFusion.copy();
            c.correctionEkf = this.correctionEkf.copy();
            c.correctedEstimatorMode = this.correctedEstimatorMode;
            return c;
        }
    }

    private final Config cfg;
    private final AprilTagVisionLane visionLane;
    private final TagLayout fixedFieldTagLayout;
    private final PinpointOdometryPredictor predictor;
    private final AprilTagPoseEstimator aprilTagPoseEstimator;
    private final LimelightFieldPoseEstimator limelightFieldPoseEstimator;
    private final AbsolutePoseEstimator correctionEstimator;
    private final CorrectedPoseEstimator globalEstimator;

    /**
     * Creates the localization lane from one FTC hardware map, one shared vision lane, one field
     * tag layout, and one config snapshot.
     */
    public FtcOdometryAprilTagLocalizationLane(HardwareMap hardwareMap,
                                               AprilTagVisionLane visionLane,
                                               TagLayout fixedFieldTagLayout,
                                               Config config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.visionLane = Objects.requireNonNull(visionLane, "visionLane");
        this.fixedFieldTagLayout = Objects.requireNonNull(fixedFieldTagLayout, "fixedFieldTagLayout");
        this.cfg = Objects.requireNonNull(config, "config").copy();

        this.predictor = new PinpointOdometryPredictor(hardwareMap, this.cfg.predictor.copy());

        AprilTagPoseEstimator.Config aprilTagCfg =
                this.cfg.aprilTags.toAprilTagPoseEstimatorConfig(this.visionLane.cameraMountConfig());
        this.aprilTagPoseEstimator = new AprilTagPoseEstimator(
                this.visionLane.tagSensor(),
                this.fixedFieldTagLayout,
                aprilTagCfg
        );

        this.limelightFieldPoseEstimator = createLimelightFieldPoseEstimator();
        this.correctionEstimator = createCorrectionEstimator();
        this.globalEstimator = createGlobalEstimator(this.predictor, this.correctionEstimator);
    }

    /** @return defensive copy of the lane config owned by this localization owner. */
    public Config config() {
        return cfg.copy();
    }

    /**
     * @return shared vision lane backing this localization owner.
     */
    public AprilTagVisionLane visionLane() {
        return visionLane;
    }

    /**
     * @return fixed field layout trusted for localization and targeting.
     */
    public TagLayout fixedFieldTagLayout() {
        return fixedFieldTagLayout;
    }

    /**
     * @return predictor used for short-term propagation and replay.
     */
    public PinpointOdometryPredictor predictor() {
        return predictor;
    }

    /** @return raw AprilTag pose estimator built on top of the shared AprilTag sensor. */
    public AprilTagPoseEstimator aprilTagPoseEstimator() {
        return aprilTagPoseEstimator;
    }

    /**
     * Returns the optional direct Limelight field-pose estimator.
     *
     * @return direct Limelight field pose when the active backend is Limelight; otherwise {@code null}
     */
    public LimelightFieldPoseEstimator limelightFieldPoseEstimator() {
        return limelightFieldPoseEstimator;
    }

    /**
     * @return absolute correction source currently feeding the corrected/global estimator.
     */
    public AbsolutePoseEstimator correctionEstimator() {
        return correctionEstimator;
    }

    /** @return corrected/global estimator owned by this lane. */
    public CorrectedPoseEstimator globalEstimator() {
        return globalEstimator;
    }

    /**
     * Advances the localization lane for the current loop.
     *
     * <p>The corrected/global estimator drives the main predictor + active-correction update path.
     * The lane also refreshes any non-active absolute pose views afterward so callers can compare
     * raw AprilTag pose, direct Limelight field pose, and corrected/global pose side by side in the
     * same loop without needing to know which one is currently feeding correction.</p>
     */
    public void update(LoopClock clock) {
        globalEstimator.update(clock);

        if (correctionEstimator != aprilTagPoseEstimator) {
            aprilTagPoseEstimator.update(clock);
        }
        if (limelightFieldPoseEstimator != null && correctionEstimator != limelightFieldPoseEstimator) {
            limelightFieldPoseEstimator.update(clock);
        }
    }

    /** Emits a structured debug summary of the lane and its owned estimators. */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "localizationLane" : prefix;
        dbg.addData(p + ".correctionSourceMode", cfg.correctionSource.mode)
                .addData(p + ".correctedEstimatorMode", cfg.correctedEstimatorMode);
        visionLane.debugDump(dbg, p + ".visionLane");
        predictor.debugDump(dbg, p + ".predictor");
        aprilTagPoseEstimator.debugDump(dbg, p + ".aprilTagPoseEstimator");
        if (limelightFieldPoseEstimator != null) {
            limelightFieldPoseEstimator.debugDump(dbg, p + ".limelightFieldPoseEstimator");
        }
        correctionEstimator.debugDump(dbg, p + ".correctionEstimator");
        globalEstimator.debugDump(dbg, p + ".globalEstimator");
    }

    private LimelightFieldPoseEstimator createLimelightFieldPoseEstimator() {
        if (!(visionLane instanceof FtcLimelightAprilTagVisionLane)) {
            return null;
        }
        return new LimelightFieldPoseEstimator(
                (FtcLimelightAprilTagVisionLane) visionLane,
                predictor,
                cfg.correctionSource.limelightFieldPose.copy()
        );
    }

    private AbsolutePoseEstimator createCorrectionEstimator() {
        switch (cfg.correctionSource.mode) {
            case APRILTAG_POSE:
                return aprilTagPoseEstimator;

            case LIMELIGHT_FIELD_POSE:
                if (limelightFieldPoseEstimator == null) {
                    throw new IllegalArgumentException(
                            "FtcOdometryAprilTagLocalizationLane.Config.correctionSource.mode is LIMELIGHT_FIELD_POSE, "
                                    + "but the active vision lane is not FtcLimelightAprilTagVisionLane"
                    );
                }
                return limelightFieldPoseEstimator;

            default:
                throw new IllegalStateException("Unsupported correction source mode: " + cfg.correctionSource.mode);
        }
    }

    private CorrectedPoseEstimator createGlobalEstimator(PinpointOdometryPredictor predictor,
                                                         AbsolutePoseEstimator correction) {
        switch (cfg.correctedEstimatorMode) {
            case EKF:
                return new OdometryCorrectionEkfEstimator(
                        predictor,
                        correction,
                        cfg.correctionEkf.validatedCopy("FtcOdometryAprilTagLocalizationLane.Config.correctionEkf")
                );

            case FUSION:
            default:
                return new OdometryCorrectionFusionEstimator(
                        predictor,
                        correction,
                        cfg.correctionFusion.validatedCopy("FtcOdometryAprilTagLocalizationLane.Config.correctionFusion")
                );
        }
    }
}
