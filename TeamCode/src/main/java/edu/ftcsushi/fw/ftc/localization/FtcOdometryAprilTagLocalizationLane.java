package edu.ftcsushi.fw.ftc.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.field.TagLayouts;
import edu.ftcsushi.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcsushi.fw.ftc.vision.FtcLimelightAprilTagVisionLane;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.MotionPredictor;
import edu.ftcsushi.fw.localization.apriltag.AprilTagPoseEstimator;
import edu.ftcsushi.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcsushi.fw.localization.fusion.CorrectedPoseEstimator;
import edu.ftcsushi.fw.localization.fusion.OdometryCorrectionEkfEstimator;
import edu.ftcsushi.fw.localization.fusion.OdometryCorrectionFusionEstimator;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * FTC-boundary owner for the common "predictor + AprilTag + corrected global pose" stack.
 *
 * <p>This lane consumes three stable inputs:</p>
 * <ul>
 *   <li>one {@link MotionPredictor},</li>
 *   <li>a shared {@link AprilTagVisionLane}, and</li>
 *   <li>a field-fixed {@link TagLayout} describing which tags are trusted landmarks.</li>
 * </ul>
 *
 * <p>The ordinary FTC construction path creates and owns a configured
 * {@link PinpointOdometryPredictor}. Integrations that already own a predictor use
 * {@link #withPredictor(MotionPredictor, AprilTagVisionLane, TagLayout, EstimatorConfig)} so this
 * lane and the integration share that one backend-neutral source instead of creating a competing
 * hardware owner.</p>
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
            c.fieldPoseSolver = this.fieldPoseSolver != null
                    ? this.fieldPoseSolver.copy()
                    : null;
            return c;
        }

        /**
         * Validates and snapshots the shared AprilTag-localization policy without touching vision
         * or hardware.
         *
         * @param context diagnostic owner/field prefix; null or blank uses this Config's canonical
         *                class name
         * @return independent validated snapshot
         * @throws NullPointerException if the active solver draft is null
         * @throws IllegalArgumentException if the age or any solver value is outside its
         *                                  documented domain
         */
        public AprilTagLocalizationConfig validatedCopy(String context) {
            String owner = normalizedContext(context, AprilTagLocalizationConfig.class);
            AprilTagLocalizationConfig c = copy();
            if (!Double.isFinite(c.maxDetectionAgeSec) || c.maxDetectionAgeSec < 0.0) {
                throw new IllegalArgumentException(
                        owner + ".maxDetectionAgeSec must be finite and >= 0; received "
                                + c.maxDetectionAgeSec
                );
            }
            FixedTagFieldPoseSolver.Config solverConfig = Objects.requireNonNull(
                    c.fieldPoseSolver,
                    owner + ".fieldPoseSolver must not be null"
            );
            validateFieldPoseSolver(solverConfig, owner + ".fieldPoseSolver");
            return c;
        }

        /**
         * Builds a raw {@link AprilTagPoseEstimator.Config} authoring copy by combining this
         * localization tuning with the specific camera mount from a shared vision lane.
         *
         * <p>This conversion does not validate or invent defaults. It independently copies a
         * non-null solver draft, preserves a null draft or mount, and leaves the active estimator
         * owner to reject incomplete configuration.</p>
         */
        public AprilTagPoseEstimator.Config toAprilTagPoseEstimatorConfig(CameraMountConfig cameraMount) {
            AprilTagPoseEstimator.Config c = AprilTagPoseEstimator.Config.defaults();
            c.fieldPoseSolver = this.fieldPoseSolver != null ? this.fieldPoseSolver.copy() : null;
            c.maxDetectionAgeSec = this.maxDetectionAgeSec;
            c.cameraMount = cameraMount;
            return c;
        }
    }

    /**
     * Configuration for how the lane chooses and builds its absolute correction source.
     */
    public static final class CorrectionSourceConfig {

        /** Which absolute correction source should be used for the corrected/global estimator. */
        public CorrectionSourceMode mode = CorrectionSourceMode.APRILTAG_POSE;

        /**
         * Direct Limelight field-pose tuning.
         *
         * <p>This draft is active when {@link #mode} selects
         * {@link CorrectionSourceMode#LIMELIGHT_FIELD_POSE}, and whenever the supplied vision lane
         * is a Limelight backend because the composite also retains the direct field-pose estimator
         * as a diagnostic view.</p>
         */
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
            c.limelightFieldPose = this.limelightFieldPose != null
                    ? this.limelightFieldPose.copy()
                    : null;
            return c;
        }
    }

    /**
     * Estimation policy shared by both lane construction paths.
     *
     * <p>This config deliberately excludes predictor hardware and calibration. An integration that
     * already owns a predictor can therefore supply this complete estimator policy without also
     * authoring an ignored Pinpoint subsection.</p>
     */
    public static final class EstimatorConfig {
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

        private EstimatorConfig() {
        }

        /** @return new mutable config instance populated with framework defaults. */
        public static EstimatorConfig defaults() {
            return new EstimatorConfig();
        }

        /**
         * Returns a raw authoring copy.
         *
         * <p>Every non-null mutable nested draft is copied independently. A null draft remains null
         * so an inactive branch can be copied without inventing a default or failing before the
         * active owner selects its graph.</p>
         *
         * @return independent, nonvalidating authoring copy
         */
        public EstimatorConfig copy() {
            EstimatorConfig c = new EstimatorConfig();
            c.aprilTags = this.aprilTags != null ? this.aprilTags.copy() : null;
            c.correctionSource = this.correctionSource != null
                    ? this.correctionSource.copy()
                    : null;
            c.correctionFusion = this.correctionFusion != null
                    ? this.correctionFusion.copy()
                    : null;
            c.correctionEkf = this.correctionEkf != null
                    ? this.correctionEkf.copy()
                    : null;
            c.correctedEstimatorMode = this.correctedEstimatorMode;
            return c;
        }
    }

    /**
     * Complete configuration for the ordinary lane-owned Pinpoint path.
     *
     * <p>The lane owns both the Pinpoint predictor described by {@link #predictor} and the estimator
     * graph described by {@link #estimation}. Camera hardware and trusted field facts remain owned
     * by the supplied vision lane and {@link TagLayout}, respectively.</p>
     */
    public static final class Config {

        /** Pinpoint identity and calibration owned by the ordinary {@link HardwareMap} path. */
        public PinpointOdometryPredictor.Config predictor = PinpointOdometryPredictor.Config.defaults();

        /** Estimation policy applied around the lane-owned predictor. */
        public EstimatorConfig estimation = EstimatorConfig.defaults();

        private Config() {
        }

        /** @return new mutable complete config populated with framework defaults. */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Returns a raw authoring copy whose non-null nested drafts are independent.
         *
         * <p>Null drafts remain null and are rejected only if the active construction path owns
         * them.</p>
         *
         * @return independent, nonvalidating authoring copy
         */
        public Config copy() {
            Config c = new Config();
            c.predictor = this.predictor != null ? this.predictor.copy() : null;
            c.estimation = this.estimation != null ? this.estimation.copy() : null;
            return c;
        }

        /**
         * Validates and snapshots every intrinsically active branch before FTC resource effects.
         *
         * <p>The selected Fusion/EKF policy and a directly selected Limelight correction policy
         * are active here. A Limelight diagnostic policy that becomes active only when an opened
         * lane is actually Limelight remains a post-open check in the aggregate owner.</p>
         *
         * @param context diagnostic owner/field prefix; null or blank uses this Config's canonical
         *                class name
         * @return independent validated snapshot
         * @throws NullPointerException for a required null active field
         * @throws IllegalArgumentException for an invalid active value
         */
        public Config validatedCopy(String context) {
            String owner = normalizedContext(context, Config.class);
            Config c = copy();
            c.predictor = Objects.requireNonNull(
                    c.predictor,
                    owner + ".predictor must not be null"
            ).validatedCopy(owner + ".predictor");
            c.estimation = captureIntrinsicEstimatorConfig(
                    Objects.requireNonNull(
                            c.estimation,
                            owner + ".estimation must not be null"
                    ),
                    owner + ".estimation"
            );
            return c;
        }
    }

    private final AprilTagVisionLane visionLane;
    private final MotionPredictor predictor;
    private final AprilTagPoseEstimator aprilTagPoseEstimator;
    private final LimelightFieldPoseEstimator limelightFieldPoseEstimator;
    private final AbsolutePoseEstimator correctionEstimator;
    private final CorrectedPoseEstimator globalEstimator;
    private final CorrectionSourceMode correctionSourceMode;
    private final GlobalEstimatorMode correctedEstimatorMode;
    private long lastUpdateCycle = Long.MIN_VALUE;
    private boolean updateInProgress;
    private RuntimeException lastUpdateFailure;

    /** Effect-free estimator inputs captured before owned Pinpoint acquisition. */
    private static final class EstimatorInputs {
        final AprilTagVisionLane visionLane;
        final FtcLimelightAprilTagVisionLane limelightVisionLane;
        final AprilTagPoseEstimator aprilTagPoseEstimator;
        final LimelightFieldPoseEstimator.Config limelightFieldPoseConfig;
        final OdometryCorrectionFusionEstimator.Config correctionFusionConfig;
        final OdometryCorrectionEkfEstimator.Config correctionEkfConfig;
        final CorrectionSourceMode correctionSourceMode;
        final GlobalEstimatorMode correctedEstimatorMode;

        EstimatorInputs(AprilTagVisionLane visionLane,
                        FtcLimelightAprilTagVisionLane limelightVisionLane,
                        AprilTagPoseEstimator aprilTagPoseEstimator,
                        LimelightFieldPoseEstimator.Config limelightFieldPoseConfig,
                        OdometryCorrectionFusionEstimator.Config correctionFusionConfig,
                        OdometryCorrectionEkfEstimator.Config correctionEkfConfig,
                        CorrectionSourceMode correctionSourceMode,
                        GlobalEstimatorMode correctedEstimatorMode) {
            this.visionLane = visionLane;
            this.limelightVisionLane = limelightVisionLane;
            this.aprilTagPoseEstimator = aprilTagPoseEstimator;
            this.limelightFieldPoseConfig = limelightFieldPoseConfig;
            this.correctionFusionConfig = correctionFusionConfig;
            this.correctionEkfConfig = correctionEkfConfig;
            this.correctionSourceMode = correctionSourceMode;
            this.correctedEstimatorMode = correctedEstimatorMode;
        }
    }

    /** Complete, validated constructor inputs after the applicable predictor is selected. */
    private static final class ConstructionInputs {
        final MotionPredictor predictor;
        final EstimatorInputs estimation;

        ConstructionInputs(MotionPredictor predictor, EstimatorInputs estimation) {
            this.predictor = predictor;
            this.estimation = estimation;
        }
    }

    /**
     * Creates the localization lane with a new configured Pinpoint predictor, one shared vision
     * lane, one field tag layout, and one config snapshot.
     *
     * <p>This is the ordinary FTC/TeleOp convenience path. The lane constructs the predictor from
     * {@link Config#predictor} and then owns its per-loop update as part of the corrected-localization
     * graph.</p>
     *
     * <p>Construction validates the complete active estimator branches, snapshots the fixed tag
     * layout, and captures the vision lane's mount and sensor exactly once before Pinpoint device
     * lookup, configuration, or reset. Invalid aggregate configuration therefore cannot begin the
     * owned hardware graph.</p>
     *
     * @param hardwareMap FTC hardware registry used only after complete preflight
     * @param visionLane shared AprilTag vision owner whose mount and sensor are captured once
     * @param fixedFieldTagLayout fixed FTC-field tag facts snapshotted before hardware effects
     * @param config complete predictor-plus-estimator authoring draft
     * @throws NullPointerException for a required null owner, collaborator, or active Config field
     * @throws IllegalArgumentException for an invalid active Config value
     */
    public FtcOdometryAprilTagLocalizationLane(HardwareMap hardwareMap,
                                               AprilTagVisionLane visionLane,
                                               TagLayout fixedFieldTagLayout,
                                               Config config) {
        this(ownedPinpointInputs(hardwareMap, visionLane, fixedFieldTagLayout, config));
    }

    /**
     * Creates a localization lane around one already-configured backend-neutral motion predictor.
     *
     * <p>This path is for an integration or composition root that already owns predictor
     * construction. The supplied predictor becomes the one predictor used by correction fusion,
     * optional Limelight motion gating, debug output, and {@link #predictor()}; the lane never
     * replaces it with a Pinpoint instance.</p>
     *
     * <p>The supplied {@link EstimatorConfig} contains no Pinpoint subsection, so this path cannot
     * accept or report hardware configuration that belongs to the external owner. Construction
     * validates and snapshots the complete active estimator graph without sampling, resetting, or
     * otherwise touching {@code predictor}.</p>
     *
     * @param predictor          already-configured predictor shared with the external integration
     * @param visionLane         shared AprilTag vision owner
     * @param fixedFieldTagLayout fixed FTC-field tag layout used for localization
     * @param estimation         correction and estimator configuration draft
     * @return localization lane using exactly {@code predictor}
     */
    public static FtcOdometryAprilTagLocalizationLane withPredictor(
            MotionPredictor predictor,
            AprilTagVisionLane visionLane,
            TagLayout fixedFieldTagLayout,
            EstimatorConfig estimation) {
        return new FtcOdometryAprilTagLocalizationLane(injectedInputs(
                predictor,
                visionLane,
                fixedFieldTagLayout,
                estimation
        ));
    }

    /** Build the shared estimator graph around one already-created predictor. */
    private FtcOdometryAprilTagLocalizationLane(ConstructionInputs inputs) {
        this.predictor = inputs.predictor;
        EstimatorInputs estimation = inputs.estimation;
        this.visionLane = estimation.visionLane;
        this.aprilTagPoseEstimator = estimation.aprilTagPoseEstimator;
        this.correctionSourceMode = estimation.correctionSourceMode;
        this.correctedEstimatorMode = estimation.correctedEstimatorMode;

        this.limelightFieldPoseEstimator = estimation.limelightVisionLane != null
                ? new LimelightFieldPoseEstimator(
                        estimation.limelightVisionLane,
                        this.predictor,
                        estimation.limelightFieldPoseConfig
                )
                : null;
        this.correctionEstimator = selectCorrectionEstimator(
                this.correctionSourceMode,
                this.aprilTagPoseEstimator,
                this.limelightFieldPoseEstimator
        );
        this.globalEstimator = createGlobalEstimator(
                this.predictor,
                this.correctionEstimator,
                estimation
        );
    }

    /**
     * @return predictor used for short-term propagation and replay.
     */
    public MotionPredictor predictor() {
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
     *
     * <p>The lane claims the cycle before traversing any child owner. A repeated successful call in
     * that cycle is a no-op, a repeated call after failure rethrows the exact first
     * {@link RuntimeException}, and recursive entry fails before another child update. The complete
     * predictor, correction, global, and diagnostic-view set therefore remains one coherent
     * snapshot until the next cycle.</p>
     */
    public void update(LoopClock clock) {
        LoopClock requiredClock = Objects.requireNonNull(clock, "clock");
        long cycle = requiredClock.cycle();
        if (updateInProgress) {
            throw new IllegalStateException(
                    "FtcOdometryAprilTagLocalizationLane.update(clock) was reentered during cycle "
                            + cycle + "; the localization lane may advance only once per cycle"
            );
        }
        if (cycle == lastUpdateCycle) {
            if (lastUpdateFailure != null) {
                throw lastUpdateFailure;
            }
            return;
        }

        // Claim the whole traversal before any predictor or correction owner can call back in.
        lastUpdateCycle = cycle;
        updateInProgress = true;
        lastUpdateFailure = null;
        try {
            updateCurrentCycle(requiredClock);
        } catch (RuntimeException failure) {
            lastUpdateFailure = failure;
            throw failure;
        } finally {
            updateInProgress = false;
        }
    }

    /** Advance the complete predictor/correction/diagnostic graph for one claimed cycle. */
    private void updateCurrentCycle(LoopClock clock) {
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
        dbg.addData(p + ".correctionSourceMode", correctionSourceMode)
                .addData(p + ".correctedEstimatorMode", correctedEstimatorMode);
        visionLane.debugDump(dbg, p + ".visionLane");
        predictor.debugDump(dbg, p + ".predictor");
        aprilTagPoseEstimator.debugDump(dbg, p + ".aprilTagPoseEstimator");
        if (limelightFieldPoseEstimator != null) {
            limelightFieldPoseEstimator.debugDump(dbg, p + ".limelightFieldPoseEstimator");
        }
        correctionEstimator.debugDump(dbg, p + ".correctionEstimator");
        globalEstimator.debugDump(dbg, p + ".globalEstimator");
    }

    /** Select the already-constructed absolute source after backend compatibility was preflighted. */
    private static AbsolutePoseEstimator selectCorrectionEstimator(
            CorrectionSourceMode mode,
            AprilTagPoseEstimator aprilTags,
            LimelightFieldPoseEstimator limelightFieldPose
    ) {
        switch (mode) {
            case APRILTAG_POSE:
                return aprilTags;

            case LIMELIGHT_FIELD_POSE:
                if (limelightFieldPose == null) {
                    throw new IllegalStateException(
                            "Limelight correction source passed preflight without a Limelight estimator"
                    );
                }
                return limelightFieldPose;

            default:
                throw new IllegalStateException("Unsupported correction source mode: " + mode);
        }
    }

    /** Construct the selected corrected estimator from its already-validated active config. */
    private static CorrectedPoseEstimator createGlobalEstimator(
            MotionPredictor predictor,
            AbsolutePoseEstimator correction,
            EstimatorInputs estimation
    ) {
        switch (estimation.correctedEstimatorMode) {
            case EKF:
                return new OdometryCorrectionEkfEstimator(
                        predictor,
                        correction,
                        estimation.correctionEkfConfig
                );

            case FUSION:
                return new OdometryCorrectionFusionEstimator(
                        predictor,
                        correction,
                        estimation.correctionFusionConfig
                );

            default:
                throw new IllegalStateException(
                        "Unsupported corrected estimator mode: "
                                + estimation.correctedEstimatorMode
                );
        }
    }

    /**
     * Capture every authored input before creating the convenience-path Pinpoint hardware owner.
     *
     * <p>All configuration, backend, layout, and completed-vision facts are validated first. Only
     * after that effect-free preflight succeeds may the Pinpoint constructor look up, configure,
     * and reset its hardware device.</p>
     */
    private static ConstructionInputs ownedPinpointInputs(HardwareMap hardwareMap,
                                                          AprilTagVisionLane visionLane,
                                                          TagLayout fixedFieldTagLayout,
                                                          Config config) {
        HardwareMap requiredHardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        AprilTagVisionLane requiredVisionLane = Objects.requireNonNull(visionLane, "visionLane");
        TagLayout requiredLayout = Objects.requireNonNull(fixedFieldTagLayout, "fixedFieldTagLayout");
        Config copiedConfig = Objects.requireNonNull(config, "config").validatedCopy(
                FtcOdometryAprilTagLocalizationLane.class.getCanonicalName() + ".Config"
        );
        PinpointOdometryPredictor.Config predictorConfig = copiedConfig.predictor;
        EstimatorInputs estimatorInputs = captureEstimatorInputs(
                requiredVisionLane,
                requiredLayout,
                Objects.requireNonNull(
                        copiedConfig.estimation,
                        "FtcOdometryAprilTagLocalizationLane.Config.estimation"
                ),
                "FtcOdometryAprilTagLocalizationLane.Config.estimation"
        );

        return new ConstructionInputs(
                new PinpointOdometryPredictor(requiredHardwareMap, predictorConfig),
                estimatorInputs
        );
    }

    /** Validate and copy the estimator graph inputs without constructing another predictor. */
    private static ConstructionInputs injectedInputs(MotionPredictor predictor,
                                                      AprilTagVisionLane visionLane,
                                                      TagLayout fixedFieldTagLayout,
                                                      EstimatorConfig config) {
        MotionPredictor requiredPredictor = Objects.requireNonNull(predictor, "predictor");
        AprilTagVisionLane requiredVisionLane = Objects.requireNonNull(visionLane, "visionLane");
        TagLayout requiredLayout = Objects.requireNonNull(fixedFieldTagLayout, "fixedFieldTagLayout");
        return new ConstructionInputs(
                requiredPredictor,
                captureEstimatorInputs(
                        requiredVisionLane,
                        requiredLayout,
                        Objects.requireNonNull(config, "estimation"),
                        "FtcOdometryAprilTagLocalizationLane.EstimatorConfig"
                )
        );
    }

    /**
     * Raw-copy, validate, and bind one estimator graph without sampling a predictor or hardware.
     */
    private static EstimatorInputs captureEstimatorInputs(
            AprilTagVisionLane visionLane,
            TagLayout fixedFieldTagLayout,
            EstimatorConfig authoredConfig,
            String context
    ) {
        EstimatorConfig config = captureIntrinsicEstimatorConfig(authoredConfig, context);
        AprilTagLocalizationConfig aprilTags = config.aprilTags;
        CorrectionSourceConfig correctionSource = Objects.requireNonNull(
                config.correctionSource,
                context + ".correctionSource"
        );
        CorrectionSourceMode correctionSourceMode = Objects.requireNonNull(
                correctionSource.mode,
                context + ".correctionSource.mode"
        );
        GlobalEstimatorMode correctedEstimatorMode = Objects.requireNonNull(
                config.correctedEstimatorMode,
                context + ".correctedEstimatorMode"
        );

        OdometryCorrectionFusionEstimator.Config correctionFusion =
                correctedEstimatorMode == GlobalEstimatorMode.FUSION
                        ? config.correctionFusion
                        : null;
        OdometryCorrectionEkfEstimator.Config correctionEkf =
                correctedEstimatorMode == GlobalEstimatorMode.EKF
                        ? config.correctionEkf
                        : null;

        FtcLimelightAprilTagVisionLane limelightVisionLane =
                visionLane instanceof FtcLimelightAprilTagVisionLane
                        ? (FtcLimelightAprilTagVisionLane) visionLane
                        : null;
        boolean limelightConfigActive = limelightVisionLane != null
                || correctionSourceMode == CorrectionSourceMode.LIMELIGHT_FIELD_POSE;
        LimelightFieldPoseEstimator.Config limelightFieldPoseConfig = null;
        if (limelightConfigActive) {
            limelightFieldPoseConfig = Objects.requireNonNull(
                    correctionSource.limelightFieldPose,
                    context + ".correctionSource.limelightFieldPose"
            ).validatedCopy(context + ".correctionSource.limelightFieldPose");
        }

        if (correctionSourceMode == CorrectionSourceMode.LIMELIGHT_FIELD_POSE
                && limelightVisionLane == null) {
            throw new IllegalArgumentException(
                    context + ".correctionSource.mode is LIMELIGHT_FIELD_POSE, but visionLane "
                            + "must be FtcLimelightAprilTagVisionLane; received "
                            + visionLane.getClass().getName()
            );
        }

        TagLayout layoutSnapshot = TagLayouts.snapshot(fixedFieldTagLayout);
        CameraMountConfig cameraMount = Objects.requireNonNull(
                visionLane.cameraMountConfig(),
                "visionLane.cameraMountConfig()"
        );
        AprilTagSensor tagSensor = Objects.requireNonNull(
                visionLane.tagSensor(),
                "visionLane.tagSensor()"
        );
        AprilTagPoseEstimator aprilTagPoseEstimator = new AprilTagPoseEstimator(
                tagSensor,
                layoutSnapshot,
                aprilTags.toAprilTagPoseEstimatorConfig(cameraMount)
        );

        return new EstimatorInputs(
                visionLane,
                limelightVisionLane,
                aprilTagPoseEstimator,
                limelightFieldPoseConfig,
                correctionFusion,
                correctionEkf,
                correctionSourceMode,
                correctedEstimatorMode
        );
    }

    /** Copy and validate branches known to be active before an actual vision backend is opened. */
    private static EstimatorConfig captureIntrinsicEstimatorConfig(
            EstimatorConfig authoredConfig,
            String context
    ) {
        EstimatorConfig config = Objects.requireNonNull(
                authoredConfig,
                context + " must not be null"
        ).copy();
        config.aprilTags = Objects.requireNonNull(
                config.aprilTags,
                context + ".aprilTags must not be null"
        ).validatedCopy(context + ".aprilTags");
        config.correctionSource = Objects.requireNonNull(
                config.correctionSource,
                context + ".correctionSource must not be null"
        );
        config.correctionSource.mode = Objects.requireNonNull(
                config.correctionSource.mode,
                context + ".correctionSource.mode must not be null"
        );
        config.correctedEstimatorMode = Objects.requireNonNull(
                config.correctedEstimatorMode,
                context + ".correctedEstimatorMode must not be null"
        );

        switch (config.correctedEstimatorMode) {
            case FUSION:
                config.correctionFusion = Objects.requireNonNull(
                        config.correctionFusion,
                        context + ".correctionFusion must not be null"
                ).validatedCopy(context + ".correctionFusion");
                break;

            case EKF:
                config.correctionEkf = Objects.requireNonNull(
                        config.correctionEkf,
                        context + ".correctionEkf must not be null"
                ).validatedCopy(context + ".correctionEkf");
                break;

            default:
                throw new IllegalStateException(
                        context + ".correctedEstimatorMode is unsupported: "
                                + config.correctedEstimatorMode
                );
        }

        if (config.correctionSource.mode == CorrectionSourceMode.LIMELIGHT_FIELD_POSE) {
            config.correctionSource.limelightFieldPose = Objects.requireNonNull(
                    config.correctionSource.limelightFieldPose,
                    context + ".correctionSource.limelightFieldPose must not be null"
            ).validatedCopy(context + ".correctionSource.limelightFieldPose");
        }
        return config;
    }

    /** Validate the existing solver owner and translate its canonical field prefix to this owner. */
    private static void validateFieldPoseSolver(FixedTagFieldPoseSolver.Config config,
                                                String context) {
        try {
            new FixedTagFieldPoseSolver(config);
        } catch (IllegalArgumentException failure) {
            String message = failure.getMessage();
            String translated = message == null
                    ? context + " is invalid"
                    : message.replace("FixedTagFieldPoseSolver.Config", context);
            throw new IllegalArgumentException(translated, failure);
        }
    }

    /** Normalize one optional diagnostic prefix without preserving accidental surrounding spaces. */
    private static String normalizedContext(String context, Class<?> configType) {
        if (context == null || context.trim().isEmpty()) {
            return configType.getCanonicalName();
        }
        return context.trim();
    }
}
