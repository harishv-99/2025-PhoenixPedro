package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.spatial.FacingTarget2d;
import edu.ftcphoenix.fw.spatial.SpatialControlFrames;
import edu.ftcphoenix.fw.spatial.SpatialQuerySpec;
import edu.ftcphoenix.fw.spatial.TranslationTarget2d;

/**
 * Controller-neutral configuration for {@link DriveGuidance}.
 *
 * <p>A {@link DriveGuidanceSpec} now has two layers:</p>
 * <ol>
 *   <li>a shared {@link #spatialQuerySpec spatial query spec} that describes the target,
 *       controlled frames, and solve lanes</li>
 *   <li>drive-specific arbitration policy for blending/choosing between those lane results</li>
 * </ol>
 *
 * <p>A spec intentionally contains <strong>no controller tuning</strong>. That keeps “what I want”
 * reusable across TeleOp overlays, autonomous tasks, and readiness queries.</p>
 */
public final class DriveGuidanceSpec {

    /**
     * Explicit solve mode for guidance.
     */
    public enum SolveMode {
        LOCALIZATION_ONLY,
        APRIL_TAGS_ONLY,
        ADAPTIVE
    }

    /**
     * Omega arbitration policy when adaptive solve mode is active.
     */
    public enum OmegaPolicy {
        /**
         * Whenever live AprilTag omega is valid, prefer it over localization.
         */
        PREFER_APRIL_TAGS_WHEN_VALID,
        /**
         * Keep omega on the same side of the takeover decision as translation.
         */
        FOLLOW_TRANSLATION_TAKEOVER
    }

    /**
     * Behavior when guidance cannot produce a valid command from the available solve lanes.
     */
    public enum LossPolicy {
        PASS_THROUGH,
        ZERO_OUTPUT
    }

    /**
     * Robot-relative movement captured from the translation control frame when guidance enables.
     *
     * <p>This remains a drive-guidance-specific target because it is not just a geometric point.
     * It is a <em>latched target semantic</em>: capture the current translation frame in field space
     * on enable, then offset from that captured anchor in the frame's local forward/left axes.</p>
     */
    public static final class RobotRelativePoint implements TranslationTarget2d {
        public final double forwardInches;
        public final double leftInches;

        /**
         * Creates a robot-relative translation target captured from the translation control frame
         * when guidance enables.
         */
        public RobotRelativePoint(double forwardInches, double leftInches) {
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
        }
    }

    /**
     * Live AprilTag solve lane used directly by guidance.
     */
    public static final class AprilTags {
        public static final double DEFAULT_MAX_AGE_SEC = 0.50;

        public final AprilTagSensor sensor;
        public final CameraMountConfig cameraMount;
        public final double maxAgeSec;
        public final FixedTagFieldPoseSolver.Config fieldPoseSolverConfig;

        public AprilTags(AprilTagSensor sensor, CameraMountConfig cameraMount) {
            this(sensor, cameraMount, DEFAULT_MAX_AGE_SEC, FixedTagFieldPoseSolver.Config.defaults());
        }

        public AprilTags(AprilTagSensor sensor, CameraMountConfig cameraMount, double maxAgeSec) {
            this(sensor, cameraMount, maxAgeSec, FixedTagFieldPoseSolver.Config.defaults());
        }

        public AprilTags(AprilTagSensor sensor,
                         CameraMountConfig cameraMount,
                         double maxAgeSec,
                         FixedTagFieldPoseSolver.Config fieldPoseSolverConfig) {
            this.sensor = Objects.requireNonNull(sensor, "sensor");
            this.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
            this.maxAgeSec = maxAgeSec;
            this.fieldPoseSolverConfig = fieldPoseSolverConfig != null
                    ? FixedTagFieldPoseSolver.Config.normalizedValidatedCopyOf(
                    fieldPoseSolverConfig,
                    "DriveGuidanceSpec.AprilTags.fieldPoseSolverConfig"
            )
                    : FixedTagFieldPoseSolver.Config.defaults();
        }
    }

    /**
     * Localization solve lane parameters.
     */
    public static final class Localization {
        public static final double DEFAULT_MAX_AGE_SEC = 0.50;
        public static final double DEFAULT_MIN_QUALITY = 0.10;

        public final AbsolutePoseEstimator poseEstimator;
        public final double maxAgeSec;
        public final double minQuality;

        public Localization(AbsolutePoseEstimator poseEstimator) {
            this(poseEstimator, DEFAULT_MAX_AGE_SEC, DEFAULT_MIN_QUALITY);
        }

        public Localization(AbsolutePoseEstimator poseEstimator, double maxAgeSec, double minQuality) {
            this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            this.maxAgeSec = maxAgeSec;
            this.minQuality = minQuality;
        }
    }

    /**
     * Hysteresis + blending parameters for adaptive translation takeover.
     */
    public static final class TranslationTakeover {
        public static final double DEFAULT_ENTER_RANGE_IN = 9.0;
        public static final double DEFAULT_EXIT_RANGE_IN = 12.0;
        public static final double DEFAULT_BLEND_SEC = 0.15;

        public final double enterRangeInches;
        public final double exitRangeInches;
        public final double blendSec;

        public TranslationTakeover(double enterRangeInches, double exitRangeInches, double blendSec) {
            this.enterRangeInches = enterRangeInches;
            this.exitRangeInches = exitRangeInches;
            this.blendSec = blendSec;
        }

        public static TranslationTakeover defaults() {
            return new TranslationTakeover(DEFAULT_ENTER_RANGE_IN, DEFAULT_EXIT_RANGE_IN, DEFAULT_BLEND_SEC);
        }
    }

    /**
     * Explicit solve-lane configuration for guidance.
     */
    public static final class ResolveWith {
        public final SolveMode mode;
        public final AprilTags aprilTags;
        public final Localization localization;
        public final TagLayout fixedAprilTagLayout;
        public final TranslationTakeover translationTakeover;
        public final OmegaPolicy omegaPolicy;
        public final LossPolicy lossPolicy;

        private ResolveWith(SolveMode mode,
                            AprilTags aprilTags,
                            Localization localization,
                            TagLayout fixedAprilTagLayout,
                            TranslationTakeover translationTakeover,
                            OmegaPolicy omegaPolicy,
                            LossPolicy lossPolicy) {
            this.mode = mode;
            this.aprilTags = aprilTags;
            this.localization = localization;
            this.fixedAprilTagLayout = fixedAprilTagLayout;
            this.translationTakeover = translationTakeover;
            this.omegaPolicy = omegaPolicy;
            this.lossPolicy = lossPolicy;
        }

        public static ResolveWith create(SolveMode mode,
                                         AprilTags aprilTags,
                                         Localization localization,
                                         TagLayout fixedAprilTagLayout,
                                         TranslationTakeover translationTakeover,
                                         OmegaPolicy omegaPolicy,
                                         LossPolicy lossPolicy) {
            Objects.requireNonNull(mode, "mode");
            LossPolicy lp = (lossPolicy != null) ? lossPolicy : LossPolicy.PASS_THROUGH;
            OmegaPolicy op = (omegaPolicy != null) ? omegaPolicy : OmegaPolicy.PREFER_APRIL_TAGS_WHEN_VALID;

            switch (mode) {
                case LOCALIZATION_ONLY:
                    if (localization == null) {
                        throw new IllegalArgumentException("LOCALIZATION_ONLY requires localization(...)");
                    }
                    if (aprilTags != null) {
                        throw new IllegalArgumentException("LOCALIZATION_ONLY does not accept aprilTags(...); use ADAPTIVE when both lanes are intended");
                    }
                    return new ResolveWith(mode, null, localization, fixedAprilTagLayout, null, op, lp);
                case APRIL_TAGS_ONLY:
                    if (aprilTags == null) {
                        throw new IllegalArgumentException("APRIL_TAGS_ONLY requires aprilTags(...)");
                    }
                    if (localization != null) {
                        throw new IllegalArgumentException("APRIL_TAGS_ONLY does not accept localization(...); use ADAPTIVE when both lanes are intended");
                    }
                    return new ResolveWith(mode, aprilTags, null, fixedAprilTagLayout, null, op, lp);
                case ADAPTIVE:
                default:
                    if (aprilTags == null || localization == null) {
                        throw new IllegalArgumentException("ADAPTIVE requires both localization(...) and aprilTags(...)");
                    }
                    return new ResolveWith(mode,
                            aprilTags,
                            localization,
                            fixedAprilTagLayout,
                            translationTakeover != null ? translationTakeover : TranslationTakeover.defaults(),
                            op,
                            lp);
            }
        }

        public boolean hasAprilTags() {
            return aprilTags != null;
        }

        public boolean hasLocalization() {
            return localization != null;
        }

        public boolean hasFixedAprilTagLayout() {
            return fixedAprilTagLayout != null;
        }

        public boolean isAdaptive() {
            return mode == SolveMode.ADAPTIVE;
        }
    }

    public final TranslationTarget2d translationTarget;
    public final FacingTarget2d facingTarget;
    public final SpatialControlFrames controlFrames;
    public final ResolveWith resolveWith;
    public final SpatialQuerySpec spatialQuerySpec;
    public final int localizationLaneIndex;
    public final int aprilTagsLaneIndex;

    DriveGuidanceSpec(TranslationTarget2d translationTarget,
                      FacingTarget2d facingTarget,
                      SpatialControlFrames controlFrames,
                      ResolveWith resolveWith,
                      SpatialQuerySpec spatialQuerySpec,
                      int localizationLaneIndex,
                      int aprilTagsLaneIndex) {
        this.translationTarget = translationTarget;
        this.facingTarget = facingTarget;
        this.controlFrames = Objects.requireNonNull(controlFrames, "controlFrames");
        this.resolveWith = Objects.requireNonNull(resolveWith, "resolveWith");
        this.spatialQuerySpec = spatialQuerySpec;
        this.localizationLaneIndex = localizationLaneIndex;
        this.aprilTagsLaneIndex = aprilTagsLaneIndex;
    }

    public DriveOverlayMask requestedMask() {
        boolean t = translationTarget != null;
        boolean o = facingTarget != null;
        if (t && o) {
            return DriveOverlayMask.ALL;
        }
        if (t) {
            return DriveOverlayMask.TRANSLATION_ONLY;
        }
        if (o) {
            return DriveOverlayMask.OMEGA_ONLY;
        }
        return DriveOverlayMask.NONE;
    }

    public DriveOverlayMask suggestedMask() {
        return requestedMask();
    }
}
