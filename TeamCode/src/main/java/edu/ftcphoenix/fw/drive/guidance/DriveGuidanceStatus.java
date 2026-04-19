package edu.ftcphoenix.fw.drive.guidance;

import java.util.Collections;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;

/**
 * Read-only snapshot of the most recent {@link DriveGuidance} evaluation.
 *
 * <p>{@link DriveGuidanceStatus} is shared by overlays, tasks, and queries so teams can inspect
 * exactly the same solved errors and commands regardless of how the plan is being used.</p>
 */
public final class DriveGuidanceStatus {

    public enum ChannelSource {
        NONE,
        LOCALIZATION,
        APRIL_TAGS,
        BLENDED
    }

    public final String mode;
    public final DriveOverlayMask mask;
    public final DriveSignal signal;

    public final boolean hasTranslationError;
    public final double forwardErrorIn;
    public final double leftErrorIn;

    public final boolean hasOmegaError;
    public final double omegaErrorRad;

    public final ChannelSource translationSource;
    public final ChannelSource omegaSource;

    public final TagSelectionResult translationSelection;
    public final TagSelectionResult facingSelection;

    public final boolean aprilTagsInRangeForTranslation;
    public final double blendTTranslate;
    public final double blendTOmega;
    public final Pose2d fieldToTranslationFrameAnchor;

    DriveGuidanceStatus(String mode,
                        DriveOverlayMask mask,
                        DriveSignal signal,
                        boolean hasTranslationError,
                        double forwardErrorIn,
                        double leftErrorIn,
                        boolean hasOmegaError,
                        double omegaErrorRad,
                        ChannelSource translationSource,
                        ChannelSource omegaSource,
                        TagSelectionResult translationSelection,
                        TagSelectionResult facingSelection,
                        boolean aprilTagsInRangeForTranslation,
                        double blendTTranslate,
                        double blendTOmega,
                        Pose2d fieldToTranslationFrameAnchor) {
        this.mode = mode;
        this.mask = mask;
        this.signal = signal;
        this.hasTranslationError = hasTranslationError;
        this.forwardErrorIn = forwardErrorIn;
        this.leftErrorIn = leftErrorIn;
        this.hasOmegaError = hasOmegaError;
        this.omegaErrorRad = omegaErrorRad;
        this.translationSource = translationSource;
        this.omegaSource = omegaSource;
        this.translationSelection = translationSelection != null
                ? translationSelection
                : TagSelectionResult.none(Collections.<Integer>emptySet());
        this.facingSelection = facingSelection != null
                ? facingSelection
                : TagSelectionResult.none(Collections.<Integer>emptySet());
        this.aprilTagsInRangeForTranslation = aprilTagsInRangeForTranslation;
        this.blendTTranslate = blendTTranslate;
        this.blendTOmega = blendTOmega;
        this.fieldToTranslationFrameAnchor = fieldToTranslationFrameAnchor;
    }

    /**
     * Returns the planar magnitude of the current translation error in inches, or {@link Double#NaN}
     * when translation was not solved this loop.
     */
    public double translationErrorMagInches() {
        return hasTranslationError ? Math.hypot(forwardErrorIn, leftErrorIn) : Double.NaN;
    }

    /**
     * Returns {@code true} when the current translation error magnitude is within the supplied
     * tolerance.
     */
    public boolean translationWithin(double tolInches) {
        double mag = translationErrorMagInches();
        return Double.isFinite(mag) && mag <= tolInches;
    }

    /**
     * Returns {@code true} when the current omega error magnitude is within the supplied tolerance.
     */
    public boolean omegaWithin(double tolRad) {
        return hasOmegaError && Double.isFinite(omegaErrorRad) && Math.abs(omegaErrorRad) <= tolRad;
    }

    static DriveGuidanceStatus fromCore(DriveGuidanceCore core, DriveGuidanceCore.Step step) {
        String mode = (core != null && core.lastMode() != null) ? core.lastMode() : "unknown";
        DriveOverlayMask mask = (step != null && step.out != null) ? step.out.mask : DriveOverlayMask.NONE;
        DriveSignal signal = (step != null && step.out != null) ? step.out.signal : DriveSignal.zero();

        Pose2d anchor = (core != null) ? core.fieldToTranslationFrameAnchor() : null;

        boolean hasT = (step != null) && step.hasTranslationError;
        boolean hasO = (step != null) && step.hasOmegaError;

        return new DriveGuidanceStatus(
                mode,
                mask,
                signal,
                hasT,
                hasT ? step.forwardErrorIn : Double.NaN,
                hasT ? step.leftErrorIn : Double.NaN,
                hasO,
                hasO ? step.omegaErrorRad : Double.NaN,
                translationSource(step),
                omegaSource(step),
                step != null ? step.translationSelection : null,
                step != null ? step.facingSelection : null,
                (step != null) && step.aprilTagsInRangeForTranslation,
                (step != null) ? step.blendTTranslate : Double.NaN,
                (step != null) ? step.blendTOmega : Double.NaN,
                anchor
        );
    }

    private static ChannelSource translationSource(DriveGuidanceCore.Step step) {
        if (step == null || !step.hasTranslationError) {
            return ChannelSource.NONE;
        }
        boolean hasLocalization = step.localization != null && step.localization.valid && step.localization.canTranslate;
        boolean hasAprilTags = step.aprilTags != null && step.aprilTags.valid && step.aprilTags.canTranslate;
        if (hasLocalization && hasAprilTags) {
            if (step.blendTTranslate <= 0.0) {
                return ChannelSource.LOCALIZATION;
            }
            if (step.blendTTranslate >= 1.0) {
                return ChannelSource.APRIL_TAGS;
            }
            return ChannelSource.BLENDED;
        }
        if (hasAprilTags) {
            return ChannelSource.APRIL_TAGS;
        }
        return hasLocalization ? ChannelSource.LOCALIZATION : ChannelSource.NONE;
    }

    private static ChannelSource omegaSource(DriveGuidanceCore.Step step) {
        if (step == null || !step.hasOmegaError) {
            return ChannelSource.NONE;
        }
        boolean hasLocalization = step.localization != null && step.localization.valid && step.localization.canOmega;
        boolean hasAprilTags = step.aprilTags != null && step.aprilTags.valid && step.aprilTags.canOmega;
        if (hasLocalization && hasAprilTags) {
            if (step.blendTOmega <= 0.0) {
                return ChannelSource.LOCALIZATION;
            }
            if (step.blendTOmega >= 1.0) {
                return ChannelSource.APRIL_TAGS;
            }
            return ChannelSource.BLENDED;
        }
        if (hasAprilTags) {
            return ChannelSource.APRIL_TAGS;
        }
        return hasLocalization ? ChannelSource.LOCALIZATION : ChannelSource.NONE;
    }
}
