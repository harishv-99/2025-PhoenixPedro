package edu.ftcphoenix.fw.drive.guidance;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveSignal;

/**
 * Read-only snapshot of the most recent DriveGuidance evaluation.
 *
 * <p>This is intentionally <b>controller-neutral</b>: it exposes the computed guidance command
 * (if any) and the associated errors, so other code can:</p>
 * <ul>
 *   <li>gate mechanisms (“only shoot when aimed”)</li>
 *   <li>report driver telemetry (“omega error = ...”)</li>
 *   <li>write tasks that share the same math as overlays</li>
 * </ul>
 *
 * <p>It is produced by {@link DriveGuidanceQuery}, and internally by the same engine used by
 * {@link DriveGuidancePlan#overlay()} and {@link DriveGuidancePlan#task(...)}.</p>
 */
public final class DriveGuidanceStatus {

    /**
     * The guidance mode used on the last evaluation ("obs", "field", "adaptive").
     */
    public final String mode;

    /**
     * Mask produced for the last output. If this is {@link DriveOverlayMask#NONE}, the guidance
     * system could not compute a safe command on that loop.
     */
    public final DriveOverlayMask mask;

    /**
     * Guidance command (robot-centric). Meaningful only for the DOFs claimed by {@link #mask}.
     */
    public final DriveSignal signal;

    /**
     * True if translation error was computed.
     */
    public final boolean hasTranslationError;

    /**
     * Translation error in the translation control frame (+X forward, +Y left), inches.
     */
    public final double forwardErrorIn;
    public final double leftErrorIn;

    /**
     * True if omega (heading/bearing) error was computed.
     */
    public final boolean hasOmegaError;

    /**
     * Signed omega error in radians (wrapped to [-pi, +pi]).
     */
    public final double omegaErrorRad;

    /**
     * Whether the observation was considered "in range" for translation takeover.
     */
    public final boolean obsInRangeForTranslation;

    /**
     * Blend factors (0..1) used by adaptive guidance (for debug/telemetry).
     */
    public final double blendTTranslate;
    public final double blendTOmega;

    /**
     * Last observed tag id (or -1 if none).
     */
    public final int lastObservedTagId;

    /**
     * Anchor pose (field frame) captured for {@link DriveGuidanceSpec.RobotRelativePoint} targets.
     * May be null if not applicable.
     */
    public final Pose2d fieldToTranslationFrameAnchor;

    DriveGuidanceStatus(String mode,
                        DriveOverlayMask mask,
                        DriveSignal signal,
                        boolean hasTranslationError,
                        double forwardErrorIn,
                        double leftErrorIn,
                        boolean hasOmegaError,
                        double omegaErrorRad,
                        boolean obsInRangeForTranslation,
                        double blendTTranslate,
                        double blendTOmega,
                        int lastObservedTagId,
                        Pose2d fieldToTranslationFrameAnchor) {
        this.mode = mode;
        this.mask = mask;
        this.signal = signal;
        this.hasTranslationError = hasTranslationError;
        this.forwardErrorIn = forwardErrorIn;
        this.leftErrorIn = leftErrorIn;
        this.hasOmegaError = hasOmegaError;
        this.omegaErrorRad = omegaErrorRad;
        this.obsInRangeForTranslation = obsInRangeForTranslation;
        this.blendTTranslate = blendTTranslate;
        this.blendTOmega = blendTOmega;
        this.lastObservedTagId = lastObservedTagId;
        this.fieldToTranslationFrameAnchor = fieldToTranslationFrameAnchor;
    }

    /**
     * @return magnitude of translation error (inches) or NaN if unavailable
     */
    public double translationErrorMagInches() {
        return hasTranslationError ? Math.hypot(forwardErrorIn, leftErrorIn) : Double.NaN;
    }

    /**
     * @return true if translation error magnitude is available and <= tolInches
     */
    public boolean translationWithin(double tolInches) {
        double mag = translationErrorMagInches();
        return Double.isFinite(mag) && mag <= tolInches;
    }

    /**
     * @return true if omega error is available and |error| <= tolRad
     */
    public boolean omegaWithin(double tolRad) {
        return hasOmegaError && Double.isFinite(omegaErrorRad) && Math.abs(omegaErrorRad) <= tolRad;
    }

    static DriveGuidanceStatus fromCore(DriveGuidanceCore core, DriveGuidanceCore.Step step) {
        String mode = (core != null && core.lastMode() != null) ? core.lastMode() : "unknown";
        DriveOverlayMask mask = (step != null && step.out != null) ? step.out.mask : DriveOverlayMask.NONE;
        DriveSignal signal = (step != null && step.out != null) ? step.out.signal : DriveSignal.zero();

        int lastTag = (core != null) ? core.lastObservedTagId() : -1;
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
                (step != null) && step.obsInRangeForTranslation,
                (step != null) ? step.blendTTranslate : Double.NaN,
                (step != null) ? step.blendTOmega : Double.NaN,
                lastTag,
                anchor
        );
    }
}
