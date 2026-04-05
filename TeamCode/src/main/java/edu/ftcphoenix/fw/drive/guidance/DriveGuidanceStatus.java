package edu.ftcphoenix.fw.drive.guidance;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveSignal;

/**
 * Read-only snapshot of the most recent {@link DriveGuidance} evaluation.
 *
 * <p>{@link DriveGuidanceStatus} is shared by overlays, tasks, and queries so teams can inspect
 * exactly the same solved errors and commands regardless of how the plan is being used.</p>
 */
public final class DriveGuidanceStatus {

    /**
     * Human-readable mode string used for debugging. Examples: {@code "fieldPose"},
     * {@code "aprilTags"}, or {@code "adaptive"}.
     */
    public final String mode;

    /**
     * Which drive channels guidance actively overrode this step.
     */
    public final DriveOverlayMask mask;

    /**
     * Final command produced by guidance for the requested channels.
     */
    public final DriveSignal signal;

    /**
     * Whether a translation error was successfully solved this step.
     */
    public final boolean hasTranslationError;

    /**
     * Forward translation error in the translation control frame, inches.
     */
    public final double forwardErrorIn;

    /**
     * Left translation error in the translation control frame, inches.
     */
    public final double leftErrorIn;

    /**
     * Whether an aim / omega error was successfully solved this step.
     */
    public final boolean hasOmegaError;

    /**
     * Omega error in radians. Positive means rotate CCW.
     */
    public final double omegaErrorRad;

    /**
     * Whether the live AprilTag path was considered “in range” for translation takeover.
     *
     * <p>This only matters when adaptive feedback is configured.</p>
     */
    public final boolean aprilTagsInRangeForTranslation;

    /**
     * Blend factor used for translation between field-pose and AprilTag commands.
     *
     * <p>{@code 0} means pure field-pose command. {@code 1} means pure AprilTag command.</p>
     */
    public final double blendTTranslate;

    /**
     * Blend factor used for omega between field-pose and AprilTag commands.
     */
    public final double blendTOmega;

    /**
     * Anchor pose (field frame) captured for {@link DriveGuidanceSpec.RobotRelativePoint} targets.
     *
     * <p>This is {@code null} unless the plan uses a robot-relative translation target.</p>
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
        this.aprilTagsInRangeForTranslation = aprilTagsInRangeForTranslation;
        this.blendTTranslate = blendTTranslate;
        this.blendTOmega = blendTOmega;
        this.fieldToTranslationFrameAnchor = fieldToTranslationFrameAnchor;
    }

    /**
     * Computes the planar magnitude of the translation error.
     *
     * @return Euclidean translation error in inches, or {@link Double#NaN} if no translation error
     *         was available
     */
    public double translationErrorMagInches() {
        return hasTranslationError ? Math.hypot(forwardErrorIn, leftErrorIn) : Double.NaN;
    }

    /**
     * Convenience helper for “close enough?” checks on translation.
     *
     * @param tolInches allowed translation magnitude
     * @return whether the solved translation error magnitude is within tolerance
     */
    public boolean translationWithin(double tolInches) {
        double mag = translationErrorMagInches();
        return Double.isFinite(mag) && mag <= tolInches;
    }

    /**
     * Convenience helper for “aimed enough?” checks on omega.
     *
     * @param tolRad allowed absolute omega error
     * @return whether the solved omega error is within tolerance
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
                (step != null) && step.aprilTagsInRangeForTranslation,
                (step != null) ? step.blendTTranslate : Double.NaN,
                (step != null) ? step.blendTOmega : Double.NaN,
                anchor
        );
    }
}
