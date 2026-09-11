package edu.ftcsushi.fw.drive.guidance;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionResult;

/**
 * Immutable evidence and command snapshot for one explicit guidance mode.
 *
 * <p>The mode identifies the configured authority, not success. Error-presence flags identify
 * solved requested channels; a ZERO_OUTPUT fallback can own a mask without supplying any error
 * evidence. Selection snapshots retain target-identity provenance independently of solved geometry.</p>
 */
public final class DriveGuidanceStatus {
    public final DriveGuidanceSpec.SolveMode solveMode;
    public final DriveOverlayMask mask;
    public final DriveSignal signal;
    public final boolean hasTranslationError;
    public final double forwardErrorIn;
    public final double leftErrorIn;
    public final boolean hasOmegaError;
    public final double omegaErrorRad;
    public final TagSelectionResult translationSelection;
    public final TagSelectionResult facingSelection;
    public final Pose2d fieldToTranslationFrameAnchor;

    private DriveGuidanceStatus(DriveGuidanceCore core, DriveGuidanceCore.Step step) {
        solveMode = core.solveMode();
        mask = step.out.mask;
        signal = step.out.signal;
        hasTranslationError = step.hasTranslationError;
        forwardErrorIn = step.forwardErrorIn;
        leftErrorIn = step.leftErrorIn;
        hasOmegaError = step.hasOmegaError;
        omegaErrorRad = step.omegaErrorRad;
        translationSelection = step.translationSelection;
        facingSelection = step.facingSelection;
        fieldToTranslationFrameAnchor = core.fieldToTranslationFrameAnchor();
    }

    /** Planar translation-error magnitude in inches, or NaN when unavailable. */
    public double translationErrorMagInches() {
        return hasTranslationError ? Math.hypot(forwardErrorIn, leftErrorIn) : Double.NaN;
    }

    /** Whether available finite translation evidence is within the supplied inch tolerance. */
    public boolean translationWithin(double tolInches) {
        double magnitude = translationErrorMagInches();
        return Double.isFinite(magnitude) && magnitude <= tolInches;
    }

    /** Whether available finite facing evidence is within the supplied radian tolerance. */
    public boolean omegaWithin(double tolRad) {
        return hasOmegaError && Double.isFinite(omegaErrorRad) && Math.abs(omegaErrorRad) <= tolRad;
    }

    /** Adapts an already-completed evaluation without another source read. */
    static DriveGuidanceStatus fromCore(DriveGuidanceCore core, DriveGuidanceCore.Step step) {
        return new DriveGuidanceStatus(core, step);
    }
}
