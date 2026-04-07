package edu.ftcphoenix.robots.phoenix;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;

/**
 * Immutable status snapshot for Phoenix scoring-target selection and auto-aim.
 *
 * <p>The targeting layer computes this snapshot once per loop so supervisors, telemetry, and other
 * clients can all observe the same selected tag, aim state, and suggested shot velocity.</p>
 */
public final class TargetingStatus {
    public final boolean autoAimEnabled;
    public final boolean aimReady;
    public final boolean aimOkToShoot;
    public final boolean aimOverride;
    public final double aimToleranceDeg;
    public final double aimReadyToleranceDeg;
    public final TagSelectionResult selection;
    public final DriveGuidanceStatus aimStatus;
    public final String targetLabel;
    public final double aimOffsetForwardInches;
    public final double aimOffsetLeftInches;
    public final boolean hasSuggestedVelocity;
    public final double suggestedVelocityNative;
    public final Pose3d fieldToSelectedTag;
    public final Pose2d fieldToAimPoint;

    /**
     * Creates an immutable targeting snapshot.
     *
     * @param autoAimEnabled          whether the driver's auto-aim enable is currently active
     * @param aimReady                whether the current aim solution is within the configured ready tolerance
     * @param aimOkToShoot            whether current targeting policy allows feeding a shot
     * @param aimOverride             whether the driver is overriding targeting gates this loop
     * @param aimToleranceDeg         configured guidance deadband for the aim plan, in degrees
     * @param aimReadyToleranceDeg    configured looser tolerance used by the shoot gate, in degrees
     * @param selection               selected-tag snapshot for the current loop
     * @param aimStatus               current drive-guidance query status for omega-only auto-aim
     * @param targetLabel             human-readable label for the selected target family
     * @param aimOffsetForwardInches  tag-local forward aim offset in inches
     * @param aimOffsetLeftInches     tag-local left aim offset in inches
     * @param hasSuggestedVelocity    whether a fresh target observation produced a valid shot suggestion
     * @param suggestedVelocityNative suggested flywheel velocity in native units
     * @param fieldToSelectedTag      fixed field pose of the selected tag, or {@code null} if unavailable
     * @param fieldToAimPoint         fixed field pose of the selected target aim point, or {@code null} if unavailable
     */
    public TargetingStatus(boolean autoAimEnabled,
                           boolean aimReady,
                           boolean aimOkToShoot,
                           boolean aimOverride,
                           double aimToleranceDeg,
                           double aimReadyToleranceDeg,
                           TagSelectionResult selection,
                           DriveGuidanceStatus aimStatus,
                           String targetLabel,
                           double aimOffsetForwardInches,
                           double aimOffsetLeftInches,
                           boolean hasSuggestedVelocity,
                           double suggestedVelocityNative,
                           Pose3d fieldToSelectedTag,
                           Pose2d fieldToAimPoint) {
        this.autoAimEnabled = autoAimEnabled;
        this.aimReady = aimReady;
        this.aimOkToShoot = aimOkToShoot;
        this.aimOverride = aimOverride;
        this.aimToleranceDeg = aimToleranceDeg;
        this.aimReadyToleranceDeg = aimReadyToleranceDeg;
        this.selection = selection;
        this.aimStatus = aimStatus;
        this.targetLabel = targetLabel != null ? targetLabel : "";
        this.aimOffsetForwardInches = aimOffsetForwardInches;
        this.aimOffsetLeftInches = aimOffsetLeftInches;
        this.hasSuggestedVelocity = hasSuggestedVelocity;
        this.suggestedVelocityNative = suggestedVelocityNative;
        this.fieldToSelectedTag = fieldToSelectedTag;
        this.fieldToAimPoint = fieldToAimPoint;
    }
}
