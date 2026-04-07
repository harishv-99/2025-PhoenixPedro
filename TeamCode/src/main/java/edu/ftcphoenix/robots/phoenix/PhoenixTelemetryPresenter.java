package edu.ftcphoenix.robots.phoenix;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;

/**
 * Driver-facing telemetry formatting for Phoenix TeleOp.
 */
public final class PhoenixTelemetryPresenter {

    private final Telemetry telemetry;
    private final PhoenixProfile profile;

    /**
     * Creates a telemetry presenter bound to one telemetry sink and one Phoenix profile snapshot.
     *
     * @param telemetry FTC telemetry sink to write into; when {@code null}, emission becomes a no-op
     * @param profile   profile snapshot used to label values such as estimator mode and aim offsets
     */
    public PhoenixTelemetryPresenter(Telemetry telemetry, PhoenixProfile profile) {
        this.telemetry = telemetry;
        this.profile = profile.copy();
    }

    /**
     * Emits the standard Phoenix TeleOp telemetry block.
     *
     * <p>This presenter intentionally consumes precomputed snapshots instead of reaching back into
     * live subsystems. That keeps loop ownership explicit and makes the output easy to audit during
     * refactors.</p>
     *
     * @param shooter           shooter subsystem status snapshot for the current loop
     * @param scoring           scoring supervisor status snapshot for the current loop
     * @param aimReady          whether the current aim gate is satisfied
     * @param aimOkToShoot      whether aiming policy currently allows feeding a shot
     * @param aimOverride       whether the operator is overriding aim gating
     * @param shootBraceEnabled whether the translation brace overlay is latched on
     * @param globalPose        current fused/global pose estimate, or {@code null} if unavailable
     * @param odomPose          current odometry-only pose estimate, or {@code null} if unavailable
     * @param selection         currently selected scoring-tag result, or {@code null} if no selection exists
     * @param aimStatus         current guidance status for the aiming overlay, or {@code null} if inactive
     * @param gameTagLayout     fixed field tag layout used to derive field-frame target information
     */
    public void emitTeleOp(ShooterStatus shooter,
                           ScoringStatus scoring,
                           boolean aimReady,
                           boolean aimOkToShoot,
                           boolean aimOverride,
                           boolean shootBraceEnabled,
                           PoseEstimate globalPose,
                           PoseEstimate odomPose,
                           TagSelectionResult selection,
                           DriveGuidanceStatus aimStatus,
                           TagLayout gameTagLayout) {
        if (telemetry == null) {
            return;
        }

        emitShooterTelemetry(shooter, "shooter");
        telemetry.addData("shoot.mode", scoring.mode);
        telemetry.addData("feed.backlog", scoring.feedBacklog);
        telemetry.addData("intake.enabled", scoring.intakeEnabled);
        telemetry.addData("eject.requested", scoring.ejectRequested);
        telemetry.addData("flywheel.requested", scoring.flywheelRequested);
        telemetry.addData("shoot.requested", scoring.shootingRequested);
        telemetry.addData("shoot.active", scoring.shootActive);
        telemetry.addData("aim.ready", aimReady);
        telemetry.addData("aim.okToShoot", aimOkToShoot);
        telemetry.addData("aim.override", aimOverride);
        telemetry.addData("shootBrace", shootBraceEnabled);

        if (globalPose != null) {
            telemetry.addData("pose.global", globalPose);
            telemetry.addData("pose.global.mode", profile.localization.globalEstimatorMode);
        }
        if (odomPose != null) {
            telemetry.addData("pose.odom", odomPose);
        }

        if (selection != null && selection.hasSelection) {
            AprilTagObservation obs = selection.hasFreshSelectedObservation
                    ? selection.selectedObservation
                    : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
            emitTargetTelemetry(selection.selectedTagId, obs, selection.hasFreshSelectedObservation, aimStatus, gameTagLayout, aimReady, aimOverride);
        }

        telemetry.update();
    }

    private void emitShooterTelemetry(ShooterStatus shooter, String prefix) {
        String p = (prefix == null || prefix.isEmpty()) ? "shooter" : prefix;
        telemetry.addData(p + ".flywheelEnabled", shooter.flywheelEnabled);
        telemetry.addData(p + ".pidfEnabled", shooter.pidfEnabled);
        if (shooter.pidfWarning != null && !shooter.pidfWarning.isEmpty()) {
            telemetry.addData(p + ".pidfWarning", shooter.pidfWarning);
        }
        telemetry.addData(p + ".selectedVel", shooter.selectedVelocityNative);
        telemetry.addData(p + ".flywheelTarget", shooter.flywheelTargetNative);
        telemetry.addData(p + ".flywheelMeasured", shooter.flywheelMeasuredNative);
        telemetry.addData(p + ".flywheelErr", shooter.flywheelErrorNative);
        telemetry.addData(p + ".flywheelErrAbs", shooter.flywheelErrorAbsNative);
        telemetry.addData(p + ".flywheelTol", shooter.flywheelToleranceNative);
        telemetry.addData(p + ".flywheelTolBelow", shooter.flywheelToleranceBelowNative);
        telemetry.addData(p + ".flywheelTolAbove", shooter.flywheelToleranceAboveNative);
        telemetry.addData(p + ".flywheelAccel", shooter.flywheelAccelNativePerSec);
        telemetry.addData(p + ".flywheelAccelAbs", shooter.flywheelAccelAbsNativePerSec);
        telemetry.addData(p + ".readyLeadSec", shooter.readyLeadSec);
        telemetry.addData(p + ".flywheelPredAbs", shooter.predictedFlywheelAbsNative);
        telemetry.addData(p + ".flywheelPredErr", shooter.predictedFlywheelErrorNative);
        telemetry.addData(p + ".flywheelAtSetpoint", shooter.flywheelAtSetpoint);
        telemetry.addData(p + ".ready", shooter.ready);
        telemetry.addData(p + ".feedBacklog", shooter.feedBacklog);
        telemetry.addData(p + ".feedQueued", shooter.feedQueued);
        telemetry.addData(p + ".feedActive", shooter.feedActive);
        telemetry.addData(p + ".feedOut", shooter.feedOutput);
    }

    private void emitTargetTelemetry(int tagId,
                                     AprilTagObservation obs,
                                     boolean tagVisible,
                                     DriveGuidanceStatus aimStatus,
                                     TagLayout gameTagLayout,
                                     boolean aimReady,
                                     boolean aimOverride) {
        PhoenixProfile.AutoAimConfig.AimOffset aimOffset = profile.autoAim.aimOffsetForTag(tagId);

        if (aimReady) {
            telemetry.addLine(">>> AIM READY <<<");
        } else if (aimOverride) {
            telemetry.addLine(">>> AIM OVERRIDE <<<");
        }

        telemetry.addData("tagId", tagId);
        telemetry.addData("tag.visible", tagVisible);
        telemetry.addData("distIn", tagVisible ? obs.cameraRangeInches() : Double.NaN);
        telemetry.addData("bearingTagDeg", tagVisible ? Math.toDegrees(obs.cameraBearingRad()) : Double.NaN);
        telemetry.addData(
                "aimOffset(fwd,left)",
                String.format("%.1f, %.1f", aimOffset.forwardInches, aimOffset.leftInches)
        );
        telemetry.addData(
                "omegaErrDeg",
                (aimStatus != null && aimStatus.hasOmegaError)
                        ? Math.toDegrees(aimStatus.omegaErrorRad)
                        : Double.NaN
        );
        telemetry.addData(
                "aim.source",
                (aimStatus != null && aimStatus.hasOmegaError)
                        ? aimStatus.omegaSource
                        : DriveGuidanceStatus.ChannelSource.NONE
        );
        telemetry.addData("aim.tolDeg", profile.autoAim.aimToleranceDeg);
        telemetry.addData("aim.readyTolDeg", profile.autoAim.aimReadyToleranceDeg);

        if (gameTagLayout != null && gameTagLayout.has(tagId)) {
            Pose3d fieldToTag = gameTagLayout.requireFieldToTagPose(tagId);
            Pose2d fieldToAimPoint = new Pose2d(
                    fieldToTag.xInches,
                    fieldToTag.yInches,
                    fieldToTag.yawRad
            ).then(new Pose2d(aimOffset.forwardInches, aimOffset.leftInches, 0.0));

            telemetry.addData("field.tag", fieldToTag);
            telemetry.addData("field.aim", fieldToAimPoint);
        }
    }
}
