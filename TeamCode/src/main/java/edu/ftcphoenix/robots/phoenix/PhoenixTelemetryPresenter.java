package edu.ftcphoenix.robots.phoenix;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceStatus;
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
     * @param profile profile snapshot used to label values such as estimator mode
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
     * @param shooter shooter subsystem status snapshot for the current loop
     * @param scoring scoring supervisor status snapshot for the current loop
     * @param targeting scoring-targeting status snapshot for the current loop
     * @param driveAssist drive-assist status snapshot for the current loop
     * @param globalPose current fused/global pose estimate, or {@code null} if unavailable
     * @param odomPose current odometry-only pose estimate, or {@code null} if unavailable
     */
    public void emitTeleOp(ShooterStatus shooter,
                           ScoringStatus scoring,
                           TargetingStatus targeting,
                           DriveAssistStatus driveAssist,
                           PoseEstimate globalPose,
                           PoseEstimate odomPose) {
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
        telemetry.addData("aim.ready", targeting.aimReady);
        telemetry.addData("aim.okToShoot", targeting.aimOkToShoot);
        telemetry.addData("aim.override", targeting.aimOverride);
        telemetry.addData("aim.enabled", targeting.autoAimEnabled);
        emitDriveAssistTelemetry(driveAssist);

        if (globalPose != null) {
            telemetry.addData("pose.global", globalPose);
            telemetry.addData("pose.global.mode", profile.localization.globalEstimatorMode);
        }
        if (odomPose != null) {
            telemetry.addData("pose.odom", odomPose);
        }

        emitTargetTelemetry(targeting);
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


    private void emitDriveAssistTelemetry(DriveAssistStatus driveAssist) {
        if (driveAssist == null) {
            return;
        }
        telemetry.addData("drive.autoAimRequested", driveAssist.autoAimRequested);
        telemetry.addData("drive.shootBraceEligible", driveAssist.shootBraceEligible);
        telemetry.addData("drive.shootBraceEnabled", driveAssist.shootBraceEnabled);
        telemetry.addData("drive.manualTranslateMag", driveAssist.manualTranslateMagnitude);
    }

    private void emitTargetTelemetry(TargetingStatus targeting) {
        if (targeting == null) {
            return;
        }

        if (targeting.aimReady) {
            telemetry.addLine(">>> AIM READY <<<");
        } else if (targeting.aimOverride) {
            telemetry.addLine(">>> AIM OVERRIDE <<<");
        }

        TagSelectionResult selection = targeting.selection;
        if (selection == null || !selection.hasSelection) {
            return;
        }

        AprilTagObservation obs = selection.hasFreshSelectedObservation
                ? selection.selectedObservation
                : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
        DriveGuidanceStatus aimStatus = targeting.aimStatus;

        telemetry.addData("tagId", selection.selectedTagId);
        telemetry.addData("target.label", targeting.targetLabel);
        telemetry.addData("tag.visible", selection.hasFreshSelectedObservation);
        telemetry.addData("distIn", selection.hasFreshSelectedObservation ? obs.cameraRangeInches() : Double.NaN);
        telemetry.addData("bearingTagDeg", selection.hasFreshSelectedObservation ? Math.toDegrees(obs.cameraBearingRad()) : Double.NaN);
        telemetry.addData(
                "aimOffset(fwd,left)",
                String.format("%.1f, %.1f", targeting.aimOffsetForwardInches, targeting.aimOffsetLeftInches)
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
        telemetry.addData("aim.tolDeg", targeting.aimToleranceDeg);
        telemetry.addData("aim.readyTolDeg", targeting.aimReadyToleranceDeg);
        telemetry.addData("aim.suggestedVel", targeting.hasSuggestedVelocity ? targeting.suggestedVelocityNative : Double.NaN);

        if (targeting.fieldToSelectedTag != null) {
            telemetry.addData("field.tag", targeting.fieldToSelectedTag);
        }
        if (targeting.fieldToAimPoint != null) {
            telemetry.addData("field.aim", targeting.fieldToAimPoint);
        }
    }
}
