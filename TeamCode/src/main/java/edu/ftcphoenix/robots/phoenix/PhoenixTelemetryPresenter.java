package edu.ftcphoenix.robots.phoenix;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.task.Task;

/**
 * Driver-facing telemetry formatting for Phoenix TeleOp and Auto.
 */
public final class PhoenixTelemetryPresenter {

    private final Telemetry telemetry;
    private final PhoenixProfile profile;

    /**
     * Creates a telemetry presenter bound to one telemetry sink and one Phoenix profile snapshot.
     *
     * @param telemetry FTC telemetry sink to write into; when {@code null}, emission becomes a no-op
     * @param profile   profile snapshot used to label values such as estimator mode
     */
    public PhoenixTelemetryPresenter(Telemetry telemetry, PhoenixProfile profile) {
        this.telemetry = telemetry;
        this.profile = profile.copy();
    }

    /**
     * Emits the standard Phoenix TeleOp telemetry block.
     */
    public void emitTeleOp(ScoringPath.Status scoring,
                           ScoringTargeting.Status targeting,
                           PhoenixDriveAssistService.Status driveAssist,
                           PoseEstimate globalPose,
                           PoseEstimate odomPose) {
        if (telemetry == null) {
            return;
        }

        emitScoringTelemetry(scoring, "scoring");
        emitScoringIntentTelemetry(scoring);
        emitAimSummary(targeting);
        emitDriveAssistTelemetry(driveAssist);
        emitPoseTelemetry(globalPose, odomPose);
        emitTargetTelemetry(targeting);
        telemetry.update();
    }

    /**
     * Emits the standard Phoenix Auto telemetry block.
     */
    public void emitAuto(ScoringPath.Status scoring,
                         ScoringTargeting.Status targeting,
                         Task currentAutoTask,
                         int queuedAutoTasks,
                         PoseEstimate globalPose,
                         PoseEstimate odomPose) {
        if (telemetry == null) {
            return;
        }

        telemetry.addData("auto.currentTask", currentAutoTask != null ? currentAutoTask.getDebugName() : "<idle>");
        telemetry.addData("auto.currentOutcome", currentAutoTask != null ? currentAutoTask.getOutcome() : "IDLE");
        telemetry.addData("auto.queued", queuedAutoTasks);

        emitScoringTelemetry(scoring, "scoring");
        emitScoringIntentTelemetry(scoring);
        emitAimSummary(targeting);
        emitPoseTelemetry(globalPose, odomPose);
        emitTargetTelemetry(targeting);
        telemetry.update();
    }

    private void emitScoringTelemetry(ScoringPath.Status scoring, String prefix) {
        if (scoring == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "scoring" : prefix;
        telemetry.addData(p + ".flywheelEnabled", scoring.flywheelEnabled);
        telemetry.addData(p + ".pidfEnabled", scoring.pidfEnabled);
        if (scoring.pidfWarning != null && !scoring.pidfWarning.isEmpty()) {
            telemetry.addData(p + ".pidfWarning", scoring.pidfWarning);
        }
        telemetry.addData(p + ".selectedVel", scoring.selectedVelocityNative);
        telemetry.addData(p + ".flywheelTarget", scoring.flywheelTargetNative);
        telemetry.addData(p + ".flywheelMeasured", scoring.flywheelMeasuredNative);
        telemetry.addData(p + ".flywheelErr", scoring.flywheelErrorNative);
        telemetry.addData(p + ".flywheelErrAbs", scoring.flywheelErrorAbsNative);
        telemetry.addData(p + ".flywheelTol", scoring.flywheelToleranceNative);
        telemetry.addData(p + ".flywheelTolBelow", scoring.flywheelToleranceBelowNative);
        telemetry.addData(p + ".flywheelTolAbove", scoring.flywheelToleranceAboveNative);
        telemetry.addData(p + ".flywheelAccel", scoring.flywheelAccelNativePerSec);
        telemetry.addData(p + ".flywheelAccelAbs", scoring.flywheelAccelAbsNativePerSec);
        telemetry.addData(p + ".readyLeadSec", scoring.readyLeadSec);
        telemetry.addData(p + ".flywheelPredAbs", scoring.predictedFlywheelAbsNative);
        telemetry.addData(p + ".flywheelPredErr", scoring.predictedFlywheelErrorNative);
        telemetry.addData(p + ".flywheelAtTarget", scoring.flywheelAtTarget);
        telemetry.addData(p + ".ready", scoring.ready);
        telemetry.addData(p + ".feedBacklog", scoring.feedBacklog);
        telemetry.addData(p + ".feedQueued", scoring.feedQueued);
        telemetry.addData(p + ".feedActive", scoring.feedActive);
        telemetry.addData(p + ".feedOut", scoring.feedOutput);
    }

    private void emitScoringIntentTelemetry(ScoringPath.Status scoring) {
        if (scoring == null) {
            return;
        }
        telemetry.addData("shoot.mode", scoring.mode);
        telemetry.addData("feed.backlog", scoring.feedBacklog);
        telemetry.addData("intake.enabled", scoring.intakeEnabled);
        telemetry.addData("eject.requested", scoring.ejectRequested);
        telemetry.addData("flywheel.requested", scoring.flywheelRequested);
        telemetry.addData("shoot.requested", scoring.shootingRequested);
        telemetry.addData("shoot.active", scoring.shootActive);
    }

    private void emitAimSummary(ScoringTargeting.Status targeting) {
        if (targeting == null) {
            return;
        }
        telemetry.addData("aim.ready", targeting.aimReady);
        telemetry.addData("aim.okToShoot", targeting.aimOkToShoot);
        telemetry.addData("aim.override", targeting.aimOverride);
        telemetry.addData("aim.enabled", targeting.autoAimEnabled);
    }

    private void emitDriveAssistTelemetry(PhoenixDriveAssistService.Status driveAssist) {
        if (driveAssist == null) {
            return;
        }
        telemetry.addData("drive.autoAimRequested", driveAssist.autoAimRequested);
        telemetry.addData("drive.shootBraceEligible", driveAssist.shootBraceEligible);
        telemetry.addData("drive.shootBraceEnabled", driveAssist.shootBraceEnabled);
        telemetry.addData("drive.manualTranslateMag", driveAssist.manualTranslateMagnitude);
    }

    private void emitPoseTelemetry(PoseEstimate globalPose, PoseEstimate odomPose) {
        if (globalPose != null) {
            telemetry.addData("pose.global", globalPose);
            telemetry.addData("pose.global.mode", profile.localization.correctedEstimatorMode);
            telemetry.addData("pose.global.correctionSource", profile.localization.correctionSource.mode);
        }
        if (odomPose != null) {
            telemetry.addData("pose.odom", odomPose);
        }
    }

    private void emitTargetTelemetry(ScoringTargeting.Status targeting) {
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
