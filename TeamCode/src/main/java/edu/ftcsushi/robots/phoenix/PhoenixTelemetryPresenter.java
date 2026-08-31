package edu.ftcsushi.robots.phoenix;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import edu.ftcsushi.fw.core.math.MathUtil;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcsushi.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcsushi.fw.ftc.vision.VisionReadiness;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcsushi.fw.task.Task;

/**
 * Additive driver-facing telemetry formatting for Phoenix TeleOp and Auto.
 *
 * <p>This presenter contributes rows to the FTC telemetry sink supplied for each presentation but
 * never retains, clears, or commits that sink. The active lifecycle owner owns the complete frame
 * and calls
 * {@link Telemetry#update()} after every contributor has rendered.</p>
 */
final class PhoenixTelemetryPresenter {

    private final FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode estimatorMode;
    private final FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode correctionSourceMode;

    /**
     * Creates a presenter bound only to the two localization display facts it retains.
     *
     * @param estimatorMode configured global estimator mode
     * @param correctionSourceMode configured absolute-correction source mode
     */
    PhoenixTelemetryPresenter(
            FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode estimatorMode,
            FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode correctionSourceMode
    ) {
        this.estimatorMode = Objects.requireNonNull(estimatorMode, "estimatorMode");
        this.correctionSourceMode = Objects.requireNonNull(
                correctionSourceMode,
                "correctionSourceMode"
        );
    }

    /**
     * Adds the standard Phoenix TeleOp telemetry block to the current frame.
     *
     * <p>This method neither clears nor commits telemetry.</p>
     *
     * @param telemetry FTC telemetry sink to write into; when {@code null}, emission is a no-op
     */
    void emitTeleOp(Telemetry telemetry,
                    PhoenixCapabilities.ScoringStatus scoring,
                    PhoenixCapabilities.TargetingStatus targeting,
                    PhoenixDriveAssistService.Status driveAssist,
                    PhoenixReadiness.Result poseAssistReadiness,
                    VisionReadiness visionReadiness,
                    PoseEstimate globalPose,
                    PoseEstimate odomPose) {
        if (telemetry == null) {
            return;
        }

        emitScoringTelemetry(telemetry, scoring, "scoring");
        emitScoringIntentTelemetry(telemetry, scoring);
        emitAimSummary(telemetry, targeting);
        emitDriveAssistTelemetry(telemetry, driveAssist);
        emitTeleOpReadiness(telemetry, poseAssistReadiness);
        emitVisionReadiness(telemetry, visionReadiness);
        emitPoseTelemetry(telemetry, globalPose, odomPose);
        emitTargetTelemetry(telemetry, targeting);
    }

    /**
     * Emit the required availability status for Phoenix's localization-dependent TeleOp assists.
     *
     * <p>This additive helper neither clears nor commits telemetry. The composition root uses it
     * both on the INIT help frame and inside the ordinary TeleOp frame.</p>
     *
     * @param telemetry FTC telemetry sink to write into; when {@code null}, emission is a no-op
     * @param readiness immutable Phoenix readiness result for auto-aim and shoot-brace
     */
    void emitTeleOpReadiness(Telemetry telemetry, PhoenixReadiness.Result readiness) {
        if (telemetry == null || readiness == null) {
            return;
        }

        telemetry.addData(
                "drive.poseAssistReadiness",
                readiness.isAllowed() ? "READY" : "BLOCKED"
        );
        for (PhoenixReadiness.Issue issue : readiness.issues()) {
            telemetry.addLine(
                    "Pose assists [" + issue.severity() + "] " + issue.message()
                            + " | " + issue.remediation()
            );
        }
    }

    /**
     * Adds the standard Phoenix Auto telemetry block to the current frame.
     *
     * <p>This method neither clears nor commits telemetry.</p>
     *
     * @param telemetry FTC telemetry sink to write into; when {@code null}, emission is a no-op
     */
    void emitAuto(Telemetry telemetry,
                  PhoenixCapabilities.ScoringStatus scoring,
                  PhoenixCapabilities.TargetingStatus targeting,
                  Task installedAutoRoutine,
                  VisionReadiness visionReadiness,
                  PoseEstimate globalPose,
                  PoseEstimate odomPose) {
        if (telemetry == null) {
            return;
        }

        telemetry.addData(
                "auto.routine",
                installedAutoRoutine != null ? installedAutoRoutine.getDebugName() : "<not-installed>"
        );
        telemetry.addData(
                "auto.routineOutcome",
                installedAutoRoutine != null ? installedAutoRoutine.getOutcome() : "NOT_INSTALLED"
        );

        emitScoringTelemetry(telemetry, scoring, "scoring");
        emitScoringIntentTelemetry(telemetry, scoring);
        emitAimSummary(telemetry, targeting);
        emitVisionReadiness(telemetry, visionReadiness);
        emitPoseTelemetry(telemetry, globalPose, odomPose);
        emitTargetTelemetry(telemetry, targeting);
    }

    /** Emit camera-component readiness independently from target visibility. */
    private void emitVisionReadiness(Telemetry telemetry, VisionReadiness readiness) {
        if (readiness == null) {
            return;
        }
        telemetry.addData(
                "vision.componentReadiness",
                readiness.isReady() ? "READY" : "NOT_READY"
        );
        telemetry.addData("vision.readinessReason", readiness.reason());
    }

    private void emitScoringTelemetry(Telemetry telemetry,
                                      PhoenixCapabilities.ScoringStatus scoring,
                                      String prefix) {
        if (scoring == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "scoring" : prefix;
        telemetry.addData(p + ".flywheelEnabled", scoring.flywheelEnabled);
        telemetry.addData(p + ".pidfEnabled", scoring.pidfEnabled);
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

    private void emitScoringIntentTelemetry(
            Telemetry telemetry,
            PhoenixCapabilities.ScoringStatus scoring) {
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

    private void emitAimSummary(
            Telemetry telemetry,
            PhoenixCapabilities.TargetingStatus targeting) {
        if (targeting == null) {
            return;
        }
        telemetry.addData("aim.ready", targeting.aimReady);
        telemetry.addData("aim.okToShoot", targeting.aimOkToShoot);
        telemetry.addData("aim.override", targeting.aimOverride);
        telemetry.addData("aim.enabled", targeting.autoAimEnabled);
    }

    private void emitDriveAssistTelemetry(Telemetry telemetry,
                                          PhoenixDriveAssistService.Status driveAssist) {
        if (driveAssist == null) {
            return;
        }
        telemetry.addData("drive.poseAssistsAvailable", driveAssist.poseAssistsAvailable);
        telemetry.addData("drive.autoAimRequested", driveAssist.autoAimRequested);
        telemetry.addData("drive.shootBraceEligible", driveAssist.shootBraceEligible);
        telemetry.addData("drive.shootBraceEnabled", driveAssist.shootBraceEnabled);
        telemetry.addData("drive.manualTranslateMag", driveAssist.manualTranslateMagnitude);
    }

    private void emitPoseTelemetry(Telemetry telemetry,
                                   PoseEstimate globalPose,
                                   PoseEstimate odomPose) {
        if (globalPose != null) {
            telemetry.addData("pose.global", globalPose);
            telemetry.addData(
                    "pose.global.mode",
                    estimatorMode
            );
            telemetry.addData(
                    "pose.global.correctionSource",
                    correctionSourceMode
            );
        }
        if (odomPose != null) {
            telemetry.addData("pose.odom", odomPose);
        }
        if (globalPose != null && globalPose.hasPose && odomPose != null && odomPose.hasPose) {
            double dxIn = globalPose.fieldToRobotPose.xInches - odomPose.fieldToRobotPose.xInches;
            double dyIn = globalPose.fieldToRobotPose.yInches - odomPose.fieldToRobotPose.yInches;
            double headingDriftRad = MathUtil.wrapToPi(
                    globalPose.fieldToRobotPose.yawRad - odomPose.fieldToRobotPose.yawRad
            );
            telemetry.addData("pose.drift.in", Math.hypot(dxIn, dyIn));
            telemetry.addData("pose.drift.headingDeg", Math.toDegrees(headingDriftRad));
        }
    }

    private void emitTargetTelemetry(
            Telemetry telemetry,
            PhoenixCapabilities.TargetingStatus targeting) {
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
                : AprilTagObservation.noTarget();
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
