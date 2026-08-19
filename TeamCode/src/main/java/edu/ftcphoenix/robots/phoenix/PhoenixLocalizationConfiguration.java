package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import edu.ftcphoenix.fw.ftc.FtcFieldRegions;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.localization.LimelightFieldPoseEstimator;
import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcphoenix.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcphoenix.fw.localization.fusion.OdometryCorrectionEkfEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryCorrectionFusionEstimator;

/** Checked-in localization recipe for the current Phoenix robot. */
final class PhoenixLocalizationConfiguration {

    private PhoenixLocalizationConfiguration() {
    }

    /**
     * Returns a fresh localization configuration containing Phoenix's reviewed current answers.
     *
     * <p>The recipe is data only and performs no validation or hardware action. Its device name,
     * pod geometry/directions, camera assumptions, and estimator tuning remain physical claims that
     * must be verified on the adopting robot.</p>
     *
     * @return fresh mutable current Phoenix localization configuration
     */
    static FtcOdometryAprilTagLocalizationLane.Config current() {
        FtcOdometryAprilTagLocalizationLane.Config config =
                FtcOdometryAprilTagLocalizationLane.Config.defaults();
        config.estimation.correctedEstimatorMode =
                FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;
        config.estimation.correctionSource.mode =
                FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.APRILTAG_POSE;
        configurePredictor(config.predictor);
        configureAprilTags(config.estimation.aprilTags);
        configureLimelight(config.estimation.correctionSource.limelightFieldPose);
        configureFusion(config.estimation.correctionFusion);
        configureEkf(config.estimation.correctionEkf);
        return config;
    }

    private static void configurePredictor(PinpointOdometryPredictor.Config predictor) {
        predictor.hardwareMapName = "pinPoint";
        predictor.forwardPodOffsetLeftInches = 0.0;
        predictor.strafePodOffsetForwardInches = 0.0;
        predictor.forwardPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        predictor.strafePodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    private static void configureAprilTags(
            FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig aprilTags) {
        aprilTags.maxDetectionAgeSec = 0.50;
        FixedTagFieldPoseSolver.Config solver = aprilTags.fieldPoseSolver;
        solver.maxAbsBearingRad = 0.0;
        solver.preferObservationFieldPose = true;
        solver.observationFieldPoseMaxDeltaInches = 8.0;
        solver.observationFieldPoseMaxDeltaHeadingRad = Math.toRadians(12.0);
        solver.rangeSoftnessInches = 36.0;
        solver.minObservationWeight = 0.05;
        solver.outlierPositionGateInches = 18.0;
        solver.outlierHeadingGateRad = Math.toRadians(25.0);
        solver.plausibleFieldRegion = FtcFieldRegions.fullField();
        solver.maxOutsidePlausibleFieldRegionInches = 3.0;
    }

    private static void configureLimelight(LimelightFieldPoseEstimator.Config limelight) {
        limelight.mode = LimelightFieldPoseEstimator.Config.Mode.BOTPOSE;
        limelight.maxResultAgeSec = 0.25;
        limelight.minVisibleTags = 1;
        limelight.singleTagQuality = 0.55;
        limelight.multiTagQuality = 0.85;
        limelight.degradeWhenMoving = true;
        limelight.translationSpeedForZeroQualityInPerSec = 72.0;
        limelight.yawRateForZeroQualityRadPerSec = Math.toRadians(360.0);
        limelight.rejectWhenMovingTooFast = false;
        limelight.maxTranslationSpeedInPerSec = 120.0;
        limelight.maxYawRateRadPerSec = Math.toRadians(720.0);
    }

    private static void configureFusion(OdometryCorrectionFusionEstimator.Config fusion) {
        fusion.maxCorrectionAgeSec = 0.35;
        fusion.minCorrectionQuality = 0.10;
        fusion.correctionPositionGain = 0.25;
        fusion.correctionHeadingGain = 0.35;
        fusion.maxCorrectionPositionJumpIn = 24.0;
        fusion.maxCorrectionHeadingJumpRad = Math.toRadians(60.0);
        fusion.enableLatencyCompensation = true;
        fusion.predictorHistorySec = 1.0;
    }

    private static void configureEkf(OdometryCorrectionEkfEstimator.Config ekf) {
        ekf.maxCorrectionAgeSec = 0.35;
        ekf.minCorrectionQuality = 0.10;
        ekf.maxCorrectionPositionInnovationIn = 24.0;
        ekf.maxCorrectionHeadingInnovationRad = Math.toRadians(60.0);
        ekf.maxCorrectionMahalanobisSq = 14.0;
        ekf.enableLatencyCompensation = true;
        ekf.predictorHistorySec = 1.0;
        ekf.initialPositionStdIn = 6.0;
        ekf.initialHeadingStdRad = Math.toRadians(12.0);
        ekf.manualPosePositionStdIn = 3.0;
        ekf.manualPoseHeadingStdRad = Math.toRadians(6.0);
        ekf.predictorProcessPositionStdFloorIn = 0.20;
        ekf.predictorProcessPositionStdPerIn = 0.03;
        ekf.predictorProcessPositionStdPerRad = 0.55;
        ekf.predictorProcessHeadingStdFloorRad = Math.toRadians(0.35);
        ekf.predictorProcessHeadingStdPerIn = Math.toRadians(0.06);
        ekf.predictorProcessHeadingStdPerRad = 0.06;
        ekf.correctionPositionStdFloorIn = 1.75;
        ekf.correctionPositionStdScaleIn = 6.0;
        ekf.correctionHeadingStdFloorRad = Math.toRadians(3.0);
        ekf.correctionHeadingStdScaleRad = Math.toRadians(10.0);
        ekf.projectedCorrectionPositionStdPerSec = 18.0;
        ekf.projectedCorrectionHeadingStdPerSec = Math.toRadians(30.0);
    }
}
