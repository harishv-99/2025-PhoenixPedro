package edu.ftcphoenix.fw.localization.fusion;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.MotionDelta;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseResetter;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** CONFIG-04 regression coverage for corrected-estimator configuration and finite evidence. */
public final class OdometryCorrectionEstimatorValidationTest {

    private static final double EPSILON = 1e-9;

    private enum Kind {
        FUSION,
        EKF
    }

    @Test
    public void publicConstructionSurfaceRequiresOneExplicitConfig() {
        assertEquals(1, OdometryCorrectionFusionEstimator.class.getConstructors().length);
        assertEquals(1, OdometryCorrectionEkfEstimator.class.getConstructors().length);
        assertTrue(Arrays.equals(
                new Class<?>[]{
                        MotionPredictor.class,
                        AbsolutePoseEstimator.class,
                        OdometryCorrectionFusionEstimator.Config.class
                },
                OdometryCorrectionFusionEstimator.class
                        .getConstructors()[0]
                        .getParameterTypes()));
        assertTrue(Arrays.equals(
                new Class<?>[]{
                        MotionPredictor.class,
                        AbsolutePoseEstimator.class,
                        OdometryCorrectionEkfEstimator.Config.class
                },
                OdometryCorrectionEkfEstimator.class
                        .getConstructors()[0]
                        .getParameterTypes()));
        assertPublicConfigMethods(
                OdometryCorrectionFusionEstimator.Config.class,
                "defaults", "copy", "validatedCopy");
        assertPublicConfigMethods(
                OdometryCorrectionEkfEstimator.Config.class,
                "defaults", "copy", "validatedCopy");

        FakePredictor predictor = new FakePredictor();
        FakeCorrection correction = new FakeCorrection();
        expectFailureContaining(
                IllegalArgumentException.class,
                "Config must not be null",
                () -> new OdometryCorrectionFusionEstimator(predictor, correction, null));
        expectFailureContaining(
                IllegalArgumentException.class,
                "Config must not be null",
                () -> new OdometryCorrectionEkfEstimator(predictor, correction, null));
    }

    @Test
    public void fusionConfigCoversEveryNumericDomainAndExactBoundary() {
        FusionNumericCase[] cases = new FusionNumericCase[]{
                new FusionNumericCase("maxCorrectionAgeSec", -1.0,
                        (config, value) -> config.maxCorrectionAgeSec = value),
                new FusionNumericCase("minCorrectionQuality", -1.0,
                        (config, value) -> config.minCorrectionQuality = value),
                new FusionNumericCase("correctionPositionGain", -1.0,
                        (config, value) -> config.correctionPositionGain = value),
                new FusionNumericCase("correctionHeadingGain", -1.0,
                        (config, value) -> config.correctionHeadingGain = value),
                new FusionNumericCase("maxCorrectionPositionJumpIn", -1.0,
                        (config, value) -> config.maxCorrectionPositionJumpIn = value),
                new FusionNumericCase("maxCorrectionHeadingJumpRad", -1.0,
                        (config, value) -> config.maxCorrectionHeadingJumpRad = value),
                new FusionNumericCase("correctionConfidenceHoldSec", -1.0,
                        (config, value) -> config.correctionConfidenceHoldSec = value),
                new FusionNumericCase("predictorHistorySec", -1.0,
                        (config, value) -> config.predictorHistorySec = value)
        };
        assertPublicDoubleFieldsCovered(
                OdometryCorrectionFusionEstimator.Config.class,
                fusionCaseNames(cases));

        for (FusionNumericCase numericCase : cases) {
            for (double nonFinite : nonFiniteValues()) {
                assertFusionNumericFailure(numericCase, nonFinite);
            }
            assertFusionNumericFailure(numericCase, numericCase.domainInvalidValue);
        }
        assertFusionNumericFailure(
                new FusionNumericCase(
                        "minCorrectionQuality",
                        Math.nextUp(1.0),
                        (config, value) -> config.minCorrectionQuality = value),
                Math.nextUp(1.0));
        assertFusionNumericFailure(
                new FusionNumericCase(
                        "maxCorrectionHeadingJumpRad",
                        Math.nextUp(Math.PI),
                        (config, value) -> config.maxCorrectionHeadingJumpRad = value),
                Math.nextUp(Math.PI));

        OdometryCorrectionFusionEstimator.Config boundaries =
                OdometryCorrectionFusionEstimator.Config.defaults();
        boundaries.maxCorrectionAgeSec = 0.0;
        boundaries.minCorrectionQuality = 0.0;
        boundaries.correctionPositionGain = 0.0;
        boundaries.correctionHeadingGain = 0.0;
        boundaries.maxCorrectionPositionJumpIn = 0.0;
        boundaries.maxCorrectionHeadingJumpRad = 0.0;
        boundaries.correctionConfidenceHoldSec = 0.0;
        boundaries.predictorHistorySec = 0.0;
        boundaries.validatedCopy(null);

        boundaries.minCorrectionQuality = 1.0;
        boundaries.maxCorrectionHeadingJumpRad = Math.PI;
        boundaries.validatedCopy("  ");
    }

    @Test
    public void ekfConfigCoversEveryNumericDomainAndExactBoundary() {
        EkfNumericCase[] cases = new EkfNumericCase[]{
                new EkfNumericCase("maxCorrectionAgeSec", -1.0,
                        (config, value) -> config.maxCorrectionAgeSec = value),
                new EkfNumericCase("minCorrectionQuality", -1.0,
                        (config, value) -> config.minCorrectionQuality = value),
                new EkfNumericCase("maxCorrectionPositionInnovationIn", 0.0,
                        (config, value) -> config.maxCorrectionPositionInnovationIn = value),
                new EkfNumericCase("maxCorrectionHeadingInnovationRad", 0.0,
                        (config, value) -> config.maxCorrectionHeadingInnovationRad = value),
                new EkfNumericCase("maxCorrectionMahalanobisSq", 0.0,
                        (config, value) -> config.maxCorrectionMahalanobisSq = value),
                new EkfNumericCase("predictorHistorySec", -1.0,
                        (config, value) -> config.predictorHistorySec = value),
                new EkfNumericCase("initialPositionStdIn", 0.0,
                        (config, value) -> config.initialPositionStdIn = value),
                new EkfNumericCase("initialHeadingStdRad", 0.0,
                        (config, value) -> config.initialHeadingStdRad = value),
                new EkfNumericCase("manualPosePositionStdIn", 0.0,
                        (config, value) -> config.manualPosePositionStdIn = value),
                new EkfNumericCase("manualPoseHeadingStdRad", 0.0,
                        (config, value) -> config.manualPoseHeadingStdRad = value),
                new EkfNumericCase("predictorProcessPositionStdFloorIn", 0.0,
                        (config, value) -> config.predictorProcessPositionStdFloorIn = value),
                new EkfNumericCase("predictorProcessPositionStdPerIn", -1.0,
                        (config, value) -> config.predictorProcessPositionStdPerIn = value),
                new EkfNumericCase("predictorProcessPositionStdPerRad", -1.0,
                        (config, value) -> config.predictorProcessPositionStdPerRad = value),
                new EkfNumericCase("predictorProcessHeadingStdFloorRad", 0.0,
                        (config, value) -> config.predictorProcessHeadingStdFloorRad = value),
                new EkfNumericCase("predictorProcessHeadingStdPerIn", -1.0,
                        (config, value) -> config.predictorProcessHeadingStdPerIn = value),
                new EkfNumericCase("predictorProcessHeadingStdPerRad", -1.0,
                        (config, value) -> config.predictorProcessHeadingStdPerRad = value),
                new EkfNumericCase("correctionPositionStdFloorIn", 0.0,
                        (config, value) -> config.correctionPositionStdFloorIn = value),
                new EkfNumericCase("correctionPositionStdScaleIn", -1.0,
                        (config, value) -> config.correctionPositionStdScaleIn = value),
                new EkfNumericCase("projectedCorrectionPositionStdPerSec", -1.0,
                        (config, value) -> config.projectedCorrectionPositionStdPerSec = value),
                new EkfNumericCase("correctionHeadingStdFloorRad", 0.0,
                        (config, value) -> config.correctionHeadingStdFloorRad = value),
                new EkfNumericCase("correctionHeadingStdScaleRad", -1.0,
                        (config, value) -> config.correctionHeadingStdScaleRad = value),
                new EkfNumericCase("projectedCorrectionHeadingStdPerSec", -1.0,
                        (config, value) -> config.projectedCorrectionHeadingStdPerSec = value),
                new EkfNumericCase("qualityPositionStdScaleIn", 0.0,
                        (config, value) -> config.qualityPositionStdScaleIn = value),
                new EkfNumericCase("qualityHeadingStdScaleRad", 0.0,
                        (config, value) -> config.qualityHeadingStdScaleRad = value)
        };
        assertPublicDoubleFieldsCovered(
                OdometryCorrectionEkfEstimator.Config.class,
                ekfCaseNames(cases));

        for (EkfNumericCase numericCase : cases) {
            for (double nonFinite : nonFiniteValues()) {
                assertEkfNumericFailure(numericCase, nonFinite);
            }
            assertEkfNumericFailure(numericCase, numericCase.domainInvalidValue);
        }
        assertEkfNumericFailure(
                new EkfNumericCase(
                        "minCorrectionQuality",
                        Math.nextUp(1.0),
                        (config, value) -> config.minCorrectionQuality = value),
                Math.nextUp(1.0));
        assertEkfNumericFailure(
                new EkfNumericCase(
                        "maxCorrectionHeadingInnovationRad",
                        Math.nextUp(Math.PI),
                        (config, value) -> config.maxCorrectionHeadingInnovationRad = value),
                Math.nextUp(Math.PI));

        OdometryCorrectionEkfEstimator.Config boundaries =
                OdometryCorrectionEkfEstimator.Config.defaults();
        boundaries.maxCorrectionAgeSec = 0.0;
        boundaries.minCorrectionQuality = 0.0;
        boundaries.maxCorrectionHeadingInnovationRad = Math.PI;
        boundaries.predictorHistorySec = 0.0;
        boundaries.predictorProcessPositionStdPerIn = 0.0;
        boundaries.predictorProcessPositionStdPerRad = 0.0;
        boundaries.predictorProcessHeadingStdPerIn = 0.0;
        boundaries.predictorProcessHeadingStdPerRad = 0.0;
        boundaries.correctionPositionStdScaleIn = 0.0;
        boundaries.projectedCorrectionPositionStdPerSec = 0.0;
        boundaries.correctionHeadingStdScaleRad = 0.0;
        boundaries.projectedCorrectionHeadingStdPerSec = 0.0;
        boundaries.validatedCopy(null);

        boundaries.minCorrectionQuality = 1.0;
        boundaries.validatedCopy("  ");
    }

    @Test
    public void fusionConfigUsesExactHistoryAndWrappedHeadingDomains() {
        OdometryCorrectionFusionEstimator.Config exact =
                OdometryCorrectionFusionEstimator.Config.defaults();
        exact.maxCorrectionAgeSec = 0.25;
        exact.predictorHistorySec = 0.25;
        exact.maxCorrectionHeadingJumpRad = Math.PI;
        exact.validatedCopy("fusionPolicy");

        exact.predictorHistorySec = 0.25 - 0.5e-6;
        expectFailureContaining(
                IllegalArgumentException.class,
                "predictorHistorySec=",
                () -> exact.validatedCopy("fusionPolicy"));

        exact.predictorHistorySec = 0.25;
        exact.maxCorrectionHeadingJumpRad = Math.nextUp(Math.PI);
        expectFailureContaining(
                IllegalArgumentException.class,
                "got " + Math.nextUp(Math.PI),
                () -> exact.validatedCopy("fusionPolicy"));
    }

    @Test
    public void ekfConfigPreflightsDeterministicCovarianceRepresentations() {
        OdometryCorrectionEkfEstimator.Config exact =
                OdometryCorrectionEkfEstimator.Config.defaults();
        exact.maxCorrectionAgeSec = 0.25;
        exact.predictorHistorySec = 0.25;
        exact.maxCorrectionHeadingInnovationRad = Math.PI;
        exact.validatedCopy("ekfPolicy");

        exact.predictorHistorySec = 0.25 - 0.5e-6;
        expectFailureContaining(
                IllegalArgumentException.class,
                "predictorHistorySec=",
                () -> exact.validatedCopy("ekfPolicy"));

        OdometryCorrectionEkfEstimator.Config zeroHeading =
                OdometryCorrectionEkfEstimator.Config.defaults();
        zeroHeading.maxCorrectionHeadingInnovationRad = 0.0;
        expectFailureContaining(
                IllegalArgumentException.class,
                "got 0.0",
                () -> zeroHeading.validatedCopy("ekfPolicy"));

        double finiteButUnsafeToSquare = Math.sqrt(Double.MAX_VALUE) * 2.0;
        expectEkfCovarianceFailure(
                "initialPositionStdIn must square to a finite variance",
                config -> config.initialPositionStdIn = finiteButUnsafeToSquare);
        expectEkfCovarianceFailure(
                "initialHeadingStdRad must square to a finite variance",
                config -> config.initialHeadingStdRad = finiteButUnsafeToSquare);
        expectEkfCovarianceFailure(
                "manualPosePositionStdIn must square to a finite variance",
                config -> config.manualPosePositionStdIn = finiteButUnsafeToSquare);
        expectEkfCovarianceFailure(
                "manualPoseHeadingStdRad must square to a finite variance",
                config -> config.manualPoseHeadingStdRad = finiteButUnsafeToSquare);
        expectEkfCovarianceFailure(
                "predictorProcessPositionStdFloorIn must square to a finite variance",
                config -> config.predictorProcessPositionStdFloorIn = finiteButUnsafeToSquare);
        expectEkfCovarianceFailure(
                "predictorProcessHeadingStdFloorRad must square to a finite variance",
                config -> config.predictorProcessHeadingStdFloorRad = finiteButUnsafeToSquare);

        double finiteIndividuallyButCombinedUnsafe = Math.sqrt(Double.MAX_VALUE) * 0.6;
        expectEkfCovarianceFailure(
                "position correction standard deviation",
                config -> {
                    config.correctionPositionStdFloorIn = finiteIndividuallyButCombinedUnsafe;
                    config.correctionPositionStdScaleIn = finiteIndividuallyButCombinedUnsafe;
                    config.projectedCorrectionPositionStdPerSec = 0.0;
                });
        expectEkfCovarianceFailure(
                "heading correction standard deviation",
                config -> {
                    config.correctionHeadingStdFloorRad = finiteIndividuallyButCombinedUnsafe;
                    config.correctionHeadingStdScaleRad = finiteIndividuallyButCombinedUnsafe;
                    config.projectedCorrectionHeadingStdPerSec = 0.0;
                });
        expectEkfCovarianceFailure(
                "position correction standard deviation",
                config -> {
                    config.enableLatencyCompensation = false;
                    config.maxCorrectionAgeSec = Double.MAX_VALUE / 2.0;
                    config.projectedCorrectionPositionStdPerSec = 3.0;
                    config.projectedCorrectionHeadingStdPerSec = 0.0;
                });
        expectEkfCovarianceFailure(
                "heading correction standard deviation",
                config -> {
                    config.enableLatencyCompensation = false;
                    config.maxCorrectionAgeSec = Double.MAX_VALUE / 2.0;
                    config.projectedCorrectionPositionStdPerSec = 0.0;
                    config.projectedCorrectionHeadingStdPerSec = 3.0;
                });

        double finiteIndividuallyButPerRadianUnsafe = Math.sqrt(Double.MAX_VALUE) * 0.5;
        expectEkfCovarianceFailure(
                "position per-radian process standard deviation",
                config -> config.predictorProcessPositionStdPerRad =
                        finiteIndividuallyButPerRadianUnsafe);
        expectEkfCovarianceFailure(
                "heading per-radian process standard deviation",
                config -> config.predictorProcessHeadingStdPerRad =
                        finiteIndividuallyButPerRadianUnsafe);
    }

    @Test
    public void authoredPoseValidationPrecedesPredictorEffects() {
        for (Kind kind : Kind.values()) {
            Fixture fixture = new Fixture(kind, true, 0.25);

            expectFailure(NullPointerException.class, () -> fixture.estimator.setPose(null));
            expectFailureContaining(
                    IllegalArgumentException.class,
                    ".xInches must be finite, got NaN",
                    () -> fixture.estimator.setPose(new Pose2d(Double.NaN, 0.0, 0.0)));
            expectFailureContaining(
                    IllegalArgumentException.class,
                    ".yInches must be finite, got Infinity",
                    () -> fixture.estimator.setPose(
                            new Pose2d(0.0, Double.POSITIVE_INFINITY, 0.0)));
            expectFailureContaining(
                    IllegalArgumentException.class,
                    ".headingRad must be finite, got -Infinity",
                    () -> fixture.estimator.setPose(
                            new Pose2d(0.0, 0.0, Double.NEGATIVE_INFINITY)));
            assertEquals(0, fixture.predictor.setPoseCalls);

            fixture.estimator.setPose(new Pose2d(1.0, 2.0, 3.0));
            PoseEstimate accepted = fixture.estimator.getEstimate();
            assertTrue(accepted.hasPose);
            assertEquals(1.0, accepted.fieldToRobotPose.xInches, EPSILON);
            assertEquals(1, fixture.predictor.setPoseCalls);

            RuntimeException resetFailure = new IllegalArgumentException("vendor pose rejected");
            fixture.predictor.setPoseFailure = resetFailure;
            assertSame(
                    resetFailure,
                    captureFailure(() -> fixture.estimator.setPose(new Pose2d(9.0, 0.0, 0.0))));
            assertSame(accepted, fixture.estimator.getEstimate());
        }
    }

    @Test
    public void rejectedPredictorPushDoesNotPartiallyCommitAnUpdate() {
        for (Kind kind : Kind.values()) {
            Fixture fixture = new Fixture(kind, true, 1.0);
            LoopTimestamp start = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(0.0), 1.0, start, MotionDelta.none(start));
            fixture.correction.publishNone(start);
            fixture.estimator.update(fixture.time.clock());
            PoseEstimate beforeFailure = fixture.estimator.getEstimate();

            fixture.time.nextCycle(0.1);
            LoopTimestamp correctionTime = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(
                    pose(1.0),
                    1.0,
                    correctionTime,
                    motion(1.0, 1.0, start, correctionTime));
            fixture.correction.publish(pose(2.0), 1.0, correctionTime);
            RuntimeException resetFailure = new IllegalStateException("resetter rejected candidate");
            fixture.predictor.setPoseFailure = resetFailure;

            assertSame(resetFailure, captureFailure(() -> fixture.estimator.update(fixture.time.clock())));
            assertSame(beforeFailure, fixture.estimator.getEstimate());
            assertEquals(0, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);

            fixture.predictor.setPoseFailure = null;
            fixture.time.nextCycle(0.1);
            LoopTimestamp recoveredTime = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(
                    pose(1.0),
                    1.0,
                    recoveredTime,
                    MotionDelta.none(recoveredTime));
            fixture.correction.publishNone(recoveredTime);
            fixture.estimator.update(fixture.time.clock());
            assertEquals(
                    0.0,
                    fixture.estimator.getEstimate().fieldToRobotPose.xInches,
                    EPSILON);

            fixture.time.nextCycle(0.1);
            LoopTimestamp resumedTime = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(
                    pose(2.0),
                    1.0,
                    resumedTime,
                    motion(1.0, 1.0, recoveredTime, resumedTime));
            fixture.correction.publishNone(resumedTime);
            fixture.estimator.update(fixture.time.clock());
            assertEquals(
                    "failed push must restore the exact pre-update motion/replay baseline",
                    2.0,
                    fixture.estimator.getEstimate().fieldToRobotPose.xInches,
                    EPSILON);
        }
    }

    @Test
    public void invalidClaimedMotionIsIgnoredAndCannotBridgeTheNextDelta() {
        for (Kind kind : Kind.values()) {
            Fixture fixture = new Fixture(kind, false, 1.0);
            LoopTimestamp t0 = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(0.0), 1.0, t0, MotionDelta.none(t0));
            fixture.correction.publishNone(t0);
            fixture.estimator.update(fixture.time.clock());

            fixture.time.nextCycle(0.1);
            LoopTimestamp t1 = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(
                    pose(10.0),
                    1.0,
                    t1,
                    motion(10.0, Double.NaN, t0, t1));
            fixture.correction.publishNone(t1);
            fixture.estimator.update(fixture.time.clock());
            assertEquals(0.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

            fixture.time.nextCycle(0.1);
            LoopTimestamp t2 = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(11.0), 1.0, t2, motion(1.0, 1.0, t1, t2));
            fixture.correction.publishNone(t2);
            fixture.estimator.update(fixture.time.clock());
            assertEquals(1.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
        }
    }

    @Test
    public void zeroAgeIsInclusiveAndOneStaleIdentityIsRejectedOnce() {
        for (Kind kind : Kind.values()) {
            Fixture fixture = new Fixture(kind, false, 0.0);
            LoopTimestamp t0 = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(0.0), 1.0, t0, MotionDelta.none(t0));
            fixture.correction.publishNone(t0);
            fixture.estimator.update(fixture.time.clock());

            fixture.time.nextCycle(0.0);
            LoopTimestamp current = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(0.0), 1.0, current, MotionDelta.none(current));
            fixture.correction.publish(pose(0.0), 1.0, current);
            fixture.estimator.update(fixture.time.clock());
            assertEquals(1, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);

            fixture.time.nextCycle(0.1);
            LoopTimestamp stale = fixture.time.clock().timestampSecondsAgo(0.05);
            LoopTimestamp now = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(0.0), 1.0, now, MotionDelta.none(now));
            fixture.correction.publish(pose(0.0), 1.0, stale);
            fixture.estimator.update(fixture.time.clock());

            fixture.time.nextCycle(0.1);
            LoopTimestamp later = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(0.0), 1.0, later, MotionDelta.none(later));
            fixture.estimator.update(fixture.time.clock());

            CorrectionStats stats = fixture.estimator.getCorrectionStats();
            assertEquals(1, stats.acceptedCorrectionCount);
            assertEquals(1, stats.rejectedCorrectionCount);
            assertEquals(1, stats.skippedDuplicateCorrectionCount);
        }
    }

    @Test
    public void materiallyFutureCorrectionCannotPoisonTheEvaluationWatermark() {
        for (Kind kind : Kind.values()) {
            Fixture fixture = new Fixture(kind, false, 1.0);
            LoopTimestamp initial = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(0.0), 1.0, initial, MotionDelta.none(initial));
            fixture.correction.publishNone(initial);
            fixture.estimator.update(fixture.time.clock());

            fixture.time.clock().update(10.5);
            LoopTimestamp future = fixture.time.clock().nowTimestamp();
            fixture.time.clock().update(10.0);
            LoopTimestamp current = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(0.0), 1.0, current, MotionDelta.none(current));
            fixture.correction.publish(pose(4.0), 1.0, future);
            fixture.estimator.update(fixture.time.clock());

            CorrectionStats afterFuture = fixture.estimator.getCorrectionStats();
            assertEquals(0, afterFuture.acceptedCorrectionCount);
            assertEquals(0, afterFuture.rejectedCorrectionCount);
            assertFalse(afterFuture.lastEvaluatedCorrectionTimestamp.isAvailable());

            fixture.time.nextCycle(0.1);
            LoopTimestamp valid = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(0.0), 1.0, valid, MotionDelta.none(valid));
            fixture.correction.publish(pose(1.0), 1.0, valid);
            fixture.estimator.update(fixture.time.clock());

            CorrectionStats afterValid = fixture.estimator.getCorrectionStats();
            assertEquals(1, afterValid.acceptedCorrectionCount);
            assertEquals(0, afterValid.rejectedCorrectionCount);
            assertSame(valid, afterValid.lastEvaluatedCorrectionTimestamp);
        }
    }

    @Test
    public void ekfTranslationDependentProcessOverflowDoesNotCommitState() {
        Fixture fixture = new Fixture(Kind.EKF, false, 1.0);
        LoopTimestamp initial = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(pose(0.0), 1.0, initial, MotionDelta.none(initial));
        fixture.correction.publishNone(initial);
        fixture.estimator.update(fixture.time.clock());
        assertEquals(0.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        fixture.time.nextCycle(0.1);
        LoopTimestamp overflowTime = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                pose(Double.MAX_VALUE),
                1.0,
                overflowTime,
                motion(Double.MAX_VALUE, 1.0, initial, overflowTime));
        fixture.correction.publishNone(overflowTime);
        fixture.estimator.update(fixture.time.clock());

        PoseEstimate afterOverflow = fixture.estimator.getEstimate();
        assertTrue(afterOverflow.hasPose);
        assertTrue(Double.isFinite(afterOverflow.fieldToRobotPose.xInches));
        assertTrue(Double.isFinite(afterOverflow.quality));
        assertEquals(0.0, afterOverflow.fieldToRobotPose.xInches, EPSILON);
        assertEquals(0, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);
    }

    @Test
    public void laterPredictorLossDoesNotRepublishFrozenStateAsFreshEvidence() {
        for (Kind kind : Kind.values()) {
            Fixture fixture = new Fixture(kind, false, 1.0);
            LoopTimestamp initial = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(0.0), 1.0, initial, MotionDelta.none(initial));
            fixture.correction.publishNone(initial);
            fixture.estimator.update(fixture.time.clock());
            assertTrue(fixture.estimator.getEstimate().hasPose);

            fixture.time.nextCycle(0.1);
            LoopTimestamp unavailableTime = fixture.time.clock().nowTimestamp();
            fixture.predictor.publishNone(unavailableTime);
            fixture.correction.publishNone(unavailableTime);
            fixture.estimator.update(fixture.time.clock());
            PoseEstimate unavailable = fixture.estimator.getEstimate();
            assertFalse(unavailable.hasPose);
            assertEquals(0.0, unavailable.quality, EPSILON);
            assertEquals(
                    0.0,
                    unavailable.timestamp.secondsSince(unavailableTime),
                    EPSILON);
            assertEquals(0.0, unavailable.timestamp.ageSec(fixture.time.clock()), EPSILON);

            fixture.time.nextCycle(0.1);
            LoopTimestamp correctionTime = fixture.time.clock().nowTimestamp();
            fixture.predictor.publishNone(correctionTime);
            fixture.correction.publish(pose(1.0), 1.0, correctionTime);
            fixture.estimator.update(fixture.time.clock());
            assertTrue(fixture.estimator.getEstimate().hasPose);
            assertEquals(1, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);

            fixture.time.nextCycle(0.1);
            LoopTimestamp secondGap = fixture.time.clock().nowTimestamp();
            fixture.predictor.publishNone(secondGap);
            fixture.correction.publishNone(secondGap);
            fixture.estimator.update(fixture.time.clock());
            assertFalse(fixture.estimator.getEstimate().hasPose);
        }
    }

    @Test
    public void nonFinitePredictorQualityBecomesZeroWithoutDiscardingFinitePose() {
        Fixture fixture = new Fixture(Kind.FUSION, false, 1.0);
        LoopTimestamp timestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                pose(4.0),
                Double.NaN,
                timestamp,
                MotionDelta.none(timestamp));
        fixture.correction.publishNone(timestamp);

        fixture.estimator.update(fixture.time.clock());

        assertTrue(fixture.estimator.getEstimate().hasPose);
        assertEquals(4.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
        assertEquals(0.0, fixture.estimator.getEstimate().quality, EPSILON);
    }

    @Test
    public void claimedNonFinitePlanarEvidenceIsUnavailableOrRejected() {
        for (Kind kind : Kind.values()) {
            Fixture fixture = new Fixture(kind, false, 1.0);
            LoopTimestamp t0 = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(
                    new Pose3d(Double.POSITIVE_INFINITY, 0.0, 0.0, 0.0, 0.0, 0.0),
                    1.0,
                    t0,
                    MotionDelta.none(t0));
            fixture.correction.publishNone(t0);
            fixture.estimator.update(fixture.time.clock());
            assertFalse(fixture.estimator.getEstimate().hasPose);

            fixture.time.nextCycle(0.1);
            LoopTimestamp t1 = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(0.0), 1.0, t1, MotionDelta.none(t1));
            fixture.correction.publishNone(t1);
            fixture.estimator.update(fixture.time.clock());
            assertTrue(fixture.estimator.getEstimate().hasPose);

            fixture.time.nextCycle(0.1);
            LoopTimestamp t2 = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(0.0), 1.0, t2, MotionDelta.none(t2));
            fixture.correction.publish(
                    new Pose3d(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0),
                    1.0,
                    t2);
            fixture.estimator.update(fixture.time.clock());
            assertTrue(fixture.estimator.getEstimate().hasPose);
            assertTrue(Double.isFinite(
                    fixture.estimator.getEstimate().fieldToRobotPose.yawRad));
            assertEquals(1, fixture.estimator.getCorrectionStats().rejectedCorrectionCount);

            fixture.time.nextCycle(0.1);
            LoopTimestamp t3 = fixture.time.clock().nowTimestamp();
            fixture.predictor.publish(pose(0.0), 1.0, t3, MotionDelta.none(t3));
            fixture.correction.publish(pose(0.0), 1.1, t3);
            fixture.estimator.update(fixture.time.clock());
            assertEquals(2, fixture.estimator.getCorrectionStats().rejectedCorrectionCount);
        }
    }

    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock(10.0);
        final FakePredictor predictor = new FakePredictor();
        final FakeCorrection correction = new FakeCorrection();
        final CorrectedPoseEstimator estimator;

        Fixture(Kind kind, boolean push, double maxCorrectionAgeSec) {
            if (kind == Kind.FUSION) {
                OdometryCorrectionFusionEstimator.Config config =
                        OdometryCorrectionFusionEstimator.Config.defaults();
                config.maxCorrectionAgeSec = maxCorrectionAgeSec;
                config.predictorHistorySec = maxCorrectionAgeSec;
                config.correctionPositionGain = 1.0;
                config.correctionHeadingGain = 1.0;
                config.maxCorrectionPositionJumpIn = 1_000.0;
                config.maxCorrectionHeadingJumpRad = Math.PI;
                config.enablePushCorrectedPoseToPredictor = push;
                estimator = new OdometryCorrectionFusionEstimator(predictor, correction, config);
            } else {
                OdometryCorrectionEkfEstimator.Config config =
                        OdometryCorrectionEkfEstimator.Config.defaults();
                config.maxCorrectionAgeSec = maxCorrectionAgeSec;
                config.predictorHistorySec = maxCorrectionAgeSec;
                config.maxCorrectionPositionInnovationIn = 1_000.0;
                config.maxCorrectionHeadingInnovationRad = Math.PI;
                config.maxCorrectionMahalanobisSq = 1_000_000_000.0;
                config.enablePushCorrectedPoseToPredictor = push;
                estimator = new OdometryCorrectionEkfEstimator(predictor, correction, config);
            }
        }
    }

    private static final class FakePredictor implements MotionPredictor, PoseResetter {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());
        RuntimeException setPoseFailure;
        int setPoseCalls;
        long trajectorySegmentId;

        void publish(Pose3d pose,
                     double quality,
                     LoopTimestamp timestamp,
                     MotionDelta nextDelta) {
            estimate = new PoseEstimate(pose, true, quality, timestamp);
            delta = nextDelta;
        }

        void publishNone(LoopTimestamp timestamp) {
            estimate = PoseEstimate.noPose(timestamp);
            delta = MotionDelta.none(timestamp);
        }

        @Override
        public void update(LoopClock clock) {
            // Published explicitly by each deterministic test cycle.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }

        @Override
        public long trajectorySegmentId() {
            return trajectorySegmentId;
        }

        @Override
        public MotionDelta getLatestMotionDelta() {
            return delta;
        }

        @Override
        public void setPose(Pose2d pose) {
            setPoseCalls++;
            if (setPoseFailure != null) {
                throw setPoseFailure;
            }
            trajectorySegmentId++;
            LoopTimestamp timestamp = estimate.timestamp;
            estimate = new PoseEstimate(
                    new Pose3d(
                            pose.xInches,
                            pose.yInches,
                            0.0,
                            pose.headingRad,
                            0.0,
                            0.0),
                    true,
                    1.0,
                    timestamp);
            delta = MotionDelta.none(timestamp);
        }
    }

    private static final class FakeCorrection implements AbsolutePoseEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());

        void publish(Pose3d pose, double quality, LoopTimestamp timestamp) {
            estimate = new PoseEstimate(pose, true, quality, timestamp);
        }

        void publishNone(LoopTimestamp timestamp) {
            estimate = PoseEstimate.noPose(timestamp);
        }

        @Override
        public void update(LoopClock clock) {
            // Published explicitly by each deterministic test cycle.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }

    private static Pose3d pose(double xInches) {
        return new Pose3d(xInches, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    private static MotionDelta motion(double xInches,
                                      double quality,
                                      LoopTimestamp start,
                                      LoopTimestamp end) {
        return new MotionDelta(pose(xInches), true, quality, start, end);
    }

    private static void assertPublicConfigMethods(Class<?> type, String... expectedNames) {
        Set<String> actual = new HashSet<String>();
        int publicMethodCount = 0;
        for (Method method : type.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers()) && !method.isSynthetic()) {
                publicMethodCount++;
                actual.add(method.getName());
            }
        }
        assertEquals(expectedNames.length, publicMethodCount);
        assertEquals(new HashSet<String>(Arrays.asList(expectedNames)), actual);
    }

    private interface FusionNumericMutation {
        void apply(OdometryCorrectionFusionEstimator.Config config, double value);
    }

    private static final class FusionNumericCase {
        final String fieldName;
        final double domainInvalidValue;
        final FusionNumericMutation mutation;

        FusionNumericCase(String fieldName,
                          double domainInvalidValue,
                          FusionNumericMutation mutation) {
            this.fieldName = fieldName;
            this.domainInvalidValue = domainInvalidValue;
            this.mutation = mutation;
        }
    }

    private interface EkfNumericMutation {
        void apply(OdometryCorrectionEkfEstimator.Config config, double value);
    }

    private static final class EkfNumericCase {
        final String fieldName;
        final double domainInvalidValue;
        final EkfNumericMutation mutation;

        EkfNumericCase(String fieldName,
                       double domainInvalidValue,
                       EkfNumericMutation mutation) {
            this.fieldName = fieldName;
            this.domainInvalidValue = domainInvalidValue;
            this.mutation = mutation;
        }
    }

    private static void assertFusionNumericFailure(FusionNumericCase numericCase, double value) {
        OdometryCorrectionFusionEstimator.Config config =
                OdometryCorrectionFusionEstimator.Config.defaults();
        numericCase.mutation.apply(config, value);
        RuntimeException failure = captureFailure(() -> config.validatedCopy(null));
        assertTrue(failure instanceof IllegalArgumentException);
        assertTrue(
                failure.getMessage(),
                failure.getMessage().contains(
                        "OdometryCorrectionFusionEstimator.Config." + numericCase.fieldName));
        assertTrue(
                failure.getMessage(),
                failure.getMessage().contains("got " + value));
    }

    private static void assertEkfNumericFailure(EkfNumericCase numericCase, double value) {
        OdometryCorrectionEkfEstimator.Config config =
                OdometryCorrectionEkfEstimator.Config.defaults();
        numericCase.mutation.apply(config, value);
        RuntimeException failure = captureFailure(() -> config.validatedCopy("  "));
        assertTrue(failure instanceof IllegalArgumentException);
        assertTrue(
                failure.getMessage(),
                failure.getMessage().contains(
                        "OdometryCorrectionEkfEstimator.Config." + numericCase.fieldName));
        assertTrue(
                failure.getMessage(),
                failure.getMessage().contains("got " + value));
    }

    private static Set<String> fusionCaseNames(FusionNumericCase[] cases) {
        Set<String> names = new HashSet<String>();
        for (FusionNumericCase numericCase : cases) {
            names.add(numericCase.fieldName);
        }
        return names;
    }

    private static Set<String> ekfCaseNames(EkfNumericCase[] cases) {
        Set<String> names = new HashSet<String>();
        for (EkfNumericCase numericCase : cases) {
            names.add(numericCase.fieldName);
        }
        return names;
    }

    private static void assertPublicDoubleFieldsCovered(Class<?> configType,
                                                        Set<String> coveredNames) {
        Set<String> publicDoubleFields = new HashSet<String>();
        for (Field field : configType.getDeclaredFields()) {
            if (Modifier.isPublic(field.getModifiers()) && field.getType() == double.class) {
                publicDoubleFields.add(field.getName());
            }
        }
        assertEquals(publicDoubleFields, coveredNames);
    }

    private static double[] nonFiniteValues() {
        return new double[]{
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };
    }

    private interface EkfConfigMutation {
        void apply(OdometryCorrectionEkfEstimator.Config config);
    }

    private static void expectEkfCovarianceFailure(String messagePart,
                                                   EkfConfigMutation mutation) {
        OdometryCorrectionEkfEstimator.Config config =
                OdometryCorrectionEkfEstimator.Config.defaults();
        mutation.apply(config);
        expectFailureContaining(
                IllegalArgumentException.class,
                messagePart,
                () -> config.validatedCopy("ekfPolicy"));
    }

    private static RuntimeException captureFailure(Runnable action) {
        try {
            action.run();
            fail("Expected failure");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static void expectFailure(Class<? extends RuntimeException> type, Runnable action) {
        RuntimeException failure = captureFailure(action);
        assertTrue("Unexpected failure: " + failure, type.isInstance(failure));
    }

    private static void expectFailureContaining(Class<? extends RuntimeException> type,
                                                String messagePart,
                                                Runnable action) {
        RuntimeException failure = captureFailure(action);
        assertTrue("Unexpected failure: " + failure, type.isInstance(failure));
        assertTrue(failure.getMessage(), failure.getMessage().contains(messagePart));
    }
}
