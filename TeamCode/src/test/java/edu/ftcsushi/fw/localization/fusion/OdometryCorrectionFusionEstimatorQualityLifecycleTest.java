package edu.ftcsushi.fw.localization.fusion;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.MotionDelta;
import edu.ftcsushi.fw.localization.MotionPredictor;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseResetter;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/**
 * LOCALIZATION-02 lifecycle evidence through public estimator, predictor, and correction seams.
 *
 * <p>The real Fusion owner and clock retain acceptance, rollback, continuity, and quality behavior.
 * Scripted sources replace only outside pose evidence and the predictor's fallible rebase boundary.
 * The deliberately low predictor score makes retained correction quality observable; these scores
 * and exact pose expectations prove software contracts, not physical confidence or odometry accuracy.
 */
public final class OdometryCorrectionFusionEstimatorQualityLifecycleTest {

    private static final double EPSILON = 1e-9;
    private static final double PREDICTOR_QUALITY = 0.1;
    private static final double HOLD_SEC = 2.0;

    @Test
    public void successfulManualAnchorUsesPredictorQualityAndClearsCorrectionHold() {
        Fixture fixture = new Fixture();
        fixture.acceptInitialCorrection(0.8);
        long segment = fixture.estimator.trajectorySegmentId();
        fixture.predictor.publish(2.0, 0.15, fixture.now());

        fixture.estimator.setPose(new Pose2d(20.0, 0.0, 0.0));

        assertEstimate(fixture.estimator, 20.0, 0.15);
        assertEquals(segment + 1L, fixture.estimator.trajectorySegmentId());
        assertEquals(1, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);
        assertFalse(fixture.estimator.getLastCorrectionAccepted().isAvailable());
        PoseEstimate anchored = fixture.estimator.getEstimate();
        int pushes = fixture.predictor.pushes;
        fixture.estimator.update(fixture.time.clock());
        assertSame("same-cycle update must not restore the camera score", anchored,
                fixture.estimator.getEstimate());
        assertEquals(pushes, fixture.predictor.pushes);

        fixture.predictorOnlyAfter(0.5, 20.0);
        assertEstimate(fixture.estimator, 20.0, PREDICTOR_QUALITY);
    }

    @Test
    public void manualAnchorWithoutPredictorPoseKeepsImmediateOneFallbackButNoCameraHold() {
        Fixture fixture = new Fixture();
        fixture.acceptInitialCorrection(0.8);
        fixture.predictor.publishNone(fixture.now());

        fixture.estimator.setPose(new Pose2d(20.0, 0.0, 0.0));

        assertEstimate(fixture.estimator, 20.0, 1.0);
        assertFalse(fixture.estimator.getLastCorrectionAccepted().isAvailable());
        assertEquals(1, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);
        fixture.predictorOnlyAfter(0.5, 20.0);
        assertEstimate(fixture.estimator, 20.0, PREDICTOR_QUALITY);
    }

    @Test
    public void manualAnchorPreservesExistingSanitationOfReportedPredictorQuality() {
        double[] authored = {Double.NaN, Double.POSITIVE_INFINITY, -0.2, 1.2};
        double[] expected = {0.0, 0.0, 0.0, 1.0};
        for (int i = 0; i < authored.length; i++) {
            Fixture fixture = new Fixture();
            fixture.acceptInitialCorrection(0.8);
            fixture.predictor.publish(2.0, authored[i], fixture.now());

            fixture.estimator.setPose(new Pose2d(20.0, 0.0, 0.0));

            assertEstimate(fixture.estimator, 20.0, expected[i]);
            fixture.predictorOnlyAfter(0.5, 20.0);
            assertEstimate(fixture.estimator, 20.0, PREDICTOR_QUALITY);
        }
    }

    @Test
    public void invalidManualAnchorsLeavePublishedQualityAndOriginalHoldUntouched() {
        Fixture fixture = new Fixture();
        fixture.acceptInitialCorrection(0.8);
        PoseEstimate before = fixture.estimator.getEstimate();
        CorrectionStats beforeStats = fixture.estimator.getCorrectionStats();
        int pushes = fixture.predictor.pushes;
        long segment = fixture.estimator.trajectorySegmentId();

        assertThrows(NullPointerException.class, () -> fixture.estimator.setPose(null));
        assertThrows(IllegalArgumentException.class,
                () -> fixture.estimator.setPose(new Pose2d(Double.NaN, 0.0, 0.0)));
        assertThrows(IllegalArgumentException.class,
                () -> fixture.estimator.setPose(new Pose2d(0.0, Double.POSITIVE_INFINITY, 0.0)));
        assertThrows(IllegalArgumentException.class,
                () -> fixture.estimator.setPose(new Pose2d(0.0, 0.0, Double.NEGATIVE_INFINITY)));

        assertSame(before, fixture.estimator.getEstimate());
        assertSameStats(beforeStats, fixture.estimator.getCorrectionStats());
        assertEquals(pushes, fixture.predictor.pushes);
        assertEquals(segment, fixture.estimator.trajectorySegmentId());
        fixture.predictorOnlyAfter(0.5, 2.0);
        assertEstimate(fixture.estimator, 2.0, 0.6);
        fixture.predictorOnlyAfter(1.5, 2.0);
        assertEstimate(fixture.estimator, 2.0, PREDICTOR_QUALITY);
    }

    @Test
    public void transactionalManualPushFailurePreservesQualityAndItsOriginalDeadline() {
        Fixture fixture = new Fixture();
        fixture.acceptInitialCorrection(0.8);
        PoseEstimate before = fixture.estimator.getEstimate();
        CorrectionStats beforeStats = fixture.estimator.getCorrectionStats();
        long segment = fixture.estimator.trajectorySegmentId();
        RuntimeException failure = fixture.predictor.rejectPush(false);

        assertSame(failure, assertThrows(RuntimeException.class,
                () -> fixture.estimator.setPose(new Pose2d(20.0, 0.0, 0.0))));

        assertSame(before, fixture.estimator.getEstimate());
        assertSameStats(beforeStats, fixture.estimator.getCorrectionStats());
        assertEquals(segment, fixture.estimator.trajectorySegmentId());
        fixture.predictor.pushFailure = null;
        fixture.predictorOnlyAfter(0.5, 2.0);
        assertEstimate(fixture.estimator, 2.0, 0.6);
        fixture.predictorOnlyAfter(1.5, 2.0);
        assertEstimate(fixture.estimator, 2.0, PREDICTOR_QUALITY);
    }

    @Test
    public void clockEpochResetMasksOldQualityWithoutErasingHistoricalAcceptance() {
        Fixture fixture = new Fixture();
        fixture.acceptInitialCorrection(0.8);
        CorrectionStats before = fixture.estimator.getCorrectionStats();
        long segment = fixture.estimator.trajectorySegmentId();
        fixture.time.clock().reset(fixture.time.clock().nowSec());
        fixture.predictor.publish(2.0, PREDICTOR_QUALITY, fixture.now());
        // Leave the correction's exact pre-reset timestamp in place.

        fixture.estimator.update(fixture.time.clock());

        assertEstimate(fixture.estimator, 2.0, PREDICTOR_QUALITY);
        CorrectionStats after = fixture.estimator.getCorrectionStats();
        assertEquals(before.acceptedCorrectionCount, after.acceptedCorrectionCount);
        assertEquals(before.rejectedCorrectionCount, after.rejectedCorrectionCount);
        assertSame(before.lastCorrectionAccepted, after.lastCorrectionAccepted);
        assertSame(before.lastAcceptedCorrectionMeasurementTimestamp,
                after.lastAcceptedCorrectionMeasurementTimestamp);
        assertTrue(Double.isNaN(after.lastCorrectionAccepted.ageSec(fixture.time.clock())));
        assertEquals(segment, fixture.estimator.trajectorySegmentId());

        fixture.publishCorrectionAfter(0.1, 2.0, 2.0, 0.3);
        fixture.estimator.update(fixture.time.clock());
        assertEstimate(fixture.estimator, 2.0, 0.3);
        assertEquals(2, fixture.estimator.getAcceptedCorrectionCount());
    }

    @Test
    public void unexpectedPredictorRebaseDropsOldHoldAndDoesNotAcceptThatCyclesCorrection() {
        Fixture fixture = new Fixture();
        fixture.acceptInitialCorrection(0.8);
        long beforeSegment = fixture.estimator.trajectorySegmentId();
        fixture.predictor.setPose(new Pose2d(50.0, 0.0, 0.0));
        fixture.publishCorrectionAfter(0.25, 50.0, 50.0, 1.0);

        fixture.estimator.update(fixture.time.clock());

        assertEstimate(fixture.estimator, 50.0, PREDICTOR_QUALITY);
        assertEquals(beforeSegment + 1L, fixture.estimator.trajectorySegmentId());
        assertEquals(1, fixture.estimator.getAcceptedCorrectionCount());
        assertFalse(fixture.estimator.getLastCorrectionAccepted().isAvailable());
        fixture.publishCorrectionAfter(0.25, 50.0, 50.0, 0.2);
        fixture.estimator.update(fixture.time.clock());
        assertEstimate(fixture.estimator, 50.0, 0.2);
        assertEquals(2, fixture.estimator.getAcceptedCorrectionCount());
    }

    @Test
    public void transactionalInitializationPushFailureCannotPublishOrRetainCandidateQuality() {
        Fixture fixture = new Fixture();
        fixture.predictor.publish(0.0, PREDICTOR_QUALITY, fixture.now());
        fixture.correction.publish(2.0, 0.95, fixture.now());
        PoseEstimate before = fixture.estimator.getEstimate();
        CorrectionStats beforeStats = fixture.estimator.getCorrectionStats();
        RuntimeException failure = fixture.predictor.rejectPush(false);

        fixture.assertFailedUpdateIsCached(failure);

        assertSame(before, fixture.estimator.getEstimate());
        assertSameStats(beforeStats, fixture.estimator.getCorrectionStats());
        assertEquals(0L, fixture.estimator.trajectorySegmentId());
        fixture.predictor.pushFailure = null;
        fixture.predictorOnlyAfter(0.5, 0.0);
        assertEstimate(fixture.estimator, 0.0, PREDICTOR_QUALITY);
        assertEquals(0, fixture.estimator.getAcceptedCorrectionCount());
    }

    @Test
    public void transactionalOrdinaryPushFailureRollsBackQualityAndDoesNotRestartTheHold() {
        Fixture fixture = new Fixture();
        fixture.acceptInitialCorrection(0.8);
        PoseEstimate before = fixture.estimator.getEstimate();
        CorrectionStats beforeStats = fixture.estimator.getCorrectionStats();
        long segment = fixture.estimator.trajectorySegmentId();
        fixture.publishCorrectionAfter(0.5, 2.0, 4.0, 0.95);
        RuntimeException failure = fixture.predictor.rejectPush(false);

        fixture.assertFailedUpdateIsCached(failure);

        assertSame(before, fixture.estimator.getEstimate());
        assertSameStats(beforeStats, fixture.estimator.getCorrectionStats());
        assertEquals(segment, fixture.estimator.trajectorySegmentId());
        fixture.predictor.pushFailure = null;
        fixture.predictorOnlyAfter(0.5, 2.0);
        assertEstimate(fixture.estimator, 2.0, 0.4);
        assertSame(beforeStats.lastCorrectionAccepted, fixture.estimator.getLastCorrectionAccepted());
        fixture.predictorOnlyAfter(1.0, 2.0);
        assertEstimate(fixture.estimator, 2.0, PREDICTOR_QUALITY);
    }

    @Test
    public void nontransactionalInitializationPushFailureFailsClosedWithoutQualityLeak() {
        Fixture fixture = new Fixture();
        fixture.predictor.publish(0.0, PREDICTOR_QUALITY, fixture.now());
        fixture.correction.publish(2.0, 0.95, fixture.now());
        RuntimeException failure = fixture.predictor.rejectPush(true);

        fixture.assertFailedUpdateIsCached(failure);

        assertFalse(fixture.estimator.getEstimate().hasPose);
        assertEquals(0.0, fixture.estimator.getEstimate().quality, EPSILON);
        assertEquals(1L, fixture.estimator.trajectorySegmentId());
        assertEquals(0, fixture.estimator.getAcceptedCorrectionCount());
        assertFalse(fixture.estimator.getLastCorrectionAccepted().isAvailable());
        fixture.recoverWithoutCorrection(0);
    }

    @Test
    public void nontransactionalOrdinaryPushFailureClearsPreviouslyAcceptedQuality() {
        Fixture fixture = new Fixture();
        fixture.acceptInitialCorrection(0.8);
        long segment = fixture.estimator.trajectorySegmentId();
        fixture.publishCorrectionAfter(0.25, 2.0, 4.0, 0.95);
        RuntimeException failure = fixture.predictor.rejectPush(true);

        fixture.assertFailedUpdateIsCached(failure);

        assertFalse(fixture.estimator.getEstimate().hasPose);
        assertEquals(0.0, fixture.estimator.getEstimate().quality, EPSILON);
        assertEquals(segment + 1L, fixture.estimator.trajectorySegmentId());
        assertEquals(1, fixture.estimator.getAcceptedCorrectionCount());
        assertFalse(fixture.estimator.getLastCorrectionAccepted().isAvailable());
        fixture.recoverWithoutCorrection(1);
    }

    @Test
    public void nontransactionalManualPushFailureCannotRestoreAnOldCorrectionBoost() {
        Fixture fixture = new Fixture();
        fixture.acceptInitialCorrection(0.8);
        long segment = fixture.estimator.trajectorySegmentId();
        RuntimeException failure = fixture.predictor.rejectPush(true);

        assertSame(failure, assertThrows(RuntimeException.class,
                () -> fixture.estimator.setPose(new Pose2d(20.0, 0.0, 0.0))));

        assertFalse(fixture.estimator.getEstimate().hasPose);
        assertEquals(segment + 1L, fixture.estimator.trajectorySegmentId());
        assertEquals(1, fixture.estimator.getAcceptedCorrectionCount());
        assertFalse(fixture.estimator.getLastCorrectionAccepted().isAvailable());
        fixture.recoverWithoutCorrection(1);
    }

    @Test
    public void sourceFailureAfterPredictorDiscontinuityClearsPriorQualityBeforeRecovery() {
        for (boolean predictorFails : new boolean[]{true, false}) {
            Fixture fixture = new Fixture();
            fixture.acceptInitialCorrection(0.8);
            fixture.publishCorrectionAfter(0.25, 2.0, 2.0, 0.95);
            long segment = fixture.estimator.trajectorySegmentId();
            RuntimeException failure = new IllegalStateException("source failed after rebase");
            Runnable failAfterRebase = () -> {
                fixture.predictor.segment++;
                throw failure;
            };
            if (predictorFails) fixture.predictor.duringUpdate = failAfterRebase;
            else fixture.correction.duringUpdate = failAfterRebase;

            fixture.assertFailedUpdateIsCached(failure);

            assertFalse(fixture.estimator.getEstimate().hasPose);
            assertEquals(segment + 1L, fixture.estimator.trajectorySegmentId());
            assertFalse(fixture.estimator.getLastCorrectionAccepted().isAvailable());
            fixture.predictor.duringUpdate = null;
            fixture.correction.duringUpdate = null;
            fixture.predictorOnlyAfter(0.25, 2.0);
            assertEstimate(fixture.estimator, 2.0, PREDICTOR_QUALITY);
            assertEquals(1, fixture.estimator.getAcceptedCorrectionCount());
        }
    }

    @Test
    public void successfulPushAndSameCycleRepeatKeepOneExactQualitySnapshotAndEffect() {
        Fixture fixture = new Fixture();
        fixture.acceptInitialCorrection(0.8);
        fixture.publishCorrectionAfter(0.25, 2.0, 4.0, 0.4);
        fixture.estimator.update(fixture.time.clock());
        // Existing position gain 0.5 times measurement quality 0.4 applies 20% of the 2-inch gap.
        assertEstimate(fixture.estimator, 2.4, 0.4);
        PoseEstimate accepted = fixture.estimator.getEstimate();
        CorrectionStats stats = fixture.estimator.getCorrectionStats();
        int pushes = fixture.predictor.pushes;
        int predictorUpdates = fixture.predictor.updates;
        int correctionUpdates = fixture.correction.updates;
        fixture.correction.publish(100.0, 1.0, fixture.now());

        fixture.estimator.update(fixture.time.clock());

        assertSame(accepted, fixture.estimator.getEstimate());
        assertSameStats(stats, fixture.estimator.getCorrectionStats());
        assertEquals(pushes, fixture.predictor.pushes);
        assertEquals(predictorUpdates, fixture.predictor.updates);
        assertEquals(correctionUpdates, fixture.correction.updates);
        assertEquals(0L, fixture.estimator.trajectorySegmentId());
        fixture.predictorOnlyAfter(0.5, 2.4);
        assertEstimate(fixture.estimator, 2.4, 0.3);
    }

    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock(10.0);
        final RecordingPredictor predictor = new RecordingPredictor();
        final RecordingCorrection correction = new RecordingCorrection();
        final OdometryCorrectionFusionEstimator estimator;

        Fixture() {
            OdometryCorrectionFusionEstimator.Config config =
                    OdometryCorrectionFusionEstimator.Config.defaults();
            config.correctionConfidenceHoldSec = HOLD_SEC;
            config.correctionPositionGain = 0.5;
            config.correctionHeadingGain = 0.25;
            config.maxCorrectionAgeSec = 5.0;
            config.predictorHistorySec = 5.0;
            config.enableLatencyCompensation = false;
            config.enablePushCorrectedPoseToPredictor = true;
            estimator = new OdometryCorrectionFusionEstimator(predictor, correction, config);
        }

        LoopTimestamp now() { return time.clock().nowTimestamp(); }

        void acceptInitialCorrection(double quality) {
            predictor.publish(0.0, PREDICTOR_QUALITY, now());
            correction.publish(2.0, quality, now());
            estimator.update(time.clock());
            assertEstimate(estimator, 2.0, quality);
            assertEquals(1, estimator.getAcceptedCorrectionCount());
        }

        void publishCorrectionAfter(double dtSec, double predictorX, double correctionX,
                double quality) {
            time.nextCycle(dtSec);
            predictor.publish(predictorX, PREDICTOR_QUALITY, now());
            correction.publish(correctionX, quality, now());
        }

        void predictorOnlyAfter(double dtSec, double predictorX) {
            time.nextCycle(dtSec);
            predictor.publish(predictorX, PREDICTOR_QUALITY, now());
            correction.publishNone(now());
            estimator.update(time.clock());
        }

        void assertFailedUpdateIsCached(RuntimeException expected) {
            assertSame(expected, assertThrows(RuntimeException.class,
                    () -> estimator.update(time.clock())));
            int predictorUpdates = predictor.updates;
            int correctionUpdates = correction.updates;
            int pushes = predictor.pushes;
            PoseEstimate failedSnapshot = estimator.getEstimate();
            assertSame(expected, assertThrows(RuntimeException.class,
                    () -> estimator.update(time.clock())));
            assertSame(failedSnapshot, estimator.getEstimate());
            assertEquals(predictorUpdates, predictor.updates);
            assertEquals(correctionUpdates, correction.updates);
            assertEquals(pushes, predictor.pushes);
        }

        void recoverWithoutCorrection(int acceptedBeforeFailure) {
            predictor.pushFailure = null;
            time.nextCycle(0.25);
            predictor.publishNone(now());
            correction.publish(9.0, 1.0, now());
            estimator.update(time.clock());
            assertFalse("a fresh predictor base is required after the failed rebase",
                    estimator.getEstimate().hasPose);
            assertEquals(acceptedBeforeFailure, estimator.getAcceptedCorrectionCount());
            predictorOnlyAfter(0.25, 9.0);
            assertEstimate(estimator, 9.0, PREDICTOR_QUALITY);
            assertEquals(acceptedBeforeFailure, estimator.getAcceptedCorrectionCount());
        }
    }

    private static final class RecordingPredictor implements MotionPredictor, PoseResetter {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());
        int updates;
        int pushes;
        long segment;
        RuntimeException pushFailure;
        boolean failAfterEffect;
        Runnable duringUpdate;

        void publish(double xInches, double quality, LoopTimestamp timestamp) {
            estimate = new PoseEstimate(pose(xInches), true, quality, timestamp);
            delta = MotionDelta.none(timestamp);
        }

        void publishNone(LoopTimestamp timestamp) {
            estimate = PoseEstimate.noPose(timestamp);
            delta = MotionDelta.none(timestamp);
        }

        RuntimeException rejectPush(boolean afterEffect) {
            failAfterEffect = afterEffect;
            pushFailure = new IllegalStateException(afterEffect
                    ? "predictor failed after changing its segment" : "predictor rejected candidate");
            return pushFailure;
        }

        @Override public void update(LoopClock clock) {
            updates++;
            if (duringUpdate != null) duringUpdate.run();
        }

        @Override public PoseEstimate getEstimate() { return estimate; }
        @Override public MotionDelta getLatestMotionDelta() { return delta; }
        @Override public long trajectorySegmentId() { return segment; }

        @Override public void setPose(Pose2d value) {
            pushes++;
            if (pushFailure != null && !failAfterEffect) throw pushFailure;
            segment++;
            LoopTimestamp timestamp = estimate.timestamp;
            if (pushFailure != null) {
                publishNone(timestamp);
                throw pushFailure;
            }
            double quality = estimate.hasPose ? estimate.quality : PREDICTOR_QUALITY;
            estimate = new PoseEstimate(new Pose3d(value.xInches, value.yInches, 0.0,
                    value.headingRad, 0.0, 0.0), true, quality, timestamp);
            delta = MotionDelta.none(timestamp);
        }
    }

    private static final class RecordingCorrection implements AbsolutePoseEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        int updates;
        Runnable duringUpdate;

        void publish(double xInches, double quality, LoopTimestamp timestamp) {
            estimate = new PoseEstimate(pose(xInches), true, quality, timestamp);
        }

        void publishNone(LoopTimestamp timestamp) { estimate = PoseEstimate.noPose(timestamp); }

        @Override public void update(LoopClock clock) {
            updates++;
            if (duringUpdate != null) duringUpdate.run();
        }

        @Override public PoseEstimate getEstimate() { return estimate; }
    }

    private static Pose3d pose(double xInches) {
        return new Pose3d(xInches, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    private static void assertEstimate(OdometryCorrectionFusionEstimator estimator,
            double xInches, double quality) {
        PoseEstimate estimate = estimator.getEstimate();
        assertTrue(estimate.hasPose);
        assertEquals(xInches, estimate.fieldToRobotPose.xInches, EPSILON);
        assertEquals(0.0, estimate.fieldToRobotPose.yInches, EPSILON);
        assertEquals(0.0, estimate.fieldToRobotPose.yawRad, EPSILON);
        assertEquals(quality, estimate.quality, EPSILON);
    }

    private static void assertSameStats(CorrectionStats expected, CorrectionStats actual) {
        assertEquals(expected.acceptedCorrectionCount, actual.acceptedCorrectionCount);
        assertEquals(expected.rejectedCorrectionCount, actual.rejectedCorrectionCount);
        assertEquals(expected.skippedDuplicateCorrectionCount, actual.skippedDuplicateCorrectionCount);
        assertEquals(expected.skippedOutOfOrderCorrectionCount, actual.skippedOutOfOrderCorrectionCount);
        assertEquals(expected.replayedCorrectionCount, actual.replayedCorrectionCount);
        assertEquals(expected.projectedCorrectionCount, actual.projectedCorrectionCount);
        assertSame(expected.lastCorrectionAccepted, actual.lastCorrectionAccepted);
        assertSame(expected.lastAcceptedCorrectionMeasurementTimestamp,
                actual.lastAcceptedCorrectionMeasurementTimestamp);
        assertSame(expected.lastEvaluatedCorrectionTimestamp, actual.lastEvaluatedCorrectionTimestamp);
        assertEquals(expected.lastCorrectionUsedReplay, actual.lastCorrectionUsedReplay);
    }
}
