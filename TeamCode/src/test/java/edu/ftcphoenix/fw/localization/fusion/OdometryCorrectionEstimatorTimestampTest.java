package edu.ftcphoenix.fw.localization.fusion;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.MotionDelta;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Regression coverage for epoch-safe correction and predictor history. */
public final class OdometryCorrectionEstimatorTimestampTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void lightweightFusionClassifiesDuplicateAndOutOfOrderTypedMeasurements() {
        EstimatorFixture fixture = fusionFixture(10.0);

        verifyDuplicateAndOutOfOrderClassification(fixture);
    }

    @Test
    public void ekfClassifiesDuplicateAndOutOfOrderTypedMeasurements() {
        EstimatorFixture fixture = ekfFixture(10.0);

        verifyDuplicateAndOutOfOrderClassification(fixture);
    }

    @Test
    public void lightweightFusionReplaysAnInterpolatedDelayedCorrection() {
        EstimatorFixture fixture = fusionFixture(0.0);
        initializeFromPredictor(fixture, 0.0);

        LoopTimestamp t0 = fixture.predictor.estimate.timestamp;
        LoopTimestamp t1 = advancePredictor(fixture, 1.0, 10.0, t0, 10.0);

        fixture.time.nextCycle(1.0);
        LoopTimestamp t2 = fixture.time.clock().nowTimestamp();
        LoopTimestamp measurementTimestamp = fixture.time.clock().timestampSecondsAgo(0.5);
        fixture.predictor.publish(20.0, t2, motion(10.0, t1, t2));
        fixture.correction.publish(17.0, measurementTimestamp);

        fixture.estimator.update(fixture.time.clock());

        CorrectionStats stats = fixture.estimator.getCorrectionStats();
        assertEquals(1, stats.acceptedCorrectionCount);
        assertEquals(1, stats.replayedCorrectionCount);
        assertEquals(0, stats.projectedCorrectionCount);
        assertTrue(stats.lastCorrectionUsedReplay);
        assertSame(measurementTimestamp, stats.lastAcceptedCorrectionMeasurementTimestamp);
        // Predictor interpolation reconstructs x=15 at the half-second frame. Correct to x=17,
        // then replay the remaining five inches of odometry to arrive at x=22 now.
        assertEquals(22.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
    }

    @Test
    public void ekfReplaysAnInterpolatedDelayedCorrection() {
        EstimatorFixture fixture = ekfFixture(0.0);
        initializeFromPredictor(fixture, 0.0);

        LoopTimestamp t0 = fixture.predictor.estimate.timestamp;
        LoopTimestamp t1 = advancePredictor(fixture, 1.0, 10.0, t0, 10.0);

        fixture.time.nextCycle(1.0);
        LoopTimestamp t2 = fixture.time.clock().nowTimestamp();
        LoopTimestamp measurementTimestamp = fixture.time.clock().timestampSecondsAgo(0.5);
        fixture.predictor.publish(20.0, t2, motion(10.0, t1, t2));
        fixture.correction.publish(17.0, measurementTimestamp);

        fixture.estimator.update(fixture.time.clock());

        OdometryCorrectionEkfEstimator estimator =
                (OdometryCorrectionEkfEstimator) fixture.estimator;
        CorrectionStats stats = estimator.getCorrectionStats();
        assertEquals(1, stats.acceptedCorrectionCount);
        assertEquals(1, stats.replayedCorrectionCount);
        assertEquals(0, stats.projectedCorrectionCount);
        assertTrue(stats.lastCorrectionUsedReplay);
        assertSame(measurementTimestamp, stats.lastAcceptedCorrectionMeasurementTimestamp);
        // The replay comparison must occur at interpolated x=15, so the x=17 frame has a
        // two-inch innovation. Comparing the delayed frame directly with x=20 would report 3.
        assertEquals(2.0, estimator.getLastInnovationPositionIn(), EPSILON);
        assertTrue(estimator.getEstimate().fieldToRobotPose.xInches > 20.0);
        assertTrue(estimator.getEstimate().fieldToRobotPose.xInches <= 22.0);
    }

    @Test
    public void lightweightFusionRejectsPriorEpochStateAndAcceptsSameValueNewEpochMeasurement() {
        verifySameValueResetInvalidation(fusionFixture(5.0));
    }

    @Test
    public void ekfRejectsPriorEpochStateAndAcceptsSameValueNewEpochMeasurement() {
        verifySameValueResetInvalidation(ekfFixture(5.0));
    }

    private static void verifyDuplicateAndOutOfOrderClassification(EstimatorFixture fixture) {
        initializeFromPredictor(fixture, 0.0);

        fixture.time.nextCycle(0.10);
        LoopTimestamp acceptedTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(0.0, acceptedTimestamp, MotionDelta.none(acceptedTimestamp));
        fixture.correction.publish(1.0, acceptedTimestamp);
        fixture.estimator.update(fixture.time.clock());

        fixture.time.nextCycle(0.10);
        LoopTimestamp duplicateLoopTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                0.0,
                duplicateLoopTimestamp,
                MotionDelta.none(duplicateLoopTimestamp)
        );
        fixture.estimator.update(fixture.time.clock());

        fixture.time.nextCycle(0.01);
        LoopTimestamp outOfOrderLoopTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                0.0,
                outOfOrderLoopTimestamp,
                MotionDelta.none(outOfOrderLoopTimestamp)
        );
        fixture.correction.publish(
                1.0,
                fixture.time.clock().timestampSecondsAgo(0.16)
        );
        fixture.estimator.update(fixture.time.clock());

        CorrectionStats stats = fixture.estimator.getCorrectionStats();
        assertEquals(1, stats.acceptedCorrectionCount);
        assertEquals(0, stats.rejectedCorrectionCount);
        assertEquals(1, stats.skippedDuplicateCorrectionCount);
        assertEquals(1, stats.skippedOutOfOrderCorrectionCount);
        assertSame(acceptedTimestamp, stats.lastEvaluatedCorrectionTimestamp);
    }

    private static void verifySameValueResetInvalidation(EstimatorFixture fixture) {
        initializeFromPredictor(fixture, 0.0);

        fixture.time.nextCycle(0.10);
        LoopTimestamp priorEpochTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                0.0,
                priorEpochTimestamp,
                MotionDelta.none(priorEpochTimestamp)
        );
        fixture.correction.publish(1.0, priorEpochTimestamp);
        fixture.estimator.update(fixture.time.clock());
        assertEquals(1, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);

        // Reset to the exact same numeric clock value. Only the epoch distinguishes the retained
        // pre-reset measurement from a valid post-reset measurement.
        fixture.time.clock().reset(fixture.time.clock().nowSec());
        LoopTimestamp firstNewEpochTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                0.0,
                firstNewEpochTimestamp,
                MotionDelta.none(firstNewEpochTimestamp)
        );
        fixture.correction.publish(1.0, priorEpochTimestamp);
        fixture.estimator.update(fixture.time.clock());

        CorrectionStats afterRetainedMeasurement = fixture.estimator.getCorrectionStats();
        assertEquals(1, afterRetainedMeasurement.acceptedCorrectionCount);
        assertEquals(0, afterRetainedMeasurement.rejectedCorrectionCount);
        assertEquals(0, afterRetainedMeasurement.skippedDuplicateCorrectionCount);
        assertEquals(0, afterRetainedMeasurement.skippedOutOfOrderCorrectionCount);
        assertFalse(afterRetainedMeasurement.lastEvaluatedCorrectionTimestamp.isAvailable());
        assertTrue(Double.isNaN(firstNewEpochTimestamp.secondsSince(priorEpochTimestamp)));

        fixture.time.nextCycle(0.0);
        LoopTimestamp sameValueNewEpochTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                0.0,
                sameValueNewEpochTimestamp,
                MotionDelta.none(sameValueNewEpochTimestamp)
        );
        fixture.correction.publish(1.0, sameValueNewEpochTimestamp);
        fixture.estimator.update(fixture.time.clock());

        CorrectionStats afterNewMeasurement = fixture.estimator.getCorrectionStats();
        assertEquals(2, afterNewMeasurement.acceptedCorrectionCount);
        assertEquals(0, afterNewMeasurement.rejectedCorrectionCount);
        assertEquals(0, afterNewMeasurement.skippedDuplicateCorrectionCount);
        assertEquals(0, afterNewMeasurement.skippedOutOfOrderCorrectionCount);
        assertSame(sameValueNewEpochTimestamp, afterNewMeasurement.lastEvaluatedCorrectionTimestamp);
        assertSame(
                sameValueNewEpochTimestamp,
                afterNewMeasurement.lastAcceptedCorrectionMeasurementTimestamp
        );
    }

    private static void initializeFromPredictor(EstimatorFixture fixture, double xInches) {
        LoopTimestamp timestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(xInches, timestamp, MotionDelta.none(timestamp));
        fixture.correction.publishNone(timestamp);
        fixture.estimator.update(fixture.time.clock());
        assertTrue(fixture.estimator.getEstimate().hasPose);
    }

    private static LoopTimestamp advancePredictor(EstimatorFixture fixture,
                                                  double dtSec,
                                                  double xInches,
                                                  LoopTimestamp startTimestamp,
                                                  double deltaXInches) {
        fixture.time.nextCycle(dtSec);
        LoopTimestamp endTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                xInches,
                endTimestamp,
                motion(deltaXInches, startTimestamp, endTimestamp)
        );
        fixture.correction.publishNone(endTimestamp);
        fixture.estimator.update(fixture.time.clock());
        return endTimestamp;
    }

    private static MotionDelta motion(double xInches,
                                      LoopTimestamp startTimestamp,
                                      LoopTimestamp endTimestamp) {
        return new MotionDelta(
                pose(xInches),
                true,
                1.0,
                startTimestamp,
                endTimestamp
        );
    }

    private static EstimatorFixture fusionFixture(double initialTimeSec) {
        EstimatorFixture fixture = new EstimatorFixture(initialTimeSec);
        OdometryCorrectionFusionEstimator.Config config =
                OdometryCorrectionFusionEstimator.Config.defaults();
        config.maxCorrectionAgeSec = 2.0;
        config.predictorHistorySec = 2.0;
        config.correctionPositionGain = 1.0;
        config.correctionHeadingGain = 1.0;
        config.maxCorrectionPositionJumpIn = 1_000.0;
        config.enablePushCorrectedPoseToPredictor = false;
        fixture.estimator = new OdometryCorrectionFusionEstimator(
                fixture.predictor,
                fixture.correction,
                config
        );
        return fixture;
    }

    private static EstimatorFixture ekfFixture(double initialTimeSec) {
        EstimatorFixture fixture = new EstimatorFixture(initialTimeSec);
        OdometryCorrectionEkfEstimator.Config config =
                OdometryCorrectionEkfEstimator.Config.defaults();
        config.maxCorrectionAgeSec = 2.0;
        config.predictorHistorySec = 2.0;
        config.maxCorrectionPositionInnovationIn = 1_000.0;
        config.maxCorrectionHeadingInnovationRad = Math.PI;
        config.maxCorrectionMahalanobisSq = 1_000_000.0;
        config.enablePushCorrectedPoseToPredictor = false;
        fixture.estimator = new OdometryCorrectionEkfEstimator(
                fixture.predictor,
                fixture.correction,
                config
        );
        return fixture;
    }

    private static Pose3d pose(double xInches) {
        return new Pose3d(xInches, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    private static final class EstimatorFixture {
        final ManualLoopClock time;
        final FakeMotionPredictor predictor = new FakeMotionPredictor();
        final FakeAbsolutePoseEstimator correction = new FakeAbsolutePoseEstimator();
        CorrectedPoseEstimator estimator;

        EstimatorFixture(double initialTimeSec) {
            time = new ManualLoopClock(initialTimeSec);
        }
    }

    private static final class FakeMotionPredictor implements MotionPredictor {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());

        void publish(double xInches, LoopTimestamp timestamp, MotionDelta nextDelta) {
            estimate = new PoseEstimate(pose(xInches), true, 1.0, timestamp);
            delta = nextDelta;
        }

        @Override
        public void update(LoopClock clock) {
            // Values are published explicitly by each deterministic test cycle.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }

        @Override
        public MotionDelta getLatestMotionDelta() {
            return delta;
        }
    }

    private static final class FakeAbsolutePoseEstimator implements AbsolutePoseEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());

        void publish(double xInches, LoopTimestamp timestamp) {
            estimate = new PoseEstimate(pose(xInches), true, 1.0, timestamp);
        }

        void publishNone(LoopTimestamp timestamp) {
            estimate = PoseEstimate.noPose(timestamp);
        }

        @Override
        public void update(LoopClock clock) {
            // Values are published explicitly by each deterministic test cycle.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }
}
