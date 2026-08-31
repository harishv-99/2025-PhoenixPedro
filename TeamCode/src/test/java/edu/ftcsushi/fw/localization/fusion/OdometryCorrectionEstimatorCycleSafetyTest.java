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
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Regression coverage for corrected-localizer cycle and predictor-sample ownership. */
public final class OdometryCorrectionEstimatorCycleSafetyTest {

    private static final double EPSILON = 1e-9;

    private enum Kind {
        FUSION,
        EKF
    }

    @Test
    public void repeatedSameCycleUpdateIsAnExactNoOp() {
        verifyRepeatedSameCycleUpdateIsAnExactNoOp(Kind.FUSION);
        verifyRepeatedSameCycleUpdateIsAnExactNoOp(Kind.EKF);
    }

    @Test
    public void retainedDeltaIsNotAppliedInALaterCycleButFreshDeltaIsApplied() {
        verifyRetainedDeltaIsNotAppliedInALaterCycleButFreshDeltaIsApplied(Kind.FUSION);
        verifyRetainedDeltaIsNotAppliedInALaterCycleButFreshDeltaIsApplied(Kind.EKF);
    }

    @Test
    public void outOfOrderMotionIsSkippedWithoutBlockingCorrection() {
        verifyOutOfOrderMotionIsSkippedWithoutBlockingCorrection(Kind.FUSION);
        verifyOutOfOrderMotionIsSkippedWithoutBlockingCorrection(Kind.EKF);
    }

    @Test
    public void duplicateMotionDoesNotBlockANewerCorrection() {
        verifyDuplicateMotionDoesNotBlockANewerCorrection(Kind.FUSION);
        verifyDuplicateMotionDoesNotBlockANewerCorrection(Kind.EKF);
    }

    @Test
    public void duplicatePredictorTimestampPreservesDelayedReplayHistory() {
        verifyDuplicatePredictorTimestampPreservesDelayedReplayHistory(Kind.FUSION);
        verifyDuplicatePredictorTimestampPreservesDelayedReplayHistory(Kind.EKF);
    }

    @Test
    public void newClockEpochMayConsumeNewEpochMotionAtTheSameNumericTime() {
        verifyNewClockEpochMayConsumeNewEpochMotionAtTheSameNumericTime(Kind.FUSION);
        verifyNewClockEpochMayConsumeNewEpochMotionAtTheSameNumericTime(Kind.EKF);
    }

    @Test
    public void failedCycleRethrowsTheOriginalFailureWithoutRetryingChildren() {
        verifyFailedCycleRethrowsTheOriginalFailureWithoutRetryingChildren(Kind.FUSION);
        verifyFailedCycleRethrowsTheOriginalFailureWithoutRetryingChildren(Kind.EKF);
    }

    @Test
    public void reentrantUpdateFailsFastAndTheNextCycleRemainsEligible() {
        verifyReentrantUpdateFailsFastAndTheNextCycleRemainsEligible(Kind.FUSION);
        verifyReentrantUpdateFailsFastAndTheNextCycleRemainsEligible(Kind.EKF);
    }

    @Test
    public void setPosePublishesImmediatelyWithoutReopeningOrReplayingTheCycle() {
        verifySetPosePublishesImmediatelyWithoutReopeningOrReplayingTheCycle(Kind.FUSION);
        verifySetPosePublishesImmediatelyWithoutReopeningOrReplayingTheCycle(Kind.EKF);
    }

    @Test
    public void manualRebaseExcludesPreAnchorMotionFromAnAccumulatedDelta() {
        verifyManualRebaseExcludesPreAnchorMotionFromAnAccumulatedDelta(Kind.FUSION);
        verifyManualRebaseExcludesPreAnchorMotionFromAnAccumulatedDelta(Kind.EKF);
    }

    @Test
    public void correctionRebaseExcludesPreAnchorMotionFromAnAccumulatedDelta() {
        verifyCorrectionRebaseExcludesPreAnchorMotionFromAnAccumulatedDelta(Kind.FUSION);
        verifyCorrectionRebaseExcludesPreAnchorMotionFromAnAccumulatedDelta(Kind.EKF);
    }

    @Test
    public void anchorWithoutPredictorPoseSkipsTheUnpartitionableFirstInterval() {
        verifyAnchorWithoutPredictorPoseSkipsTheUnpartitionableFirstInterval(Kind.FUSION);
        verifyAnchorWithoutPredictorPoseSkipsTheUnpartitionableFirstInterval(Kind.EKF);
    }

    @Test
    public void awaitingAnchorDoesNotMoveCoverageBackToAnOlderPredictorPose() {
        verifyAwaitingAnchorDoesNotMoveCoverageBackToAnOlderPredictorPose(Kind.FUSION);
        verifyAwaitingAnchorDoesNotMoveCoverageBackToAnOlderPredictorPose(Kind.EKF);
    }

    @Test
    public void stalePredictorAtCorrectionAnchorCannotReplayItsPreAnchorPrefix() {
        verifyStalePredictorAtCorrectionAnchorCannotReplayItsPreAnchorPrefix(Kind.FUSION);
        verifyStalePredictorAtCorrectionAnchorCannotReplayItsPreAnchorPrefix(Kind.EKF);
    }

    @Test
    public void strictlyPositiveSubMicrosecondMotionIsNotLost() {
        verifyStrictlyPositiveSubMicrosecondMotionIsNotLost(Kind.FUSION);
        verifyStrictlyPositiveSubMicrosecondMotionIsNotLost(Kind.EKF);
    }

    @Test
    public void pushEnabledCorrectionRebaseIsAttemptedOnlyOncePerCycle() {
        verifyPushEnabledCorrectionRebaseIsAttemptedOnlyOncePerCycle(Kind.FUSION);
        verifyPushEnabledCorrectionRebaseIsAttemptedOnlyOncePerCycle(Kind.EKF);
    }

    @Test
    public void pushEnabledManualRebasePublishesAndPushesOnlyOnce() {
        verifyPushEnabledManualRebasePublishesAndPushesOnlyOnce(Kind.FUSION);
        verifyPushEnabledManualRebasePublishesAndPushesOnlyOnce(Kind.EKF);
    }

    @Test
    public void manualAnchorStartsExactlyOneCorrectedTrajectorySegment() {
        verifyManualAnchorStartsExactlyOneCorrectedTrajectorySegment(Kind.FUSION);
        verifyManualAnchorStartsExactlyOneCorrectedTrajectorySegment(Kind.EKF);
    }

    @Test
    public void expectedCorrectionPushStaysWithinTheCorrectedTrajectory() {
        verifyExpectedCorrectionPushStaysWithinTheCorrectedTrajectory(Kind.FUSION);
        verifyExpectedCorrectionPushStaysWithinTheCorrectedTrajectory(Kind.EKF);
    }

    @Test
    public void unexpectedPredictorRebaseStartsANewSegmentAndDropsCrossingMotion() {
        verifyUnexpectedPredictorRebaseStartsANewSegmentAndDropsCrossingMotion(Kind.FUSION);
        verifyUnexpectedPredictorRebaseStartsANewSegmentAndDropsCrossingMotion(Kind.EKF);
    }

    @Test
    public void nontransactionalManualPushFailureStartsANewFailClosedSegment() {
        verifyNontransactionalManualPushFailureStartsANewFailClosedSegment(Kind.FUSION);
        verifyNontransactionalManualPushFailureStartsANewFailClosedSegment(Kind.EKF);
    }

    @Test
    public void rejectedManualAnchorStillObservesAPreexistingPredictorRebase() {
        verifyRejectedManualAnchorStillObservesAPreexistingPredictorRebase(Kind.FUSION);
        verifyRejectedManualAnchorStillObservesAPreexistingPredictorRebase(Kind.EKF);
    }

    @Test
    public void nontransactionalCorrectionPushFailureStartsANewFailClosedSegment() {
        verifyNontransactionalCorrectionPushFailureStartsANewFailClosedSegment(Kind.FUSION);
        verifyNontransactionalCorrectionPushFailureStartsANewFailClosedSegment(Kind.EKF);
    }

    @Test
    public void correctionFailureAfterRebasingPredictorRetainsTheContinuityBarrier() {
        verifyCorrectionFailureAfterRebasingPredictorRetainsTheContinuityBarrier(Kind.FUSION);
        verifyCorrectionFailureAfterRebasingPredictorRetainsTheContinuityBarrier(Kind.EKF);
    }

    @Test
    public void clockEpochResetAloneDoesNotChangeTrajectorySegment() {
        verifyClockEpochResetAloneDoesNotChangeTrajectorySegment(Kind.FUSION);
        verifyClockEpochResetAloneDoesNotChangeTrajectorySegment(Kind.EKF);
    }

    private static void verifyRepeatedSameCycleUpdateIsAnExactNoOp(Kind kind) {
        Fixture fixture = new Fixture(kind);
        LoopTimestamp t0 = initializeFromPredictor(fixture, 0.0);

        fixture.time.nextCycle(1.0);
        LoopTimestamp t1 = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(5.0, t1, motion(5.0, t0, t1));
        fixture.correction.publish(6.0, t1);
        fixture.estimator.update(fixture.time.clock());

        PoseEstimate firstEstimate = fixture.estimator.getEstimate();
        CorrectionStats firstStats = fixture.estimator.getCorrectionStats();
        int predictorCalls = fixture.predictor.updateCalls;
        int correctionCalls = fixture.correction.updateCalls;
        double firstPositionStd = kind == Kind.EKF
                ? ((OdometryCorrectionEkfEstimator) fixture.estimator).getPositionStdIn()
                : Double.NaN;

        fixture.estimator.update(fixture.time.clock());

        assertSame(firstEstimate, fixture.estimator.getEstimate());
        assertCorrectionStatsEqual(firstStats, fixture.estimator.getCorrectionStats());
        assertEquals(predictorCalls, fixture.predictor.updateCalls);
        assertEquals(correctionCalls, fixture.correction.updateCalls);
        if (kind == Kind.EKF) {
            assertEquals(
                    firstPositionStd,
                    ((OdometryCorrectionEkfEstimator) fixture.estimator).getPositionStdIn(),
                    EPSILON
            );
        }
    }

    private static void verifyRetainedDeltaIsNotAppliedInALaterCycleButFreshDeltaIsApplied(
            Kind kind) {
        Fixture fixture = new Fixture(kind);
        LoopTimestamp t0 = initializeFromPredictor(fixture, 0.0);

        fixture.time.nextCycle(1.0);
        LoopTimestamp t1 = fixture.time.clock().nowTimestamp();
        MotionDelta firstDelta = motion(5.0, t0, t1);
        fixture.predictor.publish(5.0, t1, firstDelta);
        fixture.correction.publishNone(t1);
        fixture.estimator.update(fixture.time.clock());
        assertEquals(5.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        fixture.time.nextCycle(0.5);
        fixture.correction.publishNone(fixture.time.clock().nowTimestamp());
        fixture.estimator.update(fixture.time.clock());
        assertEquals(5.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        fixture.time.nextCycle(0.5);
        LoopTimestamp t2 = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(8.0, t2, motion(3.0, t1, t2));
        fixture.correction.publishNone(t2);
        fixture.estimator.update(fixture.time.clock());
        assertEquals(8.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
    }

    private static void verifyOutOfOrderMotionIsSkippedWithoutBlockingCorrection(Kind kind) {
        Fixture fixture = new Fixture(kind);
        initializeFromPredictor(fixture, 0.0);

        fixture.time.nextCycle(1.0);
        LoopTimestamp t1 = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(0.0, t1, MotionDelta.none(t1));
        fixture.correction.publishNone(t1);
        fixture.estimator.update(fixture.time.clock());
        assertEquals(0.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        fixture.time.nextCycle(1.0);
        LoopTimestamp oldStart = fixture.time.clock().timestampSecondsAgo(3.0);
        LoopTimestamp oldEnd = fixture.time.clock().timestampSecondsAgo(2.5);
        LoopTimestamp now = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(0.0, now, motion(50.0, oldStart, oldEnd));
        fixture.correction.publish(4.0, now);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(1, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);
        assertTrue(fixture.estimator.getEstimate().fieldToRobotPose.xInches < 10.0);
    }

    private static void verifyDuplicateMotionDoesNotBlockANewerCorrection(Kind kind) {
        Fixture fixture = new Fixture(kind);
        LoopTimestamp t0 = initializeFromPredictor(fixture, 0.0);

        fixture.time.nextCycle(1.0);
        LoopTimestamp t1 = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(5.0, t1, motion(5.0, t0, t1));
        fixture.correction.publishNone(t1);
        fixture.estimator.update(fixture.time.clock());

        fixture.time.nextCycle(0.5);
        LoopTimestamp correctionTimestamp = fixture.time.clock().nowTimestamp();
        fixture.correction.publish(9.0, correctionTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(1, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);
        assertTrue(fixture.estimator.getEstimate().fieldToRobotPose.xInches >= 5.0);
        assertTrue(fixture.estimator.getEstimate().fieldToRobotPose.xInches <= 9.0 + EPSILON);
    }

    private static void verifyDuplicatePredictorTimestampPreservesDelayedReplayHistory(Kind kind) {
        Fixture fixture = new Fixture(kind);
        LoopTimestamp t0 = initializeFromPredictor(fixture, 0.0);

        fixture.time.nextCycle(1.0);
        LoopTimestamp t1 = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(10.0, t1, motion(10.0, t0, t1));
        fixture.correction.publishNone(t1);
        fixture.estimator.update(fixture.time.clock());

        fixture.time.nextCycle(0.2);
        LoopTimestamp delayedFrameTimestamp = fixture.time.clock().timestampSecondsAgo(0.7);
        fixture.correction.publish(5.0, delayedFrameTimestamp);
        fixture.estimator.update(fixture.time.clock());

        CorrectionStats stats = fixture.estimator.getCorrectionStats();
        assertEquals(1, stats.acceptedCorrectionCount);
        assertEquals(1, stats.replayedCorrectionCount);
        assertTrue(stats.lastCorrectionUsedReplay);
    }

    private static void verifyNewClockEpochMayConsumeNewEpochMotionAtTheSameNumericTime(Kind kind) {
        Fixture fixture = new Fixture(kind);
        LoopTimestamp t0 = initializeFromPredictor(fixture, 0.0);

        fixture.time.nextCycle(1.0);
        LoopTimestamp t1 = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(5.0, t1, motion(5.0, t0, t1));
        fixture.correction.publishNone(t1);
        fixture.estimator.update(fixture.time.clock());

        fixture.time.clock().reset(fixture.time.clock().nowSec());
        LoopTimestamp newEpochEnd = fixture.time.clock().nowTimestamp();
        LoopTimestamp newEpochStart = fixture.time.clock().timestampSecondsAgo(0.5);
        fixture.predictor.publish(7.0, newEpochEnd, motion(2.0, newEpochStart, newEpochEnd));
        fixture.correction.publishNone(newEpochEnd);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(7.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
        assertFalse(Double.isFinite(newEpochEnd.secondsSince(t1)));
    }

    private static void verifyFailedCycleRethrowsTheOriginalFailureWithoutRetryingChildren(
            Kind kind) {
        Fixture fixture = new Fixture(kind);
        RuntimeException expected = new IllegalStateException("predictor failed");
        fixture.predictor.failure = expected;

        RuntimeException first = captureRuntime(
                () -> fixture.estimator.update(fixture.time.clock())
        );
        RuntimeException repeated = captureRuntime(
                () -> fixture.estimator.update(fixture.time.clock())
        );

        assertSame(expected, first);
        assertSame(first, repeated);
        assertEquals(1, fixture.predictor.updateCalls);
        assertEquals(0, fixture.correction.updateCalls);

        fixture.time.nextCycle(0.02);
        fixture.predictor.failure = null;
        LoopTimestamp recoveredTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                0.0,
                recoveredTimestamp,
                MotionDelta.none(recoveredTimestamp)
        );
        fixture.correction.publishNone(recoveredTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertTrue(fixture.estimator.getEstimate().hasPose);
        assertEquals(2, fixture.predictor.updateCalls);
        assertEquals(1, fixture.correction.updateCalls);
    }

    private static void verifyReentrantUpdateFailsFastAndTheNextCycleRemainsEligible(Kind kind) {
        Fixture fixture = new Fixture(kind);
        fixture.predictor.duringUpdate = () -> fixture.estimator.update(fixture.time.clock());

        RuntimeException first = captureRuntime(
                () -> fixture.estimator.update(fixture.time.clock())
        );
        RuntimeException repeated = captureRuntime(
                () -> fixture.estimator.update(fixture.time.clock())
        );

        assertTrue(first instanceof IllegalStateException);
        assertTrue(first.getMessage().contains("reentrantly"));
        assertSame(first, repeated);
        assertEquals(1, fixture.predictor.updateCalls);

        fixture.time.nextCycle(0.02);
        fixture.predictor.duringUpdate = null;
        LoopTimestamp recoveredTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                0.0,
                recoveredTimestamp,
                MotionDelta.none(recoveredTimestamp)
        );
        fixture.correction.publishNone(recoveredTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertTrue(fixture.estimator.getEstimate().hasPose);
        assertEquals(2, fixture.predictor.updateCalls);
    }

    private static void verifySetPosePublishesImmediatelyWithoutReopeningOrReplayingTheCycle(
            Kind kind) {
        Fixture fixture = new Fixture(kind);
        LoopTimestamp t0 = initializeFromPredictor(fixture, 0.0);

        fixture.time.nextCycle(1.0);
        LoopTimestamp t1 = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(5.0, t1, motion(5.0, t0, t1));
        fixture.correction.publish(6.0, t1);
        fixture.estimator.update(fixture.time.clock());
        assertEquals(1, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);

        fixture.estimator.setPose(new Pose2d(100.0, 0.0, 0.0));
        PoseEstimate anchored = fixture.estimator.getEstimate();
        assertTrue(anchored.hasPose);
        assertEquals(100.0, anchored.fieldToRobotPose.xInches, EPSILON);
        assertEquals(1, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);
        assertFalse(fixture.estimator.getCorrectionStats().lastCorrectionAccepted.isAvailable());
        assertFalse(fixture.estimator.getCorrectionStats()
                .lastAcceptedCorrectionMeasurementTimestamp.isAvailable());

        fixture.estimator.update(fixture.time.clock());
        assertSame(anchored, fixture.estimator.getEstimate());

        fixture.time.nextCycle(0.5);
        fixture.correction.publishNone(fixture.time.clock().nowTimestamp());
        fixture.estimator.update(fixture.time.clock());
        assertEquals(100.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
    }

    private static void verifyManualRebaseExcludesPreAnchorMotionFromAnAccumulatedDelta(
            Kind kind) {
        Fixture fixture = new Fixture(kind);
        LoopTimestamp acceptedBaseline = initializeFromPredictor(fixture, 0.0);

        fixture.time.nextCycle(0.0);
        LoopTimestamp anchorTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(5.0, anchorTimestamp, MotionDelta.none(anchorTimestamp));
        fixture.correction.publishNone(anchorTimestamp);
        fixture.estimator.update(fixture.time.clock());
        fixture.estimator.setPose(new Pose2d(100.0, 0.0, 0.0));

        // A further no-delta cycle must not move the predictor-side anchor. Its movement happened
        // after setPose and belongs in the first later positive interval.
        fixture.time.nextCycle(0.0);
        LoopTimestamp retainedTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(6.0, retainedTimestamp, MotionDelta.none(retainedTimestamp));
        fixture.correction.publishNone(retainedTimestamp);
        fixture.estimator.update(fixture.time.clock());
        assertEquals(100.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        fixture.time.nextCycle(1.0);
        LoopTimestamp laterTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                8.0,
                laterTimestamp,
                motion(8.0, acceptedBaseline, laterTimestamp)
        );
        fixture.correction.publishNone(laterTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(103.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
    }

    private static void verifyCorrectionRebaseExcludesPreAnchorMotionFromAnAccumulatedDelta(
            Kind kind) {
        Fixture fixture = new Fixture(kind);
        LoopTimestamp acceptedBaseline = fixture.time.clock().timestampSecondsAgo(1.0);
        LoopTimestamp anchorTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(5.0, anchorTimestamp, MotionDelta.none(anchorTimestamp));
        fixture.correction.publish(100.0, anchorTimestamp);
        fixture.estimator.update(fixture.time.clock());
        assertEquals(100.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        fixture.time.nextCycle(0.0);
        LoopTimestamp retainedTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(6.0, retainedTimestamp, MotionDelta.none(retainedTimestamp));
        fixture.correction.publishNone(retainedTimestamp);
        fixture.estimator.update(fixture.time.clock());
        assertEquals(100.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        fixture.time.nextCycle(1.0);
        LoopTimestamp laterTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                8.0,
                laterTimestamp,
                motion(8.0, acceptedBaseline, laterTimestamp)
        );
        fixture.correction.publishNone(laterTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(103.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
    }

    private static void verifyAnchorWithoutPredictorPoseSkipsTheUnpartitionableFirstInterval(
            Kind kind) {
        Fixture fixture = new Fixture(kind);
        fixture.estimator.setPose(new Pose2d(100.0, 0.0, 0.0));

        LoopTimestamp firstTimestamp = fixture.time.clock().nowTimestamp();
        LoopTimestamp hiddenBaseline = fixture.time.clock().timestampSecondsAgo(1.0);
        fixture.predictor.publish(
                5.0,
                firstTimestamp,
                motion(5.0, hiddenBaseline, firstTimestamp)
        );
        fixture.correction.publishNone(firstTimestamp);
        fixture.estimator.update(fixture.time.clock());
        assertEquals(100.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        fixture.time.nextCycle(1.0);
        LoopTimestamp laterTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                8.0,
                laterTimestamp,
                motion(3.0, firstTimestamp, laterTimestamp)
        );
        fixture.correction.publishNone(laterTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(103.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
    }

    private static void verifyAwaitingAnchorDoesNotMoveCoverageBackToAnOlderPredictorPose(
            Kind kind) {
        Fixture fixture = new Fixture(kind);
        LoopTimestamp anchorTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.estimate = PoseEstimate.noPose(anchorTimestamp);
        fixture.predictor.delta = MotionDelta.none(anchorTimestamp);
        fixture.correction.publish(100.0, anchorTimestamp);
        fixture.estimator.update(fixture.time.clock());
        assertEquals(100.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        fixture.time.nextCycle(1.0);
        LoopTimestamp olderTimestamp = fixture.time.clock().timestampSecondsAgo(2.0);
        fixture.predictor.publish(4.0, olderTimestamp, MotionDelta.none(olderTimestamp));
        fixture.correction.publishNone(fixture.time.clock().nowTimestamp());
        fixture.estimator.update(fixture.time.clock());
        assertEquals(100.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        fixture.time.nextCycle(1.0);
        fixture.predictor.publish(7.0, anchorTimestamp, MotionDelta.none(anchorTimestamp));
        fixture.correction.publishNone(fixture.time.clock().nowTimestamp());
        fixture.estimator.update(fixture.time.clock());
        assertEquals(100.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        fixture.time.nextCycle(1.0);
        LoopTimestamp laterTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                9.0,
                laterTimestamp,
                motion(2.0, anchorTimestamp, laterTimestamp)
        );
        fixture.correction.publishNone(laterTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(102.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
    }

    private static void verifyStalePredictorAtCorrectionAnchorCannotReplayItsPreAnchorPrefix(
            Kind kind) {
        Fixture fixture = new Fixture(kind);
        LoopTimestamp anchorTimestamp = fixture.time.clock().nowTimestamp();
        LoopTimestamp stalePredictorTimestamp = fixture.time.clock().timestampSecondsAgo(1.0);
        fixture.predictor.publish(
                5.0,
                stalePredictorTimestamp,
                MotionDelta.none(stalePredictorTimestamp)
        );
        fixture.correction.publish(100.0, anchorTimestamp);
        fixture.estimator.update(fixture.time.clock());
        assertEquals(100.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        fixture.time.nextCycle(1.0);
        LoopTimestamp firstPostAnchorTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                8.0,
                firstPostAnchorTimestamp,
                motion(8.0, stalePredictorTimestamp, firstPostAnchorTimestamp)
        );
        fixture.correction.publishNone(firstPostAnchorTimestamp);
        fixture.estimator.update(fixture.time.clock());
        assertEquals(100.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        fixture.time.nextCycle(1.0);
        LoopTimestamp laterTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                10.0,
                laterTimestamp,
                motion(2.0, firstPostAnchorTimestamp, laterTimestamp)
        );
        fixture.correction.publishNone(laterTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(102.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
    }

    private static void verifyStrictlyPositiveSubMicrosecondMotionIsNotLost(Kind kind) {
        Fixture fixture = new Fixture(kind);
        LoopTimestamp startTimestamp = initializeFromPredictor(fixture, 0.0);

        fixture.time.nextCycle(0.0000005);
        LoopTimestamp endTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                2.0,
                endTimestamp,
                motion(2.0, startTimestamp, endTimestamp)
        );
        fixture.correction.publishNone(endTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(2.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
    }

    private static void verifyPushEnabledCorrectionRebaseIsAttemptedOnlyOncePerCycle(Kind kind) {
        Fixture fixture = new Fixture(kind, true);
        LoopTimestamp startTimestamp = initializeFromPredictor(fixture, 0.0);

        fixture.time.nextCycle(1.0);
        LoopTimestamp correctionTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                5.0,
                correctionTimestamp,
                motion(5.0, startTimestamp, correctionTimestamp)
        );
        fixture.correction.publish(6.0, correctionTimestamp);
        fixture.estimator.update(fixture.time.clock());
        PoseEstimate firstEstimate = fixture.estimator.getEstimate();
        double correctedX = firstEstimate.fieldToRobotPose.xInches;
        assertEquals(1, fixture.predictor.setPoseCalls);
        assertEquals(correctedX, fixture.predictor.lastSetPoseX, EPSILON);

        fixture.estimator.update(fixture.time.clock());
        assertSame(firstEstimate, fixture.estimator.getEstimate());
        assertEquals(1, fixture.predictor.setPoseCalls);

        fixture.time.nextCycle(1.0);
        LoopTimestamp laterTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                correctedX + 3.0,
                laterTimestamp,
                motion(3.0, correctionTimestamp, laterTimestamp)
        );
        fixture.correction.publishNone(laterTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(
                correctedX + 3.0,
                fixture.estimator.getEstimate().fieldToRobotPose.xInches,
                EPSILON
        );
        assertEquals(1, fixture.predictor.setPoseCalls);
    }

    private static void verifyPushEnabledManualRebasePublishesAndPushesOnlyOnce(Kind kind) {
        Fixture fixture = new Fixture(kind, true);
        LoopTimestamp anchorTimestamp = initializeFromPredictor(fixture, 0.0);

        fixture.estimator.setPose(new Pose2d(100.0, 0.0, 0.0));
        PoseEstimate anchored = fixture.estimator.getEstimate();
        assertEquals(100.0, anchored.fieldToRobotPose.xInches, EPSILON);
        assertEquals(1, fixture.predictor.setPoseCalls);
        assertEquals(100.0, fixture.predictor.lastSetPoseX, EPSILON);

        fixture.estimator.update(fixture.time.clock());
        assertSame(anchored, fixture.estimator.getEstimate());
        assertEquals(1, fixture.predictor.setPoseCalls);

        fixture.time.nextCycle(1.0);
        LoopTimestamp laterTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                103.0,
                laterTimestamp,
                motion(3.0, anchorTimestamp, laterTimestamp)
        );
        fixture.correction.publishNone(laterTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(103.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
        assertEquals(1, fixture.predictor.setPoseCalls);
    }

    private static void verifyManualAnchorStartsExactlyOneCorrectedTrajectorySegment(Kind kind) {
        Fixture fixture = new Fixture(kind, true);
        initializeFromPredictor(fixture, 0.0);
        long initialSegment = fixture.estimator.trajectorySegmentId();

        fixture.estimator.setPose(new Pose2d(100.0, 0.0, 0.0));

        assertEquals(initialSegment + 1L, fixture.estimator.trajectorySegmentId());
        assertEquals(1L, fixture.predictor.trajectorySegmentId());

        fixture.time.nextCycle(1.0);
        LoopTimestamp laterTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(102.0, laterTimestamp, MotionDelta.none(laterTimestamp));
        fixture.correction.publishNone(laterTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(initialSegment + 1L, fixture.estimator.trajectorySegmentId());
    }

    private static void verifyExpectedCorrectionPushStaysWithinTheCorrectedTrajectory(Kind kind) {
        Fixture fixture = new Fixture(kind, true);
        LoopTimestamp initialTimestamp = initializeFromPredictor(fixture, 0.0);
        long initialSegment = fixture.estimator.trajectorySegmentId();

        fixture.time.nextCycle(1.0);
        LoopTimestamp correctionTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                4.0,
                correctionTimestamp,
                motion(4.0, initialTimestamp, correctionTimestamp)
        );
        fixture.correction.publish(6.0, correctionTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(1L, fixture.predictor.trajectorySegmentId());
        assertEquals(initialSegment, fixture.estimator.trajectorySegmentId());

        fixture.time.nextCycle(1.0);
        LoopTimestamp laterTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                fixture.estimator.getEstimate().fieldToRobotPose.xInches + 1.0,
                laterTimestamp,
                MotionDelta.none(laterTimestamp)
        );
        fixture.correction.publishNone(laterTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(initialSegment, fixture.estimator.trajectorySegmentId());
    }

    private static void verifyUnexpectedPredictorRebaseStartsANewSegmentAndDropsCrossingMotion(
            Kind kind) {
        Fixture fixture = new Fixture(kind);
        LoopTimestamp initialTimestamp = initializeFromPredictor(fixture, 0.0);
        long initialSegment = fixture.estimator.trajectorySegmentId();

        fixture.predictor.setPose(new Pose2d(50.0, 0.0, 0.0));
        fixture.time.nextCycle(1.0);
        LoopTimestamp rebasedTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                55.0,
                rebasedTimestamp,
                motion(55.0, initialTimestamp, rebasedTimestamp)
        );
        fixture.correction.publishNone(rebasedTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(initialSegment + 1L, fixture.estimator.trajectorySegmentId());
        assertEquals(55.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
        assertEquals(0, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);

        fixture.time.nextCycle(1.0);
        LoopTimestamp laterTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                57.0,
                laterTimestamp,
                motion(2.0, rebasedTimestamp, laterTimestamp)
        );
        fixture.correction.publishNone(laterTimestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(initialSegment + 1L, fixture.estimator.trajectorySegmentId());
        assertEquals(57.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
    }

    private static void verifyNontransactionalManualPushFailureStartsANewFailClosedSegment(
            Kind kind) {
        Fixture fixture = new Fixture(kind, true);
        initializeFromPredictor(fixture, 0.0);
        long initialSegment = fixture.estimator.trajectorySegmentId();
        RuntimeException failure = new IllegalStateException("manual push failed after effect");
        fixture.predictor.setPoseFailure = failure;

        RuntimeException observed = captureRuntime(
                () -> fixture.estimator.setPose(new Pose2d(20.0, 0.0, 0.0))
        );

        assertSame(failure, observed);
        assertEquals(initialSegment + 1L, fixture.estimator.trajectorySegmentId());
        assertFalse(fixture.estimator.getEstimate().hasPose);
        assertEquals(1L, fixture.predictor.trajectorySegmentId());

        fixture.predictor.setPoseFailure = null;
        fixture.time.nextCycle(1.0);
        LoopTimestamp blockedTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.estimate = PoseEstimate.noPose(blockedTimestamp);
        fixture.predictor.delta = MotionDelta.none(blockedTimestamp);
        fixture.correction.publish(99.0, blockedTimestamp);
        fixture.estimator.update(fixture.time.clock());
        assertFalse(fixture.estimator.getEstimate().hasPose);
        assertEquals(0, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);

        fixture.time.nextCycle(1.0);
        LoopTimestamp recoveryTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(5.0, recoveryTimestamp, MotionDelta.none(recoveryTimestamp));
        fixture.estimator.update(fixture.time.clock());
        assertTrue(fixture.estimator.getEstimate().hasPose);
        assertEquals(5.0, fixture.estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
        assertEquals(0, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);

        fixture.time.nextCycle(1.0);
        LoopTimestamp laterTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(5.0, laterTimestamp, MotionDelta.none(laterTimestamp));
        fixture.estimator.update(fixture.time.clock());
        assertEquals(0, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);
    }

    private static void verifyRejectedManualAnchorStillObservesAPreexistingPredictorRebase(
            Kind kind) {
        Fixture fixture = new Fixture(kind, true);
        initializeFromPredictor(fixture, 0.0);
        long initialSegment = fixture.estimator.trajectorySegmentId();

        fixture.predictor.setPose(new Pose2d(50.0, 0.0, 0.0));
        RuntimeException failure = new IllegalStateException("manual push rejected");
        fixture.predictor.setPoseFailure = failure;
        fixture.predictor.setPoseFailureAfterEffect = false;

        RuntimeException observed = captureRuntime(
                () -> fixture.estimator.setPose(new Pose2d(20.0, 0.0, 0.0))
        );

        assertSame(failure, observed);
        assertEquals(1L, fixture.predictor.trajectorySegmentId());
        assertEquals(initialSegment + 1L, fixture.estimator.trajectorySegmentId());
        assertFalse(fixture.estimator.getEstimate().hasPose);
    }

    private static void verifyNontransactionalCorrectionPushFailureStartsANewFailClosedSegment(
            Kind kind) {
        Fixture fixture = new Fixture(kind, true);
        LoopTimestamp initialTimestamp = initializeFromPredictor(fixture, 0.0);
        long initialSegment = fixture.estimator.trajectorySegmentId();
        RuntimeException failure = new IllegalStateException("correction push failed after effect");
        fixture.predictor.setPoseFailure = failure;

        fixture.time.nextCycle(1.0);
        LoopTimestamp correctionTimestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(
                2.0,
                correctionTimestamp,
                motion(2.0, initialTimestamp, correctionTimestamp)
        );
        fixture.correction.publish(4.0, correctionTimestamp);

        RuntimeException first = captureRuntime(
                () -> fixture.estimator.update(fixture.time.clock())
        );

        assertSame(failure, first);
        assertEquals(initialSegment + 1L, fixture.estimator.trajectorySegmentId());
        assertFalse(fixture.estimator.getEstimate().hasPose);
        assertEquals(0, fixture.estimator.getCorrectionStats().acceptedCorrectionCount);

        RuntimeException repeated = captureRuntime(
                () -> fixture.estimator.update(fixture.time.clock())
        );
        assertSame(first, repeated);
        assertEquals(initialSegment + 1L, fixture.estimator.trajectorySegmentId());
        assertEquals(1, fixture.predictor.setPoseCalls);
    }

    private static void verifyCorrectionFailureAfterRebasingPredictorRetainsTheContinuityBarrier(
            Kind kind) {
        Fixture fixture = new Fixture(kind);
        initializeFromPredictor(fixture, 0.0);
        long initialSegment = fixture.estimator.trajectorySegmentId();
        RuntimeException failure = new IllegalStateException("correction failed after rebase");

        fixture.time.nextCycle(1.0);
        LoopTimestamp timestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(1.0, timestamp, MotionDelta.none(timestamp));
        fixture.correction.publishNone(timestamp);
        fixture.correction.duringUpdate = () ->
                fixture.predictor.setPose(new Pose2d(50.0, 0.0, 0.0));
        fixture.correction.failure = failure;

        RuntimeException observed = captureRuntime(
                () -> fixture.estimator.update(fixture.time.clock())
        );

        assertSame(failure, observed);
        assertEquals(initialSegment + 1L, fixture.estimator.trajectorySegmentId());
        assertFalse(fixture.estimator.getEstimate().hasPose);
        assertEquals(1L, fixture.predictor.trajectorySegmentId());
    }

    private static void verifyClockEpochResetAloneDoesNotChangeTrajectorySegment(Kind kind) {
        Fixture fixture = new Fixture(kind);
        initializeFromPredictor(fixture, 0.0);
        long initialSegment = fixture.estimator.trajectorySegmentId();

        fixture.time.clock().reset(0.0);
        LoopTimestamp timestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(0.0, timestamp, MotionDelta.none(timestamp));
        fixture.correction.publishNone(timestamp);
        fixture.estimator.update(fixture.time.clock());

        assertEquals(initialSegment, fixture.estimator.trajectorySegmentId());
    }

    private static LoopTimestamp initializeFromPredictor(Fixture fixture, double xInches) {
        LoopTimestamp timestamp = fixture.time.clock().nowTimestamp();
        fixture.predictor.publish(xInches, timestamp, MotionDelta.none(timestamp));
        fixture.correction.publishNone(timestamp);
        fixture.estimator.update(fixture.time.clock());
        assertTrue(fixture.estimator.getEstimate().hasPose);
        return timestamp;
    }

    private static RuntimeException captureRuntime(Runnable action) {
        try {
            action.run();
            fail("Expected RuntimeException");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static void assertCorrectionStatsEqual(CorrectionStats expected,
                                                   CorrectionStats actual) {
        assertEquals(expected.acceptedCorrectionCount, actual.acceptedCorrectionCount);
        assertEquals(expected.rejectedCorrectionCount, actual.rejectedCorrectionCount);
        assertEquals(expected.skippedDuplicateCorrectionCount,
                actual.skippedDuplicateCorrectionCount);
        assertEquals(expected.skippedOutOfOrderCorrectionCount,
                actual.skippedOutOfOrderCorrectionCount);
        assertEquals(expected.replayedCorrectionCount, actual.replayedCorrectionCount);
        assertEquals(expected.projectedCorrectionCount, actual.projectedCorrectionCount);
        assertSame(expected.lastCorrectionAccepted, actual.lastCorrectionAccepted);
        assertSame(expected.lastAcceptedCorrectionMeasurementTimestamp,
                actual.lastAcceptedCorrectionMeasurementTimestamp);
        assertSame(expected.lastEvaluatedCorrectionTimestamp,
                actual.lastEvaluatedCorrectionTimestamp);
        assertEquals(expected.lastCorrectionUsedReplay, actual.lastCorrectionUsedReplay);
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

    private static Pose3d pose(double xInches) {
        return new Pose3d(xInches, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock(10.0);
        final RecordingPredictor predictor = new RecordingPredictor();
        final RecordingCorrection correction = new RecordingCorrection();
        final CorrectedPoseEstimator estimator;

        Fixture(Kind kind) {
            this(kind, false);
        }

        Fixture(Kind kind, boolean pushCorrectedPoseToPredictor) {
            if (kind == Kind.FUSION) {
                OdometryCorrectionFusionEstimator.Config config =
                        OdometryCorrectionFusionEstimator.Config.defaults();
                config.maxCorrectionAgeSec = 5.0;
                config.predictorHistorySec = 5.0;
                config.correctionPositionGain = 1.0;
                config.correctionHeadingGain = 1.0;
                config.maxCorrectionPositionJumpIn = 1_000.0;
                config.maxCorrectionHeadingJumpRad = Math.PI;
                config.enablePushCorrectedPoseToPredictor =
                        pushCorrectedPoseToPredictor;
                estimator = new OdometryCorrectionFusionEstimator(
                        predictor,
                        correction,
                        config
                );
            } else {
                OdometryCorrectionEkfEstimator.Config config =
                        OdometryCorrectionEkfEstimator.Config.defaults();
                config.maxCorrectionAgeSec = 5.0;
                config.predictorHistorySec = 5.0;
                config.maxCorrectionPositionInnovationIn = 1_000.0;
                config.maxCorrectionHeadingInnovationRad = Math.PI;
                config.maxCorrectionMahalanobisSq = 1_000_000_000.0;
                config.enablePushCorrectedPoseToPredictor =
                        pushCorrectedPoseToPredictor;
                estimator = new OdometryCorrectionEkfEstimator(
                        predictor,
                        correction,
                        config
                );
            }
        }
    }

    private static final class RecordingPredictor implements MotionPredictor, PoseResetter {
        int updateCalls;
        int setPoseCalls;
        double lastSetPoseX = Double.NaN;
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());
        RuntimeException failure;
        RuntimeException setPoseFailure;
        boolean setPoseFailureAfterEffect = true;
        Runnable duringUpdate;
        long trajectorySegmentId;

        void publish(double xInches, LoopTimestamp timestamp, MotionDelta nextDelta) {
            estimate = new PoseEstimate(pose(xInches), true, 1.0, timestamp);
            delta = nextDelta;
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            if (duringUpdate != null) {
                duringUpdate.run();
            }
            if (failure != null) {
                throw failure;
            }
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
            lastSetPoseX = pose.xInches;
            if (setPoseFailure != null && !setPoseFailureAfterEffect) {
                throw setPoseFailure;
            }
            trajectorySegmentId++;
            LoopTimestamp timestamp = estimate != null && estimate.timestamp != null
                    ? estimate.timestamp
                    : LoopTimestamp.unavailable();
            if (setPoseFailure != null) {
                estimate = PoseEstimate.noPose(timestamp);
                delta = MotionDelta.none(timestamp);
                throw setPoseFailure;
            }
            estimate = new PoseEstimate(
                    new Pose3d(
                            pose.xInches,
                            pose.yInches,
                            0.0,
                            pose.headingRad,
                            0.0,
                            0.0
                    ),
                    true,
                    1.0,
                    timestamp
            );
            delta = MotionDelta.none(timestamp);
        }
    }

    private static final class RecordingCorrection implements AbsolutePoseEstimator {
        int updateCalls;
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        RuntimeException failure;
        Runnable duringUpdate;

        void publish(double xInches, LoopTimestamp timestamp) {
            estimate = new PoseEstimate(pose(xInches), true, 1.0, timestamp);
        }

        void publishNone(LoopTimestamp timestamp) {
            estimate = PoseEstimate.noPose(timestamp);
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            if (duringUpdate != null) {
                duringUpdate.run();
            }
            if (failure != null) {
                throw failure;
            }
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }
}
