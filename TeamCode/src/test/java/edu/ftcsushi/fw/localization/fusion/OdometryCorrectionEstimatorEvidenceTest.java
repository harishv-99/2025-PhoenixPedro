package edu.ftcsushi.fw.localization.fusion;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.HeadingEstimate;
import edu.ftcsushi.fw.localization.MotionDelta;
import edu.ftcsushi.fw.localization.MotionPredictor;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseResetter;
import edu.ftcsushi.fw.spatial.SpatialSolveSet;
import edu.ftcsushi.fw.spatial.SpatialSolveLane;
import edu.ftcsushi.fw.spatial.SpatialSolveRequest;
import edu.ftcsushi.fw.spatial.SpatialTargets;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/**
 * Evidence-time regressions through the real corrected estimators and their public contracts.
 *
 * <p>Only sensor publications and the predictor's fallible reset boundary are scripted. The real
 * clock, correction admission, history, fusion, covariance, and downstream consumers remain intact.
 * Exact sample/arrival times and independent coordinate arithmetic prove software provenance,
 * not sensor clock accuracy, physical localization, or safe robot age thresholds.</p>
 */
public final class OdometryCorrectionEstimatorEvidenceTest {
    private static final double EPSILON = 1e-8;

    private enum Kind { FUSION, EKF }

    @Test
    public void retainedPredictorDoesNotRefreshEvidenceAcrossLoopsOrSameCycleReads() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            f.at(11.0);
            f.update();

            f.expectTime(10.0);
            f.expectPose(0.0, 0.0);
            PoseEstimate aged = f.estimator.getEstimate();
            int polls = f.predictor.polls;
            f.update();
            assertSame(aged, f.estimator.getEstimate());
            assertEquals(polls, f.predictor.polls);
            assertEquals(1.0, aged.timestamp.ageSec(f.clock), EPSILON);
        }
    }

    @Test
    public void laggedPredictorInitializationUsesSampleTimeNotDeliveryTime() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.at(11.0);
            f.predictor.publish(7.0, f.time(10.25), MotionDelta.none(f.time(10.25)));
            f.update();
            f.expectPose(7.0, 0.0);
            f.expectTime(10.25);
            assertEquals(0, f.predictor.pushes);
        }
    }

    @Test
    public void positiveDurationStationaryMotionIsFreshButNewerNoneIsNotPropagation() {
        for (Kind kind : Kind.values()) {
            for (boolean measuredInterval : new boolean[]{false, true}) {
                Fixture f = new Fixture(kind);
                f.initialize();
                f.at(11.0);
                f.predictor.publish(0.0, f.now(), measuredInterval
                        ? f.motion(0.0, 10.0, 11.0) : MotionDelta.none(f.now()));
                f.update();
                f.expectPose(0.0, 0.0);
                f.expectTime(measuredInterval ? 11.0 : 10.0);
            }
        }
    }

    @Test
    public void sameTimeNoDeltaDoesNotLoseTheNextPositiveMotionInterval() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            f.at(10.0);
            f.predictor.publish(2.0, f.now(), MotionDelta.none(f.now()));
            f.update();
            f.expectTime(10.0);
            f.expectPose(0.0, 0.0);
            f.at(11.0);
            f.predictor.publish(5.0, f.now(), f.motion(5.0, 10.0, 11.0));
            f.update();
            f.expectPose(5.0, 0.0);
            f.expectTime(11.0);
        }
    }

    @Test
    public void correctionOnlyInitializationPreservesCaptureAndSkipsHistoricalPush() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.at(11.0);
            f.predictor.publishNone(f.now());
            f.correct(4.0, 0.0, 1.0, 10.5);
            f.update();
            f.expectPose(4.0, 0.0);
            f.expectTime(10.5);
            assertEquals(0, f.predictor.pushes);
            assertEquals(1, f.stats().acceptedCorrectionCount);
            assertEquals(1, f.stats().nonReplayedCorrectionCount);
            assertTime(f.stats().lastCorrectionAccepted, f.time(11.0));
            assertTime(f.stats().lastAcceptedCorrectionMeasurementTimestamp, f.time(10.5));
        }
    }

    @Test
    public void currentDirectCorrectionWithoutPredictorRemainsAvailableAndPushEligible() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            f.at(11.0);
            f.predictor.publishNone(f.now());
            f.correct(4.0, 0.0, 1.0, 11.0);
            f.update();
            // Fusion gain=1; EKF x variances P=1 and R=1 give K=1/2.
            f.expectPose(kind == Kind.FUSION ? 4.0 : 2.0, 0.0);
            f.expectTime(11.0);
            assertEquals(1, f.predictor.pushes);
            assertEquals(1, f.stats().acceptedCorrectionCount);
        }
    }

    @Test
    public void newerDirectCaptureWithoutPredictorMayPublishAnAgedEndpoint() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            f.at(11.0);
            f.predictor.publishNone(f.now());
            f.correct(4.0, 0.0, 1.0, 10.5);
            f.update();
            f.expectPose(kind == Kind.FUSION ? 4.0 : 2.0, 0.0);
            f.expectTime(10.5);
            assertEquals(0, f.predictor.pushes);
            f.at(11.25);
            f.update();
            f.expectNoPoseNow();
        }
    }

    @Test
    public void directCaptureAfterRetainedPredictorDoesNotInventMotionToDelivery() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            f.at(11.0); // Retain predictor t10; correction is newer than it but not delivery-now.
            f.correct(4.0, 0.0, 1.0, 10.5);
            f.update();
            f.expectPose(kind == Kind.FUSION ? 4.0 : 2.0, 0.0);
            f.expectTime(10.5);
            assertEquals(0, f.predictor.pushes);
            assertEquals(1, f.stats().nonReplayedCorrectionCount);
        }
    }

    @Test
    public void ekfDelayedDirectUpdateRetainsItsConfiguredAgeNoiseWithEitherCompensationMode() {
        for (boolean compensation : new boolean[]{false, true}) {
            Fixture f = new Fixture(Kind.EKF, compensation, true, 1.0, 1.0, 4.0);
            f.initialize();
            f.at(11.0);
            f.correct(4.0, 0.0, 1.0, 10.5);
            f.update();

            // Delivery age is .5s. Position std=1+4*.5=3, so R=9 and prior P=1.
            // K=1/(1+9)=.1 incorporates .4 inches while retaining the t10.5 endpoint.
            OdometryCorrectionEkfEstimator ekf = (OdometryCorrectionEkfEstimator) f.estimator;
            assertEquals(3.0, ekf.getLastMeasurementPositionStdIn(), EPSILON);
            assertEquals(2.1, ekf.getLastMeasurementHeadingStdRad(), EPSILON);
            f.expectPose(0.4, 0.0);
            f.expectTime(10.5);
            assertEquals(1, f.stats().acceptedCorrectionCount);
            assertEquals(1, f.stats().nonReplayedCorrectionCount);
            assertEquals(0, f.predictor.pushes);
        }
    }

    @Test
    public void laggedReplayUsesSupportedEndpointAndDoesNotSwallowLaterMotion() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            f.sample(11.0, 11.0, 10.0, 10.0, 10.0);
            f.update();
            f.sample(12.5, 12.0, 20.0, 10.0, 11.0);
            f.correct(17.0, 0.0, 1.0, 11.5);
            f.update();

            // At capture x=15. Fusion corrects to17; EKF adds half the 2-inch innovation.
            // Both then replay the measured five inches to endpoint t12, not delivery t12.5.
            double correctedX = kind == Kind.FUSION ? 22.0 : 21.0;
            f.expectPose(correctedX, 0.0);
            f.expectTime(12.0);
            assertEquals(1, f.stats().replayedCorrectionCount);
            assertTrue(f.stats().lastCorrectionUsedReplay);
            assertEquals(0, f.predictor.pushes);
            assertTime(f.stats().lastCorrectionAccepted, f.time(12.5));

            f.sample(13.0, 13.0, 25.0, 5.0, 12.0);
            f.update();
            f.expectPose(correctedX + 5.0, 0.0);
            f.expectTime(13.0);
        }
    }

    @Test
    public void currentReplayMayPushButRepeatedSourceCannotPushAgain() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            f.sample(11.0, 11.0, 10.0, 10.0, 10.0);
            f.update();
            f.sample(12.0, 12.0, 20.0, 10.0, 11.0);
            f.correct(17.0, 0.0, 1.0, 11.5);
            f.update();
            f.expectTime(12.0);
            assertEquals(1, f.predictor.pushes);
            f.clock.update(12.25); // Intentionally retain the exact correction frame.
            f.update();
            assertEquals(1, f.predictor.pushes);
            assertEquals(1, f.stats().skippedDuplicateCorrectionCount);
            assertTrue(f.stats().lastCorrectionUsedReplay);
        }
    }

    @Test
    public void changedRawPoseAtSameTimeCannotReplaceTheSupportedReplayEndpoint() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind, true, false, 1.0, 1.0);
            f.initialize();
            f.sample(11.0, 11.0, 10.0, 10.0, 10.0);
            f.update();
            f.at(11.0);
            f.predictor.publish(20.0, f.now(), MotionDelta.none(f.now()));
            f.correct(7.0, 0.0, 1.0, 10.5);
            f.update();

            // Supported history is x0 at t10 to x10 at t11: capture x5, then five inches.
            // The changed raw x20 has no positive-time motion evidence and is not an endpoint.
            f.expectPose(kind == Kind.FUSION ? 12.0 : 11.0, 0.0);
            f.expectTime(11.0);
            assertEquals(1, f.stats().replayedCorrectionCount);
            assertEquals(0, f.predictor.pushes);

            // The next measured interval still starts at supported raw x10, not zero-time x20.
            // Historical correction must rebase against that same supported endpoint.
            f.sample(12.0, 12.0, 25.0, 15.0, 11.0);
            f.update();
            f.expectPose(kind == Kind.FUSION ? 27.0 : 26.0, 0.0);
            f.expectTime(12.0);
        }
    }

    @Test
    public void delayedCaptureBeforeStartupHistoryIsRejectedOnceWithoutAnyPush() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            f.at(11.0);
            f.predictor.publish(0.0, f.now(), MotionDelta.none(f.now()));
            f.correct(4.0, 0.0, 1.0, 9.5);
            f.update();
            assertEquals(0, f.stats().acceptedCorrectionCount);
            assertEquals(1, f.stats().rejectedCorrectionCount);
            assertEquals(0, f.predictor.pushes);
            f.expectPose(0.0, 0.0);
            f.clock.update(11.1);
            f.update();
            assertEquals(1, f.stats().rejectedCorrectionCount);
            assertEquals(1, f.stats().skippedDuplicateCorrectionCount);
        }
    }

    @Test
    public void newerNoneOrInvalidMotionCannotInventHistoryAcrossAnObservationGap() {
        for (Kind kind : Kind.values()) {
            for (boolean invalidMotion : new boolean[]{false, true}) {
                Fixture f = new Fixture(kind);
                f.initialize();
                f.at(10.5);
                MotionDelta missing = invalidMotion
                        ? new MotionDelta(pose(5.0, 0.0), true, Double.NaN, f.time(10.0), f.now())
                        : MotionDelta.none(f.now());
                f.predictor.publish(5.0, f.now(), missing);
                f.update();
                f.sample(11.0, 11.0, 10.0, 5.0, 10.5);
                f.correct(2.5, 0.0, 1.0, 10.25);
                f.update();
                assertEquals(0, f.stats().acceptedCorrectionCount);
                assertEquals(1, f.stats().rejectedCorrectionCount);
                assertEquals(0, f.predictor.pushes);
            }
        }
    }

    @Test
    public void coherentLongDeliveredIntervalCanBridgeWithoutASensorFrequencyGuess() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            f.sample(12.0, 12.0, 20.0, 20.0, 10.0);
            f.correct(15.0, 0.0, 1.0, 11.5);
            f.update();
            f.expectPose(20.0, 0.0);
            f.expectTime(12.0);
            assertEquals(1, f.stats().acceptedCorrectionCount);
            assertEquals(1, f.stats().replayedCorrectionCount);
        }
    }

    @Test
    public void missingPredictorCutsReplayEligibilityAndNeverCommitsIntermediatePastState() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            f.sample(11.0, 11.0, 10.0, 10.0, 10.0);
            f.update();
            f.sample(12.0, 12.0, 20.0, 10.0, 11.0);
            f.update();
            f.at(13.0);
            f.predictor.publishNone(LoopTimestamp.unavailable());
            f.correct(17.0, 0.0, 1.0, 11.5);
            f.update();

            // The selected conservative source-gap policy cuts even formerly usable history.
            // Old EKF could instead commit a measurement-time intermediate as replay success.
            f.expectNoPoseNow();
            assertEquals(0, f.stats().acceptedCorrectionCount);
            assertEquals(1, f.stats().rejectedCorrectionCount);
            assertEquals(0, f.stats().replayedCorrectionCount);
            assertEquals(0, f.predictor.pushes);
            f.at(13.25);
            f.predictor.publish(20.0, f.now(), MotionDelta.none(f.now()));
            f.update();
            f.expectTime(13.25);
            f.expectPose(20.0, 0.0);
        }
    }

    @Test
    public void disablingCompensationRejectsUnalignedPastButKeepsCurrentDirectUpdates() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind, false, true, 1.0, 1.0);
            f.initialize();
            f.sample(11.0, 11.0, 10.0, 10.0, 10.0);
            f.correct(5.0, 0.0, 1.0, 10.5);
            f.update();
            assertEquals(0, f.stats().acceptedCorrectionCount);
            assertEquals(1, f.stats().rejectedCorrectionCount);
            assertEquals(0, f.predictor.pushes);
            f.at(11.25);
            f.correct(10.0, 0.0, 1.0, 11.25);
            f.update();
            f.expectPose(10.0, 0.0);
            f.expectTime(11.25);
            assertEquals(1, f.stats().acceptedCorrectionCount);
            assertEquals(1, f.stats().nonReplayedCorrectionCount);
        }
    }

    @Test
    public void rejectedCandidateDoesNotRewriteTheLastAcceptedReplayDisposition() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            f.sample(11.0, 11.0, 10.0, 10.0, 10.0);
            f.update();
            f.sample(12.0, 12.0, 20.0, 10.0, 11.0);
            f.correct(15.0, 0.0, 1.0, 11.5);
            f.update();
            assertTrue(f.stats().lastCorrectionUsedReplay);
            LoopTimestamp accepted = f.stats().lastAcceptedCorrectionMeasurementTimestamp;
            f.at(12.25);
            f.correct(100_000.0, 0.0, 1.0, 12.25);
            f.update();
            assertEquals(1, f.stats().acceptedCorrectionCount);
            assertEquals(1, f.stats().rejectedCorrectionCount);
            assertTrue(f.stats().lastCorrectionUsedReplay);
            assertSame(accepted, f.stats().lastAcceptedCorrectionMeasurementTimestamp);
        }
    }

    @Test
    public void manualAnchorUsesLastActualPublicationLoopNotAgedEvidenceOrUnservicedTime() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            f.at(11.0);
            f.update();
            f.expectTime(10.0);
            f.at(12.0); // No estimator publication at this new clock time.
            f.estimator.setPose(new Pose2d(20.0, 0.0, 0.0));
            f.expectPose(20.0, 0.0);
            f.expectTime(11.0);
        }
    }

    @Test
    public void manualAnchorBeforeFirstPublicationHasNoInventedTimestamp() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.estimator.setPose(new Pose2d(20.0, 0.0, 0.0));
            assertTrue(f.estimator.getEstimate().hasPose);
            assertFalse(f.estimator.getEstimate().timestamp.isAvailable());
        }
    }

    @Test
    public void failedPushChangesManualPublicationTimeOnlyWhenItPublishedFailClosed() {
        for (Kind kind : Kind.values()) {
            for (boolean afterEffect : new boolean[]{false, true}) {
                Fixture f = new Fixture(kind);
                f.initialize();
                PoseEstimate before = f.estimator.getEstimate();
                f.at(11.0);
                f.correct(4.0, 0.0, 1.0, 11.0);
                f.predictor.failAfterEffect = afterEffect;
                f.predictor.pushFailure = new IllegalStateException("rejected predictor reset");
                RuntimeException failure = f.predictor.pushFailure;
                assertSame(failure, assertThrows(RuntimeException.class, f::update));
                if (afterEffect) f.expectNoPoseNow();
                else assertSame(before, f.estimator.getEstimate());
                int pushes = f.predictor.pushes;
                assertSame(failure, assertThrows(RuntimeException.class, f::update));
                assertEquals(pushes, f.predictor.pushes);

                f.predictor.pushFailure = null;
                f.at(12.0);
                f.estimator.setPose(new Pose2d(20.0, 0.0, 0.0));
                f.expectTime(afterEffect ? 11.0 : 10.0);
            }
        }
    }

    @Test
    public void resetThenChildFailureWithoutPublicationRetainsTheOlderManualBoundary() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            f.at(11.0);
            f.update();
            f.estimator.setPose(new Pose2d(20.0, 0.0, 0.0));
            PoseEstimate beforeReset = f.estimator.getEstimate();
            f.expectTime(11.0);
            f.clock.reset(11.0);
            RuntimeException failure = new IllegalStateException("predictor poll failed in new epoch");
            f.predictor.updateFailure = failure;

            assertSame(failure, assertThrows(RuntimeException.class, f::update));
            assertSame(beforeReset, f.estimator.getEstimate());
            f.predictor.updateFailure = null;
            f.estimator.setPose(new Pose2d(30.0, 0.0, 0.0));

            // Neither the reset nor the failed poll published a new boundary. The old timestamp
            // is retained by identity, but its prior epoch cannot masquerade as fresh evidence.
            f.expectPose(30.0, 0.0);
            assertSame(beforeReset.timestamp, f.estimator.getEstimate().timestamp);
            assertTrue(Double.isNaN(f.estimator.getEstimate().timestamp.ageSec(f.clock)));
        }
    }

    @Test
    public void admittedZeroWeightFusionDoesNotRefreshTimePushOrCoverLaterMotion() {
        for (boolean zeroQuality : new boolean[]{false, true}) {
            Fixture f = new Fixture(Kind.FUSION, true, true,
                    zeroQuality ? 1.0 : 0.0, zeroQuality ? 1.0 : 0.0);
            f.initialize();
            f.at(11.0);
            f.correct(4.0, 0.4, zeroQuality ? 0.0 : 1.0, 11.0);
            f.update();
            f.expectTime(10.0);
            f.expectPose(0.0, 0.0);
            assertEquals(1, f.stats().acceptedCorrectionCount);
            assertEquals(0, f.predictor.pushes);
            assertTime(f.stats().lastCorrectionAccepted, f.now());
            f.sample(12.0, 12.0, 5.0, 5.0, 10.0);
            f.update();
            f.expectPose(5.0, 0.0);
            f.expectTime(12.0);
        }
    }

    @Test
    public void zeroWeightFusionAcceptanceCannotRestoreMissingPredictorAvailability() {
        for (boolean zeroQuality : new boolean[]{false, true}) {
            Fixture f = new Fixture(Kind.FUSION, true, true,
                    zeroQuality ? 1.0 : 0.0, zeroQuality ? 1.0 : 0.0);
            f.initialize();
            f.at(11.0);
            f.predictor.publishNone(f.now());
            f.correct(4.0, 0.4, zeroQuality ? 0.0 : 1.0, 11.0);
            f.update();
            f.expectNoPoseNow();
            assertEquals(1, f.stats().acceptedCorrectionCount);
            assertEquals(0, f.predictor.pushes);
        }
    }

    @Test
    public void positivePartialGainAndZeroInnovationStillIncorporateEvidence() {
        for (boolean headingOnly : new boolean[]{false, true}) {
            Fixture f = new Fixture(Kind.FUSION, true, true,
                    headingOnly ? 0.0 : 1.0, headingOnly ? 1.0 : 0.0);
            f.initialize();
            f.at(11.0);
            f.correct(0.0, headingOnly ? 0.3 : 0.0, 1.0, 11.0);
            f.update();
            f.expectTime(11.0);
            f.expectPose(0.0, headingOnly ? 0.3 : 0.0);
            assertEquals(1, f.predictor.pushes);
        }
    }

    @Test
    public void zeroQualityInitializationCopiesPoseAndEkfZeroQualityStillHasFiniteWeight() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.at(11.0);
            f.predictor.publishNone(f.now());
            f.correct(4.0, 0.0, 0.0, 10.5);
            f.update();
            f.expectTime(10.5);
            f.expectPose(4.0, 0.0);
            assertEquals(1, f.stats().acceptedCorrectionCount);
        }
        Fixture ekf = new Fixture(Kind.EKF);
        ekf.initialize();
        ekf.at(11.0);
        ekf.predictor.publishNone(ekf.now());
        ekf.correct(4.0, 0.0, 0.0, 11.0);
        ekf.update();
        ekf.expectTime(11.0);
        ekf.expectPose(2.0, 0.0); // Explicit fixture R=1 even for quality zero; P=1 gives K=1/2.
    }

    @Test
    public void agedHighQualityFailsSpatialAgeGateAndHeadingKeepsTheSameEvidenceTime() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            SpatialSolveLane lane = SpatialSolveSet.builder().absolutePose(f.estimator, 0.1, 0.0).build().lane(0);
            DriveSource drive = new GamepadDriveSource(ScalarSource.constant(0.0),
                    ScalarSource.constant(1.0), ScalarSource.constant(0.0),
                    GamepadDriveSource.Config.defaults()).fieldRelativeTo(f.estimator, () -> 0.0, 0.1, 0.0);
            SpatialSolveRequest request = new SpatialSolveRequest(f.clock,
                    SpatialTargets.fieldPoint(8.0, 4.0), SpatialTargets.fieldHeading(0.3),
                    null, null, Pose2d.zero(), Pose2d.zero(), null);
            assertTrue(lane.solve(request).valid());
            assertTrue(Math.abs(drive.get(f.clock).axial) > 0.9);
            f.at(10.25);
            f.update();
            assertTrue(f.estimator.getEstimate().hasPose);
            assertFalse(lane.solve(request).valid());
            assertEquals(0.0, drive.get(f.clock).axial, EPSILON);
            assertEquals(0.0, drive.get(f.clock).lateral, EPSILON);
            HeadingEstimate heading = f.estimator.getHeadingEstimate();
            assertTrue(heading.hasHeading);
            assertSame(f.estimator.getEstimate().timestamp, heading.timestamp);
            assertFalse(heading.timestamp.isFresh(f.clock, 0.1));
        }
    }

    @Test
    public void realHistoryCannotRecordFrozenOutputAsCurrentOrBridgeTheGap() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            PlanarPoseHistory.Config config = PlanarPoseHistory.Config.defaults();
            config.retentionSec = 5.0;
            config.maxInterpolationGapSec = 5.0;
            PlanarPoseHistory history = new PlanarPoseHistory(f.estimator, config);
            history.recordCurrent(f.clock);
            f.at(10.5);
            f.update();
            history.recordCurrent(f.clock);
            assertFalse(history.lookupSource().getAt(f.clock, f.now()).isAvailable());
            f.sample(11.0, 11.0, 0.0, 0.0, 10.0);
            f.update();
            history.recordCurrent(f.clock);
            assertFalse(history.lookupSource().getAt(f.clock, f.time(10.5)).isAvailable());
            assertTrue(history.lookupSource().getAt(f.clock, f.now()).isAvailable());
        }
    }

    @Test
    public void clockResetRequiresNewEpochEvidenceWithoutChangingSpatialSegment() {
        for (Kind kind : Kind.values()) {
            Fixture f = new Fixture(kind);
            f.initialize();
            long segment = f.estimator.trajectorySegmentId();
            f.clock.reset(10.0);
            f.update();
            f.expectNoPoseNow();
            f.at(10.1);
            f.predictor.publish(0.0, f.now(), MotionDelta.none(f.now()));
            f.update();
            f.expectTime(10.1);
            assertEquals(segment, f.estimator.trajectorySegmentId());
        }
    }

    private static final class Fixture {
        final LoopClock clock = new LoopClock();
        final ScriptedPredictor predictor = new ScriptedPredictor();
        final ScriptedCorrection correction = new ScriptedCorrection();
        final CorrectedPoseEstimator estimator;

        Fixture(Kind kind) { this(kind, true, true, 1.0, 1.0); }

        Fixture(Kind kind, boolean compensation, boolean push, double positionGain, double headingGain) {
            this(kind, compensation, push, positionGain, headingGain, 0.0);
        }

        Fixture(Kind kind, boolean compensation, boolean push, double positionGain,
                double headingGain, double ageNoiseSlope) {
            clock.reset(10.0);
            if (kind == Kind.FUSION) {
                OdometryCorrectionFusionEstimator.Config config = OdometryCorrectionFusionEstimator.Config.defaults();
                config.enableLatencyCompensation = compensation;
                config.enablePushCorrectedPoseToPredictor = push;
                config.maxCorrectionAgeSec = 3.0;
                config.predictorHistorySec = 3.0;
                config.minCorrectionQuality = 0.0;
                config.correctionPositionGain = positionGain;
                config.correctionHeadingGain = headingGain;
                config.maxCorrectionPositionJumpIn = 1_000.0;
                estimator = new OdometryCorrectionFusionEstimator(predictor, correction, config);
            } else {
                OdometryCorrectionEkfEstimator.Config config = OdometryCorrectionEkfEstimator.Config.defaults();
                config.enableLatencyCompensation = compensation;
                config.enablePushCorrectedPoseToPredictor = push;
                config.maxCorrectionAgeSec = 3.0;
                config.predictorHistorySec = 3.0;
                config.minCorrectionQuality = 0.0;
                config.maxCorrectionPositionInnovationIn = 1_000.0;
                config.maxCorrectionMahalanobisSq = 1_000_000.0;
                config.initialPositionStdIn = 1.0;
                config.initialHeadingStdRad = 0.1;
                config.correctionPositionStdFloorIn = 1.0;
                config.correctionPositionStdScaleIn = 0.0;
                config.correctionHeadingStdFloorRad = 0.1;
                config.correctionHeadingStdScaleRad = 0.0;
                config.projectedCorrectionPositionStdPerSec = ageNoiseSlope;
                config.projectedCorrectionHeadingStdPerSec = ageNoiseSlope;
                // Required positive process std; its squared contribution is below EPSILON.
                config.predictorProcessPositionStdFloorIn = 1e-6;
                config.predictorProcessPositionStdPerIn = 0.0;
                config.predictorProcessPositionStdPerRad = 0.0;
                config.predictorProcessHeadingStdFloorRad = 1e-6;
                config.predictorProcessHeadingStdPerIn = 0.0;
                config.predictorProcessHeadingStdPerRad = 0.0;
                estimator = new OdometryCorrectionEkfEstimator(predictor, correction, config);
            }
        }

        void initialize() {
            predictor.publish(0.0, now(), MotionDelta.none(now()));
            correction.publishNone(now());
            update();
            expectTime(10.0);
        }

        void at(double deliverySec) {
            clock.update(deliverySec);
            correction.publishNone(now());
        }

        void sample(double deliverySec, double sampleSec, double x, double dx, double startSec) {
            at(deliverySec);
            predictor.publish(x, time(sampleSec), motion(dx, startSec, sampleSec));
        }

        void correct(double x, double yaw, double quality, double captureSec) {
            correction.estimate = new PoseEstimate(pose(x, yaw), true, quality, time(captureSec));
        }

        MotionDelta motion(double dx, double startSec, double endSec) {
            return new MotionDelta(pose(dx, 0.0), true, 1.0, time(startSec), time(endSec));
        }

        LoopTimestamp now() { return clock.nowTimestamp(); }
        LoopTimestamp time(double seconds) { return clock.timestampSecondsAgo(clock.nowSec() - seconds); }
        void update() { estimator.update(clock); }
        CorrectionStats stats() { return estimator.getCorrectionStats(); }

        void expectTime(double seconds) {
            assertTrue(estimator.getEstimate().hasPose);
            assertTime(estimator.getEstimate().timestamp, time(seconds));
        }

        void expectPose(double x, double yaw) {
            PoseEstimate estimate = estimator.getEstimate();
            assertTrue(estimate.hasPose);
            assertEquals(x, estimate.fieldToRobotPose.xInches, EPSILON);
            assertEquals(0.0, estimate.fieldToRobotPose.yInches, EPSILON);
            assertEquals(yaw, estimate.fieldToRobotPose.yawRad, EPSILON);
        }

        void expectNoPoseNow() {
            assertFalse(estimator.getEstimate().hasPose);
            assertEquals(0.0, estimator.getEstimate().quality, EPSILON);
            assertTime(estimator.getEstimate().timestamp, now());
        }
    }

    private static final class ScriptedPredictor implements MotionPredictor, PoseResetter {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());
        int polls;
        int pushes;
        long segment;
        RuntimeException pushFailure;
        RuntimeException updateFailure;
        boolean failAfterEffect;

        void publish(double x, LoopTimestamp timestamp, MotionDelta motion) {
            estimate = new PoseEstimate(pose(x, 0.0), true, 1.0, timestamp);
            delta = motion;
        }

        void publishNone(LoopTimestamp timestamp) {
            estimate = PoseEstimate.noPose(timestamp);
            delta = MotionDelta.none(timestamp);
        }

        @Override public void update(LoopClock clock) {
            polls++;
            if (updateFailure != null) throw updateFailure;
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
            estimate = new PoseEstimate(pose(value.xInches, value.headingRad), true, 1.0, timestamp);
            delta = MotionDelta.none(timestamp);
        }
    }

    private static final class ScriptedCorrection implements AbsolutePoseEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        void publishNone(LoopTimestamp timestamp) { estimate = PoseEstimate.noPose(timestamp); }
        @Override public void update(LoopClock clock) { }
        @Override public PoseEstimate getEstimate() { return estimate; }
    }

    private static Pose3d pose(double x, double yaw) { return new Pose3d(x, 0.0, 0.0, yaw, 0.0, 0.0); }
    private static void assertTime(LoopTimestamp actual, LoopTimestamp expected) {
        assertEquals(0.0, actual.secondsSince(expected), EPSILON);
    }
}
