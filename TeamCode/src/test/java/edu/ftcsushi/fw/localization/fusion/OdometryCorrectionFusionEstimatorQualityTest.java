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
import edu.ftcsushi.fw.spatial.AbsolutePoseSpatialSolveLane;
import edu.ftcsushi.fw.spatial.SpatialLaneResult;
import edu.ftcsushi.fw.spatial.SpatialSolveRequest;
import edu.ftcsushi.fw.spatial.SpatialTargets;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/**
 * LOCALIZATION-02 score evidence using the real Fusion owner and one real downstream spatial gate.
 *
 * <p>Only predictor/correction observations are scripted through their public interfaces. Tests
 * publish explicit poses, timestamps, and motion intervals; the fixture does not implement a
 * second fusion algorithm. Predictor push-back is disabled here; the lifecycle suite exercises
 * that separate transaction. Scores and expected coordinates prove software arithmetic, not
 * calibrated probabilities, odometry accuracy, physical motion, or safe robot thresholds.</p>
 */
public final class OdometryCorrectionFusionEstimatorQualityTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void acceptedLowHighAndUnitQualityScaleReportedScoreWithoutScalingPoseTwice() {
        for (double quality : new double[]{0.10, 0.40, 1.0}) {
            Fixture fixture = new Fixture(config(), 0.0);
            fixture.initializeFromPredictor();
            long segment = fixture.estimator.trajectorySegmentId();
            fixture.advanceTo(1.0);
            fixture.correction.publish(pose(4.0, -2.0, 0.4), quality, fixture.now());

            fixture.update();

            assertQuality(fixture, quality);
            // Position gain .5*q and heading gain .25*q each act exactly once from zero.
            assertPose(fixture, 2.0 * quality, -quality, 0.1 * quality);
            assertEquals(1, fixture.estimator.getAcceptedCorrectionCount());
            assertEquals(0, fixture.estimator.getRejectedCorrectionCount());
            assertEquals(1, fixture.estimator.getProjectedCorrectionCount());
            assertEquals(0, fixture.estimator.getReplayedCorrectionCount());
            assertEquals(segment, fixture.estimator.trajectorySegmentId());
        }
    }

    @Test
    public void zeroQualityCorrectionRemainsRejectedByDefaultAdmission() {
        Fixture fixture = new Fixture(config(), 0.20);
        fixture.initializeFromPredictor();
        fixture.advanceTo(1.0);
        fixture.correction.publish(pose(4.0, -2.0, 0.4), 0.0, fixture.now());

        fixture.update();

        assertQuality(fixture, 0.20);
        assertPose(fixture, 0.0, 0.0, 0.0);
        assertEquals(0, fixture.estimator.getAcceptedCorrectionCount());
        assertEquals(1, fixture.estimator.getRejectedCorrectionCount());
        assertFalse(fixture.estimator.getLastCorrectionAccepted().isAvailable());
    }

    @Test
    public void explicitlyAllowedZeroQualityCorrectionAddsNeitherBoostNorPoseMotion() {
        OdometryCorrectionFusionEstimator.Config config = config();
        config.minCorrectionQuality = 0.0;
        Fixture fixture = new Fixture(config, 0.20);
        fixture.initializeFromPredictor();
        fixture.advanceTo(1.0);
        fixture.correction.publish(pose(4.0, -2.0, 0.4), 0.0, fixture.now());

        fixture.update();

        assertQuality(fixture, 0.20);
        assertPose(fixture, 0.0, 0.0, 0.0);
        assertEquals(1, fixture.estimator.getAcceptedCorrectionCount());
        assertEquals(0, fixture.estimator.getRejectedCorrectionCount());
        assertEquals(0.0, fixture.estimator.getLastCorrectionAccepted().secondsSince(fixture.now()), 0.0);
    }

    @Test
    public void correctionOnlyInitializationKeepsFullAnchorPoseAndItsAcceptedQuality() {
        for (double quality : new double[]{0.0, 0.20, 1.0}) {
            OdometryCorrectionFusionEstimator.Config config = config();
            config.minCorrectionQuality = 0.0;
            Fixture fixture = new Fixture(config, 0.0);
            fixture.predictor.publishNone(fixture.now());
            fixture.correction.publish(pose(4.0, -2.0, 0.4), quality, fixture.now());

            fixture.update();

            assertQuality(fixture, quality);
            // Initialization is an anchor, not a gain-scaled movement from an invented origin.
            assertPose(fixture, 4.0, -2.0, 0.4);
            assertEquals(1, fixture.estimator.getAcceptedCorrectionCount());
            assertSame(fixture.now(), fixture.estimator.getLastAcceptedCorrectionMeasurementTimestamp());
        }
    }

    @Test
    public void defaultHoldDecaysAtHalfAndExpiresExactlyWithoutRestartingOnAbsentFrames() {
        Fixture fixture = acceptingStationary(0.80, 0.0, config());
        assertQuality(fixture, 0.80);

        fixture.advanceTo(1.375);
        fixture.update();
        assertQuality(fixture, 0.40);
        fixture.advanceTo(1.75);
        fixture.update();
        assertQuality(fixture, 0.0);
        fixture.advanceTo(2.0);
        fixture.update();
        assertQuality(fixture, 0.0);
        assertEquals(1, fixture.estimator.getAcceptedCorrectionCount());
        assertPose(fixture, 0.0, 0.0, 0.0);
    }

    @Test
    public void customHoldControlsDecayAndDeadlineInsteadOfUsingDefaultDuration() {
        OdometryCorrectionFusionEstimator.Config config = config();
        config.correctionConfidenceHoldSec = 2.0;
        Fixture fixture = acceptingStationary(0.80, 0.0, config);

        fixture.advanceTo(2.0);
        fixture.update();
        assertQuality(fixture, 0.40);
        fixture.advanceTo(2.999);
        fixture.update();
        assertQuality(fixture, 0.0004);
        fixture.advanceTo(3.0);
        fixture.update();
        assertQuality(fixture, 0.0);
    }

    @Test
    public void zeroHoldDisablesBoostEvenOnTheAcceptanceCycle() {
        OdometryCorrectionFusionEstimator.Config config = config();
        config.correctionConfidenceHoldSec = 0.0;
        Fixture fixture = acceptingStationary(0.80, 0.30, config);

        assertQuality(fixture, 0.30);
        assertEquals(1, fixture.estimator.getAcceptedCorrectionCount());
        assertTrue(fixture.estimator.getLastCorrectionAccepted().isAvailable());
    }

    @Test
    public void predictorQualityRemainsAFloorRatherThanBeingAveragedWithWeakCorrection() {
        OdometryCorrectionFusionEstimator.Config config = config();
        config.correctionConfidenceHoldSec = 1.0;
        Fixture fixture = acceptingStationary(0.20, 0.75, config);
        assertQuality(fixture, 0.75);

        fixture.predictorQuality = 0.10;
        fixture.advanceTo(1.20);
        fixture.update();
        assertQuality(fixture, 0.16);
        fixture.advanceTo(2.0);
        fixture.update();
        assertQuality(fixture, 0.10);
    }

    @Test
    public void newerAcceptedWeakerFrameReplacesTheOldQualityAndStartsItsOwnHold() {
        OdometryCorrectionFusionEstimator.Config config = config();
        config.correctionConfidenceHoldSec = 1.0;
        Fixture fixture = acceptingStationary(0.90, 0.0, config);
        fixture.advanceTo(1.10);
        LoopTimestamp weakerTimestamp = fixture.now();
        fixture.correction.publish(Pose3d.zero(), 0.20, weakerTimestamp);

        fixture.update();

        assertQuality(fixture, 0.20);
        assertEquals(2, fixture.estimator.getAcceptedCorrectionCount());
        assertEquals(0.0,
                fixture.estimator.getLastCorrectionAccepted().secondsSince(weakerTimestamp), 0.0);
        fixture.advanceTo(1.60);
        fixture.update();
        assertQuality(fixture, 0.10);
        fixture.advanceTo(2.10);
        fixture.update();
        assertQuality(fixture, 0.0);
    }

    @Test
    public void unacceptedSourceValuesCannotReplaceAcceptedQualityOrRestartItsDeadline() {
        String[] cases = {"absent", "null", "unavailable time", "disabled", "duplicate",
                "out of order", "low quality", "nonfinite quality", "jump", "stale"};
        for (String scenario : cases) {
            OdometryCorrectionFusionEstimator.Config config = config();
            config.correctionConfidenceHoldSec = 1.0;
            config.maxCorrectionAgeSec = 0.10;
            Fixture fixture = acceptingStationary(0.80, 0.10, config);
            LoopTimestamp accepted = fixture.estimator.getLastCorrectionAccepted();
            LoopTimestamp acceptedMeasurement =
                    fixture.estimator.getLastAcceptedCorrectionMeasurementTimestamp();
            fixture.advanceTo(1.25);
            int rejected = 0;
            int duplicate = 0;
            int outOfOrder = 0;
            switch (scenario) {
                case "absent":
                    break;
                case "null":
                    fixture.correction.estimate = null;
                    break;
                case "unavailable time":
                    fixture.correction.publish(Pose3d.zero(), 1.0, LoopTimestamp.unavailable());
                    break;
                case "disabled":
                    fixture.estimator.setCorrectionEnabled(false);
                    fixture.correction.publish(Pose3d.zero(), 1.0, fixture.now());
                    break;
                case "duplicate":
                    fixture.correction.publish(Pose3d.zero(), 1.0, acceptedMeasurement);
                    duplicate = 1;
                    break;
                case "out of order":
                    fixture.correction.publish(Pose3d.zero(), 1.0,
                            fixture.clock.timestampSecondsAgo(0.50));
                    outOfOrder = 1;
                    break;
                case "low quality":
                    fixture.correction.publish(Pose3d.zero(), 0.0, fixture.now());
                    rejected = 1;
                    break;
                case "nonfinite quality":
                    fixture.correction.publish(Pose3d.zero(), Double.NaN, fixture.now());
                    rejected = 1;
                    break;
                case "jump":
                    fixture.correction.publish(pose(100_000.0, 0.0, 0.0), 1.0, fixture.now());
                    rejected = 1;
                    break;
                case "stale":
                    fixture.correction.publish(Pose3d.zero(), 1.0,
                            fixture.clock.timestampSecondsAgo(0.125));
                    rejected = 1;
                    break;
                default:
                    throw new AssertionError(scenario);
            }

            fixture.update();

            assertEquals(scenario, 0.60, fixture.estimator.getEstimate().quality, EPSILON);
            assertPose(fixture, 0.0, 0.0, 0.0);
            assertEquals(scenario, 1, fixture.estimator.getAcceptedCorrectionCount());
            assertEquals(scenario, rejected, fixture.estimator.getRejectedCorrectionCount());
            assertEquals(scenario, duplicate, fixture.estimator.getSkippedDuplicateCorrectionCount());
            assertEquals(scenario, outOfOrder, fixture.estimator.getSkippedOutOfOrderCorrectionCount());
            assertSame(scenario, accepted, fixture.estimator.getLastCorrectionAccepted());
            assertSame(scenario, acceptedMeasurement,
                    fixture.estimator.getLastAcceptedCorrectionMeasurementTimestamp());
            fixture.advanceTo(2.0);
            fixture.update();
            assertEquals(scenario, 0.10, fixture.estimator.getEstimate().quality, EPSILON);
        }
    }

    @Test
    public void repeatedSameCycleReadsAndUpdatesKeepTheSameScorePoseAndSourceEffects() {
        Fixture fixture = acceptingStationary(0.40, 0.0, config());
        PoseEstimate published = fixture.estimator.getEstimate();
        int predictorUpdates = fixture.predictor.updates;
        int correctionUpdates = fixture.correction.updates;
        fixture.correction.publish(pose(20.0, 0.0, 0.0), 1.0, fixture.now());

        for (int repeat = 0; repeat < 4; repeat++) {
            fixture.update();
            assertSame(published, fixture.estimator.getEstimate());
            assertEquals(predictorUpdates, fixture.predictor.updates);
            assertEquals(correctionUpdates, fixture.correction.updates);
            assertEquals(1, fixture.estimator.getAcceptedCorrectionCount());
            assertQuality(fixture, 0.40);
        }
    }

    @Test
    public void delayedReplayUsesAcceptedQualityNowAndPreservesIndependentPoseArithmetic() {
        OdometryCorrectionFusionEstimator.Config config = config();
        config.enableLatencyCompensation = true;
        Fixture fixture = new Fixture(config, 0.0);
        fixture.initializeFromPredictor();
        long segment = fixture.estimator.trajectorySegmentId();
        LoopTimestamp t0 = fixture.now();
        fixture.advanceTo(1.0);
        LoopTimestamp t1 = fixture.now();
        fixture.predictor.publish(pose(10.0, 0.0, 0.0), 0.0, t1, motion(10.0, t0, t1));
        fixture.update();
        fixture.advanceTo(2.0);
        fixture.predictor.publish(pose(20.0, 0.0, 0.0), 0.0, fixture.now(),
                motion(10.0, t1, fixture.now()));
        LoopTimestamp capture = fixture.clock.timestampSecondsAgo(0.50);
        fixture.correction.publish(pose(17.0, 0.0, 0.0), 0.40, capture);

        fixture.update();

        // At capture odometry was x=15. Gain .5*.4=.2 adds .4 inches toward x=17;
        // replay adds the remaining 5 inches. Neither .4 quality nor its capture age scales twice.
        assertPose(fixture, 20.40, 0.0, 0.0);
        assertQuality(fixture, 0.40);
        assertEquals(1, fixture.estimator.getReplayedCorrectionCount());
        assertEquals(0, fixture.estimator.getProjectedCorrectionCount());
        assertTrue(fixture.estimator.wasLastCorrectionReplay());
        assertSame(capture, fixture.estimator.getLastAcceptedCorrectionMeasurementTimestamp());
        assertEquals(0.0, fixture.estimator.getLastCorrectionAccepted().secondsSince(fixture.now()), 0.0);
        assertEquals(segment, fixture.estimator.trajectorySegmentId());
        fixture.advanceTo(2.375);
        fixture.update();
        assertQuality(fixture, 0.20);
        assertPose(fixture, 20.40, 0.0, 0.0);
    }

    @Test
    public void projectedAcceptanceWithoutCurrentPredictorDoesNotMakeTheHoldAPoseSource() {
        OdometryCorrectionFusionEstimator.Config config = config();
        config.enableLatencyCompensation = true;
        Fixture fixture = new Fixture(config, 0.0);
        fixture.initializeFromPredictor();
        fixture.advanceTo(1.0);
        fixture.predictor.publishNone(fixture.now());
        fixture.correction.publish(pose(4.0, -2.0, 0.4), 0.40, fixture.now());

        fixture.update();

        assertQuality(fixture, 0.40);
        assertPose(fixture, 0.80, -0.40, 0.04);
        assertEquals(1, fixture.estimator.getProjectedCorrectionCount());
        assertEquals(0, fixture.estimator.getReplayedCorrectionCount());
        fixture.advanceTo(1.10);
        fixture.predictor.publishNone(fixture.now());
        fixture.update();
        assertFalse("an unexpired confidence hold cannot invent a current pose",
                fixture.estimator.getEstimate().hasPose);
        assertEquals(1, fixture.estimator.getAcceptedCorrectionCount());
    }

    @Test
    public void realSpatialGateRejectsWeakScoreAcceptsStrongerScoreAndDoesNotScaleGeometry() {
        Fixture fixture = acceptingStationary(0.20, 0.10, config());
        // .5 is an explicit software scenario threshold, not a physical safety recommendation.
        AbsolutePoseSpatialSolveLane lane = new AbsolutePoseSpatialSolveLane(fixture.estimator, 1.0, 0.50);
        SpatialSolveRequest request = new SpatialSolveRequest(fixture.clock,
                SpatialTargets.fieldPoint(8.0, 4.0), SpatialTargets.fieldHeading(0.30),
                null, null, Pose2d.zero(), Pose2d.zero(), null);
        SpatialLaneResult weak = lane.solve(request);
        assertFalse(weak.valid());
        assertQuality(fixture, 0.20);

        fixture.advanceTo(1.10);
        fixture.correction.publish(Pose3d.zero(), 0.80, fixture.now());
        fixture.update();
        SpatialLaneResult strong = lane.solve(request);
        assertTrue(strong.hasTranslation());
        assertTrue(strong.hasFacing());
        assertEquals(0.80, strong.translation.quality, EPSILON);
        assertEquals(8.0, strong.translation.robotToTargetPoint.xInches, EPSILON);
        assertEquals(4.0, strong.translation.robotToTargetPoint.yInches, EPSILON);
        assertEquals(0.30, strong.facing.facingErrorRad, EPSILON);

        fixture.advanceTo(1.475);
        fixture.update();
        assertQuality(fixture, 0.40);
        assertFalse("fresh pose can still fail the independent quality gate", lane.solve(request).valid());
        assertPose(fixture, 0.0, 0.0, 0.0);
    }

    /** Ordinary pose gains are explicit so coordinate expectations do not reuse production math. */
    private static OdometryCorrectionFusionEstimator.Config config() {
        OdometryCorrectionFusionEstimator.Config config = OdometryCorrectionFusionEstimator.Config.defaults();
        config.enablePushCorrectedPoseToPredictor = false;
        config.enableLatencyCompensation = false;
        config.correctionPositionGain = 0.50;
        config.correctionHeadingGain = 0.25;
        config.maxCorrectionAgeSec = 2.0;
        config.predictorHistorySec = 2.0;
        config.maxCorrectionPositionJumpIn = 1_000.0;
        return config;
    }

    private static Fixture acceptingStationary(double correctionQuality, double predictorQuality,
                                               OdometryCorrectionFusionEstimator.Config config) {
        Fixture fixture = new Fixture(config, predictorQuality);
        fixture.initializeFromPredictor();
        fixture.advanceTo(1.0);
        fixture.correction.publish(Pose3d.zero(), correctionQuality, fixture.now());
        fixture.update();
        assertEquals(1, fixture.estimator.getAcceptedCorrectionCount());
        return fixture;
    }

    private static void assertQuality(Fixture fixture, double expected) {
        PoseEstimate estimate = fixture.estimator.getEstimate();
        assertTrue(estimate.hasPose);
        assertEquals(expected, estimate.quality, EPSILON);
    }

    private static void assertPose(Fixture fixture, double xInches, double yInches, double yawRad) {
        PoseEstimate estimate = fixture.estimator.getEstimate();
        assertTrue(estimate.hasPose);
        assertEquals(xInches, estimate.fieldToRobotPose.xInches, EPSILON);
        assertEquals(yInches, estimate.fieldToRobotPose.yInches, EPSILON);
        assertEquals(yawRad, estimate.fieldToRobotPose.yawRad, EPSILON);
    }

    private static Pose3d pose(double xInches, double yInches, double yawRad) {
        return new Pose3d(xInches, yInches, 0.0, yawRad, 0.0, 0.0);
    }

    private static MotionDelta motion(double forwardInches, LoopTimestamp start, LoopTimestamp end) {
        return new MotionDelta(pose(forwardInches, 0.0, 0.0), true, 1.0, start, end);
    }

    private static final class Fixture {
        final LoopClock clock = new LoopClock();
        final ScriptedPredictor predictor = new ScriptedPredictor();
        final ScriptedCorrection correction = new ScriptedCorrection();
        final OdometryCorrectionFusionEstimator estimator;
        double predictorQuality;
        private LoopTimestamp currentTimestamp;

        Fixture(OdometryCorrectionFusionEstimator.Config config, double predictorQuality) {
            clock.reset(0.0);
            currentTimestamp = clock.nowTimestamp();
            this.predictorQuality = predictorQuality;
            predictor.publish(Pose3d.zero(), predictorQuality, now(), MotionDelta.none(now()));
            estimator = new OdometryCorrectionFusionEstimator(predictor, correction, config);
        }

        void initializeFromPredictor() {
            correction.publishNone(now());
            update();
            assertQuality(this, predictorQuality);
            assertEquals(0, estimator.getAcceptedCorrectionCount());
        }

        /** Publish a fresh stationary sample; individual motion tests replace its explicit delta. */
        void advanceTo(double seconds) {
            clock.update(seconds);
            currentTimestamp = clock.nowTimestamp();
            Pose3d stationary = predictor.estimate.fieldToRobotPose;
            predictor.publish(stationary, predictorQuality, now(), MotionDelta.none(now()));
            correction.publishNone(now());
        }

        LoopTimestamp now() { return currentTimestamp; }
        void update() { estimator.update(clock); }
    }

    private static final class ScriptedPredictor implements MotionPredictor {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());
        int updates;

        void publish(Pose3d fieldPose, double quality, LoopTimestamp timestamp, MotionDelta motion) {
            estimate = new PoseEstimate(fieldPose, true, quality, timestamp);
            delta = motion;
        }

        void publishNone(LoopTimestamp timestamp) {
            estimate = PoseEstimate.noPose(timestamp);
            delta = MotionDelta.none(timestamp);
        }

        @Override public void update(LoopClock clock) { updates++; }
        @Override public PoseEstimate getEstimate() { return estimate; }
        @Override public MotionDelta getLatestMotionDelta() { return delta; }
        @Override public long trajectorySegmentId() { return 0L; }
    }

    private static final class ScriptedCorrection implements AbsolutePoseEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        int updates;

        void publish(Pose3d fieldPose, double quality, LoopTimestamp timestamp) {
            estimate = new PoseEstimate(fieldPose, true, quality, timestamp);
        }

        void publishNone(LoopTimestamp timestamp) { estimate = PoseEstimate.noPose(timestamp); }
        @Override public void update(LoopClock clock) { updates++; }
        @Override public PoseEstimate getEstimate() { return estimate; }
    }
}
