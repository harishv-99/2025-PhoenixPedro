package edu.ftcsushi.robots.examples.reference.capability.targeting;

import org.junit.Test;

import edu.ftcsushi.fw.actuation.PlantTargetContext;
import edu.ftcsushi.fw.actuation.PlantTargetResolution;
import edu.ftcsushi.fw.actuation.PlantTargetResolver;
import edu.ftcsushi.fw.actuation.PlantTargets;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.math.InterpolatingTable1D;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.MotionDelta;
import edu.ftcsushi.fw.localization.MotionPredictor;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Contract tests for the optional coherent-shot calculation owner. */
public final class ReferenceCoordinatedShotServiceTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void constructionSnapshotsConfigAndDoesNotSampleOrUpdatePredictor() {
        ManualLoopClock time = new ManualLoopClock(10.0);
        FakeMotionPredictor predictor = new FakeMotionPredictor();
        ReferenceCoordinatedShotService.Config config = testConfig();
        InterpolatingTable1D originalFlywheel = config.flywheelVelocityTicksPerSecByDistance;
        InterpolatingTable1D originalHood = config.hoodPositionByDistance;
        ReferenceCoordinatedShotService service =
                new ReferenceCoordinatedShotService(predictor, config);

        assertEquals(0, predictor.estimateReads);
        assertEquals(0, predictor.deltaReads);
        assertEquals(0, predictor.updateCalls);
        assertEquals(0.50, service.plannerObservationMaxAgeSec(), 0.0);
        assertEquals(0.20, service.plannerMinimumObservationQuality(), 0.0);
        assertEquals(ReferenceCoordinatedShotService.Mode.UNAVAILABLE,
                service.solution().mode);
        assertEquals(ReferenceCoordinatedShotService.Reason.NOT_STARTED,
                service.solution().reason);

        config.targetFieldXInches = 180.0;
        config.spatialMaxAgeSec = 0.0;
        config.spatialMinQuality = 1.0;
        config.flywheelVelocityTicksPerSecByDistance =
                InterpolatingTable1D.ofSortedPairs(0.0, 9999.0);
        config.hoodPositionByDistance = InterpolatingTable1D.ofSortedPairs(0.0, 0.99);
        config.unavailableFlywheelVelocityTicksPerSec = 500.0;
        config.unavailableHoodPosition = 0.90;
        assertNotSame(originalFlywheel, config.flywheelVelocityTicksPerSecByDistance);
        assertNotSame(originalHood, config.hoodPositionByDistance);

        predictor.publishStationary(pose(0.0, 0.0, 0.0), 0.80, time.clock());
        service.start(time.clock());

        assertEquals(ReferenceCoordinatedShotService.Mode.STATIONARY_FALLBACK,
                service.solution().mode);
        assertEquals(50.0, service.solution().effectiveForwardInches, EPSILON);
        assertEquals(1500.0, service.solution().flywheelVelocityTicksPerSec, EPSILON);
        assertEquals(0.40, service.solution().hoodPosition, EPSILON);
        assertEquals(0, predictor.updateCalls);
    }

    @Test
    public void configurationRejectsInvalidGatesDomainsFallbacksAndTables() {
        FakeMotionPredictor predictor = new FakeMotionPredictor();
        assertThrows(NullPointerException.class,
                () -> new ReferenceCoordinatedShotService(null, testConfig()));
        assertThrows(NullPointerException.class,
                () -> new ReferenceCoordinatedShotService(predictor, null));

        ReferenceCoordinatedShotService.Config badQuality = testConfig();
        badQuality.motionMinQuality = 1.01;
        assertThrows(IllegalArgumentException.class,
                () -> new ReferenceCoordinatedShotService(predictor, badQuality));

        ReferenceCoordinatedShotService.Config badAge = testConfig();
        badAge.spatialMaxAgeSec = Double.NaN;
        assertThrows(IllegalArgumentException.class,
                () -> new ReferenceCoordinatedShotService(predictor, badAge));

        ReferenceCoordinatedShotService.Config reversedDistance = testConfig();
        reversedDistance.minimumModelDistanceInches = 100.0;
        reversedDistance.maximumModelDistanceInches = 99.0;
        assertThrows(IllegalArgumentException.class,
                () -> new ReferenceCoordinatedShotService(predictor, reversedDistance));

        ReferenceCoordinatedShotService.Config missingTable = testConfig();
        missingTable.hoodPositionByDistance = null;
        assertThrows(NullPointerException.class,
                () -> new ReferenceCoordinatedShotService(predictor, missingTable));

        ReferenceCoordinatedShotService.Config badFallback = testConfig();
        badFallback.unavailableHoodPosition = 1.01;
        assertThrows(IllegalArgumentException.class,
                () -> new ReferenceCoordinatedShotService(predictor, badFallback));
    }

    @Test
    public void oneImmutableSolutionIsPublishedOncePerCycle() {
        Fixture fixture = new Fixture(testConfig());
        fixture.predictor.publishStationary(
                pose(0.0, 0.0, 0.0), 0.80, fixture.time.clock());
        fixture.service.start(fixture.time.clock());
        ReferenceCoordinatedShotService.Solution first = fixture.service.solution();

        fixture.service.update(fixture.time.clock());
        assertSame(first, fixture.service.solution());
        assertEquals(1, fixture.predictor.estimateReads);
        assertEquals(1, fixture.predictor.deltaReads);

        fixture.time.nextCycle(0.01);
        fixture.predictor.publishStationary(
                pose(0.0, 0.0, 0.0), 0.80, fixture.time.clock());
        fixture.service.update(fixture.time.clock());

        assertNotSame(first, fixture.service.solution());
        assertEquals(2, fixture.predictor.estimateReads);
        assertEquals(2, fixture.predictor.deltaReads);
        assertEquals(0, fixture.predictor.updateCalls);
    }

    @Test
    public void failedCalculationDoesNotPartiallyPublishAndMayRetrySameCycle() {
        Fixture fixture = new Fixture(testConfig());
        fixture.predictor.publishStationary(
                pose(0.0, 0.0, 0.0), 0.80, fixture.time.clock());
        fixture.service.start(fixture.time.clock());
        ReferenceCoordinatedShotService.Solution prior = fixture.service.solution();

        fixture.time.nextCycle(0.01);
        fixture.predictor.publishStationary(
                pose(5.0, 0.0, 0.0), 0.80, fixture.time.clock());
        RuntimeException failure = new IllegalStateException("delta read failed");
        fixture.predictor.deltaFailure = failure;

        assertSame(failure, assertThrows(RuntimeException.class,
                () -> fixture.service.update(fixture.time.clock())));
        assertSame(prior, fixture.service.solution());
        assertEquals(2, fixture.predictor.estimateReads);
        assertEquals(2, fixture.predictor.deltaReads);

        fixture.predictor.deltaFailure = null;
        fixture.service.update(fixture.time.clock());

        assertNotSame(prior, fixture.service.solution());
        assertEquals(45.0, fixture.service.solution().effectiveForwardInches, EPSILON);
        assertEquals(2, fixture.predictor.estimateReads);
        assertEquals(3, fixture.predictor.deltaReads);
    }

    @Test
    public void movingCompensationUsesCurrentRobotAxesAndMinimumQuality() {
        ReferenceCoordinatedShotService.Config config = testConfig();
        config.targetFieldXInches = -20.0;
        config.targetFieldYInches = 100.0;
        config.illustrativeFlightTimeSec = 0.20;
        Fixture fixture = new Fixture(config);
        LoopTimestamp observation = fixture.time.clock().nowTimestamp();
        fixture.predictor.estimate = new PoseEstimate(
                pose(0.0, 0.0, Math.PI / 2.0), true, 0.80, observation);
        fixture.predictor.delta = new MotionDelta(
                pose(10.0, 0.0, Math.PI / 2.0),
                true,
                0.60,
                fixture.time.clock().timestampSecondsAgo(0.50),
                observation
        );

        fixture.service.start(fixture.time.clock());
        ReferenceCoordinatedShotService.Solution solution = fixture.service.solution();

        assertEquals(ReferenceCoordinatedShotService.Mode.MOVING_COMPENSATED, solution.mode);
        assertEquals(ReferenceCoordinatedShotService.Reason.NONE, solution.reason);
        assertSame(observation, solution.observationTimestamp);
        assertEquals(100.0, solution.effectiveForwardInches, EPSILON);
        assertEquals(24.0, solution.effectiveLeftInches, EPSILON);
        assertEquals(0.60, solution.quality, EPSILON);
        assertEquals(0, fixture.predictor.updateCalls);

        PlantTargetResolution target = resolveRequest(solution, fixture.time.clock());
        assertTrue(target.hasTarget());
        assertEquals(Math.atan2(24.0, 100.0), target.target(), EPSILON);
        assertEquals("reference-coordinated-shot", target.selectedCandidateId());
        assertEquals(0.60, target.selectedQuality(), EPSILON);
        assertSame(observation, target.selectedTimestamp());
    }

    @Test
    public void missingMotionUsesNamedStationaryFallbackAndCanRecover() {
        Fixture fixture = new Fixture(testConfig());
        fixture.predictor.publishStationary(
                pose(0.0, 0.0, 0.0), 0.80, fixture.time.clock());
        fixture.service.start(fixture.time.clock());

        assertStationary(fixture.service.solution(),
                ReferenceCoordinatedShotService.Reason.MOTION_UNAVAILABLE);
        assertEquals(50.0, fixture.service.solution().effectiveForwardInches, EPSILON);
        assertEquals(0.80, fixture.service.solution().quality, EPSILON);

        fixture.time.nextCycle(0.10);
        fixture.predictor.publishMoving(
                pose(0.0, 0.0, 0.0),
                0.80,
                pose(5.0, 0.0, 0.0),
                0.70,
                0.10,
                fixture.time.clock());
        fixture.service.update(fixture.time.clock());

        assertEquals(ReferenceCoordinatedShotService.Mode.MOVING_COMPENSATED,
                fixture.service.solution().mode);
        assertEquals(40.0, fixture.service.solution().effectiveForwardInches, EPSILON);
        assertEquals(0.70, fixture.service.solution().quality, EPSILON);
    }

    @Test
    public void staleAndMismatchedMotionUseDistinctStationaryReasons() {
        ReferenceCoordinatedShotService.Config staleConfig = testConfig();
        staleConfig.motionMaxAgeSec = 0.05;
        Fixture stale = new Fixture(staleConfig);
        LoopTimestamp oldObservation = stale.time.clock().timestampSecondsAgo(0.10);
        stale.predictor.estimate = new PoseEstimate(
                pose(0.0, 0.0, 0.0), true, 0.80, oldObservation);
        stale.predictor.delta = new MotionDelta(
                pose(1.0, 0.0, 0.0), true, 0.80,
                stale.time.clock().timestampSecondsAgo(0.15), oldObservation);
        stale.service.start(stale.time.clock());
        assertStationary(stale.service.solution(),
                ReferenceCoordinatedShotService.Reason.MOTION_STALE);

        Fixture mismatch = new Fixture(testConfig());
        LoopTimestamp observation = mismatch.time.clock().nowTimestamp();
        LoopTimestamp motionEnd = mismatch.time.clock().timestampSecondsAgo(0.01);
        mismatch.predictor.estimate = new PoseEstimate(
                pose(0.0, 0.0, 0.0), true, 0.80, observation);
        mismatch.predictor.delta = new MotionDelta(
                pose(1.0, 0.0, 0.0), true, 0.80,
                mismatch.time.clock().timestampSecondsAgo(0.06), motionEnd);
        mismatch.service.start(mismatch.time.clock());
        assertStationary(mismatch.service.solution(),
                ReferenceCoordinatedShotService.Reason.MOTION_TIMESTAMP_MISMATCH);
    }

    @Test
    public void resetInvalidatedAndInvalidMotionUseDistinctStationaryReasons() {
        ReferenceCoordinatedShotService.Config config = testConfig();
        Fixture reset = new Fixture(config);
        LoopTimestamp oldStart = reset.time.clock().nowTimestamp();
        reset.time.nextCycle(0.05);
        LoopTimestamp oldEnd = reset.time.clock().nowTimestamp();
        MotionDelta oldDelta = new MotionDelta(
                pose(1.0, 0.0, 0.0), true, 0.80, oldStart, oldEnd);
        reset.time.clock().reset(reset.time.clock().nowSec());
        LoopTimestamp current = reset.time.clock().nowTimestamp();
        reset.predictor.estimate = new PoseEstimate(
                pose(0.0, 0.0, 0.0), true, 0.80, current);
        reset.predictor.delta = oldDelta;
        reset.service.start(reset.time.clock());
        assertStationary(reset.service.solution(),
                ReferenceCoordinatedShotService.Reason.MOTION_RESET_INVALIDATED);

        Fixture invalid = new Fixture(testConfig());
        LoopTimestamp invalidEnd = invalid.time.clock().nowTimestamp();
        invalid.predictor.estimate = new PoseEstimate(
                pose(0.0, 0.0, 0.0), true, 0.80, invalidEnd);
        invalid.predictor.delta = new MotionDelta(
                pose(Double.NaN, 0.0, 0.0), true, 0.80,
                invalid.time.clock().timestampSecondsAgo(0.10), invalidEnd);
        invalid.service.start(invalid.time.clock());
        assertStationary(invalid.service.solution(),
                ReferenceCoordinatedShotService.Reason.MOTION_INVALID);
    }

    @Test
    public void unavailableSpatialAndModelFailuresFailClosedWithConfiguredIntents() {
        Fixture noPose = new Fixture(testConfig());
        noPose.predictor.estimate = PoseEstimate.noPose(noPose.time.clock().nowTimestamp());
        noPose.predictor.delta = MotionDelta.none(noPose.time.clock().nowTimestamp());
        noPose.service.start(noPose.time.clock());
        assertUnavailable(noPose.service.solution(),
                ReferenceCoordinatedShotService.Reason.SPATIAL_UNAVAILABLE);

        Fixture invalidSpatial = new Fixture(testConfig());
        invalidSpatial.predictor.publishStationary(
                pose(Double.NaN, 0.0, 0.0), 0.80, invalidSpatial.time.clock());
        invalidSpatial.service.start(invalidSpatial.time.clock());
        assertUnavailable(invalidSpatial.service.solution(),
                ReferenceCoordinatedShotService.Reason.SPATIAL_INVALID);

        ReferenceCoordinatedShotService.Config farConfig = testConfig();
        farConfig.targetFieldXInches = 201.0;
        Fixture far = new Fixture(farConfig);
        far.predictor.publishStationary(
                pose(0.0, 0.0, 0.0), 0.80, far.time.clock());
        far.service.start(far.time.clock());
        assertUnavailable(far.service.solution(),
                ReferenceCoordinatedShotService.Reason.MODEL_DISTANCE_OUT_OF_DOMAIN);

        ReferenceCoordinatedShotService.Config badModel = testConfig();
        badModel.flywheelVelocityTicksPerSecByDistance =
                InterpolatingTable1D.ofSortedPairs(0.0, 4000.0, 200.0, 4000.0);
        Fixture invalidModel = new Fixture(badModel);
        invalidModel.predictor.publishStationary(
                pose(0.0, 0.0, 0.0), 0.80, invalidModel.time.clock());
        invalidModel.service.start(invalidModel.time.clock());
        assertUnavailable(invalidModel.service.solution(),
                ReferenceCoordinatedShotService.Reason.MODEL_OUTPUT_INVALID);

        ReferenceCoordinatedShotService.Solution fallback = invalidModel.service.solution();
        assertFalse(fallback.turretRequest.hasAlternatives());
        assertEquals(0.0, fallback.flywheelVelocityTicksPerSec, 0.0);
        assertEquals(0.10, fallback.hoodPosition, 0.0);
    }

    @Test
    public void differentClockTimestampIsWiringErrorRatherThanFallback() {
        Fixture fixture = new Fixture(testConfig());
        ManualLoopClock otherTime = new ManualLoopClock(10.0);
        LoopTimestamp foreignTimestamp = otherTime.clock().nowTimestamp();
        fixture.predictor.estimate = new PoseEstimate(
                pose(0.0, 0.0, 0.0), true, 0.80, foreignTimestamp);
        fixture.predictor.delta = MotionDelta.none(foreignTimestamp);

        IllegalArgumentException failure = assertThrows(
                IllegalArgumentException.class,
                () -> fixture.service.start(fixture.time.clock()));

        assertTrue(failure.getMessage().contains("different LoopClock"));
        assertEquals(ReferenceCoordinatedShotService.Reason.NOT_STARTED,
                fixture.service.solution().reason);

        Fixture foreignNoDelta = new Fixture(testConfig());
        foreignNoDelta.predictor.estimate = new PoseEstimate(
                pose(0.0, 0.0, 0.0),
                true,
                0.80,
                foreignNoDelta.time.clock().nowTimestamp());
        foreignNoDelta.predictor.delta = MotionDelta.none(foreignTimestamp);

        assertThrows(IllegalArgumentException.class,
                () -> foreignNoDelta.service.start(foreignNoDelta.time.clock()));
        assertEquals(ReferenceCoordinatedShotService.Reason.NOT_STARTED,
                foreignNoDelta.service.solution().reason);
    }

    @Test
    public void stopIsPreStartSafeIdempotentAndTerminalWithoutBorrowedReads() {
        Fixture preStart = new Fixture(testConfig());
        preStart.service.stop();
        preStart.service.stop();

        assertEquals(ReferenceCoordinatedShotService.Reason.STOPPED,
                preStart.service.solution().reason);
        assertEquals(0, preStart.predictor.estimateReads);
        assertEquals(0, preStart.predictor.deltaReads);
        assertThrows(IllegalStateException.class,
                () -> preStart.service.update(preStart.time.clock()));
        assertThrows(IllegalStateException.class,
                () -> preStart.service.start(preStart.time.clock()));

        Fixture active = new Fixture(testConfig());
        active.predictor.publishStationary(
                pose(0.0, 0.0, 0.0), 0.80, active.time.clock());
        active.service.start(active.time.clock());
        int reads = active.predictor.estimateReads + active.predictor.deltaReads;
        active.service.stop();
        active.service.stop();

        assertUnavailable(active.service.solution(),
                ReferenceCoordinatedShotService.Reason.STOPPED);
        assertEquals(reads, active.predictor.estimateReads + active.predictor.deltaReads);
        assertEquals(0, active.predictor.updateCalls);
        assertThrows(IllegalStateException.class,
                () -> active.service.update(active.time.clock()));
        assertEquals(reads, active.predictor.estimateReads + active.predictor.deltaReads);
    }

    private static PlantTargetResolution resolveRequest(
            ReferenceCoordinatedShotService.Solution solution,
            LoopClock clock) {
        PlantTargetResolver resolver = PlantTargets.plan(solution.turretRequest)
                .nearestToMeasurement()
                .rejectUnreachable()
                .whenUnavailable()
                .reportUnavailable();
        PlantTargetContext context = PlantTargetContext.position(
                true,
                0.0,
                ScalarRange.bounded(-2.0 * Math.PI, 2.0 * Math.PI),
                PositionPlant.Periodicity.PERIODIC,
                2.0 * Math.PI,
                Double.NaN,
                Double.NaN
        );
        return resolver.resolve(context, clock);
    }

    private static void assertStationary(
            ReferenceCoordinatedShotService.Solution solution,
            ReferenceCoordinatedShotService.Reason reason) {
        assertEquals(ReferenceCoordinatedShotService.Mode.STATIONARY_FALLBACK, solution.mode);
        assertEquals(reason, solution.reason);
        assertTrue(solution.turretRequest.hasAlternatives());
    }

    private static void assertUnavailable(
            ReferenceCoordinatedShotService.Solution solution,
            ReferenceCoordinatedShotService.Reason reason) {
        assertEquals(ReferenceCoordinatedShotService.Mode.UNAVAILABLE, solution.mode);
        assertEquals(reason, solution.reason);
        assertFalse(solution.turretRequest.hasAlternatives());
    }

    private static ReferenceCoordinatedShotService.Config testConfig() {
        ReferenceCoordinatedShotService.Config config =
                ReferenceCoordinatedShotService.Config.defaults();
        config.targetFieldXInches = 50.0;
        config.targetFieldYInches = 0.0;
        config.spatialMaxAgeSec = 0.50;
        config.spatialMinQuality = 0.20;
        config.motionMaxAgeSec = 0.50;
        config.motionMinQuality = 0.30;
        config.motionMaxIntervalSec = 0.50;
        config.illustrativeFlightTimeSec = 0.20;
        config.minimumModelDistanceInches = 0.0;
        config.maximumModelDistanceInches = 200.0;
        config.flywheelVelocityTicksPerSecByDistance =
                InterpolatingTable1D.ofSortedPairs(0.0, 1000.0, 100.0, 2000.0);
        config.hoodPositionByDistance =
                InterpolatingTable1D.ofSortedPairs(0.0, 0.20, 100.0, 0.60);
        config.minimumFlywheelVelocityTicksPerSec = 0.0;
        config.maximumFlywheelVelocityTicksPerSec = 3000.0;
        config.minimumHoodPosition = 0.0;
        config.maximumHoodPosition = 1.0;
        config.unavailableFlywheelVelocityTicksPerSec = 0.0;
        config.unavailableHoodPosition = 0.10;
        return config;
    }

    private static Pose3d pose(double xInches, double yInches, double yawRad) {
        return new Pose3d(xInches, yInches, 0.0, yawRad, 0.0, 0.0);
    }

    private static final class Fixture {
        private final ManualLoopClock time = new ManualLoopClock(10.0);
        private final FakeMotionPredictor predictor = new FakeMotionPredictor();
        private final ReferenceCoordinatedShotService service;

        private Fixture(ReferenceCoordinatedShotService.Config config) {
            service = new ReferenceCoordinatedShotService(predictor, config);
        }
    }

    private static final class FakeMotionPredictor implements MotionPredictor {
        private PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        private MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());
        private RuntimeException estimateFailure;
        private RuntimeException deltaFailure;
        private int updateCalls;
        private int estimateReads;
        private int deltaReads;

        private void publishStationary(Pose3d pose, double quality, LoopClock clock) {
            LoopTimestamp timestamp = clock.nowTimestamp();
            estimate = new PoseEstimate(pose, true, quality, timestamp);
            delta = MotionDelta.none(timestamp);
        }

        private void publishMoving(Pose3d pose,
                                   double poseQuality,
                                   Pose3d deltaPose,
                                   double deltaQuality,
                                   double durationSec,
                                   LoopClock clock) {
            LoopTimestamp timestamp = clock.nowTimestamp();
            estimate = new PoseEstimate(pose, true, poseQuality, timestamp);
            delta = new MotionDelta(
                    deltaPose,
                    true,
                    deltaQuality,
                    clock.timestampSecondsAgo(durationSec),
                    timestamp
            );
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
        }

        @Override
        public PoseEstimate getEstimate() {
            estimateReads++;
            if (estimateFailure != null) {
                throw estimateFailure;
            }
            return estimate;
        }

        @Override
        public MotionDelta getLatestMotionDelta() {
            deltaReads++;
            if (deltaFailure != null) {
                throw deltaFailure;
            }
            return delta;
        }

        @Override
        public long trajectorySegmentId() {
            return 0L;
        }
    }
}
