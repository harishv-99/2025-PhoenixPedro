package edu.ftcsushi.fw.localization;

import org.junit.Test;

import java.util.concurrent.atomic.AtomicReference;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Deterministic contract tests for the bounded planar trajectory-history owner. */
public final class PlanarPoseHistoryTest {

    private static final double EPSILON = 1.0e-9;

    @Test
    public void defaultsAreDocumentedAndConstructionSnapshotsTheDraft() {
        PlanarPoseHistory.Config defaults = PlanarPoseHistory.Config.defaults();
        assertEquals(0.50, defaults.retentionSec, EPSILON);
        assertEquals(128, defaults.maxSamples);
        assertEquals(0.10, defaults.maxInterpolationGapSec, EPSILON);
        assertEquals(12.0, defaults.maxInterpolationTranslationInches, EPSILON);
        assertEquals(Math.PI / 2.0, defaults.maxInterpolationYawRad, EPSILON);

        LoopClock clock = startedAt(0.0);
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        defaults.maxInterpolationGapSec = 0.20;
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, defaults);
        defaults.maxInterpolationGapSec = 0.0;

        estimator.publish(clock, pose(0.0, 0.0, 0.0), 0.9);
        history.recordCurrent(clock);
        clock.update(0.10);
        estimator.publish(clock, pose(2.0, 0.0, 0.0), 0.8);
        history.recordCurrent(clock);

        assertEquals(
                PlanarPoseHistory.Lookup.Kind.INTERPOLATED,
                history.lookup(clock, clock.timestampSecondsAgo(0.05)).kind()
        );
    }

    @Test
    public void configRejectsInvalidIndividualBounds() {
        assertInvalidDoubleBounds(
                "retentionSec",
                (config, value) -> config.retentionSec = value,
                false
        );

        PlanarPoseHistory.Config tooFew = PlanarPoseHistory.Config.defaults();
        tooFew.maxSamples = 1;
        assertInvalid(tooFew, "maxSamples");
        PlanarPoseHistory.Config tooMany = PlanarPoseHistory.Config.defaults();
        tooMany.maxSamples = 4097;
        assertInvalid(tooMany, "maxSamples");

        assertInvalidDoubleBounds(
                "maxInterpolationGapSec",
                (config, value) -> config.maxInterpolationGapSec = value,
                false
        );
        assertInvalidDoubleBounds(
                "maxInterpolationTranslationInches",
                (config, value) -> config.maxInterpolationTranslationInches = value,
                false
        );
        assertInvalidDoubleBounds(
                "maxInterpolationYawRad",
                (config, value) -> config.maxInterpolationYawRad = value,
                true
        );

        PlanarPoseHistory.Config minimumCount = PlanarPoseHistory.Config.defaults();
        minimumCount.maxSamples = 2;
        new PlanarPoseHistory(new FakeTrajectoryEstimator(), minimumCount);
        PlanarPoseHistory.Config maximumCount = PlanarPoseHistory.Config.defaults();
        maximumCount.maxSamples = 4096;
        new PlanarPoseHistory(new FakeTrajectoryEstimator(), maximumCount);
    }

    @Test
    public void exactAndShortestYawInterpolationPreserveTimestampAndConservativeQuality() {
        LoopClock clock = startedAt(2.0);
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory.Config config = permissiveConfig();
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, config);

        LoopTimestamp firstTimestamp = clock.nowTimestamp();
        estimator.publish(clock, pose(0.0, 2.0, Math.toRadians(170.0)), 0.8);
        history.recordCurrent(clock);
        clock.update(2.10);
        LoopTimestamp secondTimestamp = clock.nowTimestamp();
        estimator.publish(clock, pose(10.0, 6.0, Math.toRadians(-170.0)), 0.4);
        history.recordCurrent(clock);

        PlanarPoseHistory.Lookup exact = history.lookup(clock, firstTimestamp);
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT, exact.kind());
        assertEquals(0.0, exact.fieldToRobotPose().xInches, EPSILON);
        assertEquals(0.8, exact.quality(), EPSILON);
        assertSame(firstTimestamp, exact.timestamp());

        LoopTimestamp midpointTimestamp = clock.timestampSecondsAgo(0.05);
        PlanarPoseHistory.Lookup midpoint = history.lookup(clock, midpointTimestamp);
        assertEquals(PlanarPoseHistory.Lookup.Kind.INTERPOLATED, midpoint.kind());
        assertEquals(5.0, midpoint.fieldToRobotPose().xInches, EPSILON);
        assertEquals(4.0, midpoint.fieldToRobotPose().yInches, EPSILON);
        assertEquals(Math.PI, Math.abs(midpoint.fieldToRobotPose().headingRad), EPSILON);
        assertEquals(0.4, midpoint.quality(), EPSILON);
        assertSame(midpointTimestamp, midpoint.timestamp());
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT,
                history.lookup(clock, secondTimestamp).kind());
    }

    @Test
    public void extremeFiniteYawEndpointsCannotPublishANonfiniteInterpolation() {
        LoopClock clock = startedAt(0.0);
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, permissiveConfig());
        TimeAwareSource<PlanarPoseHistory.Lookup> lookup = history.lookupSource();

        estimator.publish(clock, pose(0.0, 0.0, Double.MAX_VALUE), 1.0);
        history.recordCurrent(clock);
        clock.update(0.10);
        estimator.publish(clock, pose(0.0, 0.0, -Double.MAX_VALUE), 1.0);
        history.recordCurrent(clock);

        PlanarPoseHistory.Lookup midpoint = lookup.getAt(
                clock,
                clock.timestampSecondsAgo(0.05)
        );
        assertTrue(midpoint.isAvailable());
        assertTrue(Double.isFinite(midpoint.fieldToRobotPose().headingRad));
    }

    @Test
    public void lookupNeverExtrapolatesAndGuardsUnavailableFields() {
        LoopClock clock = startedAt(1.0);
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, permissiveConfig());

        LoopTimestamp emptyRequest = clock.nowTimestamp();
        PlanarPoseHistory.Lookup empty = history.lookup(clock, emptyRequest);
        assertUnavailable(empty, PlanarPoseHistory.Lookup.UnavailableReason.EMPTY);
        assertSame(emptyRequest, empty.timestamp());
        assertGuarded(empty::fieldToRobotPose);
        assertGuarded(empty::quality);

        LoopTimestamp unavailableTimestamp = LoopTimestamp.unavailable();
        PlanarPoseHistory.Lookup unavailableQuery = history.lookup(
                clock,
                unavailableTimestamp
        );
        assertUnavailable(
                unavailableQuery,
                PlanarPoseHistory.Lookup.UnavailableReason.QUERY_TIMESTAMP_UNAVAILABLE
        );
        assertSame(unavailableTimestamp, unavailableQuery.timestamp());

        clock.update(2.0);
        LoopTimestamp first = clock.nowTimestamp();
        estimator.publish(clock, pose(1.0, 1.0, 0.0), 1.0);
        history.recordCurrent(clock);
        clock.update(3.0);
        LoopTimestamp last = clock.nowTimestamp();
        estimator.publish(clock, pose(2.0, 1.0, 0.0), 1.0);
        history.recordCurrent(clock);

        assertUnavailable(
                history.lookup(clock, clock.timestampSecondsAgo(2.5)),
                PlanarPoseHistory.Lookup.UnavailableReason.BEFORE_FIRST
        );
        clock.update(4.0);
        assertUnavailable(
                history.lookup(clock, clock.timestampSecondsAgo(0.5)),
                PlanarPoseHistory.Lookup.UnavailableReason.AFTER_LATEST
        );
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT, history.lookup(clock, first).kind());
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT, history.lookup(clock, last).kind());
        assertGuarded(() -> history.lookup(clock, first).unavailableReason());
    }

    @Test
    public void eachInterpolationLimitHasAStableTypedReason() {
        assertBracketReason(
                config -> config.maxInterpolationGapSec = 0.05,
                pose(0.0, 0.0, 0.0),
                pose(1.0, 0.0, 0.0),
                PlanarPoseHistory.Lookup.UnavailableReason.INTERPOLATION_TIME_GAP
        );
        assertBracketReason(
                config -> config.maxInterpolationTranslationInches = 0.5,
                pose(0.0, 0.0, 0.0),
                pose(1.0, 0.0, 0.0),
                PlanarPoseHistory.Lookup.UnavailableReason.INTERPOLATION_TRANSLATION_GAP
        );
        assertBracketReason(
                config -> config.maxInterpolationYawRad = 0.25,
                pose(0.0, 0.0, 0.0),
                pose(0.0, 0.0, 0.5),
                PlanarPoseHistory.Lookup.UnavailableReason.INTERPOLATION_YAW_GAP
        );
    }

    @Test
    public void sourceSegmentAndUnavailableObservationBreakInterpolation() {
        LoopClock clock = startedAt(0.0);
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, permissiveConfig());

        estimator.publish(clock, pose(0.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        clock.update(0.10);
        estimator.segment++;
        estimator.publish(clock, pose(1.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        assertUnavailable(
                history.lookup(clock, clock.timestampSecondsAgo(0.05)),
                PlanarPoseHistory.Lookup.UnavailableReason.DISCONTINUITY
        );

        history.reset();
        clock.update(1.0);
        estimator.publish(clock, pose(0.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        clock.update(1.05);
        estimator.publishNoPose(clock);
        history.recordCurrent(clock);
        clock.update(1.10);
        estimator.publish(clock, pose(1.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        assertUnavailable(
                history.lookup(clock, clock.timestampSecondsAgo(0.05)),
                PlanarPoseHistory.Lookup.UnavailableReason.DISCONTINUITY
        );
    }

    @Test
    public void nonfiniteAndNoncurrentEstimatorPublicationsFormInterpolationGaps() {
        LoopClock clock = startedAt(0.0);
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, permissiveConfig());
        estimator.publish(clock, pose(0.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);

        clock.update(0.025);
        estimator.publish(clock, pose(Double.NaN, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        clock.update(0.050);
        estimator.publish(clock, pose(1.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        assertUnavailable(
                history.lookup(clock, clock.timestampSecondsAgo(0.025)),
                PlanarPoseHistory.Lookup.UnavailableReason.DISCONTINUITY
        );

        clock.update(0.075);
        estimator.publish(clock, pose(1.5, 0.0, 0.0), Double.NaN);
        history.recordCurrent(clock);
        clock.update(0.100);
        estimator.publish(clock, pose(2.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        assertUnavailable(
                history.lookup(clock, clock.timestampSecondsAgo(0.025)),
                PlanarPoseHistory.Lookup.UnavailableReason.DISCONTINUITY
        );

        LoopTimestamp staleTimestamp = clock.nowTimestamp();
        clock.update(0.125);
        estimator.publishAt(staleTimestamp, pose(2.5, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        clock.update(0.150);
        estimator.publish(clock, pose(3.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        assertUnavailable(
                history.lookup(clock, clock.timestampSecondsAgo(0.025)),
                PlanarPoseHistory.Lookup.UnavailableReason.DISCONTINUITY
        );
    }

    @Test
    public void equalTimestampIsIdempotentOnlyForTheSamePublishedValue() {
        LoopClock clock = startedAt(5.0);
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, permissiveConfig());
        LoopTimestamp timestamp = clock.nowTimestamp();

        estimator.publish(clock, pose(1.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        clock.update(5.0);
        estimator.publish(clock, pose(1.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT, history.lookup(clock, timestamp).kind());

        clock.update(5.0);
        estimator.publishNoPose(clock);
        history.recordCurrent(clock);
        clock.update(5.0);
        estimator.publish(clock, pose(1.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        assertUnavailable(
                history.lookup(clock, timestamp),
                PlanarPoseHistory.Lookup.UnavailableReason.DISCONTINUITY
        );

        clock.update(5.0);
        estimator.publish(clock, pose(2.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        assertUnavailable(
                history.lookup(clock, timestamp),
                PlanarPoseHistory.Lookup.UnavailableReason.DISCONTINUITY
        );
    }

    @Test
    public void timeAndCountBoundsEvictOldSamples() {
        LoopClock clock = startedAt(0.0);
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory.Config config = permissiveConfig();
        config.maxSamples = 2;
        config.retentionSec = 0.15;
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, config);

        LoopTimestamp first = clock.nowTimestamp();
        estimator.publish(clock, pose(0.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        clock.update(0.10);
        LoopTimestamp second = clock.nowTimestamp();
        estimator.publish(clock, pose(1.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        clock.update(0.20);
        estimator.publish(clock, pose(2.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);

        assertUnavailable(
                history.lookup(clock, first),
                PlanarPoseHistory.Lookup.UnavailableReason.EVICTED
        );
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT, history.lookup(clock, second).kind());

        clock.update(0.40);
        estimator.publishNoPose(clock);
        history.recordCurrent(clock);
        assertUnavailable(
                history.lookup(clock, clock.nowTimestamp()),
                PlanarPoseHistory.Lookup.UnavailableReason.EVICTED
        );
    }

    @Test
    public void retentionZeroRegressionTinyIntervalsAndSkippedCyclesRemainTruthful() {
        LoopClock horizonClock = startedAt(0.0);
        FakeTrajectoryEstimator horizonEstimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory.Config shortHorizon = permissiveConfig();
        shortHorizon.retentionSec = 0.10;
        PlanarPoseHistory horizonHistory = new PlanarPoseHistory(
                horizonEstimator,
                shortHorizon
        );
        LoopTimestamp beyondHorizon = horizonClock.nowTimestamp();
        horizonEstimator.publish(horizonClock, pose(0.0, 0.0, 0.0), 1.0);
        horizonHistory.recordCurrent(horizonClock);
        horizonClock.update(0.20);
        assertUnavailable(
                horizonHistory.lookupSource().getAt(horizonClock, beyondHorizon),
                PlanarPoseHistory.Lookup.UnavailableReason.EVICTED
        );

        LoopClock retentionClock = startedAt(0.0);
        FakeTrajectoryEstimator retentionEstimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory.Config zeroRetention = permissiveConfig();
        zeroRetention.retentionSec = 0.0;
        PlanarPoseHistory zeroHistory = new PlanarPoseHistory(
                retentionEstimator,
                zeroRetention
        );
        LoopTimestamp evicted = retentionClock.nowTimestamp();
        retentionEstimator.publish(retentionClock, pose(0.0, 0.0, 0.0), 1.0);
        zeroHistory.recordCurrent(retentionClock);
        retentionClock.update(0.1);
        LoopTimestamp retained = retentionClock.nowTimestamp();
        retentionEstimator.publish(retentionClock, pose(1.0, 0.0, 0.0), 1.0);
        zeroHistory.recordCurrent(retentionClock);
        assertUnavailable(
                zeroHistory.lookupSource().getAt(retentionClock, evicted),
                PlanarPoseHistory.Lookup.UnavailableReason.EVICTED
        );
        assertEquals(
                PlanarPoseHistory.Lookup.Kind.EXACT,
                zeroHistory.lookupSource().getAt(retentionClock, retained).kind()
        );

        LoopClock regressionClock = startedAt(1.0);
        FakeTrajectoryEstimator regressionEstimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory regressionHistory = new PlanarPoseHistory(
                regressionEstimator,
                permissiveConfig()
        );
        regressionEstimator.publish(regressionClock, pose(1.0, 0.0, 0.0), 1.0);
        regressionHistory.recordCurrent(regressionClock);
        regressionClock.update(0.5);
        regressionEstimator.publish(regressionClock, pose(2.0, 0.0, 0.0), 1.0);
        regressionHistory.recordCurrent(regressionClock);
        assertEquals(
                2.0,
                regressionHistory.lookupSource().get(regressionClock)
                        .fieldToRobotPose().xInches,
                EPSILON
        );

        LoopClock tinyClock = startedAt(0.0);
        FakeTrajectoryEstimator tinyEstimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory tinyHistory = new PlanarPoseHistory(
                tinyEstimator,
                permissiveConfig()
        );
        LoopTimestamp tinyFirst = tinyClock.nowTimestamp();
        tinyEstimator.publish(tinyClock, pose(0.0, 0.0, 0.0), 1.0);
        tinyHistory.recordCurrent(tinyClock);
        tinyClock.update(Math.nextUp(0.0));
        LoopTimestamp tinySecond = tinyClock.nowTimestamp();
        tinyEstimator.publish(tinyClock, pose(1.0, 0.0, 0.0), 1.0);
        tinyHistory.recordCurrent(tinyClock);
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT,
                tinyHistory.lookupSource().getAt(tinyClock, tinyFirst).kind());
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT,
                tinyHistory.lookupSource().getAt(tinyClock, tinySecond).kind());

        LoopClock skippedClock = startedAt(0.0);
        FakeTrajectoryEstimator skippedEstimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory skippedHistory = new PlanarPoseHistory(
                skippedEstimator,
                permissiveConfig()
        );
        skippedEstimator.publish(skippedClock, pose(0.0, 0.0, 0.0), 1.0);
        skippedHistory.recordCurrent(skippedClock);
        skippedClock.update(0.02);
        skippedClock.update(0.04);
        skippedClock.update(0.08);
        skippedEstimator.publish(skippedClock, pose(8.0, 0.0, 0.0), 1.0);
        skippedHistory.recordCurrent(skippedClock);
        assertEquals(
                4.0,
                skippedHistory.lookupSource().getAt(
                        skippedClock,
                        skippedClock.timestampSecondsAgo(0.04)
                ).fieldToRobotPose().xInches,
                EPSILON
        );
    }

    @Test
    public void recordingIsOncePerCycleRetainsFailuresAndRejectsCaughtReentry() {
        LoopClock clock = startedAt(0.0);
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, permissiveConfig());
        estimator.publish(clock, pose(0.0, 0.0, 0.0), 1.0);

        history.recordCurrent(clock);
        history.recordCurrent(clock);
        assertEquals(1, estimator.estimateReads);

        RuntimeException failure = new IllegalStateException("synthetic estimate failure");
        clock.update(0.1);
        estimator.readFailure = failure;
        assertSame(failure, captureFailure(() -> history.recordCurrent(clock)));
        estimator.readFailure = null;
        assertSame(failure, captureFailure(() -> history.recordCurrent(clock)));

        clock.update(0.2);
        AtomicReference<RuntimeException> nestedFailure = new AtomicReference<>();
        estimator.onEstimateRead = () -> nestedFailure.set(
                captureFailure(() -> history.recordCurrent(clock))
        );
        RuntimeException outerFailure = captureFailure(() -> history.recordCurrent(clock));
        assertSame(nestedFailure.get(), outerFailure);
        assertTrue(outerFailure.getMessage().contains("reentered"));

        estimator.onEstimateRead = null;
        clock.update(0.3);
        estimator.publish(clock, pose(3.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT,
                history.lookup(clock, clock.nowTimestamp()).kind());
    }

    @Test
    public void failedFirstAttemptDoesNotBindClockAndReadinessFailureIsRetainedPerCycle() {
        LoopClock unstarted = new LoopClock();
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, permissiveConfig());

        RuntimeException firstFailure = captureFailure(() -> history.recordCurrent(unstarted));
        RuntimeException repeatedFailure = captureFailure(() -> history.recordCurrent(unstarted));
        assertSame(firstFailure, repeatedFailure);
        assertTrue(firstFailure.getMessage().contains("LoopClock must be initialized"));
        assertEquals(0, estimator.estimateReads);

        LoopClock replacement = startedAt(4.0);
        estimator.publish(replacement, pose(4.0, 0.0, 0.0), 1.0);
        history.recordCurrent(replacement);
        assertEquals(
                PlanarPoseHistory.Lookup.Kind.EXACT,
                history.lookup(replacement, replacement.nowTimestamp()).kind()
        );
    }

    @Test
    public void changingSegmentDuringCachedReadRetainsFailureWithoutPartialAppend() {
        LoopClock clock = startedAt(0.0);
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, permissiveConfig());
        LoopTimestamp first = clock.nowTimestamp();
        estimator.publish(clock, pose(0.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);

        clock.update(0.1);
        LoopTimestamp failedTimestamp = clock.nowTimestamp();
        estimator.publish(clock, pose(1.0, 0.0, 0.0), 1.0);
        estimator.onEstimateRead = () -> estimator.segment++;
        RuntimeException firstFailure = captureFailure(() -> history.recordCurrent(clock));
        estimator.onEstimateRead = null;
        RuntimeException repeatedFailure = captureFailure(() -> history.recordCurrent(clock));
        assertSame(firstFailure, repeatedFailure);
        assertTrue(firstFailure.getMessage().contains("changed trajectorySegmentId"));
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT, history.lookup(clock, first).kind());
        assertUnavailable(
                history.lookup(clock, failedTimestamp),
                PlanarPoseHistory.Lookup.UnavailableReason.AFTER_LATEST
        );

        clock.update(0.2);
        estimator.publish(clock, pose(2.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        assertUnavailable(
                history.lookup(clock, clock.timestampSecondsAgo(0.1)),
                PlanarPoseHistory.Lookup.UnavailableReason.DISCONTINUITY
        );
    }

    @Test
    public void projectionIsStableAndCannotResetConcreteOwnerButOwnerResetRebindsLifecycle() {
        LoopClock firstClock = startedAt(1.0);
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, permissiveConfig());
        estimator.publish(firstClock, pose(4.0, 5.0, 0.3), 0.7);
        history.recordCurrent(firstClock);

        TimeAwareSource<PlanarPoseHistory.Lookup> source = history.lookupSource();
        assertSame(source, history.lookupSource());
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT, source.get(firstClock).kind());
        source.reset();
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT, source.get(firstClock).kind());
        assertEquals(0, estimator.updateCalls);
        assertEquals(0, estimator.resetCalls);

        history.reset();
        assertUnavailable(
                source.get(firstClock),
                PlanarPoseHistory.Lookup.UnavailableReason.EMPTY
        );
        assertEquals(0, estimator.resetCalls);

        LoopClock secondClock = startedAt(7.0);
        estimator.publish(secondClock, pose(7.0, 0.0, 0.0), 1.0);
        history.recordCurrent(secondClock);
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT, source.get(secondClock).kind());
    }

    @Test
    public void priorEpochFutureAndWrongClockQueriesFailClosed() {
        LoopClock clock = startedAt(2.0);
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, permissiveConfig());
        LoopTimestamp oldEpoch = clock.nowTimestamp();
        estimator.publish(clock, pose(0.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);

        clock.reset(3.0);
        assertUnavailable(
                history.lookup(clock, oldEpoch),
                PlanarPoseHistory.Lookup.UnavailableReason.QUERY_TIMESTAMP_NOT_CURRENT
        );
        LoopTimestamp currentAfterReset = clock.nowTimestamp();
        PlanarPoseHistory.Lookup retainedOldHistory = history.lookupSource().getAt(
                clock,
                currentAfterReset
        );
        assertUnavailable(
                retainedOldHistory,
                PlanarPoseHistory.Lookup.UnavailableReason.DISCONTINUITY
        );
        assertSame(currentAfterReset, retainedOldHistory.timestamp());

        estimator.publish(clock, pose(3.0, 0.0, 0.0), 1.0);
        history.recordCurrent(clock);
        assertUnavailable(
                history.lookup(clock, clock.timestampSecondsAgo(0.5)),
                PlanarPoseHistory.Lookup.UnavailableReason.DISCONTINUITY
        );

        LoopClock wrongClock = startedAt(3.0);
        RuntimeException wrong = captureFailure(
                () -> history.lookup(wrongClock, wrongClock.nowTimestamp())
        );
        assertTrue(wrong instanceof IllegalArgumentException);
        assertTrue(wrong.getMessage().contains("different LoopClock"));
    }

    @Test
    public void materiallyFutureQueryRetainsItsRequestedTimestamp() {
        LoopClock clock = startedAt(1.0);
        LoopTimestamp future = clock.nowTimestamp();
        clock.update(0.5);
        PlanarPoseHistory history = new PlanarPoseHistory(
                new FakeTrajectoryEstimator(),
                permissiveConfig()
        );

        PlanarPoseHistory.Lookup lookup = history.lookup(clock, future);
        assertUnavailable(
                lookup,
                PlanarPoseHistory.Lookup.UnavailableReason.QUERY_TIMESTAMP_NOT_CURRENT
        );
        assertSame(future, lookup.timestamp());
    }

    private interface ConfigEdit {
        void apply(PlanarPoseHistory.Config config);
    }

    private interface ConfigValueEdit {
        void apply(PlanarPoseHistory.Config config, double value);
    }

    private static void assertBracketReason(ConfigEdit edit,
                                            Pose2d firstPose,
                                            Pose2d secondPose,
                                            PlanarPoseHistory.Lookup.UnavailableReason reason) {
        LoopClock clock = startedAt(0.0);
        FakeTrajectoryEstimator estimator = new FakeTrajectoryEstimator();
        PlanarPoseHistory.Config config = permissiveConfig();
        edit.apply(config);
        PlanarPoseHistory history = new PlanarPoseHistory(estimator, config);
        estimator.publish(clock, firstPose, 1.0);
        history.recordCurrent(clock);
        clock.update(0.10);
        estimator.publish(clock, secondPose, 1.0);
        history.recordCurrent(clock);
        assertUnavailable(history.lookup(clock, clock.timestampSecondsAgo(0.05)), reason);
    }

    private static void assertInvalidDoubleBounds(String field,
                                                  ConfigValueEdit edit,
                                                  boolean rejectAbovePi) {
        double[] invalid = rejectAbovePi
                ? new double[]{Double.NaN, Double.POSITIVE_INFINITY, -0.01, Math.nextUp(Math.PI)}
                : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -0.01};
        for (double value : invalid) {
            PlanarPoseHistory.Config config = PlanarPoseHistory.Config.defaults();
            edit.apply(config, value);
            assertInvalid(config, field);
        }
    }

    private static void assertInvalid(PlanarPoseHistory.Config config, String field) {
        RuntimeException failure = captureFailure(
                () -> new PlanarPoseHistory(new FakeTrajectoryEstimator(), config)
        );
        assertTrue(failure instanceof IllegalArgumentException);
        assertTrue(failure.getMessage().contains(field));
    }

    private static void assertUnavailable(
            PlanarPoseHistory.Lookup lookup,
            PlanarPoseHistory.Lookup.UnavailableReason reason) {
        assertFalse(lookup.isAvailable());
        assertEquals(PlanarPoseHistory.Lookup.Kind.UNAVAILABLE, lookup.kind());
        assertEquals(reason, lookup.unavailableReason());
    }

    private static void assertGuarded(Runnable access) {
        RuntimeException failure = captureFailure(access);
        assertTrue(failure instanceof IllegalStateException);
    }

    private static RuntimeException captureFailure(Runnable action) {
        try {
            action.run();
            fail("Expected RuntimeException");
            return new AssertionErrorAdapter();
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static PlanarPoseHistory.Config permissiveConfig() {
        PlanarPoseHistory.Config config = PlanarPoseHistory.Config.defaults();
        config.retentionSec = 10.0;
        config.maxSamples = 64;
        config.maxInterpolationGapSec = 5.0;
        config.maxInterpolationTranslationInches = 100.0;
        config.maxInterpolationYawRad = Math.PI;
        return config;
    }

    private static LoopClock startedAt(double nowSec) {
        LoopClock clock = new LoopClock();
        clock.reset(nowSec);
        return clock;
    }

    private static Pose2d pose(double xInches, double yInches, double headingRad) {
        return new Pose2d(xInches, yInches, headingRad);
    }

    private static final class FakeTrajectoryEstimator implements PoseTrajectoryEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        long segment;
        int estimateReads;
        int updateCalls;
        int resetCalls;
        RuntimeException readFailure;
        Runnable onEstimateRead;

        void publish(LoopClock clock, Pose2d pose, double quality) {
            publishAt(clock.nowTimestamp(), pose, quality);
        }

        void publishAt(LoopTimestamp timestamp, Pose2d pose, double quality) {
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
                    quality,
                    timestamp
            );
        }

        void publishNoPose(LoopClock clock) {
            estimate = PoseEstimate.noPose(clock.nowTimestamp());
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
        }

        @Override
        public PoseEstimate getEstimate() {
            estimateReads++;
            if (onEstimateRead != null) {
                onEstimateRead.run();
            }
            if (readFailure != null) {
                throw readFailure;
            }
            return estimate;
        }

        @Override
        public long trajectorySegmentId() {
            return segment;
        }

        void reset() {
            resetCalls++;
        }
    }

    /** Unreachable adapter satisfying Java's definite-return analysis after JUnit fail(). */
    private static final class AssertionErrorAdapter extends RuntimeException {
    }
}
