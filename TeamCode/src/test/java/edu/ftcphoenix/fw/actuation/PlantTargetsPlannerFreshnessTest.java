package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies timestamp-canonical freshness and quality acceptance for Plant target planning. */
public final class PlantTargetsPlannerFreshnessTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void timelessIntentIgnoresObservationAgeAndReportsNoTimingMetadata() {
        ManualLoopClock time = new ManualLoopClock(100.0);
        PlantTargetResolver planner = ready(Source.constant(
                PlantTargetRequest.exact("preset", 12.0)))
                .accept()
                .maxObservationAgeSec(0.0)
                .minQuality(1.0)
                .doneAccept()
                .whenUnavailable().reportUnavailable();

        PlantTargetResolution plan = planner.resolve(context(0.0), time.clock());

        assertTrue(plan.hasTarget());
        assertEquals(12.0, plan.target(), EPSILON);
        assertEquals("preset", plan.selectedCandidateId());
        assertEquals(1.0, plan.selectedQuality(), EPSILON);
        assertTrue(Double.isNaN(plan.selectedAgeSec()));
        assertFalse(plan.selectedTimestamp().isAvailable());
    }

    @Test
    public void nonCandidatePlansDoNotClaimObservationTiming() {
        PlantTargetResolution[] resolutions = {
                PlantTargetResolution.exact(1.0, "exact"),
                PlantTargetResolution.fallback(2.0, "fallback"),
                PlantTargetResolution.holdLast(3.0, "hold last"),
                PlantTargetResolution.holdMeasured(4.0, "hold measured"),
                PlantTargetResolution.unavailable("unavailable")
        };

        for (PlantTargetResolution resolution : resolutions) {
            assertTrue(Double.isNaN(resolution.selectedAgeSec()));
            assertFalse(resolution.selectedTimestamp().isAvailable());
        }
    }

    @Test
    public void cachedObservationAgesThroughInclusiveMaximumBoundary() {
        ManualLoopClock time = new ManualLoopClock(5.0);
        LoopTimestamp captured = time.clock().nowTimestamp();
        PlantTargetRequest cached = PlantTargetRequest.observedExact(
                "camera", 7.0, 0.8, captured);
        PlantTargetResolver planner = ready(Source.constant(cached))
                .accept().maxObservationAgeSec(1.0).doneAccept()
                .whenUnavailable().reportUnavailable();

        PlantTargetResolution newResolution = planner.resolve(context(0.0), time.clock());
        PlantTargetResolution boundaryResolution =
                planner.resolve(context(0.0), time.nextCycle(1.0));
        PlantTargetResolution staleResolution =
                planner.resolve(context(0.0), time.nextCycle(0.001));

        assertEquals(0.0, newResolution.selectedAgeSec(), EPSILON);
        assertEquals(1.0, boundaryResolution.selectedAgeSec(), EPSILON);
        assertSame(captured, boundaryResolution.selectedTimestamp());
        assertFalse(staleResolution.hasTarget());
    }

    @Test
    public void validOldObservationRemainsEligibleWithoutMaximumAgePolicy() {
        ManualLoopClock time = new ManualLoopClock(100.0);
        LoopTimestamp oldTimestamp = time.clock().timestampSecondsAgo(99.0);
        PlantTargetResolver planner = ready(Source.constant(
                PlantTargetRequest.observedExact("old", 3.0, 0.4, oldTimestamp)))
                .whenUnavailable().reportUnavailable();

        PlantTargetResolution plan = planner.resolve(context(0.0), time.clock());

        assertTrue(plan.hasTarget());
        assertEquals(3.0, plan.target(), EPSILON);
        assertEquals(99.0, plan.selectedAgeSec(), EPSILON);
        assertSame(oldTimestamp, plan.selectedTimestamp());
        assertEquals(0.4, plan.selectedQuality(), EPSILON);
    }

    @Test
    public void malformedObservedMetadataSkipsToLaterValidCandidate() {
        ManualLoopClock time = new ManualLoopClock(10.0);
        LoopTimestamp now = time.clock().nowTimestamp();
        double[] invalidQualities = {
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY,
                -0.01,
                1.01
        };

        for (double quality : invalidQualities) {
            PlantTargetRequest request = PlantTargetRequest.oneOf(
                    PlantTargetRequest.observedExact(
                            "invalid", 0.0, quality, now),
                    PlantTargetRequest.observedExact(
                            "valid", 9.0, 0.5, now));
            PlantTargetResolution plan = ready(Source.constant(request))
                    .whenUnavailable().reportUnavailable()
                    .resolve(context(0.0), time.clock());

            assertTrue(plan.hasTarget());
            assertEquals("valid", plan.selectedCandidateId());
            assertEquals(9.0, plan.target(), EPSILON);
        }

        PlantTargetRequest unavailableTimestamp = PlantTargetRequest.oneOf(
                PlantTargetRequest.observedExact(
                        "invalid", 0.0, 0.5, LoopTimestamp.unavailable()),
                PlantTargetRequest.observedExact("valid", 9.0, 0.5, now));
        PlantTargetResolution plan = ready(Source.constant(unavailableTimestamp))
                .whenUnavailable().reportUnavailable()
                .resolve(context(0.0), time.clock());

        assertEquals("valid", plan.selectedCandidateId());
    }

    @Test
    public void slightFutureTimestampWithinNumericToleranceHasZeroAge() {
        LoopClock clock = new LoopClock();
        clock.reset(10.0);
        LoopTimestamp timestamp = clock.nowTimestamp();
        clock.update(10.0 - 0.5e-6);
        PlantTargetResolver planner = ready(Source.constant(
                PlantTargetRequest.observedExact("near-future", 4.0, 0.7, timestamp)))
                .accept().maxObservationAgeSec(0.0).doneAccept()
                .whenUnavailable().reportUnavailable();

        PlantTargetResolution plan = planner.resolve(context(0.0), clock);

        assertTrue(plan.hasTarget());
        assertEquals(0.0, plan.selectedAgeSec(), EPSILON);
        assertSame(timestamp, plan.selectedTimestamp());
    }

    @Test
    public void minimumQualityIsInclusiveAndRejectsLowerQualityCandidate() {
        ManualLoopClock time = new ManualLoopClock(10.0);
        LoopTimestamp timestamp = time.clock().nowTimestamp();
        PlantTargetRequest request = PlantTargetRequest.oneOf(
                PlantTargetRequest.observedExact("below", 0.0, 0.499, timestamp),
                PlantTargetRequest.observedExact("boundary", 8.0, 0.5, timestamp));
        PlantTargetResolver planner = ready(Source.constant(request))
                .accept().minQuality(0.5).doneAccept()
                .whenUnavailable().reportUnavailable();

        PlantTargetResolution plan = planner.resolve(context(0.0), time.clock());

        assertTrue(plan.hasTarget());
        assertEquals("boundary", plan.selectedCandidateId());
        assertEquals(0.5, plan.selectedQuality(), EPSILON);
    }

    @Test
    public void acceptanceBuilderRejectsInvalidThresholdsImmediately() {
        double[] invalidAges = {
                -0.001,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };
        for (double value : invalidAges) {
            expectIllegalArgument("maxObservationAgeSec", () -> ready(requestSource())
                    .accept().maxObservationAgeSec(value));
        }

        double[] invalidQualities = {
                -0.001,
                1.001,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };
        for (double value : invalidQualities) {
            expectIllegalArgument("minQuality", () -> ready(requestSource())
                    .accept().minQuality(value));
        }
    }

    @Test
    public void staleObservationUsesEachExplicitUnavailablePolicyAndCanRecover() {
        ManualLoopClock time = new ManualLoopClock();
        final PlantTargetRequest[] request = {
                PlantTargetRequest.observedExact(
                        "fresh", 7.0, 0.8, time.clock().nowTimestamp())
        };
        Source<PlantTargetRequest> requestSource = clock -> request[0];
        PlantTargetResolver report = ageLimited(requestSource)
                .whenUnavailable().reportUnavailable();
        PlantTargetResolver fallback = ageLimited(requestSource)
                .whenUnavailable().fallbackTo(-1.0);
        PlantTargetResolver holdLast = ageLimited(requestSource)
                .whenUnavailable().holdLastTarget(-2.0);
        PlantTargetResolver holdMeasured = ageLimited(requestSource)
                .whenUnavailable().holdMeasuredTargetOnEntry(-3.0);

        assertEquals(7.0, report.resolve(context(0.0), time.clock()).target(), EPSILON);
        assertEquals(7.0, fallback.resolve(context(0.0), time.clock()).target(), EPSILON);
        assertEquals(7.0, holdLast.resolve(context(0.0), time.clock()).target(), EPSILON);
        assertEquals(7.0, holdMeasured.resolve(context(0.0), time.clock()).target(), EPSILON);

        LoopClock staleClock = time.nextCycle(1.0);
        assertFalse(report.resolve(context(3.0), staleClock).hasTarget());
        assertPlan(fallback.resolve(context(3.0), staleClock),
                PlantTargetResolution.Kind.FALLBACK, -1.0);
        assertPlan(holdLast.resolve(context(3.0), staleClock),
                PlantTargetResolution.Kind.HOLD_LAST_TARGET, 7.0);
        assertPlan(holdMeasured.resolve(context(3.0), staleClock),
                PlantTargetResolution.Kind.HOLD_MEASURED_TARGET, 3.0);

        LoopClock recoveryClock = time.nextCycle(0.0);
        request[0] = PlantTargetRequest.observedExact(
                "recovered", 8.0, 0.8, recoveryClock.nowTimestamp());
        assertPlannedTarget(report, recoveryClock, 8.0);
        assertPlannedTarget(fallback, recoveryClock, 8.0);
        assertPlannedTarget(holdLast, recoveryClock, 8.0);
        assertPlannedTarget(holdMeasured, recoveryClock, 8.0);
    }

    @Test
    public void plannerMemoizesWithinCycleAndResetForcesFreshResolution() {
        ManualLoopClock time = new ManualLoopClock(10.0);
        ProbeRequestSource request = new ProbeRequestSource(
                PlantTargetRequest.observedExact(
                        "fresh", 5.0, 0.8, time.clock().nowTimestamp()));
        PlantTargetResolver planner = ready(request)
                .accept().maxObservationAgeSec(0.5).doneAccept()
                .whenUnavailable().reportUnavailable();

        PlantTargetResolution first = planner.resolve(context(0.0), time.clock());
        request.request = PlantTargetRequest.observedExact(
                "stale", 6.0, 0.8, time.clock().timestampSecondsAgo(10.0));
        PlantTargetResolution sameCycle = planner.resolve(context(0.0), time.clock());

        assertSame(first, sameCycle);
        assertEquals(1, request.samples);

        planner.reset();
        PlantTargetResolution afterReset = planner.resolve(context(0.0), time.clock());

        assertFalse(afterReset.hasTarget());
        assertEquals(2, request.samples);
        assertEquals(1, request.resets);

        request.request = PlantTargetRequest.observedExact(
                "next", 7.0, 0.8, time.clock().nowTimestamp());
        PlantTargetResolution nextCycle = planner.resolve(context(0.0), time.nextCycle(0.0));

        assertEquals("next", nextCycle.selectedCandidateId());
        assertEquals(3, request.samples);
    }

    @Test
    public void requestFailuresRetryWithoutPublishingDefaultOrPriorResolution() {
        ManualLoopClock time = new ManualLoopClock();
        ProbeRequestSource request = new ProbeRequestSource(
                PlantTargetRequest.exact("first", 5.0));
        PlantTargetResolver planner = ready(request)
                .whenUnavailable().reportUnavailable();
        RuntimeException firstFailure = new IllegalStateException("first request failure");
        request.sampleFailure = firstFailure;

        assertSame(firstFailure,
                captureFailure(() -> planner.resolve(context(0.0), time.clock())));
        request.sampleFailure = null;
        PlantTargetResolution first = planner.resolve(context(0.0), time.clock());
        assertEquals("first", first.selectedCandidateId());
        assertEquals(5.0, first.target(), EPSILON);

        request.request = PlantTargetRequest.exact("second", 6.0);
        RuntimeException laterFailure = new IllegalArgumentException("later request failure");
        request.sampleFailure = laterFailure;
        LoopClock nextCycle = time.nextCycle(0.02);
        assertSame(laterFailure,
                captureFailure(() -> planner.resolve(context(0.0), nextCycle)));
        request.sampleFailure = null;
        PlantTargetResolution second = planner.resolve(context(0.0), nextCycle);

        assertEquals("second", second.selectedCandidateId());
        assertEquals(6.0, second.target(), EPSILON);
        assertNotSame(first, second);
        assertSame(second, planner.resolve(context(99.0), nextCycle));
        assertEquals(4, request.samples);
    }

    @Test
    public void recursiveRequestResolutionFailsFastAndAValueRetryCanRecover() {
        final PlantTargetResolver[] planner = new PlantTargetResolver[1];
        final boolean[] recurse = {true};
        Source<PlantTargetRequest> request = clock -> {
            if (recurse[0]) planner[0].resolve(context(0.0), clock);
            return PlantTargetRequest.exact("recovered", 7.0);
        };
        planner[0] = ready(request).whenUnavailable().reportUnavailable();
        ManualLoopClock time = new ManualLoopClock();

        RuntimeException reentry = captureFailure(
                () -> planner[0].resolve(context(0.0), time.clock()));
        assertTrue(reentry instanceof IllegalStateException);
        assertTrue(reentry.getMessage().contains("reentered"));

        recurse[0] = false;
        PlantTargetResolution recovered =
                planner[0].resolve(context(0.0), time.clock());
        assertEquals("recovered", recovered.selectedCandidateId());
        assertEquals(7.0, recovered.target(), EPSILON);
    }

    @Test
    public void failedRequestResetKeepsCommittedPlannerStateUntilResetSucceeds() {
        ManualLoopClock time = new ManualLoopClock();
        ProbeRequestSource request = new ProbeRequestSource(
                PlantTargetRequest.exact("committed", 5.0));
        PlantTargetResolver planner = ready(request)
                .whenUnavailable().reportUnavailable();
        PlantTargetResolution committed = planner.resolve(context(0.0), time.clock());
        request.resetAction = () -> planner.resolve(context(0.0), time.clock());

        RuntimeException overlap = captureFailure(planner::reset);
        assertTrue(overlap instanceof IllegalStateException);
        assertTrue(overlap.getMessage().contains("reset is in progress"));
        assertSame(committed, planner.resolve(context(0.0), time.clock()));
        assertEquals(1, request.samples);

        request.resetAction = null;
        planner.reset();
        PlantTargetResolution afterReset = planner.resolve(context(0.0), time.clock());
        assertNotSame(committed, afterReset);
        assertEquals(2, request.samples);
        assertEquals(2, request.resets);
    }

    @Test
    public void sameValueClockResetMissesCycleCacheAndRejectsPriorEpochObservation() {
        ManualLoopClock time = new ManualLoopClock(10.0);
        ProbeRequestSource request = new ProbeRequestSource(
                PlantTargetRequest.observedExact(
                        "before-reset", 5.0, 0.8, time.clock().nowTimestamp()));
        PlantTargetResolver planner = ready(request)
                .whenUnavailable().reportUnavailable();

        PlantTargetResolution beforeReset = planner.resolve(context(0.0), time.clock());
        long oldCycle = time.clock().cycle();

        time.clock().reset(10.0);
        PlantTargetResolution afterReset = planner.resolve(context(0.0), time.clock());

        assertTrue(beforeReset.hasTarget());
        assertTrue(time.clock().cycle() > oldCycle);
        assertFalse(afterReset.hasTarget());
        assertEquals(2, request.samples);
        assertTrue(afterReset.reason().contains("current LoopClock epoch"));
    }

    @Test
    public void unavailableReasonExplainsTheFirstRejectedObservation() {
        ManualLoopClock time = new ManualLoopClock(10.0);
        LoopTimestamp now = time.clock().nowTimestamp();
        PlantTargetRequest request = PlantTargetRequest.oneOf(
                PlantTargetRequest.observedExact("bad-quality", 1.0, -0.1, now),
                PlantTargetRequest.observedExact(
                        "stale", 2.0, 0.8, time.clock().timestampSecondsAgo(10.0)));
        PlantTargetResolver planner = ready(Source.constant(request))
                .accept().maxObservationAgeSec(0.5).doneAccept()
                .whenUnavailable().reportUnavailable();

        PlantTargetResolution plan = planner.resolve(context(0.0), time.clock());

        assertFalse(plan.hasTarget());
        assertTrue(plan.reason().contains("bad-quality"));
        assertTrue(plan.reason().contains("quality"));
    }

    @Test
    public void observedFactoryFamiliesPreserveSelectionAndMetadataThroughRequests() {
        LoopClock clock = new LoopClock();
        clock.reset(16.0);
        LoopTimestamp[] timestamps = {
                clock.timestampSecondsAgo(5.0),
                clock.timestampSecondsAgo(4.0),
                clock.timestampSecondsAgo(3.0),
                clock.timestampSecondsAgo(2.0),
                clock.timestampSecondsAgo(1.0),
                clock.nowTimestamp()
        };
        PlantTargetRequest[] requests = {
                PlantTargetRequest.observedExact("exact", 1.0, 0.40, timestamps[0]),
                PlantTargetRequest.observedEquivalentPosition(
                        "equivalent", 2.0, 0.50, timestamps[1]),
                PlantTargetRequest.observedPeriodic(
                        "periodic", 3.0, 360.0, 0.60, timestamps[2]),
                PlantTargetRequest.observedRelative("relative", 4.0, 0.70, timestamps[3]),
                PlantTargetRequest.observedRelativeEquivalentPosition(
                        "relative-equivalent", 5.0, 0.80, timestamps[4]),
                PlantTargetRequest.observedRelativePeriodic(
                        "relative-periodic", 6.0, 360.0, 0.90, timestamps[5])
        };
        String[] ids = {
                "exact", "equivalent", "periodic", "relative",
                "relative-equivalent", "relative-periodic"
        };
        double[] expectedTargets = {1.0, 2.0, 3.0, 14.0, 15.0, 16.0};
        double[] qualities = {0.40, 0.50, 0.60, 0.70, 0.80, 0.90};
        PlantTargetContext periodicContext = PlantTargetContext.position(
                true,
                10.0,
                ScalarRange.bounded(-1000.0, 1000.0),
                PositionPlant.Periodicity.PERIODIC,
                360.0,
                Double.NaN,
                Double.NaN);

        for (int i = 0; i < requests.length; i++) {
            PlantTargetResolution resolution = ready(Source.constant(requests[i]))
                    .whenUnavailable().reportUnavailable()
                    .resolve(periodicContext, clock);

            assertTrue(resolution.hasTarget());
            assertEquals(ids[i], resolution.selectedCandidateId());
            assertEquals(expectedTargets[i], resolution.target(), EPSILON);
            assertEquals(qualities[i], resolution.selectedQuality(), EPSILON);
            assertSame(timestamps[i], resolution.selectedTimestamp());
            assertEquals(5.0 - i, resolution.selectedAgeSec(), EPSILON);
        }
    }

    @Test
    public void observedRelativeFactoriesPreserveRelativeTargetSemantics() {
        ManualLoopClock time = new ManualLoopClock(20.0);
        LoopTimestamp timestamp = time.clock().nowTimestamp();
        PlantTargetContext context = PlantTargetContext.position(
                true,
                10.0,
                ScalarRange.bounded(-1000.0, 1000.0),
                PositionPlant.Periodicity.PERIODIC,
                360.0,
                Double.NaN,
                Double.NaN);
        PlantTargetRequest[] requests = {
                PlantTargetRequest.observedRelative("relative", 5.0, 0.8, timestamp),
                PlantTargetRequest.observedRelativeEquivalentPosition(
                        "relative-equivalent", 5.0, 0.8, timestamp),
                PlantTargetRequest.observedRelativePeriodic(
                        "relative-periodic", 5.0, 360.0, 0.8, timestamp)
        };

        for (PlantTargetRequest request : requests) {
            PlantTargetResolution plan = ready(Source.constant(request))
                    .whenUnavailable().reportUnavailable()
                    .resolve(context, time.clock());

            assertTrue(plan.hasTarget());
            assertEquals(15.0, plan.target(), EPSILON);
            assertEquals(0.0, plan.selectedAgeSec(), EPSILON);
            assertSame(timestamp, plan.selectedTimestamp());
        }
    }

    private static PlantTargets.PlanReadyStage ready(Source<PlantTargetRequest> request) {
        return PlantTargets.plan(request)
                .nearestToMeasurement()
                .rejectUnreachable();
    }

    private static PlantTargets.PlanReadyStage ageLimited(Source<PlantTargetRequest> request) {
        return ready(request)
                .accept().maxObservationAgeSec(0.1).doneAccept();
    }

    private static Source<PlantTargetRequest> requestSource() {
        return Source.constant(PlantTargetRequest.exact("test", 1.0));
    }

    private static PlantTargetContext context(double measurement) {
        return PlantTargetContext.simple(
                true,
                measurement,
                ScalarRange.unbounded(),
                Double.NaN,
                Double.NaN);
    }

    private static void assertPlan(PlantTargetResolution plan,
                                   PlantTargetResolution.Kind kind,
                                   double target) {
        assertTrue(plan.hasTarget());
        assertEquals(kind, plan.kind());
        assertEquals(target, plan.target(), EPSILON);
    }

    private static void assertPlannedTarget(PlantTargetResolver resolver,
                                            LoopClock clock,
                                            double target) {
        PlantTargetResolution plan = resolver.resolve(context(0.0), clock);
        assertPlan(plan, PlantTargetResolution.Kind.PLANNED_CANDIDATE, target);
    }

    private static void expectIllegalArgument(String messageFragment, Runnable action) {
        try {
            action.run();
            fail("Expected IllegalArgumentException containing " + messageFragment);
        } catch (IllegalArgumentException expected) {
            assertTrue("Expected message containing " + messageFragment + ", got: "
                            + expected.getMessage(),
                    expected.getMessage() != null
                            && expected.getMessage().contains(messageFragment));
        }
    }

    private static RuntimeException captureFailure(Runnable action) {
        try {
            action.run();
            fail("Expected RuntimeException");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static final class ProbeRequestSource implements Source<PlantTargetRequest> {
        private PlantTargetRequest request;
        private int samples;
        private int resets;
        private RuntimeException sampleFailure;
        private Runnable resetAction;

        private ProbeRequestSource(PlantTargetRequest request) {
            this.request = request;
        }

        @Override
        public PlantTargetRequest get(LoopClock clock) {
            samples++;
            if (sampleFailure != null) throw sampleFailure;
            return request;
        }

        @Override
        public void reset() {
            resets++;
            if (resetAction != null) resetAction.run();
        }
    }
}
