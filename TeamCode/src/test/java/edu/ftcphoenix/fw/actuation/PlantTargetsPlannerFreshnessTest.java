package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies timestamp-canonical freshness and quality acceptance for Plant target planning. */
public final class PlantTargetsPlannerFreshnessTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void timelessIntentIgnoresObservationAgeAndReportsNoTimingMetadata() {
        ManualLoopClock time = new ManualLoopClock(100.0);
        PlantTargetSource planner = ready(Source.constant(
                PlantTargetRequest.exact("preset", 12.0)))
                .accept()
                .maxObservationAgeSec(0.0)
                .minQuality(1.0)
                .doneAccept()
                .whenUnavailable().reportUnavailable();

        PlantTargetPlan plan = planner.resolve(context(0.0), time.clock());

        assertTrue(plan.hasTarget());
        assertEquals(12.0, plan.target(), EPSILON);
        assertEquals("preset", plan.selectedCandidateId());
        assertEquals(1.0, plan.selectedQuality(), EPSILON);
        assertTrue(Double.isNaN(plan.selectedAgeSec()));
        assertTrue(Double.isNaN(plan.selectedTimestampSec()));
    }

    @Test
    public void nonCandidatePlansDoNotClaimObservationTiming() {
        PlantTargetPlan[] plans = {
                PlantTargetPlan.exact(1.0, "exact"),
                PlantTargetPlan.fallback(2.0, "fallback"),
                PlantTargetPlan.holdLast(3.0, "hold last"),
                PlantTargetPlan.holdMeasured(4.0, "hold measured"),
                PlantTargetPlan.unavailable("unavailable")
        };

        for (PlantTargetPlan plan : plans) {
            assertTrue(Double.isNaN(plan.selectedAgeSec()));
            assertTrue(Double.isNaN(plan.selectedTimestampSec()));
        }
    }

    @Test
    public void cachedObservationAgesThroughInclusiveMaximumBoundary() {
        PlantTargetRequest cached = PlantTargetRequest.observedExact(
                "camera", 7.0, 0.8, 5.0);
        PlantTargetSource planner = ready(Source.constant(cached))
                .accept().maxObservationAgeSec(1.0).doneAccept()
                .whenUnavailable().reportUnavailable();
        ManualLoopClock time = new ManualLoopClock(5.0);

        PlantTargetPlan newPlan = planner.resolve(context(0.0), time.clock());
        PlantTargetPlan boundaryPlan = planner.resolve(context(0.0), time.nextCycle(1.0));
        PlantTargetPlan stalePlan = planner.resolve(context(0.0), time.nextCycle(0.001));

        assertEquals(0.0, newPlan.selectedAgeSec(), EPSILON);
        assertEquals(1.0, boundaryPlan.selectedAgeSec(), EPSILON);
        assertEquals(5.0, boundaryPlan.selectedTimestampSec(), EPSILON);
        assertFalse(stalePlan.hasTarget());
    }

    @Test
    public void validOldObservationRemainsEligibleWithoutMaximumAgePolicy() {
        PlantTargetSource planner = ready(Source.constant(
                PlantTargetRequest.observedExact("old", 3.0, 0.4, 1.0)))
                .whenUnavailable().reportUnavailable();
        ManualLoopClock time = new ManualLoopClock(100.0);

        PlantTargetPlan plan = planner.resolve(context(0.0), time.clock());

        assertTrue(plan.hasTarget());
        assertEquals(3.0, plan.target(), EPSILON);
        assertEquals(99.0, plan.selectedAgeSec(), EPSILON);
        assertEquals(1.0, plan.selectedTimestampSec(), EPSILON);
        assertEquals(0.4, plan.selectedQuality(), EPSILON);
    }

    @Test
    public void malformedObservedMetadataSkipsToLaterValidCandidate() {
        ManualLoopClock time = new ManualLoopClock(10.0);
        double nowSec = time.clock().nowSec();
        double[][] invalidMetadata = {
                {Double.NaN, nowSec},
                {Double.POSITIVE_INFINITY, nowSec},
                {Double.NEGATIVE_INFINITY, nowSec},
                {-0.01, nowSec},
                {1.01, nowSec},
                {0.5, Double.NaN},
                {0.5, Double.POSITIVE_INFINITY},
                {0.5, Double.NEGATIVE_INFINITY},
                {0.5, nowSec + 2.0e-6}
        };

        for (double[] metadata : invalidMetadata) {
            PlantTargetRequest request = PlantTargetRequest.oneOf(
                    PlantTargetCandidate.observedExact(
                            "invalid", 0.0, metadata[0], metadata[1]),
                    PlantTargetCandidate.observedExact(
                            "valid", 9.0, 0.5, nowSec));
            PlantTargetPlan plan = ready(Source.constant(request))
                    .whenUnavailable().reportUnavailable()
                    .resolve(context(0.0), time.clock());

            assertTrue(plan.hasTarget());
            assertEquals("valid", plan.selectedCandidateId());
            assertEquals(9.0, plan.target(), EPSILON);
        }
    }

    @Test
    public void slightFutureTimestampWithinNumericToleranceHasZeroAge() {
        ManualLoopClock time = new ManualLoopClock(10.0);
        double timestampSec = time.clock().nowSec() + 0.5e-6;
        PlantTargetSource planner = ready(Source.constant(
                PlantTargetRequest.observedExact("near-future", 4.0, 0.7, timestampSec)))
                .accept().maxObservationAgeSec(0.0).doneAccept()
                .whenUnavailable().reportUnavailable();

        PlantTargetPlan plan = planner.resolve(context(0.0), time.clock());

        assertTrue(plan.hasTarget());
        assertEquals(0.0, plan.selectedAgeSec(), EPSILON);
        assertEquals(timestampSec, plan.selectedTimestampSec(), EPSILON);
    }

    @Test
    public void minimumQualityIsInclusiveAndRejectsLowerQualityCandidate() {
        PlantTargetRequest request = PlantTargetRequest.oneOf(
                PlantTargetCandidate.observedExact("below", 0.0, 0.499, 10.0),
                PlantTargetCandidate.observedExact("boundary", 8.0, 0.5, 10.0));
        PlantTargetSource planner = ready(Source.constant(request))
                .accept().minQuality(0.5).doneAccept()
                .whenUnavailable().reportUnavailable();

        PlantTargetPlan plan = planner.resolve(
                context(0.0), new ManualLoopClock(10.0).clock());

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
        final PlantTargetRequest[] request = {
                PlantTargetRequest.observedExact("fresh", 7.0, 0.8, 0.0)
        };
        Source<PlantTargetRequest> requestSource = clock -> request[0];
        PlantTargetSource report = ageLimited(requestSource)
                .whenUnavailable().reportUnavailable();
        PlantTargetSource fallback = ageLimited(requestSource)
                .whenUnavailable().fallbackTo(-1.0);
        PlantTargetSource holdLast = ageLimited(requestSource)
                .whenUnavailable().holdLastTarget(-2.0);
        PlantTargetSource holdMeasured = ageLimited(requestSource)
                .whenUnavailable().holdMeasuredTargetOnEntry(-3.0);
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(7.0, report.resolve(context(0.0), time.clock()).target(), EPSILON);
        assertEquals(7.0, fallback.resolve(context(0.0), time.clock()).target(), EPSILON);
        assertEquals(7.0, holdLast.resolve(context(0.0), time.clock()).target(), EPSILON);
        assertEquals(7.0, holdMeasured.resolve(context(0.0), time.clock()).target(), EPSILON);

        LoopClock staleClock = time.nextCycle(1.0);
        assertFalse(report.resolve(context(3.0), staleClock).hasTarget());
        assertPlan(fallback.resolve(context(3.0), staleClock),
                PlantTargetPlan.Kind.FALLBACK, -1.0);
        assertPlan(holdLast.resolve(context(3.0), staleClock),
                PlantTargetPlan.Kind.HOLD_LAST_TARGET, 7.0);
        assertPlan(holdMeasured.resolve(context(3.0), staleClock),
                PlantTargetPlan.Kind.HOLD_MEASURED_TARGET, 3.0);

        request[0] = PlantTargetRequest.observedExact("recovered", 8.0, 0.8, 1.0);
        LoopClock recoveryClock = time.nextCycle(0.0);
        assertPlannedTarget(report, recoveryClock, 8.0);
        assertPlannedTarget(fallback, recoveryClock, 8.0);
        assertPlannedTarget(holdLast, recoveryClock, 8.0);
        assertPlannedTarget(holdMeasured, recoveryClock, 8.0);
    }

    @Test
    public void plannerMemoizesWithinCycleAndResetForcesFreshResolution() {
        ProbeRequestSource request = new ProbeRequestSource(
                PlantTargetRequest.observedExact("fresh", 5.0, 0.8, 10.0));
        PlantTargetSource planner = ready(request)
                .accept().maxObservationAgeSec(0.5).doneAccept()
                .whenUnavailable().reportUnavailable();
        ManualLoopClock time = new ManualLoopClock(10.0);

        PlantTargetPlan first = planner.resolve(context(0.0), time.clock());
        request.request = PlantTargetRequest.observedExact("stale", 6.0, 0.8, 0.0);
        PlantTargetPlan sameCycle = planner.resolve(context(0.0), time.clock());

        assertSame(first, sameCycle);
        assertEquals(1, request.samples);

        planner.reset();
        PlantTargetPlan afterReset = planner.resolve(context(0.0), time.clock());

        assertFalse(afterReset.hasTarget());
        assertEquals(2, request.samples);
        assertEquals(1, request.resets);

        request.request = PlantTargetRequest.observedExact("next", 7.0, 0.8, 10.0);
        PlantTargetPlan nextCycle = planner.resolve(context(0.0), time.nextCycle(0.0));

        assertEquals("next", nextCycle.selectedCandidateId());
        assertEquals(3, request.samples);
    }

    @Test
    public void unavailableReasonExplainsTheFirstRejectedObservation() {
        PlantTargetRequest request = PlantTargetRequest.oneOf(
                PlantTargetCandidate.observedExact("bad-quality", 1.0, -0.1, 10.0),
                PlantTargetCandidate.observedExact("stale", 2.0, 0.8, 0.0));
        PlantTargetSource planner = ready(Source.constant(request))
                .accept().maxObservationAgeSec(0.5).doneAccept()
                .whenUnavailable().reportUnavailable();

        PlantTargetPlan plan = planner.resolve(
                context(0.0), new ManualLoopClock(10.0).clock());

        assertFalse(plan.hasTarget());
        assertTrue(plan.reason().contains("bad-quality"));
        assertTrue(plan.reason().contains("quality"));
    }

    @Test
    public void observedFactoryFamiliesAreParallelAcrossCandidateAndRequestLayers() {
        PlantTargetCandidate[] candidates = {
                PlantTargetCandidate.observedExact("exact", 1.0, 0.40, 11.0),
                PlantTargetCandidate.observedEquivalentPosition("equivalent", 2.0, 0.50, 12.0),
                PlantTargetCandidate.observedPeriodic("periodic", 3.0, 360.0, 0.60, 13.0),
                PlantTargetCandidate.observedRelative("relative", 4.0, 0.70, 14.0),
                PlantTargetCandidate.observedRelativeEquivalentPosition(
                        "relative-equivalent", 5.0, 0.80, 15.0),
                PlantTargetCandidate.observedRelativePeriodic(
                        "relative-periodic", 6.0, 360.0, 0.90, 16.0)
        };
        PlantTargetRequest[] requests = {
                PlantTargetRequest.observedExact("exact", 1.0, 0.40, 11.0),
                PlantTargetRequest.observedEquivalentPosition("equivalent", 2.0, 0.50, 12.0),
                PlantTargetRequest.observedPeriodic("periodic", 3.0, 360.0, 0.60, 13.0),
                PlantTargetRequest.observedRelative("relative", 4.0, 0.70, 14.0),
                PlantTargetRequest.observedRelativeEquivalentPosition(
                        "relative-equivalent", 5.0, 0.80, 15.0),
                PlantTargetRequest.observedRelativePeriodic(
                        "relative-periodic", 6.0, 360.0, 0.90, 16.0)
        };
        boolean[] periodic = {false, true, true, false, true, true};
        boolean[] usesPlantPeriod = {false, true, false, false, true, false};
        boolean[] relative = {false, false, false, true, true, true};

        for (int i = 0; i < candidates.length; i++) {
            PlantTargetCandidate candidate = candidates[i];
            assertTrue(candidate.isObserved());
            assertEquals(periodic[i], candidate.periodic);
            assertEquals(usesPlantPeriod[i], candidate.usesPlantPeriod);
            assertEquals(relative[i], candidate.relative);
            assertCandidateEquals(candidate, requests[i].candidates().get(0));
        }
    }

    @Test
    public void observedRelativeFactoriesPreserveRelativeTargetSemantics() {
        ManualLoopClock time = new ManualLoopClock(20.0);
        PlantTargetContext context = PlantTargetContext.position(
                true,
                10.0,
                ScalarRange.bounded(-1000.0, 1000.0),
                PositionPlant.Topology.PERIODIC,
                360.0,
                Double.NaN,
                Double.NaN);
        PlantTargetRequest[] requests = {
                PlantTargetRequest.observedRelative("relative", 5.0, 0.8, 20.0),
                PlantTargetRequest.observedRelativeEquivalentPosition(
                        "relative-equivalent", 5.0, 0.8, 20.0),
                PlantTargetRequest.observedRelativePeriodic(
                        "relative-periodic", 5.0, 360.0, 0.8, 20.0)
        };

        for (PlantTargetRequest request : requests) {
            PlantTargetPlan plan = ready(Source.constant(request))
                    .whenUnavailable().reportUnavailable()
                    .resolve(context, time.clock());

            assertTrue(plan.hasTarget());
            assertEquals(15.0, plan.target(), EPSILON);
            assertEquals(0.0, plan.selectedAgeSec(), EPSILON);
            assertEquals(20.0, plan.selectedTimestampSec(), EPSILON);
        }
    }

    private static PlantTargets.PlanReadyStage ready(Source<PlantTargetRequest> request) {
        return PlantTargets.plan()
                .request(request)
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

    private static void assertPlan(PlantTargetPlan plan,
                                   PlantTargetPlan.Kind kind,
                                   double target) {
        assertTrue(plan.hasTarget());
        assertEquals(kind, plan.kind());
        assertEquals(target, plan.target(), EPSILON);
    }

    private static void assertPlannedTarget(PlantTargetSource source,
                                            LoopClock clock,
                                            double target) {
        PlantTargetPlan plan = source.resolve(context(0.0), clock);
        assertPlan(plan, PlantTargetPlan.Kind.PLANNED_CANDIDATE, target);
    }

    private static void assertCandidateEquals(PlantTargetCandidate expected,
                                              PlantTargetCandidate actual) {
        assertEquals(expected.id, actual.id);
        assertEquals(expected.value, actual.value, EPSILON);
        assertEquals(expected.periodic, actual.periodic);
        if (Double.isNaN(expected.period)) {
            assertTrue(Double.isNaN(actual.period));
        } else {
            assertEquals(expected.period, actual.period, EPSILON);
        }
        assertEquals(expected.usesPlantPeriod, actual.usesPlantPeriod);
        assertEquals(expected.relative, actual.relative);
        assertEquals(expected.quality, actual.quality, EPSILON);
        assertEquals(expected.timestampSec, actual.timestampSec, EPSILON);
        assertEquals(expected.isObserved(), actual.isObserved());
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

    private static final class ProbeRequestSource implements Source<PlantTargetRequest> {
        private PlantTargetRequest request;
        private int samples;
        private int resets;

        private ProbeRequestSource(PlantTargetRequest request) {
            this.request = request;
        }

        @Override
        public PlantTargetRequest get(LoopClock clock) {
            samples++;
            return request;
        }

        @Override
        public void reset() {
            resets++;
        }
    }
}
