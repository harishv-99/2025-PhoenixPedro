package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the focused equivalent-position transform independently of Plant hardware policy. */
public final class PlantTargetsEquivalentPositionsTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void commandTargetResolvesToNearestPhysicalEquivalentAndKeepsLogicalEvidence() {
        ScalarTarget command = ScalarTarget.create(20.0);
        PlantTargetSource source = nearest(command).reportUnavailable();
        ManualLoopClock time = new ManualLoopClock();

        PlantTargetPlan plan = source.resolve(
                periodicContext(350.0, ScalarRange.bounded(0.0, 720.0), 360.0),
                time.clock());

        assertSame(command, PlantTargets.commandTargetOf(source));
        assertEquals(380.0, plan.target(), EPSILON);
        assertEquals(PlantTargetPlan.Kind.EQUIVALENT_POSITION, plan.kind());
        assertTrue(plan.satisfiesRequest());
        assertTrue(plan.reportsCommandResolutionFor(command));
        assertTrue(plan.satisfiesCommand(command, 20.0));
        assertFalse(plan.satisfiesCommand(command, 380.0));
    }

    @Test
    public void allPreferencesAndMidpointTiesRemainDeterministic() {
        ScalarTarget command = ScalarTarget.create(20.0);
        PlantTargetContext context = periodicContext(
                350.0, ScalarRange.bounded(0.0, 720.0), 360.0);

        assertEquals(380.0, nearest(command).reportUnavailable()
                .resolve(context, new ManualLoopClock().clock()).target(), EPSILON);
        assertEquals(380.0, PlantTargets.equivalentPositionsOf(command)
                .preferIncreasing().whenUnavailable().reportUnavailable()
                .resolve(context, new ManualLoopClock().clock()).target(), EPSILON);
        assertEquals(20.0, PlantTargets.equivalentPositionsOf(command)
                .preferDecreasing().whenUnavailable().reportUnavailable()
                .resolve(context, new ManualLoopClock().clock()).target(), EPSILON);
        assertEquals(380.0, PlantTargets.equivalentPositionsOf(command)
                .preferRangeCenter().whenUnavailable().reportUnavailable()
                .resolve(context, new ManualLoopClock().clock()).target(), EPSILON);

        PlantTargetContext tie = periodicContext(
                200.0, ScalarRange.bounded(0.0, 720.0), 360.0);
        assertEquals(20.0, nearest(command).reportUnavailable()
                .resolve(tie, new ManualLoopClock().clock()).target(), EPSILON);
    }

    @Test(timeout = 5000L)
    public void enormousEquivalentSpacesUseTheBoundedSharedSelector() {
        ScalarTarget command = ScalarTarget.create(0.0);
        PlantTargetSource source = nearest(command).reportUnavailable();

        PlantTargetPlan plan = source.resolve(
                periodicContext(123.456789,
                        ScalarRange.bounded(-1.0e9, 1.0e9), 1.0e-12),
                new ManualLoopClock().clock());

        assertTrue(plan.hasTarget());
        assertEquals(123.456789, plan.target(), 1.0e-10);
    }

    @Test
    public void finalOverlayWinnerIsTransformedAndOnlyTheBaseCommandCanSatisfyAMove() {
        ScalarTarget command = ScalarTarget.create(20.0);
        final boolean[] override = {true};
        PlantTargetSource logical = PlantTargets.overlay(command)
                .add("override", clock -> override[0], command)
                .build();
        PlantTargetSource source = PlantTargets.equivalentPositionsOf(logical)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        ManualLoopClock time = new ManualLoopClock();
        PlantTargetContext context = periodicContext(
                350.0, ScalarRange.bounded(0.0, 720.0), 360.0);

        PlantTargetPlan masked = source.resolve(context, time.clock());
        assertEquals(380.0, masked.target(), EPSILON);
        assertTrue(masked.reportsCommandResolutionFor(command));
        assertFalse(masked.satisfiesCommand(command, 20.0));

        override[0] = false;
        PlantTargetPlan base = source.resolve(context, time.nextCycle(0.02));
        assertEquals(380.0, base.target(), EPSILON);
        assertTrue(base.satisfiesCommand(command, 20.0));
    }

    @Test
    public void readOnlyLogicalGraphStaysReadOnly() {
        PlantTargetSource source = PlantTargets.equivalentPositionsOf(PlantTargets.exact(20.0))
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();

        assertNull(PlantTargets.commandTargetOf(source));
        assertEquals(380.0, source.resolve(
                periodicContext(350.0, ScalarRange.bounded(0.0, 720.0), 360.0),
                new ManualLoopClock().clock()).target(), EPSILON);
    }

    @Test
    public void transformedLogicalFallbackRetainsFallbackDiagnostics() {
        PlantTargetSource source = PlantTargets.equivalentPositionsOf(
                        PlantTargets.holdLastTarget(20.0))
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();

        PlantTargetPlan plan = source.resolve(
                periodicContext(350.0, ScalarRange.bounded(0.0, 720.0), 360.0),
                new ManualLoopClock().clock());

        assertEquals(380.0, plan.target(), EPSILON);
        assertEquals(PlantTargetPlan.Kind.HOLD_LAST_TARGET, plan.kind());
        assertTrue(plan.usedFallback());
        assertFalse(plan.satisfiesRequest());
    }

    @Test
    public void invalidTopologyAndUnreachableFamilyUseExplicitUnavailablePolicies() {
        ScalarTarget command = ScalarTarget.create(20.0);
        PlantTargetContext linear = PlantTargetContext.simple(
                true, 40.0, ScalarRange.bounded(0.0, 100.0), Double.NaN, 40.0);

        PlantTargetPlan rejected = nearest(command).reportUnavailable()
                .resolve(linear, new ManualLoopClock().clock());
        PlantTargetPlan fallback = nearest(command).fallbackTo(7.0)
                .resolve(linear, new ManualLoopClock().clock());
        PlantTargetPlan unreachable = nearest(command).reportUnavailable()
                .resolve(periodicContext(150.0, ScalarRange.bounded(100.0, 200.0), 360.0),
                        new ManualLoopClock().clock());

        assertFalse(rejected.hasTarget());
        assertTrue(rejected.reason().contains("periodic Plant topology"));
        assertEquals(7.0, fallback.target(), EPSILON);
        assertEquals(PlantTargetPlan.Kind.FALLBACK, fallback.kind());
        assertTrue(fallback.reportsCommandResolutionFor(command));
        assertFalse(fallback.satisfiesCommand(command, 20.0));
        assertFalse(unreachable.hasTarget());
        assertTrue(unreachable.reason().contains("no legal equivalent"));
    }

    @Test
    public void holdLastRetainsTheLastSelectedPhysicalEquivalent() {
        ScalarTarget command = ScalarTarget.create(20.0);
        PlantTargetSource source = nearest(command).holdLastTarget(5.0);
        ManualLoopClock time = new ManualLoopClock();

        PlantTargetPlan selected = source.resolve(
                periodicContext(350.0, ScalarRange.bounded(0.0, 720.0), 360.0),
                time.clock());
        PlantTargetPlan held = source.resolve(
                PlantTargetContext.simple(
                        true, 200.0, ScalarRange.bounded(0.0, 720.0), 380.0, 380.0),
                time.nextCycle(0.02));

        assertEquals(380.0, selected.target(), EPSILON);
        assertEquals(380.0, held.target(), EPSILON);
        assertEquals(PlantTargetPlan.Kind.HOLD_LAST_TARGET, held.kind());
        assertFalse(held.satisfiesCommand(command, 20.0));
    }

    @Test
    public void resetClearsEquivalentAndUnavailablePolicyHistory() {
        ScalarTarget command = ScalarTarget.create(20.0);
        PlantTargetSource source = nearest(command).holdLastTarget(5.0);
        ManualLoopClock time = new ManualLoopClock();

        PlantTargetPlan selected = source.resolve(
                periodicContext(350.0, ScalarRange.bounded(0.0, 720.0), 360.0),
                time.clock());
        assertEquals(380.0, selected.target(), EPSILON);

        source.reset();
        PlantTargetPlan afterReset = source.resolve(
                linearContext(30.0), time.nextCycle(0.02));

        assertEquals(5.0, afterReset.target(), EPSILON);
        assertEquals(PlantTargetPlan.Kind.HOLD_LAST_TARGET, afterReset.kind());
    }

    @Test
    public void selectionUsesPriorAppliedTargetAndRangeCenterWhenFeedbackIsUnavailable() {
        ScalarTarget command = ScalarTarget.create(20.0);
        ScalarRange range = ScalarRange.bounded(0.0, 720.0);

        PlantTargetContext priorApplied = PlantTargetContext.position(
                false, Double.NaN, range, PositionPlant.Topology.PERIODIC, 360.0,
                Double.NaN, 350.0);
        PlantTargetPlan nearestPrior = nearest(command).reportUnavailable()
                .resolve(priorApplied, new ManualLoopClock().clock());
        assertEquals(380.0, nearestPrior.target(), EPSILON);

        PlantTargetContext noReference = PlantTargetContext.position(
                false, Double.NaN, range, PositionPlant.Topology.PERIODIC, 360.0,
                Double.NaN, Double.NaN);
        PlantTargetPlan nearestUnavailable = nearest(command).reportUnavailable()
                .resolve(noReference, new ManualLoopClock().clock());
        assertFalse(nearestUnavailable.hasTarget());

        PlantTargetPlan centered = PlantTargets.equivalentPositionsOf(command)
                .preferRangeCenter()
                .whenUnavailable().reportUnavailable()
                .resolve(noReference, new ManualLoopClock().clock());
        assertEquals(380.0, centered.target(), EPSILON);
    }

    @Test
    public void measuredHoldRecapturesAfterSamplingGap() {
        ScalarTarget command = ScalarTarget.create(20.0);
        PlantTargetSource source = nearest(command).holdMeasuredTargetOnEntry(-1.0);
        ManualLoopClock time = new ManualLoopClock();

        PlantTargetPlan first = source.resolve(linearContext(10.0), time.clock());
        PlantTargetPlan consecutive = source.resolve(linearContext(20.0), time.nextCycle(0.02));
        time.nextCycle(0.02);
        PlantTargetPlan afterGap = source.resolve(linearContext(30.0), time.nextCycle(0.02));

        assertEquals(10.0, first.target(), EPSILON);
        assertEquals(10.0, consecutive.target(), EPSILON);
        assertEquals(30.0, afterGap.target(), EPSILON);
    }

    @Test
    public void sameCycleResolutionIsIdenticalAndResetResamplesTheChild() {
        CountingTargetSource child = new CountingTargetSource(20.0);
        PlantTargetSource source = PlantTargets.equivalentPositionsOf(child)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        ManualLoopClock time = new ManualLoopClock();
        PlantTargetContext context = periodicContext(
                350.0, ScalarRange.bounded(0.0, 720.0), 360.0);

        PlantTargetPlan first = source.resolve(context, time.clock());
        PlantTargetPlan repeated = source.resolve(context, time.clock());
        assertSame(first, repeated);
        assertEquals(1, child.resolutions);

        source.reset();
        PlantTargetPlan afterReset = source.resolve(context, time.clock());
        assertNotSame(first, afterReset);
        assertEquals(2, child.resolutions);
        assertEquals(1, child.resets);
    }

    @Test
    public void failedChildResolutionCanRetryInTheSameCycle() {
        CountingTargetSource child = new CountingTargetSource(20.0);
        child.failNext = true;
        PlantTargetSource source = PlantTargets.equivalentPositionsOf(child)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        ManualLoopClock time = new ManualLoopClock();
        PlantTargetContext context = periodicContext(
                350.0, ScalarRange.bounded(0.0, 720.0), 360.0);

        try {
            source.resolve(context, time.clock());
            fail("Expected the child failure");
        } catch (IllegalStateException expected) {
            assertEquals("probe failure", expected.getMessage());
        }

        PlantTargetPlan retried = source.resolve(context, time.clock());
        assertEquals(380.0, retried.target(), EPSILON);
        assertEquals(2, child.resolutions);
    }

    @Test
    public void transformedAdvancedPlanRetainsCandidateObservationMetadata() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp timestamp = time.clock().nowTimestamp();
        PlantTargetSource logical = PlantTargets.plan()
                .request(Source.constant(PlantTargetRequest.observedExact(
                        "vision", 20.0, 0.75, timestamp)))
                .nearestToMeasurement()
                .rejectUnreachable()
                .whenUnavailable().reportUnavailable();
        PlantTargetSource source = PlantTargets.equivalentPositionsOf(logical)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();

        PlantTargetPlan plan = source.resolve(
                periodicContext(350.0, ScalarRange.bounded(0.0, 720.0), 360.0),
                time.clock());

        assertEquals(PlantTargetPlan.Kind.EQUIVALENT_POSITION, plan.kind());
        assertEquals("vision", plan.selectedCandidateId());
        assertEquals(0.75, plan.selectedQuality(), EPSILON);
        assertEquals(0.0, plan.selectedAgeSec(), EPSILON);
        assertEquals(timestamp, plan.selectedTimestamp());
    }

    private static PlantTargets.UnavailableTargetBranch nearest(ScalarTarget command) {
        return PlantTargets.equivalentPositionsOf(command)
                .nearestToMeasurement()
                .whenUnavailable();
    }

    private static PlantTargetContext periodicContext(double measurement,
                                                      ScalarRange range,
                                                      double period) {
        return PlantTargetContext.position(true, measurement, range,
                PositionPlant.Topology.PERIODIC, period, Double.NaN, measurement);
    }

    private static PlantTargetContext linearContext(double measurement) {
        return PlantTargetContext.simple(true, measurement,
                ScalarRange.bounded(0.0, 100.0), Double.NaN, measurement);
    }

    private static final class CountingTargetSource implements PlantTargetSource {
        private final double target;
        private int resolutions;
        private int resets;
        private boolean failNext;

        CountingTargetSource(double target) {
            this.target = target;
        }

        @Override
        public PlantTargetPlan resolve(PlantTargetContext context, LoopClock clock) {
            resolutions++;
            if (failNext) {
                failNext = false;
                throw new IllegalStateException("probe failure");
            }
            return PlantTargetPlan.exact(target, "probe");
        }

        @Override
        public void reset() {
            resets++;
        }
    }
}
