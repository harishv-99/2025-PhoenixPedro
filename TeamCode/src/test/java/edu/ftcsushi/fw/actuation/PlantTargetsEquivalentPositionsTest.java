package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.testing.ManualLoopClock;

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
        PlantTargetResolver resolver = nearest(command).reportUnavailable();
        ManualLoopClock time = new ManualLoopClock();

        PlantTargetResolution plan = resolver.resolve(
                periodicContext(350.0, ScalarRange.bounded(0.0, 720.0), 360.0),
                time.clock());

        assertSame(command, PlantTargets.commandTargetOf(resolver));
        assertEquals(380.0, plan.target(), EPSILON);
        assertEquals(PlantTargetResolution.Kind.EQUIVALENT_POSITION, plan.kind());
        assertTrue(plan.satisfiesIntent());
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
        PlantTargetResolver resolver = nearest(command).reportUnavailable();

        PlantTargetResolution plan = resolver.resolve(
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
        PlantTargetResolver logical = PlantTargets.overlay(command)
                .add("override", clock -> override[0], command)
                .build();
        PlantTargetResolver resolver = PlantTargets.equivalentPositionsOf(logical)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        ManualLoopClock time = new ManualLoopClock();
        PlantTargetContext context = periodicContext(
                350.0, ScalarRange.bounded(0.0, 720.0), 360.0);

        PlantTargetResolution masked = resolver.resolve(context, time.clock());
        assertEquals(380.0, masked.target(), EPSILON);
        assertTrue(masked.reportsCommandResolutionFor(command));
        assertFalse(masked.satisfiesCommand(command, 20.0));

        override[0] = false;
        PlantTargetResolution base = resolver.resolve(context, time.nextCycle(0.02));
        assertEquals(380.0, base.target(), EPSILON);
        assertTrue(base.satisfiesCommand(command, 20.0));
    }

    @Test
    public void readOnlyLogicalGraphStaysReadOnly() {
        PlantTargetResolver resolver =
                PlantTargets.equivalentPositionsOf(PlantTargets.exact(20.0))
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();

        assertNull(PlantTargets.commandTargetOf(resolver));
        assertEquals(380.0, resolver.resolve(
                periodicContext(350.0, ScalarRange.bounded(0.0, 720.0), 360.0),
                new ManualLoopClock().clock()).target(), EPSILON);
    }

    @Test
    public void transformedLogicalFallbackRetainsFallbackDiagnostics() {
        PlantTargetResolver resolver = PlantTargets.equivalentPositionsOf(
                        PlantTargets.holdLastTarget(20.0))
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();

        PlantTargetResolution plan = resolver.resolve(
                periodicContext(350.0, ScalarRange.bounded(0.0, 720.0), 360.0),
                new ManualLoopClock().clock());

        assertEquals(380.0, plan.target(), EPSILON);
        assertEquals(PlantTargetResolution.Kind.HOLD_LAST_TARGET, plan.kind());
        assertTrue(plan.usedFallback());
        assertFalse(plan.satisfiesIntent());
    }

    @Test
    public void nonPeriodicCoordinateAndUnreachableFamilyUseExplicitUnavailablePolicies() {
        ScalarTarget command = ScalarTarget.create(20.0);
        PlantTargetContext nonPeriodic = PlantTargetContext.simple(
                true, 40.0, ScalarRange.bounded(0.0, 100.0), Double.NaN, 40.0);

        PlantTargetResolution rejected = nearest(command).reportUnavailable()
                .resolve(nonPeriodic, new ManualLoopClock().clock());
        PlantTargetResolution fallback = nearest(command).fallbackTo(7.0)
                .resolve(nonPeriodic, new ManualLoopClock().clock());
        PlantTargetResolution unreachable = nearest(command).reportUnavailable()
                .resolve(periodicContext(150.0, ScalarRange.bounded(100.0, 200.0), 360.0),
                        new ManualLoopClock().clock());

        assertFalse(rejected.hasTarget());
        assertTrue(rejected.reason().contains("periodic Plant coordinate"));
        assertEquals(7.0, fallback.target(), EPSILON);
        assertEquals(PlantTargetResolution.Kind.FALLBACK, fallback.kind());
        assertTrue(fallback.reportsCommandResolutionFor(command));
        assertFalse(fallback.satisfiesCommand(command, 20.0));
        assertFalse(unreachable.hasTarget());
        assertTrue(unreachable.reason().contains("no legal equivalent"));
    }

    @Test
    public void holdLastRetainsTheLastSelectedPhysicalEquivalent() {
        ScalarTarget command = ScalarTarget.create(20.0);
        PlantTargetResolver resolver = nearest(command).holdLastTarget(5.0);
        ManualLoopClock time = new ManualLoopClock();

        PlantTargetResolution selected = resolver.resolve(
                periodicContext(350.0, ScalarRange.bounded(0.0, 720.0), 360.0),
                time.clock());
        PlantTargetResolution held = resolver.resolve(
                PlantTargetContext.simple(
                        true, 200.0, ScalarRange.bounded(0.0, 720.0), 380.0, 380.0),
                time.nextCycle(0.02));

        assertEquals(380.0, selected.target(), EPSILON);
        assertEquals(380.0, held.target(), EPSILON);
        assertEquals(PlantTargetResolution.Kind.HOLD_LAST_TARGET, held.kind());
        assertFalse(held.satisfiesCommand(command, 20.0));
    }

    @Test
    public void resetClearsEquivalentAndUnavailablePolicyHistory() {
        ScalarTarget command = ScalarTarget.create(20.0);
        PlantTargetResolver resolver = nearest(command).holdLastTarget(5.0);
        ManualLoopClock time = new ManualLoopClock();

        PlantTargetResolution selected = resolver.resolve(
                periodicContext(350.0, ScalarRange.bounded(0.0, 720.0), 360.0),
                time.clock());
        assertEquals(380.0, selected.target(), EPSILON);

        resolver.reset();
        PlantTargetResolution afterReset = resolver.resolve(
                nonPeriodicContext(30.0), time.nextCycle(0.02));

        assertEquals(5.0, afterReset.target(), EPSILON);
        assertEquals(PlantTargetResolution.Kind.HOLD_LAST_TARGET, afterReset.kind());
    }

    @Test
    public void selectionUsesPriorAppliedTargetAndRangeCenterWhenFeedbackIsUnavailable() {
        ScalarTarget command = ScalarTarget.create(20.0);
        ScalarRange range = ScalarRange.bounded(0.0, 720.0);

        PlantTargetContext priorApplied = PlantTargetContext.position(
                false, Double.NaN, range, PositionPlant.Periodicity.PERIODIC, 360.0,
                Double.NaN, 350.0);
        PlantTargetResolution nearestPrior = nearest(command).reportUnavailable()
                .resolve(priorApplied, new ManualLoopClock().clock());
        assertEquals(380.0, nearestPrior.target(), EPSILON);

        PlantTargetContext noReference = PlantTargetContext.position(
                false, Double.NaN, range, PositionPlant.Periodicity.PERIODIC, 360.0,
                Double.NaN, Double.NaN);
        PlantTargetResolution nearestUnavailable = nearest(command).reportUnavailable()
                .resolve(noReference, new ManualLoopClock().clock());
        assertFalse(nearestUnavailable.hasTarget());

        PlantTargetResolution centered = PlantTargets.equivalentPositionsOf(command)
                .preferRangeCenter()
                .whenUnavailable().reportUnavailable()
                .resolve(noReference, new ManualLoopClock().clock());
        assertEquals(380.0, centered.target(), EPSILON);
    }

    @Test
    public void measuredHoldRecapturesAfterSamplingGap() {
        ScalarTarget command = ScalarTarget.create(20.0);
        PlantTargetResolver resolver = nearest(command).holdMeasuredTargetOnEntry(-1.0);
        ManualLoopClock time = new ManualLoopClock();

        PlantTargetResolution first = resolver.resolve(nonPeriodicContext(10.0), time.clock());
        PlantTargetResolution consecutive =
                resolver.resolve(nonPeriodicContext(20.0), time.nextCycle(0.02));
        time.nextCycle(0.02);
        PlantTargetResolution afterGap =
                resolver.resolve(nonPeriodicContext(30.0), time.nextCycle(0.02));

        assertEquals(10.0, first.target(), EPSILON);
        assertEquals(10.0, consecutive.target(), EPSILON);
        assertEquals(30.0, afterGap.target(), EPSILON);
    }

    @Test
    public void sameCycleResolutionIsIdenticalAndResetResamplesTheChild() {
        CountingTargetResolver child = new CountingTargetResolver(20.0);
        PlantTargetResolver resolver = PlantTargets.equivalentPositionsOf(child)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        ManualLoopClock time = new ManualLoopClock();
        PlantTargetContext context = periodicContext(
                350.0, ScalarRange.bounded(0.0, 720.0), 360.0);

        PlantTargetResolution first = resolver.resolve(context, time.clock());
        PlantTargetResolution repeated = resolver.resolve(context, time.clock());
        assertSame(first, repeated);
        assertEquals(1, child.resolutions);

        resolver.reset();
        PlantTargetResolution afterReset = resolver.resolve(context, time.clock());
        assertNotSame(first, afterReset);
        assertEquals(2, child.resolutions);
        assertEquals(1, child.resets);
    }

    @Test
    public void failedChildResolutionCanRetryInTheSameCycle() {
        CountingTargetResolver child = new CountingTargetResolver(20.0);
        child.failNext = true;
        PlantTargetResolver resolver = PlantTargets.equivalentPositionsOf(child)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        ManualLoopClock time = new ManualLoopClock();
        PlantTargetContext context = periodicContext(
                350.0, ScalarRange.bounded(0.0, 720.0), 360.0);

        try {
            resolver.resolve(context, time.clock());
            fail("Expected the child failure");
        } catch (IllegalStateException expected) {
            assertEquals("probe failure", expected.getMessage());
        }

        PlantTargetResolution retried = resolver.resolve(context, time.clock());
        assertEquals(380.0, retried.target(), EPSILON);
        assertEquals(2, child.resolutions);
    }

    @Test
    public void failedResolutionKeepsHoldHistoryAndRetriesWithoutPartialPublication() {
        CountingTargetResolver child = new CountingTargetResolver(20.0);
        PlantTargetResolver resolver = PlantTargets.equivalentPositionsOf(child)
                .nearestToMeasurement()
                .whenUnavailable().holdLastTarget(5.0);
        ManualLoopClock time = new ManualLoopClock();

        PlantTargetResolution selected = resolver.resolve(
                periodicContext(350.0, ScalarRange.bounded(0.0, 720.0), 360.0),
                time.clock());
        assertEquals(380.0, selected.target(), EPSILON);

        child.failNext = true;
        LoopClock nextCycle = time.nextCycle(0.02);
        RuntimeException failure = captureFailure(
                () -> resolver.resolve(nonPeriodicContext(30.0), nextCycle));
        assertEquals("probe failure", failure.getMessage());

        PlantTargetResolution retried = resolver.resolve(
                nonPeriodicContext(30.0), nextCycle);
        assertEquals(PlantTargetResolution.Kind.HOLD_LAST_TARGET, retried.kind());
        assertEquals(380.0, retried.target(), EPSILON);
        assertSame(retried, resolver.resolve(nonPeriodicContext(40.0), nextCycle));
    }

    @Test
    public void recursiveResolutionFailsFastAndAValueRetryCanRecover() {
        final PlantTargetResolver[] resolver = new PlantTargetResolver[1];
        final boolean[] recurse = {true};
        PlantTargetResolver child = (context, clock) -> recurse[0]
                ? resolver[0].resolve(context, clock)
                : PlantTargetResolution.exact(20.0, "probe");
        resolver[0] = PlantTargets.equivalentPositionsOf(child)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        ManualLoopClock time = new ManualLoopClock();
        PlantTargetContext context = periodicContext(
                350.0, ScalarRange.bounded(0.0, 720.0), 360.0);

        RuntimeException reentry = captureFailure(
                () -> resolver[0].resolve(context, time.clock()));
        assertTrue(reentry instanceof IllegalStateException);
        assertTrue(reentry.getMessage().contains("reentered"));

        recurse[0] = false;
        assertEquals(380.0,
                resolver[0].resolve(context, time.clock()).target(), EPSILON);
    }

    @Test
    public void failedChildResetKeepsTheCommittedResolutionUntilResetSucceeds() {
        CountingTargetResolver child = new CountingTargetResolver(20.0);
        PlantTargetResolver resolver = PlantTargets.equivalentPositionsOf(child)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        ManualLoopClock time = new ManualLoopClock();
        PlantTargetContext context = periodicContext(
                350.0, ScalarRange.bounded(0.0, 720.0), 360.0);
        PlantTargetResolution committed = resolver.resolve(context, time.clock());
        child.resetAction = () -> resolver.resolve(context, time.clock());

        RuntimeException overlap = captureFailure(resolver::reset);
        assertTrue(overlap instanceof IllegalStateException);
        assertTrue(overlap.getMessage().contains("reset is in progress"));
        assertSame(committed, resolver.resolve(context, time.clock()));
        assertEquals(1, child.resolutions);

        child.resetAction = null;
        resolver.reset();
        PlantTargetResolution afterReset = resolver.resolve(context, time.clock());
        assertNotSame(committed, afterReset);
        assertEquals(2, child.resolutions);
        assertEquals(2, child.resets);
    }

    @Test
    public void transformedAdvancedPlanRetainsCandidateObservationMetadata() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp timestamp = time.clock().nowTimestamp();
        PlantTargetResolver logical = PlantTargets.plan(Source.constant(
                        PlantTargetRequest.observedExact(
                                "vision", 20.0, 0.75, timestamp)))
                .nearestToMeasurement()
                .rejectUnreachable()
                .whenUnavailable().reportUnavailable();
        PlantTargetResolver resolver = PlantTargets.equivalentPositionsOf(logical)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();

        PlantTargetResolution plan = resolver.resolve(
                periodicContext(350.0, ScalarRange.bounded(0.0, 720.0), 360.0),
                time.clock());

        assertEquals(PlantTargetResolution.Kind.EQUIVALENT_POSITION, plan.kind());
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
                PositionPlant.Periodicity.PERIODIC, period, Double.NaN, measurement);
    }

    private static PlantTargetContext nonPeriodicContext(double measurement) {
        return PlantTargetContext.simple(true, measurement,
                ScalarRange.bounded(0.0, 100.0), Double.NaN, measurement);
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

    private static final class CountingTargetResolver implements PlantTargetResolver {
        private final double target;
        private int resolutions;
        private int resets;
        private boolean failNext;
        private Runnable resetAction;

        CountingTargetResolver(double target) {
            this.target = target;
        }

        @Override
        public PlantTargetResolution resolve(PlantTargetContext context, LoopClock clock) {
            resolutions++;
            if (failNext) {
                failNext = false;
                throw new IllegalStateException("probe failure");
            }
            return PlantTargetResolution.exact(target, "probe");
        }

        @Override
        public void reset() {
            resets++;
            if (resetAction != null) resetAction.run();
        }
    }
}
