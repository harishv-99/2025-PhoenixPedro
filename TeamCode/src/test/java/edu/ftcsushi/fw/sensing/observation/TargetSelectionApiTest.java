package edu.ftcsushi.fw.sensing.observation;

import org.junit.Test;

import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** The typed factory changes vocabulary, not evidence, frame-local selection or source ownership. */
public final class TargetSelectionApiTest {
    @Test public void onePublicConstructionLayerEndsWithTheTypedSelectionSource() throws Exception {
        assertTrue(Source.class.isAssignableFrom(TargetSelectionSource.class));
        assertEquals(TargetSelectionSource.class, TargetSelections.PolicyStep.class
                .getMethod("choose", TargetSelectionPolicy.class).getReturnType());
        assertEquals(1, TargetSelections.PolicyStep.class.getDeclaredMethods().length);
        assertEquals(0, TargetSelectionPolicy.class.getConstructors().length);
        assertTrue(Modifier.isFinal(TargetSelectionPolicy.class.getModifiers()));
        try {
            TargetSelections.class.getMethod("from", Source.class);
            fail("the old source-construction alias must not remain");
        } catch (NoSuchMethodException expected) { /* The named observed-evidence entry is canonical. */ }
    }

    @Test public void constructionDoesNotReadObservationsBearingOrCost() {
        ManualLoopClock time = new ManualLoopClock();
        AtomicInteger observations = new AtomicInteger();
        AtomicInteger bearings = new AtomicInteger();
        AtomicInteger costs = new AtomicInteger();
        Source<TargetObservations2d> source = Source.of(clock -> {
            observations.incrementAndGet();
            return frame(time, 2, 0, 8, 1);
        });
        TargetSelectionPolicy bearing = TargetSelectionPolicies.nearestBearingRad(clock -> {
            bearings.incrementAndGet();
            return 0;
        });
        TargetSelectionPolicy cost = TargetSelectionPolicies.lowestCost(point -> {
            costs.incrementAndGet();
            return point.forwardInches;
        });
        TargetSelections.PolicyStep factory = TargetSelections.fromVisibleObjects(source).freshWithinSec(0.2);
        TargetSelectionSource first = factory.choose(bearing);
        TargetSelectionSource second = factory.choose(cost);
        first.debugDump(null, "selected");
        bearing.toString();
        assertEquals(0, observations.get());
        assertEquals(0, bearings.get());
        assertEquals(0, costs.get());
        assertTrue(first.get(time.clock()).hasSelection());
        assertTrue(second.get(time.clock()).hasSelection());
        assertEquals(2, observations.get());
        assertEquals(1, bearings.get());
        assertEquals(2, costs.get());
    }

    @Test public void aSharedPolicyDoesNotShareSelectorCachesOrResetBorrowedSources() {
        ManualLoopClock time = new ManualLoopClock();
        AtomicInteger observations = new AtomicInteger();
        AtomicInteger bearings = new AtomicInteger();
        AtomicInteger resets = new AtomicInteger();
        TargetObservations2d frame = frame(time, 2, 0, 8, 1);
        Source<TargetObservations2d> source = new Source<TargetObservations2d>() {
            @Override public TargetObservations2d get(edu.ftcsushi.fw.core.time.LoopClock clock) {
                observations.incrementAndGet();
                return frame;
            }
            @Override public void reset() { resets.incrementAndGet(); }
        };
        ScalarSource bearing = new ScalarSource() {
            @Override public double getAsDouble(edu.ftcsushi.fw.core.time.LoopClock clock) {
                bearings.incrementAndGet();
                return 0;
            }
            @Override public void reset() { resets.incrementAndGet(); }
        };
        TargetSelectionPolicy policy = TargetSelectionPolicies.nearestBearingRad(bearing);
        TargetSelections.PolicyStep factory = TargetSelections.fromVisibleObjects(source).freshWithinSec(0.2);
        TargetSelectionSource first = factory.choose(policy);
        TargetSelectionSource second = factory.choose(policy);
        assertNotSame(first, second);
        TargetSelectionResult firstResult = first.get(time.clock());
        TargetSelectionResult secondResult = second.get(time.clock());
        assertSame(firstResult, first.get(time.clock()));
        assertNotSame(firstResult, secondResult);
        assertSame(firstResult.observation(), secondResult.observation());
        first.reset();
        assertSame(secondResult, second.get(time.clock()));
        assertNotSame(firstResult, first.get(time.clock()));
        assertEquals(3, observations.get());
        assertEquals(3, bearings.get());
        assertEquals(0, resets.get());
    }

    @Test public void unavailableEmptyStaleAndResetFramesDoNotEvaluatePolicyInputs() {
        ManualLoopClock time = new ManualLoopClock();
        TargetObservations2d original = frame(time, 2, 0);
        AtomicReference<TargetObservations2d> current = new AtomicReference<>(
                TargetObservations2d.unavailable("camera unavailable"));
        AtomicInteger bearings = new AtomicInteger();
        TargetSelectionSource selected = TargetSelections.fromVisibleObjects(Source.of(clock -> current.get()))
                .freshWithinSec(0.2).choose(TargetSelectionPolicies.nearestBearingRad(clock -> {
                    bearings.incrementAndGet();
                    return 0;
                }));
        assertFalse(selected.get(time.clock()).hasSelection());
        time.nextCycle(0.01);
        current.set(TargetObservations2d.fromFrame(time.clock().nowTimestamp(), Collections.emptyList()));
        assertFalse(selected.get(time.clock()).hasSelection());
        time.nextCycle(0.21);
        current.set(original);
        assertFalse(selected.get(time.clock()).hasSelection());
        time.clock().reset(0);
        assertFalse(selected.get(time.clock()).hasSelection());
        assertEquals(0, bearings.get());
    }

    @Test public void observationAndBearingFailuresCanRetryWithoutCachingPartialResults() {
        ManualLoopClock time = new ManualLoopClock();
        TargetObservations2d frame = frame(time, 2, 0, 8, 1);
        AtomicInteger observations = new AtomicInteger();
        AtomicInteger bearings = new AtomicInteger();
        TargetSelectionSource selected = TargetSelections.fromVisibleObjects(Source.of(clock -> {
            if (observations.incrementAndGet() == 1) throw new IllegalStateException("camera read");
            return frame;
        })).freshWithinSec(0.2).choose(TargetSelectionPolicies.nearestBearingRad(clock -> {
            if (bearings.incrementAndGet() == 1) throw new IllegalStateException("bearing read");
            return 0;
        }));
        assertEquals("camera read", failure(IllegalStateException.class, () -> selected.get(time.clock())).getMessage());
        assertEquals("bearing read", failure(IllegalStateException.class, () -> selected.get(time.clock())).getMessage());
        TargetSelectionResult successful = selected.get(time.clock());
        assertSame(successful, selected.get(time.clock()));
        assertSame(frame, successful.frame());
        assertEquals(3, observations.get());
        assertEquals(2, bearings.get());
    }

    @Test public void failureAfterOneRankedCandidateDoesNotCommitThatCandidate() {
        ManualLoopClock time = new ManualLoopClock();
        AtomicReference<TargetObservations2d> current = new AtomicReference<>(frame(time, 2, 0, 8, 1));
        AtomicInteger costs = new AtomicInteger();
        TargetSelectionSource selected = TargetSelections.fromVisibleObjects(Source.of(clock -> current.get()))
                .freshWithinSec(0.2).choose(TargetSelectionPolicies.lowestCost(point -> {
                    if (costs.incrementAndGet() == 2) throw new IllegalStateException("second candidate");
                    return point.forwardInches;
                }));
        failure(IllegalStateException.class, () -> selected.get(time.clock()));
        TargetObservations2d replacement = frame(time, 12, 0);
        current.set(replacement);
        TargetSelectionResult result = selected.get(time.clock());
        assertSame(replacement, result.frame());
        assertEquals(12, result.observation().forwardInches, 0);
        assertSame(result, selected.get(time.clock()));
        assertEquals(3, costs.get());
    }

    @Test public void recursiveSamplingAndResetOverlapRejectThenAllowANonrecursiveRetry() {
        for (boolean reset : new boolean[]{false, true}) {
            ManualLoopClock time = new ManualLoopClock();
            AtomicBoolean recurse = new AtomicBoolean(true);
            TargetSelectionSource[] selected = new TargetSelectionSource[1];
            TargetObservations2d frame = frame(time, 2, 0);
            selected[0] = TargetSelections.fromVisibleObjects(Source.of(clock -> {
                if (recurse.getAndSet(false)) {
                    if (reset) selected[0].reset();
                    else selected[0].get(clock);
                }
                return frame;
            })).freshWithinSec(0.2).choose(TargetSelectionPolicies.nearestToRobot());
            failure(IllegalStateException.class, () -> selected[0].get(time.clock()));
            TargetSelectionResult result = selected[0].get(time.clock());
            assertSame(frame, result.frame());
            assertSame(result, selected[0].get(time.clock()));
        }
    }

    @Test public void changingAndEmptyFramesDoNotKeepAnAnonymousIdentityOrAnOldSelection() {
        ManualLoopClock time = new ManualLoopClock();
        AtomicReference<TargetObservations2d> current = new AtomicReference<>(frame(time, 2, 0));
        TargetSelectionSource selected = TargetSelections.fromVisibleObjects(Source.of(clock -> current.get()))
                .freshWithinSec(0.2).choose(TargetSelectionPolicies.nearestToRobot());
        TargetSelectionResult first = selected.get(time.clock());
        time.nextCycle(0.01);
        current.set(frame(time, 12, 0));
        TargetSelectionResult second = selected.get(time.clock());
        assertNotSame(first.observation(), second.observation());
        assertEquals(-1, second.observation().targetId);
        assertEquals(12, second.observation().forwardInches, 0);
        assertSame(current.get().timestamp(), second.observation().timestamp);
        time.nextCycle(0.01);
        current.set(TargetObservations2d.fromFrame(time.clock().nowTimestamp(), Collections.emptyList()));
        assertFalse(selected.get(time.clock()).hasSelection());
        assertTrue(first.hasSelection());
    }

    @Test public void nullAndMalformedPolicyArgumentsFailBeforeSampling() {
        failure(NullPointerException.class, () -> TargetSelections.fromVisibleObjects(null));
        TargetSelections.PolicyStep choose = TargetSelections.fromVisibleObjects(Source.of(clock -> {
            throw new AssertionError("configuration must not sample observations");
        })).freshWithinSec(0.2);
        failure(NullPointerException.class, () -> choose.choose(null));
        failure(NullPointerException.class, () -> TargetSelectionPolicies.lowestCost(null));
        failure(NullPointerException.class, () -> TargetSelectionPolicies.nearestBearingRad(null));
        failure(NullPointerException.class, () -> TargetSelectionPolicies.nearestToControlFrame(null));
        failure(IllegalArgumentException.class, () -> TargetSelectionPolicies.nearestToControlFrame(
                new Pose2d(0, 0, Double.NaN)));
        failure(IllegalArgumentException.class, () -> TargetSelectionPolicies.nearFieldPoint(0, Double.NaN, 1));
        failure(IllegalArgumentException.class, () -> TargetSelectionPolicies.nearFieldPoint(0, 0, -1));
        failure(IllegalArgumentException.class, () -> TargetSelectionPolicies.mostNeighborsWithinInches(-1));
    }

    /** Build independent camera-neutral points with one exact original capture timestamp. */
    private static TargetObservations2d frame(ManualLoopClock time, double... xy) {
        LoopTimestamp timestamp = time.clock().nowTimestamp();
        TargetObservation2d[] points = new TargetObservation2d[xy.length / 2];
        for (int i = 0; i < points.length; i++) {
            points[i] = TargetObservation2d.ofRobotRelativePosition(xy[2 * i], xy[2 * i + 1], Double.NaN, timestamp);
        }
        return TargetObservations2d.fromFrame(timestamp, Arrays.asList(points));
    }

    /** Assert the exact failure family while allowing the same source to retry afterward. */
    private static RuntimeException failure(Class<? extends RuntimeException> type, Runnable action) {
        try {
            action.run();
            fail("expected " + type.getSimpleName());
        } catch (RuntimeException failure) {
            assertTrue("received " + failure.getClass().getSimpleName(), type.isInstance(failure));
            return failure;
        }
        throw new AssertionError("unreachable");
    }
}
