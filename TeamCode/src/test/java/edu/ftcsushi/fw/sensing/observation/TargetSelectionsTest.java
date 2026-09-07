package edu.ftcsushi.fw.sensing.observation;

import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicInteger;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Geometric ranking is frame-local evidence, never a physical identity/tracking promise. */
public final class TargetSelectionsTest {
    private static final double EPS = 1.0e-9;

    @Test public void nearestRobotAndControlOriginExpressDifferentDistances() {
        ManualLoopClock time = new ManualLoopClock();
        TargetObservations2d frame = frame(time, 2, 0, 10, 0);
        TargetSelections.PolicyStep policies = TargetSelections.from(Source.constant(frame)).freshWithinSec(0.2);
        assertEquals(2, policies.nearestToRobot().get(time.clock()).observation().forwardInches, EPS);
        assertEquals(10, policies.nearestToControlFrame(new Pose2d(9, 0, 1))
                .get(time.clock()).observation().forwardInches, EPS);
        assertEquals(-1, policies.nearestToRobot().get(time.clock()).observation().targetId);
    }

    @Test public void geometricTiesDoNotDependOnListOrderOrAssignIds() {
        ManualLoopClock time = new ManualLoopClock();
        TargetObservations2d frame = frame(time, 3, 4, 3, -4);
        TargetObservations2d reversed = TargetObservations2d.fromFrame(frame.timestamp(),
                Arrays.asList(frame.observations().get(1), frame.observations().get(0)));
        TargetObservation2d first = nearest(frame).get(time.clock()).observation();
        TargetObservation2d second = nearest(reversed).get(time.clock()).observation();
        assertSame(first, second);
        assertEquals(-4, first.leftInches, EPS);
        assertEquals(-1, first.targetId);
    }

    @Test public void bearingIsReadOncePerSuccessfulCycleAndResetNeverResetsBorrowedInputs() {
        ManualLoopClock time = new ManualLoopClock();
        TargetObservations2d frame = frame(time, 10, 0, 1, 10);
        AtomicInteger reads = new AtomicInteger();
        AtomicInteger resets = new AtomicInteger();
        Source<TargetObservations2d> source = new Source<TargetObservations2d>() {
            @Override public TargetObservations2d get(LoopClock clock) { return frame; }
            @Override public void reset() { resets.incrementAndGet(); }
        };
        ScalarSource bearing = new ScalarSource() {
            @Override public double getAsDouble(LoopClock clock) { reads.incrementAndGet(); return Math.PI / 2; }
            @Override public void reset() { resets.incrementAndGet(); }
        };
        Source<TargetSelectionResult> selected = TargetSelections.from(source).freshWithinSec(0.2)
                .nearestBearingRad(bearing);
        TargetSelectionResult first = selected.get(time.clock());
        assertSame(first, selected.get(time.clock()));
        assertEquals(1, reads.get());
        assertEquals(10, first.observation().leftInches, EPS);
        selected.reset();
        assertEquals(0, resets.get());
        selected.get(time.clock());
        assertEquals(2, reads.get());
    }

    @Test public void neighborsCountOnlyOtherCandidatesInOneFrame() {
        ManualLoopClock time = new ManualLoopClock();
        TargetObservations2d frame = frame(time, 1, 0, 2, 0, 3, 0, 30, 0);
        TargetSelectionResult selection = TargetSelections.from(Source.constant(frame)).freshWithinSec(0.2)
                .mostNeighborsWithinInches(1).get(time.clock());
        assertEquals(2, selection.observation().forwardInches, EPS);
        assertEquals(-2, selection.metricValue(), EPS);
        assertEquals(-1, selection.observation().targetId);
    }

    @Test public void fieldRadiusNeedsCaptureTimeFieldEvidenceAndIncludesBoundary() {
        ObservationSourcesTest.Fixture f = new ObservationSourcesTest.Fixture();
        f.publish(100, 20, 0);
        TargetObservations2d raw = frame(f.time, 2, 0, 10, 0);
        Source<TargetObservations2d> field = ObservationSources.inField(Source.constant(raw), f.history.lookupSource());
        TargetSelectionResult selected = TargetSelections.from(field).freshWithinSec(0.2)
                .nearFieldPoint(111, 20, 1).get(f.time.clock());
        assertEquals(110, selected.observation().fieldXInches, EPS);
        assertFalse(TargetSelections.from(field).freshWithinSec(0.2).nearFieldPoint(111, 20, 0.9)
                .get(f.time.clock()).hasSelection());
        assertFalse(TargetSelections.from(Source.constant(raw)).freshWithinSec(0.2)
                .nearFieldPoint(2, 0, 100).get(f.time.clock()).hasSelection());
    }

    @Test public void customCostExcludesNonfiniteAndExceptionsCanRetrySameCycle() {
        ManualLoopClock time = new ManualLoopClock();
        TargetObservations2d frame = frame(time, 2, 0, 10, 0);
        AtomicInteger calls = new AtomicInteger();
        Source<TargetSelectionResult> selected = TargetSelections.from(Source.constant(frame)).freshWithinSec(0.2)
                .lowestCost(point -> {
                    if (calls.incrementAndGet() == 1) throw new IllegalStateException("retry");
                    return point.forwardInches == 2 ? Double.NaN : -point.forwardInches;
                });
        try { selected.get(time.clock()); fail("first calculation must fail"); }
        catch (IllegalStateException expected) { }
        TargetSelectionResult result = selected.get(time.clock());
        assertEquals(10, result.observation().forwardInches, EPS);
        assertEquals(-10, result.metricValue(), EPS);
        assertSame(result, selected.get(time.clock()));
        assertEquals(3, calls.get());
    }

    @Test public void emptyUnavailableStaleAndResetFramesCannotProduceSelection() {
        ManualLoopClock time = new ManualLoopClock();
        assertEquals("camera closed", nearest(TargetObservations2d.unavailable("camera closed"))
                .get(time.clock()).reason());
        assertEquals("observed empty frame", nearest(TargetObservations2d.fromFrame(
                time.clock().nowTimestamp(), Collections.emptyList())).get(time.clock()).reason());
        TargetObservations2d frame = frame(time, 2, 0);
        Source<TargetSelectionResult> selection = nearest(frame);
        assertTrue(selection.get(time.clock()).isUsable(time.clock()));
        time.nextCycle(0.21);
        assertFalse(selection.get(time.clock()).hasSelection());
        time.clock().reset(0);
        assertFalse(selection.get(time.clock()).hasSelection());
    }

    @Test public void retainedResultCannotEvadeCaptureAgeWithoutAnotherSourceRead() {
        ManualLoopClock time = new ManualLoopClock();
        TargetSelectionResult retained = nearest(frame(time, 2, 0)).get(time.clock());
        assertEquals(0.2, retained.maxAgeSec(), EPS);
        time.nextCycle(0.2);
        assertTrue(retained.isUsable(time.clock()));
        time.nextCycle(0.001);
        assertTrue(retained.hasSelection());
        assertFalse(retained.isUsable(time.clock()));
        time.clock().reset(0);
        assertFalse(retained.isUsable(time.clock()));
    }

    @Test public void malformedConfigurationAndInventedMembershipAreRejected() {
        ManualLoopClock time = new ManualLoopClock();
        TargetObservations2d frame = frame(time, 2, 0);
        TargetSelections.FreshnessStep fresh = TargetSelections.from(Source.constant(frame));
        invalid(() -> fresh.freshWithinSec(-1));
        invalid(() -> fresh.freshWithinSec(Double.NaN));
        invalid(() -> fresh.freshWithinSec(Double.POSITIVE_INFINITY));
        TargetSelections.PolicyStep policies = fresh.freshWithinSec(0.2);
        invalid(() -> policies.nearFieldPoint(Double.NaN, 0, 1));
        invalid(() -> policies.nearFieldPoint(0, 0, -1));
        invalid(() -> policies.mostNeighborsWithinInches(Double.POSITIVE_INFINITY));
        invalid(() -> policies.nearestToControlFrame(new Pose2d(0, Double.NaN, 0)));
        TargetObservation2d invented = TargetObservation2d.ofRobotRelativePosition(2, 0, Double.NaN, frame.timestamp());
        invalid(() -> TargetSelectionResult.selected(frame, invented, 0.2, 1, "not in frame"));
        invalid(() -> TargetSelectionResult.selected(frame, frame.observations().get(0), 0.2, Double.NaN, "bad cost"));
        assertFalse(policies.nearestBearingRad(ScalarSource.constant(Double.NaN)).get(time.clock()).hasSelection());
    }

    private static Source<TargetSelectionResult> nearest(TargetObservations2d frame) {
        return TargetSelections.from(Source.constant(frame)).freshWithinSec(0.2).nearestToRobot();
    }

    private static TargetObservations2d frame(ManualLoopClock time, double... xy) {
        LoopTimestamp timestamp = time.clock().nowTimestamp();
        TargetObservation2d[] points = new TargetObservation2d[xy.length / 2];
        for (int i = 0; i < points.length; i++) {
            points[i] = TargetObservation2d.ofRobotRelativePosition(xy[2 * i], xy[2 * i + 1], Double.NaN, timestamp);
        }
        return TargetObservations2d.fromFrame(timestamp, Arrays.asList(points));
    }

    private static void invalid(Runnable action) {
        try { action.run(); fail("expected invalid input rejection"); }
        catch (IllegalArgumentException expected) { }
    }
}
