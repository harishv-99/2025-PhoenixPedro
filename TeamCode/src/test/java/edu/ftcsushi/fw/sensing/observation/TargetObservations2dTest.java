package edu.ftcsushi.fw.sensing.observation;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Snapshot and validation contracts, independent of camera or physical object assumptions. */
public final class TargetObservations2dTest {
    private final ManualLoopClock time = new ManualLoopClock(1);
    private final LoopTimestamp timestamp = time.clock().nowTimestamp();

    @Test public void observedEmptyAndUnavailableRemainDifferentAcrossTimeAndReset() {
        TargetObservations2d empty = TargetObservations2d.fromFrame(timestamp, Collections.emptyList());
        TargetObservations2d absent = TargetObservations2d.unavailable("camera opening");
        assertTrue(empty.isAvailable());
        assertTrue(empty.isFresh(time.clock(), 0));
        assertFalse(absent.isAvailable());
        assertFalse(absent.isFresh(time.clock(), 10));
        assertSame(timestamp, empty.timestamp());
        assertSame(LoopTimestamp.unavailable(), absent.timestamp());
        time.nextCycle(0.21);
        assertFalse(empty.isFresh(time.clock(), 0.2));
        time.clock().reset(1.21);
        assertFalse(empty.isFresh(time.clock(), 10));
    }

    @Test public void frameDefensivelyCopiesAndKeepsExactObservationIdentity() {
        TargetObservation2d point = point(1, 2);
        List<TargetObservation2d> draft = new ArrayList<>();
        draft.add(point);
        TargetObservations2d frame = TargetObservations2d.fromFrame(timestamp, draft);
        draft.clear();
        assertSame(point, frame.observations().get(0));
        try { frame.observations().clear(); fail("frame must be immutable"); }
        catch (UnsupportedOperationException expected) { }
    }

    @Test public void frameRejectsOverBoundMixedTimestampAbsentAndDuplicateMembers() {
        TargetObservation2d point = point(1, 2);
        expectInvalid(() -> TargetObservations2d.fromFrame(timestamp,
                Collections.nCopies(TargetObservations2d.MAX_OBSERVATIONS + 1, point)));
        expectInvalid(() -> TargetObservations2d.fromFrame(timestamp, Arrays.asList(point, point)));
        expectInvalid(() -> TargetObservations2d.fromFrame(timestamp,
                Collections.singletonList(TargetObservation2d.none())));
        expectInvalid(() -> TargetObservations2d.fromFrame(time.clock().nowTimestamp(),
                Collections.singletonList(point)));
        expectInvalid(() -> TargetObservations2d.fromFrame(LoopTimestamp.unavailable(),
                Collections.emptyList()));
        expectInvalid(() -> TargetObservations2d.unavailable("  "));
    }

    @Test public void unknownConfidenceIsNotZeroAndMalformedGeometryIsRejected() {
        TargetObservation2d point = point(1, 2);
        assertTrue(Double.isNaN(point.quality));
        assertFalse(point.hasQuality());
        assertFalse(point.hasTargetId());
        assertFalse(point.hasFieldPosition());
        assertNull(point.fieldLookup());
        for (double value : new double[] {Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY}) {
            expectInvalid(() -> point(value, 2));
            expectInvalid(() -> TargetObservation2d.ofRobotRelativeBearing(value, 0.5, timestamp));
        }
        for (double quality : new double[] {-0.1, 1.1, Double.POSITIVE_INFINITY}) {
            expectInvalid(() -> TargetObservation2d.ofRobotRelativePosition(1, 2, quality, timestamp));
        }
        expectInvalid(() -> TargetObservation2d.ofRobotRelativePosition(-2, 1, 2, 0.5, timestamp));
    }

    private TargetObservation2d point(double x, double y) {
        return TargetObservation2d.ofRobotRelativePosition(x, y, Double.NaN, timestamp);
    }

    private static void expectInvalid(Runnable action) {
        try { action.run(); fail("expected invalid input"); }
        catch (IllegalArgumentException expected) { assertFalse(expected.getMessage().isEmpty()); }
    }
}
