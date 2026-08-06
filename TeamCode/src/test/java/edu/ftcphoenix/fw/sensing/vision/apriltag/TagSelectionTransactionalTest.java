package edu.ftcphoenix.fw.sensing.vision.apriltag;

import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Regression coverage for all-or-nothing AprilTag selection observations. */
public final class TagSelectionTransactionalTest {

    @Test
    public void detectionFailureCanRetryAndOnlySuccessCommitsTheCycle() {
        ManualLoopClock time = new ManualLoopClock();
        RuntimeException failure = new RuntimeException("camera read failed");
        int[] calls = {0};
        AprilTagDetections frame = frame(time.clock(), 4);
        Source<AprilTagDetections> detections = clock -> {
            calls[0]++;
            if (calls[0] == 1) {
                throw failure;
            }
            return frame;
        };
        TagSelectionSource selection = continuousSelection(detections, 4);

        assertSame(failure, expectRuntime(() -> selection.get(time.clock())));

        TagSelectionResult result = selection.get(time.clock());
        assertEquals(4, result.selectedTagId);
        assertSame(result, selection.get(time.clock()));
        assertEquals(2, calls[0]);
    }

    @Test
    public void enableFailureDoesNotPublishPreviewStickyOrDiagnosticState() {
        ManualLoopClock time = new ManualLoopClock();
        MutableDetections detections = new MutableDetections(frame(time.clock(), 1));
        FailingEnabled enabled = new FailingEnabled();
        TagSelectionSource selection = TagSelections.from(detections)
                .among(setOf(1, 2))
                .freshWithinSec(1.0)
                .choose(firstCandidatePolicy())
                .stickyWhen(enabled)
                .reacquireAfterLossSec(0.0)
                .build();

        TagSelectionResult first = selection.get(time.clock());
        assertEquals(1, first.selectedTagId);
        assertEquals("tag-1", first.reason);

        time.nextCycle(0.02);
        detections.value = frame(time.clock(), 2);
        RuntimeException failure = new RuntimeException("enable failed");
        enabled.nextFailure = failure;

        assertSame(failure, expectRuntime(() -> selection.get(time.clock())));

        CapturingDebugSink debugAfterFailure = new CapturingDebugSink();
        selection.debugDump(debugAfterFailure, "selector");
        assertEquals(1.0,
                ((Number) debugAfterFailure.values.get("selector.selectedTagId")).doubleValue(),
                0.0);
        assertEquals("tag-1", debugAfterFailure.values.get("selector.reason"));

        TagSelectionResult retry = selection.get(time.clock());
        assertEquals(2, retry.previewTagId);
        assertEquals(2, retry.selectedTagId);
        assertEquals("tag-2", retry.reason);

        CapturingDebugSink debugAfterSuccess = new CapturingDebugSink();
        selection.debugDump(debugAfterSuccess, "selector");
        assertEquals(2.0,
                ((Number) debugAfterSuccess.values.get("selector.selectedTagId")).doubleValue(),
                0.0);
        assertEquals("tag-2", debugAfterSuccess.values.get("selector.reason"));
    }

    @Test
    public void freshnessAndPolicyFailuresKeepPriorStickyStateUntilRetrySucceeds() {
        ManualLoopClock time = new ManualLoopClock();
        MutableDetections detections = new MutableDetections(frame(time.clock(), 1));
        RuntimeException policyFailure = new RuntimeException("selection policy failed");
        boolean[] failPolicy = {false};
        TagSelectionPolicy firstCandidate = firstCandidatePolicy();
        TagSelectionSource selection = TagSelections.from(detections)
                .among(setOf(1, 2))
                .freshWithinSec(1.0)
                .choose(candidates -> {
                    if (failPolicy[0]) {
                        failPolicy[0] = false;
                        throw policyFailure;
                    }
                    return firstCandidate.choose(candidates);
                })
                .stickyWhen(BooleanSource.constant(true))
                .reacquireAfterLossSec(0.0)
                .build();

        assertEquals(1, selection.get(time.clock()).selectedTagId);

        time.nextCycle(0.02);
        LoopClock foreignClock = new LoopClock();
        foreignClock.update(0.02);
        detections.value = frame(foreignClock, 2);
        RuntimeException freshnessFailure =
                expectRuntime(() -> selection.get(time.clock()));
        assertTrue(freshnessFailure instanceof IllegalArgumentException);

        detections.value = frame(time.clock(), 2);
        failPolicy[0] = true;
        assertSame(policyFailure, expectRuntime(() -> selection.get(time.clock())));

        CapturingDebugSink debugAfterFailures = new CapturingDebugSink();
        selection.debugDump(debugAfterFailures, "selector");
        assertEquals(1.0,
                ((Number) debugAfterFailures.values.get("selector.selectedTagId")).doubleValue(),
                0.0);
        assertEquals("tag-1", debugAfterFailures.values.get("selector.reason"));

        TagSelectionResult recovered = selection.get(time.clock());
        assertEquals(2, recovered.selectedTagId);
        assertEquals("tag-2", recovered.reason);
    }

    @Test
    public void recursiveSelectionReadFailsActionablyAndCanRecoverSameCycle() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections frame = frame(time.clock(), 7);
        TagSelectionSource[] selectionRef = new TagSelectionSource[1];
        boolean[] recurse = {true};
        Source<AprilTagDetections> detections = clock -> {
            if (recurse[0]) {
                selectionRef[0].get(clock);
            }
            return frame;
        };
        selectionRef[0] = continuousSelection(detections, 7);

        RuntimeException observed = expectRuntime(() -> selectionRef[0].get(time.clock()));
        assertTrue(observed instanceof IllegalStateException);
        assertTrue(observed.getMessage().contains("reentrantly"));

        recurse[0] = false;
        assertEquals(7, selectionRef[0].get(time.clock()).selectedTagId);
    }

    @Test
    public void failedChildResetKeepsTheCommittedLocalSelectionCache() {
        ManualLoopClock time = new ManualLoopClock();
        MutableDetections detections = new MutableDetections(frame(time.clock(), 3));
        FailingEnabled enabled = new FailingEnabled();
        TagSelectionSource selection = TagSelections.from(detections)
                .among(Collections.singleton(3))
                .freshWithinSec(1.0)
                .choose(firstCandidatePolicy())
                .stickyWhen(enabled)
                .holdUntilDisabled()
                .build();
        TagSelectionResult committed = selection.get(time.clock());
        RuntimeException failure = new RuntimeException("enable reset failed");
        enabled.resetFailure = failure;

        assertSame(failure, expectRuntime(selection::reset));
        assertSame(committed, selection.get(time.clock()));

        enabled.resetFailure = null;
        selection.reset();
        assertEquals(3, selection.get(time.clock()).selectedTagId);
        assertEquals(2, detections.sampleCalls);
    }

    private static TagSelectionSource continuousSelection(
            Source<AprilTagDetections> detections,
            int candidateId
    ) {
        return TagSelections.from(detections)
                .among(Collections.singleton(candidateId))
                .freshWithinSec(1.0)
                .choose(firstCandidatePolicy())
                .continuous()
                .build();
    }

    private static TagSelectionPolicy firstCandidatePolicy() {
        return candidates -> {
            if (candidates.isEmpty()) {
                return null;
            }
            AprilTagObservation observation = candidates.get(0);
            return new TagSelectionChoice(
                    observation,
                    "firstCandidate",
                    "tag-" + observation.id,
                    observation.id
            );
        };
    }

    private static AprilTagDetections frame(LoopClock clock, int id) {
        return AprilTagDetections.fromFrame(
                clock.nowTimestamp(),
                Collections.singletonList(AprilTagObservation.target(id, Pose3d.zero()))
        );
    }

    private static Set<Integer> setOf(int first, int second) {
        return new LinkedHashSet<Integer>(Arrays.asList(first, second));
    }

    private static RuntimeException expectRuntime(Runnable action) {
        try {
            action.run();
            fail("Expected RuntimeException");
            return null;
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static final class MutableDetections implements Source<AprilTagDetections> {
        private AprilTagDetections value;
        private int sampleCalls;

        private MutableDetections(AprilTagDetections value) {
            this.value = value;
        }

        @Override
        public AprilTagDetections get(LoopClock clock) {
            sampleCalls++;
            return value;
        }
    }

    private static final class FailingEnabled implements BooleanSource {
        private RuntimeException nextFailure;
        private RuntimeException resetFailure;

        @Override
        public boolean getAsBoolean(LoopClock clock) {
            if (nextFailure != null) {
                RuntimeException failure = nextFailure;
                nextFailure = null;
                throw failure;
            }
            return true;
        }

        @Override
        public void reset() {
            if (resetFailure != null) {
                throw resetFailure;
            }
        }
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> values = new LinkedHashMap<String, Object>();

        @Override
        public DebugSink addData(String key, Object value) {
            values.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }
    }
}
