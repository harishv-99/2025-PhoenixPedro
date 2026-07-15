package edu.ftcphoenix.fw.task;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the hard execution-budget decorator exposed by {@link Tasks#withTimeout}. */
public final class TimeoutTaskTest {

    @Test
    public void validatesFactoryInputsAndKeepsImplementationInternal() {
        assertContains(
                expectNullPointer(() -> Tasks.withTimeout(null, 1.0)),
                "Tasks.withTimeout",
                "must not be null");

        for (double invalid : Arrays.asList(
                -0.01,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY)) {
            assertContains(
                    expectIllegalArgument(
                            () -> Tasks.withTimeout(new ProbeTask("child"), invalid)),
                    "timeoutSec",
                    "finite",
                    ">= 0");
        }

        assertTrue(Modifier.isFinal(TimeoutTask.class.getModifiers()));
        assertFalse(Modifier.isPublic(TimeoutTask.class.getModifiers()));
        Constructor<?>[] constructors = TimeoutTask.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertFalse(Modifier.isPublic(constructors[0].getModifiers()));
        assertEquals(TimeoutTask.class,
                Tasks.withTimeout(new ProbeTask("child"), 1.0).getClass());
    }

    @Test
    public void zeroTimeoutCompletesWithoutStartingOrCancellingChild() {
        ManualLoopClock clock = new ManualLoopClock(7.0);
        ProbeTask child = new ProbeTask("child");
        Task bounded = Tasks.withTimeout(child, 0.0);

        bounded.cancel();
        assertFalse(bounded.isComplete());
        bounded.start(clock.clock());

        assertTrue(bounded.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, bounded.getOutcome());
        assertEquals(0, child.startCount);
        assertEquals(0, child.updateCount);
        assertEquals(0, child.cancelCount);

        bounded.update(clock.clock());
        bounded.cancel();
        assertEquals(0, child.startCount);
        assertEquals(0, child.cancelCount);
        assertContains(
                expectIllegalState(() -> bounded.start(clock.clock())),
                "Tasks.withTimeout",
                "single-use",
                "fresh task");
    }

    @Test
    public void positiveTimeoutUsesStartBoundaryAndDoesNotDoubleUpdateOneCycle() {
        ManualLoopClock clock = new ManualLoopClock(7.0);
        clock.nextCycle(5.0); // This preceding dt must not be charged to the new wrapper.
        ProbeTask child = new ProbeTask("child");
        AtomicReference<Task> boundedRef = new AtomicReference<>();
        child.updateHook = () -> boundedRef.get().update(clock.clock());
        Task bounded = Tasks.withTimeout(child, 1.0);
        boundedRef.set(bounded);

        bounded.start(clock.clock());
        bounded.update(clock.clock());
        bounded.update(clock.clock());

        assertEquals(1, child.startCount);
        assertEquals(1, child.updateCount);
        assertFalse(bounded.isComplete());

        bounded.update(clock.nextCycle(0.5));
        assertEquals(2, child.updateCount);
        assertFalse(bounded.isComplete());

        bounded.update(clock.nextCycle(0.5));

        assertTrue(bounded.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, bounded.getOutcome());
        assertEquals(2, child.updateCount);
        assertEquals(1, child.cancelCount);
    }

    @Test
    public void firstObservationAfterThresholdCancelsWithoutAnotherChildUpdate() {
        ManualLoopClock clock = new ManualLoopClock(20.0);
        ProbeTask child = new ProbeTask("child");
        Task bounded = Tasks.withTimeout(child, 1.0);
        bounded.start(clock.clock());

        bounded.update(clock.nextCycle(1.25));

        assertTrue(bounded.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, bounded.getOutcome());
        assertEquals(0, child.updateCount);
        assertEquals(1, child.cancelCount);
    }

    @Test
    public void alreadyTerminalChildWinsAtExactBoundary() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask("child");
        Task bounded = Tasks.withTimeout(child, 1.0);
        bounded.start(clock.clock());
        bounded.update(clock.clock());
        child.finish(TaskOutcome.SUCCESS);

        bounded.update(clock.nextCycle(1.0));

        assertTrue(bounded.isComplete());
        assertEquals(TaskOutcome.SUCCESS, bounded.getOutcome());
        assertEquals(1, child.updateCount);
        assertEquals(0, child.cancelCount);
    }

    @Test
    public void naturalCompletionRetainsEveryValidTerminalOutcome() {
        for (TaskOutcome terminal : Arrays.asList(
                TaskOutcome.SUCCESS,
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN)) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask child = new ProbeTask("immediate-" + terminal);
            child.startHook = () -> child.finish(terminal);
            Task bounded = Tasks.withTimeout(child, 2.0);

            bounded.start(clock.clock());

            assertTrue(bounded.isComplete());
            assertEquals(terminal, bounded.getOutcome());
            assertEquals(0, child.updateCount);
            assertEquals(0, child.cancelCount);
        }

        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask updated = new ProbeTask("updated");
        updated.updateHook = () -> updated.finish(TaskOutcome.UNKNOWN);
        Task bounded = Tasks.withTimeout(updated, 2.0);
        bounded.start(clock.clock());
        bounded.update(clock.clock());

        assertTrue(bounded.isComplete());
        assertEquals(TaskOutcome.UNKNOWN, bounded.getOutcome());
        assertEquals(1, updated.updateCount);
    }

    @Test
    public void malformedNaturalOutcomeFailsClosed() {
        for (TaskOutcome invalid : Arrays.asList(null, TaskOutcome.NOT_DONE)) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask child = new ProbeTask("malformed");
            child.startHook = () -> child.finish(invalid);
            Task bounded = Tasks.withTimeout(child, 1.0);
            TaskRunner runner = new TaskRunner();
            runner.enqueue(bounded);

            IllegalStateException thrown =
                    expectIllegalState(() -> runner.update(clock.clock()));

            assertContains(thrown, "child was completed", String.valueOf(invalid),
                    "lifecycle contract");
            assertTrue(runner.isIdle());
            assertEquals(TaskOutcome.CANCELLED, bounded.getOutcome());
            assertEquals(1, child.cancelCount);
        }
    }

    @Test
    public void directCancellationIsActiveOnlyTerminalAndIdempotent() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask("child");
        Task bounded = Tasks.withTimeout(child, 4.0);

        bounded.cancel();
        assertFalse(bounded.isComplete());
        assertEquals(0, child.cancelCount);

        bounded.start(clock.clock());
        bounded.cancel();
        bounded.cancel();
        bounded.update(clock.clock());

        assertTrue(bounded.isComplete());
        assertEquals(TaskOutcome.CANCELLED, bounded.getOutcome());
        assertEquals(1, child.cancelCount);

        ProbeTask terminalChild = new ProbeTask("terminal");
        terminalChild.startHook = () -> terminalChild.finish(TaskOutcome.SUCCESS);
        Task terminal = Tasks.withTimeout(terminalChild, 4.0);
        terminal.start(clock.clock());
        terminal.cancel();
        assertEquals(TaskOutcome.SUCCESS, terminal.getOutcome());
        assertEquals(0, terminalChild.cancelCount);
    }

    @Test
    public void successfulTimeoutPublishesOnlyAfterTerminalCleanupAndIgnoresReentry() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask("child");
        AtomicReference<Task> boundedRef = new AtomicReference<>();
        child.cancelHook = () -> {
            Task bounded = boundedRef.get();
            assertFalse(bounded.isComplete());
            assertEquals(TaskOutcome.NOT_DONE, bounded.getOutcome());
            bounded.cancel();
            bounded.update(clock.clock());
        };
        Task bounded = Tasks.withTimeout(child, 0.5);
        boundedRef.set(bounded);
        bounded.start(clock.clock());
        bounded.update(clock.clock());

        bounded.update(clock.nextCycle(0.5));

        assertTrue(bounded.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, bounded.getOutcome());
        assertEquals(1, child.cancelCount);
        assertEquals(TaskOutcome.CANCELLED, child.getOutcome());
    }

    @Test
    public void throwingTimeoutCleanupFailsClosedAndNeverStartsContinuation() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask("child");
        RuntimeException cleanupFailure = new RuntimeException("cleanup failed");
        child.cancelFailure = cleanupFailure;
        FailureGraph graph = timeoutFailureGraph(clock, child);

        RuntimeException thrown =
                expectRuntime(() -> graph.runner.update(clock.nextCycle(0.5)));

        assertSame(cleanupFailure, thrown);
        assertTrue(graph.runner.isIdle());
        assertEquals(0, graph.continuation.startCount);
        assertEquals(1, child.cancelCount);
        assertEquals(TaskOutcome.CANCELLED, graph.bounded.getOutcome());
    }

    @Test
    public void nonterminalTimeoutCleanupFailsClosedAndIsNotRetried() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask("child");
        child.completeOnCancel = false;
        FailureGraph graph = timeoutFailureGraph(clock, child);

        IllegalStateException thrown =
                expectIllegalState(() -> graph.runner.update(clock.nextCycle(0.5)));

        assertContains(thrown, "cancel() returned", "terminal", "isComplete()");
        assertTrue(graph.runner.isIdle());
        assertEquals(0, graph.continuation.startCount);
        assertEquals(1, child.cancelCount);
        assertEquals(TaskOutcome.CANCELLED, graph.bounded.getOutcome());
    }

    @Test
    public void malformedOutcomeAfterTimeoutCleanupFailsClosedAndIsNotRetried() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask("child");
        child.cancelOutcome = TaskOutcome.NOT_DONE;
        FailureGraph graph = timeoutFailureGraph(clock, child);

        IllegalStateException thrown =
                expectIllegalState(() -> graph.runner.update(clock.nextCycle(0.5)));

        assertContains(thrown, "cancelled at the timeout", "NOT_DONE", "lifecycle contract");
        assertTrue(graph.runner.isIdle());
        assertEquals(0, graph.continuation.startCount);
        assertEquals(1, child.cancelCount);
        assertEquals(TaskOutcome.CANCELLED, graph.bounded.getOutcome());
    }

    @Test
    public void childLifecycleFailuresUseRunnerFailStopCleanup() {
        for (FailurePoint point : FailurePoint.values()) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask child = new ProbeTask("child-" + point);
            RuntimeException failure = new RuntimeException(point + " failed");
            switch (point) {
                case START:
                    child.startFailure = failure;
                    break;
                case UPDATE:
                    child.updateFailure = failure;
                    break;
                case COMPLETE:
                    child.completeFailure = failure;
                    break;
                case OUTCOME:
                    child.startHook = () -> child.finish(TaskOutcome.SUCCESS);
                    child.outcomeFailure = failure;
                    break;
                default:
                    throw new AssertionError(point);
            }
            Task bounded = Tasks.withTimeout(child, 2.0);
            TaskRunner runner = new TaskRunner();
            runner.enqueue(bounded);

            RuntimeException thrown = expectRuntime(() -> runner.update(clock.clock()));

            assertSame(failure, thrown);
            assertTrue(runner.isIdle());
            assertEquals(TaskOutcome.CANCELLED, bounded.getOutcome());
            assertEquals(1, child.cancelCount);
        }
    }

    @Test
    public void updateBeforeStartAndReentrantStartCallbacksAreGuarded() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask("child");
        Task bounded = Tasks.withTimeout(child, 1.0);
        assertContains(
                expectIllegalState(() -> bounded.update(clock.clock())),
                "Tasks.withTimeout",
                "before start",
                "TaskRunner");

        AtomicReference<Task> boundedRef = new AtomicReference<>();
        ProbeTask reentrant = new ProbeTask("reentrant");
        reentrant.startHook = () -> {
            boundedRef.get().update(clock.clock());
            boundedRef.get().cancel();
        };
        Task reentrantBounded = Tasks.withTimeout(reentrant, 1.0);
        boundedRef.set(reentrantBounded);

        reentrantBounded.start(clock.clock());

        assertEquals(TaskOutcome.CANCELLED, reentrantBounded.getOutcome());
        assertEquals(0, reentrant.updateCount);
        assertEquals(1, reentrant.cancelCount);
    }

    @Test
    public void debugDumpRetainsTimeoutAndChildState() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask("parkPrelude");
        Task bounded = Tasks.withTimeout(child, 0.5);
        bounded.start(clock.clock());
        bounded.update(clock.clock());
        bounded.update(clock.nextCycle(0.5));
        CapturingDebugSink sink = new CapturingDebugSink();

        bounded.debugDump(sink, "auto.bounded");

        assertEquals(Double.valueOf(0.5), sink.values.get("auto.bounded.timeoutSec"));
        assertEquals(Boolean.TRUE, sink.values.get("auto.bounded.timeoutFired"));
        assertEquals(Boolean.TRUE, sink.values.get("auto.bounded.complete"));
        assertEquals(TaskOutcome.TIMEOUT, sink.values.get("auto.bounded.outcome"));
        assertEquals(TaskOutcome.CANCELLED,
                sink.values.get("auto.bounded.retainedChildOutcome"));
        assertEquals("parkPrelude", sink.values.get("auto.bounded.child.name"));
        assertEquals(TaskOutcome.CANCELLED,
                sink.values.get("auto.bounded.child.outcome"));
    }

    private static FailureGraph timeoutFailureGraph(ManualLoopClock clock, ProbeTask child) {
        Task bounded = Tasks.withTimeout(child, 0.5);
        ProbeTask continuation = new ProbeTask("continuation");
        Task sequence = Tasks.sequence(bounded, continuation);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(sequence);
        runner.update(clock.clock());
        return new FailureGraph(bounded, continuation, runner);
    }

    private enum FailurePoint {
        START,
        UPDATE,
        COMPLETE,
        OUTCOME
    }

    private static final class FailureGraph {
        private final Task bounded;
        private final ProbeTask continuation;
        private final TaskRunner runner;

        private FailureGraph(Task bounded, ProbeTask continuation, TaskRunner runner) {
            this.bounded = bounded;
            this.continuation = continuation;
            this.runner = runner;
        }
    }

    private static final class ProbeTask implements Task {
        private final String name;
        private int startCount;
        private int updateCount;
        private int cancelCount;
        private boolean started;
        private boolean complete;
        private boolean completeOnCancel = true;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;
        private TaskOutcome cancelOutcome = TaskOutcome.CANCELLED;
        private Runnable startHook;
        private Runnable updateHook;
        private Runnable cancelHook;
        private RuntimeException startFailure;
        private RuntimeException updateFailure;
        private RuntimeException completeFailure;
        private RuntimeException outcomeFailure;
        private RuntimeException cancelFailure;

        private ProbeTask(String name) {
            this.name = name;
        }

        @Override
        public void start(LoopClock clock) {
            startCount++;
            started = true;
            if (startHook != null) {
                startHook.run();
            }
            if (startFailure != null) {
                throw startFailure;
            }
        }

        @Override
        public void update(LoopClock clock) {
            updateCount++;
            if (updateHook != null) {
                updateHook.run();
            }
            if (updateFailure != null) {
                throw updateFailure;
            }
        }

        @Override
        public void cancel() {
            cancelCount++;
            if (!started || complete) {
                return;
            }
            if (completeOnCancel) {
                complete = true;
                outcome = cancelOutcome;
            }
            if (cancelHook != null) {
                cancelHook.run();
            }
            if (cancelFailure != null) {
                throw cancelFailure;
            }
        }

        @Override
        public boolean isComplete() {
            if (completeFailure != null) {
                throw completeFailure;
            }
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            if (outcomeFailure != null) {
                throw outcomeFailure;
            }
            return outcome;
        }

        @Override
        public String getDebugName() {
            return name;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) {
                return;
            }
            String p = (prefix == null || prefix.isEmpty()) ? "probe" : prefix;
            dbg.addData(p + ".name", name)
                    .addData(p + ".started", started)
                    .addData(p + ".complete", complete)
                    .addData(p + ".outcome", outcome);
        }

        private void finish(TaskOutcome terminalOutcome) {
            complete = true;
            outcome = terminalOutcome;
        }
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> values = new LinkedHashMap<>();

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

    private static void assertContains(RuntimeException failure, String... fragments) {
        String message = failure.getMessage();
        for (String fragment : fragments) {
            assertTrue(
                    "Expected message to contain '" + fragment + "' but was: " + message,
                    message != null && message.contains(fragment));
        }
    }

    private static NullPointerException expectNullPointer(Runnable action) {
        try {
            action.run();
            fail("Expected NullPointerException");
            return null;
        } catch (NullPointerException expected) {
            return expected;
        }
    }

    private static IllegalArgumentException expectIllegalArgument(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalArgumentException");
            return null;
        } catch (IllegalArgumentException expected) {
            return expected;
        }
    }

    private static IllegalStateException expectIllegalState(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalStateException");
            return null;
        } catch (IllegalStateException expected) {
            return expected;
        }
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
}
