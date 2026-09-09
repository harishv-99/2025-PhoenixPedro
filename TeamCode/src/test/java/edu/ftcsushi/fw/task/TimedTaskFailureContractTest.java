package edu.ftcsushi.fw.task;

import org.junit.Test;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Exercises real public timed factories; only callbacks and sensor values replace the outside world. */
public final class TimedTaskFailureContractTest {
    @Test
    public void finiteTimingFactoriesRejectNonfiniteAndNegativeDurations() {
        for (double invalid : new double[]{-0.1, Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY}) {
            expectFailure(() -> new RunForSecondsTask(invalid, null, null, null));
            expectFailure(() -> Tasks.waitForSeconds(invalid));
            expectFailure(() -> Tasks.waitUntil(() -> false, invalid));
            expectFailure(() -> Tasks.outputForSeconds("pulse", 0.8, invalid));
            expectFailure(() -> Tasks.outputPulse("pulse").startImmediately().runOutput(0.8)
                    .forSeconds(invalid));
        }
    }

    @Test
    public void timedFactoriesShareBaseWithoutErasingOutputInterfaces() {
        List<Supplier<Task>> factories = Arrays.asList(
                () -> new RunForSecondsTask(1.0, null, null, null),
                () -> Tasks.waitForSeconds(1.0),
                () -> Tasks.waitUntil(() -> false),
                () -> Tasks.waitUntil(() -> false, 1.0),
                () -> Tasks.outputForSeconds("pulse", 0.8, 1.0),
                () -> Tasks.outputPulse("pulse").startImmediately().runOutput(0.8)
                        .forSeconds(1.0).buildTask(),
                () -> Tasks.withTimeout(Tasks.waitForSeconds(1.0), 0.5),
                () -> Tasks.withCleanup(Tasks.waitForSeconds(1.0), () -> { }));
        for (Supplier<Task> factory : factories) {
            Task task = factory.get();
            assertTrue(task.getDebugName(), task instanceof AbstractTask);
            assertFalse(task.isComplete());
            task.cancel();
            assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
            expectFailure(() -> task.start(null));
            task.cancel();
            assertFalse(task.isComplete());
            expectFailure(() -> task.start(new ManualLoopClock().clock()));
        }
        assertTrue(Tasks.outputForSeconds("pulse", 0.8, 1.0) instanceof OutputTask);
        assertTrue(Tasks.outputPulse("pulse").startImmediately().runOutput(0.8)
                .forSeconds(1.0).buildTask() instanceof OutputTask);
    }

    @Test
    public void rawTimedCallbacksRetainStartUpdateAndFinishFailuresAndNeverReplay() {
        for (String failingPhase : Arrays.asList("start", "update", "finish")) {
            RuntimeException failure = new IllegalStateException(failingPhase);
            AtomicInteger starts = new AtomicInteger();
            AtomicInteger updates = new AtomicInteger();
            AtomicInteger finishes = new AtomicInteger();
            RunForSecondsTask task = new RunForSecondsTask(0.1,
                    () -> {
                        starts.incrementAndGet();
                        if (failingPhase.equals("start")) throw failure;
                    },
                    clock -> {
                        updates.incrementAndGet();
                        if (failingPhase.equals("update")) throw failure;
                    },
                    () -> {
                        finishes.incrementAndGet();
                        if (failingPhase.equals("finish")) throw failure;
                    });
            ManualLoopClock clock = new ManualLoopClock();
            if (failingPhase.equals("start")) {
                assertSame(failure, expectFailure(() -> task.start(clock.clock())));
            } else {
                task.start(clock.clock());
                clock.nextCycle(0.1);
                assertSame(failure, expectFailure(() -> task.update(clock.clock())));
            }
            assertTrue(task.isComplete());
            assertSame(failure, expectFailure(task::getRemainingSec));
            assertSame(failure, expectFailure(task::getOutcome));
            assertSame(failure, expectFailure(() -> task.update(clock.clock())));
            clock.nextCycle(0.1);
            assertSame(failure, expectFailure(() -> task.update(clock.clock())));
            task.cancel();
            assertEquals(1, starts.get());
            assertEquals(failingPhase.equals("start") ? 0 : 1, updates.get());
            assertEquals(1, finishes.get());
        }
    }

    @Test
    public void rawTimedUpdateFailurePreservesPrimaryAndSuppressesEndingFailure() {
        RuntimeException primary = new IllegalStateException("update");
        RuntimeException secondary = new IllegalStateException("finish");
        Task task = new RunForSecondsTask(1.0, null, clock -> { throw primary; },
                () -> { throw secondary; });
        ManualLoopClock clock = new ManualLoopClock();
        task.start(clock.clock());
        assertSame(primary, expectFailure(() -> task.update(clock.clock())));
        assertEquals(1, primary.getSuppressed().length);
        assertSame(secondary, primary.getSuppressed()[0]);
        assertSame(primary, expectFailure(task::getOutcome));
    }

    @Test
    public void rawTimedRecursiveAndSameCycleUpdatesRunCallbackOnce() {
        AtomicInteger updates = new AtomicInteger();
        Task[] task = {null};
        task[0] = new RunForSecondsTask(1.0, null, clock -> {
            updates.incrementAndGet();
            task[0].update(clock);
        }, null);
        ManualLoopClock clock = new ManualLoopClock();
        task[0].start(clock.clock());
        task[0].update(clock.clock());
        task[0].update(clock.clock());
        assertEquals(1, updates.get());
        clock.nextCycle(0.1);
        task[0].update(clock.clock());
        assertEquals(2, updates.get());
    }

    @Test
    public void conditionWaitFailureIsNotRetriedByASecondTaskUpdate() {
        AtomicInteger polls = new AtomicInteger();
        RuntimeException failure = new IllegalStateException("sensor");
        Task task = Tasks.waitUntil(() -> {
            polls.incrementAndGet();
            throw failure;
        }, 1.0);
        ManualLoopClock clock = new ManualLoopClock();
        task.start(clock.clock());
        assertSame(failure, expectFailure(() -> task.update(clock.clock())));
        assertSame(failure, expectFailure(() -> task.update(clock.clock())));
        clock.nextCycle(0.1);
        assertSame(failure, expectFailure(() -> task.update(clock.clock())));
        assertSame(failure, expectFailure(task::getOutcome));
        assertEquals(1, polls.get());
    }

    @Test
    public void gatedFailureRestoresItsIdleAndQueueClearsPendingWork() {
        RuntimeException failure = new IllegalStateException("start gate");
        AtomicInteger polls = new AtomicInteger();
        OutputTask pulse = Tasks.outputPulse("feed")
                .startWhen(BooleanSource.of(() -> {
                    polls.incrementAndGet();
                    throw failure;
                }))
                .runOutput(0.8).forSeconds(0.2).idleOutput(-0.1).buildTask();
        OutputTask pending = Tasks.outputForSeconds("pending", 0.9, 0.2);
        OutputTaskRunner queue = Tasks.outputQueue(-0.2);
        queue.enqueue(pulse);
        queue.enqueue(pending);
        ManualLoopClock clock = new ManualLoopClock();
        assertSame(failure, expectFailure(() -> queue.update(clock.clock())));
        assertTrue(queue.isIdle());
        assertEquals(-0.1, pulse.getOutput(), 0.0);
        assertSame(failure, expectFailure(pulse::getOutcome));
        assertSame(failure, expectFailure(() -> pulse.update(clock.clock())));
        assertEquals(1, polls.get());
        assertEquals(TaskOutcome.NOT_DONE, pending.getOutcome());
    }

    @Test
    public void failureAfterGatedRunBeganRestoresIdleWithoutCooldownRetry() {
        RuntimeException failure = new IllegalStateException("done cue");
        AtomicInteger polls = new AtomicInteger();
        OutputTask pulse = Tasks.outputPulse("feed").startImmediately().runOutput(0.8)
                .until(BooleanSource.of(() -> {
                    if (polls.incrementAndGet() == 2) throw failure;
                    return false;
                })).maxRunSec(0.5).idleOutput(-0.1).cooldownSec(0.2).buildTask();
        ManualLoopClock clock = new ManualLoopClock();
        pulse.start(clock.clock());
        pulse.update(clock.clock());
        assertEquals(0.8, pulse.getOutput(), 0.0);
        pulse.update(clock.clock());
        assertEquals(1, polls.get());
        clock.nextCycle(0.1);
        assertSame(failure, expectFailure(() -> pulse.update(clock.clock())));
        assertEquals(-0.1, pulse.getOutput(), 0.0);
        assertTrue(pulse.isComplete());
        assertSame(failure, expectFailure(pulse::getOutcome));
        pulse.cancel();
        assertEquals(2, polls.get());
    }

    /** Return the exact thrown RuntimeException so the assertions prove failure identity. */
    private static RuntimeException expectFailure(Runnable action) {
        try {
            action.run();
            fail("Expected RuntimeException");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }
}
