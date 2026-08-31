package edu.ftcsushi.fw.task;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the bounded fresh-child composition exposed by {@link Tasks#repeatWhileSuccessful}. */
public final class RepeatWhileSuccessfulTaskTest {

    @Test
    public void validatesPublicFactoryAndKeepsImplementationInternal() throws Exception {
        for (String invalidName : Arrays.asList(null, "", " \t ")) {
            IllegalArgumentException failure = expectIllegalArgument(
                    () -> Tasks.repeatWhileSuccessful(
                            invalidName,
                            1,
                            BooleanSource.constant(true),
                            Tasks::noop));
            assertContains(failure, "Tasks.repeatWhileSuccessful", "debugName", "nonblank");
        }

        for (int invalidLimit : Arrays.asList(0, -1, Integer.MIN_VALUE)) {
            IllegalArgumentException failure = expectIllegalArgument(
                    () -> Tasks.repeatWhileSuccessful(
                            "attempts",
                            invalidLimit,
                            BooleanSource.constant(true),
                            Tasks::noop));
            assertContains(failure, "Tasks.repeatWhileSuccessful", "maxIterations", "> 0");
        }

        NullPointerException missingAdmission = expectNullPointer(
                () -> Tasks.repeatWhileSuccessful("attempts", 1, null, Tasks::noop));
        assertContains(missingAdmission, "Tasks.repeatWhileSuccessful", "mayStartIteration");

        NullPointerException missingFactory = expectNullPointer(
                () -> Tasks.repeatWhileSuccessful(
                        "attempts",
                        1,
                        BooleanSource.constant(true),
                        null));
        assertContains(missingFactory, "Tasks.repeatWhileSuccessful", "Task factory");

        Method factory = Tasks.class.getMethod(
                "repeatWhileSuccessful",
                String.class,
                int.class,
                BooleanSource.class,
                Supplier.class);
        assertTrue(Modifier.isPublic(factory.getModifiers()));
        assertTrue(Modifier.isStatic(factory.getModifiers()));
        assertEquals(Task.class, factory.getReturnType());

        assertTrue(Modifier.isFinal(RepeatWhileSuccessfulTask.class.getModifiers()));
        assertFalse(Modifier.isPublic(RepeatWhileSuccessfulTask.class.getModifiers()));
        Constructor<?>[] constructors =
                RepeatWhileSuccessfulTask.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertFalse(Modifier.isPublic(constructors[0].getModifiers()));
        assertEquals(
                RepeatWhileSuccessfulTask.class,
                Tasks.repeatWhileSuccessful(
                        "attempts",
                        1,
                        BooleanSource.constant(false),
                        Tasks::noop).getClass());

        // The positive bound is a lifecycle limit, not an eager storage allocation request.
        Task maximumBound = Tasks.repeatWhileSuccessful(
                "maximum bounded attempts",
                Integer.MAX_VALUE,
                BooleanSource.constant(false),
                Tasks::noop);
        maximumBound.start(new ManualLoopClock().clock());
        assertTrue(maximumBound.isComplete());
        assertEquals(TaskOutcome.SUCCESS, maximumBound.getOutcome());
    }

    @Test
    public void callbacksStayDeferredAndFalseFirstAdmissionCreatesZeroChildren() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger admissionCalls = new AtomicInteger();
        AtomicInteger factoryCalls = new AtomicInteger();
        Task repeat = Tasks.repeatWhileSuccessful(
                "adaptive attempts",
                4,
                observedClock -> {
                    assertSame(clock.clock(), observedClock);
                    admissionCalls.incrementAndGet();
                    return false;
                },
                () -> {
                    factoryCalls.incrementAndGet();
                    return Tasks.noop();
                });
        CapturingDebugSink debug = new CapturingDebugSink();

        assertEquals("adaptive attempts", repeat.getDebugName());
        assertFalse(repeat.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, repeat.getOutcome());
        repeat.debugDump(debug, "auto.repeat");
        repeat.debugDump(null, "ignored");
        repeat.cancel();
        repeat.cancel();
        assertContains(
                expectIllegalState(() -> repeat.update(clock.clock())),
                "Tasks.repeatWhileSuccessful",
                "before start",
                "TaskRunner");

        assertEquals(0, admissionCalls.get());
        assertEquals(0, factoryCalls.get());
        assertEquals("NOT_STARTED", stringValue(debug, "auto.repeat.phase"));
        assertEquals(Double.valueOf(0.0),
                debug.values.get("auto.repeat.admissionEvaluations"));

        repeat.start(clock.clock());

        assertTrue(repeat.isComplete());
        assertEquals(TaskOutcome.SUCCESS, repeat.getOutcome());
        assertEquals(1, admissionCalls.get());
        assertEquals(0, factoryCalls.get());
        CapturingDebugSink stopped = new CapturingDebugSink();
        repeat.debugDump(stopped, "auto.repeat");
        assertEquals("CONDITION_FALSE", stringValue(stopped, "auto.repeat.stopReason"));
        repeat.update(clock.clock());
        repeat.cancel();
        assertEquals(1, admissionCalls.get());
        assertEquals(0, factoryCalls.get());
    }

    @Test
    public void admissionFactoryAndChildStartShareTheExactClockAndOrder() {
        ManualLoopClock clock = new ManualLoopClock(12.5);
        List<String> events = new ArrayList<>();
        ProbeTask child = new ProbeTask("attempt-1");
        child.startHook = observedClock -> {
            assertSame(clock.clock(), observedClock);
            events.add("start@" + observedClock.cycle());
        };
        Task repeat = Tasks.repeatWhileSuccessful(
                "attempts",
                1,
                observedClock -> {
                    assertSame(clock.clock(), observedClock);
                    events.add("admit@" + observedClock.cycle());
                    return true;
                },
                () -> {
                    events.add("factory@" + clock.clock().cycle());
                    return child;
                });

        repeat.start(clock.clock());

        assertEquals(
                Arrays.asList("admit@1", "factory@1", "start@1"),
                events);
        assertEquals(1, child.startCount);
        assertSame(clock.clock(), child.startClock);
        assertFalse(repeat.isComplete());

        repeat.update(clock.clock());

        assertEquals(1, child.updateCount);
        assertSame(clock.clock(), child.updateClock);
        child.finish(TaskOutcome.SUCCESS);
        repeat.update(clock.nextCycle(0.02));
        assertTrue(repeat.isComplete());
        assertEquals(TaskOutcome.SUCCESS, repeat.getOutcome());
    }

    @Test
    public void laterFalseAdmissionStopsAfterSuccessWithoutAnotherFactoryCall() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger admissionCalls = new AtomicInteger();
        AtomicInteger factoryCalls = new AtomicInteger();
        ProbeTask first = ProbeTask.immediate("first", TaskOutcome.SUCCESS);
        Task repeat = Tasks.repeatWhileSuccessful(
                "attempts",
                5,
                observedClock -> admissionCalls.incrementAndGet() == 1,
                () -> {
                    factoryCalls.incrementAndGet();
                    return first;
                });

        repeat.start(clock.clock());
        repeat.update(clock.clock());
        assertFalse(repeat.isComplete());
        assertEquals(1, admissionCalls.get());
        assertEquals(1, factoryCalls.get());

        repeat.update(clock.nextCycle(0.02));

        assertTrue(repeat.isComplete());
        assertEquals(TaskOutcome.SUCCESS, repeat.getOutcome());
        assertEquals(2, admissionCalls.get());
        assertEquals(1, factoryCalls.get());
        assertEquals(1, first.startCount);
    }

    @Test
    public void maximumEndsWithoutAnotherAdmissionOrFactoryCall() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger admissionCalls = new AtomicInteger();
        AtomicInteger factoryCalls = new AtomicInteger();
        Task repeat = Tasks.repeatWhileSuccessful(
                "attempts",
                3,
                observedClock -> {
                    admissionCalls.incrementAndGet();
                    return true;
                },
                () -> {
                    factoryCalls.incrementAndGet();
                    return ProbeTask.immediate(
                            "attempt-" + factoryCalls.get(),
                            TaskOutcome.SUCCESS);
                });

        repeat.start(clock.clock());
        repeat.update(clock.clock());
        assertEquals(1, factoryCalls.get());
        repeat.update(clock.nextCycle(0.02));
        repeat.update(clock.clock());
        assertEquals(2, factoryCalls.get());
        repeat.update(clock.nextCycle(0.02));

        assertTrue(repeat.isComplete());
        assertEquals(TaskOutcome.SUCCESS, repeat.getOutcome());
        assertEquals(3, admissionCalls.get());
        assertEquals(3, factoryCalls.get());
        CapturingDebugSink stopped = new CapturingDebugSink();
        repeat.debugDump(stopped, "auto.repeat");
        assertEquals("LIMIT", stringValue(stopped, "auto.repeat.stopReason"));
        repeat.update(clock.nextCycle(0.02));
        assertEquals(3, admissionCalls.get());
        assertEquals(3, factoryCalls.get());
    }

    @Test
    public void immediateChildrenHandoffOnlyOnLaterCycles() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger admissionCalls = new AtomicInteger();
        AtomicInteger factoryCalls = new AtomicInteger();
        List<Long> factoryCycles = new ArrayList<>();
        List<Task> children = Arrays.asList(
                Tasks.noop(),
                Tasks.waitForSeconds(0.0),
                ProbeTask.immediate("terminal-at-start", TaskOutcome.SUCCESS));
        Task repeat = Tasks.repeatWhileSuccessful(
                "immediate attempts",
                children.size(),
                observedClock -> {
                    admissionCalls.incrementAndGet();
                    return true;
                },
                () -> {
                    factoryCycles.add(clock.clock().cycle());
                    return children.get(factoryCalls.getAndIncrement());
                });

        repeat.start(clock.clock());
        repeat.update(clock.clock());
        repeat.update(clock.clock());
        assertEquals(1, factoryCalls.get());
        assertEquals(1, admissionCalls.get());
        assertFalse(repeat.isComplete());

        repeat.update(clock.nextCycle(0.0));
        repeat.update(clock.clock());
        repeat.update(clock.clock());
        assertEquals(2, factoryCalls.get());
        assertEquals(2, admissionCalls.get());
        assertFalse(repeat.isComplete());

        repeat.update(clock.nextCycle(0.0));

        assertTrue(repeat.isComplete());
        assertEquals(TaskOutcome.SUCCESS, repeat.getOutcome());
        assertEquals(3, factoryCalls.get());
        assertEquals(3, admissionCalls.get());
        assertEquals(Arrays.asList(1L, 2L, 3L), factoryCycles);
    }

    @Test
    public void childCompletingDuringUpdateCannotReleaseNextChildInSameCycle() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger factoryCalls = new AtomicInteger();
        ProbeTask first = new ProbeTask("first");
        first.updateHook = observedClock -> first.finish(TaskOutcome.SUCCESS);
        ProbeTask second = new ProbeTask("second");
        Task repeat = Tasks.repeatWhileSuccessful(
                "attempts",
                2,
                BooleanSource.constant(true),
                () -> factoryCalls.incrementAndGet() == 1 ? first : second);

        repeat.start(clock.clock());
        repeat.update(clock.clock());
        repeat.update(clock.clock());

        assertEquals(1, first.updateCount);
        assertEquals(1, factoryCalls.get());
        assertEquals(0, second.startCount);

        repeat.update(clock.nextCycle(0.02));
        repeat.update(clock.clock());

        assertEquals(2, factoryCalls.get());
        assertEquals(1, second.startCount);
        assertEquals(0, second.updateCount);

        repeat.update(clock.nextCycle(0.02));

        assertEquals(1, second.updateCount);
    }

    @Test
    public void rejectsNullSelfAndPreviouslyReturnedChildIdentityWithoutReplay() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger nullFactoryCalls = new AtomicInteger();
        Task nullChild = Tasks.repeatWhileSuccessful(
                "null attempt",
                2,
                BooleanSource.constant(true),
                () -> {
                    nullFactoryCalls.incrementAndGet();
                    return null;
                });

        IllegalStateException nullFailure =
                expectIllegalState(() -> nullChild.start(clock.clock()));
        assertContains(nullFailure, "null attempt", "returned null", "fresh Task");
        assertTrue(nullChild.isComplete());
        assertEquals(TaskOutcome.CANCELLED, nullChild.getOutcome());
        assertSame(nullFailure, expectRuntime(() -> nullChild.update(clock.clock())));
        assertEquals(1, nullFactoryCalls.get());

        AtomicReference<Task> selfReference = new AtomicReference<>();
        AtomicInteger selfFactoryCalls = new AtomicInteger();
        Task selfChild = Tasks.repeatWhileSuccessful(
                "self attempt",
                2,
                BooleanSource.constant(true),
                () -> {
                    selfFactoryCalls.incrementAndGet();
                    return selfReference.get();
                });
        selfReference.set(selfChild);

        IllegalStateException selfFailure =
                expectIllegalState(() -> selfChild.start(clock.clock()));
        assertContains(selfFailure, "self attempt", "own wrapper", "distinct fresh child");
        assertTrue(selfChild.isComplete());
        assertEquals(TaskOutcome.CANCELLED, selfChild.getOutcome());
        assertEquals(1, selfFactoryCalls.get());

        ProbeTask shared = ProbeTask.immediate("shared", TaskOutcome.SUCCESS);
        AtomicInteger duplicateFactoryCalls = new AtomicInteger();
        Task duplicate = Tasks.repeatWhileSuccessful(
                "duplicate attempt",
                2,
                BooleanSource.constant(true),
                () -> {
                    duplicateFactoryCalls.incrementAndGet();
                    return shared;
                });
        duplicate.start(clock.clock());

        IllegalStateException duplicateFailure =
                expectIllegalState(() -> duplicate.update(clock.nextCycle(0.02)));

        assertContains(
                duplicateFailure,
                "duplicate attempt",
                "reused",
                "iteration 1",
                "fresh Task");
        assertTrue(duplicate.isComplete());
        assertEquals(TaskOutcome.CANCELLED, duplicate.getOutcome());
        assertEquals(2, duplicateFactoryCalls.get());
        assertEquals(1, shared.startCount);
        assertEquals(0, shared.cancelCount);
    }

    @Test
    public void retainsEveryExactTerminalChildOutcome() {
        for (TaskOutcome terminal : Arrays.asList(
                TaskOutcome.SUCCESS,
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN)) {
            ManualLoopClock clock = new ManualLoopClock();
            AtomicInteger admissionCalls = new AtomicInteger();
            AtomicInteger factoryCalls = new AtomicInteger();
            ProbeTask child = ProbeTask.immediate("child-" + terminal, terminal);
            Task repeat = Tasks.repeatWhileSuccessful(
                    "attempts-" + terminal,
                    terminal == TaskOutcome.SUCCESS ? 1 : 5,
                    observedClock -> {
                        admissionCalls.incrementAndGet();
                        return true;
                    },
                    () -> {
                        factoryCalls.incrementAndGet();
                        return child;
                    });

            repeat.start(clock.clock());

            assertTrue(repeat.isComplete());
            assertEquals(terminal, repeat.getOutcome());
            assertEquals(1, admissionCalls.get());
            assertEquals(1, factoryCalls.get());
            assertEquals(1, child.startCount);
            assertEquals(0, child.cancelCount);
            CapturingDebugSink stopped = new CapturingDebugSink();
            repeat.debugDump(stopped, "auto.repeat");
            assertEquals(
                    terminal == TaskOutcome.SUCCESS ? "LIMIT" : "CHILD_OUTCOME",
                    stringValue(stopped, "auto.repeat.stopReason"));
            repeat.update(clock.nextCycle(0.02));
            assertEquals(1, admissionCalls.get());
            assertEquals(1, factoryCalls.get());
        }
    }

    @Test
    public void malformedTerminalChildOutcomeFailsClosedAndRethrowsWithoutReplay() {
        for (TaskOutcome malformed : Arrays.asList(null, TaskOutcome.NOT_DONE)) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask child = ProbeTask.immediate("malformed", malformed);
            AtomicInteger admissionCalls = new AtomicInteger();
            AtomicInteger factoryCalls = new AtomicInteger();
            Task repeat = Tasks.repeatWhileSuccessful(
                    "malformed attempt",
                    2,
                    observedClock -> {
                        admissionCalls.incrementAndGet();
                        return true;
                    },
                    () -> {
                        factoryCalls.incrementAndGet();
                        return child;
                    });

            IllegalStateException failure =
                    expectIllegalState(() -> repeat.start(clock.clock()));

            assertContains(
                    failure,
                    "malformed attempt",
                    "complete child reporting " + malformed,
                    "lifecycle contract");
            assertTrue(repeat.isComplete());
            assertEquals(TaskOutcome.CANCELLED, repeat.getOutcome());
            assertEquals(1, child.cancelCount);
            CapturingDebugSink stopped = new CapturingDebugSink();
            repeat.debugDump(stopped, "auto.repeat");
            assertEquals("FAILED", stringValue(stopped, "auto.repeat.stopReason"));
            assertSame(failure, expectRuntime(() -> repeat.update(clock.clock())));
            repeat.cancel();
            assertEquals(1, admissionCalls.get());
            assertEquals(1, factoryCalls.get());
            assertEquals(1, child.startCount);
            assertEquals(1, child.cancelCount);
        }
    }

    @Test
    public void cancellationIsActiveOnlyTerminalAndIdempotent() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicReference<Task> repeatReference = new AtomicReference<>();
        ProbeTask child = new ProbeTask("active child");
        child.cancelHook = () -> {
            assertTrue(repeatReference.get().isComplete());
            assertEquals(TaskOutcome.CANCELLED, repeatReference.get().getOutcome());
            repeatReference.get().cancel();
            repeatReference.get().update(clock.clock());
        };
        Task repeat = Tasks.repeatWhileSuccessful(
                "attempts",
                2,
                BooleanSource.constant(true),
                () -> child);
        repeatReference.set(repeat);

        repeat.cancel();
        repeat.cancel();
        assertFalse(repeat.isComplete());
        assertEquals(0, child.cancelCount);

        repeat.start(clock.clock());
        repeat.cancel();
        repeat.cancel();
        repeat.update(clock.clock());

        assertTrue(repeat.isComplete());
        assertEquals(TaskOutcome.CANCELLED, repeat.getOutcome());
        assertEquals(1, child.startCount);
        assertEquals(1, child.cancelCount);

        ProbeTask successful = ProbeTask.immediate("successful", TaskOutcome.SUCCESS);
        Task awaiting = Tasks.repeatWhileSuccessful(
                "awaiting",
                2,
                BooleanSource.constant(true),
                () -> successful);
        awaiting.start(clock.clock());
        assertFalse(awaiting.isComplete());
        awaiting.cancel();
        awaiting.cancel();
        assertTrue(awaiting.isComplete());
        assertEquals(TaskOutcome.CANCELLED, awaiting.getOutcome());
        assertEquals(0, successful.cancelCount);

        Task terminal = Tasks.repeatWhileSuccessful(
                "terminal",
                1,
                BooleanSource.constant(false),
                Tasks::noop);
        terminal.start(clock.clock());
        terminal.cancel();
        assertEquals(TaskOutcome.SUCCESS, terminal.getOutcome());
    }

    @Test
    public void nonterminalOrThrowingDirectCancellationRetainsFailureWithoutRetry() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask nonterminal = new ProbeTask("nonterminal");
        nonterminal.completeOnCancel = false;
        Task first = Tasks.repeatWhileSuccessful(
                "nonterminal cleanup",
                2,
                BooleanSource.constant(true),
                () -> nonterminal);
        first.start(clock.clock());

        IllegalStateException nonterminalFailure = expectIllegalState(first::cancel);

        assertContains(nonterminalFailure, "cancel() returned", "terminal", "isComplete()");
        assertTrue(first.isComplete());
        assertEquals(TaskOutcome.CANCELLED, first.getOutcome());
        first.cancel();
        assertEquals(1, nonterminal.cancelCount);
        assertSame(nonterminalFailure, expectRuntime(() -> first.update(clock.clock())));

        RuntimeException cleanupFailure = new RuntimeException("cleanup failed");
        ProbeTask throwing = new ProbeTask("throwing");
        throwing.cancelFailure = cleanupFailure;
        Task second = Tasks.repeatWhileSuccessful(
                "throwing cleanup",
                2,
                BooleanSource.constant(true),
                () -> throwing);
        second.start(clock.clock());

        RuntimeException thrown = expectRuntime(second::cancel);

        assertSame(cleanupFailure, thrown);
        assertTrue(second.isComplete());
        assertEquals(TaskOutcome.CANCELLED, second.getOutcome());
        second.cancel();
        assertEquals(1, throwing.cancelCount);
        assertSame(cleanupFailure, expectRuntime(() -> second.update(clock.clock())));
    }

    @Test
    public void admissionAndFactoryFailuresAreTerminalAndNeverRetried() {
        ManualLoopClock clock = new ManualLoopClock();
        RuntimeException admissionFailure = new RuntimeException("admission failed");
        AtomicInteger admissionCalls = new AtomicInteger();
        AtomicInteger factoryCalls = new AtomicInteger();
        Task admission = Tasks.repeatWhileSuccessful(
                "admission failure",
                2,
                observedClock -> {
                    admissionCalls.incrementAndGet();
                    throw admissionFailure;
                },
                () -> {
                    factoryCalls.incrementAndGet();
                    return Tasks.noop();
                });

        assertSame(admissionFailure, expectRuntime(() -> admission.start(clock.clock())));
        assertTrue(admission.isComplete());
        assertEquals(TaskOutcome.CANCELLED, admission.getOutcome());
        assertSame(admissionFailure, expectRuntime(() -> admission.update(clock.clock())));
        assertEquals(1, admissionCalls.get());
        assertEquals(0, factoryCalls.get());

        RuntimeException factoryFailure = new RuntimeException("factory failed");
        AtomicInteger secondAdmissionCalls = new AtomicInteger();
        AtomicInteger secondFactoryCalls = new AtomicInteger();
        Task factory = Tasks.repeatWhileSuccessful(
                "factory failure",
                2,
                observedClock -> {
                    secondAdmissionCalls.incrementAndGet();
                    return true;
                },
                () -> {
                    secondFactoryCalls.incrementAndGet();
                    throw factoryFailure;
                });

        assertSame(factoryFailure, expectRuntime(() -> factory.start(clock.clock())));
        assertTrue(factory.isComplete());
        assertEquals(TaskOutcome.CANCELLED, factory.getOutcome());
        assertSame(factoryFailure, expectRuntime(() -> factory.update(clock.clock())));
        assertEquals(1, secondAdmissionCalls.get());
        assertEquals(1, secondFactoryCalls.get());
    }

    @Test
    public void everyChildLifecycleFailureCancelsOnceAndRethrowsWithoutReplay() {
        for (FailurePoint point : FailurePoint.values()) {
            ManualLoopClock clock = new ManualLoopClock();
            RuntimeException primary = new RuntimeException(point + " failed");
            ProbeTask child = new ProbeTask("child-" + point);
            switch (point) {
                case START:
                    child.startFailure = primary;
                    break;
                case UPDATE:
                    child.updateFailure = primary;
                    break;
                case COMPLETE:
                    child.completeFailure = primary;
                    break;
                case OUTCOME:
                    child.startHook = observedClock -> child.finish(TaskOutcome.SUCCESS);
                    child.outcomeFailure = primary;
                    break;
                default:
                    throw new AssertionError(point);
            }
            AtomicInteger factoryCalls = new AtomicInteger();
            Task repeat = Tasks.repeatWhileSuccessful(
                    "failure-" + point,
                    2,
                    BooleanSource.constant(true),
                    () -> {
                        factoryCalls.incrementAndGet();
                        return child;
                    });

            RuntimeException thrown;
            if (point == FailurePoint.UPDATE) {
                repeat.start(clock.clock());
                thrown = expectRuntime(() -> repeat.update(clock.clock()));
            } else {
                thrown = expectRuntime(() -> repeat.start(clock.clock()));
            }

            assertSame(primary, thrown);
            assertTrue(repeat.isComplete());
            assertEquals(TaskOutcome.CANCELLED, repeat.getOutcome());
            assertEquals(1, factoryCalls.get());
            assertEquals(1, child.startCount);
            assertEquals(point == FailurePoint.UPDATE ? 1 : 0, child.updateCount);
            assertEquals(1, child.cancelCount);
            assertSame(primary, expectRuntime(() -> repeat.update(clock.nextCycle(0.02))));
            repeat.cancel();
            assertEquals(1, child.cancelCount);
            assertEquals(1, factoryCalls.get());
        }
    }

    @Test
    public void childFailurePreservesPrimaryAndSuppressesCleanupFailure() {
        ManualLoopClock clock = new ManualLoopClock();
        RuntimeException updateFailure = new RuntimeException("update failed");
        RuntimeException cleanupFailure = new RuntimeException("cleanup failed");
        ProbeTask child = new ProbeTask("child");
        child.updateFailure = updateFailure;
        child.cancelFailure = cleanupFailure;
        Task repeat = Tasks.repeatWhileSuccessful(
                "attempts",
                2,
                BooleanSource.constant(true),
                () -> child);
        repeat.start(clock.clock());

        RuntimeException thrown = expectRuntime(() -> repeat.update(clock.clock()));

        assertSame(updateFailure, thrown);
        assertEquals(1, thrown.getSuppressed().length);
        assertSame(cleanupFailure, thrown.getSuppressed()[0]);
        assertTrue(repeat.isComplete());
        assertEquals(TaskOutcome.CANCELLED, repeat.getOutcome());
        assertEquals(1, child.updateCount);
        assertEquals(1, child.cancelCount);
        assertSame(updateFailure, expectRuntime(() -> repeat.update(clock.nextCycle(0.02))));
        assertEquals(1, child.updateCount);
        assertEquals(1, child.cancelCount);
    }

    @Test
    public void reentrantCallbacksFailClosedOrCannotReleaseAChild() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicReference<Task> admissionReference = new AtomicReference<>();
        AtomicReference<RuntimeException> reentrantAdmissionFailure = new AtomicReference<>();
        AtomicInteger admissionFactoryCalls = new AtomicInteger();
        Task admission = Tasks.repeatWhileSuccessful(
                "reentrant admission",
                2,
                observedClock -> {
                    reentrantAdmissionFailure.set(
                            expectRuntime(() -> admissionReference.get().update(observedClock)));
                    return true;
                },
                () -> {
                    admissionFactoryCalls.incrementAndGet();
                    return Tasks.noop();
                });
        admissionReference.set(admission);

        RuntimeException admissionThrown =
                expectRuntime(() -> admission.start(clock.clock()));

        assertSame(reentrantAdmissionFailure.get(), admissionThrown);
        assertContains(admissionThrown, "reentrant lifecycle callback", "must not start or update");
        assertTrue(admission.isComplete());
        assertEquals(TaskOutcome.CANCELLED, admission.getOutcome());
        assertEquals(0, admissionFactoryCalls.get());

        AtomicReference<Task> factoryReference = new AtomicReference<>();
        ProbeTask neverStarted = new ProbeTask("never started");
        Task factory = Tasks.repeatWhileSuccessful(
                "factory cancellation",
                2,
                BooleanSource.constant(true),
                () -> {
                    factoryReference.get().cancel();
                    return neverStarted;
                });
        factoryReference.set(factory);

        factory.start(clock.clock());

        assertTrue(factory.isComplete());
        assertEquals(TaskOutcome.CANCELLED, factory.getOutcome());
        assertEquals(0, neverStarted.startCount);
        assertEquals(0, neverStarted.cancelCount);

        AtomicReference<Task> startReference = new AtomicReference<>();
        AtomicReference<RuntimeException> reentrantStartFailure = new AtomicReference<>();
        ProbeTask starting = new ProbeTask("starting");
        starting.startHook = observedClock -> reentrantStartFailure.set(
                expectRuntime(() -> startReference.get().start(observedClock)));
        Task childStart = Tasks.repeatWhileSuccessful(
                "reentrant child start",
                2,
                BooleanSource.constant(true),
                () -> starting);
        startReference.set(childStart);

        RuntimeException childStartThrown =
                expectRuntime(() -> childStart.start(clock.clock()));

        assertSame(reentrantStartFailure.get(), childStartThrown);
        assertContains(childStartThrown, "single-use", "fresh task");
        assertTrue(childStart.isComplete());
        assertEquals(TaskOutcome.CANCELLED, childStart.getOutcome());
        assertEquals(1, starting.startCount);
        assertEquals(1, starting.cancelCount);

        AtomicReference<Task> updateReference = new AtomicReference<>();
        AtomicReference<RuntimeException> reentrantUpdateFailure = new AtomicReference<>();
        ProbeTask updating = new ProbeTask("updating");
        updating.updateHook = observedClock -> reentrantUpdateFailure.set(
                expectRuntime(() -> updateReference.get().update(observedClock)));
        Task childUpdate = Tasks.repeatWhileSuccessful(
                "reentrant child update",
                2,
                BooleanSource.constant(true),
                () -> updating);
        updateReference.set(childUpdate);
        childUpdate.start(clock.clock());

        RuntimeException childUpdateThrown =
                expectRuntime(() -> childUpdate.update(clock.clock()));

        assertSame(reentrantUpdateFailure.get(), childUpdateThrown);
        assertContains(childUpdateThrown, "reentrant lifecycle callback");
        assertTrue(childUpdate.isComplete());
        assertEquals(TaskOutcome.CANCELLED, childUpdate.getOutcome());
        assertEquals(1, updating.updateCount);
        assertEquals(1, updating.cancelCount);
    }

    @Test
    public void nullClockAndSecondStartFailBeforePolicyOrChildReplay() {
        AtomicInteger admissionCalls = new AtomicInteger();
        AtomicInteger factoryCalls = new AtomicInteger();
        Task missingClock = Tasks.repeatWhileSuccessful(
                "missing clock",
                2,
                observedClock -> {
                    admissionCalls.incrementAndGet();
                    return true;
                },
                () -> {
                    factoryCalls.incrementAndGet();
                    return Tasks.noop();
                });

        IllegalArgumentException missing = expectIllegalArgument(() -> missingClock.start(null));

        assertContains(missing, "Tasks.repeatWhileSuccessful", "non-null LoopClock", "TaskRunner");
        assertTrue(missingClock.isComplete());
        assertEquals(TaskOutcome.CANCELLED, missingClock.getOutcome());
        assertEquals(0, admissionCalls.get());
        assertEquals(0, factoryCalls.get());
        assertContains(
                expectIllegalState(() -> missingClock.start(null)),
                "single-use",
                "fresh task");
        assertEquals(0, admissionCalls.get());
        assertEquals(0, factoryCalls.get());

        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask("child");
        Task active = Tasks.repeatWhileSuccessful(
                "active",
                2,
                BooleanSource.constant(true),
                () -> child);
        active.start(clock.clock());

        IllegalStateException secondStart =
                expectIllegalState(() -> active.start(clock.clock()));

        assertContains(secondStart, "active", "single-use", "fresh task");
        assertFalse(active.isComplete());
        assertEquals(1, child.startCount);
        assertEquals(0, child.cancelCount);
    }

    @Test
    public void debugDumpUsesOnlyCachedPolicyAndLifecycleState() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger admissionCalls = new AtomicInteger();
        AtomicInteger factoryCalls = new AtomicInteger();
        ProbeTask child = new ProbeTask("attempt child");
        Task repeat = Tasks.repeatWhileSuccessful(
                "adaptive attempts",
                3,
                observedClock -> {
                    admissionCalls.incrementAndGet();
                    return true;
                },
                () -> {
                    factoryCalls.incrementAndGet();
                    return child;
                });
        CapturingDebugSink before = new CapturingDebugSink();

        repeat.debugDump(before, null);

        assertEquals(0, admissionCalls.get());
        assertEquals(0, factoryCalls.get());
        assertEquals("adaptive attempts",
                before.values.get("repeatWhileSuccessful.name"));
        assertEquals("NOT_STARTED", stringValue(before, "repeatWhileSuccessful.phase"));
        assertEquals("NONE", stringValue(before, "repeatWhileSuccessful.stopReason"));
        assertFalse(before.values.containsKey("repeatWhileSuccessful.current.name"));

        repeat.start(clock.clock());
        int completeCallsBeforeDebug = child.completeCalls;
        int outcomeCallsBeforeDebug = child.outcomeCalls;
        CapturingDebugSink active = new CapturingDebugSink();

        repeat.debugDump(active, "auto.attempts");

        assertEquals(1, admissionCalls.get());
        assertEquals(1, factoryCalls.get());
        assertEquals(completeCallsBeforeDebug, child.completeCalls);
        assertEquals(outcomeCallsBeforeDebug, child.outcomeCalls);
        assertEquals(1, child.debugCalls);
        assertEquals("adaptive attempts", active.values.get("auto.attempts.name"));
        assertEquals(Double.valueOf(3.0), active.values.get("auto.attempts.maxIterations"));
        assertEquals("ACTIVE_CHILD", stringValue(active, "auto.attempts.phase"));
        assertEquals(Double.valueOf(1.0),
                active.values.get("auto.attempts.iterationsStarted"));
        assertEquals(Double.valueOf(0.0),
                active.values.get("auto.attempts.iterationsCompleted"));
        assertEquals(Double.valueOf(1.0),
                active.values.get("auto.attempts.admissionEvaluations"));
        assertEquals(Boolean.TRUE, active.values.get("auto.attempts.currentChildPresent"));
        assertEquals(TaskOutcome.NOT_DONE, active.values.get("auto.attempts.currentChildOutcome"));
        assertEquals(TaskOutcome.NOT_DONE, active.values.get("auto.attempts.lastChildOutcome"));
        assertEquals(TaskOutcome.NOT_DONE, active.values.get("auto.attempts.outcome"));
        assertEquals("NONE", stringValue(active, "auto.attempts.stopReason"));
        assertEquals("attempt child", active.values.get("auto.attempts.current.name"));

        repeat.cancel();
        CapturingDebugSink cancelled = new CapturingDebugSink();
        repeat.debugDump(cancelled, "auto.attempts");
        assertEquals("CANCELLED", stringValue(cancelled, "auto.attempts.phase"));
        assertEquals("CANCELLED", stringValue(cancelled, "auto.attempts.stopReason"));
        assertEquals(TaskOutcome.CANCELLED, cancelled.values.get("auto.attempts.outcome"));
    }

    @Test
    public void outerTimeoutCancelsOnlyCurrentIterationAndCannotAdmitAnother() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger admissionCalls = new AtomicInteger();
        AtomicInteger factoryCalls = new AtomicInteger();
        ProbeTask child = new ProbeTask("current attempt");
        Task repeat = Tasks.repeatWhileSuccessful(
                "attempts",
                5,
                observedClock -> {
                    admissionCalls.incrementAndGet();
                    return true;
                },
                () -> {
                    factoryCalls.incrementAndGet();
                    return child;
                });
        Task bounded = Tasks.withTimeout(repeat, 0.5);

        bounded.start(clock.clock());
        bounded.update(clock.clock());
        assertEquals(1, child.updateCount);

        bounded.update(clock.nextCycle(0.5));

        assertTrue(bounded.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, bounded.getOutcome());
        assertTrue(repeat.isComplete());
        assertEquals(TaskOutcome.CANCELLED, repeat.getOutcome());
        assertEquals(1, admissionCalls.get());
        assertEquals(1, factoryCalls.get());
        assertEquals(1, child.startCount);
        assertEquals(1, child.updateCount);
        assertEquals(1, child.cancelCount);

        bounded.update(clock.nextCycle(0.5));
        repeat.update(clock.clock());
        assertEquals(1, admissionCalls.get());
        assertEquals(1, factoryCalls.get());
        assertEquals(1, child.cancelCount);
    }

    private enum FailurePoint {
        START,
        UPDATE,
        COMPLETE,
        OUTCOME
    }

    /** Deliberately transparent Task probe used to expose exact wrapper lifecycle behavior. */
    private static final class ProbeTask implements Task {
        private final String name;
        private int startCount;
        private int updateCount;
        private int cancelCount;
        private int completeCalls;
        private int outcomeCalls;
        private int debugCalls;
        private boolean started;
        private boolean complete;
        private boolean completeOnCancel = true;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;
        private TaskOutcome cancelOutcome = TaskOutcome.CANCELLED;
        private LoopClock startClock;
        private LoopClock updateClock;
        private ClockHook startHook;
        private ClockHook updateHook;
        private Runnable cancelHook;
        private RuntimeException startFailure;
        private RuntimeException updateFailure;
        private RuntimeException cancelFailure;
        private RuntimeException completeFailure;
        private RuntimeException outcomeFailure;

        private ProbeTask(String name) {
            this.name = name;
        }

        private static ProbeTask immediate(String name, TaskOutcome terminalOutcome) {
            ProbeTask task = new ProbeTask(name);
            task.startHook = observedClock -> task.finish(terminalOutcome);
            return task;
        }

        @Override
        public void start(LoopClock clock) {
            startCount++;
            started = true;
            startClock = clock;
            if (startHook != null) {
                startHook.run(clock);
            }
            if (startFailure != null) {
                throw startFailure;
            }
        }

        @Override
        public void update(LoopClock clock) {
            updateCount++;
            updateClock = clock;
            if (updateHook != null) {
                updateHook.run(clock);
            }
            if (updateFailure != null) {
                throw updateFailure;
            }
        }

        @Override
        public void cancel() {
            cancelCount++;
            if (started && !complete && completeOnCancel) {
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
            completeCalls++;
            if (completeFailure != null) {
                throw completeFailure;
            }
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            outcomeCalls++;
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
            debugCalls++;
            if (dbg == null) {
                return;
            }
            String p = (prefix == null || prefix.isEmpty()) ? "probe" : prefix;
            dbg.addData(p + ".name", name)
                    .addData(p + ".complete", complete)
                    .addData(p + ".outcome", outcome);
        }

        private void finish(TaskOutcome terminalOutcome) {
            complete = true;
            outcome = terminalOutcome;
        }
    }

    private interface ClockHook {
        void run(LoopClock clock);
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

    private static String stringValue(CapturingDebugSink sink, String key) {
        return String.valueOf(sink.values.get(key));
    }

    private static void assertContains(Throwable failure, String... fragments) {
        String message = failure.getMessage();
        for (String fragment : fragments) {
            assertTrue(
                    "Expected '" + fragment + "' in: " + message,
                    message != null && message.contains(fragment));
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

    private static NullPointerException expectNullPointer(Runnable action) {
        try {
            action.run();
            fail("Expected NullPointerException");
            return null;
        } catch (NullPointerException expected) {
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
