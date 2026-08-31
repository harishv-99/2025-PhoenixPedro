package edu.ftcsushi.fw.task;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the deferred child graph exposed by {@link Tasks#buildAtStart}. */
public final class BuildAtStartTaskTest {

    @Test
    public void validatesFactoryInputsAndKeepsImplementationInternal() throws Exception {
        for (String invalidName : Arrays.asList(null, "", " \t ")) {
            IllegalArgumentException failure = expectIllegalArgument(
                    () -> Tasks.buildAtStart(invalidName, Tasks::noop));
            assertContains(failure, "Tasks.buildAtStart", "debugName", "nonblank");
        }

        NullPointerException missingFactory = expectNullPointer(
                () -> Tasks.buildAtStart("selected Auto", null));
        assertContains(missingFactory, "Tasks.buildAtStart", "non-null", "factory");

        Method factory = Tasks.class.getMethod(
                "buildAtStart",
                String.class,
                Supplier.class);
        assertTrue(Modifier.isPublic(factory.getModifiers()));
        assertTrue(Modifier.isStatic(factory.getModifiers()));
        assertEquals(Task.class, factory.getReturnType());

        assertTrue(Modifier.isFinal(BuildAtStartTask.class.getModifiers()));
        assertFalse(Modifier.isPublic(BuildAtStartTask.class.getModifiers()));
        Constructor<?>[] constructors = BuildAtStartTask.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertFalse(Modifier.isPublic(constructors[0].getModifiers()));
        assertEquals(
                BuildAtStartTask.class,
                Tasks.buildAtStart("selected Auto", Tasks::noop).getClass());
    }

    @Test
    public void factoryStaysDeferredAcrossInspectionDebugAndPrestartCancellation() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger factoryCalls = new AtomicInteger();
        ProbeTask child = new ProbeTask("chosenRoutine");
        Task deferred = Tasks.buildAtStart("selected Auto", () -> {
            factoryCalls.incrementAndGet();
            return child;
        });
        CapturingDebugSink debug = new CapturingDebugSink();

        assertEquals("selected Auto", deferred.getDebugName());
        assertFalse(deferred.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, deferred.getOutcome());
        deferred.debugDump(debug, "auto.deferred");
        deferred.cancel();
        deferred.cancel();
        assertContains(
                expectIllegalState(() -> deferred.update(clock.clock())),
                "Tasks.buildAtStart",
                "before start",
                "TaskRunner");

        assertEquals(0, factoryCalls.get());
        assertEquals(Boolean.FALSE, debug.values.get("auto.deferred.childCreated"));
        assertFalse(debug.values.containsKey("auto.deferred.child.name"));

        deferred.start(clock.clock());

        assertEquals(1, factoryCalls.get());
        assertEquals(1, child.startCount);
        IllegalStateException secondStart =
                expectIllegalState(() -> deferred.start(clock.clock()));
        assertContains(secondStart, "selected Auto", "single-use", "fresh task");
        assertEquals(1, factoryCalls.get());
        assertEquals(1, child.startCount);
    }

    @Test
    public void forwardsExactChildLifecycleOutcomeAndNestedDebug() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask("visionChosenRoutine");
        Task deferred = Tasks.buildAtStart("selected Auto", () -> child);

        deferred.start(clock.clock());
        deferred.update(clock.clock());

        assertSame(clock.clock(), child.startClock);
        assertSame(clock.clock(), child.updateClock);
        assertEquals(1, child.startCount);
        assertEquals(1, child.updateCount);
        assertFalse(deferred.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, deferred.getOutcome());

        child.finish(TaskOutcome.TIMEOUT);
        assertTrue(deferred.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, deferred.getOutcome());

        deferred.cancel();
        deferred.update(clock.nextCycle(0.02));
        assertEquals(0, child.cancelCount);
        assertEquals(1, child.updateCount);

        CapturingDebugSink debug = new CapturingDebugSink();
        deferred.debugDump(debug, "auto.deferred");
        assertEquals("selected Auto", debug.values.get("auto.deferred.name"));
        assertEquals(Boolean.TRUE, debug.values.get("auto.deferred.complete"));
        assertEquals("visionChosenRoutine", debug.values.get("auto.deferred.child.name"));
        assertEquals(TaskOutcome.TIMEOUT,
                debug.values.get("auto.deferred.child.outcome"));
    }

    @Test
    public void rejectsNullAndSelfChildrenOnceAndStaysTerminal() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger nullFactoryCalls = new AtomicInteger();
        Task nullChild = Tasks.buildAtStart("null child", () -> {
            nullFactoryCalls.incrementAndGet();
            return null;
        });

        IllegalStateException nullFailure =
                expectIllegalState(() -> nullChild.start(clock.clock()));
        assertContains(nullFailure, "null child", "returned null", "fresh Task");
        assertTrue(nullChild.isComplete());
        assertEquals(TaskOutcome.CANCELLED, nullChild.getOutcome());
        nullChild.cancel();
        assertContains(
                expectIllegalState(() -> nullChild.start(clock.clock())),
                "single-use",
                "fresh task");
        assertEquals(1, nullFactoryCalls.get());

        AtomicInteger selfFactoryCalls = new AtomicInteger();
        AtomicReference<Task> selfReference = new AtomicReference<>();
        Task selfChild = Tasks.buildAtStart("self child", () -> {
            selfFactoryCalls.incrementAndGet();
            return selfReference.get();
        });
        selfReference.set(selfChild);

        IllegalStateException selfFailure =
                expectIllegalState(() -> selfChild.start(clock.clock()));
        assertContains(selfFailure, "self child", "own wrapper", "distinct fresh child");
        assertTrue(selfChild.isComplete());
        assertEquals(TaskOutcome.CANCELLED, selfChild.getOutcome());
        assertEquals(1, selfFactoryCalls.get());
    }

    @Test
    public void factoryFailureIsTerminalAndCannotBeRetried() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger factoryCalls = new AtomicInteger();
        RuntimeException factoryFailure = new RuntimeException("selection unavailable");
        Task deferred = Tasks.buildAtStart("selected Auto", () -> {
            factoryCalls.incrementAndGet();
            throw factoryFailure;
        });

        RuntimeException thrown = expectRuntime(() -> deferred.start(clock.clock()));

        assertSame(factoryFailure, thrown);
        assertTrue(deferred.isComplete());
        assertEquals(TaskOutcome.CANCELLED, deferred.getOutcome());
        deferred.cancel();
        assertContains(
                expectIllegalState(() -> deferred.start(clock.clock())),
                "single-use",
                "fresh task");
        assertEquals(1, factoryCalls.get());
    }

    @Test
    public void childStartFailureCancelsRetainedChildAndSuppressesCleanupFailure() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger factoryCalls = new AtomicInteger();
        RuntimeException startFailure = new RuntimeException("child start failed");
        RuntimeException cleanupFailure = new RuntimeException("child cleanup failed");
        ProbeTask child = new ProbeTask("partialRoutine");
        child.startFailure = startFailure;
        child.cancelFailure = cleanupFailure;
        Task deferred = Tasks.buildAtStart("selected Auto", () -> {
            factoryCalls.incrementAndGet();
            return child;
        });

        RuntimeException thrown = expectRuntime(() -> deferred.start(clock.clock()));

        assertSame(startFailure, thrown);
        assertEquals(1, thrown.getSuppressed().length);
        assertSame(cleanupFailure, thrown.getSuppressed()[0]);
        assertEquals(1, child.startCount);
        assertEquals(1, child.cancelCount);
        assertTrue(deferred.isComplete());
        assertEquals(TaskOutcome.CANCELLED, deferred.getOutcome());

        deferred.cancel();
        deferred.update(clock.clock());
        assertEquals(1, child.cancelCount);
        assertEquals(0, child.updateCount);
        assertContains(
                expectIllegalState(() -> deferred.start(clock.clock())),
                "single-use",
                "fresh task");
        assertEquals(1, factoryCalls.get());

        CapturingDebugSink debug = new CapturingDebugSink();
        deferred.debugDump(debug, "auto.deferred");
        assertEquals("partialRoutine", debug.values.get("auto.deferred.child.name"));
        assertEquals(Boolean.TRUE,
                debug.values.get("auto.deferred.childCancellationAttempted"));
    }

    @Test
    public void activeCancellationIsTerminalBeforeOneChildCleanup() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicReference<Task> wrapper = new AtomicReference<>();
        ProbeTask child = new ProbeTask("activeRoutine");
        child.cancelHook = () -> {
            assertTrue(wrapper.get().isComplete());
            assertEquals(TaskOutcome.CANCELLED, wrapper.get().getOutcome());
            wrapper.get().cancel();
            wrapper.get().update(clock.clock());
        };
        Task deferred = Tasks.buildAtStart("selected Auto", () -> child);
        wrapper.set(deferred);

        deferred.start(clock.clock());
        deferred.cancel();
        deferred.cancel();
        deferred.update(clock.clock());

        assertTrue(deferred.isComplete());
        assertEquals(TaskOutcome.CANCELLED, deferred.getOutcome());
        assertEquals(1, child.cancelCount);
        assertEquals(0, child.updateCount);
    }

    @Test
    public void reentrantFactoryCancellationRetainsButNeverStartsReturnedChild() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicReference<Task> wrapper = new AtomicReference<>();
        ProbeTask child = new ProbeTask("neverStartedRoutine");
        Task deferred = Tasks.buildAtStart("selected Auto", () -> {
            wrapper.get().cancel();
            return child;
        });
        wrapper.set(deferred);

        deferred.start(clock.clock());

        assertTrue(deferred.isComplete());
        assertEquals(TaskOutcome.CANCELLED, deferred.getOutcome());
        assertEquals(0, child.startCount);
        assertEquals(0, child.cancelCount);

        CapturingDebugSink debug = new CapturingDebugSink();
        deferred.debugDump(debug, "auto.deferred");
        assertEquals(Boolean.TRUE, debug.values.get("auto.deferred.childCreated"));
        assertEquals(Boolean.FALSE,
                debug.values.get("auto.deferred.childStartAttempted"));
        assertEquals("neverStartedRoutine", debug.values.get("auto.deferred.child.name"));
    }

    @Test
    public void reentrantChildStartCancellationUsesRetainedChildExactlyOnce() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicReference<Task> wrapper = new AtomicReference<>();
        ProbeTask child = new ProbeTask("reentrantRoutine");
        child.startHook = () -> wrapper.get().cancel();
        Task deferred = Tasks.buildAtStart("selected Auto", () -> child);
        wrapper.set(deferred);

        deferred.start(clock.clock());

        assertTrue(deferred.isComplete());
        assertEquals(TaskOutcome.CANCELLED, deferred.getOutcome());
        assertEquals(1, child.startCount);
        assertEquals(1, child.cancelCount);
        deferred.cancel();
        assertEquals(1, child.cancelCount);
    }

    private static final class ProbeTask implements Task {
        private final String name;
        private int startCount;
        private int updateCount;
        private int cancelCount;
        private boolean started;
        private boolean complete;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;
        private LoopClock startClock;
        private LoopClock updateClock;
        private Runnable startHook;
        private Runnable cancelHook;
        private RuntimeException startFailure;
        private RuntimeException cancelFailure;

        private ProbeTask(String name) {
            this.name = name;
        }

        @Override
        public void start(LoopClock clock) {
            startCount++;
            started = true;
            startClock = clock;
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
            updateClock = clock;
        }

        @Override
        public void cancel() {
            cancelCount++;
            if (!started || complete) {
                return;
            }
            complete = true;
            outcome = TaskOutcome.CANCELLED;
            if (cancelHook != null) {
                cancelHook.run();
            }
            if (cancelFailure != null) {
                throw cancelFailure;
            }
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
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
