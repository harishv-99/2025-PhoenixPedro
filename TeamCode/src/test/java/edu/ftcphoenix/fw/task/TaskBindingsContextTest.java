package edu.ftcphoenix.fw.task;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Verifies the one TaskBindings factory against root and contextual registrars. */
public final class TaskBindingsContextTest {

    @Test
    public void rootRegistrarInvokesFreshTaskFactoryForEachAcceptedRise() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        TaskRunner runner = new TaskRunner();
        TaskBindings taskBindings = TaskBindings.of(bindings, runner);
        boolean[] button = {false};
        int[] factoryCalls = {0};

        taskBindings.onRise(BooleanSource.of(() -> button[0]), () -> {
            factoryCalls[0]++;
            return new HoldingTask();
        });

        update(bindings, manualClock);
        button[0] = true;
        update(bindings, manualClock);
        assertEquals(1, factoryCalls[0]);
        assertEquals(1, runner.queuedCount());

        button[0] = false;
        update(bindings, manualClock);
        button[0] = true;
        update(bindings, manualClock);
        assertEquals(2, factoryCalls[0]);
        assertEquals(2, runner.queuedCount());
    }

    @Test
    public void contextRegistrarSuppressesHeldActivationAndDoesNotCancelAcceptedWork() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        TaskRunner runner = new TaskRunner();
        boolean[] active = {false};
        boolean[] button = {true};
        int[] factoryCalls = {0};

        Bindings.ControlContext context = bindings.contextWhen(
                BooleanSource.of(() -> active[0]),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL);
        TaskBindings taskBindings = TaskBindings.of(context, runner);
        taskBindings.onRise(BooleanSource.of(() -> button[0]), () -> {
            factoryCalls[0]++;
            return new HoldingTask();
        });

        update(bindings, manualClock);
        active[0] = true;
        update(bindings, manualClock);
        update(bindings, manualClock);
        assertEquals(0, factoryCalls[0]);
        assertTrue(runner.isIdle());

        button[0] = false;
        update(bindings, manualClock);
        button[0] = true;
        update(bindings, manualClock);
        assertEquals(1, factoryCalls[0]);
        assertEquals(1, runner.queuedCount());

        manualClock.nextCycle(0.02);
        runner.update(manualClock.clock());
        Task accepted = runner.currentTaskOrNull();
        assertTrue(runner.hasActiveTask());

        active[0] = false;
        bindings.update(manualClock.clock());
        assertSame(accepted, runner.currentTaskOrNull());
        assertTrue(runner.hasActiveTask());
        assertFalse(runner.isIdle());
    }

    @Test
    public void acceptCurrentContextDoesNotManufactureTaskOnHeldActivation() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        TaskRunner runner = new TaskRunner();
        boolean[] active = {true};
        boolean[] button = {true};
        int[] factoryCalls = {0};

        Bindings.ControlContext context = bindings.contextWhen(
                BooleanSource.of(() -> active[0]),
                Bindings.ActivationPolicy.ACCEPT_CURRENT);
        TaskBindings.of(context, runner).onRise(BooleanSource.of(() -> button[0]), () -> {
            factoryCalls[0]++;
            return new HoldingTask();
        });

        update(bindings, manualClock);
        update(bindings, manualClock);
        assertEquals(0, factoryCalls[0]);

        button[0] = false;
        update(bindings, manualClock);
        button[0] = true;
        update(bindings, manualClock);
        assertEquals(1, factoryCalls[0]);
    }

    @Test
    public void edgeAndChangeKindsEnqueueTasksInBindingDeclarationOrder() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        TaskRunner runner = new TaskRunner();
        TaskBindings taskBindings = TaskBindings.of(bindings, runner);
        boolean[] button = {false};
        BooleanSource signal = BooleanSource.of(() -> button[0]);
        List<String> starts = new ArrayList<>();

        taskBindings.toggleOnRise(
                signal,
                () -> Tasks.runOnce(() -> starts.add("toggle-on")),
                () -> Tasks.runOnce(() -> starts.add("toggle-off")));
        taskBindings.mirrorOnChange(signal,
                high -> Tasks.runOnce(() -> starts.add("mirror-" + high)));
        taskBindings.onRise(signal, () -> Tasks.runOnce(() -> starts.add("rise")));

        update(bindings, manualClock);
        runner.update(manualClock.clock());
        assertEquals(Arrays.asList("mirror-false"), starts);

        starts.clear();
        button[0] = true;
        update(bindings, manualClock);
        runner.update(manualClock.clock());

        assertEquals(Arrays.asList("toggle-on", "mirror-true", "rise"), starts);
        assertTrue(runner.isIdle());
    }

    private static void update(Bindings bindings, ManualLoopClock manualClock) {
        manualClock.nextCycle(0.02);
        bindings.update(manualClock.clock());
    }

    private static final class HoldingTask implements Task {
        private boolean started;
        private boolean cancelled;

        @Override
        public void start(LoopClock clock) {
            started = true;
            // Started by the runner; this task deliberately remains active.
        }

        @Override
        public void update(LoopClock clock) {
            // Remain active until an explicit owner cancels this test task.
        }

        @Override
        public void cancel() {
            if (!started || cancelled) {
                return;
            }
            cancelled = true;
        }

        @Override
        public boolean isComplete() {
            return cancelled;
        }

        @Override
        public TaskOutcome getOutcome() {
            return cancelled ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }
    }
}
