package edu.ftcsushi.fw.task;

import org.junit.Test;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Regression coverage for transactional OutputTaskRunner value observations. */
public final class OutputTaskRunnerTransactionalTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void failedOutputGetterCanRetryAndOnlySuccessCommitsTheCycle() {
        ManualLoopClock time = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(-0.25);
        GetterTask task = new GetterTask(0.75);
        RuntimeException failure = new RuntimeException("output read failed");
        task.nextOutputFailure = failure;
        runner.enqueue(task);
        runner.update(time.clock());

        assertSame(failure, expectRuntime(() -> runner.output(time.clock())));
        assertEquals(0.75, runner.output(time.clock()), EPSILON);
        assertEquals(0.75, runner.output(time.clock()), EPSILON);
        assertEquals(2, task.outputCalls);
        assertTrue(runner.hasActiveTask());
    }

    @Test
    public void recursiveOutputReadFailsActionablyAndCanRecoverSameCycle() {
        ManualLoopClock time = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(0.0);
        GetterTask task = new GetterTask(0.6);
        task.outputCallback = () -> runner.output(time.clock());
        runner.enqueue(task);
        runner.update(time.clock());

        RuntimeException observed = expectRuntime(() -> runner.output(time.clock()));
        assertTrue(observed instanceof IllegalStateException);
        assertTrue(observed.getMessage().contains("reentrantly"));
        assertTrue(runner.hasActiveTask());

        task.outputCallback = null;
        assertEquals(0.6, runner.output(time.clock()), EPSILON);
    }

    @Test
    public void outputGetterCannotRecursivelyObserveTheActiveView() {
        ManualLoopClock time = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(0.0);
        GetterTask task = new GetterTask(0.65);
        BooleanSource active = runner.activeSource();
        task.outputCallback = () -> active.getAsBoolean(time.clock());
        runner.enqueue(task);
        runner.update(time.clock());

        RuntimeException observed = expectRuntime(() -> runner.output(time.clock()));
        assertTrue(observed instanceof IllegalStateException);
        assertTrue(observed.getMessage().contains("reentrantly"));
        assertTrue(runner.hasActiveTask());

        task.outputCallback = null;
        assertEquals(0.65, runner.output(time.clock()), EPSILON);
    }

    @Test
    public void cancellationDuringOutputReadCannotOverwriteIdleInvalidation() {
        ManualLoopClock time = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(-0.4);
        GetterTask task = new GetterTask(0.9);
        task.outputCallback = runner::cancelAndClear;
        runner.enqueue(task);
        runner.update(time.clock());

        RuntimeException observed = expectRuntime(() -> runner.output(time.clock()));
        assertTrue(observed instanceof IllegalStateException);
        assertTrue(observed.getMessage().contains("changed while it was being observed"));
        assertTrue(runner.isIdle());
        assertEquals(-0.4, runner.output(time.clock()), EPSILON);
    }

    @Test
    public void cancellationDuringActiveReadCannotPublishDetachedTaskState() {
        ManualLoopClock time = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(0.0);
        GetterTask task = new GetterTask(0.7);
        task.updateCallback = runner::cancelAndClear;
        runner.enqueue(task);
        BooleanSource active = runner.activeSource();

        RuntimeException observed = expectRuntime(() -> active.getAsBoolean(time.clock()));
        assertTrue(observed instanceof IllegalStateException);
        assertTrue(observed.getMessage().contains("changed while it was being observed"));
        assertTrue(runner.isIdle());
        assertFalse(active.getAsBoolean(time.clock()));
    }

    @Test
    public void cancellationDuringScalarSourceUpdateCannotBecomeAQuietIdleSuccess() {
        ManualLoopClock time = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(-0.3);
        GetterTask task = new GetterTask(0.7);
        task.updateCallback = runner::cancelAndClear;
        runner.enqueue(task);

        RuntimeException observed = expectRuntime(() -> runner.getAsDouble(time.clock()));
        assertTrue(observed instanceof IllegalStateException);
        assertTrue(observed.getMessage().contains("changed while it was being observed"));
        assertTrue(runner.isIdle());
        assertEquals(-0.3, runner.getAsDouble(time.clock()), EPSILON);
    }

    @Test
    public void activeSourceRecursionKeepsTaskLifecycleFailStopSemantics() {
        ManualLoopClock time = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(0.0);
        GetterTask task = new GetterTask(0.5);
        BooleanSource active = runner.activeSource();
        task.updateCallback = () -> active.getAsBoolean(time.clock());
        runner.enqueue(task);

        RuntimeException observed = expectRuntime(() -> active.getAsBoolean(time.clock()));
        assertTrue(observed instanceof IllegalStateException);
        assertTrue(observed.getMessage().contains("reentrantly"));
        assertEquals(1, task.cancelCalls);
        assertTrue(runner.isIdle());
        assertFalse(active.getAsBoolean(time.clock()));
    }

    @Test
    public void activeViewCannotRecursivelyObserveOutputDuringTaskUpdate() {
        ManualLoopClock time = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(-0.1);
        GetterTask task = new GetterTask(0.5);
        BooleanSource active = runner.activeSource();
        task.updateCallback = () -> runner.output(time.clock());
        runner.enqueue(task);

        RuntimeException observed = expectRuntime(() -> active.getAsBoolean(time.clock()));
        assertTrue(observed instanceof IllegalStateException);
        assertTrue(observed.getMessage().contains("reentrantly"));
        assertEquals(1, task.cancelCalls);
        assertTrue(runner.isIdle());
        assertFalse(active.getAsBoolean(time.clock()));
        assertEquals(-0.1, runner.output(time.clock()), EPSILON);
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

    private static final class GetterTask implements OutputTask {
        private final double output;
        private boolean started;
        private boolean complete;
        private RuntimeException nextOutputFailure;
        private Runnable outputCallback;
        private Runnable updateCallback;
        private int outputCalls;
        private int cancelCalls;

        private GetterTask(double output) {
            this.output = output;
        }

        @Override
        public void start(LoopClock clock) {
            started = true;
        }

        @Override
        public void update(LoopClock clock) {
            if (updateCallback != null) {
                updateCallback.run();
            }
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            cancelCalls++;
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }

        @Override
        public double getOutput() {
            outputCalls++;
            if (nextOutputFailure != null) {
                RuntimeException failure = nextOutputFailure;
                nextOutputFailure = null;
                throw failure;
            }
            if (outputCallback != null) {
                outputCallback.run();
            }
            return output;
        }
    }
}
