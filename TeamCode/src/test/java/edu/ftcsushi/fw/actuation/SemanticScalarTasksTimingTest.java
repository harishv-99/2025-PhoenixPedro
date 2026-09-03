package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies semantic timed ownership, occurrence identity, and Task lifecycle boundaries. */
public final class SemanticScalarTasksTimingTest {

    private enum Mode {
        IDLE,
        ACTIVE,
        OVERRIDE
    }

    @Test
    public void positiveTimedRequestStartsAtItsOwnBoundaryAndKeepsOneIdentityWhileCurrent() {
        ManualLoopClock time = new ManualLoopClock();
        time.nextCycle(1.0);
        SemanticScalarCommand<Mode> command = command();
        Task timed = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .forSeconds(0.10)
                .then(Mode.IDLE)
                .build();
        TaskRunner runner = new TaskRunner();
        runner.enqueue(timed);

        runner.update(time.clock());
        SemanticScalarCommand.Request<Mode> active = command.request();

        assertFalse(timed.isComplete());
        assertEquals(Mode.ACTIVE, active.semantic());
        assertEquals(0.75, active.commandTarget(), 0.0);

        runner.update(time.clock());
        assertSame(active, command.request());
        time.nextCycle(0.09);
        runner.update(time.clock());
        assertFalse(timed.isComplete());
        assertSame(active, command.request());

        time.nextCycle(0.02);
        runner.update(time.clock());

        assertTrue(timed.isComplete());
        assertEquals(TaskOutcome.SUCCESS, timed.getOutcome());
        assertEquals(Mode.IDLE, command.request().semantic());
        assertNotSame(active, command.request());
    }

    @Test
    public void timedRequestPublishesOneFreshOccurrenceOnlyWhenReclaimingAfterSupersession() {
        int[] mappings = {0};
        SemanticScalarCommand<Mode> command = SemanticScalarCommand.create(Mode.IDLE, mode -> {
            mappings[0]++;
            return targetFor(mode);
        });
        Task timed = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .forSeconds(1.0)
                .then(Mode.IDLE)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        timed.start(time.clock());
        SemanticScalarCommand.Request<Mode> firstActive = command.request();
        timed.update(time.clock());
        assertSame(firstActive, command.request());

        SemanticScalarCommand.Request<Mode> override = command.set(Mode.OVERRIDE);
        timed.update(time.clock());
        SemanticScalarCommand.Request<Mode> reclaimed = command.request();

        assertNotSame(firstActive, override);
        assertNotSame(override, reclaimed);
        assertNotSame("A stale request occurrence must never be resurrected",
                firstActive, reclaimed);
        assertEquals(Mode.ACTIVE, reclaimed.semantic());

        timed.update(time.clock());
        assertSame(reclaimed, command.request());
        assertEquals("Reassertion publishes prepared values without remapping", 4, mappings[0]);

        timed.cancel();
        assertEquals(TaskOutcome.CANCELLED, timed.getOutcome());
        assertEquals(Mode.IDLE, command.request().semantic());
        assertNotSame(reclaimed, command.request());
        assertEquals(4, mappings[0]);
    }

    @Test
    public void leaveThereDoesNotOverwriteTheLatestRequestOnActiveCancellation() {
        SemanticScalarCommand<Mode> command = command();
        Task timed = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .forSeconds(1.0)
                .leaveThere()
                .build();
        ManualLoopClock time = new ManualLoopClock();

        timed.cancel();
        SemanticScalarCommand.Request<Mode> initial = command.request();
        assertEquals(TaskOutcome.NOT_DONE, timed.getOutcome());
        assertSame(initial, command.request());

        timed.start(time.clock());
        SemanticScalarCommand.Request<Mode> override = command.set(Mode.OVERRIDE);
        timed.cancel();
        timed.cancel();

        assertEquals(TaskOutcome.CANCELLED, timed.getOutcome());
        assertSame(override, command.request());
    }

    @Test
    public void thenPublishesOneFreshEndingOccurrenceOnActiveCancellation() {
        SemanticScalarCommand<Mode> command = command();
        Task timed = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .forSeconds(1.0)
                .then(Mode.IDLE)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        timed.start(time.clock());
        SemanticScalarCommand.Request<Mode> active = command.request();
        timed.cancel();
        SemanticScalarCommand.Request<Mode> ending = command.request();

        assertEquals(TaskOutcome.CANCELLED, timed.getOutcome());
        assertEquals(Mode.IDLE, ending.semantic());
        assertNotSame(active, ending);

        timed.cancel();
        timed.update(time.clock());
        assertSame(ending, command.request());
    }

    @Test
    public void zeroDurationFollowsNumericImmediateFinishBehavior() {
        SemanticScalarCommand<Mode> command = command();
        SemanticScalarCommand.Request<Mode> initial = command.request();
        Task then = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .forSeconds(0.0)
                .then(Mode.OVERRIDE)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        then.start(time.clock());

        assertTrue(then.isComplete());
        assertEquals(TaskOutcome.SUCCESS, then.getOutcome());
        assertEquals(Mode.OVERRIDE, command.request().semantic());
        assertNotSame(initial, command.request());

        Task leave = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .forSeconds(0.0)
                .leaveThere()
                .build();
        leave.start(time.clock());

        assertTrue(leave.isComplete());
        assertEquals(TaskOutcome.SUCCESS, leave.getOutcome());
        assertEquals(Mode.ACTIVE, command.request().semantic());
    }

    @Test
    public void immediateSemanticSetIsActiveCancellationSafeAndSingleUse() {
        SemanticScalarCommand<Mode> command = command();
        SemanticScalarCommand.Request<Mode> initial = command.request();
        Task set = SemanticScalarTasks.set(command, Mode.ACTIVE).build();
        ManualLoopClock time = new ManualLoopClock();

        set.cancel();
        assertFalse(set.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, set.getOutcome());
        assertSame(initial, command.request());
        assertLifecycleFailure(() -> set.update(time.clock()), "before start");

        set.start(time.clock());
        SemanticScalarCommand.Request<Mode> published = command.request();
        assertTrue(set.isComplete());
        assertEquals(TaskOutcome.SUCCESS, set.getOutcome());
        assertEquals(Mode.ACTIVE, published.semantic());

        set.cancel();
        set.update(time.clock());
        assertSame(published, command.request());
        assertLifecycleFailure(() -> set.start(time.clock()), "single-use");
        assertSame(published, command.request());
    }

    @Test
    public void timedTasksRejectUpdateBeforeStartAndEverySecondStart() {
        SemanticScalarCommand<Mode> command = command();
        Task timed = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .forSeconds(1.0)
                .leaveThere()
                .build();
        ManualLoopClock time = new ManualLoopClock();

        assertLifecycleFailure(() -> timed.update(time.clock()), "before start");
        timed.start(time.clock());
        assertLifecycleFailure(() -> timed.start(time.clock()), "single-use");
        timed.cancel();
        assertLifecycleFailure(() -> timed.start(time.clock()), "single-use");
    }

    private static SemanticScalarCommand<Mode> command() {
        return SemanticScalarCommand.create(Mode.IDLE,
                SemanticScalarTasksTimingTest::targetFor);
    }

    private static double targetFor(Mode mode) {
        switch (mode) {
            case ACTIVE:
                return 0.75;
            case OVERRIDE:
                return -0.5;
            default:
                return 0.0;
        }
    }

    private static void assertLifecycleFailure(Runnable action, String messagePart) {
        try {
            action.run();
            fail("expected lifecycle failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains(messagePart));
        }
    }
}
