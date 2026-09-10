package edu.ftcsushi.robots.examples.visionpickup;

import java.util.Arrays;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.Tasks;

import static org.junit.Assert.*;

/** Adversarial maintainer evidence: exceptions never become ordinary loss or release continuation. */
public final class VisionPickupFailureTest {
    @Test public void intakeFailureRemainsPrimaryAndEndingIsAttemptedOnce() {
        VisionPickupTestRig r = rig();
        RuntimeException primary = new IllegalStateException("intake enable failed");
        RuntimeException cleanup = new IllegalStateException("intake idle failed");
        r.intakeHook = enabled -> { throw enabled ? primary : cleanup; };
        Task task = r.pickup.createPickupTask(clock -> true);
        task.start(r.clock());
        r.step(0.05, new Pose2d(2, 0, 0), 10, 0);
        r.pickup.update(r.clock());
        task.update(r.clock());
        r.step(0.05, new Pose2d(2, 0, 0), 10, 0);
        r.pickup.update(r.clock());
        sameFailure(primary, () -> task.update(r.clock()));
        sameFailure(primary, task::getOutcome);
        sameFailure(primary, () -> task.update(r.clock()));
        task.cancel();
        r.pickup.stop();
        assertEquals(Arrays.asList(true, false), r.intakeRequests);
        assertEquals(1, primary.getSuppressed().length);
        assertSame(cleanup, primary.getSuppressed()[0]);
        assertFailed(r);
    }

    @Test public void neitherSequenceFamilyContinuesAfterACaughtChildFailure() {
        for (boolean recoverOnCompletion : new boolean[]{false, true}) {
            VisionPickupTestRig r = rig();
            RuntimeException primary = new IllegalStateException("selection failed");
            int[] continued = {0};
            Task child = r.pickup.createPickupTask(clock -> true);
            Task next = Tasks.runOnce(() -> continued[0]++);
            Task root = recoverOnCompletion ? Tasks.sequenceOnCompletion(child, next)
                    : Tasks.sequence(child, next);
            root.start(r.clock());
            r.step(0.05, new Pose2d(2, 0, 0), 10, 0);
            r.pickup.update(r.clock());
            root.update(r.clock());
            assertEquals(VisionPickup.Phase.RECHECK, r.pickup.status().phase);
            r.step(0.05, new Pose2d(2, 0, 0), 10, 0);
            r.pickup.update(r.clock());
            r.selectionHook = () -> { throw primary; };
            // An external caller catches the child failure before the sequence observes it.
            sameFailure(primary, () -> child.update(r.clock()));
            sameFailure(primary, child::getOutcome);
            sameFailure(primary, () -> root.update(r.clock()));
            root.update(r.clock()); // A wrapper already terminalized by failure is inert.
            assertEquals(0, continued[0]);
            assertFailed(r);
        }
    }

    @Test public void exceptionAfterReentrantCancellationStillStopsTheOwner() {
        VisionPickupTestRig r = rig();
        RuntimeException primary = new IllegalStateException("failed after cancellation");
        Task task = r.pickup.createPickupTask(clock -> {
            r.pickup.cancelPickup();
            throw primary;
        });
        sameFailure(primary, () -> task.start(r.clock()));
        sameFailure(primary, task::getOutcome);
        assertFailed(r);
        Task replacement = r.pickup.createPickupTask(clock -> true);
        replacement.start(r.clock());
        assertTrue(replacement.isComplete());
        assertEquals(0, r.drive().axial, 0);
    }

    @Test public void cleanupCannotExposeASuccessfulOrCancelledResultBeforeItSettles() {
        VisionPickupTestRig r = rig();
        Task task = r.enterFinal();
        r.intakeHook = enabled -> {
            if (!enabled) {
                assertEquals(TaskOutcome.NOT_DONE, r.pickup.status().outcome);
                try { task.getOutcome(); fail("ending outcome must remain unavailable"); }
                catch (IllegalStateException expected) { /* Deliberately caught by callback. */ }
            }
        };
        try { task.cancel(); fail("caught pending-outcome failure must remain retained"); }
        catch (IllegalStateException expected) {
            sameFailure(expected, task::getOutcome);
        }
        assertFailed(r);
        assertEquals(Arrays.asList(true, false), r.intakeRequests);
    }

    @Test public void aimCallbackFailureIsNotARecoverableManualFallback() {
        VisionPickupTestRig r = rig();
        RuntimeException primary = new IllegalStateException("target source failed");
        r.selectionHook = () -> { r.pickup.setAimEnabled(false); throw primary; };
        r.pickup.setAimEnabled(true);
        sameFailure(primary, () -> r.pickup.update(r.clock()));
        assertFailed(r);
        r.selectionHook = null;
        r.pickup.setAimEnabled(false);
        r.pickup.setAimEnabled(true);
        assertEquals(0, r.drive().omega, 0);
        assertEquals(VisionPickup.AssistState.STOPPED, r.pickup.status().assistState);
    }

    @Test public void ordinaryChildCancellationMayRunExplicitRecoveryButRootCancellationMayNot() {
        VisionPickupTestRig r = rig();
        int[] continued = {0};
        Task child = r.pickup.createPickupTask(clock -> false);
        Task root = Tasks.sequenceOnCompletion(child, Tasks.runOnce(() -> continued[0]++));
        root.start(r.clock());
        root.update(r.clock());
        assertEquals(1, continued[0]);
        assertEquals(TaskOutcome.CANCELLED, root.getOutcome());

        VisionPickupTestRig other = rig();
        Task cancelled = Tasks.sequenceOnCompletion(other.pickup.createPickupTask(clock -> true),
                Tasks.runOnce(() -> continued[0]++));
        cancelled.start(other.clock());
        cancelled.cancel();
        cancelled.update(other.clock());
        assertEquals(1, continued[0]);
    }

    private static VisionPickupTestRig rig() {
        VisionPickupTestRig r = new VisionPickupTestRig(
                VisionPickupTestRig.configured(), Pose2d.zero(), 10, 0);
        r.manual = new DriveSignal(0.4, 0.2, -0.3);
        return r;
    }

    private static void assertFailed(VisionPickupTestRig r) {
        assertTrue(r.pickup.status().hasFailure);
        assertEquals(TaskOutcome.NOT_DONE, r.pickup.status().outcome);
        assertEquals(VisionPickup.AssistState.STOPPED, r.pickup.status().assistState);
        assertEquals(0, r.drive().axial, 0);
        assertEquals(0, r.drive().omega, 0);
    }

    private static void sameFailure(RuntimeException expected, Runnable action) {
        try { action.run(); fail("expected retained failure"); }
        catch (RuntimeException actual) { assertSame(expected, actual); }
    }
}
