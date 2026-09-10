package edu.ftcsushi.robots.examples.reference.capability.launcher;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.junit.Test;
import java.lang.reflect.Field;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.robots.examples.reference.control.ReferenceFeedingControls;

import static edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncherTestRig.STEP;
import static org.junit.Assert.*;

/** Maintainer checks for actual edges, pending admission, and no-motion acknowledgement. */
public final class ReferenceFeedingControlsTest {
    @Test public void additionalPressWhilePendingDoesNotConstructAnotherFeed() throws Exception {
        ControlsRig r = new ControlsRig();
        r.gamepad.a = true;
        r.bindingsOnly(); // Factory accepted, but the runner has not started the feed.
        Task pending = r.retainedFeed();
        assertFalse(pending.isComplete());
        assertFalse(r.robot.mechanism.status().attemptActive());
        r.gamepad.a = false;
        r.bindingsOnly();
        r.gamepad.a = true;
        r.bindingsOnly();
        assertSame("a queued feed is already pending", pending, r.retainedFeed());
        r.runner.update(r.robot.time.clock());
        r.robot.mechanism.update(r.robot.time.clock());
        r.reachRelease();
        r.cycle();
        r.robot.staged(false);
        r.finish();
        assertEquals(TaskOutcome.SUCCESS, pending.getOutcome());
        assertSame(pending, r.retainedFeed());
        assertTrue(r.runner.isIdle());
    }

    @Test public void heldAThroughCompletionDoesNotCreateAnArtificialRise() throws Exception {
        ControlsRig r = new ControlsRig();
        r.gamepad.a = true;
        r.cycle();
        Task first = r.retainedFeed();
        r.reachRelease();
        r.cycle();
        r.robot.staged(false);
        r.finish();
        long idleRequest = r.robot.mechanism.flywheels().requestId();
        for (int i = 0; i < 8; i++) r.cycle();
        assertEquals(TaskOutcome.SUCCESS, first.getOutcome());
        assertSame(first, r.retainedFeed());
        assertEquals(idleRequest, r.robot.mechanism.flywheels().requestId());
        assertTrue(r.runner.isIdle());
        r.gamepad.a = false;
        r.cycle();
        r.gamepad.a = true;
        r.cycle();
        assertNotSame("a later real edge can request a new attempt", first, r.retainedFeed());
        assertTrue(r.robot.mechanism.status().attemptActive());
    }

    @Test public void simultaneousAAndBInvalidatesTheJustCreatedTask() throws Exception {
        ControlsRig r = new ControlsRig();
        r.gamepad.a = true;
        r.gamepad.b = true;
        r.cycle();
        assertEquals(TaskOutcome.CANCELLED, r.retainedFeed().getOutcome());
        assertFalse(r.robot.mechanism.status().attemptActive());
        r.robot.assertIdle();
        for (int i = 0; i < 4; i++) r.cycle();
        assertTrue(r.runner.isIdle());
    }

    @Test public void acknowledgementButtonUsesCachedEvidenceWithoutMovingOrResuming() throws Exception {
        ControlsRig r = new ControlsRig();
        r.gamepad.a = true;
        r.cycle();
        Task old = r.retainedFeed();
        r.reachRelease();
        r.gamepad.a = false;
        r.gamepad.b = true;
        r.cycle();
        assertTrue(r.robot.mechanism.status().recoveryRequired());
        r.gamepad.b = false;
        r.cycle(); // A distinct post-failure idle publication.
        int motorWrites = r.robot.left.velocityWrites();
        int releaseWrites = r.robot.release.positionWrites();
        int transferWrites = r.robot.transfer.powerWrites();
        int reads = r.robot.first.stateReadCalls();
        r.gamepad.x = true;
        r.bindingsOnly();
        assertEquals(ReferenceLauncher.RecoveryResult.ACKNOWLEDGED,
                r.controls.lastRecoveryResult());
        assertEquals(motorWrites, r.robot.left.velocityWrites());
        assertEquals(releaseWrites, r.robot.release.positionWrites());
        assertEquals(transferWrites, r.robot.transfer.powerWrites());
        assertEquals(reads, r.robot.first.stateReadCalls());
        assertEquals(TaskOutcome.CANCELLED, old.getOutcome());
        assertSame(old, r.retainedFeed());
        assertFalse(r.robot.mechanism.status().attemptActive());
    }

    @Test public void controlsBindExactlyOnceWithoutCreatingATaskDuringSetup() throws Exception {
        ControlsRig r = new ControlsRig();
        assertNull(r.retainedFeed());
        assertNull(r.controls.lastRecoveryResult());
        assertThrows(IllegalStateException.class, () -> r.controls.bind(
                r.bindings, TaskBindings.of(r.bindings, r.runner), r.robot.mechanism));
        assertTrue(r.runner.isIdle());
        assertFalse(r.robot.mechanism.status().attemptActive());
    }

    private static final class ControlsRig {
        final ReferenceLauncherTestRig robot = new ReferenceLauncherTestRig();
        final Gamepad gamepad = new Gamepad();
        final Bindings bindings = new Bindings();
        final TaskRunner runner = new TaskRunner();
        final ReferenceFeedingControls controls = new ReferenceFeedingControls(new GamepadDevice(gamepad));

        ControlsRig() {
            controls.bind(bindings, TaskBindings.of(bindings, runner), robot.mechanism);
            bindings.update(robot.time.clock()); // Neutral baseline before any real button edge.
            robot.mechanism.update(robot.time.clock());
        }

        void bindingsOnly() { bindings.update(robot.time.nextCycle(STEP)); }

        void cycle() {
            bindingsOnly();
            runner.update(robot.time.clock());
            robot.mechanism.update(robot.time.clock());
        }

        void reachRelease() {
            for (int i = 0; i < 30 && robot.mechanism.status().phase()
                    != ReferenceLauncher.Phase.RELEASING; i++) cycle();
            assertEquals(ReferenceLauncher.Phase.RELEASING, robot.mechanism.status().phase());
            assertTrue(robot.mechanism.status().attemptActive());
        }

        void finish() {
            for (int i = 0; i < 80 && !runner.isIdle(); i++) cycle();
            assertTrue(runner.isIdle());
        }

        Task retainedFeed() throws Exception {
            Field field = ReferenceFeedingControls.class.getDeclaredField("lastFeed");
            field.setAccessible(true);
            return (Task) field.get(controls); // Inspect identity only; no lifecycle mutation.
        }
    }
}
