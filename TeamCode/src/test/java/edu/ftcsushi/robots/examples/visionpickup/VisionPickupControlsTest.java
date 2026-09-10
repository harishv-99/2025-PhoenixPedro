package edu.ftcsushi.robots.examples.visionpickup;

import org.junit.Test;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.haptic.HapticSink;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.TaskRunner;

import static org.junit.Assert.*;

/** Maintainer scenarios: real bindings and runner; only operator and sensor evidence is scripted. */
public final class VisionPickupControlsTest {
    @Test public void releaseThenRepressCannotAuthorizeTheOlderQueuedGesture() {
        ControlsRig r = new ControlsRig();
        r.cycle(false, true, false, false);
        Task old = r.runner.nextQueuedTaskOrNull();
        r.cycle(false, false, false, false);
        r.cycle(false, true, false, true);
        assertEquals(TaskOutcome.CANCELLED, old.getOutcome());
        assertTrue(r.runner.hasActiveTask());
        assertNotSame(old, r.runner.currentTaskOrNull());
        assertEquals(VisionPickup.Phase.STAGING, r.rig.pickup.status().phase);
        old.cancel();
        assertTrue(r.runner.hasActiveTask());
        assertTrue(r.rig.intakeRequests.isEmpty());
    }

    @Test public void overridePermanentlyWithdrawsQueuedPickupEvenAfterItIsReleased() {
        ControlsRig r = new ControlsRig();
        r.cycle(false, true, false, false);
        Task old = r.runner.nextQueuedTaskOrNull();
        r.cycle(false, true, true, false);
        r.cycle(false, true, false, true);
        assertEquals(TaskOutcome.CANCELLED, old.getOutcome());
        assertTrue(r.runner.isIdle());
        assertEquals(0, r.rig.pickup.status().assistLossCount);
    }

    @Test public void releasedQueuedPickupCannotWithdrawANewerActiveAimSession() {
        ControlsRig r = new ControlsRig();
        r.cycle(false, true, false, false);
        Task obsolete = r.runner.nextQueuedTaskOrNull();
        r.cycle(false, false, false, false);
        r.cycle(true, false, false, false);
        r.cycle(true, false, false, false);
        assertEquals(VisionPickup.AssistState.AIMING, r.rig.pickup.status().assistState);
        long session = r.rig.pickup.status().aimSessionId;

        r.cycle(true, false, false, true);
        assertEquals(TaskOutcome.CANCELLED, obsolete.getOutcome());
        assertTrue(r.runner.isIdle());
        assertEquals(VisionPickup.AssistState.AIMING, r.rig.pickup.status().assistState);
        assertEquals(session, r.rig.pickup.status().aimSessionId);
        assertEquals(VisionPickup.Phase.IDLE, r.rig.pickup.status().phase);
        assertEquals(0, r.rig.pickup.status().assistLossCount);
        assertTrue(r.rig.drive().omega > 0);
        assertTrue(r.rig.intakeRequests.isEmpty());
    }

    @Test public void sameCycleReleaseOrOverrideConsumesLossWithoutALateAlarm() {
        for (boolean override : new boolean[]{false, true}) {
            ControlsRig r = new ControlsRig();
            List<Double> pulses = new ArrayList<>();
            VisionPickupFeedback feedback = new VisionPickupFeedback(r.rig.pickup, new HapticSink() {
                @Override public void pulse(double strength, double durationSec) { pulses.add(durationSec); }
                @Override public void stop() { }
            });
            VisionPickupPresenter presenter = new VisionPickupPresenter(r.rig.pickup);
            Map<String, Object> rows = new LinkedHashMap<>();
            Telemetry telemetry = (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(),
                    new Class<?>[]{Telemetry.class}, (proxy, method, args) -> {
                        if (!method.getName().equals("addData")) {
                            throw new AssertionError("presenter called " + method.getName());
                        }
                        rows.put((String) args[0], args[1]);
                        return null;
                    });
            r.cycle(true, false, false, true);
            r.cycle(true, false, false, true);
            feedback.update(r.rig.clock());
            assertTrue(pulses.isEmpty());

            // Services sees missing vision before Bindings sees this loop's intentional handoff.
            r.input[0] = override;
            r.input[2] = override;
            r.rig.step(0.05, Pose2d.zero());
            r.rig.pickup.update(r.rig.clock());
            assertEquals(VisionPickup.AssistState.LOST, r.rig.pickup.status().assistState);
            r.bindings.update(r.rig.clock());
            r.runner.update(r.rig.clock());
            r.rig.drive();
            feedback.update(r.rig.clock());
            presenter.present(r.rig.clock(), telemetry);
            assertEquals(VisionPickup.AssistState.IDLE, r.rig.pickup.status().assistState);
            assertEquals("Assistance off", rows.get("assist"));
            assertEquals("Manual control in TeleOp / zero idle in Auto", rows.get("assist.control"));
            assertFalse(rows.containsKey("assist.retry"));
            assertManual(r);
            assertEquals(1, r.rig.pickup.status().assistLossCount);
            assertTrue(pulses.isEmpty());

            // The consumed event cannot reappear when a later deliberate request is evaluated.
            r.cycle(false, false, false, true);
            feedback.update(r.rig.clock());
            r.cycle(true, false, false, true);
            feedback.update(r.rig.clock());
            r.cycle(true, false, false, true);
            feedback.update(r.rig.clock());
            assertEquals(VisionPickup.AssistState.AIMING, r.rig.pickup.status().assistState);
            assertTrue(pulses.isEmpty());
        }
    }

    @Test public void pickupPressedDuringOverrideIsNotBankedForLater() {
        ControlsRig r = new ControlsRig();
        r.cycle(false, true, true, false);
        r.cycle(false, true, false, true);
        assertTrue(r.runner.isIdle());
        assertEquals(VisionPickup.Phase.IDLE, r.rig.pickup.status().phase);
        r.cycle(false, false, false, true);
        r.cycle(false, true, false, true);
        assertTrue(r.runner.hasActiveTask());
    }

    @Test public void overrideReleaseCannotCreateAnAimPressOrFailureAlarm() {
        ControlsRig r = new ControlsRig();
        r.cycle(true, false, false, true);
        assertEquals(VisionPickup.AssistState.REQUESTED, r.rig.pickup.status().assistState);
        assertManual(r);
        r.cycle(true, false, false, true);
        assertEquals(VisionPickup.AssistState.AIMING, r.rig.pickup.status().assistState);
        r.cycle(true, false, true, true);
        assertManual(r);
        r.cycle(true, false, false, true);
        assertManual(r);
        assertEquals(0, r.rig.pickup.status().assistLossCount);
        r.cycle(false, false, false, true);
        r.cycle(true, false, false, true);
        assertManual(r); // The new request cannot reuse an earlier service evaluation.
        r.cycle(true, false, false, true);
        assertEquals(VisionPickup.AssistState.AIMING, r.rig.pickup.status().assistState);
    }

    @Test public void aimPressedDuringOverrideNeedsAnotherRealPress() {
        ControlsRig r = new ControlsRig();
        r.cycle(true, false, true, true);
        r.cycle(true, false, false, true);
        r.cycle(true, false, false, true);
        assertManual(r);
        r.cycle(false, false, false, true);
        r.cycle(true, false, false, true);
        r.cycle(true, false, false, true);
        assertEquals(VisionPickup.AssistState.AIMING, r.rig.pickup.status().assistState);
    }

    @Test public void endingPickupDoesNotRestoreAnOlderHeldAim() {
        ControlsRig r = new ControlsRig();
        r.cycle(true, false, false, true);
        r.cycle(true, false, false, true);
        r.cycle(true, true, false, true);
        assertEquals(VisionPickup.AssistState.PICKUP, r.rig.pickup.status().assistState);
        r.cycle(true, false, false, true);
        r.cycle(true, false, false, true);
        assertManual(r);
        assertEquals(0, r.rig.pickup.status().assistLossCount);
        r.cycle(false, false, false, true);
        r.cycle(true, false, false, true);
        r.cycle(true, false, false, true);
        assertEquals(VisionPickup.AssistState.AIMING, r.rig.pickup.status().assistState);
    }

    @Test public void stopCannotBeUndoneBySubsequentInputEdges() {
        ControlsRig r = new ControlsRig();
        r.cycle(true, true, false, true);
        r.runner.cancelAndClear();
        r.rig.pickup.stop();
        r.cycle(false, false, false, true);
        r.cycle(true, true, false, true);
        assertEquals(VisionPickup.AssistState.STOPPED, r.rig.pickup.status().assistState);
        assertEquals(0, r.rig.drive().axial, 0);
        assertEquals(0, r.rig.drive().omega, 0);
        assertTrue(r.rig.intakeRequests.isEmpty());
    }

    @Test public void bindingTwiceFailsWithoutChangingExistingRegistrations() {
        ControlsRig r = new ControlsRig();
        try {
            r.controls.bind(r.bindings, TaskBindings.of(r.bindings, r.runner), r.rig.pickup);
            fail("one controls registration");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("once"));
        }
        r.cycle(false, true, false, false);
        assertEquals(1, r.runner.queuedCount());
    }

    private static void assertManual(ControlsRig r) {
        assertEquals(r.rig.manual.axial, r.rig.drive().axial, 0);
        assertEquals(r.rig.manual.lateral, r.rig.drive().lateral, 0);
        assertEquals(r.rig.manual.omega, r.rig.drive().omega, 0);
    }

    /** Retains production phase order; selectively pauses only the runner to exercise pending work. */
    private static final class ControlsRig {
        final VisionPickupTestRig rig = new VisionPickupTestRig(
                VisionPickupTestRig.configured(), Pose2d.zero(), 10, 4);
        final boolean[] input = new boolean[3];
        final Bindings bindings = new Bindings();
        final TaskRunner runner = new TaskRunner();
        final VisionPickupControls controls = new VisionPickupControls(
                clock -> input[0], clock -> input[1], clock -> input[2]);

        ControlsRig() {
            rig.manual = new DriveSignal(0.3, -0.2, -0.4);
            controls.bind(bindings, TaskBindings.of(bindings, runner), rig.pickup);
            rig.pickup.update(rig.clock());
            bindings.update(rig.clock());
        }

        void cycle(boolean aim, boolean pickup, boolean override, boolean runTasks) {
            input[0] = aim;
            input[1] = pickup;
            input[2] = override;
            rig.step(0.05, Pose2d.zero(), 10, 4);
            rig.pickup.update(rig.clock());
            bindings.update(rig.clock());
            if (runTasks) runner.update(rig.clock());
            rig.drive();
        }
    }
}
