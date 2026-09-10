package edu.ftcsushi.robots.examples.visionpickup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.haptic.HapticSink;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Real pickup policy, guidance, selection, and feedback owner with scripted outside-world evidence
 * and a recording command sink. Pulse requests prove neither controller support nor delivery,
 * recognizability, physical heading accuracy, capture, or safe robot motion.
 */
public final class VisionPickupFeedbackTest {
    @Test
    public void firstAlignmentPulsesOnceDespiteRepeatedReadsAndToleranceChatter() {
        Fixture f = new Fixture(10, 4);
        f.rig.pickup.setAimEnabled(true);
        f.updateAll();
        assertEquals(VisionPickup.AssistState.AIMING, f.rig.pickup.status().assistState);
        assertTrue(f.sink.durations.isEmpty());

        f.step(10, 0);
        assertEquals(VisionPickup.AssistState.ALIGNED, f.rig.pickup.status().assistState);
        f.feedback.update(f.rig.clock());
        f.feedback.update(f.rig.clock());
        f.step(10, 4);
        f.step(10, 0);
        assertEquals(Arrays.asList(0.10), f.sink.durations);
        assertEquals(Arrays.asList(1.0), f.sink.strengths);

        f.rig.pickup.setAimEnabled(false);
        f.step(10, 0);
        f.rig.pickup.setAimEnabled(true);
        f.step(10, 0);
        assertEquals(Arrays.asList(0.10, 0.10), f.sink.durations);
    }

    @Test
    public void lostAssistanceHasOneLongCueAndReturningEvidenceCannotClaimRealignment() {
        Fixture f = new Fixture(10, 0);
        f.rig.pickup.setAimEnabled(true);
        f.updateAll();
        f.step();
        assertEquals(VisionPickup.AssistState.LOST, f.rig.pickup.status().assistState);
        assertEquals(Arrays.asList(0.10, 0.50), f.sink.durations);
        f.feedback.update(f.rig.clock());
        f.step(10, 0);
        f.step(10, 0);
        assertEquals(VisionPickup.AssistState.LOST, f.rig.pickup.status().assistState);
        assertEquals(Arrays.asList(0.10, 0.50), f.sink.durations);
    }

    @Test
    public void newLossWinsOverAnAlignmentFirstObservedInTheSameFeedbackPass() {
        Fixture f = new Fixture(10, 4);
        f.rig.pickup.setAimEnabled(true);
        f.updateAll();
        // Deliberately delay only the feedback owner while real policy loses its target and a
        // fresh deliberate aim session reaches alignment. The retained loss still wins.
        f.rig.step(0.02, Pose2d.zero());
        f.rig.pickup.update(f.rig.clock());
        f.rig.drive();
        assertEquals(VisionPickup.AssistState.LOST, f.rig.pickup.status().assistState);
        f.rig.pickup.setAimEnabled(false);
        f.rig.step(0.02, Pose2d.zero(), 10, 0);
        f.rig.pickup.setAimEnabled(true);
        f.rig.pickup.update(f.rig.clock());
        f.rig.drive();
        assertEquals(VisionPickup.AssistState.ALIGNED, f.rig.pickup.status().assistState);
        f.feedback.update(f.rig.clock());
        f.step(10, 0);
        assertEquals(Arrays.asList(0.50), f.sink.durations);
    }

    @Test
    public void distinctLossesCanPulseAgainOnlyAfterDeliberateNewRequests() {
        Fixture f = new Fixture(10, 4);
        f.rig.pickup.setAimEnabled(true);
        f.updateAll();
        f.step();
        f.rig.pickup.setAimEnabled(false);
        f.step(10, 4);
        f.rig.pickup.setAimEnabled(true);
        f.step(10, 4);
        f.step();
        assertEquals(Arrays.asList(0.50, 0.50), f.sink.durations);
        assertEquals(2, f.rig.pickup.status().assistLossCount);
    }

    @Test
    public void intentionalReleaseAndStopDoNotSoundLikeFailure() {
        Fixture f = new Fixture(10, 4);
        f.rig.pickup.setAimEnabled(true);
        f.updateAll();
        f.rig.pickup.setAimEnabled(false);
        f.step();
        assertEquals(VisionPickup.AssistState.IDLE, f.rig.pickup.status().assistState);
        assertTrue(f.sink.durations.isEmpty());
        f.rig.pickup.stop();
        f.step();
        assertEquals(VisionPickup.AssistState.STOPPED, f.rig.pickup.status().assistState);
        assertTrue(f.sink.durations.isEmpty());
    }

    @Test
    public void stoppedPickupSuppressesAnUnobservedLossCue() {
        Fixture f = new Fixture(10, 4);
        f.rig.pickup.setAimEnabled(true);
        f.updateAll();
        f.rig.step(0.02, Pose2d.zero());
        f.rig.pickup.update(f.rig.clock());
        f.rig.drive();
        assertEquals(VisionPickup.AssistState.LOST, f.rig.pickup.status().assistState);
        f.rig.pickup.stop();
        f.feedback.update(f.rig.clock());
        assertTrue(f.sink.durations.isEmpty());
    }

    @Test
    public void terminalStopBeforeFirstUpdateIsIdempotentAndCannotBeRearmed() {
        Fixture f = new Fixture(10, 0);
        f.feedback.stop();
        f.feedback.stop();
        f.rig.pickup.setAimEnabled(true);
        f.updateAll();
        assertEquals(1, f.sink.stops);
        assertTrue(f.sink.durations.isEmpty());
    }

    @Test
    public void stopInsidePulsePreventsEveryLaterOwnerEffect() {
        Fixture f = new Fixture(10, 0);
        f.sink.pulseHook = f.feedback::stop;
        f.rig.pickup.setAimEnabled(true);
        f.updateAll();
        f.step();
        f.feedback.stop();
        assertEquals(Arrays.asList(0.10), f.sink.durations);
        assertEquals(1, f.sink.stops);
    }

    @Test
    public void failedPulseIsNotRetriedInTheSameOrLaterCycle() {
        Fixture f = new Fixture(10, 0);
        RuntimeException failed = new IllegalStateException("scripted haptic boundary failure");
        f.sink.pulseHook = () -> { throw failed; };
        f.rig.pickup.setAimEnabled(true);
        assertSame(failed, thrown(f::updateAll));
        assertSame(failed, thrown(() -> f.feedback.update(f.rig.clock())));
        f.rig.time.nextCycle(0.02);
        assertSame(failed, thrown(() -> f.feedback.update(f.rig.clock())));
        assertEquals(Arrays.asList(0.10), f.sink.durations);
        f.feedback.stop();
        assertEquals(1, f.sink.stops);
    }

    @Test
    public void caughtRecursiveUpdateStillFailsWithoutDispatchingAnotherPulse() {
        Fixture f = new Fixture(10, 0);
        RuntimeException[] nested = {null};
        f.sink.pulseHook = () -> nested[0] = thrown(() -> f.feedback.update(f.rig.clock()));
        f.rig.pickup.setAimEnabled(true);
        RuntimeException failed = thrown(f::updateAll);
        assertSame(nested[0], failed);
        assertTrue(failed.getMessage().contains("reentrant"));
        assertSame(failed, thrown(() -> f.feedback.update(f.rig.clock())));
        assertEquals(Arrays.asList(0.10), f.sink.durations);
    }

    @Test
    public void failedStopIsAttemptedOnceAndLeavesFeedbackTerminal() {
        Fixture f = new Fixture(10, 0);
        RuntimeException failed = new IllegalStateException("scripted haptic stop failure");
        f.sink.stopFailure = failed;
        assertSame(failed, thrown(f.feedback::stop));
        f.feedback.stop();
        f.rig.pickup.setAimEnabled(true);
        f.updateAll();
        assertEquals(1, f.sink.stops);
        assertTrue(f.sink.durations.isEmpty());
    }

    @Test
    public void feedbackAndPresenterReadCachedStateWithoutSamplingBehaviorSources() {
        Fixture f = new Fixture(10, 0);
        f.rig.pickup.setAimEnabled(true);
        f.rig.pickup.update(f.rig.clock());
        f.rig.drive();
        VisionPickup.Status snapshot = f.rig.pickup.status();
        f.rig.selectionHook = () -> fail("feedback/presenter must not sample vision");
        f.rig.captureHook = () -> fail("feedback/presenter must not sample capture feedback");
        f.rig.localizer.estimate = null;
        f.feedback.update(f.rig.clock());
        f.presenter.present(f.rig.clock(), f.telemetry.telemetry);
        assertEquals(snapshot.assistState, f.rig.pickup.status().assistState);
        assertEquals(snapshot.aimSessionId, f.rig.pickup.status().aimSessionId);
        assertEquals(snapshot.assistLossCount, f.rig.pickup.status().assistLossCount);
        assertEquals(snapshot.reason, f.rig.pickup.status().reason);
        assertEquals("Aligned - assistance active", f.telemetry.rows.get("assist"));
        assertEquals(Arrays.asList(0.10), f.sink.durations);
        assertFalse(f.telemetry.rows.containsKey("assist.retry"));
    }

    @Test
    public void requiredDisplayDistinguishesAlignedFromLossWhileBothCommandsCanBeZero() {
        Fixture f = new Fixture(10, 0);
        f.rig.pickup.setAimEnabled(true);
        f.updateAll();
        assertEquals(0.0, f.rig.drive().omega, 0.0);
        f.presenter.present(f.rig.clock(), f.telemetry.telemetry);
        assertEquals("Aligned - assistance active", f.telemetry.rows.get("assist"));
        f.step();
        assertEquals(0.0, f.rig.drive().omega, 0.0);
        f.presenter.present(f.rig.clock(), f.telemetry.telemetry);
        assertEquals("Assist lost - not completed", f.telemetry.rows.get("assist"));
        assertEquals("Manual control in TeleOp / zero idle in Auto",
                f.telemetry.rows.get("assist.control"));
        assertEquals(f.rig.pickup.status().assistReason, f.telemetry.rows.get("assist.reason"));
        assertEquals("Aim: release/repress. Pickup: new press. No automatic retry.",
                f.telemetry.rows.get("assist.retry"));
        assertEquals(Arrays.asList(0.10, 0.50), f.sink.durations);
    }

    @Test
    public void differentClockIsRejectedWithoutAnotherPulse() {
        Fixture f = new Fixture(10, 0);
        f.rig.pickup.setAimEnabled(true);
        f.updateAll();
        RuntimeException failed = thrown(() -> f.feedback.update(new ManualLoopClock().clock()));
        assertTrue(failed.getMessage().contains("same shared LoopClock"));
        assertEquals(Arrays.asList(0.10), f.sink.durations);
    }

    @Test
    public void displaySeparatesRequestedOffAndNormalStopWithoutImplyingAlignment() {
        Fixture f = new Fixture(10, 0);
        f.presenter.present(f.rig.clock(), f.telemetry.telemetry);
        assertEquals("Assistance off", f.telemetry.rows.get("assist"));
        f.rig.pickup.setAimEnabled(true);
        f.presenter.present(f.rig.clock(), f.telemetry.telemetry);
        assertEquals("Aim requested - evidence not yet checked", f.telemetry.rows.get("assist"));
        assertEquals("Manual control in TeleOp / zero idle in Auto",
                f.telemetry.rows.get("assist.control"));
        f.rig.pickup.stop();
        f.presenter.present(f.rig.clock(), f.telemetry.telemetry);
        assertEquals("Stopped", f.telemetry.rows.get("assist"));
        assertEquals("Zero drive requested", f.telemetry.rows.get("assist.control"));
        assertTrue(f.sink.durations.isEmpty());
    }

    @Test
    public void displayExplainsExceptionalFailureInsteadOfClaimingNormalCancellation() {
        Fixture f = new Fixture(10, 0);
        RuntimeException failed = new IllegalStateException("scripted camera failure");
        f.rig.selectionHook = () -> { throw failed; };
        f.rig.pickup.setAimEnabled(true);
        assertSame(failed, thrown(f::updateAll));
        f.presenter.present(f.rig.clock(), f.telemetry.telemetry);
        assertEquals("Fault - stopped", f.telemetry.rows.get("assist"));
        assertEquals("Zero drive requested", f.telemetry.rows.get("assist.control"));
        assertEquals("LIFECYCLE FAILURE - no normal outcome", f.telemetry.rows.get("pickup.outcome"));
        assertTrue(f.rig.pickup.status().hasFailure);
        assertTrue(f.sink.durations.isEmpty());
    }

    @Test
    public void intentionalPickupCancellationHasNoLossCueButNewRejectedPickupDoes() {
        Fixture f = new Fixture(10, 0);
        Task attempt = f.rig.pickup.createPickupTask(clock -> true);
        attempt.start(f.rig.clock());
        f.feedback.update(f.rig.clock());
        f.presenter.present(f.rig.clock(), f.telemetry.telemetry);
        assertEquals("Pickup owns drive", f.telemetry.rows.get("assist.control"));
        attempt.cancel();
        f.step();
        assertTrue(f.sink.durations.isEmpty());

        Task rejected = f.rig.pickup.createPickupTask(clock -> true);
        rejected.start(f.rig.clock());
        f.rig.time.nextCycle(0.02);
        f.feedback.update(f.rig.clock());
        f.presenter.present(f.rig.clock(), f.telemetry.telemetry);
        assertEquals("Assist lost - not completed", f.telemetry.rows.get("assist"));
        assertEquals("Aim: release/repress. Pickup: new press. No automatic retry.",
                f.telemetry.rows.get("assist.retry"));
        assertEquals(Arrays.asList(0.50), f.sink.durations);
    }

    /** Asserts identity-preserving RuntimeException paths, not Java Error recovery. */
    private static RuntimeException thrown(Runnable action) {
        try {
            action.run();
            fail("expected RuntimeException");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    /** Test order mirrors pickup service, source-driven drive, then feedback output. */
    private static final class Fixture {
        final VisionPickupTestRig rig;
        final RecordingHaptics sink = new RecordingHaptics();
        final VisionPickupFeedback feedback;
        final VisionPickupPresenter presenter;
        final RecordingTelemetry telemetry = new RecordingTelemetry();

        Fixture(double... fieldTargets) {
            rig = new VisionPickupTestRig(VisionPickupTestRig.configured(), Pose2d.zero(), fieldTargets);
            feedback = new VisionPickupFeedback(rig.pickup, sink);
            presenter = new VisionPickupPresenter(rig.pickup);
        }

        void updateAll() {
            rig.pickup.update(rig.clock());
            rig.drive();
            feedback.update(rig.clock());
        }

        void step(double... fieldTargets) {
            rig.step(0.02, Pose2d.zero(), fieldTargets);
            updateAll();
        }
    }

    /** Records attempted requests before an optional adversarial outside-boundary callback. */
    private static final class RecordingHaptics implements HapticSink {
        final List<Double> strengths = new ArrayList<>();
        final List<Double> durations = new ArrayList<>();
        int stops;
        Runnable pulseHook;
        RuntimeException stopFailure;

        @Override public void pulse(double strength, double durationSec) {
            strengths.add(strength);
            durations.add(durationSec);
            if (pulseHook != null) pulseHook.run();
        }

        @Override public void stop() {
            stops++;
            if (stopFailure != null) throw stopFailure;
        }
    }

    /** Rejects frame ownership or hidden behavior work: a presenter may only append data. */
    private static final class RecordingTelemetry {
        final Map<String, Object> rows = new LinkedHashMap<>();
        final Telemetry telemetry = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(), new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    if (!method.getName().equals("addData")) {
                        throw new AssertionError("presenter called " + method.getName());
                    }
                    rows.put((String) args[0], args[1]);
                    return null;
                });
    }
}
