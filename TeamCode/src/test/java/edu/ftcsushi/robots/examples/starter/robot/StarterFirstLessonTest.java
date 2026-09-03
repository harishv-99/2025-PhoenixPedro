package edu.ftcsushi.robots.examples.starter.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.RecordingCallbackBindings;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;

import static org.junit.Assert.assertEquals;

/**
 * Hardware-free proof of the Starter's first operator meanings.
 *
 * <p>The testing idea is deliberately small: replace the intake with a recorder, stimulate the
 * real bindings with gamepad edges, and inspect only the semantic requests that cross that
 * boundary. This test is about controls vocabulary, so it does not pretend to prove motor wiring
 * or physical motion.</p>
 */
public final class StarterFirstLessonTest {

    @Test
    public void buttonsRequestSemanticModesOnRisingEdgesWithoutCreatingTasks() {
        // ARRANGE: real controls and bindings, but a tiny recorder instead of hardware.
        Gamepad driver = new Gamepad();
        RecordingIntake intake = new RecordingIntake();
        RecordingCallbackBindings callbacks = new RecordingCallbackBindings();
        StarterTeleOpControls controls =
                new StarterTeleOpControls(new GamepadDevice(driver));
        controls.bind(callbacks, intake);
        assertEquals(3, callbacks.successfulRegistrations());

        Bindings bindings = callbacks.root();
        ManualLoopClock time = new ManualLoopClock();
        bindings.update(time.clock());

        // ACT: one A-button rising edge means COLLECT.
        driver.a = true;
        bindings.update(time.nextCycle(0.02));

        // EVIDENCE: assert the meaning at the boundary, not a private implementation detail.
        assertEquals(Arrays.asList(StarterIntake.Mode.COLLECT), intake.modeRequests);

        // EDGE CONTRACT: holding and releasing the button must not repeat the request.
        bindings.update(time.nextCycle(0.02)); // Holding A is not another rising edge.
        driver.a = false;
        bindings.update(time.nextCycle(0.02)); // Releasing A is not a rising edge.
        assertEquals(Arrays.asList(StarterIntake.Mode.COLLECT), intake.modeRequests);

        pulseB(driver, bindings, time);
        pulseX(driver, bindings, time);

        // COVERAGE: the remaining buttons preserve the same semantic, direct-command pattern.
        assertEquals(
                Arrays.asList(
                        StarterIntake.Mode.COLLECT,
                        StarterIntake.Mode.EJECT,
                        StarterIntake.Mode.STOPPED),
                intake.modeRequests);
        assertEquals(0, intake.taskRequests);
    }

    private static void pulseB(Gamepad driver, Bindings bindings, ManualLoopClock time) {
        driver.b = true;
        bindings.update(time.nextCycle(0.02));
        driver.b = false;
        bindings.update(time.nextCycle(0.02));
    }

    private static void pulseX(Gamepad driver, Bindings bindings, ManualLoopClock time) {
        driver.x = true;
        bindings.update(time.nextCycle(0.02));
        driver.x = false;
        bindings.update(time.nextCycle(0.02));
    }

    private static final class RecordingIntake implements StarterIntake {
        private final List<Mode> modeRequests = new ArrayList<Mode>();
        private int taskRequests;

        @Override
        public void setMode(Mode mode) {
            modeRequests.add(mode);
        }

        @Override
        public Task collectForSeconds(double durationSec) {
            taskRequests++;
            return Tasks.noop();
        }

        @Override
        public Status status() {
            throw new AssertionError("RecordingIntake status is not used by this lesson test");
        }
    }
}
