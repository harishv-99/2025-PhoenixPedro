package edu.ftcphoenix.robots.examples.starter.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.ftc.input.GamepadDevice;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.fw.testing.ManualLoopClock;
import edu.ftcphoenix.fw.testing.RecordingCallbackBindings;
import edu.ftcphoenix.robots.examples.starter.capability.intake.StarterIntake;

import static org.junit.Assert.assertEquals;

/** Hardware-free proof of the Starter's first operator meanings. */
public final class StarterFirstLessonTest {

    @Test
    public void buttonsRequestSemanticModesOnRisingEdgesWithoutCreatingTasks() {
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

        driver.a = true;
        bindings.update(time.nextCycle(0.02));
        assertEquals(Arrays.asList(StarterIntake.Mode.COLLECT), intake.modeRequests);

        bindings.update(time.nextCycle(0.02)); // Holding A is not another rising edge.
        driver.a = false;
        bindings.update(time.nextCycle(0.02)); // Releasing A is not a rising edge.
        assertEquals(Arrays.asList(StarterIntake.Mode.COLLECT), intake.modeRequests);

        pulseB(driver, bindings, time);
        pulseX(driver, bindings, time);
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
            Mode mode = modeRequests.isEmpty()
                    ? Mode.STOPPED
                    : modeRequests.get(modeRequests.size() - 1);
            return new Status(mode, 0.0);
        }
    }
}
