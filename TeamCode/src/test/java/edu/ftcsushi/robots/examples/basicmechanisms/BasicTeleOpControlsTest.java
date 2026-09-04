package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.RecordingCallbackBindings;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Maintainer contract for the independent home, full-lift, and claw control vocabularies. */
public final class BasicTeleOpControlsTest {

    @Test
    public void buttonsMapToNamedLiftAndClawIntent() {
        Gamepad driver = new Gamepad();
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        RecordingCallbackBindings callbacks = new RecordingCallbackBindings();
        TaskRunner runner = new TaskRunner();
        GamepadDevice driverDevice = new GamepadDevice(driver);
        BasicLiftControls liftControls = new BasicLiftControls(driverDevice);
        BasicClawControls clawControls = new BasicClawControls(driverDevice);
        liftControls.bind(callbacks, TaskBindings.of(callbacks, runner), lift);
        clawControls.bind(callbacks, claw);
        Bindings bindings = callbacks.root();
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(7, callbacks.successfulRegistrations());
        bindings.update(time.clock());

        pulse(driver, bindings, time, Button.DPAD_DOWN);
        pulse(driver, bindings, time, Button.DPAD_LEFT);
        pulse(driver, bindings, time, Button.DPAD_UP);
        pulse(driver, bindings, time, Button.A);
        pulse(driver, bindings, time, Button.Y);
        pulse(driver, bindings, time, Button.B);

        assertEquals(
                Arrays.asList(
                        BasicLift.Height.STOWED,
                        BasicLift.Height.LOW,
                        BasicLift.Height.HIGH),
                lift.heightRequests);
        assertEquals(
                Arrays.asList(
                        BasicClaw.State.CLOSED,
                        BasicClaw.State.HALF,
                        BasicClaw.State.OPEN),
                claw.stateRequests);
    }

    @Test
    public void homeOnlyControlsBuildAndQueueAFreshTaskForEveryXButtonRise() {
        Gamepad driver = new Gamepad();
        RecordingLift lift = new RecordingLift();
        RecordingCallbackBindings callbacks = new RecordingCallbackBindings();
        TaskRunner runner = new TaskRunner();
        BasicLiftHomeControls controls = new BasicLiftHomeControls(new GamepadDevice(driver));
        controls.bind(TaskBindings.of(callbacks, runner), lift);
        Bindings bindings = callbacks.root();
        ManualLoopClock time = new ManualLoopClock();

        bindings.update(time.clock());
        pulse(driver, bindings, time, Button.X);
        pulse(driver, bindings, time, Button.X);

        assertEquals(2, lift.homeTasks.size());
        assertNotSame(lift.homeTasks.get(0), lift.homeTasks.get(1));
        assertEquals(2, runner.queuedCount());
    }

    @Test
    public void fullLiftControlsAlsoBuildAFreshHomeTaskForEveryXButtonRise() {
        Gamepad driver = new Gamepad();
        RecordingLift lift = new RecordingLift();
        RecordingCallbackBindings callbacks = new RecordingCallbackBindings();
        TaskRunner runner = new TaskRunner();
        BasicLiftControls controls = new BasicLiftControls(new GamepadDevice(driver));
        controls.bind(callbacks, TaskBindings.of(callbacks, runner), lift);
        Bindings bindings = callbacks.root();
        ManualLoopClock time = new ManualLoopClock();

        bindings.update(time.clock());
        pulse(driver, bindings, time, Button.X);
        pulse(driver, bindings, time, Button.X);

        assertEquals(2, lift.homeTasks.size());
        assertNotSame(lift.homeTasks.get(0), lift.homeTasks.get(1));
        assertEquals(2, runner.queuedCount());
    }

    @Test
    public void eachFocusedBindingValidatesFirstAndIsOneShot() {
        GamepadDevice driver = new GamepadDevice(new Gamepad());
        BasicLiftHomeControls homeControls = new BasicLiftHomeControls(driver);
        BasicLiftControls liftControls = new BasicLiftControls(driver);
        BasicClawControls clawControls = new BasicClawControls(driver);
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();

        RecordingCallbackBindings homeCallbacks = new RecordingCallbackBindings();
        TaskBindings homeTasks = TaskBindings.of(homeCallbacks, new TaskRunner());
        expectNullPointer(() -> homeControls.bind(null, lift));
        expectNullPointer(() -> homeControls.bind(homeTasks, null));
        assertEquals(0, homeCallbacks.registrationAttempts());

        homeControls.bind(homeTasks, lift);
        assertEquals(1, homeCallbacks.successfulRegistrations());

        RecordingCallbackBindings homeRetry = new RecordingCallbackBindings();
        expectAlreadyBound(() -> homeControls.bind(
                TaskBindings.of(homeRetry, new TaskRunner()), lift));
        assertEquals(0, homeRetry.registrationAttempts());

        RecordingCallbackBindings liftCallbacks = new RecordingCallbackBindings();
        TaskBindings liftTasks = TaskBindings.of(liftCallbacks, new TaskRunner());

        expectNullPointer(() -> liftControls.bind(null, liftTasks, lift));
        expectNullPointer(() -> liftControls.bind(liftCallbacks, null, lift));
        expectNullPointer(() -> liftControls.bind(liftCallbacks, liftTasks, null));
        assertEquals(0, liftCallbacks.registrationAttempts());

        liftControls.bind(liftCallbacks, liftTasks, lift);
        assertEquals(4, liftCallbacks.successfulRegistrations());

        RecordingCallbackBindings liftRetry = new RecordingCallbackBindings();
        expectAlreadyBound(() -> liftControls.bind(
                liftRetry, TaskBindings.of(liftRetry, new TaskRunner()), lift));
        assertEquals(0, liftRetry.registrationAttempts());

        RecordingCallbackBindings clawCallbacks = new RecordingCallbackBindings();
        expectNullPointer(() -> clawControls.bind(null, claw));
        expectNullPointer(() -> clawControls.bind(clawCallbacks, null));
        assertEquals(0, clawCallbacks.registrationAttempts());

        clawControls.bind(clawCallbacks, claw);
        assertEquals(3, clawCallbacks.successfulRegistrations());

        RecordingCallbackBindings clawRetry = new RecordingCallbackBindings();
        expectAlreadyBound(() -> clawControls.bind(clawRetry, claw));
        assertEquals(0, clawRetry.registrationAttempts());
    }

    private enum Button {
        A,
        B,
        Y,
        X,
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT
    }

    private static void pulse(Gamepad gamepad,
                              Bindings bindings,
                              ManualLoopClock time,
                              Button button) {
        setButton(gamepad, button, true);
        bindings.update(time.nextCycle(0.02));
        setButton(gamepad, button, false);
        bindings.update(time.nextCycle(0.02));
    }

    private static void setButton(Gamepad gamepad, Button button, boolean value) {
        switch (button) {
            case A:
                gamepad.a = value;
                return;
            case B:
                gamepad.b = value;
                return;
            case Y:
                gamepad.y = value;
                return;
            case X:
                gamepad.x = value;
                return;
            case DPAD_UP:
                gamepad.dpad_up = value;
                return;
            case DPAD_DOWN:
                gamepad.dpad_down = value;
                return;
            case DPAD_LEFT:
                gamepad.dpad_left = value;
                return;
            default:
                throw new AssertionError("Unsupported button " + button);
        }
    }

    private static void expectNullPointer(Runnable action) {
        try {
            action.run();
            fail("Expected null argument to fail");
        } catch (NullPointerException expected) {
            // Expected.
        }
    }

    private static void expectAlreadyBound(Runnable action) {
        try {
            action.run();
            fail("Expected a repeated bind to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().toLowerCase().contains("bound"));
        }
    }

    private static final class RecordingLift implements BasicLift {
        private final List<Height> heightRequests = new ArrayList<Height>();
        private final List<Task> homeTasks = new ArrayList<Task>();

        @Override
        public void setHeight(Height height) {
            heightRequests.add(height);
        }

        @Override
        public Task moveTo(Height height) {
            return Tasks.runOnce(() -> setHeight(height));
        }

        @Override
        public Task home() {
            Task task = Tasks.runOnce(() -> heightRequests.add(Height.STOWED));
            homeTasks.add(task);
            return task;
        }

        @Override
        public Status status() {
            throw new AssertionError("RecordingLift status is not used by this controls test");
        }
    }

    private static final class RecordingClaw implements BasicClaw {
        private final List<State> stateRequests = new ArrayList<State>();

        @Override
        public void setState(State state) {
            stateRequests.add(state);
        }

        @Override
        public Task setStateTask(State state) {
            State requiredState = Objects.requireNonNull(state, "state");
            return Tasks.runOnce(() -> setState(requiredState));
        }

        @Override
        public Status status() {
            throw new AssertionError("RecordingClaw status is not used by this controls test");
        }
    }
}
