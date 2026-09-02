package edu.ftcsushi.robots.examples.reference.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.RecordingCallbackBindings;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncher;
import edu.ftcsushi.robots.examples.reference.capability.lift.ReferenceLift;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies one-attempt binding and deterministic launcher abort ordering. */
public final class ReferenceTeleOpControlsTest {

    @Test
    public void constructionOnlyBuildsSourcesAndBindSurfaceIsUnique() throws Exception {
        Constructor<?>[] constructors = ReferenceTeleOpControls.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertEquals(
                Arrays.asList(GamepadDevice.class),
                Arrays.asList(constructors[0].getParameterTypes()));

        Method bind = ReferenceTeleOpControls.class.getDeclaredMethod(
                "bind",
                CallbackBindings.class,
                TaskBindings.class,
                ReferenceLift.class,
                ReferenceLauncher.class);
        assertEquals(Void.TYPE, bind.getReturnType());
        assertEquals(1, declaredMethodCount("bind"));

        ReferenceTeleOpControls controls =
                new ReferenceTeleOpControls(new GamepadDevice(new Gamepad()));
        assertTrue(controls.driveSource() != null);
    }

    @Test
    public void bindValidatesEveryArgumentBeforeClaimingItsOneAttempt() {
        ReferenceTeleOpControls controls =
                new ReferenceTeleOpControls(new GamepadDevice(new Gamepad()));
        RecordingCallbackBindings callbacks = new RecordingCallbackBindings();
        TaskBindings tasks = TaskBindings.of(callbacks, new TaskRunner());
        RecordingLift lift = new RecordingLift();
        RecordingLauncher launcher = new RecordingLauncher();

        expectNullPointer(() -> controls.bind(null, tasks, lift, launcher));
        expectNullPointer(() -> controls.bind(callbacks, null, lift, launcher));
        expectNullPointer(() -> controls.bind(callbacks, tasks, null, launcher));
        expectNullPointer(() -> controls.bind(callbacks, tasks, lift, null));
        assertEquals(0, callbacks.registrationAttempts());

        controls.bind(callbacks, tasks, lift, launcher);
        assertEquals(6, callbacks.successfulRegistrations());
    }

    @Test
    public void secondOrPartiallyFailedBindCannotMutateAnotherGraph() {
        ReferenceTeleOpControls controls =
                new ReferenceTeleOpControls(new GamepadDevice(new Gamepad()));
        RecordingLift lift = new RecordingLift();
        RecordingLauncher launcher = new RecordingLauncher();
        RecordingCallbackBindings failingCallbacks = new RecordingCallbackBindings(5);
        TaskBindings failingTasks = TaskBindings.of(failingCallbacks, new TaskRunner());

        try {
            controls.bind(failingCallbacks, failingTasks, lift, launcher);
            fail("Expected injected callback registration failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("injected callback registration failure"));
        }
        assertEquals(5, failingCallbacks.registrationAttempts());
        assertEquals(4, failingCallbacks.successfulRegistrations());

        RecordingCallbackBindings retryCallbacks = new RecordingCallbackBindings();
        TaskBindings retryTasks = TaskBindings.of(retryCallbacks, new TaskRunner());
        expectAlreadyBound(() -> controls.bind(retryCallbacks, retryTasks, lift, launcher));
        assertEquals(0, retryCallbacks.registrationAttempts());
    }

    @Test
    public void simultaneousYAndBCreateLaunchBeforeSynchronousAbort() {
        Gamepad driver = new Gamepad();
        ReferenceTeleOpControls controls =
                new ReferenceTeleOpControls(new GamepadDevice(driver));
        RecordingCallbackBindings callbacks = new RecordingCallbackBindings();
        TaskRunner runner = new TaskRunner();
        TaskBindings tasks = TaskBindings.of(callbacks, runner);
        RecordingLift lift = new RecordingLift();
        RecordingLauncher launcher = new RecordingLauncher();
        controls.bind(callbacks, tasks, lift, launcher);
        Bindings root = callbacks.root();
        ManualLoopClock time = new ManualLoopClock();

        root.update(time.clock());
        driver.y = true;
        driver.b = true;
        root.update(time.nextCycle(0.02));

        assertEquals(Arrays.asList("launchOne", "abortLaunches"), launcher.events);
        assertEquals(1, runner.queuedCount());
    }

    private static int declaredMethodCount(String name) {
        int count = 0;
        for (Method method : ReferenceTeleOpControls.class.getDeclaredMethods()) {
            if (method.getName().equals(name)) {
                count++;
            }
        }
        return count;
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
            fail("Expected repeated bind to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage() != null);
            assertTrue(expected.getMessage().toLowerCase().contains("bind"));
        }
    }

    private static final class RecordingLift implements ReferenceLift {
        @Override
        public void setHeight(Height height) {
            // No mapping assertion is needed for this abort-order test.
        }

        @Override
        public Task moveTo(Height height) {
            return Tasks.noop();
        }

        @Override
        public Task home() {
            return Tasks.noop();
        }

        @Override
        public Status status() {
            throw new AssertionError("RecordingLift status is not used by this controls test");
        }
    }

    private static final class RecordingLauncher implements ReferenceLauncher {
        private final List<String> events = new ArrayList<String>();

        @Override
        public void setTargetVelocityTicksPerSec(double velocityTicksPerSec) {
            events.add("setTargetVelocityTicksPerSec");
        }

        @Override
        public void abortLaunches() {
            events.add("abortLaunches");
        }

        @Override
        public Task launchOne() {
            events.add("launchOne");
            return Tasks.noop();
        }

        @Override
        public Status status() {
            throw new AssertionError(
                    "RecordingLauncher status is not used by this controls test");
        }
    }
}
