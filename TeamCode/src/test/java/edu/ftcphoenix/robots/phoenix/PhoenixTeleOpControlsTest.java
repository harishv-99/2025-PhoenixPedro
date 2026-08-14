package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.ftc.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.input.binding.CallbackBindings;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.testing.ManualLoopClock;
import edu.ftcphoenix.fw.testing.RecordingCallbackBindings;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Protects Phoenix's intentional cross-kind control declaration order. */
public final class PhoenixTeleOpControlsTest {

    @Test
    public void controlsExposeRegistrationButNoIndependentBindingLifecycle() {
        for (java.lang.reflect.Method method : PhoenixTeleOpControls.class.getDeclaredMethods()) {
            assertFalse("controls must not own an update heartbeat", method.getName().equals("update"));
            assertFalse("controls must not own binding cleanup", method.getName().equals("clear"));
        }
    }

    @Test
    public void controlsConstructionBuildsSourcesWithoutRegisteringCallbacks() throws Exception {
        Constructor<?>[] constructors = PhoenixTeleOpControls.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertEquals(
                Arrays.asList(Gamepads.class, PhoenixProfile.TeleOpControlsConfig.class),
                Arrays.asList(constructors[0].getParameterTypes()));

        Method bind = PhoenixTeleOpControls.class.getDeclaredMethod(
                "bind",
                CallbackBindings.class,
                PhoenixCapabilities.class);
        assertEquals(Void.TYPE, bind.getReturnType());
        assertEquals(1, declaredMethodCount(PhoenixTeleOpControls.class, "bind"));

        PhoenixTeleOpControls controls = new PhoenixTeleOpControls(
                Gamepads.create(new Gamepad(), new Gamepad()),
                new PhoenixProfile.TeleOpControlsConfig());
        RecordingCallbackBindings callbackBindings = new RecordingCallbackBindings();

        assertTrue(controls.manualDriveSource() != null);
        assertTrue(controls.autoAimEnabledSource() != null);
        assertEquals(0, callbackBindings.registrationAttempts());
    }

    @Test
    public void simultaneousSuggestedVelocityCapturePrecedesStudentNudge() {
        Gamepad driver = new Gamepad();
        Gamepad operator = new Gamepad();
        PhoenixProfile.TeleOpControlsConfig config = new PhoenixProfile.TeleOpControlsConfig();
        config.selectedVelocityStepNative = 40.0;

        RecordingCallbackBindings callbackBindings = new RecordingCallbackBindings();
        PhoenixTeleOpControls controls = new PhoenixTeleOpControls(
                Gamepads.create(driver, operator), config);
        RecordingScoring scoring = new RecordingScoring(2_000.0);
        controls.bind(callbackBindings, capabilities(scoring));
        Bindings bindingRoot = callbackBindings.root();

        ManualLoopClock manualClock = new ManualLoopClock();
        update(bindingRoot, manualClock); // Establish edge and mirror baselines.
        scoring.calls.clear();

        operator.left_bumper = true;
        operator.dpad_up = true;
        update(bindingRoot, manualClock);

        assertEquals(Arrays.asList("capture", "adjust"), scoring.calls);
        assertEquals(2_040.0, scoring.selectedVelocityNative, 0.0);
        assertEquals(6, callbackBindings.successfulRegistrations());
    }

    @Test
    public void bindValidatesArgumentsBeforeClaimingTheOneShotAttempt() {
        PhoenixTeleOpControls controls = new PhoenixTeleOpControls(
                Gamepads.create(new Gamepad(), new Gamepad()),
                new PhoenixProfile.TeleOpControlsConfig());
        RecordingCallbackBindings callbackBindings = new RecordingCallbackBindings();
        PhoenixCapabilities capabilities = capabilities(new RecordingScoring(2_000.0));

        expectNullPointer(() -> controls.bind(null, capabilities));
        expectNullPointer(() -> controls.bind(callbackBindings, null));
        assertEquals(0, callbackBindings.registrationAttempts());

        controls.bind(callbackBindings, capabilities);
        assertEquals(6, callbackBindings.successfulRegistrations());
    }

    @Test
    public void secondBindFailsBeforeMutatingAnotherCallbackSurface() {
        PhoenixTeleOpControls controls = new PhoenixTeleOpControls(
                Gamepads.create(new Gamepad(), new Gamepad()),
                new PhoenixProfile.TeleOpControlsConfig());
        PhoenixCapabilities capabilities = capabilities(new RecordingScoring(2_000.0));
        RecordingCallbackBindings first = new RecordingCallbackBindings();
        RecordingCallbackBindings second = new RecordingCallbackBindings();

        controls.bind(first, capabilities);
        expectAlreadyBound(() -> controls.bind(second, capabilities));

        assertEquals(6, first.successfulRegistrations());
        assertEquals(0, second.registrationAttempts());
    }

    @Test
    public void partialRegistrationFailureConsumesBindAndRetryAddsNothing() {
        PhoenixTeleOpControls controls = new PhoenixTeleOpControls(
                Gamepads.create(new Gamepad(), new Gamepad()),
                new PhoenixProfile.TeleOpControlsConfig());
        PhoenixCapabilities capabilities = capabilities(new RecordingScoring(2_000.0));
        RecordingCallbackBindings failing = new RecordingCallbackBindings(3);

        try {
            controls.bind(failing, capabilities);
            fail("Expected injected registration failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("injected callback registration failure"));
        }
        assertEquals(3, failing.registrationAttempts());
        assertEquals(2, failing.successfulRegistrations());

        RecordingCallbackBindings retry = new RecordingCallbackBindings();
        expectAlreadyBound(() -> controls.bind(retry, capabilities));
        assertEquals(0, retry.registrationAttempts());
    }

    private static void update(Bindings bindingRoot, ManualLoopClock manualClock) {
        manualClock.nextCycle(0.02);
        bindingRoot.update(manualClock.clock());
    }

    private static PhoenixCapabilities capabilities(RecordingScoring scoring) {
        return new PhoenixCapabilities(scoring, new UnusedTargeting());
    }

    private static int declaredMethodCount(Class<?> type, String name) {
        int count = 0;
        for (Method method : type.getDeclaredMethods()) {
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

    private static final class RecordingScoring implements PhoenixCapabilities.Scoring {
        private final double suggestedVelocityNative;
        private final List<String> calls = new ArrayList<>();
        private double selectedVelocityNative;

        private RecordingScoring(double suggestedVelocityNative) {
            this.suggestedVelocityNative = suggestedVelocityNative;
        }

        @Override
        public void setIntakeEnabled(boolean enabled) {
        }

        @Override
        public void setFlywheelEnabled(boolean enabled) {
        }

        @Override
        public void setShootingEnabled(boolean enabled) {
        }

        @Override
        public void setEjectEnabled(boolean enabled) {
        }

        @Override
        public void requestSingleShot() {
        }

        @Override
        public void requestShots(int shotCount) {
        }

        @Override
        public void cancelTransientActions() {
        }

        @Override
        public void setSelectedVelocityNative(double velocityNative) {
            selectedVelocityNative = velocityNative;
        }

        @Override
        public void adjustSelectedVelocityNative(double deltaNative) {
            calls.add("adjust");
            selectedVelocityNative += deltaNative;
        }

        @Override
        public void captureSuggestedShotVelocity() {
            calls.add("capture");
            selectedVelocityNative = suggestedVelocityNative;
        }

        @Override
        public boolean hasPendingShots() {
            return false;
        }

        @Override
        public PhoenixCapabilities.ScoringStatus status() {
            return null;
        }
    }

    private static final class UnusedTargeting implements PhoenixCapabilities.Targeting {
        @Override
        public PhoenixCapabilities.TargetingStatus status() {
            return null;
        }

        @Override
        public Task aimTask(DriveCommandSink driveSink, DriveGuidanceTask.Config config) {
            return null;
        }
    }
}
