package edu.ftcsushi.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcsushi.fw.ftc.input.Gamepads;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.RecordingCallbackBindings;

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
                Arrays.asList(Gamepads.class, PhoenixTeleOpControls.Config.class),
                Arrays.asList(constructors[0].getParameterTypes()));

        Method bind = PhoenixTeleOpControls.class.getDeclaredMethod(
                "bind",
                CallbackBindings.class,
                PhoenixCapabilities.class);
        assertEquals(Void.TYPE, bind.getReturnType());
        assertEquals(1, declaredMethodCount(PhoenixTeleOpControls.class, "bind"));

        PhoenixTeleOpControls controls = new PhoenixTeleOpControls(
                Gamepads.create(new Gamepad(), new Gamepad()),
                PhoenixTeleOpControls.Config.defaults());
        RecordingCallbackBindings callbackBindings = new RecordingCallbackBindings();

        assertTrue(controls.manualDriveSource() != null);
        assertTrue(controls.autoAimEnabledSource() != null);
        assertEquals(0, callbackBindings.registrationAttempts());
    }

    @Test
    public void simultaneousSuggestedVelocityCapturePrecedesStudentNudge() {
        Gamepad driver = new Gamepad();
        Gamepad operator = new Gamepad();
        PhoenixTeleOpControls.Config config = PhoenixTeleOpControls.Config.defaults();
        config.selectedVelocityStepNative = 40.0;

        RecordingCallbackBindings callbackBindings = new RecordingCallbackBindings();
        PhoenixTeleOpControls controls = new PhoenixTeleOpControls(
                Gamepads.create(driver, operator), config);
        config.selectedVelocityStepNative = 400.0;
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
                PhoenixTeleOpControls.Config.defaults());
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
                PhoenixTeleOpControls.Config.defaults());
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
                PhoenixTeleOpControls.Config.defaults());
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

    @Test
    public void manualDriveIsRequiredBeforeOuterScalarValidation() {
        PhoenixTeleOpControls.Config config = PhoenixTeleOpControls.Config.defaults();
        config.manualDrive = null;
        config.slowTranslateScale = Double.NaN;

        try {
            createControls(config);
            fail("Expected missing manual-drive draft to fail");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("PhoenixTeleOpControls.Config.manualDrive"));
        }
    }

    @Test
    public void outerScalarValidationPrecedesNestedFrameworkValidation() {
        PhoenixTeleOpControls.Config config = PhoenixTeleOpControls.Config.defaults();
        config.manualDrive.deadband = Double.NaN;
        config.slowTranslateScale = Double.POSITIVE_INFINITY;

        expectInvalidConfig(config, "slowTranslateScale");
    }

    @Test
    public void outerScalarsRejectEveryDocumentedInvalidDomain() {
        PhoenixTeleOpControls.Config slowTranslation = PhoenixTeleOpControls.Config.defaults();
        slowTranslation.slowTranslateScale = -0.01;
        expectInvalidConfig(slowTranslation, "slowTranslateScale");

        PhoenixTeleOpControls.Config axialRate = PhoenixTeleOpControls.Config.defaults();
        axialRate.maxAxialRatePerSec = 0.0;
        expectInvalidConfig(axialRate, "maxAxialRatePerSec");

        PhoenixTeleOpControls.Config lateralRate = PhoenixTeleOpControls.Config.defaults();
        lateralRate.maxLateralRatePerSec = Double.NaN;
        expectInvalidConfig(lateralRate, "maxLateralRatePerSec");

        PhoenixTeleOpControls.Config omegaRate = PhoenixTeleOpControls.Config.defaults();
        omegaRate.maxOmegaRatePerSec = Double.POSITIVE_INFINITY;
        expectInvalidConfig(omegaRate, "maxOmegaRatePerSec");

        PhoenixTeleOpControls.Config slowOmega = PhoenixTeleOpControls.Config.defaults();
        slowOmega.slowOmegaScale = 1.01;
        expectInvalidConfig(slowOmega, "slowOmegaScale");

        PhoenixTeleOpControls.Config velocityStep = PhoenixTeleOpControls.Config.defaults();
        velocityStep.selectedVelocityStepNative = 0.0;
        expectInvalidConfig(velocityStep, "selectedVelocityStepNative");
    }

    @Test
    public void validOuterScalarsDelegateNestedValidationToGamepadDriveSource() {
        PhoenixTeleOpControls.Config config = PhoenixTeleOpControls.Config.defaults();
        config.manualDrive.deadband = Double.NaN;

        try {
            createControls(config);
            fail("Expected invalid nested manual-drive config");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("GamepadDriveSource.Config.deadband"));
        }
    }

    @Test
    public void outerScalarsAcceptTheirInclusiveAndPositiveBoundaries() {
        PhoenixTeleOpControls.Config config = PhoenixTeleOpControls.Config.defaults();
        config.slowTranslateScale = 0.0;
        config.maxAxialRatePerSec = Double.MIN_VALUE;
        config.maxLateralRatePerSec = Double.MIN_VALUE;
        config.maxOmegaRatePerSec = Double.MIN_VALUE;
        config.slowOmegaScale = 1.0;
        config.selectedVelocityStepNative = Double.MIN_VALUE;

        assertTrue(createControls(config) != null);
    }

    private static void update(Bindings bindingRoot, ManualLoopClock manualClock) {
        manualClock.nextCycle(0.02);
        bindingRoot.update(manualClock.clock());
    }

    private static PhoenixCapabilities capabilities(RecordingScoring scoring) {
        return new PhoenixCapabilities(scoring, new UnusedTargeting());
    }

    private static PhoenixTeleOpControls createControls(PhoenixTeleOpControls.Config config) {
        return new PhoenixTeleOpControls(
                Gamepads.create(new Gamepad(), new Gamepad()),
                config
        );
    }

    private static void expectInvalidConfig(
            PhoenixTeleOpControls.Config config,
            String expectedField
    ) {
        try {
            createControls(config);
            fail("Expected invalid controls config field " + expectedField);
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains(
                    "PhoenixTeleOpControls.Config." + expectedField
            ));
        }
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
