package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

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
    public void simultaneousSuggestedVelocityCapturePrecedesStudentNudge() {
        Gamepad driver = new Gamepad();
        Gamepad operator = new Gamepad();
        PhoenixProfile.TeleOpControlsConfig config = new PhoenixProfile.TeleOpControlsConfig();
        config.selectedVelocityStepNative = 40.0;

        Bindings bindingRoot = new Bindings();
        PhoenixTeleOpControls controls = new PhoenixTeleOpControls(
                bindingRoot, Gamepads.create(driver, operator), config);
        RecordingScoring scoring = new RecordingScoring(2_000.0);
        controls.bind(new PhoenixCapabilities(scoring, new UnusedTargeting()));

        ManualLoopClock manualClock = new ManualLoopClock();
        update(bindingRoot, manualClock); // Establish edge and mirror baselines.
        scoring.calls.clear();

        operator.left_bumper = true;
        operator.dpad_up = true;
        update(bindingRoot, manualClock);

        assertEquals(Arrays.asList("capture", "adjust"), scoring.calls);
        assertEquals(2_040.0, scoring.selectedVelocityNative, 0.0);
    }

    private static void update(Bindings bindingRoot, ManualLoopClock manualClock) {
        manualClock.nextCycle(0.02);
        bindingRoot.update(manualClock.clock());
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
        public ScoringPath.Status status() {
            return null;
        }
    }

    private static final class UnusedTargeting implements PhoenixCapabilities.Targeting {
        @Override
        public ScoringTargeting.Status status() {
            return null;
        }

        @Override
        public Task aimTask(DriveCommandSink driveSink, DriveGuidanceTask.Config config) {
            return null;
        }
    }
}
