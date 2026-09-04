package edu.ftcsushi.robots.examples.basicflywheel;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.RecordingCallbackBindings;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;

/** Maintainer contract for the isolated flywheel fixture's direct A/B control vocabulary. */
public final class BasicFlywheelControlsTest {

    @Test
    public void risingEdgesPublishCapturedCandidateAndZeroAndBindingIsOneShot() {
        BasicFlywheelProfile profile = BasicFlywheelProfile.current();
        double capturedCandidate = profile.candidateVelocityTicksPerSec;
        Gamepad operator = new Gamepad();
        RecordingFlywheel flywheel = new RecordingFlywheel();
        RecordingCallbackBindings callbacks = new RecordingCallbackBindings();
        BasicFlywheelControls controls = new BasicFlywheelControls(
                new GamepadDevice(operator), capturedCandidate);
        controls.bind(callbacks, flywheel);
        profile.candidateVelocityTicksPerSec = capturedCandidate + 50.0;

        Bindings bindings = callbacks.root();
        ManualLoopClock time = new ManualLoopClock();
        bindings.update(time.clock());

        operator.a = true;
        bindings.update(time.nextCycle(0.02));
        bindings.update(time.nextCycle(0.02)); // Held A does not retrigger onRise.
        operator.a = false;
        bindings.update(time.nextCycle(0.02));

        operator.b = true;
        bindings.update(time.nextCycle(0.02));
        bindings.update(time.nextCycle(0.02)); // Held B does not retrigger onRise.

        assertEquals(Arrays.asList(capturedCandidate, 0.0), flywheel.velocityRequests);
        assertEquals(2, callbacks.successfulRegistrations());

        RecordingCallbackBindings retry = new RecordingCallbackBindings();
        assertThrows(IllegalStateException.class, () -> controls.bind(retry, flywheel));
        assertEquals(0, retry.registrationAttempts());
    }

    private static final class RecordingFlywheel implements BasicFlywheel {
        private final List<Double> velocityRequests = new ArrayList<Double>();

        @Override
        public void setVelocityTicksPerSec(double velocityTicksPerSec) {
            velocityRequests.add(velocityTicksPerSec);
        }

        @Override
        public Task setVelocityTask(double velocityTicksPerSec) {
            throw new AssertionError("Feedback Tasks are not used by these direct controls");
        }

        @Override
        public Status status() {
            throw new AssertionError("Status is not used by these direct controls");
        }
    }
}
