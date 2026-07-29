package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargets;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.input.GamepadDevice;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the starter controls vocabulary and the intake's source-driven realization. */
public final class StarterIntakeAndControlsTest {

    @Test
    public void controlsMapOnlyA_B_XToSemanticIntakeModes() {
        Gamepad driver = new Gamepad();
        RecordingIntake intake = new RecordingIntake();
        StarterTeleOpControls controls = new StarterTeleOpControls(
                new GamepadDevice(driver),
                intake);
        ManualLoopClock time = new ManualLoopClock();

        controls.update(time.clock());
        pulse(driver, controls, time, 'a');
        pulse(driver, controls, time, 'b');
        pulse(driver, controls, time, 'x');

        assertEquals(
                Arrays.asList(
                        StarterIntake.Mode.COLLECT,
                        StarterIntake.Mode.EJECT,
                        StarterIntake.Mode.STOPPED),
                intake.modeRequests);
        assertEquals(0, intake.taskRequests);
    }

    @Test
    public void mechanismMapsModesThroughOneCommandTargetAndSubmitsStop() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        Plant plant = Plants.power(output, ScalarTarget.create(0.0));
        StarterIntakeMechanism intake =
                new StarterIntakeMechanism(plant, 0.65, -0.45);
        ManualLoopClock time = new ManualLoopClock();

        assertMode(intake, time.clock(), StarterIntake.Mode.STOPPED, 0.0, output);

        intake.setMode(StarterIntake.Mode.COLLECT);
        assertMode(intake, time.nextCycle(0.02), StarterIntake.Mode.COLLECT, 0.65, output);

        intake.setMode(StarterIntake.Mode.EJECT);
        assertMode(intake, time.nextCycle(0.02), StarterIntake.Mode.EJECT, -0.45, output);

        intake.setMode(StarterIntake.Mode.STOPPED);
        assertMode(intake, time.nextCycle(0.02), StarterIntake.Mode.STOPPED, 0.0, output);

        intake.setMode(StarterIntake.Mode.EJECT);
        intake.update(time.nextCycle(0.02));
        intake.stop();

        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commandedPower, 0.0);
        assertEquals(StarterIntake.Mode.STOPPED, intake.status().mode());
    }

    @Test
    public void stopStillStopsPlantWhenClearingTheRetainedRequestFails() {
        ScalarTarget failingTarget = new ScalarTarget() {
            @Override
            public void set(double value) {
                throw new IllegalStateException("request write failed");
            }

            @Override
            public double get() {
                return 0.65;
            }
        };
        RecordingPowerOutput output = new RecordingPowerOutput();
        StarterIntakeMechanism intake = new StarterIntakeMechanism(
                Plants.power(output, failingTarget),
                0.65,
                -0.45);

        try {
            intake.stop();
            fail("Expected retained-request failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("request write failed"));
        }

        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commandedPower, 0.0);
    }

    @Test
    public void hardwareNeutralSeamRejectsAmbiguousActionPowers() {
        assertInvalidActionPowers(0.0, -0.45);
        assertInvalidActionPowers(0.65, 0.65);
        assertInvalidActionPowers(Double.NaN, -0.45);
        assertInvalidActionPowers(0.65, -1.01);
    }

    @Test
    public void hardwareNeutralSeamRejectsPlantWithoutCommandTarget() {
        Plant plant = Plants.power(
                new RecordingPowerOutput(),
                PlantTargets.exact(0.0));

        try {
            new StarterIntakeMechanism(
                    plant,
                    0.65,
                    -0.45);
            fail("Expected Plant without command target to fail");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("command target"));
        }
    }

    @Test
    public void collectTasksAreFreshCompleteOnTimeAndCancelToZero() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        StarterIntakeMechanism intake = new StarterIntakeMechanism(
                Plants.power(output, ScalarTarget.create(0.0)),
                0.70,
                -0.50);
        ManualLoopClock time = new ManualLoopClock();

        Task first = intake.collectForSeconds(0.50);
        Task second = intake.collectForSeconds(0.50);
        assertNotSame(first, second);

        first.start(time.clock());
        intake.update(time.clock());
        assertEquals(0.70, output.commandedPower, 0.0);
        assertFalse(first.isComplete());

        first.update(time.nextCycle(0.49));
        intake.update(time.clock());
        assertFalse(first.isComplete());
        assertEquals(0.70, output.commandedPower, 0.0);

        first.update(time.nextCycle(0.01));
        intake.update(time.clock());
        assertTrue(first.isComplete());
        assertEquals(TaskOutcome.SUCCESS, first.getOutcome());
        assertEquals(0.0, output.commandedPower, 0.0);
        assertEquals(StarterIntake.Mode.STOPPED, intake.status().mode());

        second.start(time.nextCycle(0.02));
        intake.update(time.clock());
        assertEquals(0.70, output.commandedPower, 0.0);

        second.cancel();
        second.cancel();
        intake.update(time.nextCycle(0.02));
        assertTrue(second.isComplete());
        assertEquals(TaskOutcome.CANCELLED, second.getOutcome());
        assertEquals(0.0, output.commandedPower, 0.0);
        assertEquals(StarterIntake.Mode.STOPPED, intake.status().mode());

        assertIllegalDuration(intake, 0.0);
        assertIllegalDuration(intake, Double.NaN);
    }

    private static void assertMode(StarterIntakeMechanism intake,
                                   edu.ftcphoenix.fw.core.time.LoopClock clock,
                                   StarterIntake.Mode expectedMode,
                                   double expectedPower,
                                   RecordingPowerOutput output) {
        intake.update(clock);
        StarterIntake.Status status = intake.status();
        assertEquals(expectedMode, status.mode());
        assertEquals(expectedPower, status.appliedTargetPower(), 0.0);
        assertEquals(expectedPower, output.commandedPower, 0.0);
    }

    private static void pulse(Gamepad driver,
                              StarterTeleOpControls controls,
                              ManualLoopClock time,
                              char button) {
        setButton(driver, button, true);
        controls.update(time.nextCycle(0.02));
        setButton(driver, button, false);
        controls.update(time.nextCycle(0.02));
    }

    private static void setButton(Gamepad driver, char button, boolean value) {
        if (button == 'a') {
            driver.a = value;
        } else if (button == 'b') {
            driver.b = value;
        } else if (button == 'x') {
            driver.x = value;
        } else {
            throw new AssertionError("unsupported test button " + button);
        }
    }

    private static void assertIllegalDuration(StarterIntake intake, double durationSec) {
        try {
            intake.collectForSeconds(durationSec);
            fail("Expected invalid collect duration to fail");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("durationSec"));
        }
    }

    private static void assertInvalidActionPowers(double collectPower, double ejectPower) {
        try {
            new StarterIntakeMechanism(
                    Plants.power(new RecordingPowerOutput(), ScalarTarget.create(0.0)),
                    collectPower,
                    ejectPower);
            fail("Expected invalid action powers to fail");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("Power"));
        }
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

    private static final class RecordingPowerOutput implements PowerOutput {
        private double commandedPower;
        private int stopCalls;

        @Override
        public void setPower(double power) {
            commandedPower = power;
        }

        @Override
        public double getCommandedPower() {
            return commandedPower;
        }

        @Override
        public void stop() {
            stopCalls++;
            commandedPower = 0.0;
        }
    }
}
