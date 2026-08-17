package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetStatus;
import edu.ftcphoenix.fw.actuation.PlantTargets;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.ftc.input.GamepadDevice;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.input.binding.CallbackBindings;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.fw.testing.ManualLoopClock;
import edu.ftcphoenix.fw.testing.RecordingCallbackBindings;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the starter controls vocabulary and the intake's source-driven realization. */
public final class StarterIntakeAndControlsTest {

    @Test
    public void controlsConstructionBuildsSourcesWithoutRegisteringCallbacks() throws Exception {
        Constructor<?>[] constructors = StarterTeleOpControls.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertEquals(
                Arrays.asList(GamepadDevice.class),
                Arrays.asList(constructors[0].getParameterTypes()));

        Method bind = StarterTeleOpControls.class.getDeclaredMethod(
                "bind",
                CallbackBindings.class,
                StarterIntake.class);
        assertEquals(Void.TYPE, bind.getReturnType());
        assertEquals(1, declaredMethodCount(StarterTeleOpControls.class, "bind"));

        StarterTeleOpControls controls =
                new StarterTeleOpControls(new GamepadDevice(new Gamepad()));
        RecordingCallbackBindings callbackBindings = new RecordingCallbackBindings();

        assertTrue(controls.driveSource() != null);
        assertEquals(0, callbackBindings.registrationAttempts());
    }

    @Test
    public void bindMapsOnlyA_B_XToSemanticIntakeModes() {
        Gamepad driver = new Gamepad();
        RecordingIntake intake = new RecordingIntake();
        RecordingCallbackBindings callbackBindings = new RecordingCallbackBindings();
        StarterTeleOpControls controls =
                new StarterTeleOpControls(new GamepadDevice(driver));
        controls.bind(callbackBindings, intake);
        Bindings bindings = callbackBindings.root();
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(3, callbackBindings.successfulRegistrations());
        bindings.update(time.clock());
        pulse(driver, bindings, time, 'a');
        pulse(driver, bindings, time, 'b');
        pulse(driver, bindings, time, 'x');

        assertEquals(
                Arrays.asList(
                        StarterIntake.Mode.COLLECT,
                        StarterIntake.Mode.EJECT,
                        StarterIntake.Mode.STOPPED),
                intake.modeRequests);
        assertEquals(0, intake.taskRequests);
    }

    @Test
    public void bindValidatesArgumentsBeforeClaimingTheOneShotAttempt() {
        StarterTeleOpControls controls =
                new StarterTeleOpControls(new GamepadDevice(new Gamepad()));
        RecordingCallbackBindings callbackBindings = new RecordingCallbackBindings();
        RecordingIntake intake = new RecordingIntake();

        expectNullPointer(() -> controls.bind(null, intake));
        expectNullPointer(() -> controls.bind(callbackBindings, null));
        assertEquals(0, callbackBindings.registrationAttempts());

        controls.bind(callbackBindings, intake);
        assertEquals(3, callbackBindings.successfulRegistrations());
    }

    @Test
    public void secondBindFailsBeforeMutatingAnotherCallbackSurface() {
        StarterTeleOpControls controls =
                new StarterTeleOpControls(new GamepadDevice(new Gamepad()));
        RecordingIntake intake = new RecordingIntake();
        RecordingCallbackBindings first = new RecordingCallbackBindings();
        RecordingCallbackBindings second = new RecordingCallbackBindings();

        controls.bind(first, intake);
        expectAlreadyBound(() -> controls.bind(second, intake));

        assertEquals(3, first.successfulRegistrations());
        assertEquals(0, second.registrationAttempts());
    }

    @Test
    public void partialRegistrationFailureConsumesBindAndRetryAddsNothing() {
        StarterTeleOpControls controls =
                new StarterTeleOpControls(new GamepadDevice(new Gamepad()));
        RecordingIntake intake = new RecordingIntake();
        RecordingCallbackBindings failing = new RecordingCallbackBindings(2);

        try {
            controls.bind(failing, intake);
            fail("Expected injected registration failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("injected callback registration failure"));
        }
        assertEquals(2, failing.registrationAttempts());
        assertEquals(1, failing.successfulRegistrations());

        RecordingCallbackBindings retry = new RecordingCallbackBindings();
        expectAlreadyBound(() -> controls.bind(retry, intake));
        assertEquals(0, retry.registrationAttempts());
    }

    @Test
    public void mechanismMapsModesThroughOneCommandTargetAndSubmitsStop() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetFromNewCommand(0.0)
                .build();
        StarterIntakeMechanism intake = new StarterIntakeMechanism(plant);
        ManualLoopClock time = new ManualLoopClock();

        assertMode(intake, time.clock(), StarterIntake.Mode.STOPPED, 0.0, output);

        intake.setMode(StarterIntake.Mode.COLLECT);
        assertMode(intake, time.nextCycle(0.02), StarterIntake.Mode.COLLECT, 0.20, output);

        intake.setMode(StarterIntake.Mode.EJECT);
        assertMode(intake, time.nextCycle(0.02), StarterIntake.Mode.EJECT, -0.20, output);

        intake.setMode(StarterIntake.Mode.STOPPED);
        assertMode(intake, time.nextCycle(0.02), StarterIntake.Mode.STOPPED, 0.0, output);

        intake.setMode(StarterIntake.Mode.EJECT);
        intake.update(time.nextCycle(0.02));
        intake.stop();

        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commandedPower, 0.0);
        assertEquals(-0.20, plant.commandTarget().get(), 0.0);
        assertEquals(StarterIntake.Mode.EJECT, intake.status().mode());

        intake.setMode(StarterIntake.Mode.COLLECT);
        intake.update(time.nextCycle(0.02));
        assertEquals(0.0, output.commandedPower, 0.0);
        assertEquals(StarterIntake.Mode.COLLECT, intake.status().mode());
    }

    @Test
    public void stopDoesNotRewriteTheGraphOwnedCommand() {
        ScalarTarget failingTarget = new ScalarTarget() {
            @Override
            public void set(double value) {
                throw new IllegalStateException("request write failed");
            }

            @Override
            public double get() {
                return 0.20;
            }
        };
        RecordingPowerOutput output = new RecordingPowerOutput();
        StarterIntakeMechanism intake = new StarterIntakeMechanism(
                Plants.fromOutputs()
                        .power(output)
                        .targetFromResolver(PlantTargets.exact(failingTarget))
                        .build());

        intake.stop();

        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commandedPower, 0.0);
    }

    @Test
    public void hardwareNeutralSeamRejectsPlantWithoutCommandTarget() {
        Plant plant = Plants.fromOutputs()
                .power(new RecordingPowerOutput())
                .targetFromResolver(PlantTargets.exact(0.0))
                .build();

        try {
            new StarterIntakeMechanism(plant);
            fail("Expected Plant without command target to fail");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("command target"));
        }
    }

    @Test
    public void hardwareNeutralSeamRejectsClaimedButMissingCommandTarget() {
        Plant plant = new Plant() {
            @Override
            public void update(edu.ftcphoenix.fw.core.time.LoopClock clock) {
            }

            @Override
            public double getRequestedTarget() {
                return 0.0;
            }

            @Override
            public double getAppliedTarget() {
                return 0.0;
            }

            @Override
            public PlantTargetStatus getTargetStatus() {
                return PlantTargetStatus.ACCEPTED;
            }

            @Override
            public boolean hasCommandTarget() {
                return true;
            }

            @Override
            public ScalarTarget commandTarget() {
                return null;
            }

            @Override
            public void stop() {
            }
        };

        try {
            new StarterIntakeMechanism(plant);
            fail("Expected null command target to fail");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("plant.commandTarget()"));
        }
    }

    @Test
    public void collectTasksAreFreshCompleteOnTimeAndCancelToZero() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        StarterIntakeMechanism intake = new StarterIntakeMechanism(
                Plants.fromOutputs()
                        .power(output)
                        .targetFromNewCommand(0.0)
                        .build());
        ManualLoopClock time = new ManualLoopClock();

        Task first = intake.collectForSeconds(0.50);
        Task second = intake.collectForSeconds(0.50);
        assertNotSame(first, second);

        first.start(time.clock());
        intake.update(time.clock());
        assertEquals(0.20, output.commandedPower, 0.0);
        assertFalse(first.isComplete());

        first.update(time.nextCycle(0.49));
        intake.update(time.clock());
        assertFalse(first.isComplete());
        assertEquals(0.20, output.commandedPower, 0.0);

        first.update(time.nextCycle(0.01));
        intake.update(time.clock());
        assertTrue(first.isComplete());
        assertEquals(TaskOutcome.SUCCESS, first.getOutcome());
        assertEquals(0.0, output.commandedPower, 0.0);
        assertEquals(StarterIntake.Mode.STOPPED, intake.status().mode());

        second.start(time.nextCycle(0.02));
        intake.update(time.clock());
        assertEquals(0.20, output.commandedPower, 0.0);

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
                              Bindings bindings,
                              ManualLoopClock time,
                              char button) {
        setButton(driver, button, true);
        bindings.update(time.nextCycle(0.02));
        setButton(driver, button, false);
        bindings.update(time.nextCycle(0.02));
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

    private static void expectNullPointer(Runnable action) {
        try {
            action.run();
            fail("Expected null argument to fail");
        } catch (NullPointerException expected) {
            // Expected.
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

    private static void expectAlreadyBound(Runnable action) {
        try {
            action.run();
            fail("Expected repeated bind to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage() != null);
            assertTrue(expected.getMessage().toLowerCase().contains("bind"));
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
