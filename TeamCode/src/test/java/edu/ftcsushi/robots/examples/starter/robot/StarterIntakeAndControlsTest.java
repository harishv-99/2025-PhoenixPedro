package edu.ftcsushi.robots.examples.starter.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.RecordingCallbackBindings;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntakeMechanism;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the starter controls vocabulary and the intake's source-driven realization. */
public final class StarterIntakeAndControlsTest {

    @Test
    public void statusIsADelegateOnlyDomainView() {
        Field[] fields = StarterIntake.Status.class.getDeclaredFields();
        assertEquals(1, fields.length);
        assertEquals(
                edu.ftcsushi.fw.actuation.SemanticScalarSnapshot.class,
                fields[0].getType());
        assertTrue(Modifier.isPrivate(fields[0].getModifiers()));
        assertTrue(Modifier.isFinal(fields[0].getModifiers()));

        Constructor<?>[] constructors = StarterIntake.Status.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPublic(constructors[0].getModifiers()));
        assertEquals(1, constructors[0].getParameterTypes().length);
        assertEquals(
                edu.ftcsushi.fw.actuation.SemanticScalarSnapshot.class,
                constructors[0].getParameterTypes()[0]);

        try {
            new StarterIntake.Status(null);
            fail("Expected a null intake status delegate to fail");
        } catch (NullPointerException expected) {
            assertEquals("delegate", expected.getMessage());
        }
    }

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
    public void mechanismMapsSemanticModesForwardAndStagesAppliedPower() {
        StarterIntakeMechanism.Config config = StarterIntakeMechanism.Config.defaults();
        config.collectPower = 0.65;
        config.ejectPower = -0.45;
        FtcTestHardware hardwareMap =
                new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardwareMap.addMotor(config.motorName);
        StarterIntakeMechanism intake = new StarterIntakeMechanism(hardwareMap, config);
        ManualLoopClock time = new ManualLoopClock();

        assertStatus(intake, StarterIntake.Mode.STOPPED, 0.0, 0.0);
        assertEquals(0, motor.powerWrites());

        intake.setMode(StarterIntake.Mode.COLLECT);
        assertStatus(intake, StarterIntake.Mode.COLLECT, 0.65, 0.0);
        assertEquals(0, motor.powerWrites());
        intake.update(time.clock());
        assertStatus(intake, StarterIntake.Mode.COLLECT, 0.65, 0.65);
        assertEquals(0.65, motor.power(), 0.0);

        intake.setMode(StarterIntake.Mode.EJECT);
        assertStatus(intake, StarterIntake.Mode.EJECT, -0.45, 0.65);
        intake.update(time.nextCycle(0.02));
        assertStatus(intake, StarterIntake.Mode.EJECT, -0.45, -0.45);
        assertEquals(-0.45, motor.power(), 0.0);

        expectNullPointer(() -> intake.setMode(null));
        assertStatus(intake, StarterIntake.Mode.EJECT, -0.45, -0.45);

        intake.setMode(StarterIntake.Mode.STOPPED);
        assertStatus(intake, StarterIntake.Mode.STOPPED, 0.0, -0.45);
        intake.update(time.nextCycle(0.02));
        assertStatus(intake, StarterIntake.Mode.STOPPED, 0.0, 0.0);
        assertEquals(0.0, motor.power(), 0.0);
    }

    @Test
    public void equalPowersDoNotEraseTheRequestedMode() {
        StarterIntakeMechanism.Config config = StarterIntakeMechanism.Config.defaults();
        config.collectPower = 0.40;
        config.ejectPower = 0.40;
        FtcTestHardware hardwareMap =
                new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardwareMap.addMotor(config.motorName);
        StarterIntakeMechanism intake = new StarterIntakeMechanism(hardwareMap, config);
        ManualLoopClock time = new ManualLoopClock();

        intake.setMode(StarterIntake.Mode.COLLECT);
        intake.update(time.clock());
        assertStatus(intake, StarterIntake.Mode.COLLECT, 0.40, 0.40);
        assertEquals(0.40, motor.power(), 0.0);

        intake.setMode(StarterIntake.Mode.EJECT);
        assertEquals(StarterIntake.Mode.EJECT, intake.status().mode());
        intake.update(time.nextCycle(0.02));
        assertStatus(intake, StarterIntake.Mode.EJECT, 0.40, 0.40);
        assertEquals(0.40, motor.power(), 0.0);
    }

    @Test
    public void terminalStopZerosHardwareWithoutRewritingTheRequestedMode() {
        StarterIntakeMechanism.Config config = StarterIntakeMechanism.Config.defaults();
        config.collectPower = 0.65;
        config.ejectPower = -0.45;
        FtcTestHardware hardwareMap =
                new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardwareMap.addMotor(config.motorName);
        StarterIntakeMechanism intake = new StarterIntakeMechanism(hardwareMap, config);
        ManualLoopClock time = new ManualLoopClock();

        intake.setMode(StarterIntake.Mode.EJECT);
        intake.update(time.clock());
        intake.stop();

        assertEquals(0.0, motor.power(), 0.0);
        assertStatus(intake, StarterIntake.Mode.EJECT, -0.45, 0.0);

        intake.setMode(StarterIntake.Mode.COLLECT);
        intake.update(time.nextCycle(0.02));
        assertEquals(0.0, motor.power(), 0.0);
        assertStatus(intake, StarterIntake.Mode.COLLECT, 0.65, 0.0);
    }

    @Test
    public void collectTasksAreFreshReassertAndCompleteAtTheirOwnBoundary() {
        StarterIntakeMechanism.Config config = StarterIntakeMechanism.Config.defaults();
        config.collectPower = 0.65;
        config.ejectPower = -0.45;
        FtcTestHardware hardwareMap =
                new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardwareMap.addMotor(config.motorName);
        StarterIntakeMechanism intake = new StarterIntakeMechanism(hardwareMap, config);
        ManualLoopClock time = new ManualLoopClock();

        Task first = intake.collectForSeconds(0.50);
        Task second = intake.collectForSeconds(0.50);
        assertNotSame(first, second);

        time.nextCycle(37.0);
        first.start(time.clock());
        assertStatus(intake, StarterIntake.Mode.COLLECT, 0.65, 0.0);
        intake.update(time.clock());
        assertEquals(0.65, motor.power(), 0.0);
        assertFalse(first.isComplete());

        first.update(time.nextCycle(0.49));
        intake.update(time.clock());
        assertFalse(first.isComplete());
        assertEquals(0.65, motor.power(), 0.0);

        intake.setMode(StarterIntake.Mode.EJECT);
        assertStatus(intake, StarterIntake.Mode.EJECT, -0.45, 0.65);
        first.update(time.clock());
        assertStatus(intake, StarterIntake.Mode.COLLECT, 0.65, 0.65);

        first.update(time.nextCycle(0.01));
        assertTrue(first.isComplete());
        assertEquals(TaskOutcome.SUCCESS, first.getOutcome());
        assertStatus(intake, StarterIntake.Mode.STOPPED, 0.0, 0.65);
        intake.update(time.clock());
        assertStatus(intake, StarterIntake.Mode.STOPPED, 0.0, 0.0);
        assertEquals(0.0, motor.power(), 0.0);

        second.start(time.nextCycle(0.02));
        intake.update(time.clock());
        assertEquals(0.65, motor.power(), 0.0);
    }

    @Test
    public void collectTaskCancellationIsActiveOnlyIdempotentAndSingleUse() {
        StarterIntakeMechanism.Config config = StarterIntakeMechanism.Config.defaults();
        config.collectPower = 0.65;
        config.ejectPower = -0.45;
        FtcTestHardware hardwareMap =
                new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardwareMap.addMotor(config.motorName);
        StarterIntakeMechanism intake = new StarterIntakeMechanism(hardwareMap, config);
        ManualLoopClock time = new ManualLoopClock();
        Task task = intake.collectForSeconds(0.50);

        intake.setMode(StarterIntake.Mode.EJECT);
        task.cancel();
        assertFalse(task.isComplete());
        assertEquals(StarterIntake.Mode.EJECT, intake.status().mode());
        expectUpdateBeforeStart(() -> task.update(time.clock()));

        task.start(time.clock());
        intake.update(time.clock());
        assertEquals(0.65, motor.power(), 0.0);

        task.cancel();
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertStatus(intake, StarterIntake.Mode.STOPPED, 0.0, 0.65);
        intake.update(time.nextCycle(0.02));
        assertEquals(0.0, motor.power(), 0.0);

        intake.setMode(StarterIntake.Mode.EJECT);
        task.cancel();
        assertEquals(StarterIntake.Mode.EJECT, intake.status().mode());
        expectSecondStart(() -> task.start(time.nextCycle(0.02)));
        assertEquals(StarterIntake.Mode.EJECT, intake.status().mode());

        assertIllegalDuration(intake, 0.0);
        assertIllegalDuration(intake, -0.01);
        assertIllegalDuration(intake, Double.NaN);
        assertIllegalDuration(intake, Double.POSITIVE_INFINITY);
        assertIllegalDuration(intake, Double.NEGATIVE_INFINITY);
    }

    private static void assertStatus(StarterIntakeMechanism intake,
                                     StarterIntake.Mode expectedMode,
                                     double expectedRequestedPower,
                                     double expectedAppliedPower) {
        StarterIntake.Status status = intake.status();
        assertEquals(expectedMode, status.mode());
        assertEquals(expectedRequestedPower, status.requestedPower(), 0.0);
        assertEquals(expectedAppliedPower, status.appliedPower(), 0.0);
        assertEquals(expectedAppliedPower, status.plantSnapshot().appliedTarget(), 0.0);
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

    private static void expectUpdateBeforeStart(Runnable action) {
        try {
            action.run();
            fail("Expected update before start to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().toLowerCase().contains("before start"));
        }
    }

    private static void expectSecondStart(Runnable action) {
        try {
            action.run();
            fail("Expected second start to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().toLowerCase().contains("single-use"));
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
            throw new AssertionError("RecordingIntake status is not used by these controls tests");
        }
    }

}
