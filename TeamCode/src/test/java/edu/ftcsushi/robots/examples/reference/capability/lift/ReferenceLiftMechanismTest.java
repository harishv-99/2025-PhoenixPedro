package edu.ftcsushi.robots.examples.reference.capability.lift;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Reference lift configuration, homing coherence, and feedback move semantics. */
public final class ReferenceLiftMechanismTest {

    @Test
    public void statusIsADelegateOnlyDomainView() {
        Field[] fields = ReferenceLift.Status.class.getDeclaredFields();
        assertEquals(1, fields.length);
        assertEquals(
                edu.ftcsushi.fw.actuation.SemanticScalarSnapshot.class,
                fields[0].getType());
        assertTrue(Modifier.isPrivate(fields[0].getModifiers()));
        assertTrue(Modifier.isFinal(fields[0].getModifiers()));

        Constructor<?>[] constructors = ReferenceLift.Status.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPublic(constructors[0].getModifiers()));
        assertEquals(1, constructors[0].getParameterTypes().length);
        assertEquals(
                edu.ftcsushi.fw.actuation.SemanticScalarSnapshot.class,
                constructors[0].getParameterTypes()[0]);

        try {
            new ReferenceLift.Status(null);
            fail("Expected a null lift status delegate to fail");
        } catch (NullPointerException expected) {
            assertEquals("delegate", expected.getMessage());
        }
    }

    @Test
    public void defaultsExposeARealStowedStatusAndFreshFeedbackMoves() {
        Fixture f = fixture();

        ReferenceLift.Status initial = f.lift.status();
        assertEquals(ReferenceLift.Height.STOWED, initial.requestedHeight());
        assertEquals(f.config.stowedHeightIn, initial.requestedPositionIn(), 0.0);
        assertFalse(initial.referenced());
        assertEquals(initial.measuredPositionIn(), initial.plantSnapshot().measurement(), 0.0);

        Task first = f.lift.moveTo(ReferenceLift.Height.LOW);
        Task second = f.lift.moveTo(ReferenceLift.Height.LOW);
        assertNotSame(first, second);
        assertEquals(f.config.stowedHeightIn, f.lift.status().requestedPositionIn(), 0.0);
    }

    @Test
    public void directSemanticRequestPublishesItsPairedPositionSynchronously() {
        Fixture f = fixture();
        ReferenceLift.Status before = f.lift.status();
        int targetWritesBefore = f.motor.targetPositionWrites();

        f.lift.setHeight(ReferenceLift.Height.HIGH);

        ReferenceLift.Status requested = f.lift.status();
        assertEquals(ReferenceLift.Height.HIGH, requested.requestedHeight());
        assertEquals(f.config.highHeightIn, requested.requestedPositionIn(), 0.0);
        assertEquals(before.appliedPositionIn(), requested.appliedPositionIn(), 0.0);
        assertEquals(before.measuredPositionIn(), requested.measuredPositionIn(), 0.0);
        assertEquals(before.referenced(), requested.referenced());
        assertFalse(requested.atTarget());
        assertEquals(
                requested.appliedPositionIn(),
                requested.plantSnapshot().appliedTarget(),
                0.0);
        assertEquals(targetWritesBefore, f.motor.targetPositionWrites());
    }

    @Test
    public void terminalStopInvalidatesArrivalWithoutChangingThePairedRequest() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);
        f.motor.setCurrentPositionTicks(
                (int) Math.round(f.config.stowedHeightIn * f.config.ticksPerIn));
        f.lift.update(time.nextCycle(0.02));
        ReferenceLift.Status beforeStop = f.lift.status();
        assertTrue(beforeStop.atTarget());

        f.lift.stop();

        ReferenceLift.Status stopped = f.lift.status();
        assertEquals(beforeStop.requestedHeight(), stopped.requestedHeight());
        assertEquals(beforeStop.requestedPositionIn(), stopped.requestedPositionIn(), 0.0);
        assertEquals(beforeStop.measuredPositionIn(), stopped.measuredPositionIn(), 0.0);
        assertEquals(beforeStop.referenced(), stopped.referenced());
        assertFalse(stopped.atTarget());
    }

    @Test
    public void moveStartsItsSemanticAndNumericRequestTogetherAndWaitsForFeedback() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        Task move = f.lift.moveTo(ReferenceLift.Height.LOW);
        f.motor.setCurrentPositionTicks((int) (f.config.lowHeightIn * f.config.ticksPerIn));
        move.start(time.nextCycle(0.02));

        f.lift.update(time.clock());
        move.update(time.nextCycle(0.02));
        f.lift.update(time.clock());

        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
        assertEquals(ReferenceLift.Height.LOW, f.lift.status().requestedHeight());
        assertEquals(f.config.lowHeightIn, f.lift.status().requestedPositionIn(), 0.0);
        assertTrue(f.lift.status().atTarget());
    }

    @Test
    public void moveTimeoutAndCancellationLeaveThePersistentSelectedTarget() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        f.motor.setCurrentPositionTicks(0);
        Task timeout = f.lift.moveTo(ReferenceLift.Height.HIGH);
        timeout.start(time.nextCycle(0.02));
        f.lift.update(time.clock());
        timeout.update(time.nextCycle(f.config.moveTimeoutSec));
        f.lift.update(time.clock());

        assertEquals(TaskOutcome.TIMEOUT, timeout.getOutcome());
        assertEquals(ReferenceLift.Height.HIGH, f.lift.status().requestedHeight());
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn(), 0.0);

        Task cancelled = f.lift.moveTo(ReferenceLift.Height.LOW);
        cancelled.start(time.nextCycle(0.02));
        f.lift.update(time.clock());
        cancelled.cancel();
        cancelled.cancel();
        f.lift.update(time.nextCycle(0.02));

        assertEquals(TaskOutcome.CANCELLED, cancelled.getOutcome());
        assertEquals(ReferenceLift.Height.LOW, f.lift.status().requestedHeight());
        assertEquals(f.config.lowHeightIn, f.lift.status().requestedPositionIn(), 0.0);
    }

    @Test
    public void moveCannotSucceedFromFeedbackForASupersededSemanticRequest() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        Task move = f.lift.moveTo(ReferenceLift.Height.LOW);
        move.start(time.nextCycle(0.02));
        f.motor.setCurrentPositionTicks(
                (int) Math.round(f.config.lowHeightIn * f.config.ticksPerIn));
        f.lift.update(time.clock());
        assertTrue(f.lift.status().atTarget());

        f.lift.setHeight(ReferenceLift.Height.HIGH);
        move.update(time.nextCycle(0.01));
        assertFalse(move.isComplete());
        assertEquals(ReferenceLift.Height.HIGH, f.lift.status().requestedHeight());
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn(), 0.0);

        move.update(time.nextCycle(f.config.moveTimeoutSec));
        assertEquals(TaskOutcome.TIMEOUT, move.getOutcome());
        assertEquals(ReferenceLift.Height.HIGH, f.lift.status().requestedHeight());
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn(), 0.0);
    }

    @Test
    public void moveCannotSucceedAfterAnEqualValuedRequestSupersedesIt() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        Task move = f.lift.moveTo(ReferenceLift.Height.LOW);
        move.start(time.nextCycle(0.02));
        f.motor.setCurrentPositionTicks(
                (int) Math.round(f.config.lowHeightIn * f.config.ticksPerIn));
        f.lift.update(time.clock());
        assertTrue(f.lift.status().atTarget());

        f.lift.setHeight(ReferenceLift.Height.LOW);
        assertFalse(f.lift.status().atTarget());
        f.lift.update(time.nextCycle(0.01));
        assertTrue("the newer equal-valued request itself has arrived",
                f.lift.status().atTarget());

        move.update(time.clock());
        assertFalse(move.isComplete());
        move.update(time.nextCycle(f.config.moveTimeoutSec));

        assertEquals(TaskOutcome.TIMEOUT, move.getOutcome());
        assertEquals(ReferenceLift.Height.LOW, f.lift.status().requestedHeight());
        assertEquals(f.config.lowHeightIn, f.lift.status().requestedPositionIn(), 0.0);
    }

    @Test
    public void homingSuccessSelectsStowedButTimeoutPreservesConcurrentRequest() {
        Fixture success = fixture();
        ManualLoopClock successTime = new ManualLoopClock();
        Task successfulHome = success.lift.home();
        successfulHome.start(successTime.clock());
        success.lift.update(successTime.clock());
        success.lift.setHeight(ReferenceLift.Height.HIGH);
        success.bottomSwitch.setHigh(false);
        update(successfulHome, success.lift, successTime, 0.01);
        int targetWritesBeforeSuccess = success.motor.targetPositionWrites();
        successfulHome.update(successTime.nextCycle(0.03));

        assertEquals(TaskOutcome.SUCCESS, successfulHome.getOutcome());
        assertEquals(ReferenceLift.Height.STOWED, success.lift.status().requestedHeight());
        assertEquals(success.config.stowedHeightIn,
                success.lift.status().requestedPositionIn(), 0.0);
        assertEquals(targetWritesBeforeSuccess, success.motor.targetPositionWrites());

        success.lift.update(successTime.clock());
        assertTrue(success.lift.status().referenced());
        assertEquals(
                (int) Math.round(success.config.stowedHeightIn * success.config.ticksPerIn),
                success.motor.targetPositionTicks());

        Fixture timeout = fixture();
        ManualLoopClock timeoutTime = new ManualLoopClock();
        timeout.lift.setHeight(ReferenceLift.Height.HIGH);
        Task timedOutHome = timeout.lift.home();
        timedOutHome.start(timeoutTime.clock());
        timeout.lift.update(timeoutTime.clock());
        update(timedOutHome, timeout.lift, timeoutTime, timeout.config.homingTimeoutSec);

        assertEquals(TaskOutcome.TIMEOUT, timedOutHome.getOutcome());
        assertEquals(ReferenceLift.Height.HIGH, timeout.lift.status().requestedHeight());
        assertEquals(timeout.config.highHeightIn,
                timeout.lift.status().requestedPositionIn(), 0.0);
    }

    @Test
    public void activeHomingCancellationKeepsConcurrentRequestCoherent() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        Task home = f.lift.home();
        home.start(time.clock());
        f.lift.update(time.clock());
        f.lift.setHeight(ReferenceLift.Height.HIGH);

        home.cancel();
        f.lift.update(time.nextCycle(0.02));

        assertEquals(TaskOutcome.CANCELLED, home.getOutcome());
        assertEquals(ReferenceLift.Height.HIGH, f.lift.status().requestedHeight());
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn(), 0.0);
    }

    @Test
    public void invalidMoveTimeoutAndUnorderedSemanticHeightsFailBeforeHardwareLookup() {
        ReferenceLiftMechanism.Config timeout = config();
        timeout.moveTimeoutSec = Double.NaN;
        assertConfigFailureBeforeLookup(timeout, "moveTimeoutSec");

        ReferenceLiftMechanism.Config unordered = config();
        unordered.lowHeightIn = unordered.stowedHeightIn;
        assertConfigFailureBeforeLookup(unordered, "stowedHeightIn < lowHeightIn");

        ReferenceLiftMechanism.Config reversed = config();
        reversed.highHeightIn = reversed.lowHeightIn - 1.0;
        assertConfigFailureBeforeLookup(reversed, "lowHeightIn < highHeightIn");
    }

    private static void homeSuccessfully(Fixture f, ManualLoopClock time) {
        Task home = f.lift.home();
        home.start(time.clock());
        f.lift.update(time.clock());
        f.bottomSwitch.setHigh(false);
        update(home, f.lift, time, 0.01);
        update(home, f.lift, time, 0.03);
        assertEquals(TaskOutcome.SUCCESS, home.getOutcome());
        assertTrue(f.lift.status().referenced());
    }

    private static void update(Task task,
                               ReferenceLiftMechanism lift,
                               ManualLoopClock time,
                               double dtSec) {
        task.update(time.nextCycle(dtSec));
        lift.update(time.clock());
    }

    private static Fixture fixture() {
        ReferenceLiftMechanism.Config config = config();
        FtcTestHardware hardware =
                new FtcTestHardware();
        FtcTestHardware.DigitalProbe bottomSwitch =
                hardware.addDigitalInput(config.bottomSwitchName);
        FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        return new Fixture(
                config,
                motor,
                bottomSwitch,
                new ReferenceLiftMechanism(hardware, config));
    }

    private static ReferenceLiftMechanism.Config config() {
        ReferenceLiftMechanism.Config config = ReferenceLiftMechanism.Config.defaults();
        config.motorName = "lift";
        config.bottomSwitchName = "bottom";
        config.maximumHeightIn = 10.0;
        config.ticksPerIn = 10.0;
        config.toleranceIn = 0.10;
        config.stowedHeightIn = 1.0;
        config.lowHeightIn = 4.0;
        config.highHeightIn = 8.0;
        config.homingTimeoutSec = 0.10;
        config.moveTimeoutSec = 0.10;
        return config;
    }

    private static void assertConfigFailureBeforeLookup(
            ReferenceLiftMechanism.Config config,
            String expectedMessage) {
        FtcTestHardware hardware =
                new FtcTestHardware();
        try {
            new ReferenceLiftMechanism(hardware, config);
            fail("Expected invalid Reference lift configuration");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains(expectedMessage));
        }
        assertEquals(0, hardware.lookupCalls());
    }

    private static final class Fixture {
        private final ReferenceLiftMechanism.Config config;
        private final FtcTestHardware.MotorProbe motor;
        private final FtcTestHardware.DigitalProbe bottomSwitch;
        private final ReferenceLiftMechanism lift;

        private Fixture(ReferenceLiftMechanism.Config config,
                        FtcTestHardware.MotorProbe motor,
                        FtcTestHardware.DigitalProbe bottomSwitch,
                        ReferenceLiftMechanism lift) {
            this.config = config;
            this.motor = motor;
            this.bottomSwitch = bottomSwitch;
            this.lift = lift;
        }
    }
}
