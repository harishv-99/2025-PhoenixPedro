package edu.ftcphoenix.robots.examples.reference.capability.lift;

import org.junit.Test;

import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.testing.ManualLoopClock;
import edu.ftcphoenix.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Reference lift configuration, homing coherence, and feedback move semantics. */
public final class ReferenceLiftMechanismTest {

    @Test
    public void defaultsExposeARealStowedStatusAndFreshFeedbackMoves() {
        Fixture f = fixture();

        ReferenceLift.Status initial = f.lift.status();
        assertEquals(ReferenceLift.Height.STOWED, initial.requestedHeight);
        assertEquals(f.config.stowedHeightIn, initial.requestedPositionIn, 0.0);
        assertFalse(initial.referenced);

        Task first = f.lift.moveTo(ReferenceLift.Height.LOW);
        Task second = f.lift.moveTo(ReferenceLift.Height.LOW);
        assertNotSame(first, second);
        assertEquals(f.config.stowedHeightIn, f.lift.status().requestedPositionIn, 0.0);
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
        assertEquals(ReferenceLift.Height.LOW, f.lift.status().requestedHeight);
        assertEquals(f.config.lowHeightIn, f.lift.status().requestedPositionIn, 0.0);
        assertTrue(f.lift.status().atTarget);
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
        assertEquals(ReferenceLift.Height.HIGH, f.lift.status().requestedHeight);
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn, 0.0);

        Task cancelled = f.lift.moveTo(ReferenceLift.Height.LOW);
        cancelled.start(time.nextCycle(0.02));
        f.lift.update(time.clock());
        cancelled.cancel();
        cancelled.cancel();
        f.lift.update(time.nextCycle(0.02));

        assertEquals(TaskOutcome.CANCELLED, cancelled.getOutcome());
        assertEquals(ReferenceLift.Height.LOW, f.lift.status().requestedHeight);
        assertEquals(f.config.lowHeightIn, f.lift.status().requestedPositionIn, 0.0);
    }

    @Test
    public void successfulAndTimedOutHomingRepairConcurrentHeightRequestsToStowed() {
        Fixture success = fixture();
        ManualLoopClock successTime = new ManualLoopClock();
        Task successfulHome = success.lift.home();
        successfulHome.start(successTime.clock());
        success.lift.update(successTime.clock());
        success.lift.setHeight(ReferenceLift.Height.HIGH);
        success.bottomSwitch.setHigh(false);
        update(successfulHome, success.lift, successTime, 0.01);
        update(successfulHome, success.lift, successTime, 0.03);

        assertEquals(TaskOutcome.SUCCESS, successfulHome.getOutcome());
        assertEquals(ReferenceLift.Height.STOWED, success.lift.status().requestedHeight);
        assertEquals(success.config.stowedHeightIn,
                success.lift.status().requestedPositionIn, 0.0);

        Fixture timeout = fixture();
        ManualLoopClock timeoutTime = new ManualLoopClock();
        Task timedOutHome = timeout.lift.home();
        timedOutHome.start(timeoutTime.clock());
        timeout.lift.update(timeoutTime.clock());
        timeout.lift.setHeight(ReferenceLift.Height.HIGH);
        update(timedOutHome, timeout.lift, timeoutTime, timeout.config.homingTimeoutSec);

        assertEquals(TaskOutcome.TIMEOUT, timedOutHome.getOutcome());
        assertEquals(ReferenceLift.Height.STOWED, timeout.lift.status().requestedHeight);
        assertEquals(timeout.config.stowedHeightIn,
                timeout.lift.status().requestedPositionIn, 0.0);
    }

    @Test
    public void activeHomingCancellationSkipsFinalRepairAndKeepsConcurrentRequestCoherent() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        Task home = f.lift.home();
        home.start(time.clock());
        f.lift.update(time.clock());
        f.lift.setHeight(ReferenceLift.Height.HIGH);

        home.cancel();
        f.lift.update(time.nextCycle(0.02));

        assertEquals(TaskOutcome.CANCELLED, home.getOutcome());
        assertEquals(ReferenceLift.Height.HIGH, f.lift.status().requestedHeight);
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn, 0.0);
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
        assertTrue(f.lift.status().referenced);
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
