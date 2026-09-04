package edu.ftcsushi.robots.examples.basicmechanisms;

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

/** Maintainer regression contract behind the focused, student-facing lift software scenario. */
public final class BasicLiftMechanismTest {

    @Test
    public void semanticRequestReachesHardwareOnTheOutputHeartbeat() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(BasicLift.Height.STOWED, f.lift.status().requestedHeight());
        assertFalse(f.lift.status().referenced());
        assertFalse(f.lift.status().atTarget());
        assertTrue(Double.isNaN(f.lift.status().measuredPositionIn()));

        homeSuccessfully(f, time);
        BasicLift.Status beforeRequest = f.lift.status();
        int targetWritesBeforeRequest = f.motor.targetPositionWrites();
        f.lift.setHeight(BasicLift.Height.LOW);

        // Semantic intent is synchronous; hardware and feedback evidence wait for the heartbeat.
        assertEquals(BasicLift.Height.LOW, f.lift.status().requestedHeight());
        assertEquals(f.originalLowHeightIn, f.lift.status().requestedPositionIn(), 0.0);
        assertEquals(beforeRequest.measuredPositionIn(),
                f.lift.status().measuredPositionIn(), 0.0);
        assertEquals(beforeRequest.referenced(), f.lift.status().referenced());
        assertFalse(f.lift.status().atTarget());
        assertEquals(targetWritesBeforeRequest, f.motor.targetPositionWrites());

        f.lift.update(time.nextCycle(0.02));
        assertEquals(BasicLift.Height.LOW, f.lift.status().requestedHeight());
        assertEquals(f.originalLowHeightIn, f.lift.status().requestedPositionIn(), 0.0);
        assertEquals(
                (int) Math.round(f.originalLowHeightIn * f.originalTicksPerIn),
                f.motor.targetPositionTicks());

        f.motor.setCurrentPositionTicks(25);
        f.lift.update(time.nextCycle(0.02));
        assertEquals(2.5, f.lift.status().measuredPositionIn(), 0.0);
    }

    @Test
    public void statusIsANewFlatImmutableViewOfOneSourceSnapshot() {
        Fixture f = fixture();

        BasicLift.Status before = f.lift.status();
        BasicLift.Status sameState = f.lift.status();

        assertNotSame(before, sameState);
        Field[] fields = BasicLift.Status.class.getDeclaredFields();
        assertEquals(1, fields.length);
        assertEquals(
                edu.ftcsushi.fw.actuation.SemanticScalarSnapshot.class,
                fields[0].getType());
        assertTrue(Modifier.isPrivate(fields[0].getModifiers()));
        assertTrue(Modifier.isFinal(fields[0].getModifiers()));

        Constructor<?>[] constructors = BasicLift.Status.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPublic(constructors[0].getModifiers()));
        assertEquals(1, constructors[0].getParameterTypes().length);
        assertEquals(
                edu.ftcsushi.fw.actuation.SemanticScalarSnapshot.class,
                constructors[0].getParameterTypes()[0]);
        try {
            new BasicLift.Status(null);
            fail("Expected a null lift status snapshot to fail");
        } catch (NullPointerException expected) {
            assertEquals("snapshot", expected.getMessage());
        }

        assertEquals(before.plantSnapshot().appliedTarget(), before.appliedPositionIn(), 0.0);
        assertEquals(before.plantSnapshot().measurement(), before.measuredPositionIn(), 0.0);
        assertEquals(before.plantSnapshot().isReferenced(), before.referenced());
        assertEquals(BasicLift.Height.STOWED, before.requestedHeight());
        assertEquals(f.config.stowedHeightIn, before.requestedPositionIn(), 0.0);

        f.lift.setHeight(BasicLift.Height.HIGH);
        BasicLift.Status after = f.lift.status();

        assertEquals(BasicLift.Height.STOWED, before.requestedHeight());
        assertEquals(f.config.stowedHeightIn, before.requestedPositionIn(), 0.0);
        assertEquals(BasicLift.Height.HIGH, after.requestedHeight());
        assertEquals(f.config.highHeightIn, after.requestedPositionIn(), 0.0);
        assertFalse(after.atTarget());
    }

    @Test
    public void constructorDefensivelySnapshotsConfiguration() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        f.config.lowHeightIn = 9.0;
        f.config.ticksPerIn = 50.0;
        f.config.motorName = "replacementMotor";

        f.lift.setHeight(BasicLift.Height.LOW);
        f.lift.update(time.nextCycle(0.02));

        assertEquals(f.originalLowHeightIn, f.lift.status().requestedPositionIn(), 0.0);
        assertEquals(
                (int) Math.round(f.originalLowHeightIn * f.originalTicksPerIn),
                f.motor.targetPositionTicks());
    }

    @Test
    public void feedbackMovesAreFreshSingleUseAndCompleteOnlyFromEvidence() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        Task first = f.lift.moveTo(BasicLift.Height.HIGH);
        Task second = f.lift.moveTo(BasicLift.Height.HIGH);
        assertNotSame(first, second);

        f.motor.setCurrentPositionTicks(
                (int) Math.round(f.config.highHeightIn * f.originalTicksPerIn));
        first.start(time.nextCycle(0.02));
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight());
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn(), 0.0);
        assertFalse(f.lift.status().atTarget());
        f.lift.update(time.clock());
        assertFalse(first.isComplete());

        first.update(time.nextCycle(0.02));
        f.lift.update(time.clock());
        assertEquals(TaskOutcome.SUCCESS, first.getOutcome());
        assertTrue(f.lift.status().atTarget());

        expectSingleUse(() -> first.start(time.nextCycle(0.02)));
    }

    @Test
    public void unreferencedHighRequestFailsClosedWithoutInventingFeedback() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        f.lift.setHeight(BasicLift.Height.HIGH);
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight());
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn(), 0.0);
        assertTrue(Double.isNaN(f.lift.status().measuredPositionIn()));
        assertFalse(f.lift.status().referenced());
        assertFalse(f.lift.status().atTarget());
        assertEquals(0, f.motor.targetPositionWrites());

        f.lift.update(time.clock());

        assertEquals(0, f.motor.targetPositionWrites());
        assertEquals(0.0, f.motor.power(), 0.0);
        assertFalse(f.lift.status().referenced());
        assertFalse(f.lift.status().atTarget());
    }

    @Test
    public void feedbackMoveTimesOutAndLeavesItsPersistentRequest() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        Task move = f.lift.moveTo(BasicLift.Height.HIGH);
        move.start(time.nextCycle(0.02));
        f.lift.update(time.clock());
        update(move, f.lift, time, f.config.moveTimeoutSec + 0.01);

        assertEquals(TaskOutcome.TIMEOUT, move.getOutcome());
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight());
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn(), 0.0);
        assertEquals(
                (int) Math.round(f.config.highHeightIn * f.originalTicksPerIn),
                f.motor.targetPositionTicks());
        assertFalse(f.lift.status().atTarget());
    }

    @Test
    public void activeMoveCancellationLeavesTheSelectedTargetAndInvalidatesArrival() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        Task move = f.lift.moveTo(BasicLift.Height.LOW);
        move.start(time.nextCycle(0.02));
        f.lift.update(time.clock());
        int selectedTicks = (int) Math.round(
                f.config.lowHeightIn * f.originalTicksPerIn);
        assertEquals(selectedTicks, f.motor.targetPositionTicks());

        move.cancel();
        move.cancel();
        f.lift.update(time.nextCycle(0.02));

        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
        assertEquals(BasicLift.Height.LOW, f.lift.status().requestedHeight());
        assertEquals(f.config.lowHeightIn, f.lift.status().requestedPositionIn(), 0.0);
        assertEquals(selectedTicks, f.motor.targetPositionTicks());
        assertFalse(f.lift.status().atTarget());
    }

    @Test
    public void feedbackMoveCannotSucceedFromSupersededSemanticRequest() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        Task move = f.lift.moveTo(BasicLift.Height.LOW);
        move.start(time.nextCycle(0.02));
        f.motor.setCurrentPositionTicks(
                (int) Math.round(f.config.lowHeightIn * f.originalTicksPerIn));
        f.lift.update(time.clock());
        assertTrue(f.lift.status().atTarget());

        // A newer semantic request invalidates the old arrival before the move samples it.
        f.lift.setHeight(BasicLift.Height.HIGH);
        move.update(time.nextCycle(0.01));
        assertFalse(move.isComplete());
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight());
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn(), 0.0);

        move.update(time.nextCycle(f.config.moveTimeoutSec));
        assertEquals(TaskOutcome.TIMEOUT, move.getOutcome());
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight());
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn(), 0.0);
    }

    @Test
    public void feedbackMoveRetainsItsExactRequestWhenSameHeightIsRequestedAgain() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        Task move = f.lift.moveTo(BasicLift.Height.LOW);
        move.start(time.nextCycle(0.02));
        f.motor.setCurrentPositionTicks(
                (int) Math.round(f.config.lowHeightIn * f.originalTicksPerIn));
        f.lift.update(time.clock());
        assertTrue(f.lift.status().atTarget());

        f.lift.setHeight(BasicLift.Height.LOW);
        assertFalse(f.lift.status().atTarget());
        f.lift.update(time.nextCycle(0.01));
        assertTrue("the newer equal-valued request itself has arrived",
                f.lift.status().atTarget());

        move.update(time.clock());
        assertFalse(move.isComplete());

        move.update(time.nextCycle(f.config.moveTimeoutSec));
        assertEquals(TaskOutcome.TIMEOUT, move.getOutcome());
        assertEquals(BasicLift.Height.LOW, f.lift.status().requestedHeight());
        assertEquals(f.config.lowHeightIn, f.lift.status().requestedPositionIn(), 0.0);
    }

    @Test
    public void newRequestAndTerminalStopInvalidatePriorArrivalEvidence() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        homeSuccessfully(f, time);

        f.motor.setCurrentPositionTicks(
                (int) Math.round(f.config.lowHeightIn * f.originalTicksPerIn));
        f.lift.setHeight(BasicLift.Height.LOW);
        f.lift.update(time.nextCycle(0.02));
        assertTrue(f.lift.status().atTarget());
        double lowMeasurement = f.lift.status().measuredPositionIn();

        f.lift.setHeight(BasicLift.Height.HIGH);
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight());
        assertEquals(lowMeasurement, f.lift.status().measuredPositionIn(), 0.0);
        assertTrue(f.lift.status().referenced());
        assertFalse(f.lift.status().atTarget());

        f.motor.setCurrentPositionTicks(
                (int) Math.round(f.config.highHeightIn * f.originalTicksPerIn));
        f.lift.update(time.nextCycle(0.02));
        assertTrue(f.lift.status().atTarget());
        double highMeasurement = f.lift.status().measuredPositionIn();

        f.lift.stop();
        assertEquals(highMeasurement, f.lift.status().measuredPositionIn(), 0.0);
        assertTrue(f.lift.status().referenced());
        assertFalse(f.lift.status().atTarget());
    }

    @Test
    public void homingSuccessSelectsStowedButTimeoutPreservesTheLatestRequest() {
        Fixture success = fixture();
        Task firstHome = success.lift.home();
        Task secondHome = success.lift.home();
        assertNotSame(firstHome, secondHome);

        ManualLoopClock successTime = new ManualLoopClock();
        success.lift.setHeight(BasicLift.Height.HIGH);
        firstHome.start(successTime.clock());
        assertEquals(BasicLift.Height.HIGH, success.lift.status().requestedHeight());
        assertEquals(success.config.highHeightIn,
                success.lift.status().requestedPositionIn(), 0.0);
        success.lift.update(successTime.clock());
        assertEquals(success.config.homingPower, success.motor.power(), 0.0);

        success.bottomSwitch.setHigh(false);
        update(firstHome, success.lift, successTime, 0.01);
        int targetWritesBeforeSuccess = success.motor.targetPositionWrites();
        firstHome.update(successTime.nextCycle(0.03));

        assertEquals(TaskOutcome.SUCCESS, firstHome.getOutcome());
        assertEquals(BasicLift.Height.STOWED, success.lift.status().requestedHeight());
        assertEquals(success.config.stowedHeightIn,
                success.lift.status().requestedPositionIn(), 0.0);
        assertEquals(targetWritesBeforeSuccess, success.motor.targetPositionWrites());

        success.lift.update(successTime.clock());
        assertTrue(success.lift.status().referenced());
        assertEquals(
                (int) Math.round(success.config.stowedHeightIn * success.originalTicksPerIn),
                success.motor.targetPositionTicks());

        Fixture timeout = fixture();
        ManualLoopClock timeoutTime = new ManualLoopClock();
        Task timedOutHome = timeout.lift.home();
        timedOutHome.start(timeoutTime.clock());
        timeout.lift.setHeight(BasicLift.Height.HIGH);
        assertEquals(BasicLift.Height.HIGH, timeout.lift.status().requestedHeight());
        timeout.lift.update(timeoutTime.clock());
        update(timedOutHome, timeout.lift, timeoutTime, timeout.config.homingTimeoutSec);

        assertEquals(TaskOutcome.TIMEOUT, timedOutHome.getOutcome());
        assertFalse(timeout.lift.status().referenced());
        assertEquals(BasicLift.Height.HIGH, timeout.lift.status().requestedHeight());
        assertEquals(timeout.config.highHeightIn,
                timeout.lift.status().requestedPositionIn(), 0.0);
        assertEquals(0.0, timeout.motor.power(), 0.0);
    }

    @Test
    public void activeHomingCancellationZerosPowerAndDoesNotEstablishAReference() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        f.lift.setHeight(BasicLift.Height.HIGH);
        Task home = f.lift.home();

        home.start(time.clock());
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight());
        f.lift.update(time.clock());
        assertEquals(f.config.homingPower, f.motor.power(), 0.0);

        home.cancel();
        home.cancel();
        f.lift.update(time.nextCycle(0.02));

        assertEquals(TaskOutcome.CANCELLED, home.getOutcome());
        assertEquals(0.0, f.motor.power(), 0.0);
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight());
        assertEquals(f.config.highHeightIn, f.lift.status().requestedPositionIn(), 0.0);
        assertFalse(f.lift.status().referenced());
        assertFalse(f.lift.status().atTarget());
    }

    @Test
    public void invalidConfigurationFailsBeforeAnyHardwareLookup() {
        BasicLiftMechanism.Config timeout = config();
        timeout.moveTimeoutSec = Double.NaN;
        assertConfigFailureBeforeLookup(timeout, "moveTimeoutSec");

        BasicLiftMechanism.Config unordered = config();
        unordered.lowHeightIn = unordered.stowedHeightIn;
        assertConfigFailureBeforeLookup(unordered, "stowedHeightIn < lowHeightIn");

        BasicLiftMechanism.Config overlappingNamedHeights = config();
        overlappingNamedHeights.toleranceIn =
                (overlappingNamedHeights.lowHeightIn
                        - overlappingNamedHeights.stowedHeightIn) / 2.0;
        assertConfigFailureBeforeLookup(
                overlappingNamedHeights,
                "toleranceIn must be strictly less than half the closest adjacent named-height gap");

        BasicLiftMechanism.Config unnamed = config();
        unnamed.motorName = "  ";
        assertConfigFailureBeforeLookup(unnamed, "motorName");
    }

    private static void homeSuccessfully(Fixture f, ManualLoopClock time) {
        Task home = f.lift.home();
        home.start(time.clock());
        f.lift.update(time.clock());
        f.bottomSwitch.setHigh(false);
        update(home, f.lift, time, 0.01);
        update(home, f.lift, time, 0.03);
        assertEquals(TaskOutcome.SUCCESS, home.getOutcome());
    }

    private static void update(Task task,
                               BasicLiftMechanism lift,
                               ManualLoopClock time,
                               double dtSec) {
        task.update(time.nextCycle(dtSec));
        lift.update(time.clock());
    }

    private static Fixture fixture() {
        BasicLiftMechanism.Config config = config();
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        FtcTestHardware.DigitalProbe bottomSwitch =
                hardware.addDigitalInput(config.bottomSwitchName);
        return new Fixture(config, motor, bottomSwitch,
                new BasicLiftMechanism(hardware, config));
    }

    private static BasicLiftMechanism.Config config() {
        BasicLiftMechanism.Config config = BasicLiftMechanism.Config.defaults();
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
            BasicLiftMechanism.Config config,
            String expectedMessage) {
        FtcTestHardware hardware = new FtcTestHardware();
        try {
            new BasicLiftMechanism(hardware, config);
            fail("Expected invalid Basic lift configuration");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains(expectedMessage));
        }
        assertEquals(0, hardware.lookupCalls());
    }

    private static void expectSingleUse(Runnable action) {
        try {
            action.run();
            fail("Expected a second Task start to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().toLowerCase().contains("single-use"));
        }
    }

    private static final class Fixture {
        private final BasicLiftMechanism.Config config;
        private final double originalLowHeightIn;
        private final double originalTicksPerIn;
        private final FtcTestHardware.MotorProbe motor;
        private final FtcTestHardware.DigitalProbe bottomSwitch;
        private final BasicLiftMechanism lift;

        private Fixture(BasicLiftMechanism.Config config,
                        FtcTestHardware.MotorProbe motor,
                        FtcTestHardware.DigitalProbe bottomSwitch,
                        BasicLiftMechanism lift) {
            this.config = config;
            this.originalLowHeightIn = config.lowHeightIn;
            this.originalTicksPerIn = config.ticksPerIn;
            this.motor = motor;
            this.bottomSwitch = bottomSwitch;
            this.lift = lift;
        }
    }
}
