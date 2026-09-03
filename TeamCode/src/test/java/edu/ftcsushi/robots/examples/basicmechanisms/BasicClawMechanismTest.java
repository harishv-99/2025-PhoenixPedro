package edu.ftcsushi.robots.examples.basicmechanisms;

import org.junit.Test;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Teaches semantic servo requests without pretending a standard servo has feedback. */
public final class BasicClawMechanismTest {

    private static final double EPSILON = 1.0e-12;
    private static final double CLOSED_TARGET = 0.0;
    private static final double OPEN_TARGET = 1.0;

    @Test
    public void firstHeartbeatAppliesTheConfiguredInitialClosedRequest() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        assertStatus(f.claw, BasicClaw.State.CLOSED, Double.NaN);
        assertEquals(0, f.servo.positionWrites());

        f.claw.update(time.clock());

        assertStatus(f.claw, BasicClaw.State.CLOSED, CLOSED_TARGET);
        assertEquals(f.originalClosedNativePosition, f.servo.position(), 0.0);
        assertEquals(1, f.servo.positionWrites());
    }

    @Test
    public void requestIsImmediateButAppliedTargetWaitsForTheOutputHeartbeat() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        assertStatus(f.claw, BasicClaw.State.CLOSED, Double.NaN);
        assertEquals(0, f.servo.positionWrites());

        f.claw.setState(BasicClaw.State.OPEN);
        assertStatus(f.claw, BasicClaw.State.OPEN, Double.NaN);
        assertEquals(0, f.servo.positionWrites());

        f.claw.update(time.clock());

        assertStatus(f.claw, BasicClaw.State.OPEN, OPEN_TARGET);
        assertEquals(f.originalOpenNativePosition, f.servo.position(), 0.0);
    }

    @Test
    public void stateTasksAreFreshDeferredAndSingleUse() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        Task first = f.claw.setStateTask(BasicClaw.State.OPEN);
        Task second = f.claw.setStateTask(BasicClaw.State.OPEN);

        assertNotSame(first, second);
        assertStatus(f.claw, BasicClaw.State.CLOSED, Double.NaN);
        assertEquals(0, f.servo.positionWrites());

        first.start(time.clock());

        assertTrue(first.isComplete());
        assertEquals(TaskOutcome.SUCCESS, first.getOutcome());
        assertStatus(f.claw, BasicClaw.State.OPEN, Double.NaN);
        assertEquals(0, f.servo.positionWrites());

        f.claw.update(time.clock());
        assertStatus(f.claw, BasicClaw.State.OPEN, OPEN_TARGET);
        assertEquals(1, f.servo.positionWrites());

        f.claw.setState(BasicClaw.State.CLOSED);
        try {
            first.start(time.clock());
            fail("Expected one claw state Task instance to reject a second start");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("single-use"));
        }
        assertStatus(f.claw, BasicClaw.State.CLOSED, OPEN_TARGET);

        second.start(time.clock());
        assertEquals(TaskOutcome.SUCCESS, second.getOutcome());
        assertStatus(f.claw, BasicClaw.State.OPEN, OPEN_TARGET);
    }

    @Test
    public void stateTaskRejectsNullWithoutChangingTheRequest() {
        Fixture f = fixture();

        try {
            f.claw.setStateTask(null);
            fail("Expected null claw state Task request to fail");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("state"));
        }

        assertStatus(f.claw, BasicClaw.State.CLOSED, Double.NaN);
        assertEquals(0, f.servo.positionWrites());
    }

    @Test
    public void stopBeforeFirstHeartbeatKeepsAppliedTargetUnknownAndWritesNothing() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        f.claw.setState(BasicClaw.State.OPEN);
        f.claw.stop();
        assertStatus(f.claw, BasicClaw.State.OPEN, Double.NaN);
        assertEquals(0, f.servo.positionWrites());

        f.claw.setState(BasicClaw.State.CLOSED);
        f.claw.update(time.clock());
        assertStatus(f.claw, BasicClaw.State.CLOSED, Double.NaN);
        assertEquals(0, f.servo.positionWrites());
    }

    @Test
    public void standardServoStopRetainsTheLastCommandWithoutClaimingArrival() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        f.claw.setState(BasicClaw.State.OPEN);
        f.claw.update(time.clock());
        int writesBeforeStop = f.servo.positionWrites();

        f.claw.stop();
        assertStatus(f.claw, BasicClaw.State.OPEN, OPEN_TARGET);
        assertEquals(f.originalOpenNativePosition, f.servo.position(), 0.0);
        int writesAfterStop = f.servo.positionWrites();
        assertTrue(writesAfterStop >= writesBeforeStop);

        f.claw.setState(BasicClaw.State.CLOSED);
        f.claw.update(time.nextCycle(0.02));
        assertStatus(f.claw, BasicClaw.State.CLOSED, OPEN_TARGET);
        assertEquals(writesAfterStop, f.servo.positionWrites());
        assertEquals(f.originalOpenNativePosition, f.servo.position(), 0.0);
    }

    @Test
    public void constructorDefensivelySnapshotsNamedPositions() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        f.config.openNativePosition = 0.95;
        f.config.closedNativePosition = 0.05;
        f.config.servoName = "replacementServo";

        f.claw.setState(BasicClaw.State.OPEN);
        f.claw.update(time.clock());

        assertStatus(f.claw, BasicClaw.State.OPEN, OPEN_TARGET);
        assertEquals(f.originalOpenNativePosition, f.servo.position(), 0.0);
    }

    @Test
    public void reversedNativeEndpointsKeepTheSemanticPlantCoordinateNormalized() {
        BasicClawMechanism.Config config = config();
        config.closedNativePosition = 0.85;
        config.openNativePosition = 0.15;
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.ServoProbe servo = hardware.addServo(config.servoName);
        BasicClawMechanism claw = new BasicClawMechanism(hardware, config);
        ManualLoopClock time = new ManualLoopClock();

        claw.update(time.clock());
        assertStatus(claw, BasicClaw.State.CLOSED, CLOSED_TARGET);
        assertEquals(config.closedNativePosition, servo.position(), EPSILON);

        claw.setState(BasicClaw.State.OPEN);
        claw.update(time.nextCycle(0.02));
        assertStatus(claw, BasicClaw.State.OPEN, OPEN_TARGET);
        assertEquals(config.openNativePosition, servo.position(), EPSILON);
    }

    @Test
    public void invalidConfigurationFailsBeforeAnyHardwareLookup() {
        BasicClawMechanism.Config unnamed = config();
        unnamed.servoName = "  ";
        assertConfigFailureBeforeLookup(unnamed, "servoName");

        BasicClawMechanism.Config nonFinite = config();
        nonFinite.openNativePosition = Double.NaN;
        assertConfigFailureBeforeLookup(nonFinite, "openNativePosition");

        BasicClawMechanism.Config indistinguishable = config();
        indistinguishable.openNativePosition = indistinguishable.closedNativePosition;
        assertConfigFailureBeforeLookup(indistinguishable, "must be distinct");
    }

    private static Fixture fixture() {
        BasicClawMechanism.Config config = config();
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.ServoProbe servo = hardware.addServo(config.servoName);
        return new Fixture(config, servo, new BasicClawMechanism(hardware, config));
    }

    private static BasicClawMechanism.Config config() {
        BasicClawMechanism.Config config = BasicClawMechanism.Config.defaults();
        config.servoName = "claw";
        config.closedNativePosition = 0.20;
        config.openNativePosition = 0.80;
        return config;
    }

    private static void assertStatus(BasicClaw claw,
                                     BasicClaw.State expectedState,
                                     double expectedAppliedCoordinate) {
        BasicClaw.Status status = claw.status();
        assertEquals(expectedState, status.requestedState);
        assertEquals(expectedAppliedCoordinate, status.appliedCoordinate, 0.0);
    }

    private static void assertConfigFailureBeforeLookup(
            BasicClawMechanism.Config config,
            String expectedMessage) {
        FtcTestHardware hardware = new FtcTestHardware();
        try {
            new BasicClawMechanism(hardware, config);
            fail("Expected invalid Basic claw configuration");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains(expectedMessage));
        }
        assertEquals(0, hardware.lookupCalls());
    }

    private static final class Fixture {
        private final BasicClawMechanism.Config config;
        private final double originalClosedNativePosition;
        private final double originalOpenNativePosition;
        private final FtcTestHardware.ServoProbe servo;
        private final BasicClawMechanism claw;

        private Fixture(BasicClawMechanism.Config config,
                        FtcTestHardware.ServoProbe servo,
                        BasicClawMechanism claw) {
            this.config = config;
            this.originalClosedNativePosition = config.closedNativePosition;
            this.originalOpenNativePosition = config.openNativePosition;
            this.servo = servo;
            this.claw = claw;
        }
    }
}
