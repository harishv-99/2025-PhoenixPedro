package edu.ftcsushi.robots.examples.basicmechanisms;

import org.junit.Test;

import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Teaches semantic servo requests without pretending a standard servo has feedback. */
public final class BasicClawMechanismTest {

    @Test
    public void firstHeartbeatAppliesTheConfiguredInitialClosedRequest() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        assertStatus(f.claw, BasicClaw.State.CLOSED, Double.NaN);
        assertEquals(0, f.servo.positionWrites());

        f.claw.update(time.clock());

        assertStatus(f.claw, BasicClaw.State.CLOSED, f.originalClosedPosition);
        assertEquals(f.originalClosedPosition, f.servo.position(), 0.0);
        assertEquals(1, f.servo.positionWrites());
    }

    @Test
    public void requestIsImmediateButAppliedCommandWaitsForTheOutputHeartbeat() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        assertStatus(f.claw, BasicClaw.State.CLOSED, Double.NaN);
        assertEquals(0, f.servo.positionWrites());

        f.claw.setState(BasicClaw.State.OPEN);
        assertStatus(f.claw, BasicClaw.State.OPEN, Double.NaN);
        assertEquals(0, f.servo.positionWrites());

        f.claw.update(time.clock());

        assertStatus(f.claw, BasicClaw.State.OPEN, f.originalOpenPosition);
        assertEquals(f.originalOpenPosition, f.servo.position(), 0.0);
    }

    @Test
    public void stopBeforeFirstHeartbeatKeepsAppliedCommandUnknownAndWritesNothing() {
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
        assertStatus(f.claw, BasicClaw.State.OPEN, f.originalOpenPosition);
        assertEquals(f.originalOpenPosition, f.servo.position(), 0.0);
        int writesAfterStop = f.servo.positionWrites();
        assertTrue(writesAfterStop >= writesBeforeStop);

        f.claw.setState(BasicClaw.State.CLOSED);
        f.claw.update(time.nextCycle(0.02));
        assertStatus(f.claw, BasicClaw.State.CLOSED, f.originalOpenPosition);
        assertEquals(writesAfterStop, f.servo.positionWrites());
        assertEquals(f.originalOpenPosition, f.servo.position(), 0.0);
    }

    @Test
    public void constructorDefensivelySnapshotsNamedPositions() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();

        f.config.openPosition = 0.95;
        f.config.closedPosition = 0.05;
        f.config.servoName = "replacementServo";

        f.claw.setState(BasicClaw.State.OPEN);
        f.claw.update(time.clock());

        assertStatus(f.claw, BasicClaw.State.OPEN, f.originalOpenPosition);
        assertEquals(f.originalOpenPosition, f.servo.position(), 0.0);
    }

    @Test
    public void invalidConfigurationFailsBeforeAnyHardwareLookup() {
        BasicClawMechanism.Config unnamed = config();
        unnamed.servoName = "  ";
        assertConfigFailureBeforeLookup(unnamed, "servoName");

        BasicClawMechanism.Config nonFinite = config();
        nonFinite.openPosition = Double.NaN;
        assertConfigFailureBeforeLookup(nonFinite, "openPosition");

        BasicClawMechanism.Config indistinguishable = config();
        indistinguishable.openPosition = indistinguishable.closedPosition;
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
        config.closedPosition = 0.20;
        config.openPosition = 0.80;
        return config;
    }

    private static void assertStatus(BasicClaw claw,
                                     BasicClaw.State expectedState,
                                     double expectedAppliedPosition) {
        BasicClaw.Status status = claw.status();
        assertEquals(expectedState, status.requestedState);
        assertEquals(expectedAppliedPosition, status.appliedPosition, 0.0);
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
        private final double originalClosedPosition;
        private final double originalOpenPosition;
        private final FtcTestHardware.ServoProbe servo;
        private final BasicClawMechanism claw;

        private Fixture(BasicClawMechanism.Config config,
                        FtcTestHardware.ServoProbe servo,
                        BasicClawMechanism claw) {
            this.config = config;
            this.originalClosedPosition = config.closedPosition;
            this.originalOpenPosition = config.openPosition;
            this.servo = servo;
            this.claw = claw;
        }
    }
}
