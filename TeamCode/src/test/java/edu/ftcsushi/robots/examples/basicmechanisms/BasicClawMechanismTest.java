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

/** Compact regression contract behind the student-facing claw software scenario. */
public final class BasicClawMechanismTest {

    @Test
    public void stateTasksAreFreshDeferredAndSingleUse() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        Task first = f.claw.setStateTask(BasicClaw.State.OPEN);
        Task second = f.claw.setStateTask(BasicClaw.State.HALF);

        assertNotSame(first, second);
        assertStatus(f.claw, BasicClaw.State.CLOSED, 0.0, 0.0);
        assertEquals(0, f.servo.positionWrites());

        first.start(time.clock());
        assertEquals(TaskOutcome.SUCCESS, first.getOutcome());
        assertStatus(f.claw, BasicClaw.State.OPEN, 1.0, 0.0);
        assertEquals(0, f.servo.positionWrites());

        f.claw.update(time.clock());
        assertStatus(f.claw, BasicClaw.State.OPEN, 1.0, 1.0);
        assertEquals(f.openNativePosition, f.servo.position(), 0.0);

        f.claw.setState(BasicClaw.State.CLOSED);
        try {
            first.start(time.clock());
            fail("Expected one claw Task instance to reject a second start");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("single-use"));
        }
        assertStatus(f.claw, BasicClaw.State.CLOSED, 0.0, 1.0);

        second.start(time.clock());
        assertStatus(f.claw, BasicClaw.State.HALF, 0.5, 1.0);
        f.claw.update(time.nextCycle(0.02));
        assertStatus(f.claw, BasicClaw.State.HALF, 0.5, 0.5);
        assertEquals(0.5, f.servo.position(), 1e-12);
    }

    @Test
    public void stopRetainsTheLastServoCommandAndPreventsLaterWrites() {
        Fixture f = fixture();
        ManualLoopClock time = new ManualLoopClock();
        f.claw.setState(BasicClaw.State.OPEN);
        f.claw.update(time.clock());

        f.claw.stop();
        int writesAfterStop = f.servo.positionWrites();
        f.claw.setState(BasicClaw.State.CLOSED);
        f.claw.update(time.nextCycle(0.02));

        assertStatus(f.claw, BasicClaw.State.CLOSED, 0.0, 1.0);
        assertEquals(writesAfterStop, f.servo.positionWrites());
        assertEquals(f.openNativePosition, f.servo.position(), 0.0);
    }

    @Test
    public void configurationIsSnapshottedAndMayReverseNativeEndpoints() {
        BasicClawMechanism.Config config = config();
        config.closedNativePosition = 0.85;
        config.openNativePosition = 0.15;
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.ServoProbe servo = hardware.addServo(config.servoName);
        BasicClawMechanism claw = new BasicClawMechanism(hardware, config);
        ManualLoopClock time = new ManualLoopClock();

        config.closedNativePosition = 0.05;
        config.openNativePosition = 0.95;
        claw.update(time.clock());
        assertEquals(0.85, servo.position(), 0.0);

        claw.setState(BasicClaw.State.OPEN);
        claw.update(time.nextCycle(0.02));
        assertEquals(0.15, servo.position(), 1e-12);
        assertStatus(claw, BasicClaw.State.OPEN, 1.0, 1.0);
    }

    @Test
    public void invalidConfigurationFailsBeforeHardwareLookup() {
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
        return new Fixture(config.openNativePosition, servo,
                new BasicClawMechanism(hardware, config));
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
                                     double expectedRequestedCoordinate,
                                     double expectedAppliedCoordinate) {
        BasicClaw.Status status = claw.status();
        assertEquals(expectedState, status.requestedState());
        assertEquals(expectedRequestedCoordinate, status.requestedCoordinate(), 0.0);
        assertEquals(expectedAppliedCoordinate, status.appliedCoordinate(), 0.0);
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
        private final double openNativePosition;
        private final FtcTestHardware.ServoProbe servo;
        private final BasicClawMechanism claw;

        private Fixture(double openNativePosition,
                        FtcTestHardware.ServoProbe servo,
                        BasicClawMechanism claw) {
            this.openNativePosition = openNativePosition;
            this.servo = servo;
            this.claw = claw;
        }
    }
}
