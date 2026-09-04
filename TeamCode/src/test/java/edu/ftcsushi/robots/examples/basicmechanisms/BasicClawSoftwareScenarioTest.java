package edu.ftcsushi.robots.examples.basicmechanisms;

import org.junit.Test;

import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;

/**
 * Beginner software proof for one named-position standard servo.
 *
 * <p>The test separates the team's semantic request, Sushi's managed output heartbeat, and the
 * command recorded by a software Servo probe. The probe does not simulate linkage motion, so this
 * scenario intentionally makes no physical-arrival assertion.</p>
 */
public final class BasicClawSoftwareScenarioTest {

    @Test
    public void closedHalfAndOpenMapAcrossOneConfiguredNativeRange() {
        // ARRANGE: author two software-valid endpoint candidates and register the named Servo.
        BasicClawMechanism.Config config = BasicClawMechanism.Config.defaults();
        config.closedNativePosition = 0.25;
        config.openNativePosition = 0.70;
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.ServoProbe servo = hardware.addServo(config.servoName);
        BasicClawMechanism claw = new BasicClawMechanism(hardware, config);
        ManualLoopClock time = new ManualLoopClock();

        // CLOSED: the default 0.0 request maps to the configured native lower endpoint.
        assertEquals(0, servo.positionWrites());
        claw.update(time.clock());
        assertEquals(BasicClaw.State.CLOSED, claw.status().requestedState());
        assertEquals(0.0, claw.status().appliedCoordinate(), 0.0);
        assertEquals(0.25, servo.position(), 0.0);
        assertEquals(1, servo.positionWrites());

        // HALF: semantic intent changes now; the normal heartbeat derives the native midpoint.
        claw.setState(BasicClaw.State.HALF);
        assertEquals(BasicClaw.State.HALF, claw.status().requestedState());
        assertEquals(0.5, claw.status().requestedCoordinate(), 0.0);
        assertEquals(1, servo.positionWrites());
        claw.update(time.nextCycle(0.02));
        assertEquals(0.5, claw.status().appliedCoordinate(), 0.0);
        assertEquals(0.475, servo.position(), 1e-12);
        assertEquals(2, servo.positionWrites());

        // OPEN: the same mapping sends normalized 1.0 to the configured native upper endpoint.
        claw.setState(BasicClaw.State.OPEN);
        claw.update(time.nextCycle(0.02));
        assertEquals(1.0, claw.status().appliedCoordinate(), 0.0);
        assertEquals(0.70, servo.position(), 1e-12);
        assertEquals(3, servo.positionWrites());

        // These are recorded commands, not arrival evidence. Verify travel on the real mechanism.
    }
}
