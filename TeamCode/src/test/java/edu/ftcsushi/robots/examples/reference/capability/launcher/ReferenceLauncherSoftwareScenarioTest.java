package edu.ftcsushi.robots.examples.reference.capability.launcher;

import org.junit.Test;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.*;

/**
 * Question: do completed feed commands alone prove the staged position became vacant?
 * Keep real: launcher, inventory, Plants, Tasks, shared clock, and Task-before-output order.
 * Replace: FTC devices, with independently authored speeds and electrical sensor levels.
 * Observe: commands, conditioned staged position, and exact Task outcome.
 * Cannot conclude: physical motion, safe interruption, launch, or scoring.
 */
public final class ReferenceLauncherSoftwareScenarioTest {
    @Test
    public void completedCommandsWaitForAnObservedDeparture() {
        // ARRANGE / REQUEST: first position occupied; both independent readings are 1000 ticks/s.
        Scenario s = new Scenario();
        Task feed = s.launcher.feedOne();
        feed.start(s.time.clock());
        feed.update(s.time.clock());
        s.launcher.update(s.time.clock());
        assertFalse("a request is not a completed feed", feed.isComplete());

        // HEARTBEAT: sampled settling, release, then transfer all run in normal loop order.
        s.advance(feed, 20);
        assertEquals(ReferenceLauncher.Phase.CONFIRMING, s.launcher.status().phase());
        assertEquals(0.0, s.transfer.power(), 1e-9);
        assertEquals(s.config.releaseRetractedNativePosition, s.release.position(), 1e-9);
        assertTrue(s.launcher.status().inventory().firstPositionOccupied);
        assertEquals("finished commands do not establish vacancy", TaskOutcome.NOT_DONE,
                feed.getOutcome());

        // INJECT EVIDENCE: HIGH means vacant for this active-low staged sensor.
        s.staged.setHigh(true);
        s.advance(feed, 1); // Output publishes the new observation after this cycle's Task.
        assertFalse(feed.isComplete());
        s.advance(feed, 1); // The next Task phase consumes that later observation.
        assertEquals(TaskOutcome.SUCCESS, feed.getOutcome());
        assertEquals(ReferenceLauncher.Reason.DEPARTURE_OBSERVED, s.launcher.status().reason());
        assertEquals(0.0, s.left.commandedVelocityTicksPerSec(), 1e-9);
        assertFalse(s.launcher.status().recoveryRequired());
        // NEXT GATE: validate actual staging, loaded-wheel response, and safe interruption on robot.
    }

    /** Only outside readings are authored; recorded commands never become simulated feedback. */
    private static final class Scenario {
        final ReferenceLauncherMechanism.Config config = ReferenceLauncherMechanism.Config.defaults();
        final FtcTestHardware hardware = new FtcTestHardware();
        final ManualLoopClock time = new ManualLoopClock();
        final FtcTestHardware.MotorProbe left;
        final FtcTestHardware.CrServoProbe transfer;
        final FtcTestHardware.ServoProbe release;
        final FtcTestHardware.DigitalProbe staged;
        final ReferenceLauncherMechanism launcher;

        Scenario() {
            // Explicit software-only timings, not reviewed hardware settings.
            config.feedVelocityTicksPerSec = 1000.0;
            config.readySettlingSec = 0.125;
            config.evidenceMaxAgeSec = 0.25;
            config.releaseDurationSec = 0.125;
            config.transferDurationSec = 0.25;
            config.departureTimeoutSec = 0.75;
            config.inventory.occupiedDebounceSec = 0.0;
            config.inventory.vacatedDebounceSec = 0.0;
            left = hardware.addMotor(config.flywheels.leftMotorName);
            FtcTestHardware.MotorProbe right = hardware.addMotor(config.flywheels.rightMotorName);
            left.setMeasuredVelocityTicksPerSec(1000.0);
            right.setMeasuredVelocityTicksPerSec(1000.0);
            transfer = hardware.addCrServo(config.transferName);
            release = hardware.addServo(config.releaseServoName);
            staged = hardware.addDigitalInput(config.inventory.firstPositionSensorName);
            staged.setHigh(false);
            hardware.addDigitalInput(config.inventory.secondPositionSensorName).setHigh(true);
            hardware.addDigitalInput(config.inventory.thirdPositionSensorName).setHigh(true);
            launcher = new ReferenceLauncherMechanism(hardware, config);
        }

        void advance(Task feed, int cycles) {
            for (int i = 0; i < cycles; i++) {
                time.nextCycle(0.03125);
                feed.update(time.clock());
                launcher.update(time.clock());
            }
        }
    }
}
