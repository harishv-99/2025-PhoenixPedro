package edu.ftcsushi.robots.examples.reference.capability.launcher;

import org.junit.Test;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Causal software scenarios for the launcher's feed policy after paired-wheel readiness.
 *
 * <p>Question: does one real launch Task wait for independent readiness, order release before
 * transfer, preserve outcomes, and clean its requests? Keep real: launcher, Plants, Task graph,
 * and heartbeat. Replace: FTC devices only. Observe: commands, cached status, and Task outcome.
 * This cannot prove safe motion, flywheel balance under load, object release, or scoring.</p>
 */
public final class ReferenceLauncherSoftwareScenarioTest {

    private static final double CYCLE_SEC = 0.02;
    private static final double EPSILON = 1e-9;

    @Test
    public void launchReactsToIndependentMeasurementsThenRunsReleaseAndTransfer() {
        // ARRANGE / REQUEST: start one fresh launch with zero measured wheel velocity.
        Scenario scenario = new Scenario();
        scenario.launch = scenario.launcher.launchOne();
        scenario.launch.start(scenario.time.clock());
        scenario.launch.update(scenario.time.clock());
        scenario.launcher.update(scenario.time.clock());

        double targetTicksPerSec = scenario.config.launchVelocityTicksPerSec;
        assertEquals(targetTicksPerSec,
                scenario.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(targetTicksPerSec,
                scenario.right.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0,
                scenario.launcher.status().flywheels()
                        .leftMeasuredVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0,
                scenario.launcher.status().flywheels()
                        .rightMeasuredVelocityTicksPerSec(), EPSILON);
        assertFalse("a recorded command is not measured feedback",
                scenario.launcher.status().flywheels().ready());
        assertFeedIdle(scenario);

        // INJECT EVIDENCE: a correct group mean is insufficient when both members are wrong.
        double outsideTolerance =
                scenario.config.flywheels.velocityToleranceTicksPerSec + 25.0;
        scenario.left.setMeasuredVelocityTicksPerSec(
                targetTicksPerSec + outsideTolerance);
        scenario.right.setMeasuredVelocityTicksPerSec(
                targetTicksPerSec - outsideTolerance);
        scenario.advance(CYCLE_SEC);
        assertFalse("opposite errors must not become ready by averaging",
                scenario.launcher.status().flywheels().ready());
        assertFeedIdle(scenario);

        // INJECT EVIDENCE: both independent measurements now justify the feed transition.
        scenario.left.setMeasuredVelocityTicksPerSec(targetTicksPerSec);
        scenario.right.setMeasuredVelocityTicksPerSec(targetTicksPerSec);
        // HEARTBEAT / ASSERT: release is observable before the temporary transfer pulse.
        scenario.advance(CYCLE_SEC);
        assertTrue(scenario.launcher.status().flywheels().leftAtTarget());
        assertTrue(scenario.launcher.status().flywheels().rightAtTarget());
        assertTrue(scenario.launcher.status().flywheels().ready());
        assertFeedIdle(scenario);

        scenario.advance(CYCLE_SEC);
        assertEquals(scenario.config.releaseExtendedNativePosition,
                scenario.release.position(), EPSILON);
        assertEquals("release must be observable before transfer starts",
                0.0, scenario.transfer.power(), EPSILON);
        assertFalse(scenario.launch.isComplete());

        scenario.advance(scenario.config.releaseDurationSec + CYCLE_SEC);
        assertEquals(scenario.config.releaseRetractedNativePosition,
                scenario.release.position(), EPSILON);
        assertEquals(scenario.config.transferPower,
                scenario.transfer.power(), EPSILON);
        assertTrue(scenario.launcher.status().transferPulseActive());
        assertFalse(scenario.launch.isComplete());

        // ASSERT: natural success retains its outcome and leaves every mechanism request idle.
        scenario.advance(scenario.config.transferDurationSec + CYCLE_SEC);
        assertTrue(scenario.launch.isComplete());
        assertEquals(TaskOutcome.SUCCESS, scenario.launch.getOutcome());
        assertActiveMatchIdle(scenario);
    }

    @Test
    public void stalledWheelTimesOutWithoutFeedingThenCleansUp() {
        // ARRANGE / REQUEST: one wheel can reach the request while its partner remains stalled.
        Scenario scenario = new Scenario();
        scenario.launch = scenario.launcher.launchOne();
        scenario.launch.start(scenario.time.clock());
        scenario.launch.update(scenario.time.clock());
        scenario.launcher.update(scenario.time.clock());

        double targetTicksPerSec = scenario.config.launchVelocityTicksPerSec;
        assertEquals(targetTicksPerSec,
                scenario.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(targetTicksPerSec,
                scenario.right.commandedVelocityTicksPerSec(), EPSILON);
        assertFeedIdle(scenario);

        scenario.left.setMeasuredVelocityTicksPerSec(targetTicksPerSec);
        scenario.right.setMeasuredVelocityTicksPerSec(0.0);
        scenario.advance(CYCLE_SEC);
        assertTrue(scenario.launcher.status().flywheels().leftAtTarget());
        assertFalse(scenario.launcher.status().flywheels().rightAtTarget());
        assertFalse(scenario.launcher.status().flywheels().ready());
        assertFeedIdle(scenario);

        double finalCycleSec = 0.01;
        double preDeadlineSec = scenario.config.spinUpTimeoutSec - finalCycleSec;
        scenario.advance(preDeadlineSec - scenario.time.clock().nowSec());
        assertEquals(preDeadlineSec, scenario.time.clock().nowSec(), EPSILON);
        assertFalse(scenario.launch.isComplete());
        assertFeedIdle(scenario);

        // HEARTBEAT / ASSERT: the exact deadline reports TIMEOUT and never starts feed.
        scenario.advance(finalCycleSec);
        assertEquals(scenario.config.spinUpTimeoutSec,
                scenario.time.clock().nowSec(), EPSILON);
        assertTrue(scenario.launch.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, scenario.launch.getOutcome());
        assertActiveMatchIdle(scenario);
    }

    private static void assertActiveMatchIdle(Scenario scenario) {
        ReferenceLauncher.Status status = scenario.launcher.status();
        assertEquals(0.0, status.flywheels().requestedVelocityTicksPerSec(), EPSILON);
        assertFalse(status.flywheels().ready());
        assertEquals(0.0, scenario.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, scenario.right.commandedVelocityTicksPerSec(), EPSILON);
        assertFeedIdle(scenario);
    }

    private static void assertFeedIdle(Scenario scenario) {
        assertFalse(scenario.launcher.status().transferPulseActive());
        assertEquals(0.0, scenario.transfer.power(), EPSILON);
        assertEquals(scenario.config.releaseRetractedNativePosition,
                scenario.release.position(), EPSILON);
    }

    private static final class Scenario {
        private final ReferenceLauncherMechanism.Config config =
                ReferenceLauncherMechanism.Config.defaults();
        private final FtcTestHardware hardware = new FtcTestHardware();
        private final FtcTestHardware.MotorProbe left =
                hardware.addMotor(config.flywheels.leftMotorName);
        private final FtcTestHardware.MotorProbe right =
                hardware.addMotor(config.flywheels.rightMotorName);
        private final FtcTestHardware.CrServoProbe transfer =
                hardware.addCrServo(config.transferName);
        private final FtcTestHardware.ServoProbe release =
                hardware.addServo(config.releaseServoName);
        private final FtcTestHardware.DigitalProbe objectSensor =
                hardware.addDigitalInput(config.objectSensorName);
        private final ManualLoopClock time = new ManualLoopClock();
        private final ReferenceLauncherMechanism launcher;

        private Task launch;

        private Scenario() {
            left.setMeasuredVelocityTicksPerSec(0.0);
            right.setMeasuredVelocityTicksPerSec(0.0);
            objectSensor.setHigh(true); // HIGH is explicit active-low evidence for no object.
            launcher = new ReferenceLauncherMechanism(hardware, config);
        }

        /** Advance one cycle in managed Task-before-output order. */
        private void advance(double dtSec) {
            time.nextCycle(dtSec);
            launch.update(time.clock());
            launcher.update(time.clock());
        }
    }
}
