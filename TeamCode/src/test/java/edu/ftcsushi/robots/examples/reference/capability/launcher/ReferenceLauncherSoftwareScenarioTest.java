package edu.ftcsushi.robots.examples.reference.capability.launcher;

import org.junit.Test;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Reactive hardware-free scenarios with explicit observations and no simulated flywheel physics. */
public final class ReferenceLauncherSoftwareScenarioTest {

    private static final double CYCLE_SEC = 0.02;
    private static final double EPSILON = 1e-9;

    @Test
    public void launchReactsToIndependentMeasurementsThenRunsReleaseAndTransfer() {
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
                scenario.launcher.status().leftMeasuredVelocityTicksPerSec, EPSILON);
        assertEquals(0.0,
                scenario.launcher.status().rightMeasuredVelocityTicksPerSec, EPSILON);
        assertFalse("a recorded command is not measured feedback",
                scenario.launcher.status().ready);
        assertFeedIdle(scenario);

        double outsideTolerance = scenario.config.velocityToleranceTicksPerSec + 25.0;
        scenario.left.setMeasuredVelocityTicksPerSec(
                targetTicksPerSec + outsideTolerance);
        scenario.right.setMeasuredVelocityTicksPerSec(
                targetTicksPerSec - outsideTolerance);
        scenario.advance(CYCLE_SEC);
        assertFalse("opposite errors must not become ready by averaging",
                scenario.launcher.status().ready);
        assertFeedIdle(scenario);

        scenario.left.setMeasuredVelocityTicksPerSec(targetTicksPerSec);
        scenario.right.setMeasuredVelocityTicksPerSec(targetTicksPerSec);
        scenario.advance(CYCLE_SEC);
        assertTrue(scenario.launcher.status().leftAtTarget);
        assertTrue(scenario.launcher.status().rightAtTarget);
        assertTrue(scenario.launcher.status().ready);
        assertFeedIdle(scenario);

        scenario.advance(CYCLE_SEC);
        assertEquals(scenario.config.releaseExtendedPosition,
                scenario.release.position(), EPSILON);
        assertEquals("release must be observable before transfer starts",
                0.0, scenario.transfer.power(), EPSILON);
        assertFalse(scenario.launch.isComplete());

        scenario.advance(scenario.config.releaseDurationSec + CYCLE_SEC);
        assertEquals(scenario.config.releaseRetractedPosition,
                scenario.release.position(), EPSILON);
        assertEquals(scenario.config.transferPower,
                scenario.transfer.power(), EPSILON);
        assertTrue(scenario.launcher.status().transferPulseActive);
        assertFalse(scenario.launch.isComplete());

        scenario.advance(scenario.config.transferDurationSec + CYCLE_SEC);
        assertTrue(scenario.launch.isComplete());
        assertEquals(TaskOutcome.SUCCESS, scenario.launch.getOutcome());
        assertActiveMatchIdle(scenario);
    }

    @Test
    public void stalledWheelTimesOutWithoutFeedingThenCleansUp() {
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
        assertTrue(scenario.launcher.status().leftAtTarget);
        assertFalse(scenario.launcher.status().rightAtTarget);
        assertFalse(scenario.launcher.status().ready);
        assertFeedIdle(scenario);

        double finalCycleSec = 0.01;
        double preDeadlineSec = scenario.config.spinUpTimeoutSec - finalCycleSec;
        scenario.advance(preDeadlineSec - scenario.time.clock().nowSec());
        assertEquals(preDeadlineSec, scenario.time.clock().nowSec(), EPSILON);
        assertFalse(scenario.launch.isComplete());
        assertFeedIdle(scenario);

        scenario.advance(finalCycleSec);
        assertEquals(scenario.config.spinUpTimeoutSec,
                scenario.time.clock().nowSec(), EPSILON);
        assertTrue(scenario.launch.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, scenario.launch.getOutcome());
        assertActiveMatchIdle(scenario);
    }

    private static void assertActiveMatchIdle(Scenario scenario) {
        ReferenceLauncher.Status status = scenario.launcher.status();
        assertEquals(0.0, status.targetVelocityTicksPerSec, EPSILON);
        assertFalse(status.ready);
        assertEquals(0.0, scenario.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, scenario.right.commandedVelocityTicksPerSec(), EPSILON);
        assertFeedIdle(scenario);
    }

    private static void assertFeedIdle(Scenario scenario) {
        assertFalse(scenario.launcher.status().transferPulseActive);
        assertEquals(0.0, scenario.transfer.power(), EPSILON);
        assertEquals(scenario.config.releaseRetractedPosition,
                scenario.release.position(), EPSILON);
    }

    private static final class Scenario {
        private final ReferenceLauncherMechanism.Config config =
                ReferenceLauncherMechanism.Config.defaults();
        private final FtcTestHardware hardware = new FtcTestHardware();
        private final FtcTestHardware.MotorProbe left =
                hardware.addMotor(config.leftFlywheelName);
        private final FtcTestHardware.MotorProbe right =
                hardware.addMotor(config.rightFlywheelName);
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
