package edu.ftcsushi.robots.examples.reference.capability.launcher;

import org.junit.Test;

import java.lang.reflect.Field;

import edu.ftcsushi.fw.task.OutputTask;
import edu.ftcsushi.fw.task.OutputTaskRunner;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Maintainer contracts for the launcher's distinct feed, cleanup, and invalidation policy. */
public final class ReferenceLauncherMechanismTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void configValidatesNestedFlywheelsBeforeLookupAndSnapshotsEveryValue() {
        ReferenceLauncherMechanism.Config invalid = testConfig();
        invalid.flywheels.rightMotorName = "  " + invalid.flywheels.leftMotorName + "  ";
        FtcTestHardware rejected = new FtcTestHardware();

        IllegalArgumentException failure = assertThrows(
                IllegalArgumentException.class,
                () -> new ReferenceLauncherMechanism(rejected, invalid));
        assertTrue(failure.getMessage().contains("leftMotorName"));
        assertTrue(failure.getMessage().contains("rightMotorName"));
        assertEquals(0, rejected.lookupCalls());

        Rig rig = new Rig();
        rig.config.flywheels.maximumVelocityTicksPerSec = 1.0;
        rig.config.launchVelocityTicksPerSec = 1.0;
        rig.config.releaseRetractedNativePosition = 0.9;
        rig.mechanism.flywheels().setVelocityTicksPerSec(750.0);
        rig.mechanism.update(rig.time.clock());

        assertEquals(750.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.25, rig.release.position(), EPSILON);
    }

    @Test
    public void launchTasksAreFreshSingleUseAndPreStartCancellationHasNoEffect() {
        Rig rig = new Rig();
        Task first = rig.mechanism.launchOne();
        Task second = rig.mechanism.launchOne();
        assertNotSame(first, second);

        assertThrows(IllegalStateException.class, () -> second.update(rig.time.clock()));
        first.cancel();
        assertFalse(first.isComplete());

        first.start(rig.time.clock());
        assertThrows(IllegalStateException.class, () -> first.start(rig.time.clock()));
        first.cancel();
        rig.mechanism.update(rig.time.clock());

        assertEquals(TaskOutcome.CANCELLED, first.getOutcome());
        assertIdle(rig);
    }

    @Test
    public void successfulLaunchRunsReleaseThenTransferAndCleansEveryRequest() {
        Rig rig = new Rig();
        Task launch = rig.mechanism.launchOne();
        primeSpinUp(rig, launch);
        reachRelease(rig, launch);

        assertEquals(rig.config.releaseExtendedNativePosition,
                rig.release.position(), EPSILON);
        assertEquals(0.0, rig.transfer.power(), EPSILON);

        cycle(rig, launch, rig.config.releaseDurationSec);
        assertEquals(rig.config.releaseRetractedNativePosition,
                rig.release.position(), EPSILON);
        assertEquals(rig.config.transferPower, rig.transfer.power(), EPSILON);
        assertTrue(rig.mechanism.status().transferPulseActive());

        cycle(rig, launch, rig.config.transferDurationSec);
        assertEquals(TaskOutcome.SUCCESS, launch.getOutcome());
        assertIdle(rig);
    }

    @Test
    public void timeoutRemainsTimeoutAndNeverFeeds() {
        Rig rig = new Rig();
        Task launch = rig.mechanism.launchOne();
        primeSpinUp(rig, launch);

        cycle(rig, launch, rig.config.spinUpTimeoutSec);

        assertEquals(TaskOutcome.TIMEOUT, launch.getOutcome());
        assertIdle(rig);
    }

    @Test
    public void cancellationDuringEveryPhaseCleansSafely() {
        Rig spinUpRig = new Rig();
        Task spinUp = spinUpRig.mechanism.launchOne();
        primeSpinUp(spinUpRig, spinUp);
        cancelAndAssertIdle(spinUpRig, spinUp);

        Rig releaseRig = new Rig();
        Task release = releaseRig.mechanism.launchOne();
        primeSpinUp(releaseRig, release);
        reachRelease(releaseRig, release);
        cancelAndAssertIdle(releaseRig, release);

        Rig transferRig = new Rig();
        Task transfer = transferRig.mechanism.launchOne();
        primeSpinUp(transferRig, transfer);
        reachRelease(transferRig, transfer);
        cycle(transferRig, transfer, transferRig.config.releaseDurationSec);
        assertTrue(transferRig.mechanism.status().transferPulseActive());
        cancelAndAssertIdle(transferRig, transfer);
    }

    @Test
    public void abortInvalidatesOldTasksWithoutOverwritingLaterFlywheelIntent() {
        Rig rig = new Rig();
        Task active = rig.mechanism.launchOne();
        Task queued = rig.mechanism.launchOne();
        TaskRunner runner = new TaskRunner();
        runner.enqueue(active);
        runner.enqueue(queued);
        runner.update(rig.time.clock());
        rig.mechanism.update(rig.time.clock());

        rig.mechanism.abortLaunches();
        rig.mechanism.flywheels().setVelocityTicksPerSec(600.0);
        runner.update(rig.time.nextCycle(0.02));
        rig.mechanism.update(rig.time.clock());
        assertEquals(TaskOutcome.CANCELLED, active.getOutcome());
        assertEquals(600.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);

        runner.update(rig.time.nextCycle(0.02));
        rig.mechanism.update(rig.time.clock());
        assertEquals(TaskOutcome.CANCELLED, queued.getOutcome());
        assertEquals(600.0,
                rig.mechanism.status().flywheels().requestedVelocityTicksPerSec(), EPSILON);

        Task later = rig.mechanism.launchOne();
        runner.enqueue(later);
        runner.update(rig.time.nextCycle(0.02));
        rig.mechanism.update(rig.time.clock());
        assertFalse(later.isComplete());
        assertEquals(rig.config.launchVelocityTicksPerSec,
                rig.left.commandedVelocityTicksPerSec(), EPSILON);
        later.cancel();
    }

    @Test
    public void reversedReleaseEndpointsStillUseNormalizedRetractedAndExtendedIntent() {
        ReferenceLauncherMechanism.Config config = testConfig();
        config.releaseRetractedNativePosition = 0.85;
        config.releaseExtendedNativePosition = 0.15;
        Rig rig = new Rig(config);
        Task launch = rig.mechanism.launchOne();
        primeSpinUp(rig, launch);

        assertEquals(0.85, rig.release.position(), EPSILON);
        reachRelease(rig, launch);
        assertEquals(0.15, rig.release.position(), EPSILON);

        launch.cancel();
        rig.mechanism.update(rig.time.nextCycle(0.02));
        assertEquals(0.85, rig.release.position(), EPSILON);
    }

    @Test
    public void cleanupFailureLeavesTaskTerminalAndStillZerosOtherRequests() throws Exception {
        Rig rig = new Rig();
        OutputTaskRunner overrides = transferOverrides(rig.mechanism);
        overrides.enqueue(new ThrowingCancelOutputTask());
        overrides.update(rig.time.clock());
        Task launch = rig.mechanism.launchOne();
        launch.start(rig.time.clock());

        IllegalStateException failure = assertThrows(IllegalStateException.class, launch::cancel);

        assertTrue(failure.getMessage().contains("injected transfer cleanup failure"));
        assertEquals(TaskOutcome.CANCELLED, launch.getOutcome());
        rig.mechanism.update(rig.time.nextCycle(0.02));
        assertEquals(0.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(rig.config.releaseRetractedNativePosition,
                rig.release.position(), EPSILON);
    }

    private static void primeSpinUp(Rig rig, Task launch) {
        launch.start(rig.time.clock());
        launch.update(rig.time.clock());
        rig.mechanism.update(rig.time.clock());
    }

    private static void reachRelease(Rig rig, Task launch) {
        rig.left.setMeasuredVelocityTicksPerSec(rig.config.launchVelocityTicksPerSec);
        rig.right.setMeasuredVelocityTicksPerSec(rig.config.launchVelocityTicksPerSec);
        cycle(rig, launch, 0.02);
        cycle(rig, launch, 0.02);
        assertTrue(rig.mechanism.status().flywheels().ready());
    }

    private static void cycle(Rig rig, Task task, double dtSec) {
        task.update(rig.time.nextCycle(dtSec));
        rig.mechanism.update(rig.time.clock());
    }

    private static void cancelAndAssertIdle(Rig rig, Task task) {
        task.cancel();
        rig.mechanism.update(rig.time.nextCycle(0.02));
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertIdle(rig);
    }

    private static void assertIdle(Rig rig) {
        ReferenceLauncher.Status status = rig.mechanism.status();
        assertEquals(0.0, status.flywheels().requestedVelocityTicksPerSec(), EPSILON);
        assertFalse(status.flywheels().ready());
        assertFalse(status.transferPulseActive());
        assertEquals(0.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, rig.right.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, rig.transfer.power(), EPSILON);
        assertEquals(rig.config.releaseRetractedNativePosition,
                rig.release.position(), EPSILON);
    }

    private static OutputTaskRunner transferOverrides(ReferenceLauncherMechanism mechanism)
            throws Exception {
        Field field = ReferenceLauncherMechanism.class.getDeclaredField("transferOverrides");
        field.setAccessible(true);
        return (OutputTaskRunner) field.get(mechanism);
    }

    private static ReferenceLauncherMechanism.Config testConfig() {
        ReferenceLauncherMechanism.Config config =
                ReferenceLauncherMechanism.Config.defaults();
        config.flywheels.leftMotorName = "left";
        config.flywheels.rightMotorName = "right";
        config.flywheels.maximumVelocityTicksPerSec = 2000.0;
        config.flywheels.velocityToleranceTicksPerSec = 50.0;
        config.launchVelocityTicksPerSec = 1000.0;
        config.spinUpTimeoutSec = 1.0;
        config.releaseDurationSec = 0.10;
        config.transferDurationSec = 0.20;
        return config;
    }

    private static final class Rig {
        private final ReferenceLauncherMechanism.Config config;
        private final FtcTestHardware hardware = new FtcTestHardware();
        private final FtcTestHardware.MotorProbe left;
        private final FtcTestHardware.MotorProbe right;
        private final FtcTestHardware.CrServoProbe transfer;
        private final FtcTestHardware.ServoProbe release;
        private final ManualLoopClock time = new ManualLoopClock();
        private final ReferenceLauncherMechanism mechanism;

        private Rig() {
            this(testConfig());
        }

        private Rig(ReferenceLauncherMechanism.Config config) {
            this.config = config;
            left = hardware.addMotor(config.flywheels.leftMotorName);
            right = hardware.addMotor(config.flywheels.rightMotorName);
            transfer = hardware.addCrServo(config.transferName);
            release = hardware.addServo(config.releaseServoName);
            hardware.addDigitalInput(config.objectSensorName).setHigh(true);
            mechanism = new ReferenceLauncherMechanism(hardware, config);
        }
    }

    private static final class ThrowingCancelOutputTask implements OutputTask {
        private boolean started;
        private boolean complete;

        @Override
        public void start(edu.ftcsushi.fw.core.time.LoopClock clock) {
            started = true;
        }

        @Override
        public void update(edu.ftcsushi.fw.core.time.LoopClock clock) {
            if (!started) throw new IllegalStateException("update before start");
        }

        @Override
        public void cancel() {
            if (!started || complete) return;
            complete = true;
            throw new IllegalStateException("injected transfer cleanup failure");
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }

        @Override
        public double getOutput() {
            return 0.2;
        }
    }
}
