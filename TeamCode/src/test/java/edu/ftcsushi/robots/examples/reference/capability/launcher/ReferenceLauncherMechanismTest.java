package edu.ftcsushi.robots.examples.reference.capability.launcher;

import org.junit.Test;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncher.Phase;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncher.Reason;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncher.RecoveryResult;

import static edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncherTestRig.EPS;
import static edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncherTestRig.STEP;
import static org.junit.Assert.*;

/** Maintainer evidence for sampled feed policy; these probes model no mechanism physics. */
public final class ReferenceLauncherMechanismTest {
    @Test public void invalidConfigurationFailsBeforeAnyHardwareLookup() {
        invalid(c -> c.flywheels.rightMotorName = " " + c.flywheels.leftMotorName + " ");
        invalid(c -> c.inventory.secondPositionSensorName = c.inventory.firstPositionSensorName);
        invalid(c -> c.inventory.occupiedDebounceSec = Double.NaN);
        invalid(c -> c.feedVelocityTicksPerSec = Double.POSITIVE_INFINITY);
        invalid(c -> c.feedVelocityTicksPerSec = c.flywheels.velocityToleranceTicksPerSec);
        invalid(c -> c.readySettlingSec = Double.NaN);
        invalid(c -> c.readySettlingSec = -1.0);
        invalid(c -> c.readySettlingSec = c.spinUpTimeoutSec);
        invalid(c -> c.evidenceMaxAgeSec = 0.0);
        invalid(c -> c.departureTimeoutSec = Double.POSITIVE_INFINITY);
        invalid(c -> c.departureTimeoutSec = c.releaseDurationSec + c.transferDurationSec);
        invalid(c -> c.releaseDurationSec = 0.0);
        invalid(c -> c.transferDurationSec = -1.0);
        invalid(c -> c.transferPower = 1.1);
        invalid(c -> c.releaseExtendedNativePosition = c.releaseRetractedNativePosition);
    }

    @Test public void configurationIsCapturedIncludingNestedEvidencePolicy() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        r.config.flywheels.maximumVelocityTicksPerSec = 1.0;
        r.config.feedVelocityTicksPerSec = 1.0;
        r.config.readySettlingSec = 100.0;
        r.config.inventory.occupiedDebounceSec = 100.0;
        r.config.releaseRetractedNativePosition = 0.9;
        Task task = r.start();
        r.reach(task, Phase.RELEASING);
        assertEquals(1000.0, r.left.commandedVelocityTicksPerSec(), EPS);
        task.cancel();
        r.outputCycle();
        assertEquals(0.25, r.release.position(), EPS);
    }

    @Test public void laterConstructorFailureStopsAlreadyAcquiredActuators() {
        ReferenceLauncherMechanism.Config c = ReferenceLauncherTestRig.config();
        FtcTestHardware hardware = new FtcTestHardware();
        hardware.addDigitalInput(c.inventory.firstPositionSensorName);
        hardware.addDigitalInput(c.inventory.secondPositionSensorName);
        hardware.addDigitalInput(c.inventory.thirdPositionSensorName);
        FtcTestHardware.MotorProbe left = hardware.addMotor(c.flywheels.leftMotorName);
        FtcTestHardware.MotorProbe right = hardware.addMotor(c.flywheels.rightMotorName);
        FtcTestHardware.CrServoProbe transfer = hardware.addCrServo(c.transferName);
        // The release lookup fails after real flywheel and transfer construction.
        assertThrows(IllegalArgumentException.class,
                () -> new ReferenceLauncherMechanism(hardware, c));
        assertTrue(left.powerWrites() > 0);
        assertTrue(right.powerWrites() > 0);
        assertTrue(transfer.powerWrites() > 0);
        assertEquals(0.0, left.power(), EPS);
        assertEquals(0.0, right.power(), EPS);
        assertEquals(0.0, transfer.power(), EPS);
    }

    @Test public void tasksAreFreshSingleUseAndPreStartCancellationIsInert() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task first = r.mechanism.feedOne();
        assertNotSame(first, r.mechanism.feedOne());
        assertThrows(IllegalStateException.class, () -> first.update(r.time.clock()));
        first.cancel();
        assertFalse(first.isComplete());
        assertEquals(0, r.left.velocityWrites());
        first.start(r.time.clock());
        assertThrows(IllegalStateException.class, () -> first.start(r.time.clock()));
        first.cancel();
        r.mechanism.update(r.time.clock());
        assertEquals(TaskOutcome.CANCELLED, first.getOutcome());
        assertFalse(r.mechanism.status().recoveryRequired());
        r.assertIdle();
    }

    @Test public void noStagedObjectTimesOutWithoutReleaseOrRecovery() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        r.staged(false);
        Task task = r.start();
        r.finish(task);
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(Reason.PREREQUISITE_TIMEOUT, r.mechanism.status().reason());
        assertEquals(Phase.SETTLING, r.mechanism.status().phase());
        assertFalse(r.mechanism.status().recoveryRequired());
        assertEquals(RecoveryResult.NOT_REQUIRED, r.mechanism.acknowledgeRecovery());
        r.assertIdle();
    }

    @Test public void contradictoryInventoryCannotAdmitWithStagedObjectAndReadyWheels() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        r.third.setHigh(false); // First + third occupied, second vacant: ordered-fill gap.
        Task task = r.start();
        r.finish(task);
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(Reason.PREREQUISITE_TIMEOUT, r.mechanism.status().reason());
        r.assertIdle();
    }

    @Test public void oppositeWheelErrorsCannotHideBehindCorrectGroupMean() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        r.measured(800.0, 1200.0);
        Task task = r.start();
        assertTrue(r.mechanism.status().flywheels().plantSnapshot().atCommandTarget());
        assertFalse(r.mechanism.status().flywheels().ready());
        r.finish(task);
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        r.assertIdle();
    }

    @Test public void fleetingReadinessDoesNotBridgeABadSample() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        r.cycles(task, 3);
        r.measured(1000.0, 800.0);
        r.cycles(task, 2); // Publish the bad sample, then consume it.
        r.measured(1000.0, 1000.0);
        r.cycles(task, 4);
        assertEquals(Phase.SETTLING, r.mechanism.status().phase());
        r.reach(task, Phase.RELEASING);
    }

    @Test public void exactSampledSettlingBoundaryStartsReleaseButNotBefore() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        r.cycles(task, 4); // Task time .125 consumes the preceding sample at .09375.
        assertEquals(0.125, r.time.clock().nowSec(), EPS);
        assertEquals(Phase.SETTLING, r.mechanism.status().phase());
        r.assertFeedIdle();
        r.cycle(task); // The sample at .125 spans exactly .125 from the first sample at zero.
        assertEquals(Phase.RELEASING, r.mechanism.status().phase());
        assertEquals(r.config.releaseExtendedNativePosition, r.release.position(), EPS);
    }

    @Test public void readyAtPrerequisiteDeadlineWinsButLateProcessingDoesNot() {
        for (boolean late : new boolean[]{false, true}) {
            ReferenceLauncherMechanism.Config c = ReferenceLauncherTestRig.config();
            c.spinUpTimeoutSec = 0.125;
            c.readySettlingSec = 0.0625;
            ReferenceLauncherTestRig r = new ReferenceLauncherTestRig(c);
            r.measured(0.0, 0.0);
            Task task = r.start();
            r.measured(1000.0, 1000.0);
            r.cycles(task, 3); // Ready samples .03125, .0625, .09375.
            r.cycle(task, late ? 2 * STEP : STEP);
            if (late) {
                assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
                assertEquals(Reason.PREREQUISITE_TIMEOUT, r.mechanism.status().reason());
                r.assertIdle();
            } else {
                assertEquals(Phase.RELEASING, r.mechanism.status().phase());
                assertFalse(task.isComplete());
            }
        }
    }

    @Test public void sameCycleRereadsDoNotRepublishOrAccumulateSettling() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        ReferenceLauncher.Status captured = r.mechanism.status();
        int reads = r.first.stateReadCalls();
        int writes = r.left.velocityWrites();
        for (int i = 0; i < 20; i++) {
            task.update(r.time.clock());
            r.mechanism.update(r.time.clock());
        }
        assertSame(captured, r.mechanism.status());
        assertEquals(reads, r.first.stateReadCalls());
        assertEquals(writes, r.left.velocityWrites());
        assertEquals(Phase.SETTLING, r.mechanism.status().phase());
        assertEquals(0.0, captured.sampledAt().ageSec(r.time.clock()), EPS);
        assertEquals(r.time.clock().cycle(), captured.sampleCycle());
    }

    @Test public void repeatedCachedEvidenceCannotAccumulateASettlingInterval() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        for (int i = 0; i < 7; i++) task.update(r.time.nextCycle(STEP));
        assertEquals(Phase.SETTLING, r.mechanism.status().phase());
        assertFalse(task.isComplete());
        r.assertFeedIdle();
    }

    @Test public void observationGapResetsSettlingInsteadOfProvingDwell() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        r.cycles(task, 3);
        r.cycle(task, r.config.evidenceMaxAgeSec + STEP);
        r.cycles(task, 4);
        assertEquals(Phase.SETTLING, r.mechanism.status().phase());
        r.reach(task, Phase.RELEASING);
    }

    @Test public void sameValueAndAwayThenBackRequestsRestartSettling() {
        for (boolean awayFirst : new boolean[]{false, true}) {
            ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
            Task task = r.start();
            r.cycles(task, 3);
            if (awayFirst) r.mechanism.flywheels().setVelocityTicksPerSec(600.0);
            r.mechanism.flywheels().setVelocityTicksPerSec(1000.0);
            r.cycles(task, 4);
            assertEquals("matching numbers are not the original request", Phase.SETTLING,
                    r.mechanism.status().phase());
            r.reach(task, Phase.RELEASING);
        }
    }

    @Test public void differentRequestDuringSettlingIsNotFought() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        r.mechanism.flywheels().setVelocityTicksPerSec(600.0);
        r.cycles(task, 4);
        assertEquals(600.0, r.left.commandedVelocityTicksPerSec(), EPS);
        assertEquals(Phase.SETTLING, r.mechanism.status().phase());
        r.finish(task);
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertFalse(r.mechanism.status().recoveryRequired());
    }

    @Test public void departureDoesNotSkipTimedPhasesAndSuccessRemainsExact() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        r.reach(task, Phase.RELEASING);
        assertEquals(r.config.releaseExtendedNativePosition, r.release.position(), EPS);
        assertEquals(0.0, r.transfer.power(), EPS);
        r.cycle(task); // Consume occupied evidence from the realized-release cycle.
        r.staged(false);
        r.cycles(task, 2); // Publish then consume vacancy while release is still bounded.
        assertFalse(task.isComplete());
        r.reach(task, Phase.TRANSFERRING);
        assertEquals(r.config.transferPower, r.transfer.power(), EPS);
        assertTrue(r.mechanism.status().transferActive());
        assertEquals(r.config.releaseRetractedNativePosition, r.release.position(), EPS);
        r.finish(task);
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(Reason.DEPARTURE_OBSERVED, r.mechanism.status().reason());
        assertFalse(r.mechanism.status().recoveryRequired());
        r.assertIdle();
        terminalDoesNotOwnLaterIntent(r, task, TaskOutcome.SUCCESS);
    }

    @Test public void vacancyBeforeFirstRealizedReleaseCannotProveDeparture() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        for (int i = 0; i < 20 && r.mechanism.status().phase() != Phase.RELEASING; i++) {
            task.update(r.time.nextCycle(STEP));
            if (r.mechanism.status().phase() == Phase.RELEASING) r.staged(false);
            r.mechanism.update(r.time.clock());
        }
        assertEquals(Phase.RELEASING, r.mechanism.status().phase());
        r.finish(task);
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(Reason.DEPARTURE_TIMEOUT, r.mechanism.status().reason());
        assertTrue(r.mechanism.status().recoveryRequired());
    }

    @Test public void noDepartureTimesOutAfterBothCommandsComplete() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        r.reach(task, Phase.CONFIRMING);
        assertEquals(0.0, r.transfer.power(), EPS);
        assertFalse(task.isComplete());
        r.finish(task);
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(Reason.DEPARTURE_TIMEOUT, r.mechanism.status().reason());
        assertEquals(Phase.CONFIRMING, r.mechanism.status().phase());
        assertTrue(r.mechanism.status().recoveryRequired());
        r.assertIdle();
        terminalDoesNotOwnLaterIntent(r, task, TaskOutcome.TIMEOUT);
    }

    @Test public void eachTimedPhaseAndDepartureBoundUsesItsOwnExactStart() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        r.reach(task, Phase.RELEASING);
        double releaseStart = r.time.clock().nowSec();
        r.cycles(task, 3);
        assertEquals(Phase.RELEASING, r.mechanism.status().phase());
        assertEquals(r.config.releaseExtendedNativePosition, r.release.position(), EPS);
        r.cycle(task);
        assertEquals(releaseStart + 0.125, r.time.clock().nowSec(), EPS);
        assertEquals(Phase.TRANSFERRING, r.mechanism.status().phase());
        r.cycles(task, 7);
        assertEquals(Phase.TRANSFERRING, r.mechanism.status().phase());
        assertEquals(r.config.transferPower, r.transfer.power(), EPS);
        r.cycle(task);
        assertEquals(releaseStart + 0.375, r.time.clock().nowSec(), EPS);
        assertEquals(Phase.CONFIRMING, r.mechanism.status().phase());
        assertEquals(0.0, r.transfer.power(), EPS);
        r.cycles(task, 11);
        assertFalse(task.isComplete());
        r.cycle(task);
        assertEquals(releaseStart + r.config.departureTimeoutSec, r.time.clock().nowSec(), EPS);
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(Reason.DEPARTURE_TIMEOUT, r.mechanism.status().reason());
    }

    @Test public void movementOrderGapDoesNotInventAJam() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        r.second.setHigh(false); // Admission: first and second occupied, consistent.
        Task task = r.start();
        r.reach(task, Phase.RELEASING);
        r.cycle(task);
        r.staged(false); // Moving objects may create SECOND_WITHOUT_FIRST.
        r.finish(task);
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(Reason.DEPARTURE_OBSERVED, r.mechanism.status().reason());
    }

    @Test public void wheelDroopDuringEveryFeedPhaseRequiresRecovery() {
        for (Phase phase : new Phase[]{Phase.RELEASING, Phase.TRANSFERRING, Phase.CONFIRMING}) {
            ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
            Task task = r.start();
            r.reach(task, phase);
            r.measured(1000.0, 800.0);
            r.cycles(task, 2);
            assertEquals(phase.name(), TaskOutcome.CANCELLED, task.getOutcome());
            assertEquals(Reason.WHEEL_SPEED_LOST, r.mechanism.status().reason());
            assertTrue(r.mechanism.status().recoveryRequired());
            r.assertIdle();
        }
    }

    @Test public void staleEvidenceDuringFeedCancels() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        r.reach(task, Phase.RELEASING);
        task.update(r.time.nextCycle(r.config.evidenceMaxAgeSec + STEP));
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(Reason.EVIDENCE_LOST, r.mechanism.status().reason());
        assertTrue(r.mechanism.status().recoveryRequired());
        r.mechanism.update(r.time.clock());
        r.assertIdle();
    }

    @Test public void sameValuedRequestDuringFeedLosesOwnership() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        r.reach(task, Phase.RELEASING);
        r.mechanism.flywheels().setVelocityTicksPerSec(1000.0);
        r.cycle(task);
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(Reason.REQUEST_CHANGED, r.mechanism.status().reason());
        assertTrue(r.mechanism.status().recoveryRequired());
        r.assertIdle();
    }

    @Test public void cancellationAndStopAreTerminalInEveryPhase() {
        for (Phase phase : new Phase[]{Phase.SETTLING, Phase.RELEASING,
                Phase.TRANSFERRING, Phase.CONFIRMING}) {
            ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
            Task task = r.start();
            r.reach(task, phase);
            task.cancel();
            task.cancel();
            r.outputCycle();
            assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
            assertEquals(Reason.CANCELLED, r.mechanism.status().reason());
            assertEquals(phase != Phase.SETTLING, r.mechanism.status().recoveryRequired());
            r.assertIdle();

            ReferenceLauncherTestRig stopped = new ReferenceLauncherTestRig();
            Task old = stopped.start();
            stopped.reach(old, phase);
            stopped.mechanism.stop();
            int writes = stopped.left.velocityWrites();
            int reads = stopped.first.stateReadCalls();
            stopped.mechanism.stop();
            stopped.mechanism.update(stopped.time.nextCycle(STEP));
            old.update(stopped.time.clock());
            assertEquals(TaskOutcome.CANCELLED, old.getOutcome());
            assertEquals(Reason.STOPPED, stopped.mechanism.status().reason());
            assertEquals(writes, stopped.left.velocityWrites());
            assertEquals(reads, stopped.first.stateReadCalls());
            assertFalse(stopped.mechanism.status().sampledAt().isAvailable());
            assertEquals(RecoveryResult.STOPPED, stopped.mechanism.acknowledgeRecovery());
            assertEquals(0.0, stopped.transfer.power(), EPS);
            assertEquals(0.0, stopped.left.power(), EPS);
        }
    }

    @Test public void abortInvalidatesOldTasksWithoutOverwritingLaterIntent() {
        for (Phase phase : new Phase[]{Phase.SETTLING, Phase.RELEASING,
                Phase.TRANSFERRING, Phase.CONFIRMING}) {
            ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
            Task queued = r.mechanism.feedOne();
            Task task = r.start();
            r.reach(task, phase);
            long requestBeforeAbort = r.mechanism.flywheels().requestId();
            r.mechanism.abortFeedAttempts();
            assertEquals("one owner restores idle once", requestBeforeAbort + 1,
                    r.mechanism.flywheels().requestId());
            r.mechanism.flywheels().setVelocityTicksPerSec(600.0);
            task.cancel();
            r.cycle(task);
            queued.start(r.time.clock());
            queued.update(r.time.clock());
            assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
            assertEquals(TaskOutcome.CANCELLED, queued.getOutcome());
            assertEquals(600.0, r.left.commandedVelocityTicksPerSec(), EPS);
            assertEquals(Reason.ABORTED, r.mechanism.status().reason());
            r.assertFeedIdle();
        }
    }

    @Test public void overlappingAttemptEndsBusyWithoutChangingOwnedAttempt() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task active = r.start();
        Task competing = r.mechanism.feedOne();
        ReferenceLauncher.Status before = r.mechanism.status();
        competing.start(r.time.clock());
        assertEquals(TaskOutcome.CANCELLED, competing.getOutcome());
        assertFalse(active.isComplete());
        assertSame(before, r.mechanism.status());
        assertEquals(Reason.BUSY, debug(competing).get("attempt.reason"));
        assertEquals(RecoveryResult.ATTEMPT_ACTIVE, r.mechanism.acknowledgeRecovery());
        r.reach(active, Phase.RELEASING);
    }

    @Test public void queuedOldWorkDoesNotResumeAfterAbort() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task first = r.mechanism.feedOne();
        Task second = r.mechanism.feedOne();
        TaskRunner runner = new TaskRunner();
        runner.enqueue(first);
        runner.enqueue(second);
        runner.update(r.time.clock());
        r.mechanism.update(r.time.clock());
        r.mechanism.abortFeedAttempts();
        r.mechanism.flywheels().setVelocityTicksPerSec(600.0);
        for (int i = 0; i < 3; i++) {
            runner.update(r.time.nextCycle(STEP));
            r.mechanism.update(r.time.clock());
        }
        assertEquals(TaskOutcome.CANCELLED, first.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, second.getOutcome());
        assertEquals(600.0, r.left.commandedVelocityTicksPerSec(), EPS);
    }

    @Test public void recoveryRequiresFreshIdleAndInventoryButAcknowledgementNeverMoves() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task old = r.start();
        r.reach(old, Phase.RELEASING);
        old.cancel();
        Task rejected = r.mechanism.feedOne();
        assertEquals(RecoveryResult.WAITING_FOR_IDLE, r.mechanism.acknowledgeRecovery());
        rejected.start(r.time.clock());
        assertEquals(TaskOutcome.CANCELLED, rejected.getOutcome());
        assertEquals(Reason.RECOVERY_REQUIRED, debug(rejected).get("attempt.reason"));
        r.second.setHigh(false);
        r.staged(false);
        r.outputCycle();
        assertEquals(RecoveryResult.INVENTORY_UNAVAILABLE, r.mechanism.acknowledgeRecovery());
        r.second.setHigh(true);
        r.outputCycle(); // Empty, consistent inventory permits acknowledgement, not feeding.
        Task beforeAck = r.mechanism.feedOne();
        int writes = r.left.velocityWrites();
        int servoWrites = r.release.positionWrites();
        int reads = r.first.stateReadCalls();
        assertEquals(RecoveryResult.ACKNOWLEDGED, r.mechanism.acknowledgeRecovery());
        assertEquals(writes, r.left.velocityWrites());
        assertEquals(servoWrites, r.release.positionWrites());
        assertEquals(reads, r.first.stateReadCalls());
        assertFalse(r.mechanism.status().recoveryRequired());
        beforeAck.start(r.time.clock());
        assertEquals(TaskOutcome.CANCELLED, beforeAck.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, old.getOutcome());
        Task fresh = r.mechanism.feedOne();
        fresh.start(r.time.clock());
        r.cycles(fresh, 8);
        assertEquals(Phase.SETTLING, r.mechanism.status().phase());
        r.staged(true);
        r.reach(fresh, Phase.RELEASING);
        r.cycle(fresh);
        r.staged(false);
        r.finish(fresh);
        assertEquals(TaskOutcome.SUCCESS, fresh.getOutcome());
    }

    @Test public void initialOutputPreservesTaskButResetInvalidatesOldEvidence() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        assertFalse(task.isComplete());
        r.reach(task, Phase.RELEASING);
        ReferenceLauncher.Status old = r.mechanism.status();
        r.time.clock().reset(0.0);
        task.update(r.time.clock());
        r.mechanism.update(r.time.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(Reason.CLOCK_RESET, r.mechanism.status().reason());
        assertTrue(r.mechanism.status().recoveryRequired());
        assertTrue(Double.isNaN(old.sampledAt().ageSec(r.time.clock())));
        assertTrue(r.mechanism.status().sampledAt().isFresh(r.time.clock(), 0.0));
    }

    @Test public void freshTaskAfterLaterResetCanStartBeforeOutputButOldConstructionCannot() {
        for (boolean outputBeforeStart : new boolean[]{false, true}) {
            ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
            r.mechanism.update(r.time.clock());
            Task old = r.mechanism.feedOne();
            r.time.clock().reset(0.0);
            Task fresh = r.mechanism.feedOne();
            if (outputBeforeStart) r.mechanism.update(r.time.clock());
            fresh.start(r.time.clock());
            fresh.update(r.time.clock());
            r.mechanism.update(r.time.clock());
            assertFalse(fresh.isComplete());
            old.start(r.time.clock());
            assertEquals(TaskOutcome.CANCELLED, old.getOutcome());
            assertEquals(Reason.INVALIDATED, debug(old).get("attempt.reason"));
            r.reach(fresh, Phase.RELEASING);
        }
    }

    @Test public void postResetConstructionDoesNotBypassExplicitAbort() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        r.mechanism.update(r.time.clock());
        r.time.clock().reset(0.0);
        Task task = r.mechanism.feedOne();
        r.mechanism.abortFeedAttempts();
        task.start(r.time.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(Reason.INVALIDATED, debug(task).get("attempt.reason"));
        r.mechanism.update(r.time.clock());
        r.assertIdle();
    }

    @Test public void freshPublicationDoesNotBridgeAnUnconsumedGapDuringFeed() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        r.reach(task, Phase.RELEASING);
        r.cycle(task);
        for (int i = 0; i < 9; i++) r.outputCycle();
        assertTrue(r.mechanism.status().sampledAt().isFresh(r.time.clock(), 0.0));
        task.update(r.time.nextCycle(STEP));
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(Reason.EVIDENCE_LOST, r.mechanism.status().reason());
        assertTrue(r.mechanism.status().recoveryRequired());
    }

    @Test public void reversedEndpointsPreserveNormalizedReleaseIntent() {
        ReferenceLauncherMechanism.Config c = ReferenceLauncherTestRig.config();
        c.releaseRetractedNativePosition = 0.85;
        c.releaseExtendedNativePosition = 0.15;
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig(c);
        Task task = r.start();
        assertEquals(0.85, r.release.position(), EPS);
        r.reach(task, Phase.RELEASING);
        assertEquals(0.15, r.release.position(), EPS);
        task.cancel();
        r.outputCycle();
        assertEquals(0.85, r.release.position(), EPS);
    }

    @Test public void phaseCleanupFailureRetainsExceptionAndStillRequestsIdle() throws Exception {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        r.reach(task, Phase.RELEASING);
        IllegalStateException failure = new IllegalStateException("injected phase cancellation");
        CancelFailure child = new CancelFailure(failure);
        Field field = task.getClass().getDeclaredField("phaseTask");
        field.setAccessible(true);
        field.set(task, child); // Private maintainer fault probe, not a new robot seam.
        assertSame(failure, assertThrows(IllegalStateException.class, task::cancel));
        assertTrue(task.isComplete());
        assertSame(failure, assertThrows(IllegalStateException.class, task::getOutcome));
        assertSame(failure, assertThrows(IllegalStateException.class,
                () -> task.update(r.time.clock())));
        task.cancel();
        assertEquals(1, child.cancelCalls);
        r.outputCycle();
        r.assertIdle();
        assertEquals(Reason.FAILED, r.mechanism.status().reason());
    }

    @Test public void terminalPhaseTimeoutIsNotReplacedByElapsedTimerSuccess() throws Exception {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        r.reach(task, Phase.RELEASING);
        Field field = task.getClass().getDeclaredField("phaseTask");
        field.setAccessible(true);
        field.set(task, new Task() {
            @Override public void start(LoopClock clock) { }
            @Override public void update(LoopClock clock) { }
            @Override public void cancel() { }
            @Override public boolean isComplete() { return true; }
            @Override public TaskOutcome getOutcome() { return TaskOutcome.TIMEOUT; }
        }); // A private impossible-for-this-timer probe checks orchestration outcome fidelity.
        r.cycle(task);
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(Reason.FAILED, r.mechanism.status().reason());
        assertTrue(r.mechanism.status().recoveryRequired());
        r.assertIdle();
    }

    @Test public void invalidActiveClockFailsClosedAndRetainsOriginalFailure() {
        ReferenceLauncherTestRig r = new ReferenceLauncherTestRig();
        Task task = r.start();
        NullPointerException failure = assertThrows(NullPointerException.class,
                () -> task.update(null));
        assertTrue(task.isComplete());
        assertSame(failure, assertThrows(NullPointerException.class, task::getOutcome));
        assertSame(failure, assertThrows(NullPointerException.class,
                () -> task.update(r.time.clock())));
        task.cancel();
        r.outputCycle();
        r.assertIdle();
    }

    private static void invalid(Consumer<ReferenceLauncherMechanism.Config> edit) {
        ReferenceLauncherMechanism.Config c = ReferenceLauncherTestRig.config();
        edit.accept(c);
        FtcTestHardware hardware = new FtcTestHardware();
        assertThrows(IllegalArgumentException.class,
                () -> new ReferenceLauncherMechanism(hardware, c));
        assertEquals(0, hardware.lookupCalls());
    }

    private static void terminalDoesNotOwnLaterIntent(ReferenceLauncherTestRig r, Task task,
                                                     TaskOutcome outcome) {
        Reason reason = r.mechanism.status().reason();
        Phase phase = r.mechanism.status().phase();
        r.mechanism.flywheels().setVelocityTicksPerSec(600.0);
        task.cancel();
        r.cycles(task, 2);
        assertEquals(outcome, task.getOutcome());
        assertEquals(reason, r.mechanism.status().reason());
        assertEquals(phase, r.mechanism.status().phase());
        assertEquals(600.0, r.left.commandedVelocityTicksPerSec(), EPS);
        r.assertFeedIdle();
    }

    private static Map<String, Object> debug(Task task) {
        Map<String, Object> rows = new HashMap<>();
        task.debugDump(new DebugSink() {
            @Override public DebugSink addData(String key, Object value) {
                rows.put(key, value); return this;
            }
            @Override public DebugSink addLine(String text) { return this; }
        }, "attempt");
        return rows;
    }

    /** Deliberately failing active child; only the private phase seam is replaced. */
    private static final class CancelFailure implements Task {
        private final RuntimeException failure;
        int cancelCalls;
        CancelFailure(RuntimeException failure) { this.failure = failure; }
        @Override public void start(LoopClock clock) { }
        @Override public void update(LoopClock clock) { }
        @Override public void cancel() { cancelCalls++; throw failure; }
        @Override public boolean isComplete() { return false; }
        @Override public TaskOutcome getOutcome() { return TaskOutcome.NOT_DONE; }
    }
}
