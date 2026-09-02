package edu.ftcsushi.robots.examples.reference.capability.launcher;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.PlantSnapshot;
import edu.ftcsushi.fw.actuation.PlantTargetResolution;
import edu.ftcsushi.fw.actuation.PlantTargetStatus;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.OutputTask;
import edu.ftcsushi.fw.task.OutputTaskRunner;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware.CrServoProbe;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware.MotorProbe;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware.ServoProbe;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused proof for per-wheel readiness, launch cleanup, and generation invalidation. */
public final class ReferenceLauncherMechanismTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void publicSurfaceUsesExplicitTicksPerSecondVocabularyOnly() throws Exception {
        Set<String> methodNames = new HashSet<String>();
        for (Method method : ReferenceLauncher.class.getDeclaredMethods()) {
            methodNames.add(method.getName());
        }
        assertEquals(
                new HashSet<String>(Arrays.asList(
                        "setTargetVelocityTicksPerSec",
                        "abortLaunches",
                        "launchOne",
                        "status")),
                methodNames);

        Set<String> publicStatusMethodNames = new HashSet<String>();
        for (Method method : ReferenceLauncher.Status.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicStatusMethodNames.add(method.getName());
            }
        }
        assertEquals(
                new HashSet<String>(Arrays.asList(
                        "requestedVelocityTicksPerSec",
                        "appliedVelocityTicksPerSec",
                        "leftMeasuredVelocityTicksPerSec",
                        "rightMeasuredVelocityTicksPerSec",
                        "leftAtTarget",
                        "rightAtTarget",
                        "ready",
                        "objectPresent",
                        "transferPulseActive",
                        "flywheelSnapshot")),
                publicStatusMethodNames);
        assertEquals("Status exposes no public primitive fields",
                0, ReferenceLauncher.Status.class.getFields().length);
        for (Field field : ReferenceLauncher.Status.class.getDeclaredFields()) {
            assertTrue(Modifier.isPrivate(field.getModifiers()));
            assertTrue(Modifier.isFinal(field.getModifiers()));
        }
        assertEquals(1, ReferenceLauncher.Status.class.getConstructors().length);
        assertNotNull(ReferenceLauncher.Status.class.getConstructor(
                PlantSnapshot.class,
                double.class,
                double.class,
                double.class,
                boolean.class,
                boolean.class));

        assertConfigField("maximumVelocityTicksPerSec");
        assertConfigField("velocityToleranceTicksPerSec");
        assertConfigField("launchVelocityTicksPerSec");
        assertMissingField(ReferenceLauncherMechanism.Config.class, "maximumVelocity");
        assertMissingField(ReferenceLauncherMechanism.Config.class, "velocityTolerance");
        assertMissingField(ReferenceLauncherMechanism.Config.class, "launchVelocity");
    }

    @Test
    public void statusComposesPlantFactsAndDerivesFinitePerWheelReadiness() {
        Rig rig = new Rig();
        rig.mechanism.setTargetVelocityTicksPerSec(1000.0);
        PlantSnapshot pending = flywheel(rig.mechanism).snapshot();
        ReferenceLauncher.Status pendingStatus = new ReferenceLauncher.Status(
                pending,
                1000.0,
                1000.0,
                rig.config.velocityToleranceTicksPerSec,
                false,
                true);

        assertSame(pending, pendingStatus.flywheelSnapshot());
        assertEquals(1000.0, pendingStatus.requestedVelocityTicksPerSec(), 0.0);
        assertEquals(0.0, pendingStatus.appliedVelocityTicksPerSec(), 0.0);
        assertTrue(pendingStatus.leftAtTarget());
        assertTrue(pendingStatus.rightAtTarget());
        assertFalse("pre-heartbeat command is not accepted as requested/applied",
                pendingStatus.ready());
        assertFalse(pendingStatus.objectPresent());
        assertTrue(pendingStatus.transferPulseActive());

        rig.mechanism.setTargetVelocityTicksPerSec(0.0);
        rig.left.setMeasuredVelocityTicksPerSec(0.0);
        rig.right.setMeasuredVelocityTicksPerSec(0.0);
        rig.mechanism.update(rig.time.clock());
        ReferenceLauncher.Status idle = rig.mechanism.status();
        assertTrue(idle.leftAtTarget());
        assertTrue(idle.rightAtTarget());
        assertFalse(idle.ready());

        rig.mechanism.setTargetVelocityTicksPerSec(1000.0);
        rig.left.setMeasuredVelocityTicksPerSec(1000.0);
        rig.right.setMeasuredVelocityTicksPerSec(1000.0);
        rig.mechanism.update(rig.time.nextCycle(0.02));
        ReferenceLauncher.Status accepted = rig.mechanism.status();
        assertTrue(accepted.ready());

        ReferenceLauncher.Status unavailableWheel = new ReferenceLauncher.Status(
                accepted.flywheelSnapshot(),
                Double.NaN,
                1000.0,
                rig.config.velocityToleranceTicksPerSec,
                false,
                false);
        assertFalse(unavailableWheel.leftAtTarget());
        assertTrue(unavailableWheel.rightAtTarget());
        assertFalse(unavailableWheel.ready());

        ReferenceLauncher.Status zeroTolerance = new ReferenceLauncher.Status(
                accepted.flywheelSnapshot(),
                1000.0,
                1000.01,
                0.0,
                false,
                false);
        assertTrue(zeroTolerance.leftAtTarget());
        assertFalse(zeroTolerance.rightAtTarget());
        assertFalse(zeroTolerance.ready());

        assertThrows(IllegalArgumentException.class, () -> new ReferenceLauncher.Status(
                accepted.flywheelSnapshot(),
                1000.0,
                1000.0,
                Double.NaN,
                false,
                false));
        assertThrows(IllegalArgumentException.class, () -> new ReferenceLauncher.Status(
                accepted.flywheelSnapshot(),
                1000.0,
                1000.0,
                -1.0,
                false,
                false));
        assertThrows(IllegalArgumentException.class, () -> new ReferenceLauncher.Status(
                plant(rig.mechanism, "transfer").snapshot(),
                0.0,
                0.0,
                0.0,
                false,
                false));

        ReferenceLauncher.Status fallback = new ReferenceLauncher.Status(
                new SnapshotOnlyPlant(PlantTargetResolution.fallback(
                        1000.0,
                        "same-valued fallback")).snapshot(),
                1000.0,
                1000.0,
                rig.config.velocityToleranceTicksPerSec,
                false,
                false);
        assertFalse("fallback intent is not active launcher readiness", fallback.ready());

        Rig nonfiniteRig = new Rig();
        Plant nonfiniteFlywheel = flywheel(nonfiniteRig.mechanism);
        nonfiniteFlywheel.commandTarget().set(Double.NaN);
        PlantSnapshot nonfiniteCommand = nonfiniteFlywheel.snapshot();
        assertThrows(IllegalArgumentException.class, () -> new ReferenceLauncher.Status(
                nonfiniteCommand,
                0.0,
                0.0,
                0.0,
                false,
                false));
    }

    @Test
    public void statusIsSideEffectFreeAndPublishesOnlyCompleteHeartbeatEvidence()
            throws Exception {
        Rig rig = new Rig();
        ReferenceLauncher.Status initial = rig.mechanism.status();
        int initialLeftReads = rig.left.velocityReadCalls();
        int initialRightReads = rig.right.velocityReadCalls();
        int initialObjectReads = rig.objectSensor.stateReadCalls();
        OutputTaskRunner transferOverrides = transferOverrides(rig.mechanism);
        transferOverrides.enqueue(Tasks.outputForSeconds("queuedTransfer", 0.2, 1.0));

        assertSame(initial, rig.mechanism.status());
        assertSame(initial, rig.mechanism.status());
        assertEquals(initialLeftReads, rig.left.velocityReadCalls());
        assertEquals(initialRightReads, rig.right.velocityReadCalls());
        assertEquals(initialObjectReads, rig.objectSensor.stateReadCalls());
        assertEquals(1, transferOverrides.queuedCount());
        assertFalse(transferOverrides.hasActiveTask());

        rig.mechanism.setTargetVelocityTicksPerSec(1000.0);
        rig.left.setMeasuredVelocityTicksPerSec(1000.0);
        rig.right.setMeasuredVelocityTicksPerSec(1000.0);
        assertSame("a command alone does not publish a mixed-age Status",
                initial, rig.mechanism.status());
        assertEquals(0.0, initial.requestedVelocityTicksPerSec(), 0.0);

        rig.mechanism.update(rig.time.clock());
        ReferenceLauncher.Status published = rig.mechanism.status();
        assertNotSame(initial, published);
        assertEquals(1000.0, published.requestedVelocityTicksPerSec(), 0.0);
        assertEquals(1000.0, published.appliedVelocityTicksPerSec(), 0.0);
        assertTrue(published.ready());
        assertTrue(published.transferPulseActive());

        int publishedLeftReads = rig.left.velocityReadCalls();
        int publishedRightReads = rig.right.velocityReadCalls();
        int publishedObjectReads = rig.objectSensor.stateReadCalls();
        assertSame(published, rig.mechanism.status());
        assertEquals(publishedLeftReads, rig.left.velocityReadCalls());
        assertEquals(publishedRightReads, rig.right.velocityReadCalls());
        assertEquals(publishedObjectReads, rig.objectSensor.stateReadCalls());

        rig.mechanism.setTargetVelocityTicksPerSec(500.0);
        assertSame(published, rig.mechanism.status());
        assertEquals(1000.0, published.requestedVelocityTicksPerSec(), 0.0);
        rig.left.setMeasuredVelocityTicksPerSec(500.0);
        rig.right.setMeasuredVelocityTicksPerSec(500.0);
        rig.mechanism.update(rig.time.nextCycle(0.02));
        ReferenceLauncher.Status replacement = rig.mechanism.status();
        assertNotSame(published, replacement);
        assertEquals(500.0, replacement.requestedVelocityTicksPerSec(), 0.0);
        assertEquals("older immutable captures do not drift",
                1000.0, published.requestedVelocityTicksPerSec(), 0.0);
    }

    @Test
    public void failedCustomEvidenceReadDoesNotPublishPartialPlantFacts() {
        Rig rig = new Rig();
        rig.mechanism.setTargetVelocityTicksPerSec(1000.0);
        rig.left.setMeasuredVelocityTicksPerSec(1000.0);
        rig.right.setMeasuredVelocityTicksPerSec(1000.0);
        rig.mechanism.update(rig.time.clock());
        ReferenceLauncher.Status prior = rig.mechanism.status();
        assertTrue(prior.ready());

        rig.mechanism.setTargetVelocityTicksPerSec(500.0);
        rig.left.setMeasuredVelocityTicksPerSec(500.0);
        rig.right.setMeasuredVelocityTicksPerSec(500.0);
        RuntimeException readFailure = new RuntimeException("injected object read failure");
        rig.objectSensor.setReadFailure(readFailure);

        RuntimeException observed = assertThrows(
                RuntimeException.class,
                () -> rig.mechanism.update(rig.time.nextCycle(0.02)));

        assertSame(readFailure, observed);
        assertSame("failed publication retains the prior complete Status",
                prior, rig.mechanism.status());
        assertEquals(1000.0, prior.requestedVelocityTicksPerSec(), 0.0);
        assertEquals(1000.0, prior.appliedVelocityTicksPerSec(), 0.0);

        rig.objectSensor.setReadFailure(null);
        rig.mechanism.update(rig.time.nextCycle(0.02));
        ReferenceLauncher.Status recovered = rig.mechanism.status();
        assertNotSame(prior, recovered);
        assertEquals(500.0, recovered.requestedVelocityTicksPerSec(), 0.0);
        assertEquals(500.0, recovered.appliedVelocityTicksPerSec(), 0.0);
        assertTrue(recovered.ready());
    }

    @Test
    public void successfulStopPublishesTerminalStatusAndPreservesOlderCapture() {
        Rig rig = new Rig();
        rig.mechanism.setTargetVelocityTicksPerSec(1000.0);
        rig.left.setMeasuredVelocityTicksPerSec(1000.0);
        rig.right.setMeasuredVelocityTicksPerSec(1000.0);
        rig.mechanism.update(rig.time.clock());
        ReferenceLauncher.Status active = rig.mechanism.status();
        assertTrue(active.ready());
        int leftReads = rig.left.velocityReadCalls();
        int rightReads = rig.right.velocityReadCalls();
        int objectReads = rig.objectSensor.stateReadCalls();

        rig.mechanism.stop();
        ReferenceLauncher.Status stopped = rig.mechanism.status();

        assertNotSame(active, stopped);
        assertEquals(PlantTargetStatus.Kind.STOPPED,
                stopped.flywheelSnapshot().targetStatus().kind());
        assertEquals(1000.0, stopped.requestedVelocityTicksPerSec(), 0.0);
        assertEquals(0.0, stopped.appliedVelocityTicksPerSec(), 0.0);
        assertEquals(1000.0, stopped.leftMeasuredVelocityTicksPerSec(), 0.0);
        assertEquals(1000.0, stopped.rightMeasuredVelocityTicksPerSec(), 0.0);
        assertTrue(stopped.leftAtTarget());
        assertTrue(stopped.rightAtTarget());
        assertFalse(stopped.ready());
        assertFalse(stopped.transferPulseActive());
        assertTrue("older captures remain historical", active.ready());
        assertEquals(PlantTargetStatus.Kind.ACCEPTED,
                active.flywheelSnapshot().targetStatus().kind());
        assertEquals(leftReads, rig.left.velocityReadCalls());
        assertEquals(rightReads, rig.right.velocityReadCalls());
        assertEquals(objectReads, rig.objectSensor.stateReadCalls());
    }

    @Test
    public void failedStopDoesNotFabricateTerminalStatus() throws Exception {
        Rig rig = new Rig();
        rig.mechanism.setTargetVelocityTicksPerSec(1000.0);
        rig.left.setMeasuredVelocityTicksPerSec(1000.0);
        rig.right.setMeasuredVelocityTicksPerSec(1000.0);
        rig.mechanism.update(rig.time.clock());
        ReferenceLauncher.Status prior = rig.mechanism.status();
        assertTrue(prior.ready());

        OutputTaskRunner transferOverrides = transferOverrides(rig.mechanism);
        transferOverrides.enqueue(new ThrowingCancelOutputTask());
        transferOverrides.update(rig.time.nextCycle(0.02));

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                rig.mechanism::stop);

        assertTrue(failure.getMessage().contains("injected transfer cleanup failure"));
        assertSame(prior, rig.mechanism.status());
        assertTrue(prior.ready());
        assertEquals("cleanup still attempts the grouped Plant stop",
                PlantTargetStatus.Kind.STOPPED,
                flywheel(rig.mechanism).snapshot().targetStatus().kind());
    }

    @Test
    public void duplicateNamesAndLaunchAtToleranceFailBeforeHardwareLookup() {
        FtcTestHardware duplicateMap = new FtcTestHardware();
        ReferenceLauncherMechanism.Config duplicate = testConfig();
        duplicate.rightFlywheelName = "  " + duplicate.leftFlywheelName + "  ";

        IllegalArgumentException duplicateFailure = assertThrows(
                IllegalArgumentException.class,
                () -> new ReferenceLauncherMechanism(duplicateMap, duplicate));

        assertTrue(duplicateFailure.getMessage().contains("leftFlywheelName"));
        assertTrue(duplicateFailure.getMessage().contains("rightFlywheelName"));
        assertTrue(duplicateFailure.getMessage().contains("after trimming"));
        assertEquals(0, duplicateMap.lookupCalls());

        FtcTestHardware toleranceMap = new FtcTestHardware();
        ReferenceLauncherMechanism.Config atTolerance = testConfig();
        atTolerance.launchVelocityTicksPerSec =
                atTolerance.velocityToleranceTicksPerSec;

        IllegalArgumentException toleranceFailure = assertThrows(
                IllegalArgumentException.class,
                () -> new ReferenceLauncherMechanism(toleranceMap, atTolerance));

        assertTrue(toleranceFailure.getMessage().contains("launchVelocityTicksPerSec"));
        assertTrue(toleranceFailure.getMessage().contains("velocityToleranceTicksPerSec"));
        assertEquals(0, toleranceMap.lookupCalls());
    }

    @Test
    public void tuningFactoryCreatesFreshExclusivePlantFromCanonicalRecipe() {
        ReferenceLauncherMechanism.Config config = testConfig();
        FtcTestHardware firstHardware = new FtcTestHardware();
        MotorProbe firstLeft = firstHardware.addMotor(config.leftFlywheelName);
        MotorProbe firstRight = firstHardware.addMotor(config.rightFlywheelName);
        edu.ftcsushi.fw.actuation.Plant first =
                ReferenceLauncherMechanism.createFlywheelPlantForTuning(
                        firstHardware,
                        config);

        FtcTestHardware secondHardware = new FtcTestHardware();
        secondHardware.addMotor(config.leftFlywheelName);
        secondHardware.addMotor(config.rightFlywheelName);
        edu.ftcsushi.fw.actuation.Plant second =
                ReferenceLauncherMechanism.createFlywheelPlantForTuning(
                        secondHardware,
                        config);

        assertNotSame(first, second);
        first.commandTarget().set(1000.0);
        first.update(new ManualLoopClock().clock());
        assertEquals(1000.0, firstLeft.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(1000.0, firstRight.commandedVelocityTicksPerSec(), EPSILON);

        first.stop();
        assertEquals(0.0, firstLeft.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, firstRight.commandedVelocityTicksPerSec(), EPSILON);
        second.stop();
    }

    @Test
    public void readinessRequiresPositiveTargetAndBothFiniteIndependentMeasurements() {
        Rig rig = new Rig();
        rig.mechanism.setTargetVelocityTicksPerSec(1000.0);
        rig.left.setMeasuredVelocityTicksPerSec(1075.0);
        rig.right.setMeasuredVelocityTicksPerSec(925.0);

        rig.mechanism.update(rig.time.clock());
        ReferenceLauncher.Status status = rig.mechanism.status();

        assertEquals(1000.0, status.requestedVelocityTicksPerSec(), 0.0);
        assertEquals(1000.0, status.appliedVelocityTicksPerSec(), 0.0);
        assertEquals(1075.0, status.leftMeasuredVelocityTicksPerSec(), 0.0);
        assertEquals(925.0, status.rightMeasuredVelocityTicksPerSec(), 0.0);
        assertFalse("opposite errors whose average is the target must not pass",
                status.leftAtTarget());
        assertFalse(status.rightAtTarget());
        assertFalse(status.ready());

        int leftReads = rig.left.velocityReadCalls();
        int rightReads = rig.right.velocityReadCalls();
        rig.mechanism.update(rig.time.clock());
        assertEquals("per-wheel evidence must be memoized in one cycle",
                leftReads, rig.left.velocityReadCalls());
        assertEquals("per-wheel evidence must be memoized in one cycle",
                rightReads, rig.right.velocityReadCalls());

        rig.left.setMeasuredVelocityTicksPerSec(950.0);
        rig.right.setMeasuredVelocityTicksPerSec(1050.0);
        rig.mechanism.update(rig.time.nextCycle(0.02));
        status = rig.mechanism.status();
        assertTrue(status.leftAtTarget());
        assertTrue(status.rightAtTarget());
        assertTrue(status.ready());

        rig.right.setMeasuredVelocityTicksPerSec(Double.NaN);
        rig.mechanism.update(rig.time.nextCycle(0.02));
        status = rig.mechanism.status();
        assertTrue(status.leftAtTarget());
        assertFalse(status.rightAtTarget());
        assertFalse(status.ready());

        rig.mechanism.setTargetVelocityTicksPerSec(0.0);
        rig.left.setMeasuredVelocityTicksPerSec(0.0);
        rig.right.setMeasuredVelocityTicksPerSec(0.0);
        rig.mechanism.update(rig.time.nextCycle(0.02));
        status = rig.mechanism.status();
        assertTrue(status.leftAtTarget());
        assertTrue(status.rightAtTarget());
        assertFalse(status.ready());
    }

    @Test
    public void readinessUsesLaterPerWheelEvidenceWithoutAggregateArrivalGate()
            throws Exception {
        Rig rig = new Rig();
        rig.mechanism.setTargetVelocityTicksPerSec(1000.0);
        rig.left.setMeasuredVelocityTicksPerSec(0.0);
        rig.right.setMeasuredVelocityTicksPerSec(0.0);
        replaceScalarSource(
                rig.mechanism,
                "leftMeasuredVelocityTicksPerSec",
                ScalarSource.of(() -> 1000.0));
        replaceScalarSource(
                rig.mechanism,
                "rightMeasuredVelocityTicksPerSec",
                ScalarSource.of(() -> 1000.0));

        rig.mechanism.update(rig.time.clock());
        ReferenceLauncher.Status status = rig.mechanism.status();

        assertFalse("the earlier grouped feedback sample is not at the command",
                status.flywheelSnapshot().atTarget());
        assertFalse("aggregate arrival must not become the launcher's later readiness gate",
                status.flywheelSnapshot().atCommandTarget());
        assertTrue(status.leftAtTarget());
        assertTrue(status.rightAtTarget());
        assertTrue("later independent wheel evidence is authoritative for active readiness",
                status.ready());
    }

    @Test
    public void launchTasksAreFreshSingleUseAndPreStartCancellationHasNoEffect() {
        Rig rig = new Rig();
        Task first = rig.mechanism.launchOne();
        Task second = rig.mechanism.launchOne();
        assertNotSame(first, second);

        assertThrows(IllegalStateException.class, () -> second.update(rig.time.clock()));
        assertFalse(second.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, second.getOutcome());
        assertEquals(0.0, requestedTarget(rig), 0.0);

        first.cancel();
        assertFalse(first.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, first.getOutcome());

        first.start(rig.time.clock());
        assertEquals(1000.0, requestedTarget(rig), 0.0);
        assertThrows(IllegalStateException.class, () -> first.start(rig.time.clock()));
        assertEquals(1000.0, requestedTarget(rig), 0.0);

        first.cancel();
        assertTrue(first.isComplete());
        assertEquals(TaskOutcome.CANCELLED, first.getOutcome());
        assertEquals(0.0, requestedTarget(rig), 0.0);
    }

    @Test
    public void successfulLaunchRunsBothTemporaryPhasesThenCleansEveryRequest() {
        Rig rig = new Rig();
        Task launch = rig.mechanism.launchOne();
        primeSpinUp(rig, launch);
        reachReleasePhase(rig, launch);

        assertEquals(rig.config.releaseExtendedPosition, rig.release.position(), EPSILON);
        assertFalse(launch.isComplete());

        launch.update(rig.time.nextCycle(0.11));
        rig.mechanism.update(rig.time.clock());
        assertEquals(rig.config.releaseRetractedPosition, rig.release.position(), EPSILON);
        assertEquals(rig.config.transferPower, rig.transfer.power(), EPSILON);
        assertTrue(rig.mechanism.status().transferPulseActive());

        launch.update(rig.time.nextCycle(0.21));
        assertTrue(launch.isComplete());
        assertEquals(TaskOutcome.SUCCESS, launch.getOutcome());
        assertSafeAfterNextOutput(rig);
    }

    @Test
    public void timeoutRemainsTimeoutAfterNoopFailureBranchAndCleanup() {
        Rig rig = new Rig();
        Task launch = rig.mechanism.launchOne();
        primeSpinUp(rig, launch);

        launch.update(rig.time.nextCycle(rig.config.spinUpTimeoutSec));

        assertTrue(launch.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, launch.getOutcome());
        assertEquals(0.0, rig.transfer.power(), EPSILON);
        assertSafeAfterNextOutput(rig);
        assertEquals(TaskOutcome.TIMEOUT, launch.getOutcome());
    }

    @Test
    public void readinessWinsAtTheExactSpinUpDeadline() {
        Rig rig = new Rig();
        Task launch = rig.mechanism.launchOne();
        primeSpinUp(rig, launch);
        rig.left.setMeasuredVelocityTicksPerSec(rig.config.launchVelocityTicksPerSec);
        rig.right.setMeasuredVelocityTicksPerSec(rig.config.launchVelocityTicksPerSec);

        launch.update(rig.time.nextCycle(rig.config.spinUpTimeoutSec * 0.5));
        rig.mechanism.update(rig.time.clock());
        assertTrue(rig.mechanism.status().ready());

        launch.update(rig.time.nextCycle(rig.config.spinUpTimeoutSec * 0.5));
        rig.mechanism.update(rig.time.clock());

        assertFalse(launch.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, launch.getOutcome());
        launch.cancel();
        assertEquals(TaskOutcome.CANCELLED, launch.getOutcome());
    }

    @Test
    public void cancellationDuringSpinUpReleaseOrTransferCleansSafely() {
        Rig spinUpRig = new Rig();
        Task spinUp = spinUpRig.mechanism.launchOne();
        primeSpinUp(spinUpRig, spinUp);
        spinUp.cancel();
        assertEquals(TaskOutcome.CANCELLED, spinUp.getOutcome());
        assertSafeAfterNextOutput(spinUpRig);

        Rig releaseRig = new Rig();
        Task release = releaseRig.mechanism.launchOne();
        primeSpinUp(releaseRig, release);
        reachReleasePhase(releaseRig, release);
        assertEquals(releaseRig.config.releaseExtendedPosition,
                releaseRig.release.position(), EPSILON);
        release.cancel();
        assertEquals(TaskOutcome.CANCELLED, release.getOutcome());
        assertSafeAfterNextOutput(releaseRig);

        Rig transferRig = new Rig();
        Task transfer = transferRig.mechanism.launchOne();
        primeSpinUp(transferRig, transfer);
        reachReleasePhase(transferRig, transfer);
        transfer.update(transferRig.time.nextCycle(0.11));
        transferRig.mechanism.update(transferRig.time.clock());
        assertTrue(transferRig.mechanism.status().transferPulseActive());
        transfer.cancel();
        assertEquals(TaskOutcome.CANCELLED, transfer.getOutcome());
        assertSafeAfterNextOutput(transferRig);
    }

    @Test
    public void abortInvalidatesActiveAndQueuedTasksWhileLaterCreatedTaskWorks() {
        Rig rig = new Rig();
        Task active = rig.mechanism.launchOne();
        Task queued = rig.mechanism.launchOne();
        TaskRunner runner = new TaskRunner();
        runner.enqueue(active);
        runner.enqueue(queued);

        runner.update(rig.time.clock());
        rig.mechanism.update(rig.time.clock());
        assertEquals(rig.config.launchVelocityTicksPerSec,
                rig.left.commandedVelocityTicksPerSec(), EPSILON);

        rig.mechanism.abortLaunches();
        rig.mechanism.setTargetVelocityTicksPerSec(600.0);
        runner.update(rig.time.nextCycle(0.02));
        rig.mechanism.update(rig.time.clock());
        assertEquals(TaskOutcome.CANCELLED, active.getOutcome());
        assertEquals("a stale active update must not overwrite later manual intent",
                600.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);

        runner.update(rig.time.nextCycle(0.02));
        rig.mechanism.update(rig.time.clock());
        assertEquals(TaskOutcome.CANCELLED, queued.getOutcome());
        assertEquals("a stale queued start must have no request side effects",
                600.0, rig.mechanism.status().requestedVelocityTicksPerSec(), EPSILON);
        assertEquals(600.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);

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
    public void staleUpdateAndCancelCannotOverwriteFreshLaunchReleaseOrFlywheelIntent() {
        Rig rig = new Rig();
        Task staleUpdate = rig.mechanism.launchOne();
        Task staleCancel = rig.mechanism.launchOne();
        staleUpdate.start(rig.time.clock());
        staleUpdate.update(rig.time.clock());
        staleCancel.start(rig.time.clock());
        staleCancel.update(rig.time.clock());
        rig.mechanism.update(rig.time.clock());

        rig.mechanism.abortLaunches();
        Task fresh = rig.mechanism.launchOne();
        primeSpinUp(rig, fresh);
        reachReleasePhase(rig, fresh);
        assertEquals(rig.config.releaseExtendedPosition,
                rig.release.position(), EPSILON);

        staleUpdate.update(rig.time.clock());
        staleCancel.cancel();
        assertEquals(TaskOutcome.CANCELLED, staleUpdate.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, staleCancel.getOutcome());
        assertEquals(rig.config.launchVelocityTicksPerSec, requestedTarget(rig), EPSILON);

        rig.mechanism.update(rig.time.nextCycle(0.02));
        assertEquals(rig.config.launchVelocityTicksPerSec,
                rig.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(rig.config.releaseExtendedPosition,
                rig.release.position(), EPSILON);
        fresh.cancel();
    }

    @Test
    public void activeCleanupFailureLeavesLaunchTerminalAndStillZerosOtherRequests()
            throws Exception {
        Rig rig = new Rig();
        OutputTaskRunner transferOverrides = transferOverrides(rig.mechanism);
        transferOverrides.enqueue(new ThrowingCancelOutputTask());
        transferOverrides.update(rig.time.clock());
        Task launch = rig.mechanism.launchOne();
        launch.start(rig.time.clock());

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                launch::cancel);

        assertTrue(failure.getMessage().contains("injected transfer cleanup failure"));
        assertTrue(launch.isComplete());
        assertEquals(TaskOutcome.CANCELLED, launch.getOutcome());
        assertEquals(0.0, requestedTarget(rig), EPSILON);
        assertSafeAfterNextOutput(rig);
    }

    private static void primeSpinUp(Rig rig, Task launch) {
        launch.start(rig.time.clock());
        launch.update(rig.time.clock());
        rig.mechanism.update(rig.time.clock());
    }

    private static void reachReleasePhase(Rig rig, Task launch) {
        rig.left.setMeasuredVelocityTicksPerSec(rig.config.launchVelocityTicksPerSec);
        rig.right.setMeasuredVelocityTicksPerSec(rig.config.launchVelocityTicksPerSec);
        launch.update(rig.time.nextCycle(0.02));
        rig.mechanism.update(rig.time.clock());
        assertTrue(rig.mechanism.status().ready());

        launch.update(rig.time.nextCycle(0.02));
        rig.mechanism.update(rig.time.clock());
    }

    private static void assertSafeAfterNextOutput(Rig rig) {
        rig.mechanism.update(rig.time.nextCycle(0.02));
        ReferenceLauncher.Status status = rig.mechanism.status();
        assertEquals(0.0, status.requestedVelocityTicksPerSec(), EPSILON);
        assertFalse(status.transferPulseActive());
        assertFalse(status.ready());
        assertEquals(0.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, rig.right.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, rig.transfer.power(), EPSILON);
        assertEquals(rig.config.releaseRetractedPosition, rig.release.position(), EPSILON);
    }

    private static double requestedTarget(Rig rig) {
        return commandTarget(rig.mechanism);
    }

    private static double commandTarget(ReferenceLauncherMechanism mechanism) {
        return flywheel(mechanism).commandTarget().get();
    }

    private static Plant flywheel(ReferenceLauncherMechanism mechanism) {
        return plant(mechanism, "flywheel");
    }

    private static Plant plant(ReferenceLauncherMechanism mechanism, String fieldName) {
        try {
            Field field = ReferenceLauncherMechanism.class.getDeclaredField(fieldName);
            field.setAccessible(true);
            return (Plant) field.get(mechanism);
        } catch (ReflectiveOperationException failure) {
            throw new AssertionError(
                    "Could not inspect private Plant field " + fieldName,
                    failure);
        }
    }

    private static void replaceScalarSource(ReferenceLauncherMechanism mechanism,
                                            String fieldName,
                                            ScalarSource replacement)
            throws ReflectiveOperationException {
        Field field = ReferenceLauncherMechanism.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(mechanism, replacement);
    }

    private static OutputTaskRunner transferOverrides(ReferenceLauncherMechanism mechanism)
            throws Exception {
        Field field = ReferenceLauncherMechanism.class.getDeclaredField("transferOverrides");
        field.setAccessible(true);
        return (OutputTaskRunner) field.get(mechanism);
    }

    private static void assertConfigField(String name) throws Exception {
        assertEquals(double.class,
                ReferenceLauncherMechanism.Config.class.getDeclaredField(name).getType());
    }

    private static void assertMissingField(Class<?> type, String name) {
        try {
            type.getDeclaredField(name);
            fail("Expected field " + name + " to be absent");
        } catch (NoSuchFieldException expected) {
            // Expected.
        }
    }

    private static ReferenceLauncherMechanism.Config testConfig() {
        ReferenceLauncherMechanism.Config config =
                ReferenceLauncherMechanism.Config.defaults();
        config.maximumVelocityTicksPerSec = 2000.0;
        config.velocityToleranceTicksPerSec = 50.0;
        config.launchVelocityTicksPerSec = 1000.0;
        config.spinUpTimeoutSec = 1.0;
        config.releaseDurationSec = 0.10;
        config.transferDurationSec = 0.20;
        return config;
    }

    private static final class Rig {
        private final ReferenceLauncherMechanism.Config config = testConfig();
        private final FtcTestHardware hardware = new FtcTestHardware();
        private final MotorProbe left = hardware.addMotor(config.leftFlywheelName);
        private final MotorProbe right = hardware.addMotor(config.rightFlywheelName);
        private final CrServoProbe transfer = hardware.addCrServo(config.transferName);
        private final ServoProbe release = hardware.addServo(config.releaseServoName);
        private final FtcTestHardware.DigitalProbe objectSensor =
                hardware.addDigitalInput(config.objectSensorName);
        private final ManualLoopClock time = new ManualLoopClock();
        private final ReferenceLauncherMechanism mechanism;

        private Rig() {
            mechanism = new ReferenceLauncherMechanism(hardware, config);
        }
    }

    private static final class ThrowingCancelOutputTask implements OutputTask {
        private boolean started;
        private boolean complete;

        @Override
        public void start(LoopClock clock) {
            started = true;
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException("update before start");
            }
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
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

    /** Supplies one real PlantSnapshot with caller-selected resolution semantics. */
    private static final class SnapshotOnlyPlant implements Plant {
        private final ScalarTarget command = ScalarTarget.create(1000.0);
        private final PlantTargetResolution resolution;

        private SnapshotOnlyPlant(PlantTargetResolution resolution) {
            this.resolution = resolution;
        }

        @Override
        public void update(LoopClock clock) {
        }

        @Override
        public double getRequestedTarget() {
            return 1000.0;
        }

        @Override
        public double getAppliedTarget() {
            return 1000.0;
        }

        @Override
        public PlantTargetResolution getTargetResolution() {
            return resolution;
        }

        @Override
        public PlantTargetStatus getTargetStatus() {
            return PlantTargetStatus.ACCEPTED;
        }

        @Override
        public boolean hasFeedback() {
            return true;
        }

        @Override
        public double getMeasurement() {
            return 1000.0;
        }

        @Override
        public boolean atTarget() {
            return true;
        }

        @Override
        public boolean atTarget(double target) {
            return Double.compare(target, 1000.0) == 0;
        }

        @Override
        public boolean hasCommandTarget() {
            return true;
        }

        @Override
        public ScalarTarget commandTarget() {
            return command;
        }

        @Override
        public void stop() {
        }
    }
}
