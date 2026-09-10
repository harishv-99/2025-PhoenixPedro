package edu.ftcsushi.robots.examples.reference.capability.launcher;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Proxy;
import java.util.Arrays;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/**
 * Maintainer-only failure probes around the real launcher graph and independently authored FTC
 * readings. Private Plant decorators inject lifecycle callbacks/failures without replacing the
 * normal resolver, commands, or realization. No result proves physical interruption is safe.
 */
public final class ReferenceLauncherOutputFailureTest {
    @Test
    public void failedOutputInEveryActivePhaseRemainsTheTaskFailureAndIsNeverReplayed() {
        for (ReferenceLauncher.Phase phase : new ReferenceLauncher.Phase[]{
                ReferenceLauncher.Phase.SETTLING, ReferenceLauncher.Phase.RELEASING,
                ReferenceLauncher.Phase.TRANSFERRING, ReferenceLauncher.Phase.CONFIRMING}) {
            ReferenceLauncherTestRig rig = new ReferenceLauncherTestRig();
            Task task = rig.start();
            rig.reach(task, phase);
            ReferenceLauncher.Status prior = rig.mechanism.status();
            RuntimeException failure = new IllegalStateException("authored sensor failure in " + phase);
            rig.first.setReadFailure(failure);
            rig.time.nextCycle(ReferenceLauncherTestRig.STEP);
            task.update(rig.time.clock());
            assertSame(failure, assertThrows(RuntimeException.class,
                    () -> rig.mechanism.update(rig.time.clock())));

            assertTrue(task.isComplete());
            assertSame(failure, assertThrows(RuntimeException.class, task::getOutcome));
            assertSame(failure, assertThrows(RuntimeException.class,
                    () -> task.update(rig.time.clock())));
            assertEquals(ReferenceLauncher.Reason.FAILED, rig.mechanism.status().reason());
            assertEquals(phase != ReferenceLauncher.Phase.SETTLING,
                    rig.mechanism.status().recoveryRequired());
            assertUnavailableAndPowerStopped(rig);
            assertTrue("old immutable evidence remains historical", prior.sampledAt().isAvailable());
            assertEquals(ReferenceLauncherTestRig.STEP,
                    prior.sampledAt().ageSec(rig.time.clock()), 1e-9);

            int reads = reads(rig);
            int writes = writes(rig);
            rig.first.setReadFailure(null);
            assertSame(failure, assertThrows(RuntimeException.class,
                    () -> rig.mechanism.update(rig.time.clock())));
            assertSame(failure, assertThrows(RuntimeException.class,
                    () -> rig.mechanism.update(rig.time.nextCycle(ReferenceLauncherTestRig.STEP))));
            task.cancel();
            rig.mechanism.stop();
            assertEquals(reads, reads(rig));
            assertEquals(writes, writes(rig));
            assertSame(failure, assertThrows(RuntimeException.class, task::getOutcome));
        }
    }

    @Test
    public void cleanupFailureIsSuppressedWhileEveryRemainingOutputStillStops() throws Exception {
        ReferenceLauncherTestRig rig = new ReferenceLauncherTestRig();
        Task task = rig.start();
        rig.reach(task, ReferenceLauncher.Phase.TRANSFERRING);
        PlantProbe transfer = PlantProbe.install(rig.mechanism, "transfer");
        PlantProbe release = PlantProbe.install(rig.mechanism, "release");
        RuntimeException cleanupFailure = new IllegalStateException("transfer stop callback failed");
        transfer.afterStopFailure = cleanupFailure;
        RuntimeException failure = new IllegalStateException("sensor observation failed");
        rig.second.setReadFailure(failure);
        rig.time.nextCycle(ReferenceLauncherTestRig.STEP);
        task.update(rig.time.clock());

        assertSame(failure, assertThrows(RuntimeException.class,
                () -> rig.mechanism.update(rig.time.clock())));
        assertTrue(Arrays.asList(failure.getSuppressed()).contains(cleanupFailure));
        assertSame(failure, assertThrows(RuntimeException.class, task::getOutcome));
        assertEquals(1, release.stopCalls);
        assertEquals(1, transfer.stopCalls);
        assertUnavailableAndPowerStopped(rig);
        int writes = writes(rig);
        rig.mechanism.stop();
        assertEquals(1, transfer.stopCalls);
        assertEquals(writes, writes(rig));
    }

    @Test
    public void swallowedReentrantOutputFailurePreventsLaterOrdinaryOutputs() throws Exception {
        ReferenceLauncherTestRig rig = new ReferenceLauncherTestRig();
        Task task = rig.start();
        PlantProbe transfer = PlantProbe.install(rig.mechanism, "transfer");
        PlantProbe release = PlantProbe.install(rig.mechanism, "release");
        RuntimeException[] caught = new RuntimeException[1];
        transfer.beforeNextUpdate = () -> caught[0] = assertThrows(IllegalStateException.class,
                () -> rig.mechanism.update(rig.time.clock()));
        int inventoryReads = inventoryReads(rig);
        rig.time.nextCycle(ReferenceLauncherTestRig.STEP);
        RuntimeException failure = assertThrows(RuntimeException.class,
                () -> rig.mechanism.update(rig.time.clock()));

        assertSame(caught[0], failure);
        assertSame(failure, assertThrows(RuntimeException.class, task::getOutcome));
        assertEquals(0, release.updateCalls);
        assertEquals(inventoryReads, inventoryReads(rig));
        assertEquals(1, transfer.stopCalls);
        assertEquals(1, release.stopCalls);
        assertUnavailableAndPowerStopped(rig);
    }

    @Test
    public void stopDuringOutputBlocksLaterWritesAndReadsWithoutFabricatingFailure() throws Exception {
        ReferenceLauncherTestRig rig = new ReferenceLauncherTestRig();
        Task task = rig.start();
        rig.reach(task, ReferenceLauncher.Phase.TRANSFERRING);
        PlantProbe transfer = PlantProbe.install(rig.mechanism, "transfer");
        PlantProbe release = PlantProbe.install(rig.mechanism, "release");
        transfer.beforeNextUpdate = rig.mechanism::stop;
        int inventoryReads = inventoryReads(rig);
        rig.mechanism.update(rig.time.nextCycle(ReferenceLauncherTestRig.STEP));

        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(ReferenceLauncher.Reason.STOPPED, rig.mechanism.status().reason());
        assertEquals(0, release.updateCalls);
        assertEquals(inventoryReads, inventoryReads(rig));
        assertEquals(1, transfer.stopCalls);
        assertEquals(1, release.stopCalls);
        assertUnavailableAndPowerStopped(rig);
        int writes = writes(rig);
        rig.mechanism.update(rig.time.nextCycle(ReferenceLauncherTestRig.STEP));
        assertEquals(writes, writes(rig));
    }

    @Test
    public void stopDuringInventoryReadCannotPublishTheRestOfThatObservation() {
        ReferenceLauncherTestRig rig = new ReferenceLauncherTestRig();
        Task task = rig.start();
        int secondReads = rig.second.stateReadCalls();
        int thirdReads = rig.third.stateReadCalls();
        rig.first.beforeNextRead(rig.mechanism::stop);
        rig.mechanism.update(rig.time.nextCycle(ReferenceLauncherTestRig.STEP));
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(secondReads, rig.second.stateReadCalls());
        assertEquals(thirdReads, rig.third.stateReadCalls());
        assertUnavailableAndPowerStopped(rig);
    }

    @Test
    public void wrongClockIsRejectedBeforeEffectsAndDoesNotConsumeTheRealOwnersNextCycle() {
        ReferenceLauncherTestRig rig = new ReferenceLauncherTestRig();
        Task task = rig.start();
        ReferenceLauncher.Status prior = rig.mechanism.status();
        int reads = reads(rig);
        int writes = writes(rig);
        // Both clocks initially have the same cycle number; identity must be checked before dedup.
        ManualLoopClock other = new ManualLoopClock();
        assertEquals(rig.time.clock().cycle(), other.clock().cycle());
        assertThrows(IllegalArgumentException.class, () -> rig.mechanism.update(other.clock()));
        assertSame(prior, rig.mechanism.status());
        assertEquals(reads, reads(rig));
        assertEquals(writes, writes(rig));
        assertFalse(task.isComplete());
        rig.cycle(task);
        assertNotSame(prior, rig.mechanism.status());
        assertEquals(rig.time.clock().cycle(), rig.mechanism.status().sampleCycle());
        assertFalse(task.isComplete());
        rig.mechanism.stop();
    }

    @Test
    public void clockAdvanceInsideOutputFailsWithoutPublishingAMixedCycle() throws Exception {
        ReferenceLauncherTestRig rig = new ReferenceLauncherTestRig();
        Task task = rig.start();
        PlantProbe transfer = PlantProbe.install(rig.mechanism, "transfer");
        PlantProbe release = PlantProbe.install(rig.mechanism, "release");
        ReferenceLauncher.Status prior = rig.mechanism.status();
        transfer.beforeNextUpdate = () -> rig.time.nextCycle(ReferenceLauncherTestRig.STEP);
        rig.time.nextCycle(ReferenceLauncherTestRig.STEP);
        int inventoryReads = inventoryReads(rig);
        RuntimeException failure = assertThrows(IllegalStateException.class,
                () -> rig.mechanism.update(rig.time.clock()));

        assertSame(failure, assertThrows(RuntimeException.class, task::getOutcome));
        assertEquals(0, release.updateCalls);
        assertEquals(inventoryReads, inventoryReads(rig));
        assertEquals(2 * ReferenceLauncherTestRig.STEP,
                prior.sampledAt().ageSec(rig.time.clock()), 0.0);
        assertUnavailableAndPowerStopped(rig);
        int writes = writes(rig);
        assertSame(failure, assertThrows(RuntimeException.class,
                () -> rig.mechanism.update(rig.time.clock())));
        assertEquals(writes, writes(rig));
    }

    @Test
    public void commandsPolicyChangesAndDuplicateOutputDoNotRefreshSamplingEvidence() {
        ReferenceLauncherTestRig rig = new ReferenceLauncherTestRig();
        Task first = rig.start();
        ReferenceLauncher.Status sampled = rig.mechanism.status();
        int reads = reads(rig);
        int writes = writes(rig);
        rig.mechanism.update(rig.time.clock());
        assertSame(sampled, rig.mechanism.status());
        rig.mechanism.flywheels().setVelocityTicksPerSec(500.0);
        assertSame(sampled, rig.mechanism.status());
        rig.mechanism.abortFeedAttempts();
        assertEquals(TaskOutcome.CANCELLED, first.getOutcome());
        ReferenceLauncher.Status policyOnly = rig.mechanism.status();
        assertNotSame(sampled, policyOnly);
        assertSame(sampled.sampledAt(), policyOnly.sampledAt());
        assertSame(sampled.inventory(), policyOnly.inventory());
        assertSame(sampled.flywheels(), policyOnly.flywheels());
        assertEquals(sampled.sampleCycle(), policyOnly.sampleCycle());
        rig.mechanism.update(rig.time.clock());
        assertSame(policyOnly, rig.mechanism.status());
        assertEquals(reads, reads(rig));
        assertEquals(writes, writes(rig));

        rig.time.nextCycle(0.125);
        Task replacement = rig.mechanism.feedOne();
        replacement.start(rig.time.clock());
        replacement.update(rig.time.clock());
        assertSame(sampled.sampledAt(), rig.mechanism.status().sampledAt());
        assertEquals(0.125, rig.mechanism.status().sampledAt().ageSec(rig.time.clock()), 0.0);
        assertEquals(ReferenceLauncher.Phase.SETTLING, rig.mechanism.status().phase());
        assertEquals(reads, reads(rig));
        assertEquals(writes, writes(rig));
        rig.mechanism.stop();
    }

    /** STOP withdraws software evidence and stops power; no servo-motion/arrival claim is made. */
    private static void assertUnavailableAndPowerStopped(ReferenceLauncherTestRig rig) {
        assertFalse(rig.mechanism.status().sampledAt().isAvailable());
        assertEquals(-1, rig.mechanism.status().sampleCycle());
        assertFalse(rig.mechanism.status().inventory().observed);
        assertFalse(rig.mechanism.status().inventory().sampledAt.isAvailable());
        assertFalse(rig.mechanism.status().flywheels().sampledAt().isAvailable());
        assertFalse(rig.mechanism.status().attemptActive());
        assertEquals(0.0, rig.transfer.power(), 0.0);
        assertEquals(0.0, rig.left.commandedVelocityTicksPerSec(), 0.0);
        assertEquals(0.0, rig.right.commandedVelocityTicksPerSec(), 0.0);
    }

    private static int inventoryReads(ReferenceLauncherTestRig rig) {
        return rig.first.stateReadCalls() + rig.second.stateReadCalls() + rig.third.stateReadCalls();
    }

    private static int reads(ReferenceLauncherTestRig rig) {
        return inventoryReads(rig) + rig.left.velocityReadCalls() + rig.right.velocityReadCalls();
    }

    private static int writes(ReferenceLauncherTestRig rig) {
        return rig.left.velocityWrites() + rig.right.velocityWrites()
                + rig.transfer.powerWrites() + rig.release.positionWrites();
    }

    /**
     * Narrow reflective maintainer instrumentation of a privately owned real Plant. Every normal
     * call, including final stop, still delegates to that exact production Plant. No public owner
     * construction seam or parallel target/Plant graph is introduced for testing.
     */
    private static final class PlantProbe {
        final Plant delegate;
        int updateCalls;
        int stopCalls;
        Runnable beforeNextUpdate;
        RuntimeException afterStopFailure;

        private PlantProbe(Plant delegate) { this.delegate = delegate; }

        static PlantProbe install(ReferenceLauncherMechanism owner, String fieldName) throws Exception {
            Field field = ReferenceLauncherMechanism.class.getDeclaredField(fieldName);
            field.setAccessible(true);
            PlantProbe probe = new PlantProbe((Plant) field.get(owner));
            Plant wrapper = (Plant) Proxy.newProxyInstance(Plant.class.getClassLoader(),
                    new Class<?>[]{Plant.class}, (proxy, method, args) -> {
                        boolean stopping = "stop".equals(method.getName());
                        if ("update".equals(method.getName())) {
                            probe.updateCalls++;
                            if (probe.beforeNextUpdate != null) {
                                Runnable callback = probe.beforeNextUpdate;
                                probe.beforeNextUpdate = null;
                                callback.run();
                            }
                        } else if (stopping) {
                            probe.stopCalls++;
                        }
                        Object result;
                        try {
                            result = method.invoke(probe.delegate, args);
                        } catch (InvocationTargetException failure) {
                            throw failure.getCause();
                        }
                        if (stopping && probe.afterStopFailure != null) {
                            throw probe.afterStopFailure;
                        }
                        return result;
                    });
            field.set(owner, wrapper);
            return probe;
        }
    }
}
