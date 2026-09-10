package edu.ftcsushi.robots.examples.reference.capability.flywheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Proxy;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/**
 * Maintainer evidence for real paired-owner publication, request identity, and lifecycle. Only the
 * outside FTC motor calls are substituted; submitted velocity never manufactures measured speed.
 * These tests establish software sampling, not native frame freshness or safe loaded-wheel behavior.
 */
public final class ReferenceFlywheelEvidenceTest {
    @Test
    public void initialAndStoppedEvidenceIsUnavailableWhileSuccessfulDuplicatesAreIdentical() {
        Fixture fixture = new Fixture();
        assertFalse(fixture.owner.status().sampledAt().isAvailable());
        assertEquals(-1, fixture.owner.status().sampleCycle());
        assertEquals(0, fixture.owner.requestId());
        fixture.readyMeasurements();
        fixture.owner.setVelocityTicksPerSec(1000.0);
        fixture.owner.update(fixture.time.clock());
        ReferenceFlywheels.Status first = fixture.owner.status();
        assertTrue(first.ready());
        assertEquals(fixture.owner.requestId(), first.requestId());
        assertEquals(fixture.time.clock().cycle(), first.sampleCycle());
        assertEquals(0.0, first.sampledAt().ageSec(fixture.time.clock()), 0.0);
        int reads = fixture.reads();
        int writes = fixture.writes();
        fixture.owner.update(fixture.time.clock());
        assertSame(first, fixture.owner.status());
        assertEquals(reads, fixture.reads());
        assertEquals(writes, fixture.writes());

        fixture.owner.update(fixture.time.nextCycle(0.125));
        assertNotSame(first, fixture.owner.status());
        assertEquals(0.125, first.sampledAt().ageSec(fixture.time.clock()), 0.0);
        assertEquals(0.0, fixture.owner.status().sampledAt().ageSec(fixture.time.clock()), 0.0);
        fixture.owner.stop();
        ReferenceFlywheels.Status stopped = fixture.owner.status();
        assertFalse(stopped.sampledAt().isAvailable());
        assertEquals(-1, stopped.sampleCycle());
        assertFalse(stopped.ready());
        assertEquals(1000.0, stopped.leftMeasuredVelocityTicksPerSec(), 0.0);
        assertEquals(0.0, fixture.left.commandedVelocityTicksPerSec(), 0.0);
        writes = fixture.writes();
        reads = fixture.reads();
        fixture.owner.stop();
        fixture.owner.update(fixture.time.nextCycle(0.125));
        assertSame(stopped, fixture.owner.status());
        assertEquals(writes, fixture.writes());
        assertEquals(reads, fixture.reads());
        assertThrows(IllegalStateException.class, () -> fixture.owner.setVelocityTicksPerSec(1000));
    }

    @Test
    public void everyAcceptedRequestHasItsOwnOccurrenceWithoutInventingAPublication() {
        Fixture fixture = new Fixture();
        fixture.owner.setVelocityTicksPerSec(1000.0);
        long firstId = fixture.owner.requestId();
        fixture.owner.update(fixture.time.clock());
        ReferenceFlywheels.Status first = fixture.owner.status();
        fixture.owner.setVelocityTicksPerSec(1000.0);
        assertEquals(firstId + 1, fixture.owner.requestId());
        assertSame(first, fixture.owner.status());
        fixture.owner.update(fixture.time.clock());
        assertSame(first, fixture.owner.status());
        assertNotEquals(fixture.owner.requestId(), first.requestId());
        fixture.owner.setVelocityTicksPerSec(500.0);
        fixture.owner.setVelocityTicksPerSec(1000.0);
        assertEquals(firstId + 3, fixture.owner.requestId());
        assertThrows(IllegalArgumentException.class,
                () -> fixture.owner.setVelocityTicksPerSec(Double.NaN));
        assertEquals(firstId + 3, fixture.owner.requestId());
        fixture.owner.update(fixture.time.nextCycle(0.125));
        assertEquals(fixture.owner.requestId(), fixture.owner.status().requestId());
        assertEquals(firstId, first.requestId());
    }

    @Test
    public void taskUsesTheSameSetterAndCannotReusePreRequestOrSameCycleReadiness() {
        Fixture fixture = new Fixture();
        fixture.readyMeasurements();
        fixture.owner.setVelocityTicksPerSec(1000.0);
        fixture.owner.update(fixture.time.clock());
        long oldId = fixture.owner.requestId();
        Task task = fixture.owner.setVelocityTask(1000.0, 1.0);
        task.cancel();
        assertEquals(oldId, fixture.owner.requestId());
        assertThrows(IllegalStateException.class, () -> task.update(fixture.time.clock()));
        task.start(fixture.time.clock());
        assertEquals(oldId + 1, fixture.owner.requestId());
        task.update(fixture.time.clock());
        assertFalse(task.isComplete());
        fixture.owner.update(fixture.time.clock());
        task.update(fixture.time.clock());
        assertFalse(task.isComplete());

        fixture.time.nextCycle(0.125);
        task.update(fixture.time.clock());
        assertFalse(task.isComplete());
        fixture.owner.update(fixture.time.clock());
        task.update(fixture.time.clock());
        assertFalse("Task updates are also once per cycle", task.isComplete());
        task.update(fixture.time.nextCycle(0.125));
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(oldId + 1, fixture.owner.requestId());
        assertThrows(IllegalStateException.class, () -> task.start(fixture.time.clock()));
    }

    @Test
    public void sameValueAndAwayBackReplacementCannotCompleteOriginalTask() {
        for (boolean awayAndBack : new boolean[]{false, true}) {
            Fixture fixture = new Fixture();
            fixture.readyMeasurements();
            Task task = fixture.owner.setVelocityTask(1000.0, 1.0);
            task.start(fixture.time.clock());
            long taskRequest = fixture.owner.requestId();
            fixture.owner.update(fixture.time.clock());
            if (awayAndBack) {
                fixture.owner.setVelocityTicksPerSec(500.0);
            }
            fixture.owner.setVelocityTicksPerSec(1000.0);
            assertNotEquals(taskRequest, fixture.owner.requestId());
            task.update(fixture.time.nextCycle(0.125));
            assertFalse(task.isComplete());
            fixture.owner.update(fixture.time.clock());
            assertTrue(fixture.owner.status().ready());
            task.update(fixture.time.nextCycle(0.875));
            assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
            assertEquals(1000.0, fixture.owner.status().requestedVelocityTicksPerSec(), 0.0);
        }
    }

    @Test
    public void staleEpochSampleCannotCompleteAndUnchangedMeasuredValuesCanBeSampledAgain() {
        Fixture fixture = new Fixture();
        fixture.readyMeasurements();
        Task task = fixture.owner.setVelocityTask(1000.0, 1.0);
        task.start(fixture.time.clock());
        fixture.owner.update(fixture.time.clock());
        ReferenceFlywheels.Status old = fixture.owner.status();
        fixture.time.clock().reset(0.0);
        task.update(fixture.time.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(Double.isNaN(old.sampledAt().ageSec(fixture.time.clock())));
        Task fresh = fixture.owner.setVelocityTask(1000.0, 1.0);
        fresh.start(fixture.time.clock());
        fixture.owner.update(fixture.time.clock());
        assertNotSame(old, fixture.owner.status());
        assertTrue(fixture.owner.status().ready());
        assertEquals(fixture.time.clock().cycle(), fixture.owner.status().sampleCycle());
        assertEquals(0.0, fixture.owner.status().sampledAt().ageSec(fixture.time.clock()), 0.0);
        assertTrue(Double.isNaN(old.sampledAt().ageSec(fixture.time.clock())));
        fresh.update(fixture.time.clock());
        assertEquals(TaskOutcome.SUCCESS, fresh.getOutcome());
    }

    @Test
    public void failedIndependentReadRetainsPublicationAndExceptionWithoutRetryingEffects() {
        Fixture fixture = new Fixture();
        fixture.owner.update(fixture.time.clock());
        ReferenceFlywheels.Status prior = fixture.owner.status();
        Task task = fixture.owner.setVelocityTask(1000.0, 1.0);
        task.start(fixture.time.nextCycle(0.125));
        RuntimeException failure = new IllegalStateException("independent right read failed");
        // The first right read belongs to the real grouped Plant; the second is the member view.
        fixture.edge.rightReadFailureAt = fixture.right.velocityReadCalls() + 2;
        fixture.edge.readFailure = failure;
        assertSame(failure, assertThrows(RuntimeException.class,
                () -> fixture.owner.update(fixture.time.clock())));
        assertSame(prior, fixture.owner.status());
        assertEquals(0.125, prior.sampledAt().ageSec(fixture.time.clock()), 0.0);
        int reads = fixture.reads();
        int writes = fixture.writes();
        fixture.edge.readFailure = null;
        assertSame(failure, assertThrows(RuntimeException.class,
                () -> fixture.owner.update(fixture.time.clock())));
        assertSame(failure, assertThrows(RuntimeException.class,
                () -> fixture.owner.update(fixture.time.nextCycle(0.125))));
        assertEquals(reads, fixture.reads());
        assertEquals(writes, fixture.writes());
        assertSame(failure, assertThrows(RuntimeException.class, () -> task.update(fixture.time.clock())));
        assertTrue(task.isComplete());
        assertSame(failure, assertThrows(RuntimeException.class, task::getOutcome));
        fixture.owner.stop();
        assertFalse(fixture.owner.status().sampledAt().isAvailable());
        assertEquals(0.0, fixture.left.commandedVelocityTicksPerSec(), 0.0);
    }

    @Test
    public void swallowedReentrantUpdateStillFailsClosedAndCannotPublish() {
        Fixture fixture = new Fixture();
        ReferenceFlywheels.Status initial = fixture.owner.status();
        RuntimeException[] caught = new RuntimeException[1];
        fixture.edge.beforeNextLeftRead = () -> caught[0] = assertThrows(
                IllegalStateException.class, () -> fixture.owner.update(fixture.time.clock()));
        RuntimeException failure = assertThrows(RuntimeException.class,
                () -> fixture.owner.update(fixture.time.clock()));
        assertSame(caught[0], failure);
        assertSame(initial, fixture.owner.status());
        int reads = fixture.reads();
        int writes = fixture.writes();
        assertSame(failure, assertThrows(RuntimeException.class,
                () -> fixture.owner.update(fixture.time.clock())));
        assertEquals(reads, fixture.reads());
        assertEquals(writes, fixture.writes());
        fixture.owner.stop();
    }

    @Test
    public void requestChangedInsideSamplingCannotBorrowThatInFlightPublication() {
        Fixture fixture = new Fixture();
        fixture.readyMeasurements();
        fixture.owner.setVelocityTicksPerSec(1000.0);
        ReferenceFlywheels.Status prior = fixture.owner.status();
        fixture.edge.beforeNextLeftRead = () -> fixture.owner.setVelocityTicksPerSec(1000.0);
        fixture.owner.update(fixture.time.clock());
        assertSame(prior, fixture.owner.status());
        fixture.owner.update(fixture.time.nextCycle(0.125));
        assertEquals(fixture.owner.requestId(), fixture.owner.status().requestId());
        assertTrue(fixture.owner.status().ready());
    }

    @Test
    public void stopInsideSamplingCannotPublishReadyOrReapplyVelocityAfterStop() {
        Fixture fixture = new Fixture();
        fixture.readyMeasurements();
        fixture.owner.setVelocityTicksPerSec(1000.0);
        fixture.edge.beforeNextLeftRead = fixture.owner::stop;
        fixture.owner.update(fixture.time.clock());
        assertFalse(fixture.owner.status().sampledAt().isAvailable());
        assertFalse(fixture.owner.status().ready());
        assertEquals(0.0, fixture.left.commandedVelocityTicksPerSec(), 0.0);
        assertEquals(0.0, fixture.right.commandedVelocityTicksPerSec(), 0.0);
        int reads = fixture.reads();
        fixture.owner.update(fixture.time.nextCycle(0.125));
        assertEquals(reads, fixture.reads());
    }

    /** Real owner and shared clock, with independent numeric observations supplied at the FTC edge. */
    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock();
        final FtcTestHardware hardware = new FtcTestHardware();
        final FtcTestHardware.MotorProbe left = hardware.addMotor("flywheelLeft");
        final FtcTestHardware.MotorProbe right = hardware.addMotor("flywheelRight");
        final MotorEdge edge = new MotorEdge(hardware, right);
        final ReferenceFlywheelMechanism owner = new ReferenceFlywheelMechanism(
                edge, ReferenceFlywheelMechanism.Config.defaults());

        void readyMeasurements() {
            left.setMeasuredVelocityTicksPerSec(1000.0);
            right.setMeasuredVelocityTicksPerSec(1000.0);
        }

        int reads() {
            return left.velocityReadCalls() + right.velocityReadCalls();
        }

        int writes() {
            return left.velocityWrites() + right.velocityWrites();
        }
    }

    /**
     * Outside-SDK callback/failure injection around the existing motor probes. All normal methods
     * delegate to those probes; this is not a completed-Plant or synthetic Status constructor seam.
     */
    private static final class MotorEdge extends HardwareMap {
        final DcMotorEx left;
        final DcMotorEx right;
        final FtcTestHardware.MotorProbe rightProbe;
        Runnable beforeNextLeftRead;
        int rightReadFailureAt = -1;
        RuntimeException readFailure;

        MotorEdge(FtcTestHardware hardware, FtcTestHardware.MotorProbe rightProbe) {
            super(null, null);
            this.rightProbe = rightProbe;
            left = wrap(hardware.get(DcMotorEx.class, "flywheelLeft"), true);
            right = wrap(hardware.get(DcMotorEx.class, "flywheelRight"), false);
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            return type.cast("flywheelLeft".equals(name) ? left : right);
        }

        private DcMotorEx wrap(DcMotorEx delegate, boolean isLeft) {
            return (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                        if ("getVelocity".equals(method.getName()) && method.getParameterCount() == 0) {
                            if (isLeft && beforeNextLeftRead != null) {
                                Runnable callback = beforeNextLeftRead;
                                beforeNextLeftRead = null;
                                callback.run();
                            }
                            if (!isLeft && readFailure != null
                                    && rightProbe.velocityReadCalls() + 1 == rightReadFailureAt) {
                                throw readFailure;
                            }
                        }
                        try {
                            return method.invoke(delegate, args);
                        } catch (InvocationTargetException failure) {
                            throw failure.getCause();
                        }
                    });
        }
    }
}
