package edu.ftcsushi.fw.drive.guidance;

import org.junit.Test;

import java.util.HashMap;
import java.util.Map;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.spatial.FacingSolution;
import edu.ftcsushi.fw.spatial.SpatialControlFrames;
import edu.ftcsushi.fw.spatial.SpatialLaneResult;
import edu.ftcsushi.fw.spatial.SpatialQuerySpec;
import edu.ftcsushi.fw.spatial.SpatialSolveLane;
import edu.ftcsushi.fw.spatial.SpatialSolveRequest;
import edu.ftcsushi.fw.spatial.SpatialSolveSet;
import edu.ftcsushi.fw.spatial.SpatialTargets;
import edu.ftcsushi.fw.spatial.TranslationSolution;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Test-only custom spatial lane injects partial evidence into the real guidance/Task owners. */
public final class DriveGuidanceLossEvidenceTest {

    @Test public void zeroOutputFallbackDoesNotBypassNoGuidanceDeadline() {
        Fixture rig = new Fixture(DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT);
        DriveGuidanceTask task = rig.task(null);
        task.start(rig.time.clock());
        task.update(rig.time.clock());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertEquals(0, rig.drive.commands);
        assertEquals(2, rig.drive.stops);
        DriveGuidanceStatus status = rig.plan.query().get(rig.time.clock());
        assertEquals(DriveOverlayMask.ALL, status.mask);
        assertFalse(status.hasTranslationError);
        assertFalse(status.hasOmegaError);
        assertEquals(DriveGuidanceSpec.SolveMode.ABSOLUTE_POSE, status.solveMode);
        rig.time.nextCycle(0.11);
        task.update(rig.time.clock());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(3, rig.drive.stops);
        task.update(rig.time.clock());
        task.cancel();
        assertEquals(3, rig.drive.stops);
        assertEquals(0, rig.drive.commands);
    }

    @Test public void partialEvidenceStopsAutoButOverlayKeepsEachSolvedRequestedChannel() {
        for (DriveGuidanceSpec.LossPolicy policy : DriveGuidanceSpec.LossPolicy.values()) {
            for (boolean translationOnly : new boolean[]{true, false}) {
                Fixture rig = new Fixture(policy);
                rig.lane.translation = translationOnly;
                rig.lane.facing = !translationOnly;
                DriveGuidanceStatus status = rig.plan.query().get(rig.time.clock());
                assertEquals(translationOnly, status.hasTranslationError);
                assertEquals(!translationOnly, status.hasOmegaError);
                assertEquals(policy == DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT ? DriveOverlayMask.ALL
                        : translationOnly ? DriveOverlayMask.TRANSLATION_ONLY : DriveOverlayMask.OMEGA_ONLY,
                        status.mask);
                assertEquals(translationOnly ? 0.4 : 0.0, status.signal.axial, 1e-9);
                assertEquals(translationOnly ? 0.2 : 0.0, status.signal.lateral, 1e-9);
                assertEquals(translationOnly ? 0.0 : 0.75, status.signal.omega, 1e-9);
                DriveGuidanceTask task = rig.task(null);
                task.start(rig.time.clock());
                task.update(rig.time.clock());
                assertEquals(0, rig.drive.commands);
                assertEquals(2, rig.drive.stops);
                rig.time.nextCycle(0.11);
                task.update(rig.time.clock());
                assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
            }
        }
    }

    @Test public void exactAxialMaskNeverWidensToLateral() {
        for (DriveGuidanceSpec.LossPolicy policy : DriveGuidanceSpec.LossPolicy.values()) {
            Fixture rig = new Fixture(policy);
            rig.lane.translation = true;
            rig.lane.facing = true;
            DriveOverlayMask axialOnly = new DriveOverlayMask(true, false, false);
            DriveGuidanceStatus status = rig.plan.query().sample(rig.time.clock(), axialOnly);
            assertEquals(axialOnly, status.mask);
            assertEquals(0.4, status.signal.axial, 1e-9);
            assertEquals(0, status.signal.lateral, 0);
            assertEquals(0, status.signal.omega, 0);
            assertTrue(status.hasTranslationError);
            assertFalse(status.hasOmegaError);
            DriveGuidanceTask task = rig.task(axialOnly);
            task.start(rig.time.clock());
            task.update(rig.time.clock());
            assertEquals(1, rig.drive.commands);
            assertEquals(0, rig.drive.last.lateral, 0);
        }
    }

    @Test public void emptyRequestedMaskNeverReportsSuccess() {
        Fixture rig = new Fixture(DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT);
        rig.lane.translation = true;
        rig.lane.facing = true;
        DriveGuidanceTask task = rig.task(DriveOverlayMask.NONE);
        task.start(rig.time.clock());
        task.update(rig.time.clock());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        rig.time.nextCycle(0.11);
        task.update(rig.time.clock());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(0, rig.drive.commands);
    }

    @Test public void recoveringAllChannelsResetsLossIntervalAndLaterLossClearsOldErrors() {
        Fixture rig = new Fixture(DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT);
        DriveGuidanceTask task = rig.task(null);
        task.start(rig.time.clock());
        task.update(rig.time.clock());
        rig.time.nextCycle(0.05);
        rig.lane.translation = true;
        rig.lane.facing = true;
        task.update(rig.time.clock());
        assertEquals(1, rig.drive.commands);
        rig.time.nextCycle(0.05);
        rig.lane.translation = false;
        rig.lane.facing = false;
        task.update(rig.time.clock());
        CapturingDebug debug = new CapturingDebug();
        task.debugDump(debug, "task");
        assertFalse(debug.values.containsKey("task.translationErrorIn"));
        assertFalse(debug.values.containsKey("task.omegaErrorRad"));
        assertEquals(0.0, ((Number) debug.values.get("task.noGuidanceSec")).doubleValue(), 0);
        rig.time.nextCycle(0.09);
        task.update(rig.time.clock());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        rig.time.nextCycle(0.02);
        task.update(rig.time.clock());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(1, rig.drive.commands);
    }

    @Test public void nonFiniteChannelIsLossNotArrivalOrACommand() {
        Fixture rig = new Fixture(DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT);
        rig.lane.translation = true;
        rig.lane.facing = true;
        rig.lane.headingError = Double.NaN;
        DriveGuidanceTask task = rig.task(null);
        task.start(rig.time.clock());
        task.update(rig.time.clock());
        assertEquals(0, rig.drive.commands);
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
    }

    @Test public void cancellationDuringZeroFallbackStopWinsOverLossTimeout() {
        Fixture rig = new Fixture(DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT);
        DriveGuidanceTask task = rig.task(null);
        task.start(rig.time.clock());
        rig.drive.onStop = task::cancel;
        rig.time.nextCycle(0.11);
        task.update(rig.time.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(2, rig.drive.stops);
    }

    @Test public void failedZeroFallbackStopIsNotRetried() {
        Fixture rig = new Fixture(DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT);
        DriveGuidanceTask task = rig.task(null);
        task.start(rig.time.clock());
        RuntimeException failure = new IllegalStateException("stop failed");
        rig.drive.onStop = () -> { throw failure; };
        assertSame(failure, assertThrows(RuntimeException.class, () -> task.update(rig.time.clock())));
        assertSame(failure, assertThrows(RuntimeException.class, () -> task.update(rig.time.clock())));
        assertSame(failure, assertThrows(RuntimeException.class, task::getOutcome));
        assertEquals(2, rig.drive.stops);
    }

    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock();
        final PartialLane lane = new PartialLane();
        final RecordingDrive drive = new RecordingDrive();
        final DriveGuidancePlan plan;

        Fixture(DriveGuidanceSpec.LossPolicy policy) {
            SpatialTargets.FieldPoint target = SpatialTargets.fieldPoint(8, 4);
            SpatialControlFrames frames = SpatialControlFrames.robotCenter();
            SpatialQuerySpec spatial = SpatialQuerySpec.builder().translateTo(target).andFaceTo(target)
                    .solveWith(SpatialSolveSet.builder().add(lane).build()).build();
            AbsolutePoseEstimator unused = new AbsolutePoseEstimator() {
                @Override public void update(LoopClock clock) { fail("Borrowed estimator update"); }
                @Override public PoseEstimate getEstimate() { return PoseEstimate.noPose(LoopTimestamp.unavailable()); }
            };
            DriveGuidanceSpec.ResolveWith resolve = DriveGuidanceSpec.ResolveWith.create(
                    DriveGuidanceSpec.SolveMode.ABSOLUTE_POSE, null,
                    new DriveGuidanceSpec.AbsolutePose(unused, 0.5, 0.1), null, policy);
            plan = new DriveGuidancePlan(new DriveGuidanceSpec(target, target, frames, resolve, spatial),
                    DriveGuidancePlan.Tuning.defaults());
        }

        DriveGuidanceTask task(DriveOverlayMask mask) {
            DriveGuidanceTask.Config config = new DriveGuidanceTask.Config();
            config.timeoutSec = 5;
            config.maxNoGuidanceSec = 0.1;
            config.positionTolInches = 0.01;
            config.headingTolRad = 0.01;
            config.requestedMask = mask;
            return plan.task(drive, config);
        }
    }

    private static final class PartialLane implements SpatialSolveLane {
        boolean translation;
        boolean facing;
        double headingError = 0.3;

        @Override public SpatialLaneResult solve(SpatialSolveRequest request) {
            Pose2d point = new Pose2d(8, 4, 0);
            LoopTimestamp timestamp = request.clock.nowTimestamp();
            return SpatialLaneResult.of(translation
                            ? new TranslationSolution(point, point, false, Double.NaN, 1, timestamp) : null,
                    facing ? new FacingSolution(headingError, 1, timestamp) : null, null, null);
        }
    }

    private static final class RecordingDrive implements DriveCommandSink {
        int commands;
        int stops;
        DriveSignal last;
        Runnable onStop;
        @Override public void drive(DriveSignal signal) { commands++; last = signal; }
        @Override public void stop() { stops++; if (onStop != null) onStop.run(); }
    }

    private static final class CapturingDebug implements DebugSink {
        final Map<String, Object> values = new HashMap<>();
        @Override public DebugSink addData(String key, Object value) { values.put(key, value); return this; }
        @Override public DebugSink addLine(String text) { return this; }
    }
}
