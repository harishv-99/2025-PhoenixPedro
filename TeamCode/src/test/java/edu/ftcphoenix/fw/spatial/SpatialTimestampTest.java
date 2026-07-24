package edu.ftcphoenix.fw.spatial;

import java.util.Collections;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.TimeAwareSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies epoch-safe timing at the spatial-query selection boundary. */
public final class SpatialTimestampTest {

    @Test
    public void selectorsEvaluateMeasurementTimeAgainstCoherentQuerySample() {
        ManualLoopClock time = new ManualLoopClock(10.0);
        LoopTimestamp measurementTimestamp = time.clock().nowTimestamp();
        time.nextCycle(0.5);
        LoopTimestamp boundarySample = time.clock().nowTimestamp();

        FacingSolution solution = new FacingSolution(0.2, 0.8, measurementTimestamp);
        SpatialSolutionGate gate = SpatialSolutionGate.builder()
                .maxAgeSec(0.5)
                .minQuality(0.8)
                .build();
        SpatialQueryResult boundary = result(boundarySample, solution);

        SpatialFacingSelection selected =
                SpatialQuerySelectors.firstValidFacing(boundary, gate);

        assertNotNull(selected);
        assertSame(measurementTimestamp, selected.timestamp());

        time.nextCycle(0.001);
        SpatialQueryResult stale = result(time.clock().nowTimestamp(), solution);
        assertNull(SpatialQuerySelectors.firstValidFacing(stale, gate));
    }

    @Test
    public void facingAndTranslationTimingPathsStayParallel() {
        ManualLoopClock time = new ManualLoopClock(6.0);
        LoopTimestamp timestamp = time.clock().nowTimestamp();
        FacingSolution facing = new FacingSolution(0.1, 0.9, timestamp);
        TranslationSolution translation = new TranslationSolution(
                new Pose2d(8.0, 2.0, 0.0),
                new Pose2d(8.0, 2.0, 0.0),
                true,
                Math.hypot(8.0, 2.0),
                0.9,
                timestamp
        );
        SpatialSolutionGate gate = SpatialSolutionGate.builder()
                .maxAgeSec(0.0)
                .minQuality(0.9)
                .build();
        SpatialQueryResult result = result(timestamp, translation, facing);

        assertTrue(gate.accepts(facing, result.sampleTimestamp));
        assertTrue(gate.accepts(translation, result.sampleTimestamp));
        assertSame(timestamp,
                SpatialQuerySelectors.firstValidFacing(result, gate).timestamp());
        assertSame(timestamp,
                SpatialQuerySelectors.firstValidTranslation(result, gate).timestamp());
    }

    @Test
    public void selectorsRejectUnavailableAndPriorEpochTimestampsEvenWithoutAgeLimit() {
        ManualLoopClock time = new ManualLoopClock(4.0);
        LoopTimestamp beforeReset = time.clock().nowTimestamp();
        SpatialSolutionGate unbounded = SpatialSolutionGate.defaults();

        SpatialQueryResult unavailable = result(
                time.clock().nowTimestamp(),
                new FacingSolution(0.0, 1.0, LoopTimestamp.unavailable())
        );
        assertNull(SpatialQuerySelectors.firstValidFacing(unavailable, unbounded));

        time.clock().reset(4.0);
        SpatialQueryResult priorEpoch = result(
                time.clock().nowTimestamp(),
                new FacingSolution(0.0, 1.0, beforeReset)
        );
        assertNull(SpatialQuerySelectors.firstValidFacing(priorEpoch, unbounded));
    }

    @Test
    public void gateRejectsMalformedPolicyButKeepsDocumentedUnboundedDefault() {
        assertTrue(Double.isInfinite(SpatialSolutionGate.defaults().maxAgeSec));

        for (double invalidAge : new double[]{
                -0.01,
                Double.NaN,
                Double.NEGATIVE_INFINITY
        }) {
            expectIllegalArgument(() -> SpatialSolutionGate.builder()
                    .maxAgeSec(invalidAge)
                    .build());
        }
        for (double invalidQuality : new double[]{
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        }) {
            expectIllegalArgument(() -> SpatialSolutionGate.builder()
                    .minQuality(invalidQuality)
                    .build());
        }
    }

    @Test
    public void selectorAllowsOnlyNumericNoiseFromSlightlyFutureMeasurement() {
        ManualLoopClock time = new ManualLoopClock(2.0);
        LoopTimestamp sampleTimestamp = time.clock().nowTimestamp();
        SpatialSolutionGate zeroAge = SpatialSolutionGate.builder().maxAgeSec(0.0).build();

        time.nextCycle(0.5e-6);
        FacingSolution tolerated = new FacingSolution(
                0.0,
                1.0,
                time.clock().nowTimestamp()
        );
        assertNotNull(SpatialQuerySelectors.firstValidFacing(
                result(sampleTimestamp, tolerated),
                zeroAge
        ));

        time.nextCycle(1.5e-6);
        FacingSolution materiallyFuture = new FacingSolution(
                0.0,
                1.0,
                time.clock().nowTimestamp()
        );
        assertNull(SpatialQuerySelectors.firstValidFacing(
                result(sampleTimestamp, materiallyFuture),
                zeroAge
        ));
    }

    @Test
    public void exactClockResetCannotReuseSpatialQueryCycleCache() {
        ManualLoopClock time = new ManualLoopClock(0.0);
        CountingLane lane = new CountingLane();
        SpatialQuery query = SpatialQuery.builder()
                .faceTo(SpatialTargets.fieldHeading(0.0))
                .solveWith(SpatialSolveSet.builder().add(lane).build())
                .build();

        SpatialQueryResult beforeReset = query.get(time.clock());
        time.clock().reset(0.0);
        SpatialQueryResult afterReset = query.get(time.clock());

        assertNotSame(beforeReset, afterReset);
        assertTrue(afterReset.sampleTimestamp.isAvailable());
        assertTrue(Double.isNaN(
                afterReset.sampleTimestamp.secondsSince(beforeReset.sampleTimestamp)
        ));
        assertEquals(2, lane.solveCount);
    }

    @Test
    public void requestFallsBackToSampledFramesForUnavailableOrPriorEpochTime() {
        ManualLoopClock time = new ManualLoopClock(3.0);
        RecordingFrameSource frameHistory = new RecordingFrameSource();
        Pose2d sampledTranslation = new Pose2d(1.0, 0.0, 0.0);
        Pose2d sampledFacing = new Pose2d(2.0, 0.0, 0.0);
        SpatialSolveRequest request = new SpatialSolveRequest(
                time.clock(),
                null,
                null,
                frameHistory,
                frameHistory,
                sampledTranslation,
                sampledFacing,
                null
        );

        LoopTimestamp current = time.clock().nowTimestamp();
        assertEquals(frameHistory.historical, request.robotToTranslationFrameAt(current));
        assertEquals(frameHistory.historical, request.robotToFacingFrameAt(current));
        assertEquals(2, frameHistory.getAtCalls);

        time.clock().reset(3.0);

        assertEquals(sampledTranslation, request.robotToTranslationFrameAt(current));
        assertEquals(sampledFacing, request.robotToFacingFrameAt(current));
        assertEquals(sampledTranslation,
                request.robotToTranslationFrameAt(LoopTimestamp.unavailable()));
        assertEquals(sampledFacing,
                request.robotToFacingFrameAt(LoopTimestamp.unavailable()));
        assertEquals(2, frameHistory.getAtCalls);
    }

    @Test
    public void aprilTagLaneQueriesMountHistoryOnlyForFreshCurrentEpochFrame() {
        ManualLoopClock time = new ManualLoopClock(9.0);
        final AprilTagDetections[] latest = {AprilTagDetections.none()};
        RecordingMountSource mountHistory = new RecordingMountSource();
        AprilTagSpatialSolveLane lane = new AprilTagSpatialSolveLane(
                clock -> latest[0],
                mountHistory,
                0.5
        );
        SpatialSolveRequest request = new SpatialSolveRequest(
                time.clock(),
                null,
                null,
                null,
                null,
                Pose2d.zero(),
                Pose2d.zero(),
                null
        );

        lane.solve(request);
        assertEquals(0, mountHistory.getAtCalls);

        LoopTimestamp priorEpoch = time.clock().nowTimestamp();
        latest[0] = detectionsAt(priorEpoch);
        time.clock().reset(9.0);
        lane.solve(request);
        assertEquals(0, mountHistory.getAtCalls);

        LoopTimestamp current = time.clock().nowTimestamp();
        latest[0] = detectionsAt(current);
        lane.solve(request);
        assertEquals(1, mountHistory.getAtCalls);
        assertSame(current, mountHistory.lastTimestamp);
    }

    private static SpatialQueryResult result(LoopTimestamp sampleTimestamp,
                                             FacingSolution facing) {
        return result(sampleTimestamp, null, facing);
    }

    private static SpatialQueryResult result(LoopTimestamp sampleTimestamp,
                                             TranslationSolution translation,
                                             FacingSolution facing) {
        return new SpatialQueryResult(
                sampleTimestamp,
                null,
                null,
                Pose2d.zero(),
                Pose2d.zero(),
                Collections.singletonList(SpatialLaneResult.of(translation, facing, null, null))
        );
    }

    private static void expectIllegalArgument(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage() != null && !expected.getMessage().isEmpty());
        }
    }

    private static AprilTagDetections detectionsAt(LoopTimestamp timestamp) {
        AprilTagObservation observation = AprilTagObservation.target(
                5,
                Pose3d.zero(),
                timestamp
        );
        return AprilTagDetections.of(timestamp, Collections.singletonList(observation));
    }

    private static final class CountingLane implements SpatialSolveLane {
        private int solveCount;

        @Override
        public SpatialLaneResult solve(SpatialSolveRequest request) {
            solveCount++;
            return SpatialLaneResult.of(
                    null,
                    new FacingSolution(0.0, 1.0, request.clock.nowTimestamp()),
                    null,
                    null
            );
        }
    }

    private static final class RecordingFrameSource implements TimeAwareSource<Pose2d> {
        final Pose2d historical = new Pose2d(4.0, 5.0, 0.25);
        int getAtCalls;

        @Override
        public Pose2d getAt(LoopClock clock, LoopTimestamp timestamp) {
            getAtCalls++;
            return historical;
        }
    }

    private static final class RecordingMountSource
            implements TimeAwareSource<CameraMountConfig> {
        int getAtCalls;
        LoopTimestamp lastTimestamp = LoopTimestamp.unavailable();

        @Override
        public CameraMountConfig getAt(LoopClock clock, LoopTimestamp timestamp) {
            getAtCalls++;
            lastTimestamp = timestamp;
            return CameraMountConfig.identity();
        }
    }
}
