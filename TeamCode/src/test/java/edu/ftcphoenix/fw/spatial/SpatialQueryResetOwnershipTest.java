package edu.ftcphoenix.fw.spatial;

import java.util.Collections;
import java.util.Set;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.source.TimeAwareSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;

/** Verifies that a spatial runtime resets only state that the runtime itself owns. */
public final class SpatialQueryResetOwnershipTest {

    @Test
    public void resetClearsOneRuntimeCacheWithoutResettingSharedSpecDependencies() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingFrameSource frames = new RecordingFrameSource();
        RecordingLane lane = new RecordingLane();
        RecordingSelection selection = new RecordingSelection();
        TranslationTarget2d target = SpatialTargets.point(
                References.relativeToSelectedTagPoint(selection, 0.0, 0.0)
        );
        SpatialQuerySpec spec = SpatialQuerySpec.builder()
                .translateTo(target)
                .controlFrames(SpatialControlFrames.of(frames, frames))
                .solveWith(SpatialSolveSet.builder().add(lane).build())
                .build();
        SpatialQuery first = SpatialQuery.from(spec);
        SpatialQuery second = SpatialQuery.from(spec);

        SpatialQueryResult firstResult = first.get(time.clock());
        assertSame(firstResult, first.get(time.clock()));
        assertEquals(1, lane.solveCount);

        SpatialQueryResult secondResult = second.get(time.clock());
        assertNotSame(firstResult, secondResult);
        assertEquals(2, lane.solveCount);

        first.reset();

        assertEquals(0, frames.resetCount);
        assertEquals(0, lane.resetCount);
        assertEquals(0, selection.resetCount);

        SpatialQueryResult firstAfterReset = first.get(time.clock());
        assertNotSame(firstResult, firstAfterReset);
        assertEquals(3, lane.solveCount);
        assertSame(secondResult, second.get(time.clock()));
        assertEquals(3, lane.solveCount);
    }

    private static final class RecordingFrameSource implements TimeAwareSource<Pose2d> {
        private int resetCount;

        @Override
        public Pose2d getAt(LoopClock clock, LoopTimestamp timestamp) {
            return Pose2d.zero();
        }

        @Override
        public void reset() {
            resetCount++;
        }
    }

    private static final class RecordingLane implements SpatialSolveLane {
        private int solveCount;
        private int resetCount;

        @Override
        public SpatialLaneResult solve(SpatialSolveRequest request) {
            solveCount++;
            return SpatialLaneResult.none();
        }

        @Override
        public void reset() {
            resetCount++;
        }
    }

    private static final class RecordingSelection implements TagSelectionSource {
        private int resetCount;

        @Override
        public TagSelectionResult get(LoopClock clock) {
            return TagSelectionResult.none(Collections.singleton(5));
        }

        @Override
        public Set<Integer> candidateIds() {
            return Collections.singleton(5);
        }

        @Override
        public void reset() {
            resetCount++;
        }
    }
}
