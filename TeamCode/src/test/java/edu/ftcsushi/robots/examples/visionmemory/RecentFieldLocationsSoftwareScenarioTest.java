package edu.ftcsushi.robots.examples.visionmemory;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseTrajectoryEstimator;
import edu.ftcsushi.fw.sensing.observation.FieldTargetMemory;
import edu.ftcsushi.fw.sensing.observation.ObservationSources;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.spatial.ReferenceSelectionResult;
import edu.ftcsushi.fw.spatial.SpatialLaneResult;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/**
 * Supplied software scenarios, not physics: retain the maintained service and real field-history,
 * memory, selection, reference, and query graph; replace only authored poses and camera observations.
 */
public final class RecentFieldLocationsSoftwareScenarioTest {

    /** Three authored stationary field points; robot-relative inputs are independently specified. */
    @Test
    public void turningAndAnEmptyImageDoNotMoveOrRefreshUnseenLocations() {
        // ARRANGE: field points A=(20,0), B=(20,10), C=(40,0); robot at the field origin.
        Fixture f = new Fixture();
        f.publish(0, 0, 0, 20, 0, 20, 10, 40, 0);
        assertTrue(f.recent.status().memory.entries().isEmpty()); // BEFORE HEARTBEAT
        f.recent.update(f.clock());
        FieldTargetMemory.Entry unseenC = f.recent.status().memory.entries().get(2);
        LoopTimestamp firstSighting = unseenC.lastSighting().timestamp;
        assertEquals(3, f.recent.status().memory.entries().size());
        assertEquals(40, unseenC.lastSighting().fieldXInches, 1e-9);

        // INJECT EVIDENCE: at 0.60s robot moved to (10,0) and turned left 90 degrees.
        // A is now 0 forward/10 right; B is 10 forward/10 right. C is not observed.
        f.time.nextCycle(0.60);
        f.publish(10, 0, Math.PI / 2, 0, -10, 10, -10);
        f.recent.update(f.clock());
        assertEquals(3, f.recent.status().memory.entries().size());
        assertSame(unseenC, f.recent.status().memory.entries().get(2));
        assertSame(firstSighting, unseenC.lastSighting().timestamp);
        SpatialLaneResult lane = f.recent.status().geometry.laneResult(0);
        assertEquals(ReferenceSelectionResult.Kind.REMEMBERED_TARGET,
                lane.translationSelection.kind());
        assertEquals(20, lane.translationSelection.rememberedTarget()
                .entry().lastSighting().fieldXInches, 1e-9);
        assertEquals(0, lane.translation.robotForwardInches(), 1e-9);
        assertEquals(-10, lane.translation.robotLeftInches(), 1e-9);

        // HEARTBEAT at 1.10s with an actual empty image: C expires, while A/B remain remembered.
        f.time.nextCycle(0.50);
        f.publish(10, 0, Math.PI / 2);
        f.recent.update(f.clock());
        assertEquals(2, f.recent.status().memory.entries().size());
        assertFalse(unseenC.isUsable(f.clock()));
        assertEquals(0.50, f.recent.status().geometry.laneResult(0)
                .translation.targetObservationTimestamp.ageSec(f.clock()), 1e-9);

        // NEXT GATE: at 1.70s all sightings exceed the 1.0s bound; no geometry is solved.
        f.time.nextCycle(0.60);
        f.publish(10, 0, Math.PI / 2);
        f.recent.update(f.clock());
        assertTrue(f.recent.status().memory.entries().isEmpty());
        assertFalse(f.recent.status().geometry.laneResult(0).valid());
        assertEquals(0, f.localizer.updates); // The consumer never owns localization's heartbeat.
    }

    /** Memory remains field evidence when localization is unavailable, but cannot solve guidance. */
    @Test
    public void losingPoseDoesNotInventACameraOnlyAnswer() {
        Fixture f = new Fixture();
        f.publish(0, 0, 0, 20, 0);
        f.recent.update(f.clock());
        f.time.nextCycle(0.10);
        f.localizer.estimate = PoseEstimate.noPose(f.clock().nowTimestamp());
        f.frame = TargetObservations2d.unavailable("camera covered in software fixture");
        f.recent.update(f.clock());
        assertEquals(1, f.recent.status().memory.entries().size());
        assertFalse(f.recent.status().geometry.laneResult(0).valid());
    }

    /** Reset belongs before a possibly failing transition, not after a successful camera change. */
    @Test
    public void explicitTransitionResetFencesCachedImagesAndDoesNotResetBorrowedHistory() {
        Fixture f = new Fixture();
        f.publish(0, 0, 0, 20, 0);
        f.recent.update(f.clock());
        FieldTargetMemory.Entry old = f.recent.status().memory.entries().get(0);
        LoopTimestamp capture = old.lastSighting().timestamp;
        f.recent.resetBeforeTransition(f.clock());
        assertFalse(old.isUsable(f.clock()));
        assertNull(f.recent.status().geometry);
        assertTrue(f.history.lookupSource().getAt(f.clock(), capture).isAvailable());
        f.time.nextCycle(0.10); // The camera still returns its old cached image.
        f.recent.update(f.clock());
        assertTrue(f.recent.status().memory.entries().isEmpty());
        f.time.nextCycle(0.10);
        f.publish(0, 0, 0, 20, 0); // A genuinely newer image can repopulate memory.
        f.recent.update(f.clock());
        assertEquals(1, f.recent.status().memory.entries().size());
        assertNotEquals(old.key(), f.recent.status().memory.entries().get(0).key());
    }

    /** Known trajectory changes invalidate before memory and the spatial query run. */
    @Test
    public void trajectoryDiscontinuityInvalidatesThenWaitsForANewerCapture() {
        Fixture f = new Fixture();
        f.publish(0, 0, 0, 20, 0);
        f.recent.update(f.clock());
        FieldTargetMemory.Entry old = f.recent.status().memory.entries().get(0);
        f.time.nextCycle(0.10);
        f.localizer.segment++;
        f.publish(100, 0, 0, 20, 0);
        f.recent.update(f.clock());
        assertTrue(f.recent.status().memory.entries().isEmpty());
        assertFalse(old.isUsable(f.clock()));
        assertFalse(f.recent.status().geometry.laneResult(0).valid());
        f.time.nextCycle(0.10);
        f.publish(100, 0, 0, 20, 0);
        f.recent.update(f.clock());
        assertEquals(120, f.recent.status().memory.entries().get(0)
                .lastSighting().fieldXInches, 1e-9);
    }

    /** Status is passive and duplicate updates do not poll the borrowed camera twice. */
    @Test
    public void statusAndRepeatedUpdatesArePassiveAndStopIsTerminal() {
        Fixture f = new Fixture();
        assertEquals(0, f.frameReads);
        f.publish(0, 0, 0, 20, 0);
        f.recent.update(f.clock());
        FieldTargetMemory.Entry old = f.recent.status().memory.entries().get(0);
        f.recent.status();
        f.recent.update(f.clock());
        assertEquals(1, f.frameReads);
        f.recent.stop();
        f.recent.stop();
        f.recent.resetBeforeTransition(f.clock());
        f.time.nextCycle(0.10);
        f.recent.update(f.clock());
        assertEquals(1, f.frameReads);
        assertFalse(old.isUsable(f.clock()));
        assertTrue(f.recent.status().memory.stopped());
        assertNull(f.recent.status().geometry);
    }

    /** STOP reached from a borrowed source cannot be undone by the enclosing update. */
    @Test
    public void stopBeforeStartAndStopDuringPollingCannotPublishNewEvidence() {
        Fixture beforeStart = new Fixture();
        beforeStart.recent.stop();
        beforeStart.recent.update(beforeStart.clock());
        assertEquals(0, beforeStart.frameReads);
        Fixture duringRead = new Fixture();
        duringRead.publish(0, 0, 0, 20, 0);
        duringRead.onFrameRead = duringRead.recent::stop;
        duringRead.recent.update(duringRead.clock());
        assertTrue(duringRead.recent.status().memory.stopped());
        assertNull(duringRead.recent.status().geometry);
    }

    /** Outside-world fixture; real history and field projection remain upstream of the service. */
    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock();
        final AuthoredTrajectory localizer = new AuthoredTrajectory();
        final PlanarPoseHistory history = new PlanarPoseHistory(localizer, historyConfig());
        TargetObservations2d frame = TargetObservations2d.unavailable("not yet observed");
        int frameReads;
        Runnable onFrameRead;
        final Source<TargetObservations2d> objects = Source.of(clock -> {
            frameReads++;
            if (onFrameRead != null) onFrameRead.run();
            return frame;
        });
        final RecentFieldLocations recent = new RecentFieldLocations(
                ObservationSources.inField(objects, history.lookupSource()), localizer);

        LoopClock clock() { return time.clock(); }

        /** Injects independent robot-frame coordinates, not output from the production transform. */
        void publish(double fieldX, double fieldY, double yawRad, double... forwardLeftPairs) {
            LoopTimestamp capture = clock().nowTimestamp();
            localizer.estimate = new PoseEstimate(new Pose3d(fieldX, fieldY, 0, yawRad, 0, 0),
                    true, 1.0, capture);
            history.recordCurrent(clock());
            List<TargetObservation2d> observations = new ArrayList<>();
            for (int i = 0; i < forwardLeftPairs.length; i += 2) {
                observations.add(TargetObservation2d.ofRobotRelativePosition(
                        forwardLeftPairs[i], forwardLeftPairs[i + 1], Double.NaN, capture));
            }
            frame = TargetObservations2d.fromFrame(capture, observations);
        }
    }

    /** Exact timestamp fixture; interpolation is disabled rather than silently supplying motion. */
    private static PlanarPoseHistory.Config historyConfig() {
        PlanarPoseHistory.Config config = PlanarPoseHistory.Config.defaults();
        config.retentionSec = 2.0;
        config.maxSamples = 8;
        config.maxInterpolationGapSec = 0.0;
        config.maxInterpolationTranslationInches = 0.0;
        config.maxInterpolationYawRad = 0.0;
        return config;
    }

    /** Scripted localization boundary; the production consumer must never call update. */
    private static final class AuthoredTrajectory implements PoseTrajectoryEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        long segment;
        int updates;
        @Override public long trajectorySegmentId() { return segment; }
        @Override public void update(LoopClock clock) { updates++; }
        @Override public PoseEstimate getEstimate() { return estimate; }
    }
}
