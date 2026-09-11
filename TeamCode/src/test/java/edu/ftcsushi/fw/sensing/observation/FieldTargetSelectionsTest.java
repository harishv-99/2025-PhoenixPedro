package edu.ftcsushi.fw.sensing.observation;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseTrajectoryEstimator;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Real-memory selection checks; synthetic frames and cached poses replace only sensor evidence. */
public final class FieldTargetSelectionsTest {
    private static final double EPS = 1.0e-9;

    @Test public void constructionAndDiagnosticsDoNotPollAndRetainedAgeStagesStayIndependent() {
        Fixture f = new Fixture(1, 4);
        CountingPose robot = new CountingPose();
        FieldTargetSelectionPolicy nearRobot = FieldTargetSelectionPolicies.nearestToRobot(robot, 0.2, 0.1);
        TargetSelections.RecentFieldSelectionStep stages = TargetSelections.fromRecentFieldLocations(f.memory.source());
        TargetSelections.FieldPolicyStep strictStage = stages.freshWithinSec(0.1);
        FieldTargetSelectionSource inherited = stages.choose(nearRobot);
        FieldTargetSelectionSource strict = strictStage.choose(nearRobot);
        stages.freshWithinSec(0.8).choose(nearRobot);
        inherited.debugDump(new QuietDebugSink(), null);
        strict.debugDump(new QuietDebugSink(), "strict");
        inherited.reset();
        assertEquals(0, f.reads);
        assertEquals(0, f.resets);
        assertEquals(0, robot.reads);
        assertEquals(0, robot.updates);

        f.publish(0, 0, 0, 10, 0);
        robot.estimate = pose(f.time, 0, 0);
        assertEquals(1, inherited.get(f.time.clock()).maxAgeSec(), EPS);
        assertEquals(0.1, strict.get(f.time.clock()).maxAgeSec(), EPS);
        assertNotSame(inherited.get(f.time.clock()), strict.get(f.time.clock()));
    }

    @Test public void inheritedAndStricterBoundsAreInclusiveAndDoNotRenewLastSightings() {
        Fixture f = new Fixture(1, 4);
        f.publish(0, 0, 0, 10, 0);
        FieldTargetSelectionSource inherited = f.near(10, 0, 0);
        FieldTargetSelectionSource strict = TargetSelections.fromRecentFieldLocations(f.memory.source())
                .freshWithinSec(0.25).choose(FieldTargetSelectionPolicies.nearFieldPoint(10, 0, 0));
        FieldTargetSelectionResult retained = strict.get(f.time.clock());
        LoopTimestamp capture = retained.entry().lastSighting().timestamp;
        f.time.nextCycle(0.25);
        assertTrue(strict.get(f.time.clock()).hasSelection());
        assertTrue(retained.isUsable(f.time.clock()));
        f.time.nextCycle(0.01);
        assertFalse(strict.get(f.time.clock()).hasSelection());
        assertFalse(retained.isUsable(f.time.clock()));
        assertTrue(inherited.get(f.time.clock()).hasSelection());
        assertSame(capture, inherited.get(f.time.clock()).entry().lastSighting().timestamp);
        f.time.nextCycle(0.74);
        assertTrue(inherited.get(f.time.clock()).isUsable(f.time.clock()));
        f.time.nextCycle(0.001);
        assertFalse(inherited.get(f.time.clock()).hasSelection());
        assertEquals(1, f.reads); // Selection never advances memory to expire entries.
    }

    @Test public void fieldPointRadiusIncludesBoundaryAndCanonicalTiesIgnoreDetectorOrder() {
        Fixture first = new Fixture(1, 4);
        Fixture reversed = new Fixture(1, 4);
        first.publish(100, 20, 0, 3, 4, 3, -4);
        reversed.publish(100, 20, 0, 3, -4, 3, 4);
        FieldTargetSelectionResult a = first.near(100, 20, 5).get(first.time.clock());
        FieldTargetSelectionResult b = reversed.near(100, 20, 5).get(reversed.time.clock());
        assertEquals(103, a.entry().lastSighting().fieldXInches, EPS);
        assertEquals(16, a.entry().lastSighting().fieldYInches, EPS);
        assertEquals(a.entry().lastSighting().fieldYInches, b.entry().lastSighting().fieldYInches, EPS);
        assertEquals(5, a.metricValue(), EPS);
        assertNull(a.rankingPose());
        assertFalse(first.near(100, 20, 4.99).get(first.time.clock()).hasSelection());
        assertTrue(first.near(103, 16, 0).get(first.time.clock()).hasSelection());
        assertFalse(a.entry().lastSighting().hasQuality());
        assertEquals(-1, a.entry().lastSighting().targetId);
    }

    @Test public void currentPoseRanksFieldPointsNotTheDifferentCaptureOrigins() {
        Fixture f = new Fixture(1, 4);
        f.publish(0, 0, 0, 10, 0);
        LoopTimestamp firstCapture = f.memory.snapshot().entries().get(0).lastSighting().timestamp;
        f.time.nextCycle(0.1);
        f.publish(20, 0, Math.PI / 2, 0, -10); // New sighting is field (30, 0).
        CountingPose robot = new CountingPose();
        robot.estimate = pose(f.time, 28, 0);
        FieldTargetSelectionSource selected = TargetSelections.fromRecentFieldLocations(f.memory.source())
                .choose(FieldTargetSelectionPolicies.nearestToRobot(robot, 0.2, 0.1));
        FieldTargetSelectionResult first = selected.get(f.time.clock());
        assertEquals(30, first.entry().lastSighting().fieldXInches, EPS);
        assertEquals(2, first.metricValue(), EPS);
        assertSame(robot.estimate, first.rankingPose());
        assertSame(f.memory.snapshot(), first.snapshot());
        assertSame(first, selected.get(f.time.clock()));
        assertEquals(1, robot.reads);
        assertEquals(0, robot.updates);

        f.time.nextCycle(0.1);
        robot.estimate = pose(f.time, 9, 0);
        FieldTargetSelectionResult next = selected.get(f.time.clock());
        assertEquals(10, next.entry().lastSighting().fieldXInches, EPS);
        assertSame(firstCapture, next.entry().lastSighting().timestamp);
        assertEquals(1, next.metricValue(), EPS);
        assertEquals(2, robot.reads);
        assertEquals(2, f.reads);
    }

    @Test public void historicalRankingPoseDoesNotReplaceDownstreamPoseOrRefreshSighting() {
        Fixture f = new Fixture(1, 4);
        f.publish(0, 0, 0, 10, 0);
        CountingPose robot = new CountingPose();
        robot.estimate = pose(f.time, 0, 0);
        FieldTargetSelectionSource source = TargetSelections.fromRecentFieldLocations(f.memory.source())
                .choose(FieldTargetSelectionPolicies.nearestToRobot(robot, 0.1, 0.1));
        FieldTargetSelectionResult retained = source.get(f.time.clock());
        f.time.nextCycle(0.2);
        assertFalse(source.get(f.time.clock()).hasSelection()); // New selection needs a current ranking pose.
        assertTrue(retained.isUsable(f.time.clock())); // Historical selection is still an eligible field point.
        assertEquals(0.2, retained.entry().lastSighting().ageSec(f.time.clock()), EPS);
        robot.estimate = pose(f.time, 8, 0);
        source.reset();
        FieldTargetSelectionResult newerRanking = source.get(f.time.clock());
        assertSame(retained.entry(), newerRanking.entry());
        assertNotSame(retained.rankingPose(), newerRanking.rankingPose());
    }

    @Test public void unavailableStaleMalformedForeignAndLowQualityPosesDoNotSelect() {
        Fixture f = new Fixture(2, 4);
        f.publish(0, 0, 0, 10, 0);
        CountingPose robot = new CountingPose();
        FieldTargetSelectionSource source = TargetSelections.fromRecentFieldLocations(f.memory.source())
                .choose(FieldTargetSelectionPolicies.nearestToRobot(robot, 0.2, 0.5));
        PoseEstimate[] invalid = {
                null,
                PoseEstimate.noPose(f.time.clock().nowTimestamp()),
                new PoseEstimate(Pose3d.zero(), true, 0.49, f.time.clock().nowTimestamp()),
                new PoseEstimate(Pose3d.zero(), true, Double.NaN, f.time.clock().nowTimestamp()),
                new PoseEstimate(Pose3d.zero(), true, 1.01, f.time.clock().nowTimestamp()),
                new PoseEstimate(new Pose3d(Double.NaN, 0, 0, 0, 0, 0), true, 1, f.time.clock().nowTimestamp()),
                new PoseEstimate(new Pose3d(0, 0, 0, Double.NaN, 0, 0), true, 1, f.time.clock().nowTimestamp()),
                new PoseEstimate(Pose3d.zero(), true, 1, new ManualLoopClock().clock().nowTimestamp()),
                new PoseEstimate(Pose3d.zero(), true, 1, LoopTimestamp.unavailable())
        };
        for (PoseEstimate rejected : invalid) {
            robot.estimate = rejected;
            source.reset();
            FieldTargetSelectionResult result = source.get(f.time.clock());
            assertFalse(result.hasSelection());
            assertSame(rejected, result.rankingPose());
            assertTrue(result.reason().contains("ranking pose"));
            assertTrue(Double.isNaN(result.metricValue()));
        }
        robot.estimate = new PoseEstimate(Pose3d.zero(), true, 0.5, f.time.clock().nowTimestamp());
        source.reset();
        f.time.nextCycle(0.2);
        assertTrue(source.get(f.time.clock()).hasSelection());
        f.time.nextCycle(0.001);
        assertFalse(source.get(f.time.clock()).hasSelection());
        assertEquals(0, robot.updates);
    }

    @Test public void futureRankingPoseIsRejectedWithoutRefreshingMemory() {
        Fixture f = new Fixture(2, 4);
        f.publish(0, 0, 0, 10, 0);
        f.time.clock().update(0.5);
        CountingPose robot = new CountingPose();
        robot.estimate = pose(f.time, 0, 0);
        f.time.clock().update(0.1); // Synthetic clock fault puts the cached pose in the future.
        FieldTargetSelectionResult result = TargetSelections.fromRecentFieldLocations(f.memory.source())
                .choose(FieldTargetSelectionPolicies.nearestToRobot(robot, 1, 0.1)).get(f.time.clock());
        assertFalse(result.hasSelection());
        assertSame(robot.estimate, result.rankingPose());
        assertTrue(f.memory.snapshot().entries().get(0).isUsable(f.time.clock()));
        assertEquals(1, f.reads);
    }

    @Test public void emptyAndUnavailableMemoryDoNotRequirePoseAndNoSelectionAccessIsGuarded() {
        Fixture f = new Fixture(1, 4);
        CountingPose robot = new CountingPose();
        FieldTargetSelectionSource selected = TargetSelections.fromRecentFieldLocations(f.memory.source())
                .choose(FieldTargetSelectionPolicies.nearestToRobot(robot, 0.2, 0.1));
        FieldTargetSelectionResult initial = selected.get(f.time.clock());
        assertFalse(initial.hasSelection());
        assertNull(initial.rankingPose());
        assertSame(f.memory.snapshot(), initial.snapshot());
        illegalState(initial::entry);
        assertEquals(0, robot.reads);
        f.memory.update(f.time.clock());
        selected.reset();
        assertFalse(selected.get(f.time.clock()).hasSelection());
        assertEquals(0, robot.reads);
    }

    @Test public void missingCameraFrameDoesNotEraseUsableMemoryOrCreateAnEmptyFrameClaim() {
        Fixture f = new Fixture(1, 4);
        f.publish(0, 0, 0, 10, 0);
        FieldTargetSelectionSource selected = f.near(10, 0, 0);
        FieldTargetMemory.Entry entry = selected.get(f.time.clock()).entry();
        f.time.nextCycle(0.1);
        f.frame = TargetObservations2d.unavailable("camera obstructed");
        f.memory.update(f.time.clock());
        FieldTargetSelectionResult retained = selected.get(f.time.clock());
        assertTrue(retained.isUsable(f.time.clock()));
        assertSame(entry, retained.entry());
        assertTrue(retained.snapshot().inputReason().contains("camera obstructed"));
    }

    @Test public void exceptionsRetrySameCycleAndResetNeverPropagatesToBorrowedOwners() {
        Fixture f = new Fixture(1, 4);
        f.publish(0, 0, 0, 10, 0);
        CountingPose robot = new CountingPose();
        robot.estimate = pose(f.time, 0, 0);
        robot.failure = new IllegalStateException("temporary value read");
        FieldTargetSelectionSource selected = TargetSelections.fromRecentFieldLocations(f.memory.source())
                .choose(FieldTargetSelectionPolicies.nearestToRobot(robot, 0.2, 0.1));
        try { selected.get(f.time.clock()); fail("expected failed pose read"); }
        catch (IllegalStateException expected) { assertSame(robot.failure, expected); }
        robot.failure = null;
        FieldTargetSelectionResult accepted = selected.get(f.time.clock());
        assertSame(accepted, selected.get(f.time.clock()));
        assertEquals(2, robot.reads);
        selected.reset();
        assertNotSame(accepted, selected.get(f.time.clock()));
        assertEquals(3, robot.reads);
        assertEquals(0, robot.updates);
        assertEquals(0, f.resets);
        assertEquals(1, f.reads);
        assertTrue(accepted.isUsable(f.time.clock()));
    }

    @Test public void recursiveReadAndResetOverlapFailWithoutCommittingAndCanRetry() {
        Fixture f = new Fixture(1, 4);
        f.publish(0, 0, 0, 10, 0);
        CountingPose robot = new CountingPose();
        robot.estimate = pose(f.time, 0, 0);
        FieldTargetSelectionSource selected = TargetSelections.fromRecentFieldLocations(f.memory.source())
                .choose(FieldTargetSelectionPolicies.nearestToRobot(robot, 0.2, 0.1));
        robot.onRead = () -> selected.get(f.time.clock());
        illegalState(() -> selected.get(f.time.clock()));
        robot.onRead = selected::reset;
        illegalState(() -> selected.get(f.time.clock()));
        robot.onRead = null;
        assertTrue(selected.get(f.time.clock()).hasSelection());
        assertEquals(3, robot.reads);
    }

    @Test public void resetStopEvictionAndClockResetInvalidateHistoricalSelections() {
        Fixture reset = new Fixture(1, 4);
        reset.publish(0, 0, 0, 10, 0);
        FieldTargetSelectionResult beforeReset = reset.near(10, 0, 0).get(reset.time.clock());
        reset.memory.reset(reset.time.clock());
        assertTrue(beforeReset.hasSelection());
        assertFalse(beforeReset.isUsable(reset.time.clock()));
        assertEquals(1, beforeReset.snapshot().entries().size());

        Fixture stopped = new Fixture(1, 4);
        stopped.publish(0, 0, 0, 10, 0);
        FieldTargetSelectionResult beforeStop = stopped.near(10, 0, 0).get(stopped.time.clock());
        stopped.memory.stop();
        assertTrue(beforeStop.hasSelection());
        assertFalse(beforeStop.isUsable(stopped.time.clock()));

        Fixture evicted = new Fixture(1, 1);
        evicted.publish(0, 0, 0, 10, 0);
        FieldTargetSelectionResult beforeEviction = evicted.near(10, 0, 0).get(evicted.time.clock());
        evicted.time.nextCycle(0.1);
        evicted.publish(0, 0, 0, 20, 0);
        assertFalse(beforeEviction.isUsable(evicted.time.clock()));

        Fixture epoch = new Fixture(1, 4);
        epoch.publish(0, 0, 0, 10, 0);
        FieldTargetSelectionResult beforeEpoch = epoch.near(10, 0, 0).get(epoch.time.clock());
        epoch.time.clock().reset(0);
        assertFalse(beforeEpoch.isUsable(epoch.time.clock()));
    }

    @Test public void memoryFailureSuspendsCachedAndRetainedEligibilityUntilSuccessfulAdvance() {
        Fixture f = new Fixture(1, 4);
        f.publish(0, 0, 0, 10, 0);
        FieldTargetSelectionSource selected = f.near(10, 0, 0);
        FieldTargetSelectionResult retained = selected.get(f.time.clock());
        f.time.nextCycle(0.1);
        FieldTargetSelectionResult cachedBeforeFailedUpdate = selected.get(f.time.clock());
        f.failure = new IllegalStateException("failed camera read");
        illegalState(() -> f.memory.update(f.time.clock()));
        assertSame(cachedBeforeFailedUpdate, selected.get(f.time.clock()));
        assertFalse(retained.isUsable(f.time.clock()));
        assertFalse(cachedBeforeFailedUpdate.isUsable(f.time.clock()));
        selected.reset();
        illegalState(() -> selected.get(f.time.clock()));
        f.failure = null;
        f.time.nextCycle(0.1);
        f.memory.update(f.time.clock());
        assertTrue(retained.isUsable(f.time.clock()));
        assertTrue(selected.get(f.time.clock()).isUsable(f.time.clock()));
    }

    @Test public void refreshedKeyCannotMoveOrRedateAnAlreadyReturnedChoice() {
        Fixture f = new Fixture(1, 4);
        f.publish(0, 0, 0, 10, 0);
        FieldTargetSelectionSource selected = f.near(10, 0, 10);
        FieldTargetSelectionResult first = selected.get(f.time.clock());
        f.time.nextCycle(0.1);
        f.publish(0, 0, 0, 11, 0);
        FieldTargetSelectionResult refreshed = selected.get(f.time.clock());
        assertSame(first.entry().key(), refreshed.entry().key());
        assertNotSame(first.entry(), refreshed.entry());
        assertEquals(10, first.entry().lastSighting().fieldXInches, EPS);
        assertEquals(11, refreshed.entry().lastSighting().fieldXInches, EPS);
        assertEquals(0.1, first.entry().lastSighting().ageSec(f.time.clock()), EPS);
        assertEquals(0, refreshed.entry().lastSighting().ageSec(f.time.clock()), EPS);
        f.time.nextCycle(0.901);
        assertFalse(first.isUsable(f.time.clock()));
        assertTrue(refreshed.isUsable(f.time.clock()));
    }

    @Test public void zeroRetentionAndNonfiniteDistanceRemainHonestBounds() {
        Fixture zero = new Fixture(0, 4);
        zero.publish(0, 0, 0, 10, 0);
        FieldTargetSelectionSource selected = zero.near(10, 0, 0);
        assertTrue(selected.get(zero.time.clock()).hasSelection());
        zero.time.nextCycle(0.001);
        assertFalse(selected.get(zero.time.clock()).hasSelection());

        Fixture extreme = new Fixture(1, 4);
        extreme.publish(0, 0, 0, Double.MAX_VALUE, 0);
        assertFalse(extreme.near(-Double.MAX_VALUE, 0, Double.MAX_VALUE)
                .get(extreme.time.clock()).hasSelection());
        CountingPose robot = new CountingPose();
        robot.estimate = pose(extreme.time, -Double.MAX_VALUE, 0);
        assertFalse(TargetSelections.fromRecentFieldLocations(extreme.memory.source())
                .choose(FieldTargetSelectionPolicies.nearestToRobot(robot, 0.1, 0.1))
                .get(extreme.time.clock()).hasSelection());
    }

    @Test public void configurationFailsAtConstructionAndNoPublicAssemblyCanInventMembership() {
        Fixture f = new Fixture(1, 4);
        TargetSelections.RecentFieldSelectionStep stages = TargetSelections.fromRecentFieldLocations(f.memory.source());
        for (double invalid : new double[] {-1, Double.NaN, Double.POSITIVE_INFINITY, 1.01}) {
            invalid(() -> stages.freshWithinSec(invalid));
        }
        invalid(() -> FieldTargetSelectionPolicies.nearFieldPoint(Double.NaN, 0, 1));
        invalid(() -> FieldTargetSelectionPolicies.nearFieldPoint(0, Double.POSITIVE_INFINITY, 1));
        invalid(() -> FieldTargetSelectionPolicies.nearFieldPoint(0, 0, -1));
        invalid(() -> FieldTargetSelectionPolicies.nearFieldPoint(0, 0, Double.POSITIVE_INFINITY));
        CountingPose robot = new CountingPose();
        for (double bad : new double[] {-1, Double.NaN, Double.POSITIVE_INFINITY}) {
            invalid(() -> FieldTargetSelectionPolicies.nearestToRobot(robot, bad, 0));
        }
        for (double bad : new double[] {-0.1, 1.1, Double.NaN, Double.POSITIVE_INFINITY}) {
            invalid(() -> FieldTargetSelectionPolicies.nearestToRobot(robot, 0, bad));
        }
        nullArgument(() -> TargetSelections.fromRecentFieldLocations(null));
        nullArgument(() -> stages.choose(null));
        nullArgument(() -> stages.freshWithinSec(0).choose(null));
        nullArgument(() -> FieldTargetSelectionPolicies.nearestToRobot(null, 0, 0));
        assertEquals(0, f.reads);
        assertEquals(0, robot.reads);
        for (Class<?> type : Arrays.asList(FieldTargetSelectionResult.class, FieldTargetSelectionPolicy.class)) {
            for (Constructor<?> constructor : type.getDeclaredConstructors()) {
                assertFalse("no public primitive constructor", Modifier.isPublic(constructor.getModifiers()));
            }
            for (Method method : type.getDeclaredMethods()) {
                assertFalse("no public assembly factory", Modifier.isPublic(method.getModifiers())
                        && Modifier.isStatic(method.getModifiers()) && method.getReturnType() == type);
            }
        }
    }

    /** A complete production history/projection/memory path with authored cached sensor inputs. */
    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock();
        final CachedTrajectory trajectory = new CachedTrajectory();
        final PlanarPoseHistory history = new PlanarPoseHistory(trajectory, PlanarPoseHistory.Config.defaults());
        TargetObservations2d frame = TargetObservations2d.unavailable("no frame yet");
        int reads;
        int resets;
        RuntimeException failure;
        final FieldTargetMemory memory;

        Fixture(double retentionSec, int capacity) {
            Source<TargetObservations2d> raw = new Source<TargetObservations2d>() {
                @Override public TargetObservations2d get(LoopClock clock) {
                    reads++;
                    if (failure != null) throw failure;
                    return frame;
                }
                @Override public void reset() { resets++; }
            };
            memory = FieldTargetMemory.fromFieldObjects(ObservationSources.inField(raw, history.lookupSource()))
                    .retainingForSec(retentionSec).matchingWithinInches(2).maxEntries(capacity);
        }

        void publish(double fieldX, double fieldY, double heading, double... robotXY) {
            LoopTimestamp capture = time.clock().nowTimestamp();
            trajectory.estimate = new PoseEstimate(new Pose3d(fieldX, fieldY, 0, heading, 0, 0), true, 0.8, capture);
            history.recordCurrent(time.clock());
            TargetObservation2d[] points = new TargetObservation2d[robotXY.length / 2];
            for (int i = 0; i < points.length; i++) {
                points[i] = TargetObservation2d.ofRobotRelativePosition(robotXY[2 * i], robotXY[2 * i + 1],
                        Double.NaN, capture);
            }
            frame = TargetObservations2d.fromFrame(capture, Arrays.asList(points));
            memory.update(time.clock());
        }

        FieldTargetSelectionSource near(double x, double y, double radius) {
            return TargetSelections.fromRecentFieldLocations(memory.source())
                    .choose(FieldTargetSelectionPolicies.nearFieldPoint(x, y, radius));
        }
    }

    /** Historical poses are already published; this test never updates a vendor localizer. */
    private static final class CachedTrajectory implements PoseTrajectoryEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        @Override public void update(LoopClock clock) { fail("memory cannot update localization"); }
        @Override public PoseEstimate getEstimate() { return estimate; }
        @Override public long trajectorySegmentId() { return 0; }
    }

    /** Ranking evidence is separate from capture-history evidence and counts every value read. */
    private static final class CountingPose implements AbsolutePoseEstimator {
        PoseEstimate estimate;
        int reads;
        int updates;
        RuntimeException failure;
        Runnable onRead;
        @Override public void update(LoopClock clock) { updates++; }
        @Override public PoseEstimate getEstimate() {
            reads++;
            if (failure != null) throw failure;
            if (onRead != null) onRead.run();
            return estimate;
        }
    }

    /** Cached diagnostic sink with no behavior-changing input. */
    private static final class QuietDebugSink implements DebugSink {
        @Override public DebugSink addData(String key, Object value) { return this; }
        @Override public DebugSink addLine(String text) { return this; }
    }

    private static PoseEstimate pose(ManualLoopClock time, double x, double y) {
        return new PoseEstimate(new Pose3d(x, y, 0, 0, 0, 0), true, 0.8, time.clock().nowTimestamp());
    }

    private static void invalid(Runnable action) {
        try { action.run(); fail("expected invalid configuration"); }
        catch (IllegalArgumentException expected) { }
    }

    private static void illegalState(Runnable action) {
        try { action.run(); fail("expected lifecycle or unavailable-selection failure"); }
        catch (IllegalStateException expected) { }
    }

    private static void nullArgument(Runnable action) {
        try { action.run(); fail("expected missing required input"); }
        catch (NullPointerException expected) { }
    }
}
