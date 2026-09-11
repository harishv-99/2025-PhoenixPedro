package edu.ftcsushi.fw.sensing.vision.apriltag;

import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.sensing.observation.ObservationSources;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Software geometry, provenance, and selection-lifetime checks; no physical calibration claim. */
public final class TagSelectionEvidenceTest {
    private static final double EPS = 1e-9;

    @Test public void publicConstructionRequiresSourceSpecificEvidenceWithoutPrimitiveResultAssembly() throws Exception {
        assertEquals(0, TagSelectionCandidate.class.getConstructors().length);
        assertEquals(0, TagSelectionResult.class.getConstructors().length);
        assertEquals(1, TagSelectionChoice.class.getConstructors().length);
        assertEquals(TagSelections.VisibleFreshnessStep.class,
                TagSelections.VisibleCandidateStep.class.getMethod("among", Set.class).getReturnType());
        assertEquals(TagSelections.PolicyStep.class,
                TagSelections.VisibleFreshnessStep.class.getMethod("freshWithinSec", double.class).getReturnType());
        assertEquals(TagSelections.FieldPoseQualityStep.class,
                TagSelections.FieldPoseFreshnessStep.class.getMethod("freshWithinSec", double.class).getReturnType());
        assertEquals(TagSelections.PolicyStep.class,
                TagSelections.FieldPoseQualityStep.class.getMethod("minQuality", double.class).getReturnType());
        for (java.lang.reflect.Method method : TagSelections.class.getDeclaredMethods()) {
            assertNotEquals("from", method.getName());
        }
    }

    @Test public void parallelSourcesRankTheSameFullGeometryButOnlyObservationClaimsVisibility() {
        ManualLoopClock time = new ManualLoopClock();
        Pose3d fieldToRobot = new Pose3d(20, -7, 3, 0.5, -0.1, 0.2);
        CameraMountConfig mount = CameraMountConfig.of(4, 2, 8, 0.3, 0.1, -0.2);
        Pose3d cameraToTag = new Pose3d(40, 8, 4, 0.4, -0.2, 0.1);
        Pose3d fieldToTag = fieldToRobot.then(mount.robotToCameraPose()).then(cameraToTag);
        FakePose pose = new FakePose();
        pose.value = new PoseEstimate(fieldToRobot, true, 0.8, time.clock().nowTimestamp());
        TagSelectionResult observed = visible(time, mount, TagSelectionPolicies.closestRange(),
                AprilTagObservation.target(7, cameraToTag)).get(time.clock());
        TagSelectionResult inferred = TagSelections.fromFieldPose(pose,
                new SimpleTagLayout().addPose(7, fieldToTag), mount)
                .among(ids(7)).freshWithinSec(0.2).minQuality(0.5)
                .choose(TagSelectionPolicies.closestRange()).continuous().build().get(time.clock());
        assertEquals(7, inferred.selectedTagId);
        assertPose(cameraToTag, inferred.currentSelectedCandidate.cameraToTagPose);
        assertPose(observed.currentSelectedCandidate.robotToTagPose,
                inferred.currentSelectedCandidate.robotToTagPose);
        assertEquals(TagSelectionCandidate.EvidenceKind.FIELD_POSE, inferred.currentSelectedCandidate.evidenceKind);
        assertNull(inferred.currentSelectedCandidate.observation);
        assertFalse(inferred.hasFreshSelectedObservation);
        assertFalse(inferred.selectedObservation.hasTarget);
        assertFalse(inferred.visibilityTimestamp.isAvailable());
        assertTrue(inferred.visibleCandidateIds.isEmpty());
        assertTrue(observed.hasFreshSelectedObservation);
        assertSame(observed.selectedObservation, observed.currentSelectedCandidate.observation);
        assertEquals(ids(7), observed.visibleCandidateIds);
        assertTrue(observed.visibilityTimestamp.isAvailable());
        assertEquals(0, pose.updates);
    }

    @Test public void freshEmptyFrameIsDifferentFromUnknownVisibility() {
        ManualLoopClock time = new ManualLoopClock();
        TagSelectionResult empty = visible(time, CameraMountConfig.identity(),
                TagSelectionPolicies.closestRange()).get(time.clock());
        assertTrue(empty.visibilityTimestamp.isAvailable());
        assertFalse(empty.hasSelection);
        TagSelectionResult unknown = TagSelections.fromVisibleTags(Source.constant(AprilTagDetections.none()),
                CameraMountConfig.identity()).among(ids(7)).freshWithinSec(0.2)
                .choose(TagSelectionPolicies.closestRange()).continuous().build().get(time.clock());
        assertFalse(unknown.visibilityTimestamp.isAvailable());
    }

    @Test public void previewDecisionAndCurrentEvidenceStaySeparateAcrossMovementAndLoss() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections[] frames = {frame(time, tag(7, 30, 1), tag(8, 30, 10))};
        TagSelectionSource selection = TagSelections.fromVisibleTags(clock -> frames[0], CameraMountConfig.identity())
                .among(ids(7, 8)).freshWithinSec(0.2).choose(TagSelectionPolicies.smallestAbsCameraBearing())
                .stickyWhen(BooleanSource.constant(true)).holdUntilDisabled().build();
        TagSelectionResult first = selection.get(time.clock());
        TagSelectionChoice decision = first.selectionDecision;
        time.nextCycle(0.02);
        frames[0] = frame(time, tag(7, 25, 10), tag(8, 30, 0));
        TagSelectionResult next = selection.get(time.clock());
        assertEquals(8, next.previewTagId);
        assertEquals(7, next.selectedTagId);
        assertSame(decision, next.selectionDecision);
        assertNotSame(decision.candidate, next.currentSelectedCandidate);
        assertSame(frames[0].frameTimestamp(), next.currentSelectedCandidate.evidenceTimestamp);
        time.nextCycle(0.02);
        frames[0] = frame(time, tag(8, 30, 0));
        TagSelectionResult lost = selection.get(time.clock());
        assertEquals(7, lost.selectedTagId);
        assertSame(decision, lost.selectionDecision);
        assertNull(lost.currentSelectedCandidate);
        assertFalse(lost.hasFreshSelectedObservation);
    }

    @Test public void releaseRepressAndExplicitResetPermitNewSelectionWithoutResettingInputs() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections[] frames = {frame(time, tag(7, 30, 0))};
        boolean[] enabled = {true};
        int[] resets = {0};
        Source<AprilTagDetections> source = new Source<AprilTagDetections>() {
            @Override public AprilTagDetections get(LoopClock clock) { return frames[0]; }
            @Override public void reset() { resets[0]++; }
        };
        BooleanSource gate = new BooleanSource() {
            @Override public boolean getAsBoolean(LoopClock clock) { return enabled[0]; }
            @Override public void reset() { resets[0]++; }
        };
        TagSelectionSource selection = TagSelections.fromVisibleTags(source, CameraMountConfig.identity())
                .among(ids(7, 8)).freshWithinSec(0.2).choose(TagSelectionPolicies.closestRange())
                .stickyWhen(gate).holdUntilDisabled().build();
        assertEquals(7, selection.get(time.clock()).selectedTagId);
        time.nextCycle(0.02);
        enabled[0] = false;
        frames[0] = frame(time, tag(8, 30, 0));
        assertFalse(selection.get(time.clock()).hasSelection);
        time.nextCycle(0.02);
        enabled[0] = true;
        assertEquals(8, selection.get(time.clock()).selectedTagId);
        TagSelectionSources.hasSelection(selection).reset();
        TagSelectionSources.hasFreshSelectedObservation(selection).reset();
        TagSelectionSources.selectedTagId(selection, -1).reset();
        TagSelectionSources.selectedObservation(selection).reset();
        ObservationSources.aprilTag(selection).reset();
        assertEquals(8, selection.get(time.clock()).selectedTagId);
        selection.reset();
        frames[0] = frame(time, tag(7, 30, 0));
        assertEquals(7, selection.get(time.clock()).selectedTagId);
        assertEquals(0, resets[0]);
    }

    @Test public void clockEpochClearsHeldIdentityAndRejectsOldGeometryEvenBeforeAReplacementFrame() {
        ManualLoopClock time = new ManualLoopClock();
        TagSelectionSource selection = TagSelections.fromVisibleTags(Source.constant(frame(time, tag(7, 30, 0))),
                CameraMountConfig.identity()).among(ids(7)).freshWithinSec(100)
                .choose(TagSelectionPolicies.closestRange()).stickyUntilReset().holdUntilReset().build();
        assertEquals(7, selection.get(time.clock()).selectedTagId);
        time.clock().reset(0);
        TagSelectionResult reset = selection.get(time.clock());
        assertFalse(reset.hasSelection);
        assertFalse(reset.hasPreview);
        assertFalse(reset.visibilityTimestamp.isAvailable());
    }

    @Test public void lossTimeoutStartsWhenEvidenceIsLostAndReacquiresAtInclusiveBoundary() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections[] frames = {frame(time, tag(7, 30, 0))};
        TagSelectionSource selection = TagSelections.fromVisibleTags(clock -> frames[0], CameraMountConfig.identity())
                .among(ids(7, 8)).freshWithinSec(1).choose(TagSelectionPolicies.closestRange())
                .stickyUntilReset().reacquireAfterLossSec(0.25).build();
        assertEquals(7, selection.get(time.clock()).selectedTagId);
        time.nextCycle(5);
        frames[0] = frame(time, tag(8, 30, 0));
        assertEquals(7, selection.get(time.clock()).selectedTagId);
        time.nextCycle(0.25);
        assertEquals(8, selection.get(time.clock()).selectedTagId);
    }

    @Test public void cameraAndRobotBearingsUseOneMountButAnswerDifferentQuestions() {
        ManualLoopClock time = new ManualLoopClock();
        CameraMountConfig mount = CameraMountConfig.of(0, 0, 0, Math.PI / 2, 0, 0);
        AprilTagObservation[] tags = {tag(7, 30, 0), tag(8, 0, -30)};
        assertEquals(7, visible(time, mount, TagSelectionPolicies.smallestAbsCameraBearing(), tags)
                .get(time.clock()).selectedTagId);
        assertEquals(8, visible(time, mount, TagSelectionPolicies.smallestAbsRobotBearing(), tags)
                .get(time.clock()).selectedTagId);
    }

    @Test public void boundedBearingsAreInclusiveAndUnboundedCanChooseBehind() {
        ManualLoopClock time = new ManualLoopClock();
        assertEquals(7, visible(time, CameraMountConfig.identity(),
                TagSelectionPolicies.smallestAbsCameraBearing(Math.PI / 4), tag(7, 10, 10))
                .get(time.clock()).selectedTagId);
        assertFalse(visible(time, CameraMountConfig.identity(),
                TagSelectionPolicies.smallestAbsCameraBearing(0.1), tag(7, -10, 0))
                .get(time.clock()).hasSelection);
        assertEquals(7, visible(time, CameraMountConfig.identity(),
                TagSelectionPolicies.smallestAbsCameraBearing(), tag(7, -10, 0))
                .get(time.clock()).selectedTagId);
        assertFalse(visible(time, CameraMountConfig.identity(),
                TagSelectionPolicies.smallestAbsRobotBearing(0), tag(7, 10, 1))
                .get(time.clock()).hasSelection);
    }

    @Test public void tiesChooseLowerIdRegardlessOfFrameOrCandidateSetOrder() {
        ManualLoopClock time = new ManualLoopClock();
        for (TagSelectionPolicy policy : Arrays.asList(TagSelectionPolicies.closestRange(),
                TagSelectionPolicies.smallestAbsCameraBearing(), TagSelectionPolicies.smallestAbsRobotBearing())) {
            assertEquals(7, visible(time, CameraMountConfig.identity(), policy,
                    tag(8, 30, -2), tag(7, 30, 2)).get(time.clock()).selectedTagId);
        }
        FakePose pose = new FakePose();
        pose.value = new PoseEstimate(Pose3d.zero(), true, 1, time.clock().nowTimestamp());
        assertEquals(7, TagSelections.fromFieldPose(pose, new SimpleTagLayout()
                .addPose(8, new Pose3d(30, -2, 0, 0, 0, 0))
                .addPose(7, new Pose3d(30, 2, 0, 0, 0, 0)), CameraMountConfig.identity())
                .among(ids(8, 7)).freshWithinSec(0.2).minQuality(0)
                .choose(TagSelectionPolicies.closestRange()).continuous().build().get(time.clock()).selectedTagId);
    }

    @Test public void nonfiniteGeometryNeverReachesCustomPolicyAndDuplicateIdUsesFirstOccurrence() {
        ManualLoopClock time = new ManualLoopClock();
        TagSelectionResult result = visible(time, CameraMountConfig.identity(), candidates -> {
            assertEquals(1, candidates.size());
            assertEquals(8, candidates.get(0).tagId);
            return new TagSelectionChoice(candidates.get(0), "test", "finite only", 0);
        }, AprilTagObservation.target(7, new Pose3d(10, 0, 0, Double.NaN, 0, 0)),
                tag(7, 10, 0), tag(8, 20, 0)).get(time.clock());
        assertEquals(8, result.selectedTagId);
        assertEquals(ids(7, 8), result.visibleCandidateIds);
    }

    @Test public void policyCannotReturnAnotherInvocationCandidateAndCanRetryWithoutStateCommit() {
        ManualLoopClock time = new ManualLoopClock();
        TagSelectionCandidate other = visible(time, CameraMountConfig.identity(),
                TagSelectionPolicies.closestRange(), tag(7, 10, 0)).get(time.clock()).currentSelectedCandidate;
        boolean[] useOther = {true};
        TagSelectionSource selection = visible(time, CameraMountConfig.identity(), candidates ->
                new TagSelectionChoice(useOther[0] ? other : candidates.get(0), "test", "identity", 0), tag(7, 10, 0));
        expect(IllegalArgumentException.class, () -> selection.get(time.clock()));
        useOther[0] = false;
        assertEquals(7, selection.get(time.clock()).selectedTagId);
        expect(IllegalArgumentException.class, () -> new TagSelectionChoice(other, "test", "bad metric", Double.NaN));
    }

    @Test public void unrepresentableCameraRangeCannotBecomeABearingOnlyCandidate() {
        ManualLoopClock time = new ManualLoopClock();
        TagSelectionResult result = visible(time, CameraMountConfig.identity(),
                TagSelectionPolicies.smallestAbsCameraBearing(),
                tag(7, Double.MAX_VALUE, Double.MAX_VALUE)).get(time.clock());
        assertFalse(result.hasSelection);
        assertEquals(ids(7), result.visibleCandidateIds);
    }

    @Test public void poseGateRejectsUnknownOutOfRangeQualityNonfiniteGeometryAndOldEvidence() {
        ManualLoopClock time = new ManualLoopClock();
        FakePose pose = new FakePose();
        TagSelectionSource selection = field(pose, 0.2, 0);
        for (double quality : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -0.1, 1.1}) {
            pose.value = new PoseEstimate(Pose3d.zero(), true, quality, time.clock().nowTimestamp());
            assertFalse(selection.get(time.clock()).hasSelection);
            time.nextCycle(0.01);
        }
        pose.value = new PoseEstimate(new Pose3d(0, 0, Double.NaN, 0, 0, 0), true, 1, time.clock().nowTimestamp());
        assertFalse(selection.get(time.clock()).hasSelection);
        time.nextCycle(0.01);
        pose.value = new PoseEstimate(Pose3d.zero(), true, 0, time.clock().nowTimestamp());
        assertTrue(selection.get(time.clock()).hasSelection);
        time.nextCycle(0.21);
        assertFalse(selection.get(time.clock()).hasSelection);
        assertEquals(0, pose.updates);
    }

    @Test public void mountHistoryIsNotSampledAtConstructionAndOnlyOriginalEvidenceTimeIsRequested() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp capture = time.clock().nowTimestamp();
        AprilTagDetections frame = frame(time, tag(7, 30, 0), tag(8, 30, 2));
        int[] calls = {0};
        TimeAwareSource<CameraMountConfig> mount = (clock, timestamp) -> {
            calls[0]++;
            assertSame(frame.frameTimestamp(), timestamp);
            return CameraMountConfig.of(2, 0, 0, 0, 0, 0);
        };
        TagSelectionSource selection = TagSelections.fromVisibleTags(Source.constant(frame), mount)
                .among(ids(7, 8)).freshWithinSec(0.2).choose(TagSelectionPolicies.closestRange()).continuous().build();
        assertEquals(0, calls[0]);
        time.nextCycle(0.1);
        TagSelectionResult selected = selection.get(time.clock());
        assertEquals(32, selected.currentSelectedCandidate.robotToTagPose.xInches, EPS);
        assertEquals(1, calls[0]);
        assertSame(selected, selection.get(time.clock()));
        assertEquals(1, calls[0]);
        FakePose pose = new FakePose();
        pose.value = new PoseEstimate(Pose3d.zero(), true, 1, capture);
        TagSelectionSource field = TagSelections.fromFieldPose(pose, layout(), (clock, timestamp) -> {
            assertSame(capture, timestamp);
            return CameraMountConfig.identity();
        }).among(ids(7)).freshWithinSec(0.2).minQuality(0)
                .choose(TagSelectionPolicies.closestRange()).continuous().build();
        assertSame(capture, field.get(time.clock()).currentSelectedCandidate.evidenceTimestamp);
    }

    @Test public void nullAndThrowingMountHistoryFailAtomicallyAndCanRetrySameCycle() {
        ManualLoopClock time = new ManualLoopClock();
        int[] calls = {0};
        RuntimeException failure = new IllegalStateException("missing historical mount");
        TagSelectionSource selection = TagSelections.fromVisibleTags(Source.constant(frame(time, tag(7, 30, 0))),
                (clock, timestamp) -> {
                    calls[0]++;
                    if (calls[0] == 1) throw failure;
                    if (calls[0] == 2) return null;
                    return CameraMountConfig.identity();
                }).among(ids(7)).freshWithinSec(0.2).choose(TagSelectionPolicies.closestRange()).continuous().build();
        assertSame(failure, expect(IllegalStateException.class, () -> selection.get(time.clock())));
        expect(NullPointerException.class, () -> selection.get(time.clock()));
        assertTrue(selection.get(time.clock()).hasSelection);
        assertEquals(3, calls[0]);
    }

    @Test public void layoutAndEligibleIdsAreSnapshotsAndMissingMappedIdFailsAtBuild() {
        FakePose pose = new FakePose();
        ManualLoopClock time = new ManualLoopClock();
        pose.value = new PoseEstimate(Pose3d.zero(), true, 1, time.clock().nowTimestamp());
        SimpleTagLayout layout = layout();
        Set<Integer> ids = ids(7);
        TagSelectionSource selection = TagSelections.fromFieldPose(pose, layout, CameraMountConfig.identity())
                .among(ids).freshWithinSec(1).minQuality(0).choose(TagSelectionPolicies.closestRange()).continuous().build();
        ids.add(8);
        layout.addPose(7, new Pose3d(100, 0, 0, 0, 0, 0));
        assertEquals(Collections.singleton(7), selection.candidateIds());
        assertEquals(30, selection.get(time.clock()).currentSelectedCandidate.cameraRangeInches(), EPS);
        expect(IllegalArgumentException.class, () -> TagSelections.fromFieldPose(pose, layout,
                CameraMountConfig.identity()).among(ids(8)).freshWithinSec(1).minQuality(0)
                .choose(TagSelectionPolicies.closestRange()).continuous().build());
    }

    @Test public void configurationRejectsInvalidBoundsAndAuthoredIdentityContainsNoEvidence() {
        FakePose pose = new FakePose();
        for (double value : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -0.1, 1.1}) {
            expect(IllegalArgumentException.class, () -> TagSelections.fromFieldPose(pose, layout(),
                    CameraMountConfig.identity()).among(ids(7)).freshWithinSec(1).minQuality(value));
        }
        for (double value : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -0.1, Math.PI + 0.1}) {
            expect(IllegalArgumentException.class, () -> TagSelectionPolicies.smallestAbsCameraBearing(value));
            expect(IllegalArgumentException.class, () -> TagSelectionPolicies.smallestAbsRobotBearing(value));
        }
        TagSelectionResult authored = TagSelectionResult.forTagId(7);
        assertTrue(authored.hasSelection);
        assertNull(authored.selectionDecision);
        assertNull(authored.currentSelectedCandidate);
        assertFalse(authored.hasPreview);
        assertFalse(authored.hasFreshSelectedObservation);
        assertFalse(authored.visibilityTimestamp.isAvailable());
        expect(IllegalArgumentException.class, () -> TagSelectionResult.forTagId(-1));
    }

    private static TagSelectionSource visible(ManualLoopClock time, CameraMountConfig mount,
                                             TagSelectionPolicy policy, AprilTagObservation... tags) {
        return TagSelections.fromVisibleTags(Source.constant(frame(time, tags)), mount)
                .among(ids(7, 8)).freshWithinSec(0.2).choose(policy).continuous().build();
    }

    private static TagSelectionSource field(FakePose pose, double age, double quality) {
        return TagSelections.fromFieldPose(pose, layout(), CameraMountConfig.identity())
                .among(ids(7)).freshWithinSec(age).minQuality(quality)
                .choose(TagSelectionPolicies.closestRange()).continuous().build();
    }

    private static SimpleTagLayout layout() {
        return new SimpleTagLayout().addPose(7, new Pose3d(30, 0, 0, 0, 0, 0));
    }

    private static AprilTagDetections frame(ManualLoopClock time, AprilTagObservation... observations) {
        return AprilTagDetections.fromFrame(time.clock().nowTimestamp(), Arrays.asList(observations));
    }

    private static AprilTagObservation tag(int id, double x, double y) {
        return AprilTagObservation.target(id, new Pose3d(x, y, 0, 0, 0, 0));
    }

    private static Set<Integer> ids(Integer... ids) { return new LinkedHashSet<>(Arrays.asList(ids)); }

    private static void assertPose(Pose3d expected, Pose3d actual) {
        assertEquals(expected.xInches, actual.xInches, EPS);
        assertEquals(expected.yInches, actual.yInches, EPS);
        assertEquals(expected.zInches, actual.zInches, EPS);
        assertEquals(expected.yawRad, actual.yawRad, EPS);
        assertEquals(expected.pitchRad, actual.pitchRad, EPS);
        assertEquals(expected.rollRad, actual.rollRad, EPS);
    }

    private static RuntimeException expect(Class<? extends RuntimeException> type, Runnable work) {
        try { work.run(); fail("expected " + type.getSimpleName()); return null; }
        catch (RuntimeException failure) { assertTrue(type.isInstance(failure)); return failure; }
    }

    private static final class FakePose implements AbsolutePoseEstimator {
        PoseEstimate value = PoseEstimate.noPose(LoopTimestamp.unavailable());
        int updates;
        @Override public void update(LoopClock clock) { updates++; }
        @Override public PoseEstimate getEstimate() { return value; }
    }
}
