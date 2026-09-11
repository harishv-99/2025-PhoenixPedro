package edu.ftcsushi.fw.sensing.vision.apriltag;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Terminal construction answers preserve independent selection lifetimes and borrowed evidence. */
public final class TagSelectionConstructionTest {
    @Test public void onlyFiveCompleteLifetimeAnswersArePublic() {
        Set<String> expected = new HashSet<>(Arrays.asList("continuous", "holdWhile", "holdUntilReset",
                "holdWhileReacquiringAfterLossSec", "holdUntilResetReacquiringAfterLossSec"));
        Method[] methods = TagSelections.ModeStep.class.getDeclaredMethods();
        assertEquals(expected.size(), methods.length);
        for (Method method : methods) {
            assertTrue(expected.remove(method.getName()));
            assertEquals(TagSelectionSource.class, method.getReturnType());
        }
        assertTrue(expected.isEmpty());
        for (Class<?> stage : TagSelections.class.getDeclaredClasses()) {
            if (!Modifier.isPublic(stage.getModifiers())) continue;
            assertNotEquals("BuildStep", stage.getSimpleName());
            assertNotEquals("StickyWhenLossStep", stage.getSimpleName());
            assertNotEquals("StickyUntilResetLossStep", stage.getSimpleName());
            for (Method method : stage.getDeclaredMethods()) {
                assertNotEquals("build", method.getName());
                assertNotEquals("stickyWhen", method.getName());
                assertNotEquals("stickyUntilReset", method.getName());
            }
        }
    }

    @Test public void allEvidenceFactoriesSupportEveryLifetimeWithoutConstructionTimeSampling() {
        for (int path = 0; path < 4; path++) {
            ManualLoopClock time = new ManualLoopClock();
            int[] reads = {0};
            AprilTagDetections frame = frame(time, 7);
            Source<AprilTagDetections> detections = clock -> { reads[0]++; return frame; };
            CameraMountConfig mount = CameraMountConfig.identity();
            TimeAwareSource<CameraMountConfig> history = (clock, timestamp) -> {
                reads[0]++;
                assertSame(frame.frameTimestamp(), timestamp);
                return mount;
            };
            AbsolutePoseEstimator pose = new AbsolutePoseEstimator() {
                @Override public void update(LoopClock clock) { fail("selector must not update localization"); }
                @Override public PoseEstimate getEstimate() {
                    reads[0]++;
                    return new PoseEstimate(Pose3d.zero(), true, 1, frame.frameTimestamp());
                }
            };
            SimpleTagLayout layout = new SimpleTagLayout().addPose(7, new Pose3d(10, 0, 0, 0, 0, 0));
            TagSelections.PolicyStep policy;
            if (path < 2) {
                TagSelections.VisibleCandidateStep entry = path == 0
                        ? TagSelections.fromVisibleTags(detections, mount)
                        : TagSelections.fromVisibleTags(detections, history);
                policy = entry.among(Collections.singleton(7)).freshWithinSec(1);
            } else {
                TagSelections.FieldPoseCandidateStep entry = path == 2
                        ? TagSelections.fromFieldPose(pose, layout, mount)
                        : TagSelections.fromFieldPose(pose, layout, history);
                policy = entry.among(Collections.singleton(7)).freshWithinSec(1).minQuality(0);
            }
            TagSelections.ModeStep modes = policy.choose(TagSelectionPolicies.closestRange());
            BooleanSource enabled = clock -> { reads[0]++; return true; };
            TagSelectionSource[] selections = allLifetimes(modes, enabled, 0.25);
            assertEquals("no live reads during any terminal factory", 0, reads[0]);
            for (int mode = 0; mode < selections.length; mode++) {
                TagSelectionResult result = selections[mode].get(time.clock());
                assertEquals(7, result.selectedTagId);
                assertEquals(mode != 0, result.latched);
                assertEquals(path < 2, result.hasFreshSelectedObservation);
                int afterFirstRead = reads[0];
                assertSame(result, selections[mode].get(time.clock()));
                assertEquals(afterFirstRead, reads[0]);
            }
        }
    }

    @Test public void retainedLifetimeStageCreatesIndependentHoldingAndReacquiringSources() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections[] frames = {frame(time, 7)};
        boolean[] enabled = {true};
        TagSelections.ModeStep modes = TagSelections.fromVisibleTags(clock -> frames[0],
                CameraMountConfig.identity()).among(new HashSet<>(Arrays.asList(7, 8)))
                .freshWithinSec(1).choose(TagSelectionPolicies.closestRange());
        TagSelectionSource[] selections = allLifetimes(modes, clock -> enabled[0], 0.25);
        for (TagSelectionSource selection : selections) assertEquals(7, selection.get(time.clock()).selectedTagId);

        time.nextCycle(0.25);
        frames[0] = frame(time, 8);
        for (int mode = 0; mode < selections.length; mode++) {
            assertEquals(mode == 0 ? 8 : 7, selections[mode].get(time.clock()).selectedTagId);
        }
        time.nextCycle(0.25);
        assertEquals(8, selections[0].get(time.clock()).selectedTagId);
        assertEquals(7, selections[1].get(time.clock()).selectedTagId);
        assertEquals(7, selections[2].get(time.clock()).selectedTagId);
        assertEquals(8, selections[3].get(time.clock()).selectedTagId);
        assertEquals(8, selections[4].get(time.clock()).selectedTagId);

        time.nextCycle(0.25);
        enabled[0] = false;
        assertFalse(selections[1].get(time.clock()).hasSelection);
        assertFalse(selections[3].get(time.clock()).hasSelection);
        assertEquals(7, selections[2].get(time.clock()).selectedTagId);
        assertEquals(8, selections[4].get(time.clock()).selectedTagId);
        selections[2].reset();
        assertEquals(8, selections[2].get(time.clock()).selectedTagId);
    }

    @Test public void eachTerminalFactorySnapshotsTheLayoutAtItsOwnConstructionBoundary() {
        ManualLoopClock time = new ManualLoopClock();
        AbsolutePoseEstimator pose = new AbsolutePoseEstimator() {
            @Override public void update(LoopClock clock) { fail("selector must not update localization"); }
            @Override public PoseEstimate getEstimate() {
                return new PoseEstimate(Pose3d.zero(), true, 1, time.clock().nowTimestamp());
            }
        };
        SimpleTagLayout layout = new SimpleTagLayout().addPose(7, new Pose3d(10, 0, 0, 0, 0, 0));
        Set<Integer> ids = new HashSet<>(Collections.singleton(7));
        TagSelections.ModeStep modes = TagSelections.fromFieldPose(pose, layout, CameraMountConfig.identity())
                .among(ids).freshWithinSec(1).minQuality(0).choose(TagSelectionPolicies.closestRange());
        ids.add(8);
        TagSelectionSource first = modes.continuous();
        layout.addPose(7, new Pose3d(30, 0, 0, 0, 0, 0));
        TagSelectionSource second = modes.holdUntilReset();
        layout.addPose(7, new Pose3d(50, 0, 0, 0, 0, 0));
        assertEquals(Collections.singleton(7), first.candidateIds());
        assertEquals(Collections.singleton(7), second.candidateIds());
        assertEquals(10, first.get(time.clock()).currentSelectedCandidate.cameraRangeInches(), 0);
        assertEquals(30, second.get(time.clock()).currentSelectedCandidate.cameraRangeInches(), 0);
    }

    @Test public void invalidLifetimeArgumentsRejectWithoutPoisoningTheRetainedStage() {
        ManualLoopClock time = new ManualLoopClock();
        TagSelections.ModeStep modes = TagSelections.fromVisibleTags(Source.constant(frame(time, 7)),
                CameraMountConfig.identity()).among(Collections.singleton(7)).freshWithinSec(1)
                .choose(TagSelectionPolicies.closestRange());
        expect(NullPointerException.class, () -> modes.holdWhile(null));
        expect(NullPointerException.class, () -> modes.holdWhileReacquiringAfterLossSec(null, 0));
        for (double seconds : new double[]{Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, -0.1}) {
            expect(IllegalArgumentException.class, () -> modes.holdWhileReacquiringAfterLossSec(
                    BooleanSource.constant(true), seconds));
            expect(IllegalArgumentException.class, () -> modes.holdUntilResetReacquiringAfterLossSec(seconds));
        }
        assertEquals(7, modes.holdWhile(BooleanSource.constant(true)).get(time.clock()).selectedTagId);
        assertEquals(7, modes.holdUntilResetReacquiringAfterLossSec(0).get(time.clock()).selectedTagId);
    }

    private static TagSelectionSource[] allLifetimes(TagSelections.ModeStep modes,
                                                    BooleanSource enabled, double lossSec) {
        return new TagSelectionSource[]{modes.continuous(), modes.holdWhile(enabled),
                modes.holdUntilReset(), modes.holdWhileReacquiringAfterLossSec(enabled, lossSec),
                modes.holdUntilResetReacquiringAfterLossSec(lossSec)};
    }

    private static AprilTagDetections frame(ManualLoopClock time, int id) {
        return AprilTagDetections.fromFrame(time.clock().nowTimestamp(), Collections.singletonList(
                AprilTagObservation.target(id, new Pose3d(10, 0, 0, 0, 0, 0))));
    }

    private static void expect(Class<? extends RuntimeException> type, Runnable action) {
        try { action.run(); fail("expected " + type.getSimpleName()); }
        catch (RuntimeException failure) { assertTrue(type.isInstance(failure)); }
    }
}
