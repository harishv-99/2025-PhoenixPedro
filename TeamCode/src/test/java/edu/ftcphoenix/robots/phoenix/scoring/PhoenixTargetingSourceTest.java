package edu.ftcphoenix.robots.phoenix.scoring;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Phoenix's ordinary Source graph and successful-only targeting publication. */
public final class PhoenixTargetingSourceTest {

    @Test
    public void lateCalculationFailureDoesNotAdvanceAimReadinessAndCanRetrySameCycle() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixProfile.AutoAimConfig autoAim = profile.autoAim.copy();
        autoAim.aimReadyDebounceSec = 0.5;

        int scoringTagId = autoAim.scoringTagIds().iterator().next();
        CurrentFrameAprilTagSensor tagSensor = new CurrentFrameAprilTagSensor(scoringTagId);
        LateFailingTagLayout fieldLayout = new LateFailingTagLayout(
                profile.field.fixedAprilTagLayout
        );
        int runtimeSetupHasCalls = autoAim.scoringTagIds().size();
        if (!fieldLayout.ids().equals(autoAim.scoringTagIds())) {
            runtimeSetupHasCalls += autoAim.scoringTagIds().size();
        }
        PhoenixTargeting targeting = new PhoenixTargeting(
                autoAim,
                profile.localization.aprilTags.fieldPoseSolver.copy(),
                tagSensor,
                CameraMountConfig.identity(),
                new NoPoseEstimator(),
                fieldLayout,
                Source.constant(autoAim.scoringTagIds()),
                BooleanSource.constant(true),
                BooleanSource.constant(false),
                autoAim.shotVelocityTable
        );

        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        PhoenixCapabilities.TargetingStatus initial = targeting.status();
        assertFalse(initial.selection.hasSelection);
        clock.update(0.25);
        RuntimeException injectedFailure =
                new RuntimeException("injected late field-layout failure");
        fieldLayout.failDirectHasAfter(
                runtimeSetupHasCalls,
                injectedFailure
        );

        try {
            targeting.update(clock);
            fail("expected injected late targeting-calculation failure");
        } catch (RuntimeException actual) {
            assertSame(injectedFailure, actual);
        }
        assertSame("a failed update must retain the prior published snapshot",
                initial, targeting.status());

        clock.update(0.5);
        targeting.update(clock);
        PhoenixCapabilities.TargetingStatus recovered = targeting.status();
        assertTrue(recovered.selection.hasSelection);
        assertTrue(recovered.aimStatus.hasOmegaError);
        assertTrue(recovered.aimStatus.omegaWithin(Math.toRadians(autoAim.aimReadyToleranceDeg)));
        assertFalse("the failed prior cycle must not contribute its dt to readiness", recovered.aimReady);
        assertSame(recovered, targeting.status());

        clock.update(0.75);
        targeting.update(clock);
        assertTrue("two successful ready cycles should satisfy the configured debounce",
                targeting.status().aimReady);

        targeting.reset();
        PhoenixCapabilities.TargetingStatus resetStatus = targeting.status();
        assertFalse(resetStatus.selection.hasSelection);
        clock.update(1.0);
        RuntimeException sameCycleFailure =
                new RuntimeException("injected late same-cycle retry failure");
        fieldLayout.failDirectHasAfter(
                runtimeSetupHasCalls,
                sameCycleFailure
        );
        try {
            targeting.update(clock);
            fail("expected injected late same-cycle retry failure");
        } catch (RuntimeException actual) {
            assertSame(sameCycleFailure, actual);
        }
        assertSame(resetStatus, targeting.status());

        targeting.update(clock);
        PhoenixCapabilities.TargetingStatus sameCycleRecovered = targeting.status();
        assertTrue(sameCycleRecovered.selection.hasSelection);
        assertTrue(sameCycleRecovered.aimStatus.hasOmegaError);
        assertFalse(sameCycleRecovered.aimReady);
        assertSame(sameCycleRecovered, targeting.status());
    }

    @Test
    public void failedCalculationRetriesWithoutPublishingDefaultAndBorrowedResetStopsAtView() {
        PhoenixProfile profile = PhoenixProfile.current();
        RuntimeException injectedFailure = new RuntimeException("injected override read failure");
        int[] overrideReads = {0};
        BooleanSource failOnceOverride = BooleanSource.of(() -> {
            overrideReads[0]++;
            if (overrideReads[0] == 1) {
                throw injectedFailure;
            }
            return false;
        });

        PhoenixTargeting targeting = new PhoenixTargeting(
                profile.autoAim,
                profile.localization.aprilTags.fieldPoseSolver.copy(),
                new EmptyAprilTagSensor(),
                CameraMountConfig.identity(),
                new NoPoseEstimator(),
                profile.field.fixedAprilTagLayout,
                Source.constant(profile.autoAim.scoringTagIds()),
                BooleanSource.constant(true),
                failOnceOverride,
                profile.autoAim.shotVelocityTable
        );
        LoopClock clock = new LoopClock();
        clock.update(1.0);
        PhoenixCapabilities.TargetingStatus initial = targeting.status();

        try {
            targeting.update(clock);
            fail("expected injected targeting input failure");
        } catch (RuntimeException actual) {
            assertSame(injectedFailure, actual);
        }
        assertSame(initial, targeting.status());

        targeting.update(clock);
        PhoenixCapabilities.TargetingStatus recovered = targeting.status();
        assertTrue(recovered.autoAimEnabled);
        assertFalse(recovered.aimOverride);
        assertEquals(2, overrideReads[0]);
        assertSame(recovered, targeting.status());

        targeting.aimOkToShootSource().reset();
        assertSame(recovered, targeting.status());
        assertEquals(2, overrideReads[0]);

        targeting.reset();
        PhoenixCapabilities.TargetingStatus afterOwnerReset = targeting.status();
        assertNotSame(recovered, afterOwnerReset);
        assertFalse(afterOwnerReset.autoAimEnabled);
        assertEquals(2, overrideReads[0]);

        targeting.update(clock);
        PhoenixCapabilities.TargetingStatus republished = targeting.status();
        assertNotSame(afterOwnerReset, republished);
        assertTrue(republished.autoAimEnabled);
        assertEquals(3, overrideReads[0]);
    }

    @Test
    public void eligibleTargetsFreezeBeforeStickyPolicyAndRefreshOnlyAfterOwnerReset() {
        PhoenixProfile profile = PhoenixProfile.current();
        Integer[] configuredTagIds = profile.autoAim.scoringTagIds().toArray(new Integer[0]);
        assertTrue("Phoenix test profile must contain both alliance targets",
                configuredTagIds.length >= 2);
        int firstTagId = configuredTagIds[0];
        int secondTagId = configuredTagIds[1];

        Set<Integer> eligibleTagIds = new LinkedHashSet<Integer>();
        eligibleTagIds.add(firstTagId);
        int[] eligibilityReads = {0};
        CurrentFrameMultipleAprilTagSensor tagSensor =
                new CurrentFrameMultipleAprilTagSensor(firstTagId, secondTagId);
        PhoenixTargeting targeting = targetingFor(
                profile,
                tagSensor,
                Source.of(clock -> {
                    eligibilityReads[0]++;
                    return eligibleTagIds;
                })
        );

        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        targeting.update(clock);
        assertTrue(targeting.status().selection.hasSelection);
        assertEquals(firstTagId, targeting.status().selection.selectedTagId);
        assertEquals(
                "the opposite-alliance observation must be filtered before selection policy",
                Collections.singleton(firstTagId),
                targeting.status().selection.visibleCandidateIds
        );

        eligibleTagIds.clear();
        eligibleTagIds.add(secondTagId);
        clock.update(0.02);
        targeting.update(clock);

        assertTrue(targeting.status().selection.hasSelection);
        assertEquals(
                "the first validated eligibility snapshot must remain frozen for the session",
                firstTagId,
                targeting.status().selection.selectedTagId
        );
        assertEquals(
                profile.autoAim.targetProfileForTag(firstTagId).label,
                targeting.status().targetLabel
        );
        assertEquals(
                "target eligibility must not mutate the full configured scoring catalog",
                configuredTagIds.length,
                profile.autoAim.scoringTagIds().size()
        );
        assertEquals("eligibility must be sampled once per targeting session", 1, eligibilityReads[0]);

        targeting.reset();
        clock.update(0.04);
        targeting.update(clock);
        assertEquals(
                "an explicit owner reset may freeze a new eligible target family",
                secondTagId,
                targeting.status().selection.selectedTagId
        );
        assertEquals(
                Collections.singleton(secondTagId),
                targeting.status().selection.visibleCandidateIds
        );
        assertEquals(2, eligibilityReads[0]);
    }

    @Test
    public void inactiveCatalogTagMissingFromFixedLayoutDoesNotBlockExactEligibleRuntime() {
        PhoenixProfile profile = PhoenixProfile.current();
        Integer[] configuredTagIds = profile.autoAim.scoringTagIds().toArray(new Integer[0]);
        assertTrue("Phoenix test profile must contain both alliance targets",
                configuredTagIds.length >= 2);
        int eligibleTagId = configuredTagIds[0];
        int inactiveTagId = configuredTagIds[1];
        assertTrue(profile.field.fixedAprilTagLayout.has(eligibleTagId));

        CountingCurrentFrameMultipleAprilTagSensor tagSensor =
                new CountingCurrentFrameMultipleAprilTagSensor(eligibleTagId, inactiveTagId);
        int[] eligibilityReads = {0};
        PhoenixTargeting targeting = targetingFor(
                profile,
                tagSensor,
                new OmittingTagLayout(profile.field.fixedAprilTagLayout, inactiveTagId),
                Source.of(clock -> {
                    eligibilityReads[0]++;
                    return Collections.singleton(eligibleTagId);
                })
        );

        assertEquals("construction must not sample mode selection", 0, eligibilityReads[0]);
        assertEquals("construction must not sample the sensor", 0, tagSensor.reads);

        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        targeting.update(clock);

        assertEquals(1, eligibilityReads[0]);
        assertEquals(1, tagSensor.reads);
        assertTrue(targeting.status().selection.hasSelection);
        assertEquals(eligibleTagId, targeting.status().selection.selectedTagId);
        assertEquals(
                "the inactive catalog observation must not enter the exact selector",
                Collections.singleton(eligibleTagId),
                targeting.status().selection.visibleCandidateIds
        );
    }

    @Test
    public void selectedTagMissingFromFixedLayoutFailsBeforeSensorRead() {
        PhoenixProfile profile = PhoenixProfile.current();
        int selectedTagId = profile.autoAim.scoringTagIds().iterator().next();
        CountingEmptyAprilTagSensor tagSensor = new CountingEmptyAprilTagSensor();
        int[] eligibilityReads = {0};
        PhoenixTargeting targeting = targetingFor(
                profile,
                tagSensor,
                new OmittingTagLayout(profile.field.fixedAprilTagLayout, selectedTagId),
                Source.of(clock -> {
                    eligibilityReads[0]++;
                    return Collections.singleton(selectedTagId);
                })
        );

        assertEquals("construction must not sample mode selection", 0, eligibilityReads[0]);
        LoopClock clock = new LoopClock();
        clock.update(1.0);
        try {
            targeting.update(clock);
            fail("expected selected tag absent from the fixed layout to fail");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("eligibleScoringTagIds"));
            assertTrue(expected.getMessage().toLowerCase().contains("fixed"));
            assertTrue(expected.getMessage().contains(Integer.toString(selectedTagId)));
        }

        assertEquals(1, eligibilityReads[0]);
        assertEquals("fixed-layout validation must precede sensor selection", 0, tagSensor.reads);
        assertFalse(targeting.status().selection.hasSelection);
    }

    @Test
    public void missingCatalogDefersToPrestartAndFailsActionablyBeforeSensorRead() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixProfile.AutoAimConfig autoAim = profile.autoAim.copy();
        autoAim.scoringTargets = null;
        CountingEmptyAprilTagSensor tagSensor = new CountingEmptyAprilTagSensor();
        int[] eligibilityReads = {0};

        PhoenixTargeting targeting = new PhoenixTargeting(
                autoAim,
                profile.localization.aprilTags.fieldPoseSolver.copy(),
                tagSensor,
                CameraMountConfig.identity(),
                new NoPoseEstimator(),
                profile.field.fixedAprilTagLayout,
                Source.of(clock -> {
                    eligibilityReads[0]++;
                    return Collections.singleton(autoAim.redAllianceScoringTagId);
                }),
                BooleanSource.constant(true),
                BooleanSource.constant(false),
                autoAim.shotVelocityTable
        );

        assertEquals("constructor must leave the missing catalog to managed prestart", 0,
                eligibilityReads[0]);
        assertEquals(0, tagSensor.reads);
        assertFalse(targeting.status().selection.hasSelection);

        LoopClock clock = new LoopClock();
        clock.update(1.0);
        try {
            targeting.update(clock);
            fail("expected a missing catalog to fail if targeting update is improperly reached");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("PhoenixProfile.autoAim.scoringTargets"));
            assertTrue(expected.getMessage().contains("prestart readiness"));
            assertTrue(expected.getMessage().contains("block START"));
        }

        assertEquals("catalog validation must precede mode-selection sampling", 0,
                eligibilityReads[0]);
        assertEquals("catalog validation must precede sensor selection", 0, tagSensor.reads);
    }

    @Test
    public void eligibleTargetsFreezeOnceEvenWhenFirstSensorCalculationFails() {
        PhoenixProfile profile = PhoenixProfile.current();
        Integer[] configuredTagIds = profile.autoAim.scoringTagIds().toArray(new Integer[0]);
        assertTrue("Phoenix test profile must contain both alliance targets",
                configuredTagIds.length >= 2);
        int firstTagId = configuredTagIds[0];
        int secondTagId = configuredTagIds[1];

        Set<Integer> eligibleTagIds = new LinkedHashSet<Integer>();
        eligibleTagIds.add(firstTagId);
        int[] eligibilityReads = {0};
        RuntimeException injectedFailure = new RuntimeException("injected first sensor failure");
        FailOnceCurrentFrameMultipleAprilTagSensor tagSensor =
                new FailOnceCurrentFrameMultipleAprilTagSensor(
                        injectedFailure,
                        firstTagId,
                        secondTagId
                );
        PhoenixTargeting targeting = targetingFor(
                profile,
                tagSensor,
                Source.of(clock -> {
                    eligibilityReads[0]++;
                    return eligibleTagIds;
                })
        );
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        PhoenixCapabilities.TargetingStatus initial = targeting.status();

        try {
            targeting.update(clock);
            fail("expected injected first sensor failure");
        } catch (RuntimeException actual) {
            assertSame(injectedFailure, actual);
        }
        assertEquals(1, eligibilityReads[0]);
        assertEquals(1, tagSensor.reads);
        assertSame("failed calculation must not publish a partial status", initial, targeting.status());

        eligibleTagIds.clear();
        eligibleTagIds.add(secondTagId);
        targeting.update(clock);

        assertEquals("validated eligibility must remain frozen across calculation retry",
                1, eligibilityReads[0]);
        assertEquals(2, tagSensor.reads);
        assertTrue(targeting.status().selection.hasSelection);
        assertEquals(firstTagId, targeting.status().selection.selectedTagId);
        assertEquals(Collections.singleton(firstTagId),
                targeting.status().selection.visibleCandidateIds);
    }

    @Test
    public void eligibleTargetsFreezeOnceEvenWhenFirstGuidanceQueryFails() {
        PhoenixProfile profile = PhoenixProfile.current();
        Integer[] configuredTagIds = profile.autoAim.scoringTagIds().toArray(new Integer[0]);
        assertTrue("Phoenix test profile must contain both alliance targets",
                configuredTagIds.length >= 2);
        int firstTagId = configuredTagIds[0];
        int secondTagId = configuredTagIds[1];

        Set<Integer> eligibleTagIds = new LinkedHashSet<Integer>();
        eligibleTagIds.add(firstTagId);
        int[] eligibilityReads = {0};
        RuntimeException injectedFailure = new RuntimeException("injected first query failure");
        FailOncePoseEstimator poseEstimator = new FailOncePoseEstimator(injectedFailure);
        PhoenixTargeting targeting = new PhoenixTargeting(
                profile.autoAim,
                profile.localization.aprilTags.fieldPoseSolver.copy(),
                new CurrentFrameMultipleAprilTagSensor(firstTagId, secondTagId),
                CameraMountConfig.identity(),
                poseEstimator,
                profile.field.fixedAprilTagLayout,
                Source.of(clock -> {
                    eligibilityReads[0]++;
                    return eligibleTagIds;
                }),
                BooleanSource.constant(true),
                BooleanSource.constant(false),
                profile.autoAim.shotVelocityTable
        );
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        PhoenixCapabilities.TargetingStatus initial = targeting.status();

        try {
            targeting.update(clock);
            fail("expected injected first guidance-query failure");
        } catch (RuntimeException actual) {
            assertSame(injectedFailure, actual);
        }
        assertEquals(1, eligibilityReads[0]);
        assertEquals(1, poseEstimator.reads);
        assertSame("failed query must not publish a partial status", initial, targeting.status());

        eligibleTagIds.clear();
        eligibleTagIds.add(secondTagId);
        targeting.update(clock);

        assertEquals("validated eligibility must remain frozen across query retry",
                1, eligibilityReads[0]);
        assertEquals(2, poseEstimator.reads);
        assertTrue(targeting.status().selection.hasSelection);
        assertEquals(firstTagId, targeting.status().selection.selectedTagId);
        assertEquals(Collections.singleton(firstTagId),
                targeting.status().selection.visibleCandidateIds);
    }

    @Test
    public void aimConsumersFailActionablyUntilTargetingPublishesItsRuntime() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixTargeting targeting = targetingFor(
                profile,
                new EmptyAprilTagSensor(),
                Source.constant(Collections.singleton(
                        profile.autoAim.scoringTagIds().iterator().next()
                ))
        );
        DriveOverlay overlay = targeting.aimOverlay();
        Task aimTask = targeting.aimTask(new NoopDriveSink(), null);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);

        try {
            targeting.aimOverlay().onEnable(clock);
            fail("expected overlay enable before targeting.update(clock) to fail");
        } catch (IllegalStateException expected) {
            assertActionableTargetingUpdateMessage(expected);
        }

        try {
            overlay.get(clock);
            fail("expected an overlay sample before targeting.update(clock) to fail");
        } catch (IllegalStateException expected) {
            assertActionableTargetingUpdateMessage(expected);
        }

        try {
            aimTask.start(clock);
            fail("expected an aim Task start before targeting.update(clock) to fail");
        } catch (IllegalStateException expected) {
            assertActionableTargetingUpdateMessage(expected);
        }
    }

    @Test
    public void resetRequiresFreshOverlayWithoutPoisoningTheNextSession() {
        PhoenixProfile profile = PhoenixProfile.current();
        int selectedTagId = profile.autoAim.scoringTagIds().iterator().next();
        PhoenixTargeting targeting = targetingFor(
                profile,
                new CurrentFrameAprilTagSensor(selectedTagId),
                Source.constant(Collections.singleton(selectedTagId))
        );
        DriveOverlay firstSessionOverlay = targeting.aimOverlay();
        LoopClock clock = new LoopClock();
        clock.reset(0.0);

        targeting.update(clock);
        firstSessionOverlay.onEnable(clock);
        assertTrue(firstSessionOverlay.get(clock) != null);

        firstSessionOverlay.onDisable(clock);
        targeting.reset();
        clock.update(0.02);
        targeting.update(clock);
        try {
            firstSessionOverlay.get(clock);
            fail("expected an overlay retained across reset to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("reset"));
            assertTrue(expected.getMessage().contains("fresh aimOverlay"));
        }

        DriveOverlay secondSessionOverlay = targeting.aimOverlay();
        secondSessionOverlay.onEnable(clock);
        assertTrue("a fresh overlay must own an independent next-session lifecycle",
                secondSessionOverlay.get(clock) != null);
    }

    @Test
    public void invalidEligibleTargetSetFailsBeforeSensorSelectionWithActionableErrors() {
        PhoenixProfile profile = PhoenixProfile.current();
        CountingEmptyAprilTagSensor tagSensor = new CountingEmptyAprilTagSensor();
        int unknownTagId = Integer.MAX_VALUE;
        PhoenixTargeting targeting = targetingFor(
                profile,
                tagSensor,
                Source.constant(Collections.singleton(unknownTagId))
        );
        LoopClock clock = new LoopClock();
        clock.update(1.0);

        try {
            targeting.update(clock);
            fail("expected an eligible tag absent from the configured catalog to fail");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("eligibleScoringTagIds"));
            assertTrue(expected.getMessage().contains("scoringTargets"));
            assertTrue(expected.getMessage().contains(Integer.toString(unknownTagId)));
            for (Integer configuredTagId : profile.autoAim.scoringTagIds()) {
                assertTrue(expected.getMessage().contains(configuredTagId.toString()));
            }
        }
        assertEquals("eligibility must be validated before reading detections", 0, tagSensor.reads);
        assertFalse(targeting.status().selection.hasSelection);
    }

    @Test
    public void nullEmptyAndNullMemberEligibleTargetSetsAreRejected() {
        PhoenixProfile profile = PhoenixProfile.current();
        assertInvalidEligibleSet(
                profile,
                new NullEligibleTagIdsSource(),
                "returned null"
        );
        assertInvalidEligibleSet(
                profile,
                Source.constant(Collections.<Integer>emptySet()),
                "at least one"
        );
        Set<Integer> containsNull = new LinkedHashSet<Integer>();
        containsNull.add(null);
        assertInvalidEligibleSet(
                profile,
                Source.constant(containsNull),
                "must not contain null"
        );
    }

    private static void assertInvalidEligibleSet(PhoenixProfile profile,
                                                 Source<Set<Integer>> eligibleTagIds,
                                                 String expectedMessage) {
        CountingEmptyAprilTagSensor tagSensor = new CountingEmptyAprilTagSensor();
        PhoenixTargeting targeting = targetingFor(profile, tagSensor, eligibleTagIds);
        LoopClock clock = new LoopClock();
        clock.update(1.0);

        try {
            targeting.update(clock);
            fail("expected invalid eligible scoring tags to fail");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("eligibleScoringTagIds"));
            assertTrue(expected.getMessage().contains(expectedMessage));
        }
        assertEquals("eligibility must be validated before reading detections", 0, tagSensor.reads);
    }

    private static PhoenixTargeting targetingFor(PhoenixProfile profile,
                                                  AprilTagSensor tagSensor,
                                                  Source<Set<Integer>> eligibleTagIds) {
        return targetingFor(
                profile,
                tagSensor,
                profile.field.fixedAprilTagLayout,
                eligibleTagIds
        );
    }

    private static PhoenixTargeting targetingFor(PhoenixProfile profile,
                                                  AprilTagSensor tagSensor,
                                                  TagLayout fieldTagLayout,
                                                  Source<Set<Integer>> eligibleTagIds) {
        return new PhoenixTargeting(
                profile.autoAim,
                profile.localization.aprilTags.fieldPoseSolver.copy(),
                tagSensor,
                CameraMountConfig.identity(),
                new NoPoseEstimator(),
                fieldTagLayout,
                eligibleTagIds,
                BooleanSource.constant(true),
                BooleanSource.constant(false),
                profile.autoAim.shotVelocityTable
        );
    }

    private static void assertActionableTargetingUpdateMessage(IllegalStateException failure) {
        assertTrue(failure.getMessage().contains("PhoenixTargeting"));
        assertTrue(failure.getMessage().contains("update"));
    }

    private static final class CurrentFrameAprilTagSensor implements AprilTagSensor {
        private final int tagId;
        private long lastCycle = Long.MIN_VALUE;
        private AprilTagDetections last = AprilTagDetections.none();

        CurrentFrameAprilTagSensor(int tagId) {
            this.tagId = tagId;
        }

        @Override
        public AprilTagDetections get(LoopClock clock) {
            if (clock.cycle() != lastCycle) {
                last = AprilTagDetections.fromFrame(
                        clock.nowTimestamp(),
                        Collections.singletonList(AprilTagObservation.target(
                                tagId,
                                new Pose3d(36.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                        ))
                );
                lastCycle = clock.cycle();
            }
            return last;
        }
    }

    private static final class CurrentFrameMultipleAprilTagSensor implements AprilTagSensor {
        private final int[] tagIds;
        private long lastCycle = Long.MIN_VALUE;
        private AprilTagDetections last = AprilTagDetections.none();

        CurrentFrameMultipleAprilTagSensor(int... tagIds) {
            this.tagIds = tagIds.clone();
        }

        @Override
        public AprilTagDetections get(LoopClock clock) {
            if (clock.cycle() != lastCycle) {
                List<AprilTagObservation> observations =
                        new ArrayList<AprilTagObservation>();
                for (int i = 0; i < tagIds.length; i++) {
                    observations.add(AprilTagObservation.target(
                            tagIds[i],
                            new Pose3d(36.0, i * 4.0, 0.0, 0.0, 0.0, 0.0)
                    ));
                }
                last = AprilTagDetections.fromFrame(clock.nowTimestamp(), observations);
                lastCycle = clock.cycle();
            }
            return last;
        }
    }

    private static final class CountingCurrentFrameMultipleAprilTagSensor
            implements AprilTagSensor {
        private final int[] tagIds;
        private long lastCycle = Long.MIN_VALUE;
        private AprilTagDetections last = AprilTagDetections.none();
        int reads;

        CountingCurrentFrameMultipleAprilTagSensor(int... tagIds) {
            this.tagIds = tagIds.clone();
        }

        @Override
        public AprilTagDetections get(LoopClock clock) {
            if (clock.cycle() != lastCycle) {
                reads++;
                last = currentFrameDetections(clock, tagIds);
                lastCycle = clock.cycle();
            }
            return last;
        }
    }

    private static final class FailOnceCurrentFrameMultipleAprilTagSensor
            implements AprilTagSensor {
        private final RuntimeException firstFailure;
        private final int[] tagIds;
        private long lastCycle = Long.MIN_VALUE;
        private AprilTagDetections last = AprilTagDetections.none();
        int reads;

        FailOnceCurrentFrameMultipleAprilTagSensor(RuntimeException firstFailure, int... tagIds) {
            this.firstFailure = firstFailure;
            this.tagIds = tagIds.clone();
        }

        @Override
        public AprilTagDetections get(LoopClock clock) {
            if (clock.cycle() != lastCycle) {
                reads++;
                if (reads == 1) {
                    throw firstFailure;
                }
                last = currentFrameDetections(clock, tagIds);
                lastCycle = clock.cycle();
            }
            return last;
        }
    }

    private static AprilTagDetections currentFrameDetections(LoopClock clock, int[] tagIds) {
        List<AprilTagObservation> observations = new ArrayList<AprilTagObservation>();
        for (int i = 0; i < tagIds.length; i++) {
            observations.add(AprilTagObservation.target(
                    tagIds[i],
                    new Pose3d(36.0, i * 4.0, 0.0, 0.0, 0.0, 0.0)
            ));
        }
        return AprilTagDetections.fromFrame(clock.nowTimestamp(), observations);
    }

    private static final class CountingEmptyAprilTagSensor implements AprilTagSensor {
        int reads;

        @Override
        public AprilTagDetections get(LoopClock clock) {
            reads++;
            return AprilTagDetections.none();
        }
    }

    /** Named test boundary adapter used to verify an invalid advanced Source implementation. */
    private static final class NullEligibleTagIdsSource implements Source<Set<Integer>> {
        @Override
        public Set<Integer> get(LoopClock clock) {
            return null;
        }
    }

    private static final class LateFailingTagLayout implements TagLayout {
        private final TagLayout delegate;
        private RuntimeException nextDirectHasFailure;
        private int successfulDirectHasCallsBeforeFailure;

        LateFailingTagLayout(TagLayout delegate) {
            this.delegate = delegate;
        }

        void failDirectHasAfter(int successfulCalls, RuntimeException failure) {
            successfulDirectHasCallsBeforeFailure = successfulCalls;
            nextDirectHasFailure = failure;
        }

        @Override
        public Pose3d getFieldToTagPose(int id) {
            return delegate.getFieldToTagPose(id);
        }

        @Override
        public Set<Integer> ids() {
            return delegate.ids();
        }

        @Override
        public boolean has(int id) {
            if (nextDirectHasFailure != null) {
                if (successfulDirectHasCallsBeforeFailure > 0) {
                    successfulDirectHasCallsBeforeFailure--;
                    return getFieldToTagPose(id) != null;
                }
                RuntimeException failure = nextDirectHasFailure;
                nextDirectHasFailure = null;
                throw failure;
            }
            return getFieldToTagPose(id) != null;
        }
    }

    private static final class OmittingTagLayout implements TagLayout {
        private final TagLayout delegate;
        private final int omittedTagId;

        OmittingTagLayout(TagLayout delegate, int omittedTagId) {
            this.delegate = delegate;
            this.omittedTagId = omittedTagId;
        }

        @Override
        public Pose3d getFieldToTagPose(int id) {
            return id == omittedTagId ? null : delegate.getFieldToTagPose(id);
        }

        @Override
        public Set<Integer> ids() {
            LinkedHashSet<Integer> retained = new LinkedHashSet<Integer>(delegate.ids());
            retained.remove(omittedTagId);
            return Collections.unmodifiableSet(retained);
        }
    }

    /** Named test boundary adapter; direct Source implementation is not an ordinary robot recipe. */
    private static final class EmptyAprilTagSensor implements AprilTagSensor {
        @Override
        public AprilTagDetections get(LoopClock clock) {
            return AprilTagDetections.none();
        }
    }

    private static final class NoPoseEstimator implements AbsolutePoseEstimator {
        private final PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());

        @Override
        public void update(LoopClock clock) {
            // No stateful localization is needed for this targeting-source test.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }

    private static final class FailOncePoseEstimator implements AbsolutePoseEstimator {
        private final RuntimeException firstFailure;
        private final PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        int reads;

        FailOncePoseEstimator(RuntimeException firstFailure) {
            this.firstFailure = firstFailure;
        }

        @Override
        public void update(LoopClock clock) {
            // Guidance reads the already-owned estimator snapshot in this focused test.
        }

        @Override
        public PoseEstimate getEstimate() {
            reads++;
            if (reads == 1) {
                throw firstFailure;
            }
            return estimate;
        }
    }

    private static final class NoopDriveSink implements DriveCommandSink {
        @Override
        public void drive(DriveSignal signal) {
            // No hardware output is needed for deferred aim-Task construction tests.
        }

        @Override
        public void stop() {
            // No hardware output is owned by this test sink.
        }
    }
}
