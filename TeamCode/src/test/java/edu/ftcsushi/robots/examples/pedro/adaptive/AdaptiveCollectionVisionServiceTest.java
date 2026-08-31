package edu.ftcsushi.robots.examples.pedro.adaptive;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.ftc.vision.FtcLimelightVisionLane;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseTrajectoryEstimator;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Hardware-free contract tests for the optional adaptive-collection vision owner. */
public final class AdaptiveCollectionVisionServiceTest {
    private static final double EPSILON = 1.0e-8;

    @Test
    public void publicSurfaceAndConfigurationKeepOneValidatedOwnerPath() {
        Constructor<?>[] publicConstructors =
                AdaptiveCollectionVisionService.class.getConstructors();
        assertEquals(1, publicConstructors.length);
        assertArrayEquals(
                new Class<?>[]{
                        HardwareMap.class,
                        TimeAwareSource.class,
                        AdaptiveCollectionVisionService.Config.class
                },
                publicConstructors[0].getParameterTypes()
        );

        UnusedPoseLookup poseHistory = new UnusedPoseLookup();
        FakeFrameOwner frames = new FakeFrameOwner();

        AdaptiveCollectionVisionService.Config config = validConfig();
        config.limelight = null;
        assertInvalid(config, poseHistory, frames);

        config = validConfig();
        config.cameraMount = null;
        assertInvalid(config, poseHistory, frames);

        for (double invalid : new double[]{
                0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY
        }) {
            config = validConfig();
            config.maxFrameAgeSec = invalid;
            assertInvalid(config, poseHistory, frames);

            config = validConfig();
            config.bandWidthInches = invalid;
            assertInvalid(config, poseHistory, frames);
        }

        config = validConfig();
        config.minCollectionFieldXInches = 4.0;
        config.maxCollectionFieldXInches = 3.0;
        assertInvalid(config, poseHistory, frames);

        config = validConfig();
        config.minCollectionFieldYInches = 4.0;
        config.maxCollectionFieldYInches = 4.0;
        assertInvalid(config, poseHistory, frames);

        config = validConfig();
        config.maxCollectionFieldYInches = Double.NaN;
        assertInvalid(config, poseHistory, frames);

        config = validConfig();
        config.bandWidthInches = 41.0;
        assertInvalid(config, poseHistory, frames);

        assertThrows(NullPointerException.class,
                () -> new AdaptiveCollectionVisionService(poseHistory, validConfig(), null));
        assertThrows(NullPointerException.class,
                () -> new AdaptiveCollectionVisionService(null, validConfig(), frames));

        NoLookupHardwareMap hardwareMap = new NoLookupHardwareMap();
        config = validConfig();
        config.limelight.pipelineIndex = -1;
        AdaptiveCollectionVisionService.Config invalidNestedConfig = config;
        assertThrows(IllegalArgumentException.class,
                () -> new AdaptiveCollectionVisionService(
                        hardwareMap, poseHistory, invalidNestedConfig));
        assertEquals(0, hardwareMap.lookupCalls);
    }

    @Test
    public void configurationIsSnapshottedBeforeLaterAuthoringMutation() {
        Fixture fixture = new Fixture();
        AdaptiveCollectionVisionService.Config config = fixture.config;

        config.cameraMount = CameraMountConfig.identity();
        config.maxFrameAgeSec = 1.0e-9;
        config.minCollectionFieldXInches = 50.0;
        config.maxCollectionFieldXInches = 60.0;
        config.minCollectionFieldYInches = 50.0;
        config.maxCollectionFieldYInches = 60.0;
        config.bandWidthInches = 1.0;
        config.limelight.hardwareName = "mutated";

        LoopTimestamp exposure = fixture.recordCurrentPose(Pose3d.zero());
        fixture.time.nextCycle(0.1);
        fixture.frames.frame = observed(exposure, detectorForFloorPoint(10.0, 0.0));
        fixture.service.start(fixture.time.clock());

        assertTrue(fixture.service.decision().hasSelection());
        assertSame(exposure, fixture.service.decision().frameTimestamp());
        assertEquals(0.1, fixture.service.decision().frameAgeSec(), EPSILON);
        assertEquals(-5.0, fixture.service.decision().selectedBandCenterYInches(), EPSILON);
    }

    @Test
    public void realSdkDetectorObjectsAreImmediatelyCopiedToTwoImmutableAngles() {
        LLResultTypes.DetectorResult detector = detectorResult(12.5, -7.25);
        ArrayList<LLResultTypes.DetectorResult> sdkResults =
                new ArrayList<LLResultTypes.DetectorResult>();
        sdkResults.add(detector);

        List<AdaptiveCollectionVisionService.DetectorAngles> copied =
                AdaptiveCollectionVisionService.copyDetectorAngles(sdkResults);
        sdkResults.clear();
        setDoubleField(detector, "targetXDegreesNoCrosshair", 99.0);
        setDoubleField(detector, "targetYDegreesNoCrosshair", 88.0);

        assertEquals(1, copied.size());
        assertEquals(12.5, copied.get(0).horizontalRightDeg, 0.0);
        assertEquals(-7.25, copied.get(0).verticalDownDeg, 0.0);
        assertThrows(UnsupportedOperationException.class,
                () -> copied.add(new AdaptiveCollectionVisionService.DetectorAngles(0.0, 0.0)));

        Field[] retainedFields =
                AdaptiveCollectionVisionService.DetectorAngles.class.getDeclaredFields();
        assertEquals(2, retainedFields.length);
        assertEquals(double.class, retainedFields[0].getType());
        assertEquals(double.class, retainedFields[1].getType());
    }

    @Test
    public void zeroOneAndManyDetectionsPublishTypedDeterministicDecisions() {
        Fixture fixture = new Fixture();
        fixture.recordCurrentPose(Pose3d.zero());
        fixture.frames.frame = observed(
                fixture.time.clock().nowTimestamp(), Collections.emptyList());
        fixture.service.start(fixture.time.clock());
        assertUnavailable(fixture.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.ZERO_DETECTIONS);
        assertEquals(1, fixture.frames.reads);
        assertEquals(0, fixture.poseLookups.queries);
        assertFalse(fixture.service.decision().hasPoseLookup());
        assertThrows(IllegalStateException.class,
                fixture.service.decision()::poseLookup);
        assertEquals(0, fixture.pose.updateCalls);

        fixture.nextObservation(detectorForFloorPoint(10.0, 0.0));
        AdaptiveCollectionVisionService.Decision one = fixture.service.decision();
        assertTrue(one.hasSelection());
        assertEquals(1, one.detectionCount());
        assertEquals(1, one.projectablePointCount());
        assertEquals(1, one.inBoxPointCount());
        assertEquals(-10.0, one.selectedBandStartYInches(), EPSILON);
        assertEquals(0.0, one.selectedBandEndYInches(), EPSILON);
        assertEquals(-5.0, one.selectedBandCenterYInches(), EPSILON);
        assertEquals(1, one.selectedBandPointCount());
        assertThrows(IllegalStateException.class, one::unavailableReason);

        List<AdaptiveCollectionVisionService.DetectorAngles> ordered = Arrays.asList(
                detectorForFloorPoint(10.0, -12.0),
                detectorForFloorPoint(10.0, -11.0),
                detectorForFloorPoint(10.0, 11.0),
                detectorForFloorPoint(10.0, 12.0)
        );
        fixture.nextObservation(ordered);
        AdaptiveCollectionVisionService.Decision orderedDecision = fixture.service.decision();
        assertEquals(-20.0, orderedDecision.selectedBandStartYInches(), EPSILON);
        assertEquals(2, orderedDecision.selectedBandPointCount());

        ArrayList<AdaptiveCollectionVisionService.DetectorAngles> reversed =
                new ArrayList<AdaptiveCollectionVisionService.DetectorAngles>(ordered);
        Collections.reverse(reversed);
        fixture.nextObservation(reversed);
        AdaptiveCollectionVisionService.Decision reversedDecision = fixture.service.decision();
        assertEquals(orderedDecision.selectedBandStartYInches(),
                reversedDecision.selectedBandStartYInches(), 0.0);
        assertEquals(orderedDecision.selectedBandPointCount(),
                reversedDecision.selectedBandPointCount());

        List<AdaptiveCollectionVisionService.DetectorAngles> countFirst = Arrays.asList(
                detectorForFloorPoint(10.0, -18.0),
                detectorForFloorPoint(10.0, 6.0),
                detectorForFloorPoint(10.0, 7.0),
                detectorForFloorPoint(10.0, 8.0)
        );
        fixture.nextObservation(countFirst);
        AdaptiveCollectionVisionService.Decision countFirstDecision = fixture.service.decision();
        assertEquals(-2.0, countFirstDecision.selectedBandStartYInches(), EPSILON);
        assertEquals(3, countFirstDecision.selectedBandPointCount());

        Collections.reverse(countFirst);
        fixture.nextObservation(countFirst);
        assertEquals(-2.0, fixture.service.decision().selectedBandStartYInches(), EPSILON);
        assertEquals(3, fixture.service.decision().selectedBandPointCount());

        assertTrue(one.hasSelection());
        assertEquals(-5.0, one.selectedBandCenterYInches(), EPSILON);
    }

    @Test
    public void frameTimingDistinguishesUnavailableResetFutureStaleAndWrongClock() {
        Fixture unavailable = new Fixture();
        unavailable.frames.frame = AdaptiveCollectionVisionService.CopiedFrame.unavailable();
        unavailable.service.start(unavailable.time.clock());
        assertUnavailable(unavailable.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.FRAME_UNAVAILABLE);
        assertSame(LoopTimestamp.unavailable(),
                unavailable.service.decision().frameTimestamp());
        assertFalse(unavailable.service.decision().hasPoseLookup());
        assertThrows(IllegalStateException.class,
                unavailable.service.decision()::poseLookup);

        Fixture stale = new Fixture();
        stale.recordCurrentPose(Pose3d.zero());
        stale.frames.frame = observed(
                stale.time.clock().timestampSecondsAgo(0.251),
                detectorForFloorPoint(10.0, 0.0));
        stale.service.start(stale.time.clock());
        assertUnavailable(stale.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.FRAME_STALE);
        assertEquals(0.251, stale.service.decision().frameAgeSec(), EPSILON);
        assertEquals(0, stale.poseLookups.queries);

        Fixture inclusiveBoundary = new Fixture();
        LoopTimestamp inclusiveExposure =
                inclusiveBoundary.recordCurrentPose(Pose3d.zero());
        inclusiveBoundary.time.nextCycle(0.25);
        inclusiveBoundary.frames.frame = observed(
                inclusiveExposure,
                detectorForFloorPoint(10.0, 0.0));
        inclusiveBoundary.service.start(inclusiveBoundary.time.clock());
        assertTrue(inclusiveBoundary.service.decision().hasSelection());

        Fixture reset = new Fixture();
        LoopTimestamp priorEpoch = reset.time.clock().nowTimestamp();
        reset.time.clock().reset(0.0);
        reset.recordCurrentPose(Pose3d.zero());
        reset.frames.frame = observed(priorEpoch, detectorForFloorPoint(10.0, 0.0));
        reset.service.start(reset.time.clock());
        assertUnavailable(reset.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.FRAME_RESET_OR_FUTURE);

        Fixture future = new Fixture(1.0);
        LoopTimestamp later = future.time.clock().nowTimestamp();
        future.time.clock().update(0.5);
        future.recordCurrentPose(Pose3d.zero());
        future.frames.frame = observed(later, detectorForFloorPoint(10.0, 0.0));
        future.service.start(future.time.clock());
        assertUnavailable(future.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.FRAME_RESET_OR_FUTURE);

        Fixture wrongClock = new Fixture();
        wrongClock.recordCurrentPose(Pose3d.zero());
        wrongClock.frames.frame = observed(
                wrongClock.time.clock().nowTimestamp(),
                detectorForFloorPoint(10.0, 0.0));
        wrongClock.service.start(wrongClock.time.clock());
        AdaptiveCollectionVisionService.Decision prior = wrongClock.service.decision();
        ManualLoopClock otherTime = new ManualLoopClock();
        wrongClock.frames.frame = observed(
                otherTime.clock().nowTimestamp(),
                detectorForFloorPoint(10.0, 0.0));
        assertThrows(IllegalArgumentException.class,
                () -> wrongClock.service.update(wrongClock.time.clock()));
        assertSame(prior, wrongClock.service.decision());
    }

    @Test
    public void exactAndInterpolatedHistoryLookupsSupportMovingProjectionAndAreRetained() {
        Fixture exact = new Fixture();
        LoopTimestamp exactTimestamp = exact.recordCurrentPose(Pose3d.zero());
        exact.frames.frame = observed(
                exactTimestamp, detectorForFloorPoint(10.0, 0.0));
        exact.service.start(exact.time.clock());

        AdaptiveCollectionVisionService.Decision exactDecision = exact.service.decision();
        assertTrue(exactDecision.hasSelection());
        assertTrue(exactDecision.hasPoseLookup());
        assertSame(exact.poseLookups.lastLookup, exactDecision.poseLookup());
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT,
                exactDecision.poseLookup().kind());
        assertSame(exactTimestamp, exactDecision.poseLookup().timestamp());
        assertThrows(IllegalStateException.class,
                exactDecision.poseLookup()::unavailableReason);

        Fixture moving = new Fixture();
        moving.recordCurrentPose(new Pose3d(0.0, -4.0, 0.0, 0.0, 0.0, 0.0));
        moving.time.nextCycle(0.1);
        moving.recordCurrentPose(new Pose3d(6.0, 4.0, 0.0, 0.0, 0.0, 0.0));
        LoopTimestamp exposure = moving.time.clock().timestampSecondsAgo(0.05);
        moving.frames.frame = observed(exposure, detectorForFloorPoint(10.0, 0.0));
        moving.service.start(moving.time.clock());

        AdaptiveCollectionVisionService.Decision movingDecision = moving.service.decision();
        assertTrue(movingDecision.hasSelection());
        assertSame(moving.poseLookups.lastLookup, movingDecision.poseLookup());
        assertEquals(PlanarPoseHistory.Lookup.Kind.INTERPOLATED,
                movingDecision.poseLookup().kind());
        assertEquals(3.0, movingDecision.poseLookup().fieldToRobotPose().xInches, EPSILON);
        assertEquals(0.0, movingDecision.poseLookup().fieldToRobotPose().yInches, EPSILON);
        assertEquals(-10.0, movingDecision.selectedBandStartYInches(), EPSILON);
        assertEquals(0.0, movingDecision.selectedBandEndYInches(), EPSILON);
        assertSame(exposure, movingDecision.poseLookup().timestamp());
        assertEquals(0, moving.pose.updateCalls);
    }

    @Test
    public void mismatchedHistoryTimestampFailsWithoutReplacingTheCachedDecision() {
        Fixture fixture = new Fixture();
        LoopTimestamp firstTimestamp = fixture.recordCurrentPose(Pose3d.zero());
        fixture.frames.frame = observed(
                firstTimestamp,
                detectorForFloorPoint(10.0, 0.0)
        );
        fixture.service.start(fixture.time.clock());
        AdaptiveCollectionVisionService.Decision retained = fixture.service.decision();
        assertTrue(retained.hasSelection());

        fixture.time.nextCycle(0.1);
        LoopTimestamp nextTimestamp = fixture.recordCurrentPose(
                new Pose3d(4.0, 4.0, 0.0, 0.0, 0.0, 0.0)
        );
        fixture.frames.frame = observed(
                nextTimestamp,
                detectorForFloorPoint(10.0, 0.0)
        );
        fixture.poseLookups.forcedLookup = fixture.poseHistory.lookupSource().getAt(
                fixture.time.clock(),
                firstTimestamp
        );

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                () -> fixture.service.update(fixture.time.clock())
        );

        assertTrue(failure.getMessage().contains("different timestamp"));
        assertSame(retained, fixture.service.decision());
    }

    @Test
    public void historyFailuresRetainExactTypedLookupForExplicitFallback() {
        Fixture empty = new Fixture();
        empty.frames.frame = observed(
                empty.time.clock().nowTimestamp(), detectorForFloorPoint(10.0, 0.0));
        empty.service.start(empty.time.clock());
        assertHistoryUnavailable(
                empty,
                PlanarPoseHistory.Lookup.UnavailableReason.EMPTY
        );

        Fixture afterLatest = new Fixture();
        afterLatest.recordCurrentPose(Pose3d.zero());
        afterLatest.time.nextCycle(0.1);
        afterLatest.frames.frame = observed(
                afterLatest.time.clock().nowTimestamp(),
                detectorForFloorPoint(10.0, 0.0));
        afterLatest.service.start(afterLatest.time.clock());
        assertHistoryUnavailable(
                afterLatest,
                PlanarPoseHistory.Lookup.UnavailableReason.AFTER_LATEST
        );

        Fixture largeCorrection = new Fixture();
        largeCorrection.recordCurrentPose(Pose3d.zero());
        largeCorrection.time.nextCycle(0.1);
        largeCorrection.recordCurrentPose(
                new Pose3d(13.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        largeCorrection.frames.frame = observed(
                largeCorrection.time.clock().timestampSecondsAgo(0.05),
                detectorForFloorPoint(10.0, 0.0));
        largeCorrection.service.start(largeCorrection.time.clock());
        assertHistoryUnavailable(
                largeCorrection,
                PlanarPoseHistory.Lookup.UnavailableReason.INTERPOLATION_TRANSLATION_GAP
        );

        Fixture resetGap = new Fixture();
        resetGap.recordCurrentPose(Pose3d.zero());
        resetGap.time.nextCycle(0.1);
        resetGap.pose.trajectorySegment++;
        resetGap.recordCurrentPose(
                new Pose3d(1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        resetGap.frames.frame = observed(
                resetGap.time.clock().timestampSecondsAgo(0.05),
                detectorForFloorPoint(10.0, 0.0));
        resetGap.service.start(resetGap.time.clock());
        assertHistoryUnavailable(
                resetGap,
                PlanarPoseHistory.Lookup.UnavailableReason.DISCONTINUITY
        );
    }

    @Test
    public void invalidRaysAndOffBoxPointsFailClosedWithExactCounts() {
        Fixture rays = new Fixture();
        rays.recordCurrentPose(Pose3d.zero());
        rays.frames.frame = observed(rays.time.clock().nowTimestamp(), Arrays.asList(
                new AdaptiveCollectionVisionService.DetectorAngles(Double.NaN, 45.0),
                new AdaptiveCollectionVisionService.DetectorAngles(0.0, 0.0),
                new AdaptiveCollectionVisionService.DetectorAngles(0.0, -45.0)
        ));
        rays.service.start(rays.time.clock());
        AdaptiveCollectionVisionService.Decision noProjection = rays.service.decision();
        assertUnavailable(noProjection,
                AdaptiveCollectionVisionService.UnavailableReason
                        .NO_PROJECTABLE_FLOOR_INTERSECTIONS);
        assertEquals(3, noProjection.detectionCount());
        assertEquals(0, noProjection.projectablePointCount());

        Fixture offBox = new Fixture();
        offBox.recordCurrentPose(Pose3d.zero());
        offBox.frames.frame = observed(
                offBox.time.clock().nowTimestamp(),
                detectorForFloorPoint(100.0, 0.0));
        offBox.service.start(offBox.time.clock());
        AdaptiveCollectionVisionService.Decision outside = offBox.service.decision();
        assertUnavailable(outside,
                AdaptiveCollectionVisionService.UnavailableReason.NO_POINTS_IN_COLLECTION_BOX);
        assertEquals(1, outside.projectablePointCount());
        assertEquals(0, outside.inBoxPointCount());

        Fixture belowFloor = new Fixture();
        belowFloor.config.cameraMount = CameraMountConfig.of(0, 0, -10, 0, 0, 0);
        belowFloor.rebuildService();
        LoopTimestamp belowFloorTimestamp = belowFloor.recordCurrentPose(Pose3d.zero());
        belowFloor.frames.frame = observed(
                belowFloorTimestamp, detectorForFloorPoint(10.0, 0.0));
        belowFloor.service.start(belowFloor.time.clock());
        assertUnavailable(belowFloor.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason
                        .NO_PROJECTABLE_FLOOR_INTERSECTIONS);
    }

    @Test
    public void fieldAndCameraTransformsAreAppliedBeforeInclusiveBoxSelection() {
        Fixture fixture = new Fixture();
        fixture.config.minCollectionFieldXInches = 4.0;
        fixture.config.maxCollectionFieldXInches = 6.0;
        fixture.config.minCollectionFieldYInches = 15.0;
        fixture.config.maxCollectionFieldYInches = 17.0;
        fixture.config.bandWidthInches = 2.0;
        fixture.rebuildService();

        fixture.recordCurrentPose(new Pose3d(5.0, 6.0, 0.0,
                Math.PI / 2.0, 0.0, 0.0));
        fixture.frames.frame = observed(
                fixture.time.clock().nowTimestamp(),
                detectorForFloorPoint(10.0, 0.0));
        fixture.service.start(fixture.time.clock());

        AdaptiveCollectionVisionService.Decision decision = fixture.service.decision();
        assertTrue(decision.hasSelection());
        assertEquals(15.0, decision.selectedBandStartYInches(), EPSILON);
        assertEquals(17.0, decision.selectedBandEndYInches(), EPSILON);
    }

    @Test
    public void publicationIsAtomicAndStopClearsBeforeIdempotentOwnerClose() {
        Fixture fixture = new Fixture();
        fixture.recordCurrentPose(Pose3d.zero());
        fixture.frames.frame = observed(
                fixture.time.clock().nowTimestamp(),
                detectorForFloorPoint(10.0, 0.0));
        fixture.service.start(fixture.time.clock());
        AdaptiveCollectionVisionService.Decision prior = fixture.service.decision();

        RuntimeException failure = new RuntimeException("frame failed");
        fixture.frames.failure = failure;
        assertSame(failure, assertThrows(RuntimeException.class,
                () -> fixture.service.update(fixture.time.nextCycle(0.01))));
        assertSame(prior, fixture.service.decision());

        fixture.service.stop();
        assertUnavailable(fixture.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.NOT_OBSERVED);
        assertEquals(1, fixture.frames.closeCalls);
        assertEquals(0, fixture.poseLookups.resetCalls);
        fixture.service.stop();
        assertEquals(1, fixture.frames.closeCalls);
        assertThrows(IllegalStateException.class,
                fixture.service.decision()::selectedBandCenterYInches);

        Fixture stopBeforeStart = new Fixture();
        stopBeforeStart.service.stop();
        assertEquals(0, stopBeforeStart.frames.reads);
        assertEquals(1, stopBeforeStart.frames.closeCalls);
    }

    private static void assertInvalid(AdaptiveCollectionVisionService.Config config,
                                      TimeAwareSource<PlanarPoseHistory.Lookup> poseHistory,
                                      FakeFrameOwner frames) {
        assertThrows(RuntimeException.class,
                () -> new AdaptiveCollectionVisionService(poseHistory, config, frames));
    }

    private static void assertHistoryUnavailable(
            Fixture fixture,
            PlanarPoseHistory.Lookup.UnavailableReason reason) {
        AdaptiveCollectionVisionService.Decision decision = fixture.service.decision();
        assertUnavailable(decision,
                AdaptiveCollectionVisionService.UnavailableReason.POSE_HISTORY_UNAVAILABLE);
        assertTrue(decision.hasPoseLookup());
        assertSame(fixture.poseLookups.lastLookup, decision.poseLookup());
        assertFalse(decision.poseLookup().isAvailable());
        assertEquals(PlanarPoseHistory.Lookup.Kind.UNAVAILABLE,
                decision.poseLookup().kind());
        assertEquals(reason, decision.poseLookup().unavailableReason());
        assertSame(decision.frameTimestamp(), decision.poseLookup().timestamp());
        assertThrows(IllegalStateException.class,
                decision.poseLookup()::fieldToRobotPose);
    }

    private static void assertUnavailable(
            AdaptiveCollectionVisionService.Decision decision,
            AdaptiveCollectionVisionService.UnavailableReason reason) {
        assertFalse(decision.hasSelection());
        assertEquals(reason, decision.unavailableReason());
        assertThrows(IllegalStateException.class, decision::selectedBandStartYInches);
        assertThrows(IllegalStateException.class, decision::selectedBandCenterYInches);
    }

    private static AdaptiveCollectionVisionService.Config validConfig() {
        AdaptiveCollectionVisionService.Config config =
                AdaptiveCollectionVisionService.Config.defaults();
        config.cameraMount = CameraMountConfig.of(0, 0, 10, 0, 0, 0);
        config.maxFrameAgeSec = 0.25;
        config.minCollectionFieldXInches = 0.0;
        config.maxCollectionFieldXInches = 30.0;
        config.minCollectionFieldYInches = -20.0;
        config.maxCollectionFieldYInches = 20.0;
        config.bandWidthInches = 10.0;
        return config;
    }

    private static PlanarPoseHistory.Config validHistoryConfig() {
        PlanarPoseHistory.Config config = PlanarPoseHistory.Config.defaults();
        config.retentionSec = 1.0;
        config.maxSamples = 32;
        config.maxInterpolationGapSec = 0.2;
        config.maxInterpolationTranslationInches = 12.0;
        config.maxInterpolationYawRad = Math.PI / 2.0;
        return config;
    }

    private static AdaptiveCollectionVisionService.CopiedFrame observed(
            LoopTimestamp timestamp,
            AdaptiveCollectionVisionService.DetectorAngles angle) {
        return observed(timestamp, Collections.singletonList(angle));
    }

    private static AdaptiveCollectionVisionService.CopiedFrame observed(
            LoopTimestamp timestamp,
            List<AdaptiveCollectionVisionService.DetectorAngles> angles) {
        return AdaptiveCollectionVisionService.CopiedFrame.observed(timestamp, angles);
    }

    /** Returns detector angles whose identity-camera ray reaches (x,y,0) from (0,0,10). */
    private static AdaptiveCollectionVisionService.DetectorAngles detectorForFloorPoint(
            double xInches,
            double yInches) {
        return new AdaptiveCollectionVisionService.DetectorAngles(
                Math.toDegrees(Math.atan(-yInches / xInches)),
                Math.toDegrees(Math.atan(10.0 / xInches))
        );
    }

    private static PoseEstimate pose(Pose3d fieldToRobotPose, LoopTimestamp timestamp) {
        return new PoseEstimate(fieldToRobotPose, true, 1.0, timestamp);
    }

    private static LLResultTypes.DetectorResult detectorResult(double txDeg, double tyDeg) {
        try {
            Class<?> unsafeClass = Class.forName("sun.misc.Unsafe");
            Field singleton = unsafeClass.getDeclaredField("theUnsafe");
            singleton.setAccessible(true);
            Object unsafe = singleton.get(null);
            Method allocate = unsafeClass.getMethod("allocateInstance", Class.class);
            LLResultTypes.DetectorResult detector =
                    (LLResultTypes.DetectorResult) allocate.invoke(
                            unsafe, LLResultTypes.DetectorResult.class);
            setDoubleField(detector, "targetXDegreesNoCrosshair", txDeg);
            setDoubleField(detector, "targetYDegreesNoCrosshair", tyDeg);
            return detector;
        } catch (ReflectiveOperationException e) {
            throw new AssertionError("Could not create test-only SDK DetectorResult", e);
        }
    }

    private static void setDoubleField(Object owner, String fieldName, double value) {
        try {
            Field field = owner.getClass().getDeclaredField(fieldName);
            field.setAccessible(true);
            field.setDouble(owner, value);
        } catch (ReflectiveOperationException e) {
            throw new AssertionError("Could not set test SDK field " + fieldName, e);
        }
    }

    private static final class Fixture {
        final ManualLoopClock time;
        final FakePoseEstimator pose = new FakePoseEstimator();
        final PlanarPoseHistory poseHistory;
        final CapturingPoseLookup poseLookups;
        final FakeFrameOwner frames = new FakeFrameOwner();
        final AdaptiveCollectionVisionService.Config config = validConfig();
        AdaptiveCollectionVisionService service;

        Fixture() {
            this(0.0);
        }

        Fixture(double initialTimeSec) {
            time = new ManualLoopClock(initialTimeSec);
            poseHistory = new PlanarPoseHistory(pose, validHistoryConfig());
            poseLookups = new CapturingPoseLookup(poseHistory.lookupSource());
            rebuildService();
        }

        void rebuildService() {
            service = new AdaptiveCollectionVisionService(poseLookups, config, frames);
        }

        LoopTimestamp recordCurrentPose(Pose3d fieldToRobotPose) {
            pose.publishCurrent(time.clock(), fieldToRobotPose);
            poseHistory.recordCurrent(time.clock());
            return pose.estimate.timestamp;
        }

        void nextObservation(AdaptiveCollectionVisionService.DetectorAngles angle) {
            nextObservation(Collections.singletonList(angle));
        }

        void nextObservation(List<AdaptiveCollectionVisionService.DetectorAngles> angles) {
            time.nextCycle(0.01);
            recordCurrentPose(Pose3d.zero());
            frames.frame = observed(time.clock().nowTimestamp(), angles);
            service.update(time.clock());
        }
    }

    private static final class FakePoseEstimator implements PoseTrajectoryEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        long trajectorySegment;
        int updateCalls;

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }

        @Override
        public long trajectorySegmentId() {
            return trajectorySegment;
        }

        void publishCurrent(LoopClock clock, Pose3d fieldToRobotPose) {
            estimate = pose(fieldToRobotPose, clock.nowTimestamp());
        }
    }

    private static final class CapturingPoseLookup
            implements TimeAwareSource<PlanarPoseHistory.Lookup> {
        private final TimeAwareSource<PlanarPoseHistory.Lookup> delegate;
        PlanarPoseHistory.Lookup lastLookup;
        PlanarPoseHistory.Lookup forcedLookup;
        int queries;
        int resetCalls;

        CapturingPoseLookup(TimeAwareSource<PlanarPoseHistory.Lookup> delegate) {
            this.delegate = delegate;
        }

        @Override
        public PlanarPoseHistory.Lookup getAt(LoopClock clock, LoopTimestamp timestamp) {
            queries++;
            lastLookup = forcedLookup != null
                    ? forcedLookup
                    : delegate.getAt(clock, timestamp);
            return lastLookup;
        }

        @Override
        public void reset() {
            resetCalls++;
            delegate.reset();
        }
    }

    private static final class UnusedPoseLookup
            implements TimeAwareSource<PlanarPoseHistory.Lookup> {
        @Override
        public PlanarPoseHistory.Lookup getAt(LoopClock clock, LoopTimestamp timestamp) {
            throw new AssertionError("invalid construction must not query pose history");
        }
    }

    private static final class FakeFrameOwner
            implements AdaptiveCollectionVisionService.FrameOwner {
        AdaptiveCollectionVisionService.CopiedFrame frame =
                AdaptiveCollectionVisionService.CopiedFrame.unavailable();
        RuntimeException failure;
        int reads;
        int closeCalls;
        boolean closeAttempted;

        @Override
        public AdaptiveCollectionVisionService.CopiedFrame confirmedFrame(LoopClock clock) {
            reads++;
            if (failure != null) {
                throw failure;
            }
            return frame;
        }

        @Override
        public void close() {
            if (closeAttempted) {
                return;
            }
            closeAttempted = true;
            closeCalls++;
        }
    }

    private static final class NoLookupHardwareMap extends HardwareMap {
        int lookupCalls;

        NoLookupHardwareMap() {
            super(null, null);
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCalls++;
            throw new AssertionError("invalid config must fail before FTC hardware lookup");
        }
    }
}
