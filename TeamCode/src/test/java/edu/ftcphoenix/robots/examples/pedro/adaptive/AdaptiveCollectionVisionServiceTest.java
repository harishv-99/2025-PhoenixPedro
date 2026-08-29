package edu.ftcphoenix.robots.examples.pedro.adaptive;

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

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.ftc.vision.FtcLimelightVisionLane;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
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
                        AbsolutePoseEstimator.class,
                        AdaptiveCollectionVisionService.Config.class
                },
                publicConstructors[0].getParameterTypes()
        );

        for (Field field : AdaptiveCollectionVisionService.Config.class.getFields()) {
            assertFalse("Config must not pretend to prove stationarity: " + field.getName(),
                    field.getName().toLowerCase().contains("stationary"));
        }

        FakePoseEstimator pose = new FakePoseEstimator();
        FakeFrameOwner frames = new FakeFrameOwner();

        AdaptiveCollectionVisionService.Config config = validConfig();
        config.limelight = null;
        assertInvalid(config, pose, frames);

        config = validConfig();
        config.cameraMount = null;
        assertInvalid(config, pose, frames);

        for (double invalid : new double[]{
                0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY
        }) {
            config = validConfig();
            config.maxFrameAgeSec = invalid;
            assertInvalid(config, pose, frames);

            config = validConfig();
            config.bandWidthInches = invalid;
            assertInvalid(config, pose, frames);
        }

        config = validConfig();
        config.minCollectionFieldXInches = 4.0;
        config.maxCollectionFieldXInches = 3.0;
        assertInvalid(config, pose, frames);

        config = validConfig();
        config.minCollectionFieldYInches = 4.0;
        config.maxCollectionFieldYInches = 4.0;
        assertInvalid(config, pose, frames);

        config = validConfig();
        config.maxCollectionFieldYInches = Double.NaN;
        assertInvalid(config, pose, frames);

        config = validConfig();
        config.bandWidthInches = 41.0;
        assertInvalid(config, pose, frames);

        assertThrows(NullPointerException.class,
                () -> new AdaptiveCollectionVisionService(pose, validConfig(), null));
        assertThrows(NullPointerException.class,
                () -> new AdaptiveCollectionVisionService(null, validConfig(), frames));

        NoLookupHardwareMap hardwareMap = new NoLookupHardwareMap();
        config = validConfig();
        config.limelight.pipelineIndex = -1;
        AdaptiveCollectionVisionService.Config invalidNestedConfig = config;
        assertThrows(IllegalArgumentException.class,
                () -> new AdaptiveCollectionVisionService(
                        hardwareMap, pose, invalidNestedConfig));
        assertEquals(0, hardwareMap.lookupCalls);
    }

    @Test
    public void configurationIsSnapshottedBeforeLaterAuthoringMutation() {
        ManualLoopClock time = new ManualLoopClock();
        FakePoseEstimator pose = new FakePoseEstimator();
        FakeFrameOwner frames = new FakeFrameOwner();
        AdaptiveCollectionVisionService.Config config = validConfig();
        AdaptiveCollectionVisionService service =
                new AdaptiveCollectionVisionService(pose, config, frames);

        config.cameraMount = CameraMountConfig.identity();
        config.maxFrameAgeSec = 1.0e-9;
        config.minCollectionFieldXInches = 50.0;
        config.maxCollectionFieldXInches = 60.0;
        config.minCollectionFieldYInches = 50.0;
        config.maxCollectionFieldYInches = 60.0;
        config.bandWidthInches = 1.0;
        config.limelight.hardwareName = "mutated";

        pose.publishCurrent(time.clock(), Pose3d.zero());
        LoopTimestamp exposure = time.clock().timestampSecondsAgo(0.1);
        frames.frame = observed(exposure, detectorForFloorPoint(10.0, 0.0));
        service.start(time.clock());

        assertTrue(service.decision().hasSelection());
        assertSame(exposure, service.decision().frameTimestamp());
        assertEquals(0.1, service.decision().frameAgeSec(), EPSILON);
        assertEquals(-5.0, service.decision().selectedBandCenterYInches(), EPSILON);
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
        fixture.publishCurrentPose(Pose3d.zero());
        fixture.frames.frame = observed(
                fixture.time.clock().nowTimestamp(), Collections.emptyList());
        fixture.service.start(fixture.time.clock());
        assertUnavailable(fixture.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.ZERO_DETECTIONS);
        assertEquals(1, fixture.frames.reads);
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

        Fixture stale = new Fixture();
        stale.publishCurrentPose(Pose3d.zero());
        stale.frames.frame = observed(
                stale.time.clock().timestampSecondsAgo(0.251),
                detectorForFloorPoint(10.0, 0.0));
        stale.service.start(stale.time.clock());
        assertUnavailable(stale.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.FRAME_STALE);
        assertEquals(0.251, stale.service.decision().frameAgeSec(), EPSILON);

        Fixture inclusiveBoundary = new Fixture();
        inclusiveBoundary.publishCurrentPose(Pose3d.zero());
        inclusiveBoundary.frames.frame = observed(
                inclusiveBoundary.time.clock().timestampSecondsAgo(0.25),
                detectorForFloorPoint(10.0, 0.0));
        inclusiveBoundary.service.start(inclusiveBoundary.time.clock());
        assertTrue(inclusiveBoundary.service.decision().hasSelection());

        Fixture reset = new Fixture();
        LoopTimestamp priorEpoch = reset.time.clock().nowTimestamp();
        reset.time.clock().reset(0.0);
        reset.publishCurrentPose(Pose3d.zero());
        reset.frames.frame = observed(priorEpoch, detectorForFloorPoint(10.0, 0.0));
        reset.service.start(reset.time.clock());
        assertUnavailable(reset.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.FRAME_RESET_OR_FUTURE);

        Fixture future = new Fixture(1.0);
        LoopTimestamp later = future.time.clock().nowTimestamp();
        future.time.clock().update(0.5);
        future.publishCurrentPose(Pose3d.zero());
        future.frames.frame = observed(later, detectorForFloorPoint(10.0, 0.0));
        future.service.start(future.time.clock());
        assertUnavailable(future.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.FRAME_RESET_OR_FUTURE);

        Fixture wrongClock = new Fixture();
        wrongClock.publishCurrentPose(Pose3d.zero());
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
    public void poseMustBeAvailableFiniteAndCurrentButServiceNeverAdvancesIt() {
        Fixture unavailable = new Fixture();
        unavailable.frames.frame = observed(
                unavailable.time.clock().nowTimestamp(),
                detectorForFloorPoint(10.0, 0.0));
        unavailable.pose.estimate = PoseEstimate.noPose(
                unavailable.time.clock().nowTimestamp());
        unavailable.service.start(unavailable.time.clock());
        assertUnavailable(unavailable.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.POSE_UNAVAILABLE);
        assertEquals(0, unavailable.pose.updateCalls);

        Fixture nonFinite = new Fixture();
        nonFinite.frames.frame = observed(
                nonFinite.time.clock().nowTimestamp(),
                detectorForFloorPoint(10.0, 0.0));
        nonFinite.pose.estimate = pose(
                new Pose3d(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0),
                nonFinite.time.clock().nowTimestamp());
        nonFinite.service.start(nonFinite.time.clock());
        assertUnavailable(nonFinite.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.POSE_UNAVAILABLE);

        Fixture stale = new Fixture();
        stale.frames.frame = observed(
                stale.time.clock().nowTimestamp(),
                detectorForFloorPoint(10.0, 0.0));
        stale.pose.estimate = pose(Pose3d.zero(),
                stale.time.clock().timestampSecondsAgo(0.001));
        stale.service.start(stale.time.clock());
        assertUnavailable(stale.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.POSE_NOT_CURRENT);

        Fixture reset = new Fixture();
        LoopTimestamp priorEpoch = reset.time.clock().nowTimestamp();
        reset.time.clock().reset(0.0);
        reset.frames.frame = observed(
                reset.time.clock().nowTimestamp(),
                detectorForFloorPoint(10.0, 0.0));
        reset.pose.estimate = pose(Pose3d.zero(), priorEpoch);
        reset.service.start(reset.time.clock());
        assertUnavailable(reset.service.decision(),
                AdaptiveCollectionVisionService.UnavailableReason.POSE_NOT_CURRENT);
    }

    @Test
    public void invalidRaysAndOffBoxPointsFailClosedWithExactCounts() {
        Fixture rays = new Fixture();
        rays.publishCurrentPose(Pose3d.zero());
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
        offBox.publishCurrentPose(Pose3d.zero());
        offBox.frames.frame = observed(
                offBox.time.clock().nowTimestamp(),
                detectorForFloorPoint(100.0, 0.0));
        offBox.service.start(offBox.time.clock());
        AdaptiveCollectionVisionService.Decision outside = offBox.service.decision();
        assertUnavailable(outside,
                AdaptiveCollectionVisionService.UnavailableReason.NO_POINTS_IN_COLLECTION_BOX);
        assertEquals(1, outside.projectablePointCount());
        assertEquals(0, outside.inBoxPointCount());

        AdaptiveCollectionVisionService.Config belowFloorConfig = validConfig();
        belowFloorConfig.cameraMount = CameraMountConfig.of(0, 0, -10, 0, 0, 0);
        ManualLoopClock time = new ManualLoopClock();
        FakePoseEstimator pose = new FakePoseEstimator();
        pose.publishCurrent(time.clock(), Pose3d.zero());
        FakeFrameOwner frames = new FakeFrameOwner();
        frames.frame = observed(
                time.clock().nowTimestamp(), detectorForFloorPoint(10.0, 0.0));
        AdaptiveCollectionVisionService belowFloor =
                new AdaptiveCollectionVisionService(pose, belowFloorConfig, frames);
        belowFloor.start(time.clock());
        assertUnavailable(belowFloor.decision(),
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

        fixture.publishCurrentPose(new Pose3d(5.0, 6.0, 0.0,
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
        fixture.publishCurrentPose(Pose3d.zero());
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
                                      FakePoseEstimator pose,
                                      FakeFrameOwner frames) {
        assertThrows(RuntimeException.class,
                () -> new AdaptiveCollectionVisionService(pose, config, frames));
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
        final FakeFrameOwner frames = new FakeFrameOwner();
        final AdaptiveCollectionVisionService.Config config = validConfig();
        AdaptiveCollectionVisionService service;

        Fixture() {
            this(0.0);
        }

        Fixture(double initialTimeSec) {
            time = new ManualLoopClock(initialTimeSec);
            rebuildService();
        }

        void rebuildService() {
            service = new AdaptiveCollectionVisionService(pose, config, frames);
        }

        void publishCurrentPose(Pose3d fieldToRobotPose) {
            pose.publishCurrent(time.clock(), fieldToRobotPose);
        }

        void nextObservation(AdaptiveCollectionVisionService.DetectorAngles angle) {
            nextObservation(Collections.singletonList(angle));
        }

        void nextObservation(List<AdaptiveCollectionVisionService.DetectorAngles> angles) {
            time.nextCycle(0.01);
            publishCurrentPose(Pose3d.zero());
            frames.frame = observed(time.clock().nowTimestamp(), angles);
            service.update(time.clock());
        }
    }

    private static final class FakePoseEstimator implements AbsolutePoseEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        int updateCalls;

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }

        void publishCurrent(LoopClock clock, Pose3d fieldToRobotPose) {
            estimate = pose(fieldToRobotPose, clock.nowTimestamp());
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
