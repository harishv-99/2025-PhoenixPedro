package edu.ftcphoenix.fw.ftc.vision;

import android.graphics.Canvas;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.junit.Test;

import java.util.ArrayList;
import java.util.IdentityHashMap;

import edu.ftcphoenix.fw.core.geometry.Mat3;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.ftc.FtcFrames;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused ownership, processor-state, and result-generation coverage for webcam vision. */
public final class FtcWebcamVisionPortalLaneTest {

    @Test
    public void cameraMountConversionMatchesFtcRobotAndOpticalCameraFrames() {
        FtcWebcamAprilTagSupport.SdkCameraPose identity =
                FtcWebcamAprilTagSupport.toSdkCameraPose(CameraMountConfig.identity());
        assertEquals(0.0, identity.rightInches, 1e-9);
        assertEquals(0.0, identity.forwardInches, 1e-9);
        assertEquals(0.0, identity.upInches, 1e-9);
        assertEquals(0.0, identity.yawRad, 1e-6);
        assertEquals(-Math.PI / 2.0, identity.pitchRad, 1e-6);
        assertEquals(0.0, identity.rollRad, 1e-6);

        FtcWebcamAprilTagSupport.SdkCameraPose sample =
                FtcWebcamAprilTagSupport.toSdkCameraPose(
                        CameraMountConfig.ofDegrees(7.0, 5.0, 12.0, 0.0, 0.0, 0.0));
        assertEquals(-5.0, sample.rightInches, 1e-9);
        assertEquals(7.0, sample.forwardInches, 1e-9);
        assertEquals(12.0, sample.upInches, 1e-9);
        assertEquals(0.0, sample.yawRad, 1e-6);
        assertEquals(-Math.PI / 2.0, sample.pitchRad, 1e-6);
        assertEquals(0.0, sample.rollRad, 1e-6);

        CameraMountConfig nontrivial = CameraMountConfig.ofDegrees(
                4.0, -3.0, 8.0, 30.0, -15.0, 20.0);
        FtcWebcamAprilTagSupport.SdkCameraPose converted =
                FtcWebcamAprilTagSupport.toSdkCameraPose(nontrivial);
        Mat3 ftcRobotFromPhoenix = new Mat3(
                0, -1, 0,
                1, 0, 0,
                0, 0, 1
        );
        Mat3 phoenixCameraFromFtcOptical = new Mat3(
                0, 0, 1,
                -1, 0, 0,
                0, -1, 0
        );
        Mat3 expected = ftcRobotFromPhoenix
                .mul(nontrivial.robotToCameraPose().rotation())
                .mul(phoenixCameraFromFtcOptical);
        Mat3 reconstructed = Mat3.fromYawPitchRoll(converted.yawRad, 0.0, 0.0)
                .mul(Mat3.fromYawPitchRoll(0.0, 0.0, converted.pitchRad))
                .mul(Mat3.fromYawPitchRoll(converted.rollRad, 0.0, 0.0));
        assertMatrixEquals(expected, reconstructed, 1e-5);
    }

    @Test
    public void invalidConfigurationAndProcessorSetsNeverOpenThePortal() {
        RecordingFactory factory = new RecordingFactory(new FakePortal());
        FakeProcessor processor = new FakeProcessor("one", false);

        FtcWebcamVisionPortalLane.Config cfg = validConfig();
        cfg.webcamName = "  ";
        FtcWebcamVisionPortalLane.Config blankName = cfg;
        expectFailure(IllegalArgumentException.class,
                () -> testOwner(blankName, factory, processor));

        cfg = validConfig();
        cfg.cameraResolution = null;
        FtcWebcamVisionPortalLane.Config nullResolution = cfg;
        expectFailure(NullPointerException.class,
                () -> testOwner(nullResolution, factory, processor));

        cfg = validConfig();
        cfg.cameraResolution = new Size(0, 480);
        FtcWebcamVisionPortalLane.Config invalidWidth = cfg;
        expectFailure(IllegalArgumentException.class,
                () -> new FtcWebcamVisionPortalLane(
                        invalidWidth,
                        factory,
                        new MutableNanoClock(100),
                        INVALID_WIDTH_READER,
                        processor));

        expectFailure(NullPointerException.class,
                () -> testOwner(validConfig(), factory, (VisionProcessor[]) null));
        expectFailure(IllegalArgumentException.class,
                () -> testOwner(validConfig(), factory, new VisionProcessor[0]));
        expectFailure(NullPointerException.class,
                () -> testOwner(validConfig(), factory, processor, null));
        expectFailure(IllegalArgumentException.class,
                () -> testOwner(validConfig(), factory, processor, processor));

        FakeProcessor equalOne = new FakeProcessor("equal", true);
        FakeProcessor equalTwo = new FakeProcessor("equal", true);
        expectFailure(IllegalArgumentException.class,
                () -> testOwner(validConfig(), factory, equalOne, equalTwo));

        assertEquals(0, factory.openCount);
    }

    @Test
    public void readinessSeparatesStreamStateFromProcessorState() {
        FakePortal portal = new FakePortal();
        FakeProcessor processor = new FakeProcessor("main", false);
        FtcWebcamVisionPortalLane lane = open(portal, processor);

        portal.cameraState = VisionPortal.CameraState.OPENING_CAMERA_DEVICE;
        assertNotReady(lane.readiness(), "not streaming");
        portal.cameraState = VisionPortal.CameraState.ERROR;
        assertNotReady(lane.readiness(), "ERROR");
        portal.cameraState = VisionPortal.CameraState.CAMERA_DEVICE_CLOSED;
        assertNotReady(lane.readiness(), "closed");
        portal.cameraState = VisionPortal.CameraState.STREAMING;
        assertTrue(lane.readiness().isReady());
        assertTrue(lane.processorReadiness(processor).isReady());

        portal.enabled.put(processor, Boolean.FALSE);
        assertNotReady(lane.processorReadiness(processor), "disabled");

        int readsBefore = portal.enabledReads;
        expectFailure(IllegalArgumentException.class,
                () -> lane.processorReadiness(new FakeProcessor("foreign", false)));
        assertEquals(readsBefore, portal.enabledReads);

        portal.enabledFailure = new IllegalStateException("state unavailable");
        assertNotReady(lane.processorReadiness(processor), "state unavailable");
    }

    @Test
    public void processorAndStreamingTransitionsAreIdempotentAndInvalidateData() {
        FakePortal portal = new FakePortal();
        FakeProcessor first = new FakeProcessor("first", false);
        FakeProcessor second = new FakeProcessor("second", false);
        MutableNanoClock nanos = new MutableNanoClock(100);
        FtcWebcamVisionPortalLane lane = new FtcWebcamVisionPortalLane(
                validConfig(), new RecordingFactory(portal), nanos,
                VALID_RESOLUTION_READER, first, second);

        assertEquals(0L, lane.processorDataGeneration(first));
        lane.setProcessorEnabled(first, true);
        assertEquals(0, portal.enableWrites);

        lane.setProcessorEnabled(first, false);
        assertEquals(1, portal.enableWrites);
        assertEquals(1L, lane.processorDataGeneration(first));
        lane.setProcessorEnabled(first, false);
        assertEquals(1, portal.enableWrites);

        nanos.now = 200;
        lane.setProcessorEnabled(first, true);
        assertEquals(2, portal.enableWrites);
        assertEquals(2L, lane.processorDataGeneration(first));
        assertEquals(200L, lane.acceptProcessorFramesAfterNanos(first));

        lane.stopStreaming();
        lane.stopStreaming();
        assertEquals(1, portal.stopStreamingCalls);
        assertNotReady(lane.readiness(), "stopped");
        assertEquals(3L, lane.processorDataGeneration(first));
        assertEquals(1L, lane.processorDataGeneration(second));

        nanos.now = 300;
        lane.resumeStreaming();
        lane.resumeStreaming();
        assertEquals(1, portal.resumeStreamingCalls);
        assertEquals(4L, lane.processorDataGeneration(first));
        assertEquals(2L, lane.processorDataGeneration(second));
        assertEquals(300L, lane.acceptProcessorFramesAfterNanos(second));
    }

    @Test
    public void specializedOwnerControlsBuiltInAprilTagProcessorWithoutExposingIt() {
        FakePortal portal = new FakePortal();
        RecordingFactory factory = new RecordingFactory(portal);
        FakeProcessor customProcessor = new FakeProcessor("custom", false);
        FakeAprilTagProcessor aprilTagProcessor = new FakeAprilTagProcessor();
        MutableNanoClock nanos = new MutableNanoClock(100);
        FtcWebcamAprilTagVisionLane lane = new FtcWebcamAprilTagVisionLane(
                FtcWebcamAprilTagVisionLane.Config.defaults(),
                aprilTagProcessor,
                factory,
                nanos,
                VALID_RESOLUTION_READER,
                customProcessor
        );

        assertEquals(1, factory.openCount);
        assertEquals(2, portal.openProcessors.length);
        VisionProcessor builtInAprilTagProcessor = portal.openProcessors[0];
        assertSame(aprilTagProcessor, builtInAprilTagProcessor);
        assertSame(customProcessor, portal.openProcessors[1]);
        assertTrue(lane.isAprilTagProcessorEnabled());
        assertTrue(lane.readiness(new ManualLoopClock().clock()).isReady());

        lane.setAprilTagProcessorEnabled(false);
        lane.setAprilTagProcessorEnabled(false);
        assertEquals(1, portal.enableWrites);
        assertFalse(lane.isAprilTagProcessorEnabled());
        assertNotReady(lane.readiness(new ManualLoopClock().clock()), "disabled");
        assertEquals(1L, lane.processorDataGeneration(builtInAprilTagProcessor));

        nanos.now = 200;
        lane.setAprilTagProcessorEnabled(true);
        lane.setAprilTagProcessorEnabled(true);
        assertEquals(2, portal.enableWrites);
        assertTrue(lane.isAprilTagProcessorEnabled());
        assertTrue(lane.readiness(new ManualLoopClock().clock()).isReady());
        assertEquals(2L, lane.processorDataGeneration(builtInAprilTagProcessor));
        assertEquals(200L, lane.acceptProcessorFramesAfterNanos(builtInAprilTagProcessor));

        lane.close();
        assertEquals(1, portal.closeCalls);
    }

    @Test
    public void postOpenVerificationFailureClosesAndPreservesCleanupFailure() {
        FakePortal portal = new FakePortal();
        RuntimeException verification = new IllegalStateException("not registered");
        RuntimeException cleanup = new IllegalArgumentException("close failed");
        portal.enabledFailure = verification;
        portal.closeFailure = cleanup;
        RecordingFactory factory = new RecordingFactory(portal);

        try {
            new FtcWebcamVisionPortalLane(
                    validConfig(), factory, new MutableNanoClock(100),
                    VALID_RESOLUTION_READER, new FakeProcessor("main", false));
            fail("Expected verification failure");
        } catch (RuntimeException actual) {
            assertSame(verification, actual);
            assertEquals(1, actual.getSuppressed().length);
            assertSame(cleanup, actual.getSuppressed()[0]);
        }
        assertEquals(1, factory.openCount);
        assertEquals(1, portal.closeCalls);
    }

    @Test
    public void closeFailureIsVisibleExactlyOnceAndPostCloseUseFails() {
        FakePortal portal = new FakePortal();
        FakeProcessor processor = new FakeProcessor("main", false);
        FtcWebcamVisionPortalLane lane = open(portal, processor);
        RuntimeException closeFailure = new IllegalStateException("USB close failed");
        portal.closeFailure = closeFailure;

        try {
            lane.close();
            fail("Expected close failure");
        } catch (RuntimeException actual) {
            assertSame(closeFailure, actual);
        }
        assertTrue(lane.isCloseAttempted());
        assertFalse(lane.isClosed());
        assertTrue(lane.closeFailureReason().contains("USB close failed"));
        lane.close();
        assertEquals(1, portal.closeCalls);
        assertNotReady(lane.readiness(), "close failed");
        expectFailure(IllegalStateException.class,
                () -> lane.setProcessorEnabled(processor, true));
        expectFailureContaining(
                IllegalStateException.class,
                "do not open a replacement",
                lane::resumeStreaming
        );
    }

    @Test
    public void aprilTagAdapterIsCycleStableAndRejectsOldBoundaryAndFutureFrames() {
        FakePortal portal = new FakePortal();
        FakeAprilTagProcessor processor = new FakeAprilTagProcessor();
        MutableNanoClock nanos = new MutableNanoClock(100);
        FtcWebcamVisionPortalLane owner = new FtcWebcamVisionPortalLane(
                validConfig(), new RecordingFactory(portal), nanos,
                VALID_RESOLUTION_READER, processor);
        FtcWebcamAprilTagSupport.PortalAprilTagSensor sensor =
                new FtcWebcamAprilTagSupport.PortalAprilTagSensor(owner, processor);
        ManualLoopClock manual = new ManualLoopClock();
        LoopClock clock = manual.clock();

        nanos.now = 200;
        processor.setDetection(100);
        assertTrue(sensor.get(clock).observations.isEmpty());

        processor.setDetection(150);
        manual.nextCycle(0.02);
        AprilTagDetections fresh = sensor.get(clock);
        assertEquals(1, fresh.observations.size());
        assertEquals(7, fresh.observations.get(0).id);
        Pose3d expected = FtcFrames.toPhoenixFromFtcDetectionFrame(
                new Pose3d(1.0, 2.0, 3.0, 0.1, 0.3, 0.2));
        Pose3d actual = fresh.observations.get(0).cameraToTagPose;
        assertEquals(expected.xInches, actual.xInches, 1e-9);
        assertEquals(expected.yInches, actual.yInches, 1e-9);
        assertEquals(expected.zInches, actual.zInches, 1e-9);
        assertEquals(expected.yawRad, actual.yawRad, 1e-9);
        assertEquals(expected.pitchRad, actual.pitchRad, 1e-9);
        assertEquals(expected.rollRad, actual.rollRad, 1e-9);
        assertSame(fresh, sensor.get(clock));

        owner.setProcessorEnabled(processor, false);
        assertTrue(sensor.get(clock).observations.isEmpty());

        nanos.now = 210;
        owner.setProcessorEnabled(processor, true);
        processor.setDetection(150);
        assertTrue(sensor.get(clock).observations.isEmpty());

        nanos.now = 220;
        processor.setDetection(211);
        assertTrue("same-cycle sampling must remain stable",
                sensor.get(clock).observations.isEmpty());
        manual.nextCycle(0.02);
        AprilTagDetections afterGenerationChange = sensor.get(clock);
        assertEquals(1, afterGenerationChange.observations.size());
        assertNotSame(fresh.frameTimestamp(), afterGenerationChange.frameTimestamp());

        nanos.now = 250;
        processor.setDetection(300);
        manual.nextCycle(0.02);
        assertTrue("future frame timestamps are not trustworthy",
                sensor.get(clock).observations.isEmpty());

        owner.close();
        owner.close();
        assertEquals(1, portal.closeCalls);
    }

    @Test
    public void aprilTagAdapterUsesRawPoseAndConvertsOptionalRobotPose() {
        FakePortal portal = new FakePortal();
        FakeAprilTagProcessor processor = new FakeAprilTagProcessor();
        MutableNanoClock nanos = new MutableNanoClock(100);
        FtcWebcamVisionPortalLane owner = new FtcWebcamVisionPortalLane(
                validConfig(), new RecordingFactory(portal), nanos,
                VALID_RESOLUTION_READER, processor);
        FtcWebcamAprilTagSupport.PortalAprilTagSensor sensor =
                new FtcWebcamAprilTagSupport.PortalAprilTagSensor(owner, processor);
        nanos.now = 200;

        Mat3 rawRotation = Mat3.fromYawPitchRoll(0.2, -0.1, 0.3);
        AprilTagPoseRaw rawPose = new AprilTagPoseRaw(
                4.0,
                -2.0,
                10.0,
                generalMatrix(rawRotation)
        );
        Pose3D robotPose = new Pose3D(
                new Position(DistanceUnit.INCH, 20.0, 30.0, 4.0, 0),
                new YawPitchRollAngles(AngleUnit.RADIANS, 0.4, -0.2, 0.1, 0)
        );
        processor.setRawDetection(150, rawPose, robotPose);

        ManualLoopClock manual = new ManualLoopClock();
        AprilTagDetections detections = sensor.get(manual.clock());
        assertEquals(1, detections.observations.size());
        AprilTagObservation observation = detections.observations.get(0);
        assertEquals(10.0, observation.cameraToTagPose.xInches, 1e-9);
        assertEquals(-4.0, observation.cameraToTagPose.yInches, 1e-9);
        assertEquals(2.0, observation.cameraToTagPose.zInches, 1e-9);
        assertMatrixEquals(
                FtcFrames.phoenixFromAprilTagRawCameraFrame().mul(rawRotation),
                observation.cameraToTagPose.rotation(),
                1e-5
        );
        assertTrue(observation.hasFieldToRobotPose());
        assertEquals(20.0, observation.fieldToRobotPose.xInches, 1e-9);
        assertEquals(30.0, observation.fieldToRobotPose.yInches, 1e-9);
        assertEquals(4.0, observation.fieldToRobotPose.zInches, 1e-9);
        assertEquals(0.4, observation.fieldToRobotPose.yawRad, 1e-9);
        assertEquals(-0.2, observation.fieldToRobotPose.pitchRad, 1e-9);
        assertEquals(0.1, observation.fieldToRobotPose.rollRad, 1e-9);

        owner.close();
        assertEquals(1, portal.closeCalls);
    }

    @Test
    public void aprilTagAdapterRetainsOneTimestampForRepeatedFrameAndSourceReset() {
        FakePortal portal = new FakePortal();
        FakeAprilTagProcessor processor = new FakeAprilTagProcessor();
        MutableNanoClock nanos = new MutableNanoClock(1_000_000_000L);
        FtcWebcamVisionPortalLane owner = new FtcWebcamVisionPortalLane(
                validConfig(), new RecordingFactory(portal), nanos,
                VALID_RESOLUTION_READER, processor);
        FtcWebcamAprilTagSupport.PortalAprilTagSensor sensor =
                new FtcWebcamAprilTagSupport.PortalAprilTagSensor(owner, processor);
        ManualLoopClock time = new ManualLoopClock(10.0);

        AprilTagDetections emptySdkList = sensor.get(time.clock());
        assertTrue(emptySdkList.observations.isEmpty());
        assertFalse(emptySdkList.frameTimestamp().isAvailable());

        time.nextCycle(0.0);
        nanos.now = 2_000_000_000L;
        processor.setDetection(1_900_000_000L);

        AprilTagDetections first = sensor.get(time.clock());
        LoopTimestamp firstTimestamp = first.frameTimestamp();
        assertEquals(0.1, first.frameAgeSec(time.clock()), 1e-9);
        assertSame(firstTimestamp, first.observations.get(0).frameTimestamp());

        // A source reset invalidates only the per-cycle cache. The retained SDK frame identity
        // must not be translated into a new timestamp.
        sensor.reset();
        AprilTagDetections afterSourceReset = sensor.get(time.clock());
        assertSame(firstTimestamp, afterSourceReset.frameTimestamp());

        // Advance the two clocks by deliberately different amounts. Re-translating the changing
        // SDK age here would move the capture time; retaining the anchor keeps it exact.
        time.nextCycle(0.4);
        nanos.now = 2_200_000_000L;
        AprilTagDetections repeated = sensor.get(time.clock());
        assertSame(firstTimestamp, repeated.frameTimestamp());
        assertEquals(0.5, repeated.frameAgeSec(time.clock()), 1e-9);

        processor.setDetection(2_100_000_000L);
        time.nextCycle(0.0);
        AprilTagDetections newer = sensor.get(time.clock());
        assertNotSame(firstTimestamp, newer.frameTimestamp());
        assertEquals(0.1, newer.frameAgeSec(time.clock()), 1e-9);
        assertEquals(0.4, newer.frameTimestamp().secondsSince(firstTimestamp), 1e-9);
        owner.close();
    }

    @Test
    public void aprilTagAdapterBlocksRetainedFrameAcrossClockResetUntilNewFrame() {
        FakePortal portal = new FakePortal();
        FakeAprilTagProcessor processor = new FakeAprilTagProcessor();
        MutableNanoClock nanos = new MutableNanoClock(1_000_000_000L);
        FtcWebcamVisionPortalLane owner = new FtcWebcamVisionPortalLane(
                validConfig(), new RecordingFactory(portal), nanos,
                VALID_RESOLUTION_READER, processor);
        FtcWebcamAprilTagSupport.PortalAprilTagSensor sensor =
                new FtcWebcamAprilTagSupport.PortalAprilTagSensor(owner, processor);
        ManualLoopClock time = new ManualLoopClock(10.0);
        nanos.now = 2_000_000_000L;
        processor.setDetection(1_900_000_000L);

        assertEquals(1, sensor.get(time.clock()).observations.size());

        // TIME-01 makes even a same-value reset a distinct timestamp epoch and cycle.
        time.clock().reset(10.0);
        assertTrue(sensor.get(time.clock()).observations.isEmpty());

        // Resetting the sensor must not erase the blocked vendor-frame identity.
        sensor.reset();
        assertTrue(sensor.get(time.clock()).observations.isEmpty());

        processor.setDetection(1_950_000_000L);
        time.nextCycle(0.0);
        AprilTagDetections newFrame = sensor.get(time.clock());
        assertEquals(1, newFrame.observations.size());
        assertEquals(0.05, newFrame.frameAgeSec(time.clock()), 1e-9);

        // A second reset advances the anchor's bounded reset barrier. The owner's monotonic
        // capture identity still rejects replay of the older first frame.
        time.clock().reset(10.0);
        assertTrue(sensor.get(time.clock()).observations.isEmpty());
        processor.setDetection(1_900_000_000L);
        time.nextCycle(0.0);
        assertTrue(sensor.get(time.clock()).observations.isEmpty());

        processor.setDetection(1_975_000_000L);
        time.nextCycle(0.0);
        assertEquals(1, sensor.get(time.clock()).observations.size());
        owner.close();
    }

    @Test
    public void aprilTagAdapterRejectsMixedAndRegressingUsableFrames() {
        FakePortal portal = new FakePortal();
        FakeAprilTagProcessor processor = new FakeAprilTagProcessor();
        MutableNanoClock nanos = new MutableNanoClock(1_000_000_000L);
        FtcWebcamVisionPortalLane owner = new FtcWebcamVisionPortalLane(
                validConfig(), new RecordingFactory(portal), nanos,
                VALID_RESOLUTION_READER, processor);
        FtcWebcamAprilTagSupport.PortalAprilTagSensor sensor =
                new FtcWebcamAprilTagSupport.PortalAprilTagSensor(owner, processor);
        ManualLoopClock time = new ManualLoopClock(10.0);
        nanos.now = 2_000_000_000L;

        processor.setValidDetections(1_900_000_000L, 1_500_000_000L);
        AprilTagDetections mixed = sensor.get(time.clock());

        assertTrue(mixed.observations.isEmpty());

        time.nextCycle(0.0);
        processor.setValidDetections(1_900_000_000L, 0L);
        assertTrue("one usable detection with invalid timing closes the whole frame",
                sensor.get(time.clock()).observations.isEmpty());

        time.nextCycle(0.0);
        processor.setValidAndInvalidDetections(1_900_000_000L, 1_500_000_000L);
        AprilTagDetections invalidOlder = sensor.get(time.clock());

        assertEquals(1, invalidOlder.observations.size());
        assertEquals(0.1, invalidOlder.frameAgeSec(time.clock()), 1e-9);

        time.nextCycle(0.0);
        processor.setDetection(1_800_000_000L);
        assertTrue(sensor.get(time.clock()).observations.isEmpty());

        time.nextCycle(0.0);
        processor.setDetection(1_950_000_000L);
        assertEquals(1, sensor.get(time.clock()).observations.size());
        owner.close();
    }

    @Test
    public void freshReconstructionHasIndependentConfigProcessorsAndOwnership() {
        FtcWebcamVisionPortalLane.Config cfg = validConfig();
        FakePortal firstPortal = new FakePortal();
        FakeProcessor firstProcessor = new FakeProcessor("first", false);
        FtcWebcamVisionPortalLane first = new FtcWebcamVisionPortalLane(
                cfg, new RecordingFactory(firstPortal), new MutableNanoClock(100),
                VALID_RESOLUTION_READER, firstProcessor);
        cfg.webcamName = "mutated after construction";
        assertEquals("Webcam 1", first.webcamName());
        first.close();

        FakePortal secondPortal = new FakePortal();
        FakeProcessor secondProcessor = new FakeProcessor("second", false);
        FtcWebcamVisionPortalLane second = new FtcWebcamVisionPortalLane(
                validConfig(), new RecordingFactory(secondPortal), new MutableNanoClock(100),
                VALID_RESOLUTION_READER, secondProcessor);
        assertTrue(second.ownsProcessor(secondProcessor));
        assertFalse(second.ownsProcessor(firstProcessor));
        assertFalse(second.isCloseAttempted());
    }

    private static FtcWebcamVisionPortalLane open(
            FakePortal portal,
            VisionProcessor... processors
    ) {
        return new FtcWebcamVisionPortalLane(
                validConfig(), new RecordingFactory(portal), new MutableNanoClock(100),
                VALID_RESOLUTION_READER, processors);
    }

    private static FtcWebcamVisionPortalLane testOwner(
            FtcWebcamVisionPortalLane.Config config,
            FtcWebcamVisionPortalLane.PortalFactory factory,
            VisionProcessor... processors
    ) {
        return new FtcWebcamVisionPortalLane(
                config, factory, new MutableNanoClock(100), VALID_RESOLUTION_READER, processors);
    }

    private static FtcWebcamVisionPortalLane.Config validConfig() {
        return FtcWebcamVisionPortalLane.Config.defaults();
    }

    private static void assertNotReady(VisionReadiness readiness, String reasonFragment) {
        assertFalse(readiness.toString(), readiness.isReady());
        assertTrue(readiness.reason(), readiness.reason().contains(reasonFragment));
    }

    private static void assertMatrixEquals(Mat3 expected, Mat3 actual, double tolerance) {
        assertEquals(expected.m00, actual.m00, tolerance);
        assertEquals(expected.m01, actual.m01, tolerance);
        assertEquals(expected.m02, actual.m02, tolerance);
        assertEquals(expected.m10, actual.m10, tolerance);
        assertEquals(expected.m11, actual.m11, tolerance);
        assertEquals(expected.m12, actual.m12, tolerance);
        assertEquals(expected.m20, actual.m20, tolerance);
        assertEquals(expected.m21, actual.m21, tolerance);
        assertEquals(expected.m22, actual.m22, tolerance);
    }

    private static GeneralMatrixF generalMatrix(Mat3 matrix) {
        return new GeneralMatrixF(3, 3, new float[]{
                (float) matrix.m00, (float) matrix.m01, (float) matrix.m02,
                (float) matrix.m10, (float) matrix.m11, (float) matrix.m12,
                (float) matrix.m20, (float) matrix.m21, (float) matrix.m22
        });
    }

    private static void expectFailure(
            Class<? extends RuntimeException> expectedType,
            Runnable action
    ) {
        try {
            action.run();
            fail("Expected " + expectedType.getSimpleName());
        } catch (RuntimeException actual) {
            assertTrue("Unexpected failure: " + actual, expectedType.isInstance(actual));
        }
    }

    private static void expectFailureContaining(
            Class<? extends RuntimeException> expectedType,
            String messageFragment,
            Runnable action
    ) {
        try {
            action.run();
            fail("Expected " + expectedType.getSimpleName());
        } catch (RuntimeException actual) {
            assertTrue("Unexpected failure: " + actual, expectedType.isInstance(actual));
            assertTrue("Unexpected message: " + actual.getMessage(),
                    actual.getMessage() != null && actual.getMessage().contains(messageFragment));
        }
    }

    private static final class MutableNanoClock implements FtcWebcamVisionPortalLane.NanoClock {
        long now;

        MutableNanoClock(long now) {
            this.now = now;
        }

        @Override
        public long nowNanos() {
            return now;
        }
    }

    private static final FtcWebcamVisionPortalLane.ResolutionReader VALID_RESOLUTION_READER =
            new FtcWebcamVisionPortalLane.ResolutionReader() {
                @Override
                public int width(Size size) {
                    return 640;
                }

                @Override
                public int height(Size size) {
                    return 480;
                }
            };

    private static final FtcWebcamVisionPortalLane.ResolutionReader INVALID_WIDTH_READER =
            new FtcWebcamVisionPortalLane.ResolutionReader() {
                @Override
                public int width(Size size) {
                    return 0;
                }

                @Override
                public int height(Size size) {
                    return 480;
                }
            };

    private static final class RecordingFactory implements FtcWebcamVisionPortalLane.PortalFactory {
        private final FakePortal portal;
        private int openCount;

        RecordingFactory(FakePortal portal) {
            this.portal = portal;
        }

        @Override
        public FtcWebcamVisionPortalLane.PortalDevice open(
                FtcWebcamVisionPortalLane.Config config,
                VisionProcessor[] processors
        ) {
            openCount++;
            portal.openConfig = config;
            portal.openProcessors = processors;
            for (VisionProcessor processor : processors) {
                portal.enabled.put(processor, Boolean.TRUE);
            }
            return portal;
        }
    }

    private static final class FakePortal implements FtcWebcamVisionPortalLane.PortalDevice {
        final IdentityHashMap<VisionProcessor, Boolean> enabled =
                new IdentityHashMap<VisionProcessor, Boolean>();
        VisionPortal.CameraState cameraState = VisionPortal.CameraState.STREAMING;
        FtcWebcamVisionPortalLane.Config openConfig;
        VisionProcessor[] openProcessors;
        RuntimeException enabledFailure;
        RuntimeException closeFailure;
        int enabledReads;
        int enableWrites;
        int stopStreamingCalls;
        int resumeStreamingCalls;
        int closeCalls;

        @Override
        public VisionPortal.CameraState cameraState() {
            return cameraState;
        }

        @Override
        public void setProcessorEnabled(VisionProcessor processor, boolean value) {
            enableWrites++;
            enabled.put(processor, value);
        }

        @Override
        public boolean isProcessorEnabled(VisionProcessor processor) {
            enabledReads++;
            if (enabledFailure != null) throw enabledFailure;
            Boolean value = enabled.get(processor);
            if (value == null) throw new IllegalArgumentException("processor is not registered");
            return value;
        }

        @Override
        public void stopStreaming() {
            stopStreamingCalls++;
        }

        @Override
        public void resumeStreaming() {
            resumeStreamingCalls++;
        }

        @Override
        public float framesPerSecond() {
            return 30.0f;
        }

        @Override
        public <T extends CameraControl> T cameraControl(Class<T> controlType) {
            return null;
        }

        @Override
        public void close() {
            closeCalls++;
            if (closeFailure != null) throw closeFailure;
        }
    }

    private static class FakeProcessor implements VisionProcessor {
        private final String equalityGroup;
        private final boolean useValueEquality;

        FakeProcessor(String equalityGroup, boolean useValueEquality) {
            this.equalityGroup = equalityGroup;
            this.useValueEquality = useValueEquality;
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            return null;
        }

        @Override
        public void onDrawFrame(
                Canvas canvas,
                int onscreenWidth,
                int onscreenHeight,
                float scaleBmpPxToCanvasPx,
                float scaleCanvasDensity,
                Object userContext
        ) {
        }

        @Override
        public boolean equals(Object other) {
            if (!useValueEquality || !(other instanceof FakeProcessor)) {
                return this == other;
            }
            FakeProcessor that = (FakeProcessor) other;
            return that.useValueEquality && equalityGroup.equals(that.equalityGroup);
        }

        @Override
        public int hashCode() {
            return useValueEquality ? equalityGroup.hashCode() : System.identityHashCode(this);
        }
    }

    private static final class FakeAprilTagProcessor extends AprilTagProcessor {
        private final ArrayList<AprilTagDetection> detections = new ArrayList<AprilTagDetection>();

        void setDetection(long frameAcquisitionNanos) {
            setDetection(
                    frameAcquisitionNanos,
                    new AprilTagPoseFtc(1.0, 2.0, 3.0, 0.1, 0.2, 0.3,
                            2.0, 0.0, 0.0),
                    null,
                    null
            );
        }

        void setRawDetection(
                long frameAcquisitionNanos,
                AprilTagPoseRaw rawPose,
                Pose3D robotPose
        ) {
            setDetection(frameAcquisitionNanos, null, rawPose, robotPose);
        }

        void setValidDetections(long firstFrameNanos, long secondFrameNanos) {
            detections.clear();
            detections.add(detection(7, firstFrameNanos, defaultFtcPose()));
            detections.add(detection(8, secondFrameNanos, defaultFtcPose()));
        }

        void setValidAndInvalidDetections(long validFrameNanos, long invalidFrameNanos) {
            detections.clear();
            detections.add(detection(7, validFrameNanos, defaultFtcPose()));
            detections.add(detection(8, invalidFrameNanos, null));
        }

        private void setDetection(
                long frameAcquisitionNanos,
                AprilTagPoseFtc ftcPose,
                AprilTagPoseRaw rawPose,
                Pose3D robotPose
        ) {
            detections.clear();
            detections.add(new AprilTagDetection(
                    7, 0, 1.0f, null, null, null,
                    ftcPose, rawPose, robotPose, frameAcquisitionNanos));
        }

        private static AprilTagDetection detection(int id,
                                                   long frameAcquisitionNanos,
                                                   AprilTagPoseFtc ftcPose) {
            return new AprilTagDetection(
                    id, 0, 1.0f, null, null, null,
                    ftcPose, null, null, frameAcquisitionNanos);
        }

        private static AprilTagPoseFtc defaultFtcPose() {
            return new AprilTagPoseFtc(
                    1.0, 2.0, 3.0, 0.1, 0.2, 0.3,
                    2.0, 0.0, 0.0);
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            return null;
        }

        @Override
        public void onDrawFrame(
                Canvas canvas,
                int onscreenWidth,
                int onscreenHeight,
                float scaleBmpPxToCanvasPx,
                float scaleCanvasDensity,
                Object userContext
        ) {
        }

        @Override
        public void setDecimation(float decimation) {
        }

        @Override
        public void setPoseSolver(PoseSolver poseSolver) {
        }

        @Override
        public int getPerTagAvgPoseSolveTime() {
            return 0;
        }

        @Override
        public ArrayList<AprilTagDetection> getDetections() {
            return new ArrayList<AprilTagDetection>(detections);
        }

        @Override
        public ArrayList<AprilTagDetection> getFreshDetections() {
            return getDetections();
        }
    }
}
