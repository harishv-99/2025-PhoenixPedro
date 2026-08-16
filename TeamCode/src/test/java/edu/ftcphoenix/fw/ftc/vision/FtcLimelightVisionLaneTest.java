package edu.ftcphoenix.fw.ftc.vision;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.ftc.localization.LimelightFieldPoseEstimator;
import edu.ftcphoenix.fw.localization.MotionDelta;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused lifecycle, generation, and freshness coverage for the Limelight owner. */
public final class FtcLimelightVisionLaneTest {

    @Test
    public void invalidConfigurationNeverOpensHardware() {
        RecordingFactory factory = new RecordingFactory(new FakeDevice());

        FtcLimelightVisionLane.Config cfg = validConfig();
        cfg.hardwareName = "   ";
        FtcLimelightVisionLane.Config blankName = cfg;
        expectFailureContaining(IllegalArgumentException.class,
                "FtcLimelightVisionLane.Config.hardwareName must not be blank",
                () -> new FtcLimelightVisionLane(blankName, factory));

        cfg = validConfig();
        cfg.hardwareName = null;
        FtcLimelightVisionLane.Config nullName = cfg;
        expectFailureContaining(IllegalArgumentException.class,
                "FtcLimelightVisionLane.Config.hardwareName must not be blank",
                () -> new FtcLimelightVisionLane(nullName, factory));

        cfg = validConfig();
        cfg.pipelineIndex = -1;
        FtcLimelightVisionLane.Config invalidPipeline = cfg;
        expectFailureContaining(IllegalArgumentException.class,
                "FtcLimelightVisionLane.Config.pipelineIndex "
                        + "must be within [0, 9], got -1",
                () -> new FtcLimelightVisionLane(invalidPipeline, factory));

        cfg = validConfig();
        cfg.pipelineIndex = 10;
        FtcLimelightVisionLane.Config highPipeline = cfg;
        expectFailureContaining(IllegalArgumentException.class,
                "FtcLimelightVisionLane.Config.pipelineIndex "
                        + "must be within [0, 9], got 10",
                () -> new FtcLimelightVisionLane(highPipeline, factory));

        cfg = validConfig();
        cfg.pollRateHz = 0;
        FtcLimelightVisionLane.Config zeroPoll = cfg;
        expectFailureContaining(IllegalArgumentException.class,
                "FtcLimelightVisionLane.Config.pollRateHz "
                        + "must be within [1, 250], got 0",
                () -> new FtcLimelightVisionLane(zeroPoll, factory));

        cfg = validConfig();
        cfg.pollRateHz = 251;
        FtcLimelightVisionLane.Config highPoll = cfg;
        expectFailureContaining(IllegalArgumentException.class,
                "FtcLimelightVisionLane.Config.pollRateHz "
                        + "must be within [1, 250], got 251",
                () -> new FtcLimelightVisionLane(highPoll, factory));

        for (double age : new double[]{
                0.0,
                -1.0,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        }) {
            cfg = validConfig();
            cfg.maxResultAgeSec = age;
            FtcLimelightVisionLane.Config invalidAge = cfg;
            expectFailureContaining(IllegalArgumentException.class,
                    "FtcLimelightVisionLane.Config.maxResultAgeSec "
                            + "must be finite and > 0, got " + age,
                    () -> new FtcLimelightVisionLane(invalidAge, factory));
        }

        expectFailureContaining(NullPointerException.class,
                "FtcLimelightVisionLane.Config",
                () -> new FtcLimelightVisionLane(null, factory));
        expectFailureContaining(NullPointerException.class,
                "deviceFactory",
                () -> new FtcLimelightVisionLane(validConfig(), null));
        assertEquals(0, factory.openCount);
    }

    @Test
    public void invalidAprilTagSpecializationNeverOpensHardware() {
        RecordingFactory factory = new RecordingFactory(new FakeDevice());

        FtcLimelightAprilTagVisionLane.Config cfg =
                FtcLimelightAprilTagVisionLane.Config.defaults();
        cfg.hardwareName = null;
        FtcLimelightAprilTagVisionLane.Config nullName = cfg;
        expectFailureContaining(IllegalArgumentException.class,
                "FtcLimelightAprilTagVisionLane.Config.hardwareName must not be blank",
                () -> new FtcLimelightAprilTagVisionLane(nullName, factory));

        cfg = FtcLimelightAprilTagVisionLane.Config.defaults();
        cfg.hardwareName = "   ";
        FtcLimelightAprilTagVisionLane.Config blankName = cfg;
        expectFailureContaining(IllegalArgumentException.class,
                "FtcLimelightAprilTagVisionLane.Config.hardwareName must not be blank",
                () -> new FtcLimelightAprilTagVisionLane(blankName, factory));

        for (int pipelineIndex : new int[]{-1, 10}) {
            cfg = FtcLimelightAprilTagVisionLane.Config.defaults();
            cfg.pipelineIndex = pipelineIndex;
            FtcLimelightAprilTagVisionLane.Config invalidPipeline = cfg;
            expectFailureContaining(IllegalArgumentException.class,
                    "FtcLimelightAprilTagVisionLane.Config.pipelineIndex "
                            + "must be within [0, 9], got " + pipelineIndex,
                    () -> new FtcLimelightAprilTagVisionLane(invalidPipeline, factory));
        }

        for (int pollRateHz : new int[]{0, 251}) {
            cfg = FtcLimelightAprilTagVisionLane.Config.defaults();
            cfg.pollRateHz = pollRateHz;
            FtcLimelightAprilTagVisionLane.Config invalidPoll = cfg;
            expectFailureContaining(IllegalArgumentException.class,
                    "FtcLimelightAprilTagVisionLane.Config.pollRateHz "
                            + "must be within [1, 250], got " + pollRateHz,
                    () -> new FtcLimelightAprilTagVisionLane(invalidPoll, factory));
        }

        for (double age : new double[]{
                0.0,
                -1.0,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        }) {
            cfg = FtcLimelightAprilTagVisionLane.Config.defaults();
            cfg.maxResultAgeSec = age;
            FtcLimelightAprilTagVisionLane.Config invalidAge = cfg;
            expectFailureContaining(IllegalArgumentException.class,
                    "FtcLimelightAprilTagVisionLane.Config.maxResultAgeSec "
                            + "must be finite and > 0, got " + age,
                    () -> new FtcLimelightAprilTagVisionLane(invalidAge, factory));
        }

        cfg = FtcLimelightAprilTagVisionLane.Config.defaults();
        cfg.cameraMount = null;
        FtcLimelightAprilTagVisionLane.Config nullMount = cfg;
        expectFailureContaining(IllegalArgumentException.class,
                "FtcLimelightAprilTagVisionLane.Config.cameraMount must not be null",
                () -> new FtcLimelightAprilTagVisionLane(nullMount, factory));

        expectFailureContaining(NullPointerException.class,
                "FtcLimelightAprilTagVisionLane.Config",
                () -> new FtcLimelightAprilTagVisionLane(null, factory));
        assertEquals(0, factory.openCount);
    }

    @Test
    public void constructionOrdersSetupAndRejectedRequestNeverRetriesInTheLoop() {
        FakeDevice device = new FakeDevice();
        device.switchAccepted = false;
        RecordingFactory factory = new RecordingFactory(device);
        FtcLimelightVisionLane.Config cfg = validConfig();
        cfg.hardwareName = "  limelight-main  ";

        FtcLimelightVisionLane lane = new FtcLimelightVisionLane(cfg, factory);

        assertEquals(Arrays.asList(
                "open:limelight-main", "poll:100", "switch:0", "now", "start"
        ), device.events);
        assertEquals("limelight-main", lane.hardwareName());
        assertEquals(1L, lane.pipelineGeneration());
        assertFalse(lane.wasPipelineRequestAccepted());

        ManualLoopClock clock = new ManualLoopClock();
        assertFalse(lane.pipelineReadiness(clock.clock()).isReady());
        assertFalse(lane.confirmedPipelineResult(clock.clock()).hasResult());
        assertFalse(lane.requestPipeline(0));
        assertEquals(1, device.switchCalls);
        assertEquals(0, device.latestReads);
    }

    @Test
    public void setupFailurePreservesPrimaryAndAttemptsStopThenClose() {
        FakeDevice device = new FakeDevice();
        RuntimeException primary = new IllegalStateException("start failed");
        RuntimeException stop = new IllegalArgumentException("stop failed");
        RuntimeException close = new UnsupportedOperationException("close failed");
        device.startFailure = primary;
        device.stopFailure = stop;
        device.closeFailure = close;

        try {
            new FtcLimelightVisionLane(validConfig(), new RecordingFactory(device));
            fail("Expected setup failure");
        } catch (RuntimeException actual) {
            assertSame(primary, actual);
            assertEquals(2, actual.getSuppressed().length);
            assertSame(stop, actual.getSuppressed()[0]);
            assertSame(close, actual.getSuppressed()[1]);
        }
        assertEquals(Arrays.asList(
                "open:limelight", "poll:100", "switch:0", "now", "start", "stop", "close"
        ), device.events);

        RecordingFactory nullFactory = new RecordingFactory(null);
        expectFailure(IllegalStateException.class,
                () -> new FtcLimelightVisionLane(validConfig(), nullFactory));
    }

    @Test
    public void configIsDefensivelyCopiedAndRetryUsesAFreshOwner() {
        FtcLimelightVisionLane.Config cfg = validConfig();
        cfg.hardwareName = "first-limelight";
        cfg.pipelineIndex = 2;
        FakeDevice firstDevice = new FakeDevice();
        FtcLimelightVisionLane first = new FtcLimelightVisionLane(
                cfg, new RecordingFactory(firstDevice));

        cfg.hardwareName = "mutated-after-open";
        cfg.pipelineIndex = 7;
        assertEquals("first-limelight", first.hardwareName());
        assertEquals(2, first.requestedPipelineIndex());

        assertEquals("first-limelight", first.hardwareName());
        assertEquals(2, first.requestedPipelineIndex());

        first.close();
        assertTrue(first.isCloseAttempted());
        assertTrue(first.isClosed());
        assertEquals("", first.closeFailureReason());
        FakeDevice secondDevice = new FakeDevice();
        FtcLimelightVisionLane.Config retryCfg = validConfig();
        retryCfg.hardwareName = "first-limelight";
        retryCfg.pipelineIndex = 2;
        FtcLimelightVisionLane second = new FtcLimelightVisionLane(
                retryCfg, new RecordingFactory(secondDevice));

        assertEquals(1, firstDevice.stopCalls);
        assertEquals(1, firstDevice.closeCalls);
        assertFalse(second.isClosed());
        assertEquals(1, secondDevice.switchCalls);
        assertTrue(secondDevice.running);
        second.close();
    }

    @Test
    public void readinessRejectsTransportGenerationPipelineAndAgeButNotNoTarget() {
        FakeDevice device = new FakeDevice();
        FtcLimelightVisionLane lane = open(device);
        ManualLoopClock manual = new ManualLoopClock();
        LoopClock clock = manual.clock();

        device.running = false;
        device.result = result(101, 0.01, 0, true);
        assertNotReady(lane.pipelineReadiness(clock), "not running");
        assertEquals(0, device.latestReads);

        manual.nextCycle(0.02);
        device.running = true;
        device.connected = false;
        assertNotReady(lane.pipelineReadiness(clock), "disconnected");
        assertEquals(0, device.latestReads);

        manual.nextCycle(0.02);
        device.connected = true;
        device.result = null;
        assertNotReady(lane.pipelineReadiness(clock), "not produced");

        assertResultState(lane, device, manual, result(100, 0.01, 0, true), false, "newer");
        assertResultState(lane, device, manual, result(101, 0.01, 1, true), false, "reports pipeline");
        assertResultState(lane, device, manual, result(101, Double.NaN, 0, true), false, "invalid receipt timing");
        assertResultState(lane, device, manual, result(101, -0.01, 0, true), false, "invalid receipt timing");
        assertResultState(lane, device, manual,
                result(101, Double.POSITIVE_INFINITY, 0, true), false,
                "invalid receipt timing");
        assertResultState(lane, device, manual,
                timedResult(101, 0.01, 1000.0, Double.NaN, 0.0, 0, true), false,
                "invalid or reset-stale frame timing");
        assertResultState(lane, device, manual,
                timedResult(101, 0.01, 1000.0, 0.0, -1.0, 0, true), false,
                "invalid or reset-stale frame timing");
        assertResultState(lane, device, manual, result(101, 0.251, 0, true), false, "stale");

        assertResultState(lane, device, manual,
                timedResult(101, 0.25, 2000.0, 0.0, 0.0, 0, false), true, "ready");
        FtcLimelightVisionLane.ResultSnapshot noTarget = lane.confirmedPipelineResult(manual.clock());
        assertTrue(noTarget.hasResult());
        assertFalse(noTarget.isTargetValid());
        assertEquals(0, noTarget.pipelineIndex());
        assertEquals(101L, noTarget.resultReceivedAtControlHubMillis());
        assertEquals(0.25, noTarget.frameTimestamp().ageSec(manual.clock()), 1e-9);
    }

    @Test
    public void samplingIsCycleStableAndDebugDoesNotReadTheDevice() {
        FakeDevice device = new FakeDevice();
        FtcLimelightVisionLane lane = open(device);
        device.result = result(101, 0.01, 0, false);
        ManualLoopClock manual = new ManualLoopClock();

        assertTrue(lane.pipelineReadiness(manual.clock()).isReady());
        assertTrue(lane.confirmedPipelineResult(manual.clock()).hasResult());
        assertTrue(lane.pipelineReadiness(manual.clock()).isReady());
        assertEquals(1, device.runningReads);
        assertEquals(1, device.connectedReads);
        assertEquals(1, device.latestReads);

        device.result = result(102, 0.01, 0, true);
        assertFalse(lane.confirmedPipelineResult(manual.clock()).isTargetValid());
        lane.debugDump(new RecordingDebugSink(), "vision");
        assertEquals(1, device.latestReads);

        manual.nextCycle(0.02);
        assertTrue(lane.confirmedPipelineResult(manual.clock()).isTargetValid());
        assertEquals(2, device.latestReads);
    }

    @Test
    public void stableResultIdentityRetainsOneExposureTimestampAndNewResultAdvancesIt() {
        FakeDevice device = new FakeDevice();
        FtcLimelightVisionLane lane = open(device);
        ManualLoopClock manual = new ManualLoopClock(20.0);

        device.result = timedResult(101, 0.05, 500.0, 20.0, 30.0, 0, true);
        FtcLimelightVisionLane.ResultSnapshot first =
                lane.confirmedPipelineResult(manual.clock());
        assertTrue(first.hasResult());
        assertEquals(0.10, first.frameTimestamp().ageSec(manual.clock()), 1e-9);

        manual.nextCycle(0.10);
        device.result = timedResult(102, 0.15, 500.0, 20.0, 30.0, 0, true);
        FtcLimelightVisionLane.ResultSnapshot repeated =
                lane.confirmedPipelineResult(manual.clock());
        assertSame(first.frameTimestamp(), repeated.frameTimestamp());
        assertEquals(0.20, repeated.frameTimestamp().ageSec(manual.clock()), 1e-9);

        manual.nextCycle(0.10);
        device.result = timedResult(102, 0.02, 501.0, 20.0, 30.0, 0, true);
        FtcLimelightVisionLane.ResultSnapshot newer =
                lane.confirmedPipelineResult(manual.clock());
        assertTrue(newer.hasResult());
        assertFalse(first.frameTimestamp() == newer.frameTimestamp());
        assertEquals(0.07, newer.frameTimestamp().ageSec(manual.clock()), 1e-9);
    }

    @Test
    public void missingLocalIdentityFallsBackToStableControlHubReceipt() {
        FakeDevice device = new FakeDevice();
        FtcLimelightVisionLane lane = open(device);
        ManualLoopClock manual = new ManualLoopClock(3.0);

        device.result = timedResult(101, 0.01, 0.0, 0.0, 0.0, 0, true);
        FtcLimelightVisionLane.ResultSnapshot first =
                lane.confirmedPipelineResult(manual.clock());
        manual.nextCycle(0.05);
        device.result = timedResult(101, 0.06, Double.NaN, 0.0, 0.0, 0, true);
        FtcLimelightVisionLane.ResultSnapshot repeated =
                lane.confirmedPipelineResult(manual.clock());
        assertSame(first.frameTimestamp(), repeated.frameTimestamp());

        manual.nextCycle(0.05);
        device.result = timedResult(102, 0.01, 0.0, 0.0, 0.0, 0, true);
        FtcLimelightVisionLane.ResultSnapshot newer =
                lane.confirmedPipelineResult(manual.clock());
        assertTrue(newer.hasResult());
        assertFalse(first.frameTimestamp() == newer.frameTimestamp());
    }

    @Test
    public void sameGenerationLocalAndFallbackIdentityRegressionsFailClosed() {
        FakeDevice localDevice = new FakeDevice();
        FtcLimelightVisionLane localLane = open(localDevice);
        ManualLoopClock localClock = new ManualLoopClock(3.0);

        localDevice.result = timedResult(101, 0.01, 500.0, 0.0, 0.0, 0, true);
        assertTrue(localLane.confirmedPipelineResult(localClock.clock()).hasResult());
        localDevice.result = timedResult(102, 0.01, 501.0, 0.0, 0.0, 0, true);
        localClock.nextCycle(0.02);
        assertTrue(localLane.confirmedPipelineResult(localClock.clock()).hasResult());
        localDevice.result = timedResult(103, 0.01, 500.0, 0.0, 0.0, 0, true);
        localClock.nextCycle(0.02);
        assertNotReady(localLane.pipelineReadiness(localClock.clock()), "frame timing");
        assertFalse(localLane.confirmedPipelineResult(localClock.clock()).hasResult());

        FakeDevice fallbackDevice = new FakeDevice();
        FtcLimelightVisionLane fallbackLane = open(fallbackDevice);
        ManualLoopClock fallbackClock = new ManualLoopClock(6.0);
        fallbackDevice.result = timedResult(101, 0.01, 0.0, 0.0, 0.0, 0, true);
        assertTrue(fallbackLane.confirmedPipelineResult(fallbackClock.clock()).hasResult());
        fallbackDevice.result = timedResult(102, 0.01, 0.0, 0.0, 0.0, 0, true);
        fallbackClock.nextCycle(0.02);
        assertTrue(fallbackLane.confirmedPipelineResult(fallbackClock.clock()).hasResult());
        fallbackDevice.result = timedResult(101, 0.01, 0.0, 0.0, 0.0, 0, true);
        fallbackClock.nextCycle(0.02);
        assertNotReady(fallbackLane.pipelineReadiness(fallbackClock.clock()), "frame timing");
        assertFalse(fallbackLane.confirmedPipelineResult(fallbackClock.clock()).hasResult());
    }

    @Test
    public void clockResetBlocksRetainedResultUntilGenuinelyNewIdentityArrives() {
        FakeDevice device = new FakeDevice();
        FtcLimelightVisionLane lane = open(device);
        ManualLoopClock manual = new ManualLoopClock(4.0);
        device.result = timedResult(101, 0.01, 700.0, 0.0, 0.0, 0, true);

        FtcLimelightVisionLane.ResultSnapshot beforeReset =
                lane.confirmedPipelineResult(manual.clock());
        assertTrue(beforeReset.hasResult());

        manual.clock().reset(4.0);
        assertNotReady(lane.pipelineReadiness(manual.clock()), "reset-stale frame timing");
        assertFalse(lane.confirmedPipelineResult(manual.clock()).hasResult());

        manual.nextCycle(0.02);
        assertFalse(lane.confirmedPipelineResult(manual.clock()).hasResult());

        device.result = timedResult(102, 0.01, 701.0, 0.0, 0.0, 0, true);
        manual.nextCycle(0.02);
        FtcLimelightVisionLane.ResultSnapshot afterReset =
                lane.confirmedPipelineResult(manual.clock());
        assertTrue(afterReset.hasResult());
        assertFalse(beforeReset.frameTimestamp() == afterReset.frameTimestamp());
        assertEquals(0.01, afterReset.frameTimestamp().ageSec(manual.clock()), 1e-9);

        // On another clock reset, the bounded anchor blocks the retained result and the owner's
        // monotonic local timestamp rejects replay of the older pre-reset result.
        manual.clock().reset(4.04);
        assertFalse(lane.confirmedPipelineResult(manual.clock()).hasResult());
        device.result = timedResult(103, 0.01, 700.0, 0.0, 0.0, 0, true);
        manual.nextCycle(0.02);
        assertFalse(lane.confirmedPipelineResult(manual.clock()).hasResult());

        device.result = timedResult(104, 0.01, 702.0, 0.0, 0.0, 0, true);
        manual.nextCycle(0.02);
        assertTrue(lane.confirmedPipelineResult(manual.clock()).hasResult());
    }

    @Test
    public void sdkAbsentPoseSentinelsAreUnavailableAndReturnedPosesAreDefensive() {
        FakeDevice device = new FakeDevice();
        FtcLimelightVisionLane lane = open(device);
        ManualLoopClock manual = new ManualLoopClock();
        Pose3D zeroPose = pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        device.result = resultWithPoses(101, zeroPose, zeroPose);

        FtcLimelightVisionLane.ResultSnapshot absent =
                lane.confirmedPipelineResult(manual.clock());
        assertTrue(absent.hasResult());
        assertNull(absent.botpose());
        assertNull(absent.botposeMt2());
        assertNull(FtcLimelightAprilTagVisionLane.phoenixCameraToTagPose(zeroPose));

        Pose3D source = pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
        device.result = resultWithPoses(102, source, null);
        source.getPosition().x = 99.0;
        manual.nextCycle(0.02);
        FtcLimelightVisionLane.ResultSnapshot present =
                lane.confirmedPipelineResult(manual.clock());
        Pose3D firstRead = present.botpose();
        assertEquals(1.0, firstRead.getPosition().x, 1e-9);
        firstRead.getPosition().x = 77.0;
        assertEquals(1.0, present.botpose().getPosition().x, 1e-9);
    }

    @Test
    public void directPoseEstimatorUsesCurrentLoopTimeWhenNoResultExists() {
        FtcLimelightAprilTagVisionLane.Config laneCfg =
                FtcLimelightAprilTagVisionLane.Config.defaults();
        laneCfg.hardwareName = "  limelight-main  ";
        FtcLimelightAprilTagVisionLane lane = new FtcLimelightAprilTagVisionLane(
                laneCfg, new RecordingFactory(new FakeDevice()));
        assertEquals("limelight-main", lane.hardwareName());
        LimelightFieldPoseEstimator.Config estimatorCfg =
                LimelightFieldPoseEstimator.Config.defaults();
        LimelightFieldPoseEstimator estimator =
                new LimelightFieldPoseEstimator(lane, null, estimatorCfg);
        ManualLoopClock manual = new ManualLoopClock(12.5);

        estimator.update(manual.clock());
        PoseEstimate estimate = estimator.getEstimate();
        assertFalse(estimate.hasPose);
        assertEquals(0.0, estimate.timestamp.ageSec(manual.clock()), 1e-9);

        estimatorCfg.mode = null;
        expectFailure(IllegalArgumentException.class,
                () -> new LimelightFieldPoseEstimator(lane, null, estimatorCfg));
        LimelightFieldPoseEstimator.Config zeroAgeCfg =
                LimelightFieldPoseEstimator.Config.defaults();
        zeroAgeCfg.maxResultAgeSec = 0.0;
        expectFailure(IllegalArgumentException.class,
                () -> new LimelightFieldPoseEstimator(lane, null, zeroAgeCfg));
        lane.close();
    }

    @Test
    public void directPoseEstimatorForwardsTheOwnersExactFrameTimestamp() {
        FakeDevice device = new FakeDevice();
        FtcLimelightAprilTagVisionLane.Config laneCfg =
                FtcLimelightAprilTagVisionLane.Config.defaults();
        FtcLimelightAprilTagVisionLane lane = new FtcLimelightAprilTagVisionLane(
                laneCfg, new RecordingFactory(device));
        ManualLoopClock manual = new ManualLoopClock(8.0);
        device.result = resultWithPoseAndOneFiducial(
                101,
                0.04,
                900.0,
                10.0,
                20.0,
                pose(1.0, 2.0, 0.1, 0.2, 0.0, 0.0)
        );

        FtcLimelightVisionLane.ResultSnapshot result =
                lane.confirmedAprilTagResult(manual.clock());
        assertTrue(result.hasResult());
        assertEquals(0.07, result.frameTimestamp().ageSec(manual.clock()), 1e-9);

        LimelightFieldPoseEstimator.Config estimatorCfg =
                LimelightFieldPoseEstimator.Config.defaults();
        estimatorCfg.maxResultAgeSec = 0.5;
        LimelightFieldPoseEstimator estimator =
                new LimelightFieldPoseEstimator(lane, null, estimatorCfg);
        estimator.update(manual.clock());

        PoseEstimate estimate = estimator.getEstimate();
        assertTrue(estimate.hasPose);
        assertSame(result.frameTimestamp(), estimate.timestamp);
        assertEquals(0.07, estimate.timestamp.ageSec(manual.clock()), 1e-9);
    }

    @Test
    public void directPoseEstimatorPublishesYawAndSamplesResultOncePerCycle() {
        FakeDevice device = new FakeDevice();
        FtcLimelightAprilTagVisionLane lane = new FtcLimelightAprilTagVisionLane(
                FtcLimelightAprilTagVisionLane.Config.defaults(),
                new RecordingFactory(device)
        );
        ManualLoopClock time = new ManualLoopClock(8.0);
        RecordingMotionPredictor predictor = new RecordingMotionPredictor();
        predictor.publish(0.4, time.clock().nowTimestamp());
        device.result = resultWithMt2PoseAndOneFiducial(
                101,
                0.04,
                900.0,
                10.0,
                20.0,
                pose(1.0, 2.0, 0.1, 0.2, 0.0, 0.0)
        );
        LimelightFieldPoseEstimator.Config config =
                LimelightFieldPoseEstimator.Config.defaults();
        config.mode = LimelightFieldPoseEstimator.Config.Mode.BOTPOSE_MT2;
        config.maxResultAgeSec = 0.5;
        LimelightFieldPoseEstimator estimator =
                new LimelightFieldPoseEstimator(lane, predictor, config);

        estimator.update(time.clock());
        PoseEstimate first = estimator.getEstimate();
        assertTrue(first.hasPose);
        assertEquals(1, device.orientationCalls);
        assertEquals(1, device.latestReads);

        device.result = resultWithMt2PoseAndOneFiducial(
                121,
                0.02,
                920.0,
                10.0,
                20.0,
                pose(3.0, 4.0, 0.1, -0.1, 0.0, 0.0)
        );
        estimator.update(time.clock());

        assertEquals(1, device.orientationCalls);
        assertEquals(1, device.latestReads);
        assertSame(first, estimator.getEstimate());

        time.nextCycle(0.02);
        predictor.publish(0.6, time.clock().nowTimestamp());
        estimator.update(time.clock());

        assertEquals(2, device.orientationCalls);
        assertEquals(2, device.latestReads);
        assertFalse(first == estimator.getEstimate());
        assertEquals(0.6, device.lastYawDegrees * Math.PI / 180.0, 1e-9);
    }

    @Test
    public void directPoseEstimatorDefersPipelineTransitionUntilTheNextCycle() {
        FakeDevice device = new FakeDevice();
        FtcLimelightAprilTagVisionLane lane = new FtcLimelightAprilTagVisionLane(
                FtcLimelightAprilTagVisionLane.Config.defaults(),
                new RecordingFactory(device)
        );
        ManualLoopClock time = new ManualLoopClock();
        device.result = resultWithPoseAndOneFiducial(
                101,
                0.01,
                100.0,
                0.0,
                0.0,
                pose(1.0, 2.0, 0.0, 0.1, 0.0, 0.0)
        );
        LimelightFieldPoseEstimator estimator = new LimelightFieldPoseEstimator(
                lane,
                null,
                LimelightFieldPoseEstimator.Config.defaults()
        );

        estimator.update(time.clock());
        PoseEstimate beforeSwitch = estimator.getEstimate();
        assertTrue(beforeSwitch.hasPose);
        assertTrue(lane.requestPipeline(1));

        estimator.update(time.clock());
        assertSame(beforeSwitch, estimator.getEstimate());

        time.nextCycle(0.02);
        estimator.update(time.clock());
        assertFalse(estimator.getEstimate().hasPose);
    }

    @Test
    public void directPoseEstimatorRetainsFailureAndRejectsReentryForTheCycle() {
        FakeDevice device = new FakeDevice();
        FtcLimelightAprilTagVisionLane lane = new FtcLimelightAprilTagVisionLane(
                FtcLimelightAprilTagVisionLane.Config.defaults(),
                new RecordingFactory(device)
        );
        ManualLoopClock time = new ManualLoopClock();
        RecordingMotionPredictor predictor = new RecordingMotionPredictor();
        predictor.publish(0.2, time.clock().nowTimestamp());
        LimelightFieldPoseEstimator.Config config =
                LimelightFieldPoseEstimator.Config.defaults();
        config.mode = LimelightFieldPoseEstimator.Config.Mode.BOTPOSE_MT2;
        LimelightFieldPoseEstimator estimator =
                new LimelightFieldPoseEstimator(lane, predictor, config);
        device.duringOrientation = () -> estimator.update(time.clock());

        RuntimeException reentry = captureFailure(() -> estimator.update(time.clock()));
        assertTrue(reentry instanceof IllegalStateException);
        assertTrue(reentry.getMessage().contains("reentered"));
        assertSame(reentry, captureFailure(() -> estimator.update(time.clock())));
        assertEquals(1, device.orientationCalls);
        assertEquals(0, device.latestReads);

        time.nextCycle(0.02);
        device.duringOrientation = null;
        RuntimeException sampleFailure = new IllegalStateException("latest result failed");
        device.latestFailure = sampleFailure;
        assertSame(sampleFailure, captureFailure(() -> estimator.update(time.clock())));
        assertSame(sampleFailure, captureFailure(() -> estimator.update(time.clock())));
        assertEquals(2, device.orientationCalls);
        assertEquals(1, device.latestReads);

        time.nextCycle(0.02);
        device.latestFailure = null;
        estimator.update(time.clock());
        assertEquals(3, device.orientationCalls);
        assertEquals(2, device.latestReads);
    }

    @Test
    public void directPoseEstimatorRequiresTheSharedLoopClock() {
        FtcLimelightAprilTagVisionLane lane = new FtcLimelightAprilTagVisionLane(
                FtcLimelightAprilTagVisionLane.Config.defaults(),
                new RecordingFactory(new FakeDevice())
        );
        LimelightFieldPoseEstimator estimator = new LimelightFieldPoseEstimator(
                lane,
                null,
                LimelightFieldPoseEstimator.Config.defaults()
        );

        RuntimeException failure = captureFailure(() -> estimator.update(null));

        assertTrue(failure instanceof NullPointerException);
        assertTrue(failure.getMessage().contains("clock"));
    }

    @Test
    public void pipelineGenerationRejectsCachedSwitchAwayAndSwitchBackResults() {
        FakeDevice device = new FakeDevice();
        FtcLimelightVisionLane lane = open(device);
        ManualLoopClock manual = new ManualLoopClock();
        device.result = timedResult(101, 0.01, 777.0, 0.0, 0.0, 0, true);
        FtcLimelightVisionLane.ResultSnapshot generationOne =
                lane.confirmedPipelineResult(manual.clock());
        assertTrue(generationOne.hasResult());

        device.nowMillis = 200;
        assertTrue(lane.requestPipeline(1));
        assertEquals(2L, lane.pipelineGeneration());
        assertNotReady(lane.pipelineReadiness(manual.clock()), "newer");
        assertFalse(lane.confirmedPipelineResult(manual.clock()).hasResult());

        device.result = timedResult(201, 0.01, 777.0, 0.0, 0.0, 1, true);
        manual.nextCycle(0.02);
        FtcLimelightVisionLane.ResultSnapshot generationTwo =
                lane.confirmedPipelineResult(manual.clock());
        assertTrue(generationTwo.hasResult());
        assertFalse(generationOne.frameTimestamp() == generationTwo.frameTimestamp());

        device.nowMillis = 300;
        assertTrue(lane.requestPipeline(0));
        assertEquals(3L, lane.pipelineGeneration());
        assertNotReady(lane.pipelineReadiness(manual.clock()), "newer");

        device.result = result(101, 0.01, 0, true);
        manual.nextCycle(0.02);
        assertNotReady(lane.pipelineReadiness(manual.clock()), "newer");

        device.result = result(301, 0.01, 0, true);
        manual.nextCycle(0.02);
        assertTrue(lane.pipelineReadiness(manual.clock()).isReady());
    }

    @Test
    public void aprilTagSensorRechecksReadinessAfterSameCyclePipelineTransitions() {
        FakeDevice device = new FakeDevice();
        FtcLimelightAprilTagVisionLane.Config cfg =
                FtcLimelightAprilTagVisionLane.Config.defaults();
        FtcLimelightAprilTagVisionLane lane =
                new FtcLimelightAprilTagVisionLane(cfg, new RecordingFactory(device));
        ManualLoopClock manual = new ManualLoopClock();
        device.result = result(101, 0.01, 0, false);

        assertTrue(lane.tagSensor().get(manual.clock()).observations.isEmpty());
        assertTrue(lane.tagSensor().get(manual.clock()).frameTimestamp().isAvailable());
        assertEquals(1, device.latestReads);

        device.nowMillis = 200;
        lane.requestPipeline(1);
        assertTrue(lane.tagSensor().get(manual.clock()).observations.isEmpty());
        assertFalse(lane.tagSensor().get(manual.clock()).frameTimestamp().isAvailable());
        assertEquals("switch-away must bypass the same-cycle tag cache", 1,
                device.latestReads);

        device.nowMillis = 300;
        lane.requestPipeline(0);
        assertTrue(lane.tagSensor().get(manual.clock()).observations.isEmpty());
        assertEquals("switch-back must resample instead of reviving generation-one data", 2,
                device.latestReads);

        lane.close();
        assertTrue(lane.tagSensor().get(manual.clock()).observations.isEmpty());
        assertEquals("closed sensor must not read the device", 2, device.latestReads);
    }

    @Test
    public void confirmedNoTargetIsTimestampedEmptyFrameWhileUnavailableResultIsNot() {
        FakeDevice device = new FakeDevice();
        FtcLimelightAprilTagVisionLane lane = new FtcLimelightAprilTagVisionLane(
                FtcLimelightAprilTagVisionLane.Config.defaults(),
                new RecordingFactory(device)
        );
        ManualLoopClock manual = new ManualLoopClock(5.0);
        device.result = timedResult(101, 0.02, 333.0, 5.0, 10.0, 0, false);

        edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections emptyFrame =
                lane.tagSensor().get(manual.clock());
        assertTrue(emptyFrame.observations.isEmpty());
        assertTrue(emptyFrame.frameTimestamp().isAvailable());
        assertEquals(0.035, emptyFrame.frameAgeSec(manual.clock()), 1e-9);

        lane.tagSensor().reset();
        manual.nextCycle(0.02);
        edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections afterSourceReset =
                lane.tagSensor().get(manual.clock());
        assertSame(emptyFrame.frameTimestamp(), afterSourceReset.frameTimestamp());
        assertEquals(0.055, afterSourceReset.frameAgeSec(manual.clock()), 1e-9);

        device.result = null;
        manual.nextCycle(0.02);
        edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections unavailable =
                lane.tagSensor().get(manual.clock());
        assertTrue(unavailable.observations.isEmpty());
        assertFalse(unavailable.frameTimestamp().isAvailable());
    }

    @Test
    public void failedPipelineTransitionIsTerminalWithoutAnExtraClockRead() {
        FakeDevice device = new FakeDevice();
        FtcLimelightVisionLane lane = open(device);
        RuntimeException requestFailure = new IllegalStateException("switch transport failed");
        RuntimeException baselineFailure = new IllegalArgumentException("clock failed");
        device.switchFailure = requestFailure;
        device.nowFailure = baselineFailure;

        try {
            lane.requestPipeline(1);
            fail("Expected pipeline request failure");
        } catch (RuntimeException actual) {
            assertSame(requestFailure, actual);
            assertEquals(0, actual.getSuppressed().length);
        }

        assertEquals(2L, lane.pipelineGeneration());
        assertFalse(lane.wasPipelineRequestAccepted());
        assertEquals("failed pipeline switch must not trigger a speculative baseline read",
                1, device.nowCalls);
        ManualLoopClock clock = new ManualLoopClock();
        assertNotReady(lane.pipelineReadiness(clock.clock()), "transition failed");
        assertFalse(lane.confirmedPipelineResult(clock.clock()).hasResult());
        assertEquals(0, device.latestReads);

        RecordingDebugSink debug = new RecordingDebugSink();
        lane.debugDump(debug, "vision");
        assertTrue(String.valueOf(debug.value("vision.terminalFailure"))
                .contains("transition failed"));

        device.switchFailure = null;
        device.nowFailure = null;
        expectFailure(IllegalStateException.class, () -> lane.requestPipeline(1));
        expectFailure(IllegalStateException.class, () -> lane.requestPipeline(2));
        expectFailure(IllegalStateException.class, () -> lane.updateRobotFieldYawRad(0.0));
        assertEquals(2, device.switchCalls);
        assertEquals(0, device.orientationCalls);

        lane.close();
        assertTrue(lane.isClosed());
        assertEquals(1, device.stopCalls);
        assertEquals(1, device.closeCalls);
    }

    @Test
    public void closeIsFailStopIdempotentAndOrientationUsesExplicitUnits() {
        FakeDevice device = new FakeDevice();
        FtcLimelightVisionLane lane = open(device);
        assertTrue(lane.updateRobotFieldYawRad(Math.PI / 2.0));
        assertEquals(90.0, device.lastYawDegrees, 1e-9);
        expectFailure(IllegalArgumentException.class,
                () -> lane.updateRobotFieldYawRad(Double.NaN));
        assertEquals(1, device.orientationCalls);

        RuntimeException stopFailure = new IllegalStateException("stop failed");
        RuntimeException closeFailure = new IllegalArgumentException("close failed");
        device.stopFailure = stopFailure;
        device.closeFailure = closeFailure;
        final FtcLimelightVisionLane[] holder = new FtcLimelightVisionLane[]{lane};
        device.duringStop = () -> {
            assertTrue(holder[0].isCloseAttempted());
            assertFalse(holder[0].isClosed());
            assertNotReady(
                    holder[0].pipelineReadiness(new ManualLoopClock().clock()),
                    "close is in progress");
        };

        try {
            lane.close();
            fail("Expected close failure");
        } catch (RuntimeException actual) {
            assertSame(stopFailure, actual);
            assertEquals(1, actual.getSuppressed().length);
            assertSame(closeFailure, actual.getSuppressed()[0]);
        }
        assertTrue(lane.isCloseAttempted());
        assertFalse(lane.isClosed());
        assertTrue(lane.closeFailureReason().contains("stop failed"));
        lane.close();
        assertEquals(1, device.stopCalls);
        assertEquals(1, device.closeCalls);
        assertNotReady(lane.pipelineReadiness(new ManualLoopClock().clock()), "close failed");

        RecordingDebugSink debug = new RecordingDebugSink();
        lane.debugDump(debug, "vision");
        assertEquals(Boolean.TRUE, debug.value("vision.closeAttempted"));
        assertEquals(Boolean.FALSE, debug.value("vision.closeSucceeded"));
        assertTrue(String.valueOf(debug.value("vision.closeFailure")).contains("stop failed"));
        expectFailure(IllegalStateException.class, () -> lane.requestPipeline(1));
        expectFailure(IllegalStateException.class, () -> lane.updateRobotFieldYawRad(0.0));
    }

    private static FtcLimelightVisionLane open(FakeDevice device) {
        return new FtcLimelightVisionLane(validConfig(), new RecordingFactory(device));
    }

    private static FtcLimelightVisionLane.Config validConfig() {
        return FtcLimelightVisionLane.Config.defaults();
    }

    private static FtcLimelightVisionLane.DeviceResult result(
            long timestampMillis,
            double ageSec,
            int pipelineIndex,
            boolean targetValid
    ) {
        return timedResult(
                timestampMillis,
                ageSec,
                timestampMillis,
                0.0,
                0.0,
                pipelineIndex,
                targetValid
        );
    }

    private static FtcLimelightVisionLane.DeviceResult timedResult(
            long resultReceivedAtControlHubMillis,
            double receiptStalenessSec,
            double limelightTimestampMillis,
            double captureLatencyMillis,
            double targetingLatencyMillis,
            int pipelineIndex,
            boolean targetValid
    ) {
        return FtcLimelightVisionLane.DeviceResult.metadata(
                resultReceivedAtControlHubMillis,
                receiptStalenessSec,
                limelightTimestampMillis,
                captureLatencyMillis,
                targetingLatencyMillis,
                pipelineIndex,
                targetValid
        );
    }

    private static FtcLimelightVisionLane.DeviceResult resultWithPoses(
            long timestampMillis,
            Pose3D botpose,
            Pose3D botposeMt2
    ) {
        return new FtcLimelightVisionLane.DeviceResult(
                timestampMillis,
                0.01,
                timestampMillis,
                0.0,
                0.0,
                0,
                "fiducial",
                true,
                null,
                null,
                null,
                null,
                null,
                botpose,
                botposeMt2
        );
    }

    private static FtcLimelightVisionLane.DeviceResult resultWithPoseAndOneFiducial(
            long resultReceivedAtControlHubMillis,
            double receiptStalenessSec,
            double limelightTimestampMillis,
            double captureLatencyMillis,
            double targetingLatencyMillis,
            Pose3D botpose
    ) {
        return new FtcLimelightVisionLane.DeviceResult(
                resultReceivedAtControlHubMillis,
                receiptStalenessSec,
                limelightTimestampMillis,
                captureLatencyMillis,
                targetingLatencyMillis,
                0,
                "fiducial",
                true,
                null,
                null,
                null,
                Collections.<LLResultTypes.FiducialResult>singletonList(
                        testFiducialResult()),
                null,
                botpose,
                null
        );
    }

    private static FtcLimelightVisionLane.DeviceResult resultWithMt2PoseAndOneFiducial(
            long resultReceivedAtControlHubMillis,
            double receiptStalenessSec,
            double limelightTimestampMillis,
            double captureLatencyMillis,
            double targetingLatencyMillis,
            Pose3D botposeMt2
    ) {
        return new FtcLimelightVisionLane.DeviceResult(
                resultReceivedAtControlHubMillis,
                receiptStalenessSec,
                limelightTimestampMillis,
                captureLatencyMillis,
                targetingLatencyMillis,
                0,
                "fiducial",
                true,
                null,
                null,
                null,
                Collections.<LLResultTypes.FiducialResult>singletonList(
                        testFiducialResult()),
                null,
                null,
                botposeMt2
        );
    }

    private static LLResultTypes.FiducialResult testFiducialResult() {
        try {
            // FTC exposes no public constructor for this SDK value. This fixture needs only one
            // non-null list member because the direct-pose estimator reads the list cardinality,
            // not any fiducial fields.
            Class<?> unsafeType = Class.forName("sun.misc.Unsafe");
            java.lang.reflect.Field singleton = unsafeType.getDeclaredField("theUnsafe");
            singleton.setAccessible(true);
            Object unsafe = singleton.get(null);
            java.lang.reflect.Method allocate =
                    unsafeType.getMethod("allocateInstance", Class.class);
            return (LLResultTypes.FiducialResult) allocate.invoke(
                    unsafe,
                    LLResultTypes.FiducialResult.class
            );
        } catch (ReflectiveOperationException e) {
            throw new AssertionError("Could not create a test-only Limelight fiducial value", e);
        }
    }

    private static Pose3D pose(
            double x,
            double y,
            double z,
            double yaw,
            double pitch,
            double roll
    ) {
        return new Pose3D(
                new Position(DistanceUnit.METER, x, y, z, 0),
                new YawPitchRollAngles(AngleUnit.RADIANS, yaw, pitch, roll, 0)
        );
    }

    private static void assertResultState(
            FtcLimelightVisionLane lane,
            FakeDevice device,
            ManualLoopClock manual,
            FtcLimelightVisionLane.DeviceResult result,
            boolean expectedReady,
            String reasonFragment
    ) {
        device.result = result;
        manual.nextCycle(0.02);
        VisionReadiness readiness = lane.pipelineReadiness(manual.clock());
        assertEquals(expectedReady, readiness.isReady());
        assertTrue(readiness.reason(), readiness.reason().contains(reasonFragment));
    }

    private static void assertNotReady(VisionReadiness readiness, String reasonFragment) {
        assertFalse(readiness.toString(), readiness.isReady());
        assertTrue(readiness.reason(), readiness.reason().contains(reasonFragment));
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
            String expectedMessage,
            Runnable action
    ) {
        try {
            action.run();
            fail("Expected " + expectedType.getSimpleName());
        } catch (RuntimeException actual) {
            assertTrue("Unexpected failure: " + actual, expectedType.isInstance(actual));
            assertEquals(expectedMessage, actual.getMessage());
        }
    }

    private static RuntimeException captureFailure(Runnable action) {
        try {
            action.run();
            fail("Expected update failure");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static final class RecordingFactory implements FtcLimelightVisionLane.DeviceFactory {
        private final FakeDevice device;
        private int openCount;

        RecordingFactory(FakeDevice device) {
            this.device = device;
        }

        @Override
        public FtcLimelightVisionLane.Device open(String hardwareName) {
            openCount++;
            if (device != null) {
                device.events.add("open:" + hardwareName);
            }
            return device;
        }
    }

    private static final class FakeDevice implements FtcLimelightVisionLane.Device {
        final List<String> events = new ArrayList<String>();
        boolean switchAccepted = true;
        boolean running;
        boolean connected = true;
        long nowMillis = 100;
        FtcLimelightVisionLane.DeviceResult result;
        RuntimeException switchFailure;
        RuntimeException nowFailure;
        RuntimeException startFailure;
        RuntimeException stopFailure;
        RuntimeException closeFailure;
        RuntimeException latestFailure;
        Runnable duringStop;
        Runnable duringOrientation;
        int switchCalls;
        int runningReads;
        int connectedReads;
        int latestReads;
        int stopCalls;
        int closeCalls;
        int orientationCalls;
        int nowCalls;
        double lastYawDegrees;

        @Override
        public void setPollRateHz(int pollRateHz) {
            events.add("poll:" + pollRateHz);
        }

        @Override
        public boolean pipelineSwitch(int pipelineIndex) {
            switchCalls++;
            events.add("switch:" + pipelineIndex);
            if (switchFailure != null) throw switchFailure;
            return switchAccepted;
        }

        @Override
        public void start() {
            events.add("start");
            if (startFailure != null) throw startFailure;
            running = true;
        }

        @Override
        public void stop() {
            stopCalls++;
            events.add("stop");
            if (duringStop != null) duringStop.run();
            running = false;
            if (stopFailure != null) throw stopFailure;
        }

        @Override
        public boolean isRunning() {
            runningReads++;
            return running;
        }

        @Override
        public boolean isConnected() {
            connectedReads++;
            return connected;
        }

        @Override
        public FtcLimelightVisionLane.DeviceResult latestResult() {
            latestReads++;
            if (latestFailure != null) throw latestFailure;
            return result;
        }

        @Override
        public boolean updateRobotOrientationDegrees(double fieldYawDegrees) {
            orientationCalls++;
            lastYawDegrees = fieldYawDegrees;
            if (duringOrientation != null) duringOrientation.run();
            return true;
        }

        @Override
        public long nowMillis() {
            nowCalls++;
            events.add("now");
            if (nowFailure != null) throw nowFailure;
            return nowMillis;
        }

        @Override
        public void close() {
            closeCalls++;
            events.add("close");
            if (closeFailure != null) throw closeFailure;
        }
    }

    private static final class RecordingDebugSink implements DebugSink {
        private final Map<String, Object> values = new LinkedHashMap<String, Object>();

        @Override
        public DebugSink addData(String key, Object value) {
            values.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }

        Object value(String key) {
            return values.get(key);
        }
    }

    private static final class RecordingMotionPredictor implements MotionPredictor {
        private PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        private MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());

        void publish(double yawRad, LoopTimestamp timestamp) {
            estimate = new PoseEstimate(
                    new Pose3d(0.0, 0.0, 0.0, yawRad, 0.0, 0.0),
                    true,
                    1.0,
                    timestamp
            );
            delta = MotionDelta.none(timestamp);
        }

        @Override
        public void update(LoopClock clock) {
            // The direct Limelight estimator reads a predictor snapshot; it does not own updates.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }

        @Override
        public MotionDelta getLatestMotionDelta() {
            return delta;
        }
    }
}
