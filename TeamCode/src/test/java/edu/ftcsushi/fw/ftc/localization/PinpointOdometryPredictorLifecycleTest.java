package edu.ftcsushi.fw.ftc.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.MotionDelta;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Deterministic CONFIG-04 Pinpoint device-status, reset, and rebase lifecycle coverage. */
public final class PinpointOdometryPredictorLifecycleTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void constructionValidatesBeforeLookupAndConfiguresThenResetsExactlyOnce() {
        PinpointOdometryPredictor.Config invalid = PinpointOdometryPredictor.Config.defaults();
        invalid.quality = Double.NaN;
        RecordingLookup invalidLookup = new RecordingLookup(new FakeDevice());

        RuntimeException validationFailure = capture(() ->
                new PinpointOdometryPredictor(invalidLookup, invalid));

        assertTrue(validationFailure instanceof IllegalArgumentException);
        assertEquals(0, invalidLookup.lookupCount);

        FakeDevice device = new FakeDevice();
        RecordingLookup lookup = new RecordingLookup(device);
        PinpointOdometryPredictor.Config config = PinpointOdometryPredictor.Config.defaults();
        config.hardwareMapName = "  pinPoint  ";
        config.forwardPodOffsetLeftInches = -3.0;
        config.strafePodOffsetForwardInches = 4.0;
        config.yawScalar = 1.002;
        config.encoderResolution =
                PinpointOdometryPredictor.EncoderResolution.ticksPerInch(1234.5);
        config.forwardPodDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        config.strafePodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        config.quality = 0.63;

        PinpointOdometryPredictor predictor = new PinpointOdometryPredictor(lookup, config);

        assertEquals(1, lookup.lookupCount);
        assertEquals("  pinPoint  ", lookup.lastName);
        assertEquals(Arrays.asList(
                "offsets:-3.0,4.0",
                "yaw:1.002",
                "customResolution:1234.5",
                "directions:REVERSED,FORWARD",
                "reset"
        ), device.effects);
        assertEquals(1, device.resetCount);
        assertEquals(GoBildaPinpointDriver.DeviceStatus.NOT_READY,
                predictor.lastDeviceStatus());
        assertFalse(predictor.getEstimate().hasPose);
        assertEquals(0L, predictor.trajectorySegmentId());

        // The owner retained a validated snapshot rather than the mutable draft.
        config.quality = 0.01;
        device.status = GoBildaPinpointDriver.DeviceStatus.READY;
        device.position = pose(1.0, 2.0, 0.0);
        ManualLoopClock time = new ManualLoopClock();
        time.nextCycle(0.02);
        predictor.update(time.clock());
        assertEquals(0.63, predictor.getEstimate().quality, 0.0);
        assertEquals(0L, predictor.trajectorySegmentId());
    }

    @Test
    public void partialConfigurationFailureStopsBeforeResetAndNamesStageAndDevice() {
        FakeDevice device = new FakeDevice();
        device.customResolutionFailure = new IllegalStateException("I2C write failed");
        PinpointOdometryPredictor.Config config = PinpointOdometryPredictor.Config.defaults();
        config.hardwareMapName = "tracking";
        config.encoderResolution =
                PinpointOdometryPredictor.EncoderResolution.ticksPerInch(900.0);

        RuntimeException failure = capture(() -> new PinpointOdometryPredictor(
                new RecordingLookup(device),
                config
        ));

        assertTrue(failure instanceof IllegalStateException);
        assertTrue(failure.getMessage().contains("Pinpoint hardware 'tracking'"));
        assertTrue(failure.getMessage().contains("encoder-resolution configuration"));
        assertSame(device.customResolutionFailure, failure.getCause());
        assertEquals(Arrays.asList("offsets:0.0,0.0", "customResolution:900.0"),
                device.effects);
        assertEquals(0, device.resetCount);
    }

    @Test
    public void onlyReadyPublishesAndRecoveryNeverBridgesStatusGap() {
        FakeDevice device = new FakeDevice();
        PinpointOdometryPredictor predictor = predictor(device);
        ManualLoopClock time = new ManualLoopClock();

        device.status = GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        device.position = pose(50.0, 60.0, 1.0);
        time.nextCycle(0.1);
        predictor.update(time.clock());
        assertEquals(GoBildaPinpointDriver.DeviceStatus.CALIBRATING,
                predictor.lastDeviceStatus());
        assertFalse(predictor.getEstimate().hasPose);
        assertEquals(0, device.positionReadCount);

        device.status = GoBildaPinpointDriver.DeviceStatus.READY;
        device.position = pose(0.0, 0.0, 0.1);
        device.velocityX = 4.0;
        device.velocityY = -2.0;
        device.headingVelocity = 0.5;
        time.nextCycle(0.5);
        predictor.update(time.clock());
        assertTrue(predictor.getEstimate().hasPose);
        assertFalse(predictor.getLatestMotionDelta().hasDelta);
        assertTrue(predictor.getKinematicSnapshot().hasUsableKinematics());
        assertEquals(0.0, predictor.getKinematicSnapshot().totalHeadingRad, EPSILON);

        device.position = pose(2.0, 0.0, 0.3);
        time.nextCycle(0.5);
        predictor.update(time.clock());
        assertTrue(predictor.getLatestMotionDelta().hasDelta);
        assertEquals(2.0 * Math.cos(0.1),
                predictor.getLatestMotionDelta().deltaPose.xInches,
                EPSILON);
        assertEquals(0.2, predictor.getKinematicSnapshot().totalHeadingRad, EPSILON);

        int readsBeforeFault = device.positionReadCount;
        device.status = GoBildaPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED;
        device.position = pose(100.0, 100.0, 2.0);
        time.nextCycle(0.5);
        predictor.update(time.clock());
        assertFalse(predictor.getEstimate().hasPose);
        assertFalse(predictor.getKinematicSnapshot().hasPose);
        assertEquals(readsBeforeFault, device.positionReadCount);
        assertEquals(0.2, predictor.getKinematicSnapshot().totalHeadingRad, EPSILON);

        device.status = GoBildaPinpointDriver.DeviceStatus.READY;
        device.position = pose(100.0, 100.0, 2.0);
        time.nextCycle(0.5);
        predictor.update(time.clock());
        assertTrue(predictor.getEstimate().hasPose);
        assertFalse(predictor.getLatestMotionDelta().hasDelta);
        assertEquals(0.2, predictor.getKinematicSnapshot().totalHeadingRad, EPSILON);

        device.position = pose(101.0, 100.0, 2.2);
        time.nextCycle(0.5);
        predictor.update(time.clock());
        assertTrue(predictor.getLatestMotionDelta().hasDelta);
        assertEquals(0.4, predictor.getKinematicSnapshot().totalHeadingRad, EPSILON);
    }

    @Test
    public void everyNonReadyStatusSuppressesReadsAndClearsMeasuredBaselines() {
        for (GoBildaPinpointDriver.DeviceStatus status
                : GoBildaPinpointDriver.DeviceStatus.values()) {
            if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
                continue;
            }

            FakeDevice device = new FakeDevice();
            PinpointOdometryPredictor predictor = predictor(device);
            ManualLoopClock time = new ManualLoopClock();

            device.status = GoBildaPinpointDriver.DeviceStatus.READY;
            device.position = pose(0.0, 0.0, 0.0);
            time.nextCycle(0.1);
            predictor.update(time.clock());
            device.position = pose(1.0, 0.0, 0.1);
            time.nextCycle(0.1);
            predictor.update(time.clock());
            assertTrue(status.name(), predictor.getLatestMotionDelta().hasDelta);

            int readsBeforeStatus = device.positionReadCount;
            device.status = status;
            device.position = pose(50.0, 60.0, 2.0);
            time.nextCycle(0.1);
            predictor.update(time.clock());
            assertEquals(status.name(), status, predictor.lastDeviceStatus());
            assertFalse(status.name(), predictor.getEstimate().hasPose);
            assertFalse(status.name(), predictor.getLatestMotionDelta().hasDelta);
            assertFalse(status.name(), predictor.getKinematicSnapshot().hasPose);
            assertFalse(status.name(), predictor.getKinematicSnapshot().hasVelocity);
            assertEquals(status.name(), readsBeforeStatus, device.positionReadCount);

            device.status = GoBildaPinpointDriver.DeviceStatus.READY;
            device.position = pose(50.0, 60.0, 2.0);
            time.nextCycle(0.1);
            predictor.update(time.clock());
            assertTrue(status.name(), predictor.getEstimate().hasPose);
            assertTrue(status.name(), predictor.getKinematicSnapshot().hasVelocity);
            assertFalse(status.name(), predictor.getLatestMotionDelta().hasDelta);
        }
    }

    @Test
    public void nullStatusAndInvalidReadyPoseAreUnavailableAndClearBaselines() {
        FakeDevice device = new FakeDevice();
        PinpointOdometryPredictor predictor = predictor(device);
        ManualLoopClock time = new ManualLoopClock();

        device.status = null;
        time.nextCycle(0.1);
        predictor.update(time.clock());
        assertEquals(GoBildaPinpointDriver.DeviceStatus.NOT_READY,
                predictor.lastDeviceStatus());
        assertFalse(predictor.getEstimate().hasPose);

        device.status = GoBildaPinpointDriver.DeviceStatus.READY;
        device.position = pose(0.0, 0.0, 0.0);
        time.nextCycle(0.1);
        predictor.update(time.clock());
        device.position = pose(1.0, 0.0, 0.1);
        time.nextCycle(0.1);
        predictor.update(time.clock());
        assertTrue(predictor.getLatestMotionDelta().hasDelta);

        device.position = pose(Double.NaN, 0.0, 0.2);
        time.nextCycle(0.1);
        predictor.update(time.clock());
        assertEquals(GoBildaPinpointDriver.DeviceStatus.READY,
                predictor.lastDeviceStatus());
        assertFalse(predictor.getEstimate().hasPose);

        device.position = pose(50.0, 0.0, 1.0);
        time.nextCycle(0.1);
        predictor.update(time.clock());
        assertFalse(predictor.getLatestMotionDelta().hasDelta);
    }

    @Test
    public void readySetPosePreservesMeasuredEvidenceAndRebasesMotion() {
        FakeDevice device = new FakeDevice();
        PinpointOdometryPredictor predictor = predictor(device);
        ManualLoopClock time = new ManualLoopClock();
        device.status = GoBildaPinpointDriver.DeviceStatus.READY;
        device.position = pose(5.0, 6.0, 0.0);
        device.velocityX = 3.0;
        device.velocityY = 2.0;
        device.headingVelocity = 0.4;
        time.nextCycle(0.2);
        predictor.update(time.clock());
        LoopTimestamp measuredTimestamp = predictor.getEstimate().timestamp;
        long measuredCycle = predictor.getKinematicSnapshot().cycle;
        long segmentBeforeRebase = predictor.trajectorySegmentId();
        device.setPositionObserver = () -> assertEquals(
                segmentBeforeRebase + 1L,
                predictor.trajectorySegmentId()
        );

        predictor.setPose(new Pose2d(40.0, -20.0, Math.PI * 3.0));

        assertEquals(segmentBeforeRebase + 1L, predictor.trajectorySegmentId());
        assertEquals(GoBildaPinpointDriver.DeviceStatus.READY,
                predictor.lastDeviceStatus());
        assertTrue(predictor.getEstimate().hasPose);
        assertSame(measuredTimestamp, predictor.getEstimate().timestamp);
        assertEquals(Math.PI, predictor.getEstimate().fieldToRobotPose.yawRad, EPSILON);
        assertTrue(predictor.getKinematicSnapshot().hasVelocity);
        assertEquals(3.0,
                predictor.getKinematicSnapshot().fieldVelocityXInchesPerSec,
                EPSILON);
        assertEquals(measuredCycle, predictor.getKinematicSnapshot().cycle);
        assertSame(measuredTimestamp, predictor.getKinematicSnapshot().timestamp);

        device.position = pose(39.0, -20.0, Math.PI);
        time.nextCycle(0.5);
        predictor.update(time.clock());
        MotionDelta afterRebase = predictor.getLatestMotionDelta();
        assertTrue(afterRebase.hasDelta);
        // Facing pi means field -X is +X motion in the rebased robot-relative delta.
        assertEquals(1.0, afterRebase.deltaPose.xInches, EPSILON);
        assertEquals(0.0, predictor.getKinematicSnapshot().totalHeadingRad, EPSILON);
        assertEquals(segmentBeforeRebase + 1L, predictor.trajectorySegmentId());
    }

    @Test
    public void resetAndRecalibrationForceNotReadyAndSetPoseCannotInventMeasurement() {
        FakeDevice device = new FakeDevice();
        PinpointOdometryPredictor predictor = predictor(device);
        ManualLoopClock time = new ManualLoopClock();
        device.status = GoBildaPinpointDriver.DeviceStatus.READY;
        device.position = pose(2.0, 3.0, 0.4);
        device.velocityX = 1.0;
        time.nextCycle(0.1);
        predictor.update(time.clock());

        device.resetObserver = () -> assertEquals(1L, predictor.trajectorySegmentId());
        predictor.resetPosAndIMU();
        assertEquals(1L, predictor.trajectorySegmentId());
        assertEquals(GoBildaPinpointDriver.DeviceStatus.NOT_READY,
                predictor.lastDeviceStatus());
        assertFalse(predictor.getEstimate().hasPose);
        assertFalse(predictor.getKinematicSnapshot().hasVelocity);
        assertEquals(0.0, predictor.getKinematicSnapshot().totalHeadingRad, EPSILON);

        predictor.setPose(new Pose2d(20.0, 30.0, 0.5));
        assertEquals(2L, predictor.trajectorySegmentId());
        assertTrue(predictor.getEstimate().hasPose);
        assertFalse(predictor.getEstimate().timestamp.isAvailable());
        assertFalse(predictor.getKinematicSnapshot().hasVelocity);
        assertFalse(predictor.getKinematicSnapshot().timestamp.isAvailable());
        assertFalse(predictor.getKinematicSnapshot().isCurrentFor(time.clock()));

        device.status = GoBildaPinpointDriver.DeviceStatus.READY;
        device.position = pose(20.0, 30.0, 0.5);
        time.nextCycle(0.1);
        predictor.update(time.clock());
        assertFalse(predictor.getLatestMotionDelta().hasDelta);

        device.recalibrateObserver = () -> assertEquals(3L, predictor.trajectorySegmentId());
        predictor.recalibrateIMU();
        assertEquals(3L, predictor.trajectorySegmentId());
        assertEquals(GoBildaPinpointDriver.DeviceStatus.NOT_READY,
                predictor.lastDeviceStatus());
        assertFalse(predictor.getEstimate().hasPose);
        assertFalse(predictor.getKinematicSnapshot().hasPose);
        assertEquals(1, device.recalibrateCount);
    }

    @Test
    public void readyReadFailureThenSetPoseUsesUnavailableTimestampAndFreshBaseline() {
        FakeDevice device = new FakeDevice();
        PinpointOdometryPredictor predictor = predictor(device);
        ManualLoopClock time = new ManualLoopClock();
        device.status = GoBildaPinpointDriver.DeviceStatus.READY;
        device.position = null;
        time.nextCycle(0.1);
        predictor.update(time.clock());
        assertEquals(GoBildaPinpointDriver.DeviceStatus.READY,
                predictor.lastDeviceStatus());
        assertFalse(predictor.getEstimate().hasPose);

        predictor.setPose(new Pose2d(70.0, 80.0, 0.7));
        assertTrue(predictor.getEstimate().hasPose);
        assertFalse(predictor.getEstimate().timestamp.isAvailable());
        assertFalse(predictor.getKinematicSnapshot().hasVelocity);

        device.position = pose(70.0, 80.0, 0.7);
        time.nextCycle(0.1);
        predictor.update(time.clock());
        assertFalse(predictor.getLatestMotionDelta().hasDelta);
        assertEquals(0.0, predictor.getKinematicSnapshot().totalHeadingRad, EPSILON);
    }

    @Test
    public void setPoseValidationPrecedesCacheAndVendorEffects() {
        FakeDevice device = new FakeDevice();
        PinpointOdometryPredictor predictor = predictor(device);
        int writesBefore = device.setPositionCount;

        List<Pose2d> invalid = Arrays.asList(
                new Pose2d(Double.NaN, 0.0, 0.0),
                new Pose2d(0.0, Double.POSITIVE_INFINITY, 0.0),
                new Pose2d(0.0, 0.0, Double.NEGATIVE_INFINITY),
                new Pose2d(Double.MAX_VALUE, 0.0, 0.0),
                new Pose2d(Double.MIN_VALUE, 0.0, 0.0),
                new Pose2d(0.0, -Double.MIN_VALUE, 0.0),
                new Pose2d(0.0, 0.0, Double.MIN_VALUE)
        );
        RuntimeException nullFailure = capture(() -> predictor.setPose(null));
        assertTrue(nullFailure instanceof NullPointerException);
        for (Pose2d pose : invalid) {
            RuntimeException failure = capture(() -> predictor.setPose(pose));
            assertTrue(failure instanceof IllegalArgumentException);
        }
        assertEquals(writesBefore, device.setPositionCount);
        assertFalse(predictor.getEstimate().hasPose);
        assertEquals(0L, predictor.trajectorySegmentId());

        predictor.setPose(new Pose2d(1.0, 2.0, Math.PI * 2.0));
        assertEquals(writesBefore + 1, device.setPositionCount);
        assertEquals(1L, predictor.trajectorySegmentId());
        assertEquals(0.0, predictor.getEstimate().fieldToRobotPose.yawRad, EPSILON);
    }

    @Test
    public void failedSetPoseFailsClosedWithoutChangingCachedStatus() {
        FakeDevice device = new FakeDevice();
        PinpointOdometryPredictor predictor = predictor(device);
        ManualLoopClock time = new ManualLoopClock();
        device.status = GoBildaPinpointDriver.DeviceStatus.READY;
        device.position = pose(1.0, 0.0, 0.0);
        time.nextCycle(0.1);
        predictor.update(time.clock());
        device.setPositionFailure = new IllegalStateException("write failed");
        device.setPositionObserver = () -> assertEquals(1L, predictor.trajectorySegmentId());

        RuntimeException failure = capture(() ->
                predictor.setPose(new Pose2d(10.0, 0.0, 0.0)));

        assertSame(device.setPositionFailure, failure);
        assertEquals(1L, predictor.trajectorySegmentId());
        assertEquals(GoBildaPinpointDriver.DeviceStatus.READY,
                predictor.lastDeviceStatus());
        assertFalse(predictor.getEstimate().hasPose);
        assertFalse(predictor.getKinematicSnapshot().hasPose);

        device.setPositionFailure = null;
        device.position = pose(100.0, 0.0, 0.0);
        time.nextCycle(0.1);
        predictor.update(time.clock());
        assertFalse(predictor.getLatestMotionDelta().hasDelta);
    }

    @Test
    public void failedResetAndRecalibrationAdvanceSegmentBeforeVendorEffects() {
        FakeDevice device = new FakeDevice();
        PinpointOdometryPredictor predictor = predictor(device);

        RuntimeException resetFailure = new IllegalStateException("reset failed after effect");
        device.resetFailure = resetFailure;
        device.resetObserver = () -> assertEquals(1L, predictor.trajectorySegmentId());

        RuntimeException observedResetFailure = capture(predictor::resetPosAndIMU);

        assertSame(resetFailure, observedResetFailure);
        assertEquals(1L, predictor.trajectorySegmentId());
        assertEquals(2, device.resetCount);
        assertFalse(predictor.getEstimate().hasPose);
        assertEquals(GoBildaPinpointDriver.DeviceStatus.NOT_READY,
                predictor.lastDeviceStatus());

        RuntimeException recalibrateFailure =
                new IllegalStateException("recalibration failed after effect");
        device.recalibrateFailure = recalibrateFailure;
        device.recalibrateObserver = () -> assertEquals(2L, predictor.trajectorySegmentId());

        RuntimeException observedRecalibrateFailure = capture(predictor::recalibrateIMU);

        assertSame(recalibrateFailure, observedRecalibrateFailure);
        assertEquals(2L, predictor.trajectorySegmentId());
        assertEquals(1, device.recalibrateCount);
        assertFalse(predictor.getEstimate().hasPose);
        assertEquals(GoBildaPinpointDriver.DeviceStatus.NOT_READY,
                predictor.lastDeviceStatus());
    }

    @Test
    public void thrownUpdateReplaysFailurePreservesStatusAndClearsMeasuredBaselines() {
        FakeDevice device = new FakeDevice();
        PinpointOdometryPredictor predictor = predictor(device);
        ManualLoopClock time = new ManualLoopClock();
        device.status = GoBildaPinpointDriver.DeviceStatus.READY;
        device.position = pose(0.0, 0.0, 0.0);
        time.nextCycle(0.1);
        predictor.update(time.clock());

        RuntimeException pollFailure = new IllegalStateException("poll failed");
        device.updateFailure = pollFailure;
        time.nextCycle(0.1);
        RuntimeException first = capture(() -> predictor.update(time.clock()));
        int updatesAfterFailure = device.updateCount;
        RuntimeException repeated = capture(() -> predictor.update(time.clock()));
        assertSame(pollFailure, first);
        assertSame(pollFailure, repeated);
        assertEquals(updatesAfterFailure, device.updateCount);
        assertEquals(GoBildaPinpointDriver.DeviceStatus.READY,
                predictor.lastDeviceStatus());

        device.updateFailure = null;
        device.position = pose(90.0, 0.0, 2.0);
        time.nextCycle(0.1);
        predictor.update(time.clock());
        assertFalse(predictor.getLatestMotionDelta().hasDelta);
        assertEquals(0.0, predictor.getKinematicSnapshot().totalHeadingRad, EPSILON);
    }

    @Test
    public void debugDumpUsesCachedStatusWithoutAnotherDeviceRead() {
        FakeDevice device = new FakeDevice();
        PinpointOdometryPredictor predictor = predictor(device);
        ManualLoopClock time = new ManualLoopClock();
        device.status = GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        time.nextCycle(0.1);
        predictor.update(time.clock());
        int statusReads = device.statusReadCount;
        RecordingDebugSink debug = new RecordingDebugSink();

        predictor.debugDump(debug, "odo");

        assertEquals(statusReads, device.statusReadCount);
        assertTrue(debug.entries.contains("odo.driverStatus=CALIBRATING"));
        assertTrue(debug.entries.stream().anyMatch(value ->
                value.startsWith("odo.cfg.encoderResolution=")));
    }

    private static PinpointOdometryPredictor predictor(FakeDevice device) {
        return new PinpointOdometryPredictor(
                new RecordingLookup(device),
                PinpointOdometryPredictor.Config.defaults()
        );
    }

    private static Pose2D pose(double xInches, double yInches, double headingRad) {
        return new Pose2D(
                DistanceUnit.INCH,
                xInches,
                yInches,
                AngleUnit.RADIANS,
                headingRad
        );
    }

    private static RuntimeException capture(Runnable action) {
        try {
            action.run();
            fail("Expected failure");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static final class RecordingLookup
            implements PinpointOdometryPredictor.PinpointDeviceLookup {
        private final FakeDevice device;
        int lookupCount;
        String lastName;

        private RecordingLookup(FakeDevice device) {
            this.device = device;
        }

        @Override
        public PinpointOdometryPredictor.PinpointDevice get(String hardwareMapName) {
            lookupCount++;
            lastName = hardwareMapName;
            return device;
        }
    }

    private static final class FakeDevice implements PinpointOdometryPredictor.PinpointDevice {
        final List<String> effects = new ArrayList<>();
        GoBildaPinpointDriver.DeviceStatus status =
                GoBildaPinpointDriver.DeviceStatus.NOT_READY;
        Pose2D position = pose(0.0, 0.0, 0.0);
        double velocityX;
        double velocityY;
        double headingVelocity;
        RuntimeException customResolutionFailure;
        RuntimeException updateFailure;
        RuntimeException setPositionFailure;
        RuntimeException resetFailure;
        RuntimeException recalibrateFailure;
        Runnable resetObserver;
        Runnable recalibrateObserver;
        Runnable setPositionObserver;
        int resetCount;
        int recalibrateCount;
        int updateCount;
        int statusReadCount;
        int positionReadCount;
        int setPositionCount;

        @Override
        public void setOffsetsInches(double forwardPodOffsetLeftInches,
                                     double strafePodOffsetForwardInches) {
            effects.add("offsets:" + forwardPodOffsetLeftInches + ','
                    + strafePodOffsetForwardInches);
        }

        @Override
        public void setYawScalar(double yawScalar) {
            effects.add("yaw:" + yawScalar);
        }

        @Override
        public void setGoBildaEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods pod) {
            effects.add("podResolution:" + pod);
        }

        @Override
        public void setCustomEncoderResolutionTicksPerInch(double ticksPerInch) {
            effects.add("customResolution:" + ticksPerInch);
            if (customResolutionFailure != null) {
                throw customResolutionFailure;
            }
        }

        @Override
        public void setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection forwardDirection,
                GoBildaPinpointDriver.EncoderDirection strafeDirection) {
            effects.add("directions:" + forwardDirection + ',' + strafeDirection);
        }

        @Override
        public void resetPosAndIMU() {
            effects.add("reset");
            resetCount++;
            if (resetObserver != null) {
                resetObserver.run();
            }
            if (resetFailure != null) {
                throw resetFailure;
            }
        }

        @Override
        public void recalibrateIMU() {
            recalibrateCount++;
            if (recalibrateObserver != null) {
                recalibrateObserver.run();
            }
            if (recalibrateFailure != null) {
                throw recalibrateFailure;
            }
        }

        @Override
        public void update() {
            updateCount++;
            if (updateFailure != null) {
                throw updateFailure;
            }
        }

        @Override
        public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
            statusReadCount++;
            return status;
        }

        @Override
        public Pose2D getPosition() {
            positionReadCount++;
            return position;
        }

        @Override
        public double getVelXInchesPerSec() {
            return velocityX;
        }

        @Override
        public double getVelYInchesPerSec() {
            return velocityY;
        }

        @Override
        public double getHeadingVelocityRadPerSec() {
            return headingVelocity;
        }

        @Override
        public void setPosition(Pose2D pose) {
            setPositionCount++;
            if (setPositionObserver != null) {
                setPositionObserver.run();
            }
            if (setPositionFailure != null) {
                throw setPositionFailure;
            }
            position = pose;
        }
    }

    private static final class RecordingDebugSink implements DebugSink {
        final List<String> entries = new ArrayList<>();

        @Override
        public DebugSink addData(String key, Object value) {
            entries.add(key + '=' + value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            entries.add(text);
            return this;
        }
    }
}
