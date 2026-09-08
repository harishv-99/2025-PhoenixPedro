package edu.ftcsushi.fw.tools.tester.calibration;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibratorTimingTest.Fixture;
import edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibratorTimingTest.ScriptedCamera;

import static edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibratorTimingTest.field;
import static edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibratorTimingTest.invoke;
import static edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibratorTimingTest.setField;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/**
 * CAL-06 maintainer evidence: the real calibrator matches real timestamped AprilTag solves against
 * real raw-Pinpoint history. The shared fixture replaces hardware acquisition, camera observations,
 * and drive outputs only; it never manufactures a matched endpoint or edits a solved tag estimate.
 * Independent planar geometry below distinguishes capture-time arithmetic from delivery-time
 * motion and differently oriented coordinate systems. This is not physical latency or CAL-08 pod
 * geometry validation.
 */
public final class PinpointPodOffsetCalibratorEvidenceTest {
    private static final double EPS = 1e-8;

    @Test
    public void bothSearchesCountDistinctCaptureTimesNotLoopsObjectsOrChangedCoordinates()
            throws Exception {
        for (boolean end : new boolean[]{false, true}) {
            Fixture f = search(end);
            double base = f.clock.nowSec();
            freshFrame(f, base + 0.02, Pose2d.zero());
            LoopTimestamp equalTimeNewObject = f.clock.nowTimestamp();
            assertEquals(1, count(f));

            for (int repeat = 1; repeat <= 5; repeat++) {
                // A newly allocated frame and changed geometry still have the same capture time.
                f.frameAt(equalTimeNewObject, new Pose2d(repeat, 0.0, 0.0));
                f.loopAt(base + 0.02 + repeat * 0.02);
                assertEquals(1, count(f));
                assertEquals(end ? "SEARCH_TAG_END" : "SEARCH_TAG_START", f.phase());
                f.owner.loop(100.0);
                assertEquals(1, count(f));
            }

            freshFrame(f, base + 0.14, Pose2d.zero());
            assertEquals(2, count(f));
            freshFrame(f, base + 0.16, Pose2d.zero());
            assertEquals(end ? "IDLE" : "ROTATING", f.phase());
            if (end) assertResult(f);
            else assertEquals("assisted start must not write a historical pose to Pinpoint",
                    0, f.device.rebases);
        }
    }

    @Test
    public void missingAndOutOfOrderFramesBreakStreakWithoutRecountingConsumedTime()
            throws Exception {
        Fixture f = search(false);
        freshFrame(f, 0.02, Pose2d.zero());
        LoopTimestamp first = f.clock.nowTimestamp();
        freshFrame(f, 0.04, Pose2d.zero());
        LoopTimestamp second = f.clock.nowTimestamp();
        assertEquals(2, count(f));
        f.noFrame();
        f.loopAt(0.06);
        assertEquals(0, count(f));
        f.frameAt(second, Pose2d.zero());
        f.loopAt(0.08);
        assertEquals(0, count(f));
        f.frameAt(first, Pose2d.zero());
        f.loopAt(0.10);
        assertEquals(0, count(f));
        freshFrame(f, 0.12, Pose2d.zero());
        assertEquals(1, count(f));
        freshFrame(f, 0.14, Pose2d.zero());
        assertEquals(2, count(f));
        freshFrame(f, 0.16, Pose2d.zero());
        assertEquals("ROTATING", f.phase());
    }

    @Test
    public void staleRetainedFrameBreaksStreakRatherThanBecomingNewEvidence() throws Exception {
        Fixture f = search(false);
        freshFrame(f, 0.02, Pose2d.zero());
        assertEquals(1, count(f));
        f.loopAt(0.70);
        assertEquals("SEARCH_TAG_START", f.phase());
        assertEquals(0, count(f));
        assertNull(field(f.owner, "startAssistEndpoint"));
        freshFrame(f, 0.72, Pose2d.zero());
        assertEquals(1, count(f));
    }

    @Test
    public void directVisibleEndpointsUseOneMatchedFrameEvenWhenSearchCountIsThree()
            throws Exception {
        Fixture f = new Fixture(true, true);
        f.startAutoWithTag();
        assertEquals("ROTATING", f.phase());
        assertEquals(0, f.device.rebases);
        f.clock.update(0.10);
        f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(
                new Pose2d(4.0, 4.0, Math.PI));
        f.frameAt(f.clock.nowTimestamp(), new Pose2d(0.0, 0.0, Math.PI));
        f.owner.loop(f.clock.dtSec());
        assertEquals("IDLE", f.phase());
        assertResult(f);
        assertEquals(-2.0, result(f, "lastRecommendedStrafePodOffsetForwardInches"), EPS);
        assertEquals(-2.0, result(f, "lastRecommendedForwardPodOffsetLeftInches"), EPS);
        assertEquals(0, f.device.rebases);
    }

    @Test
    public void exactAndInterpolatedCapturesCompareTheirOwnStartBodyFramesNotDeliveryPoses()
            throws Exception {
        for (boolean interpolate : new boolean[]{false, true}) {
            Fixture f = new Fixture(true, false, config -> config.enablePostRotateRecenter = false);
            // Odom start = (10,-5,90deg), tag start = (100,50,-90deg). Their field axes differ.
            f.poseAt(0.02, new Pose2d(10.0, interpolate ? -6.0 : -5.0,
                    Math.PI / 2.0 - (interpolate ? 0.10 : 0.0)));
            LoopTimestamp exactStart = f.clock.nowTimestamp();
            f.clock.update(0.10);
            LoopTimestamp startCapture = interpolate
                    ? f.clock.timestampSecondsAgo(0.04) : exactStart;
            f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(
                    new Pose2d(10.0, -4.0, Math.PI / 2.0 + 0.10));
            f.frameAt(startCapture, new Pose2d(100.0, 50.0, -Math.PI / 2.0));
            f.queuePrimary();
            f.owner.loop(f.clock.dtSec());
            assertEquals("ROTATING", f.phase());
            assertEquals(0, f.device.rebases);

            f.noFrame();
            // The experimental start remains fixed even after its live history sample is evicted.
            f.poseAt(1.00, new Pose2d(2.0, interpolate ? 0.0 : 1.0,
                    -Math.PI / 2.0 - (interpolate ? 0.10 : 0.0)));
            LoopTimestamp exactEnd = f.clock.nowTimestamp();
            PlanarPoseHistory history = (PlanarPoseHistory) field(f.owner, "assistOdometryHistory");
            assertFalse(history.lookupSource().getAt(f.clock, startCapture).isAvailable());
            f.clock.update(1.08);
            LoopTimestamp endCapture = interpolate
                    ? f.clock.timestampSecondsAgo(0.04) : exactEnd;
            f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(
                    new Pose2d(2.0, 2.0, -Math.PI / 2.0 + 0.10));
            f.frameAt(endCapture, new Pose2d(104.0, 48.0, Math.PI / 2.0));
            f.queuePrimary();
            f.owner.loop(f.clock.dtSec());

            assertResult(f);
            // Odom field delta (-8,+6) -> own start axes (+6,+8).
            // Tag field delta (+4,-2) -> own start axes (+2,+4). Residual = (+4,+4).
            assertEquals(4.0, result(f, "lastDxStartBodyInches"), EPS);
            assertEquals(4.0, result(f, "lastDyStartBodyInches"), EPS);
            assertEquals(Math.PI, Math.abs(result(f, "lastDeltaHeadingRad")), EPS);
            // At a half turn, each reported offset error is half that residual.
            assertEquals(-2.0, result(f, "lastRecommendedStrafePodOffsetForwardInches"), EPS);
            assertEquals(-2.0, result(f, "lastRecommendedForwardPodOffsetLeftInches"), EPS);
            assertEquals(0, f.device.rebases);
        }
    }

    @Test
    public void startCaptureInterpolatesAcrossYawWrapWithoutMovingAutomaticTurnBoundary()
            throws Exception {
        Fixture f = new Fixture(true, true, config -> config.targetTurnRad = Math.PI / 2.0);
        f.poseAt(0.02, new Pose2d(0.0, 0.0, Math.PI - 0.20));
        f.clock.update(0.10);
        f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(
                new Pose2d(0.0, 0.0, -Math.PI + 0.20));
        f.frameAt(f.clock.timestampSecondsAgo(0.04), new Pose2d(0.0, 0.0, Math.PI));
        f.queueAuto();
        f.owner.loop(f.clock.dtSec());
        assertEquals("ROTATING", f.phase());
        f.noFrame();
        f.poseAt(0.20, new Pose2d(0.0, 0.0, -Math.PI / 2.0));
        assertEquals("capture yaw pi must not start the control turn early", "ROTATING", f.phase());
        f.poseAt(0.30, new Pose2d(0.0, 0.0, -Math.PI / 2.0 + 0.21));
        assertEquals("SEARCH_TAG_END", f.phase());
    }

    @Test
    public void solveHeadingUsesCaptureEndpointsRatherThanCurrentTurnProgress() throws Exception {
        Fixture f = new Fixture(true, true, config -> {
            config.targetTurnRad = Math.PI / 2.0;
            config.enableAutoTagSearchAtEnd = false;
        });
        f.poseAt(0.02, Pose2d.zero());
        LoopTimestamp startCapture = f.clock.nowTimestamp();
        f.clock.update(0.10);
        f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(new Pose2d(0.0, 0.0, 0.50));
        f.frameAt(startCapture, Pose2d.zero());
        f.queueAuto();
        f.owner.loop(f.clock.dtSec());
        f.noFrame();
        f.poseAt(0.20, new Pose2d(0.0, 0.0, Math.PI / 2.0));
        assertEquals("ROTATING", f.phase());
        double captureHeading = Math.PI / 2.0 + 0.40;
        f.poseAt(0.28, new Pose2d(3.0, 2.0, captureHeading));
        LoopTimestamp endCapture = f.clock.nowTimestamp();
        f.clock.update(0.30);
        f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(
                new Pose2d(4.0, 3.0, Math.PI / 2.0 + 0.51));
        f.frameAt(endCapture, new Pose2d(0.0, 0.0, captureHeading));
        f.owner.loop(f.clock.dtSec());
        assertResult(f);
        assertEquals(captureHeading, result(f, "lastDeltaHeadingRad"), EPS);
        assertEquals(3.0, result(f, "lastDxStartBodyInches"), EPS);
        assertEquals(2.0, result(f, "lastDyStartBodyInches"), EPS);
    }

    @Test
    public void directStartCannotUseCurrentPoseWhenCaptureHasNoHistoryBracket() throws Exception {
        for (boolean powered : new boolean[]{false, true}) {
            Fixture f = new Fixture(true, powered, config -> config.enableAutoTagSearchAtStart = false);
            f.owner.loop(0.0);
            f.clock.update(0.20);
            f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(new Pose2d(4.0, 0.0, 0.0));
            f.frameAt(f.clock.timestampSecondsAgo(0.10), Pose2d.zero());
            f.queuePrimary();
            f.owner.loop(f.clock.dtSec());
            assertDiscarded(f);
            assertEquals(0, f.device.rebases);

            f.clock.update(0.22);
            f.frameAt(f.clock.nowTimestamp(), Pose2d.zero());
            f.queuePrimary();
            f.owner.loop(f.clock.dtSec());
            assertEquals("ROTATING", f.phase());
            assertEquals(0, f.device.rebases);
        }
    }

    @Test
    public void missingStalePrehistoryPriorEpochAndFutureStartsStayUnavailable()
            throws Exception {
        for (int condition = 0; condition < 5; condition++) {
            Fixture f = new Fixture(true, false);
            // none() is the public unavailable-time observation; fromFrame rejects unavailable.
            if (condition == 1) {
                f.frameAt(f.clock.nowTimestamp(), Pose2d.zero());
                f.clock.update(0.70);
            }
            if (condition == 2) f.frameAt(f.clock.timestampSecondsAgo(0.10), Pose2d.zero());
            if (condition == 3) {
                f.frameAt(f.clock.nowTimestamp(), Pose2d.zero());
                f.clock.reset(0.0);
            }
            if (condition == 4) {
                f.clock.update(0.20);
                f.frameAt(f.clock.nowTimestamp(), Pose2d.zero());
                f.clock.update(0.10);
            }
            f.queuePrimary();
            f.owner.loop(f.clock.dtSec());
            assertDiscarded(f);
            assertEquals(0, f.device.rebases);
        }
    }

    @Test
    public void wrongClockCameraEvidenceFailsClosedBeforeQueuedMotion() throws Exception {
        Fixture f = new Fixture(true, true);
        LoopClock otherClock = new LoopClock();
        otherClock.reset(0.0);
        f.frameAt(otherClock.nowTimestamp(), Pose2d.zero());
        f.queuePrimary();
        f.queueAuto();
        f.owner.loop(0.0);
        assertEquals("IDLE", f.phase());
        assertNull(field(f.owner, "startAssistEndpoint"));
        assertNull(field(f.owner, "lastRecommendedStrafePodOffsetForwardInches"));
        assertTrue((Boolean) field(f.owner, "visionRetryBlocked"));
        assertNotNull(field(f.owner, "visionFailure"));
        f.assertOnlyZeroCommandsSince(0);
    }

    @Test
    public void sameTimeChangedRawCoordinatesInvalidateBeforeQueuedAAndY() throws Exception {
        Fixture f = new Fixture(true, true);
        f.startAutoWithTag();
        long segment = f.predictor.trajectorySegmentId();
        int commands = f.commands.size();
        f.clock.update(f.clock.nowSec());
        f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(new Pose2d(2.0, 0.0, 0.0));
        f.frameAt(f.clock.nowTimestamp(), Pose2d.zero());
        f.queuePrimary();
        f.queueAuto();
        f.owner.loop(0.0);
        assertEquals("this is not a setPose/segment-reset surrogate", segment,
                f.predictor.trajectorySegmentId());
        assertDiscarded(f);
        assertTrue(((String) field(f.owner, "lastAttemptFailure")).contains("continuity"));
        assertNull(field(f.owner, "startAssistEndpoint"));
        f.assertOnlyZeroCommandsSince(commands);
    }

    @Test
    public void retainedStartFrameCannotAlsoServeAsTheEndOrFallBackToUncorrectedSolve()
            throws Exception {
        Fixture f = new Fixture(true, true, config -> config.enableAutoTagSearchAtEnd = false);
        LoopTimestamp start = f.clock.nowTimestamp();
        f.startAutoWithTag();
        f.frameAt(start, new Pose2d(0.0, 0.0, Math.PI));
        f.poseAt(0.10, new Pose2d(4.0, 4.0, Math.PI));
        assertDiscarded(f);
        assertEquals(0, f.device.rebases);
    }

    @Test
    public void unmatchedEndCannotBeRescuedByCurrentOdometryOrDisabledRecenter() throws Exception {
        for (boolean recenter : new boolean[]{false, true}) {
            Fixture f = new Fixture(true, true, config -> {
                config.enableAutoTagSearchAtEnd = false;
                config.enablePostRotateRecenter = recenter;
            });
            f.startAutoWithTag();
            f.clock.update(0.20);
            f.frameAt(f.clock.timestampSecondsAgo(0.10), new Pose2d(0.0, 0.0, Math.PI));
            f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(
                    new Pose2d(4.0, 4.0, Math.PI));
            f.owner.loop(f.clock.dtSec());
            assertDiscarded(f);
        }
    }

    @Test
    public void manualAndDeferredAutoRecenterAcquireFinalEndpointWithoutEarlyEvidence()
            throws Exception {
        for (int scenario = 0; scenario < 4; scenario++) {
            boolean auto = scenario >= 2;
            boolean finalFrame = scenario % 2 == 1;
            Fixture f = new Fixture(true, auto, config -> config.autoComputeAfterAutoSample = false);
            f.frameAt(f.clock.nowTimestamp(), Pose2d.zero());
            if (auto) f.queueAuto();
            else f.queuePrimary();
            f.owner.loop(0.0);
            assertEquals("ROTATING", f.phase());
            f.noFrame();
            f.clock.update(0.10);
            f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(new Pose2d(8.0, 8.0, Math.PI));
            if (!auto) f.queuePrimary();
            f.owner.loop(f.clock.dtSec());
            assertEquals("POST_RECENTER", f.phase());
            assertNull(field(f.owner, "endAssistEndpoint"));
            f.clock.update(0.20);
            f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(new Pose2d(4.0, 4.0, Math.PI));
            if (finalFrame) f.frameAt(f.clock.nowTimestamp(), new Pose2d(0.0, 0.0, Math.PI));
            f.queuePrimary();
            f.owner.loop(f.clock.dtSec());
            if (finalFrame) {
                assertResult(f);
                assertEquals(4.0, result(f, "lastDxStartBodyInches"), EPS);
            } else assertDiscarded(f);
        }
    }

    @Test
    public void rawTrajectoryClockEpochAndCameraReplacementInvalidateFrozenStart() throws Exception {
        for (int change = 0; change < 3; change++) {
            Fixture f = new Fixture(true, false, config -> config.enablePostRotateRecenter = false);
            f.frameAt(f.clock.nowTimestamp(), Pose2d.zero());
            f.queuePrimary();
            f.owner.loop(0.0);
            assertEquals("ROTATING", f.phase());
            if (change == 0) f.predictor.setPose(Pose2d.zero());
            if (change == 1) f.clock.reset(0.05);
            if (change == 2) {
                f.camera.owned.close();
                f.camera = new ScriptedCamera();
                f.installCamera();
            }
            f.clock.update(0.10);
            f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(new Pose2d(4.0, 4.0, Math.PI));
            f.frameAt(f.clock.nowTimestamp(), new Pose2d(0.0, 0.0, Math.PI));
            f.queuePrimary();
            f.owner.loop(f.clock.dtSec());
            assertDiscarded(f);
        }
    }

    @Test
    public void readyGapAbortsBeforeLateEndAndFreshAttemptNeedsItsOwnStart() throws Exception {
        Fixture f = new Fixture(true, false, config -> config.enablePostRotateRecenter = false);
        f.frameAt(f.clock.nowTimestamp(), Pose2d.zero());
        f.queuePrimary();
        f.owner.loop(0.0);
        f.device.status = GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        f.noFrame();
        f.loopAt(0.02);
        assertEquals("IDLE", f.phase());
        assertNull(field(f.owner, "startAssistEndpoint"));
        f.device.status = GoBildaPinpointDriver.DeviceStatus.READY;
        f.clock.update(0.04);
        f.frameAt(f.clock.nowTimestamp(), new Pose2d(0.0, 0.0, Math.PI));
        f.owner.loop(f.clock.dtSec());
        assertEquals("IDLE", f.phase());
        assertNull(field(f.owner, "lastRecommendedStrafePodOffsetForwardInches"));
        f.clock.update(0.06);
        f.frameAt(f.clock.nowTimestamp(), Pose2d.zero());
        f.queuePrimary();
        f.owner.loop(f.clock.dtSec());
        assertEquals("ROTATING", f.phase());
    }

    @Test
    public void abortResetStopAndCameraFailureClearPartiallyAcquiredEvidence() throws Exception {
        for (int ending = 0; ending < 4; ending++) {
            Fixture f = search(false);
            freshFrame(f, 0.02, Pose2d.zero());
            freshFrame(f, 0.04, Pose2d.zero());
            assertEquals(2, count(f));
            if (ending == 0) invoke(f.owner, "abortSample");
            if (ending == 1) setField(f.owner, "resetRequested", true);
            if (ending == 2) f.owner.stop();
            if (ending == 3) f.camera.failure = new IllegalStateException("camera lost");
            if (ending != 2) f.loopAt(0.06);
            assertEquals("IDLE", f.phase());
            assertEquals(0, count(f));
            assertNull(field(f.owner, "startAssistEndpoint"));
            assertNull(field(f.owner, "endAssistEndpoint"));
            f.assertStopped();
        }
    }

    @Test
    public void explicitStartSkipKeepsNoTagWorkflowButEndSkipDiscardsAssistedAttempt()
            throws Exception {
        Fixture start = search(false);
        start.queuePrimary();
        start.loopAt(0.02);
        assertEquals("ROTATING", start.phase());
        assertNull(field(start.owner, "startAssistEndpoint"));
        assertEquals(1, start.device.rebases);
        assertFalse("explicit no-tag choice must not promise assisted auto-compute",
                start.telemetry.stream().anyMatch(row -> row.contains("will auto-compute")));
        start.poseAt(0.04, new Pose2d(4.0, 4.0, Math.PI));
        assertEquals("POST_RECENTER", start.phase());
        start.queuePrimary();
        start.loopAt(0.06);
        assertResult(start);

        Fixture end = search(true);
        end.queuePrimary();
        end.loopAt(0.04);
        assertDiscarded(end);
    }

    @Test
    public void angularExhaustionBeatsAnOtherwiseCompletingDistinctFrameInBothSearches()
            throws Exception {
        for (boolean end : new boolean[]{false, true}) {
            for (double extra : new double[]{0.0, 0.10}) {
                Fixture f = new Fixture(true, true, config -> {
                    config.tagSearchMaxTurnRad = 1.0;
                    config.tagEndSearchMaxExtraTurnRad = 1.0;
                });
                double startHeading = end ? Math.PI : 0.0;
                if (end) {
                    f.startAutoWithTag();
                    f.poseAt(0.02, new Pose2d(0.0, 0.0, startHeading));
                } else {
                    f.queueAuto();
                    f.owner.loop(0.0);
                }
                double enteredAt = f.clock.nowSec();
                freshFrame(f, enteredAt + 0.02, Pose2d.zero());
                freshFrame(f, enteredAt + 0.04, Pose2d.zero());
                assertEquals(2, count(f));
                int commands = f.commands.size();
                f.clock.update(enteredAt + 0.06);
                f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(
                        new Pose2d(0.0, 0.0, startHeading + 1.0 + extra));
                f.frameAt(f.clock.nowTimestamp(), Pose2d.zero());
                f.owner.loop(f.clock.dtSec());
                assertDiscarded(f);
                assertTrue(((String) field(f.owner, "lastAttemptFailure")).contains("angular limit"));
                assertEquals(0, count(f));
                f.assertOnlyZeroCommandsSince(commands);
            }
        }
    }

    @Test
    public void finalDistinctFrameAtDeadlineCannotRescueEndSearchOrPollAgain() throws Exception {
        Fixture f = search(true);
        freshFrame(f, 0.04, new Pose2d(0.0, 0.0, Math.PI));
        freshFrame(f, 0.06, new Pose2d(0.0, 0.0, Math.PI));
        assertEquals(2, count(f));
        int polls = f.device.polls;
        int cameraPolls = f.camera.polls;
        f.clock.update(10.02);
        f.frameAt(f.clock.nowTimestamp(), new Pose2d(0.0, 0.0, Math.PI));
        f.queuePrimary();
        f.queueAuto();
        f.owner.loop(f.clock.dtSec());
        assertDiscarded(f);
        assertEquals(polls, f.device.polls);
        assertEquals(cameraPolls, f.camera.polls);
        assertTrue(((String) field(f.owner, "lastAttemptFailure")).contains("timed out"));
        assertNull(field(f.owner, "startAssistEndpoint"));
    }

    @Test
    public void independentNoVisionManualRecenterStillProducesItsOwnResult() throws Exception {
        Fixture f = new Fixture(false, false);
        f.queuePrimary();
        f.owner.loop(0.0);
        assertEquals(1, f.device.rebases);
        f.clock.update(100.0);
        f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(new Pose2d(8.0, 8.0, Math.PI));
        f.queuePrimary();
        f.owner.loop(f.clock.dtSec());
        assertEquals("POST_RECENTER", f.phase());
        f.clock.update(200.0);
        f.device.pose = PinpointPodOffsetCalibratorTimingTest.sdkPose(new Pose2d(4.0, 4.0, Math.PI));
        f.queuePrimary();
        f.owner.loop(f.clock.dtSec());
        assertResult(f);
        assertEquals(-2.0, result(f, "lastRecommendedStrafePodOffsetForwardInches"), EPS);
        assertEquals(-2.0, result(f, "lastRecommendedForwardPodOffsetLeftInches"), EPS);
        assertTrue(f.commands.isEmpty());
        assertNull(field(f.owner, "assistOdometryHistory"));
    }

    private static Fixture search(boolean end) throws Exception {
        Fixture f = new Fixture(true, true);
        if (end) {
            f.startAutoWithTag();
            f.poseAt(0.02, new Pose2d(0.0, 0.0, Math.PI));
        } else {
            f.queueAuto();
            f.owner.loop(0.0);
        }
        assertEquals(end ? "SEARCH_TAG_END" : "SEARCH_TAG_START", f.phase());
        return f;
    }

    private static void freshFrame(Fixture f, double timeSec, Pose2d tagRobotPose) {
        f.clock.update(timeSec);
        f.frameAt(f.clock.nowTimestamp(), tagRobotPose);
        f.owner.loop(f.clock.dtSec());
    }

    private static int count(Fixture f) throws Exception {
        return (Integer) field(f.owner, "tagStableFrames");
    }

    private static double result(Fixture f, String name) throws Exception {
        return (Double) field(f.owner, name);
    }

    private static void assertResult(Fixture f) throws Exception {
        assertEquals("IDLE", f.phase());
        assertNull(field(f.owner, "lastAttemptFailure"));
        assertNotNull(field(f.owner, "lastRecommendedStrafePodOffsetForwardInches"));
        assertNotNull(field(f.owner, "lastRecommendedForwardPodOffsetLeftInches"));
        f.assertStopped();
    }

    private static void assertDiscarded(Fixture f) throws Exception {
        assertEquals("IDLE", f.phase());
        assertNotNull("a failed assisted experiment needs an actionable retained reason",
                field(f.owner, "lastAttemptFailure"));
        assertNull(field(f.owner, "lastRecommendedStrafePodOffsetForwardInches"));
        assertNull(field(f.owner, "lastRecommendedForwardPodOffsetLeftInches"));
        f.assertStopped();
    }
}
