package edu.ftcsushi.fw.tools.tester.calibration;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibratorTimingTest.Fixture;

import static edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibratorTimingTest.field;
import static edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibratorTimingTest.invoke;
import static edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibratorTimingTest.sdkPose;
import static edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibratorTimingTest.setField;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/**
 * CAL-08 independent geometry and sample-boundary regressions.
 *
 * <p>The real calibrator, Pinpoint predictor and clock run over the existing scripted device seam.
 * Pending A/Y intent substitutes only for gamepad binding edges; this is not an INIT/hardware test.
 * Chosen true geometry is the oracle, not a second copy of the production inverse solve. Synthetic
 * travel does not establish physical pod resolution, heading accuracy, slip or calibration safety.</p>
 */
public final class PinpointPodOffsetCalibratorGeometryTest {
    private static final double EPS = 1e-9;
    // Bound numerical quadrature error, not physical calibration tolerance.
    private static final double MODEL_EPS = 1e-7;
    private static final Geometry MIXED = new Geometry(6.0, -4.0, 2.0, 1.0);

    @Test
    public void finalAWithoutRecenterUsesThisPollHeadingAndPosition() throws Exception {
        assertFinalACurrentHeading(false);
    }

    @Test
    public void finalAAfterRecenterUsesThisPollHeadingAndPosition() throws Exception {
        assertFinalACurrentHeading(true);
    }

    @Test
    public void manualAStartDoesNotFeedThePreRebaseHeadingIntoTheNewSample() throws Exception {
        Fixture f = configured(false, true);
        f.poseAt(0.01, new Pose2d(20.0, -8.0, 2.0 * Math.PI / 3.0));
        f.queuePrimary();
        f.loopAt(0.02);

        assertEquals("ROTATING", f.phase());
        assertEquals(1, f.device.rebases);
        assertEquals(0.0, result(f, "startHeadingUnwrappedRad"), 0.0);
        assertEquals("the sample's software zero must survive its entry loop", 0.0,
                (Double) invoke(field(f.owner, "headingUnwrapper"), "getUnwrappedRad"), 0.0);
        assertNull(field(f.owner, "lastRecommendedStrafePodOffsetForwardInches"));
    }

    @Test
    public void automaticYStartAtOldHeadingDoesNotImmediatelyCompleteTheNewTurn() throws Exception {
        Fixture f = configured(true, true);
        f.poseAt(0.01, new Pose2d(20.0, -8.0, 2.0 * Math.PI / 3.0));
        f.queueAuto();
        f.loopAt(0.02);

        assertEquals("the pre-reset 120 degrees is not progress toward the new 90-degree turn",
                "ROTATING", f.phase());
        assertEquals(1, f.device.rebases);
        assertEquals(0.0,
                (Double) invoke(field(f.owner, "headingUnwrapper"), "getUnwrappedRad"), 0.0);
        f.assertPowered();
        f.poseAt(0.03, Pose2d.zero());
        assertEquals("ROTATING", f.phase());
        f.assertPowered();
    }

    @Test
    public void forwardWheelTravelRecoversChosenGeometryForBothTurnDirections() throws Exception {
        Geometry[] geometries = {MIXED, new Geometry(-3.5, 7.0, 4.0, -2.0),
                new Geometry(-5.0, 3.0, -5.0, 3.0)};
        for (Geometry geometry : geometries) {
            for (double turn : new double[]{Math.PI / 2.0, -Math.PI / 2.0,
                    Math.PI, -Math.PI, 1.20, -1.20, 2.40, -2.40}) {
                Fixture f = manual(geometry, false);
                startManual(f);
                finishManualTurn(f, geometry, turn, 0.0, 0.0);
                assertRecovered(f, geometry);
                assertEquals(turn, result(f, "lastDeltaHeadingRad"), EPS);
            }
        }
    }

    @Test
    public void alreadyCorrectOffCenterPodsNeedNoOffsetChange() throws Exception {
        Geometry correct = new Geometry(6.0, -4.0, 6.0, -4.0);
        Fixture f = manual(correct, false);
        startManual(f);
        finishManualTurn(f, correct, -2.10, 0.0, 0.0);

        assertRecovered(f, correct);
        assertEquals(0.0, result(f, "lastDxStartBodyInches"), MODEL_EPS);
        assertEquals(0.0, result(f, "lastDyStartBodyInches"), MODEL_EPS);
        assertEquals(0.0, result(f, "lastXErrorInches"), MODEL_EPS);
        assertEquals(0.0, result(f, "lastYErrorInches"), MODEL_EPS);
    }

    @Test
    public void manualRecenterRemovesRealTranslationButRetainsGeometryResidual() throws Exception {
        for (double turn : new double[]{Math.PI / 2.0, -Math.PI / 2.0}) {
            Fixture f = manual(MIXED, true);
            startManual(f);
            publishTurn(f, MIXED, turn, 3.0, -2.0);
            f.queuePrimary();
            nextPose(f, MIXED.reported(turn, 3.0, -2.0));
            assertEquals("POST_RECENTER", f.phase());
            // Translate the same robot reference point back, without changing its final facing.
            f.queuePrimary();
            nextPose(f, MIXED.reported(turn, 0.0, 0.0));

            assertRecovered(f, MIXED);
            assertEquals(turn, result(f, "lastDeltaHeadingRad"), EPS);
        }
    }

    @Test
    public void unaccountedRealTranslationContaminatesAnOtherwiseValidRecommendation()
            throws Exception {
        Fixture f = manual(MIXED, false);
        startManual(f);
        finishManualTurn(f, MIXED, Math.PI, 4.0, 6.0);

        assertCompleted(f);
        // At a half turn, real +4 forward/+6 left is indistinguishable from +2/+3 offset error.
        // The true geometry is (6,-4), but the uncorrected sample necessarily recommends (4,-7).
        assertEquals(4.0, result(f, "lastRecommendedStrafePodOffsetForwardInches"), MODEL_EPS);
        assertEquals(-7.0, result(f, "lastRecommendedForwardPodOffsetLeftInches"), MODEL_EPS);
        assertTrue(Math.abs(result(f, "lastRecommendedStrafePodOffsetForwardInches")
                - MIXED.trueStrafeForward) > 1.0);
    }

    @Test
    public void assistedKnownTranslationRecoversGeometryAcrossIndependentFramesAndCaptureDelays()
            throws Exception {
        Pose2d[] rawStarts = {new Pose2d(10.0, -5.0, 0.60),
                new Pose2d(-7.0, 11.0, -1.10), new Pose2d(3.0, 8.0, 2.50)};
        // Keep each camera inside the existing fixed tags' view, including after translation.
        double[] tagHeadings = {-Math.PI / 2.0, 0.35, 2.80};
        for (int i = 0; i < rawStarts.length; i++) {
            for (boolean interpolate : new boolean[]{false, true}) {
                for (double turn : new double[]{1.20, -Math.PI / 2.0, Math.PI}) {
                    try {
                        assertAssistedRecovery(rawStarts[i],
                                new Pose2d(100.0, 50.0, tagHeadings[i]), turn, interpolate);
                    } catch (AssertionError failure) {
                        throw new AssertionError("assisted frame=" + i + ", turn=" + turn
                                + ", interpolate=" + interpolate, failure);
                    }
                }
            }
        }
    }

    @Test
    public void conditioningThresholdAcceptsOnlyTheWellSeparatedSideInBothDirections()
            throws Exception {
        // A unit-radius chord has length sqrt(0.5) at this angle. That geometric separation,
        // squared, is the documented conditioning boundary; this is not the inverse solve.
        double boundary = 2.0 * Math.asin(Math.sqrt(0.5) / 2.0);
        for (double sign : new double[]{-1.0, 1.0}) {
            for (double margin : new double[]{-1e-6, 1e-6}) {
                Fixture f = manual(MIXED, false);
                startManual(f);
                finishManualTurn(f, MIXED, sign * (boundary + margin), 0.0, 0.0);
                if (margin > 0.0) assertRecovered(f, MIXED);
                else assertConditioningRejected(f);
            }
        }
    }

    @Test
    public void zeroSmallAndNearFullTurnsRejectWithoutPublishingOffsets() throws Exception {
        for (double turn : new double[]{0.0, 1e-6, -1e-6,
                2.0 * Math.PI - 0.01, -2.0 * Math.PI + 0.01,
                2.0 * Math.PI, -2.0 * Math.PI}) {
            Fixture f = manual(MIXED, false);
            startManual(f);
            finishManualTurn(f, MIXED, turn, 0.0, 0.0);

            assertConditioningRejected(f);
            assertEquals("near-full turns must be accumulated rather than guessed from endpoints",
                    turn, result(f, "lastDeltaHeadingRad"), EPS);
        }
    }

    @Test
    public void invalidDevicePosesCannotFinishTheSampleOrReplaceReadinessWithCachedEvidence()
            throws Exception {
        Pose2d[] invalid = {null, new Pose2d(Double.NaN, 0.0, 1.0),
                new Pose2d(Double.POSITIVE_INFINITY, 0.0, 1.0),
                new Pose2d(0.0, Double.NEGATIVE_INFINITY, 1.0),
                new Pose2d(0.0, 0.0, Double.NaN)};
        for (Pose2d bad : invalid) {
            Fixture f = manual(MIXED, false);
            startManual(f);
            publishTurn(f, MIXED, Math.PI / 2.0, 0.0, 0.0);
            f.device.pose = bad == null ? null : sdkPose(bad);
            int polls = f.device.polls;
            f.queuePrimary();
            f.loopAt(f.clock.nowSec() + 0.01);

            assertEquals(polls + 1, f.device.polls);
            assertFalse(f.predictor.getKinematicSnapshot().hasPose);
            assertNoNumericResult(f);
            f.assertStopped();
        }
    }

    @Test
    public void nonNativeArithmeticProbesRejectNonfiniteDisplacementsAndIntermediateOverflow()
            throws Exception {
        // Direct maintainer probes deliberately bypass SDK transport and READY validation. They
        // test local arithmetic defense, not a claim that native Pinpoint can produce these values.
        Pose2d[][] endpoints = {
                {Pose2d.zero(), new Pose2d(Double.NaN, 0.0, Math.PI)},
                {Pose2d.zero(), new Pose2d(Double.POSITIVE_INFINITY, 0.0, Math.PI)},
                {new Pose2d(-Double.MAX_VALUE, 0.0, 0.0),
                        new Pose2d(Double.MAX_VALUE, 0.0, Math.PI)},
                {new Pose2d(0.0, 0.0, Math.PI / 4.0),
                        new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, 5.0 * Math.PI / 4.0)},
                // Finite residual, but the half-turn numerator multiplies it by two.
                {Pose2d.zero(), new Pose2d(Double.MAX_VALUE, 0.0, Math.PI)}
        };
        for (Pose2d[] pair : endpoints) {
            Fixture f = manual(MIXED, false);
            startManual(f);
            probeArithmetic(f, pair[0], pair[1], Math.PI);

            assertNoNumericResult(f);
            String failure = (String) field(f.owner, "lastAttemptFailure");
            assertNotNull(failure);
            assertTrue(failure, failure.contains("non-finite"));
            assertTrue(failure, failure.contains("Attempt discarded"));
            f.assertStopped();
        }
        for (double badHeading : new double[]{Double.NaN, Double.POSITIVE_INFINITY}) {
            Fixture f = manual(MIXED, false);
            startManual(f);
            probeArithmetic(f, Pose2d.zero(), new Pose2d(1.0, 2.0, 0.0), badHeading);
            assertNoNumericResult(f);
            assertNotNull(field(f.owner, "lastAttemptFailure"));
        }
    }

    @Test
    public void nonNativeLargeFiniteControlIsNotRejectedMerelyForItsMagnitude() throws Exception {
        Fixture f = manual(MIXED, false);
        startManual(f);
        // Same calculation-only seam as overflow probes, but all intermediates remain finite.
        double displacement = Double.MAX_VALUE / 8.0;
        probeArithmetic(f, Pose2d.zero(), new Pose2d(displacement, 0.0, Math.PI), Math.PI);

        assertCompleted(f);
        double recommendation = result(f, "lastRecommendedStrafePodOffsetForwardInches");
        assertTrue(Double.isFinite(recommendation));
        assertEquals(-displacement / 2.0, recommendation, displacement * 1e-15);
        assertTrue(Double.isFinite(result(f, "lastRecommendedForwardPodOffsetLeftInches")));
        // This is finite numerical output, not a physically plausible or valid hardware profile.
    }

    @Test
    public void rejectedSolveResetAbortAndFreshAttemptCannotReuseAnOldRecommendation()
            throws Exception {
        Fixture f = manual(MIXED, false);
        startManual(f);
        finishManualTurn(f, MIXED, Math.PI / 2.0, 0.0, 0.0);
        assertRecovered(f, MIXED);

        startManual(f);
        assertNoNumericResultInPhase(f, "ROTATING");
        finishManualTurn(f, MIXED, 0.10, 0.0, 0.0);
        assertConditioningRejected(f);
        setField(f.owner, "resetRequested", true);
        f.loopAt(f.clock.nowSec() + 0.01);
        assertNoNumericResult(f);

        startManual(f);
        publishTurn(f, MIXED, -Math.PI / 2.0, 0.0, 0.0);
        invoke(f.owner, "abortSample");
        f.queuePrimary();
        f.owner.loop(0.0);
        assertNoNumericResult(f);

        startManual(f);
        probeArithmetic(f, Pose2d.zero(), new Pose2d(Double.MAX_VALUE, 0.0, Math.PI), Math.PI);
        assertNoNumericResult(f);
        assertNotNull(field(f.owner, "lastAttemptFailure"));
        int pollsAfterFailure = f.device.polls;
        f.queuePrimary();
        f.owner.loop(0.0);
        assertNoNumericResult(f);
        assertEquals("failed arithmetic must inhibit same-cycle re-entry before polling",
                pollsAfterFailure, f.device.polls);

        startManual(f);
        finishManualTurn(f, MIXED, -1.20, 0.0, 0.0);
        assertRecovered(f, MIXED);
        assertNull(field(f.owner, "lastAttemptFailure"));
        assertTrue(f.commands.isEmpty());
    }

    /** Both final-A paths must use the latest complete observation, even if its heading changed. */
    private static void assertFinalACurrentHeading(boolean recenter) throws Exception {
        Fixture f = configured(false, recenter);
        f.queuePrimary();
        f.owner.loop(0.0);
        // True offsets (strafe forward, forward left) = (6, -4), configured = (2, 1).
        // Forward pod travel per radian = +4, strafe pod travel per radian = +6.
        // Applying the configured corrections gives body travel (+5, +4) per radian.
        // Integrating through a quarter turn gives (1, 9); a half turn gives (-8, 10).
        f.poseAt(0.10, new Pose2d(1.0, 9.0, Math.PI / 2.0));
        if (recenter) {
            f.queuePrimary();
            f.poseAt(0.20, new Pose2d(1.0, 9.0, Math.PI / 2.0));
            assertEquals("POST_RECENTER", f.phase());
        }
        f.queuePrimary();
        f.poseAt(0.30, new Pose2d(-8.0, 10.0, Math.PI));

        assertEquals("IDLE", f.phase());
        assertNull(field(f.owner, "lastAttemptFailure"));
        assertEquals(Math.PI, result(f, "lastDeltaHeadingRad"), EPS);
        assertEquals(-8.0, result(f, "lastDxStartBodyInches"), EPS);
        assertEquals(10.0, result(f, "lastDyStartBodyInches"), EPS);
        assertEquals(6.0, result(f, "lastRecommendedStrafePodOffsetForwardInches"), EPS);
        assertEquals(-4.0, result(f, "lastRecommendedForwardPodOffsetLeftInches"), EPS);
        f.assertStopped();
    }

    private static Fixture configured(boolean powered, boolean recenter) throws Exception {
        return new Fixture(false, powered, cfg -> {
            cfg.pinpoint.strafePodOffsetForwardInches = 2.0;
            cfg.pinpoint.forwardPodOffsetLeftInches = 1.0;
            cfg.targetTurnRad = Math.PI / 2.0;
            cfg.enablePostRotateRecenter = recenter;
        });
    }

    /** Construct one real owner and assert that its real predictor received the same pod facts. */
    private static Fixture manual(Geometry geometry, boolean recenter) throws Exception {
        Fixture f = new Fixture(false, false, cfg -> {
            cfg.pinpoint.strafePodOffsetForwardInches = geometry.configuredStrafeForward;
            cfg.pinpoint.forwardPodOffsetLeftInches = geometry.configuredForwardLeft;
            cfg.enablePostRotateRecenter = recenter;
        });
        assertEquals(geometry.configuredStrafeForward,
                f.device.configuredStrafePodOffsetForwardInches, 0.0);
        assertEquals(geometry.configuredForwardLeft,
                f.device.configuredForwardPodOffsetLeftInches, 0.0);
        return f;
    }

    /** A fresh cycle matters after abort/reset; the tool's own A path performs the software rebase. */
    private static void startManual(Fixture f) throws Exception {
        f.queuePrimary();
        f.loopAt(f.clock.nowSec() + 0.01);
        assertEquals("ROTATING", f.phase());
        Pose2d start = (Pose2d) field(f.owner, "startPinpointPose");
        assertEquals(0.0, start.xInches, 0.0);
        assertEquals(0.0, start.yInches, 0.0);
        assertEquals(0.0, start.headingRad, 0.0);
    }

    /** Publish enough actual owner cycles to make even a near-full turn's direction unambiguous. */
    private static void publishTurn(Fixture f, Geometry geometry, double turn,
                                    double realForward, double realLeft) {
        int observations = Math.max(1, (int) Math.ceil(Math.abs(turn) / 0.25));
        for (int i = 1; i <= observations; i++) {
            double fraction = (double) i / observations;
            nextPose(f, geometry.reported(turn * fraction,
                    realForward * fraction, realLeft * fraction));
        }
    }

    /** Finish through ordinary A; final-cycle changes have their own explicit regressions above. */
    private static void finishManualTurn(Fixture f, Geometry geometry, double turn,
                                         double realForward, double realLeft) throws Exception {
        publishTurn(f, geometry, turn, realForward, realLeft);
        f.queuePrimary();
        nextPose(f, geometry.reported(turn, realForward, realLeft));
    }

    private static void nextPose(Fixture f, Pose2d pose) {
        f.poseAt(f.clock.nowSec() + 0.01, pose);
    }

    /**
     * Keep the real tag solver and pose history; replace camera observations only. Each selected
     * capture endpoint has known wheel-travel truth, while delivery occurs at a different pose.
     */
    private static void assertAssistedRecovery(Pose2d rawStart, Pose2d tagStart,
                                               double turn, boolean interpolate) throws Exception {
        Fixture f = new Fixture(true, false, cfg -> {
            cfg.pinpoint.strafePodOffsetForwardInches = MIXED.configuredStrafeForward;
            cfg.pinpoint.forwardPodOffsetLeftInches = MIXED.configuredForwardLeft;
            cfg.enablePostRotateRecenter = false;
        });
        f.poseAt(0.02, interpolate ? deliveryOffset(rawStart, -1.0) : rawStart);
        LoopTimestamp exactStart = f.clock.nowTimestamp();
        f.clock.update(0.10);
        LoopTimestamp startCapture = interpolate
                ? f.clock.timestampSecondsAgo(0.04) : exactStart;
        f.device.pose = sdkPose(deliveryOffset(rawStart, 1.0));
        f.frameAt(startCapture, tagStart);
        f.queuePrimary();
        f.owner.loop(f.clock.dtSec());
        assertEquals("ROTATING", f.phase());
        f.noFrame();

        // True reference-point motion is (+3,-2) in its own starting robot axes. The raw and
        // tag streams report that motion in unrelated field frames, so no field delta is reused.
        f.poseAt(0.50, inField(rawStart, MIXED.reported(turn / 2.0, 1.5, -1.0)));
        Pose2d rawEnd = inField(rawStart, MIXED.reported(turn, 3.0, -2.0));
        Pose2d tagEnd = inField(tagStart, new Pose2d(3.0, -2.0, turn));
        f.poseAt(1.00, interpolate ? deliveryOffset(rawEnd, -1.0) : rawEnd);
        LoopTimestamp exactEnd = f.clock.nowTimestamp();
        f.clock.update(1.08);
        LoopTimestamp endCapture = interpolate
                ? f.clock.timestampSecondsAgo(0.04) : exactEnd;
        f.device.pose = sdkPose(deliveryOffset(rawEnd, 1.0));
        f.frameAt(endCapture, tagEnd);
        f.queuePrimary();
        f.owner.loop(f.clock.dtSec());

        assertRecovered(f, MIXED);
        Pose2d rotationOnly = MIXED.reported(turn, 0.0, 0.0);
        assertEquals(rotationOnly.xInches, result(f, "lastDxStartBodyInches"), MODEL_EPS);
        assertEquals(rotationOnly.yInches, result(f, "lastDyStartBodyInches"), MODEL_EPS);
        assertEquals(Math.sin(turn), Math.sin(result(f, "lastDeltaHeadingRad")), EPS);
        assertEquals(Math.cos(turn), Math.cos(result(f, "lastDeltaHeadingRad")), EPS);
        assertTrue((Boolean) field(f.owner, "lastHadTagStart"));
        assertTrue((Boolean) field(f.owner, "lastHadTagEnd"));
        assertEquals("capture history must never rebase the raw owner", 0, f.device.rebases);
        assertTrue(f.commands.isEmpty());
    }

    /** Symmetric saved poses make the midpoint known without reproducing history interpolation. */
    private static Pose2d deliveryOffset(Pose2d endpoint, double sign) {
        return new Pose2d(endpoint.xInches, endpoint.yInches + sign,
                endpoint.headingRad + 0.10 * sign);
    }

    /** Forward placement of one local trajectory into a deliberately chosen independent field. */
    private static Pose2d inField(Pose2d origin, Pose2d local) {
        double c = Math.cos(origin.headingRad);
        double s = Math.sin(origin.headingRad);
        return new Pose2d(origin.xInches + c * local.xInches - s * local.yInches,
                origin.yInches + s * local.xInches + c * local.yInches,
                origin.headingRad + local.headingRad);
    }

    /**
     * Calculation-only defensive probe. These private writes deliberately bypass device and
     * readiness validation; no normal constructor, device representability or motion is implied.
     */
    private static void probeArithmetic(Fixture f, Pose2d start, Pose2d end, double heading)
            throws Exception {
        setField(f.owner, "startPinpointPose", start);
        setField(f.owner, "latestPinpointPose", end);
        setField(f.owner, "startHeadingUnwrappedRad", 0.0);
        setField(field(f.owner, "headingUnwrapper"), "unwrappedRad", heading);
        invoke(f.owner, "finishSampleAndCompute");
    }

    private static void assertRecovered(Fixture f, Geometry geometry) throws Exception {
        assertCompleted(f);
        assertEquals(geometry.trueStrafeForward,
                result(f, "lastRecommendedStrafePodOffsetForwardInches"), MODEL_EPS);
        assertEquals(geometry.trueForwardLeft,
                result(f, "lastRecommendedForwardPodOffsetLeftInches"), MODEL_EPS);
        assertEquals(geometry.configuredStrafeForward - geometry.trueStrafeForward,
                result(f, "lastXErrorInches"), MODEL_EPS);
        assertEquals(geometry.configuredForwardLeft - geometry.trueForwardLeft,
                result(f, "lastYErrorInches"), MODEL_EPS);
    }

    private static void assertCompleted(Fixture f) throws Exception {
        assertEquals("IDLE", f.phase());
        assertNull(field(f.owner, "lastAttemptFailure"));
        assertNull(field(f.owner, "lastSolveNote"));
        assertNotNull(field(f.owner, "lastRecommendedStrafePodOffsetForwardInches"));
        assertNotNull(field(f.owner, "lastRecommendedForwardPodOffsetLeftInches"));
        f.assertStopped();
    }

    private static void assertConditioningRejected(Fixture f) throws Exception {
        assertEquals("IDLE", f.phase());
        assertNull(field(f.owner, "lastRecommendedStrafePodOffsetForwardInches"));
        assertNull(field(f.owner, "lastRecommendedForwardPodOffsetLeftInches"));
        assertNull(field(f.owner, "lastXErrorInches"));
        assertNull(field(f.owner, "lastYErrorInches"));
        assertNotNull(field(f.owner, "lastSolveNote"));
        assertTrue(((String) field(f.owner, "lastSolveNote")).contains("stable solve"));
        assertTrue(Double.isFinite(result(f, "lastDeltaHeadingRad")));
        f.assertStopped();
    }

    private static void assertNoNumericResult(Fixture f) throws Exception {
        assertNoNumericResultInPhase(f, "IDLE");
    }

    private static void assertNoNumericResultInPhase(Fixture f, String phase) throws Exception {
        assertEquals(phase, f.phase());
        for (String name : new String[]{"lastDxStartBodyInches", "lastDyStartBodyInches",
                "lastDeltaHeadingRad", "lastXErrorInches", "lastYErrorInches",
                "lastRecommendedStrafePodOffsetForwardInches",
                "lastRecommendedForwardPodOffsetLeftInches"}) {
            assertNull(name + " must not describe a rejected/unfinished sample", field(f.owner, name));
        }
    }

    private static double result(Fixture f, String name) throws Exception {
        return (Double) field(f.owner, name);
    }

    /** Known physical pod geometry plus independently chosen configured corrections, in inches. */
    private static final class Geometry {
        final double trueStrafeForward;
        final double trueForwardLeft;
        final double configuredStrafeForward;
        final double configuredForwardLeft;

        Geometry(double trueStrafeForward, double trueForwardLeft,
                 double configuredStrafeForward, double configuredForwardLeft) {
            this.trueStrafeForward = trueStrafeForward;
            this.trueForwardLeft = trueForwardLeft;
            this.configuredStrafeForward = configuredStrafeForward;
            this.configuredForwardLeft = configuredForwardLeft;
        }

        /**
         * Independent forward model: first construct wheel travel from true rigid-body motion,
         * then apply the configured correction to those travels, then integrate through heading.
         * Midpoint quadrature uses no production geometry/solve helper or inverse coefficients.
         * Real translation is supplied in the robot's starting axes, not its changing body axes.
         */
        Pose2d reported(double turn, double realForward, double realLeft) {
            int steps = 32768;
            double dHeading = turn / steps;
            double dx = realForward / steps;
            double dy = realLeft / steps;
            double reportedForward = 0.0;
            double reportedLeft = 0.0;
            for (int i = 0; i < steps; i++) {
                double middleHeading = (i + 0.5) * dHeading;
                double c = Math.cos(middleHeading);
                double s = Math.sin(middleHeading);
                double centerForwardTravel = c * dx + s * dy;
                double centerLeftTravel = -s * dx + c * dy;
                double forwardWheelTravel = centerForwardTravel - trueForwardLeft * dHeading;
                double strafeWheelTravel = centerLeftTravel + trueStrafeForward * dHeading;
                double correctedForwardTravel = forwardWheelTravel
                        + configuredForwardLeft * dHeading;
                double correctedLeftTravel = strafeWheelTravel
                        - configuredStrafeForward * dHeading;
                reportedForward += c * correctedForwardTravel - s * correctedLeftTravel;
                reportedLeft += s * correctedForwardTravel + c * correctedLeftTravel;
            }
            return new Pose2d(reportedForward, reportedLeft, turn);
        }
    }
}
