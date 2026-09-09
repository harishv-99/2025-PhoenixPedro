package edu.ftcsushi.fw.localization.fusion;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.MotionDelta;
import edu.ftcsushi.fw.localization.MotionPredictor;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/**
 * Maintainer-only synthetic localization comparisons, not a robot simulator or tuning prescription.
 *
 * <p>Real Fusion/EKF/history owners consume independently authored numeric sensor publications.
 * Three predictors and two correction sources share immutable inputs, never one mutable queue.
 * No predictor implements PoseResetter: corrected results cannot change the next sensor input.
 * Truth uses explicit scalar field geometry, not production pose composition. Camera processing,
 * physical sensor errors, calibrated probabilities, and the default pushback stack are not tested.</p>
 *
 * <p>Only TEST02_SUMMARY lines are printed, in ordinary JUnit output. Error/ranking and modeled
 * uncertainty summaries describe these authored cases, not which estimator a robot should adopt.</p>
 */
public final class LocalizationRobustnessScenarioTest {
    private static final double EPS = 1e-8;
    private static final double RECOVERY_POSITION_IN = 0.75;
    private static final double RECOVERY_HEADING_RAD = Math.toRadians(4.0);
    private static final double RECOVERY_MAX_AGE_SEC = 0.15;
    private static final double RECOVERY_HOLD_SEC = 0.20;
    private static final String[] BRANCHES = {"RAW", "FUSION", "EKF"};

    @Test
    public void cleanMoveHoldTurnAndYawWrapMatchIndependentGeometry() {
        Run course = run(periodic("clean-course", Path.COURSE, 8000, 50, 50, 200, 0,
                Fault.CLEAN, Double.NaN), false, true);
        Run wrap = run(periodic("clean-yaw-wrap", Path.WRAP, 2000, 50, 50, 200, 0,
                Fault.CLEAN, Double.NaN), false, true);
        for (Run result : Arrays.asList(course, wrap)) {
            for (Row row : result.rows) for (Snapshot sample : row.samples) {
                assertTrue(sample.hasPose);
                assertEquals("clean independent field position", 0.0, sample.positionError, EPS);
                assertEquals("clean independent wrapped heading", 0.0, sample.headingError, EPS);
                assertEquals("current samples do not acquire delivery lag", 0.0, sample.ageSec, EPS);
            }
            result.print();
        }
        assertEquals(-Math.PI + Math.toRadians(10),
                wrap.last().samples[1].pose.yaw, EPS);
    }

    @Test
    public void driftSlipAndJitterRemainDeterministicAcrossBranchAndScenarioOrder() {
        Scenario drift = periodic("drift-slip-jitter", Path.COURSE, 8000, 50, 50, 200, 0,
                Fault.DRIFT_SLIP_JITTER, Double.NaN);
        Scenario clean = periodic("order-control", Path.STRAIGHT, 2000, 50, 50, 200, 0,
                Fault.CLEAN, Double.NaN);
        Run first = run(drift, false, true);
        Run control = run(clean, false, true);
        Run reverseControl = run(clean, true, true);
        Run reverse = run(drift, true, true);
        Run repeat = run(drift, false, true);
        assertEquals(control.signature(), reverseControl.signature());
        assertEquals(first.signature(), reverse.signature());
        assertEquals(first.signature(), repeat.signature());
        assertTrue("the authored raw fault must actually differ from truth",
                first.metrics(0).maxPosition > 1.0);
        for (int branch = 1; branch <= 2; branch++) {
            assertTrue(first.last().samples[branch].counts.accepted > 0);
            assertTrue(first.last().samples[branch].counts.replayed > 0);
        }
        first.print();
    }

    @Test
    public void correctionDropoutAndReacquisitionHaveABoundedIllustrativeRecovery() {
        Run result = run(periodic("dropout-reacquisition", Path.STRAIGHT, 4000, 50, 50, 100, 0,
                Fault.SLIP_THEN_REACQUIRE, 1.0), false, true);
        assertEquals(3.0, result.at(950).samples[0].positionError, EPS);
        for (int branch = 1; branch <= 2; branch++) {
            int beforeDropout = result.at(200).samples[branch].counts.accepted;
            assertTrue("reacquisition follows actual earlier accepted sightings", beforeDropout > 0);
            assertEquals("no new camera frame during dropout", beforeDropout,
                    result.at(950).samples[branch].counts.accepted);
            double recovery = result.metrics(branch).recoverySec;
            assertTrue("this constructed clean reacquisition must recover within two seconds",
                    Double.isFinite(recovery) && recovery >= RECOVERY_HOLD_SEC - EPS && recovery < 2.0);
        }
        assertTrue("uncorrected slip never recovers", Double.isNaN(result.metrics(0).recoverySec));
        result.print();
    }

    @Test
    public void isolatedOutlierIsRejectedButPersistentAdmissibleBiasCanLookConfident() {
        Run outlier = run(periodic("isolated-outlier", Path.STRAIGHT, 2000, 50, 50, 200, 0,
                Fault.OUTLIER, Double.NaN), false, true);
        for (int branch = 1; branch <= 2; branch++) {
            Snapshot before = outlier.at(950).samples[branch];
            Snapshot after = outlier.at(1000).samples[branch];
            assertEquals(before.counts.rejected + 1, after.counts.rejected);
            assertEquals(before.counts.accepted, after.counts.accepted);
            assertEquals(before.counts.lastAcceptedCaptureSec, after.counts.lastAcceptedCaptureSec, EPS);
            assertEquals("a rejected 100-inch event cannot move a clean trajectory",
                    0.0, after.positionError, EPS);
        }
        Run bias = run(periodic("persistent-admissible-bias", Path.STILL, 4000, 50, 50, 200, 0,
                Fault.BIASED_CORRECTION, Double.NaN), false, true);
        assertEquals(0.0, bias.last().samples[0].positionError, EPS);
        for (int branch = 1; branch <= 2; branch++) {
            Snapshot last = bias.last().samples[branch];
            assertTrue(last.counts.accepted > 5);
            assertEquals(0, last.counts.rejected);
            assertTrue("accepted biased evidence can worsen an accurate raw pose", last.positionError > 1.0);
            assertTrue("reported quality is not measured accuracy", last.quality > 0.5);
        }
        outlier.print();
        bias.print();
    }

    @Test
    public void retainedCameraAndRetainedPredictorHaveDifferentFreshnessConsequences() {
        Run camera = run(periodic("frozen-camera", Path.STRAIGHT, 2000, 50, 50, 200, 0,
                Fault.FROZEN_CAMERA, Double.NaN), false, true);
        for (int branch = 1; branch <= 2; branch++) {
            Snapshot atFreeze = camera.at(1000).samples[branch];
            Snapshot last = camera.last().samples[branch];
            assertEquals(atFreeze.counts.accepted, last.counts.accepted);
            assertTrue(last.counts.duplicate > atFreeze.counts.duplicate);
            assertEquals(1.0, last.counts.lastAcceptedCaptureSec, EPS);
            assertEquals(0.0, last.ageSec, EPS);
            assertEquals(0.0, last.positionError, EPS);
        }
        Run predictor = run(periodic("frozen-predictor", Path.STRAIGHT, 2000, 50, 50, 0, 0,
                Fault.FROZEN_PREDICTOR, Double.NaN), false, true);
        // Query before thaw and while 1.0 remains inside the history's 0.5-second retention.
        Run duringFreeze = run(periodic("frozen-predictor-history-probe", Path.STRAIGHT,
                1400, 50, 50, 0, 0, Fault.FROZEN_PREDICTOR, Double.NaN), false, true);
        for (int branch = 0; branch < 3; branch++) {
            Snapshot frozen = predictor.at(1400).samples[branch];
            assertEquals(0.4, frozen.ageSec, EPS);
            assertEquals("old geometry remains correct at its own time", 0.0, frozen.positionError, EPS);
            assertEquals("present-time error separately reveals lag", 4.0, frozen.presentPositionError, EPS);
            assertFalse("real history cannot turn frozen publications into a current pose",
                    duringFreeze.fixture.histories[branch].lookupSource().getAt(
                            duringFreeze.fixture.clock(), duringFreeze.fixture.stamp(1.25)).isAvailable());
            assertTrue("the supported old endpoint has not merely expired",
                    duringFreeze.fixture.histories[branch].lookupSource().getAt(
                            duringFreeze.fixture.clock(), duringFreeze.fixture.stamp(1.0)).isAvailable());
            assertEquals("the next complete positive interval must not lose frozen motion",
                    0.0, predictor.last().samples[branch].positionError, EPS);
        }
        camera.print();
        predictor.print();
    }

    @Test
    public void deliveryBatchUsesLatestArrivalAndOlderCapturesRemainOutOfOrder() {
        List<Frame> frames = Arrays.asList(
                frame("delivered-first", 20, 50, Path.STRAIGHT.at(0.02)),
                frame("delivered-last", 40, 70, Path.STRAIGHT.at(0.04)),
                frame("newer-capture", 160, 200, Path.STRAIGHT.at(0.16)),
                frame("older-arrives-late", 100, 300, Path.STRAIGHT.at(0.10)));
        Run result = run(scheduled("latest-delivered-and-out-of-order", Path.STRAIGHT,
                500, 100, 100, Fault.CLEAN, frames, Double.NaN), false, true);
        assertEquals("delivered-last", result.at(100).deliveredFrame);
        for (int branch = 1; branch <= 2; branch++) {
            assertEquals("two arrivals before one poll supply one latest observation", 1,
                    result.at(100).samples[branch].counts.accepted);
            assertEquals(0.04, result.at(100).samples[branch].counts.lastAcceptedCaptureSec, EPS);
            Counts before = result.at(200).samples[branch].counts;
            Counts after = result.at(300).samples[branch].counts;
            assertEquals(before.accepted, after.accepted);
            assertEquals(before.outOfOrder + 1, after.outOfOrder);
            assertEquals(0.0, result.at(300).samples[branch].positionError, EPS);
        }
        result.print();
    }

    @Test
    public void missingPredictorBreaksReplayUntilANewContinuousBaselineExists() {
        List<Step> steps = new ArrayList<>();
        steps.add(step(0, sample(0, Path.STRAIGHT.at(0)), Collections.emptyList()));
        steps.add(step(100, sample(100, Path.STRAIGHT.at(0.1)), Collections.emptyList()));
        steps.add(step(200, Sample.missing(200), Collections.singletonList(
                frame("crosses-missing-predictor", 50, 200, Path.STRAIGHT.at(0.05)))));
        steps.add(step(300, sample(300, Path.STRAIGHT.at(0.3)), Collections.emptyList()));
        steps.add(step(400, sample(400, Path.STRAIGHT.at(0.4)), Collections.singletonList(
                frame("new-continuous-history", 350, 400, Path.STRAIGHT.at(0.35)))));
        Run result = run(new Scenario("missing-history-recovery", Path.STRAIGHT, steps, Double.NaN),
                false, true);
        for (int branch = 1; branch <= 2; branch++) {
            assertFalse(result.at(200).samples[branch].hasPose);
            assertEquals(1, result.at(200).samples[branch].counts.rejected);
            assertEquals(0, result.at(200).samples[branch].counts.accepted);
            assertTrue(result.at(400).samples[branch].hasPose);
            assertEquals(1, result.at(400).samples[branch].counts.accepted);
            assertEquals(0.35, result.at(400).samples[branch].counts.lastAcceptedCaptureSec, EPS);
            assertFalse(result.fixture.histories[branch].lookupSource().getAt(result.fixture.clock(),
                    result.fixture.stamp(0.2)).isAvailable());
        }
        result.print();
    }

    @Test
    public void oldHistoryAndOverAgeCorrectionsStayUnavailableRatherThanClamping() {
        List<Frame> frames = Collections.singletonList(
                frame("over-age-frame", 200, 1500, Path.STRAIGHT.at(0.2)));
        Run result = run(scheduled("expired-history-and-frame", Path.STRAIGHT,
                1500, 100, 100, Fault.CLEAN, frames, Double.NaN), false, true);
        for (int branch = 0; branch < 3; branch++) {
            assertFalse(result.fixture.histories[branch].lookupSource().getAt(result.fixture.clock(),
                    result.fixture.stamp(0.2)).isAvailable());
            assertTrue(result.fixture.histories[branch].lookupSource().getAt(result.fixture.clock(),
                    result.fixture.stamp(1.5)).isAvailable());
        }
        for (int branch = 1; branch <= 2; branch++) {
            assertEquals(1, result.last().samples[branch].counts.rejected);
            assertEquals(0, result.last().samples[branch].counts.accepted);
            assertEquals(0.0, result.last().samples[branch].positionError, EPS);
        }
        result.print();
    }

    @Test
    public void clockResetKeepsOldTimestampObjectsOldAndFreezesPriorScores() {
        Fixture fixture = new Fixture("clock-reset", Path.STRAIGHT, Double.NaN);
        fixture.apply(step(0, sample(0, Path.STRAIGHT.at(0)), Collections.emptyList()), false, true);
        fixture.apply(step(100, sample(100, Path.STRAIGHT.at(0.1)), Collections.singletonList(
                frame("old-epoch", 100, 100, Path.STRAIGHT.at(0.1)))), false, true);
        LoopTimestamp old = fixture.corrections[0].estimate.timestamp;
        Row before = fixture.rows.get(1);
        String frozen = before.signature();
        Row reset = fixture.apply(new Step(100, null, Collections.emptyList(), true), false, true);
        assertTrue(Double.isNaN(old.ageSec(fixture.clock())));
        assertEquals("previous numeric measurements must survive epoch invalidation", frozen, before.signature());
        for (int branch = 1; branch <= 2; branch++) {
            assertFalse(reset.samples[branch].hasPose);
            assertTrue(Double.isNaN(reset.samples[branch].counts.lastAcceptedCaptureSec));
        }
        Row reacquired = fixture.apply(step(100, sample(100, Path.STRAIGHT.at(0.1)),
                Collections.singletonList(frame("new-epoch-same-numbers", 100, 100,
                        Path.STRAIGHT.at(0.1)))), false, true);
        assertFalse(old.isFresh(fixture.clock(), 1.0));
        assertTrue(fixture.corrections[0].estimate.timestamp.isFresh(fixture.clock(), 0.0));
        for (int branch = 1; branch <= 2; branch++) {
            assertTrue(reacquired.samples[branch].hasPose);
            assertEquals(before.samples[branch].counts.accepted + 1, reacquired.samples[branch].counts.accepted);
            assertEquals(0.1, reacquired.samples[branch].evidenceSec, EPS);
        }
        new Run(fixture).print();
    }

    @Test
    public void predictorCoordinateSegmentChangeDoesNotReplayAcrossTheBoundary() {
        Fixture fixture = new Fixture("predictor-segment-change", Path.STRAIGHT, Double.NaN);
        fixture.apply(step(0, sample(0, Path.STRAIGHT.at(0)), Collections.emptyList()), false, true);
        Row prior = fixture.apply(step(100, sample(100, new Point(2, 0, 0)),
                Collections.emptyList()), false, true);
        Row changed = fixture.apply(step(200, new Sample(200, Path.STRAIGHT.at(0.2), false, 1),
                Collections.singletonList(frame("pre-rebase-capture", 150, 200,
                        Path.STRAIGHT.at(0.15)))), false, true);
        for (int branch = 1; branch <= 2; branch++) {
            assertTrue(prior.samples[branch].segment != changed.samples[branch].segment);
            assertEquals(prior.samples[branch].counts.accepted, changed.samples[branch].counts.accepted);
            assertFalse(fixture.histories[branch].lookupSource().getAt(fixture.clock(), fixture.stamp(0.15))
                    .isAvailable());
        }
        Row recovered = fixture.apply(step(300, new Sample(300, Path.STRAIGHT.at(0.3), true, 1),
                Collections.singletonList(frame("post-rebase", 300, 300, Path.STRAIGHT.at(0.3)))),
                false, true);
        for (int branch = 1; branch <= 2; branch++) {
            assertEquals(1, recovered.samples[branch].counts.accepted);
            assertEquals(0.0, recovered.samples[branch].positionError, EPS);
        }
        new Run(fixture).print();
    }

    @Test
    public void samplingComparisonsChangeOnlyOneRateDimensionAndUseCommonCheckpoints() {
        Run baseline = run(periodic("rate-baseline", Path.COURSE, 4000, 50, 100, 200, 0,
                Fault.CLEAN, Double.NaN), false, true);
        Run polls = run(periodic("rate-extra-loop-polls", Path.COURSE, 4000, 25, 100, 200, 0,
                Fault.CLEAN, Double.NaN), false, true);
        Run predictor = run(periodic("rate-predictor-only", Path.COURSE, 4000, 50, 50, 200, 0,
                Fault.CLEAN, Double.NaN), false, true);
        Run correction = run(periodic("rate-correction-only", Path.COURSE, 4000, 50, 100, 400, 0,
                Fault.CLEAN, Double.NaN), false, true);
        Run delay = run(periodic("rate-delivery-delay-only", Path.COURSE, 4000, 50, 100, 200, 100,
                Fault.CLEAN, Double.NaN), false, true);
        for (int ms = 0; ms <= 4000; ms += 100) {
            for (int branch = 0; branch < 3; branch++) {
                Snapshot ordinary = baseline.at(ms).samples[branch];
                Snapshot extraPolls = polls.at(ms).samples[branch];
                assertEquals(ordinary.pose.x, extraPolls.pose.x, EPS);
                assertEquals(ordinary.pose.y, extraPolls.pose.y, EPS);
                assertEquals(ordinary.pose.yaw, extraPolls.pose.yaw, EPS);
                assertEquals(ordinary.evidenceSec, extraPolls.evidenceSec, EPS);
                assertEquals(ordinary.positionStd, extraPolls.positionStd, EPS);
                assertEquals(ordinary.headingStd, extraPolls.headingStd, EPS);
                assertEquals(ordinary.counts.accepted, extraPolls.counts.accepted);
            }
        }
        for (Run result : Arrays.asList(baseline, polls, predictor, correction, delay)) {
            for (int branch = 0; branch < 3; branch++) {
                assertEquals("same actual 100ms checkpoints", 41, result.metrics(branch).checkpoints);
                assertEquals(41, result.metrics(branch).available);
                assertEquals(41, result.metrics(branch).scored);
                assertEquals(41, result.metrics(branch).presentScored);
                assertEquals("clean rate variants preserve geometric correctness", 0.0,
                        result.metrics(branch).maxPosition, EPS);
                assertEquals(0.0, result.metrics(branch).maxHeading, EPS);
            }
            // Actual predictor/correction-rate and latency effects on covariance are descriptive.
            // Neither equal covariance nor an estimator ranking is an invariant of those changes.
            result.print();
        }
    }

    @Test
    public void unavailableAndUndefinedTruthCannotBecomeZeroErrorOrRecovery() {
        Fixture fixture = new Fixture("missing-truth-coverage", Path.LATE_TRUTH, 0.0);
        fixture.apply(step(0, null, Collections.emptyList()), false, true);
        fixture.apply(step(100, sample(100, new Point(0, 0, 0)), Collections.emptyList()), false, true);
        fixture.apply(step(200, sample(200, new Point(0, 0, 0)), Collections.emptyList()), false, true);
        fixture.apply(step(300, Sample.missing(300), Collections.emptyList()), false, true);
        Run run = new Run(fixture);
        for (int branch = 0; branch < 3; branch++) {
            Metrics metrics = run.metrics(branch);
            assertEquals(4, metrics.checkpoints);
            assertEquals(2, metrics.available);
            assertEquals("only t=.2 has independently defined truth", 1, metrics.scored);
            assertEquals(1, metrics.undefinedTruth);
            assertTrue(Double.isNaN(metrics.recoverySec));
        }
        // A new good window starts at .4, not the .2 sample before missing evidence at .3.
        fixture.apply(step(400, sample(400, new Point(0, 0, 0)), Collections.singletonList(
                frame("after-gap", 400, 400, new Point(0, 0, 0)))), false, true);
        fixture.apply(step(500, sample(500, new Point(0, 0, 0)), Collections.emptyList()), false, true);
        for (int branch = 0; branch < 3; branch++)
            assertTrue("missing evidence interrupts rather than bridges a recovery window",
                    Double.isNaN(new Run(fixture).metrics(branch).recoverySec));
        fixture.apply(step(600, sample(600, new Point(0, 0, 0)), Collections.emptyList()), false, true);
        Run recovered = new Run(fixture);
        for (int branch = 0; branch < 3; branch++)
            assertEquals(0.60, recovered.metrics(branch).recoverySec, EPS);
        recovered.print();
    }

    @Test
    public void metricArithmeticUsesOnlyAvailableTruthPairsAndExplicitUnits() {
        Fixture fixture = new Fixture("metric-arithmetic", Path.STILL, Double.NaN);
        fixture.apply(step(0, sample(0, new Point(3, 4, Math.toRadians(3))),
                Collections.emptyList()), false, true);
        fixture.apply(step(100, sample(100, new Point(0, 0, Math.toRadians(-4))),
                Collections.emptyList()), false, true);
        fixture.apply(step(200, Sample.missing(200), Collections.emptyList()), false, true);
        Run result = new Run(fixture);
        for (int branch = 0; branch < 3; branch++) {
            Metrics m = result.metrics(branch);
            assertEquals(3, m.checkpoints);
            assertEquals(2, m.available);
            assertEquals(2, m.scored);
            assertEquals(2, m.presentScored);
            // Position magnitudes are 5 and 0 inches; heading magnitudes are 3 and 4 degrees.
            assertEquals(Math.sqrt(25.0 / 2), m.rms(m.squaredPosition, m.scored), EPS);
            assertEquals(5.0, m.maxPosition, EPS);
            assertEquals(Math.toRadians(Math.sqrt(25.0 / 2)), m.rms(m.squaredHeading, m.scored), EPS);
            assertEquals(Math.toRadians(4), m.maxHeading, EPS);
            assertEquals(Math.sqrt(25.0 / 2), m.rms(m.squaredPresent, m.presentScored), EPS);
            assertEquals(Math.toRadians(Math.sqrt(25.0 / 2)),
                    m.rms(m.squaredPresentHeading, m.presentScored), EPS);
        }
        result.print();
    }

    private enum Fault { CLEAN, DRIFT_SLIP_JITTER, SLIP_THEN_REACQUIRE, OUTLIER,
        BIASED_CORRECTION, FROZEN_CAMERA, FROZEN_PREDICTOR }

    /** Authored field coordinates; linear pieces are the declared synthetic truth, not physics. */
    private enum Path {
        STILL, STRAIGHT, COURSE, WRAP, LATE_TRUTH;

        /** Evaluates only this trace's declared field frame/time domain; null means undefined truth. */
        Point at(double seconds) {
            if (!Double.isFinite(seconds) || seconds < -EPS || seconds > 8.0 + EPS) return null;
            if (this == LATE_TRUTH) return seconds < 0.2 - EPS ? null : new Point(0, 0, 0);
            if (this == STILL) return new Point(0, 0, 0);
            if (this == STRAIGHT) return new Point(10 * seconds, 0, 0);
            if (this == WRAP) return new Point(3, -2, Math.toRadians(170 + 10 * seconds));
            // Move, turn in place, move left, hold, turn, move, turn, move back.
            double[][] knots = {{0, 0, 0}, {12, 0, 0}, {12, 0, Math.PI / 2},
                    {12, 12, Math.PI / 2}, {12, 12, Math.PI / 2}, {12, 12, Math.PI},
                    {0, 12, Math.PI}, {0, 12, 3 * Math.PI / 2}, {0, 0, 3 * Math.PI / 2}};
            int left = Math.min(7, Math.max(0, (int) Math.floor(seconds)));
            double fraction = Math.max(0, Math.min(1, seconds - left));
            return new Point(knots[left][0] + fraction * (knots[left + 1][0] - knots[left][0]),
                    knots[left][1] + fraction * (knots[left + 1][1] - knots[left][1]),
                    knots[left][2] + fraction * (knots[left + 1][2] - knots[left][2]));
        }
    }

    /** Immutable field X/Y in inches and CCW yaw in radians; geometry only packages the inputs. */
    private static final class Point {
        final double x, y, yaw;
        Point(double x, double y, double yaw) { this.x = x; this.y = y; this.yaw = yaw; }
        Pose3d pose() { return new Pose3d(x, y, 0, wrap(yaw), 0, 0); }
    }

    private static double wrap(double angle) { return Math.atan2(Math.sin(angle), Math.cos(angle)); }

    /** One raw publication; null point means missing, unlike a null Sample that retains the last one. */
    private static final class Sample {
        final int captureMs;
        final Point point;
        final boolean continuous;
        final long segment;
        Sample(int captureMs, Point point, boolean continuous, long segment) {
            this.captureMs = captureMs; this.point = point; this.continuous = continuous; this.segment = segment;
        }
        static Sample missing(int ms) { return new Sample(ms, null, false, 0); }
    }

    /** One immutable absolute observation with separate authored capture and delivery milliseconds. */
    private static final class Frame {
        final String id;
        final int captureMs, deliveryMs;
        final Point point;
        Frame(String id, int captureMs, int deliveryMs, Point point) {
            assertTrue("a delivered observation cannot be captured afterward", captureMs <= deliveryMs);
            this.id = id; this.captureMs = captureMs; this.deliveryMs = deliveryMs; this.point = point;
        }
    }

    /** One serviced loop, its optional raw publication, and every frame that arrived before polling. */
    private static final class Step {
        final int deliveryMs;
        final Sample predictor;
        final List<Frame> delivered;
        final boolean resetEpoch;
        Step(int deliveryMs, Sample predictor, List<Frame> delivered, boolean resetEpoch) {
            this.deliveryMs = deliveryMs; this.predictor = predictor;
            this.delivered = Collections.unmodifiableList(new ArrayList<>(delivered));
            this.resetEpoch = resetEpoch;
        }
    }

    /** Bounded immutable input schedule; no mutable source or correction result is shared by runs. */
    private static final class Scenario {
        final String name;
        final Path truth;
        final List<Step> steps;
        final double faultEndSec;
        Scenario(String name, Path truth, List<Step> steps, double faultEndSec) {
            assertTrue("scenario has a fixed bounded number of loop rows", steps.size() <= 400);
            this.name = name; this.truth = truth;
            this.steps = Collections.unmodifiableList(new ArrayList<>(steps));
            this.faultEndSec = faultEndSec;
        }
    }

    private static Sample sample(int ms, Point point) { return new Sample(ms, point, true, 0); }
    private static Frame frame(String id, int capture, int delivery, Point point) {
        return new Frame(id, capture, delivery, point);
    }
    private static Step step(int ms, Sample sample, List<Frame> frames) { return new Step(ms, sample, frames, false); }

    /** Schedules captures separately from loop and predictor sampling; no random draws during polling. */
    private static Scenario periodic(String name, Path path, int endMs, int loopMs, int predictorMs,
                                     int cameraMs, int delayMs, Fault fault, double faultEndSec) {
        List<Frame> frames = new ArrayList<>();
        if (cameraMs > 0) for (int capture = 0; capture <= endMs; capture += cameraMs) {
            if (fault == Fault.SLIP_THEN_REACQUIRE && capture > 200 && capture < 1000) continue;
            if (fault == Fault.FROZEN_CAMERA && capture > 1000) continue;
            if (fault == Fault.BIASED_CORRECTION && capture == 0) continue;
            Point truth = path.at(capture / 1000.0);
            Point observation = truth;
            if (fault == Fault.OUTLIER && capture == 1000)
                observation = new Point(truth.x + 100, truth.y, truth.yaw);
            if (fault == Fault.BIASED_CORRECTION)
                observation = new Point(truth.x + 2, truth.y, truth.yaw + Math.toRadians(5));
            int delay = delayMs;
            if (fault == Fault.DRIFT_SLIP_JITTER) delay += new int[]{0, 100, 50}[capture / cameraMs % 3];
            frames.add(frame("capture-" + capture, capture, capture + delay, observation));
        }
        return scheduled(name, path, endMs, loopMs, predictorMs, fault, frames, faultEndSec);
    }

    /** Freezes the observed raw path and latest-delivered-frame rule before any owner is polled. */
    private static Scenario scheduled(String name, Path path, int endMs, int loopMs, int predictorMs,
                                      Fault fault, List<Frame> frames, double faultEndSec) {
        List<Frame> deliveries = new ArrayList<>(frames);
        // Stable sort preserves authored last-arrival order for equal delivery times.
        deliveries.sort((a, b) -> Integer.compare(a.deliveryMs, b.deliveryMs));
        List<Step> steps = new ArrayList<>();
        int nextFrame = 0;
        for (int ms = 0; ms <= endMs; ms += loopMs) {
            Sample predictor = null;
            if (ms % predictorMs == 0 && !(fault == Fault.FROZEN_PREDICTOR && ms > 1000 && ms < 1500)) {
                Point truth = path.at(ms / 1000.0);
                Point measured = truth;
                if (fault == Fault.DRIFT_SLIP_JITTER) measured = new Point(
                        1.03 * truth.x + 0.25 * ms / 1000.0 + (ms >= 2000 ? 2 : 0),
                        1.03 * truth.y - 0.10 * ms / 1000.0, truth.yaw + 0.01 * ms / 1000.0);
                if (fault == Fault.SLIP_THEN_REACQUIRE && ms >= 500)
                    measured = new Point(truth.x + 3, truth.y, truth.yaw);
                predictor = sample(ms, measured);
            }
            List<Frame> arrived = new ArrayList<>();
            while (nextFrame < deliveries.size() && deliveries.get(nextFrame).deliveryMs <= ms)
                arrived.add(deliveries.get(nextFrame++));
            steps.add(step(ms, predictor, arrived));
        }
        return new Scenario(name, path, steps, faultEndSec);
    }

    private static Run run(Scenario scenario, boolean reverse, boolean repeatSameCycle) {
        Fixture fixture = new Fixture(scenario.name, scenario.truth, scenario.faultEndSec);
        for (Step step : scenario.steps) fixture.apply(step, reverse, repeatSameCycle);
        return new Run(fixture);
    }

    /** Owns one clock and three independent input/estimator/history graphs. */
    private static final class Fixture {
        final String name;
        final Path truth;
        final double faultEndSec;
        final ManualLoopClock time = new ManualLoopClock();
        final ScriptedPredictor[] predictors = {new ScriptedPredictor(), new ScriptedPredictor(), new ScriptedPredictor()};
        final ScriptedCorrection[] corrections = {new ScriptedCorrection(), new ScriptedCorrection()};
        final AbsolutePoseEstimator[] owners;
        final PlanarPoseHistory[] histories;
        final List<Row> rows = new ArrayList<>();
        int previousMs;
        String deliveredFrame = "none";

        Fixture(String name, Path truth, double faultEndSec) {
            this.name = name; this.truth = truth; this.faultEndSec = faultEndSec;
            OdometryCorrectionFusionEstimator.Config fusion = OdometryCorrectionFusionEstimator.Config.defaults();
            fusion.enablePushCorrectedPoseToPredictor = false;
            fusion.maxCorrectionAgeSec = 0.60;
            OdometryCorrectionEkfEstimator.Config ekf = OdometryCorrectionEkfEstimator.Config.defaults();
            ekf.enablePushCorrectedPoseToPredictor = false;
            ekf.maxCorrectionAgeSec = 0.60;
            owners = new AbsolutePoseEstimator[]{predictors[0],
                    new OdometryCorrectionFusionEstimator(predictors[1], corrections[0], fusion),
                    new OdometryCorrectionEkfEstimator(predictors[2], corrections[1], ekf)};
            histories = new PlanarPoseHistory[3];
            for (int i = 0; i < 3; i++) {
                PlanarPoseHistory.Config history = PlanarPoseHistory.Config.defaults();
                history.maxInterpolationGapSec = 0.20;
                histories[i] = new PlanarPoseHistory(i == 0 ? predictors[0]
                        : (CorrectedPoseEstimator) owners[i], history);
            }
        }

        LoopClock clock() { return time.clock(); }
        LoopTimestamp stamp(double captureSec) { return capturedAt(clock(), captureSec); }

        /** Advances the sole clock once, publishes inputs, updates owners, then freezes observations. */
        Row apply(Step step, boolean reverse, boolean repeated) {
            assertTrue(step.deliveryMs >= previousMs);
            if (step.resetEpoch) {
                assertEquals("reset preserves ManualLoopClock's numerical controller time", previousMs, step.deliveryMs);
                clock().reset(clock().nowSec());
            } else time.nextCycle(step.deliveryMs / 1000.0 - clock().nowSec());
            previousMs = step.deliveryMs;
            for (ScriptedPredictor predictor : predictors) predictor.publish(step.predictor, clock());
            for (Frame frame : step.delivered) {
                assertTrue(frame.deliveryMs <= step.deliveryMs);
                for (ScriptedCorrection correction : corrections) correction.publish(frame, clock());
                deliveredFrame = frame.id;
            }
            int[] order = reverse ? new int[]{2, 1, 0} : new int[]{0, 1, 2};
            for (int branch : order) owners[branch].update(clock());
            for (PlanarPoseHistory history : histories) history.recordCurrent(clock());
            Row row = observe(step);
            if (repeated) {
                int[] calls = {predictors[0].polls, predictors[1].polls, predictors[2].polls,
                        corrections[0].polls, corrections[1].polls};
                PoseEstimate[] estimates = {owners[0].getEstimate(), owners[1].getEstimate(), owners[2].getEstimate()};
                for (int branch : order) owners[branch].update(clock());
                for (PlanarPoseHistory history : histories) history.recordCurrent(clock());
                for (int branch = 0; branch < 3; branch++) assertSame(estimates[branch], owners[branch].getEstimate());
                assertEquals(Arrays.toString(calls), Arrays.toString(new int[]{predictors[0].polls,
                        predictors[1].polls, predictors[2].polls, corrections[0].polls, corrections[1].polls}));
                assertEquals("same-cycle reads cannot change evidence, counters, or covariance",
                        row.signature(), observe(step).signature());
            }
            rows.add(row);
            return row;
        }

        /** Captures numerical diagnostics before a later reset can invalidate their timestamps. */
        Row observe(Step step) {
            Snapshot[] values = new Snapshot[3];
            for (int branch = 0; branch < 3; branch++) {
                long segment = branch == 0 ? predictors[0].trajectorySegmentId()
                        : ((CorrectedPoseEstimator) owners[branch]).trajectorySegmentId();
                values[branch] = new Snapshot(owners[branch], segment, truth, clock());
            }
            return new Row(step.deliveryMs, !step.resetEpoch && step.deliveryMs % 100 == 0,
                    deliveredFrame, values);
        }
    }

    /** Polling observes a scheduled snapshot; it never consumes input or invents new noise. */
    private static final class ScriptedPredictor implements MotionPredictor {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());
        Point baseline;
        LoopTimestamp baselineTime = LoopTimestamp.unavailable();
        long segment, cycle = Long.MIN_VALUE;
        int polls;

        void publish(Sample sample, LoopClock clock) {
            if (sample == null) return; // Same object/time/delta: retained, not a fresh stationary sample.
            LoopTimestamp at = capturedAt(clock, sample.captureMs / 1000.0);
            if (sample.point == null) {
                estimate = PoseEstimate.noPose(at); delta = MotionDelta.none(at); baseline = null;
                baselineTime = LoopTimestamp.unavailable(); return;
            }
            Point measured = sample.point;
            double elapsed = at.secondsSince(baselineTime);
            if (baseline != null && sample.continuous && sample.segment == segment && elapsed > 0) {
                double dxField = measured.x - baseline.x, dyField = measured.y - baseline.y;
                double c = Math.cos(baseline.yaw), s = Math.sin(baseline.yaw);
                // Explicit field-to-start-body rotation; no production composition in the oracle.
                Point body = new Point(c * dxField + s * dyField,
                        -s * dxField + c * dyField, wrap(measured.yaw - baseline.yaw));
                delta = new MotionDelta(body.pose(), true, 0.75, baselineTime, at);
            } else delta = MotionDelta.none(at);
            estimate = new PoseEstimate(measured.pose(), true, 0.75, at);
            if (baseline == null || elapsed > 0 || !Double.isFinite(elapsed) || sample.segment != segment) {
                baseline = measured; baselineTime = at;
            }
            segment = sample.segment;
        }
        @Override public void update(LoopClock clock) {
            if (cycle == clock.cycle()) return;
            cycle = clock.cycle(); polls++;
        }
        @Override public PoseEstimate getEstimate() { return estimate; }
        @Override public MotionDelta getLatestMotionDelta() { return delta; }
        @Override public long trajectorySegmentId() { return segment; }
    }

    /** Retains the actual last frame and timestamp; extra polls cannot manufacture a new sighting. */
    private static final class ScriptedCorrection implements AbsolutePoseEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        long cycle = Long.MIN_VALUE;
        int polls;
        void publish(Frame frame, LoopClock clock) {
            LoopTimestamp capture = capturedAt(clock, frame.captureMs / 1000.0);
            estimate = frame.point == null ? PoseEstimate.noPose(capture)
                    : new PoseEstimate(frame.point.pose(), true, 1.0, capture);
        }
        @Override public void update(LoopClock clock) {
            if (cycle == clock.cycle()) return;
            cycle = clock.cycle(); polls++;
        }
        @Override public PoseEstimate getEstimate() { return estimate; }
    }

    /** Numeric diagnostics are frozen while their original timestamp epoch remains valid. */
    private static final class Counts {
        final int accepted, rejected, duplicate, outOfOrder, replayed, direct;
        final double lastAcceptedCaptureSec;
        Counts(CorrectionStats stats, LoopClock clock) {
            accepted = stats.acceptedCorrectionCount; rejected = stats.rejectedCorrectionCount;
            duplicate = stats.skippedDuplicateCorrectionCount; outOfOrder = stats.skippedOutOfOrderCorrectionCount;
            replayed = stats.replayedCorrectionCount; direct = stats.nonReplayedCorrectionCount;
            lastAcceptedCaptureSec = representedTime(stats.lastAcceptedCorrectionMeasurementTimestamp, clock);
            assertEquals("acceptance partitions without claiming incorporation", accepted, replayed + direct);
        }
        String counters() { return accepted + "/" + rejected + "/" + duplicate + "/" + outOfOrder
                + "/" + replayed + "/" + direct; }
        String signature() { return counters() + "/" + lastAcceptedCaptureSec; }
    }

    /** Creates a timestamp only at publication; tolerates sub-nanosecond decimal clock roundoff. */
    private static LoopTimestamp capturedAt(LoopClock clock, double captureSec) {
        double age = clock.nowSec() - captureSec;
        assertTrue("authored capture must not be in the future", age >= -1e-12);
        return clock.timestampSecondsAgo(Math.max(0.0, age));
    }

    private static double representedTime(LoopTimestamp timestamp, LoopClock clock) {
        double age = timestamp.ageSec(clock);
        return Double.isFinite(age) ? clock.nowSec() - age : Double.NaN;
    }

    /** Frozen evidence-time and present-time errors plus the owner's same-observation diagnostics. */
    private static final class Snapshot {
        final boolean hasPose;
        final Point pose;
        final long segment;
        final double ageSec, evidenceSec, quality, positionError, headingError;
        final double presentPositionError, presentHeadingError, positionStd, headingStd, innovation;
        final Counts counts;
        Snapshot(AbsolutePoseEstimator owner, long segment, Path truth, LoopClock clock) {
            PoseEstimate value = owner.getEstimate();
            hasPose = value.hasPose; this.segment = segment;
            pose = new Point(value.fieldToRobotPose.xInches, value.fieldToRobotPose.yInches,
                    value.fieldToRobotPose.yawRad);
            ageSec = value.timestamp.ageSec(clock); evidenceSec = representedTime(value.timestamp, clock);
            quality = value.quality;
            if (hasPose) {
                assertTrue(Double.isFinite(pose.x) && Double.isFinite(pose.y) && Double.isFinite(pose.yaw));
                assertTrue(Double.isFinite(quality) && quality >= 0 && quality <= 1);
            }
            Point then = hasPose ? truth.at(evidenceSec) : null;
            Point now = hasPose ? truth.at(clock.nowSec()) : null;
            positionError = then == null ? Double.NaN : Math.hypot(pose.x - then.x, pose.y - then.y);
            headingError = then == null ? Double.NaN : Math.abs(wrap(pose.yaw - then.yaw));
            presentPositionError = now == null ? Double.NaN : Math.hypot(pose.x - now.x, pose.y - now.y);
            presentHeadingError = now == null ? Double.NaN : Math.abs(wrap(pose.yaw - now.yaw));
            counts = new Counts(owner instanceof CorrectedPoseEstimator
                    ? ((CorrectedPoseEstimator) owner).getCorrectionStats() : CorrectionStats.none(), clock);
            if (owner instanceof OdometryCorrectionEkfEstimator) {
                OdometryCorrectionEkfEstimator ekf = (OdometryCorrectionEkfEstimator) owner;
                positionStd = ekf.getPositionStdIn(); headingStd = ekf.getHeadingStdRad();
                innovation = ekf.getLastInnovationPositionIn();
                assertTrue(Double.isFinite(positionStd) && positionStd > 0);
                assertTrue(Double.isFinite(headingStd) && headingStd > 0);
            } else { positionStd = Double.NaN; headingStd = Double.NaN; innovation = Double.NaN; }
        }
        String signature() { return hasPose + ":" + segment + ":" + counts.signature() + ":"
                + Arrays.toString(new double[]{pose.x, pose.y, pose.yaw, ageSec, evidenceSec, quality,
                positionError, headingError, presentPositionError, presentHeadingError,
                positionStd, headingStd, innovation}); }
    }

    /** One frozen observation of all three independent branches at an actual serviced loop. */
    private static final class Row {
        final int ms;
        final boolean checkpoint;
        final String deliveredFrame;
        final Snapshot[] samples;
        Row(int ms, boolean checkpoint, String deliveredFrame, Snapshot[] samples) {
            this.ms = ms; this.checkpoint = checkpoint; this.deliveredFrame = deliveredFrame;
            this.samples = samples.clone();
        }
        String signature() {
            StringBuilder text = new StringBuilder(ms + ":" + checkpoint + ":" + deliveredFrame);
            for (Snapshot sample : samples) text.append('|').append(sample.signature());
            return text.toString();
        }
    }

    /** Aggregates only common actual 100ms checkpoints; no estimator-output interpolation. */
    private static final class Metrics {
        int checkpoints, available, scored, undefinedTruth, presentScored;
        double squaredPosition, squaredHeading, maxPosition, maxHeading, squaredPresent, maxPresent;
        double squaredPresentHeading, maxPresentHeading;
        double recoverySec = Double.NaN;
        Metrics(List<Row> rows, int branch, double faultEndSec) {
            double goodSince = Double.NaN;
            for (Row row : rows) {
                Snapshot s = row.samples[branch];
                double now = row.ms / 1000.0;
                if (Double.isFinite(faultEndSec) && now >= faultEndSec - EPS) {
                    boolean good = s.hasPose && Double.isFinite(s.positionError)
                            && s.presentPositionError <= RECOVERY_POSITION_IN
                            && s.presentHeadingError <= RECOVERY_HEADING_RAD
                            && Double.isFinite(s.ageSec) && s.ageSec <= RECOVERY_MAX_AGE_SEC + EPS;
                    if (!good) goodSince = Double.NaN;
                    else if (!Double.isFinite(goodSince)) goodSince = now;
                    if (good && !Double.isFinite(recoverySec) && now - goodSince >= RECOVERY_HOLD_SEC - EPS)
                        recoverySec = now - faultEndSec;
                }
                if (!row.checkpoint) continue;
                checkpoints++;
                if (!s.hasPose) continue;
                available++;
                if (Double.isFinite(s.positionError) && Double.isFinite(s.headingError)) {
                    scored++; squaredPosition += s.positionError * s.positionError;
                    squaredHeading += s.headingError * s.headingError;
                    maxPosition = Math.max(maxPosition, s.positionError);
                    maxHeading = Math.max(maxHeading, s.headingError);
                } else undefinedTruth++;
                if (Double.isFinite(s.presentPositionError) && Double.isFinite(s.presentHeadingError)) {
                    presentScored++; squaredPresent += s.presentPositionError * s.presentPositionError;
                    maxPresent = Math.max(maxPresent, s.presentPositionError);
                    squaredPresentHeading += s.presentHeadingError * s.presentHeadingError;
                    maxPresentHeading = Math.max(maxPresentHeading, s.presentHeadingError);
                }
            }
        }
        double rms(double sum, int count) { return count == 0 ? Double.NaN : Math.sqrt(sum / count); }
    }

    /** Completed immutable rows; retains its fixture only for immediate public history queries. */
    private static final class Run {
        final Fixture fixture;
        final List<Row> rows;
        Run(Fixture fixture) { this.fixture = fixture; rows = Collections.unmodifiableList(new ArrayList<>(fixture.rows)); }
        Row last() { return rows.get(rows.size() - 1); }
        Row at(int ms) {
            for (Row row : rows) if (row.ms == ms) return row;
            throw new AssertionError("Missing actual checkpoint " + ms);
        }
        Metrics metrics(int branch) { return new Metrics(rows, branch, fixture.faultEndSec); }
        String signature() {
            StringBuilder text = new StringBuilder();
            for (Row row : rows) text.append(row.signature()).append('\n');
            return text.toString();
        }
        /** Emits three bounded summaries into ordinary JUnit stdout, never Robot Controller storage. */
        void print() {
            for (int branch = 0; branch < 3; branch++) {
                Metrics m = metrics(branch); Snapshot last = last().samples[branch];
                System.out.printf(Locale.US, "TEST02_SUMMARY scenario=%s branch=%s checkpoints=%d available=%d "
                                + "scored=%d undefinedTruth=%d evidenceRmsIn=%.6f evidenceMaxIn=%.6f "
                                + "headingRmsRad=%.6f headingMaxRad=%.6f presentScored=%d presentRmsIn=%.6f "
                                + "presentMaxIn=%.6f presentHeadingRmsRad=%.6f presentHeadingMaxRad=%.6f "
                                + "recoverySec=%s lastAgeSec=%.6f lastQuality=%.6f "
                                + "accepted/rejected/duplicate/outOfOrder/replayed/nonReplayed=%s "
                                + "lastAcceptedCaptureSec=%.6f lastEvidenceErrorIn=%.6f lastEvidenceErrorRad=%.6f "
                                + "lastEkfPositionStdIn=%.6f lastEkfHeadingStdRad=%.6f lastInnovationIn=%.6f%n",
                        fixture.name, BRANCHES[branch], m.checkpoints, m.available, m.scored, m.undefinedTruth,
                        m.rms(m.squaredPosition, m.scored), m.scored == 0 ? Double.NaN : m.maxPosition,
                        m.rms(m.squaredHeading, m.scored), m.scored == 0 ? Double.NaN : m.maxHeading,
                        m.presentScored, m.rms(m.squaredPresent, m.presentScored),
                        m.presentScored == 0 ? Double.NaN : m.maxPresent,
                        m.rms(m.squaredPresentHeading, m.presentScored),
                        m.presentScored == 0 ? Double.NaN : m.maxPresentHeading,
                        Double.isFinite(m.recoverySec) ? String.format(Locale.US, "%.6f", m.recoverySec)
                                : Double.isFinite(fixture.faultEndSec) ? "not-recovered" : "not-requested",
                        last.ageSec, last.quality, last.counts.counters(), last.counts.lastAcceptedCaptureSec,
                        last.positionError, last.headingError,
                        last.positionStd, last.headingStd, last.innovation);
            }
        }
    }
}
