package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Verifies bounded, deterministic selection of periodic Plant target candidates. */
public final class PlantTargetsPeriodicPlannerTest {

    private static final double EPSILON = 1e-9;

    private enum Preference {
        NEAREST,
        INCREASING,
        DECREASING,
        RANGE_CENTER
    }

    @Test(timeout = 5000L)
    public void enormousCandidateSpacesAndIndexesBeyondLongResolveWithBoundedWork() {
        PlantTargetPlan enormousSpace = resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic("tiny-period", 0.0, 1.0e-12),
                context(123.456789, ScalarRange.bounded(-1.0e9, 1.0e9)));

        assertTrue(enormousSpace.hasTarget());
        assertEquals(123.456789, enormousSpace.target(), 1.0e-10);

        // The ideal periodic index is approximately 1e20, well beyond the long range. The
        // selected target is nevertheless an ordinary, finite value in the Plant coordinate.
        PlantTargetPlan beyondLong = resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic("beyond-long", 0.0, 1.0e-10),
                context(1.0e10, ScalarRange.bounded(1.0e10 - 1.0, 1.0e10 + 1.0)));

        assertTrue(beyondLong.hasTarget());
        assertEquals(1.0e10, beyondLong.target(), 1.0e-5);
    }

    @Test
    public void targetSpaceSelectionPreservesRepresentativeBeyondExactDoubleIndexRange() {
        PlantTargetPlan plan = resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic(
                        "beyond-exact-index", -9007199254740992.0, 1.0),
                context(1.0, ScalarRange.bounded(1.0, 1.0)));

        assertTrue(plan.hasTarget());
        assertEquals(1.0, plan.target(), 0.0);
        assertTrue(plan.satisfiesRequest());
        assertFalse(plan.clampedByPlanner());
    }

    @Test
    public void targetSpaceSelectionPreservesSignedSubnormalPhase() {
        PlantTargetPlan plan = resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic(
                        "signed-subnormal-phase", -Double.MIN_VALUE, Double.MAX_VALUE),
                context(0.0, ScalarRange.unbounded()));

        assertTrue(plan.hasTarget());
        assertEquals(-Double.MIN_VALUE, plan.target(), 0.0);
        assertTrue(plan.satisfiesRequest());
        assertFalse(plan.clampedByPlanner());
    }

    @Test
    public void reducedPhaseDoesNotFabricateMeasurementAtLargeBaseMidpoint() {
        PlantTargetPlan plan = resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic(
                        "large-base-midpoint", 10000000000000000.0, 2.0),
                context(-1.0, ScalarRange.unbounded()));

        assertTrue(plan.hasTarget());
        assertEquals(-2.0, plan.target(), 0.0);
        assertTrue(plan.satisfiesRequest());
        assertFalse(plan.clampedByPlanner());
    }

    @Test
    public void reducedPhaseWrapDoesNotFabricateAdjacentMeasurement() {
        double measurement = Math.nextUp(-1.0);
        PlantTargetPlan plan = resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic("remainder-wrap", 1.0, 2.0),
                context(measurement, ScalarRange.unbounded()));

        assertTrue(plan.hasTarget());
        assertEquals(-1.0, plan.target(), 0.0);
        assertFalse(Double.doubleToLongBits(measurement)
                == Double.doubleToLongBits(plan.target()));
        assertTrue(plan.satisfiesRequest());
        assertFalse(plan.clampedByPlanner());
    }

    @Test
    public void allPreferencesWorkAcrossBoundedHalfBoundedAndUnboundedRanges() {
        ScalarRange[] ranges = {
                ScalarRange.bounded(-12.0, 26.0),
                ScalarRange.minOnly(4.0),
                ScalarRange.maxOnly(-4.0),
                ScalarRange.unbounded()
        };
        double[][] expectedByRangeAndPreference = {
                {0.0, 10.0, 0.0, 10.0},
                {10.0, 10.0, 10.0, 10.0},
                {-10.0, -10.0, -10.0, -10.0},
                {0.0, 10.0, 0.0, 0.0}
        };

        for (int rangeIndex = 0; rangeIndex < ranges.length; rangeIndex++) {
            for (Preference preference : Preference.values()) {
                PlantTargetPlan plan = resolve(
                        preference,
                        PlantTargetRequest.periodic("family", 0.0, 10.0),
                        context(3.0, ranges[rangeIndex]));

                assertTrue("Expected a target for " + preference + " in " + ranges[rangeIndex],
                        plan.hasTarget());
                assertEquals(expectedByRangeAndPreference[rangeIndex][preference.ordinal()],
                        plan.target(), EPSILON);
            }
        }
    }

    @Test
    public void analyticSelectionMatchesIndependentBruteForceForOrdinaryFiniteRanges() {
        ScalarRange[] ranges = {
                ScalarRange.bounded(-17.0, 28.0),
                ScalarRange.bounded(-30.0, -2.0),
                ScalarRange.bounded(4.0, 35.0)
        };
        double[] bases = {-5.0, 0.0, 3.5};
        double[] periods = {2.5, 7.0, 10.0};
        double[] measurements = {-25.0, -7.5, 0.0, 5.0, 40.0};

        for (ScalarRange range : ranges) {
            for (double base : bases) {
                for (double period : periods) {
                    for (double measurement : measurements) {
                        for (Preference preference : Preference.values()) {
                            double expected = bruteForce(
                                    preference, base, period, measurement, range);
                            PlantTargetPlan plan = resolve(
                                    preference,
                                    PlantTargetRequest.periodic("oracle", base, period),
                                    context(measurement, range));

                            if (Double.isNaN(expected)) {
                                assertFalse(caseDescription(preference, base, period,
                                        measurement, range), plan.hasTarget());
                            } else {
                                assertTrue(caseDescription(preference, base, period,
                                        measurement, range), plan.hasTarget());
                                assertNear(expected, plan.target());
                            }
                        }
                    }
                }
            }
        }
    }

    @Test
    public void inclusiveBoundsAndMidpointTiesAreDeterministic() {
        ScalarRange range = ScalarRange.bounded(-5.0, 15.0);

        assertEquals(15.0, resolve(
                Preference.INCREASING,
                PlantTargetRequest.periodic("upper", 5.0, 10.0),
                context(15.0, range)).target(), EPSILON);
        assertEquals(-5.0, resolve(
                Preference.DECREASING,
                PlantTargetRequest.periodic("lower", 5.0, 10.0),
                context(-5.0, range)).target(), EPSILON);

        // 15 is exactly halfway between 10 and 20. The lower family member wins even though
        // Math.rint(1.5) would choose the even index 2 and therefore target 20.
        assertEquals(10.0, resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic("nearest-tie", 0.0, 10.0),
                context(15.0, ScalarRange.unbounded())).target(), EPSILON);

        // The finite range center is 15, so the same lower-target rule applies to center ties.
        assertEquals(10.0, resolve(
                Preference.RANGE_CENTER,
                PlantTargetRequest.periodic("center-tie", 0.0, 10.0),
                context(28.0, ScalarRange.bounded(0.0, 30.0))).target(), EPSILON);
    }

    @Test
    public void exactInclusiveBoundarySurvivesPeriodicIndexMaterialization() {
        double base = 372.70038574934006;
        double period = 29.157182029099207;
        double boundary = base + (-1131.0 * period);
        PlantTargetPlan plan = resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic("exact-boundary", base, period),
                context(boundary, ScalarRange.bounded(boundary, boundary)));

        assertTrue(plan.hasTarget());
        assertEquals(boundary, plan.target(), 0.0);
        assertTrue(plan.satisfiesRequest());
        assertFalse(plan.clampedByPlanner());
    }

    @Test
    public void exactHighIndexBoundarySurvivesDirectStencil() {
        double base = -1.3980913225044397E100;
        double period = 8.65816902795636E93;
        double index = -6108515264063315d;
        double boundary = base + index * period;
        PlantTargetPlan plan = resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic("exact-high-index-boundary", base, period),
                context(boundary, ScalarRange.bounded(boundary, boundary)));

        assertTrue(plan.hasTarget());
        assertEquals(Double.doubleToRawLongBits(boundary),
                Double.doubleToRawLongBits(plan.target()));
        assertTrue(plan.satisfiesRequest());
        assertFalse(plan.clampedByPlanner());
    }

    @Test
    public void exactCrossCandidateTieRetainsRequestOrder() {
        PlantTargetRequest request = PlantTargetRequest.oneOf(
                PlantTargetCandidate.periodic("declared-first", 10.0, 100.0),
                PlantTargetCandidate.periodic("declared-second", -10.0, 100.0));

        PlantTargetPlan plan = resolve(
                Preference.NEAREST,
                request,
                context(0.0, ScalarRange.bounded(-20.0, 20.0)));

        assertTrue(plan.hasTarget());
        assertEquals(10.0, plan.target(), EPSILON);
        assertEquals("declared-first", plan.selectedCandidateId());
    }

    @Test
    public void nearestComparatorDistinguishesExtremeAndSubnormalDistances() {
        PlantTargetRequest oppositeExtremes = PlantTargetRequest.oneOf(
                PlantTargetCandidate.exact("negative-maximum", -Double.MAX_VALUE),
                PlantTargetCandidate.exact("positive-maximum", Double.MAX_VALUE));
        PlantTargetPlan extremePlan = resolve(
                Preference.NEAREST,
                oppositeExtremes,
                context(Double.MIN_VALUE, ScalarRange.unbounded()));

        assertTrue(extremePlan.hasTarget());
        assertEquals(Double.MAX_VALUE, extremePlan.target(), 0.0);
        assertEquals("positive-maximum", extremePlan.selectedCandidateId());

        PlantTargetRequest subnormalDistances = PlantTargetRequest.oneOf(
                PlantTargetCandidate.exact("zero", 0.0),
                PlantTargetCandidate.exact("three-minimums", 3.0 * Double.MIN_VALUE));
        PlantTargetPlan subnormalPlan = resolve(
                Preference.NEAREST,
                subnormalDistances,
                context(2.0 * Double.MIN_VALUE, ScalarRange.unbounded()));

        assertTrue(subnormalPlan.hasTarget());
        assertEquals(3.0 * Double.MIN_VALUE, subnormalPlan.target(), 0.0);
        assertEquals("three-minimums", subnormalPlan.selectedCandidateId());
    }

    @Test
    public void directionalPreferencesAreDirectionFirstAndIndependentOfPlantScale() {
        PlantTargetRequest increasing = PlantTargetRequest.oneOf(
                PlantTargetCandidate.periodic("near-below", -1.0, 4.0e9),
                PlantTargetCandidate.periodic("far-above", 1.5e9, 4.0e9));
        PlantTargetPlan increasingPlan = resolve(
                Preference.INCREASING,
                increasing,
                context(0.0, ScalarRange.bounded(-1.0, 2.0e9)));

        assertTrue(increasingPlan.hasTarget());
        assertEquals("far-above", increasingPlan.selectedCandidateId());
        assertEquals(1.5e9, increasingPlan.target(), EPSILON);

        PlantTargetRequest decreasing = PlantTargetRequest.oneOf(
                PlantTargetCandidate.periodic("near-above", 1.0, 4.0e9),
                PlantTargetCandidate.periodic("far-below", -1.5e9, 4.0e9));
        PlantTargetPlan decreasingPlan = resolve(
                Preference.DECREASING,
                decreasing,
                context(0.0, ScalarRange.bounded(-2.0e9, 1.0)));

        assertTrue(decreasingPlan.hasTarget());
        assertEquals("far-below", decreasingPlan.selectedCandidateId());
        assertEquals(-1.5e9, decreasingPlan.target(), EPSILON);
    }

    @Test
    public void rangeCenterRemainsFiniteNearPositiveAndNegativeDoubleMaximum() {
        double max = Double.MAX_VALUE;
        double min = max * 0.5;
        double period = max * 0.25;
        double expectedCenter = min * 0.5 + max * 0.5;
        ScalarRange range = ScalarRange.bounded(min, max);

        assertTrue(Double.isFinite(range.center()));
        assertEquals(expectedCenter, range.center(), Math.ulp(expectedCenter));

        PlantTargetPlan plan = resolve(
                Preference.RANGE_CENTER,
                PlantTargetRequest.periodic("large-center", min, period),
                context(min, range));

        assertTrue(plan.hasTarget());
        assertEquals(expectedCenter, plan.target(), Math.ulp(expectedCenter));

        double negativeMin = -max;
        double negativeMax = -min;
        double expectedNegativeCenter = -expectedCenter;
        ScalarRange negativeRange = ScalarRange.bounded(negativeMin, negativeMax);

        assertTrue(Double.isFinite(negativeRange.center()));
        assertEquals(expectedNegativeCenter, negativeRange.center(),
                Math.ulp(expectedNegativeCenter));

        PlantTargetPlan negativePlan = resolve(
                Preference.RANGE_CENTER,
                PlantTargetRequest.periodic("large-negative-center", negativeMin, period),
                context(negativeMax, negativeRange));

        assertTrue(negativePlan.hasTarget());
        assertEquals(expectedNegativeCenter, negativePlan.target(),
                Math.ulp(expectedNegativeCenter));
    }

    @Test
    public void crossZeroSubnormalCenterRoundsToPositiveZero() {
        double center = ScalarRange.bounded(
                -Double.MIN_VALUE,
                2.0 * Double.MIN_VALUE).center();

        assertEquals(Double.doubleToRawLongBits(0.0), Double.doubleToRawLongBits(center));
    }

    @Test
    public void oneSidedRangesFindTheNearestRepresentativeFarFromMeasurement() {
        PlantTargetPlan lowerBounded = resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic("minimum", 0.0, 7.0),
                context(0.0, ScalarRange.minOnly(1000.0)));
        PlantTargetPlan upperBounded = resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic("maximum", 0.0, 7.0),
                context(0.0, ScalarRange.maxOnly(-1000.0)));

        assertTrue(lowerBounded.hasTarget());
        assertEquals(1001.0, lowerBounded.target(), EPSILON);
        assertTrue(upperBounded.hasTarget());
        assertEquals(-1001.0, upperBounded.target(), EPSILON);
    }

    @Test
    public void unreachablePeriodicFamilyRejectsOrClampsItsCanonicalBase() {
        PlantTargetRequest request = PlantTargetRequest.periodic("outside", 15.0, 100.0);
        PlantTargetContext context = context(5.0, ScalarRange.bounded(0.0, 10.0));

        PlantTargetPlan rejected = resolve(Preference.NEAREST, request, context);
        PlantTargetPlan clamped = resolve(Preference.NEAREST, request, context, true);

        assertFalse(rejected.hasTarget());
        assertTrue(clamped.hasTarget());
        assertEquals(10.0, clamped.target(), EPSILON);
        assertTrue(clamped.clampedByPlanner());
        assertFalse(clamped.satisfiesRequest());
        assertEquals("outside", clamped.selectedCandidateId());
    }

    @Test
    public void explicitAndPlantPeriodsSupportAbsoluteAndRelativeRequests() {
        ScalarRange range = ScalarRange.bounded(-100.0, 100.0);

        assertEquals(22.0, resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic("explicit", 2.0, 10.0),
                context(18.0, range)).target(), EPSILON);
        assertEquals(15.0, resolve(
                Preference.NEAREST,
                PlantTargetRequest.relativePeriodic("relative-explicit", 3.0, 10.0),
                context(12.0, range)).target(), EPSILON);
        assertEquals(10.0, resolve(
                Preference.NEAREST,
                PlantTargetRequest.equivalentPosition("plant-period", 2.0),
                periodicContext(13.0, range, 8.0)).target(), EPSILON);
        assertEquals(15.0, resolve(
                Preference.NEAREST,
                PlantTargetRequest.relativeEquivalentPosition("relative-plant-period", 3.0),
                periodicContext(12.0, range, 8.0)).target(), EPSILON);
    }

    @Test
    public void plantPeriodCandidateRequiresPeriodicPlantTopology() {
        PlantTargetContext linearWithNumericPeriod = PlantTargetContext.position(
                true,
                0.0,
                ScalarRange.bounded(-1000.0, 1000.0),
                PositionPlant.Topology.LINEAR,
                360.0,
                Double.NaN,
                Double.NaN);

        PlantTargetPlan plantPeriod = resolve(
                Preference.NEAREST,
                PlantTargetRequest.equivalentPosition("requires-periodic-plant", 10.0),
                linearWithNumericPeriod);
        PlantTargetPlan explicitPeriod = resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic("explicit-period", 10.0, 360.0),
                linearWithNumericPeriod);

        assertFalse(plantPeriod.hasTarget());
        assertTrue(explicitPeriod.hasTarget());
        assertEquals(10.0, explicitPeriod.target(), EPSILON);
    }

    @Test
    public void extremeArithmeticRecoversFiniteRepresentativesButInvalidRelativeBaseFailsClosed() {
        PlantTargetPlan relativeBaseOverflow = resolve(
                Preference.NEAREST,
                PlantTargetRequest.relativePeriodic(
                        "relative-overflow", Double.MAX_VALUE, 1.0),
                context(Double.MAX_VALUE,
                        ScalarRange.bounded(-Double.MAX_VALUE, Double.MAX_VALUE)),
                true);

        PlantTargetPlan quotientOverflow = resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic("quotient-overflow", 0.0, Double.MIN_VALUE),
                context(Double.MAX_VALUE, ScalarRange.unbounded()),
                true);

        assertFalse(relativeBaseOverflow.hasTarget());
        assertTrue(quotientOverflow.hasTarget());
        assertEquals(Double.MAX_VALUE, quotientOverflow.target(), 0.0);
    }

    @Test
    public void targetSpaceSelectionRecoversRepresentativeAcrossProductOverflow() {
        PlantTargetPlan plan = resolve(
                Preference.NEAREST,
                PlantTargetRequest.periodic(
                        "product-overflow", Double.MAX_VALUE, Double.MAX_VALUE),
                context(-Double.MAX_VALUE, ScalarRange.unbounded()));

        assertTrue(plan.hasTarget());
        assertEquals(-Double.MAX_VALUE, plan.target(), 0.0);
        assertTrue(plan.satisfiesRequest());
        assertFalse(plan.clampedByPlanner());
    }

    private static PlantTargetPlan resolve(Preference preference,
                                           PlantTargetRequest request,
                                           PlantTargetContext context) {
        return resolve(preference, request, context, false);
    }

    private static PlantTargetPlan resolve(Preference preference,
                                           PlantTargetRequest request,
                                           PlantTargetContext context,
                                           boolean clampUnreachable) {
        PlantTargets.PlanPreferenceStage preferenceStage = PlantTargets.plan()
                .request(Source.constant(request));
        PlantTargets.PlanUnreachableStage unreachableStage;
        switch (preference) {
            case INCREASING:
                unreachableStage = preferenceStage.preferIncreasing();
                break;
            case DECREASING:
                unreachableStage = preferenceStage.preferDecreasing();
                break;
            case RANGE_CENTER:
                unreachableStage = preferenceStage.preferRangeCenter();
                break;
            case NEAREST:
            default:
                unreachableStage = preferenceStage.nearestToMeasurement();
                break;
        }
        PlantTargets.PlanReadyStage readyStage = clampUnreachable
                ? unreachableStage.clampUnreachableToRange()
                : unreachableStage.rejectUnreachable();
        PlantTargetSource planner = readyStage.whenUnavailable().reportUnavailable();
        LoopClock clock = new ManualLoopClock().clock();
        return planner.resolve(context, clock);
    }

    private static PlantTargetContext context(double measurement, ScalarRange range) {
        return PlantTargetContext.simple(
                true,
                measurement,
                range,
                Double.NaN,
                Double.NaN);
    }

    private static PlantTargetContext periodicContext(double measurement,
                                                      ScalarRange range,
                                                      double period) {
        return PlantTargetContext.position(
                true,
                measurement,
                range,
                PositionPlant.Topology.PERIODIC,
                period,
                Double.NaN,
                Double.NaN);
    }

    private static double bruteForce(Preference preference,
                                     double base,
                                     double period,
                                     double measurement,
                                     ScalarRange range) {
        double best = Double.NaN;
        for (int k = -100; k <= 100; k++) {
            double target = base + k * period;
            if (!range.valid || target < range.minValue || target > range.maxValue) continue;
            if (Double.isNaN(best)
                    || compare(preference, target, best, measurement, range) < 0) {
                best = target;
            }
        }
        return best;
    }

    private static int compare(Preference preference,
                               double first,
                               double second,
                               double measurement,
                               ScalarRange range) {
        int firstDirection = directionRank(preference, first, measurement);
        int secondDirection = directionRank(preference, second, measurement);
        if (firstDirection != secondDirection) {
            return Integer.compare(firstDirection, secondDirection);
        }

        double center = range.valid
                && Double.isFinite(range.minValue)
                && Double.isFinite(range.maxValue)
                ? range.minValue / 2.0 + range.maxValue / 2.0
                : Double.NaN;
        double anchor = preference == Preference.RANGE_CENTER && Double.isFinite(center)
                ? center
                : measurement;
        int distance = Double.compare(Math.abs(first - anchor), Math.abs(second - anchor));
        if (distance != 0) return distance;
        return Double.compare(first, second);
    }

    private static int directionRank(Preference preference,
                                     double target,
                                     double measurement) {
        if (preference == Preference.INCREASING) {
            return target >= measurement ? 0 : 1;
        }
        if (preference == Preference.DECREASING) {
            return target <= measurement ? 0 : 1;
        }
        return 0;
    }

    private static void assertNear(double expected, double actual) {
        assertEquals(expected, actual, Math.max(EPSILON, Math.ulp(expected) * 4.0));
    }

    private static String caseDescription(Preference preference,
                                          double base,
                                          double period,
                                          double measurement,
                                          ScalarRange range) {
        return "preference=" + preference
                + ", base=" + base
                + ", period=" + period
                + ", measurement=" + measurement
                + ", range=" + range;
    }
}
