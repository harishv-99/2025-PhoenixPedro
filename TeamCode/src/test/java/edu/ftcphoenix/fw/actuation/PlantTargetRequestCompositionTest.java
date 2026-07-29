package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the single composable Request API and immutable target-planner stages. */
public final class PlantTargetRequestCompositionTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void varargsListAndDirectArrayProduceTheSameOrderedAlternatives() {
        PlantTargetRequest first = PlantTargetRequest.exact("first", -2.0);
        PlantTargetRequest second = PlantTargetRequest.periodic("second", 4.0, 10.0);
        PlantTargetRequest third = PlantTargetRequest.relative("third", 3.0);
        PlantTargetRequest[] array = {first, second, third};
        List<PlantTargetRequest> list = new ArrayList<PlantTargetRequest>(
                Arrays.asList(first, second, third));

        PlantTargetRequest fromVarargs = PlantTargetRequest.oneOf(first, second, third);
        PlantTargetRequest fromArray = PlantTargetRequest.oneOf(array);
        PlantTargetRequest fromList = PlantTargetRequest.oneOf(list);

        assertEquals("", first.reason());
        assertAlternativesEqual(fromVarargs, fromArray);
        assertAlternativesEqual(fromVarargs, fromList);
        assertAlternativeIds(fromVarargs, "first", "second", "third");
        assertEquals("", fromVarargs.reason());
        assertEquals("", fromArray.reason());
        assertEquals("", fromList.reason());

        array[0] = PlantTargetRequest.exact("replacement", 99.0);
        list.clear();

        assertAlternativeIds(fromArray, "first", "second", "third");
        assertAlternativeIds(fromList, "first", "second", "third");
        expectUnsupported(() -> fromList.alternatives().clear());
    }

    @Test
    public void nestedRequestsFlattenInDeclarationOrderAndSkipUnavailableInputs() {
        PlantTargetRequest nested = PlantTargetRequest.oneOf(
                PlantTargetRequest.none("nested source unavailable"),
                PlantTargetRequest.oneOf(
                        PlantTargetRequest.exact("second", 10.0),
                        PlantTargetRequest.exact("third", 20.0)));
        PlantTargetRequest combined = PlantTargetRequest.oneOf(
                PlantTargetRequest.exact("first", -10.0),
                PlantTargetRequest.none("ignored because alternatives remain"),
                nested,
                PlantTargetRequest.exact("fourth", 30.0));

        assertAlternativeIds(combined, "first", "second", "third", "fourth");

        PlantTargetResolution resolution = plannerFor(combined).resolve(
                linearContext(0.0, ScalarRange.bounded(-100.0, 100.0)),
                new ManualLoopClock().clock());

        assertEquals("first", resolution.selectedCandidateId());
        assertEquals(-10.0, resolution.target(), EPSILON);
    }

    @Test
    public void emptyAndAllUnavailableCompositionRetainOneActionableReason() {
        PlantTargetRequest emptyVarargs = PlantTargetRequest.oneOf();
        PlantTargetRequest emptyList = PlantTargetRequest.oneOf(
                Collections.<PlantTargetRequest>emptyList());
        PlantTargetRequest allUnavailable = PlantTargetRequest.oneOf(
                PlantTargetRequest.oneOf(),
                PlantTargetRequest.none("camera has no goal"),
                PlantTargetRequest.none("localization unavailable"));

        assertFalse(emptyVarargs.hasAlternatives());
        assertFalse(emptyList.hasAlternatives());
        assertEquals("no plant target alternatives", emptyVarargs.reason());
        assertEquals(emptyVarargs.reason(), emptyList.reason());
        assertFalse(allUnavailable.hasAlternatives());
        assertEquals("camera has no goal", allUnavailable.reason());

        PlantTargetRequest mixed = PlantTargetRequest.oneOf(
                PlantTargetRequest.none("camera missing"),
                PlantTargetRequest.exact("preset", 7.0));
        assertTrue(mixed.hasAlternatives());
        assertAlternativeIds(mixed, "preset");
        assertEquals("", mixed.reason());
    }

    @Test
    public void compositionRejectsNullContainersAndElementsActionably() {
        expectNullPointer("alternatives",
                () -> PlantTargetRequest.oneOf((PlantTargetRequest[]) null));
        expectNullPointer("alternatives",
                () -> PlantTargetRequest.oneOf((List<PlantTargetRequest>) null));
        expectNullPointer("must not contain null",
                () -> PlantTargetRequest.oneOf(
                        PlantTargetRequest.exact("valid", 1.0), null));
        expectNullPointer("must not contain null",
                () -> PlantTargetRequest.oneOf(Arrays.asList(
                        PlantTargetRequest.exact("valid", 1.0), null)));
    }

    @Test
    public void allTwelveFactoriesEnterTheSameSingleAlternativeRepresentation() {
        LoopClock clock = new LoopClock();
        clock.reset(5.0);
        LoopTimestamp timestamp = clock.nowTimestamp();
        PlantTargetRequest[] requests = {
                PlantTargetRequest.exact("exact", 1.0),
                PlantTargetRequest.equivalentPosition("equivalent", 2.0),
                PlantTargetRequest.periodic("periodic", 3.0, 360.0),
                PlantTargetRequest.relative("relative", 4.0),
                PlantTargetRequest.relativeEquivalentPosition("relative-equivalent", 5.0),
                PlantTargetRequest.relativePeriodic("relative-periodic", 6.0, 360.0),
                PlantTargetRequest.observedExact("observed-exact", 7.0, 0.40, timestamp),
                PlantTargetRequest.observedEquivalentPosition(
                        "observed-equivalent", 8.0, 0.50, timestamp),
                PlantTargetRequest.observedPeriodic(
                        "observed-periodic", 9.0, 360.0, 0.60, timestamp),
                PlantTargetRequest.observedRelative(
                        "observed-relative", 10.0, 0.70, timestamp),
                PlantTargetRequest.observedRelativeEquivalentPosition(
                        "observed-relative-equivalent", 11.0, 0.80, timestamp),
                PlantTargetRequest.observedRelativePeriodic(
                        "observed-relative-periodic", 12.0, 360.0, 0.90, timestamp)
        };
        boolean[] periodic = {
                false, true, true, false, true, true,
                false, true, true, false, true, true
        };
        boolean[] usesPlantPeriod = {
                false, true, false, false, true, false,
                false, true, false, false, true, false
        };
        boolean[] relative = {
                false, false, false, true, true, true,
                false, false, false, true, true, true
        };

        for (int i = 0; i < requests.length; i++) {
            PlantTargetRequest request = requests[i];
            assertTrue(request.hasAlternatives());
            assertEquals(1, request.alternatives().size());
            PlantTargetRequest.Alternative alternative = request.alternatives().get(0);
            assertEquals(periodic[i], alternative.periodic());
            assertEquals(usesPlantPeriod[i], alternative.usesPlantPeriod());
            assertEquals(relative[i], alternative.relative());
            assertEquals(i >= 6, alternative.observed());
        }
    }

    @Test
    public void fixedAndLiveRequestPlanOverloadsResolveEquivalently() {
        PlantTargetRequest request = PlantTargetRequest.oneOf(
                PlantTargetRequest.exact("far", 20.0),
                PlantTargetRequest.exact("near", 3.0));
        PlantTargetResolver fixed = plannerFor(PlantTargets.plan(request));
        PlantTargetResolver live = plannerFor(PlantTargets.plan(Source.constant(request)));
        PlantTargetContext context = linearContext(
                0.0, ScalarRange.bounded(-100.0, 100.0));

        PlantTargetResolution fixedResolution =
                fixed.resolve(context, new ManualLoopClock().clock());
        PlantTargetResolution liveResolution =
                live.resolve(context, new ManualLoopClock().clock());

        assertEquivalentResolution(fixedResolution, liveResolution);
        assertEquals("near", fixedResolution.selectedCandidateId());
        assertEquals(3.0, fixedResolution.target(), EPSILON);
    }

    @Test
    public void retainedPlannerStagesBranchWithoutOverwritingEarlierAnswers() {
        PlantTargetRequest request = PlantTargetRequest.oneOf(
                PlantTargetRequest.exact("nearest-below", -1.0),
                PlantTargetRequest.exact("increasing", 10.0));
        PlantTargets.PlanPreferenceStage preference = PlantTargets.plan(request);
        PlantTargets.PlanUnreachableStage nearest = preference.nearestToMeasurement();
        PlantTargets.PlanUnreachableStage increasing = preference.preferIncreasing();
        assertNotSame(nearest, increasing);

        PlantTargetResolver nearestResolver = nearest.rejectUnreachable()
                .whenUnavailable().reportUnavailable();
        PlantTargetResolver increasingResolver = increasing.rejectUnreachable()
                .whenUnavailable().reportUnavailable();
        PlantTargetContext context = linearContext(
                0.0, ScalarRange.bounded(-20.0, 20.0));

        assertEquals(-1.0, nearestResolver.resolve(
                context, new ManualLoopClock().clock()).target(), EPSILON);
        assertEquals(10.0, increasingResolver.resolve(
                context, new ManualLoopClock().clock()).target(), EPSILON);

        PlantTargets.PlanUnreachableStage reachability = PlantTargets
                .plan(PlantTargetRequest.exact("outside", 15.0))
                .nearestToMeasurement();
        PlantTargets.PlanReadyStage reject = reachability.rejectUnreachable();
        PlantTargets.PlanReadyStage clamp = reachability.clampUnreachableToRange();
        assertNotSame(reject, clamp);

        PlantTargetContext bounded = linearContext(
                5.0, ScalarRange.bounded(0.0, 10.0));
        assertFalse(reject.whenUnavailable().reportUnavailable()
                .resolve(bounded, new ManualLoopClock().clock()).hasTarget());
        PlantTargetResolution clamped = clamp.whenUnavailable().reportUnavailable()
                .resolve(bounded, new ManualLoopClock().clock());
        assertEquals(10.0, clamped.target(), EPSILON);
        assertTrue(clamped.clampedByPlanner());
    }

    @Test
    public void retainedAcceptanceBranchesKeepIndependentQualityAndAgePolicies() {
        ManualLoopClock time = new ManualLoopClock(10.0);
        PlantTargetRequest request = PlantTargetRequest.observedExact(
                "camera", 6.0, 0.70, time.clock().timestampSecondsAgo(0.50));
        PlantTargets.PlanReadyStage defaults = PlantTargets.plan(request)
                .nearestToMeasurement()
                .rejectUnreachable();
        PlantTargets.PlanAcceptBranch permissiveBranch = defaults.accept()
                .minQuality(0.50)
                .maxObservationAgeSec(1.0);
        PlantTargets.PlanAcceptBranch strictBranch = defaults.accept()
                .minQuality(0.80)
                .maxObservationAgeSec(0.25);
        PlantTargets.PlanReadyStage permissive = permissiveBranch.doneAccept();
        PlantTargets.PlanReadyStage strict = strictBranch.doneAccept();

        assertNotSame(permissiveBranch, strictBranch);
        assertNotSame(permissive, strict);
        assertNotSame(defaults, permissive);
        assertNotSame(defaults, strict);

        PlantTargetResolver defaultResolver =
                defaults.whenUnavailable().reportUnavailable();
        PlantTargetResolver permissiveResolver =
                permissive.whenUnavailable().reportUnavailable();
        PlantTargetResolver strictResolver =
                strict.whenUnavailable().reportUnavailable();
        PlantTargetContext context = linearContext(
                0.0, ScalarRange.bounded(-10.0, 10.0));

        assertEquals(6.0, defaultResolver.resolve(
                context, time.clock()).target(), EPSILON);
        assertEquals(6.0, permissiveResolver.resolve(
                context, time.clock()).target(), EPSILON);
        assertFalse(strictResolver.resolve(context, time.clock()).hasTarget());
    }

    @Test
    public void retainedEquivalentPositionStageBranchesAreIndependentSnapshots() {
        ScalarTarget command = ScalarTarget.create(20.0);
        PlantTargets.EquivalentPositionPreferenceStage preference =
                PlantTargets.equivalentPositionsOf(command);
        PlantTargets.EquivalentPositionReadyStage nearest = preference.nearestToMeasurement();
        PlantTargets.EquivalentPositionReadyStage decreasing = preference.preferDecreasing();
        assertNotSame(nearest, decreasing);

        PlantTargetResolver nearestResolver =
                nearest.whenUnavailable().reportUnavailable();
        PlantTargetResolver decreasingResolver =
                decreasing.whenUnavailable().reportUnavailable();
        PlantTargetContext context = PlantTargetContext.position(
                true,
                350.0,
                ScalarRange.bounded(0.0, 720.0),
                PositionPlant.Topology.PERIODIC,
                360.0,
                Double.NaN,
                Double.NaN);

        assertEquals(380.0, nearestResolver.resolve(
                context, new ManualLoopClock().clock()).target(), EPSILON);
        assertEquals(20.0, decreasingResolver.resolve(
                context, new ManualLoopClock().clock()).target(), EPSILON);
    }

    private static PlantTargetResolver plannerFor(PlantTargetRequest request) {
        return plannerFor(PlantTargets.plan(request));
    }

    private static PlantTargetResolver plannerFor(PlantTargets.PlanPreferenceStage stage) {
        return stage.nearestToMeasurement()
                .rejectUnreachable()
                .whenUnavailable().reportUnavailable();
    }

    private static PlantTargetContext linearContext(double measurement, ScalarRange range) {
        return PlantTargetContext.simple(
                true, measurement, range, Double.NaN, Double.NaN);
    }

    private static void assertAlternativeIds(PlantTargetRequest request, String... expectedIds) {
        assertTrue(request.hasAlternatives());
        assertEquals(expectedIds.length, request.alternatives().size());
        for (int i = 0; i < expectedIds.length; i++) {
            assertEquals(expectedIds[i], request.alternatives().get(i).id());
        }
    }

    private static void assertAlternativesEqual(
            PlantTargetRequest expected, PlantTargetRequest actual) {
        assertEquals(expected.alternatives().size(), actual.alternatives().size());
        for (int i = 0; i < expected.alternatives().size(); i++) {
            PlantTargetRequest.Alternative left = expected.alternatives().get(i);
            PlantTargetRequest.Alternative right = actual.alternatives().get(i);
            assertEquals(left.id(), right.id());
            assertEquals(left.value(), right.value(), EPSILON);
            assertEquals(left.periodic(), right.periodic());
            if (Double.isNaN(left.period())) {
                assertTrue(Double.isNaN(right.period()));
            } else {
                assertEquals(left.period(), right.period(), EPSILON);
            }
            assertEquals(left.usesPlantPeriod(), right.usesPlantPeriod());
            assertEquals(left.relative(), right.relative());
            assertEquals(left.quality(), right.quality(), EPSILON);
            assertSame(left.timestamp(), right.timestamp());
            assertEquals(left.observed(), right.observed());
        }
    }

    private static void assertEquivalentResolution(
            PlantTargetResolution expected, PlantTargetResolution actual) {
        assertEquals(expected.hasTarget(), actual.hasTarget());
        assertEquals(expected.kind(), actual.kind());
        assertEquals(expected.target(), actual.target(), EPSILON);
        assertEquals(expected.selectedCandidateId(), actual.selectedCandidateId());
        assertEquals(expected.satisfiesIntent(), actual.satisfiesIntent());
        assertEquals(expected.clampedByPlanner(), actual.clampedByPlanner());
    }

    private static void expectNullPointer(String messageFragment, Runnable action) {
        try {
            action.run();
            fail("Expected NullPointerException containing " + messageFragment);
        } catch (NullPointerException expected) {
            assertTrue("Expected message containing '" + messageFragment + "', got: "
                            + expected.getMessage(),
                    expected.getMessage() != null
                            && expected.getMessage().contains(messageFragment));
        }
    }

    private static void expectUnsupported(Runnable action) {
        try {
            action.run();
            fail("Expected immutable alternatives list");
        } catch (UnsupportedOperationException expected) {
            // Expected.
        }
    }
}
