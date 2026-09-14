package edu.ftcsushi.fw.spatial;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

import edu.ftcsushi.fw.core.geometry.Pose2d;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Independent exact fixtures for fixed-heading object-center sweep geometry. */
public final class ToolSweep2dTest {
    private static final double EPS = 1e-9;

    @Test public void offsetFixtureReportsRobotTravelNotToolRange() {
        ToolSweep2d sweep = ToolSweep2d.straightFrom(new Pose2d(10, 20, 0))
                .toFieldPoint(30, 20).throughTool(new Pose2d(6, 1, 0))
                .centerWindowInches(-1, 2, 4);
        assertEncounter(sweep.encounterFieldCenter(25, 22), 7, 10);
        assertEncounter(sweep.encounterFieldCenter(25, 19), 7, 10);
        assertFalse(sweep.encounterFieldCenter(25, Math.nextDown(19)).hasEncounter());
        assertFalse(sweep.encounterFieldCenter(25, Math.nextUp(23)).hasEncounter());
    }

    @Test public void closedLongitudinalEndpointsAndZeroDepthAreIncluded() {
        ToolSweep2d sweep = forward(10, 0, 0, 2);
        assertEncounter(sweep.encounterFieldCenter(0, 0), 0, 0);
        assertEncounter(sweep.encounterFieldCenter(10, 0), 10, 10);
        assertFalse(sweep.encounterFieldCenter(Math.nextDown(0.0), 0).hasEncounter());
        assertFalse(sweep.encounterFieldCenter(Math.nextUp(10.0), 0).hasEncounter());
        assertEncounter(sweep.encounterFieldCenter(5, 1), 5, 5);
        assertFalse(sweep.encounterFieldCenter(5, Math.nextUp(1.0)).hasEncounter());
    }

    @Test public void startingAndEndingInsideClipToSegment() {
        ToolSweep2d sweep = forward(10, -2, 3, 2);
        assertEncounter(sweep.encounterFieldCenter(1, 0), 0, 3);
        assertEncounter(sweep.encounterFieldCenter(11, 0), 8, 10);
        assertEncounter(sweep.encounterFieldCenter(13, 0), 10, 10);
        assertEncounter(sweep.encounterFieldCenter(-2, 0), 0, 0);
        assertFalse(sweep.encounterFieldCenter(Math.nextUp(13.0), 0).hasEncounter());
    }

    @Test public void reversalChangesTravelOrderButNotToolHeading() {
        ToolSweep2d original = ToolSweep2d.straightFrom(new Pose2d(10, 20, 0))
                .toFieldPoint(30, 20).throughTool(new Pose2d(6, 1, 0))
                .centerWindowInches(-1, 2, 4);
        ToolSweep2d reverse = ToolSweep2d.straightFrom(new Pose2d(30, 20, 0))
                .toFieldPoint(10, 20).throughTool(new Pose2d(6, 1, 0))
                .centerWindowInches(-1, 2, 4);
        assertEncounter(original.encounterFieldCenter(25, 22), 7, 10);
        assertEncounter(reverse.encounterFieldCenter(25, 22), 10, 13);
    }

    @Test public void sidewaysMotionClipsTheLateralAxis() {
        ToolSweep2d sweep = ToolSweep2d.straightFrom(Pose2d.zero()).toFieldPoint(0, 10)
                .throughTool(new Pose2d(2, 1, 0)).centerWindowInches(-1, 2, 4);
        assertEncounter(sweep.encounterFieldCenter(3, 6), 3, 7);
        assertFalse(sweep.encounterFieldCenter(Math.nextUp(4.0), 6).hasEncounter());
    }

    @Test public void diagonalClipsBothAxesAndRejectsBoundingBoxFalsePositive() {
        ToolSweep2d sweep = ToolSweep2d.straightFrom(Pose2d.zero()).toFieldPoint(10, 10)
                .throughTool(Pose2d.zero()).centerWindowInches(-1, 1, 2);
        assertEncounter(sweep.encounterFieldCenter(6, 5), 5 * Math.sqrt(2), 6 * Math.sqrt(2));
        // The swept field-axis bounding box contains (0, 10), but the actual corridor does not.
        assertFalse(sweep.encounterFieldCenter(0, 10).hasEncounter());
        // Opposite window corners touch the same center at exactly one modeled travel position.
        assertEncounter(sweep.encounterFieldCenter(6, 4), 5 * Math.sqrt(2), 5 * Math.sqrt(2));
        assertFalse(sweep.encounterFieldCenter(6, 3.9).hasEncounter());
    }

    @Test public void diagonalThreeFourFiveMovementUsesEuclideanTravel() {
        ToolSweep2d sweep = ToolSweep2d.straightFrom(Pose2d.zero()).toFieldPoint(3, 4)
                .throughTool(Pose2d.zero()).centerWindowInches(0, 1, 2);
        // X allows t in [1/3, 2/3]; Y allows [1/4, 3/4]. Segment length is exactly 5.
        assertEncounter(sweep.encounterFieldCenter(2, 2), 5.0 / 3, 10.0 / 3);
    }

    @Test public void stationaryWindowRetainsItsToolOffsetAndClosedBoundaries() {
        ToolSweep2d sweep = ToolSweep2d.straightFrom(new Pose2d(10, 20, 0))
                .toFieldPoint(10, 20).throughTool(new Pose2d(6, 1, 0))
                .centerWindowInches(-1, 2, 4);
        assertEncounter(sweep.encounterFieldCenter(15, 19), 0, 0);
        assertEncounter(sweep.encounterFieldCenter(18, 23), 0, 0);
        assertFalse(sweep.encounterFieldCenter(Math.nextDown(15.0), 21).hasEncounter());
        assertFalse(sweep.encounterFieldCenter(17, Math.nextUp(23.0)).hasEncounter());
    }

    @Test public void rotatedRobotAndToolApplyTheMountOnce() {
        ToolSweep2d sweep = ToolSweep2d.straightFrom(new Pose2d(10, 20, Math.PI / 2))
                .toFieldPoint(10, 40).throughTool(new Pose2d(6, 1, -Math.PI / 2))
                .centerWindowInches(-1, 2, 4);
        // Start tool is (9, 26), facing field +X. Robot translation is field +Y.
        assertEncounter(sweep.encounterFieldCenter(10, 35), 7, 11);
        assertFalse(sweep.encounterFieldCenter(12, 35).hasEncounter());
    }

    @Test public void negativeForwardWindowIsDeliberatelyBehindTheTool() {
        ToolSweep2d sweep = forward(10, -4, -2, 2);
        assertEncounter(sweep.encounterFieldCenter(0, 0), 2, 4);
        assertEncounter(sweep.encounterFieldCenter(-3, 0), 0, 1);
    }

    @Test public void innerCenterWindowDoesNotInflateForBallOverlapOrShrinkAgain() {
        // An illustrative physical opening is 12 inches wide, and a ball is 4 inches across.
        // This authored +/-3 center window leaves another inch per side beyond whole-ball fit.
        ToolSweep2d sweep = forward(10, 0, 1, 6);
        assertFalse(sweep.encounterFieldCenter(5, 6).hasEncounter());
        assertFalse(sweep.encounterFieldCenter(5, 4).hasEncounter());
        assertFalse(sweep.encounterFieldCenter(5, 3.5).hasEncounter());
        assertEncounter(sweep.encounterFieldCenter(5, 3), 4, 5);
        assertEncounter(sweep.encounterFieldCenter(5, -3), 4, 5);
    }

    @Test public void tinyAxisDoesNotOverflowAnIrrelevantCrossingOrBecomeAnEpsilon() {
        ToolSweep2d sweep = ToolSweep2d.straightFrom(Pose2d.zero())
                .toFieldPoint(10, Double.MIN_VALUE).throughTool(Pose2d.zero())
                .centerWindowInches(0, 1, 2);
        assertEncounter(sweep.encounterFieldCenter(5, 0), 4, 5);
        ToolSweep2d tinyLateral = ToolSweep2d.straightFrom(Pose2d.zero())
                .toFieldPoint(0, 2 * Double.MIN_VALUE).throughTool(Pose2d.zero())
                .centerWindowInches(0, 0, 2 * Double.MIN_VALUE);
        ToolSweep2d.Encounter result = tinyLateral.encounterFieldCenter(0, 2 * Double.MIN_VALUE);
        assertTrue(result.hasEncounter());
        assertEquals(Double.MIN_VALUE, result.entryTravelInches(), 0);
        assertEquals(2 * Double.MIN_VALUE, result.exitTravelInches(), 0);
    }

    @Test public void tinyTravelFractionStillProducesRepresentableDistance() {
        ToolSweep2d sweep = forward(Double.MAX_VALUE, 0, 0, 2);
        ToolSweep2d.Encounter result = sweep.encounterFieldCenter(Double.MIN_VALUE, 0);
        assertTrue(result.hasEncounter());
        assertEquals(Double.MIN_VALUE, result.entryTravelInches(), 0);
        assertEquals(Double.MIN_VALUE, result.exitTravelInches(), 0);
    }

    @Test public void hugeRepresentableLengthUsesHypotRatherThanSquaring() {
        ToolSweep2d sweep = forward(1e200, 0, 1e199, 2);
        ToolSweep2d.Encounter result = sweep.encounterFieldCenter(5e199, 0);
        assertTrue(result.hasEncounter());
        assertEquals(4e199, result.entryTravelInches(), 1e184);
        assertEquals(5e199, result.exitTravelInches(), 1e184);
    }

    @Test public void finiteLargeHeadingsUsePoseCompositionWithoutWrapping() {
        double robotHeading = 1e16;
        double toolHeading = 3;
        Pose2d robot = new Pose2d(10, 20, robotHeading);
        Pose2d tool = new Pose2d(4, -1, toolHeading);
        Pose2d composed = robot.then(tool);
        // Build the center directly from Pose2d's documented composed frame.
        Pose2d center = composed.then(new Pose2d(1, 0, 0));
        ToolSweep2d sweep = ToolSweep2d.straightFrom(robot).toFieldPoint(10, 20)
                .throughTool(tool).centerWindowInches(0.5, 1.5, 0.5);
        assertEncounter(sweep.encounterFieldCenter(center.xInches, center.yInches), 0, 0);
    }

    @Test public void retainedStagesAndResultsAreIndependentAcrossAllAnswers() {
        ToolSweep2d.EndpointStage endpoint = ToolSweep2d.straightFrom(Pose2d.zero());
        ToolSweep2d.ToolStage tool = endpoint.toFieldPoint(10, 0);
        ToolSweep2d.CenterWindowStage window = tool.throughTool(Pose2d.zero());
        ToolSweep2d first = window.centerWindowInches(0, 1, 2);
        ToolSweep2d.Encounter retained = first.encounterFieldCenter(5, 0);
        ToolSweep2d shorter = endpoint.toFieldPoint(2, 0).throughTool(Pose2d.zero())
                .centerWindowInches(0, 1, 2);
        ToolSweep2d offset = tool.throughTool(new Pose2d(2, 0, 0)).centerWindowInches(0, 1, 2);
        ToolSweep2d wider = window.centerWindowInches(0, 2, 8);
        assertEncounter(first.encounterFieldCenter(5, 0), 4, 5);
        assertEncounter(retained, 4, 5);
        assertFalse(shorter.encounterFieldCenter(5, 0).hasEncounter());
        assertEncounter(offset.encounterFieldCenter(5, 0), 2, 3);
        assertEncounter(wider.encounterFieldCenter(5, 3), 3, 5);
        assertFalse(first.encounterFieldCenter(5, 3).hasEncounter());
        assertEncounter(first.encounterFieldCenter(5, 0), 4, 5);
    }

    @Test public void missDistancesThrowRatherThanPretendZeroTravel() {
        ToolSweep2d.Encounter miss = forward(10, 0, 1, 2).encounterFieldCenter(5, 2);
        assertFalse(miss.hasEncounter());
        IllegalStateException entry = assertThrows(IllegalStateException.class, miss::entryTravelInches);
        assertTrue(entry.getMessage().contains("hasEncounter()"));
        assertThrows(IllegalStateException.class, miss::exitTravelInches);
        assertTrue(miss.toString().contains("miss"));
    }

    @Test public void nonfiniteArgumentsAreRejectedAtTheOwningStageWithNames() {
        for (double bad : new double[]{Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            namedInvalid("fieldToRobotStart.xInches", () -> ToolSweep2d.straightFrom(new Pose2d(bad, 0, 0)));
            namedInvalid("fieldToRobotStart.yInches", () -> ToolSweep2d.straightFrom(new Pose2d(0, bad, 0)));
            namedInvalid("fieldToRobotStart.headingRad", () -> ToolSweep2d.straightFrom(new Pose2d(0, 0, bad)));
            namedInvalid("fieldXInches", () -> ToolSweep2d.straightFrom(Pose2d.zero()).toFieldPoint(bad, 0));
            namedInvalid("fieldYInches", () -> ToolSweep2d.straightFrom(Pose2d.zero()).toFieldPoint(0, bad));
            ToolSweep2d.ToolStage tool = ToolSweep2d.straightFrom(Pose2d.zero()).toFieldPoint(10, 0);
            namedInvalid("robotToTool.xInches", () -> tool.throughTool(new Pose2d(bad, 0, 0)));
            namedInvalid("robotToTool.yInches", () -> tool.throughTool(new Pose2d(0, bad, 0)));
            namedInvalid("robotToTool.headingRad", () -> tool.throughTool(new Pose2d(0, 0, bad)));
            ToolSweep2d.CenterWindowStage window = tool.throughTool(Pose2d.zero());
            namedInvalid("minForwardInches", () -> window.centerWindowInches(bad, 1, 2));
            namedInvalid("maxForwardInches", () -> window.centerWindowInches(0, bad, 2));
            namedInvalid("fullWidthInches", () -> window.centerWindowInches(0, 1, bad));
            ToolSweep2d sweep = window.centerWindowInches(0, 1, 2);
            namedInvalid("fieldXInches", () -> sweep.encounterFieldCenter(bad, 0));
            namedInvalid("fieldYInches", () -> sweep.encounterFieldCenter(0, bad));
        }
    }

    @Test public void nullGeometryAndInvalidWindowFailBeforeQuery() {
        NullPointerException start = assertThrows(NullPointerException.class, () -> ToolSweep2d.straightFrom(null));
        assertEquals("fieldToRobotStart", start.getMessage());
        ToolSweep2d.ToolStage tool = ToolSweep2d.straightFrom(Pose2d.zero()).toFieldPoint(10, 0);
        assertEquals("robotToTool", assertThrows(NullPointerException.class, () -> tool.throughTool(null)).getMessage());
        ToolSweep2d.CenterWindowStage window = tool.throughTool(Pose2d.zero());
        namedInvalid("minForwardInches", () -> window.centerWindowInches(2, 1, 2));
        for (double width : new double[]{0, -0.0, -1, Double.MIN_VALUE}) {
            namedInvalid("fullWidthInches", () -> window.centerWindowInches(0, 1, width));
        }
    }

    @Test public void nonrepresentableDerivedGeometryCannotBecomeAnOrdinaryMiss() {
        namedInvalid("displacement", () -> ToolSweep2d.straightFrom(new Pose2d(-Double.MAX_VALUE, 0, 0))
                .toFieldPoint(Double.MAX_VALUE, 0));
        namedInvalid("travelInches", () -> ToolSweep2d.straightFrom(Pose2d.zero())
                .toFieldPoint(Double.MAX_VALUE, Double.MAX_VALUE));
        namedInvalid("fieldToToolStart", () -> ToolSweep2d.straightFrom(new Pose2d(Double.MAX_VALUE, 0, 0))
                .toFieldPoint(Double.MAX_VALUE, 0).throughTool(new Pose2d(Double.MAX_VALUE, 0, 0)));
        namedInvalid("fieldToToolEnd", () -> ToolSweep2d.straightFrom(new Pose2d(-Double.MAX_VALUE / 2, 0, 0))
                .toFieldPoint(Double.MAX_VALUE / 2, 0).throughTool(new Pose2d(Double.MAX_VALUE, 0, 0)));
        namedInvalid("headingRad", () -> ToolSweep2d.straightFrom(new Pose2d(0, 0, Double.MAX_VALUE))
                .toFieldPoint(0, 0).throughTool(new Pose2d(0, 0, Double.MAX_VALUE)));
        ToolSweep2d remote = ToolSweep2d.straightFrom(new Pose2d(-Double.MAX_VALUE, 0, 0))
                .toFieldPoint(-Double.MAX_VALUE, 0).throughTool(Pose2d.zero())
                .centerWindowInches(0, 1, 2);
        namedInvalid("center field X offset", () -> remote.encounterFieldCenter(Double.MAX_VALUE, 0));
        ToolSweep2d endOverflow = forward(Double.MAX_VALUE, 0, 1, 2);
        namedInvalid("end center", () -> endOverflow.encounterFieldCenter(-Double.MAX_VALUE, 5));
        ToolSweep2d rotated = ToolSweep2d.straightFrom(new Pose2d(0, 0, Math.PI / 4))
                .toFieldPoint(0, 0).throughTool(Pose2d.zero()).centerWindowInches(0, 1, 2);
        namedInvalid("tool-forward", () -> rotated.encounterFieldCenter(Double.MAX_VALUE, Double.MAX_VALUE));
    }

    @Test public void solePublicFactoryAndStagesExposeNoAlternativeConstructionPath() {
        assertEquals(0, ToolSweep2d.class.getConstructors().length);
        assertEquals(0, ToolSweep2d.Encounter.class.getConstructors().length);
        int factoryCount = 0;
        for (Method method : ToolSweep2d.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers()) && Modifier.isStatic(method.getModifiers())) {
                assertEquals("straightFrom", method.getName());
                assertEquals(ToolSweep2d.EndpointStage.class, method.getReturnType());
                factoryCount++;
            }
        }
        assertEquals(1, factoryCount);
        assertStage(ToolSweep2d.EndpointStage.class, "toFieldPoint", ToolSweep2d.ToolStage.class);
        assertStage(ToolSweep2d.ToolStage.class, "throughTool", ToolSweep2d.CenterWindowStage.class);
        assertStage(ToolSweep2d.CenterWindowStage.class, "centerWindowInches", ToolSweep2d.class);
    }

    private static void assertStage(Class<?> type, String name, Class<?> returned) {
        assertEquals(1, type.getDeclaredMethods().length);
        assertEquals(name, type.getDeclaredMethods()[0].getName());
        assertEquals(returned, type.getDeclaredMethods()[0].getReturnType());
    }

    private static ToolSweep2d forward(double endX, double min, double max, double width) {
        return ToolSweep2d.straightFrom(Pose2d.zero()).toFieldPoint(endX, 0)
                .throughTool(Pose2d.zero()).centerWindowInches(min, max, width);
    }

    private static void assertEncounter(ToolSweep2d.Encounter result, double entry, double exit) {
        assertTrue(result.toString(), result.hasEncounter());
        assertEquals(entry, result.entryTravelInches(), EPS);
        assertEquals(exit, result.exitTravelInches(), EPS);
    }

    private static void namedInvalid(String expected, Runnable action) {
        IllegalArgumentException failure = assertThrows(IllegalArgumentException.class, action::run);
        assertTrue(failure.getMessage(), failure.getMessage().contains(expected));
    }
}
