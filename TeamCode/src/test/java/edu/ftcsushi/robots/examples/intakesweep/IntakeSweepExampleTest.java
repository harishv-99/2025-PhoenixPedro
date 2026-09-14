package edu.ftcsushi.robots.examples.intakesweep;

import org.junit.Test;

import edu.ftcsushi.fw.spatial.ToolSweep2d;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/**
 * Optional hardware-free checkpoint for the real intake-sweep example, not a robot simulation.
 *
 * <p>Keep real: the maintained example's movement, tool offset, center window, and Sushi's
 * continuous sweep calculation. Replace: estimated ball-center observations with explicitly
 * authored field points. No sensor, pose estimator, clock, controller, or physical collection is
 * present. Passing results establish only these numerical fixtures; robot-specific center
 * estimation, intake effectiveness, travel clearance, and capture feedback remain separate gates.</p>
 */
public final class IntakeSweepExampleTest {
    /** Question: over which part of the modeled move does this ball center enter the window? */
    @Test
    public void reportsTravelIntervalForAuthoredBallCenter() {
        // ARRANGE: this is an authored center estimate, not a camera-box center or ball edge.
        double fieldCenterXInches = 25.0;
        double fieldCenterYInches = 22.0;

        // REQUEST: calculate now using the real example's private geometry; nothing drives.
        ToolSweep2d.Encounter encounter = IntakeSweepExample.encounterBallCenter(
                fieldCenterXInches, fieldCenterYInches);

        // ASSERT: the tool starts at field X=16, so its forward window starts at [15, 18].
        // It first contains X=25 after 7 inches, and last contains it after 10 inches.
        // The center is 1 inch left of the tool, inside the chosen [-3, +3] side bounds.
        assertTrue(encounter.hasEncounter());
        assertEquals(7.0, encounter.entryTravelInches(), 1e-9);
        assertEquals(10.0, encounter.exitTravelInches(), 1e-9);
        // NEXT GATE: no result here proves a real ball entered or remained inside the intake.
    }

    /** Question: is a center on the physical opening edge accepted as though it were inside? */
    @Test
    public void rejectsPhysicalEdgeAndBallOverlapOutsideChosenCenterWindow() {
        // ARRANGE: the intake's centerline is field Y=21; its illustrative opening edge is Y=27.
        // A 4-inch ball centered at Y=25 overlaps the smaller window, but its center is outside.
        ToolSweep2d.Encounter physicalEdge = IntakeSweepExample.encounterBallCenter(25.0, 27.0);
        ToolSweep2d.Encounter ballEdgeOverlap = IntakeSweepExample.encounterBallCenter(25.0, 25.0);

        // ASSERT: neither physical contact nor overlap inflates the already-reduced center window.
        assertFalse(physicalEdge.hasEncounter());
        assertFalse(ballEdgeOverlap.hasEncounter());
    }

    /** Question: does the chosen inset boundary count without pretending it is an opening edge? */
    @Test
    public void includesChosenInnerCenterBoundary() {
        // ARRANGE: Y=24 is 3 inches left of the tool; the modeled ball extends another 2 inches.
        // That leaves 1 illustrative extra inch before the physical opening edge at Y=27.
        ToolSweep2d.Encounter innerBoundary = IntakeSweepExample.encounterBallCenter(25.0, 24.0);

        // ASSERT: exact mathematical inclusion does not claim tested physical clearance.
        assertTrue(innerBoundary.hasEncounter());
        assertEquals(7.0, innerBoundary.entryTravelInches(), 1e-9);
        assertEquals(10.0, innerBoundary.exitTravelInches(), 1e-9);
    }

    /** Question: does the calculation stop at the authored movement's endpoints? */
    @Test
    public void includesEndpointsButDoesNotExtendTheModeledMove() {
        // ARRANGE: the initial forward window is [15, 18]; the final window is [35, 38].
        ToolSweep2d.Encounter initialBoundary = IntakeSweepExample.encounterBallCenter(15.0, 21.0);
        ToolSweep2d.Encounter finalBoundary = IntakeSweepExample.encounterBallCenter(38.0, 21.0);
        ToolSweep2d.Encounter beyondEnd = IntakeSweepExample.encounterBallCenter(38.5, 21.0);

        // ASSERT: a point may touch a closed boundary for one travel value only.
        assertTrue(initialBoundary.hasEncounter());
        assertEquals(0.0, initialBoundary.entryTravelInches(), 0.0);
        assertEquals(0.0, initialBoundary.exitTravelInches(), 0.0);
        assertTrue(finalBoundary.hasEncounter());
        assertEquals(20.0, finalBoundary.entryTravelInches(), 0.0);
        assertEquals(20.0, finalBoundary.exitTravelInches(), 0.0);
        assertFalse(beyondEnd.hasEncounter());
    }

    /** Question: can a later point query change a previously returned encounter? */
    @Test
    public void repeatedQueriesKeepGeometryWithoutCountingObjects() {
        ToolSweep2d.Encounter first = IntakeSweepExample.encounterBallCenter(25.0, 22.0);
        ToolSweep2d.Encounter miss = IntakeSweepExample.encounterBallCenter(25.0, 27.0);
        ToolSweep2d.Encounter repeated = IntakeSweepExample.encounterBallCenter(25.0, 22.0);

        assertFalse(miss.hasEncounter());
        assertTrue(first.hasEncounter());
        assertTrue(repeated.hasEncounter());
        assertEquals(7.0, first.entryTravelInches(), 1e-9);
        assertEquals(10.0, first.exitTravelInches(), 1e-9);
        assertEquals(first.entryTravelInches(), repeated.entryTravelInches(), 0.0);
        assertEquals(first.exitTravelInches(), repeated.exitTravelInches(), 0.0);
        // There is no object identity or unique-ball count in either result.
    }

    /** Question: is an invalid center rejected rather than silently classified as a miss? */
    @Test
    public void rejectsNonFiniteCenterInputs() {
        assertThrows(IllegalArgumentException.class,
                () -> IntakeSweepExample.encounterBallCenter(Double.NaN, 22.0));
        assertThrows(IllegalArgumentException.class,
                () -> IntakeSweepExample.encounterBallCenter(25.0, Double.POSITIVE_INFINITY));
    }
}
