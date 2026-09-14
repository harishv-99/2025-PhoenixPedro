package edu.ftcsushi.fw.spatial;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Independently authored coordinate and numerical cases; not physical capture validation. */
public final class ToolSweepBoundaryTest {
    @Test public void ninetyDegreeFieldRotationPreservesTravelWithoutReusingTheTransform() {
        // Rotate the (10,20)->(30,20), center(25,22) fixture by +90 degrees about field origin.
        ToolSweep2d rotated = ToolSweep2d.straightFrom(new Pose2d(-20, 10, Math.PI / 2))
                .toFieldPoint(-20, 30).throughTool(new Pose2d(6, 1, 0))
                .centerWindowInches(-1, 2, 6);
        assertInterval(rotated.encounterFieldCenter(-22, 25), 7, 10, 1e-12);
        assertFalse(rotated.encounterFieldCenter(-27, 25).hasEncounter());
    }

    @Test public void translatingTheWholeFieldDoesNotChangeAnEncounter() {
        // Add (-100,+200) to every field coordinate, not to the robot-relative mount.
        ToolSweep2d translated = ToolSweep2d.straightFrom(new Pose2d(-90, 220, 0))
                .toFieldPoint(-70, 220).throughTool(new Pose2d(6, 1, 0))
                .centerWindowInches(-1, 2, 6);
        assertInterval(translated.encounterFieldCenter(-75, 222), 7, 10, 0);
    }

    @Test public void obliqueToolUsesEuclideanRobotTravelAndItsOwnAxes() {
        // The tool's forward unit vector is (3/5,4/5), and the segment is 10 inches long.
        // Field (3,4) lies 5 inches along it; forward window [0,1] reaches it at travel4..5.
        ToolSweep2d oblique = ToolSweep2d.straightFrom(Pose2d.zero()).toFieldPoint(6, 8)
                .throughTool(new Pose2d(0, 0, Math.atan2(4, 3)))
                .centerWindowInches(0, 1, 2);
        assertInterval(oblique.encounterFieldCenter(3, 4), 4, 5, 1e-12);
        // Add two inches along tool-left (-4/5,+3/5); center is outside the +/-1 window.
        assertFalse(oblique.encounterFieldCenter(1.4, 5.2).hasEncounter());
    }

    @Test public void stationaryFixedClawChecksCenterPlacementNotAnIntakeStroke() {
        ToolSweep2d claw = ToolSweep2d.straightFrom(new Pose2d(20, 30, 0))
                .toFieldPoint(20, 30).throughTool(new Pose2d(8, -2, 0))
                .centerWindowInches(-0.25, 0.25, 1);
        assertInterval(claw.encounterFieldCenter(28, 28), 0, 0, 0);
        assertInterval(claw.encounterFieldCenter(28.25, 28.5), 0, 0, 0);
        assertFalse(claw.encounterFieldCenter(Math.nextUp(28.25), 28).hasEncounter());
        // No actuator, gripping, height, object orientation, or confirmation participates.
    }

    @Test public void encounterNearWallCanCoexistWithRobotOutsideKnownBox() {
        ToolSweep2d sweep = ToolSweep2d.straightFrom(new Pose2d(10, 10, 0))
                .toFieldPoint(26, 10).throughTool(new Pose2d(4, 0, 0))
                .centerWindowInches(0, 1, 4);
        assertInterval(sweep.encounterFieldCenter(30, 10), 15, 16, 0);
        // At the first encounter the robot center is X=25, but its front reaches X=31.
        AxisAlignedBoxRegion2d knownBox = new AxisAlignedBoxRegion2d(0, 30, 0, 30);
        assertFalse(RobotFrameRectangle2d.centeredInches(12, 12)
                .fullyInside(knownBox, new Pose2d(25, 10, 0)));
    }

    @Test public void subnormalRatioRetainsTinyRepresentableTravelInEitherDirection() {
        double tiny = Math.scalb(1.0, -900);
        double huge = Math.scalb(1.0, 900);
        ToolSweep2d positive = ToolSweep2d.straightFrom(Pose2d.zero()).toFieldPoint(huge, 0)
                .throughTool(Pose2d.zero()).centerWindowInches(0, 0, 2);
        ToolSweep2d negative = ToolSweep2d.straightFrom(Pose2d.zero()).toFieldPoint(-huge, 0)
                .throughTool(Pose2d.zero()).centerWindowInches(0, 0, 2);
        assertInterval(positive.encounterFieldCenter(tiny, 0), tiny, tiny, 0);
        assertInterval(negative.encounterFieldCenter(-tiny, 0), tiny, tiny, 0);
    }

    @Test public void invalidOtherAxisCannotHideBehindAnOrdinaryMiss() {
        ToolSweep2d sweep = ToolSweep2d.straightFrom(Pose2d.zero())
                .toFieldPoint(0, -Double.MAX_VALUE).throughTool(Pose2d.zero())
                .centerWindowInches(0, 1, 2);
        // X is an ordinary miss, but Y's required endpoint coordinate overflows.
        IllegalArgumentException error = assertThrows(IllegalArgumentException.class,
                () -> sweep.encounterFieldCenter(2, Double.MAX_VALUE));
        assertTrue(error.getMessage().contains("end center tool-left"));
    }

    @Test public void finiteHeadingSumOverflowIsRejectedAtTheTransformAnswer() {
        ToolSweep2d.ToolStage stage = ToolSweep2d.straightFrom(
                new Pose2d(0, 0, Double.MAX_VALUE)).toFieldPoint(0, 0);
        IllegalArgumentException error = assertThrows(IllegalArgumentException.class,
                () -> stage.throughTool(new Pose2d(0, 0, Double.MAX_VALUE)));
        assertTrue(error.getMessage().contains("derived fieldToToolStart"));
    }

    private static void assertInterval(ToolSweep2d.Encounter result, double entry, double exit,
                                       double tolerance) {
        assertTrue(result.hasEncounter());
        assertEquals(entry, result.entryTravelInches(), tolerance);
        assertEquals(exit, result.exitTravelInches(), tolerance);
    }
}
