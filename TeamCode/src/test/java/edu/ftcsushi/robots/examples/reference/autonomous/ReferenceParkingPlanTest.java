package edu.ftcsushi.robots.examples.reference.autonomous;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.spatial.AxisAlignedBoxRegion2d;
import edu.ftcsushi.fw.spatial.RobotFrameRectangle2d;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public final class ReferenceParkingPlanTest {
    @Test
    public void validatesEveryCandidateAndFreezesNearestAuthoredHeading() {
        ReferenceParkingPlan plan = new ReferenceParkingPlan(
                RobotFrameRectangle2d.centeredInches(4.0, 2.0),
                new AxisAlignedBoxRegion2d(-5.0, 5.0, -5.0, 5.0),
                0.0,
                0.0,
                0.0,
                Math.PI);

        Pose2d frozen = plan.freezeTargetFrom(new Pose2d(20.0, 20.0, 2.8));

        assertEquals(Math.PI, frozen.headingRad, 0.0);
        assertTrue(plan.hasAnyCornerInside(Pose2d.zero()));
        assertFalse(plan.hasAnyCornerInside(new Pose2d(20.0, 20.0, 0.0)));
    }

    @Test
    public void rejectsCandidateThatDoesNotPutFullRectangleInKnownClearBox() {
        assertThrows(IllegalArgumentException.class, () -> new ReferenceParkingPlan(
                RobotFrameRectangle2d.centeredInches(8.0, 2.0),
                new AxisAlignedBoxRegion2d(-3.0, 3.0, -3.0, 3.0),
                0.0,
                0.0,
                0.0));
    }
}
