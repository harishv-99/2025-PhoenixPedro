package edu.ftcsushi.fw.spatial;

import java.lang.reflect.Method;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.drive.guidance.GoToPoseTasks;
import edu.ftcsushi.fw.ftc.FtcFieldRegions;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.task.Task;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks the intentionally smaller spatial API and its existing go-to-pose composition. */
public final class SpatialApiReplacementTest {

    private static final String SPATIAL_PACKAGE = "edu.ftcsushi.fw.spatial.";

    @Test
    public void dormantPublicFamilyIsAbsent() {
        String[] removedSimpleNames = {
                "RobotFootprint2d",
                "CircleFootprint2d",
                "RectangleFootprint2d",
                "RobotGeometry2d",
                "RobotZone2d",
                "RobotZones2d",
                "ZoneLatch",
                "RobotHeading2d",
                "RobotHeadings2d",
                "HeadingLatch",
                "CircleRegion2d",
                "ConvexPolygonRegion2d",
                "ConvexRegions2d",
                "ConvexRegion2d"
        };

        for (String simpleName : removedSimpleNames) {
            try {
                Class.forName(SPATIAL_PACKAGE + simpleName);
                fail(simpleName + " must be absent from the supported spatial API");
            } catch (ClassNotFoundException expected) {
                // Expected: no compatibility alias or dormant implementation remains.
            }
        }
    }

    @Test
    public void axisAlignedBoxAndFtcFieldExposeOnlyTheProvenRegionContract() throws Exception {
        assertEquals(1, AxisAlignedBoxRegion2d.class.getInterfaces().length);
        assertSame(Region2d.class, AxisAlignedBoxRegion2d.class.getInterfaces()[0]);

        Method fullField = FtcFieldRegions.class.getDeclaredMethod("fullField");
        Method fullFieldWithMargin = FtcFieldRegions.class.getDeclaredMethod(
                "fullFieldWithMargin",
                double.class
        );
        assertSame(Region2d.class, fullField.getReturnType());
        assertSame(Region2d.class, fullFieldWithMargin.getReturnType());

        Region2d standard = FtcFieldRegions.fullField();
        assertTrue(standard.contains(72.0, 72.0));
        assertFalse(standard.contains(Math.nextUp(72.0), 0.0));

        Region2d expanded = FtcFieldRegions.fullFieldWithMargin(1.0);
        assertTrue(expanded.contains(73.0, -73.0));
        assertFalse(expanded.contains(Math.nextUp(73.0), 0.0));
    }

    @Test
    public void fixedAndFrozenNearestHeadingsComposeWithExistingGoToPoseTask() {
        AxisAlignedBoxRegion2d parkBox = new AxisAlignedBoxRegion2d(-10.0, 10.0, -10.0, 10.0);
        RobotFrameRectangle2d rectangle = RobotFrameRectangle2d.centeredInches(18.0, 16.0);
        double[] acceptableHeadingRads = {0.0, Math.PI};

        Pose2d fixedHeadingTarget = new Pose2d(0.0, 0.0, acceptableHeadingRads[0]);
        assertTrue(rectangle.fullyInside(parkBox, fixedHeadingTarget));
        for (double acceptableHeadingRad : acceptableHeadingRads) {
            assertTrue(rectangle.fullyInside(
                    parkBox,
                    new Pose2d(0.0, 0.0, acceptableHeadingRad)
            ));
        }
        assertFalse(rectangle.fullyInside(
                parkBox,
                new Pose2d(0.0, 0.0, Math.PI / 4.0)
        ));

        double selectedHeadingRad = SpatialMath2d.nearestHeadingRad(
                2.8,
                acceptableHeadingRads
        );
        Pose2d frozenNearestTarget = new Pose2d(0.0, 0.0, selectedHeadingRad);
        assertEquals(Math.PI, frozenNearestTarget.headingRad, 0.0);
        assertTrue(rectangle.fullyInside(parkBox, frozenNearestTarget));

        acceptableHeadingRads[1] = Math.PI / 2.0;
        assertEquals(Math.PI, frozenNearestTarget.headingRad, 0.0);

        NoPoseEstimator estimator = new NoPoseEstimator();
        RecordingDriveSink driveSink = new RecordingDriveSink();
        Task fixedTask = GoToPoseTasks.goToPoseFieldRelative(
                estimator,
                driveSink,
                fixedHeadingTarget,
                DriveGuidancePlan.Tuning.defaults(),
                null
        );
        Task nearestTask = GoToPoseTasks.goToPoseFieldRelative(
                estimator,
                driveSink,
                frozenNearestTarget,
                DriveGuidancePlan.Tuning.defaults(),
                null
        );

        assertNotNull(fixedTask);
        assertNotNull(nearestTask);
        assertEquals(0, estimator.updateCalls);
        assertEquals(0, driveSink.driveCalls);
        assertEquals(0, driveSink.stopCalls);
    }

    private static final class NoPoseEstimator implements AbsolutePoseEstimator {
        int updateCalls;

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
        }

        @Override
        public PoseEstimate getEstimate() {
            return PoseEstimate.noPose(edu.ftcsushi.fw.core.time.LoopTimestamp.unavailable());
        }
    }

    private static final class RecordingDriveSink implements DriveCommandSink {
        int driveCalls;
        int stopCalls;

        @Override
        public void drive(DriveSignal signal) {
            driveCalls++;
        }

        @Override
        public void stop() {
            stopCalls++;
        }
    }
}
