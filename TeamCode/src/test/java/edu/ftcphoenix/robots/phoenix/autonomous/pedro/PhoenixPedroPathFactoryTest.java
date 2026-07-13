package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathPoint;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Pinned Pedro geometry checks for Phoenix's start-time return-path boundary. */
public final class PhoenixPedroPathFactoryTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void coincidentReturnUsesFinitePointPathAndKeepsTargetHeading() {
        Pose sampledCurrentPose = new Pose(17.5, -4.25, 0.35);
        Pose returnPose = new Pose(17.5, -4.25, 1.20);
        Curve curve = PhoenixPedroPathFactory.returnCurveFrom(
                sampledCurrentPose,
                returnPose
        );

        assertTrue(curve instanceof BezierPoint);

        Path path = new Path(curve, PathConstraints.defaultConstraints.copy());
        path.setLinearHeadingInterpolation(
                sampledCurrentPose.getHeading(),
                returnPose.getHeading()
        );
        path.init();
        PathPoint closestPoint = path.updateClosestPose(sampledCurrentPose);

        assertFinite(closestPoint.getTValue());
        assertFinite(closestPoint.getPose().getX());
        assertFinite(closestPoint.getPose().getY());
        assertFinite(closestPoint.getTangentVector().getXComponent());
        assertFinite(closestPoint.getTangentVector().getYComponent());
        assertEquals(1.0, closestPoint.getTValue(), EPSILON);
        assertEquals(returnPose.getHeading(), path.getHeadingGoal(closestPoint), EPSILON);
        assertTrue(path.isAtParametricEnd());
    }

    @Test
    public void displacedReturnLineStartsAtTheSampledLivePose() {
        Pose sampledCurrentPose = new Pose(9.75, -6.5, 0.45);
        Pose returnPose = new Pose(-2.0, 3.0, -0.80);
        Curve curve = PhoenixPedroPathFactory.returnCurveFrom(
                sampledCurrentPose,
                returnPose
        );

        assertTrue(curve instanceof BezierLine);
        assertSame(sampledCurrentPose, curve.getFirstControlPoint());
        assertEquals(sampledCurrentPose.getX(), curve.getFirstControlPoint().getX(), EPSILON);
        assertEquals(sampledCurrentPose.getY(), curve.getFirstControlPoint().getY(), EPSILON);
        assertEquals(
                sampledCurrentPose.getHeading(),
                curve.getFirstControlPoint().getHeading(),
                EPSILON
        );
        assertSame(
                sampledCurrentPose.getCoordinateSystem(),
                curve.getFirstControlPoint().getCoordinateSystem()
        );
    }

    @Test
    public void nonFiniteEndpointFieldsFailWithTheirEndpointAndFieldNames() {
        Pose finiteCurrent = new Pose(1.0, 2.0, 0.25);
        Pose finiteReturn = new Pose(3.0, 4.0, 0.75);

        assertRejected(
                new Pose(Double.NaN, 2.0, 0.25),
                finiteReturn,
                "Pedro Follower current pose.x"
        );
        assertRejected(
                new Pose(1.0, Double.POSITIVE_INFINITY, 0.25),
                finiteReturn,
                "Pedro Follower current pose.y"
        );
        assertRejected(
                new Pose(1.0, 2.0, Double.NaN),
                finiteReturn,
                "Pedro Follower current pose.heading"
        );
        assertRejected(
                finiteCurrent,
                new Pose(Double.NEGATIVE_INFINITY, 4.0, 0.75),
                "pedroReturnPose.x"
        );
        assertRejected(
                finiteCurrent,
                new Pose(3.0, Double.NaN, 0.75),
                "pedroReturnPose.y"
        );
        assertRejected(
                finiteCurrent,
                new Pose(3.0, 4.0, Double.POSITIVE_INFINITY),
                "pedroReturnPose.heading"
        );
    }

    private static void assertFinite(double value) {
        assertTrue("Expected a finite Pedro path value, but was " + value, Double.isFinite(value));
    }

    private static void assertRejected(Pose currentPose,
                                       Pose returnPose,
                                       String expectedMessagePart) {
        try {
            PhoenixPedroPathFactory.returnCurveFrom(currentPose, returnPose);
            fail("Expected invalid return geometry to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(
                    "Expected message containing '" + expectedMessagePart + "' but was '"
                            + expected.getMessage() + "'",
                    expected.getMessage().contains(expectedMessagePart)
            );
        }
    }
}
