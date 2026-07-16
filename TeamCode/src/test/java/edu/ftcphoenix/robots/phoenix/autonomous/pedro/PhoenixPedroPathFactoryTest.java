package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathPoint;

import org.junit.Test;

import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoStrategyId;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Pinned Pedro geometry checks for Phoenix's start-time return-path boundary. */
public final class PhoenixPedroPathFactoryTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void everyCheckedInSelectionIsExplicitlyIntegrationOnly() {
        for (PhoenixAutoSpec.Alliance alliance : PhoenixAutoSpec.Alliance.values()) {
            for (PhoenixAutoSpec.StartPosition startPosition
                    : PhoenixAutoSpec.StartPosition.values()) {
                for (PhoenixAutoSpec.PartnerPlan partnerPlan
                        : PhoenixAutoSpec.PartnerPlan.values()) {
                    for (PhoenixAutoStrategyId strategy : PhoenixAutoStrategyId.values()) {
                        PhoenixAutoSpec spec = PhoenixAutoSpec.builder()
                                .alliance(alliance)
                                .startPosition(startPosition)
                                .partnerPlan(partnerPlan)
                                .strategy(strategy)
                                .build();

                        PhoenixPedroPathFactory.RouteAvailability availability =
                                PhoenixPedroPathFactory.routeAvailabilityFor(spec);

                        assertEquals(
                                PhoenixPedroPathFactory.RouteAvailability.Maturity.INTEGRATION_ONLY,
                                availability.maturity
                        );
                        assertFalse(availability.isMatchReady());
                        assertTrue(availability.reason.contains("integration placeholder"));
                        assertTrue(availability.reason.contains(spec.summary()));
                        assertFinite(availability.expectedPedroStartPose.getX());
                        assertFinite(availability.expectedPedroStartPose.getY());
                        assertFinite(availability.expectedPedroStartPose.getHeading());
                        assertEquals(
                                0.0,
                                availability.expectedPedroStartPose.getX(),
                                EPSILON
                        );
                        assertEquals(
                                startPosition == PhoenixAutoSpec.StartPosition.AUDIENCE
                                        ? 0.0 : 24.0,
                                availability.expectedPedroStartPose.getY(),
                                EPSILON
                        );
                        assertEquals(
                                alliance == PhoenixAutoSpec.Alliance.RED ? 0.0 : Math.PI,
                                availability.expectedPedroStartPose.getHeading(),
                                EPSILON
                        );
                    }
                }
            }
        }
    }

    @Test
    public void declaredStartAcceptsMatchingTranslationAndWrappedHeading() {
        Pose routeStart = new Pose(8.5, -3.25, 0.0);
        PathChain outboundPath = singleLinePath(routeStart, new Pose(12.0, -3.25, 0.0));

        PhoenixPedroPathFactory.validateDeclaredStart(
                outboundPath,
                new Pose(8.5, -3.25, 2.0 * Math.PI)
        );
    }

    @Test
    public void declaredStartUsesEffectivePathChainHeading() {
        PathChain outboundPath = singleLinePath(
                new Pose(8.5, -3.25, 0.0),
                new Pose(12.0, -3.25, 0.0)
        );
        outboundPath.setHeadingInterpolator(HeadingInterpolator.constant(0.5));

        assertDeclaredStartRejected(
                outboundPath,
                new Pose(8.5, -3.25, 0.0),
                "Update the route geometry and RouteAvailability together"
        );
    }

    @Test
    public void declaredStartRejectsMissingAndEmptyPaths() {
        assertDeclaredStartRejected(
                null,
                new Pose(),
                "must not be null"
        );
        assertDeclaredStartRejected(
                new PathChain(),
                new Pose(),
                "must contain at least one path"
        );
    }

    @Test
    public void declaredStartRejectsNonFiniteFirstGeometry() {
        Path firstPath = new Path(
                new BezierPoint(new Pose(Double.NaN, 2.0, 0.0)),
                PathConstraints.defaultConstraints.copy()
        );
        firstPath.setConstantHeadingInterpolation(0.0);

        assertDeclaredStartRejected(
                new PathChain(firstPath),
                new Pose(1.0, 2.0, 0.0),
                "outboundPath first geometry.x must be finite"
        );
    }

    @Test
    public void declaredStartRejectsTranslationOrHeadingMismatchActionably() {
        assertDeclaredStartRejected(
                singleLinePath(
                        new Pose(1.0, 2.0, 0.25),
                        new Pose(5.0, 2.0, 0.25)
                ),
                new Pose(1.5, 2.0, 0.25),
                "expectedPedroStartPose"
        );
        assertDeclaredStartRejected(
                singleLinePath(
                        new Pose(1.0, 2.0, 0.25),
                        new Pose(5.0, 2.0, 0.25)
                ),
                new Pose(1.0, 2.0, 0.75),
                "Update the route geometry and RouteAvailability together"
        );
    }

    @Test
    public void integrationPlaceholderRequiresAFiniteUsefulDistanceAndEndpoint() {
        Pose end = PhoenixPedroPathFactory.integrationPlaceholderEndPose(
                new Pose(3.0, 4.0, 0.5),
                12.0
        );
        assertEquals(15.0, end.getX(), EPSILON);
        assertEquals(4.0, end.getY(), EPSILON);
        assertEquals(0.5, end.getHeading(), EPSILON);

        assertIntegrationDistanceRejected(0.0, "must be > 0");
        assertIntegrationDistanceRejected(-1.0, "must be > 0");
        assertIntegrationDistanceRejected(
                Double.NaN,
                "pedroIntegrationTestDistanceIn must be finite"
        );
        assertIntegrationDistanceRejected(
                Double.POSITIVE_INFINITY,
                "pedroIntegrationTestDistanceIn must be finite"
        );

        try {
            PhoenixPedroPathFactory.integrationPlaceholderEndPose(
                    new Pose(Double.MAX_VALUE, 0.0, 0.0),
                    Double.MAX_VALUE
            );
            fail("Expected an overflowing integration endpoint to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("integration route end.x must be finite"));
        }
    }

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

    private static PathChain singleLinePath(Pose startPose, Pose endPose) {
        Path path = new Path(
                new BezierLine(startPose, endPose),
                PathConstraints.defaultConstraints.copy()
        );
        path.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
        return new PathChain(path);
    }

    private static void assertDeclaredStartRejected(PathChain outboundPath,
                                                    Pose expectedStartPose,
                                                    String expectedMessagePart) {
        try {
            PhoenixPedroPathFactory.validateDeclaredStart(outboundPath, expectedStartPose);
            fail("Expected invalid declared-start geometry to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(
                    "Expected message containing '" + expectedMessagePart + "' but was '"
                            + expected.getMessage() + "'",
                    expected.getMessage().contains(expectedMessagePart)
            );
        }
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

    private static void assertIntegrationDistanceRejected(double distanceIn,
                                                          String expectedMessagePart) {
        try {
            PhoenixPedroPathFactory.integrationPlaceholderEndPose(
                    new Pose(0.0, 0.0, 0.0),
                    distanceIn
            );
            fail("Expected invalid integration distance to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains(expectedMessagePart));
        }
    }
}
