package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public final class PedroFieldTransformTest {

    private static final double EPSILON = 1e-9;
    private final PedroFieldTransform transform = PedroFieldTransform.decodeInvertedFtc();

    @Test
    public void decodeCenterAxesAndHeadingsMatchExplicitContract() {
        assertPedroPose(72.0, 72.0, Math.PI / 2.0,
                transform.phoenixFieldToPedroPose(new Pose2d(0.0, 0.0, 0.0)));
        assertPedroPose(72.0, 82.0, Math.PI / 2.0,
                transform.phoenixFieldToPedroPose(new Pose2d(10.0, 0.0, 0.0)));
        assertPedroPose(62.0, 72.0, Math.PI,
                transform.phoenixFieldToPedroPose(new Pose2d(0.0, 10.0, Math.PI / 2.0)));

        Pose2d phoenix = transform.pedroToPhoenixFieldPose(new Pose(72.0, 72.0, 0.0));
        assertEquals(0.0, phoenix.xInches, EPSILON);
        assertEquals(0.0, phoenix.yInches, EPSILON);
        assertEquals(-Math.PI / 2.0, phoenix.headingRad, EPSILON);
    }

    @Test
    public void poseTransformIsAnInverseAcrossWrapBoundaries() {
        Pose2d[] cases = new Pose2d[] {
                new Pose2d(0.0, 0.0, 0.0),
                new Pose2d(21.25, -37.5, Math.PI),
                new Pose2d(-70.0, 69.0, -Math.PI + 1e-6),
                new Pose2d(8.0, 9.0, 5.0 * Math.PI / 2.0)
        };

        for (Pose2d original : cases) {
            Pose2d roundTrip = transform.pedroToPhoenixFieldPose(
                    transform.phoenixFieldToPedroPose(original)
            );
            assertEquals(original.xInches, roundTrip.xInches, EPSILON);
            assertEquals(original.yInches, roundTrip.yInches, EPSILON);
            assertEquals(0.0,
                    Pose2d.wrapToPi(roundTrip.headingRad - original.headingRad),
                    EPSILON);
        }
    }

    @Test
    public void velocityRotatesWithoutFieldOriginTranslationAndRoundTrips() {
        Pose pedroVelocity = transform.phoenixFieldVelocityToPedro(3.0, 4.0, 0.5);

        assertPedroPose(-4.0, 3.0, 0.5, pedroVelocity);
        PedroFieldTransform.PhoenixFieldVelocity roundTrip =
                transform.pedroFieldVelocityToPhoenix(pedroVelocity);
        assertEquals(3.0, roundTrip.xInchesPerSec, EPSILON);
        assertEquals(4.0, roundTrip.yInchesPerSec, EPSILON);
        assertEquals(0.5, roundTrip.angularRadPerSec, EPSILON);
    }

    @Test
    public void inverseRejectsAmbiguousVendorCoordinateTags() {
        try {
            transform.pedroToPhoenixFieldPose(
                    new Pose(0.0, 0.0, 0.0, InvertedFTCCoordinates.INSTANCE)
            );
            fail("Expected an explicit Pedro coordinate requirement");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("PedroCoordinates"));
        }
    }

    @Test
    public void transformRejectsNonFiniteComponents() {
        try {
            transform.phoenixFieldToPedroPose(new Pose2d(Double.NaN, 0.0, 0.0));
            fail("Expected non-finite Phoenix pose rejection");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("xInches"));
        }

        try {
            transform.phoenixFieldVelocityToPedro(0.0, Double.POSITIVE_INFINITY, 0.0);
            fail("Expected non-finite velocity rejection");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("VelocityY"));
        }
    }

    private static void assertPedroPose(double x, double y, double heading, Pose actual) {
        assertEquals(x, actual.getX(), EPSILON);
        assertEquals(y, actual.getY(), EPSILON);
        assertEquals(heading, actual.getHeading(), EPSILON);
        assertSame(PedroCoordinates.INSTANCE, actual.getCoordinateSystem());
    }
}
