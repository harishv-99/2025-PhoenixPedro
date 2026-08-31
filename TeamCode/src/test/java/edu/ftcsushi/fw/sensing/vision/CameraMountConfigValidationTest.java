package edu.ftcsushi.fw.sensing.vision;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose3d;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies finite authored camera-mount geometry without constraining generic poses. */
public final class CameraMountConfigValidationTest {

    private static final double[] NON_FINITE = {
            Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY
    };
    private static final String[] RADIAN_FIELDS = {
            "robotToCameraPose.xInches",
            "robotToCameraPose.yInches",
            "robotToCameraPose.zInches",
            "robotToCameraPose.yawRad",
            "robotToCameraPose.pitchRad",
            "robotToCameraPose.rollRad"
    };
    private static final String[] DEGREE_FIELDS = {
            "xInches", "yInches", "zInches", "yawDeg", "pitchDeg", "rollDeg"
    };

    @Test
    public void ofPoseRejectsEveryNonFiniteField() {
        for (int field = 0; field < RADIAN_FIELDS.length; field++) {
            for (double invalid : NON_FINITE) {
                final double[] values = validValues();
                values[field] = invalid;
                assertRejected(RADIAN_FIELDS[field], () -> CameraMountConfig.ofPose(pose(values)));
            }
        }
    }

    @Test
    public void radiansFactoryRejectsEveryNonFiniteField() {
        for (int field = 0; field < RADIAN_FIELDS.length; field++) {
            for (double invalid : NON_FINITE) {
                final double[] values = validValues();
                values[field] = invalid;
                assertRejected(RADIAN_FIELDS[field], () -> CameraMountConfig.of(
                        values[0], values[1], values[2], values[3], values[4], values[5]));
            }
        }
    }

    @Test
    public void degreesFactoryNamesAndRejectsEveryInputBeforeConversion() {
        for (int field = 0; field < DEGREE_FIELDS.length; field++) {
            for (double invalid : NON_FINITE) {
                final double[] values = validValues();
                values[field] = invalid;
                assertRejected(DEGREE_FIELDS[field], () -> CameraMountConfig.ofDegrees(
                        values[0], values[1], values[2], values[3], values[4], values[5]));
            }
        }
    }

    @Test
    public void finiteSignedAndUnnormalizedValuesArePreserved() {
        CameraMountConfig radians = CameraMountConfig.of(
                -0.0, -7.0, 12.0, 8.0 * Math.PI, -5.0 * Math.PI, 3.0 * Math.PI);

        assertEquals(Double.doubleToRawLongBits(-0.0),
                Double.doubleToRawLongBits(radians.xInches()));
        assertEquals(-7.0, radians.yInches(), 0.0);
        assertEquals(8.0 * Math.PI, radians.yawRad(), 0.0);
        assertEquals(-5.0 * Math.PI, radians.pitchRad(), 0.0);
        assertEquals(3.0 * Math.PI, radians.rollRad(), 0.0);

        CameraMountConfig degrees = CameraMountConfig.ofDegrees(
                1.0, -2.0, 3.0, 1080.0, -720.0, 540.0);
        assertEquals(Math.toRadians(1080.0), degrees.yawRad(), 0.0);
        assertEquals(Math.toRadians(-720.0), degrees.pitchRad(), 0.0);
        assertEquals(Math.toRadians(540.0), degrees.rollRad(), 0.0);
    }

    @Test
    public void genericPose3dRemainsPermissive() {
        Pose3d runtimeEvidence = new Pose3d(
                Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY,
                Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
        assertTrue(Double.isNaN(runtimeEvidence.xInches));
    }

    private static double[] validValues() {
        return new double[]{1.0, -2.0, 3.0, 4.0, -5.0, 6.0};
    }

    private static Pose3d pose(double[] values) {
        return new Pose3d(
                values[0], values[1], values[2], values[3], values[4], values[5]);
    }

    private static void assertRejected(String expectedField, Action action) {
        try {
            action.run();
            fail("Expected non-finite " + expectedField + " to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(expectedField));
            assertTrue(expected.getMessage(), expected.getMessage().contains("finite"));
        }
    }

    private interface Action {
        void run();
    }
}
