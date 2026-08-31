package edu.ftcsushi.fw.drive.guidance;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies immediate validation of the latched robot-relative translation target. */
public final class DriveGuidanceRobotRelativePointValidationTest {

    private static final double[] NON_FINITE = {
            Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY
    };

    @Test
    public void stagedBuilderRejectsBothNonFiniteOffsetsImmediately() {
        for (double invalid : NON_FINITE) {
            assertRejected("forwardInches", () -> DriveGuidance.spec()
                    .translateTo()
                    .robotRelativePointInches(invalid, 2.0));
            assertRejected("leftInches", () -> DriveGuidance.spec()
                    .translateTo()
                    .robotRelativePointInches(1.0, invalid));
        }
    }

    @Test
    public void signedFiniteOffsetsArePreservedWithoutNormalization() {
        DriveGuidanceSpec.RobotRelativePoint point =
                new DriveGuidanceSpec.RobotRelativePoint(-0.0, -17.5);

        assertEquals(Double.doubleToRawLongBits(-0.0),
                Double.doubleToRawLongBits(point.forwardInches));
        assertEquals(-17.5, point.leftInches, 0.0);
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
