package edu.ftcsushi.fw.ftc.vision;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Vec3;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Pure-JVM checks of the production parser, not evidence about physical Limelight geometry. */
public final class LimelightColorFrameTest {

    @Test
    public void explicitZeroAnglesAreValidButMissingAnglesAreNotZero() {
        LimelightColorFrame center = parseCandidates("{\"tx_nocross\":0,\"ty_nocross\":0}");
        assertTrue(center.reason(), center.isAvailable());
        assertEquals(1.0, center.rays().get(0).x, 0.0);
        assertEquals(0.0, center.rays().get(0).y, 0.0);
        assertEquals(0.0, center.rays().get(0).z, 0.0);
        assertFalse(parseCandidates("{\"tx\":0,\"ty\":0}").isAvailable());
        assertFalse(parseCandidates("{\"tx_nocross\":0}").isAvailable());
        assertFalse(parseCandidates("{\"ty_nocross\":0}").isAvailable());
    }

    @Test
    public void independentOffAxisPlaneAnglesConvertRightToLeftAndPreserveUp() {
        LimelightColorFrame frame = parseCandidates("{\"tx_nocross\":30,\"ty_nocross\":20}");
        assertTrue(frame.reason(), frame.isAvailable());
        Vec3 ray = frame.rays().get(0);
        assertEquals(1.0, ray.x, 0.0);
        assertEquals(-Math.tan(Math.toRadians(30)), ray.y, 1e-12);
        assertEquals(Math.tan(Math.toRadians(20)), ray.z, 1e-12);
        // Off-axis input specifically distinguishes this contract from spherical yaw/elevation.
        assertTrue(Math.abs(ray.z - Math.tan(Math.toRadians(20))
                / Math.cos(Math.toRadians(30))) > 0.01);
    }

    @Test
    public void oppositeQuadrantAndMultipleCandidateOrderAreRetainedWithoutTrackingIds() {
        LimelightColorFrame frame = parseCandidates(
                "{\"tx_nocross\":-12,\"ty_nocross\":-8},"
                        + "{\"tx_nocross\":5,\"ty_nocross\":-3}");
        assertEquals(2, frame.rays().size());
        assertTrue(frame.rays().get(0).y > 0.0);
        assertTrue(frame.rays().get(0).z < 0.0);
        assertTrue(frame.rays().get(1).y < 0.0);
        try {
            frame.rays().clear();
            fail("copied candidate list must be immutable");
        } catch (UnsupportedOperationException expected) {
            // No vendor-owned collection escapes.
        }
    }

    @Test
    public void noCrossAnglesDoNotDependOnWinnerCrosshairPixelOrAreaFields() {
        String fields = "\"tx_nocross\":10,\"ty_nocross\":-5";
        LimelightColorFrame first = parseCandidates("{" + fields
                + ",\"tx\":0,\"ty\":0,\"ta\":1,\"t6c_ts\":[1,2,3]}");
        LimelightColorFrame second = parseCandidates("{" + fields
                + ",\"tx\":45,\"ty\":35,\"ta\":99,\"pts\":[[100,200]]}");
        assertTrue(first.isAvailable());
        assertTrue(second.isAvailable());
        assertEquals(first.rays().get(0).y, second.rays().get(0).y, 0.0);
        assertEquals(first.rays().get(0).z, second.rays().get(0).z, 0.0);
    }

    @Test
    public void confirmedEmptyIsDifferentFromMissingArrayOrWrongPipeline() {
        LimelightColorFrame empty = LimelightColorFrame.parse("{\"pTYPE\":\"color\",\"Retro\":[]}");
        assertTrue(empty.reason(), empty.isAvailable());
        assertTrue(empty.rays().isEmpty());
        assertFalse(LimelightColorFrame.parse("{\"pTYPE\":\"color\"}").isAvailable());
        assertFalse(LimelightColorFrame.parse("{\"pipelineType\":\"color\",\"Retro\":[]}").isAvailable());
        assertFalse(LimelightColorFrame.parse("{\"pTYPE\":\"fiducial\",\"Retro\":[]}").isAvailable());
        assertFalse(LimelightColorFrame.parse("{\"pTYPE\":0,\"Retro\":[]}").isAvailable());
        assertFalse(LimelightColorFrame.parse("{\"Results\":{\"pTYPE\":\"color\",\"Retro\":[]}}")
                .isAvailable());
    }

    @Test
    public void winnerValidityDoesNotEraseValidPerCandidateEvidence() {
        LimelightColorFrame frame = LimelightColorFrame.parse(
                "{\"pTYPE\":\"color\",\"v\":0,\"Retro\":[{\"tx_nocross\":0,\"ty_nocross\":-10}]}");
        assertTrue(frame.reason(), frame.isAvailable());
        assertEquals(1, frame.rays().size());
    }

    @Test
    public void malformedCandidateRejectsEntireFrameRatherThanHidingAmbiguity() {
        LimelightColorFrame frame = parseCandidates(
                "{\"tx_nocross\":0,\"ty_nocross\":-5},{\"tx_nocross\":10}");
        assertFalse(frame.isAvailable());
        assertTrue(frame.rays().isEmpty());
        assertTrue(frame.reason().contains("candidate 1"));
    }

    @Test
    public void nonNumbersNonFiniteAndNonForwardAnglesAreRejected() {
        for (String value : new String[]{"null", "\"0\"", "true", "[]", "{}", "1e999",
                "-1e999", "90", "-90", "100", "-100", "NaN"}) {
            assertFalse(value, parseCandidates(
                    "{\"tx_nocross\":" + value + ",\"ty_nocross\":0}").isAvailable());
            assertFalse(value, parseCandidates(
                    "{\"tx_nocross\":0,\"ty_nocross\":" + value + "}").isAvailable());
        }
        assertFalse(parseCandidates("null").isAvailable());
        assertFalse(parseCandidates("[]").isAvailable());
    }

    @Test
    public void boundedCandidateCountFailsClosedWithoutTruncation() {
        StringBuilder candidates = new StringBuilder();
        for (int i = 0; i < LimelightColorFrame.MAX_CANDIDATES; i++) {
            if (i != 0) {
                candidates.append(',');
            }
            candidates.append("{\"tx_nocross\":0,\"ty_nocross\":-5}");
        }
        assertEquals(64, parseCandidates(candidates.toString()).rays().size());
        candidates.append(",{\"tx_nocross\":0,\"ty_nocross\":-5}");
        assertFalse(parseCandidates(candidates.toString()).isAvailable());
    }

    @Test
    public void absentMalformedAndOversizedJsonAreUnavailable() {
        for (String input : new String[]{null, "", "{", "[]", "null", "1"}) {
            assertFalse(LimelightColorFrame.parse(input).isAvailable());
        }
        StringBuilder tooLarge = new StringBuilder();
        for (int i = 0; i < 262145; i++) {
            tooLarge.append(' ');
        }
        assertFalse(LimelightColorFrame.parse(tooLarge.toString()).isAvailable());
    }

    @Test
    public void excessiveNestedUnknownMetadataCannotOverflowTheRecursiveParser() {
        StringBuilder nested = new StringBuilder("{\"pTYPE\":\"color\",\"Retro\":[],\"extra\":");
        for (int i = 0; i < 1000; i++) nested.append('[');
        nested.append('0');
        for (int i = 0; i < 1000; i++) nested.append(']');
        nested.append('}');
        assertFalse(LimelightColorFrame.parse(nested.toString()).isAvailable());
        StringBuilder bracketText = new StringBuilder();
        for (int i = 0; i < 100; i++) bracketText.append('[');
        assertTrue(LimelightColorFrame.parse(
                "{\"pTYPE\":\"color\",\"Retro\":[],\"note\":\"" + bracketText + "\"}")
                .isAvailable());
    }

    private static LimelightColorFrame parseCandidates(String candidates) {
        return LimelightColorFrame.parse("{\"pTYPE\":\"color\",\"Retro\":[" + candidates + "]}");
    }
}
