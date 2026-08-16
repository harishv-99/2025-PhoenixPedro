package edu.ftcphoenix.fw.ftc;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies that malformed FTC metadata cannot become fixed field geometry. */
public final class FtcGameTagLayoutFiniteMetadataTest {

    private static final float[] NON_FINITE = {
            Float.NaN, Float.POSITIVE_INFINITY, Float.NEGATIVE_INFINITY
    };

    @Test
    public void everyConvertedPositionFieldRejectsEveryNonFiniteValue() {
        String[] fields = {
                "fieldToTagPose.xInches",
                "fieldToTagPose.yInches",
                "fieldToTagPose.zInches"
        };
        for (int slot = 0; slot < fields.length; slot++) {
            for (float invalid : NON_FINITE) {
                float[] position = {1.0f, -2.0f, 3.0f};
                position[slot] = invalid;
                AprilTagMetadata metadata = metadata(
                        new VectorF(position), Quaternion.identityQuaternion(), DistanceUnit.INCH);
                assertRejected(fields[slot], metadata);
            }
        }
    }

    @Test
    public void everyQuaternionComponentRejectsEveryNonFiniteValue() {
        for (int slot = 0; slot < 4; slot++) {
            for (float invalid : NON_FINITE) {
                float[] quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
                quaternion[slot] = invalid;
                AprilTagMetadata metadata = metadata(
                        new VectorF(1.0f, -2.0f, 3.0f),
                        new Quaternion(
                                quaternion[0], quaternion[1], quaternion[2], quaternion[3], 0L),
                        DistanceUnit.INCH);
                assertRejected("fieldToTagPose.", metadata);
            }
        }
    }

    @Test
    public void finiteSdkMetadataIsConvertedInItsDeclaredUnit() {
        AprilTagMetadata metadata = metadata(
                new VectorF(25.4f, -50.8f, 76.2f),
                Quaternion.identityQuaternion(),
                DistanceUnit.MM);

        Pose3d pose = layoutFor(metadata).getFieldToTagPose(7);

        assertEquals(1.0, pose.xInches, 1e-5);
        assertEquals(-2.0, pose.yInches, 1e-5);
        assertEquals(3.0, pose.zInches, 1e-5);
        assertEquals(0.0, pose.yawRad, 0.0);
        assertEquals(0.0, pose.pitchRad, 0.0);
        assertEquals(0.0, pose.rollRad, 0.0);
    }

    @Test
    public void negativeSdkMetadataIdIsRejectedAtTheLayoutBoundary() {
        AprilTagMetadata metadata = new AprilTagMetadata(
                -7,
                "invalid tag",
                2.0,
                new VectorF(1.0f, 2.0f, 3.0f),
                DistanceUnit.INCH,
                Quaternion.identityQuaternion()
        );

        try {
            layoutFor(metadata);
            fail("Expected a negative AprilTag metadata ID to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("id"));
            assertTrue(expected.getMessage(), expected.getMessage().contains("non-negative"));
            assertTrue(expected.getMessage(), expected.getMessage().contains("-7"));
        }
    }

    private static AprilTagMetadata metadata(VectorF position,
                                              Quaternion orientation,
                                              DistanceUnit unit) {
        return new AprilTagMetadata(7, "test tag", 2.0, position, unit, orientation);
    }

    private static FtcGameTagLayout layoutFor(AprilTagMetadata metadata) {
        AprilTagLibrary library = new AprilTagLibrary.Builder().addTag(metadata).build();
        return FtcGameTagLayout.fromLibraryAllTags(library);
    }

    private static void assertRejected(String expectedField, AprilTagMetadata metadata) {
        try {
            layoutFor(metadata);
            fail("Expected malformed AprilTag metadata to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("id=7"));
            assertTrue(expected.getMessage(), expected.getMessage().contains(expectedField));
            assertTrue(expected.getMessage(), expected.getMessage().contains("finite"));
        }
    }
}
