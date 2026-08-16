package edu.ftcphoenix.fw.spatial;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Modifier;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.source.TimeAwareSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionSource;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks finite authored spatial leaves to their semantic construction boundaries. */
public final class SpatialAuthoredGeometryValidationTest {

    private static final double[] NON_FINITE = {
            Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY
    };
    private static final TagSelectionSource SELECTION = new TagSelectionSource() {
        @Override
        public Set<Integer> candidateIds() {
            return Collections.singleton(7);
        }

        @Override
        public TagSelectionResult get(LoopClock clock) {
            return TagSelectionResult.none(Collections.<Integer>emptySet());
        }
    };

    @Test
    public void everyReferencesNumericPathRejectsNonFiniteValues() {
        for (double invalid : NON_FINITE) {
            assertRejected("fieldPoint.xInches", () -> References.fieldPoint(invalid, 2.0));
            assertRejected("fieldPoint.yInches", () -> References.fieldPoint(1.0, invalid));

            assertRejected("fieldFrame.xInches", () -> References.fieldFrame(invalid, 2.0, 3.0));
            assertRejected("fieldFrame.yInches", () -> References.fieldFrame(1.0, invalid, 3.0));
            assertRejected("fieldFrame.headingRad", () -> References.fieldFrame(1.0, 2.0, invalid));

            assertRejected("TagPointOffset.forwardInches", () -> References.pointOffset(invalid, 2.0));
            assertRejected("TagPointOffset.leftInches", () -> References.pointOffset(1.0, invalid));
            assertRejected("TagFrameOffset.forwardInches", () -> References.frameOffset(invalid, 2.0, 3.0));
            assertRejected("TagFrameOffset.leftInches", () -> References.frameOffset(1.0, invalid, 3.0));
            assertRejected("TagFrameOffset.headingRad", () -> References.frameOffset(1.0, 2.0, invalid));

            ReferenceFrame2d base = References.fieldFrame(1.0, 2.0, 3.0);
            assertRejected("framePoint.forwardInches", () -> References.framePoint(base, invalid, 2.0));
            assertRejected("framePoint.leftInches", () -> References.framePoint(base, 1.0, invalid));

            assertRejected("relativeToTagPoint.forwardInches",
                    () -> References.relativeToTagPoint(7, invalid, 2.0));
            assertRejected("relativeToTagPoint.leftInches",
                    () -> References.relativeToTagPoint(7, 1.0, invalid));
            assertRejected("relativeToTagFrame.forwardInches",
                    () -> References.relativeToTagFrame(7, invalid, 2.0, 3.0));
            assertRejected("relativeToTagFrame.leftInches",
                    () -> References.relativeToTagFrame(7, 1.0, invalid, 3.0));
            assertRejected("relativeToTagFrame.headingRad",
                    () -> References.relativeToTagFrame(7, 1.0, 2.0, invalid));

            assertRejected("selectedTagPoint.forwardInches",
                    () -> References.relativeToSelectedTagPoint(SELECTION, invalid, 2.0));
            assertRejected("selectedTagPoint.leftInches",
                    () -> References.relativeToSelectedTagPoint(SELECTION, 1.0, invalid));
            assertRejected("selectedTagFrame.forwardInches",
                    () -> References.relativeToSelectedTagFrame(SELECTION, invalid, 2.0, 3.0));
            assertRejected("selectedTagFrame.leftInches",
                    () -> References.relativeToSelectedTagFrame(SELECTION, 1.0, invalid, 3.0));
            assertRejected("selectedTagFrame.headingRad",
                    () -> References.relativeToSelectedTagFrame(SELECTION, 1.0, 2.0, invalid));
        }
    }

    @Test
    public void selectedTagMapPathsAcceptFiniteFactoryOwnedOffsets() {
        Map<Integer, References.TagPointOffset> points =
                new LinkedHashMap<Integer, References.TagPointOffset>();
        points.put(7, References.pointOffset(-4.0, 2.0));
        Map<Integer, References.TagFrameOffset> frames =
                new LinkedHashMap<Integer, References.TagFrameOffset>();
        frames.put(7, References.frameOffset(-4.0, 2.0, 9.0 * Math.PI));

        assertTrue(References.isSelectedTagPoint(
                References.relativeToSelectedTagPoint(SELECTION, points)));
        assertTrue(References.isSelectedTagFrame(
                References.relativeToSelectedTagFrame(SELECTION, frames)));
    }

    @Test
    public void spatialTargetsRejectEveryNonFiniteNumericSlot() {
        ReferenceFrame2d frame = References.fieldFrame(1.0, 2.0, 3.0);
        for (double invalid : NON_FINITE) {
            assertRejected("FieldPoint.xInches", () -> SpatialTargets.fieldPoint(invalid, 2.0));
            assertRejected("FieldPoint.yInches", () -> SpatialTargets.fieldPoint(1.0, invalid));
            assertRejected("FieldHeading.fieldHeadingRad",
                    () -> SpatialTargets.fieldHeading(invalid));
            assertRejected("ReferenceFrameHeadingTarget.headingOffsetRad",
                    () -> SpatialTargets.frameHeading(frame, invalid));
        }
    }

    @Test
    public void everyFixedControlFramePathRejectsEveryNonFinitePoseSlot() {
        for (int slot = 0; slot < 3; slot++) {
            for (double invalid : NON_FINITE) {
                final Pose2d bad = poseWith(slot, invalid);
                assertRejected(poseField(slot), () -> RobotFrames.rigid(bad));
                assertRejected(poseField(slot),
                        () -> SpatialControlFrames.of(bad, Pose2d.zero()));
                assertRejected(poseField(slot),
                        () -> SpatialControlFrames.of(Pose2d.zero(), bad));
                assertRejected(poseField(slot),
                        () -> SpatialControlFrames.robotCenter().withTranslationFrame(bad));
                assertRejected(poseField(slot),
                        () -> SpatialControlFrames.robotCenter().withFacingFrame(bad));
            }
        }
    }

    @Test
    public void dynamicControlFrameProvidersAreNeverReadAtConstruction() {
        CountingThrowingSource currentTranslation = new CountingThrowingSource();
        CountingThrowingSource currentFacing = new CountingThrowingSource();
        CountingThrowingTimeAwareSource historicTranslation =
                new CountingThrowingTimeAwareSource();
        CountingThrowingTimeAwareSource historicFacing = new CountingThrowingTimeAwareSource();

        SpatialControlFrames currentOnly =
                SpatialControlFrames.of(currentTranslation, currentFacing);
        SpatialControlFrames replacedCurrentOnly = SpatialControlFrames.robotCenter()
                .withTranslationFrame(currentTranslation)
                .withFacingFrame(currentFacing);
        SpatialControlFrames timeAware =
                SpatialControlFrames.of(historicTranslation, historicFacing);
        SpatialControlFrames replacedTimeAware = SpatialControlFrames.robotCenter()
                .withTranslationFrame(historicTranslation)
                .withFacingFrame(historicFacing);

        assertTrue(currentOnly.toString().contains("SpatialControlFrames"));
        assertTrue(replacedCurrentOnly.toString().contains("SpatialControlFrames"));
        assertTrue(timeAware.toString().contains("SpatialControlFrames"));
        assertTrue(replacedTimeAware.toString().contains("SpatialControlFrames"));
        assertEquals(0, currentTranslation.reads);
        assertEquals(0, currentFacing.reads);
        assertEquals(0, historicTranslation.reads);
        assertEquals(0, historicFacing.reads);
    }

    @Test
    public void axisAlignedBoxRejectsEveryNonFiniteBound() {
        for (int slot = 0; slot < 4; slot++) {
            for (double invalid : NON_FINITE) {
                final double[] bounds = {-1.0, 2.0, -3.0, 4.0};
                bounds[slot] = invalid;
                assertRejected(boxField(slot), () -> new AxisAlignedBoxRegion2d(
                        bounds[0], bounds[1], bounds[2], bounds[3]));
            }
        }
    }

    @Test
    public void finiteSignsAnglesAndAabbNormalizationRemainSupported() {
        References.FieldFrameRef frame = (References.FieldFrameRef) References.fieldFrame(
                -0.0, -12.0, 9.0 * Math.PI);
        assertEquals(Double.doubleToRawLongBits(-0.0),
                Double.doubleToRawLongBits(frame.xInches));
        assertEquals(-12.0, frame.yInches, 0.0);
        assertEquals(9.0 * Math.PI, frame.headingRad, 0.0);

        SpatialTargets.FieldPoint point = SpatialTargets.fieldPoint(-0.0, -4.0);
        assertEquals(Double.doubleToRawLongBits(-0.0),
                Double.doubleToRawLongBits(point.xInches));
        assertEquals(-4.0, point.yInches, 0.0);
        assertEquals(-7.0 * Math.PI,
                SpatialTargets.fieldHeading(-7.0 * Math.PI).fieldHeadingRad, 0.0);

        AxisAlignedBoxRegion2d reversed = new AxisAlignedBoxRegion2d(5.0, -5.0, 8.0, -8.0);
        assertEquals(-5.0, reversed.minXInches, 0.0);
        assertEquals(5.0, reversed.maxXInches, 0.0);
        assertEquals(-8.0, reversed.minYInches, 0.0);
        assertEquals(8.0, reversed.maxYInches, 0.0);
        assertTrue(reversed instanceof Region2d);

        AxisAlignedBoxRegion2d zeroArea = new AxisAlignedBoxRegion2d(2.0, 2.0, -3.0, -3.0);
        assertTrue(zeroArea.contains(2.0, -3.0));
        assertFalse(zeroArea.contains(2.01, -3.0));
    }

    @Test
    public void genericPose2dRemainsPermissive() {
        Pose2d runtimeEvidence = new Pose2d(
                Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
        assertTrue(Double.isNaN(runtimeEvidence.xInches));
        assertFalse(SpatialValidation.isFinite(runtimeEvidence));
    }

    @Test
    public void namedFactoriesAreTheOnlyPublicConstructionPaths() {
        assertNoPublicConstructor(References.TagPointOffset.class);
        assertNoPublicConstructor(References.TagFrameOffset.class);
        assertNoPublicConstructor(SpatialTargets.FieldPoint.class);
        assertNoPublicConstructor(SpatialTargets.FieldHeading.class);
        assertNoPublicConstructor(SpatialTargets.ReferencePointTarget.class);
        assertNoPublicConstructor(SpatialTargets.ReferenceFrameHeadingTarget.class);
    }

    private static Pose2d poseWith(int slot, double value) {
        double[] fields = {1.0, -2.0, 3.0};
        fields[slot] = value;
        return new Pose2d(fields[0], fields[1], fields[2]);
    }

    private static String poseField(int slot) {
        return new String[]{"xInches", "yInches", "headingRad"}[slot];
    }

    private static String boxField(int slot) {
        return new String[]{"minXInches", "maxXInches", "minYInches", "maxYInches"}[slot];
    }

    private static void assertNoPublicConstructor(Class<?> type) {
        assertEquals(type.getName(), 0, type.getConstructors().length);
        for (Constructor<?> constructor : type.getDeclaredConstructors()) {
            assertFalse(type.getName(), Modifier.isPublic(constructor.getModifiers()));
        }
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

    private static final class CountingThrowingSource implements Source<Pose2d> {
        int reads;

        @Override
        public Pose2d get(LoopClock clock) {
            reads++;
            throw new AssertionError("dynamic current frame was sampled during construction");
        }
    }

    private static final class CountingThrowingTimeAwareSource
            implements TimeAwareSource<Pose2d> {
        int reads;

        @Override
        public Pose2d getAt(LoopClock clock, LoopTimestamp timestamp) {
            reads++;
            throw new AssertionError("dynamic historic frame was sampled during construction");
        }
    }
}
