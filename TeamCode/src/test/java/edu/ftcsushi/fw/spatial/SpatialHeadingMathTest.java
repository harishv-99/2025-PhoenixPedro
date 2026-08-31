package edu.ftcsushi.fw.spatial;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Collections;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies frozen heading selection and overflow-safe retained heading solves. */
public final class SpatialHeadingMathTest {

    private static final double EPS = 1e-12;

    @Test
    public void publicSurfaceAddsOnePureSelectorWithoutAnOptionsType() throws Exception {
        Method selector = SpatialMath2d.class.getDeclaredMethod(
                "nearestHeadingRad",
                double.class,
                double[].class
        );
        assertTrue(Modifier.isPublic(selector.getModifiers()));
        assertTrue(Modifier.isStatic(selector.getModifiers()));
        assertEquals(double.class, selector.getReturnType());
        for (Class<?> nestedType : SpatialMath2d.class.getDeclaredClasses()) {
            assertFalse(nestedType.getName(), Modifier.isPublic(nestedType.getModifiers()));
        }

        int selectorOverloads = 0;
        for (Method method : SpatialMath2d.class.getDeclaredMethods()) {
            if ("nearestHeadingRad".equals(method.getName())
                    && Modifier.isPublic(method.getModifiers())) {
                selectorOverloads++;
            }
        }
        assertEquals(1, selectorOverloads);
    }

    @Test
    public void nearestHeadingReturnsExactAuthoredCandidateAndKeepsFirstExactTie() {
        double unnormalized = 9.0 * Math.PI;
        assertEquals(
                unnormalized,
                SpatialMath2d.nearestHeadingRad(0.1, unnormalized),
                0.0
        );

        assertEquals(
                Math.PI,
                SpatialMath2d.nearestHeadingRad(Math.toRadians(170.0), 0.0, Math.PI),
                0.0
        );
        assertEquals(
                Math.PI / 2.0,
                SpatialMath2d.nearestHeadingRad(0.0, Math.PI / 2.0, -Math.PI / 2.0),
                0.0
        );
        assertEquals(
                Math.PI,
                SpatialMath2d.nearestHeadingRad(0.0, Math.PI, -Math.PI),
                0.0
        );
        assertEquals(
                -Math.PI,
                SpatialMath2d.nearestHeadingRad(0.0, -Math.PI, Math.PI),
                0.0
        );

        double selectedSignedZero = SpatialMath2d.nearestHeadingRad(0.0, -0.0, +0.0);
        assertEquals(
                Double.doubleToRawLongBits(-0.0),
                Double.doubleToRawLongBits(selectedSignedZero)
        );

        double equivalentFirst = 2.0 * Math.PI;
        assertEquals(
                equivalentFirst,
                SpatialMath2d.nearestHeadingRad(0.0, equivalentFirst, 0.0),
                0.0
        );
    }

    @Test
    public void nearestHeadingLeavesEveryCandidateRawBitPatternUnchanged() {
        double[] candidates = {
                -0.0,
                +0.0,
                Double.MIN_VALUE,
                -Double.MIN_VALUE,
                9.0 * Math.PI,
                Double.MAX_VALUE,
                -Double.MAX_VALUE
        };
        long[] authoredBits = new long[candidates.length];
        for (int index = 0; index < candidates.length; index++) {
            authoredBits[index] = Double.doubleToRawLongBits(candidates[index]);
        }

        SpatialMath2d.nearestHeadingRad(0.25, candidates);

        for (int index = 0; index < candidates.length; index++) {
            assertEquals(
                    "candidateHeadingRads[" + index + "]",
                    authoredBits[index],
                    Double.doubleToRawLongBits(candidates[index])
            );
        }
    }

    @Test
    public void nearestHeadingHandlesWrapSeamAndOppositeHugeFiniteOperands() {
        double nearNegativePi = -Math.PI + 0.01;
        double nearPositivePi = Math.PI - 0.01;
        assertEquals(
                nearPositivePi,
                SpatialMath2d.nearestHeadingRad(nearNegativePi, 0.0, nearPositivePi),
                0.0
        );

        double referenceHeadingRad = -Double.MAX_VALUE;
        double wrappedReferenceHeadingRad = Pose2d.wrapToPi(referenceHeadingRad);
        double antipode = Pose2d.wrapToPi(wrappedReferenceHeadingRad + Math.PI);
        double hugeCandidateDistance = wrappedDistanceRad(
                referenceHeadingRad,
                Double.MAX_VALUE
        );
        double antipodeDistance = wrappedDistanceRad(referenceHeadingRad, antipode);
        assertTrue(hugeCandidateDistance < antipodeDistance);
        assertEquals(
                Double.MAX_VALUE,
                SpatialMath2d.nearestHeadingRad(
                        referenceHeadingRad,
                        antipode,
                        Double.MAX_VALUE
                ),
                0.0
        );
    }

    @Test
    public void nearestHeadingRejectsNullEmptyAndEveryNamedNonFiniteInput() {
        expectNullPointer(
                () -> SpatialMath2d.nearestHeadingRad(0.0, (double[]) null),
                "candidateHeadingRads"
        );
        expectIllegalArgument(
                () -> SpatialMath2d.nearestHeadingRad(0.0),
                "at least one"
        );

        for (double invalid : new double[]{
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        }) {
            expectIllegalArgument(
                    () -> SpatialMath2d.nearestHeadingRad(invalid, 0.0),
                    "referenceHeadingRad"
            );
            for (int invalidIndex = 0; invalidIndex < 3; invalidIndex++) {
                final int index = invalidIndex;
                double[] candidates = {0.0, 1.0, 2.0};
                candidates[index] = invalid;
                expectIllegalArgument(
                        () -> SpatialMath2d.nearestHeadingRad(0.0, candidates),
                        "candidateHeadingRads[" + index + "]"
                );
            }
        }
    }

    @Test
    public void headingHelpersWrapOperandsBeforeEverySumAndDifference() {
        double sum = SpatialSolveMath.wrappedHeadingSumRad(
                Double.MAX_VALUE,
                Double.MAX_VALUE
        );
        assertTrue(Double.isFinite(sum));
        assertEquals(
                Pose2d.wrapToPi(
                        Pose2d.wrapToPi(Double.MAX_VALUE)
                                + Pose2d.wrapToPi(Double.MAX_VALUE)
                ),
                sum,
                0.0
        );

        double error = SpatialSolveMath.wrappedHeadingErrorRad(
                Double.MAX_VALUE,
                -Double.MAX_VALUE
        );
        assertTrue(Double.isFinite(error));
        assertEquals(
                Pose2d.wrapToPi(
                        Pose2d.wrapToPi(Double.MAX_VALUE)
                                - Pose2d.wrapToPi(-Double.MAX_VALUE)
                ),
                error,
                0.0
        );

        LoopTimestamp timestamp = LoopTimestamp.unavailable();
        FacingSolution fieldFacing = SpatialSolveMath.facingFromFieldHeading(
                new Pose2d(0.0, 0.0, Double.MAX_VALUE),
                new Pose2d(0.0, 0.0, Double.MAX_VALUE),
                -Double.MAX_VALUE,
                1.0,
                timestamp
        );
        assertTrue(Double.isFinite(fieldFacing.facingErrorRad));
        assertEquals(
                SpatialSolveMath.wrappedHeadingErrorRad(
                        -Double.MAX_VALUE,
                        SpatialSolveMath.wrappedHeadingSumRad(
                                Double.MAX_VALUE,
                                Double.MAX_VALUE
                        )
                ),
                fieldFacing.facingErrorRad,
                0.0
        );

        FacingSolution robotFacing = SpatialSolveMath.facingFromRobotHeading(
                new Pose2d(0.0, 0.0, -Double.MAX_VALUE),
                Double.MAX_VALUE,
                1.0,
                timestamp
        );
        assertTrue(Double.isFinite(robotFacing.facingErrorRad));
        assertEquals(error, robotFacing.facingErrorRad, 0.0);
    }

    @Test
    public void headingHelpersPreserveOrdinarySignsWrapSeamAndPositivePiAntipode() {
        LoopTimestamp timestamp = LoopTimestamp.unavailable();

        assertEquals(
                Math.PI,
                SpatialSolveMath.wrappedHeadingSumRad(Math.PI / 2.0, Math.PI / 2.0),
                0.0
        );

        assertEquals(
                0.25,
                SpatialSolveMath.facingFromFieldHeading(
                        new Pose2d(0.0, 0.0, 0.25),
                        new Pose2d(0.0, 0.0, 0.50),
                        1.0,
                        1.0,
                        timestamp
                ).facingErrorRad,
                EPS
        );
        assertEquals(
                -0.25,
                SpatialSolveMath.facingFromRobotHeading(
                        new Pose2d(0.0, 0.0, 0.50),
                        0.25,
                        1.0,
                        timestamp
                ).facingErrorRad,
                EPS
        );
        assertEquals(
                Math.PI,
                SpatialSolveMath.facingFromRobotHeading(
                        new Pose2d(0.0, 0.0, Math.PI),
                        0.0,
                        1.0,
                        timestamp
                ).facingErrorRad,
                0.0
        );
        assertEquals(
                0.02,
                SpatialSolveMath.wrappedHeadingErrorRad(
                        -Math.PI + 0.01,
                        Math.PI - 0.01
                ),
                EPS
        );
    }

    @Test
    public void absolutePoseFrameTargetAdditionCannotOverflow() {
        ManualLoopClock time = new ManualLoopClock(2.0);
        PoseEstimate estimate = new PoseEstimate(
                Pose3d.zero(),
                true,
                1.0,
                time.clock().nowTimestamp()
        );
        AbsolutePoseSpatialSolveLane lane = new AbsolutePoseSpatialSolveLane(
                new FixedEstimator(estimate)
        );

        FacingTarget2d target = SpatialTargets.frameHeading(
                References.fieldFrame(0.0, 0.0, Double.MAX_VALUE),
                Double.MAX_VALUE
        );
        SpatialLaneResult result = lane.solve(request(time.clock(), target, null));

        assertNotNull(result.facing);
        assertTrue(Double.isFinite(result.facing.facingErrorRad));
        assertEquals(
                SpatialSolveMath.wrappedHeadingSumRad(
                        Double.MAX_VALUE,
                        Double.MAX_VALUE
                ),
                result.facing.facingErrorRad,
                0.0
        );

        FacingTarget2d wrapTarget = SpatialTargets.frameHeading(
                References.fieldFrame(0.0, 0.0, Math.PI - 0.05),
                0.10
        );
        SpatialLaneResult wrapResult = lane.solve(request(time.clock(), wrapTarget, null));
        assertNotNull(wrapResult.facing);
        assertEquals(-Math.PI + 0.05, wrapResult.facing.facingErrorRad, EPS);

        SimpleTagLayout layout = new SimpleTagLayout().addPose(
                11,
                new Pose3d(0.0, 0.0, 0.0, Double.MAX_VALUE, 0.0, 0.0)
        );
        FacingTarget2d tagRelativeTarget = SpatialTargets.frameHeading(
                References.relativeToTagFrame(11, 0.0, 0.0, Double.MAX_VALUE),
                Double.MAX_VALUE
        );
        SpatialLaneResult tagRelativeResult = lane.solve(
                request(time.clock(), tagRelativeTarget, layout)
        );
        double expectedTagRelativeHeading = SpatialSolveMath.wrappedHeadingSumRad(
                SpatialSolveMath.wrappedHeadingSumRad(
                        Double.MAX_VALUE,
                        Double.MAX_VALUE
                ),
                Double.MAX_VALUE
        );
        assertNotNull(tagRelativeResult.facing);
        assertTrue(Double.isFinite(tagRelativeResult.facing.facingErrorRad));
        assertEquals(
                expectedTagRelativeHeading,
                tagRelativeResult.facing.facingErrorRad,
                0.0
        );
    }

    @Test
    public void directAprilTagFrameTargetAdditionCannotOverflow() {
        ManualLoopClock time = new ManualLoopClock(3.0);
        AprilTagDetections detections = detections(
                time.clock().nowTimestamp(),
                AprilTagObservation.target(7, Pose3d.zero())
        );
        AprilTagSpatialSolveLane lane = new AprilTagSpatialSolveLane(
                clock -> detections,
                CameraMountConfig.identity(),
                0.5
        );

        FacingTarget2d target = SpatialTargets.frameHeading(
                References.relativeToTagFrame(7, 0.0, 0.0, Double.MAX_VALUE),
                Double.MAX_VALUE
        );
        SpatialLaneResult result = lane.solve(request(time.clock(), target, null));

        assertNotNull(result.facing);
        assertTrue(Double.isFinite(result.facing.facingErrorRad));
        assertEquals(
                SpatialSolveMath.wrappedHeadingSumRad(
                        Double.MAX_VALUE,
                        Double.MAX_VALUE
                ),
                result.facing.facingErrorRad,
                0.0
        );

        FacingTarget2d wrapTarget = SpatialTargets.frameHeading(
                References.relativeToTagFrame(7, 0.0, 0.0, Math.PI - 0.05),
                0.10
        );
        SpatialLaneResult wrapResult = lane.solve(request(time.clock(), wrapTarget, null));
        assertNotNull(wrapResult.facing);
        assertEquals(-Math.PI + 0.05, wrapResult.facing.facingErrorRad, EPS);
    }

    @Test
    public void aprilTagFieldFrameFallbackAdditionCannotOverflow() {
        ManualLoopClock time = new ManualLoopClock(4.0);
        AprilTagDetections detections = detections(
                time.clock().nowTimestamp(),
                AprilTagObservation.target(8, Pose3d.zero(), Pose3d.zero())
        );
        AprilTagSpatialSolveLane lane = new AprilTagSpatialSolveLane(
                clock -> detections,
                CameraMountConfig.identity(),
                0.5
        );
        SimpleTagLayout layout = new SimpleTagLayout()
                .addPose(8, Pose3d.zero())
                .addPose(
                        9,
                        new Pose3d(0.0, 0.0, 0.0, Double.MAX_VALUE, 0.0, 0.0)
                );

        FacingTarget2d target = SpatialTargets.frameHeading(
                References.fieldFrame(0.0, 0.0, Double.MAX_VALUE),
                Double.MAX_VALUE
        );
        SpatialLaneResult result = lane.solve(request(time.clock(), target, layout));

        assertNotNull(result.facing);
        assertTrue(Double.isFinite(result.facing.facingErrorRad));
        assertEquals(
                SpatialSolveMath.wrappedHeadingSumRad(
                        Double.MAX_VALUE,
                        Double.MAX_VALUE
                ),
                result.facing.facingErrorRad,
                0.0
        );

        FacingTarget2d wrapTarget = SpatialTargets.frameHeading(
                References.fieldFrame(0.0, 0.0, Math.PI - 0.05),
                0.10
        );
        SpatialLaneResult wrapResult = lane.solve(request(time.clock(), wrapTarget, layout));
        assertNotNull(wrapResult.facing);
        assertEquals(-Math.PI + 0.05, wrapResult.facing.facingErrorRad, EPS);

        FacingTarget2d tagRelativeTarget = SpatialTargets.frameHeading(
                References.relativeToTagFrame(9, 0.0, 0.0, Double.MAX_VALUE),
                Double.MAX_VALUE
        );
        SpatialLaneResult tagRelativeResult = lane.solve(
                request(time.clock(), tagRelativeTarget, layout)
        );
        double expectedTagRelativeHeading = SpatialSolveMath.wrappedHeadingSumRad(
                SpatialSolveMath.wrappedHeadingSumRad(
                        Double.MAX_VALUE,
                        Double.MAX_VALUE
                ),
                Double.MAX_VALUE
        );
        assertNotNull(tagRelativeResult.facing);
        assertTrue(Double.isFinite(tagRelativeResult.facing.facingErrorRad));
        assertEquals(
                expectedTagRelativeHeading,
                tagRelativeResult.facing.facingErrorRad,
                0.0
        );
    }

    @Test
    public void selectedTagFrameCompositionCannotOverflowBeforeHeadingSolve() {
        ManualLoopClock time = new ManualLoopClock(4.5);
        SimpleTagLayout layout = new SimpleTagLayout().addPose(
                9,
                new Pose3d(0.0, 0.0, 0.0, Double.MAX_VALUE, 0.0, 0.0)
        );
        TagSelectionSource selection = new TagSelectionSource() {
            @Override
            public java.util.Set<Integer> candidateIds() {
                return Collections.singleton(9);
            }

            @Override
            public TagSelectionResult get(LoopClock clock) {
                return new TagSelectionResult(
                        false,
                        -1,
                        AprilTagObservation.noTarget(),
                        true,
                        9,
                        true,
                        false,
                        AprilTagObservation.noTarget(),
                        Collections.singleton(9),
                        "test",
                        "selected",
                        0.0
                );
            }
        };
        ReferenceFrame2d selectedFrame = References.relativeToSelectedTagFrame(
                selection,
                0.0,
                0.0,
                Double.MAX_VALUE
        );

        Pose2d resolved = References.tryResolveFieldFrame(
                selectedFrame,
                layout,
                time.clock()
        );

        assertNotNull(resolved);
        assertTrue(Double.isFinite(resolved.headingRad));
        assertEquals(
                SpatialSolveMath.wrappedHeadingSumRad(
                        Double.MAX_VALUE,
                        Double.MAX_VALUE
                ),
                resolved.headingRad,
                0.0
        );
    }

    @Test
    public void nonzeroDirectTagFieldFallbackAndAbsoluteLanesAgree() {
        ManualLoopClock time = new ManualLoopClock(5.0);
        LoopTimestamp frameTimestamp = time.clock().nowTimestamp();
        Pose3d fieldToRobot = new Pose3d(10.0, -8.0, 0.0, 0.30, 0.0, 0.0);
        Pose3d robotToCamera = new Pose3d(4.0, 1.5, 7.0, 0.20, 0.0, 0.0);
        Pose3d fieldToLocatorTag = new Pose3d(42.0, -2.0, 12.0, -0.50, 0.0, 0.0);
        Pose3d fieldToTargetTag = new Pose3d(48.0, 18.0, 14.0, 1.10, 0.0, 0.0);
        CameraMountConfig mount = CameraMountConfig.ofPose(robotToCamera);
        SimpleTagLayout layout = new SimpleTagLayout()
                .addPose(8, fieldToLocatorTag)
                .addPose(9, fieldToTargetTag);

        AprilTagDetections directFrame = detections(
                frameTimestamp,
                observationForFieldTag(9, fieldToRobot, robotToCamera, fieldToTargetTag)
        );
        AprilTagDetections locatorFrame = detections(
                frameTimestamp,
                observationForFieldTag(8, fieldToRobot, robotToCamera, fieldToLocatorTag)
        );
        AprilTagSpatialSolveLane directLane = new AprilTagSpatialSolveLane(
                clock -> directFrame,
                mount,
                0.5
        );
        AprilTagSpatialSolveLane fallbackLane = new AprilTagSpatialSolveLane(
                clock -> locatorFrame,
                mount,
                0.5
        );
        AbsolutePoseSpatialSolveLane absoluteLane = new AbsolutePoseSpatialSolveLane(
                new FixedEstimator(new PoseEstimate(
                        fieldToRobot,
                        true,
                        1.0,
                        frameTimestamp
                ))
        );

        double referenceHeadingRad = 0.45;
        double targetHeadingOffsetRad = -0.20;
        Pose2d robotToFacingFrame = new Pose2d(1.5, -0.75, 0.18);
        FacingTarget2d target = SpatialTargets.frameHeading(
                References.relativeToTagFrame(
                        9,
                        6.0,
                        -2.5,
                        referenceHeadingRad
                ),
                targetHeadingOffsetRad
        );

        SpatialLaneResult direct = directLane.solve(request(
                time.clock(), target, null, robotToFacingFrame));
        SpatialLaneResult fallback = fallbackLane.solve(request(
                time.clock(), target, layout, robotToFacingFrame));
        SpatialLaneResult absolute = absoluteLane.solve(request(
                time.clock(), target, layout, robotToFacingFrame));

        assertNotNull(direct.facing);
        assertNotNull(fallback.facing);
        assertNotNull(absolute.facing);
        double expectedErrorRad = Pose2d.wrapToPi(
                fieldToTargetTag.yawRad
                        + referenceHeadingRad
                        + targetHeadingOffsetRad
                        - fieldToRobot.yawRad
                        - robotToFacingFrame.headingRad
        );
        assertEquals(expectedErrorRad, direct.facing.facingErrorRad, EPS);
        assertEquals(expectedErrorRad, fallback.facing.facingErrorRad, EPS);
        assertEquals(expectedErrorRad, absolute.facing.facingErrorRad, EPS);
    }

    private static SpatialSolveRequest request(LoopClock clock,
                                               FacingTarget2d target,
                                               SimpleTagLayout layout) {
        return request(clock, target, layout, Pose2d.zero());
    }

    private static SpatialSolveRequest request(LoopClock clock,
                                               FacingTarget2d target,
                                               SimpleTagLayout layout,
                                               Pose2d robotToFacingFrame) {
        return new SpatialSolveRequest(
                clock,
                null,
                target,
                null,
                null,
                Pose2d.zero(),
                robotToFacingFrame,
                layout
        );
    }

    private static AprilTagDetections detections(LoopTimestamp timestamp,
                                                 AprilTagObservation observation) {
        return AprilTagDetections.fromFrame(
                timestamp,
                Collections.singletonList(observation)
        );
    }

    private static AprilTagObservation observationForFieldTag(int id,
                                                              Pose3d fieldToRobot,
                                                              Pose3d robotToCamera,
                                                              Pose3d fieldToTag) {
        Pose3d cameraToTag = fieldToRobot
                .then(robotToCamera)
                .inverse()
                .then(fieldToTag);
        return AprilTagObservation.target(id, cameraToTag, fieldToRobot);
    }

    private static double wrappedDistanceRad(double referenceHeadingRad,
                                             double candidateHeadingRad) {
        return Math.abs(Pose2d.wrapToPi(
                Pose2d.wrapToPi(candidateHeadingRad)
                        - Pose2d.wrapToPi(referenceHeadingRad)
        ));
    }

    private static void expectIllegalArgument(Runnable action, String messagePart) {
        try {
            action.run();
            fail("Expected IllegalArgumentException containing " + messagePart);
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(messagePart));
        }
    }

    private static void expectNullPointer(Runnable action, String messagePart) {
        try {
            action.run();
            fail("Expected NullPointerException containing " + messagePart);
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(messagePart));
        }
    }

    private static final class FixedEstimator implements AbsolutePoseEstimator {
        private final PoseEstimate estimate;

        FixedEstimator(PoseEstimate estimate) {
            this.estimate = estimate;
        }

        @Override
        public void update(LoopClock clock) {
            // No-op test estimator.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }
}
