package edu.ftcphoenix.fw.localization.apriltag;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Collections;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.field.SimpleTagLayout;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.spatial.Region2d;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused CONFIG-03 contracts for configured fixed-tag solver ownership and numerics. */
public final class FixedTagFieldPoseSolverConfigTest {

    @Test
    public void configAndSolverExposeOnlyConfiguredOwnerSurface() throws Exception {
        Class<FixedTagFieldPoseSolver.Config> configType = FixedTagFieldPoseSolver.Config.class;
        assertTrue(Modifier.isFinal(configType.getModifiers()));
        Constructor<?>[] configConstructors = configType.getDeclaredConstructors();
        assertEquals(1, configConstructors.length);
        assertTrue(Modifier.isPrivate(configConstructors[0].getModifiers()));
        assertDeclaredMethodAbsent(configType, "validate", String.class);
        assertDeclaredMethodAbsent(configType, "validatedCopy", String.class);
        assertDeclaredMethodAbsent(configType, "copyOf", configType);
        assertDeclaredMethodAbsent(configType, "normalizedValidatedCopyOf", configType, String.class);

        Constructor<?>[] solverConstructors = FixedTagFieldPoseSolver.class.getConstructors();
        assertEquals(1, solverConstructors.length);
        assertEquals(configType, solverConstructors[0].getParameterTypes()[0]);
        Method solve = FixedTagFieldPoseSolver.class.getDeclaredMethod(
                "solve",
                java.util.List.class,
                edu.ftcphoenix.fw.field.TagLayout.class,
                CameraMountConfig.class
        );
        assertTrue(Modifier.isPublic(solve.getModifiers()));
        assertFalse(Modifier.isStatic(solve.getModifiers()));
        assertDeclaredMethodAbsent(
                FixedTagFieldPoseSolver.class,
                "solve",
                java.util.List.class,
                edu.ftcphoenix.fw.field.TagLayout.class,
                CameraMountConfig.class,
                configType
        );

        try {
            new FixedTagFieldPoseSolver(null);
            fail("Expected non-null solver Config");
        } catch (NullPointerException expected) {
            assertEquals("config", expected.getMessage());
        }

        try {
            new FixedTagFieldPoseSolver(FixedTagFieldPoseSolver.Config.defaults()).solve(
                    Collections.<AprilTagObservation>emptyList(),
                    new SimpleTagLayout(),
                    null
            );
            fail("Expected non-null camera mount");
        } catch (NullPointerException expected) {
            assertEquals("cameraMount", expected.getMessage());
        }
    }

    @Test
    public void configCopyIsIndependentAndConfiguredOwnerIgnoresLaterDraftMutation() {
        FixedTagFieldPoseSolver.Config draft = FixedTagFieldPoseSolver.Config.defaults();
        draft.rangeSoftnessInches = 17.0;
        FixedTagFieldPoseSolver.Config copy = draft.copy();
        FixedTagFieldPoseSolver solver = new FixedTagFieldPoseSolver(draft);

        assertNotSame(draft, copy);
        assertEquals(17.0, copy.rangeSoftnessInches, 0.0);
        draft.rangeSoftnessInches = 91.0;

        assertEquals(17.0, copy.rangeSoftnessInches, 0.0);
        assertTrue(solver.toString().contains("rangeSoftnessInches=17.0"));
        assertFalse(solver.toString().contains("rangeSoftnessInches=91.0"));
    }

    @Test
    public void exactClosedAndOpenNumericBoundariesAreAccepted() {
        FixedTagFieldPoseSolver.Config lower = FixedTagFieldPoseSolver.Config.defaults();
        lower.maxAbsBearingRad = 0.0;
        lower.observationFieldPoseMaxDeltaInches = 0.0;
        lower.observationFieldPoseMaxDeltaHeadingRad = 0.0;
        lower.rangeSoftnessInches = Double.MIN_VALUE;
        lower.minObservationWeight = 0.0;
        lower.outlierPositionGateInches = Double.MIN_VALUE;
        lower.outlierHeadingGateRad = Double.MIN_VALUE;
        lower.consistencyPositionScaleInches = Double.MIN_VALUE;
        lower.consistencyHeadingScaleRad = Double.MIN_VALUE;
        lower.maxOutsidePlausibleFieldRegionInches = 0.0;
        new FixedTagFieldPoseSolver(lower);

        FixedTagFieldPoseSolver.Config upper = FixedTagFieldPoseSolver.Config.defaults();
        upper.maxAbsBearingRad = Math.PI;
        upper.observationFieldPoseMaxDeltaHeadingRad = Math.PI;
        upper.minObservationWeight = 1.0;
        upper.outlierHeadingGateRad = Math.PI;
        new FixedTagFieldPoseSolver(upper);
    }

    @Test
    public void everyNumericFieldRejectsItsInvalidSideWithFieldDomainAndValue() {
        assertInvalid("maxAbsBearingRad", -Double.MIN_VALUE,
                cfg -> cfg.maxAbsBearingRad = -Double.MIN_VALUE, "[0.0, " + Math.PI + "]");
        assertInvalid("maxAbsBearingRad", Math.nextUp(Math.PI),
                cfg -> cfg.maxAbsBearingRad = Math.nextUp(Math.PI), "[0.0, " + Math.PI + "]");
        assertInvalid("observationFieldPoseMaxDeltaInches", -Double.MIN_VALUE,
                cfg -> cfg.observationFieldPoseMaxDeltaInches = -Double.MIN_VALUE, ">= 0");
        assertInvalid("observationFieldPoseMaxDeltaHeadingRad", -Double.MIN_VALUE,
                cfg -> cfg.observationFieldPoseMaxDeltaHeadingRad = -Double.MIN_VALUE,
                "[0.0, " + Math.PI + "]");
        assertInvalid("observationFieldPoseMaxDeltaHeadingRad", Math.nextUp(Math.PI),
                cfg -> cfg.observationFieldPoseMaxDeltaHeadingRad = Math.nextUp(Math.PI),
                "[0.0, " + Math.PI + "]");
        assertInvalid("rangeSoftnessInches", 0.0,
                cfg -> cfg.rangeSoftnessInches = 0.0, "> 0");
        assertInvalid("minObservationWeight", -Double.MIN_VALUE,
                cfg -> cfg.minObservationWeight = -Double.MIN_VALUE, "[0.0, 1.0]");
        assertInvalid("minObservationWeight", Math.nextUp(1.0),
                cfg -> cfg.minObservationWeight = Math.nextUp(1.0), "[0.0, 1.0]");
        assertInvalid("outlierPositionGateInches", 0.0,
                cfg -> cfg.outlierPositionGateInches = 0.0, "> 0");
        assertInvalid("outlierHeadingGateRad", 0.0,
                cfg -> cfg.outlierHeadingGateRad = 0.0, "(0, " + Math.PI + "]");
        assertInvalid("outlierHeadingGateRad", Math.nextUp(Math.PI),
                cfg -> cfg.outlierHeadingGateRad = Math.nextUp(Math.PI),
                "(0, " + Math.PI + "]");
        assertInvalid("consistencyPositionScaleInches", 0.0,
                cfg -> cfg.consistencyPositionScaleInches = 0.0, "> 0");
        assertInvalid("consistencyHeadingScaleRad", 0.0,
                cfg -> cfg.consistencyHeadingScaleRad = 0.0, "> 0");
        assertInvalid("maxOutsidePlausibleFieldRegionInches", -Double.MIN_VALUE,
                cfg -> cfg.maxOutsidePlausibleFieldRegionInches = -Double.MIN_VALUE, ">= 0");
        assertInvalid("rangeSoftnessInches", Double.NaN,
                cfg -> cfg.rangeSoftnessInches = Double.NaN, "finite");
        assertInvalid("rangeSoftnessInches", Double.POSITIVE_INFINITY,
                cfg -> cfg.rangeSoftnessInches = Double.POSITIVE_INFINITY, "finite");
    }

    @Test
    public void tinyPositiveSoftnessIsUsedExactlyInsteadOfReplacedByDefault() {
        FixedTagFieldPoseSolver.Config config = FixedTagFieldPoseSolver.Config.defaults();
        config.rangeSoftnessInches = Double.MIN_VALUE;
        config.minObservationWeight = 0.0;
        FixedTagFieldPoseSolver solver = new FixedTagFieldPoseSolver(config);

        FixedTagFieldPoseSolver.Result result = solver.solve(
                Collections.singletonList(AprilTagObservation.target(
                        1,
                        new Pose3d(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                        Pose3d.zero()
                )),
                new SimpleTagLayout().addPose(1, Pose3d.zero()),
                CameraMountConfig.identity()
        );

        assertFalse(result.hasPose);
    }

    @Test
    public void tinyPositiveOutsideToleranceContributesItsAuthoredQualityScale() {
        FixedTagFieldPoseSolver.Config config = FixedTagFieldPoseSolver.Config.defaults();
        config.plausibleFieldRegion = constantRegion(-5e-11);
        config.maxOutsidePlausibleFieldRegionInches = 1e-10;
        FixedTagFieldPoseSolver.Result result = new FixedTagFieldPoseSolver(config).solve(
                Collections.singletonList(AprilTagObservation.target(
                        1,
                        Pose3d.zero(),
                        Pose3d.zero()
                )),
                new SimpleTagLayout().addPose(1, Pose3d.zero()),
                CameraMountConfig.identity()
        );

        assertTrue(result.hasPose);
        assertTrue(result.quality > 0.0);
    }

    @Test
    public void nonFiniteRegionDistanceAndFiniteAggregateOverflowFailClosed() {
        FixedTagFieldPoseSolver.Config badRegion = FixedTagFieldPoseSolver.Config.defaults();
        badRegion.plausibleFieldRegion = constantRegion(Double.NaN);
        FixedTagFieldPoseSolver.Result regionResult = new FixedTagFieldPoseSolver(badRegion).solve(
                Collections.singletonList(AprilTagObservation.target(1, Pose3d.zero(), Pose3d.zero())),
                new SimpleTagLayout().addPose(1, Pose3d.zero()),
                CameraMountConfig.identity()
        );
        assertFalse(regionResult.hasPose);

        Pose3d huge = new Pose3d(Double.MAX_VALUE, 0.0, 0.0, 0.0, 0.0, 0.0);
        FixedTagFieldPoseSolver.Result overflow = new FixedTagFieldPoseSolver(
                FixedTagFieldPoseSolver.Config.defaults()
        ).solve(
                Arrays.asList(
                        AprilTagObservation.target(1, Pose3d.zero(), huge),
                        AprilTagObservation.target(2, Pose3d.zero(), huge)
                ),
                new SimpleTagLayout().addPose(1, huge).addPose(2, huge),
                CameraMountConfig.identity()
        );
        assertFalse(overflow.hasPose);
    }

    @Test
    public void observationPreferenceAndOutlierConsensusBehaviorArePreserved() {
        SimpleTagLayout oneTag = new SimpleTagLayout().addPose(1, Pose3d.zero());
        AprilTagObservation alternatePose = AprilTagObservation.target(
                1,
                Pose3d.zero(),
                new Pose3d(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        );

        FixedTagFieldPoseSolver.Config preferredConfig =
                FixedTagFieldPoseSolver.Config.defaults();
        FixedTagFieldPoseSolver.Result preferred = new FixedTagFieldPoseSolver(
                preferredConfig
        ).solve(Collections.singletonList(alternatePose), oneTag, CameraMountConfig.identity());
        assertTrue(preferred.hasPose);
        assertEquals(1.0, preferred.fieldToRobotPose.xInches, 1e-9);
        assertTrue(preferred.acceptedContributions.get(0).usedObservationFieldPose);

        FixedTagFieldPoseSolver.Config geometryConfig = preferredConfig.copy();
        geometryConfig.preferObservationFieldPose = false;
        FixedTagFieldPoseSolver.Result geometry = new FixedTagFieldPoseSolver(
                geometryConfig
        ).solve(Collections.singletonList(alternatePose), oneTag, CameraMountConfig.identity());
        assertTrue(geometry.hasPose);
        assertEquals(0.0, geometry.fieldToRobotPose.xInches, 1e-9);
        assertFalse(geometry.acceptedContributions.get(0).usedObservationFieldPose);

        Pose3d nearZero = Pose3d.zero();
        Pose3d nearOne = new Pose3d(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        Pose3d far = new Pose3d(100.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        SimpleTagLayout threeTags = new SimpleTagLayout()
                .addPose(1, nearZero)
                .addPose(2, nearOne)
                .addPose(3, far);
        FixedTagFieldPoseSolver.Result consensus = new FixedTagFieldPoseSolver(
                FixedTagFieldPoseSolver.Config.defaults()
        ).solve(
                Arrays.asList(
                        AprilTagObservation.target(1, Pose3d.zero(), nearZero),
                        AprilTagObservation.target(2, Pose3d.zero(), nearOne),
                        AprilTagObservation.target(3, Pose3d.zero(), far)
                ),
                threeTags,
                CameraMountConfig.identity()
        );
        assertTrue(consensus.hasPose);
        assertEquals(3, consensus.candidateCount);
        assertEquals(2, consensus.acceptedCount);
        assertEquals(0.5, consensus.fieldToRobotPose.xInches, 1e-9);
    }

    private static Region2d constantRegion(double distance) {
        return (xInches, yInches) -> distance;
    }

    private static void assertInvalid(String field,
                                      double value,
                                      ConfigMutation mutation,
                                      String domainText) {
        FixedTagFieldPoseSolver.Config config = FixedTagFieldPoseSolver.Config.defaults();
        mutation.apply(config);
        try {
            new FixedTagFieldPoseSolver(config);
            fail("Expected invalid " + field);
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(
                    "FixedTagFieldPoseSolver.Config." + field));
            assertTrue(expected.getMessage(), expected.getMessage().contains(domainText));
            assertTrue(expected.getMessage(), expected.getMessage().contains(String.valueOf(value)));
        }
    }

    private static void assertDeclaredMethodAbsent(Class<?> owner,
                                                   String name,
                                                   Class<?>... parameters) {
        try {
            owner.getDeclaredMethod(name, parameters);
            fail(owner.getSimpleName() + "." + name + " must be absent");
        } catch (NoSuchMethodException expected) {
            // Expected.
        }
    }

    private interface ConfigMutation {
        void apply(FixedTagFieldPoseSolver.Config config);
    }
}
