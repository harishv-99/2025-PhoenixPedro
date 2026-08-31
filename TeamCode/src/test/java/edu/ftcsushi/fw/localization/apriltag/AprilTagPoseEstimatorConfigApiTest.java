package edu.ftcsushi.fw.localization.apriltag;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks down CONFIG-03's composed, raw-copy estimator configuration surface. */
public final class AprilTagPoseEstimatorConfigApiTest {

    @Test
    public void configIsFinalPrivatelyConstructedCompositionWithOnlyRawCopyFactories() {
        Class<AprilTagPoseEstimator.Config> type = AprilTagPoseEstimator.Config.class;

        assertTrue(Modifier.isFinal(type.getModifiers()));
        assertEquals(Object.class, type.getSuperclass());
        Constructor<?>[] constructors = type.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPrivate(constructors[0].getModifiers()));

        assertDeclaredPublicMethod(type, "defaults");
        assertDeclaredPublicMethod(type, "copy");
        assertDeclaredMethodAbsent(type, "toSolverConfig");
        assertDeclaredMethodAbsent(type, "validate", String.class);
        assertDeclaredMethodAbsent(type, "validatedCopy", String.class);
        assertDeclaredMethodAbsent(type, "withCameraMount",
                edu.ftcsushi.fw.sensing.vision.CameraMountConfig.class);
        assertDeclaredMethodAbsent(type, "withMaxDetectionAgeSec", double.class);
        assertDeclaredMethodAbsent(
                type,
                "withPlausibleFieldRegion",
                edu.ftcsushi.fw.spatial.Region2d.class
        );
        assertDeclaredMethodAbsent(
                type,
                "withMaxOutsidePlausibleFieldRegionInches",
                double.class
        );
    }

    @Test
    public void copyDeepCopiesNonNullSolverDraftAndRetainsStableMount() {
        AprilTagPoseEstimator.Config source = AprilTagPoseEstimator.Config.defaults();
        source.fieldPoseSolver.rangeSoftnessInches = 17.0;

        AprilTagPoseEstimator.Config copy = source.copy();

        assertNotSame(source, copy);
        assertNotSame(source.fieldPoseSolver, copy.fieldPoseSolver);
        assertEquals(17.0, copy.fieldPoseSolver.rangeSoftnessInches, 0.0);
        assertSame(source.cameraMount, copy.cameraMount);

        source.fieldPoseSolver.rangeSoftnessInches = 91.0;
        assertEquals(17.0, copy.fieldPoseSolver.rangeSoftnessInches, 0.0);
    }

    @Test
    public void copyPreservesNullNestedSolverDraftWithoutValidationOrDefaults() {
        AprilTagPoseEstimator.Config source = AprilTagPoseEstimator.Config.defaults();
        source.fieldPoseSolver = null;

        AprilTagPoseEstimator.Config copy = source.copy();

        assertNull(copy.fieldPoseSolver);
    }

    private static void assertDeclaredPublicMethod(Class<?> owner,
                                                   String name,
                                                   Class<?>... parameters) {
        try {
            Method method = owner.getDeclaredMethod(name, parameters);
            assertTrue(Modifier.isPublic(method.getModifiers()));
        } catch (NoSuchMethodException e) {
            fail(owner.getSimpleName() + "." + name + " must be declared");
        }
    }

    private static void assertDeclaredMethodAbsent(Class<?> owner,
                                                   String name,
                                                   Class<?>... parameters) {
        try {
            owner.getDeclaredMethod(name, parameters);
            fail(owner.getSimpleName() + "." + name + " must be absent");
        } catch (NoSuchMethodException expected) {
            // Expected: CONFIG-03 keeps one raw-copy configuration surface.
        }
    }
}
