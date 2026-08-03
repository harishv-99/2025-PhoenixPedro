package edu.ftcphoenix.fw.localization.apriltag;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks down CLEAN-01's single shared-solver config conversion name. */
public final class AprilTagPoseEstimatorConfigApiTest {

    @Test
    public void sharedSolverConfigUsesOnlyToSolverConfig() throws Exception {
        Method conversion = AprilTagPoseEstimator.Config.class.getDeclaredMethod(
                "toSolverConfig");
        assertTrue(Modifier.isPublic(conversion.getModifiers()));
        assertFalse(Modifier.isStatic(conversion.getModifiers()));
        assertEquals(FixedTagFieldPoseSolver.Config.class, conversion.getReturnType());

        AprilTagPoseEstimator.Config estimatorConfig =
                AprilTagPoseEstimator.Config.defaults();
        FixedTagFieldPoseSolver.Config solverConfig = estimatorConfig.toSolverConfig();
        assertNotNull(solverConfig);
        assertNotSame(estimatorConfig, solverConfig);

        assertDeclaredMethodAbsent(AprilTagPoseEstimator.Config.class, "solverConfig");
    }

    private static void assertDeclaredMethodAbsent(Class<?> owner, String name) {
        try {
            owner.getDeclaredMethod(name);
            fail(owner.getSimpleName() + "." + name + " must be absent");
        } catch (NoSuchMethodException expected) {
            // Expected: CLEAN-01 removes the parallel alias.
        }
    }
}
