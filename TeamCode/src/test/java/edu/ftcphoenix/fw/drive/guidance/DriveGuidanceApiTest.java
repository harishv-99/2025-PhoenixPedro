package edu.ftcphoenix.fw.drive.guidance;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveOverlayStack;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.field.TagLayout;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks down the plan-owned public construction surface for Drive Guidance. */
public final class DriveGuidanceApiTest {

    @Test
    public void canonicalPlanOwnedRuntimeFactoriesRemainPublic() throws Exception {
        assertPublicStaticMethod(
                DriveGuidance.class,
                "plan",
                DriveGuidance.PlanBuilder0.class
        );
        assertPublicStaticMethod(
                DriveGuidance.class,
                "plan",
                DriveGuidance.PlanFromSpecBuilder.class,
                DriveGuidanceSpec.class
        );
        assertPublicStaticMethod(
                DriveGuidance.class,
                "spec",
                DriveGuidance.SpecBuilder0.class
        );
        assertPublicStaticMethod(
                DriveOverlayStack.class,
                "on",
                DriveOverlayStack.Builder.class,
                DriveSource.class
        );

        assertPublicInstanceMethod(
                DriveGuidancePlan.class,
                "overlay",
                DriveOverlay.class
        );
        assertPublicInstanceMethod(
                DriveGuidancePlan.class,
                "task",
                DriveGuidanceTask.class,
                DriveCommandSink.class,
                DriveGuidanceTask.Config.class
        );
        assertPublicInstanceMethod(
                DriveGuidancePlan.class,
                "query",
                DriveGuidanceQuery.class
        );
        assertPublicInstanceMethod(
                DriveGuidancePlan.class,
                "requestedMask",
                DriveOverlayMask.class
        );
        assertPublicInstanceMethod(
                DriveGuidanceSpec.class,
                "requestedMask",
                DriveOverlayMask.class
        );

        assertPublicInstanceMethod(
                DriveGuidanceQuery.class,
                "get",
                DriveGuidanceStatus.class,
                LoopClock.class
        );
        assertPublicInstanceMethod(
                DriveGuidanceQuery.class,
                "sample",
                DriveGuidanceStatus.class,
                LoopClock.class
        );
        assertPublicInstanceMethod(
                DriveGuidanceQuery.class,
                "sample",
                DriveGuidanceStatus.class,
                LoopClock.class,
                DriveOverlayMask.class
        );

        assertPublicStaticMethod(
                DriveGuidancePlan.Tuning.class,
                "defaults",
                DriveGuidancePlan.Tuning.class
        );
        assertTrue(Modifier.isPublic(
                DriveGuidanceTask.Config.class.getConstructor().getModifiers()));
        assertEquals(1, DriveGuidanceTask.Config.class.getConstructors().length);

        assertPublicMutableField(DriveGuidanceTask.Config.class, "positionTolInches", double.class);
        assertPublicMutableField(DriveGuidanceTask.Config.class, "headingTolRad", double.class);
        assertPublicMutableField(DriveGuidanceTask.Config.class, "timeoutSec", double.class);
        assertPublicMutableField(DriveGuidanceTask.Config.class, "maxNoGuidanceSec", double.class);
        assertPublicMutableField(
                DriveGuidanceTask.Config.class,
                "requestedMask",
                DriveOverlayMask.class
        );
    }

    @Test
    public void legacyAliasesAndParallelTaskFacadeAreAbsent() {
        assertNoDeclaredMethod(DriveSource.class, "overlayStack");
        assertNoDeclaredMethod(DriveGuidancePlan.class, "suggestedMask");
        assertNoDeclaredMethod(DriveGuidanceSpec.class, "suggestedMask");

        try {
            Class.forName(
                    "edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTasks",
                    false,
                    DriveGuidanceApiTest.class.getClassLoader()
            );
            fail("DriveGuidanceTasks must be removed; use DriveGuidancePlan.task(...)");
        } catch (ClassNotFoundException expected) {
            // Expected: guidance Task construction belongs to DriveGuidancePlan.
        }
    }

    @Test
    public void runtimeAndSpecConstructionBypassesAreNotPublic() throws Exception {
        Class<?>[] factoryOwnedTypes = {
                DriveGuidancePlan.class,
                DriveGuidanceSpec.class,
                DriveGuidanceQuery.class,
                DriveGuidanceTask.class,
                DriveGuidancePlan.Tuning.class,
                DriveGuidanceSpec.RobotRelativePoint.class,
                DriveGuidanceSpec.AprilTags.class,
                DriveGuidanceSpec.Localization.class,
                DriveGuidanceSpec.TranslationTakeover.class,
                DriveGuidanceSpec.ResolveWith.class
        };

        for (Class<?> type : factoryOwnedTypes) {
            assertTrue(type.getName() + " must remain a public read/result type",
                    Modifier.isPublic(type.getModifiers()));
            assertEquals(type.getName() + " must not expose a public constructor",
                    0,
                    type.getConstructors().length);
        }

        Method create = DriveGuidanceSpec.ResolveWith.class.getDeclaredMethod(
                "create",
                DriveGuidanceSpec.SolveMode.class,
                DriveGuidanceSpec.AprilTags.class,
                DriveGuidanceSpec.Localization.class,
                TagLayout.class,
                DriveGuidanceSpec.TranslationTakeover.class,
                DriveGuidanceSpec.OmegaPolicy.class,
                DriveGuidanceSpec.LossPolicy.class
        );
        assertFalse("ResolveWith.create(...) must stay builder-internal",
                Modifier.isPublic(create.getModifiers()));
        assertTrue(Modifier.isStatic(create.getModifiers()));
        assertEquals(DriveGuidanceSpec.ResolveWith.class, create.getReturnType());
    }

    private static void assertNoDeclaredMethod(Class<?> owner, String name) {
        for (Method method : owner.getDeclaredMethods()) {
            if (name.equals(method.getName())) {
                fail(owner.getName() + "." + name + " must be removed");
            }
        }
    }

    private static void assertPublicMutableField(Class<?> owner,
                                                 String name,
                                                 Class<?> fieldType) throws Exception {
        java.lang.reflect.Field field = owner.getDeclaredField(name);
        assertTrue(Modifier.isPublic(field.getModifiers()));
        assertFalse(Modifier.isFinal(field.getModifiers()));
        assertEquals(fieldType, field.getType());
    }

    private static void assertPublicStaticMethod(Class<?> owner,
                                                 String name,
                                                 Class<?> returnType,
                                                 Class<?>... parameterTypes) throws Exception {
        Method method = owner.getDeclaredMethod(name, parameterTypes);
        assertTrue(Modifier.isPublic(method.getModifiers()));
        assertTrue(Modifier.isStatic(method.getModifiers()));
        assertEquals(returnType, method.getReturnType());
    }

    private static void assertPublicInstanceMethod(Class<?> owner,
                                                   String name,
                                                   Class<?> returnType,
                                                   Class<?>... parameterTypes) throws Exception {
        Method method = owner.getDeclaredMethod(name, parameterTypes);
        assertTrue(Modifier.isPublic(method.getModifiers()));
        assertFalse(Modifier.isStatic(method.getModifiers()));
        assertEquals(returnType, method.getReturnType());
    }
}
