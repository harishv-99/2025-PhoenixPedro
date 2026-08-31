package edu.ftcsushi.fw.drive.guidance;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveOverlay;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.drive.DriveOverlayStack;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.task.Task;

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

        assertPublicStaticMethod(
                DriveGuidance.class,
                "poseLock",
                DriveOverlay.class,
                AbsolutePoseEstimator.class
        );
        assertPublicStaticMethod(
                DriveGuidance.class,
                "poseLock",
                DriveOverlay.class,
                AbsolutePoseEstimator.class,
                DriveGuidancePlan.Tuning.class
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
        assertTuningSurface();
        assertDriveTuningStageSurface();
        assertGoToPoseTaskSurface();

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
                    "edu.ftcsushi.fw.drive.guidance.DriveGuidanceTasks",
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

    private static void assertTuningSurface() throws Exception {
        assertTrue(Modifier.isPublic(DriveGuidancePlan.Tuning.class.getModifiers()));
        assertTrue(Modifier.isStatic(DriveGuidancePlan.Tuning.class.getModifiers()));
        assertTrue(Modifier.isFinal(DriveGuidancePlan.Tuning.class.getModifiers()));
        assertEquals(0, DriveGuidancePlan.Tuning.class.getConstructors().length);
        assertEquals(1, DriveGuidancePlan.Tuning.class.getDeclaredConstructors().length);
        java.lang.reflect.Constructor<?> tuningConstructor =
                DriveGuidancePlan.Tuning.class.getDeclaredConstructor(
                        double.class,
                        double.class,
                        double.class,
                        double.class,
                        double.class,
                        double.class
                );
        int constructorModifiers = tuningConstructor.getModifiers();
        assertFalse(Modifier.isPublic(constructorModifiers));
        assertFalse(Modifier.isProtected(constructorModifiers));
        assertFalse(Modifier.isPrivate(constructorModifiers));

        assertPublicFinalField(DriveGuidancePlan.Tuning.class, "kPTranslate", double.class);
        assertPublicFinalField(DriveGuidancePlan.Tuning.class, "maxTranslateCmd", double.class);
        assertPublicFinalField(DriveGuidancePlan.Tuning.class, "kPAim", double.class);
        assertPublicFinalField(DriveGuidancePlan.Tuning.class, "maxOmegaCmd", double.class);
        assertPublicFinalField(DriveGuidancePlan.Tuning.class, "minOmegaCmd", double.class);
        assertPublicFinalField(DriveGuidancePlan.Tuning.class, "aimDeadbandRad", double.class);

        assertPublicInstanceMethod(
                DriveGuidancePlan.Tuning.class,
                "withTranslateKp",
                DriveGuidancePlan.Tuning.class,
                double.class
        );
        assertPublicInstanceMethod(
                DriveGuidancePlan.Tuning.class,
                "withMaxTranslateCmd",
                DriveGuidancePlan.Tuning.class,
                double.class
        );
        assertPublicInstanceMethod(
                DriveGuidancePlan.Tuning.class,
                "withAimKp",
                DriveGuidancePlan.Tuning.class,
                double.class
        );
        assertPublicInstanceMethod(
                DriveGuidancePlan.Tuning.class,
                "withMaxOmegaCmd",
                DriveGuidancePlan.Tuning.class,
                double.class
        );
        assertPublicInstanceMethod(
                DriveGuidancePlan.Tuning.class,
                "withMinOmegaCmd",
                DriveGuidancePlan.Tuning.class,
                double.class
        );
        assertPublicInstanceMethod(
                DriveGuidancePlan.Tuning.class,
                "withAimDeadbandRad",
                DriveGuidancePlan.Tuning.class,
                double.class
        );

        int publicDeclaredMethods = 0;
        for (Method method : DriveGuidancePlan.Tuning.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicDeclaredMethods++;
            }
        }
        assertEquals("Tuning must retain defaults() plus exactly six named withers",
                7, publicDeclaredMethods);
    }

    private static void assertDriveTuningStageSurface() throws Exception {
        assertPublicInstanceMethod(
                DriveGuidance.PlanOptionalTuningStage.class,
                "driveTuning",
                DriveGuidance.DriveTuningBranch.class
        );
        assertPublicInstanceMethod(
                DriveGuidance.DriveTuningBranch.class,
                "use",
                DriveGuidance.DriveTuningBranch.class,
                DriveGuidancePlan.Tuning.class
        );
        assertPublicInstanceMethod(
                DriveGuidance.DriveTuningBranch.class,
                "translateKp",
                DriveGuidance.DriveTuningBranch.class,
                double.class
        );
        assertPublicInstanceMethod(
                DriveGuidance.DriveTuningBranch.class,
                "maxTranslateCmd",
                DriveGuidance.DriveTuningBranch.class,
                double.class
        );
        assertPublicInstanceMethod(
                DriveGuidance.DriveTuningBranch.class,
                "aimKp",
                DriveGuidance.DriveTuningBranch.class,
                double.class
        );
        assertPublicInstanceMethod(
                DriveGuidance.DriveTuningBranch.class,
                "maxOmegaCmd",
                DriveGuidance.DriveTuningBranch.class,
                double.class
        );
        assertPublicInstanceMethod(
                DriveGuidance.DriveTuningBranch.class,
                "minOmegaCmd",
                DriveGuidance.DriveTuningBranch.class,
                double.class
        );
        assertPublicInstanceMethod(
                DriveGuidance.DriveTuningBranch.class,
                "aimDeadbandRad",
                DriveGuidance.DriveTuningBranch.class,
                double.class
        );
        assertPublicInstanceMethod(
                DriveGuidance.DriveTuningBranch.class,
                "doneDriveTuning",
                DriveGuidance.PlanOptionalTuningStage.class
        );
    }

    private static void assertGoToPoseTaskSurface() throws Exception {
        assertPublicStaticMethod(
                GoToPoseTasks.class,
                "goToPoseFieldRelative",
                Task.class,
                AbsolutePoseEstimator.class,
                DriveCommandSink.class,
                Pose2d.class,
                DriveGuidancePlan.Tuning.class,
                DriveGuidanceTask.Config.class
        );
        assertPublicStaticMethod(
                GoToPoseTasks.class,
                "goToPoseTagRelative",
                Task.class,
                AbsolutePoseEstimator.class,
                DriveCommandSink.class,
                TagLayout.class,
                int.class,
                double.class,
                double.class,
                double.class,
                DriveGuidancePlan.Tuning.class,
                DriveGuidanceTask.Config.class
        );
        assertPublicStaticMethod(
                GoToPoseTasks.class,
                "holdPositionAndAimFieldHeading",
                Task.class,
                AbsolutePoseEstimator.class,
                DriveCommandSink.class,
                double.class,
                DriveGuidancePlan.Tuning.class,
                DriveGuidanceTask.Config.class
        );
        assertPublicStaticMethod(
                GoToPoseTasks.class,
                "aimOnlyFieldHeading",
                Task.class,
                AbsolutePoseEstimator.class,
                DriveCommandSink.class,
                double.class,
                DriveGuidancePlan.Tuning.class,
                DriveGuidanceTask.Config.class
        );
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

    private static void assertPublicFinalField(Class<?> owner,
                                               String name,
                                               Class<?> fieldType) throws Exception {
        java.lang.reflect.Field field = owner.getDeclaredField(name);
        assertTrue(Modifier.isPublic(field.getModifiers()));
        assertTrue(Modifier.isFinal(field.getModifiers()));
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
