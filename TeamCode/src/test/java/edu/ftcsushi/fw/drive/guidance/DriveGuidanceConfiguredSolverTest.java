package edu.ftcsushi.fw.drive.guidance;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.spatial.AprilTagSpatialSolveLane;
import edu.ftcsushi.fw.spatial.References;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks down configured solver passage through Drive Guidance into its spatial lane. */
public final class DriveGuidanceConfiguredSolverTest {

    @Test
    public void tuningStagesExposeConfiguredSolverAndRemoveMutableConfigAnswer() throws Exception {
        assertConfiguredSolverMethod(
                DriveGuidance.AprilTagsOnlyTuningStage.class,
                DriveGuidance.AprilTagsOnlyTuningStage.class
        );
        assertConfiguredSolverMethod(
                DriveGuidance.AdaptiveTuningStage.class,
                DriveGuidance.AdaptiveTuningStage.class
        );
        assertNoMethodNamed(
                DriveGuidance.AprilTagsOnlyTuningStage.class,
                "aprilTagFieldPoseConfig"
        );
        assertNoMethodNamed(
                DriveGuidance.AdaptiveTuningStage.class,
                "aprilTagFieldPoseConfig"
        );
    }

    @Test
    public void planCarriesOneConfiguredSolverIntoCompletedSpecAndSpatialLane() throws Exception {
        FixedTagFieldPoseSolver solver = new FixedTagFieldPoseSolver(
                FixedTagFieldPoseSolver.Config.defaults()
        );
        AprilTagSensor sensor = clock -> AprilTagDetections.none();

        DriveGuidancePlan plan = DriveGuidance.plan()
                .faceTo()
                    .point(References.relativeToTagPoint(1, 0.0, 0.0))
                .solveWith()
                    .aprilTagsOnly()
                    .aprilTags(sensor, CameraMountConfig.identity())
                    .aprilTagFieldPoseSolver(solver)
                    .doneAprilTagsOnly()
                .build();

        assertSame(solver, plan.spec.resolveWith.aprilTags.fieldPoseSolver);
        Object lane = plan.spec.spatialQuerySpec.solveSet.lane(0);
        assertTrue(lane instanceof AprilTagSpatialSolveLane);
        Field solverField = AprilTagSpatialSolveLane.class.getDeclaredField("fieldPoseSolver");
        solverField.setAccessible(true);
        assertSame(solver, solverField.get(lane));
    }

    @Test
    public void customSolverAnswerRejectsNullInsteadOfInventingDefaults() {
        try {
            DriveGuidance.plan()
                    .faceTo()
                        .point(References.relativeToTagPoint(1, 0.0, 0.0))
                    .solveWith()
                        .aprilTagsOnly()
                        .aprilTags(clock -> AprilTagDetections.none(), CameraMountConfig.identity())
                        .aprilTagFieldPoseSolver(null);
            fail("Expected configured solver requirement");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("solver"));
        }
    }

    @Test
    public void adaptivePlanCarriesTheSameConfiguredSolverIntoItsSpatialLane() throws Exception {
        FixedTagFieldPoseSolver solver = new FixedTagFieldPoseSolver(
                FixedTagFieldPoseSolver.Config.defaults()
        );
        AprilTagSensor sensor = clock -> AprilTagDetections.none();
        AbsolutePoseEstimator localization = new AbsolutePoseEstimator() {
            @Override
            public void update(LoopClock clock) {
                // No-op test estimator.
            }

            @Override
            public PoseEstimate getEstimate() {
                return PoseEstimate.noPose(LoopTimestamp.unavailable());
            }
        };

        DriveGuidancePlan plan = DriveGuidance.plan()
                .faceTo()
                    .point(References.relativeToTagPoint(1, 0.0, 0.0))
                .solveWith()
                    .adaptive()
                    .localization(localization)
                    .aprilTags(sensor, CameraMountConfig.identity())
                    .aprilTagFieldPoseSolver(solver)
                    .fixedAprilTagLayout(new SimpleTagLayout().addPose(1, Pose3d.zero()))
                    .doneAdaptive()
                .build();

        assertSame(solver, plan.spec.resolveWith.aprilTags.fieldPoseSolver);
        Object lane = plan.spec.spatialQuerySpec.solveSet.lane(1);
        assertTrue(lane instanceof AprilTagSpatialSolveLane);
        Field solverField = AprilTagSpatialSolveLane.class.getDeclaredField("fieldPoseSolver");
        solverField.setAccessible(true);
        assertSame(solver, solverField.get(lane));
    }

    private static void assertConfiguredSolverMethod(Class<?> owner,
                                                     Class<?> returnType) throws Exception {
        Method method = owner.getDeclaredMethod(
                "aprilTagFieldPoseSolver",
                FixedTagFieldPoseSolver.class
        );
        assertTrue(Modifier.isPublic(method.getModifiers()));
        assertFalse(Modifier.isStatic(method.getModifiers()));
        assertEquals(returnType, method.getReturnType());
    }

    private static void assertNoMethodNamed(Class<?> owner, String name) {
        for (Method method : owner.getDeclaredMethods()) {
            if (name.equals(method.getName())) {
                fail(owner.getName() + "." + name + " must be absent");
            }
        }
    }
}
