package edu.ftcphoenix.fw.spatial;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;

import edu.ftcphoenix.fw.core.source.TimeAwareSource;
import edu.ftcphoenix.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies configured solver ownership across fixed and timestamp-aware spatial paths. */
public final class AprilTagSpatialSolveLaneConfiguredSolverTest {

    @Test
    public void customConstructorsAndBuilderStagesTakeConfiguredSolver() throws Exception {
        AprilTagSpatialSolveLane.class.getConstructor(
                AprilTagSensor.class,
                CameraMountConfig.class,
                double.class,
                FixedTagFieldPoseSolver.class
        );
        AprilTagSpatialSolveLane.class.getConstructor(
                AprilTagSensor.class,
                TimeAwareSource.class,
                double.class,
                FixedTagFieldPoseSolver.class
        );
        assertConstructorAbsent(
                AprilTagSensor.class,
                CameraMountConfig.class,
                double.class,
                FixedTagFieldPoseSolver.Config.class
        );
        assertConstructorAbsent(
                AprilTagSensor.class,
                TimeAwareSource.class,
                double.class,
                FixedTagFieldPoseSolver.Config.class
        );

        assertBuilderMethod(SpatialSolveSet.FirstLaneStep.class, CameraMountConfig.class);
        assertBuilderMethod(SpatialSolveSet.FirstLaneStep.class, TimeAwareSource.class);
        assertBuilderMethod(SpatialSolveSet.MoreLanesStep.class, CameraMountConfig.class);
        assertBuilderMethod(SpatialSolveSet.MoreLanesStep.class, TimeAwareSource.class);
        assertBuilderMethodAbsent(SpatialSolveSet.FirstLaneStep.class, CameraMountConfig.class);
        assertBuilderMethodAbsent(SpatialSolveSet.FirstLaneStep.class, TimeAwareSource.class);
        assertBuilderMethodAbsent(SpatialSolveSet.MoreLanesStep.class, CameraMountConfig.class);
        assertBuilderMethodAbsent(SpatialSolveSet.MoreLanesStep.class, TimeAwareSource.class);
    }

    @Test
    public void fixedAndDynamicSolveSetsRetainTheExactConfiguredSolverCapability() throws Exception {
        FixedTagFieldPoseSolver solver = new FixedTagFieldPoseSolver(
                FixedTagFieldPoseSolver.Config.defaults()
        );
        AprilTagSensor sensor = clock -> AprilTagDetections.none();
        TimeAwareSource<CameraMountConfig> dynamicMount =
                (clock, timestamp) -> CameraMountConfig.identity();

        SpatialSolveSet fixed = SpatialSolveSet.builder()
                .aprilTags(sensor, CameraMountConfig.identity(), 0.5, solver)
                .build();
        SpatialSolveSet dynamic = SpatialSolveSet.builder()
                .aprilTags(sensor, dynamicMount, 0.5, solver)
                .build();

        assertSame(solver, solverFrom(fixed.lane(0)));
        assertSame(solver, solverFrom(dynamic.lane(0)));
    }

    @Test
    public void customLaneRejectsNullSolverRatherThanConstructingDefaults() {
        try {
            new AprilTagSpatialSolveLane(
                    clock -> AprilTagDetections.none(),
                    CameraMountConfig.identity(),
                    0.5,
                    null
            );
            fail("Expected configured solver requirement");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("fieldPoseSolver"));
        }
    }

    private static Object solverFrom(SpatialSolveLane lane) throws Exception {
        Field field = AprilTagSpatialSolveLane.class.getDeclaredField("fieldPoseSolver");
        field.setAccessible(true);
        return field.get(lane);
    }

    private static void assertBuilderMethod(Class<?> owner,
                                            Class<?> mountType) throws Exception {
        Method method = owner.getDeclaredMethod(
                "aprilTags",
                AprilTagSensor.class,
                mountType,
                double.class,
                FixedTagFieldPoseSolver.class
        );
        assertTrue(method.getReturnType() == SpatialSolveSet.MoreLanesStep.class);
    }

    private static void assertConstructorAbsent(Class<?>... parameters) {
        try {
            Constructor<?> ignored = AprilTagSpatialSolveLane.class.getConstructor(parameters);
            fail("Unexpected mutable-config constructor: " + ignored);
        } catch (NoSuchMethodException expected) {
            // Expected.
        }
    }

    private static void assertBuilderMethodAbsent(Class<?> owner, Class<?> mountType) {
        try {
            Method ignored = owner.getDeclaredMethod(
                    "aprilTags",
                    AprilTagSensor.class,
                    mountType,
                    double.class,
                    FixedTagFieldPoseSolver.Config.class
            );
            fail("Unexpected mutable-config builder method: " + ignored);
        } catch (NoSuchMethodException expected) {
            // Expected.
        }
    }
}
