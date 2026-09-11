package edu.ftcsushi.fw.drive.guidance;

import org.junit.Test;

import java.lang.reflect.Method;
import java.util.Collections;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.spatial.References;
import edu.ftcsushi.fw.spatial.SpatialControlFrames;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Real guidance/spatial owners with authored pose and camera evidence, not hardware accuracy tests. */
public final class DriveGuidanceEvidenceModeTest {

    @Test public void absolutePoseAndRelativeTagKeepTheirDifferentAuthorities() {
        ManualLoopClock time = new ManualLoopClock();
        PoseEstimate pose = new PoseEstimate(new Pose3d(5, 0, 0, 0, 0, 0), true, 1,
                time.clock().nowTimestamp());
        int[] reads = {0};
        AbsolutePoseEstimator estimator = estimator(pose);
        AprilTagSensor sensor = clock -> {
            reads[0]++;
            return AprilTagDetections.fromFrame(clock.nowTimestamp(), Collections.singletonList(
                    AprilTagObservation.target(1, new Pose3d(20, 0, 0, 0, 0, 0))));
        };
        DriveGuidancePlan field = DriveGuidance.plan().translateTo()
                .point(References.relativeToTagPoint(1, 0, 0))
                .solveWith().absolutePose(estimator)
                .fixedAprilTagLayout(new SimpleTagLayout().addPose(1, new Pose3d(100, 0, 0, 0, 0, 0)))
                .doneAbsolutePose().build();
        DriveGuidancePlan local = DriveGuidance.plan().translateTo()
                .point(References.relativeToTagPoint(1, 0, 0))
                .solveWith().relativeAprilTags(sensor, CameraMountConfig.identity())
                .doneRelativeAprilTags().build();

        DriveGuidanceStatus fieldStatus = field.query().get(time.clock());
        assertEquals(95.0, fieldStatus.forwardErrorIn, 1e-9);
        assertEquals(0, reads[0]);
        DriveGuidanceStatus localStatus = local.query().get(time.clock());
        assertEquals(20.0, localStatus.forwardErrorIn, 1e-9);
        assertEquals(1, reads[0]);
        assertEquals(DriveGuidanceSpec.SolveMode.ABSOLUTE_POSE, fieldStatus.solveMode);
        assertEquals(DriveGuidanceSpec.SolveMode.RELATIVE_APRIL_TAGS, localStatus.solveMode);
        assertEquals(0.50, field.spec.resolveWith.absolutePose.maxAgeSec, 0);
        assertEquals(0.10, field.spec.resolveWith.absolutePose.minQuality, 0);
        assertEquals(0.50, local.spec.resolveWith.relativeAprilTags.maxAgeSec, 0);
        assertEquals(DriveGuidanceSpec.LossPolicy.PASS_THROUGH, local.spec.resolveWith.lossPolicy);
    }

    @Test public void directTagGeometryUsesMountToolAndTagOffsetWithoutSdkFieldPose() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections frame = AprilTagDetections.fromFrame(time.clock().nowTimestamp(),
                Collections.singletonList(AprilTagObservation.target(7,
                        new Pose3d(10, 2, 0, Math.PI / 2, 0, 0),
                        new Pose3d(999, -999, 0, Math.PI, 0, 0))));
        Pose2d tool = new Pose2d(1, -1, 0);
        DriveGuidanceQuery query = DriveGuidance.plan().translateTo()
                .point(References.relativeToTagPoint(7, 2, 0))
                .andFaceTo().point(References.relativeToTagPoint(7, 2, 0))
                .controlFrames(SpatialControlFrames.robotCenter()
                        .withTranslationFrame(tool).withFacingFrame(tool))
                .solveWith().relativeAprilTags(clock -> frame,
                        CameraMountConfig.ofPose(new Pose3d(2, 3, 0, 0, 0, 0)))
                .maxAgeSec(0.10).doneRelativeAprilTags().build().query();
        DriveGuidanceStatus status = query.get(time.clock());
        assertEquals(11, status.forwardErrorIn, 1e-9);
        assertEquals(8, status.leftErrorIn, 1e-9);
        assertEquals(Math.atan2(8, 11), status.omegaErrorRad, 1e-9);
        assertSame(status, query.get(time.clock()));
        time.nextCycle(0.11);
        DriveGuidanceStatus stale = query.get(time.clock());
        assertFalse(stale.hasTranslationError);
        assertFalse(stale.hasOmegaError);
        assertEquals(DriveOverlayMask.NONE, stale.mask);
        assertFalse(stale.translationWithin(1000));
        assertFalse(stale.omegaWithin(Math.PI));
        assertEquals(DriveGuidanceSpec.SolveMode.RELATIVE_APRIL_TAGS, stale.solveMode);
    }

    @Test public void relativeModeRejectsFieldOnlyTargetsAtBuild() {
        AprilTagSensor noTags = clock -> AprilTagDetections.none();
        assertBuildRejected(() -> DriveGuidance.plan().translateTo().fieldPointInches(10, 3)
                .solveWith().relativeAprilTags(noTags, CameraMountConfig.identity())
                .doneRelativeAprilTags().build());
        assertBuildRejected(() -> DriveGuidance.plan().faceTo().fieldHeadingRad(0)
                .solveWith().relativeAprilTags(noTags, CameraMountConfig.identity())
                .doneRelativeAprilTags().build());
        assertBuildRejected(() -> DriveGuidance.plan().faceTo()
                .frameHeading(References.fieldFrame(1, 2, 0))
                .solveWith().relativeAprilTags(noTags, CameraMountConfig.identity())
                .doneRelativeAprilTags().build());
        assertBuildRejected(() -> DriveGuidance.plan().translateTo().robotRelativePointInches(1, 2)
                .solveWith().relativeAprilTags(noTags, CameraMountConfig.identity())
                .doneRelativeAprilTags().build());
    }

    @Test public void relativeBranchCannotAcquireFieldLayoutOrFieldSolver() {
        for (Method method : DriveGuidance.RelativeAprilTagsTuningStage.class.getDeclaredMethods()) {
            assertFalse(method.getName().contains("Field"));
            assertFalse(method.getName().contains("localization"));
        }
        for (java.lang.reflect.Field field : DriveGuidanceSpec.RelativeAprilTags.class.getFields()) {
            assertFalse(field.getName().contains("Solver"));
        }
    }

    @Test public void ageAndQualitySettingsFailWithActionableErrors() {
        ManualLoopClock time = new ManualLoopClock();
        AbsolutePoseEstimator estimator = estimator(new PoseEstimate(Pose3d.zero(), true, 1,
                time.clock().nowTimestamp()));
        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -0.1}) {
            IllegalStateException error = assertThrows(IllegalStateException.class,
                    () -> DriveGuidance.plan().faceTo().fieldHeadingRad(0)
                            .solveWith().absolutePose(estimator).maxAgeSec(invalid)
                            .doneAbsolutePose().build());
            assertTrue(error.getMessage().contains("maxAgeSec"));
        }
        IllegalStateException error = assertThrows(IllegalStateException.class,
                () -> DriveGuidance.plan().faceTo().fieldHeadingRad(0)
                        .solveWith().absolutePose(estimator).minQuality(1.01)
                        .doneAbsolutePose().build());
        assertTrue(error.getMessage().contains("minQuality"));
        assertThrows(NullPointerException.class, () -> DriveGuidance.plan().faceTo().fieldHeadingRad(0)
                .solveWith().absolutePose(null));
    }

    private static AbsolutePoseEstimator estimator(PoseEstimate estimate) {
        return new AbsolutePoseEstimator() {
            @Override public void update(LoopClock clock) { fail("Guidance must not update localization"); }
            @Override public PoseEstimate getEstimate() { return estimate; }
        };
    }

    private static void assertBuildRejected(Runnable action) {
        IllegalStateException error = assertThrows(IllegalStateException.class, action::run);
        assertTrue(error.getMessage(), error.getMessage().contains("relativeAprilTags"));
        assertTrue(error.getMessage(), error.getMessage().contains("absolutePose"));
    }
}
