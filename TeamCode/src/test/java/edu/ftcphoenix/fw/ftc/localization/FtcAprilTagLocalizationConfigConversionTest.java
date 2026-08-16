package edu.ftcphoenix.fw.ftc.localization;

import org.junit.Test;

import edu.ftcphoenix.fw.localization.apriltag.AprilTagPoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;

/** Verifies CONFIG-03's mount-free FTC config to composed estimator-config conversion. */
public final class FtcAprilTagLocalizationConfigConversionTest {

    @Test
    public void conversionCopiesSolverDraftAndAssignsOnlyAgeAndActiveVisionMount() {
        FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig source =
                FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig.defaults();
        source.fieldPoseSolver.rangeSoftnessInches = 23.0;
        source.maxDetectionAgeSec = 0.17;
        CameraMountConfig mount = CameraMountConfig.of(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);

        AprilTagPoseEstimator.Config converted =
                source.toAprilTagPoseEstimatorConfig(mount);

        assertNotSame(source.fieldPoseSolver, converted.fieldPoseSolver);
        assertEquals(23.0, converted.fieldPoseSolver.rangeSoftnessInches, 0.0);
        assertEquals(0.17, converted.maxDetectionAgeSec, 0.0);
        assertSame(mount, converted.cameraMount);

        source.fieldPoseSolver.rangeSoftnessInches = 91.0;
        assertEquals(23.0, converted.fieldPoseSolver.rangeSoftnessInches, 0.0);
    }

    @Test
    public void rawConversionPreservesNullDraftsForTheEstimatorOwnerToReject() {
        FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig source =
                FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig.defaults();
        source.fieldPoseSolver = null;

        AprilTagPoseEstimator.Config converted =
                source.toAprilTagPoseEstimatorConfig(null);

        assertNull(converted.fieldPoseSolver);
        assertNull(converted.cameraMount);
    }
}
