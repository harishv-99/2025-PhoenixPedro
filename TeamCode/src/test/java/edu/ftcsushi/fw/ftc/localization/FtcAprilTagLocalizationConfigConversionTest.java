package edu.ftcsushi.fw.ftc.localization;

import org.junit.Test;

import edu.ftcsushi.fw.localization.apriltag.AprilTagPoseEstimator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;

/** Verifies CONFIG-03's mount-free FTC config to composed estimator-config conversion. */
public final class FtcAprilTagLocalizationConfigConversionTest {

    @Test
    public void conversionCopiesSolverDraftAndAgeWithoutSensorMount() {
        FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig source =
                FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig.defaults();
        source.fieldPoseSolver.rangeSoftnessInches = 23.0;
        source.maxDetectionAgeSec = 0.17;

        AprilTagPoseEstimator.Config converted =
                source.toAprilTagPoseEstimatorConfig();

        assertNotSame(source.fieldPoseSolver, converted.fieldPoseSolver);
        assertEquals(23.0, converted.fieldPoseSolver.rangeSoftnessInches, 0.0);
        assertEquals(0.17, converted.maxDetectionAgeSec, 0.0);

        source.fieldPoseSolver.rangeSoftnessInches = 91.0;
        assertEquals(23.0, converted.fieldPoseSolver.rangeSoftnessInches, 0.0);
    }

    @Test
    public void rawConversionPreservesNullDraftsForTheEstimatorOwnerToReject() {
        FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig source =
                FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig.defaults();
        source.fieldPoseSolver = null;

        AprilTagPoseEstimator.Config converted =
                source.toAprilTagPoseEstimatorConfig();

        assertNull(converted.fieldPoseSolver);
    }
}
