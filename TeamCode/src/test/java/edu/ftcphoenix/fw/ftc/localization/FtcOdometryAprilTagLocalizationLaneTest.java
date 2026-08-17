package edu.ftcphoenix.fw.ftc.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.field.SimpleTagLayout;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.VisionReadiness;
import edu.ftcphoenix.fw.localization.MotionDelta;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.fusion.OdometryCorrectionEkfEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryCorrectionFusionEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused ownership coverage for the localization lane's injected-predictor path. */
public final class FtcOdometryAprilTagLocalizationLaneTest {

    @Test
    public void withPredictorUsesExactlyTheSuppliedPredictor() {
        RecordingPredictor predictor = new RecordingPredictor();
        FtcOdometryAprilTagLocalizationLane.EstimatorConfig cfg =
                FtcOdometryAprilTagLocalizationLane.EstimatorConfig.defaults();

        FtcOdometryAprilTagLocalizationLane lane =
                FtcOdometryAprilTagLocalizationLane.withPredictor(
                        predictor,
                        noDetectionsVisionLane(),
                        new SimpleTagLayout(),
                        cfg
                );

        assertSame(predictor, lane.predictor());

        ManualLoopClock clock = new ManualLoopClock();
        lane.update(clock.nextCycle(0.02));

        assertEquals(1, predictor.updateCount);
        assertEquals(1.0, lane.globalEstimator().getEstimate().fieldToRobotPose.xInches, 1e-9);
    }

    @Test
    public void withPredictorStillDefensivelyCapturesCorrectionConfig() {
        FtcOdometryAprilTagLocalizationLane.EstimatorConfig cfg =
                FtcOdometryAprilTagLocalizationLane.EstimatorConfig.defaults();
        cfg.correctedEstimatorMode = FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.EKF;
        cfg.correctionFusion = null;

        FtcOdometryAprilTagLocalizationLane lane =
                FtcOdometryAprilTagLocalizationLane.withPredictor(
                        new RecordingPredictor(),
                        noDetectionsVisionLane(),
                        new SimpleTagLayout(),
                        cfg
                );

        cfg.correctedEstimatorMode = FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;

        assertTrue(lane.globalEstimator() instanceof OdometryCorrectionEkfEstimator);
    }

    @Test
    public void configShapesCopyNonNullDraftsAndPreserveNullDrafts() {
        FtcOdometryAprilTagLocalizationLane.Config complete =
                FtcOdometryAprilTagLocalizationLane.Config.defaults();
        FtcOdometryAprilTagLocalizationLane.Config completeCopy = complete.copy();

        assertNotSame(complete.predictor, completeCopy.predictor);
        assertNotSame(complete.estimation, completeCopy.estimation);

        complete.predictor = null;
        complete.estimation = null;
        completeCopy = complete.copy();
        assertNull(completeCopy.predictor);
        assertNull(completeCopy.estimation);

        FtcOdometryAprilTagLocalizationLane.EstimatorConfig estimation =
                FtcOdometryAprilTagLocalizationLane.EstimatorConfig.defaults();
        FtcOdometryAprilTagLocalizationLane.EstimatorConfig estimationCopy = estimation.copy();
        assertNotSame(estimation.aprilTags, estimationCopy.aprilTags);
        assertNotSame(estimation.aprilTags.fieldPoseSolver,
                estimationCopy.aprilTags.fieldPoseSolver);
        assertNotSame(estimation.correctionSource, estimationCopy.correctionSource);
        assertNotSame(estimation.correctionSource.limelightFieldPose,
                estimationCopy.correctionSource.limelightFieldPose);
        assertNotSame(estimation.correctionFusion, estimationCopy.correctionFusion);
        assertNotSame(estimation.correctionEkf, estimationCopy.correctionEkf);

        estimation.aprilTags = null;
        estimation.correctionSource = null;
        estimation.correctionFusion = null;
        estimation.correctionEkf = null;
        estimationCopy = estimation.copy();
        assertNull(estimationCopy.aprilTags);
        assertNull(estimationCopy.correctionSource);
        assertNull(estimationCopy.correctionFusion);
        assertNull(estimationCopy.correctionEkf);
    }

    @Test
    public void aprilTagPolicyValidatedCopyPreservesZeroAndOwnsSolverDiagnostics() {
        FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig source =
                FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig.defaults();
        source.maxDetectionAgeSec = 0.0;
        source.fieldPoseSolver.maxAbsBearingRad = 0.7;

        FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig captured =
                source.validatedCopy("  tool.aprilTags  ");

        assertNotSame(source, captured);
        assertNotSame(source.fieldPoseSolver, captured.fieldPoseSolver);
        assertEquals(0.0, captured.maxDetectionAgeSec, 0.0);
        assertEquals(0.7, captured.fieldPoseSolver.maxAbsBearingRad, 0.0);

        source.maxDetectionAgeSec = 0.4;
        source.fieldPoseSolver.maxAbsBearingRad = 0.2;
        assertEquals(0.0, captured.maxDetectionAgeSec, 0.0);
        assertEquals(0.7, captured.fieldPoseSolver.maxAbsBearingRad, 0.0);

        source.fieldPoseSolver.rangeSoftnessInches = Double.NaN;
        RuntimeException solverFailure = captureFailure(
                () -> source.validatedCopy("  tool.aprilTags  ")
        );
        assertTrue(solverFailure.getMessage().contains(
                "tool.aprilTags.fieldPoseSolver.rangeSoftnessInches"
        ));
        assertTrue(solverFailure.getMessage().contains("NaN"));

        FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig invalidAge =
                FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig.defaults();
        invalidAge.maxDetectionAgeSec = Double.POSITIVE_INFINITY;
        RuntimeException canonicalFailure = captureFailure(
                () -> invalidAge.validatedCopy("  ")
        );
        assertTrue(canonicalFailure.getMessage().contains(
                FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig.class
                        .getCanonicalName() + ".maxDetectionAgeSec"
        ));
    }

    @Test
    public void outerValidatedCopyChecksOnlyIntrinsicActiveBranchesAndIsIndependent() {
        FtcOdometryAprilTagLocalizationLane.Config source =
                FtcOdometryAprilTagLocalizationLane.Config.defaults();
        source.estimation.correctedEstimatorMode =
                FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;
        source.estimation.correctionFusion.correctionPositionGain = 0.23;
        source.estimation.correctionEkf = null;
        source.estimation.correctionSource.mode =
                FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.APRILTAG_POSE;
        source.estimation.correctionSource.limelightFieldPose.maxResultAgeSec = Double.NaN;

        FtcOdometryAprilTagLocalizationLane.Config captured =
                source.validatedCopy("  robot.localization  ");

        assertNotSame(source, captured);
        assertNotSame(source.predictor, captured.predictor);
        assertNotSame(source.estimation, captured.estimation);
        assertNotSame(source.estimation.aprilTags, captured.estimation.aprilTags);
        assertNotSame(
                source.estimation.correctionSource,
                captured.estimation.correctionSource
        );
        assertNotSame(
                source.estimation.correctionSource.limelightFieldPose,
                captured.estimation.correctionSource.limelightFieldPose
        );
        assertNull(captured.estimation.correctionEkf);
        assertEquals(
                0.23,
                captured.estimation.correctionFusion.correctionPositionGain,
                0.0
        );
        assertTrue(Double.isNaN(
                captured.estimation.correctionSource.limelightFieldPose.maxResultAgeSec
        ));

        source.estimation.correctionFusion.correctionPositionGain = 0.71;
        source.estimation.correctionSource.limelightFieldPose.maxResultAgeSec = 0.2;
        assertEquals(
                0.23,
                captured.estimation.correctionFusion.correctionPositionGain,
                0.0
        );
        assertTrue(Double.isNaN(
                captured.estimation.correctionSource.limelightFieldPose.maxResultAgeSec
        ));

        FtcOdometryAprilTagLocalizationLane.Config direct =
                FtcOdometryAprilTagLocalizationLane.Config.defaults();
        direct.estimation.correctionSource.mode =
                FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.LIMELIGHT_FIELD_POSE;
        direct.estimation.correctionSource.limelightFieldPose.maxResultAgeSec = Double.NaN;
        RuntimeException directFailure = captureFailure(
                () -> direct.validatedCopy("  robot.localization  ")
        );
        assertTrue(directFailure.getMessage().contains(
                "robot.localization.estimation.correctionSource.limelightFieldPose.maxResultAgeSec"
        ));

        FtcOdometryAprilTagLocalizationLane.Config ekf =
                FtcOdometryAprilTagLocalizationLane.Config.defaults();
        ekf.estimation.correctedEstimatorMode =
                FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.EKF;
        ekf.estimation.correctionFusion = null;
        assertNull(ekf.validatedCopy(null).estimation.correctionFusion);
    }

    @Test
    public void injectedPathAcceptsNullInactiveDraftsAndTouchesNoPredictorAtConstruction() {
        RecordingPredictor predictor = new RecordingPredictor();
        FtcOdometryAprilTagLocalizationLane.EstimatorConfig cfg =
                FtcOdometryAprilTagLocalizationLane.EstimatorConfig.defaults();
        cfg.correctionEkf = null;
        cfg.correctionSource.limelightFieldPose = null;

        FtcOdometryAprilTagLocalizationLane lane =
                FtcOdometryAprilTagLocalizationLane.withPredictor(
                        predictor,
                        new RecordingVisionLane(),
                        new SimpleTagLayout(),
                        cfg
                );

        assertSame(predictor, lane.predictor());
        assertEquals(0, predictor.updateCount);
        assertTrue(lane.globalEstimator() instanceof OdometryCorrectionFusionEstimator);
        assertNull(lane.limelightFieldPoseEstimator());
    }

    @Test
    public void activeConfigFailsBeforeLayoutVisionOrOwnedHardwareEffects() {
        List<String> events = new ArrayList<String>();
        EventHardwareMap hardwareMap = new EventHardwareMap(events);
        EventVisionLane vision = new EventVisionLane(events);
        EventTagLayout layout = new EventTagLayout(events);
        FtcOdometryAprilTagLocalizationLane.Config cfg =
                FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.estimation.correctionFusion.maxCorrectionAgeSec = Double.NaN;

        RuntimeException failure = captureFailure(() ->
                new FtcOdometryAprilTagLocalizationLane(hardwareMap, vision, layout, cfg));

        assertTrue(failure instanceof IllegalArgumentException);
        assertTrue(failure.getMessage().contains("Config.estimation.correctionFusion"));
        assertTrue(failure.getMessage().contains("maxCorrectionAgeSec"));
        assertTrue(events.isEmpty());
        assertEquals(0, hardwareMap.lookupCount);
    }

    @Test
    public void incompleteActiveDraftsFailBeforeLayoutVisionOrOwnedHardwareEffects() {
        FtcOdometryAprilTagLocalizationLane.Config cfg;

        assertOwnedPreflightFailsBeforeCollaborators(null, "config");

        cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.predictor = null;
        assertOwnedPreflightFailsBeforeCollaborators(cfg, "Config.predictor");

        cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.predictor.hardwareMapName = "  ";
        assertOwnedPreflightFailsBeforeCollaborators(cfg, "hardwareMapName");

        cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.estimation = null;
        assertOwnedPreflightFailsBeforeCollaborators(cfg, "Config.estimation");

        cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.estimation.aprilTags = null;
        assertOwnedPreflightFailsBeforeCollaborators(cfg, "estimation.aprilTags");

        cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.estimation.aprilTags.fieldPoseSolver = null;
        assertOwnedPreflightFailsBeforeCollaborators(cfg, "aprilTags.fieldPoseSolver");

        cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.estimation.correctedEstimatorMode =
                FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.EKF;
        cfg.estimation.correctionEkf.maxCorrectionAgeSec = Double.NaN;
        assertOwnedPreflightFailsBeforeCollaborators(
                cfg,
                "Config.estimation.correctionEkf.maxCorrectionAgeSec"
        );

        cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.estimation.correctionSource = null;
        assertOwnedPreflightFailsBeforeCollaborators(cfg, "estimation.correctionSource");

        cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.estimation.correctionSource.mode = null;
        assertOwnedPreflightFailsBeforeCollaborators(cfg, "correctionSource.mode");

        cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.estimation.correctedEstimatorMode = null;
        assertOwnedPreflightFailsBeforeCollaborators(cfg, "correctedEstimatorMode");

        cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.estimation.correctionFusion = null;
        assertOwnedPreflightFailsBeforeCollaborators(cfg, "correctionFusion");

        cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.estimation.correctedEstimatorMode =
                FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.EKF;
        cfg.estimation.correctionEkf = null;
        assertOwnedPreflightFailsBeforeCollaborators(cfg, "correctionEkf");

        cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.estimation.correctionSource.mode =
                FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.LIMELIGHT_FIELD_POSE;
        cfg.estimation.correctionSource.limelightFieldPose = null;
        assertOwnedPreflightFailsBeforeCollaborators(cfg, "limelightFieldPose");
    }

    @Test
    public void incompatibleLimelightSelectionFailsBeforeLayoutVisionOrHardwareEffects() {
        List<String> events = new ArrayList<String>();
        EventHardwareMap hardwareMap = new EventHardwareMap(events);
        EventVisionLane vision = new EventVisionLane(events);
        EventTagLayout layout = new EventTagLayout(events);
        FtcOdometryAprilTagLocalizationLane.Config cfg =
                FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.estimation.correctionSource.mode =
                FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.LIMELIGHT_FIELD_POSE;

        RuntimeException failure = captureFailure(() ->
                new FtcOdometryAprilTagLocalizationLane(hardwareMap, vision, layout, cfg));

        assertTrue(failure instanceof IllegalArgumentException);
        assertTrue(failure.getMessage().contains("LIMELIGHT_FIELD_POSE"));
        assertTrue(failure.getMessage().contains("FtcLimelightAprilTagVisionLane"));
        assertTrue(events.isEmpty());
        assertEquals(0, hardwareMap.lookupCount);
    }

    @Test
    public void malformedLayoutAndMissingVisionFactsFailBeforeOwnedHardwareLookup() {
        List<String> events = new ArrayList<String>();
        EventHardwareMap hardwareMap = new EventHardwareMap(events);
        EventVisionLane vision = new EventVisionLane(events);
        EventTagLayout layout = new EventTagLayout(events);
        layout.pose = null;
        EventHardwareMap layoutHardware = hardwareMap;
        EventVisionLane invalidLayoutVision = vision;
        EventTagLayout invalidLayoutSource = layout;

        RuntimeException invalidLayout = captureFailure(() ->
                new FtcOdometryAprilTagLocalizationLane(
                        layoutHardware,
                        invalidLayoutVision,
                        invalidLayoutSource,
                        FtcOdometryAprilTagLocalizationLane.Config.defaults()
                ));
        assertTrue(invalidLayout.getMessage().contains("layout pose"));
        assertEquals(Arrays.asList("layout.ids", "layout.pose.7"), events);
        assertEquals(0, hardwareMap.lookupCount);
        assertEquals(0, vision.mountCount);
        assertEquals(0, vision.sensorCount);

        events = new ArrayList<String>();
        hardwareMap = new EventHardwareMap(events);
        vision = new EventVisionLane(events);
        layout = new EventTagLayout(events);
        vision.mount = null;
        EventHardwareMap mountHardware = hardwareMap;
        EventVisionLane missingMountVision = vision;
        EventTagLayout mountLayout = layout;

        RuntimeException missingMount = captureFailure(() ->
                new FtcOdometryAprilTagLocalizationLane(
                        mountHardware,
                        missingMountVision,
                        mountLayout,
                        FtcOdometryAprilTagLocalizationLane.Config.defaults()
                ));
        assertTrue(missingMount.getMessage().contains("cameraMountConfig"));
        assertEquals(
                Arrays.asList("layout.ids", "layout.pose.7", "vision.mount"),
                events
        );
        assertEquals(0, hardwareMap.lookupCount);
        assertEquals(0, vision.sensorCount);

        events = new ArrayList<String>();
        hardwareMap = new EventHardwareMap(events);
        vision = new EventVisionLane(events);
        layout = new EventTagLayout(events);
        vision.sensor = null;
        EventHardwareMap sensorHardware = hardwareMap;
        EventVisionLane missingSensorVision = vision;
        EventTagLayout sensorLayout = layout;

        RuntimeException missingSensor = captureFailure(() ->
                new FtcOdometryAprilTagLocalizationLane(
                        sensorHardware,
                        missingSensorVision,
                        sensorLayout,
                        FtcOdometryAprilTagLocalizationLane.Config.defaults()
                ));
        assertTrue(missingSensor.getMessage().contains("tagSensor"));
        assertEquals(
                Arrays.asList(
                        "layout.ids",
                        "layout.pose.7",
                        "vision.mount",
                        "vision.sensor"
                ),
                events
        );
        assertEquals(0, hardwareMap.lookupCount);
    }

    @Test
    public void ownedPathCompletesLayoutAndVisionCaptureBeforePinpointLookup() {
        List<String> events = new ArrayList<String>();
        EventHardwareMap hardwareMap = new EventHardwareMap(events);
        EventVisionLane vision = new EventVisionLane(events);
        EventTagLayout layout = new EventTagLayout(events);

        captureFailure(() -> new FtcOdometryAprilTagLocalizationLane(
                hardwareMap,
                vision,
                layout,
                FtcOdometryAprilTagLocalizationLane.Config.defaults()
        ));

        assertEquals(
                Arrays.asList(
                        "layout.ids",
                        "layout.pose.7",
                        "vision.mount",
                        "vision.sensor",
                        "hardware.get"
                ),
                events
        );
        assertEquals(1, layout.idsCount);
        assertEquals(1, layout.poseCount);
        assertEquals(1, vision.mountCount);
        assertEquals(1, vision.sensorCount);
        assertEquals(1, hardwareMap.lookupCount);
    }

    @Test
    public void injectedPathSnapshotsLayoutAndVisionFactsExactlyOnce() {
        List<String> events = new ArrayList<String>();
        EventVisionLane vision = new EventVisionLane(events);
        EventTagLayout layout = new EventTagLayout(events);
        RecordingAprilTagSensor capturedSensor = vision.sensor;

        FtcOdometryAprilTagLocalizationLane lane =
                FtcOdometryAprilTagLocalizationLane.withPredictor(
                        new RecordingPredictor(),
                        vision,
                        layout,
                        FtcOdometryAprilTagLocalizationLane.EstimatorConfig.defaults()
                );

        assertEquals(1, layout.idsCount);
        assertEquals(1, layout.poseCount);
        assertEquals(1, vision.mountCount);
        assertEquals(1, vision.sensorCount);

        Pose3d capturedPose = layout.pose;
        layout.pose = new Pose3d(-80.0, 55.0, 4.0, 0.25, 0.0, 0.0);
        vision.sensor = new RecordingAprilTagSensor();
        ManualLoopClock clock = new ManualLoopClock(3.0);
        capturedSensor.next = AprilTagDetections.fromFrame(
                clock.clock().nowTimestamp(),
                Collections.singletonList(AprilTagObservation.target(7, Pose3d.zero()))
        );
        lane.update(clock.clock());

        assertEquals(1, layout.idsCount);
        assertEquals(1, layout.poseCount);
        assertEquals(1, vision.mountCount);
        assertEquals(1, vision.sensorCount);
        assertEquals(1, capturedSensor.getCount);
        assertEquals(0, vision.sensor.getCount);
        PoseEstimate estimate = lane.aprilTagPoseEstimator().getEstimate();
        assertTrue(estimate.hasPose);
        assertEquals(capturedPose, estimate.fieldToRobotPose);
    }

    @Test
    public void compositePublicApiKeepsOnlySemanticViewsAndDistinctConfigShapes()
            throws Exception {
        assertEquals(
                Arrays.asList("estimation", "predictor"),
                publicFieldNames(FtcOdometryAprilTagLocalizationLane.Config.class)
        );
        assertEquals(
                Arrays.asList("copy", "defaults", "validatedCopy"),
                publicDeclaredMethodNames(FtcOdometryAprilTagLocalizationLane.Config.class)
        );
        assertEquals(
                Arrays.asList("copy", "defaults", "toAprilTagPoseEstimatorConfig", "validatedCopy"),
                publicDeclaredMethodNames(
                        FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig.class
                )
        );
        assertEquals(
                Arrays.asList(
                        "aprilTags",
                        "correctedEstimatorMode",
                        "correctionEkf",
                        "correctionFusion",
                        "correctionSource"
                ),
                publicFieldNames(FtcOdometryAprilTagLocalizationLane.EstimatorConfig.class)
        );
        assertEquals(
                Arrays.asList("copy", "defaults"),
                publicDeclaredMethodNames(
                        FtcOdometryAprilTagLocalizationLane.EstimatorConfig.class
                )
        );

        Method aprilTagValidatedCopy =
                FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig.class.getMethod(
                        "validatedCopy",
                        String.class
                );
        assertSame(
                FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig.class,
                aprilTagValidatedCopy.getReturnType()
        );
        Method outerValidatedCopy =
                FtcOdometryAprilTagLocalizationLane.Config.class.getMethod(
                        "validatedCopy",
                        String.class
                );
        assertSame(
                FtcOdometryAprilTagLocalizationLane.Config.class,
                outerValidatedCopy.getReturnType()
        );

        Constructor<?>[] publicConstructors =
                FtcOdometryAprilTagLocalizationLane.class.getConstructors();
        assertEquals(1, publicConstructors.length);
        assertEquals(
                Arrays.asList(
                        HardwareMap.class,
                        AprilTagVisionLane.class,
                        TagLayout.class,
                        FtcOdometryAprilTagLocalizationLane.Config.class
                ),
                Arrays.asList(publicConstructors[0].getParameterTypes())
        );
        assertEquals(
                Arrays.asList(
                        "aprilTagPoseEstimator",
                        "correctionEstimator",
                        "debugDump",
                        "globalEstimator",
                        "limelightFieldPoseEstimator",
                        "predictor",
                        "update",
                        "withPredictor"
                ),
                publicDeclaredMethodNames(FtcOdometryAprilTagLocalizationLane.class)
        );

        Method injectedFactory = FtcOdometryAprilTagLocalizationLane.class.getMethod(
                "withPredictor",
                MotionPredictor.class,
                AprilTagVisionLane.class,
                TagLayout.class,
                FtcOdometryAprilTagLocalizationLane.EstimatorConfig.class
        );
        assertSame(FtcOdometryAprilTagLocalizationLane.class, injectedFactory.getReturnType());
        assertNoPublicMethod("config");
        assertNoPublicMethod("visionLane");
        assertNoPublicMethod("fixedFieldTagLayout");
        assertNoPublicMethod(
                "withPredictor",
                MotionPredictor.class,
                AprilTagVisionLane.class,
                TagLayout.class,
                FtcOdometryAprilTagLocalizationLane.Config.class
        );
    }

    @Test
    public void withPredictorRejectsMissingPredictorBeforeBuildingTheGraph() {
        try {
            FtcOdometryAprilTagLocalizationLane.withPredictor(null, null, null, null);
            fail("Expected missing predictor to fail fast");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("predictor"));
        }
    }

    @Test
    public void laneTraversesEachEstimatorModeOncePerCycleAndRefreshesNextCycle() {
        for (FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode mode
                : FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.values()) {
            RecordingPredictor predictor = new RecordingPredictor();
            RecordingVisionLane vision = new RecordingVisionLane();
            FtcOdometryAprilTagLocalizationLane.EstimatorConfig cfg =
                    FtcOdometryAprilTagLocalizationLane.EstimatorConfig.defaults();
            cfg.correctedEstimatorMode = mode;
            FtcOdometryAprilTagLocalizationLane lane =
                    FtcOdometryAprilTagLocalizationLane.withPredictor(
                            predictor,
                            vision,
                            new SimpleTagLayout(),
                            cfg
                    );
            ManualLoopClock time = new ManualLoopClock();

            lane.update(time.clock());
            PoseEstimate first = lane.globalEstimator().getEstimate();
            lane.update(time.clock());

            assertEquals(mode.name(), 1, predictor.updateCount);
            assertEquals(mode.name(), 1, vision.sensor.getCount);
            assertSame(mode.name(), first, lane.globalEstimator().getEstimate());

            time.nextCycle(0.02);
            lane.update(time.clock());

            assertEquals(mode.name(), 2, predictor.updateCount);
            assertEquals(mode.name(), 2, vision.sensor.getCount);
            assertFalse(mode.name(), first == lane.globalEstimator().getEstimate());
        }
    }

    @Test
    public void failedLaneCycleRethrowsTheSameFailureWithoutRetraversal() {
        RecordingPredictor predictor = new RecordingPredictor();
        RuntimeException failure = new IllegalStateException("predictor failed");
        predictor.failure = failure;
        RecordingVisionLane vision = new RecordingVisionLane();
        FtcOdometryAprilTagLocalizationLane lane = lane(predictor, vision);
        ManualLoopClock time = new ManualLoopClock();

        assertSame(failure, captureFailure(() -> lane.update(time.clock())));
        assertSame(failure, captureFailure(() -> lane.update(time.clock())));
        assertEquals(1, predictor.updateCount);
        assertEquals(0, vision.sensor.getCount);

        time.nextCycle(0.02);
        predictor.failure = null;
        lane.update(time.clock());

        assertEquals(2, predictor.updateCount);
        assertEquals(1, vision.sensor.getCount);
    }

    @Test
    public void recursiveLaneUpdateFailsFastAndIsRetainedForTheCycle() {
        RecordingPredictor predictor = new RecordingPredictor();
        RecordingVisionLane vision = new RecordingVisionLane();
        FtcOdometryAprilTagLocalizationLane lane = lane(predictor, vision);
        ManualLoopClock time = new ManualLoopClock();
        predictor.duringUpdate = () -> lane.update(time.clock());

        RuntimeException first = captureFailure(() -> lane.update(time.clock()));
        assertTrue(first instanceof IllegalStateException);
        assertTrue(first.getMessage().contains("reentered"));
        assertSame(first, captureFailure(() -> lane.update(time.clock())));
        assertEquals(1, predictor.updateCount);

        time.nextCycle(0.02);
        predictor.duringUpdate = null;
        lane.update(time.clock());
        assertEquals(2, predictor.updateCount);
    }

    @Test
    public void laneUpdateRequiresTheSharedLoopClock() {
        FtcOdometryAprilTagLocalizationLane lane = lane(
                new RecordingPredictor(),
                new RecordingVisionLane()
        );

        RuntimeException failure = captureFailure(() -> lane.update(null));

        assertTrue(failure instanceof NullPointerException);
        assertTrue(failure.getMessage().contains("clock"));
    }

    private static FtcOdometryAprilTagLocalizationLane lane(
            RecordingPredictor predictor,
            RecordingVisionLane vision
    ) {
        return FtcOdometryAprilTagLocalizationLane.withPredictor(
                predictor,
                vision,
                new SimpleTagLayout(),
                FtcOdometryAprilTagLocalizationLane.EstimatorConfig.defaults()
        );
    }

    private static List<String> publicFieldNames(Class<?> type) {
        ArrayList<String> names = new ArrayList<String>();
        for (Field field : type.getDeclaredFields()) {
            if (Modifier.isPublic(field.getModifiers())) {
                names.add(field.getName());
            }
        }
        Collections.sort(names);
        return names;
    }

    private static List<String> publicDeclaredMethodNames(Class<?> type) {
        List<String> names = new ArrayList<String>();
        for (Method method : type.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers()) && !method.isSynthetic()) {
                names.add(method.getName());
            }
        }
        Collections.sort(names);
        return names;
    }

    private static void assertNoPublicMethod(String name, Class<?>... parameterTypes) {
        try {
            FtcOdometryAprilTagLocalizationLane.class.getMethod(name, parameterTypes);
            fail("Expected public method to be absent: " + name);
        } catch (NoSuchMethodException expected) {
            // Required removed API.
        }
    }

    private static void assertOwnedPreflightFailsBeforeCollaborators(
            FtcOdometryAprilTagLocalizationLane.Config config,
            String expectedMessageFragment
    ) {
        List<String> events = new ArrayList<String>();
        EventHardwareMap hardwareMap = new EventHardwareMap(events);
        RuntimeException failure = captureFailure(() ->
                new FtcOdometryAprilTagLocalizationLane(
                        hardwareMap,
                        new EventVisionLane(events),
                        new EventTagLayout(events),
                        config
                ));

        assertTrue(failure.getMessage(),
                failure.getMessage() != null
                        && failure.getMessage().contains(expectedMessageFragment));
        assertTrue(events.isEmpty());
        assertEquals(0, hardwareMap.lookupCount);
    }

    private static RuntimeException captureFailure(Runnable action) {
        try {
            action.run();
            fail("Expected update failure");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static AprilTagVisionLane noDetectionsVisionLane() {
        return new RecordingVisionLane();
    }

    /** Hardware-map boundary that stops construction exactly when Pinpoint lookup begins. */
    private static final class EventHardwareMap extends HardwareMap {
        final List<String> events;
        int lookupCount;

        EventHardwareMap(List<String> events) {
            super(null, null);
            this.events = events;
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCount++;
            events.add("hardware.get");
            throw new IllegalStateException("stop at test Pinpoint lookup");
        }
    }

    /** Mutable authoring layout with observable source reads. */
    private static final class EventTagLayout implements TagLayout {
        final List<String> events;
        Pose3d pose = new Pose3d(24.0, 0.0, 6.0, Math.PI, 0.0, 0.0);
        int idsCount;
        int poseCount;

        EventTagLayout(List<String> events) {
            this.events = events;
        }

        @Override
        public Pose3d getFieldToTagPose(int id) {
            poseCount++;
            events.add("layout.pose." + id);
            return pose;
        }

        @Override
        public Set<Integer> ids() {
            idsCount++;
            events.add("layout.ids");
            return Collections.singleton(7);
        }
    }

    /** Completed vision collaborator with observable one-time accessor reads. */
    private static final class EventVisionLane implements AprilTagVisionLane {
        final List<String> events;
        RecordingAprilTagSensor sensor = new RecordingAprilTagSensor();
        CameraMountConfig mount = CameraMountConfig.identity();
        int mountCount;
        int sensorCount;

        EventVisionLane(List<String> events) {
            this.events = events;
        }

        @Override
        public AprilTagSensor tagSensor() {
            sensorCount++;
            events.add("vision.sensor");
            return sensor;
        }

        @Override
        public CameraMountConfig cameraMountConfig() {
            mountCount++;
            events.add("vision.mount");
            return mount;
        }

        @Override
        public VisionReadiness readiness(LoopClock clock) {
            return VisionReadiness.ready();
        }

        @Override
        public void close() {
            // No resources in this pure-JVM fixture.
        }
    }

    private static final class RecordingVisionLane implements AprilTagVisionLane {
        final RecordingAprilTagSensor sensor = new RecordingAprilTagSensor();

        @Override
        public AprilTagSensor tagSensor() {
            return sensor;
        }

        @Override
        public CameraMountConfig cameraMountConfig() {
            return CameraMountConfig.identity();
        }

        @Override
        public VisionReadiness readiness(LoopClock clock) {
            return VisionReadiness.ready();
        }

        @Override
        public void close() {
            // No resources in this pure-JVM fixture.
        }
    }

    private static final class RecordingAprilTagSensor implements AprilTagSensor {
        int getCount;
        AprilTagDetections next = AprilTagDetections.none();

        @Override
        public AprilTagDetections get(LoopClock clock) {
            getCount++;
            return next;
        }
    }

    private static final class RecordingPredictor implements MotionPredictor {
        int updateCount;
        RuntimeException failure;
        Runnable duringUpdate;
        private PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        private MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());

        @Override
        public void update(LoopClock clock) {
            updateCount++;
            if (duringUpdate != null) {
                duringUpdate.run();
            }
            if (failure != null) {
                throw failure;
            }
            LoopTimestamp timestamp = clock.nowTimestamp();
            estimate = new PoseEstimate(
                    new Pose3d(updateCount, 0.0, 0.0, 0.0, 0.0, 0.0),
                    true,
                    1.0,
                    timestamp
            );
            delta = MotionDelta.none(timestamp);
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }

        @Override
        public MotionDelta getLatestMotionDelta() {
            return delta;
        }
    }
}
