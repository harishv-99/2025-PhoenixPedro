package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.math.InterpolatingTable1D;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.localization.LimelightFieldPoseEstimator;
import edu.ftcphoenix.fw.spatial.AxisAlignedBoxRegion2d;
import edu.ftcphoenix.robots.phoenix.scoring.PhoenixScoring;
import edu.ftcphoenix.robots.phoenix.scoring.PhoenixTargeting;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks Phoenix's current-only profile shape, fresh ownership, and checked-in owner recipes. */
public final class PhoenixProfileConfigurationTest {

    @Test
    public void profileHasOnlyTheApprovedCurrentAssemblySurface() {
        Map<String, Class<?>> expectedFields = new LinkedHashMap<>();
        expectedFields.put("drive", FtcDrives.MecanumConfig.class);
        expectedFields.put("vision", PhoenixVisionFactory.Config.class);
        expectedFields.put("localization", FtcOdometryAprilTagLocalizationLane.Config.class);
        expectedFields.put("fixedAprilTagLayout", TagLayout.class);
        expectedFields.put("controls", PhoenixTeleOpControls.Config.class);
        expectedFields.put("driveAssist", PhoenixDriveAssistService.Config.class);
        expectedFields.put("scoring", PhoenixScoring.Config.class);
        expectedFields.put("calibration", PhoenixCalibrationConfig.class);
        expectedFields.put("targeting", PhoenixTargeting.Config.class);
        expectedFields.put("auto", PhoenixAutoConfig.class);

        Field[] fields = PhoenixProfile.class.getDeclaredFields();
        assertEquals(10, fields.length);
        for (Field field : fields) {
            assertTrue(Modifier.isPublic(field.getModifiers()));
            assertFalse(Modifier.isStatic(field.getModifiers()));
            assertEquals(expectedFields.remove(field.getName()), field.getType());
        }
        assertTrue(expectedFields.isEmpty());

        Method[] methods = PhoenixProfile.class.getDeclaredMethods();
        assertEquals(1, methods.length);
        assertEquals("current", methods[0].getName());
        assertEquals(PhoenixProfile.class, methods[0].getReturnType());
        assertEquals(0, methods[0].getParameterTypes().length);
        assertTrue(Modifier.isPublic(methods[0].getModifiers()));
        assertTrue(Modifier.isStatic(methods[0].getModifiers()));

        Constructor<?>[] constructors = PhoenixProfile.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertEquals(0, constructors[0].getParameterTypes().length);
        assertTrue(Modifier.isPrivate(constructors[0].getModifiers()));
        assertEquals(0, PhoenixProfile.class.getDeclaredClasses().length);
    }

    @Test
    public void ownerConfigsExposeOnlyTheirApprovedFactoriesAndQueries() {
        assertConfigSurface(
                PhoenixVisionFactory.Config.class,
                3,
                "defaults",
                "activeDeviceName",
                "activeCameraMount"
        );
        assertConfigSurface(PhoenixTeleOpControls.Config.class, 7, "defaults");
        assertConfigSurface(PhoenixDriveAssistService.Config.class, 4, "defaults");
        assertConfigSurface(PhoenixCalibrationConfig.class, 2, "defaults");
        assertPublicFields(
                PhoenixVisionFactory.Config.class,
                "backend",
                "webcam",
                "limelight"
        );
        assertPublicFields(
                PhoenixTeleOpControls.Config.class,
                "manualDrive",
                "slowTranslateScale",
                "maxAxialRatePerSec",
                "maxLateralRatePerSec",
                "maxOmegaRatePerSec",
                "slowOmegaScale",
                "selectedVelocityStepNative"
        );
        assertPublicFields(
                PhoenixDriveAssistService.Config.class,
                "shootBraceEnterTranslateMagnitude",
                "shootBraceExitTranslateMagnitude",
                "shootBraceTranslateKp",
                "shootBraceMaxTranslateCmd"
        );
        assertPublicFields(
                PhoenixCalibrationConfig.class,
                "pinpointAxesVerified",
                "pinpointPodOffsetsCalibrated"
        );
    }

    @Test
    public void everyCurrentCallReturnsAnIndependentMutableGraph() {
        PhoenixProfile first = PhoenixProfile.current();
        PhoenixProfile second = PhoenixProfile.current();

        assertNotSame(first, second);
        assertNotSame(first.drive, second.drive);
        assertNotSame(first.drive.wiring, second.drive.wiring);
        assertNotSame(first.drive.drivebase, second.drive.drivebase);
        assertNotSame(first.vision, second.vision);
        assertNotSame(first.vision.webcam, second.vision.webcam);
        assertNotSame(first.vision.limelight, second.vision.limelight);
        assertNotSame(first.localization, second.localization);
        assertNotSame(first.localization.predictor, second.localization.predictor);
        assertNotSame(first.localization.estimation, second.localization.estimation);
        assertNotSame(
                first.localization.estimation.aprilTags,
                second.localization.estimation.aprilTags
        );
        assertNotSame(
                first.localization.estimation.aprilTags.fieldPoseSolver,
                second.localization.estimation.aprilTags.fieldPoseSolver
        );
        assertNotSame(
                first.localization.estimation.correctionSource,
                second.localization.estimation.correctionSource
        );
        assertNotSame(
                first.localization.estimation.correctionSource.limelightFieldPose,
                second.localization.estimation.correctionSource.limelightFieldPose
        );
        assertNotSame(
                first.localization.estimation.correctionFusion,
                second.localization.estimation.correctionFusion
        );
        assertNotSame(
                first.localization.estimation.correctionEkf,
                second.localization.estimation.correctionEkf
        );
        assertNotSame(first.controls, second.controls);
        assertNotSame(first.controls.manualDrive, second.controls.manualDrive);
        assertNotSame(first.driveAssist, second.driveAssist);
        assertNotSame(first.scoring, second.scoring);
        assertNotSame(first.calibration, second.calibration);
        assertNotSame(first.targeting, second.targeting);
        assertNotSame(first.targeting.scoringTargets, second.targeting.scoringTargets);
        assertNotSame(first.targeting.defaultAimOffset, second.targeting.defaultAimOffset);
        assertNotSame(
                first.targeting.scoringTargets.get(20),
                second.targeting.scoringTargets.get(20)
        );
        assertNotSame(
                first.targeting.scoringTargets.get(20).aimOffset,
                second.targeting.scoringTargets.get(20).aimOffset
        );
        assertSame(first.targeting.shotVelocityTable, second.targeting.shotVelocityTable);
        assertNotSame(first.auto, second.auto);

        first.drive.wiring.frontLeftName = "changed drive";
        first.drive.drivebase.maxAxial = 0.1;
        first.vision.webcam.webcamName = "changed webcam";
        first.vision.limelight.hardwareName = "changed limelight";
        first.localization.predictor.hardwareMapName = "changed pinpoint";
        first.localization.estimation.aprilTags.maxDetectionAgeSec = 99.0;
        first.controls.manualDrive.deadband = 0.9;
        first.driveAssist.shootBraceTranslateKp = 99.0;
        first.scoring.nameMotorIntake = "changed intake";
        first.calibration.pinpointAxesVerified = false;
        first.targeting.defaultAimOffset.forwardInches = 99.0;
        first.targeting.scoringTargets.get(20).label = "changed target";
        first.targeting.scoringTargets.get(20).aimOffset.leftInches = 99.0;
        first.auto.routeTimeoutSec = 99.0;

        assertEquals("frontLeftMotor", second.drive.wiring.frontLeftName);
        assertEquals(1.0, second.drive.drivebase.maxAxial, 0.0);
        assertEquals("Webcam 1", second.vision.webcam.webcamName);
        assertEquals("limelight", second.vision.limelight.hardwareName);
        assertEquals("pinPoint", second.localization.predictor.hardwareMapName);
        assertEquals(0.50, second.localization.estimation.aprilTags.maxDetectionAgeSec, 0.0);
        assertEquals(0.05, second.controls.manualDrive.deadband, 0.0);
        assertEquals(0.08, second.driveAssist.shootBraceTranslateKp, 0.0);
        assertEquals("intakeMotor", second.scoring.nameMotorIntake);
        assertTrue(second.calibration.pinpointAxesVerified);
        assertEquals(0.0, second.targeting.defaultAimOffset.forwardInches, 0.0);
        assertEquals("Blue scoring target", second.targeting.scoringTargets.get(20).label);
        assertEquals(0.0, second.targeting.scoringTargets.get(20).aimOffset.leftInches, 0.0);
        assertEquals(4.0, second.auto.routeTimeoutSec, 0.0);
    }

    @Test
    public void currentDriveVisionControlsAssistAndCalibrationValuesStayExact() {
        PhoenixProfile profile = PhoenixProfile.current();

        assertEquals("frontLeftMotor", profile.drive.wiring.frontLeftName);
        assertEquals(Direction.FORWARD, profile.drive.wiring.frontLeftDirection);
        assertEquals("frontRightMotor", profile.drive.wiring.frontRightName);
        assertEquals(Direction.FORWARD, profile.drive.wiring.frontRightDirection);
        assertEquals("backLeftMotor", profile.drive.wiring.backLeftName);
        assertEquals(Direction.FORWARD, profile.drive.wiring.backLeftDirection);
        assertEquals("backRightMotor", profile.drive.wiring.backRightName);
        assertEquals(Direction.FORWARD, profile.drive.wiring.backRightDirection);
        assertTrue(profile.drive.enableZeroPowerBrake);
        assertRawDouble(1.0, profile.drive.drivebase.maxAxial);
        assertRawDouble(1.0, profile.drive.drivebase.maxLateral);
        assertRawDouble(1.0, profile.drive.drivebase.maxOmega);

        assertEquals(PhoenixVisionFactory.Backend.WEBCAM, profile.vision.backend);
        assertEquals("Webcam 1", profile.vision.webcam.webcamName);
        assertSame(
                PhoenixVisionFactory.Config.defaults().webcam.cameraResolution,
                profile.vision.webcam.cameraResolution
        );
        assertNull(profile.vision.webcam.tagLibrary);
        assertEquals("limelight", profile.vision.limelight.hardwareName);
        assertEquals(0, profile.vision.limelight.pipelineIndex);
        assertEquals(100, profile.vision.limelight.pollRateHz);
        assertRawDouble(0.25, profile.vision.limelight.maxResultAgeSec);
        assertMount(profile.vision.webcam.cameraMount);
        assertMount(profile.vision.limelight.cameraMount);

        assertEquals(
                new LinkedHashSet<Integer>(Arrays.asList(20, 24)),
                profile.fixedAprilTagLayout.ids()
        );
        assertPose3d(
                -58.37269973754883,
                -55.64250183105469,
                29.5,
                2.514145856049822,
                -0.0,
                -1.5707964163634494,
                profile.fixedAprilTagLayout.requireFieldToTagPose(20)
        );
        assertPose3d(
                -58.37269973754883,
                55.64250183105469,
                29.5,
                0.6274469027077515,
                -0.0,
                -1.5707964163634494,
                profile.fixedAprilTagLayout.requireFieldToTagPose(24)
        );

        assertRawDouble(0.05, profile.controls.manualDrive.deadband);
        assertRawDouble(1.5, profile.controls.manualDrive.translateExpo);
        assertRawDouble(1.5, profile.controls.manualDrive.rotateExpo);
        assertRawDouble(1.0, profile.controls.manualDrive.translateScale);
        assertRawDouble(1.0, profile.controls.manualDrive.rotateScale);
        assertRawDouble(0.35, profile.controls.slowTranslateScale);
        assertRawDouble(4.0, profile.controls.maxAxialRatePerSec);
        assertRawDouble(4.0, profile.controls.maxLateralRatePerSec);
        assertRawDouble(6.0, profile.controls.maxOmegaRatePerSec);
        assertRawDouble(0.20, profile.controls.slowOmegaScale);
        assertRawDouble(25.0, profile.controls.selectedVelocityStepNative);

        assertRawDouble(0.06, profile.driveAssist.shootBraceEnterTranslateMagnitude);
        assertRawDouble(0.10, profile.driveAssist.shootBraceExitTranslateMagnitude);
        assertRawDouble(0.08, profile.driveAssist.shootBraceTranslateKp);
        assertRawDouble(0.35, profile.driveAssist.shootBraceMaxTranslateCmd);

        assertTrue(profile.calibration.pinpointAxesVerified);
        assertFalse(profile.calibration.pinpointPodOffsetsCalibrated);
        PhoenixCalibrationConfig conservative = PhoenixCalibrationConfig.defaults();
        assertFalse(conservative.pinpointAxesVerified);
        assertFalse(conservative.pinpointPodOffsetsCalibrated);
    }

    @Test
    public void currentScoringTargetingAndAutoValuesStayExact() throws Exception {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixScoring.Config scoring = profile.scoring;

        assertEquals("intakeMotor", scoring.nameMotorIntake);
        assertEquals(Direction.FORWARD, scoring.directionMotorIntake);
        assertEquals("intakeTransfer", scoring.nameCrServoIntakeTransfer);
        assertEquals(Direction.REVERSE, scoring.directionCrServoIntakeTransfer);
        assertEquals("shooterTransferLeft", scoring.nameCrServoShooterTransferLeft);
        assertEquals(Direction.REVERSE, scoring.directionCrServoShooterTransferLeft);
        assertEquals("shooterTransferRight", scoring.nameCrServoShooterTransferRight);
        assertEquals(Direction.FORWARD, scoring.directionCrServoShooterTransferRight);
        assertRawDouble(0.65, scoring.shooterTransferLeftScale);
        assertEquals("shooterMotor", scoring.nameMotorShooterWheel);
        assertEquals(Direction.FORWARD, scoring.directionMotorShooterWheel);
        assertRawDouble(700.0, scoring.velocityMin);
        assertRawDouble(2000.0, scoring.velocityMax);
        assertRawDouble(50.0, scoring.velocityToleranceNative);
        assertFalse(scoring.applyFlywheelVelocityPIDF);
        assertRawDouble(0.0, scoring.flywheelVelKp);
        assertRawDouble(0.0, scoring.flywheelVelKi);
        assertRawDouble(0.0, scoring.flywheelVelKd);
        assertRawDouble(0.0, scoring.flywheelVelKf);
        assertRawDouble(50.0, scoring.velocityToleranceBelowNative);
        assertRawDouble(50.0, scoring.velocityToleranceAboveNative);
        assertRawDouble(0.10, scoring.readyPredictLeadSec);
        assertRawDouble(0.03, scoring.readyStableSec);
        assertRawDouble(1.0, scoring.intakeMotorPower);
        assertRawDouble(1.0, scoring.intakeTransferPower);
        assertRawDouble(0.5, scoring.intakeShooterTransferHoldBackPower);
        assertRawDouble(1.0, scoring.ejectMotorPower);
        assertRawDouble(1.0, scoring.ejectTransferPower);
        assertRawDouble(1.0, scoring.ejectShooterTransferPower);
        assertRawDouble(1.0, scoring.shootFeedPower);
        assertRawDouble(0.22, scoring.shootFeedPulseSec);
        assertRawDouble(0.06, scoring.shootFeedCooldownSec);
        assertRawDouble(1.0, scoring.feedScaleIntakeMotor);
        assertRawDouble(1.0, scoring.feedScaleIntakeTransfer);
        assertRawDouble(1.0, scoring.feedScaleShooterTransfer);

        PhoenixTargeting.Config targeting = profile.targeting;
        assertEquals(24, targeting.redAllianceScoringTagId);
        assertEquals(20, targeting.blueAllianceScoringTagId);
        assertEquals(
                new LinkedHashSet<Integer>(Arrays.asList(20, 24)),
                targeting.scoringTargets.keySet()
        );
        assertTarget(targeting.scoringTargets.get(20), "Blue scoring target", 0.0, 0.0);
        assertTarget(targeting.scoringTargets.get(24), "Red scoring target", 0.0, 0.0);
        assertRawDouble(0.25, targeting.aimToleranceDeg);
        assertRawDouble(1.5, targeting.aimKp);
        assertRawDouble(0.80, targeting.aimMaxOmegaCmd);
        assertRawDouble(0.50, targeting.aimReadyToleranceDeg);
        assertRawDouble(0.05, targeting.aimReadyDebounceSec);
        assertRawDouble(0.05, targeting.aimMinOmegaCmd);
        assertRawDouble(0.50, targeting.selectionMaxAgeSec);
        assertRawDouble(0.20, targeting.selectionReacquireSec);
        assertRawDouble(0.0, targeting.defaultAimOffset.forwardInches);
        assertRawDouble(0.0, targeting.defaultAimOffset.leftInches);
        assertShotTable(targeting.shotVelocityTable);

        assertRawDouble(25.0, profile.auto.parkTakeoverElapsedSec);
        assertRawDouble(4.0, profile.auto.routeTimeoutSec);
        assertRawDouble(2.0, profile.auto.aimHeadingToleranceDeg);
        assertRawDouble(1.75, profile.auto.aimTimeoutSec);
        assertRawDouble(0.75, profile.auto.aimMaxNoGuidanceSec);
        assertRawDouble(0.75, profile.auto.waitForTargetSec);
        assertRawDouble(2.5, profile.auto.waitForShotCompleteSec);
        assertRawDouble(12.0, profile.auto.pedroIntegrationTestDistanceIn);
    }

    @Test
    public void currentLocalizationRecipeRetainsEveryReviewedValue() {
        FtcOdometryAprilTagLocalizationLane.Config config = PhoenixProfile.current().localization;

        assertEquals(
                FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION,
                config.estimation.correctedEstimatorMode
        );
        assertEquals(
                FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.APRILTAG_POSE,
                config.estimation.correctionSource.mode
        );
        assertEquals("pinPoint", config.predictor.hardwareMapName);
        assertRawDouble(0.0, config.predictor.forwardPodOffsetLeftInches);
        assertRawDouble(0.0, config.predictor.strafePodOffsetForwardInches);
        assertEquals(
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD,
                config.predictor.forwardPodDirection
        );
        assertEquals(
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD,
                config.predictor.strafePodDirection
        );
        final com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods[] pod =
                {null};
        final boolean[] usedCustomResolution = {false};
        config.predictor.encoderResolution.applyTo(
                selected -> pod[0] = selected,
                ignored -> usedCustomResolution[0] = true
        );
        assertEquals(
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods
                        .goBILDA_4_BAR_POD,
                pod[0]
        );
        assertFalse(usedCustomResolution[0]);
        assertNull(config.predictor.yawScalar);
        assertRawDouble(0.75, config.predictor.quality);

        assertRawDouble(0.50, config.estimation.aprilTags.maxDetectionAgeSec);
        assertRawDouble(0.0, config.estimation.aprilTags.fieldPoseSolver.maxAbsBearingRad);
        assertTrue(config.estimation.aprilTags.fieldPoseSolver.preferObservationFieldPose);
        assertRawDouble(
                8.0,
                config.estimation.aprilTags.fieldPoseSolver.observationFieldPoseMaxDeltaInches
        );
        assertRawDouble(
                Math.toRadians(12.0),
                config.estimation.aprilTags.fieldPoseSolver
                        .observationFieldPoseMaxDeltaHeadingRad
        );
        assertRawDouble(36.0, config.estimation.aprilTags.fieldPoseSolver.rangeSoftnessInches);
        assertRawDouble(0.05, config.estimation.aprilTags.fieldPoseSolver.minObservationWeight);
        assertRawDouble(
                18.0,
                config.estimation.aprilTags.fieldPoseSolver.outlierPositionGateInches
        );
        assertRawDouble(
                Math.toRadians(25.0),
                config.estimation.aprilTags.fieldPoseSolver.outlierHeadingGateRad
        );
        assertRawDouble(
                6.0,
                config.estimation.aprilTags.fieldPoseSolver.consistencyPositionScaleInches
        );
        assertRawDouble(
                Math.toRadians(8.0),
                config.estimation.aprilTags.fieldPoseSolver.consistencyHeadingScaleRad
        );
        assertTrue(
                config.estimation.aprilTags.fieldPoseSolver.plausibleFieldRegion
                        instanceof AxisAlignedBoxRegion2d
        );
        AxisAlignedBoxRegion2d fieldRegion = (AxisAlignedBoxRegion2d)
                config.estimation.aprilTags.fieldPoseSolver.plausibleFieldRegion;
        assertRawDouble(-72.0, fieldRegion.minXInches);
        assertRawDouble(72.0, fieldRegion.maxXInches);
        assertRawDouble(-72.0, fieldRegion.minYInches);
        assertRawDouble(72.0, fieldRegion.maxYInches);
        assertRawDouble(
                3.0,
                config.estimation.aprilTags.fieldPoseSolver
                        .maxOutsidePlausibleFieldRegionInches
        );

        assertLimelightLocalization(config);
        assertFusionLocalization(config);
        assertEkfLocalization(config);
    }

    @Test
    public void visionQueriesInspectOnlyTheSelectedBackend() {
        PhoenixVisionFactory.Config webcam = PhoenixVisionFactory.Config.defaults();
        webcam.limelight = null;
        assertEquals("Webcam 1", webcam.activeDeviceName());
        assertNotNull(webcam.activeCameraMount());

        PhoenixVisionFactory.Config limelight = PhoenixVisionFactory.Config.defaults();
        limelight.backend = PhoenixVisionFactory.Backend.LIMELIGHT;
        limelight.webcam = null;
        assertEquals("limelight", limelight.activeDeviceName());
        assertNotNull(limelight.activeCameraMount());

        PhoenixVisionFactory.Config missingBackend = PhoenixVisionFactory.Config.defaults();
        missingBackend.backend = null;
        try {
            missingBackend.activeDeviceName();
            fail("Expected missing selected vision backend to fail");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("PhoenixVisionFactory.Config.backend"));
        }
    }

    @Test
    public void visionFactoryDelegatesOnlyTheSelectedDraftBeforeHardwareLookup() {
        PhoenixVisionFactory.Config webcam = PhoenixVisionFactory.Config.defaults();
        webcam.limelight = null;
        webcam.webcam.webcamName = " ";
        expectVisionCreateFailure(webcam, "webcamName");

        PhoenixVisionFactory.Config limelight = PhoenixVisionFactory.Config.defaults();
        limelight.backend = PhoenixVisionFactory.Backend.LIMELIGHT;
        limelight.webcam = null;
        limelight.limelight.hardwareName = " ";
        expectVisionCreateFailure(limelight, "hardwareName");
    }

    private static void assertConfigSurface(
            Class<?> type,
            int expectedPublicFieldCount,
            String... expectedPublicMethods
    ) {
        int publicFieldCount = 0;
        for (Field field : type.getDeclaredFields()) {
            if (Modifier.isPublic(field.getModifiers())) {
                publicFieldCount++;
            }
        }
        assertEquals(expectedPublicFieldCount, publicFieldCount);

        Set<String> actualMethods = new LinkedHashSet<>();
        int publicMethodCount = 0;
        for (Method method : type.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicMethodCount++;
                actualMethods.add(method.getName());
            }
        }
        assertEquals(expectedPublicMethods.length, publicMethodCount);
        assertEquals(new LinkedHashSet<>(Arrays.asList(expectedPublicMethods)), actualMethods);

        Constructor<?>[] constructors = type.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertEquals(0, constructors[0].getParameterTypes().length);
        assertTrue(Modifier.isPrivate(constructors[0].getModifiers()));
    }

    private static void assertPublicFields(Class<?> type, String... expectedNames) {
        Set<String> actualNames = new LinkedHashSet<>();
        for (Field field : type.getDeclaredFields()) {
            if (Modifier.isPublic(field.getModifiers())) {
                actualNames.add(field.getName());
            }
        }
        assertEquals(new LinkedHashSet<>(Arrays.asList(expectedNames)), actualNames);
    }

    private static void assertTarget(PhoenixTargeting.ScoringTarget target,
                                     String expectedLabel,
                                     double expectedForwardInches,
                                     double expectedLeftInches) {
        assertNotNull(target);
        assertEquals(expectedLabel, target.label);
        assertNotNull(target.aimOffset);
        assertRawDouble(expectedForwardInches, target.aimOffset.forwardInches);
        assertRawDouble(expectedLeftInches, target.aimOffset.leftInches);
    }

    private static void assertShotTable(InterpolatingTable1D table) throws Exception {
        double[] expectedXs = {
                28.06, 36.52, 42.3, 50.3, 56.5,
                62.9, 65.8, 70.0, 74.2, 79.5,
                83.4, 93.6, 96.6, 103.2, 104.7,
                109.2, 112.0, 115.0, 120.0, 130.0
        };
        double[] expectedYs = {
                1505.6, 1427.4, 1424.35, 1450.0, 1461.0,
                1538.0, 1535.7, 1530.0, 1575.0, 1600.0,
                1625.0, 1700.0, 1700.0, 1775.0, 1800.0,
                1800.0, 1818.0, 1825.0, 1850.0, 1875.0
        };

        Field xsField = InterpolatingTable1D.class.getDeclaredField("xs");
        Field ysField = InterpolatingTable1D.class.getDeclaredField("ys");
        xsField.setAccessible(true);
        ysField.setAccessible(true);
        double[] actualXs = (double[]) xsField.get(table);
        double[] actualYs = (double[]) ysField.get(table);

        assertEquals(expectedXs.length, actualXs.length);
        assertEquals(expectedYs.length, actualYs.length);
        for (int i = 0; i < expectedXs.length; i++) {
            assertRawDouble(expectedXs[i], actualXs[i]);
            assertRawDouble(expectedYs[i], actualYs[i]);
        }
    }

    private static void assertMount(edu.ftcphoenix.fw.sensing.vision.CameraMountConfig mount) {
        assertRawDouble(9.97, mount.xInches());
        assertRawDouble(-1.80, mount.yInches());
        assertRawDouble(13.68, mount.zInches());
        assertRawDouble(Math.toRadians(1.9), mount.yawRad());
        assertRawDouble(Math.toRadians(-18.2), mount.pitchRad());
        assertRawDouble(Math.toRadians(-1.7), mount.rollRad());
    }

    private static void assertPose3d(double expectedXInches,
                                     double expectedYInches,
                                     double expectedZInches,
                                     double expectedYawRad,
                                     double expectedPitchRad,
                                     double expectedRollRad,
                                     Pose3d actual) {
        assertRawDouble(expectedXInches, actual.xInches);
        assertRawDouble(expectedYInches, actual.yInches);
        assertRawDouble(expectedZInches, actual.zInches);
        assertRawDouble(expectedYawRad, actual.yawRad);
        assertRawDouble(expectedPitchRad, actual.pitchRad);
        assertRawDouble(expectedRollRad, actual.rollRad);
    }

    private static void assertRawDouble(double expected, double actual) {
        assertEquals(
                Double.doubleToRawLongBits(expected),
                Double.doubleToRawLongBits(actual)
        );
    }

    private static void expectVisionCreateFailure(
            PhoenixVisionFactory.Config config,
            String expectedSelectedField
    ) {
        try {
            PhoenixVisionFactory.create(new HardwareMap(null, null), config);
            fail("Expected invalid selected vision field " + expectedSelectedField);
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains(expectedSelectedField));
        }
    }

    private static void assertLimelightLocalization(
            FtcOdometryAprilTagLocalizationLane.Config config) {
        LimelightFieldPoseEstimator.Config limelight =
                config.estimation.correctionSource.limelightFieldPose;
        assertEquals(LimelightFieldPoseEstimator.Config.Mode.BOTPOSE, limelight.mode);
        assertRawDouble(0.25, limelight.maxResultAgeSec);
        assertEquals(1, limelight.minVisibleTags);
        assertRawDouble(0.55, limelight.singleTagQuality);
        assertRawDouble(0.85, limelight.multiTagQuality);
        assertTrue(limelight.degradeWhenMoving);
        assertRawDouble(72.0, limelight.translationSpeedForZeroQualityInPerSec);
        assertRawDouble(Math.toRadians(360.0), limelight.yawRateForZeroQualityRadPerSec);
        assertFalse(limelight.rejectWhenMovingTooFast);
        assertRawDouble(120.0, limelight.maxTranslationSpeedInPerSec);
        assertRawDouble(Math.toRadians(720.0), limelight.maxYawRateRadPerSec);
    }

    private static void assertFusionLocalization(
            FtcOdometryAprilTagLocalizationLane.Config config) {
        assertRawDouble(0.35, config.estimation.correctionFusion.maxCorrectionAgeSec);
        assertRawDouble(0.10, config.estimation.correctionFusion.minCorrectionQuality);
        assertRawDouble(0.25, config.estimation.correctionFusion.correctionPositionGain);
        assertRawDouble(0.35, config.estimation.correctionFusion.correctionHeadingGain);
        assertRawDouble(24.0, config.estimation.correctionFusion.maxCorrectionPositionJumpIn);
        assertRawDouble(
                Math.toRadians(60.0),
                config.estimation.correctionFusion.maxCorrectionHeadingJumpRad
        );
        assertTrue(config.estimation.correctionFusion.enableInitializeFromCorrection);
        assertTrue(config.estimation.correctionFusion.enablePushCorrectedPoseToPredictor);
        assertRawDouble(0.75, config.estimation.correctionFusion.correctionConfidenceHoldSec);
        assertTrue(config.estimation.correctionFusion.enableLatencyCompensation);
        assertRawDouble(1.0, config.estimation.correctionFusion.predictorHistorySec);
    }

    private static void assertEkfLocalization(
            FtcOdometryAprilTagLocalizationLane.Config config) {
        assertRawDouble(0.35, config.estimation.correctionEkf.maxCorrectionAgeSec);
        assertRawDouble(0.10, config.estimation.correctionEkf.minCorrectionQuality);
        assertRawDouble(24.0, config.estimation.correctionEkf.maxCorrectionPositionInnovationIn);
        assertRawDouble(
                Math.toRadians(60.0),
                config.estimation.correctionEkf.maxCorrectionHeadingInnovationRad
        );
        assertRawDouble(14.0, config.estimation.correctionEkf.maxCorrectionMahalanobisSq);
        assertTrue(config.estimation.correctionEkf.enableInitializeFromCorrection);
        assertTrue(config.estimation.correctionEkf.enablePushCorrectedPoseToPredictor);
        assertTrue(config.estimation.correctionEkf.enableLatencyCompensation);
        assertRawDouble(1.0, config.estimation.correctionEkf.predictorHistorySec);
        assertRawDouble(6.0, config.estimation.correctionEkf.initialPositionStdIn);
        assertRawDouble(
                Math.toRadians(12.0),
                config.estimation.correctionEkf.initialHeadingStdRad
        );
        assertRawDouble(3.0, config.estimation.correctionEkf.manualPosePositionStdIn);
        assertRawDouble(
                Math.toRadians(6.0),
                config.estimation.correctionEkf.manualPoseHeadingStdRad
        );
        assertRawDouble(
                0.20,
                config.estimation.correctionEkf.predictorProcessPositionStdFloorIn
        );
        assertRawDouble(
                0.03,
                config.estimation.correctionEkf.predictorProcessPositionStdPerIn
        );
        assertRawDouble(
                0.55,
                config.estimation.correctionEkf.predictorProcessPositionStdPerRad
        );
        assertRawDouble(
                Math.toRadians(0.35),
                config.estimation.correctionEkf.predictorProcessHeadingStdFloorRad
        );
        assertRawDouble(
                Math.toRadians(0.06),
                config.estimation.correctionEkf.predictorProcessHeadingStdPerIn
        );
        assertRawDouble(0.06, config.estimation.correctionEkf.predictorProcessHeadingStdPerRad);
        assertRawDouble(1.75, config.estimation.correctionEkf.correctionPositionStdFloorIn);
        assertRawDouble(6.0, config.estimation.correctionEkf.correctionPositionStdScaleIn);
        assertRawDouble(
                Math.toRadians(3.0),
                config.estimation.correctionEkf.correctionHeadingStdFloorRad
        );
        assertRawDouble(
                Math.toRadians(10.0),
                config.estimation.correctionEkf.correctionHeadingStdScaleRad
        );
        assertRawDouble(
                18.0,
                config.estimation.correctionEkf.projectedCorrectionPositionStdPerSec
        );
        assertRawDouble(
                Math.toRadians(30.0),
                config.estimation.correctionEkf.projectedCorrectionHeadingStdPerSec
        );
        assertRawDouble(24.0, config.estimation.correctionEkf.qualityPositionStdScaleIn);
        assertRawDouble(
                Math.toRadians(45.0),
                config.estimation.correctionEkf.qualityHeadingStdScaleRad
        );
    }
}
