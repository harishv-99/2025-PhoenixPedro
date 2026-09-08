package edu.ftcsushi.robots.examples.calibration;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcsushi.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig;
import edu.ftcsushi.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcsushi.fw.ftc.ui.MenuItem;
import edu.ftcsushi.fw.ftc.ui.SelectionMenu;
import edu.ftcsushi.fw.ftc.vision.AprilTagCameraFactory;
import edu.ftcsushi.fw.ftc.vision.FtcLimelightVisionLane;
import edu.ftcsushi.fw.ftc.vision.FtcWebcamVisionLane;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.tools.tester.TeleOpTester;
import edu.ftcsushi.fw.tools.tester.TesterSuite;
import edu.ftcsushi.fw.tools.tester.calibration.CameraMountCalibrator;
import edu.ftcsushi.fw.tools.tester.calibration.PinpointAxisDirectionTester;
import edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibrator;
import edu.ftcsushi.fw.tools.tester.localization.AprilTagLocalizationTester;
import edu.ftcsushi.fw.tools.tester.localization.PinpointAprilTagCorrectedLocalizationTester;

import static edu.ftcsushi.robots.examples.calibration.CalibrationRobotProfile.CameraBackend.LIMELIGHT;
import static edu.ftcsushi.robots.examples.calibration.CalibrationRobotProfile.CameraBackend.NONE;
import static edu.ftcsushi.robots.examples.calibration.CalibrationRobotProfile.CameraBackend.WEBCAM;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Maintainer-only configuration and ownership evidence for the independent calibration example.
 * The actual profile, factories, tester constructors, and menu suppliers remain real; narrow
 * reflection observes their captured configuration and deferred templates without adding a
 * production testing API. No tester is initialized and no hardware, sensor readings, or physical
 * success is invented. Review booleans are synthetic inputs to software gates, not safety evidence.
 *
 * <p>The public webcam factory's Android Size-dependent active capture remains covered by the
 * framework vision-boundary tests using their existing ResolutionReader seam. This suite inspects
 * the real example's captured webcam template before that boundary; Limelight deferred capture
 * can also be exercised directly without hardware.</p>
 */
public final class CalibrationTestersTest {

    @Test
    public void freshProfileDefaultsKeepEveryOptionalCapabilityUnselected() throws Exception {
        CalibrationRobotProfile first = CalibrationRobotProfile.current();
        CalibrationRobotProfile second = CalibrationRobotProfile.current();
        assertNotSame(first, second);
        assertEquals(NONE, first.cameraBackend);
        assertFalse(first.poweredMotionReviewed);
        assertFalse(first.cameraMountAccepted);
        assertFalse(first.pinpointAxesVerified);
        assertFalse(first.pinpointOffsetsVerified);
        first.cameraBackend = LIMELIGHT;
        first.poweredMotionReviewed = true;
        first.pinpointHardwareName = "mutated";
        assertEquals(NONE, second.cameraBackend);
        assertFalse(second.poweredMotionReviewed);
        assertFalse("mutated".equals(second.pinpointHardwareName));
        assertTrue(CalibrationTestersOpMode.class.isAnnotationPresent(Disabled.class));
    }

    @Test
    public void canonicalNondefaultPinpointFactsReachFreshRawAndCorrectedDrafts() {
        CalibrationRobotProfile profile = authoredProfile();
        PinpointOdometryPredictor.Config first = profile.pinpoint();
        PinpointOdometryPredictor.Config second = profile.pinpoint();
        assertNotSame(first, second);
        assertAuthoredPinpoint(first);
        assertAuthoredPinpoint(second);
        FtcOdometryAprilTagLocalizationLane.Config localization = profile.localization();
        assertAuthoredPinpoint(localization.predictor);
        assertNotSame(first, localization.predictor);
        assertEquals(FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.APRILTAG_POSE,
                localization.estimation.correctionSource.mode);
        assertEquals("FUSION", localization.estimation.correctedEstimatorMode.name());
        assertEquals(0.19, localization.estimation.aprilTags.maxDetectionAgeSec, 0.0);
        assertEquals(11.5, localization.estimation.aprilTags.fieldPoseSolver.outlierPositionGateInches, 0.0);
        assertEquals(0.27, localization.estimation.aprilTags.fieldPoseSolver.outlierHeadingGateRad, 0.0);
        first.hardwareMapName = "corrupted draft";
        localization.predictor.quality = 0.99;
        localization.estimation.aprilTags.maxDetectionAgeSec = 9.0;
        assertAuthoredPinpoint(profile.pinpoint());
        assertEquals(0.19, profile.localization().estimation.aprilTags.maxDetectionAgeSec, 0.0);
    }

    @Test
    public void defaultSuiteContainsOnlyFreshInactiveAxisAndManualPodOwners() throws Exception {
        CalibrationRobotProfile profile = authoredProfile();
        TesterSuite suite = CalibrationTesters.create(profile);
        List<MenuItem<Supplier<TeleOpTester>>> entries = items(suite);
        assertEquals(Arrays.asList("Pinpoint axis directions", "Manual pod offsets"), labels(entries));
        assertNull(field(suite, "ctx"));
        for (MenuItem<Supplier<TeleOpTester>> entry : entries) {
            TeleOpTester first = entry.value.get();
            TeleOpTester second = entry.value.get();
            assertNotSame(first, second);
            assertNull(field(first, "ctx"));
            assertNull(field(second, "ctx"));
            assertNull(field(first, "pinpoint"));
            assertNull(field(second, "pinpoint"));
            if (first instanceof PinpointPodOffsetCalibrator) {
                assertManual((PinpointPodOffsetCalibrator) first);
                assertManual((PinpointPodOffsetCalibrator) second);
            } else {
                assertTrue(first instanceof PinpointAxisDirectionTester);
                assertAuthoredPinpoint(((PinpointAxisDirectionTester.Config) field(first, "cfg")).pinpoint);
            }
        }
    }

    @Test
    public void inactiveCameraAndDriveFactsDoNotLeakIntoTheBasicSuite() throws Exception {
        CalibrationRobotProfile profile = authoredProfile();
        profile.cameraMount = null;
        profile.maxDetectionAgeSec = Double.NaN;
        profile.limelightPipelineIndex = -1;
        profile.limelightPollRateHz = -1;
        profile.autoOmegaCmd = Double.NaN;
        profile.automaticPhaseTimeoutSec = Double.NaN;
        for (MenuItem<Supplier<TeleOpTester>> entry : items(CalibrationTesters.create(profile))) {
            TeleOpTester owner = entry.value.get();
            if (owner instanceof PinpointPodOffsetCalibrator) assertManual((PinpointPodOffsetCalibrator) owner);
            else assertTrue(owner instanceof PinpointAxisDirectionTester);
        }
    }

    @Test
    public void menuSuppliersCaptureProfileOnceWhileEverySelectionOwnsAFreshConfig() throws Exception {
        CalibrationRobotProfile profile = authoredProfile();
        List<MenuItem<Supplier<TeleOpTester>>> entries = items(CalibrationTesters.create(profile));
        profile.pinpointHardwareName = "later robot";
        profile.forwardPodOffsetLeftInches = 99.0;
        profile.odometryQuality = 0.01;
        profile.cameraBackend = LIMELIGHT;
        profile.poweredMotionReviewed = true;
        assertEquals(2, entries.size());
        PinpointAxisDirectionTester first = (PinpointAxisDirectionTester) entries.get(0).value.get();
        PinpointAxisDirectionTester second = (PinpointAxisDirectionTester) entries.get(0).value.get();
        PinpointAxisDirectionTester.Config firstConfig = (PinpointAxisDirectionTester.Config) field(first, "cfg");
        PinpointAxisDirectionTester.Config secondConfig = (PinpointAxisDirectionTester.Config) field(second, "cfg");
        assertNotSame(firstConfig, secondConfig);
        assertNotSame(firstConfig.pinpoint, secondConfig.pinpoint);
        assertAuthoredPinpoint(firstConfig.pinpoint);
        assertAuthoredPinpoint(secondConfig.pinpoint);
        // Mutating an inspected draft here is a maintainer-only isolation probe, not robot usage.
        firstConfig.pinpoint.hardwareMapName = "one inspected owner";
        assertAuthoredPinpoint(secondConfig.pinpoint);
        assertAuthoredPinpoint(((PinpointAxisDirectionTester.Config)
                field(entries.get(0).value.get(), "cfg")).pinpoint);
    }

    @Test
    public void rawCameraDraftsEnableTagsAndKeepBackendFactsAndMountSeparate() {
        CalibrationRobotProfile profile = authoredProfile();
        FtcWebcamVisionLane.Config webcam = profile.webcam();
        FtcLimelightVisionLane.Config limelight = profile.limelight();
        assertEquals("authored webcam", webcam.webcamName);
        assertEquals("authored limelight", limelight.hardwareName);
        assertSame(profile.cameraMount, webcam.cameraMount);
        assertSame(profile.cameraMount, limelight.cameraMount);
        assertNotNull(webcam.aprilTags);
        assertNotNull(limelight.aprilTags);
        assertNull(webcam.floorObjects);
        assertNull(limelight.floorObjects);
        assertEquals(4, limelight.pipelineIndex);
        assertEquals(4, limelight.aprilTags.pipelineIndex);
        assertEquals(73, limelight.pollRateHz);
        assertEquals(0.17, limelight.maxResultAgeSec, 0.0);
        assertEquals(0.19, profile.aprilTags().maxDetectionAgeSec, 0.0);
        assertNotSame(webcam, profile.webcam());
        assertNotSame(webcam.aprilTags, profile.webcam().aprilTags);
        assertNotSame(limelight, profile.limelight());
        assertNotSame(limelight.aprilTags, profile.limelight().aprilTags);
        limelight.aprilTags.pipelineIndex = 8;
        assertEquals(4, profile.limelight().aprilTags.pipelineIndex);
    }

    @Test
    public void optionalVisionOwnersUseTypedPickersAndOnlyTheSelectedBackendTemplate() throws Exception {
        for (CalibrationRobotProfile.CameraBackend backend : Arrays.asList(WEBCAM, LIMELIGHT)) {
            CalibrationRobotProfile profile = authoredProfile();
            profile.cameraBackend = backend;
            if (backend == WEBCAM) {
                profile.webcamName = null;
                // Invalid dormant peer settings must not activate the Limelight configuration.
                profile.limelightPipelineIndex = -1;
                profile.limelightPollRateHz = -1;
                profile.limelightMaxResultAgeSec = Double.NaN;
            } else {
                profile.limelightName = null;
                profile.webcamName = " ";
            }
            for (TeleOpTester owner : visionOwners(profile)) {
                assertNull(field(owner, "preferredVisionDeviceName"));
                assertEquals(backend == WEBCAM ? WebcamName.class : Limelight3A.class,
                        field(owner, "visionDeviceType"));
                assertEquals("Select " + backend, field(owner, "visionPickerTitle"));
                assertColdVision(owner);
                Object builder = field(owner, "visionLaneFactoryBuilder");
                if (backend == WEBCAM) {
                    FtcWebcamVisionLane.Config template = captured(builder, FtcWebcamVisionLane.Config.class);
                    assertNull(template.webcamName);
                    assertSame(profile.cameraMount, template.cameraMount);
                    assertNotNull(template.aprilTags);
                    assertNull(template.floorObjects);
                } else {
                    FtcLimelightVisionLane.Config template = captured(builder, FtcLimelightVisionLane.Config.class);
                    assertNull(template.hardwareName);
                    assertAuthoredLimelight(template, profile.cameraMount);
                }
            }
        }
    }

    @Test
    public void optionalOwnersShareFixedLandmarkPolicyAgeAndCanonicalCorrectedPredictor() throws Exception {
        CalibrationRobotProfile profile = authoredProfile();
        profile.cameraBackend = LIMELIGHT;
        CameraMountCalibrator mount = CalibrationTesters.cameraMount(profile);
        AprilTagLocalizationTester tags = CalibrationTesters.aprilTagLocalization(profile);
        PinpointAprilTagCorrectedLocalizationTester corrected = CalibrationTesters.correctedLocalization(profile);
        assertEquals(0.19, (double) field(mount, "maxDetectionAgeSec"), 0.0);
        assertAuthoredAprilTags((AprilTagLocalizationConfig) field(tags, "aprilTags"));
        FtcOdometryAprilTagLocalizationLane.Config localization =
                (FtcOdometryAprilTagLocalizationLane.Config) field(corrected, "localizationConfig");
        assertAuthoredPinpoint(localization.predictor);
        assertAuthoredAprilTags(localization.estimation.aprilTags);
        assertEquals(FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.APRILTAG_POSE,
                localization.estimation.correctionSource.mode);
        assertEquals("FUSION", localization.estimation.correctedEstimatorMode.name());
        assertFixedLandmarks((TagLayout) field(mount, "layout"));
        assertFixedLandmarks((TagLayout) field(tags, "layout"));
        assertFixedLandmarks((TagLayout) field(corrected, "fixedTagLayout"));
        for (TeleOpTester owner : Arrays.asList(mount, tags, corrected)) {
            assertEquals("authored limelight", field(owner, "preferredVisionDeviceName"));
            assertColdVision(owner);
        }
        profile.pinpointHardwareName = "later predictor";
        profile.maxDetectionAgeSec = 9;
        assertAuthoredPinpoint(localization.predictor);
        assertAuthoredAprilTags(localization.estimation.aprilTags);
    }

    @Test
    public void limelightPickerReplacementCapturesFreshFactoriesWithoutRereadingProfile() throws Exception {
        CalibrationRobotProfile profile = authoredProfile();
        profile.cameraBackend = LIMELIGHT;
        profile.limelightName = null;
        CameraMountConfig acceptedTemplateMount = profile.cameraMount;
        for (TeleOpTester owner : visionOwners(profile)) {
            Function<String, AprilTagCameraFactory> builder = visionBuilder(owner);
            AprilTagCameraFactory first = builder.apply("selected camera one");
            AprilTagCameraFactory second = builder.apply("selected camera two");
            assertNotSame(first, second);
            assertEquals("limelight: selected camera one", first.description());
            assertEquals("limelight: selected camera two", second.description());
            FtcLimelightVisionLane.Config firstConfig = captured(first, FtcLimelightVisionLane.Config.class);
            FtcLimelightVisionLane.Config secondConfig = captured(second, FtcLimelightVisionLane.Config.class);
            assertNotSame(firstConfig, secondConfig);
            assertNotSame(firstConfig.aprilTags, secondConfig.aprilTags);
            assertAuthoredLimelight(firstConfig, acceptedTemplateMount);
            assertAuthoredLimelight(secondConfig, acceptedTemplateMount);
            profile.cameraMount = CameraMountConfig.identity();
            profile.limelightPipelineIndex = 8;
            firstConfig.aprilTags.pipelineIndex = 9;
            FtcLimelightVisionLane.Config thirdConfig = captured(
                    builder.apply("selected camera three"), FtcLimelightVisionLane.Config.class);
            assertAuthoredLimelight(thirdConfig, acceptedTemplateMount);
            assertEquals("selected camera three", thirdConfig.hardwareName);
            assertColdVision(owner);
        }
    }

    @Test
    public void webcamTemplatesAreCapturedBeforeProfileMutationWithoutPretendingToOpenAndroidCamera() throws Exception {
        CalibrationRobotProfile profile = authoredProfile();
        profile.cameraBackend = WEBCAM;
        profile.webcamName = null;
        CameraMountConfig mount = profile.cameraMount;
        List<TeleOpTester> owners = visionOwners(profile);
        profile.webcamName = "later camera";
        profile.cameraMount = CameraMountConfig.identity();
        Object previousTemplate = null;
        for (TeleOpTester owner : owners) {
            FtcWebcamVisionLane.Config template = captured(
                    field(owner, "visionLaneFactoryBuilder"), FtcWebcamVisionLane.Config.class);
            assertNotSame(previousTemplate, template);
            previousTemplate = template;
            assertNull(template.webcamName);
            assertSame(mount, template.cameraMount);
            assertNotNull(template.aprilTags);
            assertEquals(WebcamName.class, field(owner, "visionDeviceType"));
            assertColdVision(owner);
        }
    }

    @Test
    public void directPoweredCallsRejectUnreviewedMotionBeforeAnyActiveConfigValidation() {
        CalibrationRobotProfile profile = authoredProfile();
        profile.pinpointHardwareName = null;
        profile.frontLeftMotorName = null;
        profile.automaticPhaseTimeoutSec = Double.NaN;
        profile.cameraMount = null;
        expectGate("poweredMotionReviewed", () -> CalibrationTesters.poweredPodOffsets(profile));
        expectGate("poweredMotionReviewed", () -> CalibrationTesters.assistedPodOffsets(profile));
    }

    @Test
    public void assistedCallsRejectMissingSelectionUnacceptedAndPlaceholderMountsWithoutFallback() {
        CalibrationRobotProfile profile = authoredProfile();
        profile.poweredMotionReviewed = true;
        // A poisoned active drive field makes gate precedence observable before owner capture.
        profile.frontLeftMotorName = null;
        expectGate("cameraBackend", () -> CalibrationTesters.assistedPodOffsets(profile));
        profile.cameraBackend = LIMELIGHT;
        expectGate("cameraMountAccepted", () -> CalibrationTesters.assistedPodOffsets(profile));
        profile.cameraMountAccepted = true;
        for (CameraMountConfig unsuitable : Arrays.asList(null, CameraMountConfig.identity(),
                CameraMountConfig.of(1e-8, 0, 0, 0, 0, 0))) {
            profile.cameraMount = unsuitable;
            expectGate("non-identity", () -> CalibrationTesters.assistedPodOffsets(profile));
        }
    }

    @Test
    public void directVisionCallsRejectNoneEvenWhenDormantCameraFactsAreInvalid() {
        CalibrationRobotProfile profile = authoredProfile();
        profile.cameraMount = null;
        profile.maxDetectionAgeSec = Double.NaN;
        expectGate("cameraBackend", () -> CalibrationTesters.cameraMount(profile));
        expectGate("cameraBackend", () -> CalibrationTesters.aprilTagLocalization(profile));
        expectGate("cameraBackend", () -> CalibrationTesters.correctedLocalization(profile));
    }

    @Test
    public void reviewedPoweredOwnerCapturesAllMotorAndMotionFactsButNoCamera() throws Exception {
        CalibrationRobotProfile profile = authoredPoweredProfile();
        profile.cameraBackend = LIMELIGHT;
        profile.cameraMount = null;
        profile.limelightPollRateHz = -1;
        PinpointPodOffsetCalibrator first = CalibrationTesters.poweredPodOffsets(profile);
        PinpointPodOffsetCalibrator second = CalibrationTesters.poweredPodOffsets(profile);
        PinpointPodOffsetCalibrator.Config firstConfig = podConfig(first);
        PinpointPodOffsetCalibrator.Config secondConfig = podConfig(second);
        assertPowered(first);
        assertPowered(second);
        assertNotSame(firstConfig, secondConfig);
        assertNotSame(firstConfig.pinpoint, secondConfig.pinpoint);
        assertNotSame(firstConfig.mecanum, secondConfig.mecanum);
        assertNotSame(firstConfig.mecanum.wiring, secondConfig.mecanum.wiring);
        assertNull(field(first, "visionLaneFactoryBuilder"));
        firstConfig.mecanum.wiring.frontLeftName = "one inspected owner";
        profile.autoOmegaCmd = 0.9;
        assertPowered(second);
        assertManual(CalibrationTesters.manualPodOffsets(profile));
    }

    @Test
    public void assistedOwnerConfigurationKeepsSearchesOffForEitherBackend() throws Exception {
        for (CalibrationRobotProfile.CameraBackend backend : Arrays.asList(WEBCAM, LIMELIGHT)) {
            CalibrationRobotProfile profile = authoredPoweredProfile();
            profile.cameraBackend = backend;
            profile.cameraMountAccepted = true;
            // Picker type/title are active only without a preferred name in this owner.
            profile.webcamName = null;
            profile.limelightName = null;
            PinpointPodOffsetCalibrator owner = CalibrationTesters.assistedPodOffsets(profile);
            assertPowered(owner);
            assertNotNull(field(owner, "visionLaneFactoryBuilder"));
            PinpointPodOffsetCalibrator.Config cfg = podConfig(owner);
            assertAuthoredAprilTags(cfg.aprilTags);
            assertFixedLandmarks((TagLayout) field(owner, "layout"));
            assertEquals(backend == WEBCAM ? WebcamName.class : Limelight3A.class, cfg.visionDeviceType);
            assertEquals("Select " + backend, cfg.visionPickerTitle);
            assertFalse(cfg.enableAutoTagSearchAtStart);
            assertFalse(cfg.enableAutoTagSearchAtEnd);
        }
    }

    @Test
    public void optionalMenusFreezePermissionsAndKeepAssistedAndUnassistedLabelsSeparate() throws Exception {
        CalibrationRobotProfile profile = authoredPoweredProfile();
        profile.cameraBackend = LIMELIGHT;
        profile.cameraMountAccepted = true;
        List<MenuItem<Supplier<TeleOpTester>>> entries = items(CalibrationTesters.create(profile));
        assertEquals(Arrays.asList("Pinpoint axis directions", "Manual pod offsets",
                "Vision checks", "Powered pod offsets"), labels(entries));
        assertTrue(entries.get(2).help.contains("LIMELIGHT"));
        assertTrue(entries.get(3).help.contains("FTC STOP"));
        profile.cameraBackend = NONE;
        profile.poweredMotionReviewed = false;
        profile.cameraMountAccepted = false;
        profile.cameraMount = CameraMountConfig.identity();
        profile.pinpointHardwareName = "later robot";
        List<MenuItem<Supplier<TeleOpTester>>> powered = items((TesterSuite) entries.get(3).value.get());
        assertEquals(Arrays.asList("Unassisted powered pod offsets", "Assisted powered pod offsets"),
                labels(powered));
        assertTrue(powered.get(0).help.contains("no camera"));
        assertTrue(powered.get(1).help.contains("no tag search"));
        PinpointPodOffsetCalibrator unassisted = (PinpointPodOffsetCalibrator) powered.get(0).value.get();
        PinpointPodOffsetCalibrator assisted = (PinpointPodOffsetCalibrator) powered.get(1).value.get();
        assertPowered(unassisted);
        assertPowered(assisted);
        assertNull(field(unassisted, "visionLaneFactoryBuilder"));
        assertNotNull(field(assisted, "visionLaneFactoryBuilder"));
        assertNotSame(assisted, powered.get(1).value.get());
        List<MenuItem<Supplier<TeleOpTester>>> vision = items((TesterSuite) entries.get(2).value.get());
        assertEquals(Arrays.asList("Measure camera mount", "AprilTag localization", "Pinpoint + AprilTag fusion"),
                labels(vision));
        assertColdVision(vision.get(2).value.get());
    }

    @Test
    public void mountHeuristicOrAcknowledgementAloneNeverAddsAssistedMenuEntry() throws Exception {
        for (boolean accepted : Arrays.asList(false, true)) {
            CalibrationRobotProfile profile = authoredPoweredProfile();
            profile.cameraBackend = LIMELIGHT;
            profile.cameraMountAccepted = accepted;
            if (accepted) profile.cameraMount = CameraMountConfig.identity();
            List<MenuItem<Supplier<TeleOpTester>>> root = items(CalibrationTesters.create(profile));
            TesterSuite powered = (TesterSuite) root.get(3).value.get();
            assertEquals(Arrays.asList("Unassisted powered pod offsets"), labels(items(powered)));
        }
    }

    @Test
    public void guidedStatusIsFrozenHumanOrHeuristicInputAndDoesNotAuthorizeMotion() throws Exception {
        CalibrationRobotProfile profile = authoredProfile();
        profile.cameraBackend = LIMELIGHT;
        List<MenuItem<Supplier<TeleOpTester>>> entries = items(CalibrationTesters.guidedWalkthrough(profile));
        assertEquals(5, entries.size());
        assertEquals("TODO", entries.get(0).tag);
        assertEquals("OK", entries.get(1).tag);
        assertTrue(entries.get(1).help.contains("not physical evidence"));
        assertTrue(entries.get(1).help.contains("non-default"));
        assertEquals("OK", entries.get(3).tag);
        assertTrue(entries.get(3).help.contains("not acceptance"));
        profile.pinpointAxesVerified = true;
        profile.forwardPodOffsetLeftInches = 0;
        profile.strafePodOffsetForwardInches = 0;
        profile.cameraMount = CameraMountConfig.identity();
        assertEquals("TODO", entries.get(0).tag);
        assertEquals("OK", entries.get(1).tag);
        assertEquals("OK", entries.get(3).tag);
        PinpointAxisDirectionTester axis = (PinpointAxisDirectionTester) entries.get(0).value.get();
        assertAuthoredPinpoint(((PinpointAxisDirectionTester.Config) field(axis, "cfg")).pinpoint);
        assertFalse(profile.cameraMountAccepted);
        assertFalse(profile.pinpointOffsetsVerified);
        assertFalse(profile.poweredMotionReviewed);
        expectGate("poweredMotionReviewed", () -> CalibrationTesters.poweredPodOffsets(profile));
        List<MenuItem<Supplier<TeleOpTester>>> rebuilt = items(CalibrationTesters.guidedWalkthrough(profile));
        assertEquals("OK", rebuilt.get(0).tag);
        assertEquals("TODO", rebuilt.get(1).tag);
        assertEquals("TODO", rebuilt.get(3).tag);
    }

    private static CalibrationRobotProfile authoredProfile() {
        CalibrationRobotProfile profile = CalibrationRobotProfile.current();
        profile.pinpointHardwareName = "authored pinpoint";
        profile.forwardPodOffsetLeftInches = -4.25;
        profile.strafePodOffsetForwardInches = 6.5;
        profile.encoderResolution = PinpointOdometryPredictor.EncoderResolution.ticksPerInch(543.21);
        profile.forwardPodDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        profile.strafePodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        profile.yawScalar = 1.013;
        profile.odometryQuality = 0.61;
        profile.webcamName = "authored webcam";
        profile.limelightName = "authored limelight";
        profile.cameraMount = CameraMountConfig.of(3.0, -1.0, 8.0, 0.11, -0.20, 0.03);
        profile.maxDetectionAgeSec = 0.19;
        profile.tagOutlierPositionGateInches = 11.5;
        profile.tagOutlierHeadingGateRad = 0.27;
        profile.limelightPipelineIndex = 4;
        profile.limelightPollRateHz = 73;
        profile.limelightMaxResultAgeSec = 0.17;
        return profile;
    }

    private static void assertAuthoredPinpoint(PinpointOdometryPredictor.Config config) {
        assertEquals("authored pinpoint", config.hardwareMapName);
        assertEquals(-4.25, config.forwardPodOffsetLeftInches, 0.0);
        assertEquals(6.5, config.strafePodOffsetForwardInches, 0.0);
        assertEquals(GoBildaPinpointDriver.EncoderDirection.REVERSED, config.forwardPodDirection);
        assertEquals(GoBildaPinpointDriver.EncoderDirection.FORWARD, config.strafePodDirection);
        assertEquals(1.013, config.yawScalar, 0.0);
        assertEquals(0.61, config.quality, 0.0);
        config.encoderResolution.applyTo(ignored -> fail("expected authored custom resolution"),
                ticks -> assertEquals(543.21, ticks, 0.0));
    }

    private static CalibrationRobotProfile authoredPoweredProfile() {
        CalibrationRobotProfile profile = authoredProfile();
        profile.poweredMotionReviewed = true;
        profile.frontLeftMotorName = "cal fl";
        profile.frontRightMotorName = "cal fr";
        profile.backLeftMotorName = "cal bl";
        profile.backRightMotorName = "cal br";
        profile.frontLeftMotorDirection = Direction.REVERSE;
        profile.frontRightMotorDirection = Direction.FORWARD;
        profile.backLeftMotorDirection = Direction.FORWARD;
        profile.backRightMotorDirection = Direction.REVERSE;
        profile.enableZeroPowerBrake = false;
        profile.manualOmegaScale = 0.13;
        profile.autoOmegaCmd = 0.18;
        profile.targetTurnRad = -2.4;
        profile.automaticPhaseTimeoutSec = 7.5;
        profile.recenterTranslationScale = 0.14;
        return profile;
    }

    private static void assertPowered(PinpointPodOffsetCalibrator owner) throws Exception {
        PinpointPodOffsetCalibrator.Config cfg = podConfig(owner);
        assertAuthoredPinpoint(cfg.pinpoint);
        FtcDrives.MecanumWiringConfig wiring = cfg.mecanum.wiring;
        assertEquals("cal fl", wiring.frontLeftName);
        assertEquals("cal fr", wiring.frontRightName);
        assertEquals("cal bl", wiring.backLeftName);
        assertEquals("cal br", wiring.backRightName);
        assertEquals(Direction.REVERSE, wiring.frontLeftDirection);
        assertEquals(Direction.FORWARD, wiring.frontRightDirection);
        assertEquals(Direction.FORWARD, wiring.backLeftDirection);
        assertEquals(Direction.REVERSE, wiring.backRightDirection);
        assertFalse(cfg.mecanum.enableZeroPowerBrake);
        assertEquals(0.13, cfg.manualOmegaScale, 0.0);
        assertEquals(0.18, cfg.autoOmegaCmd, 0.0);
        assertEquals(-2.4, cfg.targetTurnRad, 0.0);
        assertEquals(7.5, cfg.automaticPhaseTimeoutSec, 0.0);
        assertEquals(0.14, cfg.recenterTranslationScale, 0.0);
        assertTrue(cfg.autoComputeAfterAutoSample);
        assertTrue(cfg.enablePostRotateRecenter);
        assertFalse(cfg.enableAutoTagSearchAtStart);
        assertFalse(cfg.enableAutoTagSearchAtEnd);
        assertNull(field(owner, "drive"));
        assertNull(field(owner, "pinpoint"));
        assertColdVision(owner);
    }

    private static void assertAuthoredAprilTags(AprilTagLocalizationConfig config) {
        assertEquals(0.19, config.maxDetectionAgeSec, 0.0);
        assertEquals(11.5, config.fieldPoseSolver.outlierPositionGateInches, 0.0);
        assertEquals(0.27, config.fieldPoseSolver.outlierHeadingGateRad, 0.0);
    }

    private static void assertAuthoredLimelight(FtcLimelightVisionLane.Config config, CameraMountConfig mount) {
        assertSame(mount, config.cameraMount);
        assertNotNull(config.aprilTags);
        assertNull(config.floorObjects);
        assertEquals(4, config.pipelineIndex);
        assertEquals(4, config.aprilTags.pipelineIndex);
        assertEquals(73, config.pollRateHz);
        assertEquals(0.17, config.maxResultAgeSec, 0.0);
    }

    private static void assertFixedLandmarks(TagLayout layout) {
        // Pinned repository SDK season: 20/24 are fixed; 21/22/23 are movable identifiers.
        assertEquals(new HashSet<>(Arrays.asList(20, 24)), layout.ids());
        assertNotNull(layout.requireFieldToTagPose(20));
        assertNotNull(layout.requireFieldToTagPose(24));
        assertNull(layout.getFieldToTagPose(21));
    }

    private static List<TeleOpTester> visionOwners(CalibrationRobotProfile profile) {
        return Arrays.asList(CalibrationTesters.cameraMount(profile),
                CalibrationTesters.aprilTagLocalization(profile),
                CalibrationTesters.correctedLocalization(profile));
    }

    private static void assertColdVision(TeleOpTester owner) throws Exception {
        assertNull(field(owner, "ctx"));
        assertNull(field(owner, "visionLane"));
    }

    private static PinpointPodOffsetCalibrator.Config podConfig(PinpointPodOffsetCalibrator owner)
            throws Exception {
        return (PinpointPodOffsetCalibrator.Config) field(owner, "cfg");
    }

    private static void expectGate(String diagnostic, Runnable action) {
        try {
            action.run();
            fail("Expected explicit gate: " + diagnostic);
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(diagnostic));
        }
    }

    @SuppressWarnings("unchecked")
    private static Function<String, AprilTagCameraFactory> visionBuilder(TeleOpTester owner) throws Exception {
        return (Function<String, AprilTagCameraFactory>) field(owner, "visionLaneFactoryBuilder");
    }

    private static <T> T captured(Object owner, Class<T> captureType) throws Exception {
        // Observe the actual callback's captured template; do not invoke a substitute algorithm.
        for (Field field : owner.getClass().getDeclaredFields()) {
            if (captureType.isAssignableFrom(field.getType())) {
                field.setAccessible(true);
                return captureType.cast(field.get(owner));
            }
        }
        throw new AssertionError("Missing captured " + captureType.getSimpleName());
    }

    private static void assertManual(PinpointPodOffsetCalibrator owner) throws Exception {
        PinpointPodOffsetCalibrator.Config config = (PinpointPodOffsetCalibrator.Config) field(owner, "cfg");
        assertAuthoredPinpoint(config.pinpoint);
        assertNull(config.mecanum);
        assertNull(field(owner, "visionLaneFactoryBuilder"));
        assertNull(field(owner, "drive"));
        assertNull(field(owner, "visionLane"));
        assertNull(field(owner, "pinpoint"));
    }

    @SuppressWarnings("unchecked")
    private static List<MenuItem<Supplier<TeleOpTester>>> items(TesterSuite suite) throws Exception {
        return ((SelectionMenu<Supplier<TeleOpTester>>) field(suite, "menu")).itemsSnapshot();
    }

    private static List<String> labels(List<MenuItem<Supplier<TeleOpTester>>> items) {
        List<String> labels = new ArrayList<>();
        for (MenuItem<?> item : items) labels.add(item.label);
        return labels;
    }

    private static Object field(Object owner, String name) throws Exception {
        for (Class<?> type = owner.getClass(); type != null; type = type.getSuperclass()) {
            try {
                Field field = type.getDeclaredField(name);
                field.setAccessible(true);
                return field.get(owner);
            } catch (NoSuchFieldException missing) {
                // Base tester owns FTC context; each concrete tester owns its captured graph.
            }
        }
        throw new NoSuchFieldException(name);
    }
}
