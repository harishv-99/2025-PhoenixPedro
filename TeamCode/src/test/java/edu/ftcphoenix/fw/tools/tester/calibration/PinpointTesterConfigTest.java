package edu.ftcphoenix.fw.tools.tester.calibration;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Function;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig;
import edu.ftcphoenix.fw.ftc.ui.HardwareNamePicker;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactory;
import edu.ftcphoenix.fw.ftc.vision.VisionReadiness;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.apriltag.AprilTagPoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.tools.tester.TesterContext;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks CONFIG-06's exact Axis/Pod authoring and owner-capture contracts. */
public final class PinpointTesterConfigTest {

    @Test
    public void publicConstructionAndConfigApisAreExact() throws Exception {
        assertOnlyPublicConstructor(
                PinpointAxisDirectionTester.class,
                PinpointAxisDirectionTester.Config.class
        );
        assertOnlyPublicConstructor(
                PinpointPodOffsetCalibrator.class,
                PinpointPodOffsetCalibrator.Config.class,
                Function.class
        );

        assertConfigShape(
                PinpointAxisDirectionTester.Config.class,
                new String[]{"pinpoint", "minTranslationInches", "minRotationDeg"},
                new Class<?>[]{
                        edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor.Config.class,
                        double.class,
                        double.class
                }
        );
        assertConfigShape(
                PinpointPodOffsetCalibrator.Config.class,
                new String[]{
                        "pinpoint",
                        "mecanum",
                        "manualOmegaScale",
                        "autoOmegaCmd",
                        "targetTurnRad",
                        "autoComputeAfterAutoSample",
                        "enableAutoTagSearchAtStart",
                        "tagSearchMaxTurnRad",
                        "tagSearchOmegaCmd",
                        "tagSearchStableFrames",
                        "enableAutoTagSearchAtEnd",
                        "tagEndSearchMaxExtraTurnRad",
                        "enablePostRotateRecenter",
                        "recenterTranslationScale",
                        "preferredVisionDeviceName",
                        "visionDeviceType",
                        "visionPickerTitle",
                        "fixedTagLayout",
                        "aprilTags"
                },
                new Class<?>[]{
                        edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor.Config.class,
                        FtcDrives.MecanumConfig.class,
                        double.class,
                        double.class,
                        double.class,
                        boolean.class,
                        boolean.class,
                        double.class,
                        double.class,
                        int.class,
                        boolean.class,
                        double.class,
                        boolean.class,
                        double.class,
                        String.class,
                        Class.class,
                        String.class,
                        TagLayout.class,
                        AprilTagLocalizationConfig.class
                }
        );
    }

    @Test
    public void defaultsAreFreshAndPreserveTheSelectedSoftwareValues() {
        PinpointAxisDirectionTester.Config axisA = PinpointAxisDirectionTester.Config.defaults();
        PinpointAxisDirectionTester.Config axisB = PinpointAxisDirectionTester.Config.defaults();
        assertNotSame(axisA, axisB);
        assertNotSame(axisA.pinpoint, axisB.pinpoint);
        assertEquals(6.0, axisA.minTranslationInches, 0.0);
        assertEquals(20.0, axisA.minRotationDeg, 0.0);

        PinpointPodOffsetCalibrator.Config podA = PinpointPodOffsetCalibrator.Config.defaults();
        PinpointPodOffsetCalibrator.Config podB = PinpointPodOffsetCalibrator.Config.defaults();
        assertNotSame(podA, podB);
        assertNotSame(podA.pinpoint, podB.pinpoint);
        assertNotSame(podA.aprilTags, podB.aprilTags);
        assertNotSame(podA.aprilTags.fieldPoseSolver, podB.aprilTags.fieldPoseSolver);
        assertNull(podA.mecanum);
        assertEquals(0.6, podA.manualOmegaScale, 0.0);
        assertEquals(0.35, podA.autoOmegaCmd, 0.0);
        assertEquals(Math.PI, podA.targetTurnRad, 0.0);
        assertTrue(podA.autoComputeAfterAutoSample);
        assertTrue(podA.enableAutoTagSearchAtStart);
        assertEquals(4.0 * Math.PI, podA.tagSearchMaxTurnRad, 0.0);
        assertEquals(0.25, podA.tagSearchOmegaCmd, 0.0);
        assertEquals(3, podA.tagSearchStableFrames);
        assertTrue(podA.enableAutoTagSearchAtEnd);
        assertEquals(2.0 * Math.PI, podA.tagEndSearchMaxExtraTurnRad, 0.0);
        assertTrue(podA.enablePostRotateRecenter);
        assertEquals(0.6, podA.recenterTranslationScale, 0.0);
        assertNull(podA.preferredVisionDeviceName);
        assertSame(WebcamName.class, podA.visionDeviceType);
        assertEquals("Select Camera", podA.visionPickerTitle);
        assertNotNull(podA.fixedTagLayout);
        assertNotNull(podA.aprilTags);
        assertEquals(0.50, podA.aprilTags.maxDetectionAgeSec, 0.0);
    }

    @Test
    public void axisCapturesAllActiveDataAndRejectsInvalidThresholds() throws Exception {
        PinpointAxisDirectionTester.Config draft = PinpointAxisDirectionTester.Config.defaults();
        draft.pinpoint.hardwareMapName = "axisPinpoint";
        draft.minTranslationInches = 7.25;
        draft.minRotationDeg = 31.5;

        PinpointAxisDirectionTester owner = new PinpointAxisDirectionTester(draft);
        PinpointAxisDirectionTester.Config captured = field(owner, "cfg");
        assertNotSame(draft, captured);
        assertNotSame(draft.pinpoint, captured.pinpoint);
        assertEquals("axisPinpoint", captured.pinpoint.hardwareMapName);
        assertEquals(7.25, captured.minTranslationInches, 0.0);
        assertEquals(31.5, captured.minRotationDeg, 0.0);

        draft.pinpoint.hardwareMapName = "mutated";
        draft.minTranslationInches = 900.0;
        assertEquals("axisPinpoint", captured.pinpoint.hardwareMapName);
        assertEquals(7.25, captured.minTranslationInches, 0.0);

        assertFailureContains(
                () -> new PinpointAxisDirectionTester(null),
                "PinpointAxisDirectionTester.Config"
        );
        for (double invalid : new double[]{0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY}) {
            PinpointAxisDirectionTester.Config bad = PinpointAxisDirectionTester.Config.defaults();
            bad.minTranslationInches = invalid;
            assertFailureContains(
                    () -> new PinpointAxisDirectionTester(bad),
                    "minTranslationInches"
            );
        }
    }

    @Test
    public void podCapturesActiveDriveVisionPolicyAndLayoutExactlyOnce() throws Exception {
        PinpointPodOffsetCalibrator.Config draft = PinpointPodOffsetCalibrator.Config.defaults();
        draft.pinpoint.hardwareMapName = "podPinpoint";
        draft.mecanum = FtcDrives.MecanumConfig.defaults();
        draft.mecanum.wiring.frontLeftName = "frontLeftSentinel";
        draft.manualOmegaScale = 0.45;
        draft.autoOmegaCmd = 0.55;
        draft.targetTurnRad = -Math.PI;
        draft.autoComputeAfterAutoSample = false;
        draft.tagSearchOmegaCmd = -1.0;
        draft.tagSearchStableFrames = 5;
        draft.recenterTranslationScale = 0.35;
        draft.aprilTags.maxDetectionAgeSec = 0.0;
        draft.aprilTags.fieldPoseSolver.maxAbsBearingRad = 0.4;
        CountingLayout authoredLayout = new CountingLayout();
        authoredLayout.put(7, new Pose3d(1.0, 2.0, 3.0, 0.1, 0.2, 0.3));
        draft.fixedTagLayout = authoredLayout;

        Function<String, AprilTagVisionLaneFactory> builder = ignored -> hardwareMap -> null;
        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator(draft, builder);
        PinpointPodOffsetCalibrator.Config captured = field(owner, "cfg");
        TagLayout capturedLayout = field(owner, "layout");

        assertNotSame(draft, captured);
        assertNotSame(draft.pinpoint, captured.pinpoint);
        assertNotSame(draft.mecanum, captured.mecanum);
        assertNotSame(draft.mecanum.wiring, captured.mecanum.wiring);
        assertNotSame(draft.aprilTags, captured.aprilTags);
        assertNotSame(draft.aprilTags.fieldPoseSolver, captured.aprilTags.fieldPoseSolver);
        assertEquals("podPinpoint", captured.pinpoint.hardwareMapName);
        assertEquals("frontLeftSentinel", captured.mecanum.wiring.frontLeftName);
        assertEquals(0.45, captured.manualOmegaScale, 0.0);
        assertEquals(0.55, captured.autoOmegaCmd, 0.0);
        assertEquals(-Math.PI, captured.targetTurnRad, 0.0);
        assertFalse(captured.autoComputeAfterAutoSample);
        assertEquals(-1.0, captured.tagSearchOmegaCmd, 0.0);
        assertEquals(5, captured.tagSearchStableFrames);
        assertEquals(0.35, captured.recenterTranslationScale, 0.0);
        assertEquals(0.0, captured.aprilTags.maxDetectionAgeSec, 0.0);
        assertEquals(0.4, captured.aprilTags.fieldPoseSolver.maxAbsBearingRad, 0.0);
        assertEquals(1, authoredLayout.idsReads);
        assertEquals(1, authoredLayout.poseReads);
        assertEquals(new Pose3d(1.0, 2.0, 3.0, 0.1, 0.2, 0.3).toString(),
                capturedLayout.getFieldToTagPose(7).toString());

        draft.pinpoint.hardwareMapName = "mutated";
        draft.mecanum.wiring.frontLeftName = "mutated";
        draft.aprilTags.maxDetectionAgeSec = 99.0;
        authoredLayout.put(7, new Pose3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        assertEquals("podPinpoint", captured.pinpoint.hardwareMapName);
        assertEquals("frontLeftSentinel", captured.mecanum.wiring.frontLeftName);
        assertEquals(0.0, captured.aprilTags.maxDetectionAgeSec, 0.0);
        assertEquals(1.0, capturedLayout.getFieldToTagPose(7).xInches, 0.0);
    }

    @Test
    public void podDoesNotInspectDormantDriveOrVisionDrafts() throws Exception {
        PinpointPodOffsetCalibrator.Config draft = PinpointPodOffsetCalibrator.Config.defaults();
        draft.manualOmegaScale = Double.NaN;
        draft.autoOmegaCmd = Double.NEGATIVE_INFINITY;
        draft.targetTurnRad = 0.0;
        draft.tagSearchMaxTurnRad = Double.NaN;
        draft.tagSearchOmegaCmd = 0.0;
        draft.tagSearchStableFrames = 0;
        draft.tagEndSearchMaxExtraTurnRad = Double.NaN;
        draft.recenterTranslationScale = Double.POSITIVE_INFINITY;
        draft.preferredVisionDeviceName = "   ";
        draft.visionDeviceType = null;
        draft.visionPickerTitle = null;
        draft.fixedTagLayout = null;
        draft.aprilTags = null;

        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator(draft, null);
        PinpointPodOffsetCalibrator.Config captured = field(owner, "cfg");
        assertNull(captured.mecanum);
        assertNull(captured.preferredVisionDeviceName);
        assertNull(captured.fixedTagLayout);
        assertNull(captured.aprilTags);

        PinpointPodOffsetCalibrator.Config drivenWithoutVision =
                PinpointPodOffsetCalibrator.Config.defaults();
        drivenWithoutVision.mecanum = FtcDrives.MecanumConfig.defaults();
        drivenWithoutVision.tagSearchMaxTurnRad = Double.NaN;
        drivenWithoutVision.tagSearchOmegaCmd = 0.0;
        drivenWithoutVision.tagSearchStableFrames = 0;
        drivenWithoutVision.tagEndSearchMaxExtraTurnRad = Double.NaN;
        drivenWithoutVision.fixedTagLayout = null;
        drivenWithoutVision.aprilTags = null;
        new PinpointPodOffsetCalibrator(drivenWithoutVision, null);

        PinpointPodOffsetCalibrator.Config visionWithoutDrive =
                PinpointPodOffsetCalibrator.Config.defaults();
        visionWithoutDrive.tagSearchMaxTurnRad = Double.NaN;
        visionWithoutDrive.tagSearchOmegaCmd = 0.0;
        visionWithoutDrive.tagSearchStableFrames = 0;
        visionWithoutDrive.tagEndSearchMaxExtraTurnRad = Double.NaN;
        new PinpointPodOffsetCalibrator(
                visionWithoutDrive,
                ignored -> hardwareMap -> null
        );

        PinpointPodOffsetCalibrator.Config assistWithSearchDisabled =
                PinpointPodOffsetCalibrator.Config.defaults();
        assistWithSearchDisabled.mecanum = FtcDrives.MecanumConfig.defaults();
        assistWithSearchDisabled.enableAutoTagSearchAtStart = false;
        assistWithSearchDisabled.enableAutoTagSearchAtEnd = false;
        assistWithSearchDisabled.tagSearchMaxTurnRad = Double.NaN;
        assistWithSearchDisabled.tagSearchOmegaCmd = Double.NaN;
        assistWithSearchDisabled.tagSearchStableFrames = 0;
        assistWithSearchDisabled.tagEndSearchMaxExtraTurnRad = Double.NaN;
        PinpointPodOffsetCalibrator dormantSearchOwner = new PinpointPodOffsetCalibrator(
                assistWithSearchDisabled,
                ignored -> hardwareMap -> null
        );
        PinpointPodOffsetCalibrator.Config dormantSearchCapture =
                field(dormantSearchOwner, "cfg");
        assertEquals(0.25, dormantSearchCapture.tagSearchOmegaCmd, 0.0);
        assertEquals(3, dormantSearchCapture.tagSearchStableFrames);
    }

    @Test
    public void podPreferredAndPickerBranchesValidateOnlyTheirOwnSelectionFacts() {
        PinpointPodOffsetCalibrator.Config preferred = PinpointPodOffsetCalibrator.Config.defaults();
        preferred.preferredVisionDeviceName = "  frontCam  ";
        preferred.visionDeviceType = null;
        preferred.visionPickerTitle = null;
        final int[] builderCalls = {0};
        final String[] selectedName = {null};
        new PinpointPodOffsetCalibrator(preferred, name -> {
            builderCalls[0]++;
            selectedName[0] = name;
            return hardwareMap -> null;
        });
        assertEquals(1, builderCalls[0]);
        assertEquals("frontCam", selectedName[0]);

        PinpointPodOffsetCalibrator.Config picker = PinpointPodOffsetCalibrator.Config.defaults();
        picker.visionDeviceType = null;
        assertFailureContains(
                () -> new PinpointPodOffsetCalibrator(picker, ignored -> hardwareMap -> null),
                "visionDeviceType"
        );

        PinpointPodOffsetCalibrator.Config blankPreferred =
                PinpointPodOffsetCalibrator.Config.defaults();
        blankPreferred.preferredVisionDeviceName = " \t ";
        assertFailureContains(
                () -> new PinpointPodOffsetCalibrator(
                        blankPreferred,
                        ignored -> hardwareMap -> null
                ),
                "preferredVisionDeviceName"
        );
    }

    @Test
    public void podDriveAndSearchDomainsLockExactBoundaries() {
        PinpointPodOffsetCalibrator.Config drive = PinpointPodOffsetCalibrator.Config.defaults();
        drive.mecanum = FtcDrives.MecanumConfig.defaults();
        drive.manualOmegaScale = 0.0;
        drive.autoOmegaCmd = 1.0;
        drive.targetTurnRad = -Math.PI;
        drive.enablePostRotateRecenter = true;
        drive.recenterTranslationScale = 1.0;
        new PinpointPodOffsetCalibrator(drive, null);

        drive.autoOmegaCmd = 0.0;
        assertFailureContains(
                () -> new PinpointPodOffsetCalibrator(drive, null),
                "autoOmegaCmd"
        );
        drive.autoOmegaCmd = 0.35;
        drive.targetTurnRad = 0.0;
        assertFailureContains(
                () -> new PinpointPodOffsetCalibrator(drive, null),
                "targetTurnRad"
        );

        PinpointPodOffsetCalibrator.Config assist = PinpointPodOffsetCalibrator.Config.defaults();
        assist.mecanum = FtcDrives.MecanumConfig.defaults();
        assist.tagSearchOmegaCmd = 1.0;
        new PinpointPodOffsetCalibrator(assist, ignored -> hardwareMap -> null);
        assist.tagSearchOmegaCmd = -1.0;
        new PinpointPodOffsetCalibrator(assist, ignored -> hardwareMap -> null);
        assist.tagSearchOmegaCmd = 0.0;
        assertFailureContains(
                () -> new PinpointPodOffsetCalibrator(
                        assist,
                        ignored -> hardwareMap -> null
                ),
                "tagSearchOmegaCmd"
        );
    }

    @Test
    public void podActiveNestedFailuresNameTheOwningFieldAndEmptyLayoutIsValid()
            throws Exception {
        PinpointPodOffsetCalibrator.Config badMecanum = PinpointPodOffsetCalibrator.Config.defaults();
        badMecanum.mecanum = FtcDrives.MecanumConfig.defaults();
        badMecanum.mecanum.drivebase.maxAxial = Double.NaN;
        assertFailureContains(
                () -> new PinpointPodOffsetCalibrator(badMecanum, null),
                "PinpointPodOffsetCalibrator.Config.mecanum"
        );

        PinpointPodOffsetCalibrator.Config badApril = PinpointPodOffsetCalibrator.Config.defaults();
        badApril.aprilTags.maxDetectionAgeSec = -1.0;
        assertFailureContains(
                () -> new PinpointPodOffsetCalibrator(
                        badApril,
                        ignored -> hardwareMap -> null
                ),
                "PinpointPodOffsetCalibrator.Config.aprilTags.maxDetectionAgeSec"
        );

        PinpointPodOffsetCalibrator.Config empty = PinpointPodOffsetCalibrator.Config.defaults();
        empty.fixedTagLayout = new CountingLayout();
        PinpointPodOffsetCalibrator emptyOwner = new PinpointPodOffsetCalibrator(
                empty,
                ignored -> hardwareMap -> null
        );
        TagLayout emptySnapshot = field(emptyOwner, "layout");
        assertTrue(emptySnapshot.ids().isEmpty());

        PinpointPodOffsetCalibrator.Config game = PinpointPodOffsetCalibrator.Config.defaults();
        FtcGameTagLayout authoredGameLayout = (FtcGameTagLayout) game.fixedTagLayout;
        PinpointPodOffsetCalibrator gameOwner = new PinpointPodOffsetCalibrator(
                game,
                ignored -> hardwareMap -> null
        );
        assertEquals(
                authoredGameLayout.policySummaryLine(),
                field(gameOwner, "fixedTagLayoutPolicySummary")
        );
        assertFalse(field(gameOwner, "layout") instanceof FtcGameTagLayout);
    }

    @Test
    public void podInvalidDirectDriveIdentityFailsBeforeEveryInitOwnedEffect()
            throws Exception {
        assertPodInvalidDirectDriveRejectedBeforeEffects(
                wiring -> wiring.frontLeftName = " \t ",
                "hardware name is blank"
        );
        assertPodInvalidDirectDriveRejectedBeforeEffects(
                wiring -> wiring.frontRightDirection = null,
                "direction is required"
        );
        assertPodInvalidDirectDriveRejectedBeforeEffects(
                wiring -> wiring.backRightName = "  " + wiring.frontLeftName + "  ",
                "already used by member 1"
        );
    }

    @Test
    public void podOrdinaryInitClearingIsSilentAndStartOwnsTheFirstZero() throws Exception {
        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator(
                PinpointPodOffsetCalibrator.Config.defaults(),
                null
        );
        PowerProbe fl = new PowerProbe(null);
        PowerProbe fr = new PowerProbe(null);
        PowerProbe bl = new PowerProbe(null);
        PowerProbe br = new PowerProbe(null);
        setField(owner, "drive", drive(fl, fr, bl, br));

        invoke(owner, "clearState", new Class<?>[]{boolean.class}, false);
        invoke(owner, "abortSample", new Class<?>[0]);
        assertNoPowerEffects(fl, fr, bl, br);

        invoke(owner, "onStart", new Class<?>[0]);
        for (PowerProbe probe : new PowerProbe[]{fl, fr, bl, br}) {
            assertEquals(1, probe.setPowerCalls);
            assertEquals(0.0, probe.commandedPower, 0.0);
            assertEquals(0, probe.stopCalls);
        }
        assertTrue((Boolean) field(owner, "started"));
    }

    @Test
    public void podCleanupMayStopBeforeStartAndFailedInitPreservesItsPrimaryFailure()
            throws Exception {
        PinpointPodOffsetCalibrator stopBeforeStart = new PinpointPodOffsetCalibrator(
                PinpointPodOffsetCalibrator.Config.defaults(),
                null
        );
        PowerProbe fl = new PowerProbe(null);
        PowerProbe fr = new PowerProbe(null);
        PowerProbe bl = new PowerProbe(null);
        PowerProbe br = new PowerProbe(null);
        setField(stopBeforeStart, "drive", drive(fl, fr, bl, br));
        stopBeforeStart.stop();
        for (PowerProbe probe : new PowerProbe[]{fl, fr, bl, br}) {
            assertEquals(0, probe.setPowerCalls);
            assertEquals(1, probe.stopCalls);
        }

        PinpointPodOffsetCalibrator failedInit = new PinpointPodOffsetCalibrator(
                PinpointPodOffsetCalibrator.Config.defaults(),
                null
        );
        RuntimeException cleanupFailure = new IllegalStateException("drive stop failed");
        PowerProbe failing = new PowerProbe(cleanupFailure);
        setField(
                failedInit,
                "drive",
                drive(failing, new PowerProbe(null), new PowerProbe(null), new PowerProbe(null))
        );
        RuntimeException primary = new IllegalArgumentException("Pinpoint setup failed");
        RuntimeException returned = (RuntimeException) invoke(
                failedInit,
                "rollbackDriveAfterInitializationFailure",
                new Class<?>[]{RuntimeException.class},
                primary
        );
        assertSame(primary, returned);
        assertEquals(1, primary.getSuppressed().length);
        assertSame(cleanupFailure, primary.getSuppressed()[0]);
        assertEquals(1, failing.stopCalls);
        assertNull(field(failedInit, "drive"));
    }

    @Test
    public void podTerminalVisionFailureDiscardsSameCycleMotionIntent() throws Exception {
        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator(
                PinpointPodOffsetCalibrator.Config.defaults(),
                ignored -> hardwareMap -> null
        );
        PowerProbe fl = new PowerProbe(null);
        PowerProbe fr = new PowerProbe(null);
        PowerProbe bl = new PowerProbe(null);
        PowerProbe br = new PowerProbe(null);
        setField(owner, "drive", drive(fl, fr, bl, br));
        setField(owner, "started", true);
        setField(owner, "primaryActionRequested", true);
        setField(owner, "autoStartRequested", true);

        invoke(
                owner,
                "recordVisionFailure",
                new Class<?>[]{
                        RuntimeException.class,
                        String.class,
                        boolean.class,
                        boolean.class
                },
                new IllegalStateException("camera disconnected"),
                "Vision readiness failed",
                false,
                true
        );
        assertTrue((Boolean) invoke(
                owner,
                "terminalVisionFailureBlocksCalibration",
                new Class<?>[0]
        ));

        invoke(
                owner,
                "consumeControlRequestsAfterCurrentPoll",
                new Class<?>[]{boolean.class},
                true
        );
        for (PowerProbe probe : new PowerProbe[]{fl, fr, bl, br}) {
            assertEquals("failure abort owns one zero; stale A/Y must not submit another", 1,
                    probe.setPowerCalls);
            assertEquals(0.0, probe.commandedPower, 0.0);
        }
        assertEquals("IDLE", String.valueOf((Object) field(owner, "phase")));
        assertFalse((Boolean) field(owner, "primaryActionRequested"));
        assertFalse((Boolean) field(owner, "autoStartRequested"));

        setField(owner, "aprilTagAssistUnavailable", true);
        assertFalse("identity-mount fallback is the deliberate non-vision exception",
                (Boolean) invoke(
                        owner,
                        "terminalVisionFailureBlocksCalibration",
                        new Class<?>[0]
                ));
    }

    @Test
    public void podActiveVisionFailureRetainsLaneWhenSafetyZeroThrowsErrorForLaterStop()
            throws Exception {
        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator(
                PinpointPodOffsetCalibrator.Config.defaults(),
                ignored -> hardwareMap -> null
        );
        AssertionError zeroError = new AssertionError("drive zero failed");
        ErrorOnSetPowerProbe failing = new ErrorOnSetPowerProbe(zeroError);
        PowerProbe unused = new PowerProbe(null);
        setField(owner, "drive", new MecanumDrivebase(
                failing,
                unused,
                unused,
                unused,
                MecanumDrivebase.Config.defaults()
        ));
        setField(owner, "started", true);
        IdentityLane lane = new IdentityLane();
        setField(owner, "visionLane", lane);

        try {
            invoke(
                    owner,
                    "recordVisionFailure",
                    new Class<?>[]{
                            RuntimeException.class,
                            String.class,
                            boolean.class,
                            boolean.class
                    },
                    new IllegalStateException("camera disconnected"),
                    "Vision readiness failed",
                    false,
                    true
            );
            fail("Expected drive-zero Error");
        } catch (AssertionError actual) {
            assertSame(zeroError, actual);
        }

        assertSame("Error must leave the published lane reachable by STOP", lane,
                field(owner, "visionLane"));
        assertEquals(0, lane.closeCalls);
        assertTrue((Boolean) field(owner, "visionRetryBlocked"));

        owner.stop();
        assertEquals(1, failing.stopCalls);
        assertEquals(1, lane.closeCalls);
        assertNull(field(owner, "visionLane"));
    }

    @Test
    public void podPickerIdentityFallbackClosesOnceAndKeepsCalibrationEligible()
            throws Exception {
        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator(
                PinpointPodOffsetCalibrator.Config.defaults(),
                ignored -> hardwareMap -> null
        );
        IdentityLane lane = new IdentityLane();
        setField(
                owner,
                "visionPicker",
                new HardwareNamePicker(null, WebcamName.class, "unused")
        );
        setField(owner, "selectedVisionDeviceName", "pickedCamera");
        setField(owner, "visionLane", lane);

        invoke(owner, "disableIdentityMountAssist", new Class<?>[0]);

        assertEquals(1, lane.closeCalls);
        assertNull(field(owner, "visionLane"));
        assertNull(field(owner, "selectedVisionDeviceName"));
        assertTrue((Boolean) field(owner, "aprilTagAssistUnavailable"));
        assertTrue("clean identity fallback must expose the non-vision picker workflow",
                (Boolean) invoke(
                        owner,
                        "calibrationControlsEligible",
                        new Class<?>[0]
                ));

        owner.stop();
        assertEquals("the detached identity lane must not be closed again at STOP", 1,
                lane.closeCalls);
    }

    @Test
    public void podSearchNeverConsumesAnEstimatorPoseRetainedFromAnEarlierReadyLoop()
            throws Exception {
        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator(
                PinpointPodOffsetCalibrator.Config.defaults(),
                ignored -> hardwareMap -> null
        );
        PowerProbe fl = new PowerProbe(null);
        PowerProbe fr = new PowerProbe(null);
        PowerProbe bl = new PowerProbe(null);
        PowerProbe br = new PowerProbe(null);
        setField(owner, "drive", drive(fl, fr, bl, br));

        CountingLayout layout = new CountingLayout();
        layout.put(1, Pose3d.zero());
        AprilTagSensor sensor = clock -> AprilTagDetections.none();
        AprilTagPoseEstimator retainedEstimator = new AprilTagPoseEstimator(
                sensor,
                layout,
                AprilTagPoseEstimator.Config.defaults()
        );
        setField(
                retainedEstimator,
                "lastEstimate",
                new PoseEstimate(Pose3d.zero(), true, 1.0, LoopTimestamp.unavailable())
        );
        setField(owner, "tagEstimator", retainedEstimator);

        setField(owner, "tagStableFrames", 3);
        invoke(owner, "updateSearchForTagStart", new Class<?>[0]);
        assertEquals(0, (int) field(owner, "tagStableFrames"));
        for (PowerProbe probe : new PowerProbe[]{fl, fr, bl, br}) {
            assertEquals(1, probe.setPowerCalls);
        }

        setField(owner, "tagStableFrames", 3);
        invoke(owner, "updateSearchForTagEnd", new Class<?>[0]);
        assertEquals(0, (int) field(owner, "tagStableFrames"));
        for (PowerProbe probe : new PowerProbe[]{fl, fr, bl, br}) {
            assertEquals(2, probe.setPowerCalls);
        }
    }

    private static void assertOnlyPublicConstructor(Class<?> owner, Class<?>... parameterTypes)
            throws Exception {
        Constructor<?> expected = owner.getDeclaredConstructor(parameterTypes);
        assertTrue(Modifier.isPublic(expected.getModifiers()));
        int publicCount = 0;
        for (Constructor<?> constructor : owner.getDeclaredConstructors()) {
            if (Modifier.isPublic(constructor.getModifiers())) {
                publicCount++;
            }
        }
        assertEquals(owner.getSimpleName(), 1, publicCount);
    }

    private static void assertConfigShape(
            Class<?> configType,
            String[] expectedNames,
            Class<?>[] expectedTypes) throws Exception {
        int modifiers = configType.getModifiers();
        assertTrue(Modifier.isPublic(modifiers));
        assertTrue(Modifier.isStatic(modifiers));
        assertTrue(Modifier.isFinal(modifiers));

        Constructor<?>[] constructors = configType.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPrivate(constructors[0].getModifiers()));

        Field[] publicFields = configType.getFields();
        String[] actualNames = new String[publicFields.length];
        Class<?>[] actualTypes = new Class<?>[publicFields.length];
        for (int i = 0; i < publicFields.length; i++) {
            actualNames[i] = publicFields[i].getName();
            actualTypes[i] = publicFields[i].getType();
        }
        assertArrayEquals(expectedNames, actualNames);
        assertArrayEquals(expectedTypes, actualTypes);

        Method[] methods = configType.getDeclaredMethods();
        assertEquals(Arrays.toString(methods), 1, methods.length);
        assertEquals("defaults", methods[0].getName());
        assertTrue(Modifier.isPublic(methods[0].getModifiers()));
        assertTrue(Modifier.isStatic(methods[0].getModifiers()));
        assertEquals(configType, methods[0].getReturnType());
        assertEquals(0, methods[0].getParameterTypes().length);
    }

    @SuppressWarnings("unchecked")
    private static <T> T field(Object owner, String name) throws Exception {
        Field field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return (T) field.get(owner);
    }

    private static void setField(Object owner, String name, Object value) throws Exception {
        Field field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        field.set(owner, value);
    }

    private static Object invoke(
            Object owner,
            String name,
            Class<?>[] parameterTypes,
            Object... args) throws Exception {
        Method method = owner.getClass().getDeclaredMethod(name, parameterTypes);
        method.setAccessible(true);
        try {
            return method.invoke(owner, args);
        } catch (InvocationTargetException failure) {
            Throwable cause = failure.getCause();
            if (cause instanceof RuntimeException) {
                throw (RuntimeException) cause;
            }
            if (cause instanceof Error) {
                throw (Error) cause;
            }
            throw failure;
        }
    }

    private static MecanumDrivebase drive(PowerProbe fl,
                                          PowerProbe fr,
                                          PowerProbe bl,
                                          PowerProbe br) {
        return new MecanumDrivebase(
                fl,
                fr,
                bl,
                br,
                MecanumDrivebase.Config.defaults()
        );
    }

    private static void assertNoPowerEffects(PowerProbe... probes) {
        for (PowerProbe probe : probes) {
            assertEquals(0, probe.setPowerCalls);
            assertEquals(0, probe.stopCalls);
        }
    }

    private static void assertFailureContains(Runnable action, String expectedText) {
        try {
            action.run();
            fail("Expected failure containing: " + expectedText);
        } catch (RuntimeException failure) {
            assertTrue(
                    "Expected '" + expectedText + "' in: " + failure,
                    String.valueOf(failure.getMessage()).contains(expectedText)
            );
        }
    }

    private static void assertPodInvalidDirectDriveRejectedBeforeEffects(
            Consumer<FtcDrives.MecanumWiringConfig> makeInvalid,
            String expectedCauseText) throws Exception {
        PinpointPodOffsetCalibrator.Config config =
                PinpointPodOffsetCalibrator.Config.defaults();
        config.mecanum = FtcDrives.MecanumConfig.defaults();
        config.preferredVisionDeviceName = "frontCamera";
        makeInvalid.accept(config.mecanum.wiring);

        int[] builderCalls = {0};
        int[] openCalls = {0};
        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator(
                config,
                ignored -> {
                    builderCalls[0]++;
                    return hardwareMap -> {
                        openCalls[0]++;
                        return null;
                    };
                }
        );
        assertEquals("preferred factory capture is the sole permitted prior effect", 1,
                builderCalls[0]);

        CountingHardwareMap hardwareMap = new CountingHardwareMap();
        TelemetryProbe telemetry = new TelemetryProbe();
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        TesterContext context = new TesterContext(
                hardwareMap,
                telemetry.proxy,
                new Gamepad(),
                new Gamepad(),
                clock
        );

        try {
            owner.init(context);
            fail("Expected direct-drive identity failure");
        } catch (RuntimeException failure) {
            String message = String.valueOf(failure.getMessage());
            assertTrue(message, message.contains(
                    "PinpointPodOffsetCalibrator.Config.mecanum construction failed"));
            assertTrue(message, message.contains(expectedCauseText));
        }

        assertEquals("whole-group identity validation must precede every HardwareMap lookup", 0,
                hardwareMap.lookupCalls);
        assertEquals("invalid drive must not open vision", 0, openCalls[0]);
        assertEquals("invalid drive must not touch child telemetry/UI", 0, telemetry.calls);
        assertNull("invalid drive must not publish a drive owner", field(owner, "drive"));
        assertNull("invalid drive must precede Pinpoint construction", field(owner, "pinpoint"));
        assertNull("invalid drive must precede picker construction", field(owner, "visionPicker"));
        assertNull("invalid drive must precede vision publication", field(owner, "visionLane"));
    }

    private static final class CountingLayout implements TagLayout {
        private final Set<Integer> ids = new LinkedHashSet<Integer>();
        private final Map<Integer, Pose3d> poses = new LinkedHashMap<Integer, Pose3d>();
        int idsReads;
        int poseReads;

        void put(int id, Pose3d pose) {
            ids.add(id);
            poses.put(id, pose);
        }

        @Override
        public Pose3d getFieldToTagPose(int id) {
            poseReads++;
            return poses.get(id);
        }

        @Override
        public Set<Integer> ids() {
            idsReads++;
            return ids;
        }
    }

    private static final class CountingHardwareMap extends HardwareMap {
        private int lookupCalls;

        CountingHardwareMap() {
            super(null, null);
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCalls++;
            throw new AssertionError("validation must precede lookup of " + name);
        }
    }

    private static final class TelemetryProbe {
        private int calls;
        private final Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (ignored, method, args) -> {
                    calls++;
                    return defaultValue(method.getReturnType());
                }
        );
    }

    private static Object defaultValue(Class<?> type) {
        if (type == boolean.class) return false;
        if (type == byte.class) return (byte) 0;
        if (type == short.class) return (short) 0;
        if (type == int.class) return 0;
        if (type == long.class) return 0L;
        if (type == float.class) return 0.0f;
        if (type == double.class) return 0.0;
        if (type == char.class) return '\0';
        return null;
    }

    private static final class IdentityLane implements AprilTagVisionLane {
        private int closeCalls;

        @Override
        public AprilTagSensor tagSensor() {
            return clock -> AprilTagDetections.none();
        }

        @Override
        public CameraMountConfig cameraMountConfig() {
            return CameraMountConfig.identity();
        }

        @Override
        public VisionReadiness readiness(LoopClock clock) {
            return VisionReadiness.notReady("unused");
        }

        @Override
        public void close() {
            closeCalls++;
        }
    }

    private static final class ErrorOnSetPowerProbe implements PowerOutput {
        private final Error error;
        private int stopCalls;

        ErrorOnSetPowerProbe(Error error) {
            this.error = error;
        }

        @Override
        public void setPower(double power) {
            throw error;
        }

        @Override
        public double getCommandedPower() {
            return 0.0;
        }

        @Override
        public void stop() {
            stopCalls++;
        }
    }

    private static final class PowerProbe implements PowerOutput {
        private final RuntimeException stopFailure;
        int setPowerCalls;
        int stopCalls;
        double commandedPower;

        PowerProbe(RuntimeException stopFailure) {
            this.stopFailure = stopFailure;
        }

        @Override
        public void setPower(double power) {
            setPowerCalls++;
            commandedPower = power;
        }

        @Override
        public double getCommandedPower() {
            return commandedPower;
        }

        @Override
        public void stop() {
            stopCalls++;
            if (stopFailure != null) {
                throw stopFailure;
            }
            commandedPower = 0.0;
        }
    }
}
