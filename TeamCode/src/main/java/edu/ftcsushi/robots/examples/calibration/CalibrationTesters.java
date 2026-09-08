package edu.ftcsushi.robots.examples.calibration;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.Objects;
import java.util.function.Function;

import edu.ftcsushi.fw.ftc.vision.AprilTagCameraFactories;
import edu.ftcsushi.fw.ftc.vision.AprilTagCameraFactory;
import edu.ftcsushi.fw.ftc.vision.FtcLimelightVisionLane;
import edu.ftcsushi.fw.ftc.vision.FtcWebcamVisionLane;
import edu.ftcsushi.fw.tools.tester.TesterSuite;
import edu.ftcsushi.fw.tools.tester.calibration.CalibrationChecks;
import edu.ftcsushi.fw.tools.tester.calibration.CalibrationWalkthroughBuilder;
import edu.ftcsushi.fw.tools.tester.calibration.CameraMountCalibrator;
import edu.ftcsushi.fw.tools.tester.calibration.PinpointAxisDirectionTester;
import edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibrator;
import edu.ftcsushi.fw.tools.tester.localization.AprilTagLocalizationTester;
import edu.ftcsushi.fw.tools.tester.localization.PinpointAprilTagCorrectedLocalizationTester;

/**
 * Fresh, exclusive diagnostic owners built from one independent robot profile.
 *
 * <p>Each method snapshots its authoring input. Menu suppliers construct inactive testers only
 * when selected; those testers acquire and own hardware during init. Never share their Pinpoint,
 * camera, or drivetrain with match code. These factories do not save results or alter the profile.
 * Record accepted facts, rebuild, then construct a fresh configured verification owner.</p>
 */
public final class CalibrationTesters {
    private CalibrationTesters() { }

    /**
     * Builds the ordinary suite: two unpowered Pinpoint checks, plus explicitly enabled submenus.
     * Registration freezes facts but neither opens hardware nor constructs dormant camera/drive
     * Configs. Camera NONE and unreviewed powered motion omit their corresponding submenus.
     */
    public static TesterSuite create(CalibrationRobotProfile profile) {
        CalibrationRobotProfile captured = capture(profile);
        TesterSuite suite = new TesterSuite().setTitle("Robot calibration")
                .setHelp("Record -> rebuild -> fresh configured verification");
        suite.add("Pinpoint axis directions", "Hand motion; verify the captured signs",
                () -> axisDirections(captured));
        suite.add("Manual pod offsets", "No drive or camera; hand rotation and recentering",
                () -> manualPodOffsets(captured));
        if (captured.cameraBackend != CalibrationRobotProfile.CameraBackend.NONE) {
            suite.add("Vision checks", "Selected backend: " + captured.cameraBackend,
                    () -> visionChecks(captured));
        }
        if (captured.poweredMotionReviewed) {
            suite.add("Powered pod offsets", "Reviewed motion only; keep a person at FTC STOP",
                    () -> poweredChecks(captured));
        }
        return suite;
    }

    /** Returns a fresh hand-motion axis tester using the canonical Pinpoint facts. */
    public static PinpointAxisDirectionTester axisDirections(CalibrationRobotProfile profile) {
        CalibrationRobotProfile captured = capture(profile);
        PinpointAxisDirectionTester.Config cfg = PinpointAxisDirectionTester.Config.defaults();
        cfg.pinpoint = captured.pinpoint();
        return new PinpointAxisDirectionTester(cfg);
    }

    /** Returns a fresh hand-motion pod tester; neither camera nor drive configuration is active. */
    public static PinpointPodOffsetCalibrator manualPodOffsets(CalibrationRobotProfile profile) {
        CalibrationRobotProfile captured = capture(profile);
        PinpointPodOffsetCalibrator.Config cfg = PinpointPodOffsetCalibrator.Config.defaults();
        cfg.pinpoint = captured.pinpoint();
        cfg.mecanum = null;
        return new PinpointPodOffsetCalibrator(cfg, null);
    }

    /**
     * Returns a mount-measurement owner; no accepted mount is required because it is the unknown.
     * Requires a selected backend. The operator must supply an independently known robot pose.
     */
    public static CameraMountCalibrator cameraMount(CalibrationRobotProfile profile) {
        CalibrationRobotProfile captured = captureVision(profile);
        CameraMountCalibrator.Config cfg = CameraMountCalibrator.Config.defaults();
        cfg.preferredVisionDeviceName = visionDeviceName(captured);
        cfg.visionDeviceType = visionDeviceType(captured);
        cfg.visionPickerTitle = "Select " + captured.cameraBackend;
        cfg.fixedTagLayout = captured.fixedTagLayout();
        cfg.maxDetectionAgeSec = captured.maxDetectionAgeSec;
        return new CameraMountCalibrator(cfg, visionFactoryBuilder(captured));
    }

    /**
     * Returns a configured AprilTag-only check with the selected camera's mount and shared solver.
     * An unaccepted mount may be inspected, but does not establish accurate field localization.
     */
    public static AprilTagLocalizationTester aprilTagLocalization(CalibrationRobotProfile profile) {
        CalibrationRobotProfile captured = captureVision(profile);
        AprilTagLocalizationTester.Config cfg = AprilTagLocalizationTester.Config.defaults();
        cfg.preferredVisionDeviceName = visionDeviceName(captured);
        cfg.visionDeviceType = visionDeviceType(captured);
        cfg.visionPickerTitle = "Select " + captured.cameraBackend;
        cfg.fixedTagLayout = captured.fixedTagLayout();
        cfg.aprilTags = captured.aprilTags();
        return new AprilTagLocalizationTester(cfg, visionFactoryBuilder(captured));
    }

    /**
     * Returns fresh Pinpoint plus raw-AprilTag FUSION owners using the same canonical facts.
     * Neither this diagnostic nor a smooth fused estimate proves that those facts are calibrated.
     */
    public static PinpointAprilTagCorrectedLocalizationTester correctedLocalization(
            CalibrationRobotProfile profile) {
        CalibrationRobotProfile captured = captureVision(profile);
        PinpointAprilTagCorrectedLocalizationTester.Config cfg =
                PinpointAprilTagCorrectedLocalizationTester.Config.defaults();
        cfg.preferredVisionDeviceName = visionDeviceName(captured);
        cfg.visionDeviceType = visionDeviceType(captured);
        cfg.visionPickerTitle = "Select " + captured.cameraBackend;
        cfg.fixedTagLayout = captured.fixedTagLayout();
        cfg.localization = captured.localization();
        return new PinpointAprilTagCorrectedLocalizationTester(cfg, visionFactoryBuilder(captured));
    }

    /**
     * Returns an explicitly unassisted powered pod tester after the motion-review gate.
     * Y can turn automatically; right-stick manual turning and left-stick recentering remain
     * powered. Only automatic phases have the cooperative elapsed-time limit.
     *
     * @throws IllegalStateException before Config construction when powered motion is unreviewed
     */
    public static PinpointPodOffsetCalibrator poweredPodOffsets(CalibrationRobotProfile profile) {
        CalibrationRobotProfile captured = capture(profile);
        requireReviewedMotion(captured);
        return new PinpointPodOffsetCalibrator(poweredPodConfig(captured), null);
    }

    /**
     * Returns a separately selected assisted powered tester, never an unassisted fallback.
     * Requires reviewed motion, a selected camera, and an accepted non-identity mount. Both tag
     * searches are off: a start requires visible capture-matched evidence, and a missing required
     * end discards the attempt. The tester privately owns raw odometry history.
     *
     * @throws IllegalStateException before Config construction when any review gate is unmet
     */
    public static PinpointPodOffsetCalibrator assistedPodOffsets(CalibrationRobotProfile profile) {
        CalibrationRobotProfile captured = capture(profile);
        requireReviewedMotion(captured);
        requireVision(captured);
        if (!captured.cameraMountAccepted
                || !CalibrationChecks.canUseAprilTagAssist(captured.cameraMount)) {
            throw new IllegalStateException("Assisted pod offsets require cameraMountAccepted and "
                    + "an accepted non-identity cameraMount; record, rebuild, and verify the mount first");
        }
        PinpointPodOffsetCalibrator.Config cfg = poweredPodConfig(captured);
        cfg.preferredVisionDeviceName = visionDeviceName(captured);
        cfg.visionDeviceType = visionDeviceType(captured);
        cfg.visionPickerTitle = "Select " + captured.cameraBackend;
        cfg.fixedTagLayout = captured.fixedTagLayout();
        cfg.aprilTags = captured.aprilTags();
        return new PinpointPodOffsetCalibrator(cfg, visionFactoryBuilder(captured));
    }

    /**
     * Optional ordered view of the same fresh factories, not another implementation or save path.
     * Status is captured at construction: axis/offset acknowledgements are human declarations;
     * nonzero offsets and a non-identity camera mount are only heuristics. No tag authorizes motion.
     */
    public static TesterSuite guidedWalkthrough(CalibrationRobotProfile profile) {
        CalibrationRobotProfile captured = capture(profile);
        CalibrationWalkthroughBuilder guide =
                new CalibrationWalkthroughBuilder("Robot calibration walkthrough");
        guide.addStep("Verify Pinpoint axes", "Hand motion; record, rebuild, and verify",
                () -> CalibrationChecks.pinpointAxes(captured.pinpointAxesVerified),
                () -> axisDirections(captured));
        guide.addStep("Verify Pinpoint offsets", "Status is not physical evidence",
                () -> CalibrationChecks.pinpointOffsets(
                        captured.pinpoint(), captured.pinpointOffsetsVerified),
                () -> manualPodOffsets(captured));
        if (captured.cameraBackend != CalibrationRobotProfile.CameraBackend.NONE) {
            guide.addStep("Measure camera mount", "Use an independently known robot pose",
                    () -> cameraMount(captured));
            guide.addStep("Verify configured camera mount", "Tag detects identity, not acceptance",
                    () -> CalibrationChecks.cameraMount(captured.cameraMount),
                    () -> aprilTagLocalization(captured));
            guide.addStep("Compare corrected localization", "Only after independent facts agree",
                    () -> correctedLocalization(captured));
        }
        return guide.build();
    }

    /** Builds a fresh submenu without opening any of its camera owners. */
    private static TesterSuite visionChecks(CalibrationRobotProfile captured) {
        TesterSuite suite = new TesterSuite().setTitle("Vision checks: " + captured.cameraBackend);
        suite.add("Measure camera mount", "Independent known robot pose; record the answer",
                () -> cameraMount(captured));
        suite.add("AprilTag localization", "Inspect the configured mount; acceptance is a human check",
                () -> aprilTagLocalization(captured));
        suite.add("Pinpoint + AprilTag fusion", "Shared profile; no powered drive",
                () -> correctedLocalization(captured));
        return suite;
    }

    /** Keeps unassisted and assisted operations separately named and selected. */
    private static TesterSuite poweredChecks(CalibrationRobotProfile captured) {
        TesterSuite suite = new TesterSuite().setTitle("Reviewed powered pod offsets");
        suite.add("Unassisted powered pod offsets", "Y turns; manual recenter; no camera",
                () -> poweredPodOffsets(captured));
        if (captured.cameraBackend != CalibrationRobotProfile.CameraBackend.NONE
                && captured.cameraMountAccepted
                && CalibrationChecks.canUseAprilTagAssist(captured.cameraMount)) {
            suite.add("Assisted powered pod offsets", "Capture-matched tags required; no tag search",
                    () -> assistedPodOffsets(captured));
        }
        return suite;
    }

    /** Maps reviewed drive facts and the explicit example motion choices into one fresh Config. */
    private static PinpointPodOffsetCalibrator.Config poweredPodConfig(
            CalibrationRobotProfile captured) {
        PinpointPodOffsetCalibrator.Config cfg = PinpointPodOffsetCalibrator.Config.defaults();
        cfg.pinpoint = captured.pinpoint();
        cfg.mecanum = captured.mecanum();
        cfg.manualOmegaScale = captured.manualOmegaScale;
        cfg.autoOmegaCmd = captured.autoOmegaCmd;
        cfg.targetTurnRad = captured.targetTurnRad;
        cfg.automaticPhaseTimeoutSec = captured.automaticPhaseTimeoutSec;
        cfg.recenterTranslationScale = captured.recenterTranslationScale;
        cfg.enableAutoTagSearchAtStart = false;
        cfg.enableAutoTagSearchAtEnd = false;
        return cfg;
    }

    /**
     * Captures one backend template now; later picker callbacks change only a fresh copy's name.
     * No callback rereads the authoring profile or opens hardware. A replacement device must match
     * the same reviewed geometry and settings; a picker cannot verify that physical equivalence.
     */
    private static Function<String, AprilTagCameraFactory> visionFactoryBuilder(
            CalibrationRobotProfile captured) {
        if (captured.cameraBackend == CalibrationRobotProfile.CameraBackend.WEBCAM) {
            FtcWebcamVisionLane.Config template = captured.webcam();
            return selectedName -> {
                FtcWebcamVisionLane.Config cfg = template.copy();
                cfg.webcamName = selectedName;
                return AprilTagCameraFactories.webcam(cfg);
            };
        }
        FtcLimelightVisionLane.Config template = captured.limelight();
        return selectedName -> {
            FtcLimelightVisionLane.Config cfg = template.copy();
            cfg.hardwareName = selectedName;
            return AprilTagCameraFactories.limelight(cfg);
        };
    }

    /** Returns the preferred name only after the backend-selection gate. */
    private static String visionDeviceName(CalibrationRobotProfile captured) {
        return captured.cameraBackend == CalibrationRobotProfile.CameraBackend.WEBCAM
                ? captured.webcamName : captured.limelightName;
    }

    /** Keeps picker enumeration consistent with the frozen backend, including later replacement. */
    private static Class<? extends HardwareDevice> visionDeviceType(CalibrationRobotProfile captured) {
        return captured.cameraBackend == CalibrationRobotProfile.CameraBackend.WEBCAM
                ? WebcamName.class : Limelight3A.class;
    }

    /** Snapshots raw facts without activating optional device or estimator configurations. */
    private static CalibrationRobotProfile capture(CalibrationRobotProfile profile) {
        CalibrationRobotProfile captured = Objects.requireNonNull(profile,
                "CalibrationRobotProfile must not be null; start with current()").copy();
        Objects.requireNonNull(captured.cameraBackend,
                "CalibrationRobotProfile.cameraBackend must be NONE, WEBCAM, or LIMELIGHT");
        return captured;
    }

    /** Captures an optional vision request and rejects NONE before a camera Config is constructed. */
    private static CalibrationRobotProfile captureVision(CalibrationRobotProfile profile) {
        CalibrationRobotProfile captured = capture(profile);
        requireVision(captured);
        return captured;
    }

    /** A direct optional factory call must not quietly become a different workflow. */
    private static void requireVision(CalibrationRobotProfile captured) {
        if (captured.cameraBackend == CalibrationRobotProfile.CameraBackend.NONE) {
            throw new IllegalStateException("Select cameraBackend WEBCAM or LIMELIGHT and author "
                    + "its hardware name and cameraMount before opening a vision check");
        }
    }

    /** Enforces the same motion permission for menu selections and direct factory calls. */
    private static void requireReviewedMotion(CalibrationRobotProfile captured) {
        if (!captured.poweredMotionReviewed) {
            throw new IllegalStateException("Powered pod offsets require poweredMotionReviewed; "
                    + "review motor names/directions, commands, clearance, and the FTC STOP plan first");
        }
    }
}
