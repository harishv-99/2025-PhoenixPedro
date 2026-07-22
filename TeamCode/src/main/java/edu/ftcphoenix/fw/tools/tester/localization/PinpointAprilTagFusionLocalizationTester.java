package edu.ftcphoenix.fw.tools.tester.localization;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.Collections;
import java.util.Set;
import java.util.function.Function;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.FtcTagLayoutDebug;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.localization.LimelightFieldPoseEstimator;
import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactories;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactory;
import edu.ftcphoenix.fw.ftc.vision.FtcWebcamAprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.VisionReadiness;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.apriltag.AprilTagPoseEstimator;
import edu.ftcphoenix.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcphoenix.fw.localization.fusion.CorrectedPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.CorrectionStats;
import edu.ftcphoenix.fw.localization.fusion.OdometryCorrectionEkfEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryCorrectionFusionEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionPolicies;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelections;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.ftc.ui.HardwareNamePicker;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * End-to-end tester for Phoenix's corrected-global-localization stack.
 *
 * <p>This tester intentionally mirrors the framework lane design:</p>
 * <ul>
 *   <li>{@link PinpointOdometryPredictor} provides short-term motion prediction.</li>
 *   <li>{@link AprilTagPoseEstimator} always remains available as the raw AprilTag absolute-pose view.</li>
 *   <li>When the active backend is Limelight, {@link LimelightFieldPoseEstimator} may also be available
 *       as a direct absolute-pose source.</li>
 *   <li>{@link CorrectedPoseEstimator} combines the predictor with the selected absolute correction source.</li>
 * </ul>
 *
 * <p>The tester is therefore useful for two distinct jobs:</p>
 * <ul>
 *   <li>Validate that the raw AprilTag solve still looks sane.</li>
 *   <li>Validate that the chosen corrected/global estimator behaves well while the robot keeps moving
 *       and tags appear / disappear.</li>
 * </ul>
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>PICKER</b>: Dpad Up/Down highlight, A choose, X refresh</li>
 *   <li><b>RUN</b>:
 *     <ul>
 *       <li>START: toggle tracking mode (ANY tag in layout vs SINGLE chosen ID)</li>
 *       <li>Dpad Left/Right: decrement/increment the chosen tag ID (used in SINGLE mode)</li>
 *       <li>Y/X: alias for tag ID decrement/increment</li>
 *       <li>A: snap the corrected/global estimator to the current active correction pose (when available)</li>
 *       <li>B: toggle absolute corrections on/off while leaving prediction alive</li>
 *       <li>RB: set the corrected/global estimator pose to (0,0,0)</li>
 *       <li>BACK: return to the vision-device picker</li>
 *     </ul>
 *   </li>
 * </ul>
 */
public final class PinpointAprilTagFusionLocalizationTester extends BaseTeleOpTester {

    /**
     * Which corrected/global estimator implementation the tester should exercise.
     */
    public enum EstimatorMode {
        FUSION,
        EKF
    }

    private static final double DEFAULT_MAX_AGE_SEC = 0.35;
    private static final int DEFAULT_TAG_ID = 1;

    private final String preferredVisionDeviceName;
    private final Class<? extends HardwareDevice> visionDeviceType;
    private final String visionPickerTitle;
    private final Function<String, AprilTagVisionLaneFactory> visionLaneFactoryBuilder;
    private final FtcOdometryAprilTagLocalizationLane.Config localizationConfigTemplate;
    private final TagLayout layoutOverride;

    private HardwareNamePicker visionPicker;
    private String selectedVisionDeviceName = null;

    private boolean ready = false;
    private boolean visionClosingOrTerminal = false;
    private boolean visionTerminalRequested = false;
    private boolean visionCleanupFailed = false;
    private RuntimeException visionFailure = null;
    private String initError = null;
    private String activeVisionDescription = null;
    private VisionReadiness visionReadiness = VisionReadiness.notReady("No vision device is open");

    private TagLayout layout;
    private AprilTagVisionLane visionLane;
    private AprilTagSensor tagSensor;
    private TagSelectionSource selection;
    private FtcOdometryAprilTagLocalizationLane localizationLane;

    private boolean trackAny = true;
    private int selectedTagId = DEFAULT_TAG_ID;

    /**
     * Creates the tester with the default lightweight corrected estimator configuration.
     */
    public PinpointAprilTagFusionLocalizationTester(String preferredCameraName,
                                                    CameraMountConfig cameraMount,
                                                    PinpointOdometryPredictor.Config pinpointCfg) {
        this(preferredCameraName,
                cameraMount,
                pinpointCfg,
                OdometryCorrectionFusionEstimator.Config.defaults(),
                null,
                null,
                null);
    }

    /**
     * Creates the tester with an explicit lightweight corrected-estimator configuration.
     */
    public PinpointAprilTagFusionLocalizationTester(String preferredCameraName,
                                                    CameraMountConfig cameraMount,
                                                    PinpointOdometryPredictor.Config pinpointCfg,
                                                    OdometryCorrectionFusionEstimator.Config fusionCfg) {
        this(preferredCameraName,
                cameraMount,
                pinpointCfg,
                fusionCfg,
                null,
                null,
                null);
    }

    /**
     * Creates the tester with explicit lightweight corrected-estimator configuration plus optional
     * field-layout/library and raw AprilTag-estimator overrides.
     *
     * <p>These convenience overloads remain useful for webcam-backed bring-up, but the more general
     * backend-neutral constructor that takes a full
     * {@link FtcOdometryAprilTagLocalizationLane.Config} is now the preferred API.</p>
     */
    public PinpointAprilTagFusionLocalizationTester(String preferredCameraName,
                                                    CameraMountConfig cameraMount,
                                                    PinpointOdometryPredictor.Config pinpointCfg,
                                                    OdometryCorrectionFusionEstimator.Config fusionCfg,
                                                    TagLayout layoutOverride,
                                                    AprilTagLibrary tagLibraryOverride,
                                                    AprilTagPoseEstimator.Config aprilTagPoseOverride) {
        this(preferredCameraName,
                WebcamName.class,
                "Select Camera",
                defaultWebcamLaneFactoryBuilder(cameraMount, tagLibraryOverride),
                buildLocalizationConfig(pinpointCfg,
                        EstimatorMode.FUSION,
                        fusionCfg,
                        null,
                        aprilTagPoseOverride),
                layoutOverride);
    }

    /**
     * Creates the tester in EKF mode with the supplied configuration and optional overrides.
     */
    public static PinpointAprilTagFusionLocalizationTester ekf(String preferredCameraName,
                                                               CameraMountConfig cameraMount,
                                                               PinpointOdometryPredictor.Config pinpointCfg,
                                                               OdometryCorrectionEkfEstimator.Config ekfCfg,
                                                               TagLayout layoutOverride,
                                                               AprilTagLibrary tagLibraryOverride,
                                                               AprilTagPoseEstimator.Config aprilTagPoseOverride) {
        return new PinpointAprilTagFusionLocalizationTester(preferredCameraName,
                WebcamName.class,
                "Select Camera",
                defaultWebcamLaneFactoryBuilder(cameraMount, tagLibraryOverride),
                buildLocalizationConfig(pinpointCfg,
                        EstimatorMode.EKF,
                        null,
                        ekfCfg,
                        aprilTagPoseOverride),
                layoutOverride);
    }

    /**
     * Creates the tester in EKF mode with the supplied configuration.
     */
    public static PinpointAprilTagFusionLocalizationTester ekf(String preferredCameraName,
                                                               CameraMountConfig cameraMount,
                                                               PinpointOdometryPredictor.Config pinpointCfg,
                                                               OdometryCorrectionEkfEstimator.Config ekfCfg) {
        return ekf(preferredCameraName, cameraMount, pinpointCfg, ekfCfg, null, null, null);
    }

    /**
     * Creates a backend-neutral corrected-localization tester from a full localization-lane config.
     *
     * <p>This is the preferred constructor for robot projects because it mirrors production wiring:
     * one active AprilTag vision backend plus one shared
     * {@link FtcOdometryAprilTagLocalizationLane.Config} describing predictor, correction-source,
     * and corrected-estimator policy.</p>
     */
    public PinpointAprilTagFusionLocalizationTester(String preferredVisionDeviceName,
                                                    Class<? extends HardwareDevice> visionDeviceType,
                                                    String visionPickerTitle,
                                                    Function<String, AprilTagVisionLaneFactory> visionLaneFactoryBuilder,
                                                    FtcOdometryAprilTagLocalizationLane.Config localizationConfig,
                                                    TagLayout layoutOverride) {
        this.preferredVisionDeviceName = preferredVisionDeviceName;
        this.visionDeviceType = visionDeviceType != null ? visionDeviceType : WebcamName.class;
        this.visionPickerTitle = (visionPickerTitle == null || visionPickerTitle.trim().isEmpty())
                ? "Select Vision Device"
                : visionPickerTitle;
        this.visionLaneFactoryBuilder = visionLaneFactoryBuilder != null
                ? visionLaneFactoryBuilder
                : defaultWebcamLaneFactoryBuilder(CameraMountConfig.identity(), null);
        this.localizationConfigTemplate = localizationConfig != null
                ? localizationConfig.copy()
                : FtcOdometryAprilTagLocalizationLane.Config.defaults();
        this.layoutOverride = layoutOverride;
    }

    private static Function<String, AprilTagVisionLaneFactory> defaultWebcamLaneFactoryBuilder(CameraMountConfig cameraMount,
                                                                                               AprilTagLibrary tagLibraryOverride) {
        final CameraMountConfig mount = (cameraMount != null) ? cameraMount : CameraMountConfig.identity();
        return cameraName -> {
            FtcWebcamAprilTagVisionLane.Config cfg = FtcWebcamAprilTagVisionLane.Config.defaults();
            cfg.webcamName = cameraName;
            cfg.cameraMount = mount;
            cfg.tagLibrary = tagLibraryOverride;
            return AprilTagVisionLaneFactories.webcam(cfg);
        };
    }

    private static FtcOdometryAprilTagLocalizationLane.Config buildLocalizationConfig(PinpointOdometryPredictor.Config predictorCfg,
                                                                                      EstimatorMode estimatorMode,
                                                                                      OdometryCorrectionFusionEstimator.Config fusionCfg,
                                                                                      OdometryCorrectionEkfEstimator.Config ekfCfg,
                                                                                      AprilTagPoseEstimator.Config aprilTagPoseOverride) {
        FtcOdometryAprilTagLocalizationLane.Config cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.predictor = predictorCfg != null
                ? predictorCfg.copy()
                : PinpointOdometryPredictor.Config.defaults();
        cfg.correctedEstimatorMode = estimatorMode == EstimatorMode.EKF
                ? FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.EKF
                : FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;
        cfg.correctionFusion = fusionCfg != null
                ? fusionCfg.copy()
                : OdometryCorrectionFusionEstimator.Config.defaults();
        cfg.correctionEkf = ekfCfg != null
                ? ekfCfg.copy()
                : OdometryCorrectionEkfEstimator.Config.defaults();
        cfg.correctionSource.mode = FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.APRILTAG_POSE;

        if (aprilTagPoseOverride != null) {
            cfg.aprilTags.maxDetectionAgeSec = aprilTagPoseOverride.maxDetectionAgeSec;
            FixedTagFieldPoseSolver.Config solverCfg = aprilTagPoseOverride.toSolverConfig();
            cfg.aprilTags.fieldPoseSolver = solverCfg != null
                    ? solverCfg.copy()
                    : FixedTagFieldPoseSolver.Config.defaults();
        }
        return cfg;
    }

    private String correctionSourceLabel() {
        switch (localizationConfigTemplate.correctionSource.mode) {
            case LIMELIGHT_FIELD_POSE:
                return "Pinpoint + Direct Limelight";
            case APRILTAG_POSE:
            default:
                return "Pinpoint + AprilTag Corrections";
        }
    }

    /**
     * Returns a dynamic tester name that reflects the selected correction source and estimator mode.
     */
    @Override
    public String name() {
        return localizationConfigTemplate.correctedEstimatorMode == FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.EKF
                ? "Loc: " + correctionSourceLabel() + " (EKF)"
                : "Loc: " + correctionSourceLabel() + " (Fused)";
    }

    @Override
    protected void onInit() {
        visionPicker = new HardwareNamePicker(
                ctx.hw,
                visionDeviceType,
                visionPickerTitle,
                "Dpad: highlight | A: choose | X: refresh"
        );
        visionPicker.refresh();

        if (preferredVisionDeviceName != null && !preferredVisionDeviceName.trim().isEmpty()) {
            selectedVisionDeviceName = preferredVisionDeviceName.trim();
            visionPicker.setPreferredName(selectedVisionDeviceName);
            ensureReady();
        }

        visionPicker.bind(
                bindings,
                gamepads.p1().dpadUp(),
                gamepads.p1().dpadDown(),
                gamepads.p1().a(),
                gamepads.p1().x(),
                () -> visionLane == null && !visionClosingOrTerminal && !visionCleanupFailed,
                chosen -> {
                    selectedVisionDeviceName = chosen;
                    ensureReady();
                }
        );

        Bindings.ControlContext liveControls = bindings.contextWhen(
                BooleanSource.of(() -> ready),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL
        );

        liveControls.onRise(gamepads.p1().b(), () -> {
            if (localizationLane != null) {
                CorrectedPoseEstimator estimator = localizationLane.globalEstimator();
                estimator.setCorrectionEnabled(!estimator.isCorrectionEnabled());
            }
        });

        liveControls.onRise(gamepads.p1().start(), () -> {
            trackAny = !trackAny;
            rebuildSelection();
        });

        liveControls.onRise(gamepads.p1().dpadRight(), this::incrementSelectedTagId);
        liveControls.onRise(gamepads.p1().dpadLeft(), this::decrementSelectedTagId);
        liveControls.onRise(gamepads.p1().y(), this::incrementSelectedTagId);
        liveControls.onRise(gamepads.p1().x(), this::decrementSelectedTagId);

        liveControls.onRise(gamepads.p1().a(), () -> {
            if (localizationLane == null) return;
            PoseEstimate correction = localizationLane.correctionEstimator().getEstimate();
            if (correction != null && correction.hasPose) {
                localizationLane.globalEstimator().setPose(correction.toPose2d());
            }
        });

        liveControls.onRise(gamepads.p1().rightBumper(), () -> {
            if (localizationLane == null) return;
            localizationLane.globalEstimator().setPose(Pose2d.zero());
        });
    }

    @Override
    protected void onInitLoop(double dtSec) {
        refreshVisionReadiness();
        if (!ready) {
            renderPicker();
            return;
        }
        updateAndRender();
    }

    @Override
    protected void onLoop(double dtSec) {
        refreshVisionReadiness();
        if (!ready) {
            renderPicker();
            return;
        }
        updateAndRender();
    }

    /**
     * Returns to the vision-device picker so the same tester can be rerun against a different
     * webcam or Limelight without leaving the tester menu.
     */
    @Override
    public boolean onBackPressed() {
        if (visionClosingOrTerminal || visionCleanupFailed) {
            return true;
        }
        if (visionLane == null) {
            return false;
        }
        resetToPicker();
        return true;
    }

    @Override
    protected void onStop() {
        visionTerminalRequested = true;
        ready = false;
        visionReadiness = VisionReadiness.notReady("Vision tester is stopping");
        tagSensor = null;
        selection = null;
        localizationLane = null;
        activeVisionDescription = null;
        RuntimeException cleanupFailure = closeVisionLaneOnce();
        if (cleanupFailure != null) {
            visionCleanupFailed = true;
            visionFailure = cleanupFailure;
            throw cleanupFailure;
        }
    }

    private void resetToPicker() {
        ready = false;
        visionReadiness = VisionReadiness.notReady("No vision device is open");
        initError = null;
        layout = null;
        tagSensor = null;
        selection = null;
        localizationLane = null;
        activeVisionDescription = null;
        RuntimeException cleanupFailure = closeVisionLaneOnce();
        if (cleanupFailure != null) {
            blockVisionSelection(cleanupFailure);
        } else if (!visionTerminalRequested) {
            visionClosingOrTerminal = false;
            resetVisionPickerChoice();
        }
    }

    private void renderPicker() {
        Telemetry t = ctx.telemetry;
        t.clearAll();
        t.addLine("=== " + name() + " ===");

        if (initError != null) {
            t.addLine("Init error:");
            t.addLine(initError);
            t.addLine("");
        }

        visionPicker.render(t);
        t.addLine("");
        t.addLine("Chosen: " + (selectedVisionDeviceName == null ? "(none)" : selectedVisionDeviceName));
        if (visionLane != null) {
            t.addData("Vision readiness", visionReadiness.isReady() ? "READY" : "WAITING");
            t.addData("Vision status", visionReadiness.reason());
            t.addLine("Press BACK to close this owner and choose another device.");
        }
        if (visionCleanupFailed) {
            t.addLine("VISION DEVICE SELECTION DISABLED.");
            t.addLine("Stop and restart this OpMode before selecting another device.");
        } else if (visionLane == null) {
            t.addLine("Press A to choose the active vision device and initialize localization.");
            t.addLine("Press X to refresh the device list.");
            t.addLine("Press BACK to exit to the tester menu.");
        }
        t.update();
    }

    private void ensureReady() {
        if (visionLane != null) {
            refreshVisionReadiness();
            return;
        }
        if (visionClosingOrTerminal) return;
        if (visionCleanupFailed) return;
        if (selectedVisionDeviceName == null || selectedVisionDeviceName.trim().isEmpty()) {
            initError = "No vision device selected";
            return;
        }

        visionFailure = null;
        boolean ownerPublished = false;
        try {
            AprilTagVisionLaneFactory factory = visionLaneFactoryBuilder.apply(selectedVisionDeviceName);
            if (factory == null) {
                throw new IllegalStateException("visionLaneFactoryBuilder returned null for " + selectedVisionDeviceName);
            }

            visionLane = factory.open(ctx.hw);
            if (visionLane == null) {
                throw new IllegalStateException(
                        "vision lane factory returned null for " + selectedVisionDeviceName);
            }
            ownerPublished = true;
            tagSensor = visionLane.tagSensor();
            activeVisionDescription = factory.description();
            layout = (layoutOverride != null)
                    ? layoutOverride
                    : FtcGameTagLayout.currentGameFieldFixed();
            localizationLane = new FtcOdometryAprilTagLocalizationLane(
                    ctx.hw,
                    visionLane,
                    layout,
                    localizationConfigTemplate
            );
            rebuildSelection();

            ready = false;
            visionReadiness = VisionReadiness.notReady("Vision device is opening");
            initError = null;
            refreshVisionReadiness();
        } catch (RuntimeException e) {
            boolean unpublishedCleanupUncertain = !ownerPublished
                    && e.getSuppressed().length > 0;
            tagSensor = null;
            selection = null;
            localizationLane = null;
            activeVisionDescription = null;
            ready = false;
            visionReadiness = VisionReadiness.notReady("Vision initialization failed");
            RuntimeException cleanupFailure = closeVisionLaneOnce();
            visionFailure = e;
            if (cleanupFailure != null) {
                if (cleanupFailure != e) {
                    e.addSuppressed(cleanupFailure);
                }
                visionCleanupFailed = true;
            } else if (unpublishedCleanupUncertain) {
                // A suppressed construction rollback failure means the device may still be owned,
                // even though no lane reference was returned to this tester.
                visionCleanupFailed = true;
            } else if (!visionTerminalRequested) {
                visionClosingOrTerminal = false;
                resetVisionPickerChoice();
            }
            initError = visionFailureMessage(e);
        }
    }

    /** Refreshes asynchronous vision readiness without opening a competing device owner. */
    private void refreshVisionReadiness() {
        AprilTagVisionLane lane = visionLane;
        if (lane == null || visionClosingOrTerminal || visionCleanupFailed) {
            ready = false;
            return;
        }
        try {
            VisionReadiness current = lane.readiness(clock);
            visionReadiness = current != null
                    ? current
                    : VisionReadiness.notReady("Vision lane returned no readiness result");
            ready = visionReadiness.isReady();
        } catch (RuntimeException failure) {
            ready = false;
            visionReadiness = VisionReadiness.notReady("Vision readiness check failed");
            tagSensor = null;
            selection = null;
            localizationLane = null;
            activeVisionDescription = null;
            RuntimeException cleanupFailure = closeVisionLaneOnce();
            visionFailure = failure;
            if (cleanupFailure != null) {
                if (cleanupFailure != failure) {
                    failure.addSuppressed(cleanupFailure);
                }
                visionCleanupFailed = true;
            } else if (!visionTerminalRequested) {
                visionClosingOrTerminal = false;
                resetVisionPickerChoice();
            }
            initError = visionFailureMessage(failure);
        }
    }

    private void resetVisionPickerChoice() {
        if (visionPicker == null) {
            return;
        }
        visionPicker.clearChoice();
        visionPicker.refresh();
        if (selectedVisionDeviceName != null
                && !selectedVisionDeviceName.trim().isEmpty()) {
            visionPicker.setPreferredName(selectedVisionDeviceName);
        }
    }

    /**
     * Detaches and closes the currently owned vision lane once.
     *
     * <p>Detaching before the callback keeps reentrant and repeated shutdown paths from reaching
     * the same lane again.</p>
     *
     * @return the close failure, or {@code null} when no lane was owned or close succeeded
     */
    private RuntimeException closeVisionLaneOnce() {
        visionClosingOrTerminal = true;
        AprilTagVisionLane lane = visionLane;
        visionLane = null;
        if (lane == null) {
            return null;
        }
        try {
            lane.close();
            return null;
        } catch (RuntimeException cleanupFailure) {
            return cleanupFailure;
        }
    }

    /** Blocks further selection after cleanup leaves hardware ownership uncertain. */
    private void blockVisionSelection(RuntimeException cleanupFailure) {
        visionCleanupFailed = true;
        visionFailure = cleanupFailure;
        initError = visionFailureMessage(cleanupFailure);
    }

    /** Formats the primary failure first and retains any suppressed cleanup diagnostics. */
    private String visionFailureMessage(RuntimeException failure) {
        StringBuilder message = new StringBuilder()
                .append(failure.getClass().getSimpleName())
                .append(": ")
                .append(String.valueOf(failure.getMessage()));
        for (Throwable suppressed : failure.getSuppressed()) {
            message.append("\nCleanup also failed: ")
                    .append(suppressed.getClass().getSimpleName())
                    .append(": ")
                    .append(String.valueOf(suppressed.getMessage()));
        }
        if (visionCleanupFailed) {
            message.append("\nVision cleanup is uncertain. Stop and restart this OpMode.");
        }
        return message.toString();
    }

    private void rebuildSelection() {
        if (tagSensor == null || layout == null) {
            selection = null;
            return;
        }

        Set<Integer> ids = trackAny
                ? layout.ids()
                : Collections.singleton(selectedTagId);

        if (ids == null || ids.isEmpty()) {
            ids = Collections.singleton(selectedTagId);
            trackAny = false;
        }

        selection = TagSelections.from(tagSensor)
                .among(ids)
                .freshWithinSec(effectiveAprilTagMaxAgeSec())
                .choose(TagSelectionPolicies.closestRange())
                .continuous()
                .build();
    }

    private double effectiveAprilTagMaxAgeSec() {
        double maxAgeSec = localizationConfigTemplate.aprilTags.maxDetectionAgeSec;
        return maxAgeSec > 0.0 ? maxAgeSec : DEFAULT_MAX_AGE_SEC;
    }

    private void incrementSelectedTagId() {
        selectedTagId++;
        if (!trackAny) {
            rebuildSelection();
        }
    }

    private void decrementSelectedTagId() {
        selectedTagId = Math.max(1, selectedTagId - 1);
        if (!trackAny) {
            rebuildSelection();
        }
    }

    private void renderInternalError(String message) {
        Telemetry t = ctx.telemetry;
        t.clearAll();
        t.addLine("=== " + name() + " ===");
        t.addLine("ERROR: " + message);
        t.addLine("");
        t.addLine("Try: BACK -> device picker, then choose a device again.");
        t.addLine("If the problem persists, verify your hardware configuration and wiring.");
        t.update();
    }

    private void updateAndRender() {
        if (selection == null || localizationLane == null) {
            renderInternalError("Localization pipeline not initialized");
            return;
        }

        localizationLane.update(clock);

        MotionPredictor predictor = localizationLane.predictor();
        AprilTagPoseEstimator aprilTagEstimator = localizationLane.aprilTagPoseEstimator();
        LimelightFieldPoseEstimator limelightEstimator = localizationLane.limelightFieldPoseEstimator();
        AbsolutePoseEstimator correctionEstimator = localizationLane.correctionEstimator();
        CorrectedPoseEstimator globalEstimator = localizationLane.globalEstimator();

        PoseEstimate predictorPose = predictor.getEstimate();
        PoseEstimate aprilTagPose = aprilTagEstimator.getEstimate();
        PoseEstimate limelightPose = limelightEstimator != null ? limelightEstimator.getEstimate() : null;
        PoseEstimate correctionPose = correctionEstimator.getEstimate();
        PoseEstimate globalPose = globalEstimator.getEstimate();
        CorrectionStats stats = globalEstimator.getCorrectionStats();

        TagSelectionResult selectionResult = selection.get(clock);
        AprilTagObservation obs = selectionResult.hasFreshSelectedObservation
                ? selectionResult.selectedObservation
                : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);

        Telemetry t = ctx.telemetry;
        t.clearAll();
        t.addLine("=== " + name() + " ===");
        t.addData("Vision device", selectedVisionDeviceName);
        if (activeVisionDescription != null && !activeVisionDescription.isEmpty()) {
            t.addData("Backend", activeVisionDescription);
        }
        t.addData("Correction source", localizationConfigTemplate.correctionSource.mode);
        t.addData("Global estimator", localizationConfigTemplate.correctedEstimatorMode);
        t.addData("Track [START]", trackAny ? "ANY" : "SINGLE");
        t.addData("Tag ID [Dpad L/R or Y/X]", selectedTagId);
        t.addData("Correction [B]", globalEstimator.isCorrectionEnabled() ? "ENABLED" : "DISABLED");
        t.addData("Snap to correction [A]", (correctionPose != null && correctionPose.hasPose) ? "READY" : "waiting for correction pose");
        t.addData("Zero pose [RB]", "field pose -> (0,0,0)");
        t.addData("AprilTag MaxAge", "%.0f ms", effectiveAprilTagMaxAgeSec() * 1000.0);

        if (limelightEstimator != null) {
            t.addData("Direct Limelight mode", localizationConfigTemplate.correctionSource.limelightFieldPose.mode);
        }

        if (layout instanceof FtcGameTagLayout) {
            FtcGameTagLayout ftcLayout = (FtcGameTagLayout) layout;
            t.addData("Layout policy", ftcLayout.policySummaryLine());
        } else if (layout != null) {
            t.addData("Layout ids", layout.ids());
        }

        CameraMountConfig activeMount = visionLane != null ? visionLane.cameraMountConfig() : CameraMountConfig.identity();
        if (isLikelyIdentity(activeMount)) {
            t.addLine("");
            t.addLine("NOTE: CameraMountConfig still looks like the identity placeholder.");
            t.addLine("Run 'Calib: Camera Mount' before judging global localization quality.");
        }
        if (isLikelyDefaultPinpointOffsets(localizationConfigTemplate.predictor)) {
            t.addLine("");
            t.addLine("NOTE: Pinpoint pod offsets still look like the default 0 / 0 values.");
            t.addLine("Run 'Calib: Pinpoint Pod Offsets' before trusting the corrected pose.");
        }

        t.addLine("");
        t.addLine("Selected tag preview:");
        if (obs == null || !obs.hasTarget) {
            t.addLine("  No fresh selected-tag observation.");
            t.addLine("  Tips: check lighting, focus, layout coverage, and tag visibility.");
        } else {
            t.addData("  Tag id", obs.id);
            t.addData("  Age", "%.0f ms", obs.ageSec * 1000.0);
            t.addData("  Range", "%.1f in", obs.cameraRangeInches());
            t.addData("  Bearing", "%.1f°", Math.toDegrees(obs.cameraBearingRad()));
            t.addData("  cameraToTag", "fwd=%.1f in | left=%.1f in | up=%.1f in",
                    obs.cameraForwardInches(),
                    obs.cameraLeftInches(),
                    obs.cameraUpInches());
        }

        t.addLine("");
        t.addLine("Pose estimates:");
        renderPose(t, "  Predictor", predictorPose);
        renderPose(t, "  AprilTag solve", aprilTagPose);
        if (limelightEstimator != null) {
            renderPose(t, "  Limelight direct", limelightPose);
        }
        renderPose(t, "  Active correction", correctionPose);
        renderPose(t, localizationConfigTemplate.correctedEstimatorMode == FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.EKF
                ? "  EKF"
                : "  Corrected", globalPose);

        if (globalPose != null && globalPose.hasPose && globalPose.fieldToRobotPose != null) {
            Pose3d p = globalPose.fieldToRobotPose;
            t.addData("  Global summary", "x=%.1f y=%.1f | yaw=%.1f°",
                    p.xInches,
                    p.yInches,
                    Math.toDegrees(p.yawRad));
        }

        t.addLine("");
        t.addLine("Correction stats:");
        t.addData("  Accept / reject", "%d / %d", stats.acceptedCorrectionCount, stats.rejectedCorrectionCount);
        t.addData("  Replay / projected", "%d / %d", stats.replayedCorrectionCount, stats.projectedCorrectionCount);
        t.addData("  Skip dup / old", "%d / %d", stats.skippedDuplicateCorrectionCount, stats.skippedOutOfOrderCorrectionCount);

        if (globalEstimator instanceof OdometryCorrectionEkfEstimator) {
            OdometryCorrectionEkfEstimator ekf = (OdometryCorrectionEkfEstimator) globalEstimator;
            t.addLine("");
            t.addLine("EKF diagnostics:");
            t.addData("  EKF Std", "pos=%.2f in | head=%.2f°",
                    ekf.getPositionStdIn(),
                    Math.toDegrees(ekf.getHeadingStdRad()));
            t.addData("  EKF Innov", "pos=%.2f in | head=%.2f° | maha=%.2f",
                    ekf.getLastInnovationPositionIn(),
                    Math.toDegrees(ekf.getLastInnovationHeadingRad()),
                    ekf.getLastInnovationMahalanobisSq());
            t.addData("  EKF Meas σ", "pos=%.2f in | head=%.2f°",
                    ekf.getLastMeasurementPositionStdIn(),
                    Math.toDegrees(ekf.getLastMeasurementHeadingStdRad()));
        }

        t.addLine("");
        t.addLine("Layout summary:");
        FtcTagLayoutDebug.dumpSummary(layout, new FtcTelemetryDebugSink(t), "layout");

        t.addLine("");
        t.addLine("BACK: return to the vision-device picker.");
        t.update();
    }

    private static boolean isLikelyIdentity(CameraMountConfig mount) {
        if (mount == null) return true;
        Pose3d p = mount.robotToCameraPose();
        double tol = 1e-6;
        return Math.abs(p.xInches) < tol
                && Math.abs(p.yInches) < tol
                && Math.abs(p.zInches) < tol
                && Math.abs(p.rollRad) < tol
                && Math.abs(p.pitchRad) < tol
                && Math.abs(p.yawRad) < tol;
    }

    private static boolean isLikelyDefaultPinpointOffsets(PinpointOdometryPredictor.Config cfg) {
        if (cfg == null) return true;
        double tol = 1e-6;
        return Math.abs(cfg.forwardPodOffsetLeftInches) < tol
                && Math.abs(cfg.strafePodOffsetForwardInches) < tol;
    }

    private static void renderPose(Telemetry t, String label, PoseEstimate est) {
        if (est == null || !est.hasPose || est.fieldToRobotPose == null) {
            t.addData(label, "no pose");
            return;
        }

        Pose3d p = est.fieldToRobotPose;
        t.addData(label,
                "x=%.1f in, y=%.1f in, h=%.1f deg | q=%.2f",
                p.xInches,
                p.yInches,
                Math.toDegrees(p.yawRad),
                est.quality);
    }
}
