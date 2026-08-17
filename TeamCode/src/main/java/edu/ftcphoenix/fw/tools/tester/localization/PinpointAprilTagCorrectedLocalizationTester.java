package edu.ftcphoenix.fw.tools.tester.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.Collections;
import java.util.Objects;
import java.util.Set;
import java.util.function.Function;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.field.TagLayouts;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.FtcTagLayoutDebug;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.localization.LimelightFieldPoseEstimator;
import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcphoenix.fw.ftc.ui.HardwareNamePicker;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactory;
import edu.ftcphoenix.fw.ftc.vision.VisionReadiness;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.apriltag.AprilTagPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.CorrectedPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.CorrectionStats;
import edu.ftcphoenix.fw.localization.fusion.OdometryCorrectionEkfEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionPolicies;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelections;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;

/**
 * End-to-end tester for Phoenix's Pinpoint plus corrected-global-localization stack.
 *
 * <p>The tester owns one configured Pinpoint predictor and one freshly opened backend-neutral
 * AprilTag vision lane. It presents the raw predictor, raw AprilTag solve, optional direct
 * Limelight solve, selected correction source, and corrected global pose without exporting either
 * hardware owner.</p>
 *
 * <p>Configuration and fixed-field facts are captured in the Java constructor before a picker,
 * hardware map, factory open, Pinpoint reset, telemetry, or Panels effect. A preferred vision name
 * also captures its deferred factory there. The function supplied to the constructor must only
 * bind the normalized name to an effect-free deferred factory; it must not look up hardware or
 * open an FTC resource. Any backend config or custom SDK library borrowed by that function must
 * remain stable for this tester's full lifetime and every possible retry.</p>
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>PICKER</b>: Dpad Up/Down highlight, A choose, X refresh</li>
 *   <li><b>RUN</b>:
 *     <ul>
 *       <li>START: toggle raw-tag preview between every fixed-layout ID and one chosen ID</li>
 *       <li>Dpad Left/Right or Y/X: change the chosen preview tag ID</li>
 *       <li>A: snap corrected pose to the current active correction pose when available</li>
 *       <li>B: toggle absolute correction while prediction remains active</li>
 *       <li>RB: rebase the corrected/global estimator to field pose (0,0,0)</li>
 *       <li>BACK: cleanly close the current vision owner and return to the picker</li>
 *     </ul>
 *   </li>
 * </ul>
 */
public final class PinpointAprilTagCorrectedLocalizationTester extends BaseTeleOpTester {

    /** Complete data-only configuration for one corrected-localization tester owner. */
    public static final class Config {
        /** Preferred FTC hardware name, or null to require operator selection. */
        public String preferredVisionDeviceName = null;

        /** Hardware type enumerated by the retained replacement picker. */
        public Class<? extends HardwareDevice> visionDeviceType = WebcamName.class;

        /** Nonblank picker heading shown to the operator. */
        public String visionPickerTitle = "Select Camera";

        /** Trusted fixed-field tag facts; captured immediately by the tester. */
        public TagLayout fixedTagLayout = FtcGameTagLayout.currentGameFieldFixed();

        /** Complete production-shaped Pinpoint and corrected-estimator policy. */
        public FtcOdometryAprilTagLocalizationLane.Config localization =
                FtcOdometryAprilTagLocalizationLane.Config.defaults();

        private Config() {
        }

        /** @return a fresh mutable draft populated with framework tool defaults. */
        public static Config defaults() {
            return new Config();
        }
    }

    private static final String CONFIG_CONTEXT =
            PinpointAprilTagCorrectedLocalizationTester.class.getCanonicalName() + ".Config";
    private static final int DEFAULT_TAG_ID = 1;

    private final String preferredVisionDeviceName;
    private final Class<? extends HardwareDevice> visionDeviceType;
    private final String visionPickerTitle;
    private final Function<String, AprilTagVisionLaneFactory> visionLaneFactoryBuilder;
    private final TagLayout fixedTagLayout;
    private final String fixedTagLayoutPolicySummary;
    private final FtcOdometryAprilTagLocalizationLane.Config localizationConfig;

    /** Non-null only for the preferred path between construction and its first open attempt. */
    private AprilTagVisionLaneFactory pendingVisionFactory;
    private HardwareNamePicker visionPicker;
    private String selectedVisionDeviceName;

    private boolean ready;
    private boolean visionClosingOrTerminal;
    private boolean visionTerminalRequested;
    private boolean visionCleanupFailed;
    private RuntimeException visionFailure;
    private String initError;
    private String activeVisionDescription;
    private VisionReadiness visionReadiness =
            VisionReadiness.notReady("No vision device is open");

    private AprilTagVisionLane visionLane;
    private CameraMountConfig activeCameraMount;
    private AprilTagSensor tagSensor;
    private TagSelectionSource selection;
    private FtcOdometryAprilTagLocalizationLane localizationLane;

    private boolean trackAny = true;
    private int selectedTagId = DEFAULT_TAG_ID;

    /**
     * Creates one corrected-localization tester from a complete owner Config and one explicit
     * backend behavior dependency.
     *
     * <p>The constructor snapshots all mutable active Config data and the fixed layout before
     * invoking {@code visionLaneFactoryBuilder}. When a preferred name is configured, the builder
     * is applied exactly once here and the returned deferred factory is opened later from
     * {@link #onInit()}. With no preferred name, the builder is first applied after the operator
     * confirms a picker choice. Each retry applies it again for the normalized selected name.</p>
     *
     * @param config complete mutable authoring draft; non-null and defensively captured
     * @param visionLaneFactoryBuilder effect-free name-to-deferred-factory behavior; non-null
     * @throws NullPointerException for a required null Config, field, or behavior peer
     * @throws IllegalArgumentException for malformed active configuration or field facts
     * @throws RuntimeException if preferred-name deferred-factory capture fails
     */
    public PinpointAprilTagCorrectedLocalizationTester(
            Config config,
            Function<String, AprilTagVisionLaneFactory> visionLaneFactoryBuilder) {
        Config authored = Objects.requireNonNull(config, CONFIG_CONTEXT + " must not be null");

        String preferred = authored.preferredVisionDeviceName;
        if (preferred != null) {
            String authoredPreferred = preferred;
            preferred = preferred.trim();
            if (preferred.isEmpty()) {
                throw new IllegalArgumentException(
                        CONFIG_CONTEXT
                                + ".preferredVisionDeviceName must be null or contain a "
                                + "non-whitespace FTC hardware name; received '"
                                + authoredPreferred + "'"
                );
            }
        }
        Class<? extends HardwareDevice> deviceType = Objects.requireNonNull(
                authored.visionDeviceType,
                CONFIG_CONTEXT + ".visionDeviceType must not be null"
        );
        String authoredPickerTitle = Objects.requireNonNull(
                authored.visionPickerTitle,
                CONFIG_CONTEXT + ".visionPickerTitle must not be null"
        );
        String pickerTitle = authoredPickerTitle.trim();
        if (pickerTitle.isEmpty()) {
            throw new IllegalArgumentException(
                    CONFIG_CONTEXT + ".visionPickerTitle must contain non-whitespace text; received '"
                            + authoredPickerTitle + "'"
            );
        }

        TagLayout authoredLayout = Objects.requireNonNull(
                authored.fixedTagLayout,
                CONFIG_CONTEXT + ".fixedTagLayout must not be null"
        );
        String policySummary = authoredLayout instanceof FtcGameTagLayout
                ? ((FtcGameTagLayout) authoredLayout).policySummaryLine()
                : null;
        TagLayout layoutSnapshot = snapshotLayout(authoredLayout);
        FtcOdometryAprilTagLocalizationLane.Config localizationSnapshot =
                Objects.requireNonNull(
                        authored.localization,
                        CONFIG_CONTEXT + ".localization must not be null"
                ).validatedCopy(CONFIG_CONTEXT + ".localization");
        Function<String, AprilTagVisionLaneFactory> requiredBuilder = Objects.requireNonNull(
                visionLaneFactoryBuilder,
                PinpointAprilTagCorrectedLocalizationTester.class.getCanonicalName()
                        + ".visionLaneFactoryBuilder must not be null"
        );

        this.preferredVisionDeviceName = preferred;
        this.visionDeviceType = deviceType;
        this.visionPickerTitle = pickerTitle;
        this.fixedTagLayout = layoutSnapshot;
        this.fixedTagLayoutPolicySummary = policySummary;
        this.localizationConfig = localizationSnapshot;
        this.visionLaneFactoryBuilder = requiredBuilder;
        this.selectedVisionDeviceName = preferred;
        this.pendingVisionFactory = preferred == null
                ? null
                : requireDeferredFactory(requiredBuilder.apply(preferred), preferred);
    }

    @Override
    public String name() {
        return localizationConfig.estimation.correctedEstimatorMode
                == FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.EKF
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

        if (preferredVisionDeviceName != null) {
            visionPicker.setPreferredName(preferredVisionDeviceName);
            openSelectedVision();
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
                    openSelectedVision();
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
            if (localizationLane == null) {
                return;
            }
            PoseEstimate correction = localizationLane.correctionEstimator().getEstimate();
            if (correction != null && correction.hasPose) {
                localizationLane.globalEstimator().setPose(correction.toPose2d());
            }
        });

        liveControls.onRise(gamepads.p1().rightBumper(), () -> {
            if (localizationLane != null) {
                localizationLane.globalEstimator().setPose(Pose2d.zero());
            }
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
        pendingVisionFactory = null;
        activeCameraMount = null;
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

    private String correctionSourceLabel() {
        switch (localizationConfig.estimation.correctionSource.mode) {
            case LIMELIGHT_FIELD_POSE:
                return "Pinpoint + Direct Limelight";
            case APRILTAG_POSE:
            default:
                return "Pinpoint + AprilTag Corrections";
        }
    }

    /** Apply/open/setup one selected backend while retaining exact cleanup provenance. */
    private void openSelectedVision() {
        if (visionLane != null) {
            refreshVisionReadiness();
            return;
        }
        if (visionClosingOrTerminal || visionCleanupFailed) {
            return;
        }
        if (selectedVisionDeviceName == null) {
            initError = "No vision device selected";
            return;
        }
        String normalizedSelectedName = selectedVisionDeviceName.trim();
        if (normalizedSelectedName.isEmpty()) {
            initError = "Selected vision device name must contain non-whitespace text; received '"
                    + selectedVisionDeviceName + "'";
            return;
        }
        selectedVisionDeviceName = normalizedSelectedName;

        visionFailure = null;
        boolean openCalled = false;
        boolean ownerPublished = false;
        try {
            AprilTagVisionLaneFactory factory = pendingVisionFactory;
            pendingVisionFactory = null;
            if (factory == null) {
                factory = requireDeferredFactory(
                        visionLaneFactoryBuilder.apply(selectedVisionDeviceName),
                        selectedVisionDeviceName
                );
            }

            openCalled = true;
            AprilTagVisionLane opened = factory.open(ctx.hw);
            if (opened == null) {
                throw new IllegalStateException(
                        "vision lane factory returned null for " + selectedVisionDeviceName
                );
            }
            visionLane = opened;
            ownerPublished = true;

            activeVisionDescription = factory.description();
            localizationLane = new FtcOdometryAprilTagLocalizationLane(
                    ctx.hw,
                    visionLane,
                    fixedTagLayout,
                    localizationConfig
            );
            tagSensor = Objects.requireNonNull(
                    visionLane.tagSensor(),
                    "visionLane.tagSensor() must not return null"
            );
            activeCameraMount = Objects.requireNonNull(
                    visionLane.cameraMountConfig(),
                    "visionLane.cameraMountConfig() must not return null"
            );
            rebuildSelection();

            ready = false;
            visionReadiness = VisionReadiness.notReady("Vision device is opening");
            initError = null;
            refreshVisionReadiness();
        } catch (RuntimeException failure) {
            boolean unpublishedCleanupUncertain = openCalled
                    && !ownerPublished
                    && failure.getSuppressed().length > 0;
            clearLocalizationGraph();
            ready = false;
            visionReadiness = VisionReadiness.notReady("Vision initialization failed");
            RuntimeException cleanupFailure = closeVisionLaneOnce();
            visionFailure = failure;
            if (cleanupFailure != null) {
                if (cleanupFailure != failure) {
                    failure.addSuppressed(cleanupFailure);
                }
                visionCleanupFailed = true;
            } else if (unpublishedCleanupUncertain) {
                visionCleanupFailed = true;
            } else if (!visionTerminalRequested) {
                visionClosingOrTerminal = false;
                resetVisionPickerChoice();
            }
            initError = visionFailureMessage(failure);
        }
    }

    /** Poll one published owner; null and throwing readiness are contract failures, not WAITING. */
    private void refreshVisionReadiness() {
        AprilTagVisionLane lane = visionLane;
        if (lane == null || visionClosingOrTerminal || visionCleanupFailed) {
            ready = false;
            return;
        }
        try {
            VisionReadiness current = lane.readiness(clock);
            if (current == null) {
                throw new IllegalStateException(
                        "visionLane.readiness(clock) must not return null"
                );
            }
            visionReadiness = current;
            ready = current.isReady();
        } catch (RuntimeException failure) {
            ready = false;
            visionReadiness = VisionReadiness.notReady("Vision readiness check failed");
            clearLocalizationGraph();
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

    private void resetToPicker() {
        ready = false;
        visionReadiness = VisionReadiness.notReady("No vision device is open");
        initError = null;
        pendingVisionFactory = null;
        clearLocalizationGraph();
        RuntimeException cleanupFailure = closeVisionLaneOnce();
        if (cleanupFailure != null) {
            blockVisionSelection(cleanupFailure);
        } else if (!visionTerminalRequested) {
            visionClosingOrTerminal = false;
            resetVisionPickerChoice();
        }
    }

    private void clearLocalizationGraph() {
        activeCameraMount = null;
        tagSensor = null;
        selection = null;
        localizationLane = null;
        activeVisionDescription = null;
    }

    private void resetVisionPickerChoice() {
        if (visionPicker == null) {
            return;
        }
        visionPicker.clearChoice();
        visionPicker.refresh();
        if (selectedVisionDeviceName != null) {
            visionPicker.setPreferredName(selectedVisionDeviceName);
        }
    }

    /** Detach before close so repeated and reentrant lifecycle callbacks cannot close twice. */
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

    private void blockVisionSelection(RuntimeException cleanupFailure) {
        visionCleanupFailed = true;
        visionFailure = cleanupFailure;
        initError = visionFailureMessage(cleanupFailure);
    }

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
        if (tagSensor == null) {
            selection = null;
            return;
        }

        Set<Integer> fixedIds = fixedTagLayout.ids();
        Set<Integer> previewIds = trackAny && !fixedIds.isEmpty()
                ? fixedIds
                : Collections.singleton(selectedTagId);
        selection = TagSelections.from(tagSensor)
                .among(previewIds)
                .freshWithinSec(aprilTagMaxAgeSec())
                .choose(TagSelectionPolicies.closestRange())
                .continuous()
                .build();
    }

    private double aprilTagMaxAgeSec() {
        return localizationConfig.estimation.aprilTags.maxDetectionAgeSec;
    }

    private void incrementSelectedTagId() {
        selectedTagId++;
        if (!trackAny || fixedTagLayout.ids().isEmpty()) {
            rebuildSelection();
        }
    }

    private void decrementSelectedTagId() {
        selectedTagId = Math.max(1, selectedTagId - 1);
        if (!trackAny || fixedTagLayout.ids().isEmpty()) {
            rebuildSelection();
        }
    }

    private void renderPicker() {
        Telemetry telemetry = ctx.telemetry;
        telemetry.clearAll();
        telemetry.addLine("=== " + name() + " ===");

        if (initError != null) {
            telemetry.addLine("Init error:");
            telemetry.addLine(initError);
            telemetry.addLine("");
        }

        visionPicker.render(telemetry);
        telemetry.addLine("");
        telemetry.addLine("Chosen: " + (selectedVisionDeviceName == null
                ? "(none)"
                : selectedVisionDeviceName));
        if (visionLane != null) {
            telemetry.addData("Vision readiness", visionReadiness.isReady() ? "READY" : "WAITING");
            telemetry.addData("Vision status", visionReadiness.reason());
            telemetry.addLine("Press BACK to close this owner and choose another device.");
        }
        if (visionCleanupFailed) {
            telemetry.addLine("VISION DEVICE SELECTION DISABLED.");
            telemetry.addLine("Stop and restart this OpMode before selecting another device.");
        } else if (visionLane == null) {
            telemetry.addLine("Press A to choose the active vision device and initialize localization.");
            telemetry.addLine("Press X to refresh the device list.");
            telemetry.addLine("Press BACK to exit to the tester menu.");
        }
        telemetry.update();
    }

    private void renderInternalError(String message) {
        Telemetry telemetry = ctx.telemetry;
        telemetry.clearAll();
        telemetry.addLine("=== " + name() + " ===");
        telemetry.addLine("ERROR: " + message);
        telemetry.addLine("");
        telemetry.addLine("Try: BACK -> device picker, then choose a device again.");
        telemetry.addLine("If the problem persists, verify hardware configuration and wiring.");
        telemetry.update();
    }

    private void updateAndRender() {
        if (selection == null || localizationLane == null) {
            renderInternalError("Localization pipeline not initialized");
            return;
        }

        localizationLane.update(clock);

        MotionPredictor predictor = localizationLane.predictor();
        AprilTagPoseEstimator aprilTagEstimator = localizationLane.aprilTagPoseEstimator();
        LimelightFieldPoseEstimator limelightEstimator =
                localizationLane.limelightFieldPoseEstimator();
        AbsolutePoseEstimator correctionEstimator = localizationLane.correctionEstimator();
        CorrectedPoseEstimator globalEstimator = localizationLane.globalEstimator();

        PoseEstimate predictorPose = predictor.getEstimate();
        PoseEstimate aprilTagPose = aprilTagEstimator.getEstimate();
        PoseEstimate limelightPose = limelightEstimator != null
                ? limelightEstimator.getEstimate()
                : null;
        PoseEstimate correctionPose = correctionEstimator.getEstimate();
        PoseEstimate globalPose = globalEstimator.getEstimate();
        CorrectionStats stats = globalEstimator.getCorrectionStats();

        TagSelectionResult selectionResult = selection.get(clock);
        AprilTagObservation observation = selectionResult.hasFreshSelectedObservation
                ? selectionResult.selectedObservation
                : AprilTagObservation.noTarget();

        Telemetry telemetry = ctx.telemetry;
        telemetry.clearAll();
        telemetry.addLine("=== " + name() + " ===");
        telemetry.addData("Vision device", selectedVisionDeviceName);
        if (activeVisionDescription != null && !activeVisionDescription.isEmpty()) {
            telemetry.addData("Backend", activeVisionDescription);
        }
        telemetry.addData(
                "Correction source",
                localizationConfig.estimation.correctionSource.mode
        );
        telemetry.addData(
                "Global estimator",
                localizationConfig.estimation.correctedEstimatorMode
        );
        if (predictor instanceof PinpointOdometryPredictor) {
            GoBildaPinpointDriver.DeviceStatus pinpointStatus =
                    ((PinpointOdometryPredictor) predictor).lastDeviceStatus();
            telemetry.addData("Pinpoint status", pinpointStatus);
            if (pinpointStatus != GoBildaPinpointDriver.DeviceStatus.READY
                    || predictorPose == null
                    || !predictorPose.hasPose) {
                telemetry.addLine(
                        "Keep the robot stationary until Pinpoint READY publishes a measured pose."
                );
            }
        }
        telemetry.addData("Track [START]", trackAny ? "ANY FIXED" : "SINGLE RAW PREVIEW");
        telemetry.addData("Tag ID [Dpad L/R or Y/X]", selectedTagId);
        telemetry.addData(
                "Correction [B]",
                globalEstimator.isCorrectionEnabled() ? "ENABLED" : "DISABLED"
        );
        telemetry.addData(
                "Snap to correction [A]",
                correctionPose != null && correctionPose.hasPose
                        ? "READY"
                        : "waiting for correction pose"
        );
        telemetry.addData("Zero pose [RB]", "field pose -> (0,0,0)");
        telemetry.addData("AprilTag MaxAge", "%.0f ms", aprilTagMaxAgeSec() * 1000.0);

        if (limelightEstimator != null) {
            telemetry.addData(
                    "Direct Limelight mode",
                    localizationConfig.estimation.correctionSource.limelightFieldPose.mode
            );
        }
        if (fixedTagLayoutPolicySummary != null) {
            telemetry.addData("Layout policy", fixedTagLayoutPolicySummary);
        } else {
            telemetry.addData("Layout ids", fixedTagLayout.ids());
        }
        if (fixedTagLayout.ids().isEmpty()) {
            telemetry.addLine(
                    "No fixed tags are configured: raw vision and Pinpoint remain diagnostic, "
                            + "but AprilTag field-pose correction cannot solve."
            );
        }

        if (isLikelyIdentity(activeCameraMount)) {
            telemetry.addLine("");
            telemetry.addLine("NOTE: CameraMountConfig still looks like the identity placeholder.");
            telemetry.addLine("Run 'Calib: Camera Mount' before judging global localization quality.");
        }
        if (isLikelyDefaultPinpointOffsets(localizationConfig.predictor)) {
            telemetry.addLine("");
            telemetry.addLine("NOTE: Pinpoint pod offsets still look like the default 0 / 0 values.");
            telemetry.addLine("Run 'Calib: Pinpoint Pod Offsets' before trusting corrected pose.");
        }

        telemetry.addLine("");
        telemetry.addLine("Selected tag preview:");
        if (observation == null || !observation.hasTarget) {
            telemetry.addLine("  No fresh selected-tag observation.");
            telemetry.addLine("  Tips: check lighting, focus, layout coverage, and tag visibility.");
        } else {
            telemetry.addData("  Tag id", observation.id);
            telemetry.addData("  Age", "%.0f ms", observation.frameAgeSec(clock) * 1000.0);
            telemetry.addData("  Range", "%.1f in", observation.cameraRangeInches());
            telemetry.addData(
                    "  Bearing",
                    "%.1f°",
                    Math.toDegrees(observation.cameraBearingRad())
            );
            telemetry.addData(
                    "  cameraToTag",
                    "fwd=%.1f in | left=%.1f in | up=%.1f in",
                    observation.cameraForwardInches(),
                    observation.cameraLeftInches(),
                    observation.cameraUpInches()
            );
        }

        telemetry.addLine("");
        telemetry.addLine("Pose estimates:");
        renderPose(telemetry, "  Predictor", predictorPose);
        renderPose(telemetry, "  AprilTag solve", aprilTagPose);
        if (limelightEstimator != null) {
            renderPose(telemetry, "  Limelight direct", limelightPose);
        }
        renderPose(telemetry, "  Active correction", correctionPose);
        renderPose(
                telemetry,
                localizationConfig.estimation.correctedEstimatorMode
                        == FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.EKF
                        ? "  EKF"
                        : "  Corrected",
                globalPose
        );

        if (globalPose != null && globalPose.hasPose && globalPose.fieldToRobotPose != null) {
            Pose3d pose = globalPose.fieldToRobotPose;
            telemetry.addData(
                    "  Global summary",
                    "x=%.1f y=%.1f | yaw=%.1f°",
                    pose.xInches,
                    pose.yInches,
                    Math.toDegrees(pose.yawRad)
            );
        }

        telemetry.addLine("");
        telemetry.addLine("Correction stats:");
        telemetry.addData(
                "  Accept / reject",
                "%d / %d",
                stats.acceptedCorrectionCount,
                stats.rejectedCorrectionCount
        );
        telemetry.addData(
                "  Replay / projected",
                "%d / %d",
                stats.replayedCorrectionCount,
                stats.projectedCorrectionCount
        );
        telemetry.addData(
                "  Skip dup / old",
                "%d / %d",
                stats.skippedDuplicateCorrectionCount,
                stats.skippedOutOfOrderCorrectionCount
        );

        if (globalEstimator instanceof OdometryCorrectionEkfEstimator) {
            OdometryCorrectionEkfEstimator ekf =
                    (OdometryCorrectionEkfEstimator) globalEstimator;
            telemetry.addLine("");
            telemetry.addLine("EKF diagnostics:");
            telemetry.addData(
                    "  EKF Std",
                    "pos=%.2f in | head=%.2f°",
                    ekf.getPositionStdIn(),
                    Math.toDegrees(ekf.getHeadingStdRad())
            );
            telemetry.addData(
                    "  EKF Innov",
                    "pos=%.2f in | head=%.2f° | maha=%.2f",
                    ekf.getLastInnovationPositionIn(),
                    Math.toDegrees(ekf.getLastInnovationHeadingRad()),
                    ekf.getLastInnovationMahalanobisSq()
            );
            telemetry.addData(
                    "  EKF Meas σ",
                    "pos=%.2f in | head=%.2f°",
                    ekf.getLastMeasurementPositionStdIn(),
                    Math.toDegrees(ekf.getLastMeasurementHeadingStdRad())
            );
        }

        telemetry.addLine("");
        telemetry.addLine("Layout summary:");
        FtcTagLayoutDebug.dumpSummary(
                fixedTagLayout,
                new FtcTelemetryDebugSink(telemetry),
                "layout"
        );

        telemetry.addLine("");
        telemetry.addLine("BACK: return to the vision-device picker.");
        telemetry.update();
    }

    private static AprilTagVisionLaneFactory requireDeferredFactory(
            AprilTagVisionLaneFactory factory,
            String selectedName) {
        if (factory == null) {
            throw new IllegalStateException(
                    PinpointAprilTagCorrectedLocalizationTester.class.getCanonicalName()
                            + ".visionLaneFactoryBuilder returned null for normalized device name '"
                            + selectedName + "'"
            );
        }
        return factory;
    }

    private static TagLayout snapshotLayout(TagLayout layout) {
        try {
            return TagLayouts.snapshot(layout);
        } catch (RuntimeException failure) {
            throw new IllegalArgumentException(
                    CONFIG_CONTEXT + ".fixedTagLayout is invalid: "
                            + String.valueOf(failure.getMessage()),
                    failure
            );
        }
    }

    private static boolean isLikelyIdentity(CameraMountConfig mount) {
        if (mount == null) {
            return true;
        }
        Pose3d pose = mount.robotToCameraPose();
        double tolerance = 1e-6;
        return Math.abs(pose.xInches) < tolerance
                && Math.abs(pose.yInches) < tolerance
                && Math.abs(pose.zInches) < tolerance
                && Math.abs(pose.rollRad) < tolerance
                && Math.abs(pose.pitchRad) < tolerance
                && Math.abs(pose.yawRad) < tolerance;
    }

    private static boolean isLikelyDefaultPinpointOffsets(PinpointOdometryPredictor.Config config) {
        if (config == null) {
            return true;
        }
        double tolerance = 1e-6;
        return Math.abs(config.forwardPodOffsetLeftInches) < tolerance
                && Math.abs(config.strafePodOffsetForwardInches) < tolerance;
    }

    private static void renderPose(Telemetry telemetry, String label, PoseEstimate estimate) {
        if (estimate == null || !estimate.hasPose || estimate.fieldToRobotPose == null) {
            telemetry.addData(label, "no pose");
            return;
        }

        Pose3d pose = estimate.fieldToRobotPose;
        telemetry.addData(
                label,
                "x=%.1f in, y=%.1f in, h=%.1f deg | q=%.2f",
                pose.xInches,
                pose.yInches,
                Math.toDegrees(pose.yawRad),
                estimate.quality
        );
    }
}
