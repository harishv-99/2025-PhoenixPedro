package edu.ftcphoenix.fw.tools.tester.localization;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.Collections;
import java.util.Objects;
import java.util.Set;
import java.util.function.Function;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.field.TagLayouts;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.FtcTagLayoutDebug;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactory;
import edu.ftcphoenix.fw.ftc.vision.VisionReadiness;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.apriltag.AprilTagPoseEstimator;
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
 * Verifies that AprilTag-based localization is working end-to-end.
 *
 * <p>This tester is meant to answer two practical questions quickly:</p>
 * <ul>
 *   <li><b>Do we have fresh AprilTag detections?</b></li>
 *   <li><b>Does our derived field pose ({@code fieldToRobotPose}) look reasonable?</b></li>
 * </ul>
 *
 * <p>Internally this tester shares one raw {@link AprilTagSensor} across a selector (for telemetry)
 * and {@link AprilTagPoseEstimator} (for pose solving) with one owner-captured fixed-tag layout.
 * {@link FtcGameTagLayout#currentGameFieldFixed()} is the software default, not a claim that the
 * physical field or camera mount has been verified.</p>
 *
 * <h2>Camera mount calibration</h2>
 * <p>
 * Accurate localization requires the camera mount extrinsics (robot→camera pose).
 * If you have not measured these yet, run <b>Calib: Camera Mount</b> and paste the
 * printed {@link CameraMountConfig} into your RobotConfig.
 * </p>
 *
 * <h2>Selection</h2>
 * <p>
 * A {@code null} preferred hardware name shows the configured vision-device picker. A valid
 * preferred name is attempted first; a clean open/setup failure exposes that same replacement
 * picker with the failed name highlighted. Blank preferred names are rejected as configuration
 * errors.
 * </p>
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>PICKER (no camera chosen yet)</b>: Dpad Up/Down highlight, A choose, X refresh</li>
 *   <li><b>RUN (camera chosen)</b>:
 *     <ul>
 *       <li>START: toggle tracking mode (ANY tag in layout vs SINGLE chosen ID)</li>
 *       <li>Dpad Left/Right: decrement/increment the chosen tag ID (used in SINGLE mode)</li>
 *       <li>Y/X: alias for tag ID decrement/increment</li>
 *       <li>A: capture a pose sample (used to assess stability/jitter)</li>
 *       <li>B: clear captured samples</li>
 *       <li>BACK: return to camera picker (change camera)</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p>
 * When run inside a {@link edu.ftcphoenix.fw.tools.tester.TesterSuite}, BACK navigation is
 * handled via {@link edu.ftcphoenix.fw.tools.tester.TeleOpTester#onBackPressed()}.
 * </p>
 */
public final class AprilTagLocalizationTester extends BaseTeleOpTester {

    private static final double DEFAULT_MAX_AGE_SEC = 0.35;
    private static final int DEFAULT_TAG_ID = 1;

    /** Mutable, data-only authoring configuration for one AprilTag-localization owner. */
    public static final class Config {

        /** Preferred configured vision-device name, or {@code null} to show the picker. */
        public String preferredVisionDeviceName;

        /** Hardware type enumerated by the replacement picker. */
        public Class<? extends HardwareDevice> visionDeviceType;

        /** Nonblank title shown by the replacement picker. */
        public String visionPickerTitle;

        /** Fixed field-tag facts used by selection and the field-pose solve. */
        public TagLayout fixedTagLayout;

        /** Mount-free AprilTag age and fixed-tag solver policy. */
        public AprilTagLocalizationConfig aprilTags;

        private Config() {
        }

        /**
         * Returns a fresh software-valid authoring draft.
         *
         * <p>The current-game layout remains borrowed until the tester snapshots it. The 0.35 s
         * age is the maintained diagnostic default; the solver retains all shared localization
         * defaults. These values do not claim a calibrated camera mount or verified field setup.</p>
         */
        public static Config defaults() {
            Config c = new Config();
            c.preferredVisionDeviceName = null;
            c.visionDeviceType = WebcamName.class;
            c.visionPickerTitle = "Select Camera";
            c.fixedTagLayout = FtcGameTagLayout.currentGameFieldFixed();
            c.aprilTags = AprilTagLocalizationConfig.defaults();
            c.aprilTags.maxDetectionAgeSec = DEFAULT_MAX_AGE_SEC;
            return c;
        }
    }

    // Captured owner configuration
    private final String preferredVisionDeviceName;
    private final Class<? extends HardwareDevice> visionDeviceType;
    private final String visionPickerTitle;
    private final Function<String, AprilTagVisionLaneFactory> visionLaneFactoryBuilder;
    private final TagLayout layout;
    private final String layoutPolicySummary;
    private final AprilTagLocalizationConfig aprilTags;

    /** Factory captured for the initial preferred-name attempt; picker attempts replace it. */
    private AprilTagVisionLaneFactory pendingVisionLaneFactory;

    private HardwareNamePicker cameraPicker;
    private String selectedCameraName = null;

    private boolean visionReady = false;
    private boolean visionClosingOrTerminal = false;
    private boolean visionTerminalRequested = false;
    private boolean visionCleanupFailed = false;
    private RuntimeException visionFailure = null;
    private String visionInitError = null;
    private String activeVisionDescription = null;
    private VisionReadiness visionReadiness = VisionReadiness.notReady("No vision device is open");

    private AprilTagVisionLane visionLane;
    private AprilTagSensor tagSensor;
    private CameraMountConfig cameraMount;
    private TagSelectionSource selection;
    private AprilTagPoseEstimator poseEstimator;

    private boolean trackAny = true;
    private int selectedTagId = DEFAULT_TAG_ID;

    private final PoseSampleStats samples = new PoseSampleStats();

    /**
     * Creates one backend-neutral AprilTag localization owner.
     *
     * <p>The constructor defensively captures and validates all data before invoking
     * {@code visionLaneFactoryBuilder}. A non-null preferred name is trimmed and causes exactly one
     * effect-free builder application here; {@code null} selects the replacement picker. A blank
     * preferred name is invalid. The builder must only capture backend configuration and return a
     * deferred factory: it must not inspect the hardware map, open a portal, or acquire another FTC
     * resource. Every later picker selection applies it once for that normalized name. Every
     * returned factory must open a fresh, independently owned lane for that attempt; the factory
     * object itself need not have a new identity. Any backend template or custom SDK library
     * borrowed by the builder must remain stable for this tester's full lifetime and every possible
     * retry.</p>

     * <p>The picker type is only an enumeration contract; it cannot prove which backend an
     * arbitrary function returns or that the function honored the selected name. Camera mount,
     * sensor access, description, and asynchronous readiness remain post-open facts.</p>
     *
     * <p>After open, the lane is the sole source of camera-mount and detection ownership. This
     * tester composes that mount with the captured mount-free AprilTag policy without overriding
     * any configured solver leaf.</p>
     *
     * @param config mutable authoring draft captured by this owner
     * @param visionLaneFactoryBuilder effect-free selected-name-to-factory behavior
     * @throws NullPointerException if an active object answer is null
     * @throws IllegalArgumentException if a name, title, layout, age, or solver answer is invalid
     */
    public AprilTagLocalizationTester(
            Config config,
            Function<String, AprilTagVisionLaneFactory> visionLaneFactoryBuilder
    ) {
        Config source = Objects.requireNonNull(
                config,
                "AprilTagLocalizationTester.Config must not be null"
        );

        this.preferredVisionDeviceName = normalizePreferredName(
                source.preferredVisionDeviceName,
                "AprilTagLocalizationTester.Config.preferredVisionDeviceName"
        );
        this.visionDeviceType = Objects.requireNonNull(
                source.visionDeviceType,
                "AprilTagLocalizationTester.Config.visionDeviceType must not be null"
        );
        this.visionPickerTitle = requireTrimmedNonblank(
                source.visionPickerTitle,
                "AprilTagLocalizationTester.Config.visionPickerTitle"
        );
        TagLayout authoredLayout = Objects.requireNonNull(
                source.fixedTagLayout,
                "AprilTagLocalizationTester.Config.fixedTagLayout must not be null"
        );
        this.layoutPolicySummary = policySummary(authoredLayout);
        this.layout = snapshotLayout(
                authoredLayout,
                "AprilTagLocalizationTester.Config.fixedTagLayout"
        );
        this.aprilTags = Objects.requireNonNull(
                source.aprilTags,
                "AprilTagLocalizationTester.Config.aprilTags must not be null"
        ).validatedCopy("AprilTagLocalizationTester.Config.aprilTags");
        this.visionLaneFactoryBuilder = Objects.requireNonNull(
                visionLaneFactoryBuilder,
                "AprilTagLocalizationTester visionLaneFactoryBuilder must not be null"
        );

        selectedCameraName = preferredVisionDeviceName;
        if (preferredVisionDeviceName != null) {
            pendingVisionLaneFactory = requireVisionFactory(
                    this.visionLaneFactoryBuilder.apply(preferredVisionDeviceName),
                    preferredVisionDeviceName
            );
        }
    }

    private static String normalizePreferredName(String value, String context) {
        if (value == null) {
            return null;
        }
        return requireTrimmedNonblank(value, context);
    }

    private static String requireTrimmedNonblank(String value, String context) {
        Objects.requireNonNull(value, context + " must not be null");
        String normalized = value.trim();
        if (normalized.isEmpty()) {
            throw new IllegalArgumentException(
                    context + " must contain a non-whitespace character; received '" + value + "'"
            );
        }
        return normalized;
    }

    private static TagLayout snapshotLayout(TagLayout layout, String context) {
        try {
            return TagLayouts.snapshot(layout);
        } catch (RuntimeException failure) {
            throw new IllegalArgumentException(
                    context + " is invalid: " + String.valueOf(failure.getMessage()),
                    failure
            );
        }
    }

    private static String policySummary(TagLayout layout) {
        return layout instanceof FtcGameTagLayout
                ? ((FtcGameTagLayout) layout).policySummaryLine()
                : null;
    }

    private static AprilTagVisionLaneFactory requireVisionFactory(
            AprilTagVisionLaneFactory factory,
            String selectedName
    ) {
        if (factory == null) {
            throw new IllegalStateException(
                    "visionLaneFactoryBuilder returned null for " + selectedName
            );
        }
        return factory;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String name() {
        return "AprilTag Localization";
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onInit() {
        cameraPicker = new HardwareNamePicker(
                ctx.hw,
                visionDeviceType,
                visionPickerTitle,
                "Dpad: highlight | A: choose | X: refresh"
        );
        cameraPicker.refresh();

        if (selectedCameraName != null) {
            cameraPicker.setPreferredName(selectedCameraName);
            ensureVisionReady();
        }

        // Picker controls active only while NOT ready.
        cameraPicker.bind(
                bindings,
                gamepads.p1().dpadUp(),
                gamepads.p1().dpadDown(),
                gamepads.p1().a(),
                gamepads.p1().x(),
                () -> visionLane == null && !visionClosingOrTerminal && !visionCleanupFailed,
                chosen -> {
                    preparePickerSelection(chosen);
                }
        );

        Bindings.ControlContext liveControls = bindings.contextWhen(
                BooleanSource.of(() -> visionReady),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL
        );

        // B: clear samples once the vision pipeline is already running.
        liveControls.onRise(gamepads.p1().b(), samples::clear);

        // START: toggle tracking mode (ANY vs SINGLE)
        liveControls.onRise(gamepads.p1().start(), () -> {
            trackAny = !trackAny;
            rebuildSelectionAndEstimator();
        });

        // Tag ID selection (used in SINGLE mode). Dpad Left/Right are the primary controls;
        // Y/X remain as aliases so older muscle memory still works.
        liveControls.onRise(gamepads.p1().dpadRight(), this::incrementSelectedTagId);
        liveControls.onRise(gamepads.p1().dpadLeft(), this::decrementSelectedTagId);
        liveControls.onRise(gamepads.p1().y(), this::incrementSelectedTagId);
        liveControls.onRise(gamepads.p1().x(), this::decrementSelectedTagId);

        // A: capture sample (only when we have a pose)
        liveControls.onRise(gamepads.p1().a(), () -> {
            if (poseEstimator == null) return;

            PoseEstimate est = poseEstimator.getEstimate();
            if (est.hasPose && est.fieldToRobotPose != null) {
                samples.add(est.fieldToRobotPose);
            }
        });

        // If the user supplied a camera name, we already attempted to init vision above.
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onInitLoop(double dtSec) {
        refreshVisionReadiness();
        if (!visionReady) {
            renderCameraPicker();
            return;
        }

        updateAndRender();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onLoop(double dtSec) {
        refreshVisionReadiness();
        if (!visionReady) {
            renderCameraPicker();
            return;
        }

        updateAndRender();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean onBackPressed() {
        if (visionClosingOrTerminal || visionCleanupFailed) {
            return true;
        }
        // If no owner is retained, let the suite handle BACK (exit to suite menu).
        if (visionLane == null) {
            return false;
        }

        // Return to picker state.
        visionReady = false;
        pendingVisionLaneFactory = null;
        visionReadiness = VisionReadiness.notReady("No vision device is open");
        visionInitError = null;

        RuntimeException cleanupFailure = closeVisionLaneOnce();
        tagSensor = null;
        cameraMount = null;
        selection = null;
        poseEstimator = null;
        activeVisionDescription = null;

        samples.clear();

        if (cleanupFailure != null) {
            blockVisionSelection(cleanupFailure);
            return true;
        }
        if (!visionTerminalRequested) {
            visionClosingOrTerminal = false;
        }

        // Reset picker UI and highlight the last camera for convenience.
        resetCameraPickerChoice();

        return true;
    }

    @Override
    protected void onStop() {
        visionTerminalRequested = true;
        pendingVisionLaneFactory = null;
        visionReady = false;
        visionReadiness = VisionReadiness.notReady("Vision tester is stopping");
        tagSensor = null;
        cameraMount = null;
        selection = null;
        poseEstimator = null;
        activeVisionDescription = null;
        RuntimeException cleanupFailure = closeVisionLaneOnce();
        if (cleanupFailure != null) {
            visionCleanupFailed = true;
            visionFailure = cleanupFailure;
            throw cleanupFailure;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Vision init + localization wiring
    // ---------------------------------------------------------------------------------------------

    /** Captures one picker selection before opening its deferred owner. */
    private void preparePickerSelection(String chosenName) {
        if (visionLane != null || visionClosingOrTerminal || visionCleanupFailed) {
            return;
        }

        String normalized;
        try {
            normalized = requireTrimmedNonblank(
                    chosenName,
                    "AprilTagLocalizationTester selected vision device name"
            );
            selectedCameraName = normalized;
            AprilTagVisionLaneFactory selectedFactory = requireVisionFactory(
                    visionLaneFactoryBuilder.apply(normalized),
                    normalized
            );
            pendingVisionLaneFactory = selectedFactory;
        } catch (RuntimeException failure) {
            pendingVisionLaneFactory = null;
            recordCleanSelectionFailure(failure);
            return;
        }

        ensureVisionReady();
    }

    private void ensureVisionReady() {
        if (visionLane != null) {
            refreshVisionReadiness();
            return;
        }
        if (visionClosingOrTerminal) return;
        if (visionCleanupFailed) return;
        AprilTagVisionLaneFactory factory = pendingVisionLaneFactory;
        if (factory == null) return;
        pendingVisionLaneFactory = null;

        visionInitError = null;
        visionFailure = null;

        boolean ownerPublished = false;
        try {
            AprilTagVisionLane openedLane = factory.open(ctx.hw);
            if (openedLane == null) {
                throw new IllegalStateException(
                        "vision lane factory returned null for " + selectedCameraName);
            }
            visionLane = openedLane;
            ownerPublished = true;
            tagSensor = Objects.requireNonNull(
                    visionLane.tagSensor(),
                    "AprilTag vision lane returned a null tag sensor"
            );
            cameraMount = Objects.requireNonNull(
                    visionLane.cameraMountConfig(),
                    "AprilTag vision lane returned a null camera mount"
            );
            activeVisionDescription = factory.description();
            visionReady = false;
            visionReadiness = VisionReadiness.notReady("Vision device is opening");
            refreshVisionReadiness();
        } catch (RuntimeException ex) {
            boolean unpublishedCleanupUncertain = !ownerPublished
                    && ex.getSuppressed().length > 0;
            tagSensor = null;
            cameraMount = null;
            selection = null;
            poseEstimator = null;
            activeVisionDescription = null;
            visionReady = false;
            visionReadiness = VisionReadiness.notReady("Vision initialization failed");
            RuntimeException cleanupFailure = closeVisionLaneOnce();
            visionFailure = ex;
            if (cleanupFailure != null) {
                if (cleanupFailure != ex) {
                    ex.addSuppressed(cleanupFailure);
                }
                visionCleanupFailed = true;
            } else if (unpublishedCleanupUncertain) {
                // A failed factory may have acquired hardware and attached a failed rollback as
                // suppressed. With no published lane to close again, replacement is unsafe.
                visionCleanupFailed = true;
            } else if (!visionTerminalRequested) {
                visionClosingOrTerminal = false;
                resetCameraPickerChoice();
            }
            visionInitError = visionFailureMessage(ex);
        }
    }

    /** Refreshes asynchronous component readiness without opening a second camera owner. */
    private void refreshVisionReadiness() {
        AprilTagVisionLane lane = visionLane;
        if (lane == null || visionClosingOrTerminal || visionCleanupFailed) {
            visionReady = false;
            return;
        }

        boolean wasReady = visionReady;
        try {
            VisionReadiness current = lane.readiness(clock);
            if (current == null) {
                throw new IllegalStateException(
                        "AprilTag vision lane returned a null readiness result"
                );
            }
            visionReadiness = current;
            visionReady = visionReadiness.isReady();
            if (visionReady && (!wasReady || selection == null || poseEstimator == null)) {
                rebuildSelectionAndEstimator();
            }
        } catch (RuntimeException failure) {
            visionReady = false;
            visionReadiness = VisionReadiness.notReady("Vision readiness check failed");
            tagSensor = null;
            cameraMount = null;
            selection = null;
            poseEstimator = null;
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
                resetCameraPickerChoice();
            }
            visionInitError = visionFailureMessage(failure);
        }
    }

    /** Records an effect-free builder/selection failure and returns to the same picker. */
    private void recordCleanSelectionFailure(RuntimeException failure) {
        visionFailure = failure;
        tagSensor = null;
        cameraMount = null;
        selection = null;
        poseEstimator = null;
        activeVisionDescription = null;
        visionReady = false;
        visionReadiness = VisionReadiness.notReady("Vision initialization failed");
        if (!visionTerminalRequested) {
            visionClosingOrTerminal = false;
            resetCameraPickerChoice();
        }
        visionInitError = visionFailureMessage(failure);
    }

    private void resetCameraPickerChoice() {
        if (cameraPicker == null) {
            return;
        }
        cameraPicker.clearChoice();
        cameraPicker.refresh();
        if (selectedCameraName != null && !selectedCameraName.isEmpty()) {
            cameraPicker.setPreferredName(selectedCameraName);
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
        visionInitError = visionFailureMessage(cleanupFailure);
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

    private void rebuildSelectionAndEstimator() {
        if (!visionReady || tagSensor == null || layout == null) {
            return;
        }

        Set<Integer> ids = trackAny
                ? layout.ids()
                : Collections.singleton(selectedTagId);

        // layout.ids() should never be empty for official FTC games, but guard anyway.
        if (ids == null || ids.isEmpty()) {
            ids = Collections.singleton(selectedTagId);
            trackAny = false;
        }

        selection = TagSelections.from(tagSensor)
                .among(ids)
                .freshWithinSec(aprilTags.maxDetectionAgeSec)
                .choose(TagSelectionPolicies.closestRange())
                .continuous()
                .build();

        AprilTagPoseEstimator.Config cfg = aprilTags.toAprilTagPoseEstimatorConfig(cameraMount);

        poseEstimator = new AprilTagPoseEstimator(tagSensor, layout, cfg);

        // Reset sampling when the core solve parameters change.
        samples.clear();
    }

    // ---------------------------------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------------------------------

    private void renderCameraPicker() {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        cameraPicker.render(t);

        t.addLine("");
        t.addLine("Chosen: " + (selectedCameraName == null ? "(none)" : selectedCameraName));
        if (visionLane != null) {
            t.addData("Vision readiness", visionReadiness.isReady() ? "READY" : "WAITING");
            t.addData("Vision status", visionReadiness.reason());
            t.addLine("Press BACK to close this owner and choose another device.");
        }
        if (visionCleanupFailed) {
            t.addLine("VISION DEVICE SELECTION DISABLED.");
            t.addLine("Stop and restart this OpMode before selecting another device.");
        } else if (visionLane == null) {
            t.addLine("Press A to choose the active vision device and initialize AprilTags.");
            t.addLine("Press X to refresh the device list.");
            t.addLine("Press BACK to exit to the tester menu.");
        }

        if (activeVisionDescription != null && !activeVisionDescription.isEmpty()) {
            t.addLine("Backend: " + activeVisionDescription);
        }

        if (visionInitError != null) {
            t.addLine("");
            t.addLine("Vision init error:");
            t.addLine(visionInitError);
        }

        t.update();
    }

    private void renderInternalError(String message) {
        Telemetry t = ctx.telemetry;
        t.clearAll();
        t.addLine("=== AprilTag Localization ===");
        t.addLine("ERROR: " + message);
        t.addLine("");
        t.addLine("Try: BACK → camera picker, then choose a camera again.");
        t.addLine("If the problem persists, verify your camera configuration and wiring.");
        t.update();
    }

    private void updateAndRender() {
        // Defensive: if something failed during init, try rebuilding once instead of crashing.
        if (selection == null || poseEstimator == null) {
            rebuildSelectionAndEstimator();
        }
        if (selection == null || poseEstimator == null) {
            renderInternalError("Localization pipeline not initialized (selection/estimator is null)");
            return;
        }

        // Update tracked tag + pose solve.
        poseEstimator.update(clock);

        TagSelectionResult selectionResult = selection.get(clock);
        AprilTagObservation obs = selectionResult.hasFreshSelectedObservation
                ? selectionResult.selectedObservation
                : AprilTagObservation.noTarget();
        PoseEstimate est = poseEstimator.getEstimate();

        renderTelemetry(obs, est);
    }

    private void renderTelemetry(AprilTagObservation obs, PoseEstimate est) {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        t.addLine("=== AprilTag Localization ===");
        t.addData("Camera", selectedCameraName);
        if (activeVisionDescription != null && !activeVisionDescription.isEmpty()) {
            t.addData("Backend", activeVisionDescription);
        }
        t.addData("Track [START]", trackAny ? "ANY" : "SINGLE");
        t.addData("Tag ID [Dpad L/R or Y/X]", selectedTagId);
        t.addData("Samples [A capture | B clear]", samples.count());
        t.addData("MaxAge", "%.0f ms", aprilTags.maxDetectionAgeSec * 1000.0);

        if (layoutPolicySummary != null) {
            t.addData("Layout policy", layoutPolicySummary);
        } else {
            t.addData("Layout ids", layout.ids());
        }

        if (isLikelyIdentity(cameraMount)) {
            t.addLine("");
            t.addLine("NOTE: CameraMountConfig still looks like the identity placeholder.");
            t.addLine("Run 'Calib: Camera Mount' and update your RobotConfig for better accuracy.");
        }

        t.addLine("");
        t.addLine("Observation:");

        if (!obs.hasTarget) {
            t.addLine("  No fresh tag detection.");
            t.addLine("  Tips: check lighting, focus, camera stream, and tag visibility.");
        } else {
            t.addData("  Tag id", obs.id);
            t.addData("  Age", "%.0f ms", obs.frameAgeSec(clock) * 1000.0);
            t.addData("  Range", "%.1f in", obs.cameraRangeInches());
            t.addData("  Bearing", "%.1f°", Math.toDegrees(obs.cameraBearingRad()));
            t.addData("  cameraToTag", "fwd=%.1f in | left=%.1f in | up=%.1f in",
                    obs.cameraForwardInches(),
                    obs.cameraLeftInches(),
                    obs.cameraUpInches());

            boolean inLayout = (layout != null && layout.has(obs.id));
            t.addData("  In layout", inLayout);

            if (inLayout) {
                Pose3d ft = layout.requireFieldToTagPose(obs.id);
                t.addData("  fieldToTag", "x=%.1f y=%.1f z=%.1f | yaw=%.1f°",
                        ft.xInches, ft.yInches, ft.zInches,
                        Math.toDegrees(ft.yawRad));
            }
        }

        t.addLine("");
        t.addLine("Pose estimate (AprilTagPoseEstimator):");
        if (!est.hasPose || est.fieldToRobotPose == null) {
            t.addLine("  (no pose) Need: fresh detection for a tag present in the field layout.");
        } else {
            Pose3d p = est.fieldToRobotPose;
            t.addData("  fieldToRobot", "x=%.1f y=%.1f | yaw=%.1f°",
                    p.xInches, p.yInches, Math.toDegrees(p.yawRad));
            t.addData("  Pose age", "%.0f ms", est.timestamp.ageSec(clock) * 1000.0);
        }

        t.addLine("");
        t.addLine("Layout summary:");
        FtcTagLayoutDebug.dumpSummary(layout, new FtcTelemetryDebugSink(t), "layout");

        // Optional: compare against FTC SDK's robot pose (only available when the SDK computed it).
        if (obs.hasFieldToRobotPose()) {
            t.addLine("");
            t.addLine("FTC SDK robotPose (if available):");

            Pose3d sdk = obs.fieldToRobotPose;
            Pose3d sdkPlanar = new Pose3d(
                    sdk.xInches,
                    sdk.yInches,
                    0.0,
                    Pose2d.wrapToPi(sdk.yawRad),
                    0.0,
                    0.0
            );

            t.addData("  SDK fieldToRobot", "x=%.1f y=%.1f | yaw=%.1f°",
                    sdkPlanar.xInches, sdkPlanar.yInches, Math.toDegrees(sdkPlanar.yawRad));

            if (est.hasPose && est.fieldToRobotPose != null) {
                Pose3d p = est.fieldToRobotPose;

                double dx = p.xInches - sdkPlanar.xInches;
                double dy = p.yInches - sdkPlanar.yInches;
                double dxy = Math.hypot(dx, dy);
                double dYawRad = MathUtil.wrapToPi(p.yawRad - sdkPlanar.yawRad);

                t.addData("  Est - SDK", "dXY=%.2f in | dYaw=%.2f°",
                        dxy,
                        Math.toDegrees(dYawRad));
            }
        }

        t.addLine("");
        t.addLine("Sampling:");
        if (samples.count() <= 0) {
            t.addLine("  Samples [A]: 0  (capture while the robot is still)");
        } else {
            t.addData("  Samples [A]", samples.count());
            t.addData("  Mean", "x=%.1f y=%.1f | yaw=%.1f°",
                    samples.meanX(),
                    samples.meanY(),
                    Math.toDegrees(samples.meanYawRad()));
            t.addData("  Std dev", "x=%.2f y=%.2f | yaw=%.2f°",
                    samples.stdX(),
                    samples.stdY(),
                    Math.toDegrees(samples.circularStdYawRad()));
        }

        t.addLine("");
        t.addLine("BACK: return to the camera picker.");
        t.update();
    }

    private void incrementSelectedTagId() {
        selectedTagId++;
        if (!trackAny) {
            rebuildSelectionAndEstimator();
        }
    }

    private void decrementSelectedTagId() {
        selectedTagId = Math.max(1, selectedTagId - 1);
        if (!trackAny) {
            rebuildSelectionAndEstimator();
        }
    }

    private static boolean isLikelyIdentity(CameraMountConfig m) {
        if (m == null) return true;

        // CameraMountConfig does not expose a direct equals/identity test;
        // treat "all zeros" as identity.
        double eps = 1e-9;
        return Math.abs(m.xInches()) < eps
                && Math.abs(m.yInches()) < eps
                && Math.abs(m.zInches()) < eps
                && Math.abs(m.yawRad()) < eps
                && Math.abs(m.pitchRad()) < eps
                && Math.abs(m.rollRad()) < eps;
    }

    // ---------------------------------------------------------------------------------------------
    // Simple pose sampling stats
    // ---------------------------------------------------------------------------------------------

    /**
     * Incremental statistics for captured planar poses.
     *
     * <p>This is intentionally simple (student-friendly) and is only meant to help
     * you eyeball stability/jitter while standing still.</p>
     */
    private static final class PoseSampleStats {

        private int n = 0;

        private double meanX = 0.0;
        private double meanY = 0.0;

        private double m2X = 0.0;
        private double m2Y = 0.0;

        private double sumSinYaw = 0.0;
        private double sumCosYaw = 0.0;

        void clear() {
            n = 0;
            meanX = meanY = 0.0;
            m2X = m2Y = 0.0;
            sumSinYaw = 0.0;
            sumCosYaw = 0.0;
        }

        int count() {
            return n;
        }

        void add(Pose3d fieldToRobotPose) {
            if (fieldToRobotPose == null) return;

            // We only care about planar stability for drivebase use.
            double x = fieldToRobotPose.xInches;
            double y = fieldToRobotPose.yInches;
            double yaw = Pose2d.wrapToPi(fieldToRobotPose.yawRad);

            n++;

            // Welford mean/std for X
            double dx = x - meanX;
            meanX += dx / n;
            m2X += dx * (x - meanX);

            // Welford mean/std for Y
            double dy = y - meanY;
            meanY += dy / n;
            m2Y += dy * (y - meanY);

            // Circular mean components for yaw.
            sumSinYaw += Math.sin(yaw);
            sumCosYaw += Math.cos(yaw);
        }

        double meanX() {
            return meanX;
        }

        double meanY() {
            return meanY;
        }

        double stdX() {
            if (n < 2) return 0.0;
            return Math.sqrt(m2X / (n - 1));
        }

        double stdY() {
            if (n < 2) return 0.0;
            return Math.sqrt(m2Y / (n - 1));
        }

        double meanYawRad() {
            if (n <= 0) return 0.0;
            return Math.atan2(sumSinYaw, sumCosYaw);
        }

        /**
         * Approximate circular standard deviation for yaw (radians).
         *
         * <p>Uses the classic relation {@code sigma = sqrt(-2 ln(R))} where
         * {@code R = |mean resultant|}.</p>
         */
        double circularStdYawRad() {
            if (n < 2) return 0.0;

            double r = Math.hypot(sumSinYaw, sumCosYaw) / n;

            // Numerical safety: clamp to (0, 1].
            r = Math.max(1e-12, Math.min(1.0, r));

            return Math.sqrt(-2.0 * Math.log(r));
        }
    }
}
