package edu.ftcsushi.fw.tools.tester.calibration;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.Locale;
import java.util.Objects;
import java.util.function.Function;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.geometry.Mat3;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.field.TagLayouts;
import edu.ftcsushi.fw.ftc.FtcGameTagLayout;
import edu.ftcsushi.fw.ftc.FtcTagLayoutDebug;
import edu.ftcsushi.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcsushi.fw.ftc.vision.OwnedAprilTagCamera;
import edu.ftcsushi.fw.ftc.vision.AprilTagCameraFactory;
import edu.ftcsushi.fw.ftc.vision.VisionReadiness;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.tools.tester.BaseTeleOpTester;
import edu.ftcsushi.fw.ftc.ui.HardwareNamePicker;
import edu.ftcsushi.fw.input.binding.Bindings;

/**
 * Calibrates {@code robotToCameraPose} (camera mount extrinsics) using:
 * <ul>
 *   <li>Known AprilTag field layout (framework-owned current-game fixed layout by default), and</li>
 *   <li>A manually-entered / adjustable known robot pose {@code fieldToRobotPose}.</li>
 * </ul>
 *
 * <h2>Camera selection</h2>
 * <p>
 * A {@code null} preferred hardware name shows the configured vision-device picker. A valid
 * preferred name is attempted first; a clean open/setup failure exposes that same replacement
 * picker with the failed name highlighted. Blank preferred names are rejected as configuration
 * errors.
 * </p>
 *
 * <h2>Core math</h2>
 * <pre>
 * fieldToTagPose = fieldToRobotPose · robotToCameraPose · cameraToTagPose
 * => robotToCameraPose = inv(fieldToRobotPose) · fieldToTagPose · inv(cameraToTagPose)
 * </pre>
 *
 * <h2>Capture evidence</h2>
 * <p>A requests one sample from the current loop's ready camera and fresh frame, not the previous
 * preview. Each accepted frame timestamp must advance. B and actual tag/known-pose edits clear
 * the stationary batch and win over A in that cycle; subsequent captures must come from images
 * taken after that boundary. Camera replacement, shutdown, and Driver Station START/clock reset
 * also clear the batch. Temporary WAITING retains only historical results, never a capture request.
 * UI step/mode/field selection does not change the batch.</p>
 *
 * <p>The equal-weight mean averages translations and complete rotations, not separate Euler
 * angles. Finite but conflicting rotations remain counted; an ambiguous rotation mean has no
 * printable recommendation. This estimates mount extrinsics relative to the chosen fixed robot
 * reference point, not lens intrinsics or shooter/intake alignment. The ordinary pose controls
 * assume a level robot with its reference point at field Z=0; only X, Y, and yaw are editable.</p>
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>PICKER (no camera chosen yet)</b>: Dpad Up/Down highlight, A choose, X refresh</li>
 *   <li><b>CALIBRATE (camera chosen)</b>:
 *     <ul>
 *       <li>Y/X: increment/decrement tag ID</li>
 *       <li>A: capture one new, fresh frame into the average mount</li>
 *       <li>B: clear captured samples (wins over A in the same cycle)</li>
 *       <li>Dpad: adjust known robot pose (XY)</li>
 *       <li>LB/RB: adjust known robot yaw</li>
 *       <li>START: fine/coarse step</li>
 *       <li>BACK: return to camera picker (change camera)</li>
 *     </ul>
 *   </li>
 * </ul>
 */
public final class CameraMountCalibrator extends BaseTeleOpTester {

    private static final double DEFAULT_MAX_AGE_SEC = 0.35;
    private static final int DEFAULT_TAG_ID = 1;
    private static final Pose3d DEFAULT_P_FIELD_TO_ROBOT = Pose3d.zero();

    /** Mutable, data-only authoring configuration for one camera-mount calibration owner. */
    public static final class Config {

        /** Preferred configured vision-device name, or {@code null} to show the picker. */
        public String preferredVisionDeviceName;

        /** Hardware type enumerated by the replacement picker. */
        public Class<? extends HardwareDevice> visionDeviceType;

        /** Nonblank title shown by the replacement picker. */
        public String visionPickerTitle;

        /** Fixed field-tag facts used by the mount solve. */
        public TagLayout fixedTagLayout;

        /** Maximum accepted detection-frame age in seconds. */
        public double maxDetectionAgeSec;

        private Config() {
        }

        /**
         * Returns a fresh software-valid authoring draft.
         *
         * <p>The current-game layout is a borrowed field-fact source. The calibrator snapshots it
         * when constructed; defaults do not claim that the selected camera, mount, or field setup
         * has been physically verified.</p>
         */
        public static Config defaults() {
            Config c = new Config();
            c.preferredVisionDeviceName = null;
            c.visionDeviceType = WebcamName.class;
            c.visionPickerTitle = "Select Camera";
            c.fixedTagLayout = FtcGameTagLayout.currentGameFieldFixed();
            c.maxDetectionAgeSec = DEFAULT_MAX_AGE_SEC;
            return c;
        }
    }

    // Captured owner configuration
    private final String preferredVisionDeviceName;
    private final Class<? extends HardwareDevice> visionDeviceType;
    private final String visionPickerTitle;
    private final Function<String, AprilTagCameraFactory> visionLaneFactoryBuilder;
    private final TagLayout layout;
    private final String layoutPolicySummary;
    private final double maxDetectionAgeSec;

    /** Factory captured for the initial preferred-name attempt; picker attempts replace it. */
    private AprilTagCameraFactory pendingVisionLaneFactory;

    // Runtime state
    private OwnedAprilTagCamera visionLane;
    private AprilTagSensor tagSensor;

    private boolean visionReady = false;
    private boolean visionClosingOrTerminal = false;
    private boolean visionTerminalRequested = false;
    private boolean visionCleanupFailed = false;
    private RuntimeException visionFailure = null;
    private String selectedCameraName = null;
    private String visionInitError = null;
    private String activeVisionDescription = null;
    private VisionReadiness visionReadiness = VisionReadiness.notReady("No vision device is open");

    private HardwareNamePicker cameraPicker;

    private int selectedTagId = DEFAULT_TAG_ID;
    private Pose3d fieldToRobotPose = DEFAULT_P_FIELD_TO_ROBOT;
    private boolean fineSteps = true;

    // Input/UI mode
    private boolean editMode = false;

    private enum EditField {
        TAG_ID("Tag ID"),
        ROBOT_X("Robot X"),
        ROBOT_Y("Robot Y"),
        ROBOT_YAW("Robot Yaw");

        final String label;

        EditField(String label) {
            this.label = label;
        }
    }

    private EditField editField = EditField.ROBOT_X;

    private Pose3d lastRobotToCameraSample = null;
    private Pose3d lastObservedCameraToTag = null;

    private final PoseAverager avg = new PoseAverager();
    private boolean captureRequested;
    private long batchGeneration;
    private LoopTimestamp batchBoundary = LoopTimestamp.unavailable();
    private LoopTimestamp lastAcceptedFrame = LoopTimestamp.unavailable();
    private long captureInhibitedCycle = Long.MIN_VALUE;
    private String captureStatus = "Hold still; press A for a new, fresh frame.";

    /**
     * Creates one backend-neutral camera-mount calibration owner.
     *
     * <p>The constructor defensively captures and validates all data before invoking
     * {@code visionLaneFactoryBuilder}. A non-null preferred name is trimmed and causes exactly one
     * effect-free builder application here; {@code null} selects the replacement picker. A blank
     * preferred name is invalid. The builder must only capture backend configuration and return a
     * deferred factory: it must not inspect the hardware map, open a portal, or acquire another FTC
     * resource. Every later picker selection applies the builder once for that normalized name.
     * The returned factory must open a fresh, independently owned lane for every attempt; the
     * factory object itself need not have a new identity. Any backend template or custom SDK
     * library borrowed by the builder must remain stable for this tester's full lifetime and every
     * possible retry.</p>

     * <p>The picker type is only an enumeration contract; it cannot prove which backend an
     * arbitrary function returns or that the function honored the selected name. Lane accessors,
     * description, and asynchronous readiness remain post-open facts.</p>
     *
     * <p>The factory's lane supplies detection ownership. Its camera-mount answer is deliberately
     * irrelevant to this calibration workflow because the mount is the fact being measured.</p>
     *
     * @param config mutable authoring draft captured by this owner
     * @param visionLaneFactoryBuilder effect-free selected-name-to-factory behavior
     * @throws NullPointerException if an active object answer is null
     * @throws IllegalArgumentException if a name, title, layout, or age is invalid
     */
    public CameraMountCalibrator(
            Config config,
            Function<String, AprilTagCameraFactory> visionLaneFactoryBuilder
    ) {
        Config source = Objects.requireNonNull(config, "CameraMountCalibrator.Config must not be null");

        this.preferredVisionDeviceName = normalizePreferredName(
                source.preferredVisionDeviceName,
                "CameraMountCalibrator.Config.preferredVisionDeviceName"
        );
        this.visionDeviceType = Objects.requireNonNull(
                source.visionDeviceType,
                "CameraMountCalibrator.Config.visionDeviceType must not be null"
        );
        this.visionPickerTitle = requireTrimmedNonblank(
                source.visionPickerTitle,
                "CameraMountCalibrator.Config.visionPickerTitle"
        );
        TagLayout authoredLayout = Objects.requireNonNull(
                source.fixedTagLayout,
                "CameraMountCalibrator.Config.fixedTagLayout must not be null"
        );
        this.layoutPolicySummary = policySummary(authoredLayout);
        this.layout = snapshotLayout(
                authoredLayout,
                "CameraMountCalibrator.Config.fixedTagLayout"
        );
        this.maxDetectionAgeSec = requireFiniteNonnegative(
                source.maxDetectionAgeSec,
                "CameraMountCalibrator.Config.maxDetectionAgeSec"
        );
        this.visionLaneFactoryBuilder = Objects.requireNonNull(
                visionLaneFactoryBuilder,
                "CameraMountCalibrator visionLaneFactoryBuilder must not be null"
        );

        if (!layout.ids().isEmpty()) {
            selectedTagId = layout.ids().iterator().next();
        }
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

    private static double requireFiniteNonnegative(double value, String context) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(
                    context + " must be finite and >= 0; received " + value
            );
        }
        return value;
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

    private static AprilTagCameraFactory requireVisionFactory(
            AprilTagCameraFactory factory,
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
        return "Camera Mount Calibrator";
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
        if (selectedCameraName != null && !selectedCameraName.isEmpty()) {
            cameraPicker.setPreferredName(selectedCameraName);
        }

        // Camera menu navigation is ONLY active before visionReady.
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

        Bindings.ControlContext calibrationControls = bindings.contextWhen(
                BooleanSource.of(() -> visionReady),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL
        );

        // B clears samples once vision is already running.
        calibrationControls.onRise(gamepads.p1().b(), this::clearCapturedSamples);

        // Capture sample (only when vision is ready)
        calibrationControls.onRise(gamepads.p1().a(), () -> captureRequested = true);

        // Calibration controls (only when vision is ready)
        calibrationControls.onRise(gamepads.p1().y(), () -> changeTagId(1));

        calibrationControls.onRise(gamepads.p1().x(),
                () -> changeTagId(-1));

        calibrationControls.onRise(gamepads.p1().start(), () -> fineSteps = !fineSteps);

        // Toggle edit mode (RS). Edit mode lets you select which variable you're changing
        // instead of remembering which button maps to which axis.
        calibrationControls.onRise(gamepads.p1().rightStickButton(), () -> editMode = !editMode);

        // D-pad:
        //  - QUICK mode: dpad X adjusts X, dpad Y adjusts Y (requested)
        //  - EDIT mode: up/down selects a field, left/right changes its value
        calibrationControls.onRise(gamepads.p1().dpadUp(), () -> {
            if (editMode) {
                cycleEditField(-1);
            } else {
                adjustRobotPose(0.0, +stepXY(), 0.0);
            }
        });
        calibrationControls.onRise(gamepads.p1().dpadDown(), () -> {
            if (editMode) {
                cycleEditField(+1);
            } else {
                adjustRobotPose(0.0, -stepXY(), 0.0);
            }
        });

        calibrationControls.onRise(gamepads.p1().dpadLeft(), () -> {
            if (editMode) {
                adjustEditField(-1);
            } else {
                adjustRobotPose(-stepXY(), 0.0, 0.0);
            }
        });
        calibrationControls.onRise(gamepads.p1().dpadRight(), () -> {
            if (editMode) {
                adjustEditField(+1);
            } else {
                adjustRobotPose(+stepXY(), 0.0, 0.0);
            }
        });

        // Yaw adjustment. In QUICK mode we keep the classic LB/RB mapping.
        // In EDIT mode, bumpers behave like +/- on the selected field.
        calibrationControls.onRise(gamepads.p1().leftBumper(), () -> {
            if (editMode) {
                adjustEditField(-1);
            } else {
                adjustRobotPose(0.0, 0.0, +stepYawRad());
            }
        });
        calibrationControls.onRise(gamepads.p1().rightBumper(), () -> {
            if (editMode) {
                adjustEditField(+1);
            } else {
                adjustRobotPose(0.0, 0.0, -stepYawRad());
            }
        });

        // If user provided a camera name, try to bring vision up immediately in INIT.
        ensureVisionReady();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean onBackPressed() {
        if (visionClosingOrTerminal || visionCleanupFailed) {
            return true;
        }
        if (visionLane == null) {
            return false;
        }

        // Return to the camera picker. This allows you to re-select the active vision
        // device without leaving the tester suite.
        visionReady = false;
        pendingVisionLaneFactory = null;
        visionReadiness = VisionReadiness.notReady("No vision device is open");
        visionInitError = null;

        editMode = false;
        editField = EditField.ROBOT_X;

        RuntimeException cleanupFailure = closeVisionLaneOnce();
        tagSensor = null;
        activeVisionDescription = null;

        if (cleanupFailure != null) {
            blockVisionSelection(cleanupFailure);
            return true;
        }
        if (!visionTerminalRequested) {
            visionClosingOrTerminal = false;
        }

        // Rebuild menu entries and keep the last chosen camera highlighted.
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
        activeVisionDescription = null;
        RuntimeException cleanupFailure = closeVisionLaneOnce();
        if (cleanupFailure != null) {
            visionCleanupFailed = true;
            visionFailure = cleanupFailure;
            throw cleanupFailure;
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onInitLoop(double dtSec) {
        updateSolveAndTelemetry();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onLoop(double dtSec) {
        updateSolveAndTelemetry();
    }

    @Override
    protected void onStart() {
        clearCapturedSamples();
    }

    /** B's action also fences reentrant capture/solve work already in flight. */
    private void clearCapturedSamples() {
        batchGeneration++;
        captureRequested = false;
        lastRobotToCameraSample = null;
        lastObservedCameraToTag = null;
        avg.clear();
        lastAcceptedFrame = LoopTimestamp.unavailable();
        // Shutdown is also safe before init has installed a context.
        batchBoundary = ctx == null ? LoopTimestamp.unavailable() : ctx.clock.nowTimestamp();
        captureInhibitedCycle = ctx == null ? Long.MIN_VALUE : ctx.clock.cycle();
        captureStatus = "Batch cleared; wait for an image captured after this setup boundary.";
    }

    private boolean sameBatch(OwnedAprilTagCamera owner, long generation) {
        return sameCameraOwner(owner)
                && batchGeneration == generation
                && Double.isFinite(batchBoundary.ageSec(ctx.clock));
    }

    private boolean sameCameraOwner(OwnedAprilTagCamera owner) {
        return visionLane == owner && owner != null && !visionClosingOrTerminal
                && !visionTerminalRequested && !visionCleanupFailed;
    }

    private void invalidateResetEpoch() {
        if (ctx != null && !Double.isFinite(batchBoundary.ageSec(ctx.clock))) {
            clearCapturedSamples();
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Vision init / camera enumeration
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
                    "CameraMountCalibrator selected vision device name"
            );
            selectedCameraName = normalized;
            AprilTagCameraFactory selectedFactory = requireVisionFactory(
                    visionLaneFactoryBuilder.apply(normalized),
                    normalized
            );
            pendingVisionLaneFactory = selectedFactory;
        } catch (RuntimeException failure) {
            pendingVisionLaneFactory = null;
            recordCleanSelectionFailure("Failed to configure AprilTag camera", failure);
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
        AprilTagCameraFactory factory = pendingVisionLaneFactory;
        if (factory == null) return;
        pendingVisionLaneFactory = null;

        clearCapturedSamples();
        visionFailure = null;
        boolean ownerPublished = false;
        try {
            OwnedAprilTagCamera openedLane;
            try {
                openedLane = factory.open(ctx.hw);
            } finally {
                invalidateResetEpoch();
            }
            if (openedLane == null) {
                throw new IllegalStateException(
                        "vision lane factory returned null for " + selectedCameraName);
            }
            visionLane = openedLane;
            ownerPublished = true;
            if (visionTerminalRequested) {
                RuntimeException cleanupFailure = closeVisionLaneOnce();
                if (cleanupFailure != null) {
                    visionCleanupFailed = true;
                    throw cleanupFailure;
                }
                return;
            }
            AprilTagSensor openedSensor;
            try {
                openedSensor = Objects.requireNonNull(openedLane.aprilTags().tagSensor(),
                        "AprilTag vision lane returned a null tag sensor");
            } finally {
                invalidateResetEpoch();
            }
            if (!sameCameraOwner(openedLane)) return;
            String description;
            try {
                description = factory.description();
            } finally {
                invalidateResetEpoch();
            }
            if (!sameCameraOwner(openedLane)) return;
            tagSensor = openedSensor;
            activeVisionDescription = description;

            visionReady = false;
            visionReadiness = VisionReadiness.notReady("Vision device is opening");
            visionInitError = null;
            refreshVisionReadiness();
        } catch (RuntimeException e) {
            boolean unpublishedCleanupUncertain = !ownerPublished
                    && e.getSuppressed().length > 0;
            tagSensor = null;
            activeVisionDescription = null;
            visionReady = false;
            visionReadiness = VisionReadiness.notReady("Vision initialization failed");
            RuntimeException cleanupFailure = closeVisionLaneOnce();
            visionFailure = e;
            if (cleanupFailure != null) {
                if (cleanupFailure != e) {
                    e.addSuppressed(cleanupFailure);
                }
                visionCleanupFailed = true;
            } else if (unpublishedCleanupUncertain) {
                // A framework constructor can fail before publishing its lane and attach a failed
                // rollback/close attempt. There is no safe owner reference to close again.
                visionCleanupFailed = true;
            } else if (!visionTerminalRequested) {
                visionClosingOrTerminal = false;
                resetCameraPickerChoice();
            }
            visionInitError = visionFailureMessage("Failed to start AprilTag camera", e);
        }
    }

    /** Refreshes asynchronous camera readiness without opening a competing owner. */
    private void refreshVisionReadiness() {
        OwnedAprilTagCamera lane = visionLane;
        if (lane == null || visionClosingOrTerminal || visionCleanupFailed) {
            visionReady = false;
            return;
        }
        try {
            long generation = batchGeneration;
            VisionReadiness current;
            try {
                current = lane.aprilTags().readiness(ctx.clock);
            } finally {
                invalidateResetEpoch();
            }
            if (!sameBatch(lane, generation)) return;
            if (current == null) {
                throw new IllegalStateException(
                        "AprilTag vision lane returned a null readiness result"
                );
            }
            visionReadiness = current;
            visionReady = visionReadiness.isReady();
        } catch (RuntimeException failure) {
            visionReady = false;
            visionReadiness = VisionReadiness.notReady("Vision readiness check failed");
            tagSensor = null;
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
            visionInitError = visionFailureMessage("Vision readiness failed", failure);
        }
    }

    /** Records an effect-free builder/selection failure and returns to the same picker. */
    private void recordCleanSelectionFailure(String prefix, RuntimeException failure) {
        clearCapturedSamples();
        visionFailure = failure;
        tagSensor = null;
        activeVisionDescription = null;
        visionReady = false;
        visionReadiness = VisionReadiness.notReady("Vision initialization failed");
        if (!visionTerminalRequested) {
            visionClosingOrTerminal = false;
            resetCameraPickerChoice();
        }
        visionInitError = visionFailureMessage(prefix, failure);
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
        clearCapturedSamples();
        visionClosingOrTerminal = true;
        OwnedAprilTagCamera lane = visionLane;
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
        visionInitError = visionFailureMessage("Failed to stop AprilTag camera", cleanupFailure);
    }

    /** Formats the primary failure first and retains any suppressed cleanup diagnostics. */
    private String visionFailureMessage(String prefix, RuntimeException failure) {
        StringBuilder message = new StringBuilder(prefix)
                .append(": ")
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

    private void renderCameraPicker() {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        if (cameraPicker != null) {
            cameraPicker.render(t);
        }

        t.addLine("");
        t.addLine("Chosen: " + (selectedCameraName == null ? "(none)" : selectedCameraName));
        if (visionLane != null) {
            t.addData("Vision readiness", visionReadiness.isReady() ? "READY" : "WAITING");
            t.addData("Vision status", visionReadiness.reason());
            t.addLine("Press BACK to close this owner and choose another device.");
            t.addData("Historical captured samples (not current readiness)", avg.count());
            if (avg.meanOrNull() == null && avg.count() > 0) {
                t.addData("Historical average unavailable", avg.unavailableReason());
            }
        }
        if (visionCleanupFailed) {
            t.addLine("VISION DEVICE SELECTION DISABLED.");
            t.addLine("Stop and restart this OpMode before selecting another device.");
        } else if (visionLane == null) {
            t.addLine("Press A to choose the active vision device and initialize AprilTags.");
            t.addLine("Press X to refresh camera list.");
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

    private void updateSolveAndTelemetry() {
        // Bindings precede this phase. Drain before any external callback can fail or reenter.
        boolean requested = captureRequested;
        captureRequested = false;
        lastRobotToCameraSample = null;
        lastObservedCameraToTag = null;
        if (!Double.isFinite(batchBoundary.ageSec(ctx.clock))) {
            clearCapturedSamples();
            requested = false;
        }
        OwnedAprilTagCamera owner = visionLane;
        long generation = batchGeneration;
        int tagId = selectedTagId;
        Pose3d knownRobotPose = fieldToRobotPose;
        refreshVisionReadiness();
        if (!sameBatch(owner, generation) || !visionReady) {
            renderCameraPicker();
            return;
        }
        AprilTagDetections frame;
        try {
            frame = Objects.requireNonNull(tagSensor.get(ctx.clock),
                    "Camera mount calibration tag sensor returned null detections");
        } finally {
            invalidateResetEpoch();
        }
        if (!sameBatch(owner, generation)) return;
        // A wrong-clock frame is a source contract failure, not an ordinary missing target.
        AprilTagObservation obs = frame.forId(ctx.clock, tagId, maxDetectionAgeSec);
        captureStatus = "No fresh detection for selected tag ID.";
        if (obs.hasTarget && isFinitePose(obs.cameraToTagPose)) {
            lastObservedCameraToTag = obs.cameraToTagPose;
            Pose3d tagPose = layout.getFieldToTagPose(tagId);
            if (!isFinitePose(knownRobotPose) || !isFinitePose(tagPose)) {
                captureStatus = "Need finite known robot and fixed tag poses.";
            } else if (!(frame.frameTimestamp().secondsSince(batchBoundary) > 0.0)) {
                captureStatus = "Wait for an image captured after this setup boundary.";
            } else {
                Pose3d solved = knownRobotPose.inverse().then(tagPose)
                        .then(obs.cameraToTagPose.inverse());
                if (isFinitePose(solved)) {
                    CaptureCandidate candidate = new CaptureCandidate(owner, generation,
                            frame.frameTimestamp(), tagId, knownRobotPose, solved);
                    lastRobotToCameraSample = solved;
                    captureStatus = "Fresh preview; press A to capture a distinct frame.";
                    if (requested && captureInhibitedCycle != ctx.clock.cycle()
                            && sameBatch(candidate.owner, candidate.generation)
                            && candidate.tagId == selectedTagId
                            && candidate.knownRobotPose == fieldToRobotPose) {
                        if (lastAcceptedFrame.isAvailable()
                                && !(candidate.timestamp.secondsSince(lastAcceptedFrame) > 0.0)) {
                            captureStatus = "Frame already counted or older; wait for a new image.";
                        } else if (avg.add(candidate.mountPose)) {
                            lastAcceptedFrame = candidate.timestamp;
                            captureStatus = "Captured one new, fresh frame.";
                        } else {
                            captureStatus = "Capture rejected: finite aggregation could not be preserved.";
                        }
                    }
                } else {
                    captureStatus = "Capture unavailable: mount solve is non-finite. Check geometry.";
                }
            }
        } else if (obs.hasTarget) {
            captureStatus = "Capture unavailable: observed geometry is non-finite.";
        }
        renderCalibrationTelemetry();
    }

    /** Immutable provenance for this loop's solve; never retained as a future A-button sample. */
    private static final class CaptureCandidate {
        final OwnedAprilTagCamera owner;
        final long generation;
        final LoopTimestamp timestamp;
        final int tagId;
        final Pose3d knownRobotPose;
        final Pose3d mountPose;

        CaptureCandidate(OwnedAprilTagCamera owner, long generation, LoopTimestamp timestamp,
                         int tagId, Pose3d knownRobotPose, Pose3d mountPose) {
            this.owner = owner;
            this.generation = generation;
            this.timestamp = timestamp;
            this.tagId = tagId;
            this.knownRobotPose = knownRobotPose;
            this.mountPose = mountPose;
        }
    }

    private static boolean isFinitePose(Pose3d pose) {
        return pose != null && Double.isFinite(pose.xInches) && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.zInches) && Double.isFinite(pose.yawRad)
                && Double.isFinite(pose.pitchRad) && Double.isFinite(pose.rollRad);
    }

    private void renderCalibrationTelemetry() {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        t.addLine("=== Camera Mount Calibrator ===");
        t.addData("Camera", selectedCameraName);
        if (activeVisionDescription != null && !activeVisionDescription.isEmpty()) {
            t.addData("Backend", activeVisionDescription);
        }
        t.addData("Mode [RS]", editMode ? "EDIT" : "QUICK");
        if (editMode) {
            t.addData("Selected field [Dpad U/D]", editField.label);
        }
        t.addData("Step [START]", "%s (XY %.2f in | Yaw %.1f°)",
                fineSteps ? "FINE" : "COARSE",
                stepXY(),
                Math.toDegrees(stepYawRad()));
        t.addData(editableFieldLabel(EditField.TAG_ID, "Y/X"), "%d", selectedTagId);
        t.addData(editableFieldLabel(EditField.ROBOT_X, "Dpad L/R"), "%.2f in", fieldToRobotPose.xInches);
        t.addData(editableFieldLabel(EditField.ROBOT_Y, "Dpad U/D"), "%.2f in", fieldToRobotPose.yInches);
        t.addData(editableFieldLabel(EditField.ROBOT_YAW, "LB/RB"), "%.1f°", Math.toDegrees(fieldToRobotPose.yawRad));
        t.addData("Samples [A capture | B clear]", avg.count());
        t.addData("Capture status", captureStatus);
        t.addData("MaxAge", "%.0f ms", maxDetectionAgeSec * 1000.0);

        t.addLine("");
        t.addLine("Units: inches (angles shown in degrees)");
        t.addLine("Field frame: FTC Field Coordinate System (origin=center, +Z up)");
        t.addLine("FTC axes hint: stand at Red Wall center facing field: +X to your right, +Y away from Red Wall");
        t.addLine("AprilTag note: observation uses SDK rawPose (native AprilTag/OpenCV) converted to Sushi camera axes");
        t.addLine("             (SDK ftcPose is a convenience reframe; don't mix with game database fieldOrientation)");
        if (!editMode) {
            t.addLine("Quick controls: tag [Y/X] | robot X [Dpad L/R] | robot Y [Dpad U/D] | yaw [LB/RB]");
        } else {
            t.addLine("Edit controls: Dpad U/D chooses the field; Dpad L/R or LB/RB changes the selected value.");
        }
        t.addLine("BACK: return to the camera picker. Hold the robot still while capturing.");
        t.addLine("Tag/known-pose edits clear this fixed-setup batch; B wins over A.");

        // Show the known field pose of the selected tag (from the fixed layout or an override layout).
        t.addLine("");
        t.addLine("Selected tag pose from layout (fieldToTagPose):");
        Pose3d selectedTagPose = (layout != null) ? layout.getFieldToTagPose(selectedTagId) : null;
        if (selectedTagPose == null) {
            t.addLine("  (tag not present in layout)");
        } else {
            addPoseLine(t, "fieldToTagPose(layout)", selectedTagPose);
        }
        if (layoutPolicySummary != null) {
            t.addData("Layout policy", layoutPolicySummary);
        }
        FtcTagLayoutDebug.dumpSummary(layout, new FtcTelemetryDebugSink(t), "layout");

        t.addLine("");
        t.addLine("Known robot pose (fieldToRobotPose):");
        addPoseLine(t, "fieldToRobotPose", fieldToRobotPose);

        t.addLine("");
        t.addLine("Observation (cameraToTagPose):");
        if (lastObservedCameraToTag == null) {
            t.addLine("  No fresh detection for selected tag ID.");
        } else {
            addPoseLine(t, "cameraToTagPose(obs)", lastObservedCameraToTag);
        }

        t.addLine("");
        Pose3d mean = avg.meanOrNull();

        t.addLine("Mount solve (robotToCameraPose):");
        if (lastRobotToCameraSample == null) {
            t.addLine("  Need: (1) fresh detection AND (2) this tag present in layout.");
        } else {
            addPoseLine(t, "robotToCameraPose(sample)", lastRobotToCameraSample);

            double mountNorm = translationNormInches(lastRobotToCameraSample);
            if (mountNorm > 36.0) {
                t.addLine(String.format(Locale.US,
                        "WARNING: mount translation is large (|t|=%.1f in). Check fieldToRobotPose + tag ID.",
                        mountNorm
                ));
            }

            if (mean != null) {
                Pose3d avgToSamplePose = mean.inverse().then(lastRobotToCameraSample);
                double sampleDeltaTrans = translationDistanceInches(mean, lastRobotToCameraSample);
                if (isFinitePose(avgToSamplePose) && Double.isFinite(sampleDeltaTrans)) {
                    t.addLine(String.format(Locale.US,
                            "Sample vs avg mount: trans=%.2f in | yaw=%.2f° pitch=%.2f° roll=%.2f°",
                            sampleDeltaTrans,
                            Math.toDegrees(avgToSamplePose.yawRad),
                            Math.toDegrees(avgToSamplePose.pitchRad),
                            Math.toDegrees(avgToSamplePose.rollRad)
                    ));
                } else {
                    t.addLine("Sample vs avg mount: unavailable (non-finite comparison).");
                }
            }

            if (mean != null && selectedTagPose != null && lastObservedCameraToTag != null) {
                // Compare the live observation against the captured-average mount, not against the
                // just-solved sample. Using the same sample on both sides is a tautology and hides
                // bad robot-pose inputs, bad tag-size metadata, and other setup mistakes.
                Pose3d avgPredictedCameraToTag = fieldToRobotPose.then(mean).inverse().then(selectedTagPose);
                Pose3d avgPredToObsPose = avgPredictedCameraToTag.inverse().then(lastObservedCameraToTag);
                double trans = translationNormInches(avgPredToObsPose);
                double observedRange = translationNormInches(lastObservedCameraToTag);
                double predictedRange = translationNormInches(avgPredictedCameraToTag);

                if (isFinitePose(avgPredToObsPose) && Double.isFinite(trans)
                        && Double.isFinite(observedRange) && Double.isFinite(predictedRange)) {
                    t.addLine(String.format(Locale.US,
                            "Avg residual: trans=%.2f in | yaw=%.2f° pitch=%.2f° roll=%.2f°",
                            trans,
                            Math.toDegrees(avgPredToObsPose.yawRad),
                            Math.toDegrees(avgPredToObsPose.pitchRad),
                            Math.toDegrees(avgPredToObsPose.rollRad)
                    ));
                    t.addLine(String.format(Locale.US,
                            "Range check: obs=%.2f in | avgPred=%.2f in | Δ=%.2f in",
                            observedRange,
                            predictedRange,
                            observedRange - predictedRange
                    ));
                } else {
                    t.addLine("Avg residual / range check: unavailable (non-finite comparison).");
                }
            } else if (mean == null) {
                t.addLine("Residual check: needs a usable captured average.");
            }
        }

        t.addLine("");
        t.addLine(String.format(Locale.US, "Captured samples: %d", avg.count()));

        if (mean == null) {
            t.addLine("Average unavailable: " + avg.unavailableReason());
        } else {
            t.addLine("Historical captured average: not a physical accuracy or current-readiness claim.");
            t.addLine("Average mount (paste into CameraMountConfig.of / ofDegrees):");
            addPoseLine(t, "robotToCameraPose(avg)", mean);

            t.addLine(String.format(Locale.US,
                    "CameraMountConfig.of(%.3f, %.3f, %.3f, %.6f, %.6f, %.6f)",
                    mean.xInches, mean.yInches, mean.zInches,
                    mean.yawRad, mean.pitchRad, mean.rollRad
            ));

            t.addLine(String.format(Locale.US,
                    "CameraMountConfig.ofDegrees(%.3f, %.3f, %.3f, %.1f, %.1f, %.1f)",
                    mean.xInches, mean.yInches, mean.zInches,
                    Math.toDegrees(mean.yawRad), Math.toDegrees(mean.pitchRad), Math.toDegrees(mean.rollRad)
            ));
        }

        t.update();
    }

    private String editableFieldLabel(EditField field, String quickControl) {
        if (!editMode) {
            return field.label + " [" + quickControl + "]";
        }
        return (editField == field ? "> " : "  ") + field.label
                + (editField == field ? " [Dpad L/R or LB/RB]" : "");
    }

    private static void addPoseLine(Telemetry t, String label, Pose3d p) {
        t.addLine(String.format(Locale.US,
                "  %s: x=%.2f y=%.2f z=%.2f | yaw=%.1f° pitch=%.1f° roll=%.1f°",
                label,
                p.xInches, p.yInches, p.zInches,
                Math.toDegrees(p.yawRad),
                Math.toDegrees(p.pitchRad),
                Math.toDegrees(p.rollRad)
        ));
    }

    private static double translationDistanceInches(Pose3d a, Pose3d b) {
        double dx = b.xInches - a.xInches;
        double dy = b.yInches - a.yInches;
        double dz = b.zInches - a.zInches;
        return Math.hypot(Math.hypot(dx, dy), dz);
    }

    private static double translationNormInches(Pose3d p) {
        double dx = p.xInches;
        double dy = p.yInches;
        double dz = p.zInches;
        return Math.hypot(Math.hypot(dx, dy), dz);
    }

    // Robot pose adjustment
    private double stepXY() {
        return fineSteps ? 0.25 : 1.0;
    }

    private double stepYawRad() {
        return Math.toRadians(fineSteps ? 0.5 : 2.0);
    }

    private void adjustRobotPose(double dxInches, double dyInches, double dyawRad) {
        Pose3d adjusted = new Pose3d(
                fieldToRobotPose.xInches + dxInches,
                fieldToRobotPose.yInches + dyInches,
                fieldToRobotPose.zInches,
                fieldToRobotPose.yawRad + dyawRad,
                fieldToRobotPose.pitchRad,
                fieldToRobotPose.rollRad
        );
        if (!isFinitePose(adjusted)) return;
        if (adjusted.xInches != fieldToRobotPose.xInches
                || adjusted.yInches != fieldToRobotPose.yInches
                || adjusted.yawRad != fieldToRobotPose.yawRad) {
            clearCapturedSamples();
            fieldToRobotPose = adjusted;
        }
    }

    private void changeTagId(int delta) {
        int next = (int) Math.max(1L, Math.min(Integer.MAX_VALUE, (long) selectedTagId + delta));
        if (next != selectedTagId) {
            clearCapturedSamples();
            selectedTagId = next;
        }
    }


    // Edit-mode helpers
    private void cycleEditField(int delta) {
        EditField[] fields = EditField.values();
        int idx = editField.ordinal();
        int next = (idx + delta) % fields.length;
        if (next < 0) next += fields.length;
        editField = fields[next];
    }

    private void adjustEditField(int dir) {
        switch (editField) {
            case TAG_ID:
                changeTagId(dir > 0 ? 1 : -1);
                break;
            case ROBOT_X:
                adjustRobotPose(dir * stepXY(), 0.0, 0.0);
                break;
            case ROBOT_Y:
                adjustRobotPose(0.0, dir * stepXY(), 0.0);
                break;
            case ROBOT_YAW:
                adjustRobotPose(0.0, 0.0, dir * stepYawRad());
                break;
        }
    }

    // Equal-weight quaternion outer-product mean. All numerical work is bounded and runs only
    // when accepting a sample; rendering reads the cached answer. No physical outlier gate.
    private static final class PoseAverager {
        // Dimensionless tolerances on the normalized (trace ~1) 4x4 matrix, not accuracy scores.
        private static final double OFF_DIAGONAL_TOLERANCE = 1e-14;
        private static final double EIGENPAIR_TOLERANCE = 1e-10;
        private static final int MAX_JACOBI_ROTATIONS = 96;
        private int n = 0;
        private double meanX, meanY, meanZ;
        private double[][] rotationMoment = new double[4][4];
        private Pose3d cachedMean;
        private String unavailableReason = "No captured frames yet; hold still and press A.";

        void clear() {
            n = 0;
            meanX = meanY = meanZ = 0.0;
            rotationMoment = new double[4][4];
            cachedMean = null;
            unavailableReason = "No captured frames yet; hold still and press A.";
        }

        int count() {
            return n;
        }

        boolean add(Pose3d p) {
            if (!isFinitePose(p) || n == Integer.MAX_VALUE) return false;
            int nextCount = n + 1;
            double weight = 1.0 / nextCount;
            double x = blend(meanX, p.xInches, weight);
            double y = blend(meanY, p.yInches, weight);
            double z = blend(meanZ, p.zInches, weight);
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) return false;

            double cy = Math.cos(p.yawRad / 2), sy = Math.sin(p.yawRad / 2);
            double cp = Math.cos(p.pitchRad / 2), sp = Math.sin(p.pitchRad / 2);
            double cr = Math.cos(p.rollRad / 2), sr = Math.sin(p.rollRad / 2);
            double[] q = {cy * cp * cr + sy * sp * sr, cy * cp * sr - sy * sp * cr,
                    cy * sp * cr + sy * cp * sr, sy * cp * cr - cy * sp * sr};
            if (!normalize(q)) return false;
            double[][] nextMoment = new double[4][4];
            for (int i = 0; i < 4; i++) {
                for (int j = i; j < 4; j++) {
                    double value = blend(rotationMoment[i][j], q[i] * q[j], weight);
                    if (!Double.isFinite(value)) return false;
                    nextMoment[i][j] = nextMoment[j][i] = value;
                }
            }
            // q and -q produce the same moment. Contradictory finite captures still belong to the
            // batch: commit their count/statistics even if no unique mean can yet be printed.
            double[] meanRotation = principalRotation(nextMoment);
            Pose3d mean = null;
            if (meanRotation != null) {
                double w = meanRotation[0], qx = meanRotation[1];
                double qy = meanRotation[2], qz = meanRotation[3];
                Mat3 rotation = new Mat3(
                        1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - w * qz), 2 * (qx * qz + w * qy),
                        2 * (qx * qy + w * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - w * qx),
                        2 * (qx * qz - w * qy), 2 * (qy * qz + w * qx), 1 - 2 * (qx * qx + qy * qy));
                // Test singularity from the actual horizontal basis, not cos(asin(m20)): a
                // rounded m20 at an exactly vertical orientation can invent a nonzero cosine.
                double horizontal = Math.hypot(rotation.m00, rotation.m10);
                double pitch = Math.atan2(-rotation.m20, horizontal);
                double yaw = horizontal > 1e-9 ? Math.atan2(rotation.m10, rotation.m00)
                        : Math.atan2(-rotation.m01, rotation.m11);
                double roll = horizontal > 1e-9 ? Math.atan2(rotation.m21, rotation.m22) : 0.0;
                mean = new Pose3d(x, y, z, yaw, pitch, roll);
                if (!isFinitePose(mean)) mean = null;
            }
            n = nextCount;
            meanX = x;
            meanY = y;
            meanZ = z;
            rotationMoment = nextMoment;
            cachedMean = mean;
            unavailableReason = mean == null
                    ? "Rotation mean is ambiguous or numerically unresolved; inspect setup, capture a new frame, or B clear."
                    : "";
            return true;
        }

        Pose3d meanOrNull() {
            return cachedMean;
        }

        String unavailableReason() {
            return unavailableReason;
        }

        private static double blend(double previous, double value, double weight) {
            // Same-sign subtraction cannot overflow; opposite-sign weighted terms cannot overflow
            // their sum. This also handles finite translations whose naive running sum overflows.
            return Math.copySign(1.0, previous) == Math.copySign(1.0, value)
                    ? previous + (value - previous) * weight
                    : previous * (1.0 - weight) + value * weight;
        }

        private static boolean normalize(double[] q) {
            double norm = Math.hypot(Math.hypot(q[0], q[1]), Math.hypot(q[2], q[3]));
            if (!Double.isFinite(norm) || norm == 0.0) return false;
            for (int i = 0; i < 4; i++) q[i] /= norm;
            return true;
        }

        /** Symmetric Jacobi diagonalization examines all four directions (no fixed-start bias). */
        private static double[] principalRotation(double[][] moment) {
            double[][] a = new double[4][4];
            double[][] eigenvectors = new double[4][4];
            for (int i = 0; i < 4; i++) {
                System.arraycopy(moment[i], 0, a[i], 0, 4);
                eigenvectors[i][i] = 1.0;
            }
            boolean converged = false;
            for (int iteration = 0; iteration < MAX_JACOBI_ROTATIONS; iteration++) {
                int p = 0, r = 1;
                double largest = 0;
                for (int i = 0; i < 4; i++) {
                    for (int j = i + 1; j < 4; j++) {
                        if (Math.abs(a[i][j]) > largest) {
                            largest = Math.abs(a[i][j]);
                            p = i;
                            r = j;
                        }
                    }
                }
                if (largest <= OFF_DIAGONAL_TOLERANCE) {
                    converged = true;
                    break;
                }
                double tau = (a[r][r] - a[p][p]) / (2.0 * a[p][r]);
                double t = Math.copySign(1.0, tau) / (Math.abs(tau) + Math.hypot(1.0, tau));
                double c = 1.0 / Math.sqrt(1.0 + t * t), s = t * c;
                double offDiagonal = a[p][r];
                a[p][p] -= t * offDiagonal;
                a[r][r] += t * offDiagonal;
                a[p][r] = a[r][p] = 0;
                for (int k = 0; k < 4; k++) {
                    if (k != p && k != r) {
                        double kp = a[k][p], kr = a[k][r];
                        a[k][p] = a[p][k] = c * kp - s * kr;
                        a[k][r] = a[r][k] = s * kp + c * kr;
                    }
                    double vp = eigenvectors[k][p], vr = eigenvectors[k][r];
                    eigenvectors[k][p] = c * vp - s * vr;
                    eigenvectors[k][r] = s * vp + c * vr;
                }
            }
            if (!converged) return null;
            int top = 0;
            for (int i = 1; i < 4; i++) if (a[i][i] > a[top][top]) top = i;
            double second = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < 4; i++) if (i != top) second = Math.max(second, a[i][i]);
            double eigenvalue = a[top][top];
            if (!Double.isFinite(eigenvalue) || !Double.isFinite(second)
                    || eigenvalue - second <= EIGENPAIR_TOLERANCE) return null;
            double[] q = new double[4];
            for (int i = 0; i < 4; i++) q[i] = eigenvectors[i][top];
            if (!normalize(q)) return null;
            double residual = 0;
            for (int i = 0; i < 4; i++) {
                double component = -eigenvalue * q[i];
                for (int j = 0; j < 4; j++) component += moment[i][j] * q[j];
                residual = Math.hypot(residual, component);
            }
            return Double.isFinite(residual) && residual <= EIGENPAIR_TOLERANCE ? q : null;
        }
    }
}
