package edu.ftcphoenix.fw.tools.tester.calibration;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.Locale;
import java.util.function.Function;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.FtcTagLayoutDebug;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactories;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactory;
import edu.ftcphoenix.fw.ftc.vision.FtcWebcamAprilTagVisionLane;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.tools.tester.ui.HardwareNamePicker;

/**
 * Calibrates {@code robotToCameraPose} (camera mount extrinsics) using:
 * <ul>
 *   <li>Known AprilTag field layout (framework-owned current-game fixed layout by default), and</li>
 *   <li>A manually-entered / adjustable known robot pose {@code fieldToRobotPose}.</li>
 * </ul>
 *
 * <h2>Camera selection</h2>
 * <p>
 * If constructed without a preferred hardware name (or if the preferred device cannot be
 * initialized), this tester shows a picker for the configured vision-device type and lets you
 * choose one before calibration begins.
 * </p>
 *
 * <h2>Core math</h2>
 * <pre>
 * fieldToTagPose = fieldToRobotPose · robotToCameraPose · cameraToTagPose
 * => robotToCameraPose = inv(fieldToRobotPose) · fieldToTagPose · inv(cameraToTagPose)
 * </pre>
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>PICKER (no camera chosen yet)</b>: Dpad Up/Down highlight, A choose, B refresh</li>
 *   <li><b>CALIBRATE (camera chosen)</b>:
 *     <ul>
 *       <li>Y/X: increment/decrement tag ID</li>
 *       <li>A: capture sample (average mount)</li>
 *       <li>B: clear captured samples</li>
 *       <li>Dpad: adjust known robot pose (XY)</li>
 *       <li>LB/RB: adjust known robot yaw</li>
 *       <li>START: fine/coarse step</li>
 *       <li>BACK: return to camera picker (change camera)</li>
 *     </ul>
 *   </li>
 * </ul>
 */
public final class CameraMountCalibrator extends BaseTeleOpTester {

    // Defaults
    private static final double DEFAULT_MAX_AGE_SEC = 0.35;
    private static final int DEFAULT_TAG_ID = 1;

    private static final Pose3d DEFAULT_P_FIELD_TO_ROBOT = Pose3d.zero();

    // Injected configuration
    private final String preferredCameraName;   // may be null/empty to trigger picker
    private final Class<? extends HardwareDevice> cameraDeviceType;
    private final String cameraPickerTitle;
    private final Function<String, AprilTagVisionLaneFactory> cameraLaneFactoryBuilder;
    private final TagLayout layoutOverride;     // may be null => framework current-game fixed layout
    private final double maxAgeSec;

    // Runtime state
    private TagLayout layout;
    private AprilTagVisionLane visionLane;
    private AprilTagSensor tagSensor;

    private boolean visionReady = false;
    private String selectedCameraName = null;
    private String visionInitError = null;
    private String activeVisionDescription = null;

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

    // Construction

    /**
     * Create a calibrator with default settings.
     *
     * <p>This constructor does not force a specific camera name. The tester will present a camera
     * picker menu of configured webcams so you can choose one.</p>
     */
    public CameraMountCalibrator() {
        this(null, null, null, DEFAULT_MAX_AGE_SEC);
    }

    /**
     * Create a calibrator that prefers a specific camera name.
     *
     * <p>If {@code cameraName} is null/blank, the tester will fall back to the camera picker menu.</p>
     *
     * @param cameraName configured webcam name in the FTC Robot Configuration (nullable)
     */
    public CameraMountCalibrator(String cameraName) {
        this(cameraName, null, null, DEFAULT_MAX_AGE_SEC);
    }

    /**
     * Create a calibrator with full configuration control.
     *
     * @param cameraName     configured webcam name in the FTC Robot Configuration (nullable/blank to use picker)
     * @param layoutOverride optional field {@link TagLayout}; if null, the framework's current-game fixed layout is used
     * @param maxAgeSec      maximum age (seconds) for tag observations before they are treated as stale
     */
    public CameraMountCalibrator(String cameraName, TagLayout layoutOverride, double maxAgeSec) {
        this(cameraName, layoutOverride, null, maxAgeSec);
    }

    /**
     * Creates a camera mount calibration tester with an optional tag layout + tag library override.
     *
     * <p>This is useful when calibrating in a non-game environment using a printed tag (custom ID/size),
     * while still reusing Phoenix's calibrator flow.</p>
     *
     * @param cameraName         camera name in the hardware map (or {@code null} to pick at runtime)
     * @param layoutOverride     optional tag layout to use instead of the framework current-game fixed layout
     * @param tagLibraryOverride optional AprilTag library override (controls tag size/IDs for detection)
     * @param maxAgeSec          maximum acceptable tag observation age in seconds
     */
    public CameraMountCalibrator(String cameraName,
                                 TagLayout layoutOverride,
                                 AprilTagLibrary tagLibraryOverride,
                                 double maxAgeSec) {
        this(cameraName,
                WebcamName.class,
                "Select Camera",
                defaultWebcamLaneFactoryBuilder(tagLibraryOverride),
                layoutOverride,
                maxAgeSec);
    }

    /**
     * Creates a backend-neutral camera-mount calibrator.
     *
     * <p>This constructor is the one Phoenix and future smart-camera adopters should prefer. The
     * tester remains responsible only for the calibration workflow; the caller decides which FTC
     * hardware type to enumerate and how to open the concrete AprilTag lane once a device name has
     * been chosen.</p>
     *
     * @param preferredCameraName      preferred hardware-map name for the active vision device (nullable)
     * @param cameraDeviceType         FTC hardware type to enumerate in the picker (for example {@link WebcamName})
     * @param cameraPickerTitle        telemetry title for the picker screen
     * @param cameraLaneFactoryBuilder builder that turns a chosen hardware-map name into an opener
     * @param layoutOverride           optional fixed-tag layout override (nullable)
     * @param maxAgeSec                maximum acceptable tag observation age in seconds
     */
    public CameraMountCalibrator(String preferredCameraName,
                                 Class<? extends HardwareDevice> cameraDeviceType,
                                 String cameraPickerTitle,
                                 Function<String, AprilTagVisionLaneFactory> cameraLaneFactoryBuilder,
                                 TagLayout layoutOverride,
                                 double maxAgeSec) {
        this.preferredCameraName = preferredCameraName;
        this.cameraDeviceType = cameraDeviceType != null ? cameraDeviceType : WebcamName.class;
        this.cameraPickerTitle = (cameraPickerTitle == null || cameraPickerTitle.trim().isEmpty())
                ? "Select Vision Device"
                : cameraPickerTitle;
        this.cameraLaneFactoryBuilder = cameraLaneFactoryBuilder != null
                ? cameraLaneFactoryBuilder
                : defaultWebcamLaneFactoryBuilder(null);
        this.layoutOverride = layoutOverride;
        if (maxAgeSec < 0.0) {
            throw new IllegalArgumentException("maxAgeSec must be non-negative");
        }
        this.maxAgeSec = maxAgeSec;
    }

    private static Function<String, AprilTagVisionLaneFactory> defaultWebcamLaneFactoryBuilder(AprilTagLibrary tagLibraryOverride) {
        return cameraName -> {
            FtcWebcamAprilTagVisionLane.Config cfg = FtcWebcamAprilTagVisionLane.Config.defaults();
            cfg.webcamName = cameraName;
            cfg.tagLibrary = tagLibraryOverride;
            return AprilTagVisionLaneFactories.webcam(cfg);
        };
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
        // Layout
        this.layout = (layoutOverride != null)
                ? layoutOverride
                : FtcGameTagLayout.currentGameFieldFixed();

        if (layout != null && !layout.ids().isEmpty()) {
            selectedTagId = layout.ids().iterator().next();
        }

        // Camera selection setup
        selectedCameraName = (preferredCameraName == null || preferredCameraName.trim().isEmpty())
                ? null
                : preferredCameraName.trim();

        cameraPicker = new HardwareNamePicker(
                ctx.hw,
                cameraDeviceType,
                cameraPickerTitle,
                "Dpad: highlight | A: choose | B: refresh"
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
                gamepads.p1().b(),
                () -> !visionReady,
                chosen -> {
                    selectedCameraName = chosen;
                    ensureVisionReady();
                }
        );

        // B clears samples once vision is already running.
        bindings.onRise(gamepads.p1().b(), () -> {
            if (visionReady) {
                avg.clear();
            }
        });

        // Capture sample (only when vision is ready)
        bindings.onRise(gamepads.p1().a(), () -> {
            if (!visionReady) return;
            if (lastRobotToCameraSample != null) {
                avg.add(lastRobotToCameraSample);
            }
        });

        // Calibration controls (only when vision is ready)
        bindings.onRise(gamepads.p1().y(), () -> {
            if (!visionReady) return;
            selectedTagId++;
        });

        bindings.onRise(gamepads.p1().x(), () -> {
            if (!visionReady) return;
            selectedTagId = Math.max(1, selectedTagId - 1);
        });

        bindings.onRise(gamepads.p1().start(), () -> {
            if (!visionReady) return;
            fineSteps = !fineSteps;
        });

        // Toggle edit mode (RS). Edit mode lets you select which variable you're changing
        // instead of remembering which button maps to which axis.
        bindings.onRise(gamepads.p1().rs(), () -> {
            if (!visionReady) return;
            editMode = !editMode;
        });

        // D-pad:
        //  - QUICK mode: dpad X adjusts X, dpad Y adjusts Y (requested)
        //  - EDIT mode: up/down selects a field, left/right changes its value
        bindings.onRise(gamepads.p1().dpadUp(), () -> {
            if (!visionReady) return;
            if (editMode) {
                cycleEditField(-1);
            } else {
                adjustRobotPose(0.0, +stepXY(), 0.0);
            }
        });
        bindings.onRise(gamepads.p1().dpadDown(), () -> {
            if (!visionReady) return;
            if (editMode) {
                cycleEditField(+1);
            } else {
                adjustRobotPose(0.0, -stepXY(), 0.0);
            }
        });

        bindings.onRise(gamepads.p1().dpadLeft(), () -> {
            if (!visionReady) return;
            if (editMode) {
                adjustEditField(-1);
            } else {
                adjustRobotPose(-stepXY(), 0.0, 0.0);
            }
        });
        bindings.onRise(gamepads.p1().dpadRight(), () -> {
            if (!visionReady) return;
            if (editMode) {
                adjustEditField(+1);
            } else {
                adjustRobotPose(+stepXY(), 0.0, 0.0);
            }
        });

        // Yaw adjustment. In QUICK mode we keep the classic LB/RB mapping.
        // In EDIT mode, bumpers behave like +/- on the selected field.
        bindings.onRise(gamepads.p1().leftBumper(), () -> {
            if (!visionReady) return;
            if (editMode) {
                adjustEditField(-1);
            } else {
                adjustRobotPose(0.0, 0.0, +stepYawRad());
            }
        });
        bindings.onRise(gamepads.p1().rightBumper(), () -> {
            if (!visionReady) return;
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
        if (!visionReady) {
            return false;
        }

        // Return to the camera picker. This allows you to re-select the active vision
        // device without leaving the tester suite.
        visionReady = false;
        visionInitError = null;

        editMode = false;
        editField = EditField.ROBOT_X;

        if (visionLane != null) {
            visionLane.close();
        }
        visionLane = null;
        tagSensor = null;
        activeVisionDescription = null;

        lastRobotToCameraSample = null;
        lastObservedCameraToTag = null;

        avg.clear();

        // Rebuild menu entries and keep the last chosen camera highlighted.
        if (cameraPicker != null) {
            cameraPicker.clearChoice();
            cameraPicker.refresh();
            if (selectedCameraName != null && !selectedCameraName.isEmpty()) {
                cameraPicker.setPreferredName(selectedCameraName);
            }
        }

        return true;
    }

    @Override
    protected void onStop() {
        if (visionLane != null) {
            visionLane.close();
            visionLane = null;
        }
        tagSensor = null;
        activeVisionDescription = null;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onInitLoop(double dtSec) {
        if (!visionReady) {
            renderCameraPicker();
            return;
        }

        updateSolveAndTelemetry();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onLoop(double dtSec) {
        if (!visionReady) {
            renderCameraPicker();
            return;
        }

        updateSolveAndTelemetry();
    }

    // ---------------------------------------------------------------------------------------------
    // Vision init / camera enumeration
    // ---------------------------------------------------------------------------------------------

    private void ensureVisionReady() {
        if (visionReady) return;
        if (selectedCameraName == null || selectedCameraName.isEmpty()) return;

        try {
            AprilTagVisionLaneFactory factory = cameraLaneFactoryBuilder.apply(selectedCameraName);
            if (factory == null) {
                throw new IllegalStateException("cameraLaneFactoryBuilder returned null for " + selectedCameraName);
            }

            visionLane = factory.open(ctx.hw);
            tagSensor = visionLane.tagSensor();
            activeVisionDescription = factory.description();

            // Layout selection happens in onInit(); do not rebuild it here.
            visionReady = true;
            visionInitError = null;
        } catch (Exception e) {
            if (visionLane != null) {
                try {
                    visionLane.close();
                } catch (Exception ignored) {
                    // Best effort only.
                }
            }
            visionLane = null;
            tagSensor = null;
            activeVisionDescription = null;
            visionReady = false;
            visionInitError = "Failed to start AprilTag camera: " + e.getMessage();
        }
    }

    private void renderCameraPicker() {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        if (cameraPicker != null) {
            cameraPicker.render(t);
        }

        t.addLine("");
        t.addLine("Chosen: " + (selectedCameraName == null ? "(none)" : selectedCameraName));
        t.addLine("Press A to choose the active vision device and initialize AprilTags.");
        t.addLine("Press B to refresh camera list.");
        t.addLine("Press BACK to exit to the tester menu.");

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
        AprilTagObservation obs = tagSensor.get(ctx.clock).forId(selectedTagId, maxAgeSec);
        lastObservedCameraToTag = (obs.hasTarget) ? obs.cameraToTagPose : null;

        lastRobotToCameraSample = null;

        if (obs.hasTarget) {
            Pose3d fieldToTagPose = layout.getFieldToTagPose(obs.id);
            if (fieldToTagPose != null) {
                Pose3d cameraToTagPose = obs.cameraToTagPose;

                Pose3d robotToCameraPose = fieldToRobotPose.inverse()
                        .then(fieldToTagPose)
                        .then(cameraToTagPose.inverse());

                lastRobotToCameraSample = robotToCameraPose;

            }
        }

        renderCalibrationTelemetry();
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
        t.addData("MaxAge", "%.0f ms", maxAgeSec * 1000.0);

        t.addLine("");
        t.addLine("Units: inches (angles shown in degrees)");
        t.addLine("Field frame: FTC Field Coordinate System (origin=center, +Z up)");
        t.addLine("FTC axes hint: stand at Red Wall center facing field: +X to your right, +Y away from Red Wall");
        t.addLine("AprilTag note: observation uses SDK rawPose (native AprilTag/OpenCV) converted to Phoenix camera axes");
        t.addLine("             (SDK ftcPose is a convenience reframe; don't mix with game database fieldOrientation)");
        if (!editMode) {
            t.addLine("Quick controls: tag [Y/X] | robot X [Dpad L/R] | robot Y [Dpad U/D] | yaw [LB/RB]");
        } else {
            t.addLine("Edit controls: Dpad U/D chooses the field; Dpad L/R or LB/RB changes the selected value.");
        }
        t.addLine("BACK: return to the camera picker. Hold the robot still while capturing.");

        // Show the known field pose of the selected tag (from the fixed layout or an override layout).
        t.addLine("");
        t.addLine("Selected tag pose from layout (fieldToTagPose):");
        Pose3d selectedTagPose = (layout != null) ? layout.getFieldToTagPose(selectedTagId) : null;
        if (selectedTagPose == null) {
            t.addLine("  (tag not present in layout)");
        } else {
            addPoseLine(t, "fieldToTagPose(layout)", selectedTagPose);
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
                t.addLine(String.format(Locale.US,
                        "Sample vs avg mount: trans=%.2f in | yaw=%.2f° pitch=%.2f° roll=%.2f°",
                        sampleDeltaTrans,
                        Math.toDegrees(avgToSamplePose.yawRad),
                        Math.toDegrees(avgToSamplePose.pitchRad),
                        Math.toDegrees(avgToSamplePose.rollRad)
                ));
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
            } else if (mean == null) {
                t.addLine("Residual check: capture at least one sample to compare against an averaged mount.");
            }
        }

        t.addLine("");
        t.addLine(String.format(Locale.US, "Captured samples: %d", avg.count()));

        if (mean == null) {
            t.addLine("Average: (none yet) Press A a few times while holding still.");
        } else {
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
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    private static double translationNormInches(Pose3d p) {
        double dx = p.xInches;
        double dy = p.yInches;
        double dz = p.zInches;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Robot pose adjustment
    private double stepXY() {
        return fineSteps ? 0.25 : 1.0;
    }

    private double stepYawRad() {
        return Math.toRadians(fineSteps ? 0.5 : 2.0);
    }

    private void adjustRobotPose(double dxInches, double dyInches, double dyawRad) {
        fieldToRobotPose = new Pose3d(
                fieldToRobotPose.xInches + dxInches,
                fieldToRobotPose.yInches + dyInches,
                fieldToRobotPose.zInches,
                fieldToRobotPose.yawRad + dyawRad,
                fieldToRobotPose.pitchRad,
                fieldToRobotPose.rollRad
        );
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
                if (dir > 0) {
                    selectedTagId++;
                } else {
                    selectedTagId = Math.max(1, selectedTagId - 1);
                }
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

    // Averager
    private static final class PoseAverager {
        private int n = 0;

        private double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
        private double sumSinYaw = 0.0, sumCosYaw = 0.0;
        private double sumSinPitch = 0.0, sumCosPitch = 0.0;
        private double sumSinRoll = 0.0, sumCosRoll = 0.0;

        void clear() {
            n = 0;
            sumX = sumY = sumZ = 0.0;
            sumSinYaw = sumCosYaw = 0.0;
            sumSinPitch = sumCosPitch = 0.0;
            sumSinRoll = sumCosRoll = 0.0;
        }

        int count() {
            return n;
        }

        void add(Pose3d p) {
            n++;
            sumX += p.xInches;
            sumY += p.yInches;
            sumZ += p.zInches;

            sumSinYaw += Math.sin(p.yawRad);
            sumCosYaw += Math.cos(p.yawRad);

            sumSinPitch += Math.sin(p.pitchRad);
            sumCosPitch += Math.cos(p.pitchRad);

            sumSinRoll += Math.sin(p.rollRad);
            sumCosRoll += Math.cos(p.rollRad);
        }

        Pose3d meanOrNull() {
            if (n <= 0) return null;

            double x = sumX / n;
            double y = sumY / n;
            double z = sumZ / n;

            double yaw = Math.atan2(sumSinYaw, sumCosYaw);
            double pitch = Math.atan2(sumSinPitch, sumCosPitch);
            double roll = Math.atan2(sumSinRoll, sumCosRoll);

            return new Pose3d(x, y, z, yaw, pitch, roll);
        }
    }
}
