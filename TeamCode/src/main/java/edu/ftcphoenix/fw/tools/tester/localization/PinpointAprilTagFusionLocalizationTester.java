package edu.ftcphoenix.fw.tools.tester.localization;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.Collections;
import java.util.Locale;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.FtcTagLayoutDebug;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.ftc.FtcVision;
import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.apriltag.TagOnlyPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagEkfPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagFusionPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.VisionCorrectionPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.VisionCorrectionStats;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionPolicies;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelections;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.tools.tester.ui.HardwareNamePicker;

/**
 * End-to-end tester for "global" localization using:
 * <ul>
 *   <li>goBILDA Pinpoint odometry for continuous motion integration</li>
 *   <li>AprilTags for occasional absolute pose corrections</li>
 * </ul>
 *
 * <p>This tester exists mainly to validate the "tags disappear near the target" scenario:
 * drive toward a tag, let the global estimator settle, then rotate/score (tag not visible) and
 * confirm the pose continues to update smoothly based on odometry.</p>
 *
 * <p>The tester can exercise either the lightweight fusion localizer or the optional EKF-style
 * localizer. The EKF path is intentionally opt-in so teams can compare behavior without losing the
 * simpler, easier-to-debug default.</p>
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>PICKER</b>: Dpad Up/Down highlight, A choose, B refresh</li>
 *   <li><b>RUN</b>:
 *     <ul>
 *       <li>START: toggle tracking mode (ANY tag in layout vs SINGLE chosen ID)</li>
 *       <li>Y/X: increment/decrement the chosen tag ID (used in SINGLE mode)</li>
 *       <li>A: snap estimator pose to the current vision pose (when available)</li>
 *       <li>B: toggle vision fusion on/off</li>
 *       <li>RB: set estimator pose to (0,0,0)</li>
 *       <li>BACK: return to camera picker</li>
 *     </ul>
 *   </li>
 * </ul>
 */
public final class PinpointAprilTagFusionLocalizationTester extends BaseTeleOpTester {

    /** Localizer flavor exercised by the tester. */
    public enum EstimatorMode {
        FUSION,
        EKF
    }

    private static final double DEFAULT_MAX_AGE_SEC = 0.35;
    private static final int DEFAULT_TAG_ID = 1;

    private final String preferredCameraName;
    private final CameraMountConfig cameraMount;
    private final PinpointPoseEstimator.Config pinpointCfg;
    private final EstimatorMode estimatorMode;
    private final OdometryTagFusionPoseEstimator.Config fusionCfg;
    private final OdometryTagEkfPoseEstimator.Config ekfCfg;

    private final TagLayout layoutOverride;
    private final AprilTagLibrary tagLibraryOverride;
    private final TagOnlyPoseEstimator.Config visionEstimatorConfigOverride;

    private HardwareNamePicker cameraPicker;
    private String selectedCameraName = null;

    private boolean ready = false;
    private String initError = null;

    private TagLayout layout;

    // Vision pieces
    private AprilTagSensor tagSensor;
    private TagSelectionSource selection;
    private TagOnlyPoseEstimator visionEstimator;

    // Odometry + global localization
    private PinpointPoseEstimator odomEstimator;
    private VisionCorrectionPoseEstimator globalEstimator;

    private boolean trackAny = true;
    private int selectedTagId = DEFAULT_TAG_ID;

    /**
     * Creates the tester with the default lightweight fusion configuration.
     */
    public PinpointAprilTagFusionLocalizationTester(String preferredCameraName,
                                                    CameraMountConfig cameraMount,
                                                    PinpointPoseEstimator.Config pinpointCfg) {
        this(preferredCameraName,
                cameraMount,
                pinpointCfg,
                EstimatorMode.FUSION,
                OdometryTagFusionPoseEstimator.Config.defaults(),
                null,
                null,
                null,
                null);
    }

    /**
     * Creates the tester with an explicit lightweight fusion configuration.
     */
    public PinpointAprilTagFusionLocalizationTester(String preferredCameraName,
                                                    CameraMountConfig cameraMount,
                                                    PinpointPoseEstimator.Config pinpointCfg,
                                                    OdometryTagFusionPoseEstimator.Config fusionCfg) {
        this(preferredCameraName,
                cameraMount,
                pinpointCfg,
                EstimatorMode.FUSION,
                fusionCfg,
                null,
                null,
                null,
                null);
    }

    /**
     * Creates the tester with explicit lightweight fusion configuration plus optional
     * field-layout/library and AprilTag-localizer overrides.
     *
     * <p>When {@code visionEstimatorConfigOverride} is supplied, its
     * {@code maxDetectionAgeSec} becomes the effective freshness gate used by both the selected-tag
     * preview and the AprilTag vision estimator.</p>
     */
    public PinpointAprilTagFusionLocalizationTester(String preferredCameraName,
                                                    CameraMountConfig cameraMount,
                                                    PinpointPoseEstimator.Config pinpointCfg,
                                                    OdometryTagFusionPoseEstimator.Config fusionCfg,
                                                    TagLayout layoutOverride,
                                                    AprilTagLibrary tagLibraryOverride,
                                                    TagOnlyPoseEstimator.Config visionEstimatorConfigOverride) {
        this(preferredCameraName,
                cameraMount,
                pinpointCfg,
                EstimatorMode.FUSION,
                fusionCfg,
                null,
                layoutOverride,
                tagLibraryOverride,
                visionEstimatorConfigOverride);
    }

    /**
     * Creates the tester in EKF mode with the supplied configuration and optional overrides.
     */
    public static PinpointAprilTagFusionLocalizationTester ekf(String preferredCameraName,
                                                               CameraMountConfig cameraMount,
                                                               PinpointPoseEstimator.Config pinpointCfg,
                                                               OdometryTagEkfPoseEstimator.Config ekfCfg,
                                                               TagLayout layoutOverride,
                                                               AprilTagLibrary tagLibraryOverride,
                                                               TagOnlyPoseEstimator.Config visionEstimatorConfigOverride) {
        return new PinpointAprilTagFusionLocalizationTester(preferredCameraName,
                cameraMount,
                pinpointCfg,
                EstimatorMode.EKF,
                null,
                ekfCfg,
                layoutOverride,
                tagLibraryOverride,
                visionEstimatorConfigOverride);
    }

    /**
     * Creates the tester in EKF mode with the supplied configuration.
     */
    public static PinpointAprilTagFusionLocalizationTester ekf(String preferredCameraName,
                                                               CameraMountConfig cameraMount,
                                                               PinpointPoseEstimator.Config pinpointCfg,
                                                               OdometryTagEkfPoseEstimator.Config ekfCfg) {
        return ekf(preferredCameraName, cameraMount, pinpointCfg, ekfCfg, null, null, null);
    }

    private PinpointAprilTagFusionLocalizationTester(String preferredCameraName,
                                                     CameraMountConfig cameraMount,
                                                     PinpointPoseEstimator.Config pinpointCfg,
                                                     EstimatorMode estimatorMode,
                                                     OdometryTagFusionPoseEstimator.Config fusionCfg,
                                                     OdometryTagEkfPoseEstimator.Config ekfCfg,
                                                     TagLayout layoutOverride,
                                                     AprilTagLibrary tagLibraryOverride,
                                                     TagOnlyPoseEstimator.Config visionEstimatorConfigOverride) {
        this.preferredCameraName = preferredCameraName;
        this.cameraMount = cameraMount;
        this.pinpointCfg = pinpointCfg != null ? pinpointCfg : PinpointPoseEstimator.Config.defaults();
        this.estimatorMode = estimatorMode != null ? estimatorMode : EstimatorMode.FUSION;
        this.fusionCfg = fusionCfg != null ? fusionCfg : OdometryTagFusionPoseEstimator.Config.defaults();
        this.ekfCfg = ekfCfg != null ? ekfCfg : OdometryTagEkfPoseEstimator.Config.defaults();
        this.layoutOverride = layoutOverride;
        this.tagLibraryOverride = tagLibraryOverride;
        this.visionEstimatorConfigOverride = visionEstimatorConfigOverride != null
                ? visionEstimatorConfigOverride.copy()
                : null;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String name() {
        return estimatorMode == EstimatorMode.EKF
                ? "Loc: Pinpoint + AprilTags (EKF)"
                : "Loc: Pinpoint + AprilTags (Fused)";
    }

    @Override
    protected void onInit() {
        cameraPicker = new HardwareNamePicker(
                ctx.hw,
                WebcamName.class,
                "Select Camera",
                "Dpad: highlight | A: choose | B: refresh"
        );
        cameraPicker.refresh();

        // Prefer RobotConfig's camera if provided.
        if (preferredCameraName != null && !preferredCameraName.trim().isEmpty()) {
            selectedCameraName = preferredCameraName.trim();
            cameraPicker.setPreferredName(selectedCameraName);
            ensureReady();
        }

        cameraPicker.bind(
                bindings,
                gamepads.p1().dpadUp(),
                gamepads.p1().dpadDown(),
                gamepads.p1().a(),
                gamepads.p1().b(),
                () -> !ready,
                chosen -> {
                    selectedCameraName = chosen;
                    ensureReady();
                }
        );

        // B: refresh picker OR toggle vision fusion.
        bindings.onRise(gamepads.p1().b(), () -> {
            if (!ready) {
                cameraPicker.refresh();
            } else if (globalEstimator != null) {
                globalEstimator.setVisionEnabled(!globalEstimator.isVisionEnabled());
            }
        });

        // START: toggle tag tracking mode.
        bindings.onRise(gamepads.p1().start(), () -> {
            if (!ready) return;
            trackAny = !trackAny;
            rebuildSelectionAndEstimators();
        });

        bindings.onRise(gamepads.p1().y(), () -> {
            if (!ready) return;
            selectedTagId++;
            if (!trackAny) {
                rebuildSelectionAndEstimators();
            }
        });

        bindings.onRise(gamepads.p1().x(), () -> {
            if (!ready) return;
            selectedTagId = Math.max(1, selectedTagId - 1);
            if (!trackAny) {
                rebuildSelectionAndEstimators();
            }
        });

        // A: snap estimator pose to current vision estimate.
        bindings.onRise(gamepads.p1().a(), () -> {
            if (!ready || globalEstimator == null || visionEstimator == null) return;
            PoseEstimate v = visionEstimator.getEstimate();
            if (v != null && v.hasPose) {
                globalEstimator.setPose(v.toPose2d());
            }
        });

        // RB: reset estimator pose to 0,0,0.
        bindings.onRise(gamepads.p1().rightBumper(), () -> {
            if (!ready || globalEstimator == null) return;
            globalEstimator.setPose(Pose2d.zero());
        });
    }

    @Override
    protected void onInitLoop(double dtSec) {
        if (!ready) {
            renderPicker();
            return;
        }
        updateAndRender();
    }

    @Override
    protected void onLoop(double dtSec) {
        if (!ready) {
            renderPicker();
            return;
        }
        updateAndRender();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean onBackPressed() {
        if (!ready) {
            return false;
        }

        // Return to picker.
        ready = false;
        initError = null;
        layout = null;
        if (tagSensor != null) {
            tagSensor.close();
        }
        tagSensor = null;
        selection = null;
        visionEstimator = null;
        odomEstimator = null;
        globalEstimator = null;
        return true;
    }

    @Override
    protected void onStop() {
        if (tagSensor != null) {
            tagSensor.close();
            tagSensor = null;
        }
    }

    private void renderPicker() {
        Telemetry t = ctx.telemetry;
        t.addLine("--- " + name() + " ---");
        if (initError != null) {
            t.addLine("Init error:");
            t.addLine(initError);
            t.addLine();
        }
        cameraPicker.render(t);
        t.update();
    }

    private void ensureReady() {
        if (ready) return;
        if (selectedCameraName == null || selectedCameraName.trim().isEmpty()) {
            initError = "No camera selected";
            return;
        }

        try {
            FtcVision.Config cfg = FtcVision.Config.defaults()
                    .withCameraMount(cameraMount);
            if (tagLibraryOverride != null) {
                cfg.withTagLibrary(tagLibraryOverride);
            }
            tagSensor = FtcVision.aprilTags(ctx.hw, selectedCameraName, cfg);

            odomEstimator = new PinpointPoseEstimator(ctx.hw, pinpointCfg);

            layout = (layoutOverride != null)
                    ? layoutOverride
                    : FtcGameTagLayout.currentGameFieldFixed();

            rebuildSelectionAndEstimators();

            ready = true;
            initError = null;
        } catch (Exception e) {
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
            ready = false;
        }
    }

    private void rebuildSelectionAndEstimators() {
        if (tagSensor == null || layout == null) {
            return;
        }

        Set<Integer> ids = trackAny
                ? layout.ids()
                : Collections.singleton(selectedTagId);

        if (ids == null || ids.isEmpty()) {
            ids = Collections.singleton(selectedTagId);
            trackAny = false;
        }

        double effectiveVisionMaxAgeSec = effectiveVisionMaxAgeSec();

        selection = TagSelections.from(tagSensor)
                .among(ids)
                .freshWithin(effectiveVisionMaxAgeSec)
                .choose(TagSelectionPolicies.closestRange())
                .continuous()
                .build();

        TagOnlyPoseEstimator.Config tagCfg = (visionEstimatorConfigOverride != null)
                ? visionEstimatorConfigOverride.copy()
                : TagOnlyPoseEstimator.Config.defaults().withMaxDetectionAgeSec(effectiveVisionMaxAgeSec);
        tagCfg.withCameraMount(cameraMount);
        tagCfg.maxAbsBearingRad = 0.0;
        visionEstimator = new TagOnlyPoseEstimator(tagSensor, layout, tagCfg);

        globalEstimator = (estimatorMode == EstimatorMode.EKF)
                ? new OdometryTagEkfPoseEstimator(odomEstimator, visionEstimator, ekfCfg)
                : new OdometryTagFusionPoseEstimator(odomEstimator, visionEstimator, fusionCfg);
    }

    private double effectiveVisionMaxAgeSec() {
        return (visionEstimatorConfigOverride != null)
                ? visionEstimatorConfigOverride.maxDetectionAgeSec
                : DEFAULT_MAX_AGE_SEC;
    }

    private void updateAndRender() {
        globalEstimator.update(clock);

        PoseEstimate odom = odomEstimator.getEstimate();
        PoseEstimate vis = visionEstimator.getEstimate();
        PoseEstimate global = globalEstimator.getEstimate();
        VisionCorrectionStats stats = globalEstimator.getVisionCorrectionStats();

        Telemetry t = ctx.telemetry;
        t.addLine("--- " + name() + " ---");
        t.addLine("Controls: START any/single | X/Y tag id | A snap-to-vision | B vision on/off | RB zero | BACK picker");
        t.addLine();

        if (isLikelyIdentity(cameraMount)) {
            t.addLine("NOTE: CameraMountConfig looks uncalibrated (identity). Run 'Calib: Camera Mount'.");
        }
        if (isLikelyDefaultPinpointOffsets(pinpointCfg)) {
            t.addLine("NOTE: Pinpoint pod offsets look default (0/0). Run 'Calib: Pinpoint Pod Offsets'.");
        }
        if (isLikelyIdentity(cameraMount) || isLikelyDefaultPinpointOffsets(pinpointCfg)) {
            t.addLine("NOTE: Both fusion and EKF estimators depend on accurate camera extrinsics and odometry offsets.");
            t.addLine();
        }

        t.addData("Camera", selectedCameraName);
        t.addData("Estimator", estimatorMode);
        t.addData("Mode", trackAny ? "ANY" : ("SINGLE (id=" + selectedTagId + ")"));
        t.addData("Vision Enabled", globalEstimator.isVisionEnabled());
        t.addData("Vision MaxAge", "%.0f ms", effectiveVisionMaxAgeSec() * 1000.0);
        t.addData("Vision Accept", "%d ok / %d rej", stats.acceptedVisionCount, stats.rejectedVisionCount);
        t.addData("Vision Path", "%d replay / %d projected", stats.replayedVisionCount, stats.projectedVisionCount);
        t.addData("Vision Skip", "%d dup / %d old", stats.skippedDuplicateVisionCount, stats.skippedOutOfOrderVisionCount);
        if (layout instanceof FtcGameTagLayout) {
            FtcGameTagLayout ftcLayout = (FtcGameTagLayout) layout;
            t.addData("Layout Policy", ftcLayout.policySummaryLine());
        } else if (layout != null) {
            t.addData("Layout IDs", layout.ids());
        }

        TagSelectionResult selectionResult = selection.get(clock);
        AprilTagObservation obs = selectionResult.hasFreshSelectedObservation
                ? selectionResult.selectedObservation
                : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
        if (obs == null || !obs.hasTarget) {
            t.addData("Last Tag", "(none)");
        } else {
            t.addData("Last Tag", "id=%d age=%.0fms", obs.id, obs.ageSec * 1000.0);
            t.addData("Tag Range", "%.1f in", obs.cameraRangeInches());
            t.addData("Tag Bearing", "%.1f deg", Math.toDegrees(obs.cameraBearingRad()));
        }

        t.addLine();

        renderPose(t, "Odom", odom);
        renderPose(t, "Vision", vis);
        renderPose(t, estimatorMode == EstimatorMode.EKF ? "EKF" : "Fused", global);

        if (global != null && global.hasPose && global.fieldToRobotPose != null) {
            Pose3d p = global.fieldToRobotPose;
            t.addLine();
            t.addLine(String.format(Locale.US,
                    "%s (x,y,h)= (%.1f, %.1f, %.1fdeg)",
                    estimatorMode == EstimatorMode.EKF ? "EKF" : "Fused",
                    p.xInches, p.yInches, Math.toDegrees(p.yawRad)));
        }

        if (globalEstimator instanceof OdometryTagEkfPoseEstimator) {
            OdometryTagEkfPoseEstimator ekf = (OdometryTagEkfPoseEstimator) globalEstimator;
            t.addLine();
            t.addData("EKF Std", "pos=%.2f in, head=%.2f deg",
                    ekf.getPositionStdIn(),
                    Math.toDegrees(ekf.getHeadingStdRad()));
            t.addData("EKF Innov", "pos=%.2f in, head=%.2f deg, maha=%.2f",
                    ekf.getLastInnovationPositionIn(),
                    Math.toDegrees(ekf.getLastInnovationHeadingRad()),
                    ekf.getLastInnovationMahalanobisSq());
            t.addData("EKF Meas σ", "pos=%.2f in, head=%.2f deg",
                    ekf.getLastMeasurementPositionStdIn(),
                    Math.toDegrees(ekf.getLastMeasurementHeadingStdRad()));
        }

        t.addLine();
        t.addLine("Layout summary:");
        FtcTagLayoutDebug.dumpSummary(layout, new FtcTelemetryDebugSink(t), "layout");

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

    private static boolean isLikelyDefaultPinpointOffsets(PinpointPoseEstimator.Config cfg) {
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
