package edu.ftcphoenix.fw.tools.tester.localization;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.Collections;
import java.util.Locale;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.FtcVision;
import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.apriltag.TagOnlyPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagFusionPoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagTarget;
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
 * drive toward a tag, let the fusion settle, then rotate/score (tag not visible) and confirm the
 * fused pose continues to update smoothly based on odometry.</p>
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>PICKER</b>: Dpad Up/Down highlight, A choose, B refresh</li>
 *   <li><b>RUN</b>:
 *     <ul>
 *       <li>START: toggle tracking mode (ANY tag in layout vs SINGLE chosen ID)</li>
 *       <li>Y/X: increment/decrement the chosen tag ID (used in SINGLE mode)</li>
 *       <li>A: snap fused pose to the current vision pose (when available)</li>
 *       <li>B: toggle vision fusion on/off</li>
 *       <li>RB: set fused pose to (0,0,0)</li>
 *       <li>BACK: return to camera picker</li>
 *     </ul>
 *   </li>
 * </ul>
 */
public final class PinpointAprilTagFusionLocalizationTester extends BaseTeleOpTester {

    private static final double DEFAULT_MAX_AGE_SEC = 0.35;
    private static final int DEFAULT_TAG_ID = 1;

    private final String preferredCameraName;
    private final CameraMountConfig cameraMount;
    private final PinpointPoseEstimator.Config pinpointCfg;
    private final OdometryTagFusionPoseEstimator.Config fusionCfg;

    private final TagLayout layoutOverride;
    private final AprilTagLibrary tagLibraryOverride;

    private HardwareNamePicker cameraPicker;
    private String selectedCameraName = null;

    private boolean ready = false;
    private String initError = null;

    private TagLayout layout;

    // Vision pieces
    private AprilTagSensor tagSensor;
    private TagTarget target;
    private TagOnlyPoseEstimator visionEstimator;

    // Odometry + fusion
    private PinpointPoseEstimator odomEstimator;
    private OdometryTagFusionPoseEstimator fusedEstimator;

    private boolean trackAny = true;
    private int selectedTagId = DEFAULT_TAG_ID;

    public PinpointAprilTagFusionLocalizationTester(String preferredCameraName,
                                                    CameraMountConfig cameraMount,
                                                    PinpointPoseEstimator.Config pinpointCfg) {
        this(preferredCameraName, cameraMount, pinpointCfg, OdometryTagFusionPoseEstimator.Config.defaults());
    }

    public PinpointAprilTagFusionLocalizationTester(String preferredCameraName,
                                                    CameraMountConfig cameraMount,
                                                    PinpointPoseEstimator.Config pinpointCfg,
                                                    OdometryTagFusionPoseEstimator.Config fusionCfg) {
        this(preferredCameraName, cameraMount, pinpointCfg, fusionCfg, null, null);
    }

    public PinpointAprilTagFusionLocalizationTester(String preferredCameraName,
                                                    CameraMountConfig cameraMount,
                                                    PinpointPoseEstimator.Config pinpointCfg,
                                                    OdometryTagFusionPoseEstimator.Config fusionCfg,
                                                    TagLayout layoutOverride,
                                                    AprilTagLibrary tagLibraryOverride) {
        this.preferredCameraName = preferredCameraName;
        this.cameraMount = cameraMount;
        this.pinpointCfg = pinpointCfg != null ? pinpointCfg : PinpointPoseEstimator.Config.defaults();
        this.fusionCfg = fusionCfg != null ? fusionCfg : OdometryTagFusionPoseEstimator.Config.defaults();
        this.layoutOverride = layoutOverride;
        this.tagLibraryOverride = tagLibraryOverride;
    }


    @Override
    public String name() {
        return "Loc: Pinpoint + AprilTags (Fused)";
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
            } else if (fusedEstimator != null) {
                fusedEstimator.setVisionEnabled(!fusedEstimator.isVisionEnabled());
            }
        });

        // START: toggle tag tracking mode.
        bindings.onRise(gamepads.p1().start(), () -> {
            if (!ready) return;
            trackAny = !trackAny;
            rebuildTargetAndEstimators();
        });

        bindings.onRise(gamepads.p1().y(), () -> {
            if (!ready) return;
            selectedTagId++;
            if (!trackAny) {
                rebuildTargetAndEstimators();
            }
        });

        bindings.onRise(gamepads.p1().x(), () -> {
            if (!ready) return;
            selectedTagId = Math.max(1, selectedTagId - 1);
            if (!trackAny) {
                rebuildTargetAndEstimators();
            }
        });

        // A: snap fused pose to current vision estimate.
        bindings.onRise(gamepads.p1().a(), () -> {
            if (!ready || fusedEstimator == null || visionEstimator == null) return;
            PoseEstimate v = visionEstimator.getEstimate();
            if (v != null && v.hasPose) {
                // PoseEstimate already provides a planar projection helper.
                fusedEstimator.setPose(v.toPose2d());
            }
        });

        // RB: reset fused pose to 0,0,0.
        bindings.onRise(gamepads.p1().rightBumper(), () -> {
            if (!ready || fusedEstimator == null) return;
            fusedEstimator.setPose(Pose2d.zero());
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
        target = null;
        visionEstimator = null;
        odomEstimator = null;
        fusedEstimator = null;
        return true;
    }

    @Override
    protected void onStop() {
        // Ensure we release the camera portal on exit.
        if (tagSensor != null) {
            tagSensor.close();
            tagSensor = null;
        }
    }

    private void renderPicker() {
        Telemetry t = ctx.telemetry;
        t.addLine("--- Pinpoint + AprilTag Fusion ---");
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
            // Vision
            FtcVision.Config cfg = FtcVision.Config.defaults()
                    .withCameraMount(cameraMount);
            if (tagLibraryOverride != null) {
                cfg.withTagLibrary(tagLibraryOverride);
            }
            tagSensor = FtcVision.aprilTags(ctx.hw, selectedCameraName, cfg);

            // Odometry
            odomEstimator = new PinpointPoseEstimator(ctx.hw, pinpointCfg);

            // Field layout (current game)
            layout = (layoutOverride != null)
                    ? layoutOverride
                    : new FtcGameTagLayout(AprilTagGameDatabase.getCurrentGameTagLibrary());

            rebuildTargetAndEstimators();

            ready = true;
            initError = null;
        } catch (Exception e) {
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
            ready = false;
        }
    }

    private void rebuildTargetAndEstimators() {
        if (tagSensor == null || layout == null) {
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

        target = new TagTarget(tagSensor, ids, DEFAULT_MAX_AGE_SEC);

        TagOnlyPoseEstimator.Config tagCfg = TagOnlyPoseEstimator.Config.defaults()
                .withCameraMount(cameraMount);
        tagCfg.maxAbsBearingRad = 0.0;
        visionEstimator = new TagOnlyPoseEstimator(target, layout, tagCfg);

        fusedEstimator = new OdometryTagFusionPoseEstimator(odomEstimator, visionEstimator, fusionCfg);
    }

    private void updateAndRender() {
        // Loop ordering requirement: update TagTarget before estimators that consume it.
        target.update(clock);

        // Fusion updates both odometry + vision internally.
        fusedEstimator.update(clock);

        PoseEstimate odom = odomEstimator.getEstimate();
        PoseEstimate vis = visionEstimator.getEstimate();
        PoseEstimate fused = fusedEstimator.getEstimate();

        Telemetry t = ctx.telemetry;
        t.addLine("--- Pinpoint + AprilTag Fusion ---");
        t.addLine("Controls: START any/single | X/Y tag id | A snap-to-vision | B vision on/off | RB zero | BACK picker");
        t.addLine();

        // Dependencies / preflight hints. These testers are often used during calibration
        // bring-up; surface common "why does this look wrong?" causes directly on screen.
        if (isLikelyIdentity(cameraMount)) {
            t.addLine("NOTE: CameraMountConfig looks uncalibrated (identity). Run 'Calib: Camera Mount'.");
        }
        if (isLikelyDefaultPinpointOffsets(pinpointCfg)) {
            t.addLine("NOTE: Pinpoint pod offsets look default (0/0). Run 'Calib: Pinpoint Pod Offsets'.");
        }
        if (isLikelyIdentity(cameraMount) || isLikelyDefaultPinpointOffsets(pinpointCfg)) {
            t.addLine();
        }

        t.addData("Camera", selectedCameraName);
        t.addData("Mode", trackAny ? "ANY" : ("SINGLE (id=" + selectedTagId + ")"));
        t.addData("Vision Enabled", fusedEstimator.isVisionEnabled());
        t.addData("Vision Accept", "%d ok / %d rej", fusedEstimator.getAcceptedVisionCount(), fusedEstimator.getRejectedVisionCount());

        AprilTagObservation obs = target.last();
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
        renderPose(t, "Fused", fused);

        // Compact line for drivers.
        if (fused != null && fused.hasPose && fused.fieldToRobotPose != null) {
            Pose3d p = fused.fieldToRobotPose;
            t.addLine();
            t.addLine(String.format(Locale.US,
                    "Fused (x,y,h)= (%.1f, %.1f, %.1fdeg)",
                    p.xInches, p.yInches, Math.toDegrees(p.yawRad)));
        }

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
