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
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.FtcVision;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.apriltag.TagOnlyPoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagTarget;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.tools.tester.ui.HardwareNamePicker;

/**
 * Verifies that AprilTag-based localization is working end-to-end.
 *
 * <p>This tester is meant to answer two practical questions quickly:</p>
 * <ul>
 *   <li><b>Do we have fresh AprilTag detections?</b></li>
 *   <li><b>Does our derived field pose ({@code fieldToRobotPose}) look reasonable?</b></li>
 * </ul>
 *
 * <p>Internally this tester uses {@link TagTarget} (best-tag selection) and
 * {@link TagOnlyPoseEstimator} (single-tag field pose solve) with the official
 * FTC game tag layout from {@link AprilTagGameDatabase#getCurrentGameTagLibrary()}.</p>
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
 * If constructed without a camera name (or if the preferred camera cannot be initialized), the
 * tester shows a camera picker listing configured webcams and lets you choose one.
 * </p>
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>PICKER (no camera chosen yet)</b>: Dpad Up/Down highlight, A choose, B refresh</li>
 *   <li><b>RUN (camera chosen)</b>:
 *     <ul>
 *       <li>START: toggle tracking mode (ANY tag in layout vs SINGLE chosen ID)</li>
 *       <li>Y/X: increment/decrement the chosen tag ID (used in SINGLE mode)</li>
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

    private final String preferredCameraName;
    private final CameraMountConfig cameraMount;
    private final double maxAgeSec;
    private final TagLayout layoutOverride;
    private final AprilTagLibrary tagLibraryOverride;

    private TagLayout layout;

    private HardwareNamePicker cameraPicker;
    private String selectedCameraName = null;

    private boolean visionReady = false;
    private String visionInitError = null;

    private AprilTagSensor tagSensor;
    private TagTarget target;
    private TagOnlyPoseEstimator poseEstimator;

    private boolean trackAny = true;
    private int selectedTagId = DEFAULT_TAG_ID;

    private final PoseSampleStats samples = new PoseSampleStats();

    /**
     * Create the tester with no preferred camera name and an identity camera mount.
     *
     * <p>A camera picker will be shown so you can choose a configured webcam.</p>
     */
    public AprilTagLocalizationTester() {
        this(null, null, null, null, DEFAULT_MAX_AGE_SEC);
    }

    /**
     * Create the tester with a preferred camera name and camera mount.
     *
     * <p>If {@code cameraName} is null/blank (or cannot be initialized), the tester falls back
     * to the camera picker.</p>
     *
     * @param cameraName  configured webcam name in the FTC Robot Configuration (nullable)
     * @param cameraMount robot→camera extrinsics (nullable; if null, identity is used)
     */
    public AprilTagLocalizationTester(String cameraName, CameraMountConfig cameraMount) {
        this(cameraName, cameraMount, null, null, DEFAULT_MAX_AGE_SEC);
    }

    /**
     * Create the tester with full configuration control.
     *
     * @param cameraName  configured webcam name in the FTC Robot Configuration (nullable)
     * @param cameraMount robot→camera extrinsics (nullable; if null, identity is used)
     * @param maxAgeSec   maximum allowed tag frame age in seconds (must be non-negative)
     */
    public AprilTagLocalizationTester(String cameraName,
                                      CameraMountConfig cameraMount,
                                      double maxAgeSec) {
        this(cameraName, cameraMount, null, null, maxAgeSec);
    }

    /**
     * Create the tester with full configuration control, including optional tag layout/library overrides.
     *
     * @param cameraName         configured webcam name in the FTC Robot Configuration (nullable)
     * @param cameraMount        robot→camera extrinsics (nullable; if null, identity is used)
     * @param layoutOverride     optional tag layout override (nullable; if null, current game layout is used)
     * @param tagLibraryOverride optional tag library override (nullable; if null, current game library is used)
     * @param maxAgeSec          maximum allowed tag frame age in seconds (must be non-negative)
     */
    public AprilTagLocalizationTester(String cameraName,
                                      CameraMountConfig cameraMount,
                                      TagLayout layoutOverride,
                                      AprilTagLibrary tagLibraryOverride,
                                      double maxAgeSec) {
        this.preferredCameraName = cameraName;
        this.cameraMount = (cameraMount != null) ? cameraMount : CameraMountConfig.identity();
        this.layoutOverride = layoutOverride;
        this.tagLibraryOverride = tagLibraryOverride;
        if (maxAgeSec < 0.0) {
            throw new IllegalArgumentException("maxAgeSec must be non-negative");
        }
        this.maxAgeSec = maxAgeSec;
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
        // Field layout: default to the FTC SDK's official current-game tag library, unless overridden.
        if (layoutOverride != null) {
            layout = layoutOverride;
        } else {
            layout = new FtcGameTagLayout(AprilTagGameDatabase.getCurrentGameTagLibrary());
        }

        // Camera selection
        cameraPicker = new HardwareNamePicker(
                ctx.hw,
                WebcamName.class,
                "Select Camera",
                "Dpad: highlight | A: choose | B: refresh"
        );
        cameraPicker.refresh();

        // Prefer camera name passed in (RobotConfig), but fall back to picker if init fails.
        if (preferredCameraName != null && !preferredCameraName.trim().isEmpty()) {
            selectedCameraName = preferredCameraName.trim();
            cameraPicker.setPreferredName(selectedCameraName);
            ensureVisionReady();
        }

        // Picker controls active only while NOT ready.
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

        // B: refresh camera list (picker) OR clear samples (run).
        bindings.onPress(gamepads.p1().b(), () -> {
            if (!visionReady) {
                cameraPicker.refresh();
            } else {
                samples.clear();
            }
        });

        // START: toggle tracking mode (ANY vs SINGLE)
        bindings.onPress(gamepads.p1().start(), () -> {
            if (!visionReady) return;
            trackAny = !trackAny;
            rebuildTargetAndEstimator();
        });

        // Tag ID selection (used in SINGLE mode)
        bindings.onPress(gamepads.p1().y(), () -> {
            if (!visionReady) return;
            selectedTagId++;
            if (!trackAny) {
                rebuildTargetAndEstimator();
            }
        });

        bindings.onPress(gamepads.p1().x(), () -> {
            if (!visionReady) return;
            selectedTagId = Math.max(1, selectedTagId - 1);
            if (!trackAny) {
                rebuildTargetAndEstimator();
            }
        });

        // A: capture sample (only when we have a pose)
        bindings.onPress(gamepads.p1().a(), () -> {
            if (!visionReady) return;
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
        // If we are already in the picker, let the suite handle BACK (exit to suite menu).
        if (!visionReady) {
            return false;
        }

        // Return to picker state.
        visionReady = false;
        visionInitError = null;

        // IMPORTANT: Release camera resources before re-entering the picker.
        if (tagSensor != null) {
            tagSensor.close();
        }
        tagSensor = null;
        target = null;
        poseEstimator = null;

        samples.clear();

        // Reset picker UI and highlight the last camera for convenience.
        cameraPicker.clearChoice();
        cameraPicker.refresh();
        if (selectedCameraName != null && !selectedCameraName.isEmpty()) {
            cameraPicker.setPreferredName(selectedCameraName);
        }

        return true;
    }

    @Override
    protected void onStop() {
        // Ensure we release the VisionPortal when leaving the tester.
        if (tagSensor != null) {
            tagSensor.close();
            tagSensor = null;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Vision init + localization wiring
    // ---------------------------------------------------------------------------------------------

    private void ensureVisionReady() {
        if (visionReady) return;
        if (selectedCameraName == null || selectedCameraName.isEmpty()) return;

        visionInitError = null;

        try {
            FtcVision.Config cfg = FtcVision.Config.defaults()
                    .withCameraMount(cameraMount);
            if (tagLibraryOverride != null) {
                cfg.withTagLibrary(tagLibraryOverride);
            }
            tagSensor = FtcVision.aprilTags(ctx.hw, selectedCameraName, cfg);
            visionReady = true;

            rebuildTargetAndEstimator();
        } catch (Exception ex) {
            visionReady = false;
            tagSensor = null;
            visionInitError = ex.getClass().getSimpleName() + ": " + ex.getMessage();
        }
    }

    private void rebuildTargetAndEstimator() {
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

        target = new TagTarget(tagSensor, ids, maxAgeSec);

        TagOnlyPoseEstimator.Config cfg = TagOnlyPoseEstimator.Config.defaults()
                .withCameraMount(cameraMount);
        cfg.maxAbsBearingRad = 0.0;

        poseEstimator = new TagOnlyPoseEstimator(target, layout, cfg);

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
        t.addLine("Press A to choose a camera and initialize vision.");
        t.addLine("Press B to refresh camera list.");

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
        if (target == null || poseEstimator == null) {
            rebuildTargetAndEstimator();
        }
        if (target == null || poseEstimator == null) {
            renderInternalError("Localization pipeline not initialized (target/estimator is null)");
            return;
        }

        // Update tracked tag + pose solve.
        target.update(clock);
        poseEstimator.update(clock);

        AprilTagObservation obs = target.last();
        PoseEstimate est = poseEstimator.getEstimate();

        renderTelemetry(obs, est);
    }

    private void renderTelemetry(AprilTagObservation obs, PoseEstimate est) {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        t.addLine("=== AprilTag Localization ===");
        t.addLine(String.format(Locale.US,
                "Camera=%s | MaxAge=%.0f ms | Track=%s",
                selectedCameraName,
                maxAgeSec * 1000.0,
                trackAny ? "ANY" : ("SINGLE id=" + selectedTagId)
        ));

        t.addLine("Controls: START trackMode | Y/X tagId | A sample | B clear | BACK camera picker");

        if (isLikelyIdentity(cameraMount)) {
            t.addLine("");
            t.addLine("NOTE: CameraMountConfig looks like identity (0,0,0,0,0,0).");
            t.addLine("      Run 'Calib: Camera Mount' and update your RobotConfig for better accuracy.");
        }

        t.addLine("");
        t.addLine("Observation:");

        if (!obs.hasTarget) {
            t.addLine("  No fresh tag detection.");
            t.addLine("  Tips: check lighting, focus, camera stream, and tag visibility.");
        } else {
            t.addLine(String.format(Locale.US,
                    "  id=%d | age=%.0f ms | range=%.1f in | bearing=%.1f°",
                    obs.id,
                    obs.ageSec * 1000.0,
                    obs.cameraRangeInches(),
                    Math.toDegrees(obs.cameraBearingRad())
            ));
            t.addLine(String.format(Locale.US,
                    "  cameraToTag: forward=%.1f in | left=%.1f in | up=%.1f in",
                    obs.cameraForwardInches(),
                    obs.cameraLeftInches(),
                    obs.cameraUpInches()
            ));

            boolean inLayout = (layout != null && layout.has(obs.id));
            t.addLine("  inLayout=" + inLayout);

            if (inLayout) {
                TagLayout.TagPose tag = layout.require(obs.id);
                Pose3d ft = tag.fieldToTagPose();
                t.addLine(String.format(Locale.US,
                        "  fieldToTag: x=%.1f y=%.1f z=%.1f | yaw=%.1f°",
                        ft.xInches, ft.yInches, ft.zInches,
                        Math.toDegrees(ft.yawRad)
                ));
            }
        }

        t.addLine("");
        t.addLine("Pose estimate (TagOnlyPoseEstimator):");
        if (!est.hasPose || est.fieldToRobotPose == null) {
            t.addLine("  (no pose) Need: fresh detection for a tag present in the field layout.");
        } else {
            Pose3d p = est.fieldToRobotPose;
            t.addLine(String.format(Locale.US,
                    "  fieldToRobot: x=%.1f y=%.1f | yaw=%.1f°",
                    p.xInches, p.yInches, Math.toDegrees(p.yawRad)
            ));
            t.addLine(String.format(Locale.US,
                    "  timestamp=%.3f s | age=%.0f ms",
                    est.timestampSec,
                    est.ageSec * 1000.0
            ));
        }

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

            t.addLine(String.format(Locale.US,
                    "  fieldToRobot(sdk): x=%.1f y=%.1f | yaw=%.1f°",
                    sdkPlanar.xInches, sdkPlanar.yInches, Math.toDegrees(sdkPlanar.yawRad)
            ));

            if (est.hasPose && est.fieldToRobotPose != null) {
                Pose3d p = est.fieldToRobotPose;

                double dx = p.xInches - sdkPlanar.xInches;
                double dy = p.yInches - sdkPlanar.yInches;
                double dxy = Math.hypot(dx, dy);
                double dYawRad = MathUtil.wrapToPi(p.yawRad - sdkPlanar.yawRad);

                t.addLine(String.format(Locale.US,
                        "  delta (est - sdk): dXY=%.2f in | dYaw=%.2f°",
                        dxy,
                        Math.toDegrees(dYawRad)
                ));
            }
        }

        t.addLine("");
        t.addLine("Sampling (press A to capture):");
        if (samples.count() <= 0) {
            t.addLine("  Samples: 0  (Press A while the robot is still)");
        } else {
            t.addLine(String.format(Locale.US,
                    "  Samples: %d",
                    samples.count()
            ));

            t.addLine(String.format(Locale.US,
                    "  mean: x=%.1f y=%.1f | yaw=%.1f°",
                    samples.meanX(),
                    samples.meanY(),
                    Math.toDegrees(samples.meanYawRad())
            ));

            t.addLine(String.format(Locale.US,
                    "  std : x=%.2f y=%.2f | yaw=%.2f°",
                    samples.stdX(),
                    samples.stdY(),
                    Math.toDegrees(samples.circularStdYawRad())
            ));
        }

        t.update();
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
