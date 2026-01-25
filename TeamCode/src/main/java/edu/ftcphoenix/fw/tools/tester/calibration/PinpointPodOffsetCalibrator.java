package edu.ftcphoenix.fw.tools.tester.calibration;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.Locale;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.FtcVision;
import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.apriltag.TagOnlyPoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagTarget;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.tools.tester.ui.HardwareNamePicker;

/**
 * Calibrates goBILDA Pinpoint odometry pod offsets by observing translation drift while rotating in place.
 *
 * <p>Why this works: if either odometry pod is not located at the robot's true center of rotation,
 * rotating the robot causes a measurable translation ("drift") in the Pinpoint-reported pose. From the
 * measured drift and the known heading change, we can solve for the pod offsets.</p>
 *
 * <h2>Controls</h2>
 * <ul>
 *   <li><b>X</b>: Reset pose + clear results</li>
 *   <li><b>A</b>: Start/stop a manual sample (you rotate the robot)</li>
 *   <li><b>Y</b>: Auto sample (rotate to target angle) <i>requires mecanum wiring configured</i></li>
 *   <li><b>B</b>: Abort sample</li>
 *   <li><b>Right stick X</b>: Manual rotate (when drive is configured)</li>
 * </ul>
 *
 * <p><b>Important:</b> robot movement only happens after you press <b>PLAY</b> on the Driver Station.
 * In <b>INIT</b> (before PLAY), this tester can still show status and (optionally) let you select a
 * webcam for AprilTag assist, but it will not command the motors.</p>
 *
 * <p>Optional: enable AprilTag assist to subtract real translation while sampling. This is useful if the
 * robot doesn't rotate perfectly in place (carpet slip, uneven pod preload, etc.).</p>
 */
public final class PinpointPodOffsetCalibrator extends BaseTeleOpTester {

    /**
     * Minimum solve denominator (a^2 + b^2) required to produce a stable result.
     *
     * <p>For this calibrator, the solve becomes ill-conditioned when the net rotation is close to
     * 0° or 360° (or any multiple of 360°). In those cases, the true drift approaches 0 and noise
     * will "blow up" into absurd offset values.</p>
     *
     * <p>Denominator is: sin^2(theta) + (1 - cos(theta))^2 = 4 * sin^2(theta/2).
     * A value of 0.5 roughly corresponds to ~45° away from the degenerate cases.</p>
     */
    private static final double MIN_SOLVE_DENOM = 0.5;

    /**
     * Configuration for the tester.
     */
    public static final class Config {

        /**
         * Pinpoint estimator config (includes current pod offsets).
         */
        public PinpointPoseEstimator.Config pinpoint = PinpointPoseEstimator.Config.defaults();

        /**
         * Optional mecanum wiring. If null, the tester won't drive the robot.
         */
        public FtcDrives.MecanumWiringConfig mecanumWiring = null;

        /**
         * Optional mecanum tuning config (only used when {@link #mecanumWiring} is non-null).
         */
        public MecanumDrivebase.Config driveConfig = MecanumDrivebase.Config.defaults();

        /**
         * Manual rotation scale when using the right stick X.
         */
        public double manualOmegaScale = 0.6;

        /**
         * Auto rotation omega command (+ is CCW).
         */
        public double autoOmegaCmd = 0.35;

        /**
         * Target rotation (radians) for auto samples. Defaults to 180 degrees.
         */
        public double targetTurnRad = Math.PI;

        /**
         * If true, auto samples (Y) will compute results automatically once the rotation is done
         * <b>when AprilTag assist is active</b>.
         *
         * <p>This is the recommended way to run the calibrator: the tester uses the Pinpoint IMU
         * heading to stop at ~180°, and uses AprilTags to subtract any real translation (carpet slip,
         * imperfect pivot, etc.). When a tag pose is available at both the start and end of the
         * sample, the test becomes fully automatic: press <b>Y</b>, and it will stop and compute on
         * its own.</p>
         *
         * <p>If tags are not available (no known-pose tag seen), the tester will fall back to the
         * recenter-and-press-A flow to avoid producing misleading numbers.</p>
         */
        public boolean autoComputeAfterAutoSample = true;

        // -------------------------------------------------------------------------------------
        // AprilTag assist enhancements
        // -------------------------------------------------------------------------------------

        /**
         * If AprilTag assist is enabled and no tag is currently visible, the tester can
         * auto-rotate to search for any known-pose tag (as defined by {@link #tagLayout}).
         */
        public boolean enableAutoTagSearchAtStart = true;

        /**
         * Maximum amount of rotation (radians) to spend searching for a tag.
         *
         * <p>Defaults to ~2 full turns. If a tag isn't found within this rotation, the
         * sample will proceed without tag assist.</p>
         */
        public double tagSearchMaxTurnRad = 4.0 * Math.PI;

        /**
         * Rotation omega command used while searching for a tag.
         */
        public double tagSearchOmegaCmd = 0.25;

        /**
         * Number of consecutive loops with a valid tag pose before we consider the tag "found".
         */
        public int tagSearchStableFrames = 3;

        /**
         * If true, after an auto-rotation reaches {@link #targetTurnRad}, the tester will
         * optionally continue rotating a bit longer to reacquire a tag pose (for assist/recenter).
         */
        public boolean enableAutoTagSearchAtEnd = true;

        /**
         * Maximum extra rotation (radians) allowed while searching for a tag at the end of auto-sample.
         */
        public double tagEndSearchMaxExtraTurnRad = 2.0 * Math.PI;

        /**
         * If true, after the rotation portion of a sample finishes, the tester pauses and allows
         * manual translation to "recenter" back to the starting position before computing results.
         *
         * <p>This is especially useful for auto-turning on imperfect flooring where the robot drifts
         * laterally while rotating. Re-centering reduces bias from real translation during the turn.</p>
         */
        public boolean enablePostRotateRecenter = true;

        /**
         * Manual translation scale in recenter mode.
         */
        public double recenterTranslationScale = 0.6;

        /**
         * If true, use AprilTags to estimate and subtract real translation during the sample.
         */
        public boolean enableAprilTagAssist = false;

        /**
         * Preferred webcam name for AprilTag assist. If null, a camera picker is shown.
         */
        public String preferredCameraName = null;

        /**
         * Required when {@link #enableAprilTagAssist} is true.
         */
        public CameraMountConfig cameraMount = null;

        /**
         * Tag layout. If null and AprilTag assist is enabled, defaults to current game layout.
         */
        public TagLayout tagLayout = null;

        /**
         * Tag library override (sizes/IDs). If null, the current game tag library is used.
         */
        public AprilTagLibrary tagLibrary = null;

        /**
         * Max age (seconds) for AprilTag detections used by the estimator.
         */
        public double maxTagAgeSec = 0.2;

        private Config() {
            // Defaults assigned in field initializers.
        }

        /**
         * Create a new config instance with Phoenix defaults.
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Deep copy of this config (note: references are copied as-is).
         */
        public Config copy() {
            Config c = new Config();
            c.pinpoint = this.pinpoint;
            c.mecanumWiring = this.mecanumWiring;
            c.driveConfig = this.driveConfig;

            c.manualOmegaScale = this.manualOmegaScale;
            c.autoOmegaCmd = this.autoOmegaCmd;
            c.targetTurnRad = this.targetTurnRad;

            c.enableAutoTagSearchAtStart = this.enableAutoTagSearchAtStart;
            c.tagSearchMaxTurnRad = this.tagSearchMaxTurnRad;
            c.tagSearchOmegaCmd = this.tagSearchOmegaCmd;
            c.tagSearchStableFrames = this.tagSearchStableFrames;
            c.enableAutoTagSearchAtEnd = this.enableAutoTagSearchAtEnd;
            c.tagEndSearchMaxExtraTurnRad = this.tagEndSearchMaxExtraTurnRad;
            c.enablePostRotateRecenter = this.enablePostRotateRecenter;
            c.recenterTranslationScale = this.recenterTranslationScale;

            c.enableAprilTagAssist = this.enableAprilTagAssist;
            c.preferredCameraName = this.preferredCameraName;
            c.cameraMount = this.cameraMount;
            c.tagLayout = this.tagLayout;
            c.tagLibrary = this.tagLibrary;
            c.maxTagAgeSec = this.maxTagAgeSec;
            return c;
        }
    }

    private final Config cfg;

    private PinpointPoseEstimator pinpoint;
    private MecanumDrivebase drive;

    // AprilTag assist
    private HardwareNamePicker cameraPicker;
    private String selectedCameraName;
    private TagLayout layout;
    private AprilTagSensor tagSensor;
    private TagTarget tagTarget;
    private TagOnlyPoseEstimator tagEstimator;
    private String aprilTagAssistNotice;

    /**
     * High-level state machine for the calibration flow.
     *
     * <p>The tester is intentionally explicit about phases so telemetry and
     * control logic stay easy to reason about while students run the procedure.</p>
     */
    private enum Phase {
        /**
         * Not sampling; just showing live telemetry.
         */
        IDLE,
        /**
         * Auto-rotating to find any known-pose tag before starting a sample.
         */
        SEARCH_TAG_START,
        /**
         * Main rotation portion of the sample.
         */
        ROTATING,
        /**
         * Optional: after target turn, keep rotating briefly to reacquire a tag pose.
         */
        SEARCH_TAG_END,
        /**
         * Optional: allow manual translation to recenter before computing results.
         */
        POST_RECENTER,
    }

    private Phase phase = Phase.IDLE;

    /**
     * True if the current sample was started in auto-rotate mode (Y).
     */
    private boolean autoSample = false;

    /**
     * Tracks unwrapped heading during tag-search phases (start/end).
     */
    private final AngleUnwrapper tagSearchUnwrapper = new AngleUnwrapper();
    private double tagSearchStartUnwrappedRad = 0.0;
    private int tagStableFrames = 0;

    private Pose2d startPinpointPose = Pose2d.zero();
    private Pose2d startTagPose = null;

    // If AprilTag assist is enabled, we cache the most recent valid tag pose during the sample.
    // This makes the assist robust to brief dropouts (motion blur, occlusion, etc.).
    private Pose2d endTagPose = null;

    private double startHeadingRad = 0.0;
    private double startHeadingUnwrappedRad = 0.0;
    private final AngleUnwrapper headingUnwrapper = new AngleUnwrapper();

    // latest values
    private Pose2d latestPinpointPose = Pose2d.zero();
    private Pose2d latestTagPose = null;

    // last sample results (null means not available)
    private Double lastDxFieldInches = null;
    private Double lastDyFieldInches = null;
    private Double lastDeltaHeadingRad = null;

    private Double lastXErrorInches = null;
    private Double lastYErrorInches = null;

    private Double lastRecommendedStrafePodOffsetForwardInches = null;
    private Double lastRecommendedForwardPodOffsetLeftInches = null;

    // If the last sample couldn't be solved (too little rotation / degenerate rotation), this
    // is a short, driver-facing note explaining why.
    private String lastSolveNote = null;

    private boolean lastHadTagStart = false;
    private boolean lastHadTagEnd = false;

    public PinpointPodOffsetCalibrator() {
        this(Config.defaults());
    }

    public PinpointPodOffsetCalibrator(Config cfg) {
        this.cfg = (cfg != null) ? cfg : Config.defaults();
    }

    @Override
    public String name() {
        return "Pinpoint: Pod Offset Calibrator";
    }

    @Override
    protected void onInit() {
        // Pinpoint is required
        pinpoint = new PinpointPoseEstimator(ctx.hw, cfg.pinpoint);

        // Optional drive
        if (cfg.mecanumWiring != null) {
            drive = FtcDrives.mecanum(ctx.hw, cfg.mecanumWiring, cfg.driveConfig);
        }

        // Optional AprilTag assist
        if (cfg.enableAprilTagAssist) {
            // AprilTag assistance relies on a calibrated robot->camera mount. If it isn't available,
            // degrade gracefully: run without assist and tell the driver what to calibrate first.
            if (cfg.cameraMount == null) {
                cfg.enableAprilTagAssist = false;
                aprilTagAssistNotice = "Disabled: cfg.cameraMount not set";
            } else if (isLikelyIdentity(cfg.cameraMount)) {
                cfg.enableAprilTagAssist = false;
                aprilTagAssistNotice = "Disabled: camera mount looks uncalibrated";
            }
        }

        if (cfg.enableAprilTagAssist) {

            layout = (cfg.tagLayout != null)
                    ? cfg.tagLayout
                    : new FtcGameTagLayout(AprilTagGameDatabase.getCurrentGameTagLibrary());

            selectedCameraName = cfg.preferredCameraName;
            if (selectedCameraName == null) {
                cameraPicker = new HardwareNamePicker(ctx.hw, WebcamName.class, "Select a webcam");
                cameraPicker.bind(
                        bindings,
                        gamepads.p1().dpadUp(),
                        gamepads.p1().dpadDown(),
                        gamepads.p1().a(),
                        gamepads.p1().y(),
                        () -> tagSensor == null,
                        name -> selectedCameraName = name
                );
            }
        }

        // If AprilTag assist is disabled because the camera mount is still identity,
        // surface that clearly in telemetry so users know which calibration to run next.
        if (!cfg.enableAprilTagAssist && aprilTagAssistNotice == null
                && cfg.cameraMount != null && isLikelyIdentity(cfg.cameraMount)) {
            aprilTagAssistNotice = "Tip: run Calib: Camera Mount to enable AprilTag assist";
        }

        // Controls
        bindings.onPress(gamepads.p1().x(), this::resetAndClear);
        bindings.onPress(gamepads.p1().a(), this::onAPress);
        bindings.onPress(gamepads.p1().y(), this::onYPress);
        bindings.onPress(gamepads.p1().b(), this::abortSample);

        // Start in a clean state
        resetAndClear();
    }

    @Override
    protected void onInitLoop(double dtSec) {
        ensureAprilTagAssistReady();

        // Keep estimators warm in init so the first sample isn't stale.
        updateSensors();

        renderTelemetry(true);
    }

    @Override
    protected void onLoop(double dtSec) {
        ensureAprilTagAssistReady();

        updateSensors();

        // Update heading unwrapper for phases where a sample is active.
        if (isSampleActive()) {
            headingUnwrapper.update(latestPinpointPose.headingRad);
        }

        switch (phase) {
            case SEARCH_TAG_START:
                updateSearchForTagStart();
                break;
            case ROTATING:
                updateRotating();
                break;
            case SEARCH_TAG_END:
                updateSearchForTagEnd();
                break;
            case POST_RECENTER:
                updatePostRecenter();
                break;
            case IDLE:
            default:
                // Keep the drivetrain stopped while idle.
                if (drive != null) {
                    drive.update(ctx.clock);
                    drive.drive(DriveSignal.zero());
                }
                break;
        }

        renderTelemetry(false);
    }

    private boolean isSampleActive() {
        return phase == Phase.ROTATING || phase == Phase.SEARCH_TAG_END || phase == Phase.POST_RECENTER;
    }

    private void updateSearchForTagStart() {
        if (drive == null || tagEstimator == null) {
            // Shouldn't happen, but fail safe.
            startSampleInternal(autoSample, null);
            return;
        }

        // Continue rotating while we search for any known-pose tag.
        drive.update(ctx.clock);
        drive.drive(new DriveSignal(0.0, 0.0, cfg.tagSearchOmegaCmd));

        // Track how far we've rotated while searching.
        tagSearchUnwrapper.update(latestPinpointPose.headingRad);
        double turned = Math.abs(tagSearchUnwrapper.getUnwrappedRad() - tagSearchStartUnwrappedRad);

        PoseEstimate tagEst = tagEstimator.getEstimate();
        if (tagEst.hasPose) {
            tagStableFrames++;
        } else {
            tagStableFrames = 0;
        }

        if (tagEst.hasPose && tagStableFrames >= cfg.tagSearchStableFrames) {
            // Found a stable tag pose; align and start the actual sample.
            drive.drive(DriveSignal.zero());
            startSampleInternal(autoSample, tagEst.toPose2d());
            return;
        }

        if (turned >= Math.abs(cfg.tagSearchMaxTurnRad)) {
            // Give up and just start without tag assist.
            drive.drive(DriveSignal.zero());
            startSampleInternal(autoSample, null);
        }
    }

    private void updateRotating() {
        // Auto-stop condition for auto samples.
        if (autoSample && drive != null) {
            double delta = headingUnwrapper.getUnwrappedRad() - startHeadingUnwrappedRad;
            if (Math.abs(delta) >= Math.abs(cfg.targetTurnRad)) {
                transitionAfterRotation();
                return;
            }
        }

        if (drive == null) {
            // No drivetrain configured; the driver rotates the robot by hand.
            return;
        }

        double omega;
        if (autoSample) {
            omega = Math.signum(cfg.targetTurnRad) * cfg.autoOmegaCmd;
        } else {
            // Driver-friendly: stick right should turn right (omega negative).
            omega = -gamepads.p1().rightX().get() * cfg.manualOmegaScale;
        }

        drive.update(ctx.clock);
        drive.drive(new DriveSignal(0.0, 0.0, omega));
    }

    private void updateSearchForTagEnd() {
        if (drive == null || tagEstimator == null) {
            transitionToPostRecenterOrFinish();
            return;
        }

        // Rotate a little further to reacquire a tag pose, but cap how far we go.
        tagSearchUnwrapper.update(latestPinpointPose.headingRad);
        double turnedExtra = Math.abs(tagSearchUnwrapper.getUnwrappedRad() - tagSearchStartUnwrappedRad);

        PoseEstimate tagEst = tagEstimator.getEstimate();
        if (tagEst.hasPose) {
            tagStableFrames++;
        } else {
            tagStableFrames = 0;
        }

        if (tagEst.hasPose && tagStableFrames >= cfg.tagSearchStableFrames) {
            transitionToPostRecenterOrFinish();
            return;
        }

        if (turnedExtra >= Math.abs(cfg.tagEndSearchMaxExtraTurnRad)) {
            transitionToPostRecenterOrFinish();
            return;
        }

        double omega = Math.signum(cfg.targetTurnRad) * cfg.tagSearchOmegaCmd;
        drive.update(ctx.clock);
        drive.drive(new DriveSignal(0.0, 0.0, omega));
    }

    private void updatePostRecenter() {
        if (drive == null) {
            return;
        }

        // Allow the driver to translate (no rotation) to bring the robot back to the starting spot.
        double axial = gamepads.p1().leftY().get() * cfg.recenterTranslationScale;
        double lateral = -gamepads.p1().leftX().get() * cfg.recenterTranslationScale;

        drive.update(ctx.clock);
        drive.drive(new DriveSignal(axial, lateral, 0.0));
    }

    private void resetAndClear() {
        phase = Phase.IDLE;
        autoSample = false;
        tagStableFrames = 0;

        lastDxFieldInches = null;
        lastDyFieldInches = null;
        lastDeltaHeadingRad = null;
        lastXErrorInches = null;
        lastYErrorInches = null;
        lastRecommendedStrafePodOffsetForwardInches = null;
        lastRecommendedForwardPodOffsetLeftInches = null;
        lastHadTagStart = false;
        lastHadTagEnd = false;
        lastSolveNote = null;

        startTagPose = null;
        latestTagPose = null;

        endTagPose = null;

        // Reset pose to something deterministic
        pinpoint.setPose(Pose2d.zero());
        headingUnwrapper.reset(0.0);
        tagSearchUnwrapper.reset(0.0);

        if (drive != null) {
            drive.update(ctx.clock);
            drive.drive(DriveSignal.zero());
        }
    }

    private void onAPress() {
        switch (phase) {
            case IDLE:
                requestStartSample(false);
                break;
            case SEARCH_TAG_START:
                // Skip search and proceed without tag assist.
                startSampleInternal(autoSample, null);
                break;
            case ROTATING:
                // End rotation portion; optionally recenter before computing.
                transitionAfterRotation();
                break;
            case SEARCH_TAG_END:
                // Skip end tag search.
                transitionToPostRecenterOrFinish();
                break;
            case POST_RECENTER:
                finishSampleAndCompute();
                break;
        }
    }

    private void onYPress() {
        if (drive == null) {
            // No drive available; telemetry will explain.
            return;
        }
        if (phase != Phase.IDLE) {
            return;
        }
        requestStartSample(true);
    }

    private void abortSample() {
        phase = Phase.IDLE;
        autoSample = false;
        tagStableFrames = 0;
        if (drive != null) {
            drive.update(ctx.clock);
            drive.drive(DriveSignal.zero());
        }
    }

    private void clearLastResults() {
        lastDxFieldInches = null;
        lastDyFieldInches = null;
        lastDeltaHeadingRad = null;
        lastXErrorInches = null;
        lastYErrorInches = null;
        lastRecommendedStrafePodOffsetForwardInches = null;
        lastRecommendedForwardPodOffsetLeftInches = null;
        lastHadTagStart = false;
        lastHadTagEnd = false;
        lastSolveNote = null;
    }

    private void requestStartSample(boolean auto) {
        if (phase != Phase.IDLE) return;

        autoSample = auto;
        clearLastResults();

        // Prefer to align Pinpoint to a vision pose if we have one.
        if (cfg.enableAprilTagAssist && tagEstimator != null) {
            PoseEstimate tagEst = tagEstimator.getEstimate();
            if (tagEst.hasPose) {
                startSampleInternal(auto, tagEst.toPose2d());
                return;
            }
        }

        // If no tag right now, optionally auto-search for one.
        if (cfg.enableAprilTagAssist
                && cfg.enableAutoTagSearchAtStart
                && drive != null
                && tagEstimator != null) {
            phase = Phase.SEARCH_TAG_START;
            tagStableFrames = 0;
            tagSearchUnwrapper.reset(latestPinpointPose.headingRad);
            tagSearchStartUnwrappedRad = tagSearchUnwrapper.getUnwrappedRad();
            return;
        }

        // Otherwise, just start without tag assist.
        startSampleInternal(auto, null);
    }

    private void startSampleInternal(boolean auto, Pose2d startTagPoseOrNull) {
        clearLastResults();

        startTagPose = startTagPoseOrNull;
        lastHadTagStart = (startTagPose != null);
        lastHadTagEnd = false;

        endTagPose = null;

        // Align Pinpoint's field frame so deltas are comparable.
        if (startTagPose != null) {
            pinpoint.setPose(startTagPose);
        } else {
            pinpoint.setPose(Pose2d.zero());
        }

        // Snapshot starting pose
        startPinpointPose = pinpoint.getEstimate().toPose2d();
        startHeadingRad = startPinpointPose.headingRad;

        headingUnwrapper.reset(startHeadingRad);
        startHeadingUnwrappedRad = headingUnwrapper.getUnwrappedRad();

        autoSample = auto;
        phase = Phase.ROTATING;

        if (drive != null) {
            drive.update(ctx.clock);
            drive.drive(DriveSignal.zero());
        }
    }

    private void transitionAfterRotation() {
        // If this was an auto sample and we want to reacquire a tag at the end, do that first.
        if (autoSample
                && cfg.enableAprilTagAssist
                && cfg.enableAutoTagSearchAtEnd
                && drive != null
                && tagEstimator != null
                && startTagPose != null) {
            PoseEstimate tagEst = tagEstimator.getEstimate();
            if (tagEst.hasPose) {
                endTagPose = tagEst.toPose2d();
            }
            if (!tagEst.hasPose) {
                phase = Phase.SEARCH_TAG_END;
                tagStableFrames = 0;
                tagSearchUnwrapper.reset(latestPinpointPose.headingRad);
                tagSearchStartUnwrappedRad = tagSearchUnwrapper.getUnwrappedRad();
                return;
            }
        }

        transitionToPostRecenterOrFinish();
    }

    private void transitionToPostRecenterOrFinish() {
        // Fully automatic path (recommended): if this was an auto sample and AprilTag assist
        // produced a start+end tag pose, compute immediately (no extra button presses).
        if (autoSample
                && cfg.autoComputeAfterAutoSample
                && cfg.enableAprilTagAssist
                && startTagPose != null
                && endTagPose != null) {
            if (drive != null) {
                drive.update(ctx.clock);
                drive.drive(DriveSignal.zero());
            }
            finishSampleAndCompute();
            return;
        }

        if (cfg.enablePostRotateRecenter) {
            phase = Phase.POST_RECENTER;
        } else {
            finishSampleAndCompute();
        }

        if (drive != null) {
            drive.update(ctx.clock);
            drive.drive(DriveSignal.zero());
        }
    }

    private void finishSampleAndCompute() {
        if (!isSampleActive()) return;

        // Stop motors first.
        if (drive != null) {
            drive.update(ctx.clock);
            drive.drive(DriveSignal.zero());
        }

        // Compute deltas
        Pose2d endPinpointPose = latestPinpointPose;
        double dxField = endPinpointPose.xInches - startPinpointPose.xInches;
        double dyField = endPinpointPose.yInches - startPinpointPose.yInches;

        double deltaHeading = headingUnwrapper.getUnwrappedRad() - startHeadingUnwrappedRad;

        // AprilTag assist: subtract real translation (tag-measured) so we isolate odometry drift
        // caused by pod-offset misconfiguration, not carpet slip or an imperfect pivot.
        if (cfg.enableAprilTagAssist && startTagPose != null) {
            Pose2d end = (endTagPose != null) ? endTagPose : latestTagPose;
            if (end != null) {
                double dxTrue = end.xInches - startTagPose.xInches;
                double dyTrue = end.yInches - startTagPose.yInches;

                dxField -= dxTrue;
                dyField -= dyTrue;

                lastHadTagEnd = true;
            }
        }

        lastDxFieldInches = dxField;
        lastDyFieldInches = dyField;
        lastDeltaHeadingRad = deltaHeading;

        // Rotate drift into a frame where +X is robot-forward at the start of the sample.
        // (This makes the math independent of the absolute field heading.)
        double c = Math.cos(startHeadingRad);
        double s = Math.sin(startHeadingRad);
        double dx0 = dxField * c + dyField * s;
        double dy0 = -dxField * s + dyField * c;

        // Solve for offset errors
        double a = Math.sin(deltaHeading);
        double b = 1.0 - Math.cos(deltaHeading);
        double denom = a * a + b * b;

        if (denom < MIN_SOLVE_DENOM) {
            // Rotation is too small OR too close to a full turn (degenerate for this math).
            lastXErrorInches = null;
            lastYErrorInches = null;
            lastRecommendedStrafePodOffsetForwardInches = null;
            lastRecommendedForwardPodOffsetLeftInches = null;
            lastSolveNote = "Rotate ~180° (avoid ~360°) for a stable solve";
        } else {
            // xError = x_est - x_true (strafe pod forward offset error)
            // yError = y_est - y_true (forward pods left offset error)
            double xError = (b * dx0 - a * dy0) / denom;
            double yError = (a * dx0 + b * dy0) / denom;

            lastXErrorInches = xError;
            lastYErrorInches = yError;

            // Recommended new offsets = current_est - error
            lastRecommendedStrafePodOffsetForwardInches = cfg.pinpoint.strafePodOffsetForwardInches - xError;
            lastRecommendedForwardPodOffsetLeftInches = cfg.pinpoint.forwardPodOffsetLeftInches - yError;

            lastSolveNote = null;
        }

        // Return to idle after computing.
        phase = Phase.IDLE;
        autoSample = false;
    }

    private void ensureAprilTagAssistReady() {
        if (!cfg.enableAprilTagAssist) return;
        if (tagSensor != null) return;
        if (selectedCameraName == null) return;

        FtcVision.Config vCfg = FtcVision.Config.defaults()
                .withCameraMount(cfg.cameraMount);
        if (cfg.tagLibrary != null) {
            vCfg.withTagLibrary(cfg.tagLibrary);
        }

        tagSensor = FtcVision.aprilTags(ctx.hw, selectedCameraName, vCfg);
        tagTarget = new TagTarget(tagSensor, layout.ids(), cfg.maxTagAgeSec);

        TagOnlyPoseEstimator.Config estCfg = TagOnlyPoseEstimator.Config.defaults()
                .withCameraMount(cfg.cameraMount);
        tagEstimator = new TagOnlyPoseEstimator(tagTarget, layout, estCfg);
    }

    private void updateSensors() {
        pinpoint.update(ctx.clock);
        latestPinpointPose = pinpoint.getEstimate().toPose2d();

        if (tagTarget != null && tagEstimator != null) {
            tagTarget.update(ctx.clock);
            tagEstimator.update(ctx.clock);

            PoseEstimate tagEst = tagEstimator.getEstimate();
            latestTagPose = tagEst.hasPose ? tagEst.toPose2d() : null;
            if (isSampleActive() && latestTagPose != null) {
                endTagPose = latestTagPose;
            }
        }
    }

    private void renderTelemetry(boolean initPhase) {
        ctx.telemetry.addLine(name());

        // Quick status
        ctx.telemetry.addData("Phase", phase);
        ctx.telemetry.addData("Auto sample", autoSample);

        if (phase == Phase.SEARCH_TAG_START || phase == Phase.SEARCH_TAG_END) {
            double turned = Math.abs(tagSearchUnwrapper.getUnwrappedRad() - tagSearchStartUnwrappedRad);
            ctx.telemetry.addData("Tag search turned [deg]", String.format(Locale.US, "%.1f", Math.toDegrees(turned)));
            ctx.telemetry.addData("Tag search stable", String.format(Locale.US, "%d/%d", tagStableFrames, cfg.tagSearchStableFrames));
        }

        if (cfg.enableAprilTagAssist) {
            String cam = (selectedCameraName != null) ? selectedCameraName : "<select webcam>";
            ctx.telemetry.addData("Tag assist", true);
            ctx.telemetry.addData("Webcam", cam);
            ctx.telemetry.addData("Tag pose", latestTagPose != null ? latestTagPose.toString() : "<none>");
            if (initPhase && cameraPicker != null && selectedCameraName == null) {
                ctx.telemetry.addLine();
                cameraPicker.render(ctx.telemetry);
            }
        } else {
            ctx.telemetry.addData("Tag assist", false);
            if (aprilTagAssistNotice != null && !aprilTagAssistNotice.isEmpty()) {
                ctx.telemetry.addData("Assist note", aprilTagAssistNotice);
            }
        }

        ctx.telemetry.addLine();

        // Current pose
        ctx.telemetry.addData("Pinpoint pose", latestPinpointPose.toString());

        // Current config offsets
        ctx.telemetry.addData(
                "Current offsets (in)",
                String.format(
                        Locale.US,
                        "forwardPodOffsetLeft=%.3f, strafePodOffsetForward=%.3f",
                        cfg.pinpoint.forwardPodOffsetLeftInches,
                        cfg.pinpoint.strafePodOffsetForwardInches
                )
        );

        // Make the "copy/paste" result hard to miss.
        if (lastRecommendedForwardPodOffsetLeftInches != null
                && lastRecommendedStrafePodOffsetForwardInches != null) {
            ctx.telemetry.addData(
                    "Recommended offsets (in)",
                    String.format(
                            Locale.US,
                            "forwardPodOffsetLeft=%.3f, strafePodOffsetForward=%.3f",
                            lastRecommendedForwardPodOffsetLeftInches,
                            lastRecommendedStrafePodOffsetForwardInches
                    )
            );
            ctx.telemetry.addData(
                    "Paste into config",
                    String.format(
                            Locale.US,
                            ".withOffsets(%.3f, %.3f)",
                            lastRecommendedForwardPodOffsetLeftInches,
                            lastRecommendedStrafePodOffsetForwardInches
                    )
            );
        }

        ctx.telemetry.addLine();

        // Instructions (dynamic by phase)
        ctx.telemetry.addLine();
        if (initPhase) {
            ctx.telemetry.addLine("IMPORTANT: Press PLAY to enable motor movement (auto sample / stick rotate).");
        }
        switch (phase) {
            case IDLE:
                ctx.telemetry.addLine("Controls: X reset | A manual sample | Y auto sample | B abort");
                if (drive == null) {
                    ctx.telemetry.addLine("(No drive configured) You will rotate the robot by hand.");
                }
                if (cfg.enableAprilTagAssist) {
                    ctx.telemetry.addLine("Tip: if a known-pose tag is visible, the tester will align Pinpoint to it.");
                }
                break;

            case SEARCH_TAG_START:
                ctx.telemetry.addLine("Searching for a known-pose tag...");
                ctx.telemetry.addLine("A: skip (start without tags) | B: abort");
                break;

            case ROTATING:
                if (autoSample) {
                    if (cfg.enableAprilTagAssist && cfg.autoComputeAfterAutoSample) {
                        ctx.telemetry.addLine("Auto rotating... will stop at target and auto-compute (B abort)");
                    } else {
                        ctx.telemetry.addLine("Auto rotating... (B abort)");
                    }
                } else {
                    if (drive != null) {
                        ctx.telemetry.addLine("Rotate now: right stick X. Press A when done.");
                    } else {
                        ctx.telemetry.addLine("Rotate the robot by hand. Press A when done.");
                    }
                }
                if (autoSample && cfg.enableAprilTagAssist && cfg.autoComputeAfterAutoSample) {
                    ctx.telemetry.addLine("Tip: with tag assist, results compute automatically after the turn.");
                } else if (cfg.enablePostRotateRecenter) {
                    ctx.telemetry.addLine("After you stop rotation: you can recenter, then press A to compute.");
                } else {
                    ctx.telemetry.addLine("Press A to compute immediately when rotation is complete.");
                }
                break;

            case SEARCH_TAG_END:
                ctx.telemetry.addLine("Trying to reacquire a tag pose for the end of the sample...");
                ctx.telemetry.addLine("A: skip | B: abort");
                break;

            case POST_RECENTER:
                ctx.telemetry.addLine("Recenter (optional): translate back to the starting spot, then press A.");
                if (drive != null) {
                    ctx.telemetry.addLine("Left stick: translate (no rotate)");
                } else {
                    ctx.telemetry.addLine("(No drive configured) You can physically reposition the robot.");
                }
                if (cfg.enableAprilTagAssist && startTagPose != null && latestTagPose != null) {
                    double dxToStart = startTagPose.xInches - latestTagPose.xInches;
                    double dyToStart = startTagPose.yInches - latestTagPose.yInches;
                    ctx.telemetry.addData(
                            "To start (tag) [in]",
                            String.format(Locale.US, "Δx=%.2f, Δy=%.2f", dxToStart, dyToStart)
                    );
                }
                break;
        }

        // Results
        if (lastDxFieldInches != null && lastDyFieldInches != null && lastDeltaHeadingRad != null) {
            ctx.telemetry.addLine();
            ctx.telemetry.addLine("Last sample:");
            ctx.telemetry.addData(
                    "  Δx, Δy (field) [in]",
                    String.format(Locale.US, "%.3f, %.3f", lastDxFieldInches, lastDyFieldInches)
            );
            ctx.telemetry.addData(
                    "  Δheading [deg]",
                    String.format(Locale.US, "%.1f", Math.toDegrees(lastDeltaHeadingRad))
            );

            if (cfg.enableAprilTagAssist) {
                ctx.telemetry.addData("  Tag start/end", (lastHadTagStart ? "Y" : "N") + "/" + (lastHadTagEnd ? "Y" : "N"));
            }

            if (lastXErrorInches != null && lastYErrorInches != null) {
                ctx.telemetry.addData(
                        "  Offset error (in)",
                        String.format(Locale.US, "xError=%.3f, yError=%.3f", lastXErrorInches, lastYErrorInches)
                );
            } else {
                String note = (lastSolveNote != null) ? lastSolveNote : "<insufficient/degenerate rotation>";
                ctx.telemetry.addData("  Offset error", note);
            }

            if (lastRecommendedForwardPodOffsetLeftInches != null && lastRecommendedStrafePodOffsetForwardInches != null) {
                ctx.telemetry.addData(
                        "  Recommended offsets (in)",
                        String.format(
                                Locale.US,
                                "forwardPodOffsetLeft=%.3f, strafePodOffsetForward=%.3f",
                                lastRecommendedForwardPodOffsetLeftInches,
                                lastRecommendedStrafePodOffsetForwardInches
                        )
                );
                ctx.telemetry.addLine("Copy these into your PinpointPoseEstimator.Config / RobotConfig.");
            }
        }
    }

    private static boolean isLikelyIdentity(CameraMountConfig mount) {
        if (mount == null) {
            return true;
        }
        Pose3d p = mount.robotToCameraPose();
        double tol = 1e-6;
        return Math.abs(p.xInches) < tol
                && Math.abs(p.yInches) < tol
                && Math.abs(p.zInches) < tol
                && Math.abs(p.yawRad) < tol
                && Math.abs(p.pitchRad) < tol
                && Math.abs(p.rollRad) < tol;
    }

    @Override
    protected void onStop() {
        // Release vision resources (VisionPortal) if AprilTag assist was used.
        if (tagSensor != null) {
            tagSensor.close();
            tagSensor = null;
        }
    }

    /**
     * Tracks an unwrapped angle by accumulating wrapped deltas.
     */
    private static final class AngleUnwrapper {
        private boolean initialized = false;
        private double prevRad = 0.0;
        private double unwrappedRad = 0.0;

        public void reset(double initialRad) {
            initialized = true;
            prevRad = initialRad;
            unwrappedRad = initialRad;
        }

        public void update(double currentRad) {
            if (!initialized) {
                reset(currentRad);
                return;
            }
            double d = MathUtil.wrapToPi(currentRad - prevRad);
            unwrappedRad += d;
            prevRad = currentRad;
        }

        public double getUnwrappedRad() {
            return unwrappedRad;
        }
    }
}
