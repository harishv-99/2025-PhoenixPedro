package edu.ftcsushi.fw.tools.tester.calibration;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.Locale;
import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.core.math.MathUtil;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.MecanumDrivebase;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.field.TagLayouts;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcGameTagLayout;
import edu.ftcsushi.fw.ftc.FtcTagLayoutDebug;
import edu.ftcsushi.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcsushi.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig;
import edu.ftcsushi.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcsushi.fw.ftc.localization.PinpointKinematicSnapshot;
import edu.ftcsushi.fw.ftc.vision.OwnedAprilTagCamera;
import edu.ftcsushi.fw.ftc.vision.AprilTagCameraFactory;
import edu.ftcsushi.fw.ftc.vision.VisionReadiness;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.apriltag.AprilTagPoseEstimator;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.tools.tester.BaseTeleOpTester;
import edu.ftcsushi.fw.ftc.ui.HardwareNamePicker;
import edu.ftcsushi.fw.input.binding.Bindings;

/**
 * Calibrates goBILDA Pinpoint odometry pod offsets by observing translation drift while rotating in place.
 *
 * <p>Why this works: when configured pod offsets differ from the physical placement relative to the
 * chosen robot reference point, rotation produces a modeled translation error ("drift") in the
 * Pinpoint-reported pose. Correct offsets account for off-center pods. Given a heading change and
 * either a return of that same reference point to its starting position or independent tag-measured
 * translation, the residual lets us solve for replacement offsets. Unaccounted real translation
 * contaminates the result; this calculation cannot distinguish every source of slip or noise.</p>
 *
 * <p>Recommendations are absolute replacement offsets, not adjustments to add. A repeat with
 * corrected configuration should recommend little change, not necessarily offsets near zero.</p>
 *
 * <p>A download-capable tester host freezes a full-precision text report after a solve or an
 * active attempt's discard. It preserves available capture-aligned endpoints before cleanup;
 * formatting/publication follows the existing zero attempt. Failure reports never carry a prior
 * candidate. A new admitted attempt, reset, or stop withdraws the old report. These are bounded
 * candidate facts for an external calibration record, not a raw trace, physical acceptance,
 * proof that a zero reached hardware, or replay input. Optional transport failures do not alter
 * calibration calculations or prevent cleanup.</p>
 *
 * <h2>Controls</h2>
 * <ul>
 *   <li><b>X</b>: Reset pose + clear results</li>
 *   <li><b>A</b>: Start/stop a manual sample (you rotate the robot)</li>
 *   <li><b>Y</b>: Auto sample (rotate to target angle) <i>requires a mecanum drive config</i></li>
 *   <li><b>B</b>: Abort sample</li>
 *   <li><b>Right stick X</b>: Manual rotate (when drive is configured)</li>
 * </ul>
 *
 * <p><b>Important:</b> a successful ordinary <b>INIT</b> can configure motor direction/brake,
 * poll/reset Pinpoint, and select/open vision, but does not invoke the drive command path. START
 * clears pending motion intent and sends the drive's first explicit zero before RUN can move.
 * Failed-initialization rollback and STOP-before-START are deliberate cleanup exceptions: either
 * may command physical zero while releasing an already returned drive owner.</p>
 *
 * <p>Optional: supply a backend-neutral AprilTag vision-factory builder to subtract independently
 * measured tag displacement while sampling. Each endpoint pairs a full camera observation with
 * raw Pinpoint history at its exact capture time; missing evidence never substitutes a current
 * pose. The two displacements are expressed in their own robot-at-start frames before subtraction.
 * This software matching does not prove physical timing accuracy or remove all slip/noise.
 * A null builder explicitly keeps the independent Pinpoint workflow
 * vision-free. Put fixed field facts and mount-free solver/age policy in {@link Config}; the opened
 * vision lane remains the sole owner of camera hardware, its tag library, and camera mount.</p>
 *
 * <p>With a drive, each automatic turn or tag-search phase has its own elapsed-time limit from
 * {@link Config#automaticPhaseTimeoutSec}. A can also begin a powered start-tag search. Expiry
 * discards the attempt and requests zero before polling or accepting another phase action; B
 * likewise wins over A/Y/X in the same loop cycle. A fresh later button press can retry. These
 * cooperative checks require a serviced OpMode loop; they are not a hardware watchdog or proof
 * of safe power, clearance, braking, or a physically sufficient time limit.</p>
 */
public final class PinpointPodOffsetCalibrator extends BaseTeleOpTester {

    private static final String CONFIG_CONTEXT = "PinpointPodOffsetCalibrator.Config";

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

    /** Mutable, data-only authoring configuration for the tester. */
    public static final class Config {

        /**
         * Required Pinpoint estimator config (includes current pod offsets).
         */
        public PinpointOdometryPredictor.Config pinpoint = PinpointOdometryPredictor.Config.defaults();

        /**
         * Optional complete mecanum construction config. Null selects a hand-motion workflow and
         * leaves every drive-command tuning field dormant.
         */
        public FtcDrives.MecanumConfig mecanum = null;

        /**
         * Manual rotation scale when using the right stick X. With a drive, this must be finite in
         * the inclusive range {@code [0, 1]}.
         */
        public double manualOmegaScale = 0.6;

        /**
         * Auto rotation omega magnitude. With a drive, this must be finite in {@code (0, 1]};
         * {@link #targetTurnRad}'s sign selects clockwise or counter-clockwise motion.
         */
        public double autoOmegaCmd = 0.35;

        /**
         * Target rotation in radians for auto samples. With a drive, it must be finite and satisfy
         * {@code 4 * sin(targetTurnRad / 2)^2 >= 0.5}; its sign selects turn direction.
         */
        public double targetTurnRad = Math.PI;

        /**
         * Maximum elapsed seconds for each automatic phase: start-tag search (including one
         * initiated by A), Y's rotation, and end-tag search. Each phase starts a fresh budget.
         * With a drive this must be finite and {@code > 0}, even without vision or tag searches;
         * it is dormant without a drive. Hand/stick rotation and manual recentering are untimed.
         *
         * <p>The 10-second software default is not a hardware-validated duration. At or after the
         * deadline, the next serviced RUN loop discards the attempt, requests zero, and retains
         * the failed phase's reason. An invalid/reset clock timestamp fails the attempt too.
         * There is no automatic retry or timeout fallback to a calibration result.</p>
         */
        public double automaticPhaseTimeoutSec = 10.0;

        /**
         * If true, auto samples (Y) will compute results automatically once the rotation is done
         * <b>when AprilTag assist is active</b>.
         *
         * <p>The tester uses current Pinpoint heading to stop at the configured target turn and
         * subtracts independently measured tag displacement over matched capture endpoints.
         * With eligible evidence at both ends, Y's sample stops and computes without another
         * button press. This does not prove physical latency accuracy or remove all slip/noise.
         * When false, enabled recentering requires a final A; disabling recentering explicitly
         * selects computation at the end of rotation instead.</p>
         *
         * <p>Each assisted endpoint requires raw odometry at the tag's capture time. Missing end
         * evidence is searched for when configured, otherwise the attempt is discarded; it never
         * falls back to an unassisted solve.</p>
         */
        public boolean autoComputeAfterAutoSample = true;

        // -------------------------------------------------------------------------------------
        // AprilTag assist enhancements
        // -------------------------------------------------------------------------------------

        /**
         * If AprilTag assist is enabled and no tag is currently visible, the tester can
         * auto-rotate to search for any known-pose tag (as defined by {@link #fixedTagLayout}).
         */
        public boolean enableAutoTagSearchAtStart = true;

        /**
         * Maximum amount of rotation (radians) to spend searching for a tag. When start search,
         * drive, and AprilTag assist are active, this must be finite and {@code > 0}.
         *
         * <p>Defaults to ~2 full turns. Exhaustion discards the attempt. A may explicitly skip
         * this start search before an assisted start has been captured.</p>
         */
        public double tagSearchMaxTurnRad = 4.0 * Math.PI;

        /**
         * Rotation omega command used while searching for a tag. When drive, assist, and either
         * search branch are active, it must be finite with {@code 0 < abs(value) <= 1}; its sign
         * directly selects search direction.
         */
        public double tagSearchOmegaCmd = 0.25;

        /**
         * Number of consecutive eligible, strictly advancing capture timestamps required during
         * a tag search. Repeated frames do not count again. Missing, invalid, stale, or older
         * evidence breaks the streak without making a previously consumed frame new again.
         * Direct visible start/end acquisition requires one matched pair, not this search count.
         * This is valid-frame acquisition, not a jitter or statistical-independence test.
         * With drive and assist this must be at least one whenever either search branch is enabled.
         */
        public int tagSearchStableFrames = 3;

        /**
         * If true, when an assisted auto sample requests computation at the end of rotation,
         * missing capture-matched end evidence may start a bounded reacquisition search.
         * Merely entering optional recentering does not require a redundant end capture;
         * the final A there must acquire its own eligible matched end.
         */
        public boolean enableAutoTagSearchAtEnd = true;

        /**
         * Maximum extra rotation (radians) allowed while searching for a tag at the end of an
         * auto-sample. When that search, drive, and assist are active, it must be finite and
         * {@code > 0}.
         */
        public double tagEndSearchMaxExtraTurnRad = 2.0 * Math.PI;

        /**
         * If true, after the rotation portion of a sample finishes, the tester pauses and allows
         * manual translation to "recenter" the same chosen robot reference point back to its
         * starting position before computing results. A software pose reset does not move that point.
         *
         * <p>This is especially useful for auto-turning on imperfect flooring where the robot drifts
         * laterally while rotating. Re-centering reduces bias from real translation during the turn.</p>
         */
        public boolean enablePostRotateRecenter = true;

        /**
         * Manual translation scale in recenter mode. When recentering and drive are active, this
         * must be finite in the inclusive range {@code [0, 1]}.
         */
        public double recenterTranslationScale = 0.6;

        /**
         * Preferred FTC hardware-map name for the AprilTag vision device used by assist.
         *
         * <p>When {@code null}, the tester shows a runtime hardware picker for the configured
         * {@link #visionDeviceType}. A non-null value must be nonblank and is trimmed before it is
         * passed once to the vision-factory builder.</p>
         */
        public String preferredVisionDeviceName = null;

        /**
         * FTC hardware type to enumerate when {@link #preferredVisionDeviceName} is not supplied.
         *
         * <p>The default is {@link WebcamName}. This field is active only when the preferred name is
         * {@code null} and the constructor receives a vision-factory builder.</p>
         */
        public Class<? extends HardwareDevice> visionDeviceType = WebcamName.class;

        /**
         * Nonblank title shown above the AprilTag vision-device picker when assist needs a runtime
         * selection. It is dormant when a preferred name or null builder selects no picker.
         */
        public String visionPickerTitle = "Select Camera";

        /**
         * Fixed field-tag facts used to interpret AprilTag observations. Required and snapshotted
         * when assist is selected. An empty layout is structurally valid but cannot publish a
         * fixed-layout field-pose correction.
         */
        public TagLayout fixedTagLayout = FtcGameTagLayout.currentGameFieldFixed();

        /**
         * Mount-free AprilTag age and fixed-tag solver policy used by assist. The age must be finite
         * and {@code >= 0}; the opened vision lane remains the sole camera-mount owner.
         */
        public AprilTagLocalizationConfig aprilTags = AprilTagLocalizationConfig.defaults();

        /**
         * Bounded raw Pinpoint history used only to match assisted endpoints at camera capture
         * time. Required, validated, and defensively copied when a vision factory selects assist,
         * even without a drive; dormant with a null factory. The camera's independent acceptance
         * age remains {@link #aprilTags}'s maxDetectionAgeSec. Missing history never extrapolates
         * or substitutes the delivery-time pose. Defaults are software bounds, not physical proof.
         */
        public PlanarPoseHistory.Config assistOdometryHistory = PlanarPoseHistory.Config.defaults();

        private Config() {
            // Defaults assigned in field initializers.
        }

        /**
         * Returns a new mutable authoring draft initialized with software defaults.
         *
         * <p>Defaults do not prove motor wiring, Pinpoint calibration, camera mounting, field setup,
         * or safe motion. The tester snapshots only the branches selected by its constructor.</p>
         */
        public static Config defaults() {
            return new Config();
        }
    }

    private final Config cfg;
    private final Function<String, AprilTagCameraFactory> visionLaneFactoryBuilder;
    private final String fixedTagLayoutPolicySummary;

    private PinpointOdometryPredictor pinpoint;
    private MecanumDrivebase drive;
    private boolean started;

    // AprilTag assist
    private HardwareNamePicker visionPicker;
    private String selectedVisionDeviceName;
    private AprilTagCameraFactory pendingVisionFactory;
    private final TagLayout layout;
    private OwnedAprilTagCamera visionLane;
    private AprilTagSensor tagSensor;
    private AprilTagPoseEstimator tagEstimator;
    private PlanarPoseHistory assistOdometryHistory;
    private PoseEstimate previousAssistOdometryEstimate;
    private long assistHistorySourceSegment;
    private long assistHistoryGeneration;
    private String activeVisionDescription;
    private String aprilTagAssistNotice;
    private boolean visionCleanupFailed;
    private boolean visionTerminalRequested;
    private boolean visionRetryBlocked;
    private boolean aprilTagAssistUnavailable;
    private RuntimeException visionFailure;
    private VisionReadiness visionReadiness = VisionReadiness.notReady("No vision device is open");

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
    private LoopTimestamp automaticPhaseStartedAt = LoopTimestamp.unavailable();
    // Do not clear this in reset/phase cleanup: B and expiry must win for the entire cycle.
    private long motionInhibitedCycle = Long.MIN_VALUE;
    private String lastAttemptFailure;
    private boolean resetRequested;
    private boolean primaryActionRequested;
    private boolean autoStartRequested;

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
    private LoopTimestamp tagSearchConsumedTimestamp = LoopTimestamp.unavailable();

    private Pose2d startPinpointPose = Pose2d.zero();
    private AssistedEndpoint startAssistEndpoint;
    private AssistedEndpoint endAssistEndpoint;
    private String assistEvidenceNotice;

    private double startHeadingRad = 0.0;
    private double startHeadingUnwrappedRad = 0.0;
    private final AngleUnwrapper headingUnwrapper = new AngleUnwrapper();

    // latest values
    private Pose2d latestPinpointPose = Pose2d.zero();
    private PoseEstimate latestTagEstimate;

    // last sample results (null means not available)
    private Double lastDxStartBodyInches = null;
    private Double lastDyStartBodyInches = null;
    private Double lastDeltaHeadingRad = null;

    private Double lastXErrorInches = null;
    private Double lastYErrorInches = null;

    private Double lastRecommendedStrafePodOffsetForwardInches = null;
    private Double lastRecommendedForwardPodOffsetLeftInches = null;

    // If the last sample couldn't be solved (too little rotation / degenerate rotation), this
    // is a short, driver-facing note explaining why.
    private String lastSolveNote = null;
    private final CalibrationReport report = new CalibrationReport();

    private boolean lastHadTagStart = false;
    private boolean lastHadTagEnd = false;

    /** Frozen experimental endpoint; both poses describe one capture, not its delivery loop. */
    private static final class AssistedEndpoint {
        final PoseEstimate tagEstimate;
        final PlanarPoseHistory.Lookup odometry;
        final OwnedAprilTagCamera cameraOwner;
        final AprilTagPoseEstimator estimator;
        final long rawTrajectorySegment;
        final long historyGeneration;

        AssistedEndpoint(PoseEstimate tagEstimate, PlanarPoseHistory.Lookup odometry,
                         OwnedAprilTagCamera cameraOwner, AprilTagPoseEstimator estimator,
                         long rawTrajectorySegment, long historyGeneration) {
            this.tagEstimate = tagEstimate;
            this.odometry = odometry;
            this.cameraOwner = cameraOwner;
            this.estimator = estimator;
            this.rawTrajectorySegment = rawTrajectorySegment;
            this.historyGeneration = historyGeneration;
        }
    }

    /**
     * Creates the calibrator from one explicit configuration and optional AprilTag behavior peer.
     *
     * <p>The mutable Config is validated and defensively captured before any FTC context, hardware,
     * picker, telemetry, or Panels effect. A null {@code visionFactoryBuilder} explicitly selects
     * the non-vision Pinpoint workflow; no vision draft is then inspected. A non-null builder must
     * only validate and capture backend configuration when applied: it must not look up hardware,
     * open a portal, or otherwise acquire an FTC resource. Each factory opening must create a fresh
     * independently owned lane. Keep any template or custom SDK tag library borrowed by the builder
     * stable for this tester's lifetime and every possible picker retry.</p>
     *
     * <p>When a preferred device name is configured, the builder is applied exactly once here,
     * after intrinsic validation. With a picker it is applied once only after each confirmed
     * selection. Real hardware remains deferred to {@link #onInit()} or a later selection loop.</p>
     *
     * <p>Successful ordinary INIT constructs resources without a drive command. START owns the
     * first explicit zero and gates all later motion. Failed-init rollback and final cleanup may
     * still stop an already returned drive before START; those are safety/lifecycle effects, not
     * ordinary INIT behavior.</p>
     *
     * @param config complete mutable authoring draft; start from {@link Config#defaults()}
     * @param visionFactoryBuilder nullable backend-neutral deferred-factory builder; null disables
     *                             AprilTag assist
     * @throws NullPointerException if the Config or an active required field is null
     * @throws IllegalArgumentException if an active Config value is outside its documented domain
     * @throws RuntimeException if a preferred-name builder cannot capture a deferred factory
     */
    public PinpointPodOffsetCalibrator(
            Config config,
            Function<String, AprilTagCameraFactory> visionFactoryBuilder) {
        ConfigCapture capture = captureConfig(config, visionFactoryBuilder != null);
        this.cfg = capture.config;
        this.visionLaneFactoryBuilder = visionFactoryBuilder;
        this.layout = capture.layout;
        this.fixedTagLayoutPolicySummary = capture.layoutPolicySummary;
        this.selectedVisionDeviceName = this.cfg.preferredVisionDeviceName;
        this.pendingVisionFactory = selectedVisionDeviceName != null
                ? applyVisionFactoryBuilder(selectedVisionDeviceName)
                : null;
    }

    private static ConfigCapture captureConfig(Config config, boolean assistSelected) {
        Config draft = Objects.requireNonNull(
                config,
                CONFIG_CONTEXT + " must not be null; call Config.defaults() explicitly"
        );
        Config captured = Config.defaults();
        captured.pinpoint = Objects.requireNonNull(
                draft.pinpoint,
                CONFIG_CONTEXT + ".pinpoint must not be null"
        ).validatedCopy(CONFIG_CONTEXT + ".pinpoint");

        boolean hasDrive = draft.mecanum != null;
        if (hasDrive) {
            try {
                captured.mecanum = draft.mecanum.copy();
            } catch (RuntimeException failure) {
                throw invalidConfig(CONFIG_CONTEXT + ".mecanum", failure);
            }
            captured.manualOmegaScale = requireFiniteRange(
                    draft.manualOmegaScale,
                    0.0,
                    1.0,
                    CONFIG_CONTEXT + ".manualOmegaScale"
            );
            captured.autoOmegaCmd = requireFiniteRange(
                    draft.autoOmegaCmd,
                    0.0,
                    1.0,
                    CONFIG_CONTEXT + ".autoOmegaCmd",
                    false,
                    true
            );
            captured.targetTurnRad = requireStableTargetTurn(draft.targetTurnRad);
            captured.automaticPhaseTimeoutSec = requirePositiveFinite(
                    draft.automaticPhaseTimeoutSec,
                    CONFIG_CONTEXT + ".automaticPhaseTimeoutSec"
            );
            if (draft.enablePostRotateRecenter) {
                captured.recenterTranslationScale = requireFiniteRange(
                        draft.recenterTranslationScale,
                        0.0,
                        1.0,
                        CONFIG_CONTEXT + ".recenterTranslationScale"
                );
            }
        } else {
            captured.mecanum = null;
        }

        captured.autoComputeAfterAutoSample = draft.autoComputeAfterAutoSample;
        captured.enableAutoTagSearchAtStart = draft.enableAutoTagSearchAtStart;
        captured.enableAutoTagSearchAtEnd = draft.enableAutoTagSearchAtEnd;
        captured.enablePostRotateRecenter = draft.enablePostRotateRecenter;

        TagLayout capturedLayout = null;
        String policySummary = null;
        if (assistSelected) {
            if (draft.preferredVisionDeviceName == null) {
                captured.preferredVisionDeviceName = null;
                captured.visionDeviceType = Objects.requireNonNull(
                        draft.visionDeviceType,
                        CONFIG_CONTEXT + ".visionDeviceType must not be null when preferredVisionDeviceName is null"
                );
                captured.visionPickerTitle = requireNonBlankTrimmed(
                        draft.visionPickerTitle,
                        CONFIG_CONTEXT + ".visionPickerTitle"
                );
            } else {
                captured.preferredVisionDeviceName = requireNonBlankTrimmed(
                        draft.preferredVisionDeviceName,
                        CONFIG_CONTEXT + ".preferredVisionDeviceName"
                );
            }

            TagLayout authoredLayout = Objects.requireNonNull(
                    draft.fixedTagLayout,
                    CONFIG_CONTEXT + ".fixedTagLayout must not be null when AprilTag assist is selected"
            );
            if (authoredLayout instanceof FtcGameTagLayout) {
                policySummary = ((FtcGameTagLayout) authoredLayout).policySummaryLine();
            }
            try {
                capturedLayout = TagLayouts.snapshot(authoredLayout);
            } catch (RuntimeException failure) {
                throw invalidConfig(CONFIG_CONTEXT + ".fixedTagLayout", failure);
            }
            captured.fixedTagLayout = capturedLayout;
            captured.aprilTags = Objects.requireNonNull(
                    draft.aprilTags,
                    CONFIG_CONTEXT + ".aprilTags must not be null when AprilTag assist is selected"
            ).validatedCopy(CONFIG_CONTEXT + ".aprilTags");
            captured.assistOdometryHistory = Objects.requireNonNull(
                    draft.assistOdometryHistory,
                    CONFIG_CONTEXT + ".assistOdometryHistory must not be null when AprilTag assist is selected"
            ).validatedCopy(CONFIG_CONTEXT + ".assistOdometryHistory");

            if (hasDrive
                    && (draft.enableAutoTagSearchAtStart
                    || draft.enableAutoTagSearchAtEnd)) {
                captured.tagSearchOmegaCmd = requireSignedNormalizedNonzero(
                        draft.tagSearchOmegaCmd,
                        CONFIG_CONTEXT + ".tagSearchOmegaCmd"
                );
                if (draft.tagSearchStableFrames < 1) {
                    throw new IllegalArgumentException(
                            CONFIG_CONTEXT + ".tagSearchStableFrames must be >= 1 when tag search is enabled, got "
                                    + draft.tagSearchStableFrames
                    );
                }
                captured.tagSearchStableFrames = draft.tagSearchStableFrames;
                if (draft.enableAutoTagSearchAtStart) {
                    captured.tagSearchMaxTurnRad = requirePositiveFinite(
                            draft.tagSearchMaxTurnRad,
                            CONFIG_CONTEXT + ".tagSearchMaxTurnRad"
                    );
                }
                if (draft.enableAutoTagSearchAtEnd) {
                    captured.tagEndSearchMaxExtraTurnRad = requirePositiveFinite(
                            draft.tagEndSearchMaxExtraTurnRad,
                            CONFIG_CONTEXT + ".tagEndSearchMaxExtraTurnRad"
                    );
                }
            }
        } else {
            captured.preferredVisionDeviceName = null;
            captured.fixedTagLayout = null;
            captured.aprilTags = null;
            captured.assistOdometryHistory = null;
        }

        return new ConfigCapture(captured, capturedLayout, policySummary);
    }

    private AprilTagCameraFactory applyVisionFactoryBuilder(String selectedName) {
        final AprilTagCameraFactory factory;
        try {
            factory = visionLaneFactoryBuilder.apply(selectedName);
        } catch (RuntimeException failure) {
            throw new IllegalStateException(
                    CONFIG_CONTEXT + " vision factory builder failed for device '"
                            + selectedName + "': " + primaryFailureSummary(failure),
                    failure
            );
        }
        if (factory == null) {
            throw new IllegalStateException(
                    CONFIG_CONTEXT + " vision factory builder returned null for device '"
                            + selectedName + "'"
            );
        }
        return factory;
    }

    private static IllegalArgumentException invalidConfig(String context, RuntimeException failure) {
        return new IllegalArgumentException(
                context + " is invalid: " + failureSummary(failure),
                failure
        );
    }

    private static String requireNonBlankTrimmed(String value, String context) {
        if (value == null) {
            throw new NullPointerException(context + " must not be null");
        }
        String normalized = value.trim();
        if (normalized.isEmpty()) {
            throw new IllegalArgumentException(
                    context + " must contain a non-whitespace value, got '" + value + "'"
            );
        }
        return normalized;
    }

    private static double requireFiniteRange(
            double value,
            double min,
            double max,
            String context) {
        return requireFiniteRange(value, min, max, context, true, true);
    }

    private static double requireFiniteRange(
            double value,
            double min,
            double max,
            String context,
            boolean includeMin,
            boolean includeMax) {
        boolean below = includeMin ? value < min : value <= min;
        boolean above = includeMax ? value > max : value >= max;
        if (!Double.isFinite(value) || below || above) {
            String domain = (includeMin ? "[" : "(") + min + ", " + max
                    + (includeMax ? "]" : ")");
            throw new IllegalArgumentException(
                    context + " must be finite and in " + domain + ", got " + value
            );
        }
        return value;
    }

    private static double requirePositiveFinite(double value, String context) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(context + " must be finite and > 0, got " + value);
        }
        return value;
    }

    private static double requireSignedNormalizedNonzero(double value, String context) {
        if (!Double.isFinite(value) || value == 0.0 || Math.abs(value) > 1.0) {
            throw new IllegalArgumentException(
                    context + " must be finite with 0 < abs(value) <= 1, got " + value
            );
        }
        return value;
    }

    private static double requireStableTargetTurn(double targetTurnRad) {
        if (!Double.isFinite(targetTurnRad)) {
            throw new IllegalArgumentException(
                    CONFIG_CONTEXT + ".targetTurnRad must be finite radians, got " + targetTurnRad
            );
        }
        double halfTurnSin = Math.sin(targetTurnRad / 2.0);
        double denominator = 4.0 * halfTurnSin * halfTurnSin;
        if (denominator < MIN_SOLVE_DENOM) {
            throw new IllegalArgumentException(
                    CONFIG_CONTEXT + ".targetTurnRad must produce 4 * sin(targetTurnRad / 2)^2 >= "
                            + MIN_SOLVE_DENOM + ", got targetTurnRad=" + targetTurnRad
                            + " and denominator=" + denominator
            );
        }
        return targetTurnRad;
    }

    private static final class ConfigCapture {
        final Config config;
        final TagLayout layout;
        final String layoutPolicySummary;

        ConfigCapture(Config config, TagLayout layout, String layoutPolicySummary) {
            this.config = config;
            this.layout = layout;
            this.layoutPolicySummary = layoutPolicySummary;
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String name() {
        return "Pinpoint: Pod Offset Calibrator";
    }

    @Override
    protected void onInit() {
        // The optional drive is deliberately the first resource. FtcDrives validates the complete
        // wiring group before lookup and construction performs no drive command or run-mode access.
        if (cfg.mecanum != null) {
            try {
                drive = FtcDrives.mecanum(ctx.hw, cfg.mecanum);
            } catch (RuntimeException failure) {
                throw new IllegalStateException(
                        CONFIG_CONTEXT + ".mecanum construction failed before Pinpoint setup; "
                                + "no returned drive owner is available for rollback, so restart this OpMode "
                                + "if device construction or configuration began: "
                                + failureSummary(failure),
                        failure
                );
            }
        }

        try {
            pinpoint = new PinpointOdometryPredictor(ctx.hw, cfg.pinpoint);
            if (visionLaneFactoryBuilder != null) {
                assistOdometryHistory = new PlanarPoseHistory(pinpoint, cfg.assistOdometryHistory);
            }
        } catch (RuntimeException failure) {
            RuntimeException primary = new IllegalStateException(
                    CONFIG_CONTEXT + ".pinpoint construction failed after drive setup; "
                            + "stop and restart this OpMode because a partially configured vendor owner "
                            + "has no rollback seam: " + failureSummary(failure),
                    failure
            );
            throw rollbackDriveAfterInitializationFailure(primary);
        }

        if (visionLaneFactoryBuilder != null) {
            if (selectedVisionDeviceName == null) {
                visionPicker = new HardwareNamePicker(
                        ctx.hw,
                        cfg.visionDeviceType,
                        cfg.visionPickerTitle,
                        "Dpad: highlight | A: choose | X: refresh"
                );
                visionPicker.bind(
                        bindings,
                        gamepads.p1().dpadUp(),
                        gamepads.p1().dpadDown(),
                        gamepads.p1().a(),
                        gamepads.p1().x(),
                        () -> visionLane == null
                                && !visionCleanupFailed
                                && !visionRetryBlocked
                                && !visionTerminalRequested,
                        name -> {
                            selectedVisionDeviceName = name;
                            pendingVisionFactory = null;
                        }
                );
                visionPicker.refresh();
            }
        }

        // Keep picker-reused calibration actions separate from a vision-device picker that uses
        // A and X. Abort stays on the always-eligible root so an active drive phase can still be
        // stopped if a vision readiness failure deactivates the calibration context.
        Bindings.ControlContext calibrationControls = bindings.contextWhen(
                BooleanSource.of(this::calibrationControlsEligible),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL
        );
        calibrationControls.onRise(gamepads.p1().x(), () -> resetRequested = true);
        calibrationControls.onRise(gamepads.p1().a(), () -> primaryActionRequested = true);
        calibrationControls.onRise(gamepads.p1().y(), () -> autoStartRequested = true);
        bindings.onRise(gamepads.p1().b(), this::abortSample);

        // Construction already issued Pinpoint's mandatory nonblocking reset. Clear only internal
        // state here, without a duplicate pose reset or an INIT drive command.
        clearState(false);

        // A preferred factory was captured before any resource. Open it only after drive and
        // Pinpoint have been published. Picker-based owners wait for an explicit selection.
        ensureAprilTagAssistReady(true);
    }

    private RuntimeException rollbackDriveAfterInitializationFailure(RuntimeException primary) {
        MecanumDrivebase ownedDrive = drive;
        drive = null;
        if (ownedDrive == null) {
            return primary;
        }
        return CleanupActions.attemptAllAfterFailure(primary, ownedDrive::stop);
    }

    @Override
    protected void onStart() {
        started = true;
        clearPendingMotionIntent();
        if (drive != null) {
            // This is the first ordinary drive command. Construction/INIT configured direction and
            // brake only, so raw-power-mode preflight and explicit physical zero begin at START.
            drive.drive(DriveSignal.zero());
        }
    }

    @Override
    protected void onInitLoop(double dtSec) {
        ensureAprilTagAssistReady(true);

        // Keep estimators warm in init so the first sample isn't stale.
        updateSensors(true);
        consumeControlRequestsAfterCurrentPoll(false);

        renderTelemetry(true);
    }

    @Override
    protected void onLoop(double dtSec) {
        // Bindings have already serviced B. Neither a queued reset/start nor newly polled evidence
        // may rescue a phase whose budget has expired. Repeated calls in this cycle stay inhibited.
        if (started && (motionInhibitedThisCycle() || expireAutomaticPhaseIfNeeded())) {
            discardControlRequests();
            renderTelemetry(false);
            return;
        }
        ensureAprilTagAssistReady(false);

        updateSensors(false);
        if (motionInhibitedThisCycle()) {
            discardControlRequests();
            renderTelemetry(false);
            return;
        }
        if (isSampleActive() && startAssistEndpoint != null && !assistedStartStillCompatible()) {
            failAssistedAttempt("Assisted start lost its camera, clock epoch, or raw trajectory identity");
            renderTelemetry(false);
            return;
        }
        if (!started) {
            // A malformed/custom host cannot bypass START by invoking RUN directly.
            consumeControlRequestsAfterCurrentPoll(false);
            renderTelemetry(false);
            return;
        }
        // Finish the already-active sample's observation before A can compute or X can reset it.
        // A/Y may start a new software-zero sample below; never feed that sample the pre-reset pose.
        if (isSampleActive() && pinpointReadyForMotion()) {
            headingUnwrapper.update(latestPinpointPose.headingRad);
        }
        consumeControlRequestsAfterCurrentPoll(true);

        if (motionInhibitedThisCycle()) {
            renderTelemetry(false);
            return;
        }

        if (!pinpointReadyForMotion()) {
            abortSample();
            renderTelemetry(false);
            return;
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
                    drive.drive(DriveSignal.zero());
                }
                break;
        }

        renderTelemetry(false);
    }

    private boolean isSampleActive() {
        return phase == Phase.ROTATING || phase == Phase.SEARCH_TAG_END || phase == Phase.POST_RECENTER;
    }

    private boolean isAutomaticPhase() {
        return drive != null && (phase == Phase.SEARCH_TAG_START
                || phase == Phase.SEARCH_TAG_END || (phase == Phase.ROTATING && autoSample));
    }

    /** Capture the new phase's own boundary, never the interval preceding its entry. */
    private void enterPhase(Phase nextPhase) {
        phase = nextPhase;
        automaticPhaseStartedAt = isAutomaticPhase()
                ? ctx.clock.nowTimestamp() : LoopTimestamp.unavailable();
    }

    private boolean motionInhibitedThisCycle() {
        // Pre-init cleanup may have no context yet.
        return ctx != null && motionInhibitedCycle == ctx.clock.cycle();
    }

    private boolean expireAutomaticPhaseIfNeeded() {
        if (!isAutomaticPhase()) return false;
        double elapsedSec = automaticPhaseStartedAt.ageSec(ctx.clock);
        if (Double.isFinite(elapsedSec) && elapsedSec < cfg.automaticPhaseTimeoutSec) {
            return false;
        }
        lastAttemptFailure = Double.isFinite(elapsedSec)
                ? String.format(Locale.US, "%s timed out (%.2f / %.2f s). Attempt discarded; "
                        + "release controls, then press A or Y to retry.",
                        phase, elapsedSec, cfg.automaticPhaseTimeoutSec)
                : phase + " has invalid elapsed time (clock reset or unavailable timestamp). "
                        + "Attempt discarded; release controls, then press A or Y to retry.";
        clearLastResults();
        // Publish inactive state and inhibition before an external zero write can fail/reenter.
        abortSample();
        return true;
    }

    private void updateSearchForTagStart() {
        if (drive == null || tagEstimator == null) {
            failAssistedAttempt("Start-tag search lost its configured drive or camera");
            return;
        }

        // Track how far we've rotated while searching.
        tagSearchUnwrapper.update(latestPinpointPose.headingRad);
        double turned = Math.abs(tagSearchUnwrapper.getUnwrappedRad() - tagSearchStartUnwrappedRad);

        if (turned >= Math.abs(cfg.tagSearchMaxTurnRad)) {
            failAssistedAttempt("Start-tag search exhausted its angular limit");
            return;
        }

        AssistedEndpoint endpoint = matchLatestAssistedEndpoint(false);
        if (countSearchFrame(endpoint)) {
            startSampleInternal(autoSample, endpoint);
            return;
        }

        drive.drive(new DriveSignal(0.0, 0.0, cfg.tagSearchOmegaCmd));
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
            omega = -gamepads.p1().rightX().getAsDouble(ctx.clock) * cfg.manualOmegaScale;
        }

        drive.drive(new DriveSignal(0.0, 0.0, omega));
    }

    private void updateSearchForTagEnd() {
        if (drive == null || tagEstimator == null) {
            failAssistedAttempt("End-tag search lost its configured drive or camera");
            return;
        }

        // Rotate a little further to reacquire a tag pose, but cap how far we go.
        tagSearchUnwrapper.update(latestPinpointPose.headingRad);
        double turnedExtra = Math.abs(tagSearchUnwrapper.getUnwrappedRad() - tagSearchStartUnwrappedRad);

        if (turnedExtra >= Math.abs(cfg.tagEndSearchMaxExtraTurnRad)) {
            failAssistedAttempt("End-tag search exhausted its angular limit");
            return;
        }

        AssistedEndpoint endpoint = matchLatestAssistedEndpoint(true);
        if (countSearchFrame(endpoint)) {
            endAssistEndpoint = endpoint;
            finishSampleAndCompute();
            return;
        }

        drive.drive(new DriveSignal(0.0, 0.0, cfg.tagSearchOmegaCmd));
    }

    private void updatePostRecenter() {
        if (drive == null) {
            return;
        }

        // Allow the driver to translate (no rotation) to bring the robot back to the starting spot.
        double axial = gamepads.p1().leftY().getAsDouble(ctx.clock) * cfg.recenterTranslationScale;
        double lateral = -gamepads.p1().leftX().getAsDouble(ctx.clock) * cfg.recenterTranslationScale;

        drive.drive(new DriveSignal(axial, lateral, 0.0));
    }

    private void resetAndClear() {
        clearState(true);
    }

    private void clearState(boolean rebasePinpoint) {
        clearPendingMotionIntent();

        clearLastResults();
        if (!motionInhibitedThisCycle()) {
            lastAttemptFailure = null;
        }
        report.clear(ctx);

        if (rebasePinpoint && pinpoint != null) {
            pinpoint.setPose(Pose2d.zero());
        }
        headingUnwrapper.reset(0.0);
        tagSearchUnwrapper.reset(0.0);
    }

    private void clearPendingMotionIntent() {
        enterPhase(Phase.IDLE);
        autoSample = false;
        clearAssistEvidence();
        discardControlRequests();
    }

    private void discardControlRequests() {
        resetRequested = false;
        primaryActionRequested = false;
        autoStartRequested = false;
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
                failAssistedAttempt("End-tag search skipped after an assisted start");
                break;
            case POST_RECENTER:
                finishSampleAndCompute();
                break;
        }
    }

    /** Consume binding-edge intent only after this cycle's Pinpoint/vision update. */
    private void consumeControlRequestsAfterCurrentPoll(boolean runPhase) {
        if (motionInhibitedThisCycle()) {
            discardControlRequests();
            return;
        }
        boolean shouldReset = resetRequested;
        boolean shouldRunPrimaryAction = primaryActionRequested;
        boolean shouldStartAuto = autoStartRequested;
        discardControlRequests();

        if (terminalVisionFailureBlocksCalibration()) {
            // Identity-mount fallback explicitly clears this block through its unavailable state.
            // Every other terminal assist failure requires BACK/reopen and must discard even a
            // request captured earlier in the same cycle as the failure.
            clearPendingMotionIntent();
            return;
        }
        if (shouldReset) {
            resetAndClear();
        }
        if (!runPhase) {
            // INIT observes reset intent only. A/Y can never arm a motion phase that survives START.
            return;
        }
        if (!pinpointReadyForMotion()) {
            if (shouldRunPrimaryAction || shouldStartAuto) {
                abortSample();
            }
            return;
        }
        if (motionInhibitedThisCycle()) return;
        if (shouldRunPrimaryAction) {
            onAPress();
        }
        if (shouldStartAuto && !motionInhibitedThisCycle()) {
            onYPress();
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
        abortSample(phase == Phase.IDLE ? null : snapshotReport(
                lastAttemptFailure == null ? "ABORTED: active attempt discarded" : lastAttemptFailure,
                false));
    }

    /** Freeze before evidence clears, but give the existing physical zero precedence over export. */
    private void abortSample(Supplier<String> frozenReport) {
        if (ctx != null) {
            motionInhibitedCycle = ctx.clock.cycle();
        }
        clearPendingMotionIntent();
        try {
            if (started && drive != null) {
                drive.drive(DriveSignal.zero());
            }
        } finally {
            if (frozenReport != null && started && !visionTerminalRequested) {
                report.publish(ctx, "pinpoint-pod-offsets.txt", frozenReport);
            }
        }
    }

    private void clearLastResults() {
        lastDxStartBodyInches = null;
        lastDyStartBodyInches = null;
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
        if (motionInhibitedThisCycle()
                || terminalVisionFailureBlocksCalibration()
                || phase != Phase.IDLE
                || !pinpointReadyForMotion()) return;

        autoSample = auto;
        report.clear(ctx);
        clearLastResults();
        lastAttemptFailure = null;
        startAssistEndpoint = null;
        endAssistEndpoint = null;
        resetSearchAcquisition();

        if (!aprilTagAssistEnabled()) {
            startSampleInternal(auto, null);
            return;
        }
        AssistedEndpoint endpoint = matchLatestAssistedEndpoint(false);
        if (endpoint != null) {
            startSampleInternal(auto, endpoint);
            return;
        }

        // If no tag right now, optionally auto-search for one.
        if (aprilTagAssistEnabled()
                && cfg.enableAutoTagSearchAtStart
                && drive != null
                && tagEstimator != null) {
            enterPhase(Phase.SEARCH_TAG_START);
            resetSearchAcquisition();
            tagSearchUnwrapper.reset(latestPinpointPose.headingRad);
            tagSearchStartUnwrappedRad = tagSearchUnwrapper.getUnwrappedRad();
            return;
        }

        failAssistedAttempt("Assisted start unavailable: " + assistEvidenceNotice);
    }

    private void startSampleInternal(boolean auto, AssistedEndpoint endpoint) {
        if (motionInhibitedThisCycle()) return;
        // Leave any powered search and stop before rebasing the sensor. Publish the inactive
        // state first so a failing/reentrant zero cannot retain or revive that search's timer.
        enterPhase(Phase.IDLE);
        autoSample = false;
        if (drive != null) {
            drive.drive(DriveSignal.zero());
        }
        if (motionInhibitedThisCycle() || visionTerminalRequested) return;
        clearLastResults();

        startAssistEndpoint = endpoint;
        endAssistEndpoint = null;
        resetSearchAcquisition();
        lastHadTagStart = endpoint != null;
        lastHadTagEnd = false;

        // Manual/no-tag sampling retains its established software-zero workflow. Assisted
        // sampling never rebases a historical camera pose into the current Pinpoint frame.
        if (endpoint == null) {
            pinpoint.setPose(Pose2d.zero());
            resetAssistHistory();
        }
        if (motionInhibitedThisCycle() || visionTerminalRequested
                || terminalVisionFailureBlocksCalibration()) return;

        // The command-phase heading is current even when the assisted capture was delayed.
        startPinpointPose = pinpoint.getEstimate().toPose2d();
        startHeadingRad = startPinpointPose.headingRad;

        headingUnwrapper.reset(startHeadingRad);
        startHeadingUnwrappedRad = headingUnwrapper.getUnwrappedRad();

        autoSample = auto;
        enterPhase(Phase.ROTATING);
    }

    private void transitionAfterRotation() {
        boolean computeNow = !cfg.enablePostRotateRecenter
                || (autoSample && cfg.autoComputeAfterAutoSample && startAssistEndpoint != null);
        if (startAssistEndpoint != null && computeNow) {
            AssistedEndpoint endpoint = matchLatestAssistedEndpoint(true);
            if (endpoint != null) {
                endAssistEndpoint = endpoint;
                finishSampleAndCompute();
                return;
            }
            if (autoSample && cfg.enableAutoTagSearchAtEnd && drive != null && tagEstimator != null) {
                enterPhase(Phase.SEARCH_TAG_END);
                resetSearchAcquisition();
                tagSearchUnwrapper.reset(latestPinpointPose.headingRad);
                tagSearchStartUnwrappedRad = tagSearchUnwrapper.getUnwrappedRad();
                return;
            }
            failAssistedAttempt("Assisted end unavailable: " + assistEvidenceNotice);
            return;
        }
        transitionToPostRecenterOrFinish();
    }

    private void transitionToPostRecenterOrFinish() {
        if (cfg.enablePostRotateRecenter) {
            // Recenter is not a solve. The final A must acquire its own matched end, without
            // requiring a redundant end pair simply to enter this optional phase.
            endAssistEndpoint = null;
            enterPhase(Phase.POST_RECENTER);
        } else {
            finishSampleAndCompute();
            return;
        }

        if (drive != null) {
            drive.drive(DriveSignal.zero());
        }
    }

    private void finishSampleAndCompute() {
        if (!isSampleActive()) return;
        if (startAssistEndpoint != null && endAssistEndpoint == null) {
            endAssistEndpoint = matchLatestAssistedEndpoint(true);
            if (endAssistEndpoint == null) {
                failAssistedAttempt("Assisted end unavailable: " + assistEvidenceNotice);
                return;
            }
        }

        // Leave the phase before external cleanup. A failing zero must not leave a live timer.
        enterPhase(Phase.IDLE);
        autoSample = false;
        if (drive != null) {
            drive.drive(DriveSignal.zero());
        }
        if (motionInhibitedThisCycle() || visionTerminalRequested) return;

        double dx0;
        double dy0;
        double deltaHeading = headingUnwrapper.getUnwrappedRad() - startHeadingUnwrappedRad;
        if (startAssistEndpoint != null) {
            Pose2d rawStart = startAssistEndpoint.odometry.fieldToRobotPose();
            Pose2d rawEnd = endAssistEndpoint.odometry.fieldToRobotPose();
            Pose2d tagStart = startAssistEndpoint.tagEstimate.toPose2d();
            Pose2d tagEnd = endAssistEndpoint.tagEstimate.toPose2d();
            // Each independent field frame is reduced to its own robot-at-start body frame.
            // Subtract those comparable displacements once; never rotate the residual again.
            dx0 = displacementForward(rawStart, rawEnd) - displacementForward(tagStart, tagEnd);
            dy0 = displacementLeft(rawStart, rawEnd) - displacementLeft(tagStart, tagEnd);
            deltaHeading = MathUtil.wrapToPi(rawEnd.headingRad - rawStart.headingRad);
        } else {
            dx0 = displacementForward(startPinpointPose, latestPinpointPose);
            dy0 = displacementLeft(startPinpointPose, latestPinpointPose);
        }
        if (!Double.isFinite(dx0) || !Double.isFinite(dy0) || !Double.isFinite(deltaHeading)) {
            failAttempt("Sample displacement or heading is non-finite");
            return;
        }

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
            double recommendedX = cfg.pinpoint.strafePodOffsetForwardInches - xError;
            double recommendedY = cfg.pinpoint.forwardPodOffsetLeftInches - yError;
            if (!Double.isFinite(xError) || !Double.isFinite(yError)
                    || !Double.isFinite(recommendedX) || !Double.isFinite(recommendedY)) {
                failAttempt("Sample solve or recommended offsets are non-finite");
                return;
            }

            lastXErrorInches = xError;
            lastYErrorInches = yError;

            // Recommended new offsets = current_est - error
            lastRecommendedStrafePodOffsetForwardInches = recommendedX;
            lastRecommendedForwardPodOffsetLeftInches = recommendedY;

            lastSolveNote = null;
        }
        // Publish sample diagnostics only after all derived arithmetic has passed validation.
        lastDxStartBodyInches = dx0;
        lastDyStartBodyInches = dy0;
        lastDeltaHeadingRad = deltaHeading;
        lastHadTagEnd = startAssistEndpoint != null;
        report.publish(ctx, "pinpoint-pod-offsets.txt", snapshotReport(
                lastSolveNote == null ? "SOLVED: candidate, not applied" : "REJECTED: " + lastSolveNote,
                true));
    }

    /** Forward displacement expressed in the robot's body frame at this stream's start. */
    private static double displacementForward(Pose2d start, Pose2d end) {
        return (end.xInches - start.xInches) * Math.cos(start.headingRad)
                + (end.yInches - start.yInches) * Math.sin(start.headingRad);
    }

    /** Left displacement expressed in the robot's body frame at this stream's start. */
    private static double displacementLeft(Pose2d start, Pose2d end) {
        return -(end.xInches - start.xInches) * Math.sin(start.headingRad)
                + (end.yInches - start.yInches) * Math.cos(start.headingRad);
    }

    /** Reject one assisted attempt before cleanup can call back into the tester. */
    private void failAssistedAttempt(String reason) {
        failAttempt(reason + ". Only A during start-tag search explicitly selects a fresh no-tag sample");
    }

    /** Discard invalid evidence/results before publishing the terminal attempt's physical zero. */
    private void failAttempt(String reason) {
        lastAttemptFailure = reason + ". Attempt discarded; release controls and retry.";
        clearLastResults();
        abortSample(snapshotReport(lastAttemptFailure, false));
    }

    /**
     * Copy only retained immutable facts before cleanup; encoding runs after the existing zero.
     * This is a report, not the missing sequence of raw samples needed for replay.
     */
    private Supplier<String> snapshotReport(String outcome, boolean includeResult) {
        try {
            final double processingTime = ctx.clock.nowSec();
            final long cycle = ctx.clock.cycle();
            final Phase endedPhase = phase;
            final AssistedEndpoint start = startAssistEndpoint;
            final AssistedEndpoint end = endAssistEndpoint;
            final double startAge = start == null ? Double.NaN : start.tagEstimate.timestamp.ageSec(ctx.clock);
            final double endAge = end == null ? Double.NaN : end.tagEstimate.timestamp.ageSec(ctx.clock);
            final Pose2d rawStart = start == null && (includeResult || isSampleActive())
                    ? startPinpointPose : null;
            final PoseEstimate rawEstimate = pinpoint == null ? null : pinpoint.getEstimate();
            final Pose2d rawEnd = rawEstimate != null && rawEstimate.hasPose
                    ? rawEstimate.toPose2d() : null;
            final double rawAge = rawEstimate == null ? Double.NaN
                    : rawEstimate.timestamp.ageSec(ctx.clock);
            final String notice = assistEvidenceNotice;
            final String camera = selectedVisionDeviceName;
            final String backend = activeVisionDescription;
            final Double[] values = includeResult ? new Double[] {lastDxStartBodyInches,
                    lastDyStartBodyInches, lastDeltaHeadingRad, lastXErrorInches, lastYErrorInches,
                    lastRecommendedStrafePodOffsetForwardInches,
                    lastRecommendedForwardPodOffsetLeftInches} : new Double[7];
            return () -> {
                StringBuilder text = CalibrationReport.begin(processingTime, cycle, "Pinpoint pod offsets");
                CalibrationReport.pinpoint(text, cfg.pinpoint);
                text.append("Outcome: ").append(outcome).append('\n')
                        .append("Phase at report boundary: ").append(endedPhase).append('\n')
                        .append("Configured powered drive: ").append(cfg.mecanum != null).append('\n')
                        .append("targetTurnRad: ").append(cfg.targetTurnRad)
                        .append("; automaticPhaseTimeoutSec: ").append(cfg.automaticPhaseTimeoutSec).append('\n')
                        .append("manualOmegaScale: ").append(cfg.manualOmegaScale)
                        .append("; autoOmegaCmd: ").append(cfg.autoOmegaCmd)
                        .append("; recenterTranslationScale: ").append(cfg.recenterTranslationScale).append('\n')
                        .append("autoComputeAfterAutoSample: ").append(cfg.autoComputeAfterAutoSample)
                        .append("; enablePostRotateRecenter: ").append(cfg.enablePostRotateRecenter).append('\n')
                        .append("Actual hand motion/recenter and no-assist acquisition times: UNRECORDED.\n")
                        .append("Unaccounted real translation contaminates a no-assist solve; software zero is not motion.\n")
                        .append("Camera selection: ").append(camera == null ? "UNAVAILABLE" : camera).append('\n')
                        .append("Backend description: ").append(backend == null ? "UNAVAILABLE" : backend).append('\n')
                        .append("Layout policy description: ").append(fixedTagLayoutPolicySummary == null
                                ? "UNRECORDED" : fixedTagLayoutPolicySummary).append('\n')
                        .append("Assisted evidence notice: ").append(notice == null ? "none" : notice).append('\n');
                if (cfg.aprilTags != null) {
                    text.append("maxDetectionAgeSec: ").append(cfg.aprilTags.maxDetectionAgeSec).append('\n')
                            .append("Start/end search enabled: ").append(cfg.enableAutoTagSearchAtStart)
                            .append(" / ").append(cfg.enableAutoTagSearchAtEnd)
                            .append("; distinct search frames: ").append(cfg.tagSearchStableFrames).append('\n')
                            .append("Start/end search limits rad: ").append(cfg.tagSearchMaxTurnRad)
                            .append(" / ").append(cfg.tagEndSearchMaxExtraTurnRad)
                            .append("; search omega command: ").append(cfg.tagSearchOmegaCmd).append('\n');
                }
                CalibrationReport.pose(text, "Raw start fieldToRobotPose (no-assist start)", rawStart);
                CalibrationReport.pose(text, "Latest raw fieldToRobotPose (not a tag-aligned substitute)", rawEnd);
                text.append("Latest raw estimate age at report sec: ")
                        .append(Double.isFinite(rawAge) ? Double.toString(rawAge) : "UNAVAILABLE").append('\n');
                appendEndpointReport(text, "Start", start, startAge);
                appendEndpointReport(text, "End", end, endAge);
                String[] labels = {"Start-body residual dx in", "Start-body residual dy in",
                        "Solve delta heading rad", "Strafe offset error in", "Forward offset error in"};
                for (int i = 0; i < labels.length; i++) text.append(labels[i]).append(": ")
                        .append(values[i] == null ? "UNAVAILABLE" : values[i]).append('\n');
                if (values[5] != null && values[6] != null) {
                    text.append("Absolute replacement offsets, NOT increments (candidate only):\n")
                            .append("cfg.pinpoint.strafePodOffsetForwardInches = ").append(values[5]).append(";\n")
                            .append("cfg.pinpoint.forwardPodOffsetLeftInches = ").append(values[6]).append(";\n");
                } else text.append("Candidate offsets: UNAVAILABLE; no prior candidate carried forward.\n");
                text.append("Raw sample sequence, tag IDs used by estimator, full camera/drive/solver setup,\n")
                        .append("intrinsics and hardware readback: UNRECORDED; retain source configuration externally.\n");
                return text.toString();
            };
        } catch (RuntimeException unavailable) {
            return null;
        }
    }

    /** Format existing capture-aligned endpoints and lookup provenance without polling or solving. */
    private static void appendEndpointReport(StringBuilder text, String label,
                                             AssistedEndpoint endpoint, double ageSec) {
        if (endpoint == null) {
            text.append(label).append(" assisted endpoint: UNAVAILABLE\n");
            return;
        }
        text.append(label).append(" capture age at report sec: ")
                .append(Double.isFinite(ageSec) ? Double.toString(ageSec) : "UNAVAILABLE").append('\n')
                .append(label).append(" history lookup: ").append(endpoint.odometry.kind())
                .append("; quality: ").append(endpoint.odometry.quality())
                .append("; history generation: ").append(endpoint.historyGeneration)
                .append("; raw trajectory segment: ").append(endpoint.rawTrajectorySegment).append('\n')
                .append(label).append(" tag estimate quality: ").append(endpoint.tagEstimate.quality).append('\n');
        CalibrationReport.pose(text, label + " tag fieldToRobotPose at capture", endpoint.tagEstimate.toPose2d());
        CalibrationReport.pose(text, label + " raw fieldToRobotPose matched to capture", endpoint.odometry.fieldToRobotPose());
    }

    private void resetSearchAcquisition() {
        tagStableFrames = 0;
        tagSearchConsumedTimestamp = LoopTimestamp.unavailable();
    }

    /** Retain the consumed watermark across loss, while only new eligible frames grow a streak. */
    private boolean countSearchFrame(AssistedEndpoint endpoint) {
        if (endpoint == null) {
            tagStableFrames = 0;
            return false;
        }
        LoopTimestamp timestamp = endpoint.tagEstimate.timestamp;
        if (Double.isFinite(tagSearchConsumedTimestamp.ageSec(ctx.clock))) {
            double advance = timestamp.secondsSince(tagSearchConsumedTimestamp);
            if (!Double.isFinite(advance) || advance < 0.0) {
                tagStableFrames = 0;
                assistEvidenceNotice = "Capture timestamp is older than the consumed search frame";
                return false;
            }
            if (advance == 0.0) {
                assistEvidenceNotice = "Repeated capture frame; waiting for a newer eligible frame";
                return false;
            }
        }
        tagSearchConsumedTimestamp = timestamp;
        tagStableFrames++;
        return tagStableFrames >= cfg.tagSearchStableFrames;
    }

    /** Match one complete camera observation without substituting delivery-time odometry. */
    private AssistedEndpoint matchLatestAssistedEndpoint(boolean end) {
        PoseEstimate tag = latestTagEstimate;
        if (!aprilTagAssistEnabled() || visionLane == null || tagEstimator == null
                || assistOdometryHistory == null || !visionReadiness.isReady()) {
            assistEvidenceNotice = "Selected AprilTag source is not ready";
            return null;
        }
        if (tag == null || !tag.hasPose || tag.timestamp == null
                || !finitePose(tag.toPose2d()) || !Double.isFinite(tag.quality)
                || tag.quality < 0.0 || tag.quality > 1.0) {
            assistEvidenceNotice = "No finite fixed-tag pose with capture evidence";
            return null;
        }
        if (!tag.timestamp.isFresh(ctx.clock, cfg.aprilTags.maxDetectionAgeSec)) {
            assistEvidenceNotice = "Tag capture is stale, future, unavailable, or from another clock epoch";
            return null;
        }
        if (end && (startAssistEndpoint == null || !assistedStartStillCompatible()
                || !(tag.timestamp.secondsSince(startAssistEndpoint.tagEstimate.timestamp) > 0.0))) {
            assistEvidenceNotice = "End capture must follow the start in the same camera/clock/raw trajectory";
            return null;
        }
        PlanarPoseHistory.Lookup lookup = assistOdometryHistory.lookupSource().getAt(ctx.clock, tag.timestamp);
        if (!lookup.isAvailable()) {
            assistEvidenceNotice = "Capture-time odometry unavailable: " + lookup.unavailableReason();
            return null;
        }
        if (!finitePose(lookup.fieldToRobotPose())) {
            assistEvidenceNotice = "Capture-time odometry is non-finite";
            return null;
        }
        assistEvidenceNotice = "Capture matched: " + lookup.kind();
        return new AssistedEndpoint(tag, lookup, visionLane, tagEstimator,
                pinpoint.trajectorySegmentId(), assistHistoryGeneration);
    }

    private static boolean finitePose(Pose2d pose) {
        return pose != null && Double.isFinite(pose.xInches) && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.headingRad);
    }

    /** A frozen start need not remain in the rolling history, but its coordinate identity must. */
    private boolean assistedStartStillCompatible() {
        return startAssistEndpoint != null
                && startAssistEndpoint.cameraOwner == visionLane
                && startAssistEndpoint.estimator == tagEstimator
                && startAssistEndpoint.rawTrajectorySegment == pinpoint.trajectorySegmentId()
                && startAssistEndpoint.historyGeneration == assistHistoryGeneration
                && Double.isFinite(startAssistEndpoint.tagEstimate.timestamp.ageSec(ctx.clock));
    }

    private void clearAssistEvidence() {
        startAssistEndpoint = null;
        endAssistEndpoint = null;
        latestTagEstimate = null;
        assistEvidenceNotice = null;
        resetSearchAcquisition();
        resetAssistHistory();
    }

    private void resetAssistHistory() {
        previousAssistOdometryEstimate = null;
        assistHistoryGeneration++;
        if (assistOdometryHistory != null) assistOdometryHistory.reset();
    }

    private void ensureAprilTagAssistReady(boolean initPhase) {
        if (!aprilTagAssistEnabled()) return;
        if (visionLane != null) return;
        if (visionCleanupFailed) return;
        if (visionTerminalRequested) return;
        if (visionRetryBlocked) return;
        if (selectedVisionDeviceName == null) return;

        if (pendingVisionFactory == null) {
            try {
                pendingVisionFactory = applyVisionFactoryBuilder(selectedVisionDeviceName);
            } catch (RuntimeException failure) {
                // Builder application is contractually effect-free. Its own suppressed failures do
                // not imply an opened vision owner or uncertain camera rollback.
                recordVisionFailure(
                        failure,
                        "Vision factory capture failed",
                        initPhase,
                        false
                );
                return;
            }
        }

        AprilTagCameraFactory factory = pendingVisionFactory;
        pendingVisionFactory = null;
        try {
            OwnedAprilTagCamera openedLane = factory.open(ctx.hw);
            if (openedLane == null) {
                throw new IllegalStateException(
                        "vision lane factory returned null for device '"
                                + selectedVisionDeviceName + "'");
            }

            // Publish immediately so any later accessor/readiness failure has one closeable owner.
            visionLane = openedLane;
            CameraMountConfig cameraMount = Objects.requireNonNull(
                    visionLane.aprilTags().cameraMountConfig(),
                    "vision lane cameraMountConfig() must not return null"
            );
            if (isLikelyIdentity(cameraMount)) {
                disableIdentityMountAssist();
                return;
            }

            tagSensor = Objects.requireNonNull(
                    visionLane.aprilTags().tagSensor(),
                    "vision lane tagSensor() must not return null"
            );
            activeVisionDescription = factory.description();
            VisionReadiness initialReadiness = visionLane.aprilTags().readiness(ctx.clock);
            if (initialReadiness == null) {
                throw new IllegalStateException("vision lane returned a null readiness result");
            }

            AprilTagPoseEstimator.Config estCfg =
                    cfg.aprilTags.toAprilTagPoseEstimatorConfig(cameraMount);
            tagEstimator = new AprilTagPoseEstimator(tagSensor, layout, estCfg);
            aprilTagAssistNotice = null;
            visionFailure = null;
            visionReadiness = initialReadiness;
        } catch (RuntimeException failure) {
            handleVisionFailure(failure, "Vision setup failed", initPhase);
        }
    }

    private boolean aprilTagAssistEnabled() {
        return visionLaneFactoryBuilder != null && !aprilTagAssistUnavailable;
    }

    private boolean terminalVisionFailureBlocksCalibration() {
        return visionLaneFactoryBuilder != null
                && !aprilTagAssistUnavailable
                && (visionRetryBlocked || visionCleanupFailed);
    }

    private boolean calibrationControlsEligible() {
        return !terminalVisionFailureBlocksCalibration()
                && (visionPicker == null
                || selectedVisionDeviceName != null
                || aprilTagAssistUnavailable);
    }

    private void disableIdentityMountAssist() {
        clearAssistEvidence();
        OwnedAprilTagCamera identityLane = visionLane;
        visionLane = null;
        tagSensor = null;
        tagEstimator = null;
        activeVisionDescription = null;
        pendingVisionFactory = null;
        selectedVisionDeviceName = null;

        try {
            identityLane.close();
        } catch (RuntimeException cleanupFailure) {
            visionCleanupFailed = true;
            visionRetryBlocked = true;
            visionFailure = cleanupFailure;
            aprilTagAssistNotice = "AprilTag assist camera mount looks uncalibrated, but vision "
                    + "cleanup also failed: " + failureSummary(cleanupFailure)
                    + "; stop and restart this OpMode";
            visionReadiness = VisionReadiness.notReady(aprilTagAssistNotice);
            return;
        }

        aprilTagAssistUnavailable = true;
        visionRetryBlocked = true;
        visionFailure = null;
        aprilTagAssistNotice = "Disabled: camera mount looks uncalibrated; run Calib: Camera Mount";
        visionReadiness = VisionReadiness.notReady(aprilTagAssistNotice);
    }

    /** Samples dynamic camera readiness while leaving the non-vision calibration path available. */
    private void refreshAprilTagVisionReadiness(boolean initPhase) {
        if (visionLane == null) {
            visionReadiness = VisionReadiness.notReady(
                    aprilTagAssistNotice != null
                            ? aprilTagAssistNotice
                            : "No vision device is open");
            return;
        }
        try {
            VisionReadiness current = visionLane.aprilTags().readiness(ctx.clock);
            if (current == null) {
                throw new IllegalStateException("vision lane returned a null readiness result");
            }
            visionReadiness = current;
        } catch (RuntimeException failure) {
            handleVisionFailure(failure, "Vision readiness failed", initPhase);
        }
    }

    private void handleVisionFailure(
            RuntimeException failure,
            String context,
            boolean initPhase
    ) {
        recordVisionFailure(failure, context, initPhase, true);
    }

    private void recordVisionFailure(
            RuntimeException failure,
            String context,
            boolean initPhase,
            boolean suppressedFailureMeansUncertainRollback
    ) {
        Supplier<String> failedReport = !initPhase && phase != Phase.IDLE
                ? snapshotReport("FAILED: " + context + ": " + primaryFailureSummary(failure), false)
                : null;
        clearAssistEvidence();
        OwnedAprilTagCamera failedLane = visionLane;
        boolean uncertainFactoryRollback = failedLane == null
                && suppressedFailureMeansUncertainRollback
                && failure.getSuppressed().length > 0;
        tagSensor = null;
        tagEstimator = null;
        activeVisionDescription = null;
        pendingVisionFactory = null;
        selectedVisionDeviceName = null;

        if (!initPhase) {
            // Publish the terminal gate before the safety zero. If that callback throws Error,
            // leave the already-published lane reachable so a later STOP can still close it.
            visionRetryBlocked = true;
            try {
                // An active assist failure is terminal for this tester activation. Do not let a
                // previously armed calibration phase continue moving without its selected evidence.
                abortSample(null);
            } catch (RuntimeException driveZeroFailure) {
                if (driveZeroFailure != failure) {
                    failure.addSuppressed(driveZeroFailure);
                }
            }
        }

        if (failedLane != null && visionLane == failedLane) {
            // Detach immediately before close. A reentrant STOP may already have detached and
            // closed this lane while the drive zero callback was running.
            visionLane = null;
            try {
                failedLane.close();
            } catch (RuntimeException cleanupFailure) {
                visionCleanupFailed = true;
                if (cleanupFailure != failure) {
                    failure.addSuppressed(cleanupFailure);
                }
            }
        } else if (uncertainFactoryRollback) {
            // A framework factory can fail before publishing its lane and attach a failed
            // rollback/close attempt. There is no safe owner reference to close again.
            visionCleanupFailed = true;
        }

        boolean canRetryInPicker = !visionCleanupFailed
                && !visionTerminalRequested
                && initPhase
                && visionPicker != null;
        if (canRetryInPicker) {
            try {
                resetVisionPickerChoice();
            } catch (RuntimeException pickerFailure) {
                visionCleanupFailed = true;
                if (pickerFailure != failure) {
                    failure.addSuppressed(pickerFailure);
                }
            }
        }

        if (visionCleanupFailed || !canRetryInPicker) {
            visionRetryBlocked = true;
        }

        String recovery = visionCleanupFailed
                ? "vision cleanup failed; stop and restart this OpMode before selecting another device"
                : canRetryInPicker
                ? "select the vision device again to create a fresh owner"
                : "press BACK and reopen this tester to create a fresh owner";
        visionFailure = failure;
        aprilTagAssistNotice = context + ": " + failureSummary(failure) + "; " + recovery;
        visionReadiness = VisionReadiness.notReady(aprilTagAssistNotice);
        if (failedReport != null && started && !visionTerminalRequested) {
            report.publish(ctx, "pinpoint-pod-offsets.txt", failedReport);
        }
    }

    private static String failureSummary(RuntimeException failure) {
        String summary = primaryFailureSummary(failure);
        Throwable[] suppressed = failure.getSuppressed();
        if (suppressed.length == 0) {
            return summary;
        }
        Throwable cleanup = suppressed[0];
        String cleanupMessage = cleanup.getMessage();
        return summary + "; cleanup also failed: " + cleanup.getClass().getSimpleName()
                + ((cleanupMessage == null || cleanupMessage.trim().isEmpty())
                ? ""
                : ": " + cleanupMessage.trim());
    }

    private static String primaryFailureSummary(RuntimeException failure) {
        String message = failure.getMessage();
        return failure.getClass().getSimpleName()
                + ((message == null || message.trim().isEmpty()) ? "" : ": " + message.trim());
    }

    private void resetVisionPickerChoice() {
        if (visionPicker == null) {
            return;
        }
        visionPicker.clearChoice();
        visionPicker.refresh();
    }

    private void updateSensors(boolean initPhase) {
        pinpoint.update(ctx.clock);
        latestPinpointPose = pinpoint.getEstimate().toPose2d();
        recordAssistOdometry();
        if (motionInhibitedThisCycle() || visionTerminalRequested) return;

        refreshAprilTagVisionReadiness(initPhase);
        if (visionReadiness.isReady() && tagSensor != null && tagEstimator != null) {
            try {
                tagEstimator.update(ctx.clock);
                if (!motionInhibitedThisCycle() && !visionTerminalRequested && tagEstimator != null) {
                    latestTagEstimate = tagEstimator.getEstimate();
                }
            } catch (RuntimeException failure) {
                handleVisionFailure(failure, "Vision observation failed", initPhase);
            }
        } else {
            latestTagEstimate = null;
        }
    }

    /** Record only the already-polled raw owner, rejecting discontinuities before queued actions. */
    private void recordAssistOdometry() {
        if (assistOdometryHistory == null || !aprilTagAssistEnabled()) return;
        PoseEstimate current = pinpoint.getEstimate();
        long segment = pinpoint.trajectorySegmentId();
        PoseEstimate previous = previousAssistOdometryEstimate;
        boolean discontinuity = previous != null && segment != assistHistorySourceSegment;
        if (previous != null && previous.hasPose && current.hasPose) {
            double elapsed = current.timestamp.secondsSince(previous.timestamp);
            boolean sameTimeChangedPose = elapsed == 0.0
                    && (current.fieldToRobotPose.xInches != previous.fieldToRobotPose.xInches
                    || current.fieldToRobotPose.yInches != previous.fieldToRobotPose.yInches
                    || current.fieldToRobotPose.yawRad != previous.fieldToRobotPose.yawRad
                    || current.quality != previous.quality);
            discontinuity |= !Double.isFinite(elapsed) || elapsed < 0.0 || sameTimeChangedPose;
        }
        boolean unavailable = !pinpointReadyForMotion();
        if ((discontinuity || unavailable) && isSampleActive() && startAssistEndpoint != null) {
            failAssistedAttempt(unavailable ? "Pinpoint READY evidence was lost during the assisted attempt"
                    : "Raw odometry continuity changed during the assisted attempt");
            return;
        }
        if (discontinuity || unavailable) {
            resetAssistHistory();
            startAssistEndpoint = null;
            endAssistEndpoint = null;
            // A search cannot carry its good streak across incompatible odometry evidence.
            tagStableFrames = 0;
        }
        assistOdometryHistory.recordCurrent(ctx.clock);
        previousAssistOdometryEstimate = current;
        assistHistorySourceSegment = segment;
    }

    private boolean pinpointReadyForMotion() {
        if (pinpoint == null) {
            return false;
        }
        PinpointKinematicSnapshot snapshot = pinpoint.getKinematicSnapshot();
        return snapshot != null && hasCurrentReadyKinematics(
                pinpoint.lastDeviceStatus(),
                snapshot.hasPose,
                snapshot.hasVelocity,
                snapshot.cycle,
                ctx.clock.cycle()
        );
    }

    /** Package-private pure readiness seam for every motion-producing calibration path. */
    static boolean hasCurrentReadyKinematics(GoBildaPinpointDriver.DeviceStatus status,
                                             boolean hasPose,
                                             boolean hasVelocity,
                                             long snapshotCycle,
                                             long currentCycle) {
        return status == GoBildaPinpointDriver.DeviceStatus.READY
                && hasPose
                && hasVelocity
                && snapshotCycle == currentCycle;
    }

    private void renderTelemetry(boolean initPhase) {
        ctx.telemetry.clearAll();
        ctx.telemetry.addLine("=== " + name() + " ===");

        // Quick status + primary controls.
        ctx.telemetry.addData("Phase", phase);
        ctx.telemetry.addData(
                "Pinpoint status",
                pinpoint != null ? pinpoint.lastDeviceStatus() : "NOT_CREATED"
        );
        if (!pinpointReadyForMotion()) {
            ctx.telemetry.addLine(
                    "Keep the robot stationary; motion stays disabled until Pinpoint READY "
                            + "publishes current pose and velocity."
            );
        }
        boolean terminalVisionBlock = terminalVisionFailureBlocksCalibration();
        ctx.telemetry.addData(
                "Manual sample [A]",
                terminalVisionBlock
                        ? "unavailable (press BACK and reopen)"
                        : phase == Phase.IDLE
                        ? "start / stop"
                        : (autoSample ? "not active" : "press A to finish / skip")
        );
        ctx.telemetry.addData(
                "Auto sample [Y]",
                terminalVisionBlock
                        ? "unavailable (press BACK and reopen)"
                        : drive != null
                        ? (autoSample ? "ACTIVE" : "start configured turn")
                        : "unavailable (no drive)"
        );
        ctx.telemetry.addData("Abort [B]", phase != Phase.IDLE ? "cancel current attempt" : "idle");
        ctx.telemetry.addData("Reset [X]", "zero pose + clear results");
        if (isAutomaticPhase()) {
            ctx.telemetry.addData("Automatic phase elapsed / limit [s]", String.format(
                    Locale.US, "%.2f / %.2f", automaticPhaseStartedAt.ageSec(ctx.clock),
                    cfg.automaticPhaseTimeoutSec));
        }
        if (lastAttemptFailure != null) {
            ctx.telemetry.addData("Attempt failed", lastAttemptFailure);
        }
        if (drive != null) {
            ctx.telemetry.addData("Rotate [RightStickX]", phase == Phase.ROTATING && !autoSample ? "manual turn now" : "available in manual sample");
        }

        if (phase == Phase.SEARCH_TAG_START || phase == Phase.SEARCH_TAG_END) {
            double turned = Math.abs(tagSearchUnwrapper.getUnwrappedRad() - tagSearchStartUnwrappedRad);
            ctx.telemetry.addData("Tag search turned [deg]", String.format(Locale.US, "%.1f", Math.toDegrees(turned)));
            ctx.telemetry.addData("Tag search distinct captures", String.format(Locale.US, "%d/%d", tagStableFrames, cfg.tagSearchStableFrames));
        }

        if (aprilTagAssistEnabled()) {
            String device = (selectedVisionDeviceName != null) ? selectedVisionDeviceName : "<select vision device>";
            ctx.telemetry.addData("Tag assist", true);
            ctx.telemetry.addData("Vision device", device);
            if (activeVisionDescription != null && !activeVisionDescription.isEmpty()) {
                ctx.telemetry.addData("Vision backend", activeVisionDescription);
            }
            ctx.telemetry.addData(
                    "Vision readiness",
                    visionReadiness.isReady() ? "READY" : "WAITING"
            );
            ctx.telemetry.addData("Vision status", visionReadiness.reason());
            ctx.telemetry.addData("Tag pose at capture", latestTagEstimate != null && latestTagEstimate.hasPose
                    ? latestTagEstimate.toPose2d().toString() : "<none>");
            if (assistEvidenceNotice != null) ctx.telemetry.addData("Capture evidence", assistEvidenceNotice);
            if (startAssistEndpoint != null) {
                ctx.telemetry.addData("Start capture odometry", startAssistEndpoint.odometry.kind());
            }
            if (fixedTagLayoutPolicySummary != null) {
                ctx.telemetry.addData("layout.policy", fixedTagLayoutPolicySummary);
            }
            FtcTagLayoutDebug.dumpSummary(layout, new FtcTelemetryDebugSink(ctx.telemetry), "layout");
            if (initPhase && visionPicker != null && selectedVisionDeviceName == null
                    && !visionCleanupFailed) {
                ctx.telemetry.addLine();
                visionPicker.render(ctx.telemetry);
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
        if (isSampleActive()) {
            ctx.telemetry.addData("Current turn progress [deg]", String.format(Locale.US, "%.1f",
                    Math.toDegrees(headingUnwrapper.getUnwrappedRad() - startHeadingUnwrappedRad)));
        }

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
                            "forwardPodOffsetLeftInches = %.3f; strafePodOffsetForwardInches = %.3f;",
                            lastRecommendedForwardPodOffsetLeftInches,
                            lastRecommendedStrafePodOffsetForwardInches
                    )
            );
        }

        ctx.telemetry.addLine();

        // Instructions (dynamic by phase)
        ctx.telemetry.addLine();
        if (initPhase) {
            ctx.telemetry.addLine(
                    "IMPORTANT: Start the OpMode to enable motor movement "
                            + "(auto sample / stick rotate).");
        }
        switch (phase) {
            case IDLE:
                if (terminalVisionBlock) {
                    ctx.telemetry.addLine(
                            "Vision ownership failed; press BACK and reopen before calibrating."
                    );
                    break;
                }
                ctx.telemetry.addLine("Manual sample [A]: rotate in place, then press A again to compute.");
                ctx.telemetry.addLine("Auto sample [Y]: rotate automatically to the target heading when a drive config exists.");
                if (drive == null) {
                    ctx.telemetry.addLine("(No drive configured) Manual samples are by hand only.");
                }
                if (aprilTagAssistEnabled()) {
                    ctx.telemetry.addLine("Assist requires a tag capture matched to raw odometry; Pinpoint is not rebased.");
                }
                break;

            case SEARCH_TAG_START:
                ctx.telemetry.addLine("Searching for a known-pose tag before starting the sample...");
                ctx.telemetry.addLine("Skip assist [A] | Abort [B]");
                break;

            case ROTATING:
                if (autoSample) {
                    if (startAssistEndpoint != null && cfg.autoComputeAfterAutoSample) {
                        ctx.telemetry.addLine("Auto sample [Y] is rotating now and will auto-compute at the end.");
                    } else {
                        ctx.telemetry.addLine("Auto sample [Y] is rotating now. Abort [B] if needed.");
                    }
                } else {
                    if (drive != null) {
                        ctx.telemetry.addLine("Manual rotate [RightStickX], then finish [A].");
                    } else {
                        ctx.telemetry.addLine("Rotate the robot by hand, then finish [A].");
                    }
                }
                if (autoSample && startAssistEndpoint != null && cfg.autoComputeAfterAutoSample) {
                    ctx.telemetry.addLine("Tip: with tag assist, results can compute automatically after the turn.");
                } else if (cfg.enablePostRotateRecenter) {
                    ctx.telemetry.addLine("After rotation you can recenter, then finish [A] to compute.");
                } else {
                    ctx.telemetry.addLine("Finish [A] computes immediately when rotation is complete.");
                }
                break;

            case SEARCH_TAG_END:
                ctx.telemetry.addLine("Trying to reacquire a tag pose for the end of the sample...");
                ctx.telemetry.addLine("Discard assisted attempt [A] | Abort [B]");
                break;

            case POST_RECENTER:
                ctx.telemetry.addLine("Recenter (optional): translate back to the starting spot, then finish [A].");
                if (startAssistEndpoint != null) {
                    ctx.telemetry.addLine("Final A requires an eligible capture-matched tag end; missing evidence discards this assisted attempt.");
                }
                if (drive != null) {
                    ctx.telemetry.addLine("Translate [LeftStick] with no rotation.");
                } else {
                    ctx.telemetry.addLine("(No drive configured) You can physically reposition the robot.");
                }
                if (startAssistEndpoint != null && latestTagEstimate != null && latestTagEstimate.hasPose) {
                    Pose2d startTagPose = startAssistEndpoint.tagEstimate.toPose2d();
                    Pose2d latestTagPose = latestTagEstimate.toPose2d();
                    double dxToStart = startTagPose.xInches - latestTagPose.xInches;
                    double dyToStart = startTagPose.yInches - latestTagPose.yInches;
                    ctx.telemetry.addData(
                            "To start (tag capture field) [in]",
                            String.format(Locale.US, "Δx=%.2f, Δy=%.2f", dxToStart, dyToStart)
                    );
                }
                break;
        }

        // Results
        if (lastDxStartBodyInches != null && lastDyStartBodyInches != null && lastDeltaHeadingRad != null) {
            ctx.telemetry.addLine();
            ctx.telemetry.addLine("Last sample:");
            ctx.telemetry.addData(
                    "  Δx, Δy (start-body residual) [in]",
                    String.format(Locale.US, "%.3f, %.3f", lastDxStartBodyInches, lastDyStartBodyInches)
            );
            ctx.telemetry.addData(
                    lastHadTagStart ? "  Capture-to-capture Δheading [deg]" : "  Δheading [deg]",
                    String.format(Locale.US, "%.1f", Math.toDegrees(lastDeltaHeadingRad))
            );

            if (aprilTagAssistEnabled()) {
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
                ctx.telemetry.addLine("Copy these into your PinpointOdometryPredictor.Config / RobotConfig.");
            }
        }
        report.render(ctx);
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
        visionTerminalRequested = true;
        visionRetryBlocked = true;
        started = false;
        clearPendingMotionIntent();
        MecanumDrivebase ownedDrive = drive;
        OwnedAprilTagCamera ownedVision = visionLane;
        drive = null;
        pendingVisionFactory = null;
        visionReadiness = VisionReadiness.notReady("Vision tester is stopping");
        tagSensor = null;
        tagEstimator = null;
        activeVisionDescription = null;
        selectedVisionDeviceName = null;
        try {
            CleanupActions.attemptAll(
                () -> {
                    if (ownedDrive != null) {
                        ownedDrive.stop();
                    }
                },
                () -> {
                    if (ownedVision == null || visionLane != ownedVision) {
                        return;
                    }
                    // Detach immediately before close so a reentrant STOP cannot close twice. If
                    // an earlier drive cleanup throws Error, this action is never reached and the
                    // still-published lane remains available to a later STOP attempt.
                    visionLane = null;
                    try {
                        ownedVision.close();
                    } catch (RuntimeException cleanupFailure) {
                        visionCleanupFailed = true;
                        visionFailure = cleanupFailure;
                        throw cleanupFailure;
                    }
                }
            );
        } finally {
            report.clear(ctx);
        }
    }

    /**
     * Tracks an unwrapped angle by accumulating wrapped deltas.
     */
    private static final class AngleUnwrapper {
        private boolean initialized = false;
        private double prevRad = 0.0;
        private double unwrappedRad = 0.0;

        /**
         * Resets the unwrapped heading tracker to a new starting angle.
         */
        public void reset(double initialRad) {
            initialized = true;
            prevRad = initialRad;
            unwrappedRad = initialRad;
        }

        /**
         * Incorporates one wrapped heading sample into the unwrapped heading tracker.
         */
        public void update(double currentRad) {
            if (!initialized) {
                reset(currentRad);
                return;
            }
            double d = MathUtil.wrapToPi(currentRad - prevRad);
            unwrappedRad += d;
            prevRad = currentRad;
        }

        /**
         * Returns the current unwrapped heading in radians.
         */
        public double getUnwrappedRad() {
            return unwrappedRad;
        }
    }
}
