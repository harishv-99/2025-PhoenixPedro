package edu.ftcphoenix.fw.ftc.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.localization.MotionDelta;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseResetter;

/**
 * {@link MotionPredictor} wrapper around the goBILDA Pinpoint Odometry Computer.
 *
 * <p>This class is intentionally FTC-hardware-facing (it depends on the FTC SDK). Phoenix robot
 * logic should depend on {@link MotionPredictor} / {@link PoseEstimate} instead of FTC SDK classes.
 * It formalizes Pinpoint's actual role in the localization stack:</p>
 * <ul>
 *   <li>Pinpoint produces a smooth absolute odometry pose.</li>
 *   <li>Pinpoint also produces the incremental motion that global localizers want to replay between
 *       absolute corrections.</li>
 *   <li>{@link PinpointKinematicSnapshot} preserves the same physical poll's velocity and unwrapped
 *       heading for passive integration consumers without giving them another hardware update path.</li>
 * </ul>
 *
 * <h2>Units</h2>
 * <ul>
 *   <li>All positions are expressed in <b>inches</b>.</li>
 *   <li>Translational velocity is expressed in <b>inches per second</b>.</li>
 *   <li>Heading is expressed in <b>radians</b>, wrapped to (-pi, pi].</li>
 *   <li>Angular velocity is expressed in <b>radians per second</b>.</li>
 * </ul>
 *
 * <h2>Field-pose convention</h2>
 * <p>The output pose and translational velocity use Phoenix's current FTC season field frame:</p>
 * <ul>
 *   <li>field X/Y meanings come from the current FTC field coordinate convention, not the robot
 *       frame;</li>
 *   <li>heading/yaw is measured about field +Z and is CCW-positive;</li>
 *   <li>the pod-offset names below use robot-relative forward/left directions only for physical
 *       sensor placement.</li>
 * </ul>
 *
 * <h2>Pinpoint offsets (naming matters)</h2>
 * <p>The underlying Pinpoint driver is configured via
 * {@link GoBildaPinpointDriver#setOffsets(double, double, DistanceUnit)} using two offsets. The
 * parameter names in the goBILDA API ({@code xOffset}, {@code yOffset}) refer to the pod axes,
 * which can be easy to confuse with Phoenix's coordinate axes.</p>
 *
 * <p>To reduce mixups, Phoenix uses <b>directional names</b> in {@link Config}:</p>
 * <ul>
 *   <li>{@link Config#forwardPodOffsetLeftInches}: how far left (+) / right (-) the forward pod is</li>
 *   <li>{@link Config#strafePodOffsetForwardInches}: how far forward (+) / back (-) the strafe pod is</li>
 * </ul>
 */
public final class PinpointOdometryPredictor implements MotionPredictor, PoseResetter {

    /**
     * Phoenix standard unit for field poses.
     */
    private static final DistanceUnit CONFIG_DISTANCE_UNIT = DistanceUnit.INCH;

    /**
     * One unambiguous Pinpoint encoder-resolution choice.
     *
     * <p>Use {@link #forGoBildaPod(GoBildaPinpointDriver.GoBildaOdometryPods)} for a vendor
     * preset or {@link #ticksPerInch(double)} for a custom pod. The value is immutable, so a
     * captured {@link Config} may retain it safely.</p>
     */
    public static final class EncoderResolution {
        private final GoBildaPinpointDriver.GoBildaOdometryPods goBildaPod;
        private final double customTicksPerInch;

        private EncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods goBildaPod,
                                  double customTicksPerInch) {
            this.goBildaPod = goBildaPod;
            this.customTicksPerInch = customTicksPerInch;
        }

        /** Return a choice backed by one goBILDA pod preset. */
        public static EncoderResolution forGoBildaPod(
                GoBildaPinpointDriver.GoBildaOdometryPods pod) {
            return new EncoderResolution(
                    Objects.requireNonNull(pod, "pod must not be null"),
                    Double.NaN
            );
        }

        /**
         * Return a custom encoder-resolution choice.
         *
         * @param value finite positive encoder ticks per inch
         * @throws IllegalArgumentException when {@code value} is not finite and positive
         */
        public static EncoderResolution ticksPerInch(double value) {
            if (!Double.isFinite(value) || value <= 0.0) {
                throw new IllegalArgumentException(
                        "Pinpoint encoder resolution ticksPerInch must be finite and > 0, got "
                                + value
                );
            }
            return new EncoderResolution(null, value);
        }

        /**
         * Dispatch this choice to exactly one supplied receiver.
         *
         * <p>Both receivers are required even though only the selected variant is invoked. This
         * keeps FTC and integration translators exhaustive without exposing representation
         * getters or requiring type inspection.</p>
         */
        public void applyTo(
                Consumer<GoBildaPinpointDriver.GoBildaOdometryPods> podReceiver,
                DoubleConsumer customReceiver) {
            Consumer<GoBildaPinpointDriver.GoBildaOdometryPods> requiredPodReceiver =
                    Objects.requireNonNull(podReceiver, "podReceiver must not be null");
            DoubleConsumer requiredCustomReceiver =
                    Objects.requireNonNull(customReceiver, "customReceiver must not be null");
            if (goBildaPod != null) {
                requiredPodReceiver.accept(goBildaPod);
            } else {
                requiredCustomReceiver.accept(customTicksPerInch);
            }
        }

        /** Validate the exact float representation written by the FTC driver's custom path. */
        private void requireVendorRepresentable(String fieldName) {
            if (goBildaPod != null) {
                return;
            }
            double ticksPerMillimetre = 1.0
                    / CONFIG_DISTANCE_UNIT.toMm(1.0 / customTicksPerInch);
            float vendorValue = (float) ticksPerMillimetre;
            if (!Float.isFinite(vendorValue) || vendorValue <= 0.0f) {
                throw new IllegalArgumentException(
                        fieldName + " custom ticksPerInch must translate to a finite positive "
                                + "Pinpoint float, got " + customTicksPerInch
                );
            }
        }

        @Override
        public String toString() {
            if (goBildaPod != null) {
                return "EncoderResolution{goBildaPod=" + goBildaPod + '}';
            }
            return "EncoderResolution{ticksPerInch=" + customTicksPerInch + '}';
        }
    }

    /** Mutable, data-only authoring configuration for {@link PinpointOdometryPredictor}. */
    public static final class Config {

        /**
         * Exact FTC hardware configuration name of the Pinpoint device (commonly {@code "odo"}).
         * Whitespace-only names are invalid; otherwise active capture preserves the authored name
         * exactly for hardware lookup.
         */
        public String hardwareMapName = "odo";

        /** Left/right offset of the forward (X) pod; +left, -right, in inches. */
        public double forwardPodOffsetLeftInches = 0.0;

        /** Forward/back offset of the strafe (Y) pod; +forward, -back, in inches. */
        public double strafePodOffsetForwardInches = 0.0;

        /** The one selected vendor preset or custom encoder resolution. */
        public EncoderResolution encoderResolution = EncoderResolution.forGoBildaPod(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        /** Direction for the forward (X) pod encoder. */
        public GoBildaPinpointDriver.EncoderDirection forwardPodDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;

        /** Direction for the strafe (Y) pod encoder. */
        public GoBildaPinpointDriver.EncoderDirection strafePodDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;

        /** Optional positive yaw scalar; {@code null} retains the factory calibration. */
        public Double yawScalar = null;

        /** Quality score published with pose and motion evidence, in {@code [0, 1]}. */
        public double quality = 0.75;

        private Config() {
        }

        /** @return a new mutable authoring draft populated with software defaults */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Return a raw copy without activating or validating this draft.
         *
         * <p>Null authored values are deliberately preserved so aggregate Config copies remain
         * mutation isolation rather than hidden active-owner validation.</p>
         */
        public Config copy() {
            Config c = defaults();
            c.hardwareMapName = this.hardwareMapName;
            c.forwardPodOffsetLeftInches = this.forwardPodOffsetLeftInches;
            c.strafePodOffsetForwardInches = this.strafePodOffsetForwardInches;
            c.encoderResolution = this.encoderResolution;
            c.forwardPodDirection = this.forwardPodDirection;
            c.strafePodDirection = this.strafePodDirection;
            c.yawScalar = this.yawScalar;
            c.quality = this.quality;
            return c;
        }

        /**
         * Validate and snapshot this draft before any Pinpoint hardware effect.
         *
         * @param context diagnostic owner/field prefix; null or blank uses this Config's canonical
         *                class name
         * @return independent validated snapshot
         * @throws NullPointerException for a required null field
         * @throws IllegalArgumentException for any value outside its documented domain or exact
         *                                  vendor float representation
         */
        public Config validatedCopy(String context) {
            String owner = normalizedContext(context);
            Config c = copy();
            if (c.hardwareMapName == null) {
                throw new NullPointerException(owner + ".hardwareMapName must not be null");
            }
            if (c.hardwareMapName.trim().isEmpty()) {
                throw new IllegalArgumentException(
                        owner + ".hardwareMapName must contain a non-whitespace FTC hardware name, got '"
                                + c.hardwareMapName + "'"
                );
            }
            requireOffsetRepresentable(
                    c.forwardPodOffsetLeftInches,
                    owner + ".forwardPodOffsetLeftInches"
            );
            requireOffsetRepresentable(
                    c.strafePodOffsetForwardInches,
                    owner + ".strafePodOffsetForwardInches"
            );
            c.encoderResolution = Objects.requireNonNull(
                    c.encoderResolution,
                    owner + ".encoderResolution must not be null"
            );
            c.encoderResolution.requireVendorRepresentable(owner + ".encoderResolution");
            c.forwardPodDirection = Objects.requireNonNull(
                    c.forwardPodDirection,
                    owner + ".forwardPodDirection must not be null"
            );
            c.strafePodDirection = Objects.requireNonNull(
                    c.strafePodDirection,
                    owner + ".strafePodDirection must not be null"
            );
            if (c.yawScalar != null) {
                requirePositiveFloatRepresentable(c.yawScalar, owner + ".yawScalar");
            }
            if (!Double.isFinite(c.quality) || c.quality < 0.0 || c.quality > 1.0) {
                throw new IllegalArgumentException(
                        owner + ".quality must be finite and in [0, 1], got " + c.quality
                );
            }
            return c;
        }

        private static String normalizedContext(String context) {
            return context == null || context.trim().isEmpty()
                    ? Config.class.getCanonicalName()
                    : context;
        }

        private static void requireOffsetRepresentable(double value, String fieldName) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException(
                        fieldName + " must be finite inches, got " + value
                );
            }
            float vendorMillimetres = (float) CONFIG_DISTANCE_UNIT.toMm(value);
            if (!Float.isFinite(vendorMillimetres)
                    || (value != 0.0 && vendorMillimetres == 0.0f)) {
                throw new IllegalArgumentException(
                        fieldName + " must remain finite and preserve a nonzero value when "
                                + "translated to the Pinpoint millimetre float, got " + value
                );
            }
        }

        private static void requirePositiveFloatRepresentable(double value, String fieldName) {
            float vendorValue = (float) value;
            if (!Double.isFinite(value) || value <= 0.0
                    || !Float.isFinite(vendorValue) || vendorValue <= 0.0f) {
                throw new IllegalArgumentException(
                        fieldName + " must be finite and > 0 and remain a finite positive Pinpoint "
                                + "float, got " + value
                );
            }
        }
    }

    private static final String CONFIG_CONTEXT =
            PinpointOdometryPredictor.class.getCanonicalName() + ".Config";

    private final PinpointDevice odo;
    private final Config cfg;

    private PoseEstimate lastEstimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
    private MotionDelta lastMotionDelta = MotionDelta.none(LoopTimestamp.unavailable());
    private PinpointKinematicSnapshot lastKinematicSnapshot =
            PinpointKinematicSnapshot.unavailable(
                    PinpointKinematicSnapshot.NO_CYCLE,
                    LoopTimestamp.unavailable(),
                    0.0
            );
    private final UpdateState updateState = new UpdateState();
    private boolean hasPreviousPhysicalHeading;
    private double previousPhysicalHeadingRad;
    private double totalHeadingRad;
    private GoBildaPinpointDriver.DeviceStatus lastDeviceStatus =
            GoBildaPinpointDriver.DeviceStatus.NOT_READY;

    /**
     * Creates a Pinpoint-backed motion predictor.
     *
     * <p>The complete Config is validated and defensively captured before device lookup. After
     * configuration, construction issues exactly one nonblocking pose/IMU reset and returns. The
     * robot must remain stationary until a later {@link #update(LoopClock)} observes exact
     * {@link GoBildaPinpointDriver.DeviceStatus#READY}; use {@link #lastDeviceStatus()} and the
     * published pose availability rather than a delay.</p>
     *
     * @throws NullPointerException if {@code hardwareMap}, {@code config}, or a required Config
     *                              field is null
     * @throws IllegalArgumentException if a Config value is outside its documented domain or
     *                                  exact vendor representation
     */
    public PinpointOdometryPredictor(HardwareMap hardwareMap, Config config) {
        this(hardwareLookup(hardwareMap), config);
    }

    /**
     * Package-private deterministic seam for testing lookup/effect order without an I2C device.
     */
    PinpointOdometryPredictor(PinpointDeviceLookup deviceLookup, Config config) {
        PinpointDeviceLookup lookup = Objects.requireNonNull(deviceLookup, "deviceLookup");
        Config draft = Objects.requireNonNull(
                config,
                CONFIG_CONTEXT + " must not be null; call Config.defaults() explicitly"
        );
        this.cfg = draft.validatedCopy(CONFIG_CONTEXT);

        try {
            this.odo = Objects.requireNonNull(
                    lookup.get(this.cfg.hardwareMapName),
                    "Pinpoint hardware lookup returned null for '" + this.cfg.hardwareMapName + "'"
            );
        } catch (RuntimeException failure) {
            throw constructionFailure("device lookup", failure);
        }

        configureStage("pod-offset configuration", () -> odo.setOffsetsInches(
                this.cfg.forwardPodOffsetLeftInches,
                this.cfg.strafePodOffsetForwardInches
        ));
        if (this.cfg.yawScalar != null) {
            configureStage("yaw-scalar configuration", () -> odo.setYawScalar(this.cfg.yawScalar));
        }
        configureStage("encoder-resolution configuration", () ->
                this.cfg.encoderResolution.applyTo(
                        odo::setGoBildaEncoderResolution,
                        odo::setCustomEncoderResolutionTicksPerInch
                ));
        configureStage("encoder-direction configuration", () -> odo.setEncoderDirections(
                this.cfg.forwardPodDirection,
                this.cfg.strafePodDirection
        ));
        configureStage("initial pose/IMU reset", odo::resetPosAndIMU);
    }

    private static PinpointDeviceLookup hardwareLookup(HardwareMap hardwareMap) {
        HardwareMap requiredHardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        return hardwareMapName -> new SdkPinpointDevice(requiredHardwareMap.get(
                GoBildaPinpointDriver.class,
                hardwareMapName
        ));
    }

    private void configureStage(String stage, Runnable action) {
        try {
            action.run();
        } catch (RuntimeException failure) {
            throw constructionFailure(stage, failure);
        }
    }

    private IllegalStateException constructionFailure(String stage, RuntimeException failure) {
        return new IllegalStateException(
                "Pinpoint hardware '" + cfg.hardwareMapName + "' failed during " + stage,
                failure
        );
    }

    /**
     * Polls Pinpoint once for the supplied cycle and publishes pose, motion, and kinematics.
     *
     * <p>Repeated calls with the same {@link LoopClock#cycle()} are no-ops after a successful
     * update, so layered estimator/integration code cannot poll the hardware twice in one robot
     * loop. The cycle is claimed before touching hardware: reentrant updates fail fast, and if the
     * first attempt throws a {@link RuntimeException}, later calls in that same cycle rethrow that
     * exact failure instead of retrying a partially completed hardware operation. The next cycle
     * may attempt an update normally.</p>
     *
     * <p>A physical poll at the same timestamp as the accepted motion baseline still refreshes the
     * absolute pose and kinematic snapshot, but publishes no motion delta and does not consume the
     * changed pose. The first later positive-time sample therefore includes all motion since the
     * accepted baseline instead of silently losing it.</p>
     *
     * @param clock shared robot loop clock; must not be {@code null}
     * @throws NullPointerException if {@code clock} is {@code null}
     * @throws IllegalStateException if this method is called reentrantly
     */
    @Override
    public void update(LoopClock clock) {
        LoopClock currentClock = Objects.requireNonNull(
                clock,
                "PinpointOdometryPredictor.update(clock) requires the shared LoopClock"
        );
        if (!updateState.beginUpdate(currentClock)) {
            return;
        }

        try {
            updateFromHardware(currentClock.cycle(), currentClock.nowTimestamp());
        } catch (RuntimeException failure) {
            // A failed poll may have consumed only part of a hardware sample. Do not bridge a
            // later motion delta or physical turn across that unknown interval. The status cache
            // remains the status from the most recent completed vendor poll/status read.
            invalidateMeasuredBaselines();
            throw updateState.recordFailure(failure);
        } finally {
            updateState.endUpdate();
        }
    }

    /** Perform the one physical poll for an already-claimed loop cycle. */
    private void updateFromHardware(long cycle, LoopTimestamp timestamp) {
        odo.update();
        GoBildaPinpointDriver.DeviceStatus polledStatus = odo.getDeviceStatus();
        lastDeviceStatus = polledStatus != null
                ? polledStatus
                : GoBildaPinpointDriver.DeviceStatus.NOT_READY;
        if (lastDeviceStatus != GoBildaPinpointDriver.DeviceStatus.READY) {
            publishUnavailableMeasuredState(cycle, timestamp);
            return;
        }

        Pose2D pos = odo.getPosition();

        if (pos == null) {
            publishUnavailableMeasuredState(cycle, timestamp);
            return;
        }

        double xIn = pos.getX(DistanceUnit.INCH);
        double yIn = pos.getY(DistanceUnit.INCH);
        double headingRad = normalizeDriverHeadingRad(pos.getHeading(AngleUnit.RADIANS));

        if (!isFinite(xIn, yIn, headingRad)) {
            publishUnavailableMeasuredState(cycle, timestamp);
            return;
        }

        double nextTotalHeadingRad = totalHeadingRad;
        if (hasPreviousPhysicalHeading) {
            nextTotalHeadingRad = accumulateUnwrappedHeadingRad(
                    totalHeadingRad,
                    previousPhysicalHeadingRad,
                    headingRad
            );
            if (!Double.isFinite(nextTotalHeadingRad)) {
                publishUnavailableMeasuredState(cycle, timestamp);
                return;
            }
        }

        double fieldVelocityXInchesPerSec = odo.getVelXInchesPerSec();
        double fieldVelocityYInchesPerSec = odo.getVelYInchesPerSec();
        double angularVelocityRadPerSec = odo.getHeadingVelocityRadPerSec();
        boolean hasVelocity = isFinite(
                fieldVelocityXInchesPerSec,
                fieldVelocityYInchesPerSec,
                angularVelocityRadPerSec
        );

        Pose3d pose = new Pose3d(xIn, yIn, 0.0, headingRad, 0.0, 0.0);
        MotionDelta nextMotionDelta = updateState.observeMotion(pose, timestamp, cfg.quality);
        PoseEstimate nextEstimate = new PoseEstimate(pose, true, cfg.quality, timestamp);
        PinpointKinematicSnapshot nextKinematicSnapshot = PinpointKinematicSnapshot.sampled(
                pose.toPose2d(),
                hasVelocity,
                cycle,
                timestamp,
                fieldVelocityXInchesPerSec,
                fieldVelocityYInchesPerSec,
                angularVelocityRadPerSec,
                nextTotalHeadingRad
        );

        previousPhysicalHeadingRad = headingRad;
        hasPreviousPhysicalHeading = true;
        totalHeadingRad = nextTotalHeadingRad;
        lastMotionDelta = nextMotionDelta;
        lastEstimate = nextEstimate;
        lastKinematicSnapshot = nextKinematicSnapshot;
    }

    /** Publish one completed poll that did not provide READY finite measured evidence. */
    private void publishUnavailableMeasuredState(long cycle, LoopTimestamp timestamp) {
        invalidateMeasuredBaselines();
        lastEstimate = PoseEstimate.noPose(timestamp);
        lastMotionDelta = MotionDelta.none(timestamp);
        lastKinematicSnapshot = PinpointKinematicSnapshot.unavailable(
                cycle,
                timestamp,
                totalHeadingRad
        );
    }

    /** Clear baselines that may only span consecutive successful READY samples. */
    private void invalidateMeasuredBaselines() {
        updateState.invalidateMotionBaseline();
        hasPreviousPhysicalHeading = false;
        previousPhysicalHeadingRad = 0.0;
    }

    /**
     * Returns the latest available Pinpoint-frame coordinate.
     *
     * <p>A successful finite {@code READY} poll publishes measured pose with that poll's timestamp.
     * Non-READY or unusable polls publish no pose. A deliberate {@link #setPose(Pose2d)} may expose
     * the commanded coordinate, but its timestamp is unavailable unless a surviving READY measured
     * fact can be rebased. Evidence consumers must therefore check both {@link PoseEstimate#hasPose}
     * and timestamp freshness; callers that specifically require physical device readiness must
     * also inspect the cache-only {@link #lastDeviceStatus()}.</p>
     */
    @Override
    public PoseEstimate getEstimate() {
        return lastEstimate;
    }

    /**
     * Returns the most recent timestamped motion increment computed from successive Pinpoint poses.
     */
    @Override
    public MotionDelta getLatestMotionDelta() {
        return lastMotionDelta;
    }

    /**
     * Returns the immutable kinematic sample produced by the latest physical Pinpoint poll.
     *
     * <p>A deliberate {@link #setPose(Pose2d)} may rebase the snapshot pose afterward. It preserves
     * measured velocity and poll time only while exact READY completed-poll baselines still exist;
     * otherwise it exposes the commanded coordinate with unavailable measured timing. Physical
     * total heading is never changed by the coordinate rebase. Callers that require current-loop
     * data must also check {@link PinpointKinematicSnapshot#isCurrentFor(LoopClock)}.</p>
     */
    public PinpointKinematicSnapshot getKinematicSnapshot() {
        return lastKinematicSnapshot;
    }

    /**
     * Return the status captured from the most recent completed Pinpoint poll.
     *
     * <p>This is a cache-only read. It never polls hardware. Construction, explicit pose/IMU
     * reset, and explicit recalibration force {@link GoBildaPinpointDriver.DeviceStatus#NOT_READY}
     * until a later {@link #update(LoopClock)} captures another status.</p>
     */
    public GoBildaPinpointDriver.DeviceStatus lastDeviceStatus() {
        return lastDeviceStatus;
    }

    /**
     * Resets the Pinpoint IMU and pose back to 0,0,0.
     *
     * <p>Published pose and motion are invalidated before the vendor operation. If that operation
     * throws after acting partially, callers therefore observe unavailable state rather than a
     * replayable pre-reset delta. The call returns without waiting for calibration; keep the drive
     * owner stopped until a later poll reports exact READY and publishes a measured sample.</p>
     */
    public void resetPosAndIMU() {
        // The vendor call is not transactional. Invalidate both internal and published state
        // first so a partial reset cannot leave pre-reset motion or pose available to a caller.
        LoopTimestamp timestamp = lastTimestamp();
        invalidateMeasuredBaselines();
        lastEstimate = PoseEstimate.noPose(timestamp);
        lastMotionDelta = MotionDelta.none(timestamp);
        totalHeadingRad = 0.0;
        lastDeviceStatus = GoBildaPinpointDriver.DeviceStatus.NOT_READY;
        lastKinematicSnapshot = PinpointKinematicSnapshot.unavailable(
                lastKinematicSnapshot.cycle,
                timestamp,
                0.0
        );
        odo.resetPosAndIMU();
    }

    /**
     * Recalibrates the IMU without resetting pose.
     *
     * <p>The operation returns immediately and forces all measured pose, velocity, and motion
     * unavailable. The robot must remain stationary until a later poll captures exact READY and
     * establishes fresh physical baselines. Recalibration never restores an earlier cached pose as
     * though calibration had completed.</p>
     */
    public void recalibrateIMU() {
        LoopTimestamp timestamp = lastTimestamp();
        long previousCycle = lastKinematicSnapshot.cycle;
        invalidateMeasuredBaselines();
        lastEstimate = PoseEstimate.noPose(timestamp);
        lastMotionDelta = MotionDelta.none(timestamp);
        lastDeviceStatus = GoBildaPinpointDriver.DeviceStatus.NOT_READY;
        lastKinematicSnapshot = PinpointKinematicSnapshot.unavailable(
                previousCycle,
                timestamp,
                totalHeadingRad
        );

        odo.recalibrateIMU();
    }

    /**
     * Snaps both the underlying Pinpoint device and this predictor's cached estimate to a known field pose.
     *
     * <p>Null, non-finite, or unrepresentable poses fail before any cache or vendor effect. After
     * validation, published pose and motion fail closed before the nontransactional vendor write;
     * the commanded coordinate becomes visible only after that write succeeds.</p>
     *
     * <p>A coordinate rebase does not change cached device status and does not prove physical
     * readiness. If exact READY and corresponding completed-poll baselines still exist, the rebase
     * preserves that measured timestamp, velocity, and accumulated physical heading while moving
     * the baselines. Otherwise the commanded coordinate is published with an unavailable timestamp
     * and no measured velocity/baseline. A later READY poll must establish fresh evidence before it
     * can publish motion.</p>
     */
    @Override
    public void setPose(Pose2d pose) {
        Pose2d rebasedPose = requireVendorRepresentablePose(pose);
        Pose2D set = new Pose2D(
                DistanceUnit.INCH,
                rebasedPose.xInches,
                rebasedPose.yInches,
                AngleUnit.RADIANS,
                rebasedPose.headingRad
        );
        // The vendor call is not transactional. Fail closed before it starts so a partial pose
        // write cannot leave either the old motion interval or old coordinate snapshot visible.
        LoopTimestamp timestamp = lastTimestamp();
        PoseEstimate previousEstimate = lastEstimate;
        PinpointKinematicSnapshot previousSnapshot = lastKinematicSnapshot;
        boolean canPreserveMeasuredEvidence =
                lastDeviceStatus == GoBildaPinpointDriver.DeviceStatus.READY
                        && previousEstimate.hasPose
                        && previousSnapshot.hasPose
                        && updateState.hasMotionBaseline()
                        && hasPreviousPhysicalHeading;
        invalidateMeasuredBaselines();
        lastEstimate = PoseEstimate.noPose(timestamp);
        lastMotionDelta = MotionDelta.none(timestamp);
        lastKinematicSnapshot = PinpointKinematicSnapshot.unavailable(
                previousSnapshot.cycle,
                timestamp,
                totalHeadingRad
        );

        odo.setPosition(set);

        LoopTimestamp publishedTimestamp = canPreserveMeasuredEvidence
                ? previousEstimate.timestamp
                : LoopTimestamp.unavailable();
        lastEstimate = new PoseEstimate(new Pose3d(
                rebasedPose.xInches,
                rebasedPose.yInches,
                0.0,
                rebasedPose.headingRad,
                0.0,
                0.0
        ), true, cfg.quality, publishedTimestamp);
        lastMotionDelta = MotionDelta.none(publishedTimestamp);
        if (canPreserveMeasuredEvidence) {
            updateState.rebaseMotionBaseline(
                    lastEstimate.fieldToRobotPose,
                    publishedTimestamp
            );
            // A coordinate correction is not physical motion; retain measured velocity/turn.
            previousPhysicalHeadingRad = rebasedPose.headingRad;
            hasPreviousPhysicalHeading = true;
            lastKinematicSnapshot = previousSnapshot.withRebasedPose(rebasedPose);
        } else {
            lastKinematicSnapshot = PinpointKinematicSnapshot.commanded(
                    rebasedPose,
                    previousSnapshot.cycle,
                    totalHeadingRad
            );
        }
    }

    private LoopTimestamp lastTimestamp() {
        return lastEstimate != null && lastEstimate.timestamp != null
                ? lastEstimate.timestamp
                : LoopTimestamp.unavailable();
    }

    /** Accumulate the shortest signed physical turn across a wrapped-heading boundary. */
    static double accumulateUnwrappedHeadingRad(double totalHeadingRad,
                                                double previousHeadingRad,
                                                double currentHeadingRad) {
        return totalHeadingRad + MathUtil.wrapToPi(currentHeadingRad - previousHeadingRad);
    }

    private static double normalizeDriverHeadingRad(double headingRad) {
        if (Math.abs(headingRad) > Math.PI * 2.0 + 0.5) {
            headingRad = Math.toRadians(headingRad);
        }
        return MathUtil.wrapToPi(headingRad);
    }

    private static boolean isFinite(double... values) {
        for (double value : values) {
            if (!Double.isFinite(value)) {
                return false;
            }
        }
        return true;
    }

    private static Pose2d requireVendorRepresentablePose(Pose2d pose) {
        Pose2d requiredPose = Objects.requireNonNull(
                pose,
                "PinpointOdometryPredictor.setPose(pose) requires a non-null finite Pose2d"
        );
        requireFinitePoseComponent(requiredPose.xInches, "pose.xInches", "inches");
        requireFinitePoseComponent(requiredPose.yInches, "pose.yInches", "inches");
        requireFinitePoseComponent(requiredPose.headingRad, "pose.headingRad", "radians");

        requireDistanceFloatRepresentable(requiredPose.xInches, "pose.xInches");
        requireDistanceFloatRepresentable(requiredPose.yInches, "pose.yInches");
        double wrappedHeadingRad = MathUtil.wrapToPi(requiredPose.headingRad);
        float vendorHeadingRad = (float) wrappedHeadingRad;
        if (!Float.isFinite(vendorHeadingRad)
                || (wrappedHeadingRad != 0.0 && vendorHeadingRad == 0.0f)) {
            throw new IllegalArgumentException(
                    "PinpointOdometryPredictor.pose.headingRad must remain finite and preserve a "
                            + "nonzero wrapped value in the Pinpoint float, got "
                            + requiredPose.headingRad + " (wrapped " + wrappedHeadingRad + ")"
            );
        }
        return new Pose2d(requiredPose.xInches, requiredPose.yInches, wrappedHeadingRad);
    }

    private static void requireFinitePoseComponent(double value, String fieldName, String units) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(
                    "PinpointOdometryPredictor." + fieldName + " must be finite " + units
                            + ", got " + value
            );
        }
    }

    private static void requireDistanceFloatRepresentable(double inches, String fieldName) {
        float vendorMillimetres = (float) CONFIG_DISTANCE_UNIT.toMm(inches);
        if (!Float.isFinite(vendorMillimetres)
                || (inches != 0.0 && vendorMillimetres == 0.0f)) {
            throw new IllegalArgumentException(
                    "PinpointOdometryPredictor." + fieldName + " must remain finite and preserve "
                            + "a nonzero value in the Pinpoint millimetre float, got " + inches
            );
        }
    }

    /**
     * Emits only cached predictor state and the captured static configuration.
     *
     * <p>This method never polls Pinpoint or performs another live status read.</p>
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "pinpoint" : prefix;
        dbg.addData(p + ".class", getClass().getSimpleName())
                .addData(p + ".driverStatus", lastDeviceStatus)
                .addData(p + ".lastEstimate", lastEstimate)
                .addData(p + ".lastMotionDelta", lastMotionDelta)
                .addData(p + ".lastKinematicSnapshot", lastKinematicSnapshot)
                .addData(p + ".lastUpdateCycle", updateState.lastUpdateCycle())
                .addData(p + ".cfg.hardwareMapName", cfg.hardwareMapName)
                .addData(p + ".cfg.forwardPodOffsetLeftInches",
                        cfg.forwardPodOffsetLeftInches)
                .addData(p + ".cfg.strafePodOffsetForwardInches",
                        cfg.strafePodOffsetForwardInches)
                .addData(p + ".cfg.encoderResolution", cfg.encoderResolution)
                .addData(p + ".cfg.forwardPodDirection", cfg.forwardPodDirection)
                .addData(p + ".cfg.strafePodDirection", cfg.strafePodDirection)
                .addData(p + ".cfg.yawScalar", cfg.yawScalar)
                .addData(p + ".cfg.quality", cfg.quality);
    }

    /** Package-private acquisition seam; production still exposes only the HardwareMap constructor. */
    interface PinpointDeviceLookup {
        PinpointDevice get(String hardwareMapName);
    }

    /** Small package-private device surface used to prove lifecycle behavior without raw export. */
    interface PinpointDevice {
        void setOffsetsInches(double forwardPodOffsetLeftInches,
                              double strafePodOffsetForwardInches);

        void setYawScalar(double yawScalar);

        void setGoBildaEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods pod);

        void setCustomEncoderResolutionTicksPerInch(double ticksPerInch);

        void setEncoderDirections(GoBildaPinpointDriver.EncoderDirection forwardDirection,
                                  GoBildaPinpointDriver.EncoderDirection strafeDirection);

        void resetPosAndIMU();

        void recalibrateIMU();

        void update();

        GoBildaPinpointDriver.DeviceStatus getDeviceStatus();

        Pose2D getPosition();

        double getVelXInchesPerSec();

        double getVelYInchesPerSec();

        double getHeadingVelocityRadPerSec();

        void setPosition(Pose2D pose);
    }

    /** Exact FTC SDK translation kept private to the owned hardware boundary. */
    private static final class SdkPinpointDevice implements PinpointDevice {
        private final GoBildaPinpointDriver driver;

        private SdkPinpointDevice(GoBildaPinpointDriver driver) {
            this.driver = Objects.requireNonNull(driver, "driver");
        }

        @Override
        public void setOffsetsInches(double forwardPodOffsetLeftInches,
                                     double strafePodOffsetForwardInches) {
            driver.setOffsets(
                    forwardPodOffsetLeftInches,
                    strafePodOffsetForwardInches,
                    CONFIG_DISTANCE_UNIT
            );
        }

        @Override
        public void setYawScalar(double yawScalar) {
            driver.setYawScalar(yawScalar);
        }

        @Override
        public void setGoBildaEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods pod) {
            driver.setEncoderResolution(pod);
        }

        @Override
        public void setCustomEncoderResolutionTicksPerInch(double ticksPerInch) {
            driver.setEncoderResolution(ticksPerInch, CONFIG_DISTANCE_UNIT);
        }

        @Override
        public void setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection forwardDirection,
                GoBildaPinpointDriver.EncoderDirection strafeDirection) {
            driver.setEncoderDirections(forwardDirection, strafeDirection);
        }

        @Override
        public void resetPosAndIMU() {
            driver.resetPosAndIMU();
        }

        @Override
        public void recalibrateIMU() {
            driver.recalibrateIMU();
        }

        @Override
        public void update() {
            driver.update();
        }

        @Override
        public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
            return driver.getDeviceStatus();
        }

        @Override
        public Pose2D getPosition() {
            return driver.getPosition();
        }

        @Override
        public double getVelXInchesPerSec() {
            return driver.getVelX(DistanceUnit.INCH);
        }

        @Override
        public double getVelYInchesPerSec() {
            return driver.getVelY(DistanceUnit.INCH);
        }

        @Override
        public double getHeadingVelocityRadPerSec() {
            UnnormalizedAngleUnit radians = AngleUnit.RADIANS.getUnnormalized();
            return driver.getHeadingVelocity(radians);
        }

        @Override
        public void setPosition(Pose2D pose) {
            driver.setPosition(pose);
        }
    }

    /**
     * Pure-Java state for cycle-attempt ownership and accepted motion baselines.
     *
     * <p>Package-private visibility keeps the production hardware boundary small while allowing
     * deterministic tests to exercise the timing rules without mocking the FTC I2C driver.</p>
     */
    static final class UpdateState {
        private long lastUpdateCycle = Long.MIN_VALUE;
        private boolean updateInProgress;
        private RuntimeException lastUpdateFailure;

        private boolean hasMotionBaseline;
        private Pose3d motionBaselinePose = Pose3d.zero();
        private LoopTimestamp motionBaselineTimestamp = LoopTimestamp.unavailable();

        /** Claim one cycle before its hardware effects, or report a completed duplicate. */
        boolean beginUpdate(LoopClock clock) {
            LoopClock currentClock = Objects.requireNonNull(
                    clock,
                    "Pinpoint update state requires the shared LoopClock"
            );
            long cycle = currentClock.cycle();
            if (updateInProgress) {
                throw new IllegalStateException(
                        "PinpointOdometryPredictor.update(clock) reentered while cycle "
                                + lastUpdateCycle + " was still in progress; one Pinpoint owner "
                                + "may poll hardware once per cycle"
                );
            }
            if (lastUpdateCycle == cycle) {
                if (lastUpdateFailure != null) {
                    throw lastUpdateFailure;
                }
                return false;
            }

            lastUpdateCycle = cycle;
            lastUpdateFailure = null;
            updateInProgress = true;
            return true;
        }

        /** Retain the first RuntimeException produced by the claimed cycle. */
        RuntimeException recordFailure(RuntimeException failure) {
            RuntimeException requiredFailure = Objects.requireNonNull(failure, "failure");
            if (lastUpdateFailure == null) {
                lastUpdateFailure = requiredFailure;
            }
            return lastUpdateFailure;
        }

        /** Finish the claimed attempt without making the cycle eligible for another poll. */
        void endUpdate() {
            updateInProgress = false;
        }

        /**
         * Publish one motion delta only when positive time has elapsed from the accepted baseline.
         */
        MotionDelta observeMotion(Pose3d pose,
                                  LoopTimestamp timestamp,
                                  double quality) {
            Pose3d currentPose = Objects.requireNonNull(pose, "pose");
            LoopTimestamp currentTimestamp = Objects.requireNonNull(timestamp, "timestamp");

            if (!hasMotionBaseline) {
                rebaseMotionBaseline(currentPose, currentTimestamp);
                return MotionDelta.none(currentTimestamp);
            }

            double elapsedSec = currentTimestamp.secondsSince(motionBaselineTimestamp);
            if (!Double.isFinite(elapsedSec) || elapsedSec < 0.0) {
                rebaseMotionBaseline(currentPose, currentTimestamp);
                return MotionDelta.none(currentTimestamp);
            }
            if (elapsedSec == 0.0) {
                return MotionDelta.none(currentTimestamp);
            }

            Pose3d deltaPose = motionBaselinePose.inverse().then(currentPose);
            if (!isFinite(deltaPose.xInches, deltaPose.yInches, deltaPose.yawRad)) {
                // Both samples were finite, but their relative transform exceeded representable
                // planar arithmetic. Do not claim a delta and do not bridge this interval again.
                rebaseMotionBaseline(currentPose, currentTimestamp);
                return MotionDelta.none(currentTimestamp);
            }
            MotionDelta result = new MotionDelta(
                    deltaPose,
                    true,
                    quality,
                    motionBaselineTimestamp,
                    currentTimestamp
            );
            rebaseMotionBaseline(currentPose, currentTimestamp);
            return result;
        }

        /** Anchor later motion to a deliberate coordinate rebase. */
        void rebaseMotionBaseline(Pose3d pose, LoopTimestamp timestamp) {
            motionBaselinePose = Objects.requireNonNull(pose, "pose");
            motionBaselineTimestamp = Objects.requireNonNull(timestamp, "timestamp");
            hasMotionBaseline = true;
        }

        /** Whether a completed READY sample or deliberate READY rebase owns a motion anchor. */
        boolean hasMotionBaseline() {
            return hasMotionBaseline;
        }

        /** Require the next valid sample to establish a fresh physical-motion baseline. */
        void invalidateMotionBaseline() {
            hasMotionBaseline = false;
            motionBaselinePose = Pose3d.zero();
            motionBaselineTimestamp = LoopTimestamp.unavailable();
        }

        long lastUpdateCycle() {
            return lastUpdateCycle;
        }
    }
}
