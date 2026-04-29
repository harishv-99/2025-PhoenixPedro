package edu.ftcphoenix.fw.tools.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.LongSupplier;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.supervisor.FrameValue;
import edu.ftcphoenix.fw.supervisor.HeldValue;
import edu.ftcphoenix.fw.supervisor.RequestCounter;

/**
 * <h1>Example 09: Explicit Layered Shooter (Requests → Behavior → Realization)</h1>
 *
 * <p>This example is the first runnable framework example that names the three internal mechanism
 * roles directly:</p>
 *
 * <ol>
 *   <li><b>Requests</b> remember caller-owned intent.</li>
 *   <li><b>Behavior</b> owns execution, timing, gates, and priority.</li>
 *   <li><b>Realization</b> is the only place that touches Plants.</li>
 * </ol>
 *
 * <p>The mechanism stays deliberately small, but it is rich enough to show the three most common
 * request shapes together:</p>
 * <ul>
 *   <li><b>Held</b>: remember whether the flywheel should stay spun up, and remember the selected
 *       flywheel speed.</li>
 *   <li><b>Frame</b>: accept a manual feed power command that must be refreshed every loop.</li>
 *   <li><b>Pending</b>: queue one or more requested shots, then let behavior consume them when the
 *       flywheel is ready.</li>
 * </ul>
 *
 * <p>The example also shows a behavior-owned <b>pulse</b>: a shot request becomes a short feed
 * pulse only after the flywheel is at speed. That timing lives in the behavior layer, not in the
 * request layer and not in the plant layer.</p>
 *
 * <h2>Controls</h2>
 * <ul>
 *   <li>P1 left bumper: toggle the held flywheel request on/off.</li>
 *   <li>P1 d-pad up/down: adjust the held selected flywheel velocity.</li>
 *   <li>P1 A: request one shot.</li>
 *   <li>P1 B: request three shots.</li>
 *   <li>P1 X: cancel pending shots and any active feed pulse.</li>
 *   <li>P1 right stick Y: manual feed power (frame-valued, refreshed every loop).</li>
 * </ul>
 *
 * <p>This OpMode intentionally skips drivetrain code so the mechanism layering stays easy to see.
 * Earlier examples already cover drive streams. This one focuses purely on how a richer mechanism
 * can be structured inside one owner.</p>
 */
@TeleOp(name = "FW Ex 09: Layered Shooter", group = "Framework Examples")
@Disabled
public final class TeleOp_09_LayeredShooterMechanism extends OpMode {

    private static final String HW_SHOOTER_LEFT = "shooterLeftMotor";
    private static final String HW_SHOOTER_RIGHT = "shooterRightMotor";
    private static final String HW_FEED_LEFT = "transferLeftServo";
    private static final String HW_FEED_RIGHT = "transferRightServo";

    private static final double FLYWHEEL_MIN_VELOCITY_NATIVE = 1700.0;
    private static final double FLYWHEEL_MAX_VELOCITY_NATIVE = 2400.0;
    private static final double FLYWHEEL_DEFAULT_VELOCITY_NATIVE = 2100.0;
    private static final double FLYWHEEL_STEP_VELOCITY_NATIVE = 50.0;
    private static final double FLYWHEEL_READY_TOLERANCE_NATIVE = 100.0;

    private static final double MANUAL_FEED_DEADBAND = 0.08;
    private static final double MANUAL_FEED_ACTIVE_THRESHOLD = 0.05;
    private static final double FEED_PULSE_POWER = 0.85;
    private static final double FEED_PULSE_SEC = 0.15;
    private static final int MAX_PENDING_SHOTS = 4;

    private final LoopClock clock = new LoopClock();
    private final Bindings bindings = new Bindings();

    private Gamepads gamepads;
    private LayeredShooter shooter;

    /**
     * {@inheritDoc}
     */
    @Override
    public void init() {
        gamepads = Gamepads.create(gamepad1, gamepad2);

        ScalarTarget flywheelTarget = ScalarTarget.held(0.0);
        ScalarTarget feederTarget = ScalarTarget.held(0.0);

        Plant flywheelPlant = FtcActuators.plant(hardwareMap)
                .motor(HW_SHOOTER_LEFT, Direction.FORWARD)
                .andMotor(HW_SHOOTER_RIGHT, Direction.REVERSE)
                .velocity()
                .deviceManagedWithDefaults()
                .bounded(0.0, FLYWHEEL_MAX_VELOCITY_NATIVE)
                .nativeUnits()
                .velocityTolerance(FLYWHEEL_READY_TOLERANCE_NATIVE)
                .targetedBy(flywheelTarget)
                .build();

        Plant feederPlant = FtcActuators.plant(hardwareMap)
                .crServo(HW_FEED_LEFT, Direction.FORWARD)
                .andCrServo(HW_FEED_RIGHT, Direction.REVERSE)
                .power()
                .targetedBy(feederTarget)
                .build();

        shooter = new LayeredShooter(clock::cycle, flywheelPlant, flywheelTarget, feederPlant, feederTarget);

        // Held requests.
        bindings.toggleOnRise(gamepads.p1().leftBumper(), shooter::setFlywheelHeld);
        bindings.nudgeOnRise(gamepads.p1().dpadUp(), gamepads.p1().dpadDown(),
                FLYWHEEL_STEP_VELOCITY_NATIVE, shooter::adjustSelectedVelocityNative);

        // Pending requests.
        bindings.onRise(gamepads.p1().a(), shooter::requestSingleShot);
        bindings.onRise(gamepads.p1().b(), () -> shooter.requestShots(3));
        bindings.onRise(gamepads.p1().x(), shooter::cancelTransientActions);

        // Frame request.
        bindings.copyEachCycle(
                gamepads.p1().rightY().deadbandNormalized(MANUAL_FEED_DEADBAND, -1.0, 1.0),
                shooter::commandManualFeedPower
        );
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void loop() {
        clock.update(getRuntime());
        bindings.update(clock);

        shooter.update(clock);

        telemetry.addLine("Layer 1 / requests");
        telemetry.addData("req.flywheelHeld", shooter.flywheelHeld());
        telemetry.addData("req.selectedVelocityNative", shooter.selectedVelocityNative());
        telemetry.addData("req.manualFeedPower", shooter.manualFeedPower());
        telemetry.addData("req.pendingShots", shooter.pendingShotCount());

        telemetry.addLine("Layer 2 / behavior");
        telemetry.addData("behavior.feedMode", shooter.feedModeName());
        telemetry.addData("behavior.blocked", shooter.blockedReasonName());
        telemetry.addData("behavior.flywheelWanted", shooter.flywheelWanted());
        telemetry.addData("behavior.feedPower", shooter.behaviorFeedPower());
        telemetry.addData("behavior.pulseActive", shooter.feedPulseActive(clock));

        telemetry.addLine("Layer 3 / realization");
        telemetry.addData("rz.flywheelTargetNative", shooter.flywheelTargetNative());
        telemetry.addData("rz.flywheelMeasuredNative", shooter.flywheelMeasuredNative());
        telemetry.addData("rz.flywheelReady", shooter.flywheelReadyForShot());
        telemetry.addData("rz.feedTargetPower", shooter.feedTargetPower());
        telemetry.update();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void stop() {
        if (shooter != null) {
            shooter.stop();
        }
    }

    /**
     * Small mechanism owner that exposes capability-like methods while internally splitting state
     * into Requests, Behavior, and Realization.
     */
    private static final class LayeredShooter {

        private final Requests requests;
        private final Behavior behavior = new Behavior();
        private final Realization realization;

        LayeredShooter(LongSupplier cycleSource,
                       Plant flywheelPlant,
                       ScalarTarget flywheelTarget,
                       Plant feederPlant,
                       ScalarTarget feederTarget) {
            requests = new Requests(cycleSource);
            realization = new Realization(flywheelPlant, flywheelTarget, feederPlant, feederTarget);
        }

        // ------------------------------------------------------------------
        // Caller-facing methods: these are the equivalent of capability methods.
        // They write Requests only.
        // ------------------------------------------------------------------

        void setFlywheelHeld(boolean enabled) {
            requests.flywheelHeld.set(enabled);
        }

        void setSelectedVelocityNative(double velocityNative) {
            requests.selectedVelocityNative.set(clampVelocityNative(velocityNative));
        }

        void adjustSelectedVelocityNative(double deltaNative) {
            setSelectedVelocityNative(requests.selectedVelocityNative.get() + deltaNative);
        }

        void requestSingleShot() {
            requests.shotRequests.request();
        }

        void requestShots(int count) {
            requests.shotRequests.request(count);
        }

        void commandManualFeedPower(double power) {
            requests.manualFeedPower.set(MathUtil.clamp(power, -1.0, 1.0));
        }

        void cancelTransientActions() {
            requests.shotRequests.clear();
            behavior.cancelTransientActions();
        }

        // ------------------------------------------------------------------
        // Main update path.
        // ------------------------------------------------------------------

        void update(LoopClock clock) {
            Readback readback = realization.readback();
            BehaviorOutput out = behavior.update(clock, requests, readback);
            realization.apply(clock, out);
        }

        void stop() {
            realization.stop();
        }

        // ------------------------------------------------------------------
        // Telemetry helpers.
        // ------------------------------------------------------------------

        boolean flywheelHeld() {
            return requests.flywheelHeld.get();
        }

        double selectedVelocityNative() {
            return requests.selectedVelocityNative.get();
        }


        double manualFeedPower() {
            return requests.manualFeedPower.get();
        }

        int pendingShotCount() {
            return requests.shotRequests.count();
        }

        String feedModeName() {
            return behavior.feedMode.name();
        }

        String blockedReasonName() {
            return behavior.blockedReason.name();
        }

        boolean flywheelWanted() {
            return behavior.flywheelWanted;
        }

        double behaviorFeedPower() {
            return behavior.feedPower;
        }

        boolean feedPulseActive(LoopClock clock) {
            return behavior.isFeedPulseActive(clock);
        }

        double flywheelTargetNative() {
            return realization.flywheelTargetNative();
        }

        double flywheelMeasuredNative() {
            return realization.readback().flywheelMeasurementNative;
        }

        boolean flywheelReadyForShot() {
            return realization.readback().flywheelReadyForSelectedVelocity(requests.selectedVelocityNative.get());
        }

        double feedTargetPower() {
            return realization.feedTargetPower();
        }

        private static double clampVelocityNative(double velocityNative) {
            return MathUtil.clamp(velocityNative,
                    FLYWHEEL_MIN_VELOCITY_NATIVE,
                    FLYWHEEL_MAX_VELOCITY_NATIVE);
        }
    }

    /**
     * Layer 1: caller-owned request memory.
     *
     * <p>This layer remembers:</p>
     * <ul>
     *   <li>a held flywheel enable request,</li>
     *   <li>a held selected velocity,</li>
     *   <li>a frame-valued manual feed command,</li>
     *   <li>and a pending shot count.</li>
     * </ul>
     */
    private static final class Requests {
        final HeldValue<Boolean> flywheelHeld = new HeldValue<Boolean>(false);
        final HeldValue<Double> selectedVelocityNative =
                new HeldValue<Double>(FLYWHEEL_DEFAULT_VELOCITY_NATIVE);
        final FrameValue<Double> manualFeedPower;
        final RequestCounter shotRequests = new RequestCounter(MAX_PENDING_SHOTS);

        Requests(LongSupplier cycleSource) {
            manualFeedPower = new FrameValue<Double>(cycleSource, 0.0);
        }
    }

    /**
     * Layer 2: robot-owned behavior.
     *
     * <p>This layer decides when pending shots may start, owns the feed pulse timing, and chooses
     * which feed behavior currently wins: pulse, manual frame command, or idle.</p>
     */
    private static final class Behavior {

        enum FeedMode {
            IDLE,
            MANUAL,
            PULSE
        }

        enum BlockedReason {
            NONE,
            MANUAL_OVERRIDE,
            WAITING_FOR_FLYWHEEL
        }

        private FeedMode feedMode = FeedMode.IDLE;
        private BlockedReason blockedReason = BlockedReason.NONE;

        private boolean flywheelWanted = false;
        private double feedPower = 0.0;
        private double activeFeedPulseUntilSec = Double.NEGATIVE_INFINITY;

        BehaviorOutput update(LoopClock clock, Requests requests, Readback readback) {
            boolean pulseActive = isFeedPulseActive(clock);
            double manualFeedPower = MathUtil.clamp(requests.manualFeedPower.get(), -1.0, 1.0);
            boolean manualFeedActive = Math.abs(manualFeedPower) > MANUAL_FEED_ACTIVE_THRESHOLD;

            blockedReason = BlockedReason.NONE;

            if (!pulseActive && requests.shotRequests.hasRequest()) {
                if (manualFeedActive) {
                    blockedReason = BlockedReason.MANUAL_OVERRIDE;
                } else if (readback.flywheelReadyForSelectedVelocity(
                        requests.selectedVelocityNative.get())) {
                    requests.shotRequests.consume();
                    activeFeedPulseUntilSec = clock.nowSec() + FEED_PULSE_SEC;
                    pulseActive = true;
                } else {
                    blockedReason = BlockedReason.WAITING_FOR_FLYWHEEL;
                }
            }

            flywheelWanted = requests.flywheelHeld.get()
                    || requests.shotRequests.hasRequest()
                    || pulseActive;

            double flywheelTargetNative =
                    flywheelWanted ? requests.selectedVelocityNative.get() : 0.0;

            if (pulseActive) {
                feedMode = FeedMode.PULSE;
                feedPower = FEED_PULSE_POWER;
            } else if (manualFeedActive) {
                feedMode = FeedMode.MANUAL;
                feedPower = manualFeedPower;
            } else {
                feedMode = FeedMode.IDLE;
                feedPower = 0.0;
            }

            return new BehaviorOutput(flywheelTargetNative, feedPower);
        }

        boolean isFeedPulseActive(LoopClock clock) {
            return clock.nowSec() < activeFeedPulseUntilSec;
        }

        void cancelTransientActions() {
            activeFeedPulseUntilSec = Double.NEGATIVE_INFINITY;
            blockedReason = BlockedReason.NONE;
            feedMode = FeedMode.IDLE;
            feedPower = 0.0;
        }
    }

    /**
     * Immutable behavior result passed from Layer 2 to Layer 3.
     */
    private static final class BehaviorOutput {
        final double flywheelTargetNative;
        final double feedPower;

        BehaviorOutput(double flywheelTargetNative, double feedPower) {
            this.flywheelTargetNative = flywheelTargetNative;
            this.feedPower = feedPower;
        }
    }

    /**
     * Readback snapshot exported by realization.
     *
     * <p>Behavior uses the previous realization state to decide whether a pending shot may start.
     * The readback is intentionally small: it contains only the information behavior actually
     * needs.</p>
     */
    private static final class Readback {
        final boolean flywheelAtSetpoint;
        final double flywheelTargetNative;
        final double flywheelMeasurementNative;

        Readback(boolean flywheelAtSetpoint,
                 double flywheelTargetNative,
                 double flywheelMeasurementNative) {
            this.flywheelAtSetpoint = flywheelAtSetpoint;
            this.flywheelTargetNative = flywheelTargetNative;
            this.flywheelMeasurementNative = flywheelMeasurementNative;
        }

        boolean flywheelReadyForSelectedVelocity(double selectedVelocityNative) {
            return flywheelAtSetpoint
                    && flywheelTargetNative > 0.0
                    && Math.abs(flywheelTargetNative - selectedVelocityNative) < 1e-6;
        }
    }

    /**
     * Layer 3: plant realization.
     *
     * <p>This is the only layer allowed to touch Plants. It reads the behavior output, writes the
     * final plant targets, updates the plants, and exports a small readback snapshot for the next
     * loop.</p>
     */
    private static final class Realization {
        private final Plant flywheel;
        private final ScalarTarget flywheelTarget;
        private final Plant feeder;
        private final ScalarTarget feederTarget;

        Realization(Plant flywheel, ScalarTarget flywheelTarget, Plant feeder, ScalarTarget feederTarget) {
            this.flywheel = flywheel;
            this.flywheelTarget = flywheelTarget;
            this.feeder = feeder;
            this.feederTarget = feederTarget;
        }

        Readback readback() {
            return new Readback(
                    flywheel.atTarget(),
                    flywheel.getRequestedTarget(),
                    flywheel.getMeasurement()
            );
        }

        void apply(LoopClock clock, BehaviorOutput out) {
            flywheelTarget.set(out.flywheelTargetNative);
            feederTarget.set(out.feedPower);

            flywheel.update(clock);
            feeder.update(clock);
        }

        double flywheelTargetNative() {
            return flywheel.getRequestedTarget();
        }

        double feedTargetPower() {
            return feeder.getRequestedTarget();
        }

        void stop() {
            flywheel.stop();
            feeder.stop();
        }
    }
}
