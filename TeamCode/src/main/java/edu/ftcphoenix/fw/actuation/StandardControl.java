package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Package-private standard scalar-control runtime for framework-regulated Plants.
 *
 * <p>The Plant remains the sole owner of the requested/applied target, heartbeat, completion, and
 * hardware write. This object owns only the conversion from one Plant-supplied applied goal and
 * measurement into one normalized power command. Its construction seam is deliberately
 * package-private so ordinary and advanced robot code cannot acquire a second public controller or
 * profile lifecycle alongside the Plant grammar.</p>
 *
 * <p>One successful call per {@link LoopClock#cycle()} advances setpoint and PID state. Repeated
 * calls in that cycle return the retained result. A failed call retains and rethrows the same
 * failure for the rest of the cycle. One instance binds to one clock identity until
 * {@link #reset()} or {@link #reseed(double, LoopClock)}.</p>
 */
final class StandardControl implements ScalarRegulator {

    private enum SetpointMode {
        POSITION_DIRECT,
        POSITION_TRAPEZOIDAL,
        VELOCITY_DIRECT,
        VELOCITY_ACCELERATION_LIMITED
    }

    private enum FeedforwardMode {
        NONE,
        MOTION,
        LIFT,
        ARM
    }

    /**
     * Package-private, single-use configuration retained only while a Plant recipe is assembled.
     */
    static final class Config {
        private final SetpointMode setpointMode;
        private final double maximumVelocity;
        private final double maximumAcceleration;

        private double kP;
        private double kI;
        private double kD;
        private boolean feedbackAnswered;

        private double integralMin = Double.NEGATIVE_INFINITY;
        private double integralMax = Double.POSITIVE_INFINITY;
        private boolean integralLimitAnswered;

        private double feedbackMin = Double.NEGATIVE_INFINITY;
        private double feedbackMax = Double.POSITIVE_INFINITY;
        private boolean feedbackLimitAnswered;

        private FeedforwardMode feedforwardMode = FeedforwardMode.NONE;
        private double kS;
        private double kV;
        private double kA;
        private double kG;
        private double positionAtMaximumGravity;
        private double radiansPerPlantUnit;
        private boolean feedforwardAnswered;

        private ScalarSource supplyVoltage;
        private double referenceVoltage;
        private double minimumVoltage;
        private double maximumVoltageScale;
        private boolean voltageAnswered;

        private double minimumOutput = -1.0;
        private double maximumOutput = 1.0;
        private boolean outputLimitAnswered;
        private boolean built;

        private Config(SetpointMode setpointMode,
                       double maximumVelocity,
                       double maximumAcceleration) {
            this.setpointMode = Objects.requireNonNull(setpointMode, "setpointMode");
            this.maximumVelocity = maximumVelocity;
            this.maximumAcceleration = maximumAcceleration;
        }

        /** Configure proportional-only feedback; integral and derivative gains are exactly zero. */
        Config feedbackFromPid(double kP) {
            return feedbackFromPid(kP, 0.0, 0.0);
        }

        /** Configure one finite PID feedback law in Plant units. */
        Config feedbackFromPid(double kP, double kI, double kD) {
            requireMutable("feedbackFromPid(...)");
            if (feedbackAnswered) {
                throw new IllegalStateException("feedbackFromPid(...) has already been answered");
            }
            requireFinite("feedbackFromPid(...) kP", kP);
            requireFinite("feedbackFromPid(...) kI", kI);
            requireFinite("feedbackFromPid(...) kD", kD);
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            feedbackAnswered = true;
            return this;
        }

        /** Limit the retained integral contribution; the finite range must include zero. */
        Config feedbackIntegralLimitedTo(double minimum, double maximum) {
            requireFeedbackAnswered("feedbackIntegralLimitedTo(...)");
            if (integralLimitAnswered) {
                throw new IllegalStateException(
                        "feedbackIntegralLimitedTo(...) has already been answered");
            }
            requireFiniteOrderedRange(
                    "feedbackIntegralLimitedTo(...)", minimum, maximum);
            if (minimum > 0.0 || maximum < 0.0) {
                throw new IllegalArgumentException(
                        "feedbackIntegralLimitedTo(...) requires minimum <= 0 <= maximum; got "
                                + "minimum=" + minimum + ", maximum=" + maximum);
            }
            integralMin = minimum;
            integralMax = maximum;
            integralLimitAnswered = true;
            return this;
        }

        /** Limit the complete PID feedback contribution before feedforward is added. */
        Config feedbackOutputLimitedTo(double minimum, double maximum) {
            requireFeedbackAnswered("feedbackOutputLimitedTo(...)");
            if (feedbackLimitAnswered) {
                throw new IllegalStateException(
                        "feedbackOutputLimitedTo(...) has already been answered");
            }
            requireFiniteOrderedRange("feedbackOutputLimitedTo(...)", minimum, maximum);
            feedbackMin = minimum;
            feedbackMax = maximum;
            feedbackLimitAnswered = true;
            return this;
        }

        /** Add static and velocity motion feedforward when velocity evidence exists. */
        Config feedforwardForMotion(double kS, double kV) {
            requireVelocityEvidence("feedforwardForMotion(kS, kV)");
            return answerFeedforward(FeedforwardMode.MOTION, kS, kV, 0.0, 0.0,
                    0.0, 0.0, "feedforwardForMotion(kS, kV)");
        }

        /** Add the full static/velocity/acceleration motion model when acceleration is known. */
        Config feedforwardForMotion(double kS, double kV, double kA) {
            requireAccelerationEvidence("feedforwardForMotion(kS, kV, kA)");
            return answerFeedforward(FeedforwardMode.MOTION, kS, kV, kA, 0.0,
                    0.0, 0.0, "feedforwardForMotion(kS, kV, kA)");
        }

        /** Add constant signed gravity compensation with all optional motion gains exactly zero. */
        Config feedforwardForLift(double kG) {
            return answerFeedforward(FeedforwardMode.LIFT, 0.0, 0.0, 0.0, kG,
                    0.0, 0.0, "feedforwardForLift(kG)");
        }

        /** Add lift gravity plus static/velocity motion terms when velocity is known. */
        Config feedforwardForLift(double kS, double kV, double kG) {
            requireVelocityEvidence("feedforwardForLift(kS, kV, kG)");
            return answerFeedforward(FeedforwardMode.LIFT, kS, kV, 0.0, kG,
                    0.0, 0.0, "feedforwardForLift(kS, kV, kG)");
        }

        /** Add lift gravity plus the full motion model when acceleration is known. */
        Config feedforwardForLift(double kS, double kV, double kA, double kG) {
            requireAccelerationEvidence("feedforwardForLift(kS, kV, kA, kG)");
            return answerFeedforward(FeedforwardMode.LIFT, kS, kV, kA, kG,
                    0.0, 0.0, "feedforwardForLift(kS, kV, kA, kG)");
        }

        /**
         * Add position-dependent arm gravity compensation with optional motion gains exactly zero.
         */
        Config feedforwardForArm(double kG,
                                 double positionAtMaximumGravity,
                                 double radiansPerPlantUnit) {
            requirePositionEvidence("feedforwardForArm(kG, position, radiansPerPlantUnit)");
            return answerFeedforward(FeedforwardMode.ARM, 0.0, 0.0, 0.0, kG,
                    positionAtMaximumGravity, radiansPerPlantUnit,
                    "feedforwardForArm(kG, position, radiansPerPlantUnit)");
        }

        /** Add arm gravity plus static/velocity motion terms for a profiled position setpoint. */
        Config feedforwardForArm(double kS,
                                 double kV,
                                 double kG,
                                 double positionAtMaximumGravity,
                                 double radiansPerPlantUnit) {
            requireProfiledPositionEvidence(
                    "feedforwardForArm(kS, kV, kG, position, radiansPerPlantUnit)");
            return answerFeedforward(FeedforwardMode.ARM, kS, kV, 0.0, kG,
                    positionAtMaximumGravity, radiansPerPlantUnit,
                    "feedforwardForArm(kS, kV, kG, position, radiansPerPlantUnit)");
        }

        /** Add arm gravity plus the full motion model for a profiled position setpoint. */
        Config feedforwardForArm(double kS,
                                 double kV,
                                 double kA,
                                 double kG,
                                 double positionAtMaximumGravity,
                                 double radiansPerPlantUnit) {
            requireProfiledPositionEvidence(
                    "feedforwardForArm(kS, kV, kA, kG, position, radiansPerPlantUnit)");
            return answerFeedforward(FeedforwardMode.ARM, kS, kV, kA, kG,
                    positionAtMaximumGravity, radiansPerPlantUnit,
                    "feedforwardForArm(kS, kV, kA, kG, position, radiansPerPlantUnit)");
        }

        /**
         * Apply supply-voltage compensation after feedback and feedforward and before final output
         * policy.
         */
        Config voltageCompensatedBy(ScalarSource supplyVoltage,
                                    double referenceVoltage,
                                    double minimumVoltage,
                                    double maximumScale) {
            requireFeedbackAnswered("voltageCompensatedBy(...)");
            if (voltageAnswered) {
                throw new IllegalStateException("voltageCompensatedBy(...) has already been answered");
            }
            Objects.requireNonNull(supplyVoltage, "supplyVoltage");
            requirePositiveFinite("voltageCompensatedBy(...) referenceVoltage", referenceVoltage);
            requirePositiveFinite("voltageCompensatedBy(...) minimumVoltage", minimumVoltage);
            requireFinite("voltageCompensatedBy(...) maximumScale", maximumScale);
            if (minimumVoltage > referenceVoltage) {
                throw new IllegalArgumentException(
                        "voltageCompensatedBy(...) requires minimumVoltage <= referenceVoltage; got "
                                + "minimumVoltage=" + minimumVoltage
                                + ", referenceVoltage=" + referenceVoltage);
            }
            if (maximumScale < 1.0) {
                throw new IllegalArgumentException(
                        "voltageCompensatedBy(...) requires maximumScale >= 1.0; got "
                                + maximumScale);
            }
            this.supplyVoltage = supplyVoltage;
            this.referenceVoltage = referenceVoltage;
            this.minimumVoltage = minimumVoltage;
            this.maximumVoltageScale = maximumScale;
            voltageAnswered = true;
            return this;
        }

        /** Limit the final normalized power to one symmetric finite magnitude in {@code [0,1]}. */
        Config outputPowerLimitedTo(double maximumMagnitude) {
            requireFeedbackAnswered("outputPowerLimitedTo(...)");
            requireFinite("outputPowerLimitedTo(...) maximumMagnitude", maximumMagnitude);
            if (maximumMagnitude < 0.0 || maximumMagnitude > 1.0) {
                throw new IllegalArgumentException(
                        "outputPowerLimitedTo(...) maximumMagnitude must be within [0.0, 1.0]; got "
                                + maximumMagnitude);
            }
            return answerOutputLimit(-maximumMagnitude, maximumMagnitude);
        }

        /** Limit final normalized power to an ordered range inside {@code [-1,1]} containing zero. */
        Config outputPowerLimitedTo(double minimum, double maximum) {
            requireFeedbackAnswered("outputPowerLimitedTo(...)");
            requireFiniteOrderedRange("outputPowerLimitedTo(...)", minimum, maximum);
            if (minimum < -1.0 || maximum > 1.0 || minimum > 0.0 || maximum < 0.0) {
                throw new IllegalArgumentException(
                        "outputPowerLimitedTo(...) requires -1 <= minimum <= 0 <= maximum <= 1; got "
                                + "minimum=" + minimum + ", maximum=" + maximum);
            }
            return answerOutputLimit(minimum, maximum);
        }

        /** Freeze this recipe and create one Plant-owned runtime without sampling or side effects. */
        StandardControl build() {
            requireMutable("build()");
            if (!feedbackAnswered) {
                throw new IllegalStateException(
                        "Standard control requires feedbackFromPid(...) before build()");
            }
            built = true;
            return new StandardControl(this);
        }

        private Config answerFeedforward(FeedforwardMode mode,
                                         double kS,
                                         double kV,
                                         double kA,
                                         double kG,
                                         double positionAtMaximumGravity,
                                         double radiansPerPlantUnit,
                                         String operation) {
            requireFeedbackAnswered(operation);
            if (feedforwardAnswered) {
                throw new IllegalStateException("Feedforward has already been answered");
            }
            requireFinite(operation + " kS", kS);
            requireFinite(operation + " kV", kV);
            requireFinite(operation + " kA", kA);
            requireFinite(operation + " kG", kG);
            if (mode == FeedforwardMode.ARM) {
                requireFinite(operation + " positionAtMaximumGravity", positionAtMaximumGravity);
                requireFinite(operation + " radiansPerPlantUnit", radiansPerPlantUnit);
                if (radiansPerPlantUnit == 0.0) {
                    throw new IllegalArgumentException(
                            operation + " radiansPerPlantUnit must be non-zero");
                }
            }
            this.feedforwardMode = mode;
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kG = kG;
            this.positionAtMaximumGravity = positionAtMaximumGravity;
            this.radiansPerPlantUnit = radiansPerPlantUnit;
            feedforwardAnswered = true;
            return this;
        }

        private Config answerOutputLimit(double minimum, double maximum) {
            if (outputLimitAnswered) {
                throw new IllegalStateException("outputPowerLimitedTo(...) has already been answered");
            }
            minimumOutput = minimum;
            maximumOutput = maximum;
            outputLimitAnswered = true;
            return this;
        }

        private void requireFeedbackAnswered(String operation) {
            requireMutable(operation);
            if (!feedbackAnswered) {
                throw new IllegalStateException(
                        operation + " requires feedbackFromPid(...) first");
            }
        }

        private void requireVelocityEvidence(String operation) {
            requireMutable(operation);
            if (setpointMode == SetpointMode.POSITION_DIRECT) {
                throw new IllegalStateException(
                        operation + " requires a motion-profile or velocity setpoint; direct "
                                + "position has no motion feedforward branch");
            }
        }

        private void requireAccelerationEvidence(String operation) {
            requireMutable(operation);
            if (setpointMode != SetpointMode.POSITION_TRAPEZOIDAL
                    && setpointMode != SetpointMode.VELOCITY_ACCELERATION_LIMITED) {
                throw new IllegalStateException(
                        operation + " requires a setpoint mode with acceleration evidence");
            }
        }

        private void requirePositionEvidence(String operation) {
            requireMutable(operation);
            if (!isPositionMode(setpointMode)) {
                throw new IllegalStateException(
                        operation + " requires a position setpoint for arm geometry");
            }
        }

        private void requireProfiledPositionEvidence(String operation) {
            requireMutable(operation);
            if (setpointMode != SetpointMode.POSITION_TRAPEZOIDAL) {
                throw new IllegalStateException(
                        operation + " requires a trapezoidal position setpoint");
            }
        }

        private void requireMutable(String operation) {
            if (built) {
                throw new IllegalStateException(
                        operation + " cannot modify a built StandardControl recipe");
            }
        }
    }

    private static final class SetpointSnapshot {
        private final double position;
        private final double velocity;
        private final double acceleration;
        private final boolean positionAvailable;
        private final boolean accelerationAvailable;
        private final boolean settled;

        private SetpointSnapshot(double position,
                                 double velocity,
                                 double acceleration,
                                 boolean positionAvailable,
                                 boolean accelerationAvailable,
                                 boolean settled) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.positionAvailable = positionAvailable;
            this.accelerationAvailable = accelerationAvailable;
            this.settled = settled;
        }
    }

    private static final class Evaluation {
        private final double integral;
        private final double feedbackUnbounded;
        private final double feedback;
        private final double feedforward;
        private final double voltageScale;
        private final double beforeOutputLimit;
        private final double output;
        private final boolean feedbackLimited;
        private final boolean outputLimited;

        private Evaluation(double integral,
                           double feedbackUnbounded,
                           double feedback,
                           double feedforward,
                           double voltageScale,
                           double beforeOutputLimit,
                           double output,
                           boolean feedbackLimited,
                           boolean outputLimited) {
            this.integral = integral;
            this.feedbackUnbounded = feedbackUnbounded;
            this.feedback = feedback;
            this.feedforward = feedforward;
            this.voltageScale = voltageScale;
            this.beforeOutputLimit = beforeOutputLimit;
            this.output = output;
            this.feedbackLimited = feedbackLimited;
            this.outputLimited = outputLimited;
        }
    }

    private final SetpointMode setpointMode;
    private final double maximumVelocity;
    private final double maximumAcceleration;
    private final double kP;
    private final double kI;
    private final double kD;
    private final double integralMin;
    private final double integralMax;
    private final double feedbackMin;
    private final double feedbackMax;
    private final FeedforwardMode feedforwardMode;
    private final double kS;
    private final double kV;
    private final double kA;
    private final double kG;
    private final double positionAtMaximumGravity;
    private final double radiansPerPlantUnit;
    private final ScalarSource supplyVoltage;
    private final double referenceVoltage;
    private final double minimumVoltage;
    private final double maximumVoltageScale;
    private final double minimumOutput;
    private final double maximumOutput;

    private LoopClock boundClock;
    private boolean initialized;
    private boolean skipNextDt;
    private long attemptedCycle;
    private boolean hasAttemptedCycle;
    private RuntimeException retainedCycleFailure;

    private double lastGoal = Double.NaN;
    private double lastMeasurement = Double.NaN;
    private double setpointPosition = Double.NaN;
    private double setpointVelocity = Double.NaN;
    private double setpointAcceleration = Double.NaN;
    private boolean setpointPositionAvailable;
    private boolean setpointAccelerationAvailable;
    private boolean setpointSettled;

    private double integralContribution;
    private double previousError;
    private boolean previousErrorAvailable;
    private double lastError = Double.NaN;
    private double lastFeedbackUnbounded = Double.NaN;
    private double lastFeedback = Double.NaN;
    private double lastFeedforward = Double.NaN;
    private double lastSupplyVoltage = Double.NaN;
    private double lastVoltageScale = 1.0;
    private double lastBeforeOutputLimit = Double.NaN;
    private double lastOutput = Double.NaN;
    private boolean lastFeedbackLimited;
    private boolean lastOutputLimited;
    private boolean lastIntegralGrowthBlocked;

    /** Direct position setpoint: position follows the applied target; velocity/acceleration are zero. */
    static Config positionFromAppliedTarget() {
        return new Config(SetpointMode.POSITION_DIRECT, Double.NaN, Double.NaN);
    }

    /** Trapezoidal position setpoint with explicit positive Plant-unit motion limits. */
    static Config positionFromTrapezoidalProfile(double maximumVelocity,
                                                 double maximumAcceleration) {
        requirePositiveFinite(
                "positionFromTrapezoidalProfile(...) maximumVelocity", maximumVelocity);
        requirePositiveFinite(
                "positionFromTrapezoidalProfile(...) maximumAcceleration", maximumAcceleration);
        return new Config(SetpointMode.POSITION_TRAPEZOIDAL,
                maximumVelocity, maximumAcceleration);
    }

    /** Direct velocity setpoint: velocity follows the applied target; acceleration is unavailable. */
    static Config velocityFromAppliedTarget() {
        return new Config(SetpointMode.VELOCITY_DIRECT, Double.NaN, Double.NaN);
    }

    /** Acceleration-limited velocity setpoint with one explicit positive Plant-unit limit. */
    static Config velocityFromAccelerationLimitedProfile(double maximumAcceleration) {
        requirePositiveFinite(
                "velocityFromAccelerationLimitedProfile(...) maximumAcceleration",
                maximumAcceleration);
        return new Config(SetpointMode.VELOCITY_ACCELERATION_LIMITED,
                Double.NaN, maximumAcceleration);
    }

    private StandardControl(Config config) {
        setpointMode = config.setpointMode;
        maximumVelocity = config.maximumVelocity;
        maximumAcceleration = config.maximumAcceleration;
        kP = config.kP;
        kI = config.kI;
        kD = config.kD;
        integralMin = config.integralMin;
        integralMax = config.integralMax;
        feedbackMin = config.feedbackMin;
        feedbackMax = config.feedbackMax;
        feedforwardMode = config.feedforwardMode;
        kS = config.kS;
        kV = config.kV;
        kA = config.kA;
        kG = config.kG;
        positionAtMaximumGravity = config.positionAtMaximumGravity;
        radiansPerPlantUnit = config.radiansPerPlantUnit;
        supplyVoltage = config.supplyVoltage;
        referenceVoltage = config.referenceVoltage;
        minimumVoltage = config.minimumVoltage;
        maximumVoltageScale = config.maximumVoltageScale;
        minimumOutput = config.minimumOutput;
        maximumOutput = config.maximumOutput;
    }

    /**
     * Compute one retained setpoint/PID/feedforward/output snapshot for this Plant cycle.
     */
    @Override
    public double update(double goal, double measurement, LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        requireClockIdentity(clock);
        if (hasAttemptedCycle && attemptedCycle == clock.cycle()) {
            if (retainedCycleFailure != null) throw retainedCycleFailure;
            if (Double.compare(goal, lastGoal) != 0
                    || Double.compare(measurement, lastMeasurement) != 0) {
                retainedCycleFailure = new IllegalStateException(
                        "Standard control was updated twice in LoopClock "
                        + "cycle " + clock.cycle() + " with different goal or measurement values; "
                        + "one Plant heartbeat must own one immutable control snapshot");
                throw retainedCycleFailure;
            }
            return lastOutput;
        }

        attemptedCycle = clock.cycle();
        hasAttemptedCycle = true;
        retainedCycleFailure = null;
        try {
            double result = updateOnce(goal, measurement, clock);
            lastOutput = result;
            return result;
        } catch (RuntimeException failure) {
            retainedCycleFailure = failure;
            throw failure;
        }
    }

    /** Clear every transient fact and release this runtime's clock binding. */
    @Override
    public void reset() {
        boundClock = null;
        initialized = false;
        skipNextDt = true;
        hasAttemptedCycle = false;
        retainedCycleFailure = null;
        clearRuntimeState();
    }

    /**
     * Explicitly reseed after calibration or another coordinate discontinuity without advancing a
     * profile, PID, source, or output. The next update consumes zero elapsed time.
     */
    void reseed(double measurement, LoopClock clock) {
        requireFinite("reseed(...) measurement", measurement);
        Objects.requireNonNull(clock, "clock");
        reset();
        boundClock = clock;
        seedFromMeasurement(measurement);
        skipNextDt = true;
    }

    /** True only when the retained setpoint is settled at exactly this Plant-supplied goal. */
    boolean setpointSettledAt(double goal) {
        return initialized
                && Double.isFinite(goal)
                && Double.compare(lastGoal, goal) == 0
                && setpointSettled;
    }

    boolean setpointSettled() {
        return initialized && setpointSettled;
    }

    /** Whether this clock cycle already completed successfully and must not write hardware again. */
    boolean hasSuccessfulResultForCycle(LoopClock clock) {
        return boundClock == clock
                && hasAttemptedCycle
                && attemptedCycle == clock.cycle()
                && retainedCycleFailure == null;
    }

    /** Whether this cycle already failed and its physical fail-stop cleanup was already attempted. */
    boolean hasRetainedFailureForCycle(LoopClock clock) {
        return boundClock == clock
                && hasAttemptedCycle
                && attemptedCycle == clock.cycle()
                && retainedCycleFailure != null;
    }

    /**
     * Retain a downstream failure that occurred after this cycle's successful calculation, so a
     * repeated Plant heartbeat cannot recompute or repeat the failed hardware effect.
     */
    void retainExternalFailureForCycle(LoopClock clock, RuntimeException failure) {
        Objects.requireNonNull(clock, "clock");
        Objects.requireNonNull(failure, "failure");
        if (boundClock != clock || !hasAttemptedCycle || attemptedCycle != clock.cycle()) {
            throw new IllegalStateException(
                    "Cannot retain a downstream StandardControl failure without this cycle's "
                            + "successful control snapshot");
        }
        if (retainedCycleFailure == null) retainedCycleFailure = failure;
    }

    /**
     * Clear transient controller/profile state after fail-stop while retaining this cycle's exact
     * failure, so a repeated heartbeat cannot recompute or repeat physical cleanup.
     */
    void resetAfterFailurePreservingCycle() {
        if (!hasAttemptedCycle || retainedCycleFailure == null) {
            reset();
            return;
        }
        LoopClock retainedClock = boundClock;
        long retainedCycle = attemptedCycle;
        RuntimeException retainedFailure = retainedCycleFailure;
        initialized = false;
        skipNextDt = true;
        clearRuntimeState();
        boundClock = retainedClock;
        attemptedCycle = retainedCycle;
        hasAttemptedCycle = true;
        retainedCycleFailure = retainedFailure;
    }

    double setpointPosition() {
        return setpointPosition;
    }

    double setpointVelocity() {
        return setpointVelocity;
    }

    double setpointAcceleration() {
        return setpointAcceleration;
    }

    boolean setpointPositionAvailable() {
        return setpointPositionAvailable;
    }

    boolean setpointAccelerationAvailable() {
        return setpointAccelerationAvailable;
    }

    double feedbackOutput() {
        return lastFeedback;
    }

    double feedforwardOutput() {
        return lastFeedforward;
    }

    double output() {
        return lastOutput;
    }

    boolean outputLimited() {
        return lastOutputLimited;
    }

    boolean integralGrowthBlocked() {
        return lastIntegralGrowthBlocked;
    }

    double integralContribution() {
        return integralContribution;
    }

    /** Emit retained facts only; diagnostics never advance control or sample voltage. */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        String p = (prefix == null || prefix.isEmpty()) ? "standardControl" : prefix;
        dbg.addData(p + ".class", "StandardControl")
                .addData(p + ".setpointMode", setpointMode.name())
                .addData(p + ".feedforwardMode", feedforwardMode.name())
                .addData(p + ".goal", lastGoal)
                .addData(p + ".measurement", lastMeasurement)
                .addData(p + ".setpoint.position", setpointPosition)
                .addData(p + ".setpoint.positionAvailable", setpointPositionAvailable)
                .addData(p + ".setpoint.velocity", setpointVelocity)
                .addData(p + ".setpoint.acceleration", setpointAcceleration)
                .addData(p + ".setpoint.accelerationAvailable", setpointAccelerationAvailable)
                .addData(p + ".setpoint.settled", setpointSettled)
                .addData(p + ".feedback.error", lastError)
                .addData(p + ".feedback.integral", integralContribution)
                .addData(p + ".feedback.unbounded", lastFeedbackUnbounded)
                .addData(p + ".feedback.output", lastFeedback)
                .addData(p + ".feedback.limited", lastFeedbackLimited)
                .addData(p + ".feedback.integralGrowthBlocked", lastIntegralGrowthBlocked)
                .addData(p + ".feedforward.output", lastFeedforward)
                .addData(p + ".voltage.sample", lastSupplyVoltage)
                .addData(p + ".voltage.scale", lastVoltageScale)
                .addData(p + ".output.beforeLimit", lastBeforeOutputLimit)
                .addData(p + ".output.command", lastOutput)
                .addData(p + ".output.limited", lastOutputLimited)
                .addData(p + ".output.minimum", minimumOutput)
                .addData(p + ".output.maximum", maximumOutput);
    }

    private double updateOnce(double goal, double measurement, LoopClock clock) {
        requireFinite("StandardControl.update(...) goal", goal);
        requireFinite("StandardControl.update(...) measurement", measurement);
        requireFinite("StandardControl.update(...) clock.dtSec()", clock.dtSec());
        if (clock.dtSec() < 0.0) {
            throw new IllegalArgumentException(
                    "StandardControl.update(...) clock.dtSec() must be >= 0; got "
                            + clock.dtSec());
        }

        boolean seedBoundary = !initialized || skipNextDt;
        boolean goalChanged = !Double.isFinite(lastGoal) || Double.compare(goal, lastGoal) != 0;
        double elapsedDtSec = seedBoundary ? 0.0 : clock.dtSec();
        double feedbackDtSec = goalChanged && !isProfiledMode(setpointMode)
                ? 0.0 : elapsedDtSec;
        SetpointSnapshot candidateSetpoint;
        if (seedBoundary) {
            candidateSetpoint = initialSetpoint(goal, measurement);
        } else if (goalChanged && isProfiledMode(setpointMode)) {
            // The prior goal owned the elapsed interval. Advance that retained trajectory to this
            // boundary first, then replan the new goal without charging it the preceding dt.
            SetpointSnapshot boundary = nextSetpoint(
                    lastGoal, elapsedDtSec, setpointPosition, setpointVelocity);
            candidateSetpoint = replanAtBoundary(goal, boundary);
        } else {
            candidateSetpoint = nextSetpoint(
                    goal, feedbackDtSec, setpointPosition, setpointVelocity);
        }

        double controlledSetpoint = isPositionMode(setpointMode)
                ? candidateSetpoint.position
                : candidateSetpoint.velocity;
        double error = finiteResult("control error", controlledSetpoint - measurement);
        double pTerm = finiteResult("proportional feedback", kP * error);
        double dTerm = 0.0;
        if (previousErrorAvailable && kD != 0.0 && feedbackDtSec > 0.0) {
            double errorDelta = finiteResult("feedback error delta", error - previousError);
            dTerm = finiteResult(
                    "derivative feedback", kD * (errorDelta / feedbackDtSec));
        }

        double candidateIntegral = integralContribution;
        if (kI != 0.0 && feedbackDtSec > 0.0) {
            double increment = finiteResult(
                    "integral feedback increment", kI * error * feedbackDtSec);
            candidateIntegral = finiteResult(
                    "integral feedback contribution", integralContribution + increment);
            candidateIntegral = MathUtil.clamp(candidateIntegral, integralMin, integralMax);
        }

        double voltageSample = Double.NaN;
        double voltageScale = 1.0;
        if (supplyVoltage != null) {
            voltageSample = supplyVoltage.getAsDouble(clock);
            if (Double.isFinite(voltageSample) && voltageSample > 0.0) {
                double denominator = Math.max(voltageSample, minimumVoltage);
                voltageScale = Math.min(referenceVoltage / denominator, maximumVoltageScale);
                voltageScale = finiteResult("voltage compensation scale", voltageScale);
            }
        }

        double antiWindupIntegral = antiWindupIntegral(
                candidateSetpoint,
                pTerm,
                dTerm,
                integralContribution,
                candidateIntegral,
                voltageScale);
        boolean blocked = Double.compare(antiWindupIntegral, candidateIntegral) != 0;
        Evaluation evaluation = evaluate(candidateSetpoint, pTerm, dTerm,
                antiWindupIntegral, voltageScale);

        // Commit the complete state only after every calculation and dynamic source succeeds.
        setpointPosition = candidateSetpoint.position;
        setpointVelocity = candidateSetpoint.velocity;
        setpointAcceleration = candidateSetpoint.acceleration;
        setpointPositionAvailable = candidateSetpoint.positionAvailable;
        setpointAccelerationAvailable = candidateSetpoint.accelerationAvailable;
        setpointSettled = candidateSetpoint.settled;
        initialized = true;
        lastGoal = goal;
        lastMeasurement = measurement;
        integralContribution = evaluation.integral;
        previousError = error;
        previousErrorAvailable = true;
        lastError = error;
        lastFeedbackUnbounded = evaluation.feedbackUnbounded;
        lastFeedback = evaluation.feedback;
        lastFeedforward = evaluation.feedforward;
        lastSupplyVoltage = voltageSample;
        lastVoltageScale = evaluation.voltageScale;
        lastBeforeOutputLimit = evaluation.beforeOutputLimit;
        lastFeedbackLimited = evaluation.feedbackLimited;
        lastOutputLimited = evaluation.outputLimited;
        lastIntegralGrowthBlocked = blocked;
        skipNextDt = false;
        return evaluation.output;
    }

    private Evaluation evaluate(SetpointSnapshot setpoint,
                                double pTerm,
                                double dTerm,
                                double integral,
                                double voltageScale) {
        double feedbackUnbounded = finiteResult(
                "combined PID feedback", pTerm + integral + dTerm);
        double feedback = MathUtil.clamp(feedbackUnbounded, feedbackMin, feedbackMax);
        double feedforward = feedforward(setpoint);
        double nominal = finiteResult(
                "combined feedback and feedforward", feedback + feedforward);
        double beforeLimit = finiteResult(
                "voltage-compensated control output", nominal * voltageScale);
        double output = MathUtil.clamp(beforeLimit, minimumOutput, maximumOutput);
        return new Evaluation(
                integral,
                feedbackUnbounded,
                feedback,
                feedforward,
                voltageScale,
                beforeLimit,
                output,
                Double.compare(feedbackUnbounded, feedback) != 0,
                Double.compare(beforeLimit, output) != 0);
    }

    /**
     * Clamp a proposed integral advance at the first feedback/final-output saturation boundary.
     * Existing integral may unwind away from a saturated side, but it cannot grow farther into it.
     */
    private double antiWindupIntegral(SetpointSnapshot setpoint,
                                      double pTerm,
                                      double dTerm,
                                      double retainedIntegral,
                                      double candidateIntegral,
                                      double voltageScale) {
        if (Double.compare(retainedIntegral, candidateIntegral) == 0) {
            return candidateIntegral;
        }

        double feedforward = feedforward(setpoint);
        double feedbackBase = finiteResult("non-integral feedback", pTerm + dTerm);
        if (candidateIntegral > retainedIntegral) {
            double outputFeedbackCeiling = finiteResult(
                    "anti-windup output feedback ceiling",
                    maximumOutput / voltageScale - feedforward);
            double feedbackCeiling = Math.min(feedbackMax, outputFeedbackCeiling);
            double integralCeiling = finiteResult(
                    "anti-windup integral ceiling", feedbackCeiling - feedbackBase);
            if (candidateIntegral > integralCeiling) {
                return retainedIntegral < integralCeiling
                        ? integralCeiling
                        : retainedIntegral;
            }
        } else {
            double outputFeedbackFloor = finiteResult(
                    "anti-windup output feedback floor",
                    minimumOutput / voltageScale - feedforward);
            double feedbackFloor = Math.max(feedbackMin, outputFeedbackFloor);
            double integralFloor = finiteResult(
                    "anti-windup integral floor", feedbackFloor - feedbackBase);
            if (candidateIntegral < integralFloor) {
                return retainedIntegral > integralFloor
                        ? integralFloor
                        : retainedIntegral;
            }
        }
        return candidateIntegral;
    }

    private double feedforward(SetpointSnapshot setpoint) {
        if (feedforwardMode == FeedforwardMode.NONE) return 0.0;

        double velocity = setpoint.velocity;
        double acceleration = setpoint.accelerationAvailable ? setpoint.acceleration : 0.0;
        double sign = velocity == 0.0 ? 0.0 : Math.copySign(1.0, velocity);
        double motion = finiteResult("motion feedforward",
                kS * sign + kV * velocity + kA * acceleration);
        if (feedforwardMode == FeedforwardMode.MOTION) return motion;
        if (feedforwardMode == FeedforwardMode.LIFT) {
            return finiteResult("lift feedforward", motion + kG);
        }

        double angle = finiteResult("arm feedforward angle",
                (setpoint.position - positionAtMaximumGravity) * radiansPerPlantUnit);
        double gravity = finiteResult("arm gravity feedforward", kG * Math.cos(angle));
        return finiteResult("arm feedforward", motion + gravity);
    }

    private SetpointSnapshot nextSetpoint(double goal,
                                          double dtSec,
                                          double priorPosition,
                                          double priorVelocity) {
        if (setpointMode == SetpointMode.POSITION_DIRECT) {
            return new SetpointSnapshot(goal, 0.0, 0.0, true, true, true);
        }
        if (setpointMode == SetpointMode.VELOCITY_DIRECT) {
            return new SetpointSnapshot(Double.NaN, goal, Double.NaN,
                    false, false, true);
        }
        if (setpointMode == SetpointMode.VELOCITY_ACCELERATION_LIMITED) {
            return nextAccelerationLimitedVelocity(goal, dtSec, priorVelocity);
        }
        return nextTrapezoidalPosition(goal, dtSec, priorPosition, priorVelocity);
    }

    /** Replan a changed profiled goal at the current boundary without integrating another step. */
    private SetpointSnapshot replanAtBoundary(double goal, SetpointSnapshot boundary) {
        if (setpointMode == SetpointMode.VELOCITY_ACCELERATION_LIMITED) {
            double difference = finiteResult(
                    "velocity setpoint replan difference", goal - boundary.velocity);
            double acceleration = difference == 0.0
                    ? 0.0 : Math.copySign(maximumAcceleration, difference);
            return new SetpointSnapshot(
                    Double.NaN,
                    boundary.velocity,
                    acceleration,
                    false,
                    true,
                    Double.compare(boundary.velocity, goal) == 0);
        }
        if (setpointMode == SetpointMode.POSITION_TRAPEZOIDAL) {
            double acceleration = positionProfileAcceleration(
                    goal, boundary.position, boundary.velocity);
            boolean settled = Double.compare(boundary.position, goal) == 0
                    && boundary.velocity == 0.0;
            return new SetpointSnapshot(
                    boundary.position, boundary.velocity, acceleration, true, true, settled);
        }
        return nextSetpoint(goal, 0.0, boundary.position, boundary.velocity);
    }

    /** Construct the first transactional snapshot without mutating retained profile state. */
    private SetpointSnapshot initialSetpoint(double goal, double measurement) {
        if (setpointMode == SetpointMode.POSITION_DIRECT) {
            return new SetpointSnapshot(goal, 0.0, 0.0, true, true, true);
        }
        if (setpointMode == SetpointMode.POSITION_TRAPEZOIDAL) {
            return new SetpointSnapshot(measurement, 0.0, 0.0,
                    true, true, Double.compare(measurement, goal) == 0);
        }
        if (setpointMode == SetpointMode.VELOCITY_DIRECT) {
            return new SetpointSnapshot(Double.NaN, goal, Double.NaN,
                    false, false, true);
        }
        return new SetpointSnapshot(Double.NaN, measurement, 0.0,
                false, true, Double.compare(measurement, goal) == 0);
    }

    private SetpointSnapshot nextAccelerationLimitedVelocity(double goal,
                                                              double dtSec,
                                                              double priorVelocity) {
        if (dtSec <= 0.0) {
            boolean settled = Double.compare(priorVelocity, goal) == 0;
            return new SetpointSnapshot(Double.NaN, priorVelocity, 0.0,
                    false, true, settled);
        }
        double difference = finiteResult("velocity setpoint difference", goal - priorVelocity);
        double maximumDelta = finiteResult(
                "velocity setpoint maximum delta", maximumAcceleration * dtSec);
        double delta = MathUtil.clamp(difference, -maximumDelta, maximumDelta);
        double velocity = finiteResult("velocity setpoint", priorVelocity + delta);
        if (Math.abs(difference) <= maximumDelta) velocity = goal;
        double acceleration = finiteResult(
                "velocity setpoint acceleration", (velocity - priorVelocity) / dtSec);
        return new SetpointSnapshot(Double.NaN, velocity, acceleration,
                false, true, Double.compare(velocity, goal) == 0);
    }

    private SetpointSnapshot nextTrapezoidalPosition(double goal,
                                                     double dtSec,
                                                     double priorPosition,
                                                     double priorVelocity) {
        if (dtSec <= 0.0) {
            boolean settled = Double.compare(priorPosition, goal) == 0
                    && priorVelocity == 0.0;
            return new SetpointSnapshot(priorPosition, priorVelocity, 0.0,
                    true, true, settled);
        }

        double error = finiteResult("position profile error", goal - priorPosition);
        if (error == 0.0 && priorVelocity == 0.0) {
            return new SetpointSnapshot(goal, 0.0, 0.0, true, true, true);
        }

        double direction = error == 0.0
                ? -Math.copySign(1.0, priorVelocity)
                : Math.copySign(1.0, error);
        double velocityTowardGoal = priorVelocity * direction;
        double nextVelocity;
        if (velocityTowardGoal < 0.0) {
            nextVelocity = finiteResult(
                    "position profile velocity",
                    priorVelocity + direction * maximumAcceleration * dtSec);
            if (priorVelocity != 0.0
                    && Math.signum(nextVelocity) != Math.signum(priorVelocity)) {
                nextVelocity = 0.0;
            }
        } else {
            double accelerationStep = finiteResult(
                    "position profile acceleration step", maximumAcceleration * dtSec);
            double distance = Math.abs(error);
            double discriminant = finiteResult(
                    "position profile discrete stopping discriminant",
                    accelerationStep * accelerationStep
                            - 4.0 * accelerationStep * velocityTowardGoal
                            + 8.0 * maximumAcceleration * distance);
            double safeNextVelocity = discriminant <= 0.0
                    ? 0.0
                    : finiteResult(
                            "position profile safe next velocity",
                            0.5 * (-accelerationStep + Math.sqrt(discriminant)));
            safeNextVelocity = Math.max(0.0, safeNextVelocity);
            double desiredVelocity = Math.min(maximumVelocity, safeNextVelocity);
            double nextVelocityTowardGoal = velocityTowardGoal < desiredVelocity
                    ? Math.min(desiredVelocity, velocityTowardGoal + accelerationStep)
                    : Math.max(desiredVelocity, velocityTowardGoal - accelerationStep);
            nextVelocity = finiteResult(
                    "position profile velocity", direction * nextVelocityTowardGoal);
        }

        nextVelocity = MathUtil.clamp(nextVelocity, -maximumVelocity, maximumVelocity);
        double actualAcceleration = finiteResult(
                "position profile acceleration", (nextVelocity - priorVelocity) / dtSec);
        double nextPosition = finiteResult(
                "position profile position",
                priorPosition + 0.5 * (priorVelocity + nextVelocity) * dtSec);

        boolean crossedGoal = error != 0.0
                && (goal - nextPosition) * error <= 0.0;
        boolean canStopThisCycle = Math.abs(priorVelocity)
                <= maximumAcceleration * dtSec;
        if (crossedGoal && canStopThisCycle) {
            nextPosition = goal;
            nextVelocity = 0.0;
            actualAcceleration = finiteResult(
                    "position profile terminal acceleration",
                    -priorVelocity / dtSec);
        }

        boolean settled = Double.compare(nextPosition, goal) == 0 && nextVelocity == 0.0;
        return new SetpointSnapshot(nextPosition, nextVelocity, actualAcceleration,
                true, true, settled);
    }

    /** Select the instantaneous acceleration for a newly arrived position goal. */
    private double positionProfileAcceleration(double goal,
                                               double position,
                                               double velocity) {
        double error = finiteResult("position profile replan error", goal - position);
        if (error == 0.0 && velocity == 0.0) return 0.0;
        double direction = error == 0.0
                ? -Math.copySign(1.0, velocity)
                : Math.copySign(1.0, error);
        double velocityTowardGoal = velocity * direction;
        if (velocityTowardGoal < 0.0) return direction * maximumAcceleration;
        double stoppingDistance = finiteResult(
                "position profile replan stopping distance",
                velocityTowardGoal * velocityTowardGoal / (2.0 * maximumAcceleration));
        if (stoppingDistance >= Math.abs(error)) return -direction * maximumAcceleration;
        if (velocityTowardGoal < maximumVelocity) return direction * maximumAcceleration;
        if (velocityTowardGoal > maximumVelocity) return -direction * maximumAcceleration;
        return 0.0;
    }

    private void requireClockIdentity(LoopClock clock) {
        if (boundClock == null) {
            boundClock = clock;
        } else if (boundClock != clock) {
            throw new IllegalStateException(
                    "StandardControl is bound to one LoopClock until reset(); received a different "
                            + "clock identity");
        }
    }

    private void seedFromMeasurement(double measurement) {
        initialized = true;
        if (isPositionMode(setpointMode)) {
            setpointPosition = measurement;
            setpointVelocity = 0.0;
            setpointAcceleration = 0.0;
            setpointPositionAvailable = true;
            setpointAccelerationAvailable = true;
        } else {
            setpointPosition = Double.NaN;
            setpointVelocity = measurement;
            setpointAcceleration = setpointMode == SetpointMode.VELOCITY_DIRECT
                    ? Double.NaN : 0.0;
            setpointPositionAvailable = false;
            setpointAccelerationAvailable =
                    setpointMode == SetpointMode.VELOCITY_ACCELERATION_LIMITED;
        }
        setpointSettled = false;
        lastMeasurement = measurement;
    }

    private void clearRuntimeState() {
        lastGoal = Double.NaN;
        lastMeasurement = Double.NaN;
        setpointPosition = Double.NaN;
        setpointVelocity = Double.NaN;
        setpointAcceleration = Double.NaN;
        setpointPositionAvailable = false;
        setpointAccelerationAvailable = false;
        setpointSettled = false;
        integralContribution = 0.0;
        previousError = 0.0;
        previousErrorAvailable = false;
        lastError = Double.NaN;
        lastFeedbackUnbounded = Double.NaN;
        lastFeedback = Double.NaN;
        lastFeedforward = Double.NaN;
        lastSupplyVoltage = Double.NaN;
        lastVoltageScale = 1.0;
        lastBeforeOutputLimit = Double.NaN;
        lastOutput = Double.NaN;
        lastFeedbackLimited = false;
        lastOutputLimited = false;
        lastIntegralGrowthBlocked = false;
    }

    private static boolean isPositionMode(SetpointMode mode) {
        return mode == SetpointMode.POSITION_DIRECT
                || mode == SetpointMode.POSITION_TRAPEZOIDAL;
    }

    private static boolean isProfiledMode(SetpointMode mode) {
        return mode == SetpointMode.POSITION_TRAPEZOIDAL
                || mode == SetpointMode.VELOCITY_ACCELERATION_LIMITED;
    }

    private static void requirePositiveFinite(String name, double value) {
        requireFinite(name, value);
        if (value <= 0.0) {
            throw new IllegalArgumentException(name + " must be > 0; got " + value);
        }
    }

    private static void requireFiniteOrderedRange(String operation,
                                                  double minimum,
                                                  double maximum) {
        requireFinite(operation + " minimum", minimum);
        requireFinite(operation + " maximum", maximum);
        if (minimum > maximum) {
            throw new IllegalArgumentException(
                    operation + " requires minimum <= maximum; got minimum=" + minimum
                            + ", maximum=" + maximum);
        }
    }

    private static void requireFinite(String name, double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite; got " + value);
        }
    }

    private static double finiteResult(String name, double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalStateException(
                    "StandardControl produced non-finite " + name + " " + value);
        }
        return value;
    }
}
