package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Exclusive advanced tuning handle for one Plant-owned Phoenix standard controller.
 *
 * <p>The handle changes only the controller's complete gain snapshot. The bound Plant remains the
 * sole owner of target resolution, feedback sampling, heartbeat, output, and stop. Instances are
 * obtained through {@link StandardControlTunings}; ordinary robot code should keep checked-in
 * gains in its Plant recipe instead of retaining this live-tuning seam. The exclusive tuning
 * workflow calls mutating methods from the same owner-loop thread that updates the Plant, never
 * from a dashboard transport callback.</p>
 */
public final class StandardControlTuning {

    /** Controller domain proven by the completed Plant. */
    public enum Domain { POSITION, VELOCITY }

    /** Fixed setpoint topology selected by the Plant recipe. */
    public enum SetpointModel {
        POSITION_DIRECT,
        POSITION_TRAPEZOIDAL,
        VELOCITY_DIRECT,
        VELOCITY_ACCELERATION_LIMITED
    }

    /** Fixed feedforward topology selected by the Plant recipe. */
    public enum FeedforwardModel { NONE, MOTION, LIFT, ARM }

    /**
     * Immutable fixed facts which a tuning UI may display but cannot change.
     */
    public static final class Topology {
        private final Domain domain;
        private final SetpointModel setpointModel;
        private final FeedforwardModel feedforwardModel;
        private final double maximumVelocity;
        private final double maximumAcceleration;
        private final double positionAtMaximumGravity;
        private final double radiansPerPlantUnit;
        private final double integralMinimum;
        private final double integralMaximum;
        private final double feedbackMinimum;
        private final double feedbackMaximum;
        private final double outputMinimum;
        private final double outputMaximum;
        private final boolean voltageCompensated;
        private final double referenceVoltage;
        private final double minimumVoltage;
        private final double maximumVoltageScale;
        private final boolean kSActive;
        private final boolean kVActive;
        private final boolean kAActive;
        private final boolean kGActive;

        Topology(Domain domain,
                 SetpointModel setpointModel,
                 FeedforwardModel feedforwardModel,
                 double maximumVelocity,
                 double maximumAcceleration,
                 double positionAtMaximumGravity,
                 double radiansPerPlantUnit,
                 double integralMinimum,
                 double integralMaximum,
                 double feedbackMinimum,
                 double feedbackMaximum,
                 double outputMinimum,
                 double outputMaximum,
                 boolean voltageCompensated,
                 double referenceVoltage,
                 double minimumVoltage,
                 double maximumVoltageScale,
                 boolean kSActive,
                 boolean kVActive,
                 boolean kAActive,
                 boolean kGActive) {
            this.domain = Objects.requireNonNull(domain, "domain");
            this.setpointModel = Objects.requireNonNull(setpointModel, "setpointModel");
            this.feedforwardModel = Objects.requireNonNull(feedforwardModel, "feedforwardModel");
            this.maximumVelocity = maximumVelocity;
            this.maximumAcceleration = maximumAcceleration;
            this.positionAtMaximumGravity = positionAtMaximumGravity;
            this.radiansPerPlantUnit = radiansPerPlantUnit;
            this.integralMinimum = integralMinimum;
            this.integralMaximum = integralMaximum;
            this.feedbackMinimum = feedbackMinimum;
            this.feedbackMaximum = feedbackMaximum;
            this.outputMinimum = outputMinimum;
            this.outputMaximum = outputMaximum;
            this.voltageCompensated = voltageCompensated;
            this.referenceVoltage = referenceVoltage;
            this.minimumVoltage = minimumVoltage;
            this.maximumVoltageScale = maximumVoltageScale;
            this.kSActive = kSActive;
            this.kVActive = kVActive;
            this.kAActive = kAActive;
            this.kGActive = kGActive;
        }

        /** Return the fixed position or velocity domain. */
        public Domain domain() { return domain; }
        /** Return the fixed direct/profile setpoint model. */
        public SetpointModel setpointModel() { return setpointModel; }
        /** Return the fixed feedforward equation family. */
        public FeedforwardModel feedforwardModel() { return feedforwardModel; }
        /** Whether the setpoint recipe has a finite maximum velocity. */
        public boolean hasMaximumVelocity() { return Double.isFinite(maximumVelocity); }
        /** Return the fixed maximum velocity; throws if unavailable. */
        public double maximumVelocity() {
            return availableValue(hasMaximumVelocity(), "maximum velocity", maximumVelocity);
        }
        /** Whether the setpoint recipe has a finite maximum acceleration. */
        public boolean hasMaximumAcceleration() { return Double.isFinite(maximumAcceleration); }
        /** Return the fixed maximum acceleration; throws if unavailable. */
        public double maximumAcceleration() {
            return availableValue(
                    hasMaximumAcceleration(), "maximum acceleration", maximumAcceleration);
        }
        /** Whether fixed arm-gravity geometry is present. */
        public boolean hasArmGeometry() { return feedforwardModel == FeedforwardModel.ARM; }
        /** Return the fixed Plant position of maximum gravity; throws if unavailable. */
        public double positionAtMaximumGravity() {
            return availableValue(
                    hasArmGeometry(), "arm position at maximum gravity", positionAtMaximumGravity);
        }
        /** Return the fixed arm radians per Plant unit; throws if unavailable. */
        public double radiansPerPlantUnit() {
            return availableValue(
                    hasArmGeometry(), "arm radians per Plant unit", radiansPerPlantUnit);
        }
        /** Whether the integral contribution has a finite lower limit. */
        public boolean hasIntegralMinimum() { return Double.isFinite(integralMinimum); }
        /** Return the integral lower limit; throws when unbounded. */
        public double integralMinimum() {
            return availableValue(hasIntegralMinimum(), "integral minimum", integralMinimum);
        }
        /** Whether the integral contribution has a finite upper limit. */
        public boolean hasIntegralMaximum() { return Double.isFinite(integralMaximum); }
        /** Return the integral upper limit; throws when unbounded. */
        public double integralMaximum() {
            return availableValue(hasIntegralMaximum(), "integral maximum", integralMaximum);
        }
        /** Whether the combined feedback contribution has a finite lower limit. */
        public boolean hasFeedbackMinimum() { return Double.isFinite(feedbackMinimum); }
        /** Return the feedback lower limit; throws when unbounded. */
        public double feedbackMinimum() {
            return availableValue(hasFeedbackMinimum(), "feedback minimum", feedbackMinimum);
        }
        /** Whether the combined feedback contribution has a finite upper limit. */
        public boolean hasFeedbackMaximum() { return Double.isFinite(feedbackMaximum); }
        /** Return the feedback upper limit; throws when unbounded. */
        public double feedbackMaximum() {
            return availableValue(hasFeedbackMaximum(), "feedback maximum", feedbackMaximum);
        }
        /** Return the fixed final normalized-power lower limit. */
        public double outputMinimum() { return outputMinimum; }
        /** Return the fixed final normalized-power upper limit. */
        public double outputMaximum() { return outputMaximum; }
        /** Whether a fixed voltage-compensation policy is configured. */
        public boolean isVoltageCompensated() { return voltageCompensated; }
        /** Return the fixed reference voltage; throws if unavailable. */
        public double referenceVoltage() {
            return availableValue(voltageCompensated, "reference voltage", referenceVoltage);
        }
        /** Return the fixed minimum compensation voltage; throws if unavailable. */
        public double minimumVoltage() {
            return availableValue(voltageCompensated, "minimum voltage", minimumVoltage);
        }
        /** Return the fixed maximum voltage scale; throws if unavailable. */
        public double maximumVoltageScale() {
            return availableValue(voltageCompensated, "maximum voltage scale", maximumVoltageScale);
        }
        /** Whether static motion gain {@code kS} is an active candidate field. */
        public boolean hasKS() { return kSActive; }
        /** Whether velocity gain {@code kV} is an active candidate field. */
        public boolean hasKV() { return kVActive; }
        /** Whether acceleration gain {@code kA} is an active candidate field. */
        public boolean hasKA() { return kAActive; }
        /** Whether gravity gain {@code kG} is an active candidate field. */
        public boolean hasKG() { return kGActive; }

        private static double availableValue(boolean available, String name, double value) {
            if (!available) {
                throw new IllegalStateException(name + " is not configured in this topology");
            }
            return value;
        }
    }

    /** Immutable complete editable gain snapshot for this handle's fixed topology. */
    public static final class Parameters {
        private final Topology topology;
        private final double kP;
        private final double kI;
        private final double kD;
        private final double kS;
        private final double kV;
        private final double kA;
        private final double kG;

        Parameters(Topology topology,
                   double kP,
                   double kI,
                   double kD,
                   double kS,
                   double kV,
                   double kA,
                   double kG) {
            this.topology = Objects.requireNonNull(topology, "topology");
            requireFinite("kP", kP);
            requireFinite("kI", kI);
            requireFinite("kD", kD);
            requireActiveFinite(topology.hasKS(), "kS", kS);
            requireActiveFinite(topology.hasKV(), "kV", kV);
            requireActiveFinite(topology.hasKA(), "kA", kA);
            requireActiveFinite(topology.hasKG(), "kG", kG);
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kS = topology.hasKS() ? kS : Double.NaN;
            this.kV = topology.hasKV() ? kV : Double.NaN;
            this.kA = topology.hasKA() ? kA : Double.NaN;
            this.kG = topology.hasKG() ? kG : Double.NaN;
        }

        /** Return the fixed controller domain tag. */
        public Domain domain() { return topology.domain(); }
        /** Return the fixed setpoint-model tag. */
        public SetpointModel setpointModel() { return topology.setpointModel(); }
        /** Return the fixed feedforward-model tag. */
        public FeedforwardModel feedforwardModel() { return topology.feedforwardModel(); }
        /** Return proportional feedback gain. */
        public double kP() { return kP; }
        /** Return integral feedback gain. */
        public double kI() { return kI; }
        /** Return derivative feedback gain. */
        public double kD() { return kD; }
        /** Whether {@link #kS()} is active and available. */
        public boolean hasKS() { return topology.hasKS(); }
        /** Whether {@link #kV()} is active and available. */
        public boolean hasKV() { return topology.hasKV(); }
        /** Whether {@link #kA()} is active and available. */
        public boolean hasKA() { return topology.hasKA(); }
        /** Whether {@link #kG()} is active and available. */
        public boolean hasKG() { return topology.hasKG(); }
        /** Return static gain; throws when inactive. */
        public double kS() { return activeValue(hasKS(), "kS", kS); }
        /** Return velocity gain; throws when inactive. */
        public double kV() { return activeValue(hasKV(), "kV", kV); }
        /** Return acceleration gain; throws when inactive. */
        public double kA() { return activeValue(hasKA(), "kA", kA); }
        /** Return gravity gain; throws when inactive. */
        public double kG() { return activeValue(hasKG(), "kG", kG); }

        boolean belongsTo(Topology expected) { return topology == expected; }

        /** Return a complete candidate with new PID feedback gains. */
        public Parameters withFeedbackPid(double kP, double kI, double kD) {
            return new Parameters(topology, kP, kI, kD, kS, kV, kA, kG);
        }

        /** Return a candidate for a motion topology with only an active velocity gain. */
        public Parameters withMotionFeedforward(double kV) {
            requireShape(FeedforwardModel.MOTION, false, true, false, false,
                    "withMotionFeedforward(kV)");
            return new Parameters(topology, kP, kI, kD, kS, kV, kA, kG);
        }

        /** Return a candidate for a motion topology with active static and velocity gains. */
        public Parameters withMotionFeedforward(double kS, double kV) {
            requireShape(FeedforwardModel.MOTION, true, true, false, false,
                    "withMotionFeedforward(kS, kV)");
            return new Parameters(topology, kP, kI, kD, kS, kV, kA, kG);
        }

        /** Return a candidate for a motion topology with active static, velocity, and acceleration gains. */
        public Parameters withMotionFeedforward(double kS, double kV, double kA) {
            requireShape(FeedforwardModel.MOTION, true, true, true, false,
                    "withMotionFeedforward(kS, kV, kA)");
            return new Parameters(topology, kP, kI, kD, kS, kV, kA, kG);
        }

        /** Return a candidate for a gravity-only lift topology. */
        public Parameters withLiftFeedforward(double kG) {
            requireShape(FeedforwardModel.LIFT, false, false, false, true,
                    "withLiftFeedforward(kG)");
            return new Parameters(topology, kP, kI, kD, kS, kV, kA, kG);
        }

        /**
         * Return a candidate for a lift topology with gravity, static, and velocity gains.
         * Argument order matches {@code feedforwardFromLift(kG, kS, kV)}.
         */
        public Parameters withLiftFeedforward(double kG, double kS, double kV) {
            requireShape(FeedforwardModel.LIFT, true, true, false, true,
                    "withLiftFeedforward(kG, kS, kV)");
            return new Parameters(topology, kP, kI, kD, kS, kV, kA, kG);
        }

        /**
         * Return a candidate for a full-motion lift topology. Argument order matches
         * {@code feedforwardFromLift(kG, kS, kV, kA)}.
         */
        public Parameters withLiftFeedforward(double kG, double kS, double kV, double kA) {
            requireShape(FeedforwardModel.LIFT, true, true, true, true,
                    "withLiftFeedforward(kG, kS, kV, kA)");
            return new Parameters(topology, kP, kI, kD, kS, kV, kA, kG);
        }

        /** Return a candidate for a gravity-only arm topology. */
        public Parameters withArmFeedforward(double kG) {
            requireShape(FeedforwardModel.ARM, false, false, false, true,
                    "withArmFeedforward(kG)");
            return new Parameters(topology, kP, kI, kD, kS, kV, kA, kG);
        }

        /** Return a full-motion arm candidate with gravity first, then kS, kV, and kA. */
        public Parameters withArmFeedforward(double kG, double kS, double kV, double kA) {
            requireShape(FeedforwardModel.ARM, true, true, true, true,
                    "withArmFeedforward(kG, kS, kV, kA)");
            return new Parameters(topology, kP, kI, kD, kS, kV, kA, kG);
        }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Parameters)) return false;
            Parameters that = (Parameters) other;
            return topology == that.topology
                    && Double.compare(kP, that.kP) == 0
                    && Double.compare(kI, that.kI) == 0
                    && Double.compare(kD, that.kD) == 0
                    && Double.compare(kS, that.kS) == 0
                    && Double.compare(kV, that.kV) == 0
                    && Double.compare(kA, that.kA) == 0
                    && Double.compare(kG, that.kG) == 0;
        }

        @Override
        public int hashCode() {
            int result = System.identityHashCode(topology);
            result = 31 * result + hash(kP);
            result = 31 * result + hash(kI);
            result = 31 * result + hash(kD);
            result = 31 * result + hash(kS);
            result = 31 * result + hash(kV);
            result = 31 * result + hash(kA);
            result = 31 * result + hash(kG);
            return result;
        }

        @Override
        public String toString() {
            return "Parameters{domain=" + domain()
                    + ", setpointModel=" + setpointModel()
                    + ", feedforwardModel=" + feedforwardModel()
                    + ", kP=" + kP + ", kI=" + kI + ", kD=" + kD
                    + ", kS=" + (hasKS() ? Double.toString(kS) : "inactive")
                    + ", kV=" + (hasKV() ? Double.toString(kV) : "inactive")
                    + ", kA=" + (hasKA() ? Double.toString(kA) : "inactive")
                    + ", kG=" + (hasKG() ? Double.toString(kG) : "inactive") + '}';
        }

        private void requireShape(FeedforwardModel model,
                                  boolean kSActive,
                                  boolean kVActive,
                                  boolean kAActive,
                                  boolean kGActive,
                                  String operation) {
            if (feedforwardModel() != model
                    || hasKS() != kSActive
                    || hasKV() != kVActive
                    || hasKA() != kAActive
                    || hasKG() != kGActive) {
                throw new IllegalStateException(operation + " does not match this controller's "
                        + "fixed feedforward topology " + feedforwardModel()
                        + " (active gains: kS=" + hasKS() + ", kV=" + hasKV()
                        + ", kA=" + hasKA() + ", kG=" + hasKG() + ")");
            }
        }

        private static void requireActiveFinite(boolean active, String name, double value) {
            if (active) requireFinite(name, value);
        }

        private static void requireFinite(String name, double value) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException(name + " must be finite; got " + value);
            }
        }

        private static double activeValue(boolean active, String name, double value) {
            if (!active) {
                throw new IllegalStateException(name + " is not active in this controller topology");
            }
            return value;
        }

        private static int hash(double value) {
            long bits = Double.doubleToLongBits(value);
            return (int) (bits ^ (bits >>> 32));
        }
    }

    /** Immutable retained control evidence; reading it never samples hardware or advances control. */
    public static final class Evidence {
        private final boolean available;
        private final long cycle;
        private final double goal;
        private final double measurement;
        private final boolean setpointPositionAvailable;
        private final double setpointPosition;
        private final double setpointVelocity;
        private final boolean setpointAccelerationAvailable;
        private final double setpointAcceleration;
        private final boolean setpointSettled;
        private final double feedbackError;
        private final double feedbackIntegral;
        private final double feedbackOutput;
        private final double feedforwardOutput;
        private final boolean voltageScaleAvailable;
        private final double voltageScale;
        private final double outputBeforeLimit;
        private final double output;
        private final boolean feedbackLimited;
        private final boolean outputLimited;
        private final boolean integralGrowthBlocked;

        Evidence(boolean available,
                 long cycle,
                 double goal,
                 double measurement,
                 boolean setpointPositionAvailable,
                 double setpointPosition,
                 double setpointVelocity,
                 boolean setpointAccelerationAvailable,
                 double setpointAcceleration,
                 boolean setpointSettled,
                 double feedbackError,
                 double feedbackIntegral,
                 double feedbackOutput,
                 double feedforwardOutput,
                 boolean voltageScaleAvailable,
                 double voltageScale,
                 double outputBeforeLimit,
                 double output,
                 boolean feedbackLimited,
                 boolean outputLimited,
                 boolean integralGrowthBlocked) {
            this.available = available;
            this.cycle = cycle;
            this.goal = goal;
            this.measurement = measurement;
            this.setpointPositionAvailable = setpointPositionAvailable;
            this.setpointPosition = setpointPosition;
            this.setpointVelocity = setpointVelocity;
            this.setpointAccelerationAvailable = setpointAccelerationAvailable;
            this.setpointAcceleration = setpointAcceleration;
            this.setpointSettled = setpointSettled;
            this.feedbackError = feedbackError;
            this.feedbackIntegral = feedbackIntegral;
            this.feedbackOutput = feedbackOutput;
            this.feedforwardOutput = feedforwardOutput;
            this.voltageScaleAvailable = voltageScaleAvailable;
            this.voltageScale = voltageScale;
            this.outputBeforeLimit = outputBeforeLimit;
            this.output = output;
            this.feedbackLimited = feedbackLimited;
            this.outputLimited = outputLimited;
            this.integralGrowthBlocked = integralGrowthBlocked;
        }

        /** Whether one complete controller evaluation, rather than only a reseed, is retained. */
        public boolean isAvailable() { return available; }
        /** Whether {@link #cycle()} identifies a retained evaluation attempt. */
        public boolean hasCycle() { return cycle >= 0L; }
        /** Return its LoopClock cycle, or {@code -1} before any retained attempt. */
        public long cycle() { return cycle; }
        /** Whether a finite applied goal is cached. */
        public boolean hasGoal() { return Double.isFinite(goal); }
        /** Return the cached applied goal; throws if unavailable. */
        public double goal() { return evidenceValue(hasGoal(), "goal", goal); }
        /** Whether a finite measurement is cached, including after a reseed. */
        public boolean hasMeasurement() { return Double.isFinite(measurement); }
        /** Return the cached measurement; throws if unavailable. */
        public double measurement() {
            return evidenceValue(hasMeasurement(), "measurement", measurement);
        }
        /** Whether a position setpoint is meaningful in this topology/state. */
        public boolean hasSetpointPosition() { return setpointPositionAvailable; }
        /** Return cached setpoint position; throws if unavailable. */
        public double setpointPosition() {
            return evidenceValue(
                    setpointPositionAvailable, "setpoint position", setpointPosition);
        }
        /** Whether a finite velocity setpoint is cached. */
        public boolean hasSetpointVelocity() { return Double.isFinite(setpointVelocity); }
        /** Return cached setpoint velocity; throws if unavailable. */
        public double setpointVelocity() {
            return evidenceValue(hasSetpointVelocity(), "setpoint velocity", setpointVelocity);
        }
        /** Whether acceleration evidence is meaningful in this topology/state. */
        public boolean hasSetpointAcceleration() { return setpointAccelerationAvailable; }
        /** Return cached setpoint acceleration; throws if unavailable. */
        public double setpointAcceleration() {
            return evidenceValue(
                    setpointAccelerationAvailable, "setpoint acceleration", setpointAcceleration);
        }
        /** Whether a retained evaluation proves that its setpoint settled at the applied goal. */
        public boolean isSetpointSettled() { return available && setpointSettled; }
        /** Return cached controlled-setpoint error; throws without a complete evaluation. */
        public double feedbackError() {
            return evidenceValue(available, "feedback error", feedbackError);
        }
        /** Return cached integral contribution; throws without a complete evaluation. */
        public double feedbackIntegral() {
            return evidenceValue(available, "feedback integral", feedbackIntegral);
        }
        /** Return combined PID output after fixed feedback limits. */
        public double feedbackOutput() {
            return evidenceValue(available, "feedback output", feedbackOutput);
        }
        /** Return feedforward output from the same retained setpoint snapshot. */
        public double feedforwardOutput() {
            return evidenceValue(available, "feedforward output", feedforwardOutput);
        }
        /** Whether a voltage-compensation scale is available for the retained evaluation. */
        public boolean hasVoltageScale() { return available && voltageScaleAvailable; }
        /** Return retained voltage scale; throws if compensation/evidence is unavailable. */
        public double voltageScale() {
            return evidenceValue(hasVoltageScale(), "voltage scale", voltageScale);
        }
        /** Return combined voltage-compensated output before the final output-power limit. */
        public double outputBeforeLimit() {
            return evidenceValue(available, "output before final limit", outputBeforeLimit);
        }
        /** Return final retained normalized output. */
        public double output() { return evidenceValue(available, "final output", output); }
        /** Whether the retained PID feedback was limited; false without an evaluation. */
        public boolean isFeedbackLimited() { return available && feedbackLimited; }
        /** Whether the retained final output was limited; false without an evaluation. */
        public boolean isOutputLimited() { return available && outputLimited; }
        /** Whether anti-windup blocked integral growth; false without an evaluation. */
        public boolean isIntegralGrowthBlocked() { return available && integralGrowthBlocked; }

        private static double evidenceValue(boolean available, String name, double value) {
            if (!available || !Double.isFinite(value)) {
                throw new IllegalStateException(name + " is unavailable in this cached evidence");
            }
            return value;
        }
    }

    private final Domain domain;
    private final MappedVelocityPlant velocityPlant;
    private final MappedPositionPlant positionPlant;
    private final PositionPlantTuning positionTuning;
    private final StandardControl control;
    private final Topology topology;
    private final Parameters initialParameters;

    StandardControlTuning(MappedVelocityPlant plant, StandardControl control) {
        this.domain = Domain.VELOCITY;
        this.velocityPlant = Objects.requireNonNull(plant, "plant");
        this.positionPlant = null;
        this.positionTuning = null;
        this.control = Objects.requireNonNull(control, "control");
        this.topology = control.tuningTopology();
        this.initialParameters = control.tuningParameters();
    }

    StandardControlTuning(MappedPositionPlant plant, StandardControl control) {
        this.domain = Domain.POSITION;
        this.velocityPlant = null;
        this.positionPlant = Objects.requireNonNull(plant, "plant");
        this.positionTuning = PositionPlantTunings.claim(plant);
        this.control = Objects.requireNonNull(control, "control");
        this.topology = control.tuningTopology();
        this.initialParameters = control.tuningParameters();
    }

    /** Return the proven position or velocity controller domain. */
    public Domain domain() { return domain; }
    /** Return immutable fixed topology/policy facts. */
    public Topology topology() { return topology; }
    /** Return the exact construction-time gain snapshot. */
    public Parameters initialParameters() { return initialParameters; }
    /** Return the complete currently applied gain snapshot. */
    public Parameters appliedParameters() { return control.tuningParameters(); }
    /** Return cached controller evidence without sampling or advancing the Plant. */
    public Evidence evidence() { return control.tuningEvidence(); }

    /** Return the bound Plant's current declared target range without sampling. */
    public ScalarRange targetRange() {
        return domain == Domain.VELOCITY
                ? velocityPlant.tuningTargetRange()
                : positionTuning.targetRange();
    }

    /** Whether the final resolver is one literal exact graph-owned command. */
    public boolean hasExactCommandTarget() {
        return domain == Domain.VELOCITY
                ? velocityPlant.hasExactTuningCommandTarget()
                : positionTuning.hasExactCommandTarget();
    }

    /**
     * Atomically install a complete gain candidate and reseed from the Plant's latest finite
     * cached measurement. This method does not sample, update, or write the Plant.
     */
    public void applyAndReseed(Parameters candidate, LoopClock clock) {
        Objects.requireNonNull(candidate, "candidate");
        Objects.requireNonNull(clock, "clock");
        if (domain == Domain.VELOCITY) {
            velocityPlant.requireActiveForTuning("StandardControlTuning.applyAndReseed(...)");
        } else {
            positionPlant.requireActiveForTuning("StandardControlTuning.applyAndReseed(...)");
        }
        double measurement = cachedFiniteMeasurement("applyAndReseed(...)");
        control.applyTuningParametersAndReseed(candidate, topology, measurement, clock);
    }

    /** Restore the construction-time gains and reseed from the latest cached measurement. */
    public void restoreInitialAndReseed(LoopClock clock) {
        applyAndReseed(initialParameters, clock);
    }

    /**
     * For a position controller, sample and request the current exact position before the first
     * normal position realization, then reseed controller state without actuating. Calibration
     * search heartbeats may already have run. ASSUME_CURRENT references are established from that
     * same sample; a NEEDS_REFERENCE Plant is eligible after its reference Task succeeds.
     *
     * @return the finite requested hold position in Plant units
     */
    public double prepareHoldAtCurrent(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        if (domain != Domain.POSITION) {
            throw new IllegalStateException(
                    "prepareHoldAtCurrent(...) requires a standard position-control Plant");
        }
        double measurement = positionTuning.prepareHoldAtCurrent(clock);
        control.reseed(measurement, clock);
        return measurement;
    }

    /**
     * Stage one same-cycle position recovery hold inside the allowed physical envelope and reseed
     * the standard controller from the measured position without actuating.
     */
    public PositionPlantTuning.RecoveryHold prepareRecoveryHoldWithin(
            ScalarRange allowedPhysicalRange,
            LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        if (domain != Domain.POSITION) {
            throw new IllegalStateException(
                    "prepareRecoveryHoldWithin(...) requires a standard position-control Plant");
        }
        PositionPlantTuning.RecoveryHold hold = positionTuning.prepareRecoveryHoldWithin(
                Objects.requireNonNull(allowedPhysicalRange, "allowedPhysicalRange"), clock);
        control.reseed(hold.measurement(), clock);
        return hold;
    }

    private double cachedFiniteMeasurement(String operation) {
        double measurement = domain == Domain.VELOCITY
                ? velocityPlant.getMeasurement()
                : positionPlant.getMeasurement();
        if (!Double.isFinite(measurement)) {
            throw new IllegalStateException(operation + " requires the bound Plant's latest "
                    + "cached measurement to be finite; update the Plant first (or call "
                    + "prepareHoldAtCurrent(...) for eligible position Plants)");
        }
        return measurement;
    }
}
