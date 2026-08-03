package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetGate;
import edu.ftcphoenix.fw.actuation.PlantTargetResolver;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.actuation.ScalarRange;
import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;

/**
 * Beginner-friendly FTC boundary builder for wiring hardware into {@link Plant} instances.
 *
 * <p>The staged API is intentionally question-shaped. Required conceptual questions are answered
 * explicitly, while optional tuning only appears after the user deliberately enters a tuning branch.
 * For example, motor velocity and position control first ask who manages the loop, then ask the
 * domain-specific questions about target bounds, units, and, for position, reference policy.</p>
 *
 * <h2>Units convention</h2>
 *
 * <p>Position and velocity builders distinguish <b>plant units</b> from <b>native units</b>. Plant
 * units are what robot code, planners, ranges, references, tolerances, and
 * {@link Plant#getRequestedTarget()} use. Native units are what the selected hardware/control path
 * uses internally: motor ticks, ticks/sec, external encoder units, raw servo fractions, or a
 * caller-supplied feedback source. Public methods use plant units unless the method name
 * explicitly says {@code Native} or a controller-native unit such as {@code Ticks}. That means
 * methods like {@code bounded(...)}, {@code periodic(...)}, {@code positionTolerance(...)}, and
 * {@code velocityTolerance(...)} use plant units, while mapping methods such as
 * {@code scaleToNative(...)}, {@code rangeMapsToNative(...)}, and device methods such as
 * {@code devicePositionToleranceTicks(...)} take native/controller quantities explicitly.</p>
 *
 * <p>Velocity mappings are intentionally zero-preserving: plant velocity {@code 0.0} always maps
 * to native velocity {@code 0.0}. Phoenix therefore exposes velocity {@code nativeUnits()} and
 * {@code scaleToNative(...)} mappings, but not an offset-based velocity endpoint map.</p>
 *
 * <h2>Grouped hardware names</h2>
 *
 * <p>Every motor, standard-servo, or CR-servo command group requires nonblank, distinct configured
 * hardware names. Identity follows the FTC SDK lookup contract: surrounding whitespace is ignored
 * and case remains significant. Invalid members are rejected when added, before they enter the
 * private group or cause a new hardware effect. This is group-local command validation, not a
 * global ownership registry; a separately constructed Plant or a read-only feedback choice may
 * intentionally use the same configured device name.</p>
 *
 * <h2>Typical mechanism ownership</h2>
 *
 * <p>Ordinary FTC robot code uses these builders inside a mechanism/subsystem constructor. The
 * composition root passes {@code HardwareMap} plus a data-only mechanism config; the mechanism
 * defensively snapshots that config, constructs and privately retains its Plants, and owns their
 * update and stop lifecycle. The abbreviated fragments below show that constructor-owned Plant
 * construction. A completed-Plant injection constructor is reserved for a clearly labeled
 * hardware-neutral test, custom-adapter, portable-host, or advanced-assembly seam.</p>
 *
 * <pre>{@code
 * MechanismConfig cfg = config.copy();
 * this.lift = FtcActuators.plant(hardwareMap)
 *     .motor(cfg.liftMotorName, cfg.liftDirection)
 *     .position()
 *     .deviceManaged()
 *         .maxPower(0.8)
 *         .doneDeviceManaged()
 *     .nonPeriodic()
 *         .bounded(0.0, 4200.0)
 *         .nativeUnits()
 *         .needsReference("lift not homed")
 *     .positionTolerance(20.0)
 *     .targetFromNewCommand(0.0)
 *     .build();
 *
 * this.flywheel = FtcActuators.plant(hardwareMap)
 *     .motor(cfg.flywheelMotorName, cfg.flywheelDirection)
 *     .velocity()
 *     .deviceManagedWithDefaults()
 *     .bounded(0.0, 2600.0)
 *     .nativeUnits()
 *     .velocityTolerance(50.0)
 *     .targetFromNewCommand(0.0)
 *     .build();
 *
 * this.claw = FtcActuators.plant(hardwareMap)
 *     .servo(cfg.clawServoName, cfg.clawDirection)
 *     .position()
 *     .nonPeriodic()
 *         .bounded(0.0, 1.0)
 *         .rangeMapsToNative(0.30, 0.80)
 *     .targetFromNewCommand(0.0)
 *     .build();
 *
 * // Later, in a semantic mechanism method:
 * lift.commandTarget().set(1200.0);
 * }</pre>
 */
public final class FtcActuators {

    private FtcActuators() {
        // utility class
    }

    /**
     * Start building a plant from FTC hardware.
     *
     * @param hw FTC hardware map used to resolve named devices during the staged build
     * @return first builder step for selecting actuator hardware
     */
    public static StartStep plant(HardwareMap hw) {
        return new StartBuilder(hw);
    }

    // ---------------------------------------------------------------------------------------------
    // Builder steps
    // ---------------------------------------------------------------------------------------------

    /**
     * First builder step: choose the primary actuator family and first named device.
     */
    public interface StartStep {
        /**
         * Start building a plant from one or more FTC motors.
         *
         * @throws NullPointerException if {@code name} or {@code direction} is null
         * @throws IllegalArgumentException if {@code name} is blank after FTC-style trimming
         */
        MotorSingleStep motor(String name, Direction direction);

        /**
         * Start building a plant from one or more FTC standard servos.
         *
         * @throws NullPointerException if {@code name} or {@code direction} is null
         * @throws IllegalArgumentException if {@code name} is blank after FTC-style trimming
         */
        ServoSingleStep servo(String name, Direction direction);

        /**
         * Start building a plant from one or more FTC continuous-rotation servos.
         *
         * @throws NullPointerException if {@code name} or {@code direction} is null
         * @throws IllegalArgumentException if {@code name} is blank after FTC-style trimming
         */
        CrServoSingleStep crServo(String name, Direction direction);
    }

    /**
     * Builder step for motor-backed plants.
     */
    public interface MotorSingleStep {
        /**
         * Add another motor to the same plant group.
         *
         * @throws NullPointerException if {@code name} or {@code direction} is null
         * @throws IllegalArgumentException if the name is blank or selects an earlier motor after
         * FTC-style trimming
         */
        MotorGroupAddedStep andMotor(String name, Direction direction);

        /**
         * Build a direct normalized-power Plant over the selected motor or motor group. Its logical
         * target range is always {@code [-1.0, +1.0]}.
         */
        Plants.TargetStep<Plant> power();

        /**
         * Begin the guided motor-velocity builder.
         *
         * <p>The next required question is who manages the velocity loop: FTC device-managed
         * velocity control or a Phoenix-regulated loop driven by native velocity feedback.</p>
         */
        MotorVelocityControlStep velocity();

        /**
         * Begin the guided motor-position builder.
         *
         * <p>The next required question is who manages the position loop: FTC device-managed control
         * or a Phoenix-regulated loop driven by native feedback. Every completed choice builds a
         * feedback position Plant with the temporary calibration-search lifecycle described by
         * {@link MotorPositionControlStep}.</p>
         */
        MotorPositionControlStep position();
    }

    /**
     * Group-aware motor builder step that exposes per-child scale/bias configuration.
     */
    public interface MotorGroupAddedStep extends MotorSingleStep {
        /**
         * Set the scale applied to the most recently added motor in group target units.
         */
        MotorGroupAddedStep scale(double scale);

        /**
         * Set the bias applied to the most recently added motor in group target units.
         */
        MotorGroupAddedStep bias(double bias);

    }

    /**
     * First motor-velocity question: who manages the velocity loop?
     */
    public interface MotorVelocityControlStep {
        /**
         * Use FTC device-managed velocity control with Phoenix defaults and continue to velocity
         * target bounds.
         */
        Plants.VelocityBoundsStep deviceManagedWithDefaults();

        /**
         * Enter the FTC device-managed velocity tuning branch before continuing to target bounds.
         */
        MotorDeviceManagedVelocityStep deviceManaged();

        /**
         * Use a Phoenix-regulated velocity loop that drives motor power from native velocity feedback.
         */
        MotorRegulatedVelocityFeedbackStep regulated();
    }

    /**
     * Optional tuning branch for FTC device-managed motor velocity control.
     */
    public interface MotorDeviceManagedVelocityStep {
        /**
         * Override the FTC device-managed velocity PIDF coefficients.
         */
        MotorDeviceManagedVelocityStep velocityPidf(double p, double i, double d, double f);

        /**
         * Leave the device-managed velocity tuning branch and continue to target bounds.
         */
        Plants.VelocityBoundsStep doneDeviceManaged();
    }

    /**
     * Required native-feedback question for regulated motor velocity control.
     */
    public interface MotorRegulatedVelocityFeedbackStep {
        /**
         * Use the selected motor's SDK-reported internal encoder velocity in native ticks/sec.
         */
        MotorRegulatedVelocityRegulatorStep internalEncoder();

        /**
         * Use one named selected motor's SDK-reported internal encoder velocity in native ticks/sec.
         * Matching ignores surrounding whitespace like FTC lookup and remains case-sensitive.
         */
        MotorRegulatedVelocityRegulatorStep internalEncoder(String motorName);

        /**
         * Use the average SDK-reported internal encoder velocity of all selected motors.
         */
        MotorRegulatedVelocityRegulatorStep averageInternalEncoders();

        /**
         * Derive native velocity in ticks/sec from a named external encoder's position samples.
         *
         * <p>The FTC position counter is made continuous across signed 32-bit rollover before the
         * rate is calculated. This avoids depending on the SDK's narrower direct-velocity result
         * for a high-count-rate external encoder.</p>
         */
        MotorRegulatedVelocityRegulatorStep externalEncoder(String name);

        /**
         * Derive native velocity in ticks/sec from a named external encoder's position samples,
         * with an explicit logical direction.
         */
        MotorRegulatedVelocityRegulatorStep externalEncoder(String name, Direction direction);

        /**
         * Use a caller-supplied native velocity source.
         */
        MotorRegulatedVelocityRegulatorStep nativeFeedback(ScalarSource source);
    }

    /**
     * Required regulator question for regulated motor velocity control.
     */
    public interface MotorRegulatedVelocityRegulatorStep {
        /**
         * Select the regulator that receives plant-unit velocity setpoint and measurement.
         */
        Plants.VelocityBoundsStep regulator(ScalarRegulator regulator);
    }

    /**
     * First motor-position question: who manages the position loop?
     *
     * <p>Every choice below builds a feedback {@link PositionPlant} that supports temporary
     * raw-power calibration search. {@link PositionPlant#beginCalibrationSearch(double)} stops the
     * prior normal output and stages the search request; it does not submit that search power.
     * The mechanism or subsystem remains the sole Plant heartbeat owner, and its normal downstream
     * {@link Plant#update(edu.ftcphoenix.fw.core.time.LoopClock)} call is the sole search-command
     * writer. Search power must be finite in the inclusive normalized {@code [-1.0, +1.0]} range
     * and is rejected before acquisition rather than clamped; the mechanism owner still chooses
     * and physically validates a safe magnitude and direction.</p>
     */
    public interface MotorPositionControlStep {
        /**
         * Use FTC RUN_TO_POSITION with Phoenix defaults and continue to position periodicity.
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep> deviceManagedWithDefaults();

        /**
         * Enter the FTC RUN_TO_POSITION tuning branch before continuing to position periodicity.
         */
        MotorDeviceManagedPositionStep deviceManaged();

        /**
         * Use a Phoenix-regulated loop that drives motor power from native position feedback.
         */
        MotorRegulatedPositionFeedbackStep regulated();
    }

    /**
     * Optional tuning branch for FTC device-managed motor position control.
     */
    public interface MotorDeviceManagedPositionStep {
        /**
         * Set the motor power reapplied for each RUN_TO_POSITION target.
         */
        MotorDeviceManagedPositionStep maxPower(double maxPower);

        /**
         * Set FTC's outer position-loop proportional coefficient.
         */
        MotorDeviceManagedPositionStep outerPositionP(double outerPositionP);

        /**
         * Set FTC's inner velocity-loop PIDF coefficients used underneath RUN_TO_POSITION.
         */
        MotorDeviceManagedPositionStep innerVelocityPidf(double p, double i, double d, double f);

        /**
         * Set FTC's native device target-position tolerance in encoder ticks.
         */
        MotorDeviceManagedPositionStep devicePositionToleranceTicks(int ticks);

        /**
         * Leave the device-managed tuning branch and continue to position periodicity.
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep> doneDeviceManaged();
    }

    /**
     * Required feedback question for regulated motor position control.
     */
    public interface MotorRegulatedPositionFeedbackStep {
        /**
         * Use the selected motor's internal encoder position in native ticks.
         */
        MotorRegulatedPositionRegulatorStep internalEncoder();

        /**
         * Use one named selected motor's internal encoder position in native ticks.
         * Matching ignores surrounding whitespace like FTC lookup and remains case-sensitive.
         */
        MotorRegulatedPositionRegulatorStep internalEncoder(String motorName);

        /**
         * Use the average internal encoder position of all selected motors.
         */
        MotorRegulatedPositionRegulatorStep averageInternalEncoders();

        /**
         * Use a named external encoder's position in native ticks.
         */
        MotorRegulatedPositionRegulatorStep externalEncoder(String name);

        /**
         * Use a named external encoder's position in native ticks with an explicit logical
         * direction.
         */
        MotorRegulatedPositionRegulatorStep externalEncoder(String name, Direction direction);

        /**
         * Use a caller-supplied native position source.
         */
        MotorRegulatedPositionRegulatorStep nativeFeedback(ScalarSource source);
    }

    /**
     * Required regulator question for regulated motor position control.
     */
    public interface MotorRegulatedPositionRegulatorStep {
        /**
         * Select the regulator that receives plant-unit setpoint and measurement.
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep> regulator(ScalarRegulator regulator);
    }

    /**
     * Builder step for standard-servo-backed plants.
     */
    public interface ServoSingleStep {
        /**
         * Add another standard servo to the same plant group.
         *
         * @throws NullPointerException if {@code name} or {@code direction} is null
         * @throws IllegalArgumentException if the name is blank or selects an earlier servo after
         * FTC-style trimming
         */
        ServoGroupAddedStep andServo(String name, Direction direction);

        /**
         * Begin the guided standard-servo position builder.
         *
         * <p>Standard-servo position Plants are command-only and do not expose temporary
         * open-loop calibration search.</p>
         */
        ServoPositionPeriodicityStep position();
    }

    /**
     * Group-aware servo builder step that exposes per-child scale/bias configuration.
     */
    public interface ServoGroupAddedStep extends ServoSingleStep {
        /**
         * Set the scale applied to the most recently added servo in group target units.
         */
        ServoGroupAddedStep scale(double scale);

        /**
         * Set the bias applied to the most recently added servo in group target units.
         */
        ServoGroupAddedStep bias(double bias);

    }

    /**
     * Required caller-coordinate periodicity question for a standard-servo position Plant.
     */
    public interface ServoPositionPeriodicityStep {
        /**
         * Declare that positions have no fixed equivalence period.
         */
        ServoPositionBoundsStep nonPeriodic();

        /**
         * Declare that positions separated by {@code period} are equivalent.
         */
        ServoPositionBoundsStep periodic(double period);
    }

    /**
     * Standard servo bounds question.
     */
    public interface ServoPositionBoundsStep {
        /**
         * Declare the inclusive caller-facing servo Plant range in Plant units.
         * Both endpoints must be finite and {@code min <= max}; standard-servo construction has
         * no unbounded range answer.
         *
         * @param min inclusive finite Plant-unit minimum
         * @param max inclusive finite Plant-unit maximum
         * @return the standard-servo bounded mapping step
         * @throws IllegalArgumentException if either endpoint is non-finite or {@code min > max}
         * @throws IllegalStateException if bounds were already answered or configuration froze
         */
        ServoBoundedPositionMappingStep bounded(double min, double max);
    }

    /**
     * Standard servo bounded mapping question.
     */
    public interface ServoBoundedPositionMappingStep {
        /**
         * Use raw servo positions as plant units.
         */
        Plants.TargetStep<PositionPlant> nativeUnits();

        /**
         * Map the declared plant range to raw servo endpoints.
         *
         * <p>The arguments are <b>native raw servo values</b> at the plant-range endpoints chosen by
         * {@link ServoPositionBoundsStep#bounded(double, double)}. They are not plant values. In
         * other words, if the declared plant range is {@code [min, max]}, then this method means
         * "plant {@code min} -> native {@code nativeAtPlantMin}" and
         * "plant {@code max} -> native {@code nativeAtPlantMax}".</p>
         */
        Plants.TargetStep<PositionPlant> rangeMapsToNative(double nativeAtPlantMin, double nativeAtPlantMax);
    }

    /**
     * Builder step for continuous-rotation-servo-backed plants.
     */
    public interface CrServoSingleStep {
        /**
         * Add another continuous-rotation servo to the same plant group.
         *
         * @throws NullPointerException if {@code name} or {@code direction} is null
         * @throws IllegalArgumentException if the name is blank or selects an earlier CR servo
         * after FTC-style trimming
         */
        CrServoGroupAddedStep andCrServo(String name, Direction direction);

        /**
         * Build a direct normalized-power Plant over the selected CR servo or group. Its logical
         * target range is always {@code [-1.0, +1.0]}.
         */
        Plants.TargetStep<Plant> power();

        /**
         * Begin the guided regulated CR-servo position builder. The completed feedback position
         * Plant supports the temporary calibration-search lifecycle described by
         * {@link CrServoPositionControlStep}.
         */
        CrServoPositionControlStep position();
    }

    /**
     * Group-aware CR-servo builder step that exposes per-child scale/bias configuration.
     */
    public interface CrServoGroupAddedStep extends CrServoSingleStep {
        /**
         * Set the scale applied to the most recently added CR servo in group target units.
         */
        CrServoGroupAddedStep scale(double scale);

        /**
         * Set the bias applied to the most recently added CR servo in group target units.
         */
        CrServoGroupAddedStep bias(double bias);

    }

    /**
     * First CR-servo position question: CR servos require regulated position control.
     *
     * <p>The resulting feedback {@link PositionPlant} supports temporary open-loop calibration
     * search. {@link PositionPlant#beginCalibrationSearch(double)} stops the prior regulated output
     * and stages the search request; it does not submit that search power. The mechanism or
     * subsystem remains the sole Plant heartbeat owner, and its normal downstream
     * {@link Plant#update(edu.ftcphoenix.fw.core.time.LoopClock)} call is the sole search-command
     * writer. Search power must be finite in the inclusive normalized {@code [-1.0, +1.0]} range
     * and is rejected before acquisition rather than clamped; the mechanism owner still chooses
     * and physically validates a safe magnitude and direction.</p>
     */
    public interface CrServoPositionControlStep {
        /**
         * Use a Phoenix-regulated loop that drives CR-servo power from native position feedback.
         */
        CrServoRegulatedPositionFeedbackStep regulated();
    }

    /**
     * Required native feedback question for regulated CR-servo position control.
     */
    public interface CrServoRegulatedPositionFeedbackStep {
        /**
         * Use a named external encoder's position in native ticks.
         */
        CrServoRegulatedPositionRegulatorStep externalEncoder(String name);

        /**
         * Use a named external encoder's position in native ticks with an explicit logical
         * direction.
         */
        CrServoRegulatedPositionRegulatorStep externalEncoder(String name, Direction direction);

        /**
         * Use a caller-supplied native position source.
         */
        CrServoRegulatedPositionRegulatorStep nativeFeedback(ScalarSource source);
    }

    /**
     * Required regulator question for regulated CR-servo position control.
     */
    public interface CrServoRegulatedPositionRegulatorStep {
        /**
         * Select the regulator that receives plant-unit setpoint and measurement.
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep> regulator(ScalarRegulator regulator);
    }

    private static final class StartBuilder implements StartStep {
        private final HardwareMap hw;
        private final RecipeLifecycle lifecycle = new RecipeLifecycle();
        private boolean hardwareFamilySelected;

        private StartBuilder(HardwareMap hw) {
            this.hw = Objects.requireNonNull(hw, "HardwareMap is required");
        }

        @Override
        public MotorSingleStep motor(String name, Direction direction) {
            requireHardwareFamilyUnselected("motor(...)");
            MotorBuilder selected = new MotorBuilder(hw, lifecycle, name, direction);
            hardwareFamilySelected = true;
            return selected;
        }

        @Override
        public ServoSingleStep servo(String name, Direction direction) {
            requireHardwareFamilyUnselected("servo(...)");
            ServoBuilder selected = new ServoBuilder(hw, lifecycle, name, direction);
            hardwareFamilySelected = true;
            return selected;
        }

        @Override
        public CrServoSingleStep crServo(String name, Direction direction) {
            requireHardwareFamilyUnselected("crServo(...)");
            CrServoBuilder selected = new CrServoBuilder(hw, lifecycle, name, direction);
            hardwareFamilySelected = true;
            return selected;
        }

        private void requireHardwareFamilyUnselected(String operation) {
            lifecycle.requireMutable(operation);
            if (hardwareFamilySelected) {
                throw new IllegalStateException("This FtcActuators.plant(...) root already selected "
                        + "an actuator family; start a new Plant recipe");
            }
        }
    }

    private static final ScalarRange NORMALIZED_POWER_RANGE = ScalarRange.bounded(-1.0, 1.0);

    /** One lifecycle shared by every alias derived from a single {@link #plant(HardwareMap)} root. */
    private static final class RecipeLifecycle {
        private boolean frozen;
        private boolean buildAttempted;

        void requireMutable(String operation) {
            if (frozen) {
                throw new IllegalStateException(operation + " cannot change this Plant recipe after "
                        + "targetFromNewCommand(...) or targetFromResolver(...); start a new "
                        + "FtcActuators.plant(...) recipe");
            }
        }

        void freeze() {
            frozen = true;
        }

        void beginBuild(String plantName) {
            if (buildAttempted) {
                throw new IllegalStateException(plantName + " build() has already been attempted; "
                        + "start a new FtcActuators.plant(...) recipe");
            }
            buildAttempted = true;
        }
    }

    private enum TargetProvenance { NEW_COMMAND, RESOLVER }

    private enum GuardKind {
        RATE,
        RATES,
        HOLD_BOOLEAN,
        HOLD_TARGET,
        FALLBACK_BOOLEAN,
        FALLBACK_TARGET
    }

    /** Data-only guard answer retained until the shared Plants builder creates the guard chain. */
    private static final class GuardSpec {
        final GuardKind kind;
        final String name;
        final BooleanSource allowed;
        final PlantTargetGate gate;
        final double first;
        final double second;

        private GuardSpec(GuardKind kind,
                          String name,
                          BooleanSource allowed,
                          PlantTargetGate gate,
                          double first,
                          double second) {
            this.kind = kind;
            this.name = name;
            this.allowed = allowed;
            this.gate = gate;
            this.first = first;
            this.second = second;
        }

        static GuardSpec rate(double rate) {
            return new GuardSpec(GuardKind.RATE, null, null, null, rate, rate);
        }

        static GuardSpec rates(double up, double down) {
            return new GuardSpec(GuardKind.RATES, null, null, null, up, down);
        }

        static GuardSpec hold(String name, BooleanSource allowed) {
            return new GuardSpec(GuardKind.HOLD_BOOLEAN, name, allowed, null, 0.0, 0.0);
        }

        static GuardSpec hold(String name, PlantTargetGate gate) {
            return new GuardSpec(GuardKind.HOLD_TARGET, name, null, gate, 0.0, 0.0);
        }

        static GuardSpec fallback(String name, BooleanSource allowed, double fallback) {
            return new GuardSpec(GuardKind.FALLBACK_BOOLEAN, name, allowed, null, fallback, 0.0);
        }

        static GuardSpec fallback(String name, PlantTargetGate gate, double fallback) {
            return new GuardSpec(GuardKind.FALLBACK_TARGET, name, null, gate, fallback, 0.0);
        }
    }

    /**
     * Thin FTC target-tail adapter. It retains only data until build, then replays the recipe through
     * the shared hardware-neutral Plants stages and returns that engine's result.
     */
    private abstract static class FtcTargetBuilder<P extends Plant>
            implements Plants.TargetStep<P>, Plants.TargetGuardStep<P>, Plants.BuildStep<P> {
        private final RecipeLifecycle lifecycle;
        private final String plantName;
        private final List<GuardSpec> guardSpecs = new ArrayList<>();
        private boolean guardBranchEntered;
        private boolean guardBranchOpen;
        private boolean rateAnswered;
        private TargetProvenance targetProvenance;
        private double newCommandInitialValue;
        private PlantTargetResolver targetResolver;

        FtcTargetBuilder(RecipeLifecycle lifecycle, String plantName) {
            this.lifecycle = Objects.requireNonNull(lifecycle, "lifecycle");
            this.plantName = Objects.requireNonNull(plantName, "plantName");
        }

        protected final void requireMutable(String operation) {
            lifecycle.requireMutable(operation);
        }

        @Override
        public final Plants.TargetGuardStep<P> targetGuards() {
            requireMutable("targetGuards()");
            if (guardBranchEntered) {
                throw new IllegalStateException("targetGuards() has already been entered for this "
                        + plantName);
            }
            guardBranchEntered = true;
            guardBranchOpen = true;
            return this;
        }

        @Override
        public final Plants.TargetGuardStep<P> maxTargetRate(double maxDeltaPerSec) {
            requireOpenGuards("maxTargetRate(...)");
            requireFinitePositive(maxDeltaPerSec, "maxDeltaPerSec");
            requireRateUnanswered();
            rateAnswered = true;
            guardSpecs.add(GuardSpec.rate(maxDeltaPerSec));
            return this;
        }

        @Override
        public final Plants.TargetGuardStep<P> maxTargetRates(double maxUpPerSec,
                                                               double maxDownPerSec) {
            requireOpenGuards("maxTargetRates(...)");
            requireFinitePositive(maxUpPerSec, "maxUpPerSec");
            requireFinitePositive(maxDownPerSec, "maxDownPerSec");
            requireRateUnanswered();
            rateAnswered = true;
            guardSpecs.add(GuardSpec.rates(maxUpPerSec, maxDownPerSec));
            return this;
        }

        @Override
        public final Plants.TargetGuardStep<P> holdLastTargetUnless(String name,
                                                                    BooleanSource allowed) {
            requireOpenGuards("holdLastTargetUnless(...)");
            guardSpecs.add(GuardSpec.hold(name, Objects.requireNonNull(allowed, "allowed")));
            return this;
        }

        @Override
        public final Plants.TargetGuardStep<P> holdLastTargetUnless(String name,
                                                                    PlantTargetGate gate) {
            requireOpenGuards("holdLastTargetUnless(...)");
            guardSpecs.add(GuardSpec.hold(name, Objects.requireNonNull(gate, "gate")));
            return this;
        }

        @Override
        public final Plants.TargetGuardStep<P> fallbackTargetUnless(String name,
                                                                    BooleanSource allowed,
                                                                    double fallbackTarget) {
            requireOpenGuards("fallbackTargetUnless(...)");
            requireValidFallback(name, fallbackTarget);
            guardSpecs.add(GuardSpec.fallback(
                    name, Objects.requireNonNull(allowed, "allowed"), fallbackTarget));
            return this;
        }

        @Override
        public final Plants.TargetGuardStep<P> fallbackTargetUnless(String name,
                                                                    PlantTargetGate gate,
                                                                    double fallbackTarget) {
            requireOpenGuards("fallbackTargetUnless(...)");
            requireValidFallback(name, fallbackTarget);
            guardSpecs.add(GuardSpec.fallback(
                    name, Objects.requireNonNull(gate, "gate"), fallbackTarget));
            return this;
        }

        @Override
        public final Plants.TargetStep<P> doneTargetGuards() {
            requireOpenGuards("doneTargetGuards()");
            guardBranchOpen = false;
            return this;
        }

        @Override
        public final Plants.BuildStep<P> targetFromNewCommand(double initialValue) {
            requireTargetUnanswered();
            validateRecipe();
            requireGuardsClosed();
            requireValidNewCommandInitialValue(initialValue);
            targetProvenance = TargetProvenance.NEW_COMMAND;
            newCommandInitialValue = initialValue;
            lifecycle.freeze();
            return this;
        }

        @Override
        public final Plants.BuildStep<P> targetFromResolver(PlantTargetResolver resolver) {
            PlantTargetResolver checked = Objects.requireNonNull(resolver, "resolver");
            requireTargetUnanswered();
            validateRecipe();
            requireGuardsClosed();
            targetProvenance = TargetProvenance.RESOLVER;
            targetResolver = checked;
            lifecycle.freeze();
            return this;
        }

        @Override
        public final P build() {
            lifecycle.beginBuild(plantName);
            if (targetProvenance == null) {
                throw new IllegalStateException(plantName
                        + " requires targetFromNewCommand(...) or targetFromResolver(...) before "
                        + "build()");
            }
            requireGuardsClosed();
            validateRecipe();
            validateStoredTargets();

            Plants.TargetStep<P> sharedTarget = createSharedTargetStep();
            if (!guardSpecs.isEmpty()) {
                Plants.TargetGuardStep<P> sharedGuards = sharedTarget.targetGuards();
                for (GuardSpec spec : guardSpecs) {
                    switch (spec.kind) {
                        case RATE:
                            sharedGuards.maxTargetRate(spec.first);
                            break;
                        case RATES:
                            sharedGuards.maxTargetRates(spec.first, spec.second);
                            break;
                        case HOLD_BOOLEAN:
                            sharedGuards.holdLastTargetUnless(spec.name, spec.allowed);
                            break;
                        case HOLD_TARGET:
                            sharedGuards.holdLastTargetUnless(spec.name, spec.gate);
                            break;
                        case FALLBACK_BOOLEAN:
                            sharedGuards.fallbackTargetUnless(spec.name, spec.allowed, spec.first);
                            break;
                        case FALLBACK_TARGET:
                            sharedGuards.fallbackTargetUnless(spec.name, spec.gate, spec.first);
                            break;
                        default:
                            throw new IllegalStateException("Unsupported target guard recipe: "
                                    + spec.kind);
                    }
                }
                sharedTarget = sharedGuards.doneTargetGuards();
            }

            Plants.BuildStep<P> sharedBuild = targetProvenance == TargetProvenance.NEW_COMMAND
                    ? sharedTarget.targetFromNewCommand(newCommandInitialValue)
                    : sharedTarget.targetFromResolver(targetResolver);
            return sharedBuild.build();
        }

        protected abstract void validateRecipe();

        protected abstract ScalarRange configuredTargetRange();

        protected abstract Plants.TargetStep<P> createSharedTargetStep();

        private void requireOpenGuards(String operation) {
            requireMutable(operation);
            if (!guardBranchOpen) {
                throw new IllegalStateException(operation
                        + " requires targetGuards() and cannot run after doneTargetGuards()");
            }
        }

        private void requireRateUnanswered() {
            if (rateAnswered) {
                throw new IllegalStateException("The target-rate guard has already been answered for "
                        + "this " + plantName);
            }
        }

        private void requireTargetUnanswered() {
            if (targetProvenance != null) {
                String acceptedAnswer = targetProvenance == TargetProvenance.NEW_COMMAND
                        ? "targetFromNewCommand(...)" : "targetFromResolver(...)";
                throw new IllegalStateException(acceptedAnswer
                        + " has already been answered for this " + plantName
                        + "; start a new builder from FtcActuators.plant(...)");
            }
            requireMutable("target selection");
        }

        private void requireGuardsClosed() {
            if (guardBranchOpen) {
                throw new IllegalStateException("Call doneTargetGuards() before choosing or building "
                        + "this " + plantName);
            }
        }

        private void requireValidNewCommandInitialValue(double value) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException("targetFromNewCommand initialValue must be "
                        + "finite, got " + value);
            }
            ScalarRange range = Objects.requireNonNull(configuredTargetRange(),
                    "configuredTargetRange");
            if (!range.valid || !range.contains(value)) {
                throw new IllegalArgumentException("targetFromNewCommand initialValue " + value
                        + " is outside this " + plantName + " plant-unit range " + range);
            }
        }

        private void requireValidFallback(String guardName, double value) {
            String checkedName = guardName == null || guardName.trim().isEmpty()
                    ? "interlock" : guardName.trim();
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException(plantName + " target guard '" + checkedName
                        + "' fallbackTarget must be finite, got " + value);
            }
            ScalarRange range = Objects.requireNonNull(configuredTargetRange(),
                    "configuredTargetRange");
            if (!range.valid || !range.contains(value)) {
                throw new IllegalArgumentException(plantName + " target guard '" + checkedName
                        + "' has fallback target " + value + " outside its plant-unit range "
                        + range);
            }
        }

        private void validateStoredTargets() {
            if (targetProvenance == TargetProvenance.NEW_COMMAND) {
                requireValidNewCommandInitialValue(newCommandInitialValue);
            }
            for (GuardSpec spec : guardSpecs) {
                if (spec.kind == GuardKind.FALLBACK_BOOLEAN
                        || spec.kind == GuardKind.FALLBACK_TARGET) {
                    requireValidFallback(spec.name, spec.first);
                }
            }
        }
    }

    private static final class PowerTargetBuilder extends FtcTargetBuilder<Plant> {
        private final Supplier<PowerOutput> output;

        private PowerTargetBuilder(RecipeLifecycle lifecycle, Supplier<PowerOutput> output) {
            super(lifecycle, "power Plant");
            this.output = Objects.requireNonNull(output, "output");
        }

        @Override
        protected void validateRecipe() {
            // The normalized power target range is fixed; hardware is resolved only below.
        }

        @Override
        protected ScalarRange configuredTargetRange() {
            return NORMALIZED_POWER_RANGE;
        }

        @Override
        protected Plants.TargetStep<Plant> createSharedTargetStep() {
            return Plants.fromOutputs().power(output.get());
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Velocity control configuration
    // ---------------------------------------------------------------------------------------------

    private static final class DeviceManagedVelocityConfig {
        private double[] velocityPidf;
    }

    private enum VelocityControlKind {DEVICE_MANAGED, REGULATED}

    // ---------------------------------------------------------------------------------------------
    // Builder-side feedback resolution
    // ---------------------------------------------------------------------------------------------

    private static ScalarSource internalPositionFeedback(HardwareMap hw,
                                                         List<MotorBuilder.Spec> motorSpecs,
                                                         String motorName,
                                                         boolean average) {
        ensureMotorFeedbackAvailable(motorSpecs, "position");
        if (average) {
            List<ScalarSource> sources = new ArrayList<>();
            for (MotorBuilder.Spec spec : motorSpecs) {
                sources.add(FtcSensors.motorPositionTicks(hw, spec.name));
            }
            return averageSources(sources);
        }
        if (motorName != null) {
            for (MotorBuilder.Spec spec : motorSpecs) {
                if (FtcHardwareNameGroups.sameConfiguredName(spec.name, motorName)) {
                    return FtcSensors.motorPositionTicks(hw, spec.name);
                }
            }
            throw new IllegalStateException("internalEncoder(\"" + motorName
                    + "\") does not match any selected motor");
        }
        if (motorSpecs.size() != 1) {
            throw new IllegalStateException("internalEncoder() is ambiguous for a "
                    + motorSpecs.size() + "-motor group. Choose internalEncoder(\"name\"), "
                    + "averageInternalEncoders(), or externalEncoder(...).");
        }
        return FtcSensors.motorPositionTicks(hw, motorSpecs.get(0).name);
    }

    private static ScalarSource internalVelocityFeedback(HardwareMap hw,
                                                         List<MotorBuilder.Spec> motorSpecs,
                                                         String motorName,
                                                         boolean average) {
        ensureMotorFeedbackAvailable(motorSpecs, "velocity");
        if (average) {
            List<ScalarSource> sources = new ArrayList<>();
            for (MotorBuilder.Spec spec : motorSpecs) {
                sources.add(FtcSensors.motorVelocityTicksPerSec(hw, spec.name));
            }
            return averageSources(sources);
        }
        if (motorName != null) {
            for (MotorBuilder.Spec spec : motorSpecs) {
                if (FtcHardwareNameGroups.sameConfiguredName(spec.name, motorName)) {
                    return FtcSensors.motorVelocityTicksPerSec(hw, spec.name);
                }
            }
            throw new IllegalStateException("internalEncoder(\"" + motorName
                    + "\") does not match any selected motor");
        }
        if (motorSpecs.size() != 1) {
            throw new IllegalStateException("internalEncoder() is ambiguous for a "
                    + motorSpecs.size() + "-motor group. Choose internalEncoder(\"name\"), "
                    + "averageInternalEncoders(), or externalEncoder(...).");
        }
        return FtcSensors.motorVelocityTicksPerSec(hw, motorSpecs.get(0).name);
    }

    /**
     * Validate and append one member without exposing a second public hardware-group type.
     *
     * <p>Validation order is observable and intentional: name null, direction null, blank
     * FTC-effective name, duplicate FTC-effective name, then mutation.</p>
     */
    private static int addActuatorSpec(List<MotorBuilder.Spec> specs,
                                       String family,
                                       String name,
                                       Direction direction) {
        String requiredName = Objects.requireNonNull(name, "name");
        Direction requiredDirection = Objects.requireNonNull(direction, "direction");
        List<String> earlierNames = new ArrayList<>(specs.size());
        for (MotorBuilder.Spec spec : specs) {
            earlierNames.add(spec.name);
        }
        FtcHardwareNameGroups.requireNewMember(
                "FtcActuators",
                family + " member " + (specs.size() + 1),
                requiredName,
                earlierNames);
        specs.add(new MotorBuilder.Spec(requiredName, requiredDirection));
        return specs.size() - 1;
    }

    // ---------------------------------------------------------------------------------------------
    // Motor builder
    // ---------------------------------------------------------------------------------------------

    private static final class MotorBuilder implements MotorGroupAddedStep {
        private final HardwareMap hw;
        private final RecipeLifecycle lifecycle;
        private final List<Spec> specs = new ArrayList<>();
        private int lastIndex;

        private static final class Spec {
            private final String name;
            private final Direction direction;
            private double scale = 1.0;
            private double bias = 0.0;

            private Spec(String name, Direction direction) {
                this.name = Objects.requireNonNull(name, "name");
                this.direction = Objects.requireNonNull(direction, "direction");
            }
        }

        private MotorBuilder(HardwareMap hw,
                             RecipeLifecycle lifecycle,
                             String name,
                             Direction direction) {
            this.hw = Objects.requireNonNull(hw, "HardwareMap is required");
            this.lifecycle = Objects.requireNonNull(lifecycle, "lifecycle");
            addMotorInternal(name, direction);
        }

        private void addMotorInternal(String name, Direction direction) {
            lifecycle.requireMutable("motor group configuration");
            lastIndex = addActuatorSpec(specs, "motor", name, direction);
        }

        @Override
        public MotorGroupAddedStep andMotor(String name, Direction direction) {
            addMotorInternal(name, direction);
            return this;
        }

        @Override
        public MotorGroupAddedStep scale(double scale) {
            lifecycle.requireMutable("motor scale(...)");
            specs.get(lastIndex).scale = scale;
            return this;
        }

        @Override
        public MotorGroupAddedStep bias(double bias) {
            lifecycle.requireMutable("motor bias(...)");
            specs.get(lastIndex).bias = bias;
            return this;
        }

        @Override
        public Plants.TargetStep<Plant> power() {
            lifecycle.requireMutable("power()");
            return new PowerTargetBuilder(lifecycle, this::groupedMotorPowerWithMappings);
        }

        @Override
        public MotorVelocityControlStep velocity() {
            lifecycle.requireMutable("velocity()");
            return new MotorVelocityBuilder(this);
        }

        @Override
        public MotorPositionControlStep position() {
            lifecycle.requireMutable("position()");
            return new MotorPositionBuilder(this);
        }

        private PowerOutput groupedMotorPower() {
            if (specs.size() == 1) {
                Spec spec = specs.get(0);
                return FtcHardware.motorPower(hw, spec.name, spec.direction);
            }
            return new GroupedPowerOutput(groupedFtcMotorPowers());
        }

        private PowerOutput groupedMotorPowerWithMappings() {
            if (specs.size() == 1) {
                Spec spec = specs.get(0);
                return new ScaledBiasedPowerOutput(FtcHardware.motorPower(hw, spec.name, spec.direction), spec.scale, spec.bias);
            }
            double[] scales = new double[specs.size()];
            double[] biases = new double[specs.size()];
            for (int i = 0; i < specs.size(); i++) {
                Spec spec = specs.get(i);
                scales[i] = spec.scale;
                biases[i] = spec.bias;
            }
            return new GroupedPowerOutput(groupedFtcMotorPowers(), scales, biases);
        }

        /**
         * Resolve every grouped motor before configuring directions and coordinating power writes.
         */
        private List<PowerOutput> groupedFtcMotorPowers() {
            List<String> names = new ArrayList<>(specs.size());
            List<Direction> directions = new ArrayList<>(specs.size());
            for (Spec spec : specs) {
                names.add(spec.name);
                directions.add(spec.direction);
            }
            return FtcHardware.motorPowerGroup(hw, names, directions);
        }

        private VelocityOutput groupedMotorVelocity(DeviceManagedVelocityConfig cfg) {
            if (specs.size() == 1) {
                Spec spec = specs.get(0);
                DcMotorEx motor = hw.get(DcMotorEx.class, spec.name);
                applyDeviceManagedVelocityConfig(motor, spec.name, cfg.velocityPidf);
                return FtcHardware.motorVelocity(motor, spec.direction);
            }
            ensureFeedbackScalesNonZero("device-managed motor velocity");
            List<VelocityOutput> outs = new ArrayList<>();
            double[] scales = new double[specs.size()];
            double[] biases = new double[specs.size()];
            for (int i = 0; i < specs.size(); i++) {
                Spec spec = specs.get(i);
                DcMotorEx motor = hw.get(DcMotorEx.class, spec.name);
                applyDeviceManagedVelocityConfig(motor, spec.name, cfg.velocityPidf);
                outs.add(FtcHardware.motorVelocity(motor, spec.direction));
                scales[i] = spec.scale;
                biases[i] = spec.bias;
            }
            return new GroupedVelocityOutput(outs, scales, biases);
        }

        private ScalarSource groupedMotorVelocityMeasurement() {
            if (specs.size() == 1)
                return FtcSensors.motorVelocityTicksPerSec(hw, specs.get(0).name);
            List<ScalarSource> sources = new ArrayList<>();
            double[] scales = new double[specs.size()];
            double[] biases = new double[specs.size()];
            for (int i = 0; i < specs.size(); i++) {
                Spec spec = specs.get(i);
                sources.add(FtcSensors.motorVelocityTicksPerSec(hw, spec.name));
                scales[i] = spec.scale;
                biases[i] = spec.bias;
            }
            return averageInverseMappedSources(sources, scales, biases);
        }

        private PositionOutput groupedMotorPosition(DeviceManagedPositionConfig cfg) {
            if (specs.size() == 1) {
                Spec spec = specs.get(0);
                DcMotorEx motor = hw.get(DcMotorEx.class, spec.name);
                applyDeviceManagedPositionConfig(motor, spec.name, cfg);
                return FtcHardware.motorPosition(motor, spec.direction, cfg.maxPower);
            }
            ensureFeedbackScalesNonZero("device-managed motor position");
            List<PositionOutput> outs = new ArrayList<>();
            double[] scales = new double[specs.size()];
            double[] biases = new double[specs.size()];
            for (int i = 0; i < specs.size(); i++) {
                Spec spec = specs.get(i);
                DcMotorEx motor = hw.get(DcMotorEx.class, spec.name);
                applyDeviceManagedPositionConfig(motor, spec.name, cfg);
                outs.add(FtcHardware.motorPosition(motor, spec.direction, cfg.maxPower));
                scales[i] = spec.scale;
                biases[i] = spec.bias;
            }
            return new GroupedPositionOutput(outs, scales, biases);
        }

        private ScalarSource groupedMotorPositionMeasurement() {
            if (specs.size() == 1) return FtcSensors.motorPositionTicks(hw, specs.get(0).name);
            List<ScalarSource> sources = new ArrayList<>();
            double[] scales = new double[specs.size()];
            double[] biases = new double[specs.size()];
            for (int i = 0; i < specs.size(); i++) {
                Spec spec = specs.get(i);
                sources.add(FtcSensors.motorPositionTicks(hw, spec.name));
                scales[i] = spec.scale;
                biases[i] = spec.bias;
            }
            return averageInverseMappedSources(sources, scales, biases);
        }

        private void requireDefaultGroupScalingForRegulated(String mode) {
            if (specs.size() <= 1) return;
            for (Spec spec : specs) {
                if (Math.abs(spec.scale - 1.0) > 1e-9 || Math.abs(spec.bias) > 1e-9) {
                    throw new IllegalStateException("Regulated motor " + mode + " control requires default group scaling/bias when built through FtcActuators. Build raw outputs manually for non-trivial per-motor mappings.");
                }
            }
        }

        private void ensureFeedbackScalesNonZero(String mode) {
            for (Spec spec : specs)
                if (Math.abs(spec.scale) < 1e-9)
                    throw new IllegalStateException(mode + " requires non-zero per-motor scale");
        }
    }

    private static final class MotorVelocityBuilder extends FtcTargetBuilder<Plant>
            implements MotorVelocityControlStep,
            MotorDeviceManagedVelocityStep,
            MotorRegulatedVelocityFeedbackStep,
            MotorRegulatedVelocityRegulatorStep,
            Plants.VelocityBoundsStep,
            Plants.VelocityMappingStep,
            Plants.VelocityToleranceStep {
        private final MotorBuilder parent;
        private final DeviceManagedVelocityConfig deviceConfig = new DeviceManagedVelocityConfig();
        private VelocityControlKind controlKind;
        private Supplier<ScalarSource> feedback;
        private ScalarRegulator regulator;
        private ScalarRange range;
        private double nativePerPlantUnit;
        private double velocityTolerance;
        private boolean rangeAnswered;
        private boolean mappingAnswered;
        private boolean velocityToleranceAnswered;

        private MotorVelocityBuilder(MotorBuilder parent) {
            super(parent.lifecycle, "motor velocity Plant");
            this.parent = Objects.requireNonNull(parent, "parent");
        }

        @Override
        public Plants.VelocityBoundsStep deviceManagedWithDefaults() {
            answerControl(VelocityControlKind.DEVICE_MANAGED, "deviceManagedWithDefaults()");
            return this;
        }

        @Override
        public MotorDeviceManagedVelocityStep deviceManaged() {
            answerControl(VelocityControlKind.DEVICE_MANAGED, "deviceManaged()");
            return this;
        }

        @Override
        public MotorDeviceManagedVelocityStep velocityPidf(double p, double i, double d, double f) {
            requireMutable("velocityPidf(...)");
            requireControl(VelocityControlKind.DEVICE_MANAGED, "velocityPidf(...)");
            deviceConfig.velocityPidf = new double[]{p, i, d, f};
            return this;
        }

        @Override
        public Plants.VelocityBoundsStep doneDeviceManaged() {
            requireMutable("doneDeviceManaged()");
            requireControl(VelocityControlKind.DEVICE_MANAGED, "doneDeviceManaged()");
            return this;
        }

        @Override
        public MotorRegulatedVelocityFeedbackStep regulated() {
            answerControl(VelocityControlKind.REGULATED, "regulated()");
            return this;
        }

        @Override
        public MotorRegulatedVelocityRegulatorStep internalEncoder() {
            requireFeedbackUnanswered("internalEncoder()");
            String selectedName = requireSingleMotorFeedbackName(parent.specs, "velocity");
            feedback = () -> FtcSensors.motorVelocityTicksPerSec(parent.hw, selectedName);
            return this;
        }

        @Override
        public MotorRegulatedVelocityRegulatorStep internalEncoder(String motorName) {
            requireFeedbackUnanswered("internalEncoder(...)");
            String selectedName = requireSelectedMotorFeedbackName(
                    parent.specs, Objects.requireNonNull(motorName, "motorName"), "velocity");
            feedback = () -> FtcSensors.motorVelocityTicksPerSec(parent.hw, selectedName);
            return this;
        }

        @Override
        public MotorRegulatedVelocityRegulatorStep averageInternalEncoders() {
            requireFeedbackUnanswered("averageInternalEncoders()");
            ensureMotorFeedbackAvailable(parent.specs, "velocity");
            feedback = () -> internalVelocityFeedback(parent.hw, parent.specs, null, true);
            return this;
        }

        @Override
        public MotorRegulatedVelocityRegulatorStep externalEncoder(String name) {
            return externalEncoder(name, Direction.FORWARD);
        }

        @Override
        public MotorRegulatedVelocityRegulatorStep externalEncoder(String name, Direction direction) {
            requireFeedbackUnanswered("externalEncoder(...)");
            String checkedName = requireFeedbackName(name);
            Direction checkedDirection = Objects.requireNonNull(direction, "direction");
            feedback = () -> FtcSensors.continuousMotorPositionTicks(
                    parent.hw, checkedName, checkedDirection).ratePerSecond();
            return this;
        }

        @Override
        public MotorRegulatedVelocityRegulatorStep nativeFeedback(ScalarSource source) {
            requireFeedbackUnanswered("nativeFeedback(...)");
            ScalarSource checked = Objects.requireNonNull(source, "source");
            feedback = () -> checked;
            return this;
        }

        @Override
        public Plants.VelocityBoundsStep regulator(ScalarRegulator regulator) {
            requireMutable("regulator(...)");
            requireControl(VelocityControlKind.REGULATED, "regulator(...)");
            if (feedback == null) {
                throw new IllegalStateException("Choose regulated velocity feedback before regulator(...)");
            }
            if (this.regulator != null) {
                throw new IllegalStateException("regulator(...) has already been answered for this motor velocity Plant");
            }
            this.regulator = Objects.requireNonNull(regulator, "regulator");
            return this;
        }

        @Override
        public Plants.VelocityMappingStep bounded(double min, double max) {
            requireMutable("bounded(...)");
            requireControlAnswered("bounded(...)");
            requireRangeUnanswered();
            range = ScalarRange.bounded(min, max);
            rangeAnswered = true;
            return this;
        }

        @Override
        public Plants.VelocityMappingStep unbounded() {
            requireMutable("unbounded()");
            requireControlAnswered("unbounded()");
            requireRangeUnanswered();
            range = ScalarRange.unbounded();
            rangeAnswered = true;
            return this;
        }

        @Override
        public Plants.VelocityToleranceStep nativeUnits() {
            answerMapping(1.0, "nativeUnits()");
            return this;
        }

        @Override
        public Plants.VelocityToleranceStep scaleToNative(
                double nativeUnitsPerPlantVelocityUnit) {
            answerMapping(nativeUnitsPerPlantVelocityUnit, "scaleToNative(...)");
            return this;
        }

        @Override
        public Plants.TargetStep<Plant> velocityTolerance(double tolerance) {
            requireMutable("velocityTolerance(...)");
            if (!mappingAnswered) {
                throw new IllegalStateException("Choose nativeUnits() or scaleToNative(...) before velocityTolerance(...)");
            }
            if (velocityToleranceAnswered) {
                throw new IllegalStateException("velocityTolerance(...) has already been answered for this motor velocity Plant");
            }
            requireFiniteTolerance(tolerance, "velocityTolerance");
            velocityTolerance = tolerance;
            velocityToleranceAnswered = true;
            return this;
        }

        @Override
        protected void validateRecipe() {
            if (controlKind == null) {
                throw new IllegalStateException("Motor velocity builder requires deviceManagedWithDefaults(), deviceManaged(), or regulated()");
            }
            if (!rangeAnswered) {
                throw new IllegalStateException("Motor velocity builder requires bounded(...) or unbounded()");
            }
            if (!mappingAnswered) {
                throw new IllegalStateException("Motor velocity builder requires nativeUnits() or scaleToNative(...)");
            }
            if (!velocityToleranceAnswered) {
                throw new IllegalStateException("Motor velocity builder requires velocityTolerance(...) in plant velocity units");
            }
            if (controlKind == VelocityControlKind.DEVICE_MANAGED) {
                parent.ensureFeedbackScalesNonZero("device-managed motor velocity");
            } else {
                parent.requireDefaultGroupScalingForRegulated("velocity");
                if (feedback == null || regulator == null) {
                    throw new IllegalStateException("Regulated motor velocity requires a feedback answer "
                            + "(internalEncoder(), averageInternalEncoders(), externalEncoder(...), or "
                            + "nativeFeedback(...)) and regulator(...)");
                }
            }
        }

        @Override
        protected ScalarRange configuredTargetRange() {
            if (!rangeAnswered) {
                throw new IllegalStateException("Motor velocity builder requires bounded(...) or unbounded() before target selection");
            }
            return range;
        }

        @Override
        protected Plants.TargetStep<Plant> createSharedTargetStep() {
            Plants.VelocityBoundsStep bounds;
            if (controlKind == VelocityControlKind.DEVICE_MANAGED) {
                ScalarSource measurement = parent.groupedMotorVelocityMeasurement();
                VelocityOutput output = parent.groupedMotorVelocity(deviceConfig);
                bounds = Plants.fromOutputs().deviceManagedVelocity(output, measurement);
            } else {
                ScalarSource measurement = feedback.get();
                PowerOutput output = parent.groupedMotorPower();
                bounds = Plants.fromOutputs().regulatedVelocity(output, measurement, regulator);
            }

            Plants.VelocityMappingStep mapping = range.isUnbounded()
                    ? bounds.unbounded()
                    : bounds.bounded(range.minValue, range.maxValue);
            Plants.VelocityToleranceStep tolerance = nativePerPlantUnit == 1.0
                    ? mapping.nativeUnits()
                    : mapping.scaleToNative(nativePerPlantUnit);
            return tolerance.velocityTolerance(velocityTolerance);
        }

        private void answerControl(VelocityControlKind answer, String operation) {
            requireMutable(operation);
            if (controlKind != null) {
                throw new IllegalStateException("Motor velocity control ownership has already been answered");
            }
            controlKind = answer;
        }

        private void requireControl(VelocityControlKind required, String operation) {
            if (controlKind != required) {
                throw new IllegalStateException(operation + " requires "
                        + (required == VelocityControlKind.DEVICE_MANAGED
                        ? "deviceManaged()" : "regulated()"));
            }
        }

        private void requireControlAnswered(String operation) {
            if (controlKind == null) {
                throw new IllegalStateException(operation + " requires a velocity control ownership answer");
            }
        }

        private void requireFeedbackUnanswered(String operation) {
            requireMutable(operation);
            requireControl(VelocityControlKind.REGULATED, operation);
            if (feedback != null) {
                throw new IllegalStateException("Regulated motor velocity feedback has already been answered");
            }
        }

        private void requireRangeUnanswered() {
            if (rangeAnswered) {
                throw new IllegalStateException("Motor velocity bounds have already been answered");
            }
        }

        private void answerMapping(double scale, String operation) {
            requireMutable(operation);
            if (!rangeAnswered) {
                throw new IllegalStateException("Choose bounded(...) or unbounded() before " + operation);
            }
            if (mappingAnswered) {
                throw new IllegalStateException("Motor velocity mapping has already been answered");
            }
            nativePerPlantUnit = requireFiniteNonZero(
                    scale, "nativeUnitsPerPlantVelocityUnit");
            mappingAnswered = true;
        }
    }

    private static final class DeviceManagedPositionConfig {
        private double maxPower = 1.0;
        private Double outerPositionP;
        private double[] innerVelocityPidf;
        private Integer devicePositionToleranceTicks;
    }

    private enum PositionControlKind {DEVICE_MANAGED, REGULATED}

    private enum PositionMappingKind {NATIVE, SCALE, ENDPOINTS}

    private enum PositionReferenceKind {STATIC, ASSUME_CURRENT, NEEDS_REFERENCE}

    private abstract static class BasePositionBuilder extends FtcTargetBuilder<PositionPlant>
            implements Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep>,
            Plants.FeedbackPositionBoundsStep,
            Plants.FeedbackBoundedPositionMappingStep,
            Plants.FeedbackUnboundedPositionMappingStep,
            Plants.PositionReferenceStep,
            Plants.PositionToleranceStep {
        private boolean periodicityAnswered;
        private boolean periodic;
        private double period = Double.NaN;
        private boolean rangeAnswered;
        private boolean bounded;
        private ScalarRange range;
        private double plantMin;
        private double plantMax;
        private PositionMappingKind mappingKind;
        private double nativePerPlantUnit;
        private double nativeAtPlantMin;
        private double nativeAtPlantMax;
        private PositionReferenceKind referenceKind;
        private double plantReference;
        private double nativeReference;
        private double assumePlantPosition;
        private String referenceReason;
        private double positionTolerance;
        private boolean positionToleranceAnswered;

        BasePositionBuilder(RecipeLifecycle lifecycle, String plantName) {
            super(lifecycle, plantName);
        }

        @Override
        public Plants.FeedbackPositionBoundsStep nonPeriodic() {
            answerPeriodicity(false, Double.NaN, "nonPeriodic()");
            return this;
        }

        @Override
        public Plants.FeedbackPositionBoundsStep periodic(double period) {
            requireFinitePositive(period, "period");
            answerPeriodicity(true, period, "periodic(...)");
            return this;
        }

        @Override
        public Plants.FeedbackBoundedPositionMappingStep bounded(double min, double max) {
            requireMutable("bounded(...)");
            requirePeriodicityAnswered("bounded(...)");
            requireRangeUnanswered();
            range = ScalarRange.bounded(min, max);
            plantMin = min;
            plantMax = max;
            bounded = true;
            rangeAnswered = true;
            return this;
        }

        @Override
        public Plants.FeedbackUnboundedPositionMappingStep unbounded() {
            requireMutable("unbounded()");
            requirePeriodicityAnswered("unbounded()");
            requireRangeUnanswered();
            range = ScalarRange.unbounded();
            bounded = false;
            rangeAnswered = true;
            return this;
        }

        @Override
        public Plants.PositionReferenceStep nativeUnits() {
            answerScale(PositionMappingKind.NATIVE, 1.0, "nativeUnits()");
            return this;
        }

        @Override
        public Plants.PositionReferenceStep scaleToNative(double nativeUnitsPerPlantUnit) {
            answerScale(PositionMappingKind.SCALE, nativeUnitsPerPlantUnit, "scaleToNative(...)");
            return this;
        }

        @Override
        public Plants.PositionToleranceStep rangeMapsToNative(double nativeAtPlantMin,
                                                               double nativeAtPlantMax) {
            requireMutable("rangeMapsToNative(...)");
            requireRangeAnswered("rangeMapsToNative(...)");
            if (!bounded) {
                throw new IllegalStateException("rangeMapsToNative(...) is only valid after bounded(...)");
            }
            requireMappingUnanswered();
            requireFinitePosition(nativeAtPlantMin,
                    "FtcActuators.rangeMapsToNative(...)", "nativeAtPlantMin", "native units");
            requireFinitePosition(nativeAtPlantMax,
                    "FtcActuators.rangeMapsToNative(...)", "nativeAtPlantMax", "native units");
            double plantSpan = plantMax - plantMin;
            if (!(plantSpan > 0.0) || !Double.isFinite(plantSpan)) {
                throw new IllegalArgumentException("rangeMapsToNative(...) requires finite Plant bounds with min < max");
            }
            requireFiniteNonZero((nativeAtPlantMax - nativeAtPlantMin) / plantSpan,
                    "native endpoint scale");
            mappingKind = PositionMappingKind.ENDPOINTS;
            this.nativeAtPlantMin = nativeAtPlantMin;
            this.nativeAtPlantMax = nativeAtPlantMax;
            referenceKind = PositionReferenceKind.STATIC;
            plantReference = plantMin;
            nativeReference = nativeAtPlantMin;
            return this;
        }

        @Override
        public Plants.PositionToleranceStep alreadyReferenced() {
            return answerStaticReference(0.0, 0.0, "alreadyReferenced()");
        }

        @Override
        public Plants.PositionToleranceStep plantPositionMapsToNative(double plantPosition,
                                                                       double nativePosition) {
            return answerStaticReference(plantPosition, nativePosition,
                    "plantPositionMapsToNative(...)");
        }

        @Override
        public Plants.PositionToleranceStep assumeCurrentPositionIs(double plantPosition) {
            requireMutable("assumeCurrentPositionIs(...)");
            double checkedPlantPosition = requireFinitePosition(
                    plantPosition,
                    "FtcActuators.assumeCurrentPositionIs(...)",
                    "plantPosition",
                    "plant units");
            requireReferencePending("assumeCurrentPositionIs(...)");
            assumePlantPosition = checkedPlantPosition;
            referenceKind = PositionReferenceKind.ASSUME_CURRENT;
            return this;
        }

        @Override
        public Plants.PositionToleranceStep needsReference(String reason) {
            requireMutable("needsReference(...)");
            requireReferencePending("needsReference(...)");
            if (reason == null || reason.trim().isEmpty()) {
                throw new IllegalArgumentException("needsReference(...) reason must be nonblank");
            }
            referenceReason = reason.trim();
            referenceKind = PositionReferenceKind.NEEDS_REFERENCE;
            return this;
        }

        @Override
        public Plants.TargetStep<PositionPlant> positionTolerance(double tolerance) {
            requireMutable("positionTolerance(...)");
            if (referenceKind == null) {
                throw new IllegalStateException("Answer position reference before positionTolerance(...)");
            }
            if (positionToleranceAnswered) {
                throw new IllegalStateException("positionTolerance(...) has already been answered for this feedback position Plant");
            }
            requireFiniteTolerance(tolerance, "positionTolerance");
            positionTolerance = tolerance;
            positionToleranceAnswered = true;
            return this;
        }

        @Override
        protected final void validateRecipe() {
            validatePositionSourceRecipe();
            if (!periodicityAnswered) {
                throw new IllegalStateException("Position builder requires nonPeriodic() or periodic(period)");
            }
            if (!rangeAnswered) {
                throw new IllegalStateException("Position builder requires bounded(...) or unbounded()");
            }
            if (mappingKind == null) {
                throw new IllegalStateException("Position builder requires a Plant-to-native mapping answer");
            }
            if (referenceKind == null) {
                throw new IllegalStateException("Position builder requires a reference answer");
            }
            if (!positionToleranceAnswered) {
                throw new IllegalStateException("Feedback position builder requires positionTolerance(...) in plant position units");
            }
        }

        @Override
        protected final ScalarRange configuredTargetRange() {
            if (!rangeAnswered) {
                throw new IllegalStateException("Position builder requires bounded(...) or unbounded() before target selection");
            }
            return range;
        }

        @Override
        protected final Plants.TargetStep<PositionPlant> createSharedTargetStep() {
            Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep> periodicity =
                    createSharedPositionStart();
            Plants.FeedbackPositionBoundsStep bounds = periodic
                    ? periodicity.periodic(period)
                    : periodicity.nonPeriodic();

            Plants.PositionToleranceStep tolerance;
            if (bounded) {
                Plants.FeedbackBoundedPositionMappingStep mapping =
                        bounds.bounded(plantMin, plantMax);
                if (mappingKind == PositionMappingKind.ENDPOINTS) {
                    tolerance = mapping.rangeMapsToNative(
                            nativeAtPlantMin, nativeAtPlantMax);
                } else {
                    Plants.PositionReferenceStep reference =
                            mappingKind == PositionMappingKind.NATIVE
                                    ? mapping.nativeUnits()
                                    : mapping.scaleToNative(nativePerPlantUnit);
                    tolerance = applyReference(reference);
                }
            } else {
                Plants.FeedbackUnboundedPositionMappingStep mapping = bounds.unbounded();
                Plants.PositionReferenceStep reference =
                        mappingKind == PositionMappingKind.NATIVE
                                ? mapping.nativeUnits()
                                : mapping.scaleToNative(nativePerPlantUnit);
                tolerance = applyReference(reference);
            }
            return tolerance.positionTolerance(positionTolerance);
        }

        protected abstract void validatePositionSourceRecipe();

        protected abstract Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep>
        createSharedPositionStart();

        private Plants.PositionToleranceStep applyReference(Plants.PositionReferenceStep reference) {
            switch (referenceKind) {
                case STATIC:
                    return reference.plantPositionMapsToNative(plantReference, nativeReference);
                case ASSUME_CURRENT:
                    return reference.assumeCurrentPositionIs(assumePlantPosition);
                case NEEDS_REFERENCE:
                    return reference.needsReference(referenceReason);
                default:
                    throw new IllegalStateException("Unsupported position reference answer: "
                            + referenceKind);
            }
        }

        private void answerPeriodicity(boolean periodic, double period, String operation) {
            requireMutable(operation);
            if (periodicityAnswered) {
                throw new IllegalStateException("Position periodicity has already been answered");
            }
            this.periodic = periodic;
            this.period = period;
            periodicityAnswered = true;
        }

        private void answerScale(PositionMappingKind kind, double scale, String operation) {
            requireMutable(operation);
            requireRangeAnswered(operation);
            requireMappingUnanswered();
            nativePerPlantUnit = requireFiniteNonZero(scale, "nativeUnitsPerPlantUnit");
            mappingKind = kind;
        }

        private Plants.PositionToleranceStep answerStaticReference(
                double plantPosition,
                double nativePosition,
                String operation) {
            requireMutable(operation);
            double checkedPlantPosition = requireFinitePosition(
                    plantPosition, "FtcActuators." + operation,
                    "plantPosition", "plant units");
            double checkedNativePosition = requireFinitePosition(
                    nativePosition, "FtcActuators." + operation,
                    "nativePosition", "native units");
            requireReferencePending(operation);
            plantReference = checkedPlantPosition;
            nativeReference = checkedNativePosition;
            referenceKind = PositionReferenceKind.STATIC;
            return this;
        }

        private void requirePeriodicityAnswered(String operation) {
            if (!periodicityAnswered) {
                throw new IllegalStateException(operation
                        + " requires nonPeriodic() or periodic(period)");
            }
        }

        private void requireRangeAnswered(String operation) {
            if (!rangeAnswered) {
                throw new IllegalStateException(operation
                        + " requires bounded(...) or unbounded()");
            }
        }

        private void requireRangeUnanswered() {
            if (rangeAnswered) {
                throw new IllegalStateException("Position bounds have already been answered");
            }
        }

        private void requireMappingUnanswered() {
            if (mappingKind != null) {
                throw new IllegalStateException("Position mapping has already been answered");
            }
        }

        private void requireReferencePending(String operation) {
            if (mappingKind == null) {
                throw new IllegalStateException("Choose nativeUnits() or scaleToNative(...) before "
                        + operation);
            }
            if (referenceKind != null) {
                throw new IllegalStateException("Position reference has already been answered");
            }
        }
    }

    private static final class MotorPositionBuilder extends BasePositionBuilder implements MotorPositionControlStep,
            MotorDeviceManagedPositionStep, MotorRegulatedPositionFeedbackStep, MotorRegulatedPositionRegulatorStep {
        private final MotorBuilder parent;
        private final DeviceManagedPositionConfig deviceConfig = new DeviceManagedPositionConfig();
        private PositionControlKind controlKind;
        private Supplier<ScalarSource> feedback;
        private ScalarRegulator regulator;

        private MotorPositionBuilder(MotorBuilder parent) {
            super(parent.lifecycle, "motor position Plant");
            this.parent = Objects.requireNonNull(parent, "parent");
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep>
        deviceManagedWithDefaults() {
            answerControl(PositionControlKind.DEVICE_MANAGED, "deviceManagedWithDefaults()");
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep deviceManaged() {
            answerControl(PositionControlKind.DEVICE_MANAGED, "deviceManaged()");
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep maxPower(double maxPower) {
            requireDeviceManaged("maxPower(...)");
            if (maxPower < 0.0) throw new IllegalArgumentException("maxPower must be >= 0");
            deviceConfig.maxPower = maxPower;
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep outerPositionP(double outerPositionP) {
            requireDeviceManaged("outerPositionP(...)");
            deviceConfig.outerPositionP = outerPositionP;
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep innerVelocityPidf(double p, double i, double d, double f) {
            requireDeviceManaged("innerVelocityPidf(...)");
            deviceConfig.innerVelocityPidf = new double[]{p, i, d, f};
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep devicePositionToleranceTicks(int ticks) {
            requireDeviceManaged("devicePositionToleranceTicks(...)");
            if (ticks < 0)
                throw new IllegalArgumentException("devicePositionToleranceTicks must be >= 0");
            deviceConfig.devicePositionToleranceTicks = ticks;
            return this;
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep>
        doneDeviceManaged() {
            requireDeviceManaged("doneDeviceManaged()");
            return this;
        }

        @Override
        public MotorRegulatedPositionFeedbackStep regulated() {
            answerControl(PositionControlKind.REGULATED, "regulated()");
            return this;
        }

        @Override
        public MotorRegulatedPositionRegulatorStep internalEncoder() {
            requireFeedbackUnanswered("internalEncoder()");
            String selectedName = requireSingleMotorFeedbackName(parent.specs, "position");
            feedback = () -> FtcSensors.motorPositionTicks(parent.hw, selectedName);
            return this;
        }

        @Override
        public MotorRegulatedPositionRegulatorStep internalEncoder(String motorName) {
            requireFeedbackUnanswered("internalEncoder(...)");
            String selectedName = requireSelectedMotorFeedbackName(
                    parent.specs, Objects.requireNonNull(motorName, "motorName"), "position");
            feedback = () -> FtcSensors.motorPositionTicks(parent.hw, selectedName);
            return this;
        }

        @Override
        public MotorRegulatedPositionRegulatorStep averageInternalEncoders() {
            requireFeedbackUnanswered("averageInternalEncoders()");
            ensureMotorFeedbackAvailable(parent.specs, "position");
            feedback = () -> internalPositionFeedback(parent.hw, parent.specs, null, true);
            return this;
        }

        @Override
        public MotorRegulatedPositionRegulatorStep externalEncoder(String name) {
            return externalEncoder(name, Direction.FORWARD);
        }

        @Override
        public MotorRegulatedPositionRegulatorStep externalEncoder(String name, Direction direction) {
            requireFeedbackUnanswered("externalEncoder(...)");
            String checkedName = requireFeedbackName(name);
            Direction checkedDirection = Objects.requireNonNull(direction, "direction");
            feedback = () -> FtcSensors.motorPositionTicks(
                    parent.hw, checkedName, checkedDirection);
            return this;
        }

        @Override
        public MotorRegulatedPositionRegulatorStep nativeFeedback(ScalarSource source) {
            requireFeedbackUnanswered("nativeFeedback(...)");
            ScalarSource checked = Objects.requireNonNull(source, "source");
            feedback = () -> checked;
            return this;
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep>
        regulator(ScalarRegulator regulator) {
            requireMutable("regulator(...)");
            if (controlKind != PositionControlKind.REGULATED || feedback == null) {
                throw new IllegalStateException("Choose regulated position feedback before regulator(...)");
            }
            if (this.regulator != null) {
                throw new IllegalStateException("regulator(...) has already been answered for this motor position Plant");
            }
            this.regulator = Objects.requireNonNull(regulator, "regulator");
            return this;
        }

        @Override
        protected void validatePositionSourceRecipe() {
            if (controlKind == null) {
                throw new IllegalStateException("Motor position builder requires deviceManagedWithDefaults(), deviceManaged(), or regulated()");
            }
            if (controlKind == PositionControlKind.REGULATED) {
                parent.requireDefaultGroupScalingForRegulated("position");
                if (feedback == null || regulator == null) {
                    throw new IllegalStateException("Regulated motor position requires a feedback answer "
                            + "(internalEncoder(), averageInternalEncoders(), externalEncoder(...), or "
                            + "nativeFeedback(...)) and regulator(...)");
                }
            } else {
                parent.ensureFeedbackScalesNonZero("device-managed motor position");
            }
        }

        @Override
        protected Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep>
        createSharedPositionStart() {
            if (controlKind == PositionControlKind.DEVICE_MANAGED) {
                ScalarSource measurement = parent.groupedMotorPositionMeasurement();
                PositionOutput output = parent.groupedMotorPosition(deviceConfig);
                Plants.DeviceManagedPositionStep start = Plants.fromOutputs()
                        .deviceManagedPosition(output, measurement);
                return start.searchPowerOutput(parent.groupedMotorPower());
            }
            ScalarSource measurement = feedback.get();
            PowerOutput power = parent.groupedMotorPower();
            return Plants.fromOutputs().regulatedPosition(power, measurement, regulator);
        }

        private void answerControl(PositionControlKind answer, String operation) {
            requireMutable(operation);
            if (controlKind != null) {
                throw new IllegalStateException("Motor position control ownership has already been answered");
            }
            controlKind = answer;
        }

        private void requireDeviceManaged(String operation) {
            requireMutable(operation);
            if (controlKind != PositionControlKind.DEVICE_MANAGED) {
                throw new IllegalStateException(operation + " requires deviceManaged()");
            }
        }

        private void requireFeedbackUnanswered(String operation) {
            requireMutable(operation);
            if (controlKind != PositionControlKind.REGULATED) {
                throw new IllegalStateException(operation + " requires regulated()");
            }
            if (feedback != null) {
                throw new IllegalStateException("Regulated motor position feedback has already been answered");
            }
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Servo builder
    // ---------------------------------------------------------------------------------------------

    private static final class ServoBuilder implements ServoGroupAddedStep {
        private final HardwareMap hw;
        private final RecipeLifecycle lifecycle;
        private final List<MotorBuilder.Spec> specs = new ArrayList<>();
        private int lastIndex;

        private ServoBuilder(HardwareMap hw,
                             RecipeLifecycle lifecycle,
                             String name,
                             Direction direction) {
            this.hw = Objects.requireNonNull(hw, "HardwareMap is required");
            this.lifecycle = Objects.requireNonNull(lifecycle, "lifecycle");
            addServoInternal(name, direction);
        }

        private void addServoInternal(String name, Direction direction) {
            lifecycle.requireMutable("standard-servo group configuration");
            lastIndex = addActuatorSpec(specs, "standard-servo", name, direction);
        }

        @Override
        public ServoGroupAddedStep andServo(String name, Direction direction) {
            addServoInternal(name, direction);
            return this;
        }

        @Override
        public ServoGroupAddedStep scale(double scale) {
            lifecycle.requireMutable("standard-servo scale(...)");
            specs.get(lastIndex).scale = scale;
            return this;
        }

        @Override
        public ServoGroupAddedStep bias(double bias) {
            lifecycle.requireMutable("standard-servo bias(...)");
            specs.get(lastIndex).bias = bias;
            return this;
        }

        @Override
        public ServoPositionPeriodicityStep position() {
            lifecycle.requireMutable("position()");
            return new ServoPositionBuilder(this);
        }

        private PositionOutput groupedServoPosition() {
            if (specs.size() == 1) {
                MotorBuilder.Spec spec = specs.get(0);
                return FtcHardware.servoPosition(hw, spec.name, spec.direction);
            }
            List<PositionOutput> outs = new ArrayList<>();
            double[] scales = new double[specs.size()];
            double[] biases = new double[specs.size()];
            for (int i = 0; i < specs.size(); i++) {
                MotorBuilder.Spec spec = specs.get(i);
                outs.add(FtcHardware.servoPosition(hw, spec.name, spec.direction));
                scales[i] = spec.scale;
                biases[i] = spec.bias;
            }
            return new GroupedPositionOutput(outs, scales, biases);
        }
    }

    private static final class ServoPositionBuilder extends FtcTargetBuilder<PositionPlant>
            implements ServoPositionPeriodicityStep,
            ServoPositionBoundsStep,
            ServoBoundedPositionMappingStep {
        private final ServoBuilder parent;
        private ScalarRange range;
        private double plantMin;
        private double plantMax;
        private boolean periodicityAnswered;
        private boolean periodic;
        private double period = Double.NaN;
        private boolean boundsAnswered;
        private PositionMappingKind mappingKind;
        private double nativeAtPlantMin;
        private double nativeAtPlantMax;

        private ServoPositionBuilder(ServoBuilder parent) {
            super(parent.lifecycle, "standard-servo position Plant");
            this.parent = Objects.requireNonNull(parent, "parent");
        }

        @Override
        public ServoPositionBoundsStep nonPeriodic() {
            answerPeriodicity(false, Double.NaN, "nonPeriodic()");
            return this;
        }

        @Override
        public ServoPositionBoundsStep periodic(double period) {
            requireFinitePositive(period, "period");
            answerPeriodicity(true, period, "periodic(...)");
            return this;
        }

        @Override
        public ServoBoundedPositionMappingStep bounded(double min, double max) {
            requireMutable("bounded(...)");
            if (!periodicityAnswered) {
                throw new IllegalStateException("bounded(...) requires nonPeriodic() or periodic(period)");
            }
            if (boundsAnswered) {
                throw new IllegalStateException("Standard-servo position bounds have already been answered");
            }
            range = ScalarRange.bounded(min, max);
            plantMin = min;
            plantMax = max;
            boundsAnswered = true;
            return this;
        }

        @Override
        public Plants.TargetStep<PositionPlant> nativeUnits() {
            requireMappingPending("nativeUnits()");
            mappingKind = PositionMappingKind.NATIVE;
            return this;
        }

        @Override
        public Plants.TargetStep<PositionPlant> rangeMapsToNative(double nativeAtPlantMin,
                                                                  double nativeAtPlantMax) {
            requireMappingPending("rangeMapsToNative(...)");
            requireFinitePosition(nativeAtPlantMin,
                    "FtcActuators.rangeMapsToNative(...)", "nativeAtPlantMin", "native units");
            requireFinitePosition(nativeAtPlantMax,
                    "FtcActuators.rangeMapsToNative(...)", "nativeAtPlantMax", "native units");
            double plantSpan = plantMax - plantMin;
            if (!(plantSpan > 0.0) || !Double.isFinite(plantSpan)) {
                throw new IllegalArgumentException("rangeMapsToNative(...) requires finite Plant bounds with min < max");
            }
            requireFiniteNonZero((nativeAtPlantMax - nativeAtPlantMin) / plantSpan,
                    "native endpoint scale");
            mappingKind = PositionMappingKind.ENDPOINTS;
            this.nativeAtPlantMin = nativeAtPlantMin;
            this.nativeAtPlantMax = nativeAtPlantMax;
            return this;
        }

        @Override
        protected void validateRecipe() {
            if (!periodicityAnswered) {
                throw new IllegalStateException("Standard-servo position builder requires nonPeriodic() or periodic(period)");
            }
            if (!boundsAnswered) {
                throw new IllegalStateException("Standard-servo position builder requires bounded(...)");
            }
            if (mappingKind == null) {
                throw new IllegalStateException("Standard-servo position builder requires nativeUnits() or rangeMapsToNative(...)");
            }
        }

        @Override
        protected ScalarRange configuredTargetRange() {
            if (!boundsAnswered) {
                throw new IllegalStateException("Standard-servo position builder requires bounded(...) before target selection");
            }
            return range;
        }

        @Override
        protected Plants.TargetStep<PositionPlant> createSharedTargetStep() {
            Plants.PositionPeriodicityStep<Plants.CommandedPositionBoundsStep> periodicity =
                    Plants.fromOutputs().commandedPosition(parent.groupedServoPosition());
            Plants.CommandedPositionBoundsStep bounds = periodic
                    ? periodicity.periodic(period)
                    : periodicity.nonPeriodic();
            Plants.CommandedBoundedPositionMappingStep mapping =
                    bounds.bounded(plantMin, plantMax);
            return mappingKind == PositionMappingKind.NATIVE
                    ? mapping.nativeUnits()
                    : mapping.rangeMapsToNative(nativeAtPlantMin, nativeAtPlantMax);
        }

        private void answerPeriodicity(boolean periodic, double period, String operation) {
            requireMutable(operation);
            if (periodicityAnswered) {
                throw new IllegalStateException("Standard-servo position periodicity has already been answered");
            }
            this.periodic = periodic;
            this.period = period;
            periodicityAnswered = true;
        }

        private void requireMappingPending(String operation) {
            requireMutable(operation);
            if (!boundsAnswered) {
                throw new IllegalStateException(operation + " requires bounded(...)");
            }
            if (mappingKind != null) {
                throw new IllegalStateException("Standard-servo position mapping has already been answered");
            }
        }
    }

    // ---------------------------------------------------------------------------------------------
    // CR servo builder
    // ---------------------------------------------------------------------------------------------

    private static final class CrServoBuilder implements CrServoGroupAddedStep {
        private final HardwareMap hw;
        private final RecipeLifecycle lifecycle;
        private final List<MotorBuilder.Spec> specs = new ArrayList<>();
        private int lastIndex;

        private CrServoBuilder(HardwareMap hw,
                               RecipeLifecycle lifecycle,
                               String name,
                               Direction direction) {
            this.hw = Objects.requireNonNull(hw, "HardwareMap is required");
            this.lifecycle = Objects.requireNonNull(lifecycle, "lifecycle");
            addCrServoInternal(name, direction);
        }

        private void addCrServoInternal(String name, Direction direction) {
            lifecycle.requireMutable("CR-servo group configuration");
            lastIndex = addActuatorSpec(specs, "CR servo", name, direction);
        }

        @Override
        public CrServoGroupAddedStep andCrServo(String name, Direction direction) {
            addCrServoInternal(name, direction);
            return this;
        }

        @Override
        public CrServoGroupAddedStep scale(double scale) {
            lifecycle.requireMutable("CR-servo scale(...)");
            specs.get(lastIndex).scale = scale;
            return this;
        }

        @Override
        public CrServoGroupAddedStep bias(double bias) {
            lifecycle.requireMutable("CR-servo bias(...)");
            specs.get(lastIndex).bias = bias;
            return this;
        }

        @Override
        public Plants.TargetStep<Plant> power() {
            lifecycle.requireMutable("power()");
            return new PowerTargetBuilder(lifecycle, this::groupedCrServoPowerWithMappings);
        }

        @Override
        public CrServoPositionControlStep position() {
            lifecycle.requireMutable("position()");
            return new CrServoPositionBuilder(this);
        }

        private PowerOutput groupedCrServoPower() {
            if (specs.size() == 1) {
                MotorBuilder.Spec spec = specs.get(0);
                return FtcHardware.crServoPower(hw, spec.name, spec.direction);
            }
            List<PowerOutput> outs = new ArrayList<>();
            for (MotorBuilder.Spec spec : specs)
                outs.add(FtcHardware.crServoPower(hw, spec.name, spec.direction));
            return new GroupedPowerOutput(outs);
        }

        private PowerOutput groupedCrServoPowerWithMappings() {
            if (specs.size() == 1) {
                MotorBuilder.Spec spec = specs.get(0);
                return new ScaledBiasedPowerOutput(FtcHardware.crServoPower(hw, spec.name, spec.direction), spec.scale, spec.bias);
            }
            List<PowerOutput> outs = new ArrayList<>();
            double[] scales = new double[specs.size()];
            double[] biases = new double[specs.size()];
            for (int i = 0; i < specs.size(); i++) {
                MotorBuilder.Spec spec = specs.get(i);
                outs.add(FtcHardware.crServoPower(hw, spec.name, spec.direction));
                scales[i] = spec.scale;
                biases[i] = spec.bias;
            }
            return new GroupedPowerOutput(outs, scales, biases);
        }

        private void requireDefaultGroupScalingForRegulated() {
            if (specs.size() <= 1) return;
            for (MotorBuilder.Spec spec : specs)
                if (Math.abs(spec.scale - 1.0) > 1e-9 || Math.abs(spec.bias) > 1e-9)
                    throw new IllegalStateException("Regulated CR-servo position control requires default group scaling/bias through FtcActuators");
        }
    }

    private static final class CrServoPositionBuilder extends BasePositionBuilder implements CrServoPositionControlStep,
            CrServoRegulatedPositionFeedbackStep, CrServoRegulatedPositionRegulatorStep {
        private final CrServoBuilder parent;
        private Supplier<ScalarSource> feedback;
        private ScalarRegulator regulator;
        private boolean regulatedAnswered;

        private CrServoPositionBuilder(CrServoBuilder parent) {
            super(parent.lifecycle, "CR-servo position Plant");
            this.parent = Objects.requireNonNull(parent, "parent");
        }

        @Override
        public CrServoRegulatedPositionFeedbackStep regulated() {
            requireMutable("regulated()");
            if (regulatedAnswered) {
                throw new IllegalStateException("CR-servo position control ownership has already been answered");
            }
            regulatedAnswered = true;
            return this;
        }

        @Override
        public CrServoRegulatedPositionRegulatorStep externalEncoder(String name) {
            return externalEncoder(name, Direction.FORWARD);
        }

        @Override
        public CrServoRegulatedPositionRegulatorStep externalEncoder(String name, Direction direction) {
            requireFeedbackUnanswered("externalEncoder(...)");
            String checkedName = requireFeedbackName(name);
            Direction checkedDirection = Objects.requireNonNull(direction, "direction");
            feedback = () -> FtcSensors.motorPositionTicks(
                    parent.hw, checkedName, checkedDirection);
            return this;
        }

        @Override
        public CrServoRegulatedPositionRegulatorStep nativeFeedback(ScalarSource source) {
            requireFeedbackUnanswered("nativeFeedback(...)");
            ScalarSource checked = Objects.requireNonNull(source, "source");
            feedback = () -> checked;
            return this;
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep>
        regulator(ScalarRegulator regulator) {
            requireMutable("regulator(...)");
            if (!regulatedAnswered || feedback == null) {
                throw new IllegalStateException("Choose regulated CR-servo position feedback before regulator(...)");
            }
            if (this.regulator != null) {
                throw new IllegalStateException("regulator(...) has already been answered for this CR-servo position Plant");
            }
            this.regulator = Objects.requireNonNull(regulator, "regulator");
            return this;
        }

        @Override
        protected void validatePositionSourceRecipe() {
            if (!regulatedAnswered) {
                throw new IllegalStateException("CR-servo position builder requires regulated()");
            }
            parent.requireDefaultGroupScalingForRegulated();
            if (feedback == null || regulator == null) {
                throw new IllegalStateException("Regulated CR-servo position requires externalEncoder(...) "
                        + "or nativeFeedback(...), followed by regulator(...)");
            }
        }

        @Override
        protected Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep>
        createSharedPositionStart() {
            ScalarSource measurement = feedback.get();
            PowerOutput power = parent.groupedCrServoPower();
            return Plants.fromOutputs().regulatedPosition(power, measurement, regulator);
        }

        private void requireFeedbackUnanswered(String operation) {
            requireMutable(operation);
            if (!regulatedAnswered) {
                throw new IllegalStateException(operation + " requires regulated()");
            }
            if (feedback != null) {
                throw new IllegalStateException("Regulated CR-servo position feedback has already been answered");
            }
        }
    }

    private static final class ScaledBiasedPowerOutput implements PowerOutput {
        private final PowerOutput output;
        private final double scale;
        private final double bias;
        private double last;

        private ScaledBiasedPowerOutput(PowerOutput output, double scale, double bias) {
            this.output = Objects.requireNonNull(output, "output");
            this.scale = scale;
            this.bias = bias;
        }

        @Override
        public void setPower(double power) {
            last = power;
            output.setPower(scale * power + bias);
        }

        @Override
        public double getCommandedPower() {
            return last;
        }

        @Override
        public void stop() {
            output.stop();
            last = 0.0;
        }
    }

    private static final class GroupedPowerOutput implements PowerOutput {
        private final List<PowerOutput> outputs;
        private final double[] scales;
        private final double[] biases;
        private double last;

        private GroupedPowerOutput(List<PowerOutput> outputs) {
            this(outputs, null, null);
        }

        private GroupedPowerOutput(List<PowerOutput> outputs, double[] scales, double[] biases) {
            this.outputs = new ArrayList<>(Objects.requireNonNull(outputs, "outputs"));
            if (this.outputs.isEmpty())
                throw new IllegalArgumentException("outputs must not be empty");
            this.scales = scales != null ? scales.clone() : null;
            this.biases = biases != null ? biases.clone() : null;
            if ((this.scales != null && this.scales.length != this.outputs.size())
                    || (this.biases != null && this.biases.length != this.outputs.size())) {
                throw new IllegalArgumentException("outputs/scales/biases must have matching lengths");
            }
        }

        @Override
        public void setPower(double power) {
            last = power;
            for (int i = 0; i < outputs.size(); i++) {
                double childPower = power;
                if (scales != null) childPower = scales[i] * power + biases[i];
                outputs.get(i).setPower(childPower);
            }
        }

        @Override
        public double getCommandedPower() {
            return last;
        }

        @Override
        public void stop() {
            for (PowerOutput output : outputs) output.stop();
            last = 0.0;
        }
    }

    private static final class GroupedPositionOutput implements PositionOutput {
        private final List<PositionOutput> outputs;
        private final double[] scales;
        private final double[] biases;
        private double last;

        private GroupedPositionOutput(List<PositionOutput> outputs, double[] scales, double[] biases) {
            this.outputs = new ArrayList<>(Objects.requireNonNull(outputs, "outputs"));
            this.scales = Objects.requireNonNull(scales, "scales");
            this.biases = Objects.requireNonNull(biases, "biases");
            if (this.outputs.isEmpty() || this.outputs.size() != scales.length || scales.length != biases.length)
                throw new IllegalArgumentException("outputs/scales/biases must be non-empty and matching");
        }

        @Override
        public void setPosition(double position) {
            last = position;
            for (int i = 0; i < outputs.size(); i++)
                outputs.get(i).setPosition(scales[i] * position + biases[i]);
        }

        @Override
        public double getCommandedPosition() {
            return last;
        }

        @Override
        public void stop() {
            for (PositionOutput output : outputs) output.stop();
        }
    }

    private static final class GroupedVelocityOutput implements VelocityOutput {
        private final List<VelocityOutput> outputs;
        private final double[] scales;
        private final double[] biases;
        private double last;

        private GroupedVelocityOutput(List<VelocityOutput> outputs, double[] scales, double[] biases) {
            this.outputs = new ArrayList<>(Objects.requireNonNull(outputs, "outputs"));
            this.scales = Objects.requireNonNull(scales, "scales");
            this.biases = Objects.requireNonNull(biases, "biases");
            if (this.outputs.isEmpty() || this.outputs.size() != scales.length || scales.length != biases.length)
                throw new IllegalArgumentException("outputs/scales/biases must be non-empty and matching");
        }

        @Override
        public void setVelocity(double velocity) {
            last = velocity;
            for (int i = 0; i < outputs.size(); i++)
                outputs.get(i).setVelocity(scales[i] * velocity + biases[i]);
        }

        @Override
        public double getCommandedVelocity() {
            return last;
        }

        @Override
        public void stop() {
            for (VelocityOutput output : outputs) output.stop();
            last = 0.0;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------------------------

    private static void ensureMotorFeedbackAvailable(List<MotorBuilder.Spec> motorSpecs, String domain) {
        if (motorSpecs == null || motorSpecs.isEmpty()) {
            throw new IllegalStateException("Internal " + domain + " encoder feedback requires a "
                    + "motor builder. Use externalEncoder(...) or nativeFeedback(...) instead.");
        }
    }

    private static String requireSingleMotorFeedbackName(List<MotorBuilder.Spec> motorSpecs,
                                                         String domain) {
        ensureMotorFeedbackAvailable(motorSpecs, domain);
        if (motorSpecs.size() != 1) {
            throw new IllegalStateException("internalEncoder() is ambiguous for a "
                    + motorSpecs.size() + "-motor group. Choose internalEncoder(\"name\"), "
                    + "averageInternalEncoders(), or externalEncoder(...).");
        }
        return motorSpecs.get(0).name;
    }

    private static String requireSelectedMotorFeedbackName(List<MotorBuilder.Spec> motorSpecs,
                                                           String motorName,
                                                           String domain) {
        ensureMotorFeedbackAvailable(motorSpecs, domain);
        String checkedName = requireFeedbackName(motorName);
        for (MotorBuilder.Spec spec : motorSpecs) {
            if (FtcHardwareNameGroups.sameConfiguredName(spec.name, checkedName)) {
                return spec.name;
            }
        }
        throw new IllegalStateException("internalEncoder(\"" + motorName
                + "\") does not match any selected motor");
    }

    private static String requireFeedbackName(String name) {
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        String checked = name;
        if (checked.trim().isEmpty()) {
            throw new IllegalArgumentException("encoder name must be nonblank");
        }
        return checked;
    }

    private static ScalarSource averageSources(List<ScalarSource> sources) {
        if (sources == null || sources.isEmpty())
            throw new IllegalArgumentException("sources must not be empty");
        List<ScalarSource> copy = new ArrayList<>(sources);
        return new ScalarSource() {
            @Override
            public double getAsDouble(edu.ftcphoenix.fw.core.time.LoopClock clock) {
                double sum = 0.0;
                int count = 0;
                for (ScalarSource source : copy) {
                    sum += source.getAsDouble(clock);
                    count++;
                }
                return count > 0 ? sum / count : Double.NaN;
            }

            @Override
            public void reset() {
                for (ScalarSource source : copy) source.reset();
            }
        }.memoized();
    }

    private static ScalarSource averageInverseMappedSources(List<ScalarSource> sources, double[] scales, double[] biases) {
        if (sources == null || sources.isEmpty())
            throw new IllegalArgumentException("sources must not be empty");
        List<ScalarSource> copy = new ArrayList<>(sources);
        return new ScalarSource() {
            @Override
            public double getAsDouble(edu.ftcphoenix.fw.core.time.LoopClock clock) {
                double sum = 0.0;
                int count = 0;
                for (int i = 0; i < copy.size(); i++) {
                    if (Math.abs(scales[i]) < 1e-9) return Double.NaN;
                    double v = copy.get(i).getAsDouble(clock);
                    if (!Double.isFinite(v)) return Double.NaN;
                    sum += (v - biases[i]) / scales[i];
                    count++;
                }
                return count > 0 ? sum / count : Double.NaN;
            }

            @Override
            public void reset() {
                for (ScalarSource source : copy) source.reset();
            }
        }.memoized();
    }

    private static double requireFiniteNonZero(double value, String name) {
        if (!Double.isFinite(value) || Math.abs(value) < 1e-12)
            throw new IllegalArgumentException(name + " must be finite and non-zero");
        return value;
    }

    private static void requireFinitePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be finite and > 0, got " + value);
        }
    }

    private static void requireFiniteTolerance(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and >= 0, got " + value);
        }
    }

    /**
     * Return an unchanged finite FTC position-reference value or fail before builder mutation.
     *
     * @param value candidate primitive value
     * @param operation student-facing entry point used in the diagnostic
     * @param argument student-facing argument name used in the diagnostic
     * @param units plant or native units used in the diagnostic
     * @return the unchanged finite value, including its signed-zero representation
     * @throws IllegalArgumentException if {@code value} is not finite
     */
    private static double requireFinitePosition(double value,
                                                String operation,
                                                String argument,
                                                String units) {
        if (!Double.isFinite(value))
            throw new IllegalArgumentException(operation + ": " + argument
                    + " must be finite in " + units + ", got " + value);
        return value;
    }

    private static void applyDeviceManagedPositionConfig(DcMotorEx motor, String motorName, DeviceManagedPositionConfig cfg) {
        try {
            if (cfg.outerPositionP != null) motor.setPositionPIDFCoefficients(cfg.outerPositionP);
            if (cfg.innerVelocityPidf != null) {
                double[] c = cfg.innerVelocityPidf;
                motor.setVelocityPIDFCoefficients(c[0], c[1], c[2], c[3]);
            }
            if (cfg.devicePositionToleranceTicks != null)
                motor.setTargetPositionTolerance(cfg.devicePositionToleranceTicks);
        } catch (RuntimeException ex) {
            throw new IllegalStateException("Failed to apply device-managed position config for motor '" + motorName + "'. Check that the motor supports the requested FTC SDK APIs.", ex);
        }
    }

    private static void applyDeviceManagedVelocityConfig(DcMotorEx motor, String motorName, double[] velocityPidf) {
        try {
            if (velocityPidf != null)
                motor.setVelocityPIDFCoefficients(velocityPidf[0], velocityPidf[1], velocityPidf[2], velocityPidf[3]);
        } catch (RuntimeException ex) {
            throw new IllegalStateException("Failed to apply device-managed velocity config for motor '" + motorName + "'. Check that the motor supports the requested FTC SDK APIs.", ex);
        }
    }
}
