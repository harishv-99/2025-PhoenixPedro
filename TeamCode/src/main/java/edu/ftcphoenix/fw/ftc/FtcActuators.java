package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.actuation.MappedPositionPlant;
import edu.ftcphoenix.fw.actuation.MappedVelocityPlant;
import edu.ftcphoenix.fw.actuation.MultiPlant;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.actuation.RateLimitedPlant;
import edu.ftcphoenix.fw.actuation.RateLimitedPositionPlant;
import edu.ftcphoenix.fw.actuation.ScalarRange;
import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
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
 * {@link Plant#setTarget(double)} use. Native units are what the selected hardware/control path
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
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * PositionPlant lift = FtcActuators.plant(hardwareMap)
 *     .motor("lift", Direction.FORWARD)
 *     .position()
 *     .deviceManaged()
 *         .maxPower(0.8)
 *         .doneDeviceManaged()
 *     .linear()
 *         .bounded(0.0, 4200.0)
 *         .nativeUnits()
 *         .needsReference("lift not homed")
 *     .positionTolerance(20.0)
 *     .build();
 *
 * Plant flywheel = FtcActuators.plant(hardwareMap)
 *     .motor("flywheel", Direction.FORWARD)
 *     .velocity()
 *     .deviceManagedWithDefaults()
 *     .bounded(0.0, 2600.0)
 *     .nativeUnits()
 *     .velocityTolerance(50.0)
 *     .build();
 *
 * PositionPlant claw = FtcActuators.plant(hardwareMap)
 *     .servo("claw", Direction.FORWARD)
 *     .position()
 *     .linear()
 *         .bounded(0.0, 1.0)
 *         .rangeMapsToNative(0.30, 0.80)
 *     .build();
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
         */
        MotorSingleStep motor(String name, Direction direction);

        /**
         * Start building a plant from one or more FTC standard servos.
         */
        ServoSingleStep servo(String name, Direction direction);

        /**
         * Start building a plant from one or more FTC continuous-rotation servos.
         */
        CrServoSingleStep crServo(String name, Direction direction);
    }

    /**
     * Final builder step for generic non-position plants.
     */
    public interface ModifiersStep {
        /**
         * Wrap the current plant in a symmetric rate limiter using plant units per second.
         */
        ModifiersStep rateLimit(double maxDeltaPerSec);

        /**
         * Wrap the current plant in an asymmetric rate limiter using plant units per second.
         */
        ModifiersStep rateLimit(double maxUpPerSec, double maxDownPerSec);

        /**
         * Finish the staged build.
         */
        Plant build();
    }

    /**
     * Final builder step for position plants.
     */
    public interface PositionBuildStep {
        /**
         * Set plant-level completion tolerance in plant units.
         *
         * <p>This affects {@link PositionPlant#atSetpoint()} and planner/task completion. It is not
         * the FTC motor controller's internal target-position tolerance; that native device setting
         * is configured inside {@link MotorDeviceManagedPositionStep#devicePositionToleranceTicks(int)}.</p>
         */
        PositionBuildStep positionTolerance(double tolerance);

        /**
         * Rate-limit plant-unit position target changes symmetrically.
         */
        PositionBuildStep rateLimit(double maxDeltaPerSec);

        /**
         * Rate-limit plant-unit position target changes asymmetrically.
         */
        PositionBuildStep rateLimit(double maxUpPerSec, double maxDownPerSec);

        /**
         * Finish the staged build and return a position-aware plant.
         */
        PositionPlant build();
    }

    /**
     * Final builder step for standard-servo position plants.
     */
    public interface ServoPositionBuildStep {
        /**
         * Rate-limit plant-unit servo target changes symmetrically.
         */
        ServoPositionBuildStep rateLimit(double maxDeltaPerSec);

        /**
         * Rate-limit plant-unit servo target changes asymmetrically.
         */
        ServoPositionBuildStep rateLimit(double maxUpPerSec, double maxDownPerSec);

        /**
         * Finish the staged build and return a position-aware plant.
         */
        PositionPlant build();
    }

    /**
     * Builder step for motor-backed plants.
     */
    public interface MotorSingleStep {
        /**
         * Add another motor to the same plant group.
         */
        MotorGroupAddedStep andMotor(String name, Direction direction);

        /**
         * Build a direct power plant over the selected motor or motor group.
         */
        ModifiersStep power();

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
         * or a Phoenix-regulated loop driven by native feedback.</p>
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

        /**
         * Convenience helper that applies both scale and bias to the most recently added motor.
         */
        default MotorGroupAddedStep scaleBias(double scale, double bias) {
            return scale(scale).bias(bias);
        }
    }

    /**
     * First motor-velocity question: who manages the velocity loop?
     */
    public interface MotorVelocityControlStep {
        /**
         * Use FTC device-managed velocity control with Phoenix defaults and continue to velocity
         * target bounds.
         */
        VelocityBoundsStep deviceManagedWithDefaults();

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
        VelocityBoundsStep doneDeviceManaged();
    }

    /**
     * Required native-feedback question for regulated motor velocity control.
     */
    public interface MotorRegulatedVelocityFeedbackStep {
        /**
         * Select the native velocity feedback source used by the regulator.
         */
        MotorRegulatedVelocityRegulatorStep nativeFeedback(VelocityFeedback feedback);
    }

    /**
     * Required regulator question for regulated motor velocity control.
     */
    public interface MotorRegulatedVelocityRegulatorStep {
        /**
         * Select the regulator that receives plant-unit velocity setpoint and measurement.
         */
        VelocityBoundsStep regulator(ScalarRegulator regulator);
    }

    /**
     * Velocity target-bounds question.
     */
    public interface VelocityBoundsStep {
        /**
         * Declare a finite legal velocity target range in plant velocity units.
         *
         * <p>The supplied values are the same public units used by {@link Plant#setTarget(double)}
         * after the velocity plant is built, not native/controller units.</p>
         */
        VelocityMappingStep bounded(double min, double max);

        /**
         * Declare that the velocity target has no software bounds.
         */
        VelocityMappingStep unbounded();
    }

    /**
     * Velocity unit-mapping question.
     */
    public interface VelocityMappingStep {
        /**
         * Native velocity units and plant velocity units are the same.
         */
        VelocityBuildStep nativeUnits();

        /**
         * Convert plant velocity units to native velocity units using a zero-preserving scale.
         *
         * <p>Velocity mappings intentionally have no offset: plant velocity {@code 0.0} maps to
         * native velocity {@code 0.0}, so stop semantics stay obvious. That is why velocity does
         * not expose {@code rangeMapsToNative(...)}.</p>
         */
        VelocityBuildStep scaleToNative(double nativeUnitsPerPlantVelocityUnit);
    }

    /**
     * Final builder step for motor velocity plants.
     */
    public interface VelocityBuildStep {
        /**
         * Set plant-level completion tolerance in plant velocity units.
         */
        VelocityBuildStep velocityTolerance(double tolerance);

        /**
         * Rate-limit plant-unit velocity target changes symmetrically.
         */
        VelocityBuildStep rateLimit(double maxDeltaPerSec);

        /**
         * Rate-limit plant-unit velocity target changes asymmetrically.
         */
        VelocityBuildStep rateLimit(double maxUpPerSec, double maxDownPerSec);

        /**
         * Finish the staged build.
         */
        Plant build();
    }

    /**
     * First motor-position question: who manages the position loop?
     */
    public interface MotorPositionControlStep {
        /**
         * Use FTC RUN_TO_POSITION with Phoenix defaults and continue to position topology.
         */
        PositionTopologyStep deviceManagedWithDefaults();

        /**
         * Enter the FTC RUN_TO_POSITION tuning branch before continuing to position topology.
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
         * Leave the device-managed tuning branch and continue to position topology.
         */
        PositionTopologyStep doneDeviceManaged();
    }

    /**
     * Required feedback question for regulated motor position control.
     */
    public interface MotorRegulatedPositionFeedbackStep {
        /**
         * Select the native position feedback source used by the regulator.
         */
        MotorRegulatedPositionRegulatorStep nativeFeedback(PositionFeedback feedback);
    }

    /**
     * Required regulator question for regulated motor position control.
     */
    public interface MotorRegulatedPositionRegulatorStep {
        /**
         * Select the regulator that receives plant-unit setpoint and measurement.
         */
        PositionTopologyStep regulator(ScalarRegulator regulator);
    }

    /**
     * Builder step for standard-servo-backed plants.
     */
    public interface ServoSingleStep {
        /**
         * Add another standard servo to the same plant group.
         */
        ServoGroupAddedStep andServo(String name, Direction direction);

        /**
         * Begin the guided standard-servo position builder.
         */
        ServoPositionTopologyStep position();
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

        /**
         * Convenience helper that applies both scale and bias to the most recently added servo.
         */
        default ServoGroupAddedStep scaleBias(double scale, double bias) {
            return scale(scale).bias(bias);
        }
    }

    /**
     * Standard servos expose only a bounded linear position coordinate.
     */
    public interface ServoPositionTopologyStep {
        /**
         * Choose a linear servo plant coordinate.
         */
        ServoPositionBoundsStep linear();
    }

    /**
     * Standard servo bounds question.
     */
    public interface ServoPositionBoundsStep {
        /**
         * Declare the valid caller-facing servo plant range in plant units.
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
        ServoPositionBuildStep nativeUnits();

        /**
         * Map the declared plant range to raw servo endpoints.
         *
         * <p>The arguments are <b>native raw servo values</b> at the plant-range endpoints chosen by
         * {@link ServoPositionBoundsStep#bounded(double, double)}. They are not plant values. In
         * other words, if the declared plant range is {@code [min, max]}, then this method means
         * "plant {@code min} -> native {@code nativeAtPlantMin}" and
         * "plant {@code max} -> native {@code nativeAtPlantMax}".</p>
         */
        ServoPositionBuildStep rangeMapsToNative(double nativeAtPlantMin, double nativeAtPlantMax);
    }

    /**
     * Builder step for continuous-rotation-servo-backed plants.
     */
    public interface CrServoSingleStep {
        /**
         * Add another continuous-rotation servo to the same plant group.
         */
        CrServoGroupAddedStep andCrServo(String name, Direction direction);

        /**
         * Build a direct power plant over the selected CR servo or group.
         */
        ModifiersStep power();

        /**
         * Begin the guided regulated CR-servo position builder.
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

        /**
         * Convenience helper that applies both scale and bias to the most recently added CR servo.
         */
        default CrServoGroupAddedStep scaleBias(double scale, double bias) {
            return scale(scale).bias(bias);
        }
    }

    /**
     * First CR-servo position question: CR servos require regulated position control.
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
         * Select the native position feedback source used by the regulator.
         */
        CrServoRegulatedPositionRegulatorStep nativeFeedback(PositionFeedback feedback);
    }

    /**
     * Required regulator question for regulated CR-servo position control.
     */
    public interface CrServoRegulatedPositionRegulatorStep {
        /**
         * Select the regulator that receives plant-unit setpoint and measurement.
         */
        PositionTopologyStep regulator(ScalarRegulator regulator);
    }

    /**
     * Position topology question shared by motor and CR-servo position plants.
     */
    public interface PositionTopologyStep {
        /**
         * Choose a non-wrapping position coordinate.
         */
        PositionBoundsStep linear();

        /**
         * Choose a periodic coordinate; period is in plant units.
         */
        PositionBoundsStep periodic(double period);
    }

    /**
     * Position bounds question shared by motor and CR-servo position plants.
     */
    public interface PositionBoundsStep {
        /**
         * Declare a finite legal target range in plant units.
         *
         * <p>The supplied values are the same public units used by {@link Plant#setTarget(double)}
         * after the position plant is built, not native hardware/controller units.</p>
         */
        BoundedPositionMappingStep bounded(double min, double max);

        /**
         * Declare that the plant coordinate has no software target bounds.
         */
        UnboundedPositionMappingStep unbounded();
    }

    /**
     * Mapping question for bounded position coordinates.
     */
    public interface BoundedPositionMappingStep {
        /**
         * Native units and plant units are the same.
         */
        PositionReferenceStep nativeUnits();

        /**
         * Convert plant units to native units using only a scale; reference/offset is answered next.
         */
        PositionReferenceStep scaleToNative(double nativeUnitsPerPlantUnit);

        /**
         * Map the declared plant range endpoints to native endpoints.
         *
         * <p>This static affine map establishes both scale and offset, so no later reference step is
         * required. It is available only after {@link #bounded(double, double)} because unbounded
         * coordinates do not have finite endpoints to map. The arguments are <b>native</b> values
         * evaluated at the bounded plant minimum and bounded plant maximum; they are not plant
         * values. In other words, if the declared plant range is {@code [min, max]}, then this
         * method means "plant {@code min} -> native {@code nativeAtPlantMin}" and
         * "plant {@code max} -> native {@code nativeAtPlantMax}".</p>
         */
        PositionBuildStep rangeMapsToNative(double nativeAtPlantMin, double nativeAtPlantMax);
    }

    /**
     * Mapping question for unbounded position coordinates.
     */
    public interface UnboundedPositionMappingStep {
        /**
         * Native units and plant units are the same.
         */
        PositionReferenceStep nativeUnits();

        /**
         * Convert plant units to native units using only a scale; reference/offset is answered next.
         */
        PositionReferenceStep scaleToNative(double nativeUnitsPerPlantUnit);
    }

    /**
     * Reference question for position coordinates whose static map is not fully established.
     */
    public interface PositionReferenceStep {
        /**
         * The selected native coordinate is already aligned to plant units.
         */
        PositionBuildStep alreadyReferenced();

        /**
         * Declare a static mapping between one plant position and one native position.
         *
         * <p>The first argument is in plant units. The second argument is in native units.</p>
         */
        PositionBuildStep plantPositionMapsToNative(double plantPosition, double nativePosition);

        /**
         * On first update, treat the current native feedback reading as this plant position.
         *
         * <p>The argument is in plant units.</p>
         */
        PositionBuildStep assumeCurrentPositionIs(double plantPosition);

        /**
         * Start invalid until a homing/indexing/manual task establishes a reference.
         */
        PositionBuildStep needsReference(String reason);
    }

    private static final class StartBuilder implements StartStep {
        private final HardwareMap hw;

        private StartBuilder(HardwareMap hw) {
            this.hw = Objects.requireNonNull(hw, "HardwareMap is required");
        }

        @Override
        public MotorSingleStep motor(String name, Direction direction) {
            return new MotorBuilder(hw, name, direction);
        }

        @Override
        public ServoSingleStep servo(String name, Direction direction) {
            return new ServoBuilder(hw, name, direction);
        }

        @Override
        public CrServoSingleStep crServo(String name, Direction direction) {
            return new CrServoBuilder(hw, name, direction);
        }
    }

    private static final class ModifiersStepImpl implements ModifiersStep {
        private Plant plant;

        private ModifiersStepImpl(Plant plant) {
            this.plant = Objects.requireNonNull(plant, "plant");
        }

        @Override
        public ModifiersStep rateLimit(double maxDeltaPerSec) {
            plant = new RateLimitedPlant(plant, maxDeltaPerSec);
            return this;
        }

        @Override
        public ModifiersStep rateLimit(double maxUpPerSec, double maxDownPerSec) {
            plant = new RateLimitedPlant(plant, maxUpPerSec, maxDownPerSec);
            return this;
        }

        @Override
        public Plant build() {
            return plant;
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
    // Feedback specs
    // ---------------------------------------------------------------------------------------------

    /**
     * Builder-side selector for native position feedback.
     *
     * <p>For position plants, this source reports <b>native units</b>. If the source already reports
     * the desired plant units, choose {@code nativeUnits().alreadyReferenced()} in the later mapping
     * and reference stages.</p>
     */
    public abstract static class PositionFeedback {
        private PositionFeedback() {
        }

        /**
         * Use the selected motor's internal encoder position in native ticks.
         */
        public static PositionFeedback internalEncoder() {
            return new InternalPositionFeedback(null, false);
        }

        /**
         * Use one named selected motor's internal encoder position in native ticks.
         */
        public static PositionFeedback internalEncoder(String motorName) {
            return new InternalPositionFeedback(motorName, false);
        }

        /**
         * Use the average selected motor internal encoder position in native group ticks.
         */
        public static PositionFeedback averageInternalEncoders() {
            return new InternalPositionFeedback(null, true);
        }

        /**
         * Use an external encoder device position in native ticks.
         */
        public static PositionFeedback externalEncoder(String name) {
            return new ExternalPositionFeedback(name, Direction.FORWARD);
        }

        /**
         * Use an external encoder device position in native ticks with an explicit logical direction.
         */
        public static PositionFeedback externalEncoder(String name, Direction direction) {
            return new ExternalPositionFeedback(name, direction);
        }

        /**
         * Use a caller-supplied native position source.
         */
        public static PositionFeedback fromSource(ScalarSource source) {
            return new SourcePositionFeedback(source);
        }

        abstract ScalarSource resolve(HardwareMap hw, List<MotorBuilder.Spec> motorSpecs);
    }

    /**
     * Builder-side selector for native velocity feedback.
     *
     * <p>For velocity plants, this source reports <b>native units</b>. If the source already reports
     * the desired plant velocity units, choose {@code nativeUnits()} in the later mapping stage.</p>
     */
    public abstract static class VelocityFeedback {
        private VelocityFeedback() {
        }

        /**
         * Use the selected motor's internal encoder velocity in ticks/sec.
         */
        public static VelocityFeedback internalEncoder() {
            return new InternalVelocityFeedback(null, false);
        }

        /**
         * Use one named selected motor's internal encoder velocity in ticks/sec.
         */
        public static VelocityFeedback internalEncoder(String motorName) {
            return new InternalVelocityFeedback(motorName, false);
        }

        /**
         * Use the average selected motor internal encoder velocity in group ticks/sec.
         */
        public static VelocityFeedback averageInternalEncoders() {
            return new InternalVelocityFeedback(null, true);
        }

        /**
         * Use an external encoder device velocity in ticks/sec.
         */
        public static VelocityFeedback externalEncoder(String name) {
            return new ExternalVelocityFeedback(name, Direction.FORWARD);
        }

        /**
         * Use an external encoder device velocity in ticks/sec with an explicit logical direction.
         */
        public static VelocityFeedback externalEncoder(String name, Direction direction) {
            return new ExternalVelocityFeedback(name, direction);
        }

        /**
         * Use a caller-supplied native velocity source.
         */
        public static VelocityFeedback fromSource(ScalarSource source) {
            return new SourceVelocityFeedback(source);
        }

        abstract ScalarSource resolve(HardwareMap hw, List<MotorBuilder.Spec> motorSpecs);
    }

    private static final class InternalPositionFeedback extends PositionFeedback {
        private final String motorName;
        private final boolean average;

        private InternalPositionFeedback(String motorName, boolean average) {
            this.motorName = motorName;
            this.average = average;
        }

        @Override
        ScalarSource resolve(HardwareMap hw, List<MotorBuilder.Spec> motorSpecs) {
            ensureMotorFeedbackAvailable(motorSpecs, "position");
            if (average) {
                List<ScalarSource> sources = new ArrayList<>();
                for (MotorBuilder.Spec spec : motorSpecs)
                    sources.add(FtcSensors.motorPositionTicks(hw, spec.name));
                return averageSources(sources);
            }
            if (motorName != null) {
                for (MotorBuilder.Spec spec : motorSpecs) {
                    if (spec.name.equals(motorName))
                        return FtcSensors.motorPositionTicks(hw, spec.name);
                }
                throw new IllegalStateException("PositionFeedback.internalEncoder(\"" + motorName + "\") does not match any selected motor");
            }
            if (motorSpecs.size() != 1) {
                throw new IllegalStateException("PositionFeedback.internalEncoder() is ambiguous for a "
                        + motorSpecs.size() + "-motor group. Choose internalEncoder(\"name\"), averageInternalEncoders(), or an external encoder.");
            }
            return FtcSensors.motorPositionTicks(hw, motorSpecs.get(0).name);
        }
    }

    private static final class ExternalPositionFeedback extends PositionFeedback {
        private final String name;
        private final Direction direction;

        private ExternalPositionFeedback(String name, Direction direction) {
            this.name = Objects.requireNonNull(name, "name");
            this.direction = Objects.requireNonNull(direction, "direction");
        }

        @Override
        ScalarSource resolve(HardwareMap hw, List<MotorBuilder.Spec> motorSpecs) {
            return FtcSensors.motorPositionTicks(hw, name, direction);
        }
    }

    private static final class SourcePositionFeedback extends PositionFeedback {
        private final ScalarSource source;

        private SourcePositionFeedback(ScalarSource source) {
            this.source = Objects.requireNonNull(source, "source");
        }

        @Override
        ScalarSource resolve(HardwareMap hw, List<MotorBuilder.Spec> motorSpecs) {
            return source;
        }
    }

    private static final class InternalVelocityFeedback extends VelocityFeedback {
        private final String motorName;
        private final boolean average;

        private InternalVelocityFeedback(String motorName, boolean average) {
            this.motorName = motorName;
            this.average = average;
        }

        @Override
        ScalarSource resolve(HardwareMap hw, List<MotorBuilder.Spec> motorSpecs) {
            ensureMotorFeedbackAvailable(motorSpecs, "velocity");
            if (average) {
                List<ScalarSource> sources = new ArrayList<>();
                for (MotorBuilder.Spec spec : motorSpecs)
                    sources.add(FtcSensors.motorVelocityTicksPerSec(hw, spec.name));
                return averageSources(sources);
            }
            if (motorName != null) {
                for (MotorBuilder.Spec spec : motorSpecs)
                    if (spec.name.equals(motorName))
                        return FtcSensors.motorVelocityTicksPerSec(hw, spec.name);
                throw new IllegalStateException("VelocityFeedback.internalEncoder(\"" + motorName + "\") does not match any selected motor");
            }
            if (motorSpecs.size() != 1) {
                throw new IllegalStateException("VelocityFeedback.internalEncoder() is ambiguous for a "
                        + motorSpecs.size() + "-motor group. Choose internalEncoder(\"name\"), averageInternalEncoders(), or an external encoder.");
            }
            return FtcSensors.motorVelocityTicksPerSec(hw, motorSpecs.get(0).name);
        }
    }

    private static final class ExternalVelocityFeedback extends VelocityFeedback {
        private final String name;
        private final Direction direction;

        private ExternalVelocityFeedback(String name, Direction direction) {
            this.name = Objects.requireNonNull(name, "name");
            this.direction = Objects.requireNonNull(direction, "direction");
        }

        @Override
        ScalarSource resolve(HardwareMap hw, List<MotorBuilder.Spec> motorSpecs) {
            return FtcSensors.motorVelocityTicksPerSec(hw, name, direction);
        }
    }

    private static final class SourceVelocityFeedback extends VelocityFeedback {
        private final ScalarSource source;

        private SourceVelocityFeedback(ScalarSource source) {
            this.source = Objects.requireNonNull(source, "source");
        }

        @Override
        ScalarSource resolve(HardwareMap hw, List<MotorBuilder.Spec> motorSpecs) {
            return source;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Motor builder
    // ---------------------------------------------------------------------------------------------

    private static final class MotorBuilder implements MotorGroupAddedStep {
        private final HardwareMap hw;
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

        private MotorBuilder(HardwareMap hw, String name, Direction direction) {
            this.hw = Objects.requireNonNull(hw, "HardwareMap is required");
            addMotorInternal(name, direction);
        }

        private void addMotorInternal(String name, Direction direction) {
            specs.add(new Spec(name, direction));
            lastIndex = specs.size() - 1;
        }

        @Override
        public MotorGroupAddedStep andMotor(String name, Direction direction) {
            addMotorInternal(name, direction);
            return this;
        }

        @Override
        public MotorGroupAddedStep scale(double scale) {
            specs.get(lastIndex).scale = scale;
            return this;
        }

        @Override
        public MotorGroupAddedStep bias(double bias) {
            specs.get(lastIndex).bias = bias;
            return this;
        }

        @Override
        public ModifiersStep power() {
            if (specs.size() == 1) {
                Spec spec = specs.get(0);
                return new ModifiersStepImpl(Plants.power(FtcHardware.motorPower(hw, spec.name, spec.direction)));
            }
            MultiPlant.Builder b = MultiPlant.builder();
            for (Spec spec : specs)
                b.add(Plants.power(FtcHardware.motorPower(hw, spec.name, spec.direction)), spec.scale, spec.bias);
            return new ModifiersStepImpl(b.build());
        }

        @Override
        public MotorVelocityControlStep velocity() {
            return new MotorVelocityBuilder(this);
        }

        @Override
        public MotorPositionControlStep position() {
            return new MotorPositionBuilder(this);
        }

        private PowerOutput groupedMotorPower() {
            if (specs.size() == 1) {
                Spec spec = specs.get(0);
                return FtcHardware.motorPower(hw, spec.name, spec.direction);
            }
            List<PowerOutput> outs = new ArrayList<>();
            for (Spec spec : specs) outs.add(FtcHardware.motorPower(hw, spec.name, spec.direction));
            return new GroupedPowerOutput(outs);
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

    private static final class MotorVelocityBuilder implements MotorVelocityControlStep, MotorDeviceManagedVelocityStep,
            MotorRegulatedVelocityFeedbackStep, MotorRegulatedVelocityRegulatorStep, VelocityBoundsStep,
            VelocityMappingStep, VelocityBuildStep {
        private final MotorBuilder parent;
        private final DeviceManagedVelocityConfig deviceConfig = new DeviceManagedVelocityConfig();
        private VelocityControlKind controlKind;
        private VelocityFeedback feedback;
        private ScalarRegulator regulator;
        private ScalarRange range;
        private double nativePerPlantUnit = 1.0;
        private double velocityTolerance = 100.0;
        private Double rateUp;
        private Double rateDown;

        private MotorVelocityBuilder(MotorBuilder parent) {
            this.parent = parent;
        }

        @Override
        public VelocityBoundsStep deviceManagedWithDefaults() {
            controlKind = VelocityControlKind.DEVICE_MANAGED;
            return this;
        }

        @Override
        public MotorDeviceManagedVelocityStep deviceManaged() {
            controlKind = VelocityControlKind.DEVICE_MANAGED;
            return this;
        }

        @Override
        public MotorDeviceManagedVelocityStep velocityPidf(double p, double i, double d, double f) {
            deviceConfig.velocityPidf = new double[]{p, i, d, f};
            return this;
        }

        @Override
        public VelocityBoundsStep doneDeviceManaged() {
            return this;
        }

        @Override
        public MotorRegulatedVelocityFeedbackStep regulated() {
            controlKind = VelocityControlKind.REGULATED;
            return this;
        }

        @Override
        public MotorRegulatedVelocityRegulatorStep nativeFeedback(VelocityFeedback feedback) {
            this.feedback = Objects.requireNonNull(feedback, "feedback");
            return this;
        }

        @Override
        public VelocityBoundsStep regulator(ScalarRegulator regulator) {
            this.regulator = Objects.requireNonNull(regulator, "regulator");
            return this;
        }

        @Override
        public VelocityMappingStep bounded(double min, double max) {
            range = ScalarRange.bounded(min, max);
            return this;
        }

        @Override
        public VelocityMappingStep unbounded() {
            range = ScalarRange.unbounded();
            return this;
        }

        @Override
        public VelocityBuildStep nativeUnits() {
            nativePerPlantUnit = 1.0;
            return this;
        }

        @Override
        public VelocityBuildStep scaleToNative(double nativeUnitsPerPlantVelocityUnit) {
            nativePerPlantUnit = requireFiniteNonZero(nativeUnitsPerPlantVelocityUnit, "nativeUnitsPerPlantVelocityUnit");
            return this;
        }

        @Override
        public VelocityBuildStep velocityTolerance(double tolerance) {
            if (tolerance < 0.0 || !Double.isFinite(tolerance))
                throw new IllegalArgumentException("velocityTolerance must be finite and >= 0");
            velocityTolerance = tolerance;
            return this;
        }

        @Override
        public VelocityBuildStep rateLimit(double maxDeltaPerSec) {
            return rateLimit(maxDeltaPerSec, maxDeltaPerSec);
        }

        @Override
        public VelocityBuildStep rateLimit(double maxUpPerSec, double maxDownPerSec) {
            if (maxUpPerSec < 0.0 || maxDownPerSec < 0.0)
                throw new IllegalArgumentException("rate limits must be >= 0");
            rateUp = maxUpPerSec;
            rateDown = maxDownPerSec;
            return this;
        }

        @Override
        public Plant build() {
            if (controlKind == null)
                throw new IllegalStateException("Motor velocity builder requires deviceManagedWithDefaults(), deviceManaged(), or regulated()");
            if (range == null)
                throw new IllegalStateException("Motor velocity builder requires bounded(...) or unbounded()");

            MappedVelocityPlant plant;
            if (controlKind == VelocityControlKind.DEVICE_MANAGED) {
                plant = MappedVelocityPlant.velocityOutput(
                                parent.groupedMotorVelocity(deviceConfig),
                                parent.groupedMotorVelocityMeasurement())
                        .range(range)
                        .nativePerPlantUnit(nativePerPlantUnit)
                        .velocityTolerance(velocityTolerance)
                        .build();
            } else {
                parent.requireDefaultGroupScalingForRegulated("velocity");
                if (feedback == null || regulator == null)
                    throw new IllegalStateException("Regulated motor velocity requires nativeFeedback(...) and regulator(...)");
                plant = MappedVelocityPlant.regulated(
                                parent.groupedMotorPower(),
                                feedback.resolve(parent.hw, parent.specs),
                                regulator)
                        .range(range)
                        .nativePerPlantUnit(nativePerPlantUnit)
                        .velocityTolerance(velocityTolerance)
                        .build();
            }
            if (rateUp != null) return new RateLimitedPlant(plant, rateUp, rateDown);
            return plant;
        }
    }

    private static final class DeviceManagedPositionConfig {
        private double maxPower = 1.0;
        private Double outerPositionP;
        private double[] innerVelocityPidf;
        private Integer devicePositionToleranceTicks;
    }

    private enum PositionControlKind {DEVICE_MANAGED, REGULATED}

    private abstract static class BasePositionBuilder<T extends BasePositionBuilder<T>> implements PositionTopologyStep, PositionBoundsStep,
            BoundedPositionMappingStep, UnboundedPositionMappingStep, PositionReferenceStep, PositionBuildStep {
        protected PositionPlant.Topology topology;
        protected double period = Double.NaN;
        protected ScalarRange range;
        protected double plantMin;
        protected double plantMax;
        protected boolean bounded;
        protected double nativePerPlantUnit = 1.0;
        protected MappedPositionPlant.ReferenceMode referenceMode = MappedPositionPlant.ReferenceMode.STATIC;
        protected double plantReference = 0.0;
        protected double nativeReference = 0.0;
        protected double assumePlantPosition = 0.0;
        protected String referenceReason = "position reference not established";
        protected double positionTolerance = 10.0;
        protected Double rateUp;
        protected Double rateDown;

        @SuppressWarnings("unchecked")
        private T self() {
            return (T) this;
        }

        @Override
        public PositionBoundsStep linear() {
            topology = PositionPlant.Topology.LINEAR;
            period = Double.NaN;
            return this;
        }

        @Override
        public PositionBoundsStep periodic(double period) {
            if (!(period > 0.0) || !Double.isFinite(period))
                throw new IllegalArgumentException("period must be finite and > 0");
            topology = PositionPlant.Topology.PERIODIC;
            this.period = period;
            return this;
        }

        @Override
        public BoundedPositionMappingStep bounded(double min, double max) {
            range = ScalarRange.bounded(min, max);
            plantMin = min;
            plantMax = max;
            bounded = true;
            return this;
        }

        @Override
        public UnboundedPositionMappingStep unbounded() {
            range = ScalarRange.unbounded();
            bounded = false;
            return this;
        }

        @Override
        public PositionReferenceStep nativeUnits() {
            nativePerPlantUnit = 1.0;
            return this;
        }

        @Override
        public PositionReferenceStep scaleToNative(double nativeUnitsPerPlantUnit) {
            nativePerPlantUnit = requireFiniteNonZero(nativeUnitsPerPlantUnit, "nativeUnitsPerPlantUnit");
            return this;
        }

        @Override
        public PositionBuildStep rangeMapsToNative(double nativeAtPlantMin, double nativeAtPlantMax) {
            if (!bounded)
                throw new IllegalStateException("rangeMapsToNative(...) is only valid after bounded(...)");
            if (Math.abs(plantMax - plantMin) < 1e-12)
                throw new IllegalStateException("bounded range must have non-zero width to map endpoints");
            nativePerPlantUnit = (nativeAtPlantMax - nativeAtPlantMin) / (plantMax - plantMin);
            if (!Double.isFinite(nativePerPlantUnit) || Math.abs(nativePerPlantUnit) < 1e-12)
                throw new IllegalArgumentException("native endpoint range must be finite and non-zero");
            referenceMode = MappedPositionPlant.ReferenceMode.STATIC;
            plantReference = plantMin;
            nativeReference = nativeAtPlantMin;
            return this;
        }

        @Override
        public PositionBuildStep alreadyReferenced() {
            referenceMode = MappedPositionPlant.ReferenceMode.STATIC;
            plantReference = 0.0;
            nativeReference = 0.0;
            return this;
        }

        @Override
        public PositionBuildStep plantPositionMapsToNative(double plantPosition, double nativePosition) {
            referenceMode = MappedPositionPlant.ReferenceMode.STATIC;
            plantReference = plantPosition;
            nativeReference = nativePosition;
            return this;
        }

        @Override
        public PositionBuildStep assumeCurrentPositionIs(double plantPosition) {
            referenceMode = MappedPositionPlant.ReferenceMode.ASSUME_CURRENT;
            assumePlantPosition = plantPosition;
            return this;
        }

        @Override
        public PositionBuildStep needsReference(String reason) {
            referenceMode = MappedPositionPlant.ReferenceMode.NEEDS_REFERENCE;
            referenceReason = reason;
            return this;
        }

        @Override
        public PositionBuildStep positionTolerance(double tolerance) {
            if (tolerance < 0.0 || !Double.isFinite(tolerance))
                throw new IllegalArgumentException("positionTolerance must be finite and >= 0");
            positionTolerance = tolerance;
            return this;
        }

        @Override
        public PositionBuildStep rateLimit(double maxDeltaPerSec) {
            return rateLimit(maxDeltaPerSec, maxDeltaPerSec);
        }

        @Override
        public PositionBuildStep rateLimit(double maxUpPerSec, double maxDownPerSec) {
            if (maxUpPerSec < 0.0 || maxDownPerSec < 0.0)
                throw new IllegalArgumentException("rate limits must be >= 0");
            rateUp = maxUpPerSec;
            rateDown = maxDownPerSec;
            return this;
        }

        protected MappedPositionPlant.Builder applyCommon(MappedPositionPlant.Builder b) {
            if (topology == null)
                throw new IllegalStateException("Position builder requires linear() or periodic(period)");
            if (range == null)
                throw new IllegalStateException("Position builder requires bounded(...) or unbounded()");
            b.topology(topology, period)
                    .range(range)
                    .nativePerPlantUnit(nativePerPlantUnit)
                    .positionTolerance(positionTolerance);
            if (referenceMode == MappedPositionPlant.ReferenceMode.STATIC)
                b.plantPositionMapsToNative(plantReference, nativeReference);
            else if (referenceMode == MappedPositionPlant.ReferenceMode.ASSUME_CURRENT)
                b.assumeCurrentPositionIs(assumePlantPosition);
            else b.needsReference(referenceReason);
            return b;
        }

        protected PositionPlant maybeRateLimit(PositionPlant plant) {
            if (rateUp != null) return new RateLimitedPositionPlant(plant, rateUp, rateDown);
            return plant;
        }
    }

    private static final class MotorPositionBuilder extends BasePositionBuilder<MotorPositionBuilder> implements MotorPositionControlStep,
            MotorDeviceManagedPositionStep, MotorRegulatedPositionFeedbackStep, MotorRegulatedPositionRegulatorStep {
        private final MotorBuilder parent;
        private final DeviceManagedPositionConfig deviceConfig = new DeviceManagedPositionConfig();
        private PositionControlKind controlKind;
        private PositionFeedback feedback;
        private ScalarRegulator regulator;

        private MotorPositionBuilder(MotorBuilder parent) {
            this.parent = parent;
        }

        @Override
        public PositionTopologyStep deviceManagedWithDefaults() {
            controlKind = PositionControlKind.DEVICE_MANAGED;
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep deviceManaged() {
            controlKind = PositionControlKind.DEVICE_MANAGED;
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep maxPower(double maxPower) {
            if (maxPower < 0.0) throw new IllegalArgumentException("maxPower must be >= 0");
            deviceConfig.maxPower = maxPower;
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep outerPositionP(double outerPositionP) {
            deviceConfig.outerPositionP = outerPositionP;
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep innerVelocityPidf(double p, double i, double d, double f) {
            deviceConfig.innerVelocityPidf = new double[]{p, i, d, f};
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep devicePositionToleranceTicks(int ticks) {
            if (ticks < 0)
                throw new IllegalArgumentException("devicePositionToleranceTicks must be >= 0");
            deviceConfig.devicePositionToleranceTicks = ticks;
            return this;
        }

        @Override
        public PositionTopologyStep doneDeviceManaged() {
            return this;
        }

        @Override
        public MotorRegulatedPositionFeedbackStep regulated() {
            controlKind = PositionControlKind.REGULATED;
            return this;
        }

        @Override
        public MotorRegulatedPositionRegulatorStep nativeFeedback(PositionFeedback feedback) {
            this.feedback = Objects.requireNonNull(feedback, "feedback");
            return this;
        }

        @Override
        public PositionTopologyStep regulator(ScalarRegulator regulator) {
            this.regulator = Objects.requireNonNull(regulator, "regulator");
            return this;
        }

        @Override
        public PositionPlant build() {
            if (controlKind == null)
                throw new IllegalStateException("Motor position builder requires deviceManagedWithDefaults(), deviceManaged(), or regulated()");
            MappedPositionPlant plant;
            if (controlKind == PositionControlKind.DEVICE_MANAGED) {
                PositionOutput out = parent.groupedMotorPosition(deviceConfig);
                ScalarSource nativeMeasurement = parent.groupedMotorPositionMeasurement();
                plant = applyCommon(MappedPositionPlant.positionOutput(out, nativeMeasurement)
                        .searchPowerOutput(parent.groupedMotorPower())).build();
            } else {
                parent.requireDefaultGroupScalingForRegulated("position");
                if (feedback == null || regulator == null)
                    throw new IllegalStateException("Regulated motor position requires nativeFeedback(...) and regulator(...)");
                PowerOutput power = parent.groupedMotorPower();
                plant = applyCommon(MappedPositionPlant.regulated(power, feedback.resolve(parent.hw, parent.specs), regulator)
                        .searchPowerOutput(power)).build();
            }
            return maybeRateLimit(plant);
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Servo builder
    // ---------------------------------------------------------------------------------------------

    private static final class ServoBuilder implements ServoGroupAddedStep {
        private final HardwareMap hw;
        private final List<MotorBuilder.Spec> specs = new ArrayList<>();
        private int lastIndex;

        private ServoBuilder(HardwareMap hw, String name, Direction direction) {
            this.hw = Objects.requireNonNull(hw, "HardwareMap is required");
            addServoInternal(name, direction);
        }

        private void addServoInternal(String name, Direction direction) {
            specs.add(new MotorBuilder.Spec(name, direction));
            lastIndex = specs.size() - 1;
        }

        @Override
        public ServoGroupAddedStep andServo(String name, Direction direction) {
            addServoInternal(name, direction);
            return this;
        }

        @Override
        public ServoGroupAddedStep scale(double scale) {
            specs.get(lastIndex).scale = scale;
            return this;
        }

        @Override
        public ServoGroupAddedStep bias(double bias) {
            specs.get(lastIndex).bias = bias;
            return this;
        }

        @Override
        public ServoPositionTopologyStep position() {
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

    private static final class ServoPositionBuilder implements ServoPositionTopologyStep, ServoPositionBoundsStep,
            ServoBoundedPositionMappingStep, ServoPositionBuildStep {
        private final ServoBuilder parent;
        private ScalarRange range;
        private double plantMin;
        private double plantMax;
        private double nativePerPlantUnit = 1.0;
        private double plantReference = 0.0;
        private double nativeReference = 0.0;
        private Double rateUp;
        private Double rateDown;

        private ServoPositionBuilder(ServoBuilder parent) {
            this.parent = parent;
        }

        @Override
        public ServoPositionBoundsStep linear() {
            return this;
        }

        @Override
        public ServoBoundedPositionMappingStep bounded(double min, double max) {
            range = ScalarRange.bounded(min, max);
            plantMin = min;
            plantMax = max;
            return this;
        }

        @Override
        public ServoPositionBuildStep nativeUnits() {
            nativePerPlantUnit = 1.0;
            plantReference = 0.0;
            nativeReference = 0.0;
            return this;
        }

        @Override
        public ServoPositionBuildStep rangeMapsToNative(double nativeAtPlantMin, double nativeAtPlantMax) {
            if (range == null)
                throw new IllegalStateException("rangeMapsToNative(...) requires bounded(...)");
            if (Math.abs(plantMax - plantMin) < 1e-12)
                throw new IllegalStateException("bounded range must have non-zero width");
            nativePerPlantUnit = (nativeAtPlantMax - nativeAtPlantMin) / (plantMax - plantMin);
            if (!Double.isFinite(nativePerPlantUnit) || Math.abs(nativePerPlantUnit) < 1e-12)
                throw new IllegalArgumentException("native endpoint range must be finite and non-zero");
            plantReference = plantMin;
            nativeReference = nativeAtPlantMin;
            return this;
        }

        @Override
        public ServoPositionBuildStep rateLimit(double maxDeltaPerSec) {
            return rateLimit(maxDeltaPerSec, maxDeltaPerSec);
        }

        @Override
        public ServoPositionBuildStep rateLimit(double maxUpPerSec, double maxDownPerSec) {
            if (maxUpPerSec < 0.0 || maxDownPerSec < 0.0)
                throw new IllegalArgumentException("rate limits must be >= 0");
            rateUp = maxUpPerSec;
            rateDown = maxDownPerSec;
            return this;
        }

        @Override
        public PositionPlant build() {
            if (range == null)
                throw new IllegalStateException("Servo position builder requires bounded(...)");
            MappedPositionPlant plant = MappedPositionPlant.commanded(parent.groupedServoPosition())
                    .topology(PositionPlant.Topology.LINEAR, Double.NaN)
                    .range(range)
                    .nativePerPlantUnit(nativePerPlantUnit)
                    .plantPositionMapsToNative(plantReference, nativeReference)
                    .build();
            if (rateUp != null) return new RateLimitedPositionPlant(plant, rateUp, rateDown);
            return plant;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // CR servo builder
    // ---------------------------------------------------------------------------------------------

    private static final class CrServoBuilder implements CrServoGroupAddedStep {
        private final HardwareMap hw;
        private final List<MotorBuilder.Spec> specs = new ArrayList<>();
        private int lastIndex;

        private CrServoBuilder(HardwareMap hw, String name, Direction direction) {
            this.hw = Objects.requireNonNull(hw, "HardwareMap is required");
            addCrServoInternal(name, direction);
        }

        private void addCrServoInternal(String name, Direction direction) {
            specs.add(new MotorBuilder.Spec(name, direction));
            lastIndex = specs.size() - 1;
        }

        @Override
        public CrServoGroupAddedStep andCrServo(String name, Direction direction) {
            addCrServoInternal(name, direction);
            return this;
        }

        @Override
        public CrServoGroupAddedStep scale(double scale) {
            specs.get(lastIndex).scale = scale;
            return this;
        }

        @Override
        public CrServoGroupAddedStep bias(double bias) {
            specs.get(lastIndex).bias = bias;
            return this;
        }

        @Override
        public ModifiersStep power() {
            if (specs.size() == 1) {
                MotorBuilder.Spec spec = specs.get(0);
                return new ModifiersStepImpl(Plants.power(FtcHardware.crServoPower(hw, spec.name, spec.direction)));
            }
            MultiPlant.Builder b = MultiPlant.builder();
            for (MotorBuilder.Spec spec : specs)
                b.add(Plants.power(FtcHardware.crServoPower(hw, spec.name, spec.direction)), spec.scale, spec.bias);
            return new ModifiersStepImpl(b.build());
        }

        @Override
        public CrServoPositionControlStep position() {
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

        private void requireDefaultGroupScalingForRegulated() {
            if (specs.size() <= 1) return;
            for (MotorBuilder.Spec spec : specs)
                if (Math.abs(spec.scale - 1.0) > 1e-9 || Math.abs(spec.bias) > 1e-9)
                    throw new IllegalStateException("Regulated CR-servo position control requires default group scaling/bias through FtcActuators");
        }
    }

    private static final class CrServoPositionBuilder extends BasePositionBuilder<CrServoPositionBuilder> implements CrServoPositionControlStep,
            CrServoRegulatedPositionFeedbackStep, CrServoRegulatedPositionRegulatorStep {
        private final CrServoBuilder parent;
        private PositionFeedback feedback;
        private ScalarRegulator regulator;

        private CrServoPositionBuilder(CrServoBuilder parent) {
            this.parent = parent;
        }

        @Override
        public CrServoRegulatedPositionFeedbackStep regulated() {
            return this;
        }

        @Override
        public CrServoRegulatedPositionRegulatorStep nativeFeedback(PositionFeedback feedback) {
            this.feedback = Objects.requireNonNull(feedback, "feedback");
            return this;
        }

        @Override
        public PositionTopologyStep regulator(ScalarRegulator regulator) {
            this.regulator = Objects.requireNonNull(regulator, "regulator");
            return this;
        }

        @Override
        public PositionPlant build() {
            parent.requireDefaultGroupScalingForRegulated();
            if (feedback == null || regulator == null)
                throw new IllegalStateException("Regulated CR-servo position requires nativeFeedback(...) and regulator(...)");
            PowerOutput power = parent.groupedCrServoPower();
            MappedPositionPlant plant = applyCommon(MappedPositionPlant.regulated(power, feedback.resolve(parent.hw, null), regulator)
                    .searchPowerOutput(power)).build();
            return maybeRateLimit(plant);
        }
    }

    private static final class GroupedPowerOutput implements PowerOutput {
        private final List<PowerOutput> outputs;
        private double last;

        private GroupedPowerOutput(List<PowerOutput> outputs) {
            this.outputs = new ArrayList<>(Objects.requireNonNull(outputs, "outputs"));
            if (this.outputs.isEmpty())
                throw new IllegalArgumentException("outputs must not be empty");
        }

        @Override
        public void setPower(double power) {
            last = power;
            for (PowerOutput output : outputs) output.setPower(power);
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
            throw new IllegalStateException("Internal " + domain + " encoder feedback requires a motor builder. Use an external encoder or fromSource(...) instead.");
        }
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
