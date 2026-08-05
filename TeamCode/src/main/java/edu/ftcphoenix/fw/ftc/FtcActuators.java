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
 * <h2>Grouped child mappings</h2>
 *
 * <p>A retained child transform uses {@code childNative = scale * sharedNative + bias}. Finite
 * answers are checked before assignment, and every completed recipe is rechecked at target
 * selection and build before hardware resolution. Standard Servo commands must remain in
 * {@code [0.0, 1.0]}; direct motor/CR-servo power must preserve zero and remain in
 * {@code [-1.0, +1.0]}; device-managed velocity must preserve zero; and device-managed motor
 * position must round into FTC's signed 32-bit tick domain. Grouped outputs compute and validate
 * every child command before changing their group cache or writing any child. Inverse-mapped
 * grouped feedback returns {@link Double#NaN} if any child conversion or aggregate is non-finite.
 * A motor-position child transform describes only the native position coordinate; temporary
 * calibration-search power is a separate normalized command and fans out identically through each
 * configured motor direction.</p>
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
         * target range is always {@code [-1.0, +1.0]}. Before retaining this branch answer, every
         * child must have exact zero bias and map both full-range endpoints inside the same raw
         * power domain.
         *
         * @throws IllegalStateException if a retained child mapping violates neutral zero or the
         * complete normalized-power domain; no hardware is resolved and the group can be corrected
         * and retried
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
     * Group-aware motor builder step that exposes affine configuration for the most recently added
     * child's normal native coordinate: {@code childNative = scale * sharedNative + bias}.
     *
     * <p>Both answers must be finite and are retained only after validation. The completed branch
     * then validates the full composed child command: direct power and device-managed velocity
     * require zero bias, regulated control requires exact identity, and device-managed position
     * permits finite bias for encoder/linkage alignment subject to the FTC tick domain. A
     * device-managed position Plant's temporary raw-power calibration search does not apply this
     * position transform.</p>
     */
    public interface MotorGroupAddedStep extends MotorSingleStep {
        /**
         * Set the dimensionless scale applied to the most recently added motor's shared native
         * position or velocity coordinate, or to its direct-power command when that branch is
         * selected. It does not scale temporary motor-position calibration-search power.
         *
         * @throws IllegalArgumentException if {@code scale} is not finite; the prior value remains
         * unchanged
         */
        MotorGroupAddedStep scale(double scale);

        /**
         * Set the additive native-unit bias applied to the most recently added motor's normal
         * native position coordinate.
         * A finite nonzero bias is meaningful only for device-managed position alignment; later
         * power, velocity, and regulated branch answers reject it before hardware resolution.
         *
         * @throws IllegalArgumentException if {@code bias} is not finite; the prior value remains
         * unchanged
         */
        MotorGroupAddedStep bias(double bias);

    }

    /**
     * First motor-velocity question: who manages the velocity loop?
     */
    public interface MotorVelocityControlStep {
        /**
         * Use FTC device-managed velocity control with Phoenix defaults and continue to velocity
         * target bounds. The answer first validates that every retained child has finite nonzero
         * scale and exact zero bias, so a rejected answer can be corrected and retried without
         * hardware resolution.
         *
         * @throws IllegalStateException if a retained child mapping is incompatible
         */
        Plants.VelocityBoundsStep deviceManagedWithDefaults();

        /**
         * Enter the FTC device-managed velocity tuning branch before continuing to target bounds.
         * The answer applies the same retryable child-mapping preflight as
         * {@link #deviceManagedWithDefaults()}.
         *
         * @throws IllegalStateException if a retained child mapping is incompatible
         */
        MotorDeviceManagedVelocityStep deviceManaged();

        /**
         * Use a Phoenix-regulated velocity loop that drives motor power from native velocity feedback.
         * Grouped regulated control requires exact child scale {@code 1.0} and bias {@code 0.0};
         * this answer validates that identity before it is retained.
         *
         * @throws IllegalStateException if a grouped child mapping is not exact identity
         */
        MotorRegulatedVelocityFeedbackStep regulated();
    }

    /**
     * Required one-answer tuning branch for FTC device-managed motor velocity control.
     */
    public interface MotorDeviceManagedVelocityStep {
        /**
         * Override the FTC device-managed velocity PIDF coefficients and continue directly to
         * target bounds.
         *
         * <p>Every coefficient must be finite and inside the inclusive, symmetric pinned FTC SDK
         * 11.1 REV public conversion domain
         * {@code [-Integer.MAX_VALUE / 65536.0, +Integer.MAX_VALUE / 65536.0]}. Negative
         * coefficients remain representable; this structural domain is not a tuning
         * recommendation.</p>
         *
         * @throws IllegalArgumentException if any coefficient is non-finite or outside the
         * supported controller conversion domain; all four prior values remain unchanged
         * @throws IllegalStateException if this recipe did not enter through
         * {@link MotorVelocityControlStep#deviceManaged()}, or if this answer was already supplied
         */
        Plants.VelocityBoundsStep velocityPidf(double p, double i, double d, double f);
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
     * and is rejected before acquisition rather than clamped. For a device-managed motor group,
     * the shared search command is submitted identically to every child through its configured
     * {@link Direction}; native position scale/bias is not a raw-power policy. The mechanism owner
     * still chooses and physically validates a safe magnitude and direction.</p>
     */
    public interface MotorPositionControlStep {
        /**
         * Use FTC RUN_TO_POSITION with Phoenix defaults and continue to position periodicity.
         * Finite nonzero child scales and finite position-alignment biases are validated before this
         * answer is retained.
         *
         * @throws IllegalStateException if a retained child mapping is incompatible
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep> deviceManagedWithDefaults();

        /**
         * Enter the FTC RUN_TO_POSITION tuning branch before continuing to position periodicity.
         * This applies the same retryable child-mapping preflight as
         * {@link #deviceManagedWithDefaults()}.
         *
         * @throws IllegalStateException if a retained child mapping is incompatible
         */
        MotorDeviceManagedPositionStep deviceManaged();

        /**
         * Use a Phoenix-regulated loop that drives motor power from native position feedback.
         * Grouped regulated control requires exact child scale {@code 1.0} and bias {@code 0.0}.
         *
         * @throws IllegalStateException if a grouped child mapping is not exact identity
         */
        MotorRegulatedPositionFeedbackStep regulated();
    }

    /**
     * Nonempty tuning branch for FTC device-managed motor position control. Each individual
     * setting is optional, but at least one must be answered before the branch is closed.
     */
    public interface MotorDeviceManagedPositionStep {
        /**
         * Set the finite normalized maximum-power magnitude reapplied for each RUN_TO_POSITION
         * target. The inclusive domain is {@code [0.0, 1.0]}; configuration is rejected rather than
         * clamped.
         *
         * @throws IllegalArgumentException if {@code maxPower} is non-finite or outside
         * {@code [0.0, 1.0]}
         * @throws IllegalStateException if this setting was already answered, the tuning section is
         * closed, or the recipe did not enter through {@link MotorPositionControlStep#deviceManaged()}
         */
        MotorDeviceManagedPositionStep maxPower(double maxPower);

        /**
         * Set FTC's outer position-loop proportional coefficient in the inclusive, symmetric
         * pinned controller public conversion domain
         * {@code [-Integer.MAX_VALUE / 65536.0, +Integer.MAX_VALUE / 65536.0]}.
         *
         * @throws IllegalArgumentException if {@code outerPositionP} is non-finite or outside the
         * supported controller conversion domain
         * @throws IllegalStateException if this setting was already answered, the tuning section is
         * closed, or the recipe did not enter through {@link MotorPositionControlStep#deviceManaged()}
         */
        MotorDeviceManagedPositionStep outerPositionP(double outerPositionP);

        /**
         * Set FTC's inner velocity-loop PIDF coefficients used underneath RUN_TO_POSITION. Every
         * coefficient must be finite and inside the inclusive, symmetric pinned controller public
         * conversion domain
         * {@code [-Integer.MAX_VALUE / 65536.0, +Integer.MAX_VALUE / 65536.0]}.
         *
         * @throws IllegalArgumentException if any coefficient is non-finite or outside the
         * supported controller conversion domain; all four prior values remain unchanged
         * @throws IllegalStateException if this setting was already answered, the tuning section is
         * closed, or the recipe did not enter through {@link MotorPositionControlStep#deviceManaged()}
         */
        MotorDeviceManagedPositionStep innerVelocityPidf(double p, double i, double d, double f);

        /**
         * Set FTC's native device target-position tolerance in the inclusive {@code [0, 65535]}
         * encoder-tick domain.
         *
         * @throws IllegalArgumentException if {@code ticks} is outside {@code [0, 65535]}
         * @throws IllegalStateException if this setting was already answered, the tuning section is
         * closed, or the recipe did not enter through {@link MotorPositionControlStep#deviceManaged()}
         */
        MotorDeviceManagedPositionStep devicePositionToleranceTicks(int ticks);

        /**
         * Close a nonempty device-managed tuning section and continue to position periodicity.
         * Use {@link MotorPositionControlStep#deviceManagedWithDefaults()} instead of opening and
         * immediately closing an empty section.
         *
         * @throws IllegalStateException if no tuning setting was accepted, this section was already
         * closed, or the recipe did not enter through {@link MotorPositionControlStep#deviceManaged()}
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
     * Group-aware standard-servo builder step that exposes an affine mapping for the most recently
     * added child. Both values must be finite. At {@code nativeUnits()} or
     * {@code rangeMapsToNative(...)}, the complete composed images of both Plant endpoints must lie
     * exactly inside the raw Servo {@code [0.0, 1.0]} domain before the mapping answer is retained.
     */
    public interface ServoGroupAddedStep extends ServoSingleStep {
        /**
         * Set the dimensionless scale applied to the most recently added servo's shared native
         * command.
         *
         * @throws IllegalArgumentException if {@code scale} is not finite; the prior value remains
         * unchanged
         */
        ServoGroupAddedStep scale(double scale);

        /**
         * Set the additive raw-servo-unit bias applied to the most recently added servo.
         *
         * @throws IllegalArgumentException if {@code bias} is not finite; the prior value remains
         * unchanged
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
         *
         * @throws IllegalStateException if either bounded Plant endpoint, after every child affine
         * mapping, is non-finite or outside raw Servo {@code [0.0, 1.0]}; the mapping answer remains
         * available for retry
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
         *
         * @throws IllegalArgumentException if either endpoint or the resulting shared affine scale
         * is not finite, or the scale is zero
         * @throws IllegalStateException if either exact composed child endpoint is outside raw Servo
         * {@code [0.0, 1.0]}; a rejected answer does not consume the mapping step
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
         * target range is always {@code [-1.0, +1.0]}. Every child scale must map both full-range
         * endpoints inside that domain before this answer is retained.
         *
         * @throws IllegalStateException if a retained child scale escapes the raw power domain; no
         * hardware is resolved and the group can be corrected and retried
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
     * Group-aware CR-servo builder step that exposes zero-preserving per-child scaling.
     *
     * <p>Continuous-rotation-servo power has the same neutral-zero contract as every
     * {@link PowerOutput}: a shared command of {@code 0.0} must command every child to
     * {@code 0.0}. An additive child bias cannot satisfy that contract, so this step deliberately
     * offers only scale.</p>
     */
    public interface CrServoGroupAddedStep extends CrServoSingleStep {
        /**
         * Set the dimensionless scale applied to the most recently added CR servo's shared
         * normalized-power command.
         * The value must be finite. The complete direct-power recipe must also map the full
         * {@code [-1.0, +1.0]} command range into that same raw child range.
         *
         * @throws IllegalArgumentException if {@code scale} is not finite; the prior value remains
         * unchanged
         */
        CrServoGroupAddedStep scale(double scale);
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
         * A group requires exact child scale {@code 1.0}; direct-power scaling is not meaningful
         * once one shared regulator owns the group output.
         *
         * @throws IllegalStateException if a grouped child scale is not exactly {@code 1.0}
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
        private final Runnable recipeValidator;

        private PowerTargetBuilder(RecipeLifecycle lifecycle,
                                   Runnable recipeValidator,
                                   Supplier<PowerOutput> output) {
            super(lifecycle, "power Plant");
            this.recipeValidator = Objects.requireNonNull(recipeValidator, "recipeValidator");
            this.output = Objects.requireNonNull(output, "output");
        }

        @Override
        protected void validateRecipe() {
            recipeValidator.run();
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

    private enum VelocityControlKind {
        DEVICE_MANAGED_DEFAULTS,
        DEVICE_MANAGED_TUNED,
        REGULATED
    }

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
            specs.get(lastIndex).scale = requireFiniteChildMappingValue(
                    scale, "motor scale(...)", "scale");
            return this;
        }

        @Override
        public MotorGroupAddedStep bias(double bias) {
            lifecycle.requireMutable("motor bias(...)");
            specs.get(lastIndex).bias = requireFiniteChildMappingValue(
                    bias, "motor bias(...)", "bias");
            return this;
        }

        @Override
        public Plants.TargetStep<Plant> power() {
            lifecycle.requireMutable("power()");
            validateDirectPowerRecipe();
            return new PowerTargetBuilder(
                    lifecycle,
                    this::validateDirectPowerRecipe,
                    this::groupedMotorPowerWithMappings);
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

        /**
         * Build the exact shared-power fan-out used by regulation and position calibration search.
         * Native position/velocity child mappings do not belong to this normalized-power channel.
         */
        private PowerOutput groupedMotorPower() {
            if (specs.size() == 1) {
                Spec spec = specs.get(0);
                return FtcHardware.motorPower(hw, spec.name, spec.direction);
            }
            return new IdentityGroupedPowerOutput(
                    groupedFtcMotorPowers(),
                    childNames(specs),
                    "motor power");
        }

        /** Build the child-mapped normalized-power fan-out used only by direct-power Plants. */
        private PowerOutput groupedMotorPowerWithMappings() {
            if (specs.size() == 1) {
                Spec spec = specs.get(0);
                return FtcHardware.motorPower(hw, spec.name, spec.direction);
            }
            return groupedMotorPowerOutput(groupedFtcMotorPowers());
        }

        private PowerOutput groupedMotorPowerOutput(List<PowerOutput> outputs) {
            return new GroupedPowerOutput(
                    outputs,
                    childScales(specs),
                    childBiases(specs),
                    childNames(specs),
                    "motor power");
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
            return new GroupedVelocityOutput(
                    outs, scales, biases, childNames(specs), "motor velocity");
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
            return new GroupedPositionOutput(
                    outs,
                    scales,
                    biases,
                    childNames(specs),
                    PositionChildDomain.MOTOR_TICKS,
                    "motor position");
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
                requireFiniteChildSpec(spec, "regulated motor " + mode);
                if (spec.scale != 1.0 || spec.bias != 0.0) {
                    throw new IllegalStateException("Regulated motor " + mode
                            + " control requires exact child scale 1.0 and bias 0.0 for motor '"
                            + spec.name + "'; use separate Plants for non-trivial per-motor policy");
                }
            }
        }

        private void validateDirectPowerRecipe() {
            for (int i = 0; i < specs.size(); i++) {
                Spec spec = specs.get(i);
                requireFiniteChildSpec(spec, "motor direct power");
                if (spec.bias != 0.0) {
                    throw new IllegalStateException("Motor direct power child " + (i + 1)
                            + " ('" + spec.name + "') must preserve neutral zero with bias 0.0, got "
                            + spec.bias);
                }
                validateMappedPowerEndpoint(
                        "motor direct power", i, spec, -1.0);
                validateMappedPowerEndpoint(
                        "motor direct power", i, spec, 1.0);
            }
        }

        private void validateDeviceManagedVelocityRecipe(ScalarRange targetRange,
                                                         double nativePerPlantUnit) {
            validateDeviceManagedVelocityChildMappings();
            if (!targetRange.isUnbounded()) {
                validateDeviceManagedVelocityEndpoint(
                        targetRange.minValue, nativePerPlantUnit, "minimum");
                validateDeviceManagedVelocityEndpoint(
                        targetRange.maxValue, nativePerPlantUnit, "maximum");
            }
        }

        private void validateDeviceManagedVelocityChildMappings() {
            for (int i = 0; i < specs.size(); i++) {
                Spec spec = specs.get(i);
                requireFiniteChildSpec(spec, "device-managed motor velocity");
                if (spec.scale == 0.0) {
                    throw new IllegalStateException("Device-managed motor velocity child "
                            + (i + 1) + " ('" + spec.name + "') requires non-zero scale");
                }
                if (spec.bias != 0.0) {
                    throw new IllegalStateException("Device-managed motor velocity child "
                            + (i + 1) + " ('" + spec.name
                            + "') must preserve zero velocity with bias 0.0, got " + spec.bias);
                }
            }
        }

        private void validateDeviceManagedVelocityEndpoint(double plantEndpoint,
                                                           double nativePerPlantUnit,
                                                           String endpointName) {
            double sharedNative = requireFiniteVelocityEndpoint(
                    "Device-managed motor velocity " + endpointName + " Plant endpoint",
                    plantEndpoint,
                    nativePerPlantUnit);
            for (int i = 0; i < specs.size(); i++) {
                mappedChildCommand(
                        "device-managed motor velocity " + endpointName + " endpoint",
                        i,
                        specs.get(i),
                        sharedNative);
            }
        }

        private void validateDeviceManagedPositionRecipe(double[] boundedStaticNativeEndpoints) {
            for (MotorBuilder.Spec spec : specs) {
                requireFiniteChildSpec(spec, "device-managed motor position");
                if (spec.scale == 0.0) {
                    throw new IllegalStateException("Device-managed motor position child '"
                            + spec.name + "' requires non-zero scale");
                }
            }
            if (boundedStaticNativeEndpoints == null) {
                return;
            }
            for (int endpoint = 0; endpoint < boundedStaticNativeEndpoints.length; endpoint++) {
                double sharedNative = boundedStaticNativeEndpoints[endpoint];
                String endpointName = endpoint == 0 ? "minimum" : "maximum";
                for (int i = 0; i < specs.size(); i++) {
                    Spec spec = specs.get(i);
                    double child = mappedChildCommand(
                            "device-managed motor position " + endpointName + " endpoint",
                            i,
                            spec,
                            sharedNative);
                    checkedMotorPositionTicks(
                            child,
                            "device-managed motor position " + endpointName + " endpoint child "
                                    + (i + 1) + " ('" + spec.name + "')");
                }
            }
        }

        private void ensureFeedbackScalesNonZero(String mode) {
            for (Spec spec : specs) {
                requireFiniteChildSpec(spec, mode);
                if (spec.scale == 0.0) {
                    throw new IllegalStateException(mode + " requires non-zero per-motor scale");
                }
            }
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
        private static final String VELOCITY_PIDF_OPERATION =
                "FtcActuators.velocityPidf(...)";

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
            answerControl(
                    VelocityControlKind.DEVICE_MANAGED_DEFAULTS,
                    "deviceManagedWithDefaults()");
            return this;
        }

        @Override
        public MotorDeviceManagedVelocityStep deviceManaged() {
            answerControl(VelocityControlKind.DEVICE_MANAGED_TUNED, "deviceManaged()");
            return this;
        }

        @Override
        public Plants.VelocityBoundsStep velocityPidf(double p, double i, double d, double f) {
            requireMutable("velocityPidf(...)");
            requireControl(VelocityControlKind.DEVICE_MANAGED_TUNED, "velocityPidf(...)");
            if (deviceConfig.velocityPidf != null) {
                throw new IllegalStateException(
                        "velocityPidf(...) has already been answered for this motor velocity Plant");
            }
            deviceConfig.velocityPidf =
                    FtcControllerConfigurationValidation.requireControllerPidf(
                            p, i, d, f, VELOCITY_PIDF_OPERATION);
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
            validateControlConfiguration();
            if (!rangeAnswered) {
                throw new IllegalStateException("Motor velocity builder requires bounded(...) or unbounded()");
            }
            if (!mappingAnswered) {
                throw new IllegalStateException("Motor velocity builder requires nativeUnits() or scaleToNative(...)");
            }
            if (!velocityToleranceAnswered) {
                throw new IllegalStateException("Motor velocity builder requires velocityTolerance(...) in plant velocity units");
            }
            if (isDeviceManaged()) {
                parent.validateDeviceManagedVelocityRecipe(range, nativePerPlantUnit);
            } else {
                validateRegulatedVelocityRecipe(range, nativePerPlantUnit);
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
            if (isDeviceManaged()) {
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
            if (answer != VelocityControlKind.REGULATED) {
                parent.validateDeviceManagedVelocityChildMappings();
            } else {
                parent.requireDefaultGroupScalingForRegulated("velocity");
            }
            controlKind = answer;
        }

        private void requireControl(VelocityControlKind required, String operation) {
            if (controlKind != required) {
                throw new IllegalStateException(operation + " requires "
                        + (required == VelocityControlKind.DEVICE_MANAGED_TUNED
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
            double checkedScale = requireFiniteNonZero(
                    scale, "nativeUnitsPerPlantVelocityUnit");
            if (isDeviceManaged()) {
                parent.validateDeviceManagedVelocityRecipe(range, checkedScale);
            } else {
                validateRegulatedVelocityRecipe(range, checkedScale);
            }
            nativePerPlantUnit = checkedScale;
            mappingAnswered = true;
        }

        private boolean isDeviceManaged() {
            return controlKind == VelocityControlKind.DEVICE_MANAGED_DEFAULTS
                    || controlKind == VelocityControlKind.DEVICE_MANAGED_TUNED;
        }

        private void validateControlConfiguration() {
            if (controlKind == null) {
                throw new IllegalStateException("Motor velocity builder requires "
                        + "deviceManagedWithDefaults(), deviceManaged(), or regulated()");
            }
            if (controlKind == VelocityControlKind.DEVICE_MANAGED_DEFAULTS) {
                if (deviceConfig.velocityPidf != null) {
                    throw new IllegalStateException("deviceManagedWithDefaults() cannot retain "
                            + "device-managed velocity tuning");
                }
                return;
            }
            if (controlKind == VelocityControlKind.DEVICE_MANAGED_TUNED) {
                if (deviceConfig.velocityPidf == null) {
                    throw new IllegalStateException("deviceManaged() requires velocityPidf(...) "
                            + "before motor velocity bounds");
                }
                double[] c = deviceConfig.velocityPidf;
                FtcControllerConfigurationValidation.requireControllerPidf(
                        c[0], c[1], c[2], c[3], VELOCITY_PIDF_OPERATION);
            }
        }

        private void validateRegulatedVelocityRecipe(ScalarRange targetRange,
                                                       double mappingScale) {
            parent.requireDefaultGroupScalingForRegulated("velocity");
            if (targetRange.isUnbounded()) {
                return;
            }
            requireFiniteVelocityEndpoint(
                    "Regulated motor velocity minimum endpoint",
                    targetRange.minValue,
                    mappingScale);
            requireFiniteVelocityEndpoint(
                    "Regulated motor velocity maximum endpoint",
                    targetRange.maxValue,
                    mappingScale);
        }
    }

    private static final class DeviceManagedPositionConfig {
        private double maxPower = 1.0;
        private boolean maxPowerAnswered;
        private Double outerPositionP;
        private boolean outerPositionPAnswered;
        private double[] innerVelocityPidf;
        private boolean innerVelocityPidfAnswered;
        private Integer devicePositionToleranceTicks;
        private boolean devicePositionToleranceTicksAnswered;
        private boolean tuningClosed;

        private boolean hasAnyOverride() {
            return maxPowerAnswered
                    || outerPositionPAnswered
                    || innerVelocityPidfAnswered
                    || devicePositionToleranceTicksAnswered;
        }
    }

    private enum PositionControlKind {
        DEVICE_MANAGED_DEFAULTS,
        DEVICE_MANAGED_TUNED,
        REGULATED
    }

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
            double candidateScale = requireFiniteNonZero(
                    (nativeAtPlantMax - nativeAtPlantMin) / plantSpan,
                    "native endpoint scale");
            preflightPositionChildMapping(checkedBoundedNativeEndpoints(
                    candidateScale, plantMin, nativeAtPlantMin));
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

        /** Validate a complete candidate child mapping before a one-shot mapping/reference answer. */
        protected abstract void preflightPositionChildMapping(double[] boundedStaticNativeEndpoints);

        /**
         * Return the exact shared-native images of finite bounded endpoints when the static
         * reference is already known. Dynamic reference modes are validated when their native
         * anchor becomes available and again at the final grouped/raw command seam.
         */
        protected final double[] boundedStaticNativeEndpointsForValidation() {
            if (!rangeAnswered || !bounded || referenceKind != PositionReferenceKind.STATIC) {
                return null;
            }

            double scale;
            double validationPlantReference;
            double validationNativeReference;
            if (mappingKind == PositionMappingKind.ENDPOINTS) {
                double plantSpan = plantMax - plantMin;
                double nativeSpan = nativeAtPlantMax - nativeAtPlantMin;
                if (!Double.isFinite(plantSpan) || !(plantSpan > 0.0)
                        || !Double.isFinite(nativeSpan)) {
                    throw new IllegalStateException("Position endpoint map has non-finite span: "
                            + "plantSpan=" + plantSpan + ", nativeSpan=" + nativeSpan);
                }
                scale = nativeSpan / plantSpan;
                if (!Double.isFinite(scale) || scale == 0.0) {
                    throw new IllegalStateException("Position endpoint map requires a finite, "
                            + "non-zero native scale, got " + scale);
                }
                validationPlantReference = plantMin;
                validationNativeReference = nativeAtPlantMin;
            } else {
                scale = nativePerPlantUnit;
                validationPlantReference = plantReference;
                validationNativeReference = nativeReference;
            }

            return checkedBoundedNativeEndpoints(
                    scale, validationPlantReference, validationNativeReference);
        }

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
            double checkedScale = requireFiniteNonZero(scale, "nativeUnitsPerPlantUnit");
            preflightPositionChildMapping(null);
            nativePerPlantUnit = checkedScale;
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
            preflightPositionChildMapping(checkedBoundedNativeEndpoints(
                    nativePerPlantUnit, checkedPlantPosition, checkedNativePosition));
            plantReference = checkedPlantPosition;
            nativeReference = checkedNativePosition;
            referenceKind = PositionReferenceKind.STATIC;
            return this;
        }

        private double[] checkedBoundedNativeEndpoints(double nativePerPlantUnit,
                                                       double plantReference,
                                                       double nativeReference) {
            if (!rangeAnswered || !bounded) {
                return null;
            }
            return new double[]{
                    checkedPositionMap(
                            plantMin,
                            plantReference,
                            nativeReference,
                            nativePerPlantUnit,
                            "bounded position minimum endpoint"),
                    checkedPositionMap(
                            plantMax,
                            plantReference,
                            nativeReference,
                            nativePerPlantUnit,
                            "bounded position maximum endpoint")
            };
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
        private static final String MAX_POWER_OPERATION = "FtcActuators.maxPower(...)";
        private static final String OUTER_POSITION_P_OPERATION =
                "FtcActuators.outerPositionP(...)";
        private static final String INNER_VELOCITY_PIDF_OPERATION =
                "FtcActuators.innerVelocityPidf(...)";
        private static final String DEVICE_POSITION_TOLERANCE_OPERATION =
                "FtcActuators.devicePositionToleranceTicks(...)";

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
            answerControl(
                    PositionControlKind.DEVICE_MANAGED_DEFAULTS,
                    "deviceManagedWithDefaults()");
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep deviceManaged() {
            answerControl(PositionControlKind.DEVICE_MANAGED_TUNED, "deviceManaged()");
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep maxPower(double maxPower) {
            requireOpenDeviceManagedTuning("maxPower(...)");
            if (deviceConfig.maxPowerAnswered) {
                throw new IllegalStateException(
                        "maxPower(...) has already been answered for this motor position Plant");
            }
            double checked = FtcControllerConfigurationValidation.requireRunToPositionMaxPower(
                    maxPower, MAX_POWER_OPERATION);
            deviceConfig.maxPower = checked;
            deviceConfig.maxPowerAnswered = true;
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep outerPositionP(double outerPositionP) {
            requireOpenDeviceManagedTuning("outerPositionP(...)");
            if (deviceConfig.outerPositionPAnswered) {
                throw new IllegalStateException("outerPositionP(...) has already been answered for "
                        + "this motor position Plant");
            }
            double checked = FtcControllerConfigurationValidation.requireControllerCoefficient(
                    outerPositionP, OUTER_POSITION_P_OPERATION, "outerPositionP");
            deviceConfig.outerPositionP = checked;
            deviceConfig.outerPositionPAnswered = true;
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep innerVelocityPidf(double p, double i, double d, double f) {
            requireOpenDeviceManagedTuning("innerVelocityPidf(...)");
            if (deviceConfig.innerVelocityPidfAnswered) {
                throw new IllegalStateException("innerVelocityPidf(...) has already been answered "
                        + "for this motor position Plant");
            }
            deviceConfig.innerVelocityPidf =
                    FtcControllerConfigurationValidation.requireControllerPidf(
                            p, i, d, f, INNER_VELOCITY_PIDF_OPERATION);
            deviceConfig.innerVelocityPidfAnswered = true;
            return this;
        }

        @Override
        public MotorDeviceManagedPositionStep devicePositionToleranceTicks(int ticks) {
            requireOpenDeviceManagedTuning("devicePositionToleranceTicks(...)");
            if (deviceConfig.devicePositionToleranceTicksAnswered) {
                throw new IllegalStateException("devicePositionToleranceTicks(...) has already "
                        + "been answered for this motor position Plant");
            }
            int checked =
                    FtcControllerConfigurationValidation.requireDevicePositionToleranceTicks(
                            ticks, DEVICE_POSITION_TOLERANCE_OPERATION);
            deviceConfig.devicePositionToleranceTicks = checked;
            deviceConfig.devicePositionToleranceTicksAnswered = true;
            return this;
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep>
        doneDeviceManaged() {
            requireOpenDeviceManagedTuning("doneDeviceManaged()");
            if (!deviceConfig.hasAnyOverride()) {
                throw new IllegalStateException("deviceManaged() requires at least one controller "
                        + "override before doneDeviceManaged(); use "
                        + "deviceManagedWithDefaults() when no override is needed");
            }
            deviceConfig.tuningClosed = true;
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
                validateDeviceManagedConfiguration();
                parent.validateDeviceManagedPositionRecipe(
                        boundedStaticNativeEndpointsForValidation());
            }
        }

        @Override
        protected Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep>
        createSharedPositionStart() {
            if (isDeviceManaged()) {
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

        @Override
        protected void preflightPositionChildMapping(double[] boundedStaticNativeEndpoints) {
            if (isDeviceManaged()) {
                parent.validateDeviceManagedPositionRecipe(boundedStaticNativeEndpoints);
            } else if (controlKind == PositionControlKind.REGULATED) {
                parent.requireDefaultGroupScalingForRegulated("position");
            }
        }

        private void answerControl(PositionControlKind answer, String operation) {
            requireMutable(operation);
            if (controlKind != null) {
                throw new IllegalStateException("Motor position control ownership has already been answered");
            }
            if (answer != PositionControlKind.REGULATED) {
                parent.validateDeviceManagedPositionRecipe(null);
            } else {
                parent.requireDefaultGroupScalingForRegulated("position");
            }
            controlKind = answer;
        }

        private void requireOpenDeviceManagedTuning(String operation) {
            requireMutable(operation);
            if (controlKind != PositionControlKind.DEVICE_MANAGED_TUNED) {
                throw new IllegalStateException(operation + " requires deviceManaged()");
            }
            if (deviceConfig.tuningClosed) {
                throw new IllegalStateException(operation
                        + " cannot change the closed device-managed position tuning section");
            }
        }

        private boolean isDeviceManaged() {
            return controlKind == PositionControlKind.DEVICE_MANAGED_DEFAULTS
                    || controlKind == PositionControlKind.DEVICE_MANAGED_TUNED;
        }

        private void validateDeviceManagedConfiguration() {
            if (controlKind == PositionControlKind.DEVICE_MANAGED_DEFAULTS) {
                if (deviceConfig.hasAnyOverride() || deviceConfig.tuningClosed) {
                    throw new IllegalStateException("deviceManagedWithDefaults() cannot retain "
                            + "device-managed position tuning");
                }
            } else if (controlKind == PositionControlKind.DEVICE_MANAGED_TUNED) {
                if (!deviceConfig.hasAnyOverride()) {
                    throw new IllegalStateException("deviceManaged() requires at least one "
                            + "controller override before doneDeviceManaged(); use "
                            + "deviceManagedWithDefaults() when no override is needed");
                }
                if (!deviceConfig.tuningClosed) {
                    throw new IllegalStateException("Call doneDeviceManaged() after the "
                            + "device-managed position controller overrides");
                }
            }

            FtcControllerConfigurationValidation.requireRunToPositionMaxPower(
                    deviceConfig.maxPower, MAX_POWER_OPERATION);
            if (deviceConfig.outerPositionP != null) {
                FtcControllerConfigurationValidation.requireControllerCoefficient(
                        deviceConfig.outerPositionP,
                        OUTER_POSITION_P_OPERATION,
                        "outerPositionP");
            }
            if (deviceConfig.innerVelocityPidf != null) {
                double[] c = deviceConfig.innerVelocityPidf;
                FtcControllerConfigurationValidation.requireControllerPidf(
                        c[0], c[1], c[2], c[3], INNER_VELOCITY_PIDF_OPERATION);
            }
            if (deviceConfig.devicePositionToleranceTicks != null) {
                FtcControllerConfigurationValidation.requireDevicePositionToleranceTicks(
                        deviceConfig.devicePositionToleranceTicks,
                        DEVICE_POSITION_TOLERANCE_OPERATION);
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
            specs.get(lastIndex).scale = requireFiniteChildMappingValue(
                    scale, "standard-servo scale(...)", "scale");
            return this;
        }

        @Override
        public ServoGroupAddedStep bias(double bias) {
            lifecycle.requireMutable("standard-servo bias(...)");
            specs.get(lastIndex).bias = requireFiniteChildMappingValue(
                    bias, "standard-servo bias(...)", "bias");
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
            return new GroupedPositionOutput(
                    outs,
                    scales,
                    biases,
                    childNames(specs),
                    PositionChildDomain.SERVO_RAW,
                    "standard-servo position");
        }

        private void validatePositionRecipe(double plantMin,
                                            double plantMax,
                                            PositionMappingKind mappingKind,
                                            double nativeAtPlantMin,
                                            double nativeAtPlantMax) {
            final double sharedAtMin;
            final double sharedAtMax;
            if (mappingKind == PositionMappingKind.NATIVE) {
                sharedAtMin = plantMin;
                sharedAtMax = plantMax;
            } else {
                double plantSpan = plantMax - plantMin;
                double nativeSpan = nativeAtPlantMax - nativeAtPlantMin;
                if (!Double.isFinite(plantSpan) || !(plantSpan > 0.0)
                        || !Double.isFinite(nativeSpan)) {
                    throw new IllegalStateException("Standard-servo endpoint map has non-finite "
                            + "span: plantSpan=" + plantSpan + ", nativeSpan=" + nativeSpan);
                }
                double nativePerPlantUnit = nativeSpan / plantSpan;
                if (!Double.isFinite(nativePerPlantUnit) || nativePerPlantUnit == 0.0) {
                    throw new IllegalStateException("Standard-servo endpoint map requires a finite, "
                            + "non-zero native scale, got " + nativePerPlantUnit);
                }
                sharedAtMin = checkedPositionMap(
                        plantMin,
                        plantMin,
                        nativeAtPlantMin,
                        nativePerPlantUnit,
                        "standard-servo minimum endpoint");
                sharedAtMax = checkedPositionMap(
                        plantMax,
                        plantMin,
                        nativeAtPlantMin,
                        nativePerPlantUnit,
                        "standard-servo maximum endpoint");
            }

            validateServoEndpoint(sharedAtMin, "minimum");
            validateServoEndpoint(sharedAtMax, "maximum");
        }

        private void validateServoEndpoint(double sharedNative, String endpointName) {
            if (!Double.isFinite(sharedNative)) {
                throw new IllegalStateException("Standard-servo " + endpointName
                        + " endpoint maps to non-finite shared native position " + sharedNative);
            }
            for (int i = 0; i < specs.size(); i++) {
                MotorBuilder.Spec spec = specs.get(i);
                requireFiniteChildSpec(spec, "standard-servo position");
                double child = mappedChildCommand(
                        "standard-servo " + endpointName + " endpoint",
                        i,
                        spec,
                        sharedNative);
                requireClosedDomain(
                        child,
                        0.0,
                        1.0,
                        "standard-servo " + endpointName + " endpoint child " + (i + 1)
                                + " ('" + spec.name + "')");
            }
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
            parent.validatePositionRecipe(
                    plantMin, plantMax, PositionMappingKind.NATIVE, 0.0, 0.0);
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
            parent.validatePositionRecipe(
                    plantMin,
                    plantMax,
                    PositionMappingKind.ENDPOINTS,
                    nativeAtPlantMin,
                    nativeAtPlantMax);
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
            parent.validatePositionRecipe(
                    plantMin,
                    plantMax,
                    mappingKind,
                    nativeAtPlantMin,
                    nativeAtPlantMax);
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
            specs.get(lastIndex).scale = requireFiniteChildMappingValue(
                    scale, "CR-servo scale(...)", "scale");
            return this;
        }

        @Override
        public Plants.TargetStep<Plant> power() {
            lifecycle.requireMutable("power()");
            validateDirectPowerRecipe();
            return new PowerTargetBuilder(
                    lifecycle,
                    this::validateDirectPowerRecipe,
                    this::groupedCrServoPowerWithMappings);
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
            return groupedCrServoPowerOutput(outs);
        }

        private PowerOutput groupedCrServoPowerWithMappings() {
            if (specs.size() == 1) {
                MotorBuilder.Spec spec = specs.get(0);
                return FtcHardware.crServoPower(hw, spec.name, spec.direction);
            }
            List<PowerOutput> outs = new ArrayList<>();
            for (MotorBuilder.Spec spec : specs) {
                outs.add(FtcHardware.crServoPower(hw, spec.name, spec.direction));
            }
            return groupedCrServoPowerOutput(outs);
        }

        private PowerOutput groupedCrServoPowerOutput(List<PowerOutput> outputs) {
            return new GroupedPowerOutput(
                    outputs,
                    childScales(specs),
                    childBiases(specs),
                    childNames(specs),
                    "CR-servo power");
        }

        private void validateDirectPowerRecipe() {
            for (int i = 0; i < specs.size(); i++) {
                MotorBuilder.Spec spec = specs.get(i);
                requireFiniteChildSpec(spec, "CR-servo direct power");
                if (spec.bias != 0.0) {
                    throw new IllegalStateException("CR-servo direct power child " + (i + 1)
                            + " ('" + spec.name + "') must preserve neutral zero with bias 0.0, got "
                            + spec.bias);
                }
                validateMappedPowerEndpoint(
                        "CR-servo direct power", i, spec, -1.0);
                validateMappedPowerEndpoint(
                        "CR-servo direct power", i, spec, 1.0);
            }
        }

        private void requireDefaultGroupScalingForRegulated() {
            if (specs.size() <= 1) return;
            for (MotorBuilder.Spec spec : specs) {
                requireFiniteChildSpec(spec, "regulated CR-servo position");
                if (spec.scale != 1.0 || spec.bias != 0.0) {
                    throw new IllegalStateException("Regulated CR-servo position control requires "
                            + "exact child scale 1.0 and bias 0.0 for CR servo '" + spec.name + "'");
                }
            }
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
            parent.requireDefaultGroupScalingForRegulated();
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

        @Override
        protected void preflightPositionChildMapping(double[] boundedStaticNativeEndpoints) {
            parent.requireDefaultGroupScalingForRegulated();
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

    /**
     * Exact shared-power fan-out. Every child is validated before any write, and the original
     * value is copied directly so identity includes the signed-zero bit pattern.
     */
    private static final class IdentityGroupedPowerOutput implements PowerOutput {
        private final List<PowerOutput> outputs;
        private final List<String> names;
        private final String family;
        private double last;

        private IdentityGroupedPowerOutput(List<PowerOutput> outputs,
                                           List<String> names,
                                           String family) {
            this.outputs = new ArrayList<>(Objects.requireNonNull(outputs, "outputs"));
            this.names = new ArrayList<>(Objects.requireNonNull(names, "names"));
            this.family = Objects.requireNonNull(family, "family");
            if (this.outputs.isEmpty() || this.outputs.size() != this.names.size()) {
                throw new IllegalArgumentException(
                        "Grouped identity power output children must be non-empty and match names");
            }
        }

        @Override
        public void setPower(double power) {
            double[] childPowers = new double[outputs.size()];
            for (int i = 0; i < outputs.size(); i++) {
                childPowers[i] = power;
                requireClosedDomain(
                        childPowers[i],
                        -1.0,
                        1.0,
                        family + " runtime child " + (i + 1) + " ('" + names.get(i) + "')");
            }
            last = power;
            for (int i = 0; i < outputs.size(); i++) {
                outputs.get(i).setPower(childPowers[i]);
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

    /** Child-mapped fan-out for direct normalized-power Plants. */
    private static final class GroupedPowerOutput implements PowerOutput {
        private final List<PowerOutput> outputs;
        private final double[] scales;
        private final double[] biases;
        private final List<String> names;
        private final String family;
        private double last;

        private GroupedPowerOutput(List<PowerOutput> outputs,
                                   double[] scales,
                                   double[] biases,
                                   List<String> names,
                                   String family) {
            this.outputs = new ArrayList<>(Objects.requireNonNull(outputs, "outputs"));
            this.scales = Objects.requireNonNull(scales, "scales").clone();
            this.biases = Objects.requireNonNull(biases, "biases").clone();
            this.names = new ArrayList<>(Objects.requireNonNull(names, "names"));
            this.family = Objects.requireNonNull(family, "family");
            requireMatchingChildShape(
                    this.outputs.size(), this.scales, this.biases, this.names);
        }

        @Override
        public void setPower(double power) {
            double[] childPowers = new double[outputs.size()];
            for (int i = 0; i < outputs.size(); i++) {
                childPowers[i] = mappedChildCommand(
                        family + " runtime command", i, names.get(i), scales[i], biases[i], power);
                requireClosedDomain(
                        childPowers[i],
                        -1.0,
                        1.0,
                        family + " runtime child " + (i + 1) + " ('" + names.get(i) + "')");
            }
            last = power;
            for (int i = 0; i < outputs.size(); i++) {
                outputs.get(i).setPower(childPowers[i]);
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

    private enum PositionChildDomain {
        SERVO_RAW,
        MOTOR_TICKS
    }

    private static final class GroupedPositionOutput implements PositionOutput {
        private final List<PositionOutput> outputs;
        private final double[] scales;
        private final double[] biases;
        private final List<String> names;
        private final PositionChildDomain domain;
        private final String family;
        private double last;

        private GroupedPositionOutput(List<PositionOutput> outputs,
                                      double[] scales,
                                      double[] biases,
                                      List<String> names,
                                      PositionChildDomain domain,
                                      String family) {
            this.outputs = new ArrayList<>(Objects.requireNonNull(outputs, "outputs"));
            this.scales = Objects.requireNonNull(scales, "scales").clone();
            this.biases = Objects.requireNonNull(biases, "biases").clone();
            this.names = new ArrayList<>(Objects.requireNonNull(names, "names"));
            this.domain = Objects.requireNonNull(domain, "domain");
            this.family = Objects.requireNonNull(family, "family");
            requireMatchingChildShape(
                    this.outputs.size(), this.scales, this.biases, this.names);
        }

        @Override
        public void setPosition(double position) {
            double[] childPositions = new double[outputs.size()];
            for (int i = 0; i < outputs.size(); i++) {
                String childDescription = family + " runtime child " + (i + 1)
                        + " ('" + names.get(i) + "')";
                childPositions[i] = mappedChildCommand(
                        family + " runtime command",
                        i,
                        names.get(i),
                        scales[i],
                        biases[i],
                        position);
                if (domain == PositionChildDomain.SERVO_RAW) {
                    requireClosedDomain(childPositions[i], 0.0, 1.0, childDescription);
                } else {
                    checkedMotorPositionTicks(childPositions[i], childDescription);
                }
            }
            last = position;
            for (int i = 0; i < outputs.size(); i++) {
                outputs.get(i).setPosition(childPositions[i]);
            }
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
        private final List<String> names;
        private final String family;
        private double last;

        private GroupedVelocityOutput(List<VelocityOutput> outputs,
                                      double[] scales,
                                      double[] biases,
                                      List<String> names,
                                      String family) {
            this.outputs = new ArrayList<>(Objects.requireNonNull(outputs, "outputs"));
            this.scales = Objects.requireNonNull(scales, "scales").clone();
            this.biases = Objects.requireNonNull(biases, "biases").clone();
            this.names = new ArrayList<>(Objects.requireNonNull(names, "names"));
            this.family = Objects.requireNonNull(family, "family");
            requireMatchingChildShape(
                    this.outputs.size(), this.scales, this.biases, this.names);
        }

        @Override
        public void setVelocity(double velocity) {
            double[] childVelocities = new double[outputs.size()];
            for (int i = 0; i < outputs.size(); i++) {
                childVelocities[i] = mappedChildCommand(
                        family + " runtime command",
                        i,
                        names.get(i),
                        scales[i],
                        biases[i],
                        velocity);
            }
            last = velocity;
            for (int i = 0; i < outputs.size(); i++) {
                outputs.get(i).setVelocity(childVelocities[i]);
            }
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
                double[] samples = new double[copy.size()];
                for (int i = 0; i < copy.size(); i++) {
                    samples[i] = copy.get(i).getAsDouble(clock);
                    if (!Double.isFinite(samples[i])) {
                        return Double.NaN;
                    }
                }
                return overflowSafeMean(samples);
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
        double[] checkedScales = Objects.requireNonNull(scales, "scales").clone();
        double[] checkedBiases = Objects.requireNonNull(biases, "biases").clone();
        if (checkedScales.length != copy.size() || checkedBiases.length != copy.size()) {
            throw new IllegalArgumentException("sources/scales/biases must have matching lengths");
        }
        return new ScalarSource() {
            @Override
            public double getAsDouble(edu.ftcphoenix.fw.core.time.LoopClock clock) {
                double[] inverseMapped = new double[copy.size()];
                for (int i = 0; i < copy.size(); i++) {
                    if (!Double.isFinite(checkedScales[i]) || checkedScales[i] == 0.0
                            || !Double.isFinite(checkedBiases[i])) {
                        return Double.NaN;
                    }
                    double v = copy.get(i).getAsDouble(clock);
                    if (!Double.isFinite(v)) {
                        return Double.NaN;
                    }
                    double withoutBias = v - checkedBiases[i];
                    if (!Double.isFinite(withoutBias)) {
                        return Double.NaN;
                    }
                    inverseMapped[i] = withoutBias / checkedScales[i];
                    if (!Double.isFinite(inverseMapped[i])) {
                        return Double.NaN;
                    }
                }
                return overflowSafeMean(inverseMapped);
            }

            @Override
            public void reset() {
                for (ScalarSource source : copy) source.reset();
            }
        }.memoized();
    }

    private static double overflowSafeMean(double[] finiteValues) {
        if (finiteValues == null || finiteValues.length == 0) {
            return Double.NaN;
        }
        double maxMagnitude = 0.0;
        for (double value : finiteValues) {
            if (!Double.isFinite(value)) {
                return Double.NaN;
            }
            maxMagnitude = Math.max(maxMagnitude, Math.abs(value));
        }
        if (maxMagnitude == 0.0) {
            return 0.0;
        }

        double scaledSum = 0.0;
        for (double value : finiteValues) {
            scaledSum += value / maxMagnitude;
        }
        double mean = (scaledSum / finiteValues.length) * maxMagnitude;
        return Double.isFinite(mean) ? mean : Double.NaN;
    }

    private static double[] childScales(List<MotorBuilder.Spec> specs) {
        double[] values = new double[specs.size()];
        for (int i = 0; i < specs.size(); i++) {
            values[i] = specs.get(i).scale;
        }
        return values;
    }

    private static double[] childBiases(List<MotorBuilder.Spec> specs) {
        double[] values = new double[specs.size()];
        for (int i = 0; i < specs.size(); i++) {
            values[i] = specs.get(i).bias;
        }
        return values;
    }

    private static List<String> childNames(List<MotorBuilder.Spec> specs) {
        List<String> values = new ArrayList<>(specs.size());
        for (MotorBuilder.Spec spec : specs) {
            values.add(spec.name);
        }
        return values;
    }

    private static void requireMatchingChildShape(int outputCount,
                                                  double[] scales,
                                                  double[] biases,
                                                  List<String> names) {
        if (outputCount == 0 || scales.length != outputCount || biases.length != outputCount
                || names.size() != outputCount) {
            throw new IllegalArgumentException(
                    "outputs/scales/biases/names must be non-empty and have matching lengths");
        }
    }

    private static double requireFiniteChildMappingValue(double value,
                                                         String operation,
                                                         String argument) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(operation + " " + argument
                    + " must be finite, got " + value);
        }
        return value;
    }

    private static void requireFiniteChildSpec(MotorBuilder.Spec spec, String operation) {
        if (!Double.isFinite(spec.scale) || !Double.isFinite(spec.bias)) {
            throw new IllegalStateException(operation + " child '" + spec.name
                    + "' requires finite scale and bias, got scale=" + spec.scale
                    + ", bias=" + spec.bias);
        }
    }

    private static double mappedChildCommand(String operation,
                                             int childIndex,
                                             MotorBuilder.Spec spec,
                                             double sharedNative) {
        return mappedChildCommand(
                operation, childIndex, spec.name, spec.scale, spec.bias, sharedNative);
    }

    private static double mappedChildCommand(String operation,
                                             int childIndex,
                                             String childName,
                                             double scale,
                                             double bias,
                                             double sharedNative) {
        if (!Double.isFinite(sharedNative) || !Double.isFinite(scale) || !Double.isFinite(bias)) {
            throw new IllegalStateException(operation + " child " + (childIndex + 1) + " ('"
                    + childName + "') requires finite shared value, scale, and bias, got shared="
                    + sharedNative + ", scale=" + scale + ", bias=" + bias);
        }
        double scaled = scale * sharedNative;
        if (!Double.isFinite(scaled)) {
            throw new IllegalStateException(operation + " child " + (childIndex + 1) + " ('"
                    + childName + "') overflows scale * shared: scale=" + scale + ", shared="
                    + sharedNative + ", result=" + scaled);
        }
        double child = scaled + bias;
        if (!Double.isFinite(child)) {
            throw new IllegalStateException(operation + " child " + (childIndex + 1) + " ('"
                    + childName + "') overflows scaled + bias: scaled=" + scaled + ", bias="
                    + bias + ", result=" + child);
        }
        return child;
    }

    private static void validateMappedPowerEndpoint(String operation,
                                                    int childIndex,
                                                    MotorBuilder.Spec spec,
                                                    double sharedPower) {
        double child = mappedChildCommand(operation, childIndex, spec, sharedPower);
        requireClosedDomain(
                child,
                -1.0,
                1.0,
                operation + " child " + (childIndex + 1) + " ('" + spec.name
                        + "') at shared endpoint " + sharedPower);
    }

    private static void requireClosedDomain(double value,
                                            double min,
                                            double max,
                                            String description) {
        if (!Double.isFinite(value) || value < min || value > max) {
            throw new IllegalStateException(description + " must be finite and inside [" + min
                    + ", " + max + "], got " + value);
        }
    }

    private static int checkedMotorPositionTicks(double position, String description) {
        if (!Double.isFinite(position)) {
            throw new IllegalStateException(description
                    + " must be a finite native motor position, got " + position);
        }
        long rounded = Math.round(position);
        if (rounded < Integer.MIN_VALUE || rounded > Integer.MAX_VALUE) {
            throw new IllegalStateException(description + " rounds to " + rounded
                    + " ticks outside the FTC signed 32-bit target domain ["
                    + Integer.MIN_VALUE + ", " + Integer.MAX_VALUE + "]");
        }
        return (int) rounded;
    }

    private static double checkedPositionMap(double plantPosition,
                                             double plantReference,
                                             double nativeReference,
                                             double nativePerPlantUnit,
                                             String description) {
        double plantDelta = plantPosition - plantReference;
        if (!Double.isFinite(plantDelta)) {
            throw new IllegalStateException(description + " overflows plantPosition - "
                    + "plantReference: plantPosition=" + plantPosition + ", plantReference="
                    + plantReference + ", result=" + plantDelta);
        }
        double scaledDelta = nativePerPlantUnit * plantDelta;
        if (!Double.isFinite(scaledDelta)) {
            throw new IllegalStateException(description + " overflows native scale * plant delta: "
                    + "scale=" + nativePerPlantUnit + ", delta=" + plantDelta + ", result="
                    + scaledDelta);
        }
        double nativePosition = nativeReference + scaledDelta;
        if (!Double.isFinite(nativePosition)) {
            throw new IllegalStateException(description + " overflows native reference + scaled "
                    + "delta: nativeReference=" + nativeReference + ", scaledDelta=" + scaledDelta
                    + ", result=" + nativePosition);
        }
        return nativePosition;
    }

    private static double requireFiniteVelocityEndpoint(String description,
                                                        double plantVelocity,
                                                        double nativePerPlantUnit) {
        double nativeVelocity = plantVelocity * nativePerPlantUnit;
        if (!Double.isFinite(nativeVelocity)) {
            throw new IllegalArgumentException(description + " " + plantVelocity
                    + " overflows plantVelocity * nativePerPlantUnit with scale "
                    + nativePerPlantUnit + ", producing " + nativeVelocity);
        }
        return nativeVelocity;
    }

    private static double requireFiniteNonZero(double value, String name) {
        if (!Double.isFinite(value) || value == 0.0)
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
