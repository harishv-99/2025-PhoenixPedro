package edu.ftcsushi.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.PlantTargetGate;
import edu.ftcsushi.fw.actuation.PlantTargetResolver;
import edu.ftcsushi.fw.actuation.Plants;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.core.control.ScalarRegulator;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.hal.PowerLimitedPositionOutput;
import edu.ftcsushi.fw.core.hal.PositionOutput;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.hal.VelocityOutput;
import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;

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
 * to native velocity {@code 0.0}. Sushi therefore exposes velocity {@code nativeUnits()} and
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
 * every child command before writing any child. A grouped power output marks its top-level command
 * unknown at operation entry, publishes the requested command only after every child write returns
 * normally, and begins an ordered best-effort stop traversal after a non-finite request or
 * child-write {@link RuntimeException}. The traversal continues across {@code RuntimeException}s;
 * an {@link Error} remains uncaught and can interrupt later cleanup.
 * This prevalidation prevents a partial fan-out caused by predictable command rejection; it cannot
 * make sequential SDK writes atomic. If rejection propagates through a Plant update, the Plant may
 * then invoke the group's natural stop as a separate fail-safe cleanup. Inverse-mapped grouped
 * feedback returns {@link Double#NaN} if any child conversion or aggregate is non-finite.
 * A motor-position child transform describes only the native position coordinate; temporary
 * calibration-search power is a separate normalized command and fans out identically through each
 * configured motor direction. Stopping a grouped output traverses children in declaration order
 * and continues across {@link RuntimeException}s; the first remains primary and later distinct
 * failures are suppressed. An {@link Error} remains uncaught and can interrupt later stops.</p>
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
 *     .nonPeriodic()
 *         .bounded(0.0, 4200.0)
 *         .nativeUnits()
 *         .needsReference("lift not homed")
 *     .positionTolerance(20.0)
 *     .outputPowerLimitedTo(0.8)
 *     .targetFromNewCommand(0.0)
 *     .build();
 *
 * this.flywheel = FtcActuators.plant(hardwareMap)
 *     .motor(cfg.flywheelMotorName, cfg.flywheelDirection)
 *     .velocity()
 *     .deviceManaged()
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
 *         .rangeMapsToNative(cfg.clawClosedNativePosition,
 *                 cfg.clawOpenNativePosition)
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
         * velocity control or a Sushi-regulated loop driven by native velocity feedback.</p>
         */
        MotorVelocityControlStep velocity();

        /**
         * Begin the guided motor-position builder.
         *
         * <p>The next required question is who manages the position loop: FTC device-managed control
         * or a Sushi-regulated loop driven by native feedback. Every completed choice builds a
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
         * Use FTC device-managed velocity control without changing the controller's then-active
         * coefficients, then continue to velocity target bounds. Sushi does not know or claim
         * that those externally owned coefficients are tuned for this mechanism. The answer first
         * validates that every retained child has finite nonzero scale and exact zero bias, so a
         * rejected answer can be corrected and retried without hardware resolution.
         *
         * @throws IllegalStateException if a retained child mapping is incompatible
         */
        Plants.VelocityBoundsStep<Plants.TargetStep<Plant>> deviceManaged();

        /**
         * Enter the FTC device-managed velocity tuning branch before continuing to target bounds.
         * The answer applies the same retryable child-mapping preflight as
         * {@link #deviceManaged()}.
         *
         * @throws IllegalStateException if a retained child mapping is incompatible
         */
        MotorDeviceManagedVelocityStep deviceManagedWithOverrides();

        /**
         * Use a Sushi-regulated velocity loop that drives motor power from native velocity feedback.
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
         * {@link MotorVelocityControlStep#deviceManagedWithOverrides()}, or if this answer was
         * already supplied
         */
        Plants.VelocityBoundsStep<Plants.TargetStep<Plant>> velocityPidf(
                double p, double i, double d, double f);
    }

    /**
     * Required native-feedback question for regulated motor velocity control.
     */
    public interface MotorRegulatedVelocityFeedbackStep {
        /**
         * Use the selected motor's SDK-reported internal encoder velocity in native ticks/sec.
         */
        Plants.VelocityBoundsStep<Plants.VelocityControlStep> internalEncoder();

        /**
         * Use one named selected motor's SDK-reported internal encoder velocity in native ticks/sec.
         * Matching ignores surrounding whitespace like FTC lookup and remains case-sensitive.
         */
        Plants.VelocityBoundsStep<Plants.VelocityControlStep> internalEncoder(String motorName);

        /**
         * Use the average SDK-reported internal encoder velocity of all selected motors.
         */
        Plants.VelocityBoundsStep<Plants.VelocityControlStep> averageInternalEncoders();

        /**
         * Derive native velocity in ticks/sec from a named external encoder's position samples.
         *
         * <p>The FTC position counter is made continuous across signed 32-bit rollover before the
         * rate is calculated. This avoids depending on the SDK's narrower direct-velocity result
         * for a high-count-rate external encoder.</p>
         */
        Plants.VelocityBoundsStep<Plants.VelocityControlStep> externalEncoder(String name);

        /**
         * Derive native velocity in ticks/sec from a named external encoder's position samples,
         * with an explicit logical direction.
         */
        Plants.VelocityBoundsStep<Plants.VelocityControlStep> externalEncoder(
                String name, Direction direction);

        /**
         * Use a caller-supplied native velocity source.
         */
        Plants.VelocityBoundsStep<Plants.VelocityControlStep> nativeFeedback(ScalarSource source);
    }

    /**
     * First motor-position question: who manages the position loop?
     *
     * <p>Every choice below builds a feedback {@link PositionPlant} that supports temporary
     * raw-power calibration search. {@link PositionPlant#beginCalibrationSearch(double)} stops the
     * prior normal output and stages the search request; it does not submit that search power.
     * The mechanism or subsystem remains the sole Plant heartbeat owner, and its normal downstream
     * {@link Plant#update(edu.ftcsushi.fw.core.time.LoopClock)} call is the sole search-command
     * writer. Search power must be finite in the inclusive normalized {@code [-1.0, +1.0]} range
     * and is rejected before acquisition rather than clamped. For a device-managed motor group,
     * the shared search command is submitted identically to every child through its configured
     * {@link Direction}; native position scale/bias is not a raw-power policy. The mechanism owner
     * still chooses and physically validates a safe magnitude and direction.</p>
     */
    public interface MotorPositionControlStep {
        /**
         * Use FTC RUN_TO_POSITION without changing the controller's then-active coefficients, then
         * continue to position periodicity. Omitted output-power policy uses magnitude {@code 1.0};
         * neither that framework baseline nor the externally owned coefficients claim reviewed
         * physical safety or tuning. Finite nonzero child scales and finite position-alignment
         * biases are validated before this answer is retained.
         *
         * @throws IllegalStateException if a retained child mapping is incompatible
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>> deviceManaged();

        /**
         * Enter the FTC RUN_TO_POSITION tuning branch before continuing to position periodicity.
         * This applies the same retryable child-mapping preflight as
         * {@link #deviceManaged()}.
         *
         * @throws IllegalStateException if a retained child mapping is incompatible
         */
        MotorDeviceManagedPositionStep deviceManagedWithOverrides();

        /**
         * Use a Sushi-regulated loop that drives motor power from native position feedback.
         * Grouped regulated control requires exact child scale {@code 1.0} and bias {@code 0.0}.
         *
         * @throws IllegalStateException if a grouped child mapping is not exact identity
         */
        MotorRegulatedPositionFeedbackStep regulated();
    }

    /**
     * Nonempty override branch for FTC device-managed motor position control. Each individual
     * setting is optional, but at least one must be answered before the branch is closed.
     */
    public interface MotorDeviceManagedPositionStep {
        /**
         * Set FTC's outer position-loop proportional coefficient in the inclusive, symmetric
         * pinned controller public conversion domain
         * {@code [-Integer.MAX_VALUE / 65536.0, +Integer.MAX_VALUE / 65536.0]}.
         *
         * @throws IllegalArgumentException if {@code outerPositionP} is non-finite or outside the
         * supported controller conversion domain
         * @throws IllegalStateException if this setting was already answered, the tuning section is
         * closed, or the recipe did not enter through
         * {@link MotorPositionControlStep#deviceManagedWithOverrides()}
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
         * closed, or the recipe did not enter through
         * {@link MotorPositionControlStep#deviceManagedWithOverrides()}
         */
        MotorDeviceManagedPositionStep innerVelocityPidf(double p, double i, double d, double f);

        /**
         * Set FTC's native device target-position tolerance in the inclusive {@code [0, 65535]}
         * encoder-tick domain.
         *
         * @throws IllegalArgumentException if {@code ticks} is outside {@code [0, 65535]}
         * @throws IllegalStateException if this setting was already answered, the tuning section is
         * closed, or the recipe did not enter through
         * {@link MotorPositionControlStep#deviceManagedWithOverrides()}
         */
        MotorDeviceManagedPositionStep devicePositionToleranceTicks(int ticks);

        /**
         * Close a nonempty device-managed override section and continue to position periodicity.
         * Use {@link MotorPositionControlStep#deviceManaged()} instead of opening and immediately
         * closing an empty section.
         *
         * @throws IllegalStateException if no tuning setting was accepted, this section was already
         * closed, or the recipe did not enter through
         * {@link MotorPositionControlStep#deviceManagedWithOverrides()}
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>> doneOverrides();
    }

    /**
     * Required feedback question for regulated motor position control.
     */
    public interface MotorRegulatedPositionFeedbackStep {
        /**
         * Use the selected motor's internal encoder position in native ticks.
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> internalEncoder();

        /**
         * Use one named selected motor's internal encoder position in native ticks.
         * Matching ignores surrounding whitespace like FTC lookup and remains case-sensitive.
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> internalEncoder(String motorName);

        /**
         * Use the average internal encoder position of all selected motors.
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> averageInternalEncoders();

        /**
         * Use a named external encoder's position in native ticks.
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> externalEncoder(String name);

        /**
         * Use a named external encoder's position in native ticks with an explicit logical
         * direction.
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> externalEncoder(String name, Direction direction);

        /**
         * Use a caller-supplied native position source.
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> nativeFeedback(ScalarSource source);
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
         * <p>The bounds already supplied to {@link ServoPositionBoundsStep#bounded(double, double)}
         * must be the robot-reviewed safe native subrange. FTC's {@code [0.0, 1.0]} Servo envelope
         * is an adapter limit, not a claim that the mechanism can safely traverse that full range.
         * Builder validation proves only that submitted values fit the SDK domain; it cannot prove
         * linkage clearance or physical safety.</p>
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
         * <p>Use backed-off, hardware-reviewed endpoint facts from robot configuration. Validation
         * proves only that the resulting finite commands fit the FTC Servo {@code [0.0, 1.0]}
         * envelope; it cannot prove linkage clearance, safe travel, or physical position.</p>
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
     * {@link Plant#update(edu.ftcsushi.fw.core.time.LoopClock)} call is the sole search-command
     * writer. Search power must be finite in the inclusive normalized {@code [-1.0, +1.0]} range
     * and is rejected before acquisition rather than clamped; the mechanism owner still chooses
     * and physically validates a safe magnitude and direction.</p>
     */
    public interface CrServoPositionControlStep {
        /**
         * Use a Sushi-regulated loop that drives CR-servo power from native position feedback.
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
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> externalEncoder(String name);

        /**
         * Use a named external encoder's position in native ticks with an explicit logical
         * direction.
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> externalEncoder(String name, Direction direction);

        /**
         * Use a caller-supplied native position source.
         */
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> nativeFeedback(ScalarSource source);
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
                        + "target selection; start a new "
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
            requireRateUnanswered();
            requireTargetRateCompatible("maxTargetRate(...)");
            requireFinitePositive(maxDeltaPerSec, "maxDeltaPerSec");
            rateAnswered = true;
            guardSpecs.add(GuardSpec.rate(maxDeltaPerSec));
            return this;
        }

        @Override
        public final Plants.TargetGuardStep<P> maxTargetRates(double maxUpPerSec,
                                                               double maxDownPerSec) {
            requireOpenGuards("maxTargetRates(...)");
            requireRateUnanswered();
            requireTargetRateCompatible("maxTargetRates(...)");
            requireFinitePositive(maxUpPerSec, "maxUpPerSec");
            requireFinitePositive(maxDownPerSec, "maxDownPerSec");
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
            try {
                if (targetProvenance == null) {
                    throw new IllegalStateException(plantName
                            + " requires targetFromNewCommand(...), targetExactlyFrom(...), or "
                            + "targetFromResolver(...) before build()");
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
                                sharedGuards.fallbackTargetUnless(
                                        spec.name, spec.allowed, spec.first);
                                break;
                            case FALLBACK_TARGET:
                                sharedGuards.fallbackTargetUnless(
                                        spec.name, spec.gate, spec.first);
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
                return finishBuiltPlant(sharedBuild.build());
            } catch (RuntimeException failure) {
                onBuildFailure(failure);
                throw failure;
            }
        }

        protected abstract void validateRecipe();

        protected abstract ScalarRange configuredTargetRange();

        protected abstract Plants.TargetStep<P> createSharedTargetStep();

        /** Reject a target-rate guard before retaining an incompatible FTC control recipe. */
        protected void requireTargetRateCompatible(String answer) {
            // Most FTC Plant realizations have no competing motion-profile owner.
        }

        protected final boolean hasTargetRateConfigured() {
            return rateAnswered;
        }

        /** Return a public tail that exposes only target selection, guards, and the final build. */
        protected final Plants.TargetStep<P> targetStepView() {
            return new FtcTargetStepView<>(this);
        }

        /** Return the capability-specific symmetric-output answer followed by the narrow tail. */
        protected final Plants.SymmetricOutputPowerPolicyStep<P> symmetricOutputPowerPolicyView(
                SymmetricOutputPowerAnswer answer) {
            return new FtcSymmetricOutputPowerPolicyView<>(this, answer);
        }

        /** Add boundary-only metadata without changing the shared Plant engine. */
        protected P finishBuiltPlant(P plant) {
            return plant;
        }

        /** Best-effort boundary cleanup after a construction failure. */
        protected void onBuildFailure(RuntimeException failure) {
            // Most FTC Plant recipes have no construction-time hardware configuration to restore.
        }

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
                throw new IllegalStateException("A target has already been selected for this "
                        + plantName
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

    /** One private callback keeps the public output-policy facade independent of a wide builder. */
    private interface SymmetricOutputPowerAnswer {
        void answer(double maximumMagnitude);
    }

    /** Runtime-narrow target facade for internal FTC builders that retain several staged roles. */
    private static final class FtcTargetStepView<P extends Plant>
            implements Plants.TargetStep<P> {
        private final FtcTargetBuilder<P> delegate;

        private FtcTargetStepView(FtcTargetBuilder<P> delegate) {
            this.delegate = Objects.requireNonNull(delegate, "delegate");
        }

        @Override
        public Plants.TargetGuardStep<P> targetGuards() {
            delegate.targetGuards();
            return new FtcTargetGuardStepView<>(delegate, this);
        }

        @Override
        public Plants.BuildStep<P> targetFromNewCommand(double initialValue) {
            delegate.targetFromNewCommand(initialValue);
            return new FtcBuildStepView<>(delegate);
        }

        @Override
        public Plants.BuildStep<P> targetFromResolver(PlantTargetResolver resolver) {
            delegate.targetFromResolver(resolver);
            return new FtcBuildStepView<>(delegate);
        }
    }

    /** Runtime-narrow guard facade whose close operation restores only its target facade. */
    private static final class FtcTargetGuardStepView<P extends Plant>
            implements Plants.TargetGuardStep<P> {
        private final FtcTargetBuilder<P> delegate;
        private final Plants.TargetStep<P> targetStep;

        private FtcTargetGuardStepView(FtcTargetBuilder<P> delegate,
                                       Plants.TargetStep<P> targetStep) {
            this.delegate = Objects.requireNonNull(delegate, "delegate");
            this.targetStep = Objects.requireNonNull(targetStep, "targetStep");
        }

        @Override
        public Plants.TargetGuardStep<P> maxTargetRate(double maxDeltaPerSec) {
            delegate.maxTargetRate(maxDeltaPerSec);
            return this;
        }

        @Override
        public Plants.TargetGuardStep<P> maxTargetRates(
                double maxUpPerSec,
                double maxDownPerSec) {
            delegate.maxTargetRates(maxUpPerSec, maxDownPerSec);
            return this;
        }

        @Override
        public Plants.TargetGuardStep<P> holdLastTargetUnless(
                String name,
                BooleanSource allowed) {
            delegate.holdLastTargetUnless(name, allowed);
            return this;
        }

        @Override
        public Plants.TargetGuardStep<P> holdLastTargetUnless(
                String name,
                PlantTargetGate gate) {
            delegate.holdLastTargetUnless(name, gate);
            return this;
        }

        @Override
        public Plants.TargetGuardStep<P> fallbackTargetUnless(
                String name,
                BooleanSource allowed,
                double fallbackTarget) {
            delegate.fallbackTargetUnless(name, allowed, fallbackTarget);
            return this;
        }

        @Override
        public Plants.TargetGuardStep<P> fallbackTargetUnless(
                String name,
                PlantTargetGate gate,
                double fallbackTarget) {
            delegate.fallbackTargetUnless(name, gate, fallbackTarget);
            return this;
        }

        @Override
        public Plants.TargetStep<P> doneTargetGuards() {
            delegate.doneTargetGuards();
            return targetStep;
        }
    }

    /** Runtime-narrow final build facade. */
    private static final class FtcBuildStepView<P extends Plant>
            implements Plants.BuildStep<P> {
        private final FtcTargetBuilder<P> delegate;

        private FtcBuildStepView(FtcTargetBuilder<P> delegate) {
            this.delegate = Objects.requireNonNull(delegate, "delegate");
        }

        @Override
        public P build() {
            return delegate.build();
        }
    }

    /** Symmetric device-output capability without the software-only signed/control interfaces. */
    private static final class FtcSymmetricOutputPowerPolicyView<P extends Plant>
            implements Plants.SymmetricOutputPowerPolicyStep<P> {
        private final FtcTargetStepView<P> targetStep;
        private final SymmetricOutputPowerAnswer outputAnswer;

        private FtcSymmetricOutputPowerPolicyView(
                FtcTargetBuilder<P> delegate,
                SymmetricOutputPowerAnswer outputAnswer) {
            targetStep = new FtcTargetStepView<>(delegate);
            this.outputAnswer = Objects.requireNonNull(outputAnswer, "outputAnswer");
        }

        @Override
        public Plants.TargetStep<P> outputPowerLimitedTo(double maximumMagnitude) {
            outputAnswer.answer(maximumMagnitude);
            return targetStep;
        }

        @Override
        public Plants.TargetGuardStep<P> targetGuards() {
            return targetStep.targetGuards();
        }

        @Override
        public Plants.BuildStep<P> targetFromNewCommand(double initialValue) {
            return targetStep.targetFromNewCommand(initialValue);
        }

        @Override
        public Plants.BuildStep<P> targetFromResolver(PlantTargetResolver resolver) {
            return targetStep.targetFromResolver(resolver);
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

    private enum VelocitySetpointKind {
        DIRECT,
        ACCELERATION_LIMITED,
        CUSTOM
    }

    private enum VelocityFeedforwardKind {
        NONE,
        MOTION,
        LIFT
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
        private DcMotorEx lastSingleDeviceManagedPositionMotor;
        private List<DcMotorEx> lastDeviceManagedPositionMotors;
        private List<DcMotorEx> pendingOverrideMotors;
        private List<String> pendingOverrideNames;
        private List<FtcMotorPidfConfiguration> pendingVelocityBaselines;
        private List<PositionControllerBaseline> pendingPositionBaselines;

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

        private FtcDeviceManagedVelocityBinding deviceManagedVelocityBinding(
                DeviceManagedVelocityConfig cfg,
                double nativePerPlantUnit,
                double plantTolerance) {
            ensureFeedbackScalesNonZero("device-managed motor velocity");
            List<DcMotorEx> motors = resolveDistinctMotors("device-managed velocity");
            List<String> names = childNames(specs);
            List<Direction> directions = childDirections(specs);
            List<FtcMotorPidfConfiguration> before = null;
            if (cfg.velocityPidf != null) {
                before = captureVelocityConfigurations(motors, names,
                        "FtcActuators device-managed velocity pre-override capture");
                rememberVelocityOverride(motors, names, before);
                try {
                    applyVelocityOverride(motors, names, cfg.velocityPidf);
                } catch (RuntimeException failure) {
                    rollbackPendingOverride(failure);
                    throw controllerOverrideFailure(
                            "Failed to apply the device-managed velocity override group: "
                                    + failure.getMessage()
                                    + "; every captured controller restoration was attempted",
                            failure);
                }
            }

            try {
                return new FtcDeviceManagedVelocityBinding(
                        motors,
                        names,
                        directions,
                        childScales(specs),
                        nativePerPlantUnit,
                        plantTolerance,
                        cfg.velocityPidf);
            } catch (RuntimeException failure) {
                if (before != null) rollbackPendingOverride(failure);
                throw failure;
            }
        }

        private PowerLimitedPositionOutput groupedMotorPosition(DeviceManagedPositionConfig cfg) {
            ensureFeedbackScalesNonZero("device-managed motor position");
            List<DcMotorEx> motors = resolveDistinctMotors("device-managed position");
            lastDeviceManagedPositionMotors = new ArrayList<>(motors);
            List<String> names = childNames(specs);
            List<PositionControllerBaseline> before = null;
            if (cfg.hasAnyOverride()) {
                before = capturePositionConfigurations(motors, names,
                        "FtcActuators device-managed position pre-override capture");
                rememberPositionOverride(motors, names, before);
                try {
                    applyPositionOverrides(motors, names, cfg);
                } catch (RuntimeException failure) {
                    rollbackPendingOverride(failure);
                    throw controllerOverrideFailure(
                            "Failed to apply the device-managed position override group: "
                                    + failure.getMessage()
                                    + "; every captured controller restoration was attempted",
                            failure);
                }
            }

            List<PowerLimitedPositionOutput> outs = new ArrayList<>();
            double[] scales = new double[specs.size()];
            double[] biases = new double[specs.size()];
            try {
                for (int i = 0; i < specs.size(); i++) {
                    Spec spec = specs.get(i);
                    outs.add(FtcHardware.motorPosition(motors.get(i), spec.direction));
                    scales[i] = spec.scale;
                    biases[i] = spec.bias;
                }
            } catch (RuntimeException failure) {
                if (before != null) rollbackPendingOverride(failure);
                throw failure;
            }
            if (specs.size() == 1) {
                lastSingleDeviceManagedPositionMotor = motors.get(0);
                return outs.get(0);
            }
            return new GroupedPowerLimitedPositionOutput(
                    outs,
                    scales,
                    biases,
                    childNames(specs));
        }

        private List<DcMotorEx> resolveDistinctMotors(String purpose) {
            List<DcMotorEx> motors = new ArrayList<>(specs.size());
            IdentityHashMap<DcMotorEx, String> seen = new IdentityHashMap<>();
            for (Spec spec : specs) {
                DcMotorEx motor = hw.get(DcMotorEx.class, spec.name);
                String earlierName = seen.put(motor, spec.name);
                if (earlierName != null) {
                    throw new IllegalStateException("FTC " + purpose + " motors '" + earlierName
                            + "' and '" + spec.name + "' resolve to the same DcMotorEx object; "
                            + "each group member must be a distinct configured device");
                }
                motors.add(motor);
            }
            return motors;
        }

        private void rememberVelocityOverride(
                List<DcMotorEx> motors,
                List<String> names,
                List<FtcMotorPidfConfiguration> baselines) {
            requireNoPendingOverride();
            pendingOverrideMotors = new ArrayList<>(motors);
            pendingOverrideNames = new ArrayList<>(names);
            pendingVelocityBaselines = new ArrayList<>(baselines);
        }

        private void rememberPositionOverride(
                List<DcMotorEx> motors,
                List<String> names,
                List<PositionControllerBaseline> baselines) {
            requireNoPendingOverride();
            pendingOverrideMotors = new ArrayList<>(motors);
            pendingOverrideNames = new ArrayList<>(names);
            pendingPositionBaselines = new ArrayList<>(baselines);
        }

        private void requireNoPendingOverride() {
            if (pendingOverrideMotors != null) {
                throw new IllegalStateException(
                        "Internal FTC builder already has a pending controller override");
            }
        }

        private void rollbackPendingOverride(RuntimeException primaryFailure) {
            if (pendingOverrideMotors == null) return;
            List<DcMotorEx> motors = pendingOverrideMotors;
            List<String> names = pendingOverrideNames;
            List<FtcMotorPidfConfiguration> velocity = pendingVelocityBaselines;
            List<PositionControllerBaseline> position = pendingPositionBaselines;
            clearPendingOverride();
            if (velocity != null) {
                restoreVelocityConfigurationsAfterBuildFailure(
                        motors, names, velocity, primaryFailure);
            } else if (position != null) {
                restorePositionConfigurationsAfterBuildFailure(
                        motors, names, position, primaryFailure);
            }
        }

        private void commitPendingOverride() {
            clearPendingOverride();
        }

        private void clearPendingOverride() {
            pendingOverrideMotors = null;
            pendingOverrideNames = null;
            pendingVelocityBaselines = null;
            pendingPositionBaselines = null;
        }

        private ScalarSource deviceManagedPositionMeasurement() {
            if (lastDeviceManagedPositionMotors == null
                    || lastDeviceManagedPositionMotors.size() != specs.size()) {
                throw new IllegalStateException("Device-managed position feedback lost the exact "
                        + "resolved motor identities owned by its output");
            }
            if (specs.size() == 1) {
                return FtcSensors.motorPositionTicks(lastDeviceManagedPositionMotors.get(0));
            }
            List<ScalarSource> sources = new ArrayList<>();
            double[] scales = new double[specs.size()];
            double[] biases = new double[specs.size()];
            for (int i = 0; i < specs.size(); i++) {
                Spec spec = specs.get(i);
                sources.add(FtcSensors.motorPositionTicks(
                        lastDeviceManagedPositionMotors.get(i)));
                scales[i] = spec.scale;
                biases[i] = spec.bias;
            }
            return averageInverseMappedSources(sources, scales, biases);
        }

        private PowerOutput deviceManagedPositionSearchPower() {
            if (lastDeviceManagedPositionMotors == null
                    || lastDeviceManagedPositionMotors.size() != specs.size()) {
                throw new IllegalStateException("Device-managed position search power lost the "
                        + "exact resolved motor identities owned by its output");
            }
            List<PowerOutput> outputs = new ArrayList<>(specs.size());
            for (int index = 0; index < specs.size(); index++) {
                outputs.add(FtcHardware.motorPower(
                        lastDeviceManagedPositionMotors.get(index),
                        specs.get(index).direction));
            }
            if (outputs.size() == 1) return outputs.get(0);
            return new IdentityGroupedPowerOutput(
                    outputs,
                    childNames(specs),
                    "device-managed motor position search power");
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
            Plants.VelocityControlStep,
            Plants.VelocityDirectFeedbackStep,
            Plants.VelocityProfiledFeedbackStep,
            Plants.VelocityDirectPidStep,
            Plants.VelocityProfiledPidStep,
            Plants.OutputPowerPolicyStep<Plant>,
            Plants.OutputPowerAfterVoltageStep<Plant> {
        private static final String VELOCITY_PIDF_OPERATION =
                "FtcActuators.velocityPidf(...)";

        private final MotorBuilder parent;
        private final DeviceManagedVelocityConfig deviceConfig = new DeviceManagedVelocityConfig();
        private VelocityControlKind controlKind;
        private Supplier<ScalarSource> feedback;
        private ScalarRange range;
        private double nativePerPlantUnit;
        private double velocityTolerance;
        private boolean rangeAnswered;
        private boolean mappingAnswered;
        private boolean velocityToleranceAnswered;
        private VelocitySetpointKind setpointKind;
        private double maximumAcceleration;
        private ScalarRegulator customRegulator;
        private double kP;
        private double kI;
        private double kD;
        private boolean feedbackLawAnswered;
        private double integralMinimum;
        private double integralMaximum;
        private boolean integralLimitAnswered;
        private double feedbackMinimum;
        private double feedbackMaximum;
        private boolean feedbackLimitAnswered;
        private VelocityFeedforwardKind feedforwardKind = VelocityFeedforwardKind.NONE;
        private double kS;
        private double kV;
        private double kA;
        private double kG;
        private boolean feedforwardAnswered;
        private ScalarSource supplyVoltage;
        private double referenceVoltage;
        private double minimumVoltage;
        private double maximumVoltageScale;
        private boolean voltageAnswered;
        private double minimumOutputPower;
        private double maximumOutputPower;
        private boolean outputPowerAnswered;
        private FtcDeviceManagedVelocityBinding deviceManagedBinding;

        private MotorVelocityBuilder(MotorBuilder parent) {
            super(parent.lifecycle, "motor velocity Plant");
            this.parent = Objects.requireNonNull(parent, "parent");
        }

        @Override
        public Plants.VelocityBoundsStep<Plants.TargetStep<Plant>> deviceManaged() {
            answerControl(VelocityControlKind.DEVICE_MANAGED_DEFAULTS, "deviceManaged()");
            return coordinateSteps(targetStepView());
        }

        @Override
        protected void requireTargetRateCompatible(String answer) {
            if (setpointKind == VelocitySetpointKind.ACCELERATION_LIMITED) {
                throw new IllegalStateException(answer + " cannot be combined with "
                        + "setpointFromAccelerationLimitedProfile(...); choose one "
                        + "motion-shaping owner");
            }
        }

        @Override
        public MotorDeviceManagedVelocityStep deviceManagedWithOverrides() {
            answerControl(
                    VelocityControlKind.DEVICE_MANAGED_TUNED,
                    "deviceManagedWithOverrides()");
            return this;
        }

        @Override
        public Plants.VelocityBoundsStep<Plants.TargetStep<Plant>> velocityPidf(
                double p, double i, double d, double f) {
            requireMutable("velocityPidf(...)");
            requireControl(VelocityControlKind.DEVICE_MANAGED_TUNED, "velocityPidf(...)");
            if (deviceConfig.velocityPidf != null) {
                throw new IllegalStateException(
                        "velocityPidf(...) has already been answered for this motor velocity Plant");
            }
            deviceConfig.velocityPidf =
                    FtcControllerConfigurationValidation.requireControllerPidf(
                            p, i, d, f, VELOCITY_PIDF_OPERATION);
            return coordinateSteps(targetStepView());
        }

        @Override
        public MotorRegulatedVelocityFeedbackStep regulated() {
            answerControl(VelocityControlKind.REGULATED, "regulated()");
            return this;
        }

        @Override
        public Plants.VelocityBoundsStep<Plants.VelocityControlStep> internalEncoder() {
            requireFeedbackUnanswered("internalEncoder()");
            String selectedName = requireSingleMotorFeedbackName(parent.specs, "velocity");
            feedback = () -> FtcSensors.motorVelocityTicksPerSec(parent.hw, selectedName);
            return coordinateSteps(this);
        }

        @Override
        public Plants.VelocityBoundsStep<Plants.VelocityControlStep> internalEncoder(
                String motorName) {
            requireFeedbackUnanswered("internalEncoder(...)");
            String selectedName = requireSelectedMotorFeedbackName(
                    parent.specs, Objects.requireNonNull(motorName, "motorName"), "velocity");
            feedback = () -> FtcSensors.motorVelocityTicksPerSec(parent.hw, selectedName);
            return coordinateSteps(this);
        }

        @Override
        public Plants.VelocityBoundsStep<Plants.VelocityControlStep> averageInternalEncoders() {
            requireFeedbackUnanswered("averageInternalEncoders()");
            ensureMotorFeedbackAvailable(parent.specs, "velocity");
            feedback = () -> internalVelocityFeedback(parent.hw, parent.specs, null, true);
            return coordinateSteps(this);
        }

        @Override
        public Plants.VelocityBoundsStep<Plants.VelocityControlStep> externalEncoder(String name) {
            return externalEncoder(name, Direction.FORWARD);
        }

        @Override
        public Plants.VelocityBoundsStep<Plants.VelocityControlStep> externalEncoder(
                String name, Direction direction) {
            requireFeedbackUnanswered("externalEncoder(...)");
            String checkedName = requireFeedbackName(name);
            Direction checkedDirection = Objects.requireNonNull(direction, "direction");
            feedback = () -> FtcSensors.continuousMotorPositionTicks(
                    parent.hw, checkedName, checkedDirection).ratePerSecond();
            return coordinateSteps(this);
        }

        @Override
        public Plants.VelocityBoundsStep<Plants.VelocityControlStep> nativeFeedback(
                ScalarSource source) {
            requireFeedbackUnanswered("nativeFeedback(...)");
            ScalarSource checked = Objects.requireNonNull(source, "source");
            feedback = () -> checked;
            return coordinateSteps(this);
        }

        @Override
        public Plants.VelocityDirectFeedbackStep setpointFromAppliedTarget() {
            answerSetpoint(VelocitySetpointKind.DIRECT, "setpointFromAppliedTarget()");
            return this;
        }

        @Override
        public Plants.VelocityProfiledFeedbackStep setpointFromAccelerationLimitedProfile(
                double maximumAcceleration) {
            requireFinitePositive(maximumAcceleration, "maximumAcceleration");
            answerSetpoint(
                    VelocitySetpointKind.ACCELERATION_LIMITED,
                    "setpointFromAccelerationLimitedProfile(...)");
            this.maximumAcceleration = maximumAcceleration;
            return this;
        }

        @Override
        public Plants.OutputPowerPolicyStep<Plant> controlFromCustomRegulator(
                ScalarRegulator regulator) {
            ScalarRegulator checked = Objects.requireNonNull(regulator, "regulator");
            answerSetpoint(VelocitySetpointKind.CUSTOM, "controlFromCustomRegulator(...)");
            customRegulator = checked;
            return this;
        }

        @Override
        public MotorVelocityBuilder feedbackFromPid(double kP) {
            return feedbackFromPid(kP, 0.0, 0.0);
        }

        @Override
        public MotorVelocityBuilder feedbackFromPid(double kP, double kI, double kD) {
            requireStandardSetpoint("feedbackFromPid(...)");
            if (feedbackLawAnswered) {
                throw new IllegalStateException(
                        "feedbackFromPid(...) has already been answered for this motor velocity Plant");
            }
            this.kP = requireFiniteControlValue(kP, "feedbackFromPid(...) kP");
            this.kI = requireFiniteControlValue(kI, "feedbackFromPid(...) kI");
            this.kD = requireFiniteControlValue(kD, "feedbackFromPid(...) kD");
            feedbackLawAnswered = true;
            return this;
        }

        @Override
        public MotorVelocityBuilder feedbackIntegralLimitedTo(double minimum, double maximum) {
            requirePidAnswered("feedbackIntegralLimitedTo(...)");
            if (integralLimitAnswered) {
                throw new IllegalStateException(
                        "feedbackIntegralLimitedTo(...) has already been answered for this motor velocity Plant");
            }
            requireFiniteOrderedControlRange(
                    minimum, maximum, "feedbackIntegralLimitedTo(...)");
            if (minimum > 0.0 || maximum < 0.0) {
                throw new IllegalArgumentException(
                        "feedbackIntegralLimitedTo(...) requires minimum <= 0 <= maximum");
            }
            integralMinimum = minimum;
            integralMaximum = maximum;
            integralLimitAnswered = true;
            return this;
        }

        @Override
        public MotorVelocityBuilder feedbackOutputLimitedTo(double minimum, double maximum) {
            requirePidAnswered("feedbackOutputLimitedTo(...)");
            if (feedbackLimitAnswered) {
                throw new IllegalStateException(
                        "feedbackOutputLimitedTo(...) has already been answered for this motor velocity Plant");
            }
            requireFiniteOrderedControlRange(minimum, maximum, "feedbackOutputLimitedTo(...)");
            feedbackMinimum = minimum;
            feedbackMaximum = maximum;
            feedbackLimitAnswered = true;
            return this;
        }

        @Override
        public Plants.OutputPowerPolicyStep<Plant> feedforwardFromMotion(double kV) {
            return answerFeedforward(
                    VelocityFeedforwardKind.MOTION, 0.0, kV, 0.0, 0.0,
                    "feedforwardFromMotion(kV)", false);
        }

        @Override
        public Plants.OutputPowerPolicyStep<Plant> feedforwardFromMotion(double kS, double kV) {
            return answerFeedforward(
                    VelocityFeedforwardKind.MOTION, kS, kV, 0.0, 0.0,
                    "feedforwardFromMotion(kS, kV)", false);
        }

        @Override
        public Plants.OutputPowerPolicyStep<Plant> feedforwardFromMotion(
                double kS, double kV, double kA) {
            return answerFeedforward(
                    VelocityFeedforwardKind.MOTION, kS, kV, kA, 0.0,
                    "feedforwardFromMotion(kS, kV, kA)", true);
        }

        @Override
        public Plants.OutputPowerPolicyStep<Plant> feedforwardFromLift(double kG) {
            return answerFeedforward(
                    VelocityFeedforwardKind.LIFT, 0.0, 0.0, 0.0, kG,
                    "feedforwardFromLift(kG)", false);
        }

        @Override
        public Plants.OutputPowerPolicyStep<Plant> feedforwardFromLift(
                double kG, double kS, double kV) {
            return answerFeedforward(
                    VelocityFeedforwardKind.LIFT, kS, kV, 0.0, kG,
                    "feedforwardFromLift(kG, kS, kV)", false);
        }

        @Override
        public Plants.OutputPowerPolicyStep<Plant> feedforwardFromLift(
                double kG, double kS, double kV, double kA) {
            return answerFeedforward(
                    VelocityFeedforwardKind.LIFT, kS, kV, kA, kG,
                    "feedforwardFromLift(kG, kS, kV, kA)", true);
        }

        @Override
        public Plants.OutputPowerAfterVoltageStep<Plant> voltageCompensationFrom(
                ScalarSource supplyVoltage,
                double referenceVoltage,
                double minimumVoltage,
                double maximumScale) {
            requireControlOutputPolicyOpen("voltageCompensationFrom(...)");
            if (voltageAnswered) {
                throw new IllegalStateException(
                        "voltageCompensationFrom(...) has already been answered for this motor velocity Plant");
            }
            this.supplyVoltage = Objects.requireNonNull(supplyVoltage, "supplyVoltage");
            requireVoltagePolicy(referenceVoltage, minimumVoltage, maximumScale);
            this.referenceVoltage = referenceVoltage;
            this.minimumVoltage = minimumVoltage;
            maximumVoltageScale = maximumScale;
            voltageAnswered = true;
            return this;
        }

        @Override
        public Plants.TargetStep<Plant> outputPowerLimitedTo(double maximumMagnitude) {
            requireControlOutputPolicyOpen("outputPowerLimitedTo(...)");
            requireFiniteControlValue(maximumMagnitude, "outputPowerLimitedTo(...) maximumMagnitude");
            if (maximumMagnitude < 0.0 || maximumMagnitude > 1.0) {
                throw new IllegalArgumentException(
                        "outputPowerLimitedTo(...) maximumMagnitude must be within [0.0, 1.0]");
            }
            return answerOutputPower(-maximumMagnitude, maximumMagnitude);
        }

        @Override
        public Plants.TargetStep<Plant> outputPowerLimitedTo(double minimum, double maximum) {
            requireControlOutputPolicyOpen("outputPowerLimitedTo(...)");
            requireFiniteOrderedControlRange(minimum, maximum, "outputPowerLimitedTo(...)");
            if (minimum < -1.0 || maximum > 1.0 || minimum > 0.0 || maximum < 0.0) {
                throw new IllegalArgumentException(
                        "outputPowerLimitedTo(...) requires -1 <= minimum <= 0 <= maximum <= 1");
            }
            return answerOutputPower(minimum, maximum);
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
                if (setpointKind == VelocitySetpointKind.ACCELERATION_LIMITED
                        && hasTargetRateConfigured()) {
                    throw new IllegalStateException("A profiled velocity controller cannot also use "
                            + "maxTargetRate(...) or maxTargetRates(...); choose one "
                            + "motion-shaping owner");
                }
                validateRegulatedVelocityRecipe(range, nativePerPlantUnit);
                if (feedback == null || !controlRecipeAnswered()) {
                    throw new IllegalStateException("Regulated motor velocity requires a feedback answer "
                            + "(internalEncoder(), averageInternalEncoders(), externalEncoder(...), or "
                            + "nativeFeedback(...)), a setpoint/control answer, and PID feedback");
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
            if (isDeviceManaged()) {
                deviceManagedBinding = parent.deviceManagedVelocityBinding(
                        deviceConfig, nativePerPlantUnit, velocityTolerance);
                ScalarSource measurement = deviceManagedBinding.sharedNativeMeasurement();
                VelocityOutput output = deviceManagedBinding.output();
                Plants.VelocityBoundsStep<Plants.TargetStep<Plant>> deviceBounds =
                        Plants.fromOutputs().deviceManagedVelocity(output, measurement);
                return replayVelocityCoordinates(deviceBounds);
            } else {
                ScalarSource measurement = feedback.get();
                PowerOutput output = parent.groupedMotorPower();
                Plants.VelocityBoundsStep<Plants.VelocityControlStep> regulatedBounds =
                        Plants.fromOutputs().regulatedVelocity(output, measurement);
                Plants.VelocityControlStep control = replayVelocityCoordinates(regulatedBounds);
                return replayControl(control);
            }
        }

        @Override
        protected Plant finishBuiltPlant(Plant plant) {
            if (!isDeviceManaged()) {
                return plant;
            }
            if (deviceManagedBinding == null) {
                throw new IllegalStateException(
                        "Device-managed velocity Plant lost its FTC motor-group identity");
            }
            Plant result = new FtcDeviceManagedVelocityPlant(
                    plant,
                    deviceManagedBinding,
                    range);
            parent.commitPendingOverride();
            return result;
        }

        @Override
        protected void onBuildFailure(RuntimeException failure) {
            parent.rollbackPendingOverride(failure);
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
                        ? "deviceManagedWithOverrides()" : "regulated()"));
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
                        + "deviceManaged(), deviceManagedWithOverrides(), or regulated()");
            }
            if (controlKind == VelocityControlKind.DEVICE_MANAGED_DEFAULTS) {
                if (deviceConfig.velocityPidf != null) {
                    throw new IllegalStateException("deviceManaged() cannot retain "
                            + "device-managed velocity tuning");
                }
                return;
            }
            if (controlKind == VelocityControlKind.DEVICE_MANAGED_TUNED) {
                if (deviceConfig.velocityPidf == null) {
                    throw new IllegalStateException("deviceManagedWithOverrides() requires velocityPidf(...) "
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

        private <NEXT> Plants.VelocityBoundsStep<NEXT> coordinateSteps(NEXT next) {
            return new VelocityCoordinateSteps<>(next);
        }

        private final class VelocityCoordinateSteps<NEXT>
                implements Plants.VelocityBoundsStep<NEXT>,
                Plants.VelocityMappingStep<NEXT>,
                Plants.VelocityToleranceStep<NEXT> {
            private final NEXT next;

            private VelocityCoordinateSteps(NEXT next) {
                this.next = Objects.requireNonNull(next, "next");
            }

            @Override
            public Plants.VelocityMappingStep<NEXT> bounded(double min, double max) {
                answerBounds(ScalarRange.bounded(min, max), "bounded(...)");
                return this;
            }

            @Override
            public Plants.VelocityMappingStep<NEXT> unbounded() {
                answerBounds(ScalarRange.unbounded(), "unbounded()");
                return this;
            }

            @Override
            public Plants.VelocityToleranceStep<NEXT> nativeUnits() {
                answerMapping(1.0, "nativeUnits()");
                return this;
            }

            @Override
            public Plants.VelocityToleranceStep<NEXT> scaleToNative(
                    double nativeUnitsPerPlantVelocityUnit) {
                answerMapping(nativeUnitsPerPlantVelocityUnit, "scaleToNative(...)");
                return this;
            }

            @Override
            public NEXT velocityTolerance(double tolerance) {
                answerVelocityTolerance(tolerance);
                return next;
            }
        }

        private void answerBounds(ScalarRange answer, String operation) {
            requireMutable(operation);
            requireControlAnswered(operation);
            requireRangeUnanswered();
            range = Objects.requireNonNull(answer, "answer");
            rangeAnswered = true;
        }

        private void answerVelocityTolerance(double tolerance) {
            requireMutable("velocityTolerance(...)");
            if (!mappingAnswered) {
                throw new IllegalStateException(
                        "Choose nativeUnits() or scaleToNative(...) before velocityTolerance(...)");
            }
            if (velocityToleranceAnswered) {
                throw new IllegalStateException(
                        "velocityTolerance(...) has already been answered for this motor velocity Plant");
            }
            requireFiniteTolerance(tolerance, "velocityTolerance");
            velocityTolerance = tolerance;
            velocityToleranceAnswered = true;
        }

        private <NEXT> NEXT replayVelocityCoordinates(Plants.VelocityBoundsStep<NEXT> bounds) {
            Plants.VelocityMappingStep<NEXT> mapping = range.isUnbounded()
                    ? bounds.unbounded()
                    : bounds.bounded(range.minValue, range.maxValue);
            Plants.VelocityToleranceStep<NEXT> tolerance = nativePerPlantUnit == 1.0
                    ? mapping.nativeUnits()
                    : mapping.scaleToNative(nativePerPlantUnit);
            return tolerance.velocityTolerance(velocityTolerance);
        }

        private Plants.TargetStep<Plant> replayControl(Plants.VelocityControlStep control) {
            if (setpointKind == VelocitySetpointKind.CUSTOM) {
                return replayOutputPolicy(control.controlFromCustomRegulator(customRegulator));
            }
            if (setpointKind == VelocitySetpointKind.DIRECT) {
                Plants.VelocityDirectFeedbackStep feedbackStep =
                        control.setpointFromAppliedTarget();
                Plants.VelocityDirectPidStep pid = kI == 0.0 && kD == 0.0
                        ? feedbackStep.feedbackFromPid(kP)
                        : feedbackStep.feedbackFromPid(kP, kI, kD);
                if (integralLimitAnswered) {
                    pid = pid.feedbackIntegralLimitedTo(integralMinimum, integralMaximum);
                }
                if (feedbackLimitAnswered) {
                    pid = pid.feedbackOutputLimitedTo(feedbackMinimum, feedbackMaximum);
                }
                Plants.OutputPowerPolicyStep<Plant> output = pid;
                if (feedforwardAnswered) {
                    output = feedforwardKind == VelocityFeedforwardKind.MOTION
                            ? (kS == 0.0
                            ? pid.feedforwardFromMotion(kV)
                            : pid.feedforwardFromMotion(kS, kV))
                            : (kS == 0.0 && kV == 0.0
                            ? pid.feedforwardFromLift(kG)
                            : pid.feedforwardFromLift(kG, kS, kV));
                }
                return replayOutputPolicy(output);
            }

            Plants.VelocityProfiledFeedbackStep feedbackStep =
                    control.setpointFromAccelerationLimitedProfile(maximumAcceleration);
            Plants.VelocityProfiledPidStep pid = kI == 0.0 && kD == 0.0
                    ? feedbackStep.feedbackFromPid(kP)
                    : feedbackStep.feedbackFromPid(kP, kI, kD);
            if (integralLimitAnswered) {
                pid = pid.feedbackIntegralLimitedTo(integralMinimum, integralMaximum);
            }
            if (feedbackLimitAnswered) {
                pid = pid.feedbackOutputLimitedTo(feedbackMinimum, feedbackMaximum);
            }
            Plants.OutputPowerPolicyStep<Plant> output = pid;
            if (feedforwardAnswered) {
                if (feedforwardKind == VelocityFeedforwardKind.MOTION) {
                    output = kS == 0.0 && kA == 0.0
                            ? pid.feedforwardFromMotion(kV)
                            : pid.feedforwardFromMotion(kS, kV, kA);
                } else {
                    output = kS == 0.0 && kV == 0.0 && kA == 0.0
                            ? pid.feedforwardFromLift(kG)
                            : pid.feedforwardFromLift(kG, kS, kV, kA);
                }
            }
            return replayOutputPolicy(output);
        }

        private Plants.TargetStep<Plant> replayOutputPolicy(
                Plants.OutputPowerPolicyStep<Plant> output) {
            if (voltageAnswered) {
                Plants.OutputPowerAfterVoltageStep<Plant> afterVoltage =
                        output.voltageCompensationFrom(
                                supplyVoltage,
                                referenceVoltage,
                                minimumVoltage,
                                maximumVoltageScale);
                if (!outputPowerAnswered) {
                    return afterVoltage;
                }
                return isSymmetricOutputPower()
                        ? afterVoltage.outputPowerLimitedTo(maximumOutputPower)
                        : afterVoltage.outputPowerLimitedTo(
                        minimumOutputPower, maximumOutputPower);
            }
            if (!outputPowerAnswered) {
                return output;
            }
            return isSymmetricOutputPower()
                    ? output.outputPowerLimitedTo(maximumOutputPower)
                    : output.outputPowerLimitedTo(minimumOutputPower, maximumOutputPower);
        }

        private void answerSetpoint(VelocitySetpointKind answer, String operation) {
            requireControlReady(operation);
            if (setpointKind != null) {
                throw new IllegalStateException(
                        "Velocity setpoint/control ownership has already been answered");
            }
            setpointKind = answer;
        }

        private MotorVelocityBuilder answerFeedforward(
                VelocityFeedforwardKind kind,
                double kS,
                double kV,
                double kA,
                double kG,
                String operation,
                boolean accelerationRequired) {
            requirePidAnswered(operation);
            if (feedforwardAnswered) {
                throw new IllegalStateException(
                        "Feedforward has already been answered for this motor velocity Plant");
            }
            if (accelerationRequired
                    && setpointKind != VelocitySetpointKind.ACCELERATION_LIMITED) {
                throw new IllegalStateException(operation
                        + " requires setpointFromAccelerationLimitedProfile(...)");
            }
            this.kS = requireFiniteControlValue(kS, operation + " kS");
            this.kV = requireFiniteControlValue(kV, operation + " kV");
            this.kA = requireFiniteControlValue(kA, operation + " kA");
            this.kG = requireFiniteControlValue(kG, operation + " kG");
            feedforwardKind = kind;
            feedforwardAnswered = true;
            return this;
        }

        private void requireControlReady(String operation) {
            requireMutable(operation);
            requireControl(VelocityControlKind.REGULATED, operation);
            if (feedback == null || !velocityToleranceAnswered) {
                throw new IllegalStateException(operation
                        + " requires regulated feedback, bounds, units, and velocityTolerance(...) first");
            }
        }

        private void requireStandardSetpoint(String operation) {
            requireMutable(operation);
            if (setpointKind != VelocitySetpointKind.DIRECT
                    && setpointKind != VelocitySetpointKind.ACCELERATION_LIMITED) {
                throw new IllegalStateException(operation + " requires a standard velocity setpoint");
            }
        }

        private void requirePidAnswered(String operation) {
            requireMutable(operation);
            if (!feedbackLawAnswered) {
                throw new IllegalStateException(operation + " requires feedbackFromPid(...) first");
            }
            if (feedforwardAnswered || voltageAnswered || outputPowerAnswered) {
                throw new IllegalStateException(operation + " cannot change PID/feedforward after "
                        + "the control recipe advanced to feedforward, voltage, or output policy");
            }
        }

        private void requireControlOutputPolicyOpen(String operation) {
            requireMutable(operation);
            if (!controlRecipeAnswered()) {
                throw new IllegalStateException(operation
                        + " requires feedbackFromPid(...) or controlFromCustomRegulator(...) first");
            }
            if (outputPowerAnswered) {
                throw new IllegalStateException(
                        "outputPowerLimitedTo(...) has already closed the output policy");
            }
        }

        private Plants.TargetStep<Plant> answerOutputPower(double minimum, double maximum) {
            minimumOutputPower = minimum;
            maximumOutputPower = maximum;
            outputPowerAnswered = true;
            return this;
        }

        private boolean controlRecipeAnswered() {
            return setpointKind == VelocitySetpointKind.CUSTOM
                    ? customRegulator != null
                    : (setpointKind == VelocitySetpointKind.DIRECT
                    || setpointKind == VelocitySetpointKind.ACCELERATION_LIMITED)
                    && feedbackLawAnswered;
        }

        private boolean isSymmetricOutputPower() {
            return minimumOutputPower == -maximumOutputPower;
        }
    }

    private static final class DeviceManagedPositionConfig {
        private Double outerPositionP;
        private boolean outerPositionPAnswered;
        private double[] innerVelocityPidf;
        private boolean innerVelocityPidfAnswered;
        private Integer devicePositionToleranceTicks;
        private boolean devicePositionToleranceTicksAnswered;
        private boolean tuningClosed;

        private boolean hasAnyOverride() {
            return outerPositionPAnswered
                    || innerVelocityPidfAnswered
                    || devicePositionToleranceTicksAnswered;
        }
    }

    private enum PositionControlKind {
        DEVICE_MANAGED_DEFAULTS,
        DEVICE_MANAGED_TUNED,
        REGULATED
    }

    private enum PositionSetpointKind {
        DIRECT,
        TRAPEZOIDAL,
        CUSTOM
    }

    private enum PositionFeedforwardKind {
        NONE,
        MOTION,
        LIFT,
        ARM
    }

    private enum PositionMappingKind {NATIVE, SCALE, ENDPOINTS}

    private enum PositionReferenceKind {STATIC, ASSUME_CURRENT, NEEDS_REFERENCE}

    private abstract static class BasePositionBuilder extends FtcTargetBuilder<PositionPlant>
            implements Plants.PositionControlStep,
            Plants.PositionDirectFeedbackStep,
            Plants.PositionProfiledFeedbackStep,
            Plants.PositionDirectPidStep,
            Plants.PositionProfiledPidStep,
            Plants.OutputPowerPolicyStep<PositionPlant>,
            Plants.OutputPowerAfterVoltageStep<PositionPlant> {
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
        private PositionSetpointKind setpointKind;
        private double maximumVelocity;
        private double maximumAcceleration;
        private ScalarRegulator customRegulator;
        private double kP;
        private double kI;
        private double kD;
        private boolean feedbackLawAnswered;
        private double integralMinimum;
        private double integralMaximum;
        private boolean integralLimitAnswered;
        private double feedbackMinimum;
        private double feedbackMaximum;
        private boolean feedbackLimitAnswered;
        private PositionFeedforwardKind feedforwardKind = PositionFeedforwardKind.NONE;
        private double kS;
        private double kV;
        private double kA;
        private double kG;
        private double plantPositionAtMaximumGravity;
        private double radiansPerPlantUnit;
        private boolean feedforwardAnswered;
        private ScalarSource supplyVoltage;
        private double referenceVoltage;
        private double minimumVoltage;
        private double maximumVoltageScale;
        private boolean voltageAnswered;
        private double minimumOutputPower;
        private double maximumOutputPower;
        private boolean outputPowerAnswered;

        BasePositionBuilder(RecipeLifecycle lifecycle, String plantName) {
            super(lifecycle, plantName);
        }

        protected final <NEXT> Plants.PositionPeriodicityStep<
                Plants.FeedbackPositionBoundsStep<NEXT>> coordinateSteps(NEXT next) {
            return new PositionCoordinateSteps<>(next);
        }

        @Override
        protected final void requireTargetRateCompatible(String answer) {
            if (setpointKind == PositionSetpointKind.TRAPEZOIDAL) {
                throw new IllegalStateException(answer + " cannot be combined with "
                        + "setpointFromTrapezoidalProfile(...); choose one "
                        + "motion-shaping owner");
            }
        }

        private final class PositionCoordinateSteps<NEXT>
                implements Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<NEXT>>,
                Plants.FeedbackPositionBoundsStep<NEXT>,
                Plants.FeedbackBoundedPositionMappingStep<NEXT>,
                Plants.FeedbackUnboundedPositionMappingStep<NEXT>,
                Plants.PositionCoordinateReferenceStep<NEXT>,
                Plants.PositionToleranceStep<NEXT> {
            private final NEXT next;

            private PositionCoordinateSteps(NEXT next) {
                this.next = Objects.requireNonNull(next, "next");
            }

            @Override
            public Plants.FeedbackPositionBoundsStep<NEXT> nonPeriodic() {
                answerPeriodicity(false, Double.NaN, "nonPeriodic()");
                return this;
            }

            @Override
            public Plants.FeedbackPositionBoundsStep<NEXT> periodic(double period) {
                requireFinitePositive(period, "period");
                answerPeriodicity(true, period, "periodic(...)");
                return this;
            }

            @Override
            public Plants.FeedbackBoundedPositionMappingStep<NEXT> bounded(
                    double min, double max) {
                answerPositionBounds(ScalarRange.bounded(min, max), true, min, max,
                        "bounded(...)");
                return this;
            }

            @Override
            public Plants.FeedbackUnboundedPositionMappingStep<NEXT> unbounded() {
                answerPositionBounds(ScalarRange.unbounded(), false, Double.NaN, Double.NaN,
                        "unbounded()");
                return this;
            }

            @Override
            public Plants.PositionCoordinateReferenceStep<NEXT> nativeUnits() {
                answerScale(PositionMappingKind.NATIVE, 1.0, "nativeUnits()");
                return this;
            }

            @Override
            public Plants.PositionCoordinateReferenceStep<NEXT> scaleToNative(
                    double nativeUnitsPerPlantUnit) {
                answerScale(PositionMappingKind.SCALE, nativeUnitsPerPlantUnit,
                        "scaleToNative(...)");
                return this;
            }

            @Override
            public Plants.PositionToleranceStep<NEXT> rangeMapsToNative(
                    double nativeAtPlantMin, double nativeAtPlantMax) {
                answerEndpointMapping(nativeAtPlantMin, nativeAtPlantMax);
                return this;
            }

            @Override
            public Plants.PositionToleranceStep<NEXT> alreadyReferenced() {
                answerStaticReference(0.0, 0.0, "alreadyReferenced()");
                return this;
            }

            @Override
            public Plants.PositionToleranceStep<NEXT> plantPositionMapsToNative(
                    double plantPosition, double nativePosition) {
                answerStaticReference(
                        plantPosition, nativePosition, "plantPositionMapsToNative(...)");
                return this;
            }

            @Override
            public Plants.PositionToleranceStep<NEXT> assumeCurrentPositionIs(
                    double plantPosition) {
                answerAssumeCurrentPosition(plantPosition);
                return this;
            }

            @Override
            public Plants.PositionToleranceStep<NEXT> needsReference(String reason) {
                answerNeedsReference(reason);
                return this;
            }

            @Override
            public NEXT positionTolerance(double tolerance) {
                answerPositionTolerance(tolerance);
                return next;
            }
        }

        private void answerPositionBounds(ScalarRange answer,
                                          boolean bounded,
                                          double min,
                                          double max,
                                          String operation) {
            requireMutable(operation);
            requirePeriodicityAnswered(operation);
            requireRangeUnanswered();
            range = Objects.requireNonNull(answer, "answer");
            plantMin = min;
            plantMax = max;
            this.bounded = bounded;
            rangeAnswered = true;
        }

        private void answerEndpointMapping(double nativeAtPlantMin,
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
        }

        private void answerAssumeCurrentPosition(double plantPosition) {
            requireMutable("assumeCurrentPositionIs(...)");
            double checkedPlantPosition = requireFinitePosition(
                    plantPosition,
                    "FtcActuators.assumeCurrentPositionIs(...)",
                    "plantPosition",
                    "plant units");
            requireReferencePending("assumeCurrentPositionIs(...)");
            assumePlantPosition = checkedPlantPosition;
            referenceKind = PositionReferenceKind.ASSUME_CURRENT;
        }

        private void answerNeedsReference(String reason) {
            requireMutable("needsReference(...)");
            requireReferencePending("needsReference(...)");
            if (reason == null || reason.trim().isEmpty()) {
                throw new IllegalArgumentException("needsReference(...) reason must be nonblank");
            }
            referenceReason = reason.trim();
            referenceKind = PositionReferenceKind.NEEDS_REFERENCE;
        }

        private void answerPositionTolerance(double tolerance) {
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
        }

        @Override
        public Plants.PositionDirectFeedbackStep setpointFromAppliedTarget() {
            answerSetpoint(PositionSetpointKind.DIRECT, "setpointFromAppliedTarget()");
            return this;
        }

        @Override
        public Plants.PositionProfiledFeedbackStep setpointFromTrapezoidalProfile(
                double maximumVelocity, double maximumAcceleration) {
            requireFinitePositive(maximumVelocity, "maximumVelocity");
            requireFinitePositive(maximumAcceleration, "maximumAcceleration");
            answerSetpoint(
                    PositionSetpointKind.TRAPEZOIDAL,
                    "setpointFromTrapezoidalProfile(...)");
            this.maximumVelocity = maximumVelocity;
            this.maximumAcceleration = maximumAcceleration;
            return this;
        }

        @Override
        public Plants.OutputPowerPolicyStep<PositionPlant> controlFromCustomRegulator(
                ScalarRegulator regulator) {
            ScalarRegulator checked = Objects.requireNonNull(regulator, "regulator");
            answerSetpoint(PositionSetpointKind.CUSTOM, "controlFromCustomRegulator(...)");
            customRegulator = checked;
            return this;
        }

        @Override
        public BasePositionBuilder feedbackFromPid(double kP) {
            return feedbackFromPid(kP, 0.0, 0.0);
        }

        @Override
        public BasePositionBuilder feedbackFromPid(double kP, double kI, double kD) {
            requireStandardSetpoint("feedbackFromPid(...)");
            if (feedbackLawAnswered) {
                throw new IllegalStateException(
                        "feedbackFromPid(...) has already been answered for this position Plant");
            }
            this.kP = requireFiniteControlValue(kP, "feedbackFromPid(...) kP");
            this.kI = requireFiniteControlValue(kI, "feedbackFromPid(...) kI");
            this.kD = requireFiniteControlValue(kD, "feedbackFromPid(...) kD");
            feedbackLawAnswered = true;
            return this;
        }

        @Override
        public BasePositionBuilder feedbackIntegralLimitedTo(double minimum, double maximum) {
            requirePidAnswered("feedbackIntegralLimitedTo(...)");
            if (integralLimitAnswered) {
                throw new IllegalStateException(
                        "feedbackIntegralLimitedTo(...) has already been answered for this position Plant");
            }
            requireFiniteOrderedControlRange(
                    minimum, maximum, "feedbackIntegralLimitedTo(...)");
            if (minimum > 0.0 || maximum < 0.0) {
                throw new IllegalArgumentException(
                        "feedbackIntegralLimitedTo(...) requires minimum <= 0 <= maximum");
            }
            integralMinimum = minimum;
            integralMaximum = maximum;
            integralLimitAnswered = true;
            return this;
        }

        @Override
        public BasePositionBuilder feedbackOutputLimitedTo(double minimum, double maximum) {
            requirePidAnswered("feedbackOutputLimitedTo(...)");
            if (feedbackLimitAnswered) {
                throw new IllegalStateException(
                        "feedbackOutputLimitedTo(...) has already been answered for this position Plant");
            }
            requireFiniteOrderedControlRange(minimum, maximum, "feedbackOutputLimitedTo(...)");
            feedbackMinimum = minimum;
            feedbackMaximum = maximum;
            feedbackLimitAnswered = true;
            return this;
        }

        @Override
        public Plants.OutputPowerPolicyStep<PositionPlant> feedforwardFromMotion(double kV) {
            return answerPositionFeedforward(
                    PositionFeedforwardKind.MOTION,
                    0.0, kV, 0.0, 0.0, 0.0, 0.0,
                    "feedforwardFromMotion(kV)", true);
        }

        @Override
        public Plants.OutputPowerPolicyStep<PositionPlant> feedforwardFromMotion(
                double kS, double kV, double kA) {
            return answerPositionFeedforward(
                    PositionFeedforwardKind.MOTION,
                    kS, kV, kA, 0.0, 0.0, 0.0,
                    "feedforwardFromMotion(kS, kV, kA)", true);
        }

        @Override
        public Plants.OutputPowerPolicyStep<PositionPlant> feedforwardFromLift(double kG) {
            return answerPositionFeedforward(
                    PositionFeedforwardKind.LIFT,
                    0.0, 0.0, 0.0, kG, 0.0, 0.0,
                    "feedforwardFromLift(kG)", false);
        }

        @Override
        public Plants.OutputPowerPolicyStep<PositionPlant> feedforwardFromLift(
                double kG, double kS, double kV, double kA) {
            return answerPositionFeedforward(
                    PositionFeedforwardKind.LIFT,
                    kS, kV, kA, kG, 0.0, 0.0,
                    "feedforwardFromLift(kG, kS, kV, kA)", true);
        }

        @Override
        public Plants.OutputPowerPolicyStep<PositionPlant> feedforwardFromArm(
                double kG,
                double plantPositionAtMaximumGravity,
                double radiansPerPlantUnit) {
            return answerPositionFeedforward(
                    PositionFeedforwardKind.ARM,
                    0.0, 0.0, 0.0, kG,
                    plantPositionAtMaximumGravity, radiansPerPlantUnit,
                    "feedforwardFromArm(kG, position, radiansPerPlantUnit)", false);
        }

        @Override
        public Plants.OutputPowerPolicyStep<PositionPlant> feedforwardFromArm(
                double kG,
                double plantPositionAtMaximumGravity,
                double radiansPerPlantUnit,
                double kS,
                double kV,
                double kA) {
            return answerPositionFeedforward(
                    PositionFeedforwardKind.ARM,
                    kS, kV, kA, kG,
                    plantPositionAtMaximumGravity, radiansPerPlantUnit,
                    "feedforwardFromArm(kG, position, radiansPerPlantUnit, kS, kV, kA)", true);
        }

        @Override
        public Plants.OutputPowerAfterVoltageStep<PositionPlant> voltageCompensationFrom(
                ScalarSource supplyVoltage,
                double referenceVoltage,
                double minimumVoltage,
                double maximumScale) {
            requireControlOutputPolicyOpen("voltageCompensationFrom(...)");
            if (voltageAnswered) {
                throw new IllegalStateException(
                        "voltageCompensationFrom(...) has already been answered for this position Plant");
            }
            this.supplyVoltage = Objects.requireNonNull(supplyVoltage, "supplyVoltage");
            requireVoltagePolicy(referenceVoltage, minimumVoltage, maximumScale);
            this.referenceVoltage = referenceVoltage;
            this.minimumVoltage = minimumVoltage;
            maximumVoltageScale = maximumScale;
            voltageAnswered = true;
            return this;
        }

        @Override
        public Plants.TargetStep<PositionPlant> outputPowerLimitedTo(double maximumMagnitude) {
            requireControlOutputPolicyOpen("outputPowerLimitedTo(...)");
            requireFiniteControlValue(maximumMagnitude, "outputPowerLimitedTo(...) maximumMagnitude");
            if (maximumMagnitude < 0.0 || maximumMagnitude > 1.0) {
                throw new IllegalArgumentException(
                        "outputPowerLimitedTo(...) maximumMagnitude must be within [0.0, 1.0]");
            }
            return answerOutputPower(-maximumMagnitude, maximumMagnitude);
        }

        @Override
        public Plants.TargetStep<PositionPlant> outputPowerLimitedTo(
                double minimum, double maximum) {
            requireControlOutputPolicyOpen("outputPowerLimitedTo(...)");
            requireFiniteOrderedControlRange(minimum, maximum, "outputPowerLimitedTo(...)");
            if (minimum < -1.0 || maximum > 1.0 || minimum > 0.0 || maximum < 0.0) {
                throw new IllegalArgumentException(
                        "outputPowerLimitedTo(...) requires -1 <= minimum <= 0 <= maximum <= 1");
            }
            return answerOutputPower(minimum, maximum);
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
            if (setpointKind == PositionSetpointKind.TRAPEZOIDAL
                    && hasTargetRateConfigured()) {
                throw new IllegalStateException("A profiled position controller cannot also use "
                        + "maxTargetRate(...) or maxTargetRates(...); choose one "
                        + "motion-shaping owner");
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
            return createSharedPositionTargetStep();
        }

        protected final <NEXT> NEXT replayPositionCoordinates(
                Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<NEXT>> periodicity) {
            Plants.FeedbackPositionBoundsStep<NEXT> bounds = periodic
                    ? periodicity.periodic(period)
                    : periodicity.nonPeriodic();

            Plants.PositionToleranceStep<NEXT> tolerance;
            if (bounded) {
                Plants.FeedbackBoundedPositionMappingStep<NEXT> mapping =
                        bounds.bounded(plantMin, plantMax);
                if (mappingKind == PositionMappingKind.ENDPOINTS) {
                    tolerance = mapping.rangeMapsToNative(
                            nativeAtPlantMin, nativeAtPlantMax);
                } else {
                    Plants.PositionCoordinateReferenceStep<NEXT> reference =
                            mappingKind == PositionMappingKind.NATIVE
                                    ? mapping.nativeUnits()
                                    : mapping.scaleToNative(nativePerPlantUnit);
                    tolerance = applyReference(reference);
                }
            } else {
                Plants.FeedbackUnboundedPositionMappingStep<NEXT> mapping = bounds.unbounded();
                Plants.PositionCoordinateReferenceStep<NEXT> reference =
                        mappingKind == PositionMappingKind.NATIVE
                                ? mapping.nativeUnits()
                                : mapping.scaleToNative(nativePerPlantUnit);
                tolerance = applyReference(reference);
            }
            return tolerance.positionTolerance(positionTolerance);
        }

        protected abstract void validatePositionSourceRecipe();

        protected abstract Plants.TargetStep<PositionPlant> createSharedPositionTargetStep();

        protected abstract boolean isRegulatedControlPath();

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

        private <NEXT> Plants.PositionToleranceStep<NEXT> applyReference(
                Plants.PositionCoordinateReferenceStep<NEXT> reference) {
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

        private void answerStaticReference(
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

        protected final boolean positionControlRecipeAnswered() {
            return setpointKind == PositionSetpointKind.CUSTOM
                    ? customRegulator != null
                    : (setpointKind == PositionSetpointKind.DIRECT
                    || setpointKind == PositionSetpointKind.TRAPEZOIDAL)
                    && feedbackLawAnswered;
        }

        protected final boolean positionToleranceWasAnswered() {
            return positionToleranceAnswered;
        }

        protected final Plants.TargetStep<PositionPlant> replayPositionControl(
                Plants.PositionControlStep control) {
            if (setpointKind == PositionSetpointKind.CUSTOM) {
                return replayOutputPolicy(control.controlFromCustomRegulator(customRegulator));
            }
            if (setpointKind == PositionSetpointKind.DIRECT) {
                Plants.PositionDirectFeedbackStep feedbackStep =
                        control.setpointFromAppliedTarget();
                Plants.PositionDirectPidStep pid = kI == 0.0 && kD == 0.0
                        ? feedbackStep.feedbackFromPid(kP)
                        : feedbackStep.feedbackFromPid(kP, kI, kD);
                if (integralLimitAnswered) {
                    pid = pid.feedbackIntegralLimitedTo(integralMinimum, integralMaximum);
                }
                if (feedbackLimitAnswered) {
                    pid = pid.feedbackOutputLimitedTo(feedbackMinimum, feedbackMaximum);
                }
                Plants.OutputPowerPolicyStep<PositionPlant> output = pid;
                if (feedforwardAnswered) {
                    output = feedforwardKind == PositionFeedforwardKind.LIFT
                            ? pid.feedforwardFromLift(kG)
                            : pid.feedforwardFromArm(
                            kG, plantPositionAtMaximumGravity, radiansPerPlantUnit);
                }
                return replayOutputPolicy(output);
            }

            Plants.PositionProfiledFeedbackStep feedbackStep =
                    control.setpointFromTrapezoidalProfile(
                            maximumVelocity, maximumAcceleration);
            Plants.PositionProfiledPidStep pid = kI == 0.0 && kD == 0.0
                    ? feedbackStep.feedbackFromPid(kP)
                    : feedbackStep.feedbackFromPid(kP, kI, kD);
            if (integralLimitAnswered) {
                pid = pid.feedbackIntegralLimitedTo(integralMinimum, integralMaximum);
            }
            if (feedbackLimitAnswered) {
                pid = pid.feedbackOutputLimitedTo(feedbackMinimum, feedbackMaximum);
            }
            Plants.OutputPowerPolicyStep<PositionPlant> output = pid;
            if (feedforwardAnswered) {
                switch (feedforwardKind) {
                    case MOTION:
                        output = kS == 0.0 && kA == 0.0
                                ? pid.feedforwardFromMotion(kV)
                                : pid.feedforwardFromMotion(kS, kV, kA);
                        break;
                    case LIFT:
                        output = kS == 0.0 && kV == 0.0 && kA == 0.0
                                ? pid.feedforwardFromLift(kG)
                                : pid.feedforwardFromLift(kG, kS, kV, kA);
                        break;
                    case ARM:
                        output = kS == 0.0 && kV == 0.0 && kA == 0.0
                                ? pid.feedforwardFromArm(
                                kG, plantPositionAtMaximumGravity, radiansPerPlantUnit)
                                : pid.feedforwardFromArm(
                                kG,
                                plantPositionAtMaximumGravity,
                                radiansPerPlantUnit,
                                kS,
                                kV,
                                kA);
                        break;
                    default:
                        throw new IllegalStateException(
                                "Unsupported position feedforward: " + feedforwardKind);
                }
            }
            return replayOutputPolicy(output);
        }

        private Plants.TargetStep<PositionPlant> replayOutputPolicy(
                Plants.OutputPowerPolicyStep<PositionPlant> output) {
            if (voltageAnswered) {
                Plants.OutputPowerAfterVoltageStep<PositionPlant> afterVoltage =
                        output.voltageCompensationFrom(
                                supplyVoltage,
                                referenceVoltage,
                                minimumVoltage,
                                maximumVoltageScale);
                if (!outputPowerAnswered) {
                    return afterVoltage;
                }
                return isSymmetricOutputPower()
                        ? afterVoltage.outputPowerLimitedTo(maximumOutputPower)
                        : afterVoltage.outputPowerLimitedTo(
                        minimumOutputPower, maximumOutputPower);
            }
            if (!outputPowerAnswered) {
                return output;
            }
            return isSymmetricOutputPower()
                    ? output.outputPowerLimitedTo(maximumOutputPower)
                    : output.outputPowerLimitedTo(minimumOutputPower, maximumOutputPower);
        }

        private void answerSetpoint(PositionSetpointKind answer, String operation) {
            requireMutable(operation);
            if (!isRegulatedControlPath() || !positionToleranceAnswered) {
                throw new IllegalStateException(operation
                        + " requires regulated feedback, position coordinates, and positionTolerance(...) first");
            }
            if (setpointKind != null) {
                throw new IllegalStateException(
                        "Position setpoint/control ownership has already been answered");
            }
            setpointKind = answer;
        }

        private BasePositionBuilder answerPositionFeedforward(
                PositionFeedforwardKind kind,
                double kS,
                double kV,
                double kA,
                double kG,
                double positionAtMaximumGravity,
                double radiansPerPlantUnit,
                String operation,
                boolean profileRequired) {
            requirePidAnswered(operation);
            if (feedforwardAnswered) {
                throw new IllegalStateException(
                        "Feedforward has already been answered for this position Plant");
            }
            if (profileRequired && setpointKind != PositionSetpointKind.TRAPEZOIDAL) {
                throw new IllegalStateException(operation
                        + " requires setpointFromTrapezoidalProfile(...)");
            }
            this.kS = requireFiniteControlValue(kS, operation + " kS");
            this.kV = requireFiniteControlValue(kV, operation + " kV");
            this.kA = requireFiniteControlValue(kA, operation + " kA");
            this.kG = requireFiniteControlValue(kG, operation + " kG");
            if (kind == PositionFeedforwardKind.ARM) {
                plantPositionAtMaximumGravity = requireFiniteControlValue(
                        positionAtMaximumGravity,
                        operation + " plantPositionAtMaximumGravity");
                this.radiansPerPlantUnit = requireFiniteNonZero(
                        radiansPerPlantUnit, "radiansPerPlantUnit");
            }
            feedforwardKind = kind;
            feedforwardAnswered = true;
            return this;
        }

        private void requireStandardSetpoint(String operation) {
            requireMutable(operation);
            if (setpointKind != PositionSetpointKind.DIRECT
                    && setpointKind != PositionSetpointKind.TRAPEZOIDAL) {
                throw new IllegalStateException(operation + " requires a standard position setpoint");
            }
        }

        private void requirePidAnswered(String operation) {
            requireMutable(operation);
            if (!feedbackLawAnswered) {
                throw new IllegalStateException(operation + " requires feedbackFromPid(...) first");
            }
            if (feedforwardAnswered || voltageAnswered || outputPowerAnswered) {
                throw new IllegalStateException(operation + " cannot change PID/feedforward after "
                        + "the control recipe advanced to feedforward, voltage, or output policy");
            }
        }

        private void requireControlOutputPolicyOpen(String operation) {
            requireMutable(operation);
            if (!positionControlRecipeAnswered()) {
                throw new IllegalStateException(operation
                        + " requires feedbackFromPid(...) or controlFromCustomRegulator(...) first");
            }
            if (outputPowerAnswered) {
                throw new IllegalStateException(
                        "outputPowerLimitedTo(...) has already closed the output policy");
            }
        }

        private Plants.TargetStep<PositionPlant> answerOutputPower(
                double minimum, double maximum) {
            minimumOutputPower = minimum;
            maximumOutputPower = maximum;
            outputPowerAnswered = true;
            return this;
        }

        private boolean isSymmetricOutputPower() {
            return minimumOutputPower == -maximumOutputPower;
        }
    }

    private static final class MotorPositionBuilder extends BasePositionBuilder
            implements MotorPositionControlStep,
            MotorDeviceManagedPositionStep,
            MotorRegulatedPositionFeedbackStep {
        private static final String OUTPUT_POWER_OPERATION =
                "FtcActuators.outputPowerLimitedTo(...)";
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
        private double deviceMaximumOutputPower = 1.0;
        private boolean deviceMaximumOutputPowerAnswered;

        private MotorPositionBuilder(MotorBuilder parent) {
            super(parent.lifecycle, "motor position Plant");
            this.parent = Objects.requireNonNull(parent, "parent");
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>> deviceManaged() {
            answerControl(PositionControlKind.DEVICE_MANAGED_DEFAULTS, "deviceManaged()");
            return coordinateSteps(symmetricOutputPowerPolicyView(
                    this::answerDeviceManagedOutputPower));
        }

        @Override
        public MotorDeviceManagedPositionStep deviceManagedWithOverrides() {
            answerControl(
                    PositionControlKind.DEVICE_MANAGED_TUNED,
                    "deviceManagedWithOverrides()");
            return this;
        }

        @Override
        public Plants.TargetStep<PositionPlant> outputPowerLimitedTo(double maximumMagnitude) {
            if (!isDeviceManaged()) {
                return super.outputPowerLimitedTo(maximumMagnitude);
            }
            answerDeviceManagedOutputPower(maximumMagnitude);
            return targetStepView();
        }

        private void answerDeviceManagedOutputPower(double maximumMagnitude) {
            requireMutable("outputPowerLimitedTo(...)");
            if (deviceMaximumOutputPowerAnswered) {
                throw new IllegalStateException("outputPowerLimitedTo(...) has already been "
                        + "answered for this device-managed motor position Plant");
            }
            if (!positionCoordinatesComplete()) {
                throw new IllegalStateException("outputPowerLimitedTo(...) requires "
                        + "positionTolerance(...) first");
            }
            deviceMaximumOutputPower =
                    FtcControllerConfigurationValidation
                            .requireRunToPositionMaximumOutputPowerMagnitude(
                            maximumMagnitude, OUTPUT_POWER_OPERATION);
            deviceMaximumOutputPowerAnswered = true;
        }

        @Override
        public MotorDeviceManagedPositionStep outerPositionP(double outerPositionP) {
            requireOpenDeviceManagedOverrides("outerPositionP(...)");
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
            requireOpenDeviceManagedOverrides("innerVelocityPidf(...)");
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
            requireOpenDeviceManagedOverrides("devicePositionToleranceTicks(...)");
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
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>> doneOverrides() {
            requireOpenDeviceManagedOverrides("doneOverrides()");
            if (!deviceConfig.hasAnyOverride()) {
                throw new IllegalStateException("deviceManagedWithOverrides() requires at least "
                        + "one controller override before doneOverrides(); use deviceManaged() "
                        + "when no override is needed");
            }
            deviceConfig.tuningClosed = true;
            return coordinateSteps(symmetricOutputPowerPolicyView(
                    this::answerDeviceManagedOutputPower));
        }

        @Override
        public MotorRegulatedPositionFeedbackStep regulated() {
            answerControl(PositionControlKind.REGULATED, "regulated()");
            return this;
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> internalEncoder() {
            requireFeedbackUnanswered("internalEncoder()");
            String selectedName = requireSingleMotorFeedbackName(parent.specs, "position");
            feedback = () -> FtcSensors.motorPositionTicks(parent.hw, selectedName);
            return coordinateSteps(this);
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> internalEncoder(String motorName) {
            requireFeedbackUnanswered("internalEncoder(...)");
            String selectedName = requireSelectedMotorFeedbackName(
                    parent.specs, Objects.requireNonNull(motorName, "motorName"), "position");
            feedback = () -> FtcSensors.motorPositionTicks(parent.hw, selectedName);
            return coordinateSteps(this);
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> averageInternalEncoders() {
            requireFeedbackUnanswered("averageInternalEncoders()");
            ensureMotorFeedbackAvailable(parent.specs, "position");
            feedback = () -> internalPositionFeedback(parent.hw, parent.specs, null, true);
            return coordinateSteps(this);
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> externalEncoder(String name) {
            return externalEncoder(name, Direction.FORWARD);
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> externalEncoder(
                String name, Direction direction) {
            requireFeedbackUnanswered("externalEncoder(...)");
            String checkedName = requireFeedbackName(name);
            Direction checkedDirection = Objects.requireNonNull(direction, "direction");
            feedback = () -> FtcSensors.motorPositionTicks(
                    parent.hw, checkedName, checkedDirection);
            return coordinateSteps(this);
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> nativeFeedback(ScalarSource source) {
            requireFeedbackUnanswered("nativeFeedback(...)");
            ScalarSource checked = Objects.requireNonNull(source, "source");
            feedback = () -> checked;
            return coordinateSteps(this);
        }

        @Override
        protected void validatePositionSourceRecipe() {
            if (controlKind == null) {
                throw new IllegalStateException("Motor position builder requires deviceManaged(), "
                        + "deviceManagedWithOverrides(), or regulated()");
            }
            if (controlKind == PositionControlKind.REGULATED) {
                parent.requireDefaultGroupScalingForRegulated("position");
                if (feedback == null || !positionControlRecipeAnswered()) {
                    throw new IllegalStateException("Regulated motor position requires a feedback answer "
                            + "(internalEncoder(), averageInternalEncoders(), externalEncoder(...), or "
                            + "nativeFeedback(...)), a setpoint/control answer, and PID feedback");
                }
            } else {
                validateDeviceManagedConfiguration();
                parent.validateDeviceManagedPositionRecipe(
                        boundedStaticNativeEndpointsForValidation());
            }
        }

        @Override
        protected Plants.TargetStep<PositionPlant> createSharedPositionTargetStep() {
            if (isDeviceManaged()) {
                PowerLimitedPositionOutput output = parent.groupedMotorPosition(deviceConfig);
                ScalarSource measurement = parent.deviceManagedPositionMeasurement();
                Plants.DeviceManagedPositionStep<
                        Plants.SymmetricOutputPowerPolicyStep<PositionPlant>> start =
                        Plants.fromOutputs()
                        .deviceManagedPosition(output, measurement);
                Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                        Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>> periodicity =
                        start.searchPowerOutput(parent.deviceManagedPositionSearchPower());
                Plants.SymmetricOutputPowerPolicyStep<PositionPlant> outputPolicy =
                        replayPositionCoordinates(periodicity);
                return deviceMaximumOutputPowerAnswered
                        ? outputPolicy.outputPowerLimitedTo(deviceMaximumOutputPower)
                        : outputPolicy;
            }
            ScalarSource measurement = feedback.get();
            PowerOutput power = parent.groupedMotorPower();
            Plants.PositionControlStep control = replayPositionCoordinates(
                    Plants.fromOutputs().regulatedPosition(power, measurement));
            return replayPositionControl(control);
        }

        @Override
        protected PositionPlant finishBuiltPlant(PositionPlant plant) {
            if (!isDeviceManaged()) {
                return plant;
            }
            PositionPlant result = plant;
            if (parent.specs.size() == 1) {
                if (parent.lastSingleDeviceManagedPositionMotor == null) {
                    throw new IllegalStateException(
                            "Single-motor device-managed position Plant lost its FTC motor identity");
                }
                result = new FtcDeviceManagedPositionPlant(
                        plant,
                        parent.lastSingleDeviceManagedPositionMotor,
                        parent.specs.get(0).name,
                        configuredTargetRange());
            }
            parent.commitPendingOverride();
            return result;
        }

        @Override
        protected void onBuildFailure(RuntimeException failure) {
            parent.rollbackPendingOverride(failure);
        }

        @Override
        protected boolean isRegulatedControlPath() {
            return controlKind == PositionControlKind.REGULATED;
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

        private void requireOpenDeviceManagedOverrides(String operation) {
            requireMutable(operation);
            if (controlKind != PositionControlKind.DEVICE_MANAGED_TUNED) {
                throw new IllegalStateException(
                        operation + " requires deviceManagedWithOverrides()");
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
                    throw new IllegalStateException("deviceManaged() cannot retain "
                            + "device-managed position tuning");
                }
            } else if (controlKind == PositionControlKind.DEVICE_MANAGED_TUNED) {
                if (!deviceConfig.hasAnyOverride()) {
                    throw new IllegalStateException("deviceManagedWithOverrides() requires at "
                            + "least one controller override before doneOverrides(); use "
                            + "deviceManaged() when no override is needed");
                }
                if (!deviceConfig.tuningClosed) {
                    throw new IllegalStateException("Call doneOverrides() after the "
                            + "device-managed position controller overrides");
                }
            }

            FtcControllerConfigurationValidation
                    .requireRunToPositionMaximumOutputPowerMagnitude(
                    deviceMaximumOutputPower, OUTPUT_POWER_OPERATION);
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

        private boolean positionCoordinatesComplete() {
            return positionToleranceWasAnswered();
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

    private static final class CrServoPositionBuilder extends BasePositionBuilder
            implements CrServoPositionControlStep,
            CrServoRegulatedPositionFeedbackStep {
        private final CrServoBuilder parent;
        private Supplier<ScalarSource> feedback;
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
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> externalEncoder(String name) {
            return externalEncoder(name, Direction.FORWARD);
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> externalEncoder(
                String name, Direction direction) {
            requireFeedbackUnanswered("externalEncoder(...)");
            String checkedName = requireFeedbackName(name);
            Direction checkedDirection = Objects.requireNonNull(direction, "direction");
            feedback = () -> FtcSensors.motorPositionTicks(
                    parent.hw, checkedName, checkedDirection);
            return coordinateSteps(this);
        }

        @Override
        public Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.PositionControlStep>> nativeFeedback(ScalarSource source) {
            requireFeedbackUnanswered("nativeFeedback(...)");
            ScalarSource checked = Objects.requireNonNull(source, "source");
            feedback = () -> checked;
            return coordinateSteps(this);
        }

        @Override
        protected void validatePositionSourceRecipe() {
            if (!regulatedAnswered) {
                throw new IllegalStateException("CR-servo position builder requires regulated()");
            }
            parent.requireDefaultGroupScalingForRegulated();
            if (feedback == null || !positionControlRecipeAnswered()) {
                throw new IllegalStateException("Regulated CR-servo position requires externalEncoder(...) "
                        + "or nativeFeedback(...), followed by a setpoint/control answer and PID feedback");
            }
        }

        @Override
        protected Plants.TargetStep<PositionPlant> createSharedPositionTargetStep() {
            ScalarSource measurement = feedback.get();
            PowerOutput power = parent.groupedCrServoPower();
            Plants.PositionControlStep control = replayPositionCoordinates(
                    Plants.fromOutputs().regulatedPosition(power, measurement));
            return replayPositionControl(control);
        }

        @Override
        protected boolean isRegulatedControlPath() {
            return regulatedAnswered;
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
     * Exact shared-power fan-out. Every child is validated before any requested write, and the
     * original value is copied directly so identity includes the signed-zero bit pattern.
     * A fan-out ending in a {@link RuntimeException} abandons later requested writes and begins an
     * ordered best-effort stop traversal. The traversal continues across {@code RuntimeException}s;
     * an {@link Error} remains uncaught and can interrupt later cleanup.
     */
    private static final class IdentityGroupedPowerOutput implements PowerOutput {
        private final List<PowerOutput> outputs;
        private final List<String> names;
        private final String family;
        private double last = Double.NaN;

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
            last = Double.NaN;
            if (!Double.isFinite(power)) {
                IllegalArgumentException failure = nonFiniteGroupedPower(family, power);
                throw attemptAllStopsAfterFailure(failure, outputs, PowerOutput::stop);
            }

            double[] childPowers = new double[outputs.size()];
            for (int i = 0; i < outputs.size(); i++) {
                childPowers[i] = power;
                requireClosedDomain(
                        childPowers[i],
                        -1.0,
                        1.0,
                        family + " runtime child " + (i + 1) + " ('" + names.get(i) + "')");
            }

            try {
                for (int i = 0; i < outputs.size(); i++) {
                    outputs.get(i).setPower(childPowers[i]);
                }
            } catch (RuntimeException failure) {
                throw attemptAllStopsAfterFailure(failure, outputs, PowerOutput::stop);
            }
            last = power;
        }

        @Override
        public double getCommandedPower() {
            return last;
        }

        @Override
        public void stop() {
            last = Double.NaN;
            attemptAllStops(outputs, PowerOutput::stop);
            last = 0.0;
        }
    }

    /**
     * Child-mapped fan-out for direct normalized-power Plants. Every mapped command is validated
     * before requested writes begin; a fan-out ending in a {@link RuntimeException} begins an
     * ordered best-effort stop traversal. The traversal continues across {@code RuntimeException}s;
     * an {@link Error} remains uncaught and can interrupt later cleanup.
     */
    private static final class GroupedPowerOutput implements PowerOutput {
        private final List<PowerOutput> outputs;
        private final double[] scales;
        private final double[] biases;
        private final List<String> names;
        private final String family;
        private double last = Double.NaN;

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
            last = Double.NaN;
            if (!Double.isFinite(power)) {
                IllegalArgumentException failure = nonFiniteGroupedPower(family, power);
                throw attemptAllStopsAfterFailure(failure, outputs, PowerOutput::stop);
            }

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

            try {
                for (int i = 0; i < outputs.size(); i++) {
                    outputs.get(i).setPower(childPowers[i]);
                }
            } catch (RuntimeException failure) {
                throw attemptAllStopsAfterFailure(failure, outputs, PowerOutput::stop);
            }
            last = power;
        }

        @Override
        public double getCommandedPower() {
            return last;
        }

        @Override
        public void stop() {
            last = Double.NaN;
            attemptAllStops(outputs, PowerOutput::stop);
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
            attemptAllStops(outputs, PositionOutput::stop);
        }
    }

    /** Motor-only fan-out that preserves the single-call paired target-plus-power capability. */
    private static final class GroupedPowerLimitedPositionOutput
            implements PowerLimitedPositionOutput {
        private final List<PowerLimitedPositionOutput> outputs;
        private final double[] scales;
        private final double[] biases;
        private final List<String> names;
        private double lastPosition = Double.NaN;
        private double lastMaximumOutputPowerMagnitude = Double.NaN;

        private GroupedPowerLimitedPositionOutput(
                List<PowerLimitedPositionOutput> outputs,
                double[] scales,
                double[] biases,
                List<String> names) {
            this.outputs = new ArrayList<>(Objects.requireNonNull(outputs, "outputs"));
            this.scales = Objects.requireNonNull(scales, "scales").clone();
            this.biases = Objects.requireNonNull(biases, "biases").clone();
            this.names = new ArrayList<>(Objects.requireNonNull(names, "names"));
            requireMatchingChildShape(
                    this.outputs.size(), this.scales, this.biases, this.names);
        }

        @Override
        public void setPosition(double position, double maximumOutputPowerMagnitude) {
            double[] childPositions = new double[outputs.size()];
            for (int i = 0; i < outputs.size(); i++) {
                String childDescription = "motor position runtime child " + (i + 1)
                        + " ('" + names.get(i) + "')";
                childPositions[i] = mappedChildCommand(
                        "motor position runtime command",
                        i,
                        names.get(i),
                        scales[i],
                        biases[i],
                        position);
                checkedMotorPositionTicks(childPositions[i], childDescription);
            }
            double checkedMagnitude =
                    FtcControllerConfigurationValidation
                            .requireRunToPositionMaximumOutputPowerMagnitude(
                            maximumOutputPowerMagnitude,
                            "FtcActuators motor position outputPowerLimitedTo(...)");

            try {
                for (int i = 0; i < outputs.size(); i++) {
                    outputs.get(i).setPosition(childPositions[i], checkedMagnitude);
                }
            } catch (RuntimeException failure) {
                lastPosition = Double.NaN;
                lastMaximumOutputPowerMagnitude = Double.NaN;
                throw failure;
            }
            lastPosition = position;
            lastMaximumOutputPowerMagnitude = checkedMagnitude;
        }

        @Override
        public double getCommandedPosition() {
            return lastPosition;
        }

        @Override
        public double getCommandedMaximumOutputPowerMagnitude() {
            return lastMaximumOutputPowerMagnitude;
        }

        @Override
        public void stop() {
            attemptAllStops(outputs, PowerLimitedPositionOutput::stop);
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------------------------

    private static <T> void attemptAllStops(
            List<T> outputs,
            Consumer<? super T> stopAction) {
        CleanupActions.attemptAll(stopActions(outputs, stopAction));
    }

    private static <T> RuntimeException attemptAllStopsAfterFailure(
            RuntimeException failure,
            List<T> outputs,
            Consumer<? super T> stopAction) {
        return CleanupActions.attemptAllAfterFailure(
                failure,
                stopActions(outputs, stopAction));
    }

    private static <T> Runnable[] stopActions(
            List<T> outputs,
            Consumer<? super T> stopAction) {
        Runnable[] actions = new Runnable[outputs.size()];
        for (int index = 0; index < outputs.size(); index++) {
            T output = outputs.get(index);
            actions[index] = () -> stopAction.accept(output);
        }
        return actions;
    }

    private static IllegalArgumentException nonFiniteGroupedPower(String family, double power) {
        return new IllegalArgumentException(family + " runtime power must be finite and inside "
                + "[-1.0, 1.0], got " + power);
    }

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
            public double getAsDouble(edu.ftcsushi.fw.core.time.LoopClock clock) {
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
            public double getAsDouble(edu.ftcsushi.fw.core.time.LoopClock clock) {
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

    private static List<Direction> childDirections(List<MotorBuilder.Spec> specs) {
        List<Direction> values = new ArrayList<>(specs.size());
        for (MotorBuilder.Spec spec : specs) {
            values.add(spec.direction);
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

    private static double requireFiniteControlValue(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
        return value;
    }

    private static void requireFiniteOrderedControlRange(double minimum,
                                                         double maximum,
                                                         String operation) {
        requireFiniteControlValue(minimum, operation + " minimum");
        requireFiniteControlValue(maximum, operation + " maximum");
        if (minimum > maximum) {
            throw new IllegalArgumentException(operation
                    + " requires minimum <= maximum, got minimum=" + minimum
                    + ", maximum=" + maximum);
        }
    }

    private static void requireVoltagePolicy(double referenceVoltage,
                                             double minimumVoltage,
                                             double maximumScale) {
        requireFinitePositive(referenceVoltage, "referenceVoltage");
        requireFinitePositive(minimumVoltage, "minimumVoltage");
        requireFiniteControlValue(maximumScale, "maximumScale");
        if (minimumVoltage > referenceVoltage) {
            throw new IllegalArgumentException("voltageCompensationFrom(...) requires "
                    + "minimumVoltage <= referenceVoltage");
        }
        if (maximumScale < 1.0) {
            throw new IllegalArgumentException("voltageCompensationFrom(...) requires "
                    + "maximumScale >= 1.0");
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

    private static List<FtcMotorPidfConfiguration> captureVelocityConfigurations(
            List<DcMotorEx> motors,
            List<String> names,
            String operation) {
        List<FtcMotorPidfConfiguration> captured = new ArrayList<>(motors.size());
        for (int index = 0; index < motors.size(); index++) {
            captured.add(FtcMotorControllers.readConfiguration(
                    motors.get(index),
                    names.get(index),
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    operation));
        }
        return captured;
    }

    private static void applyVelocityOverride(List<DcMotorEx> motors,
                                              List<String> names,
                                              double[] velocityPidf) {
        for (int index = 0; index < motors.size(); index++) {
            DcMotorEx motor = motors.get(index);
            try {
                motor.setVelocityPIDFCoefficients(
                        velocityPidf[0], velocityPidf[1], velocityPidf[2], velocityPidf[3]);
                FtcMotorPidfConfiguration readback = FtcMotorControllers.readConfiguration(
                        motor,
                        names.get(index),
                        DcMotor.RunMode.RUN_USING_ENCODER,
                        "FtcActuators device-managed velocity override readback");
                if (readback.algorithm() != MotorControlAlgorithm.PIDF) {
                    throw new IllegalStateException("readback reported algorithm "
                            + readback.algorithm() + " instead of PIDF");
                }
            } catch (RuntimeException failure) {
                throw new IllegalStateException("FTC SDK device-managed velocity override failed "
                        + "for motor '" + names.get(index) + "'", failure);
            }
        }
    }

    private static void restoreVelocityConfigurationsAfterBuildFailure(
            List<DcMotorEx> motors,
            List<String> names,
            List<FtcMotorPidfConfiguration> captured,
            RuntimeException primaryFailure) {
        Runnable[] restorations = new Runnable[motors.size()];
        for (int index = 0; index < motors.size(); index++) {
            final int memberIndex = index;
            restorations[index] = () -> {
                FtcMotorPidfConfiguration expected = captured.get(memberIndex);
                motors.get(memberIndex).setPIDFCoefficients(
                        DcMotor.RunMode.RUN_USING_ENCODER,
                        expected.toSdkCoefficients());
                FtcMotorPidfConfiguration actual = FtcMotorControllers.readConfiguration(
                        motors.get(memberIndex),
                        names.get(memberIndex),
                        DcMotor.RunMode.RUN_USING_ENCODER,
                        "FtcActuators velocity build-failure restore readback");
                if (!expected.equals(actual)) {
                    throw new IllegalStateException("Velocity build-failure restoration for motor '"
                            + names.get(memberIndex) + "' did not exactly match the captured "
                            + "tuple and algorithm");
                }
            };
        }
        try {
            CleanupActions.attemptAll(restorations);
        } catch (RuntimeException restorationFailure) {
            primaryFailure.addSuppressed(restorationFailure);
        }
    }

    private static final class PositionControllerBaseline {
        private final FtcMotorPidfConfiguration outer;
        private final FtcMotorPidfConfiguration inner;
        private final int targetToleranceTicks;

        private PositionControllerBaseline(FtcMotorPidfConfiguration outer,
                                           FtcMotorPidfConfiguration inner,
                                           int targetToleranceTicks) {
            this.outer = outer;
            this.inner = inner;
            this.targetToleranceTicks = targetToleranceTicks;
        }
    }

    private static List<PositionControllerBaseline> capturePositionConfigurations(
            List<DcMotorEx> motors,
            List<String> names,
            String operation) {
        List<PositionControllerBaseline> captured = new ArrayList<>(motors.size());
        for (int index = 0; index < motors.size(); index++) {
            captured.add(new PositionControllerBaseline(
                    FtcMotorControllers.readConfiguration(
                            motors.get(index), names.get(index),
                            DcMotor.RunMode.RUN_TO_POSITION, operation),
                    FtcMotorControllers.readConfiguration(
                            motors.get(index), names.get(index),
                            DcMotor.RunMode.RUN_USING_ENCODER, operation),
                    motors.get(index).getTargetPositionTolerance()));
        }
        return captured;
    }

    private static void applyPositionOverrides(List<DcMotorEx> motors,
                                               List<String> names,
                                               DeviceManagedPositionConfig cfg) {
        for (int index = 0; index < motors.size(); index++) {
            DcMotorEx motor = motors.get(index);
            String name = names.get(index);
            try {
                if (cfg.outerPositionP != null) {
                    motor.setPositionPIDFCoefficients(cfg.outerPositionP);
                    FtcMotorPidfConfiguration outer = FtcMotorControllers.readConfiguration(
                            motor,
                            name,
                            DcMotor.RunMode.RUN_TO_POSITION,
                            "FtcActuators outer-position override readback");
                    if (outer.algorithm() != MotorControlAlgorithm.PIDF) {
                        throw new IllegalStateException("outer readback reported algorithm "
                                + outer.algorithm() + " instead of PIDF");
                    }
                }
                if (cfg.innerVelocityPidf != null) {
                    double[] inner = cfg.innerVelocityPidf;
                    motor.setVelocityPIDFCoefficients(
                            inner[0], inner[1], inner[2], inner[3]);
                    FtcMotorPidfConfiguration readback = FtcMotorControllers.readConfiguration(
                            motor,
                            name,
                            DcMotor.RunMode.RUN_USING_ENCODER,
                            "FtcActuators inner-velocity override readback");
                    if (readback.algorithm() != MotorControlAlgorithm.PIDF) {
                        throw new IllegalStateException("inner readback reported algorithm "
                                + readback.algorithm() + " instead of PIDF");
                    }
                }
                if (cfg.devicePositionToleranceTicks != null) {
                    motor.setTargetPositionTolerance(cfg.devicePositionToleranceTicks);
                    int readback = motor.getTargetPositionTolerance();
                    if (readback != cfg.devicePositionToleranceTicks) {
                        throw new IllegalStateException("tolerance read back " + readback
                                + " instead of " + cfg.devicePositionToleranceTicks);
                    }
                }
            } catch (RuntimeException failure) {
                throw new IllegalStateException("FTC SDK device-managed position override failed "
                        + "for motor '" + name + "'", failure);
            }
        }
    }

    private static IllegalStateException controllerOverrideFailure(
            String message,
            RuntimeException failure) {
        RuntimeException directCause = failure.getCause() instanceof RuntimeException
                ? (RuntimeException) failure.getCause()
                : failure;
        IllegalStateException result = new IllegalStateException(message, directCause);
        for (Throwable suppressed : failure.getSuppressed()) {
            result.addSuppressed(suppressed);
        }
        return result;
    }

    private static void restorePositionConfigurationsAfterBuildFailure(
            List<DcMotorEx> motors,
            List<String> names,
            List<PositionControllerBaseline> captured,
            RuntimeException primaryFailure) {
        Runnable[] restorations = new Runnable[motors.size() * 3];
        for (int index = 0; index < motors.size(); index++) {
            final int memberIndex = index;
            int base = index * 3;
            restorations[base] = () -> motors.get(memberIndex).setPIDFCoefficients(
                    DcMotor.RunMode.RUN_TO_POSITION,
                    captured.get(memberIndex).outer.toSdkCoefficients());
            restorations[base + 1] = () -> motors.get(memberIndex).setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    captured.get(memberIndex).inner.toSdkCoefficients());
            restorations[base + 2] = () -> motors.get(memberIndex).setTargetPositionTolerance(
                    captured.get(memberIndex).targetToleranceTicks);
        }
        try {
            CleanupActions.attemptAll(restorations);
            for (int index = 0; index < motors.size(); index++) {
                PositionControllerBaseline expected = captured.get(index);
                FtcMotorPidfConfiguration outer = FtcMotorControllers.readConfiguration(
                        motors.get(index), names.get(index), DcMotor.RunMode.RUN_TO_POSITION,
                        "FtcActuators position build-failure restore readback");
                FtcMotorPidfConfiguration inner = FtcMotorControllers.readConfiguration(
                        motors.get(index), names.get(index), DcMotor.RunMode.RUN_USING_ENCODER,
                        "FtcActuators position build-failure restore readback");
                int tolerance = motors.get(index).getTargetPositionTolerance();
                if (!expected.outer.equals(outer)
                        || !expected.inner.equals(inner)
                        || expected.targetToleranceTicks != tolerance) {
                    throw new IllegalStateException("Position build-failure restoration for motor '"
                            + names.get(index) + "' did not exactly match all captured controller "
                            + "configurations");
                }
            }
        } catch (RuntimeException restorationFailure) {
            primaryFailure.addSuppressed(restorationFailure);
        }
    }
}
