package edu.ftcphoenix.fw.actuation;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.ftc.FtcHardware;

/**
 * Beginner-friendly helpers for wiring FTC hardware into {@link Plant} instances.
 *
 * <p>The goal of {@code Actuators} is to let teams create plants in a readable,
 * staged style without having to know about {@link FtcHardware} or the underlying
 * HAL interfaces.</p>
 *
 * <h2>Examples</h2>
 *
 * <pre>{@code
 * import static edu.ftcphoenix.fw.core.hal.Direction.*;
 *
 * // Shooter: two motors, velocity control.
 * Plant shooter = Actuators.plant(hardwareMap)
 *     .motor("shooterLeft",  FORWARD)
 *     .andMotor("shooterRight", REVERSE)
 *         .scale(1.00)        // optional: last-added motor only (appears after you add a 2nd)
 *     .velocity(50.0)
 *     .rateLimit(500.0)
 *     .build();
 *
 * // Transfer: two CR servos, open-loop power.
 * Plant transfer = Actuators.plant(hardwareMap)
 *     .crServo("transferLeft",  FORWARD)
 *     .andCrServo("transferRight", REVERSE)
 *     .power()
 *     .build();
 *
 * // Pusher: one positional servo.
 * Plant pusher = Actuators.plant(hardwareMap)
 *     .servo("pusher", FORWARD)
 *     .position()
 *     .build();
 * }</pre>
 *
 * <p>Notably, method availability is restricted so IntelliSense only shows what
 * makes sense at each stage:
 * <ul>
 *   <li>Servos only offer {@code position()} (no {@code power()} or {@code velocity()}).</li>
 *   <li>CR servos only offer {@code power()}.</li>
 *   <li>{@code scale()/bias()} only appear after you add a second actuator.</li>
 * </ul>
 */
public final class Actuators {

    /**
     * Default tolerance for motor position plants (native units).
     */
    private static final double DEFAULT_MOTOR_POSITION_TOLERANCE_NATIVE = 10.0;

    /**
     * Default tolerance for motor velocity plants (native units).
     */
    private static final double DEFAULT_MOTOR_VELOCITY_TOLERANCE_NATIVE = 100.0;

    private Actuators() {
        // no instances
    }

    /**
     * Entry point for the {@link Actuators} staged builder.
     *
     * <p>This is the recommended way for students to wire FTC devices into a {@link Plant}
     * without needing to know about {@link FtcHardware} or the HAL interfaces.</p>
     *
     * <p>Typical usage:</p>
     *
     * <pre>{@code
     * import static edu.ftcphoenix.fw.core.hal.Direction.*;
     *
     * Plant intake = Actuators.plant(hardwareMap)
     *     .motor("intakeMotor", FORWARD)
     *     .power()
     *     .build();
     * }</pre>
     *
     * @param hw FTC {@link HardwareMap} used to look up configured devices
     * @return the first stage of the builder (choose motor / servo / CR servo)
     * @throws NullPointerException if {@code hw} is {@code null}
     */
    public static HardwareStep plant(HardwareMap hw) {
        return new HardwareStep(hw);
    }

    // =====================================================================
    // STAGE 1: PICK HARDWARE
    // =====================================================================

    /**
     * Stage 1 of the builder: choose what kind of FTC actuator(s) you want to control.
     *
     * <p>Once hardware is chosen, you move on to a stage where you select a control mode
     * and then call {@link ModifiersStep#build()} to get the final {@link Plant}.</p>
     */
    public static final class HardwareStep {
        private final HardwareMap hw;

        private HardwareStep(HardwareMap hw) {
            this.hw = Objects.requireNonNull(hw, "HardwareMap is required");
        }

        /**
         * Begin wiring a DC motor-backed plant.
         *
         * <p>This selects a motor by name from the FTC Robot Configuration and applies the
         * requested {@link Direction} at the FTC SDK level. After this, the rest of the
         * framework can treat positive values as "forward" for your mechanism.</p>
         *
         * <p>Next steps:</p>
         * <ul>
         *   <li>Optionally add more motors with {@link MotorSingleStep#andMotor(String, Direction)}.</li>
         *   <li>Choose a control mode: {@link MotorSingleStep#power()},
         *       {@link MotorSingleStep#velocity()}, or {@link MotorSingleStep#position()}.</li>
         *   <li>Optionally add modifiers like {@link ModifiersStep#rateLimit(double)}.</li>
         *   <li>Finish with {@link ModifiersStep#build()}.</li>
         * </ul>
         *
         * @param name      configured device name in the FTC Robot Configuration
         * @param direction logical direction for this motor channel
         * @return the next stage of the builder for motors
         * @throws NullPointerException if {@code name} or {@code direction} is {@code null}
         */

        public MotorSingleStep motor(String name, Direction direction) {
            return new MotorBuilder(hw, name, direction);
        }

        /**
         * Begin wiring a positional servo plant.
         *
         * <p>Servo plants in Phoenix are "set-and-hold": when you call
         * {@link Plant#setTarget(double)}, the servo is commanded immediately and the plant
         * reports {@link Plant#atSetpoint()} as {@code true} (there is no position feedback
         * from a standard FTC servo).</p>
         *
         * <p>The target units for servo position are FTC-native {@code 0.0 .. 1.0}.</p>
         *
         * <p>Next steps:</p>
         * <ul>
         *   <li>Optionally add more servos with {@link ServoSingleStep#andServo(String, Direction)}.</li>
         *   <li>Choose {@link ServoSingleStep#position()}.</li>
         *   <li>Finish with {@link ModifiersStep#build()}.</li>
         * </ul>
         *
         * @param name      configured device name in the FTC Robot Configuration
         * @param direction logical direction for this servo channel
         * @return the next stage of the builder for servos
         * @throws NullPointerException if {@code name} or {@code direction} is {@code null}
         */

        public ServoSingleStep servo(String name, Direction direction) {
            return new ServoBuilder(hw, name, direction);
        }

        /**
         * Begin wiring a continuous-rotation (CR) servo power plant.
         *
         * <p>CR servos are treated like small motors: the control mode is open-loop power
         * via {@link CrServoSingleStep#power()}, with targets typically in {@code [-1.0, +1.0]}.</p>
         *
         * <p>Next steps:</p>
         * <ul>
         *   <li>Optionally add more CR servos with {@link CrServoSingleStep#andCrServo(String, Direction)}.</li>
         *   <li>Choose {@link CrServoSingleStep#power()}.</li>
         *   <li>Finish with {@link ModifiersStep#build()}.</li>
         * </ul>
         *
         * @param name      configured device name in the FTC Robot Configuration
         * @param direction logical direction for this CR servo channel
         * @return the next stage of the builder for CR servos
         * @throws NullPointerException if {@code name} or {@code direction} is {@code null}
         */

        public CrServoSingleStep crServo(String name, Direction direction) {
            return new CrServoBuilder(hw, name, direction);
        }
    }

    // =====================================================================
    // COMMON: MODIFIERS + BUILD
    // =====================================================================

    /**
     * Final stage of the builder: optionally apply modifiers, then {@link #build()}.
     *
     * <p>Modifiers are small wrappers that change how the plant behaves without changing
     * your robot code. For example, {@link #rateLimit(double)} wraps the plant in a
     * {@link RateLimitedPlant} so targets cannot change faster than some limit.</p>
     */
    public interface ModifiersStep {

        /**
         * Apply a <b>symmetric</b> rate limit to target changes.
         *
         * <p>The limit is in "plant target units per second". That means:</p>
         * <ul>
         *   <li>Power plants: units are normalized power per second.</li>
         *   <li>Velocity plants: units are native velocity units per second (e.g. ticks/sec²).</li>
         *   <li>Position plants: units are native position units per second (e.g. ticks/sec).</li>
         * </ul>
         *
         * @param maxDeltaPerSec maximum absolute change allowed per second (must be {@code >= 0})
         * @return this stage for chaining
         * @throws IllegalArgumentException if {@code maxDeltaPerSec < 0}
         */
        ModifiersStep rateLimit(double maxDeltaPerSec);

        /**
         * Apply an <b>asymmetric</b> rate limit to target changes.
         *
         * <p>This is useful when you want a mechanism to ramp up slowly but ramp down quickly,
         * or vice versa (for example, soft-start an intake but allow a rapid stop).</p>
         *
         * @param maxUpPerSec   maximum increase allowed per second (must be {@code >= 0})
         * @param maxDownPerSec maximum decrease allowed per second (must be {@code >= 0})
         * @return this stage for chaining
         * @throws IllegalArgumentException if either argument is negative
         */
        ModifiersStep rateLimit(double maxUpPerSec, double maxDownPerSec);

        /**
         * Finish building and return the final {@link Plant}.
         *
         * @return the constructed plant
         */
        Plant build();
    }

    /**
     * Implementation of {@link ModifiersStep}.
     *
     * <p>This stage wraps the current {@link Plant} and applies optional modifiers
     * (like rate limiting) before returning the final plant via {@link #build()}.</p>
     */
    private static final class ModifiersStepImpl implements ModifiersStep {
        private Plant plant;

        ModifiersStepImpl(Plant plant) {
            this.plant = Objects.requireNonNull(plant, "plant");
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ModifiersStep rateLimit(double maxDeltaPerSec) {
            plant = new RateLimitedPlant(plant, maxDeltaPerSec);
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ModifiersStep rateLimit(double maxUpPerSec, double maxDownPerSec) {
            plant = new RateLimitedPlant(plant, maxUpPerSec, maxDownPerSec);
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Plant build() {
            return plant;
        }
    }

    // =====================================================================
    // DC MOTOR BUILDER
    // =====================================================================

    /**
     * Motor builder stage when exactly one motor has been selected.
     *
     * <p>At this stage, IntelliSense will not show tuning helpers like
     * {@link MotorGroupAddedStep#scale(double)} or {@link MotorGroupAddedStep#bias(double)}
     * because there is nothing to tune relative to (there is only one motor).</p>
     */
    public interface MotorSingleStep {

        /**
         * Add another motor to create a multi-motor mechanism (2+ motors).
         *
         * <p>The motor you add becomes the "last added" motor. Tuning methods such as
         * {@link MotorGroupAddedStep#scale(double)} and {@link MotorGroupAddedStep#bias(double)}
         * apply to that last added motor.</p>
         *
         * @param name      configured motor name in the FTC Robot Configuration
         * @param direction logical direction for the motor
         * @return a stage that allows tuning the motor you just added, adding more motors,
         * or selecting a control mode
         * @throws NullPointerException if {@code name} or {@code direction} is {@code null}
         */
        MotorGroupAddedStep andMotor(String name, Direction direction);

        /**
         * Choose open-loop power control for the motor(s).
         *
         * <p>Targets are normalized power, typically in {@code [-1.0, +1.0]}.</p>
         *
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         */
        ModifiersStep power();

        /**
         * Choose closed-loop velocity control with a default tolerance.
         *
         * <p>The default tolerance is expressed in the motor's native velocity units
         * (for REV encoders, typically ticks per second).</p>
         *
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         */
        ModifiersStep velocity();

        /**
         * Choose closed-loop velocity control with an explicit tolerance.
         *
         * <p>The tolerance is in the same native units reported by the motor velocity channel
         * (for REV encoders, typically ticks per second).</p>
         *
         * @param toleranceNative allowable absolute error band used by {@link Plant#atSetpoint()}
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         * @throws IllegalArgumentException if {@code toleranceNative < 0}
         */
        ModifiersStep velocity(double toleranceNative);

        /**
         * Choose encoder-backed position control with a default tolerance.
         *
         * <p>The default tolerance is expressed in the motor's native position units
         * (for REV encoders, ticks).</p>
         *
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         */
        ModifiersStep position();

        /**
         * Choose encoder-backed position control with an explicit tolerance.
         *
         * <p>The tolerance is in the motor's native position units (for REV encoders, ticks).</p>
         *
         * @param toleranceNative allowable absolute error band used by {@link Plant#atSetpoint()}
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         * @throws IllegalArgumentException if {@code toleranceNative < 0}
         */
        ModifiersStep position(double toleranceNative);
    }

    /**
     * Motor builder stage once you have 2 or more motors selected.
     *
     * <p>At this stage you can keep adding motors with {@link #andMotor(String, Direction)}
     * and then pick a control mode.</p>
     */
    public interface MotorGroupStep {

        /**
         * Add another motor to this motor group.
         *
         * <p>The motor you add becomes the "last added" motor and is the one affected by
         * {@link MotorGroupAddedStep#scale(double)} and {@link MotorGroupAddedStep#bias(double)}.</p>
         *
         * @param name      configured motor name in the FTC Robot Configuration
         * @param direction logical direction for the motor
         * @return a stage that allows tuning the motor you just added
         * @throws NullPointerException if {@code name} or {@code direction} is {@code null}
         */
        MotorGroupAddedStep andMotor(String name, Direction direction);

        /**
         * Choose open-loop power control for the motor group.
         *
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         */
        ModifiersStep power();

        /**
         * Choose closed-loop velocity control for the motor group with a default tolerance.
         *
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         */
        ModifiersStep velocity();

        /**
         * Choose closed-loop velocity control for the motor group with an explicit tolerance.
         *
         * @param toleranceNative allowable absolute error band used by {@link Plant#atSetpoint()}
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         * @throws IllegalArgumentException if {@code toleranceNative < 0}
         */
        ModifiersStep velocity(double toleranceNative);

        /**
         * Choose encoder-backed position control for the motor group with a default tolerance.
         *
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         */
        ModifiersStep position();

        /**
         * Choose encoder-backed position control for the motor group with an explicit tolerance.
         *
         * @param toleranceNative allowable absolute error band used by {@link Plant#atSetpoint()}
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         * @throws IllegalArgumentException if {@code toleranceNative < 0}
         */
        ModifiersStep position(double toleranceNative);
    }

    /**
     * Returned immediately after adding a motor to a group.
     *
     * <p>This exists so IntelliSense can show tuning methods only when they make sense.
     * The tuning methods ({@link #scale(double)}, {@link #bias(double)}, {@link #tune(double, double)})
     * apply to the <b>last motor you added</b>.</p>
     *
     * <p>Tuning uses this linear mapping:</p>
     *
     * <pre>{@code
     * childTarget = scale * groupTarget + bias
     * }</pre>
     *
     * <p>Notes:</p>
     * <ul>
     *   <li>Use {@link Direction} for reversing. Prefer not to use negative {@code scale} unless
     *       you have a very specific reason.</li>
     *   <li>{@code bias} is a constant offset in the same units as the control mode you choose
     *       (power / velocity / position). Keep it small and be aware of any saturation.</li>
     * </ul>
     */
    public interface MotorGroupAddedStep extends MotorGroupStep {

        /**
         * Set a multiplicative scale applied to the <b>last-added motor</b>.
         *
         * <p>Default is {@code 1.0} (no scaling). Example: if the right motor runs a bit stronger,
         * you can do {@code .andMotor("right", REVERSE).scale(0.98)} to command it 2% lower.</p>
         *
         * @param scale multiplier applied to the group target before it is sent to the last-added motor
         * @return this stage for chaining
         */
        MotorGroupAddedStep scale(double scale);

        /**
         * Set an additive bias applied to the <b>last-added motor</b>.
         *
         * <p>Default is {@code 0.0}. Bias is applied <em>after</em> scaling:
         * {@code childTarget = scale * groupTarget + bias}.</p>
         *
         * <p>Bias is in the same units as the control mode you choose later:</p>
         * <ul>
         *   <li>Power: normalized power offset.</li>
         *   <li>Velocity: native velocity offset.</li>
         *   <li>Position: native position offset.</li>
         * </ul>
         *
         * @param bias constant offset applied to the last-added motor's target
         * @return this stage for chaining
         */
        MotorGroupAddedStep bias(double bias);

        /**
         * Convenience method to set both {@link #scale(double)} and {@link #bias(double)} for
         * the <b>last-added motor</b>.
         *
         * @param scale multiplier applied to the group target
         * @param bias  constant offset applied after scaling
         * @return this stage for chaining
         */
        MotorGroupAddedStep tune(double scale, double bias);
    }

    /**
     * Internal builder that implements the motor stages.
     *
     * <p>We keep this private so students interact only with the staged interfaces.
     * This class collects per-motor specs and constructs the final {@link Plant}.</p>
     */
    private static final class MotorBuilder implements MotorSingleStep, MotorGroupAddedStep {

        /**
         * Specification for one motor in a multi-motor group.
         *
         * <p>Scale/bias are applied to the group target when fanning out to individual motors.</p>
         */
        private static final class Spec {
            final String name;
            final Direction direction;
            double scale = 1.0;
            double bias = 0.0;

            Spec(String name, Direction direction) {
                this.name = Objects.requireNonNull(name, "name");
                this.direction = Objects.requireNonNull(direction, "direction");
            }
        }

        private final HardwareMap hw;
        private final List<Spec> specs = new ArrayList<>();
        private int lastIndex = 0;

        MotorBuilder(HardwareMap hw, String name, Direction direction) {
            this.hw = Objects.requireNonNull(hw, "hw");
            specs.add(new Spec(name, direction));
            lastIndex = 0;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public MotorGroupAddedStep andMotor(String name, Direction direction) {
            specs.add(new Spec(name, direction));
            lastIndex = specs.size() - 1;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public MotorGroupAddedStep scale(double scale) {
            specs.get(lastIndex).scale = scale;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public MotorGroupAddedStep bias(double bias) {
            specs.get(lastIndex).bias = bias;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public MotorGroupAddedStep tune(double scale, double bias) {
            Spec s = specs.get(lastIndex);
            s.scale = scale;
            s.bias = bias;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ModifiersStep power() {
            Plant plant = buildPowerPlant();
            return new ModifiersStepImpl(plant);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ModifiersStep velocity() {
            return velocity(DEFAULT_MOTOR_VELOCITY_TOLERANCE_NATIVE);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ModifiersStep velocity(double toleranceNative) {
            Plant plant = buildVelocityPlant(toleranceNative);
            return new ModifiersStepImpl(plant);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ModifiersStep position() {
            return position(DEFAULT_MOTOR_POSITION_TOLERANCE_NATIVE);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ModifiersStep position(double toleranceNative) {
            Plant plant = buildPositionPlant(toleranceNative);
            return new ModifiersStepImpl(plant);
        }

        private Plant buildPowerPlant() {
            if (specs.size() == 1) {
                Spec s = specs.get(0);
                PowerOutput out = FtcHardware.motorPower(hw, s.name, s.direction);
                return Plants.power(out);
            }

            MultiPlant.Builder mp = MultiPlant.builder();
            for (Spec s : specs) {
                PowerOutput out = FtcHardware.motorPower(hw, s.name, s.direction);
                mp.add(Plants.power(out), s.scale, s.bias);
            }
            return mp.build();
        }

        private Plant buildVelocityPlant(double toleranceNative) {
            if (specs.size() == 1) {
                Spec s = specs.get(0);
                VelocityOutput out = FtcHardware.motorVelocity(hw, s.name, s.direction);
                return Plants.velocity(out, toleranceNative);
            }

            MultiPlant.Builder mp = MultiPlant.builder();
            for (Spec s : specs) {
                VelocityOutput out = FtcHardware.motorVelocity(hw, s.name, s.direction);
                mp.add(Plants.velocity(out, toleranceNative), s.scale, s.bias);
            }
            return mp.build();
        }

        private Plant buildPositionPlant(double toleranceNative) {
            if (specs.size() == 1) {
                Spec s = specs.get(0);
                PositionOutput out = FtcHardware.motorPosition(hw, s.name, s.direction);
                return Plants.motorPosition(out, toleranceNative);
            }

            MultiPlant.Builder mp = MultiPlant.builder();
            for (Spec s : specs) {
                PositionOutput out = FtcHardware.motorPosition(hw, s.name, s.direction);
                mp.add(Plants.motorPosition(out, toleranceNative), s.scale, s.bias);
            }
            return mp.build();
        }
    }

    // =====================================================================
    // SERVO BUILDER
    // =====================================================================

    /**
     * Servo builder stage when exactly one positional servo has been selected.
     *
     * <p>Servo plants only support {@link #position()} (standard FTC servos do not support
     * velocity/encoder-style closed-loop control).</p>
     */
    public interface ServoSingleStep {

        /**
         * Add another positional servo to create a multi-servo mechanism (2+ servos).
         *
         * <p>The servo you add becomes the "last added" servo. Tuning methods such as
         * {@link ServoGroupAddedStep#scale(double)} and {@link ServoGroupAddedStep#bias(double)}
         * apply to that last added servo.</p>
         *
         * @param name      configured servo name in the FTC Robot Configuration
         * @param direction logical direction for the servo channel
         * @return a stage that allows tuning the servo you just added, adding more servos,
         * or selecting {@link #position()}
         * @throws NullPointerException if {@code name} or {@code direction} is {@code null}
         */
        ServoGroupAddedStep andServo(String name, Direction direction);

        /**
         * Choose servo position control.
         *
         * <p>Targets are FTC-native servo positions in {@code 0.0 .. 1.0}.</p>
         *
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         */
        ModifiersStep position();
    }

    /**
     * Servo builder stage once you have 2 or more positional servos selected.
     */
    public interface ServoGroupStep {

        /**
         * Add another positional servo to this servo group.
         *
         * @param name      configured servo name in the FTC Robot Configuration
         * @param direction logical direction for the servo channel
         * @return a stage that allows tuning the servo you just added
         * @throws NullPointerException if {@code name} or {@code direction} is {@code null}
         */
        ServoGroupAddedStep andServo(String name, Direction direction);

        /**
         * Choose servo position control.
         *
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         */
        ModifiersStep position();
    }

    /**
     * Returned immediately after adding a servo to a group.
     *
     * <p>Tuning methods apply to the <b>last servo you added</b> using the mapping:</p>
     *
     * <pre>{@code
     * childTarget = scale * groupTarget + bias
     * }</pre>
     *
     * <p>Bias and scale are in servo-position units. Typical uses are small corrections for
     * mismatched linkage geometry. Prefer {@link Direction} for reversing.</p>
     */
    public interface ServoGroupAddedStep extends ServoGroupStep {

        /**
         * Set a multiplicative scale applied to the <b>last-added servo</b>.
         *
         * @param scale multiplier applied to the group target before it is sent to the last-added servo
         * @return this stage for chaining
         */
        ServoGroupAddedStep scale(double scale);

        /**
         * Set an additive bias applied to the <b>last-added servo</b>.
         *
         * @param bias constant offset applied after scaling
         * @return this stage for chaining
         */
        ServoGroupAddedStep bias(double bias);

        /**
         * Convenience method to set both {@link #scale(double)} and {@link #bias(double)} for
         * the <b>last-added servo</b>.
         *
         * @param scale multiplier applied to the group target
         * @param bias  constant offset applied after scaling
         * @return this stage for chaining
         */
        ServoGroupAddedStep tune(double scale, double bias);
    }

    /**
     * Internal builder that implements the servo stages.
     */
    private static final class ServoBuilder implements ServoSingleStep, ServoGroupAddedStep {

        /**
         * Specification for one servo in a multi-servo group.
         */
        private static final class Spec {
            final String name;
            final Direction direction;
            double scale = 1.0;
            double bias = 0.0;

            Spec(String name, Direction direction) {
                this.name = Objects.requireNonNull(name, "name");
                this.direction = Objects.requireNonNull(direction, "direction");
            }
        }

        private final HardwareMap hw;
        private final List<Spec> specs = new ArrayList<>();
        private int lastIndex = 0;

        ServoBuilder(HardwareMap hw, String name, Direction direction) {
            this.hw = Objects.requireNonNull(hw, "hw");
            specs.add(new Spec(name, direction));
            lastIndex = 0;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ServoGroupAddedStep andServo(String name, Direction direction) {
            specs.add(new Spec(name, direction));
            lastIndex = specs.size() - 1;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ServoGroupAddedStep scale(double scale) {
            specs.get(lastIndex).scale = scale;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ServoGroupAddedStep bias(double bias) {
            specs.get(lastIndex).bias = bias;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ServoGroupAddedStep tune(double scale, double bias) {
            Spec s = specs.get(lastIndex);
            s.scale = scale;
            s.bias = bias;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ModifiersStep position() {
            Plant plant = buildPositionPlant();
            return new ModifiersStepImpl(plant);
        }

        private Plant buildPositionPlant() {
            if (specs.size() == 1) {
                Spec s = specs.get(0);
                PositionOutput out = FtcHardware.servoPosition(hw, s.name, s.direction);
                return Plants.servoPosition(out);
            }

            MultiPlant.Builder mp = MultiPlant.builder();
            for (Spec s : specs) {
                PositionOutput out = FtcHardware.servoPosition(hw, s.name, s.direction);
                mp.add(Plants.servoPosition(out), s.scale, s.bias);
            }
            return mp.build();
        }
    }

    // =====================================================================
    // CR SERVO BUILDER
    // =====================================================================

    /**
     * CR-servo builder stage when exactly one continuous-rotation (CR) servo has been selected.
     *
     * <p>CR servos are controlled with {@link #power()} (open-loop power), just like a motor.</p>
     */
    public interface CrServoSingleStep {

        /**
         * Add another CR servo to create a multi-CR-servo mechanism (2+ CR servos).
         *
         * <p>The CR servo you add becomes the "last added" actuator. Tuning methods such as
         * {@link CrServoGroupAddedStep#scale(double)} and {@link CrServoGroupAddedStep#bias(double)}
         * apply to that last added actuator.</p>
         *
         * @param name      configured CR servo name in the FTC Robot Configuration
         * @param direction logical direction for the CR servo channel
         * @return a stage that allows tuning the CR servo you just added, adding more, or selecting {@link #power()}
         * @throws NullPointerException if {@code name} or {@code direction} is {@code null}
         */
        CrServoGroupAddedStep andCrServo(String name, Direction direction);

        /**
         * Choose open-loop power control.
         *
         * <p>Targets are normalized power, typically in {@code [-1.0, +1.0]}.</p>
         *
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         */
        ModifiersStep power();
    }

    /**
     * CR-servo builder stage once you have 2 or more CR servos selected.
     */
    public interface CrServoGroupStep {

        /**
         * Add another CR servo to this group.
         *
         * @param name      configured CR servo name in the FTC Robot Configuration
         * @param direction logical direction for the CR servo channel
         * @return a stage that allows tuning the CR servo you just added
         * @throws NullPointerException if {@code name} or {@code direction} is {@code null}
         */
        CrServoGroupAddedStep andCrServo(String name, Direction direction);

        /**
         * Choose open-loop power control for the CR servo group.
         *
         * @return modifier stage for optional rate limiting and {@link ModifiersStep#build()}
         */
        ModifiersStep power();
    }

    /**
     * Returned immediately after adding a CR servo to a group.
     *
     * <p>Tuning methods apply to the <b>last CR servo you added</b> using the mapping:</p>
     *
     * <pre>{@code
     * childTarget = scale * groupTarget + bias
     * }</pre>
     *
     * <p>Bias and scale are in power units. Prefer {@link Direction} for reversing.</p>
     */
    public interface CrServoGroupAddedStep extends CrServoGroupStep {

        /**
         * Set a multiplicative scale applied to the <b>last-added CR servo</b>.
         *
         * @param scale multiplier applied to the group target before it is sent to the last-added actuator
         * @return this stage for chaining
         */
        CrServoGroupAddedStep scale(double scale);

        /**
         * Set an additive bias applied to the <b>last-added CR servo</b>.
         *
         * @param bias constant offset applied after scaling
         * @return this stage for chaining
         */
        CrServoGroupAddedStep bias(double bias);

        /**
         * Convenience method to set both {@link #scale(double)} and {@link #bias(double)} for
         * the <b>last-added CR servo</b>.
         *
         * @param scale multiplier applied to the group target
         * @param bias  constant offset applied after scaling
         * @return this stage for chaining
         */
        CrServoGroupAddedStep tune(double scale, double bias);
    }

    /**
     * Internal builder that implements the continuous-rotation servo (CR servo) stages.
     */
    private static final class CrServoBuilder implements CrServoSingleStep, CrServoGroupAddedStep {

        /**
         * Specification for one CR servo in a multi-servo group.
         */
        private static final class Spec {
            final String name;
            final Direction direction;
            double scale = 1.0;
            double bias = 0.0;

            Spec(String name, Direction direction) {
                this.name = Objects.requireNonNull(name, "name");
                this.direction = Objects.requireNonNull(direction, "direction");
            }
        }

        private final HardwareMap hw;
        private final List<Spec> specs = new ArrayList<>();
        private int lastIndex = 0;

        CrServoBuilder(HardwareMap hw, String name, Direction direction) {
            this.hw = Objects.requireNonNull(hw, "hw");
            specs.add(new Spec(name, direction));
            lastIndex = 0;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public CrServoGroupAddedStep andCrServo(String name, Direction direction) {
            specs.add(new Spec(name, direction));
            lastIndex = specs.size() - 1;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public CrServoGroupAddedStep scale(double scale) {
            specs.get(lastIndex).scale = scale;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public CrServoGroupAddedStep bias(double bias) {
            specs.get(lastIndex).bias = bias;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public CrServoGroupAddedStep tune(double scale, double bias) {
            Spec s = specs.get(lastIndex);
            s.scale = scale;
            s.bias = bias;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ModifiersStep power() {
            Plant plant = buildPowerPlant();
            return new ModifiersStepImpl(plant);
        }

        private Plant buildPowerPlant() {
            if (specs.size() == 1) {
                Spec s = specs.get(0);
                PowerOutput out = FtcHardware.crServoPower(hw, s.name, s.direction);
                return Plants.power(out);
            }

            MultiPlant.Builder mp = MultiPlant.builder();
            for (Spec s : specs) {
                PowerOutput out = FtcHardware.crServoPower(hw, s.name, s.direction);
                mp.add(Plants.power(out), s.scale, s.bias);
            }
            return mp.build();
        }
    }
}
