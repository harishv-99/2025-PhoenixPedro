package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.actuation.MultiPlant;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.actuation.RateLimitedPlant;
import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.core.source.ScalarSource;

/**
 * Beginner-friendly FTC boundary builder for wiring hardware into {@link Plant} instances.
 *
 * <p>The staged API keeps the public surface semantic and parallel:</p>
 *
 * <ul>
 *   <li>{@code motor(...).power()}</li>
 *   <li>{@code motor(...).position()}</li>
 *   <li>{@code motor(...).velocity()}</li>
 *   <li>{@code servo(...).position()}</li>
 *   <li>{@code crServo(...).power()}</li>
 *   <li>{@code crServo(...).position(...)} for regulated closed-loop position control</li>
 * </ul>
 *
 * <p>The no-arg motor {@code position()} and {@code velocity()} paths use device-managed FTC motor
 * control by default. Advanced overloads let callers select a regulated framework-owned loop while
 * keeping the same top-level plant vocabulary.</p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * Plant flywheel = FtcActuators.plant(hardwareMap)
 *     .motor("flywheel", Direction.FORWARD)
 *     .velocity()
 *     .build();
 *
 * Plant arm = FtcActuators.plant(hardwareMap)
 *     .motor("arm", Direction.FORWARD)
 *     .position(
 *         MotorPositionControl.regulated(
 *             PositionFeedback.externalEncoder("armEncoder"),
 *             ScalarRegulators.pid(Pid.withGains(0.006, 0.0, 0.0002))
 *         ).positionTolerance(20.0)
 *     )
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
     * First builder step: choose the primary actuator family and the first named device.
     */
    public interface StartStep {
        /**
         * Start building a plant from one or more FTC motors.
         *
         * @param name      configured hardware name of the first motor
         * @param direction logical forward direction for Phoenix commands
         * @return next builder step for motor-specific plant selection
         */
        MotorSingleStep motor(String name, Direction direction);

        /**
         * Start building a plant from one or more FTC standard servos.
         *
         * @param name      configured hardware name of the first servo
         * @param direction logical forward direction for Phoenix commands
         * @return next builder step for servo-specific plant selection
         */
        ServoSingleStep servo(String name, Direction direction);

        /**
         * Start building a plant from one or more FTC continuous-rotation servos.
         *
         * @param name      configured hardware name of the first CR servo
         * @param direction logical forward direction for Phoenix commands
         * @return next builder step for CR-servo-specific plant selection
         */
        CrServoSingleStep crServo(String name, Direction direction);
    }

    /**
     * Final builder step used to apply optional plant decorators and then build the plant.
     */
    public interface ModifiersStep {
        /**
         * Wrap the current plant in a symmetric rate limiter.
         *
         * @param maxDeltaPerSec maximum allowed absolute target change per second in plant units
         * @return this builder step for further chaining
         */
        ModifiersStep rateLimit(double maxDeltaPerSec);

        /**
         * Wrap the current plant in an asymmetric rate limiter.
         *
         * @param maxUpPerSec   maximum allowed upward target change per second in plant units
         * @param maxDownPerSec maximum allowed downward target change per second in plant units
         * @return this builder step for further chaining
         */
        ModifiersStep rateLimit(double maxUpPerSec, double maxDownPerSec);

        /**
         * Finish the staged build and return the constructed plant.
         *
         * @return fully constructed plant with any requested decorators applied
         */
        Plant build();
    }

    /**
     * Builder step for motor-backed plants.
     */
    public interface MotorSingleStep {
        /**
         * Add another motor to the same plant group.
         *
         * @param name      configured hardware name of the additional motor
         * @param direction logical forward direction for Phoenix commands for that motor
         * @return group-aware motor builder step for optional scaling/bias and control selection
         */
        MotorGroupAddedStep andMotor(String name, Direction direction);

        /**
         * Build a direct power plant over the selected motor or motor group.
         *
         * @return final builder step for optional decorators and build
         */
        ModifiersStep power();

        /**
         * Build a motor velocity plant using the default device-managed FTC velocity path.
         *
         * @return final builder step for optional decorators and build
         */
        ModifiersStep velocity();

        /**
         * Build a motor velocity plant using an explicit advanced control specification.
         *
         * @param control advanced motor velocity control specification
         * @return final builder step for optional decorators and build
         */
        ModifiersStep velocity(MotorVelocityControl control);

        /**
         * Build a motor position plant using the default device-managed FTC position path.
         *
         * @return final builder step for optional decorators and build
         */
        ModifiersStep position();

        /**
         * Build a motor position plant using an explicit advanced control specification.
         *
         * @param control advanced motor position control specification
         * @return final builder step for optional decorators and build
         */
        ModifiersStep position(MotorPositionControl control);
    }

    /**
     * Group-aware motor builder step that also exposes per-child scale/bias configuration.
     */
    public interface MotorGroupAddedStep extends MotorSingleStep {
        /**
         * Set the scale applied to the most recently added motor when mapping the group target into
         * that child's target frame.
         *
         * @param scale per-child target scale for the most recently added motor
         * @return this builder step for further chaining
         */
        MotorGroupAddedStep scale(double scale);

        /**
         * Set the bias applied to the most recently added motor when mapping the group target into
         * that child's target frame.
         *
         * @param bias per-child target bias for the most recently added motor
         * @return this builder step for further chaining
         */
        MotorGroupAddedStep bias(double bias);

        /**
         * Convenience helper that applies both scale and bias to the most recently added motor.
         *
         * @param scale per-child target scale for the most recently added motor
         * @param bias  per-child target bias for the most recently added motor
         * @return this builder step for further chaining
         */
        default MotorGroupAddedStep scaleBias(double scale, double bias) {
            return scale(scale).bias(bias);
        }
    }

    /**
     * Builder step for standard-servo-backed plants.
     */
    public interface ServoSingleStep {
        /**
         * Add another standard servo to the same plant group.
         *
         * @param name      configured hardware name of the additional servo
         * @param direction logical forward direction for Phoenix commands for that servo
         * @return group-aware servo builder step for optional scaling/bias
         */
        ServoGroupAddedStep andServo(String name, Direction direction);

        /**
         * Build a standard-servo position plant.
         *
         * @return final builder step for optional decorators and build
         */
        ModifiersStep position();
    }

    /**
     * Group-aware servo builder step that also exposes per-child scale/bias configuration.
     */
    public interface ServoGroupAddedStep extends ServoSingleStep {
        /**
         * Set the scale applied to the most recently added servo when mapping the group target into
         * that child's target frame.
         *
         * @param scale per-child target scale for the most recently added servo
         * @return this builder step for further chaining
         */
        ServoGroupAddedStep scale(double scale);

        /**
         * Set the bias applied to the most recently added servo when mapping the group target into
         * that child's target frame.
         *
         * @param bias per-child target bias for the most recently added servo
         * @return this builder step for further chaining
         */
        ServoGroupAddedStep bias(double bias);

        /**
         * Convenience helper that applies both scale and bias to the most recently added servo.
         *
         * @param scale per-child target scale for the most recently added servo
         * @param bias  per-child target bias for the most recently added servo
         * @return this builder step for further chaining
         */
        default ServoGroupAddedStep scaleBias(double scale, double bias) {
            return scale(scale).bias(bias);
        }
    }

    /**
     * Builder step for continuous-rotation-servo-backed plants.
     */
    public interface CrServoSingleStep {
        /**
         * Add another continuous-rotation servo to the same plant group.
         *
         * @param name      configured hardware name of the additional CR servo
         * @param direction logical forward direction for Phoenix commands for that CR servo
         * @return group-aware CR-servo builder step for optional scaling/bias
         */
        CrServoGroupAddedStep andCrServo(String name, Direction direction);

        /**
         * Build a direct power plant over the selected CR servo or CR-servo group.
         *
         * @return final builder step for optional decorators and build
         */
        ModifiersStep power();

        /**
         * Build a regulated position plant over the selected CR servo or CR-servo group.
         *
         * @param control advanced regulated CR-servo position control specification
         * @return final builder step for optional decorators and build
         */
        ModifiersStep position(CrServoPositionControl control);
    }

    /**
     * Group-aware continuous-rotation-servo builder step that also exposes per-child scale/bias
     * configuration.
     */
    public interface CrServoGroupAddedStep extends CrServoSingleStep {
        /**
         * Set the scale applied to the most recently added CR servo when mapping the group target into
         * that child's target frame.
         *
         * @param scale per-child target scale for the most recently added CR servo
         * @return this builder step for further chaining
         */
        CrServoGroupAddedStep scale(double scale);

        /**
         * Set the bias applied to the most recently added CR servo when mapping the group target into
         * that child's target frame.
         *
         * @param bias per-child target bias for the most recently added CR servo
         * @return this builder step for further chaining
         */
        CrServoGroupAddedStep bias(double bias);

        /**
         * Convenience helper that applies both scale and bias to the most recently added CR servo.
         *
         * @param scale per-child target scale for the most recently added CR servo
         * @param bias  per-child target bias for the most recently added CR servo
         * @return this builder step for further chaining
         */
        default CrServoGroupAddedStep scaleBias(double scale, double bias) {
            return scale(scale).bias(bias);
        }
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
    // Control specs
    // ---------------------------------------------------------------------------------------------

    /**
     * Advanced control spec for motor position plants.
     */
    public abstract static class MotorPositionControl {
        private MotorPositionControl() {
        }

        /**
         * Use the motor/controller's device-managed FTC {@code RUN_TO_POSITION} path.
         *
         * <p>Defaults:</p>
         * <ul>
         *   <li>{@link DeviceManagedMotorPositionControl#positionTolerance(double)}: 10 ticks</li>
         *   <li>{@link DeviceManagedMotorPositionControl#maxPower(double)}: 1.0</li>
         *   <li>{@link DeviceManagedMotorPositionControl#outerPositionP(double)}: unchanged unless set</li>
         *   <li>{@link DeviceManagedMotorPositionControl#innerVelocityPidf(double, double, double, double)}:
         *       unchanged unless set</li>
         *   <li>{@link DeviceManagedMotorPositionControl#devicePositionToleranceTicks(int)}:
         *       unchanged unless set</li>
         * </ul>
         *
         * @return advanced device-managed motor position control specification with default values
         */
        public static DeviceManagedMotorPositionControl deviceManaged() {
            return new DeviceManagedMotorPositionControl();
        }

        /**
         * Use a framework-regulated position loop that drives motor power and samples the supplied
         * feedback source.
         *
         * @param feedback  builder-side specification for the authoritative position measurement
         * @param regulator framework-owned control law that converts setpoint and measurement into
         *                  motor power
         * @return advanced regulated motor position control specification
         */
        public static RegulatedMotorPositionControl regulated(PositionFeedback feedback,
                                                              ScalarRegulator regulator) {
            return new RegulatedMotorPositionControl(feedback, regulator);
        }
    }

    /**
     * Device-managed FTC motor position control.
     *
     * <p>This config corresponds to an outer FTC {@code RUN_TO_POSITION} position layer on top of
     * the motor's inner velocity loop. The position layer exposes only a proportional gain through
     * FTC's {@code setPositionPIDFCoefficients(double p)} API, while the underlying velocity loop
     * can be tuned with PIDF coefficients.</p>
     *
     * <p>The plant-level completion band is configured with {@link #positionTolerance(double)}.
     * The lower-level motor-controller completion override is configured with
     * {@link #devicePositionToleranceTicks(int)}.</p>
     */
    public static final class DeviceManagedMotorPositionControl extends MotorPositionControl {
        private double positionTolerance = 10.0;
        private Double outerPositionP;
        private double[] innerVelocityPidf;
        private Integer devicePositionToleranceTicks;
        private double maxPower = 1.0;

        private DeviceManagedMotorPositionControl() {
        }

        /**
         * Set the Phoenix plant-level position completion band used by {@link Plant#atSetpoint()}.
         *
         * <p>For the default internal-encoder motor position path, this value is in encoder ticks.
         * For other future position domains, the value is always in the plant's position units.</p>
         *
         * <p>Default: 10 ticks.</p>
         *
         * @param positionTolerance absolute completion band in plant position units
         * @return this config object for fluent chaining
         */
        public DeviceManagedMotorPositionControl positionTolerance(double positionTolerance) {
            if (positionTolerance < 0.0) {
                throw new IllegalArgumentException("positionTolerance must be >= 0, got " + positionTolerance);
            }
            this.positionTolerance = positionTolerance;
            return this;
        }

        /**
         * Set the FTC outer position-loop proportional gain used for {@code RUN_TO_POSITION}.
         *
         * <p>FTC's position-layer API exposes only this proportional term. Phoenix leaves the
         * existing device/controller value unchanged unless this setter is called.</p>
         *
         * @param outerPositionP proportional gain for FTC's outer position layer
         * @return this config object for fluent chaining
         */
        public DeviceManagedMotorPositionControl outerPositionP(double outerPositionP) {
            this.outerPositionP = outerPositionP;
            return this;
        }

        /**
         * Set the FTC inner velocity-loop PIDF coefficients used underneath position mode.
         *
         * <p>In FTC, {@code RUN_TO_POSITION} rides on top of {@code RUN_USING_ENCODER}; these inner
         * velocity-loop gains often influence how aggressively the motor approaches and settles at
         * the commanded target. Phoenix leaves the existing device/controller value unchanged unless
         * this setter is called.</p>
         *
         * @param p proportional gain for FTC's inner velocity loop
         * @param i integral gain for FTC's inner velocity loop
         * @param d derivative gain for FTC's inner velocity loop
         * @param f feedforward gain for FTC's inner velocity loop
         * @return this config object for fluent chaining
         */
        public DeviceManagedMotorPositionControl innerVelocityPidf(double p,
                                                                   double i,
                                                                   double d,
                                                                   double f) {
            this.innerVelocityPidf = new double[]{p, i, d, f};
            return this;
        }

        /**
         * Override the FTC motor-controller target-position tolerance in encoder ticks.
         *
         * <p>This is a lower-level device setting, not the same thing as
         * {@link #positionTolerance(double)}. In normal usage, teams should usually set only the
         * plant-level {@code positionTolerance(...)} and let Phoenix use that band for
         * {@link Plant#atSetpoint()}. Use this method only when you specifically want to change the
         * motor controller's own completion threshold via FTC {@code setTargetPositionTolerance(int)}.
         * </p>
         *
         * <p>Phoenix leaves the existing device/controller value unchanged unless this setter is
         * called.</p>
         *
         * @param ticks device-managed FTC target-position tolerance in encoder ticks
         * @return this config object for fluent chaining
         */
        public DeviceManagedMotorPositionControl devicePositionToleranceTicks(int ticks) {
            if (ticks < 0) {
                throw new IllegalArgumentException("devicePositionToleranceTicks must be >= 0, got " + ticks);
            }
            this.devicePositionToleranceTicks = ticks;
            return this;
        }

        /**
         * Set the motor power used when Phoenix issues a new FTC {@code RUN_TO_POSITION} target.
         *
         * <p>Default: 1.0.</p>
         *
         * @param maxPower motor power Phoenix reapplies after each new target command
         * @return this config object for fluent chaining
         */
        public DeviceManagedMotorPositionControl maxPower(double maxPower) {
            if (maxPower < 0.0) {
                throw new IllegalArgumentException("maxPower must be >= 0, got " + maxPower);
            }
            this.maxPower = maxPower;
            return this;
        }
    }

    /**
     * Framework-regulated motor position control using raw motor power and an explicit feedback
     * source.
     */
    public static final class RegulatedMotorPositionControl extends MotorPositionControl {
        private final PositionFeedback feedback;
        private final ScalarRegulator regulator;
        private double positionTolerance = 10.0;

        private RegulatedMotorPositionControl(PositionFeedback feedback, ScalarRegulator regulator) {
            this.feedback = Objects.requireNonNull(feedback, "feedback");
            this.regulator = Objects.requireNonNull(regulator, "regulator");
        }

        /**
         * Set the plant-level position completion band used by {@link Plant#atSetpoint()}.
         *
         * <p>Default: 10 plant units. When using encoder-based feedback helpers, those units are
         * ticks.</p>
         *
         * @param positionTolerance absolute completion band in the plant's position units
         * @return this config object for fluent chaining
         */
        public RegulatedMotorPositionControl positionTolerance(double positionTolerance) {
            if (positionTolerance < 0.0) {
                throw new IllegalArgumentException("positionTolerance must be >= 0, got " + positionTolerance);
            }
            this.positionTolerance = positionTolerance;
            return this;
        }
    }

    /**
     * Advanced control spec for motor velocity plants.
     */
    public abstract static class MotorVelocityControl {
        private MotorVelocityControl() {
        }

        /**
         * Use the motor/controller's device-managed FTC velocity control.
         *
         * <p>Defaults:</p>
         * <ul>
         *   <li>{@link DeviceManagedMotorVelocityControl#velocityTolerance(double)}: 100 ticks/sec</li>
         *   <li>{@link DeviceManagedMotorVelocityControl#velocityPidf(double, double, double, double)}:
         *       unchanged unless set</li>
         * </ul>
         *
         * @return advanced device-managed motor velocity control specification with default values
         */
        public static DeviceManagedMotorVelocityControl deviceManaged() {
            return new DeviceManagedMotorVelocityControl();
        }

        /**
         * Use a framework-regulated velocity loop that drives motor power and samples the supplied
         * feedback source.
         *
         * @param feedback  builder-side specification for the authoritative velocity measurement
         * @param regulator framework-owned control law that converts setpoint and measurement into
         *                  motor power
         * @return advanced regulated motor velocity control specification
         */
        public static RegulatedMotorVelocityControl regulated(VelocityFeedback feedback,
                                                              ScalarRegulator regulator) {
            return new RegulatedMotorVelocityControl(feedback, regulator);
        }
    }

    /**
     * Device-managed FTC motor velocity control.
     *
     * <p>This config corresponds to FTC's motor-side velocity loop driven through
     * {@link DcMotorEx#setVelocity(double)}. The plant-level completion band is configured with
     * {@link #velocityTolerance(double)} while the underlying FTC motor velocity PIDF can be
     * overridden with {@link #velocityPidf(double, double, double, double)}.</p>
     */
    public static final class DeviceManagedMotorVelocityControl extends MotorVelocityControl {
        private double velocityTolerance = 100.0;
        private double[] velocityPidf;

        private DeviceManagedMotorVelocityControl() {
        }

        /**
         * Set the Phoenix plant-level velocity completion band used by {@link Plant#atSetpoint()}.
         *
         * <p>Default: 100 ticks/sec.</p>
         *
         * @param velocityTolerance absolute completion band in plant velocity units
         * @return this config object for fluent chaining
         */
        public DeviceManagedMotorVelocityControl velocityTolerance(double velocityTolerance) {
            if (velocityTolerance < 0.0) {
                throw new IllegalArgumentException("velocityTolerance must be >= 0, got " + velocityTolerance);
            }
            this.velocityTolerance = velocityTolerance;
            return this;
        }

        /**
         * Override the FTC device-managed velocity PIDF coefficients.
         *
         * <p>Phoenix leaves the existing device/controller value unchanged unless this setter is
         * called.</p>
         *
         * @param p proportional gain for FTC's device-managed velocity loop
         * @param i integral gain for FTC's device-managed velocity loop
         * @param d derivative gain for FTC's device-managed velocity loop
         * @param f feedforward gain for FTC's device-managed velocity loop
         * @return this config object for fluent chaining
         */
        public DeviceManagedMotorVelocityControl velocityPidf(double p,
                                                              double i,
                                                              double d,
                                                              double f) {
            this.velocityPidf = new double[]{p, i, d, f};
            return this;
        }
    }

    /**
     * Framework-regulated motor velocity control using raw motor power and an explicit feedback
     * source.
     */
    public static final class RegulatedMotorVelocityControl extends MotorVelocityControl {
        private final VelocityFeedback feedback;
        private final ScalarRegulator regulator;
        private double velocityTolerance = 100.0;

        private RegulatedMotorVelocityControl(VelocityFeedback feedback, ScalarRegulator regulator) {
            this.feedback = Objects.requireNonNull(feedback, "feedback");
            this.regulator = Objects.requireNonNull(regulator, "regulator");
        }

        /**
         * Set the plant-level velocity completion band used by {@link Plant#atSetpoint()}.
         *
         * <p>Default: 100 plant units/sec. When using encoder-based feedback helpers, those units
         * are ticks/sec.</p>
         *
         * @param velocityTolerance absolute completion band in the plant's velocity units
         * @return this config object for fluent chaining
         */
        public RegulatedMotorVelocityControl velocityTolerance(double velocityTolerance) {
            if (velocityTolerance < 0.0) {
                throw new IllegalArgumentException("velocityTolerance must be >= 0, got " + velocityTolerance);
            }
            this.velocityTolerance = velocityTolerance;
            return this;
        }
    }

    /**
     * Advanced control spec for CR-servo position plants.
     *
     * <p>CR servos do not have a device-managed position mode, so regulated control is the only
     * supported path.</p>
     */
    public abstract static class CrServoPositionControl {
        private CrServoPositionControl() {
        }

        /**
         * Use a framework-regulated CR-servo position loop that drives CR-servo power and samples
         * the supplied position feedback source.
         *
         * @param feedback  builder-side specification for the authoritative position measurement
         * @param regulator framework-owned control law that converts setpoint and measurement into
         *                  CR-servo power
         * @return advanced regulated CR-servo position control specification
         */
        public static RegulatedCrServoPositionControl regulated(PositionFeedback feedback,
                                                                ScalarRegulator regulator) {
            return new RegulatedCrServoPositionControl(feedback, regulator);
        }
    }

    /**
     * Framework-regulated CR-servo position control using raw CR-servo power and an explicit
     * feedback source.
     */
    public static final class RegulatedCrServoPositionControl extends CrServoPositionControl {
        private final PositionFeedback feedback;
        private final ScalarRegulator regulator;
        private double positionTolerance = 10.0;

        private RegulatedCrServoPositionControl(PositionFeedback feedback, ScalarRegulator regulator) {
            this.feedback = Objects.requireNonNull(feedback, "feedback");
            this.regulator = Objects.requireNonNull(regulator, "regulator");
        }

        /**
         * Set the plant-level position completion band used by {@link Plant#atSetpoint()}.
         *
         * <p>Default: 10 plant units. When using encoder-based feedback helpers, those units are
         * typically ticks.</p>
         *
         * @param positionTolerance absolute completion band in the plant's position units
         * @return this config object for fluent chaining
         */
        public RegulatedCrServoPositionControl positionTolerance(double positionTolerance) {
            if (positionTolerance < 0.0) {
                throw new IllegalArgumentException("positionTolerance must be >= 0, got " + positionTolerance);
            }
            this.positionTolerance = positionTolerance;
            return this;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Feedback specs
    // ---------------------------------------------------------------------------------------------

    /**
     * Builder-side selector for position feedback. The staged builder resolves this into a concrete
     * {@link ScalarSource} during {@link ModifiersStep#build()}.
     */
    public abstract static class PositionFeedback {
        private PositionFeedback() {
        }

        /**
         * Use the selected motor's internal encoder position as the authoritative plant measurement.
         *
         * <p>This convenience form is valid only when exactly one motor has been selected in the
         * staged builder. For motor groups, prefer {@link #internalEncoder(String)} or
         * {@link #averageInternalEncoders()} so the intended measurement is unambiguous.</p>
         *
         * @return builder-side position feedback specification backed by an internal encoder in ticks
         */
        public static PositionFeedback internalEncoder() {
            return new InternalPositionFeedback(null, false);
        }

        /**
         * Use one named selected motor's internal encoder position as the authoritative plant
         * measurement.
         *
         * @param motorName configured hardware name of one of the motors already selected in this
         *                  staged builder
         * @return builder-side position feedback specification backed by that motor's encoder in ticks
         */
        public static PositionFeedback internalEncoder(String motorName) {
            return new InternalPositionFeedback(motorName, false);
        }

        /**
         * Use the average of all selected motors' internal encoder positions as the authoritative
         * plant measurement.
         *
         * @return builder-side position feedback specification backed by the average encoder position
         * of the selected motor group, in ticks
         */
        public static PositionFeedback averageInternalEncoders() {
            return new InternalPositionFeedback(null, true);
        }

        /**
         * Use an external encoder device exposed through the FTC motor API and read position in
         * native encoder ticks.
         *
         * @param name configured hardware name of the external encoder device
         * @return builder-side position feedback specification backed by the named external encoder
         */
        public static PositionFeedback externalEncoder(String name) {
            return new ExternalPositionFeedback(name, Direction.FORWARD);
        }

        /**
         * Use an external encoder device exposed through the FTC motor API and read position in
         * native encoder ticks with an explicit logical direction.
         *
         * @param name      configured hardware name of the external encoder device
         * @param direction logical direction to apply to the reported encoder ticks
         * @return builder-side position feedback specification backed by the named external encoder
         */
        public static PositionFeedback externalEncoder(String name, Direction direction) {
            return new ExternalPositionFeedback(name, direction);
        }

        /**
         * Use a caller-supplied position source that is already expressed in the desired plant units.
         *
         * @param source authoritative position source already converted into the plant's position units
         * @return builder-side position feedback specification backed by the supplied source
         */
        public static PositionFeedback fromSource(ScalarSource source) {
            return new SourcePositionFeedback(source);
        }

        abstract ScalarSource resolve(HardwareMap hw, List<MotorBuilder.Spec> motorSpecs);
    }

    /**
     * Builder-side selector for velocity feedback. The staged builder resolves this into a concrete
     * {@link ScalarSource} during {@link ModifiersStep#build()}.
     */
    public abstract static class VelocityFeedback {
        private VelocityFeedback() {
        }

        /**
         * Use the selected motor's internal encoder velocity as the authoritative plant measurement.
         *
         * <p>This convenience form is valid only when exactly one motor has been selected in the
         * staged builder. For motor groups, prefer {@link #internalEncoder(String)} or
         * {@link #averageInternalEncoders()} so the intended measurement is unambiguous.</p>
         *
         * @return builder-side velocity feedback specification backed by an internal encoder in
         * native ticks per second
         */
        public static VelocityFeedback internalEncoder() {
            return new InternalVelocityFeedback(null, false);
        }

        /**
         * Use one named selected motor's internal encoder velocity as the authoritative plant
         * measurement.
         *
         * @param motorName configured hardware name of one of the motors already selected in this
         *                  staged builder
         * @return builder-side velocity feedback specification backed by that motor's encoder in
         * native ticks per second
         */
        public static VelocityFeedback internalEncoder(String motorName) {
            return new InternalVelocityFeedback(motorName, false);
        }

        /**
         * Use the average of all selected motors' internal encoder velocities as the authoritative
         * plant measurement.
         *
         * @return builder-side velocity feedback specification backed by the average encoder velocity
         * of the selected motor group, in native ticks per second
         */
        public static VelocityFeedback averageInternalEncoders() {
            return new InternalVelocityFeedback(null, true);
        }

        /**
         * Use an external encoder device exposed through the FTC motor API and read velocity in
         * native ticks per second.
         *
         * @param name configured hardware name of the external encoder device
         * @return builder-side velocity feedback specification backed by the named external encoder
         */
        public static VelocityFeedback externalEncoder(String name) {
            return new ExternalVelocityFeedback(name, Direction.FORWARD);
        }

        /**
         * Use an external encoder device exposed through the FTC motor API and read velocity in
         * native ticks per second with an explicit logical direction.
         *
         * @param name      configured hardware name of the external encoder device
         * @param direction logical direction to apply to the reported encoder velocity
         * @return builder-side velocity feedback specification backed by the named external encoder
         */
        public static VelocityFeedback externalEncoder(String name, Direction direction) {
            return new ExternalVelocityFeedback(name, direction);
        }

        /**
         * Use a caller-supplied velocity source that is already expressed in the desired plant units
         * per second.
         *
         * @param source authoritative velocity source already converted into the plant's velocity units
         * @return builder-side velocity feedback specification backed by the supplied source
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
                for (MotorBuilder.Spec spec : motorSpecs) {
                    sources.add(FtcSensors.motorPositionTicks(hw, spec.name));
                }
                return averageSources(sources);
            }
            if (motorName != null) {
                for (MotorBuilder.Spec spec : motorSpecs) {
                    if (spec.name.equals(motorName)) {
                        return FtcSensors.motorPositionTicks(hw, spec.name);
                    }
                }
                throw new IllegalStateException("PositionFeedback.internalEncoder(\"" + motorName
                        + "\") does not match any motor selected in this builder");
            }
            if (motorSpecs.size() != 1) {
                throw new IllegalStateException("PositionFeedback.internalEncoder() is ambiguous for a "
                        + motorSpecs.size() + "-motor group. Choose internalEncoder(\"name\"), "
                        + "averageInternalEncoders(), or an external encoder.");
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
                for (MotorBuilder.Spec spec : motorSpecs) {
                    sources.add(FtcSensors.motorVelocityTicksPerSec(hw, spec.name));
                }
                return averageSources(sources);
            }
            if (motorName != null) {
                for (MotorBuilder.Spec spec : motorSpecs) {
                    if (spec.name.equals(motorName)) {
                        return FtcSensors.motorVelocityTicksPerSec(hw, spec.name);
                    }
                }
                throw new IllegalStateException("VelocityFeedback.internalEncoder(\"" + motorName
                        + "\") does not match any motor selected in this builder");
            }
            if (motorSpecs.size() != 1) {
                throw new IllegalStateException("VelocityFeedback.internalEncoder() is ambiguous for a "
                        + motorSpecs.size() + "-motor group. Choose internalEncoder(\"name\"), "
                        + "averageInternalEncoders(), or an external encoder.");
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
            for (Spec spec : specs) {
                b.add(Plants.power(FtcHardware.motorPower(hw, spec.name, spec.direction)), spec.scale, spec.bias);
            }
            return new ModifiersStepImpl(b.build());
        }

        @Override
        public ModifiersStep velocity() {
            return velocity(MotorVelocityControl.deviceManaged());
        }

        @Override
        public ModifiersStep velocity(MotorVelocityControl control) {
            Objects.requireNonNull(control, "control");
            if (control instanceof DeviceManagedMotorVelocityControl) {
                DeviceManagedMotorVelocityControl cfg = (DeviceManagedMotorVelocityControl) control;
                if (specs.size() == 1) {
                    Spec spec = specs.get(0);
                    DcMotorEx motor = hw.get(DcMotorEx.class, spec.name);
                    applyDeviceManagedVelocityConfig(motor, spec.name, cfg);
                    VelocityOutput out = FtcHardware.motorVelocity(motor, spec.direction);
                    ScalarSource measurement = FtcSensors.motorVelocityTicksPerSec(motor);
                    return new ModifiersStepImpl(Plants.velocity(out, measurement, cfg.velocityTolerance));
                }
                ensureFeedbackScalesNonZero("device-managed motor velocity");
                MultiPlant.Builder b = MultiPlant.builder();
                for (Spec spec : specs) {
                    DcMotorEx motor = hw.get(DcMotorEx.class, spec.name);
                    applyDeviceManagedVelocityConfig(motor, spec.name, cfg);
                    VelocityOutput out = FtcHardware.motorVelocity(motor, spec.direction);
                    ScalarSource measurement = FtcSensors.motorVelocityTicksPerSec(motor);
                    b.add(Plants.velocity(out, measurement, cfg.velocityTolerance), spec.scale, spec.bias);
                }
                return new ModifiersStepImpl(b.build());
            }
            RegulatedMotorVelocityControl cfg = (RegulatedMotorVelocityControl) control;
            requireDefaultGroupScalingForRegulated("velocity");
            PowerOutput powerOut = groupedMotorPower();
            ScalarSource measurement = cfg.feedback.resolve(hw, specs);
            return new ModifiersStepImpl(Plants.velocityFromPower(
                    powerOut,
                    measurement,
                    cfg.regulator,
                    cfg.velocityTolerance));
        }

        @Override
        public ModifiersStep position() {
            return position(MotorPositionControl.deviceManaged());
        }

        @Override
        public ModifiersStep position(MotorPositionControl control) {
            Objects.requireNonNull(control, "control");
            if (control instanceof DeviceManagedMotorPositionControl) {
                DeviceManagedMotorPositionControl cfg = (DeviceManagedMotorPositionControl) control;
                if (specs.size() == 1) {
                    Spec spec = specs.get(0);
                    DcMotorEx motor = hw.get(DcMotorEx.class, spec.name);
                    applyDeviceManagedPositionConfig(motor, spec.name, cfg);
                    PositionOutput out = FtcHardware.motorPosition(motor, spec.direction, cfg.maxPower);
                    ScalarSource measurement = FtcSensors.motorPositionTicks(motor);
                    return new ModifiersStepImpl(Plants.position(out, measurement, cfg.positionTolerance));
                }
                ensureFeedbackScalesNonZero("device-managed motor position");
                MultiPlant.Builder b = MultiPlant.builder();
                for (Spec spec : specs) {
                    DcMotorEx motor = hw.get(DcMotorEx.class, spec.name);
                    applyDeviceManagedPositionConfig(motor, spec.name, cfg);
                    PositionOutput out = FtcHardware.motorPosition(motor, spec.direction, cfg.maxPower);
                    ScalarSource measurement = FtcSensors.motorPositionTicks(motor);
                    b.add(Plants.position(out, measurement, cfg.positionTolerance), spec.scale, spec.bias);
                }
                return new ModifiersStepImpl(b.build());
            }
            RegulatedMotorPositionControl cfg = (RegulatedMotorPositionControl) control;
            requireDefaultGroupScalingForRegulated("position");
            PowerOutput powerOut = groupedMotorPower();
            ScalarSource measurement = cfg.feedback.resolve(hw, specs);
            return new ModifiersStepImpl(Plants.positionFromPower(
                    powerOut,
                    measurement,
                    cfg.regulator,
                    cfg.positionTolerance));
        }

        private PowerOutput groupedMotorPower() {
            if (specs.size() == 1) {
                Spec spec = specs.get(0);
                return FtcHardware.motorPower(hw, spec.name, spec.direction);
            }
            List<PowerOutput> outs = new ArrayList<>();
            for (Spec spec : specs) {
                outs.add(FtcHardware.motorPower(hw, spec.name, spec.direction));
            }
            return new GroupedPowerOutput(outs);
        }

        private void requireDefaultGroupScalingForRegulated(String mode) {
            if (specs.size() <= 1) {
                return;
            }
            for (Spec spec : specs) {
                if (Math.abs(spec.scale - 1.0) > 1e-9 || Math.abs(spec.bias) > 1e-9) {
                    throw new IllegalStateException("Regulated motor " + mode + " control requires default "
                            + "group scaling/bias when built through FtcActuators. For non-trivial per-motor "
                            + "scale/bias, build raw outputs manually and compose with Plants."
                            + ("position".equals(mode)
                            ? "positionFromPower(...)"
                            : "velocityFromPower(...)") + '.');
                }
            }
        }

        private void ensureFeedbackScalesNonZero(String mode) {
            for (Spec spec : specs) {
                if (Math.abs(spec.scale) < 1e-9) {
                    throw new IllegalStateException(mode + " requires non-zero per-motor scale so the group "
                            + "measurement can be mapped back into group units.");
                }
            }
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
        public ModifiersStep position() {
            if (specs.size() == 1) {
                MotorBuilder.Spec spec = specs.get(0);
                return new ModifiersStepImpl(Plants.position(FtcHardware.servoPosition(hw, spec.name, spec.direction)));
            }
            MultiPlant.Builder b = MultiPlant.builder();
            for (MotorBuilder.Spec spec : specs) {
                b.add(Plants.position(FtcHardware.servoPosition(hw, spec.name, spec.direction)), spec.scale, spec.bias);
            }
            return new ModifiersStepImpl(b.build());
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
            for (MotorBuilder.Spec spec : specs) {
                b.add(Plants.power(FtcHardware.crServoPower(hw, spec.name, spec.direction)), spec.scale, spec.bias);
            }
            return new ModifiersStepImpl(b.build());
        }

        @Override
        public ModifiersStep position(CrServoPositionControl control) {
            Objects.requireNonNull(control, "control");
            RegulatedCrServoPositionControl cfg = (RegulatedCrServoPositionControl) control;
            requireDefaultGroupScalingForRegulated();
            PowerOutput powerOut = groupedCrServoPower();
            ScalarSource measurement = cfg.feedback.resolve(hw, null);
            return new ModifiersStepImpl(Plants.positionFromPower(
                    powerOut,
                    measurement,
                    cfg.regulator,
                    cfg.positionTolerance));
        }

        private PowerOutput groupedCrServoPower() {
            if (specs.size() == 1) {
                MotorBuilder.Spec spec = specs.get(0);
                return FtcHardware.crServoPower(hw, spec.name, spec.direction);
            }
            List<PowerOutput> outs = new ArrayList<>();
            for (MotorBuilder.Spec spec : specs) {
                outs.add(FtcHardware.crServoPower(hw, spec.name, spec.direction));
            }
            return new GroupedPowerOutput(outs);
        }

        private void requireDefaultGroupScalingForRegulated() {
            if (specs.size() <= 1) {
                return;
            }
            for (MotorBuilder.Spec spec : specs) {
                if (Math.abs(spec.scale - 1.0) > 1e-9 || Math.abs(spec.bias) > 1e-9) {
                    throw new IllegalStateException("Regulated CR-servo position control requires default group "
                            + "scaling/bias when built through FtcActuators. For non-trivial per-servo scale/bias, "
                            + "build raw outputs manually and compose with Plants.positionFromPower(...).");
                }
            }
        }
    }

    private static final class GroupedPowerOutput implements PowerOutput {
        private final List<PowerOutput> outputs;
        private double last;

        private GroupedPowerOutput(List<PowerOutput> outputs) {
            this.outputs = new ArrayList<>(Objects.requireNonNull(outputs, "outputs"));
            if (this.outputs.isEmpty()) {
                throw new IllegalArgumentException("outputs must not be empty");
            }
        }

        @Override
        public void setPower(double power) {
            last = power;
            for (PowerOutput output : outputs) {
                output.setPower(power);
            }
        }

        @Override
        public double getCommandedPower() {
            return last;
        }

        @Override
        public void stop() {
            for (PowerOutput output : outputs) {
                output.stop();
            }
            last = 0.0;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------------------------

    private static void ensureMotorFeedbackAvailable(List<MotorBuilder.Spec> motorSpecs, String domain) {
        if (motorSpecs == null || motorSpecs.isEmpty()) {
            throw new IllegalStateException("Internal " + domain + " encoder feedback requires a motor builder. "
                    + "Use an external encoder or PositionFeedback/VelocityFeedback.fromSource(...) instead.");
        }
    }

    private static ScalarSource averageSources(List<ScalarSource> sources) {
        if (sources == null || sources.isEmpty()) {
            throw new IllegalArgumentException("sources must not be empty");
        }
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
                return count > 0 ? (sum / count) : Double.NaN;
            }

            @Override
            public void reset() {
                for (ScalarSource source : copy) {
                    source.reset();
                }
            }
        }.memoized();
    }

    private static void applyDeviceManagedPositionConfig(DcMotorEx motor,
                                                         String motorName,
                                                         DeviceManagedMotorPositionControl control) {
        try {
            if (control.outerPositionP != null) {
                motor.setPositionPIDFCoefficients(control.outerPositionP);
            }
            if (control.innerVelocityPidf != null) {
                double[] c = control.innerVelocityPidf;
                motor.setVelocityPIDFCoefficients(c[0], c[1], c[2], c[3]);
            }
            if (control.devicePositionToleranceTicks != null) {
                motor.setTargetPositionTolerance(control.devicePositionToleranceTicks);
            }
        } catch (RuntimeException ex) {
            throw new IllegalStateException("Failed to apply device-managed position config for motor '" + motorName
                    + "'. Check that the motor supports the requested FTC SDK APIs.", ex);
        }
    }

    private static void applyDeviceManagedVelocityConfig(DcMotorEx motor,
                                                         String motorName,
                                                         DeviceManagedMotorVelocityControl control) {
        try {
            if (control.velocityPidf != null) {
                double[] c = control.velocityPidf;
                motor.setVelocityPIDFCoefficients(c[0], c[1], c[2], c[3]);
            }
        } catch (RuntimeException ex) {
            throw new IllegalStateException("Failed to apply device-managed velocity config for motor '" + motorName
                    + "'. Check that the motor supports the requested FTC SDK APIs.", ex);
        }
    }
}
