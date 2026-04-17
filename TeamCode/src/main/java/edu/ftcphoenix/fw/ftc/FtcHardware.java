package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * FTC-boundary adapters that turn FTC SDK hardware devices into Phoenix command channels.
 *
 * <p>This class intentionally exposes <b>command-only</b> outputs. If you need measurement or
 * feedback, use {@link FtcSensors} to obtain a Phoenix source instead of trying to read back from
 * the command channel itself.</p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * PowerOutput intake = FtcHardware.motorPower(hardwareMap, "intake", Direction.FORWARD);
 * PositionOutput wrist = FtcHardware.servoPosition(hardwareMap, "wrist", Direction.FORWARD);
 * VelocityOutput flywheel = FtcHardware.motorVelocity(hardwareMap, "flywheel", Direction.FORWARD);
 * }</pre>
 */
public final class FtcHardware {

    private FtcHardware() {
        // utility class
    }

    // ---------------------------------------------------------------------------------------------
    // Power outputs
    // ---------------------------------------------------------------------------------------------

    /**
     * Create a Phoenix {@link PowerOutput} for a named FTC motor.
     *
     * <p>The returned output writes through {@link DcMotor#setPower(double)} and reports the most
     * recent commanded power through {@link PowerOutput#getCommandedPower()}.</p>
     *
     * @param hw FTC hardware map used to look up the motor
     * @param name configured hardware name of the {@link DcMotorEx}
     * @param direction logical forward direction for Phoenix commands
     * @return command-only power output for the named motor
     */
    public static PowerOutput motorPower(HardwareMap hw, String name, Direction direction) {
        requireHardwareMap(hw);
        requireName(name);
        requireDirection(direction);
        return motorPower(hw.get(DcMotorEx.class, name), direction);
    }

    /**
     * Create a Phoenix {@link PowerOutput} for an FTC motor instance.
     *
     * <p>The adapter configures the motor direction immediately and clamps every commanded value to
     * {@code [-1.0, +1.0]} before forwarding it to the FTC SDK.</p>
     *
     * @param motor     FTC motor instance to command
     * @param direction logical forward direction for Phoenix commands
     * @return command-only power output backed by {@link DcMotor#setPower(double)}
     */
    public static PowerOutput motorPower(DcMotorEx motor, Direction direction) {
        if (motor == null) {
            throw new IllegalArgumentException("motor is required");
        }
        requireDirection(direction);
        motor.setDirection(direction == Direction.REVERSE
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        return new PowerOutput() {
            private double last;

            @Override
            public void setPower(double power) {
                last = MathUtil.clampAbs(power, 1.0);
                motor.setPower(last);
            }

            @Override
            public double getCommandedPower() {
                return last;
            }
        };
    }

    /**
     * Create a Phoenix {@link PowerOutput} for a named FTC continuous-rotation servo.
     *
     * @param hw        FTC hardware map used to look up the CR servo
     * @param name      configured hardware name of the {@link CRServo}
     * @param direction logical forward direction for Phoenix commands
     * @return command-only power output for the named CR servo
     */
    public static PowerOutput crServoPower(HardwareMap hw, String name, Direction direction) {
        requireHardwareMap(hw);
        requireName(name);
        requireDirection(direction);
        return crServoPower(hw.get(CRServo.class, name), direction);
    }

    /**
     * Create a Phoenix {@link PowerOutput} for an FTC continuous-rotation servo instance.
     *
     * <p>The adapter configures the servo direction immediately and clamps every commanded value to
     * {@code [-1.0, +1.0]} before forwarding it to the FTC SDK.</p>
     *
     * @param servo FTC continuous-rotation servo to command
     * @param direction logical forward direction for Phoenix commands
     * @return command-only power output backed by {@link CRServo#setPower(double)}
     */
    public static PowerOutput crServoPower(CRServo servo, Direction direction) {
        if (servo == null) {
            throw new IllegalArgumentException("servo is required");
        }
        requireDirection(direction);
        servo.setDirection(direction == Direction.REVERSE
                ? CRServo.Direction.REVERSE
                : CRServo.Direction.FORWARD);

        return new PowerOutput() {
            private double last;

            @Override
            public void setPower(double power) {
                last = MathUtil.clampAbs(power, 1.0);
                servo.setPower(last);
            }

            @Override
            public double getCommandedPower() {
                return last;
            }
        };
    }

    // ---------------------------------------------------------------------------------------------
    // Position outputs
    // ---------------------------------------------------------------------------------------------

    /**
     * Create a Phoenix {@link PositionOutput} for a named FTC standard servo.
     *
     * <p>The returned output uses the servo's usual {@code 0.0 .. 1.0} position domain and reports
     * the most recent commanded value through {@link PositionOutput#getCommandedPosition()}.</p>
     *
     * @param hw FTC hardware map used to look up the servo
     * @param name configured hardware name of the {@link Servo}
     * @param direction logical forward direction for Phoenix commands
     * @return command-only position output for the named servo
     */
    public static PositionOutput servoPosition(HardwareMap hw, String name, Direction direction) {
        requireHardwareMap(hw);
        requireName(name);
        requireDirection(direction);
        return servoPosition(hw.get(Servo.class, name), direction);
    }

    /**
     * Create a Phoenix {@link PositionOutput} for an FTC standard servo instance.
     *
     * <p>The adapter configures the servo direction immediately and clamps every commanded position
     * to the servo domain {@code [0.0, 1.0]}.</p>
     *
     * @param servo     FTC standard servo to command
     * @param direction logical forward direction for Phoenix commands
     * @return command-only position output backed by {@link Servo#setPosition(double)}
     */
    public static PositionOutput servoPosition(Servo servo, Direction direction) {
        if (servo == null) {
            throw new IllegalArgumentException("servo is required");
        }
        requireDirection(direction);
        servo.setDirection(direction == Direction.REVERSE
                ? Servo.Direction.REVERSE
                : Servo.Direction.FORWARD);

        return new PositionOutput() {
            private double last;

            @Override
            public void setPosition(double position) {
                last = MathUtil.clamp(position, 0.0, 1.0);
                servo.setPosition(last);
            }

            @Override
            public double getCommandedPosition() {
                return last;
            }
        };
    }

    /**
     * Create a Phoenix {@link PositionOutput} for a named FTC motor using
     * {@link DcMotor.RunMode#RUN_TO_POSITION}.
     *
     * <p>The framework does <b>not</b> reset the encoder automatically. The motor's existing encoder
     * frame is preserved and the caller's target is interpreted directly in native encoder ticks.
     * The default command power is {@code 1.0}; use the overload with {@code maxPower} when you want
     * a different power applied after each new target command.</p>
     *
     * @param hw FTC hardware map used to look up the motor
     * @param name configured hardware name of the {@link DcMotorEx}
     * @param direction logical forward direction for Phoenix commands
     * @return command-only position output backed by {@code RUN_TO_POSITION}
     */
    public static PositionOutput motorPosition(HardwareMap hw,
                                               String name,
                                               Direction direction) {
        return motorPosition(hw, name, direction, 1.0);
    }

    /**
     * Create a Phoenix {@link PositionOutput} for a named FTC motor using
     * {@link DcMotor.RunMode#RUN_TO_POSITION} and an explicit command power.
     *
     * @param hw        FTC hardware map used to look up the motor
     * @param name      configured hardware name of the {@link DcMotorEx}
     * @param direction logical forward direction for Phoenix commands
     * @param maxPower  power Phoenix reapplies after each new target command; typical FTC values are
     *                  in {@code [0.0, 1.0]}
     * @return command-only position output backed by {@code RUN_TO_POSITION}
     */
    public static PositionOutput motorPosition(HardwareMap hw,
                                               String name,
                                               Direction direction,
                                               double maxPower) {
        requireHardwareMap(hw);
        requireName(name);
        requireDirection(direction);
        return motorPosition(hw.get(DcMotorEx.class, name), direction, maxPower);
    }

    /**
     * Create a Phoenix {@link PositionOutput} for an FTC motor instance using
     * {@link DcMotor.RunMode#RUN_TO_POSITION}.
     *
     * <p>Each call to {@link PositionOutput#setPosition(double)} writes the requested target in
     * native encoder ticks, switches the motor into {@code RUN_TO_POSITION}, and reapplies the
     * configured drive power. Calling {@link PositionOutput#stop()} powers the motor down and returns
     * it to {@link DcMotor.RunMode#RUN_USING_ENCODER}.</p>
     *
     * @param motor     FTC motor instance to command
     * @param direction logical forward direction for Phoenix commands
     * @param maxPower  power Phoenix reapplies after each new target command; values are clamped to
     *                  {@code [0.0, 1.0]}
     * @return command-only position output backed by {@code RUN_TO_POSITION}
     */
    public static PositionOutput motorPosition(DcMotorEx motor,
                                               Direction direction,
                                               double maxPower) {
        if (motor == null) {
            throw new IllegalArgumentException("motor is required");
        }
        requireDirection(direction);
        final double power = MathUtil.clampAbs(maxPower, 1.0);
        motor.setDirection(direction == Direction.REVERSE
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        return new PositionOutput() {
            private double last;

            @Override
            public void setPosition(double position) {
                last = position;
                motor.setTargetPosition((int) Math.round(position));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(power);
            }

            @Override
            public double getCommandedPosition() {
                return last;
            }

            @Override
            public void stop() {
                motor.setPower(0.0);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        };
    }

    // ---------------------------------------------------------------------------------------------
    // Velocity outputs
    // ---------------------------------------------------------------------------------------------

    /**
     * Create a Phoenix {@link VelocityOutput} for a named FTC motor using
     * {@link DcMotorEx#setVelocity(double)}.
     *
     * <p>FTC uses native encoder ticks per second for this command path by default.</p>
     *
     * @param hw FTC hardware map used to look up the motor
     * @param name configured hardware name of the {@link DcMotorEx}
     * @param direction logical forward direction for Phoenix commands
     * @return command-only velocity output for the named motor
     */
    public static VelocityOutput motorVelocity(HardwareMap hw,
                                               String name,
                                               Direction direction) {
        requireHardwareMap(hw);
        requireName(name);
        requireDirection(direction);
        return motorVelocity(hw.get(DcMotorEx.class, name), direction);
    }

    /**
     * Create a Phoenix {@link VelocityOutput} for an FTC motor instance using
     * {@link DcMotorEx#setVelocity(double)}.
     *
     * <p>Each call to {@link VelocityOutput#setVelocity(double)} switches the motor to
     * {@link DcMotor.RunMode#RUN_USING_ENCODER} before applying the requested velocity command.</p>
     *
     * @param motor     FTC motor instance to command
     * @param direction logical forward direction for Phoenix commands
     * @return command-only velocity output backed by {@link DcMotorEx#setVelocity(double)}
     */
    public static VelocityOutput motorVelocity(DcMotorEx motor,
                                               Direction direction) {
        if (motor == null) {
            throw new IllegalArgumentException("motor is required");
        }
        requireDirection(direction);
        motor.setDirection(direction == Direction.REVERSE
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        return new VelocityOutput() {
            private double last;

            @Override
            public void setVelocity(double velocity) {
                last = velocity;
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setVelocity(velocity);
            }

            @Override
            public double getCommandedVelocity() {
                return last;
            }

            @Override
            public void stop() {
                motor.setVelocity(0.0);
                motor.setPower(0.0);
            }
        };
    }

    private static void requireHardwareMap(HardwareMap hw) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
    }

    private static void requireName(String name) {
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
    }

    private static void requireDirection(Direction direction) {
        if (direction == null) {
            throw new IllegalArgumentException("direction is required");
        }
    }
}
