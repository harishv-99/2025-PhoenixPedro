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
 * Adapters from FTC SDK devices to Phoenix HAL output interfaces.
 *
 * <p>This class is the bridge between the FTC SDK and the framework's
 * hardware-abstraction interfaces:</p>
 *
 * <ul>
 *   <li>{@link PowerOutput} – normalized power (typically [-1, +1])</li>
 *   <li>{@link PositionOutput} – native position units
 *       <ul>
 *         <li>Servos: {@code 0.0 .. 1.0}</li>
 *         <li>Motors: encoder ticks</li>
 *       </ul>
 *   </li>
 *   <li>{@link VelocityOutput} – native velocity units
 *       <ul>
 *         <li>Motors: encoder ticks per second</li>
 *       </ul>
 *   </li>
 * </ul>
 *
 * <p>All direction handling is centralized here, using the FTC SDK's direction APIs:</p>
 *
 * <ul>
 *   <li>{@link DcMotor#setDirection(DcMotorSimple.Direction)}</li>
 *   <li>{@link CRServo#setDirection(com.qualcomm.robotcore.hardware.CRServo.Direction)}</li>
 *   <li>{@link Servo#setDirection(Servo.Direction)}</li>
 * </ul>
 *
 * <p>Framework code and {@code Plant} implementations then treat positive
 * values as "forward" in whatever coordinate system the hardware already
 * uses.</p>
 */
public final class FtcHardware {

    private FtcHardware() {
        // utility class; no instances
    }

    // ----------------------------------------------------------------------
    // POWER OUTPUTS (normalized power [-1, +1])
    // ----------------------------------------------------------------------

    /**
     * Wrap an FTC {@link DcMotorEx} as a normalized {@link PowerOutput}.
     *
     * <p>The logical power input is clamped to {@code [-1.0, +1.0]} and
     * forwarded to {@link DcMotorEx#setPower(double)}.</p>
     *
     * @param hw        hardware map
     * @param name      configured device name
     * @param direction logical direction for this channel
     * @return a {@link PowerOutput} controlling the motor
     */
    public static PowerOutput motorPower(HardwareMap hw,
                                         String name,
                                         Direction direction) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        if (direction == null) {
            throw new IllegalArgumentException("direction is required");
        }

        final DcMotorEx m = hw.get(DcMotorEx.class, name);
        m.setDirection(direction == Direction.REVERSE
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        return new PowerOutput() {
            private double last;

            /** {@inheritDoc} */
            @Override
            public void setPower(double power) {
                double cmd = MathUtil.clampAbs(power, 1.0);
                last = cmd;
                m.setPower(cmd);
            }

            /** {@inheritDoc} */
            @Override
            public double getCommandedPower() {
                return last;
            }
        };
    }

    /**
     * Wrap an FTC {@link CRServo} as a normalized {@link PowerOutput}.
     *
     * <p>The logical power input is clamped to {@code [-1.0, +1.0]} and
     * forwarded to {@link CRServo#setPower(double)}.</p>
     *
     * @param hw        hardware map
     * @param name      configured device name
     * @param direction logical direction for this channel
     * @return a {@link PowerOutput} controlling the continuous rotation servo
     */
    public static PowerOutput crServoPower(HardwareMap hw,
                                           String name,
                                           Direction direction) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        if (direction == null) {
            throw new IllegalArgumentException("direction is required");
        }

        final CRServo s = hw.get(CRServo.class, name);
        s.setDirection(direction == Direction.REVERSE
                ? CRServo.Direction.REVERSE
                : CRServo.Direction.FORWARD);

        return new PowerOutput() {
            private double last;

            /** {@inheritDoc} */
            @Override
            public void setPower(double power) {
                double cmd = MathUtil.clampAbs(power, 1.0);
                last = cmd;
                s.setPower(cmd);
            }

            /** {@inheritDoc} */
            @Override
            public double getCommandedPower() {
                return last;
            }
        };
    }

    // ----------------------------------------------------------------------
    // POSITION OUTPUTS
    //  - Servos: 0..1 (FTC native)
    //  - Motors: encoder ticks
    // ----------------------------------------------------------------------

    /**
     * Wrap an FTC {@link Servo} as a {@link PositionOutput} in the range
     * {@code 0.0 .. 1.0}.
     *
     * <p>The input is clamped to {@code [0.0, 1.0]} and forwarded to
     * {@link Servo#setPosition(double)}.</p>
     *
     * @param hw        hardware map
     * @param name      configured device name
     * @param direction logical direction for this channel
     * @return a {@link PositionOutput} controlling the servo
     */
    public static PositionOutput servoPosition(HardwareMap hw,
                                               String name,
                                               Direction direction) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        if (direction == null) {
            throw new IllegalArgumentException("direction is required");
        }

        final Servo s = hw.get(Servo.class, name);
        s.setDirection(direction == Direction.REVERSE
                ? Servo.Direction.REVERSE
                : Servo.Direction.FORWARD);

        return new PositionOutput() {
            private double last;

            /** {@inheritDoc} */
            @Override
            public void setPosition(double position) {
                double cmd = MathUtil.clamp(position, 0.0, 1.0);
                last = cmd;
                s.setPosition(cmd);
            }

            /** {@inheritDoc} */
            @Override
            public double getCommandedPosition() {
                return last;
            }
        };
    }

    /**
     * Wrap an FTC {@link DcMotorEx} as a {@link PositionOutput} in native
     * encoder ticks.
     *
     * <p>The input is interpreted as an absolute target position in encoder
     * ticks. The implementation uses {@link DcMotor.RunMode#RUN_TO_POSITION}
     * and commands full power ({@code 1.0}) when a new target is set.</p>
     *
     * <p>Higher-level code is responsible for converting to/from physical
     * units (e.g., radians) if needed. The framework standardizes on ticks
     * as the native position unit for motors.</p>
     *
     * @param hw        hardware map
     * @param name      configured device name
     * @param direction logical direction for this channel
     * @return a {@link PositionOutput} controlling the motor in ticks
     */
    public static PositionOutput motorPosition(HardwareMap hw,
                                               String name,
                                               Direction direction) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        if (direction == null) {
            throw new IllegalArgumentException("direction is required");
        }

        final DcMotorEx m = hw.get(DcMotorEx.class, name);
        m.setDirection(direction == Direction.REVERSE
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        // Ensure encoder-based positioning is enabled; reset once at setup.
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        return new PositionOutput() {
            private double lastTicks = 0.0;

            /** {@inheritDoc} */
            @Override
            public void setPosition(double positionTicks) {
                lastTicks = positionTicks;
                int target = (int) Math.round(positionTicks);
                m.setTargetPosition(target);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m.setPower(1.0); // full power; SDK manages PID profile
            }

            /** {@inheritDoc} */
            @Override
            public double getCommandedPosition() {
                return lastTicks;
            }

            /** {@inheritDoc} */
            @Override
            public double getMeasuredPosition() {
                // True encoder-based feedback in native ticks.
                return m.getCurrentPosition();
            }

            /** {@inheritDoc} */
            @Override
            public void stop() {
                // Stop actively driving toward the previous target.
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setPower(0.0);
            }
        };
    }

    // ----------------------------------------------------------------------
    // VELOCITY OUTPUTS
    //  - Motors: encoder ticks per second
    // ----------------------------------------------------------------------

    /**
     * Wrap an FTC {@link DcMotorEx} as a {@link VelocityOutput} in native
     * encoder units (ticks per second).
     *
     * <p>The input is forwarded directly to
     * {@link DcMotorEx#setVelocity(double)} while the motor is in
     * {@link DcMotor.RunMode#RUN_USING_ENCODER}.</p>
     *
     * <p>{@link VelocityOutput#getCommandedVelocity()} returns the last
     * requested velocity (ticks/sec), while
     * {@link VelocityOutput#getMeasuredVelocity()} returns the current
     * sensor-based velocity reading from {@link DcMotorEx#getVelocity()}
     * (also ticks/sec).</p>
     *
     * @param hw        hardware map
     * @param name      configured device name
     * @param direction logical direction for this channel
     * @return a {@link VelocityOutput} controlling the motor velocity
     */
    public static VelocityOutput motorVelocity(HardwareMap hw,
                                               String name,
                                               Direction direction) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        if (direction == null) {
            throw new IllegalArgumentException("direction is required");
        }

        final DcMotorEx m = hw.get(DcMotorEx.class, name);
        m.setDirection(direction == Direction.REVERSE
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return new VelocityOutput() {
            private double commanded = 0.0;

            /** {@inheritDoc} */
            @Override
            public void setVelocity(double velocityTicksPerSec) {
                commanded = velocityTicksPerSec;
                m.setVelocity(velocityTicksPerSec);
            }

            /** {@inheritDoc} */
            @Override
            public double getCommandedVelocity() {
                return commanded;
            }

            /** {@inheritDoc} */
            @Override
            public double getMeasuredVelocity() {
                // Native FTC SDK units, typically ticks per second.
                return m.getVelocity();
            }
        };
    }
}
