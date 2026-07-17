package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

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
     * <p>This factory defines motor power as raw/open-loop normalized power. Construction resolves
     * the motor and configures direction, but does not claim its run mode or write power. Each
     * {@link PowerOutput#setPower(double)} command checks for
     * {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}; when a transition is needed, the adapter commands
     * zero in the current mode, selects and verifies the required mode, and only then submits the
     * requested power. It never resets the encoder.</p>
     *
     * <p>{@link PowerOutput#stop()} writes zero directly without acquiring or restoring a mode. A
     * deliberate handoff must stop the old updater before the new owner begins commanding the
     * motor; this adapter does not make simultaneous writers valid.</p>
     *
     * @param hw FTC hardware map used to look up the motor
     * @param name configured hardware name of the {@link DcMotorEx}
     * @param direction logical forward direction for Phoenix commands
     * @return command-only raw/open-loop power output for the named motor
     */
    public static PowerOutput motorPower(HardwareMap hw, String name, Direction direction) {
        requireHardwareMap(hw);
        requireName(name);
        requireDirection(direction);
        return motorPower(
                hw.get(DcMotorEx.class, name),
                direction,
                "configured motor \"" + name + "\"");
    }

    /**
     * Create a Phoenix {@link PowerOutput} for an FTC motor instance.
     *
     * <p>The adapter configures direction immediately but leaves power and run mode untouched until
     * a command arrives. Every commanded value is clamped to {@code [-1.0, +1.0]}. If the motor is
     * not already in {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}, the command path first writes zero,
     * selects that raw-power mode, verifies it, and then writes the requested value. A failed mode
     * acquisition requests a best-effort zero and reports the motor, observed mode, and required
     * mode while retaining the underlying and suppressed cleanup failures.</p>
     *
     * <p>The adapter never resets encoder position. Its {@link PowerOutput#stop()} implementation
     * writes zero without changing mode, so stopping an inactive output cannot steal mode ownership
     * from another deliberate owner.</p>
     *
     * @param motor     FTC motor instance to command
     * @param direction logical forward direction for Phoenix commands
     * @return command-only raw/open-loop power output backed by {@link DcMotor#setPower(double)}
     */
    public static PowerOutput motorPower(DcMotorEx motor, Direction direction) {
        return motorPower(motor, direction, null);
    }

    private static FtcMotorPowerOutput motorPower(DcMotorEx motor,
                                                  Direction direction,
                                                  String description) {
        if (motor == null) {
            throw new IllegalArgumentException("motor is required");
        }
        requireDirection(direction);
        motor.setDirection(direction == Direction.REVERSE
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        return new FtcMotorPowerOutput(motor, description);
    }

    /**
     * Resolve and coordinate a framework-owned group of named FTC raw-power motors.
     *
     * <p>Every device is resolved before direction is configured. On each complete ordered group
     * command, the first child write preflights every member's raw-power mode before any requested
     * child power is submitted. This package-private factory lets FTC builders preserve that safety
     * invariant without adding a public grouping or run-mode concept.</p>
     */
    static List<PowerOutput> motorPowerGroup(HardwareMap hw,
                                             List<String> names,
                                             List<Direction> directions) {
        requireHardwareMap(hw);
        if (names == null || directions == null || names.isEmpty()
                || names.size() != directions.size()) {
            throw new IllegalArgumentException(
                    "motor names and directions must be non-empty and have matching sizes");
        }

        List<DcMotorEx> motors = new ArrayList<>(names.size());
        for (int i = 0; i < names.size(); i++) {
            requireName(names.get(i));
            requireDirection(directions.get(i));
            motors.add(hw.get(DcMotorEx.class, names.get(i)));
        }

        List<FtcMotorPowerOutput> rawOutputs = new ArrayList<>(motors.size());
        for (int i = 0; i < motors.size(); i++) {
            rawOutputs.add(motorPower(
                    motors.get(i),
                    directions.get(i),
                    "configured motor \"" + names.get(i) + "\""));
        }
        new FtcMotorPowerGroup(rawOutputs);

        List<PowerOutput> outputs = new ArrayList<>(rawOutputs.size());
        outputs.addAll(rawOutputs);
        return outputs;
    }

    /**
     * FTC motor output whose power commands conditionally acquire the SDK's raw-power run mode.
     *
     * <p>The cached command retains the adapter's existing seam-level behavior: it records the
     * clamped request before attempting hardware access. It is not hardware acknowledgement.</p>
     */
    private static final class FtcMotorPowerOutput implements PowerOutput {
        private static final DcMotor.RunMode REQUIRED_MODE =
                DcMotor.RunMode.RUN_WITHOUT_ENCODER;

        private final DcMotorEx motor;
        private final String description;
        private FtcMotorPowerGroup group;
        private double last;

        private FtcMotorPowerOutput(DcMotorEx motor, String description) {
            this.motor = motor;
            this.description = description;
        }

        @Override
        public void setPower(double power) {
            last = MathUtil.clampAbs(power, 1.0);
            if (group != null) {
                group.beforeCommand(this);
            } else {
                acquireRequiredModeIfNeeded();
            }
            motor.setPower(last);
        }

        @Override
        public double getCommandedPower() {
            return last;
        }

        /**
         * Write zero without acquiring or restoring a run mode.
         *
         * <p>This differs deliberately from {@code setPower(0.0)}, which is still a raw-power
         * command and therefore establishes {@link #REQUIRED_MODE} when necessary.</p>
         */
        @Override
        public void stop() {
            if (group != null) {
                group.invalidateBatch();
            }
            last = 0.0;
            motor.setPower(0.0);
        }

        private void acquireRequiredModeIfNeeded() {
            DcMotor.RunMode currentMode = null;
            try {
                currentMode = motor.getMode();
                if (currentMode == REQUIRED_MODE) {
                    return;
                }

                motor.setPower(0.0);
                motor.setMode(REQUIRED_MODE);
                currentMode = motor.getMode();
                if (currentMode != REQUIRED_MODE) {
                    throw new IllegalStateException(
                            "FTC motor reported an incompatible mode after mode selection");
                }
            } catch (RuntimeException primaryFailure) {
                try {
                    motor.setPower(0.0);
                } catch (RuntimeException cleanupFailure) {
                    if (cleanupFailure != primaryFailure) {
                        primaryFailure.addSuppressed(cleanupFailure);
                    }
                }
                throw new IllegalStateException(
                        "Cannot acquire raw motor-power mode: motor=" + description()
                                + ", currentMode=" + modeName(currentMode)
                                + ", requiredMode=" + REQUIRED_MODE,
                        primaryFailure);
            }
        }

        private String description() {
            return description != null ? description : describeMotor(motor);
        }
    }

    /**
     * Coordinates one synchronous ordered fan-out so mode acquisition finishes before motion.
     *
     * <p>The framework-created callers retain the children privately and always issue a complete
     * ordered batch. An unexpected order starts a fresh preflight, so an interrupted batch cannot
     * silently lend stale preparation to a later command.</p>
     */
    private static final class FtcMotorPowerGroup {
        private final List<FtcMotorPowerOutput> outputs;
        private int nextIndex = -1;
        private int remainingInBatch;

        private FtcMotorPowerGroup(List<FtcMotorPowerOutput> outputs) {
            this.outputs = new ArrayList<>(outputs);
            for (FtcMotorPowerOutput output : this.outputs) {
                output.group = this;
            }
        }

        private void beforeCommand(FtcMotorPowerOutput output) {
            int index = outputs.indexOf(output);
            if (index < 0) {
                throw new IllegalStateException(
                        "Raw motor-power output is not a member of its command group");
            }
            if (remainingInBatch == 0 || index != nextIndex) {
                invalidateBatch();
                preflightAll();
                remainingInBatch = outputs.size();
                nextIndex = index;
            }

            remainingInBatch--;
            nextIndex = (index + 1) % outputs.size();
        }

        private void preflightAll() {
            try {
                for (FtcMotorPowerOutput output : outputs) {
                    output.acquireRequiredModeIfNeeded();
                }
            } catch (RuntimeException primaryFailure) {
                for (FtcMotorPowerOutput output : outputs) {
                    try {
                        output.motor.setPower(0.0);
                    } catch (RuntimeException cleanupFailure) {
                        if (cleanupFailure != primaryFailure) {
                            primaryFailure.addSuppressed(cleanupFailure);
                        }
                    }
                }
                throw new IllegalStateException(
                        "Cannot prepare raw motor-power group for "
                                + DcMotor.RunMode.RUN_WITHOUT_ENCODER
                                + " before requested power fan-out",
                        primaryFailure);
            }
        }

        private void invalidateBatch() {
            remainingInBatch = 0;
            nextIndex = -1;
        }
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

    private static String modeName(DcMotor.RunMode mode) {
        return mode == null ? "unknown" : mode.name();
    }

    private static String describeMotor(DcMotorEx motor) {
        try {
            String connectionInfo = motor.getConnectionInfo();
            if (connectionInfo != null && !connectionInfo.trim().isEmpty()) {
                return connectionInfo;
            }
        } catch (RuntimeException ignored) {
            // Failure reporting must not replace the mode-acquisition failure.
        }
        return motor.getClass().getSimpleName();
    }
}
