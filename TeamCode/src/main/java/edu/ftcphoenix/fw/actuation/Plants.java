package edu.ftcphoenix.fw.actuation;

import edu.ftcphoenix.fw.hal.PowerOutput;
import edu.ftcphoenix.fw.hal.PositionOutput;
import edu.ftcphoenix.fw.hal.VelocityOutput;

/**
 * Helpers for constructing {@link Plant} instances on top of Phoenix HAL
 * output interfaces using native units.
 *
 * <p>This class sits just above the hardware-abstraction layer:
 * it takes {@link PowerOutput}, {@link PositionOutput}, and
 * {@link VelocityOutput} channels (in their <b>native</b> units) and wraps
 * them in simple {@link Plant} implementations.</p>
 *
 * <h2>Unit conventions</h2>
 *
 * <ul>
 *   <li><b>Power plants</b>:
 *       <ul>
 *         <li>Target: normalized power (typically {@code [-1.0, +1.0]}).</li>
 *       </ul>
 *   </li>
 *   <li><b>Position plants</b>:
 *       <ul>
 *         <li>Servos: {@code 0.0 .. 1.0} (FTC servo position).</li>
 *         <li>Motors: encoder ticks (or other platform-native units).</li>
 *       </ul>
 *   </li>
 *   <li><b>Velocity plants</b>:
 *       <ul>
 *         <li>Target: native velocity units (e.g., ticks/second) as exposed
 *             by the underlying hardware / SDK.</li>
 *       </ul>
 *   </li>
 * </ul>
 *
 * <p>Higher-level code is responsible for converting between these native
 * units and physical units (meters, radians, etc.) where necessary.</p>
 */
public final class Plants {

    private Plants() {
        // utility class; no instances
    }

    // =====================================================================
    // POWER PLANTS (PowerOutput, normalized power)
    // =====================================================================

    /**
     * Power plant over a {@link PowerOutput} in normalized units.
     *
     * <p>Target is interpreted as a normalized power in {@code [-1.0, +1.0]}.
     * The plant does no additional control logic; it simply forwards the
     * command to the underlying {@link PowerOutput}.</p>
     *
     * <p>{@link Plant#atSetpoint()} is always {@code true} for power plants,
     * as there is no meaningful "setpoint" concept without feedback.</p>
     *
     * @param out power channel in normalized units
     * @return a {@link Plant} that commands {@code out} as power
     */
    static Plant power(PowerOutput out) {
        if (out == null) {
            throw new IllegalArgumentException("PowerOutput is required");
        }
        return new PowerPlant(out);
    }

    /**
     * Power plant that drives two {@link PowerOutput}s with the same target
     * power.
     *
     * <p>Typical use: two motors or CR servos that should track the same
     * power command (for example, left/right sides of a mechanism that are
     * mechanically linked).</p>
     *
     * <p>Like {@link #power(PowerOutput)}, this is an open-loop, feedforward
     * plant with no feedback; {@link Plant#atSetpoint()} is always true.</p>
     *
     * @param a first power channel
     * @param b second power channel
     * @return a {@link Plant} that commands both channels with the same power
     */
    static Plant powerPair(PowerOutput a, PowerOutput b) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("Both PowerOutputs are required");
        }
        return new PowerPairPlant(a, b);
    }

    /**
     * Internal implementation for single-output power (normalized).
     */
    private static final class PowerPlant implements Plant {
        private final PowerOutput out;
        private double target = 0.0;

        PowerPlant(PowerOutput out) {
            this.out = out;
        }

        @Override
        public void setTarget(double target) {
            this.target = target;
            out.setPower(target);
        }

        @Override
        public double getTarget() {
            return target;
        }

        @Override
        public void update(double dtSec) {
            // No additional control logic; relies on underlying implementation.
        }

        @Override
        public void stop() {
            out.stop();
            target = 0.0;
        }

        @Override
        public boolean atSetpoint() {
            // Power plants do not have a meaningful setpoint notion.
            return true;
        }

        @Override
        public String toString() {
            return "PowerPlant{target=" + target + "}";
        }
    }

    /**
     * Internal implementation for dual-output power (normalized).
     */
    private static final class PowerPairPlant implements Plant {
        private final PowerOutput a;
        private final PowerOutput b;
        private double target = 0.0;

        PowerPairPlant(PowerOutput a, PowerOutput b) {
            this.a = a;
            this.b = b;
        }

        @Override
        public void setTarget(double target) {
            this.target = target;
            a.setPower(target);
            b.setPower(target);
        }

        @Override
        public double getTarget() {
            return target;
        }

        @Override
        public void update(double dtSec) {
            // No additional control logic; relies on underlying implementation.
        }

        @Override
        public void stop() {
            a.stop();
            b.stop();
            target = 0.0;
        }

        @Override
        public boolean atSetpoint() {
            // Power plants do not have a meaningful setpoint notion.
            return true;
        }

        @Override
        public String toString() {
            return "PowerPairPlant{target=" + target + "}";
        }
    }

    // =====================================================================
    // SERVO POSITION PLANTS (PositionOutput, open-loop set-and-hold)
    // =====================================================================

    /**
     * Servo-style position plant over a {@link PositionOutput} in native units.
     *
     * <p>For standard FTC servos this is typically {@code 0.0 .. 1.0}. For other
     * position-style outputs this may be encoder ticks or another native
     * position unit defined by the adapter.</p>
     *
     * <p>This variant is "set-and-hold": it commands the target position
     * when {@link Plant#setTarget(double)} is called and immediately
     * considers itself at setpoint (i.e., {@link Plant#atSetpoint()} always
     * returns {@code true}). It does not rely on sensor feedback and is
     * appropriate for open-loop or fire-and-forget use cases (such as
     * standard servos).</p>
     *
     * <p>For encoder-backed motors where you want sensor-based completion and
     * {@link Plant#hasFeedback()}, prefer {@link #motorPosition(PositionOutput, double)}
     * instead.</p>
     *
     * @param out position channel in native units
     * @return a {@link Plant} that commands {@code out} in position
     */
    static Plant servoPosition(PositionOutput out) {
        if (out == null) {
            throw new IllegalArgumentException("PositionOutput is required");
        }
        return new PositionPlant(out);
    }

    /**
     * Servo-style position plant that drives two {@link PositionOutput}s with
     * the same target position in native units.
     *
     * <p>This variant is also "set-and-hold" and does not use sensor
     * feedback for {@link Plant#atSetpoint()}.</p>
     *
     * <p>For encoder-backed motor pairs where you want sensor-based completion
     * and {@link Plant#hasFeedback()}, prefer
     * {@link #motorPositionPair(PositionOutput, PositionOutput, double)}
     * instead.</p>
     *
     * @param a first position channel
     * @param b second position channel
     * @return a {@link Plant} that commands both channels
     */
    static Plant servoPositionPair(PositionOutput a, PositionOutput b) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("Both PositionOutputs are required");
        }
        return new PositionPairPlant(a, b);
    }

    /**
     * Internal implementation for single-output position (set-and-hold).
     */
    private static final class PositionPlant implements Plant {
        private final PositionOutput out;
        private double target = 0.0;

        PositionPlant(PositionOutput out) {
            this.out = out;
        }

        @Override
        public void setTarget(double target) {
            this.target = target;
            out.setPosition(target);
        }

        @Override
        public double getTarget() {
            return target;
        }

        @Override
        public void update(double dtSec) {
            // No additional control logic; relies on underlying implementation.
        }

        @Override
        public void stop() {
            out.stop();
            // Keep target as-is; this plant is open-loop in terms of atSetpoint().
        }

        @Override
        public boolean atSetpoint() {
            // Open-loop set-and-hold: treat "commanded" as "reached".
            return true;
        }

        @Override
        public String toString() {
            return "PositionPlant{target=" + target + "}";
        }
    }

    /**
     * Internal implementation for dual-output position (set-and-hold).
     */
    private static final class PositionPairPlant implements Plant {
        private final PositionOutput a;
        private final PositionOutput b;
        private double target = 0.0;

        PositionPairPlant(PositionOutput a, PositionOutput b) {
            this.a = a;
            this.b = b;
        }

        @Override
        public void setTarget(double target) {
            this.target = target;
            a.setPosition(target);
            b.setPosition(target);
        }

        @Override
        public double getTarget() {
            return target;
        }

        @Override
        public void update(double dtSec) {
            // No additional control logic; relies on underlying implementation.
        }

        @Override
        public void stop() {
            a.stop();
            b.stop();
            // Keep target as-is; open-loop semantics.
        }

        @Override
        public boolean atSetpoint() {
            // Open-loop set-and-hold: treat "commanded" as "reached".
            return true;
        }

        @Override
        public String toString() {
            return "PositionPairPlant{target=" + target + "}";
        }
    }

    // =====================================================================
    // MOTOR POSITION PLANTS (PositionOutput, feedback + tolerance + reset)
    // =====================================================================

    /**
     * Position plant for an encoder-backed motor in native units (ticks).
     *
     * <p>This variant assumes that {@link PositionOutput#getMeasuredPosition()}
     * returns a real sensor reading (for example, motor encoder ticks), and
     * uses that feedback to implement {@link Plant#atSetpoint()} with a
     * configurable tolerance.</p>
     *
     * <p>It also maintains an internal <b>offset</b> so that
     * {@link Plant#reset()} can logically re-zero the position at the
     * current measured value without requiring the HAL to reset the encoder
     * itself. After {@code reset()}, the plant's coordinate system is such
     * that the current position is treated as {@code 0}.</p>
     *
     * @param out             position channel in native units (ticks)
     * @param toleranceNative allowed error magnitude in native units
     *                        for {@link Plant#atSetpoint()} to be true
     * @return a feedback-capable {@link Plant} for motor position control
     */
    static Plant motorPosition(PositionOutput out, double toleranceNative) {
        if (out == null) {
            throw new IllegalArgumentException("PositionOutput is required");
        }
        if (toleranceNative < 0.0) {
            throw new IllegalArgumentException("toleranceNative must be >= 0");
        }
        return new MotorPositionPlant(out, toleranceNative);
    }

    /**
     * Position plant for a pair of encoder-backed motors in native units
     * (ticks).
     *
     * <p>This variant commands both position channels with the same target
     * and considers the plant at setpoint when <b>both</b> motors are within
     * the specified tolerance.</p>
     *
     * <p>As with {@link MotorPositionPlant}, this plant maintains internal
     * offsets so that {@link Plant#reset()} can re-zero the coordinate frame
     * without requiring encoder resets.</p>
     *
     * @param a               first position channel in native units
     * @param b               second position channel in native units
     * @param toleranceNative allowed error magnitude in native units
     *                        for {@link Plant#atSetpoint()} to be true
     * @return a feedback-capable {@link Plant} for paired motor position
     */
    static Plant motorPositionPair(PositionOutput a,
                                   PositionOutput b,
                                   double toleranceNative) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("Both PositionOutputs are required");
        }
        if (toleranceNative < 0.0) {
            throw new IllegalArgumentException("toleranceNative must be >= 0");
        }
        return new MotorPositionPairPlant(a, b, toleranceNative);
    }

    /**
     * Internal implementation for single-output motor position with feedback.
     */
    private static final class MotorPositionPlant implements Plant {
        private final PositionOutput out;
        private final double toleranceNative;

        // Plant-frame target (after offset).
        private double target = 0.0;

        // Offset between plant-frame zero and hardware native zero:
        // hardware_position = plant_position + offsetNative
        private double offsetNative = 0.0;

        MotorPositionPlant(PositionOutput out, double toleranceNative) {
            this.out = out;
            this.toleranceNative = toleranceNative;
        }

        @Override
        public void setTarget(double target) {
            this.target = target;
            double hardwareTarget = target + offsetNative;
            out.setPosition(hardwareTarget);
        }

        @Override
        public double getTarget() {
            return target;
        }

        @Override
        public void update(double dtSec) {
            // No additional control beyond the underlying motor controller.
        }

        @Override
        public void stop() {
            out.stop();
        }

        @Override
        public void reset() {
            // Re-zero the plant's coordinate frame at the current measured position.
            out.stop();
            double measuredHardware = out.getMeasuredPosition();
            offsetNative = measuredHardware;
            target = 0.0;
        }

        @Override
        public boolean atSetpoint() {
            double measuredHardware = out.getMeasuredPosition();
            double measuredPlantFrame = measuredHardware - offsetNative;
            double error = measuredPlantFrame - target;
            return Math.abs(error) <= toleranceNative;
        }

        @Override
        public boolean hasFeedback() {
            return true;
        }

        @Override
        public String toString() {
            return "MotorPositionPlant{target=" + target +
                    ", offsetNative=" + offsetNative +
                    ", toleranceNative=" + toleranceNative + "}";
        }
    }

    /**
     * Internal implementation for dual-output motor position with feedback.
     */
    private static final class MotorPositionPairPlant implements Plant {
        private final PositionOutput a;
        private final PositionOutput b;
        private final double toleranceNative;

        // Plant-frame target (after offsets).
        private double target = 0.0;

        // Offsets between plant-frame zero and hardware native zero for each
        // motor: hardware_position = plant_position + offsetNativeX
        private double offsetNativeA = 0.0;
        private double offsetNativeB = 0.0;

        MotorPositionPairPlant(PositionOutput a,
                               PositionOutput b,
                               double toleranceNative) {
            this.a = a;
            this.b = b;
            this.toleranceNative = toleranceNative;
        }

        @Override
        public void setTarget(double target) {
            this.target = target;
            double hardwareTargetA = target + offsetNativeA;
            double hardwareTargetB = target + offsetNativeB;
            a.setPosition(hardwareTargetA);
            b.setPosition(hardwareTargetB);
        }

        @Override
        public double getTarget() {
            return target;
        }

        @Override
        public void update(double dtSec) {
            // No additional control beyond the underlying motor controllers.
        }

        @Override
        public void stop() {
            a.stop();
            b.stop();
        }

        @Override
        public void reset() {
            // Re-zero both motors at their current measured positions.
            a.stop();
            b.stop();
            double measuredA = a.getMeasuredPosition();
            double measuredB = b.getMeasuredPosition();
            offsetNativeA = measuredA;
            offsetNativeB = measuredB;
            target = 0.0;
        }

        @Override
        public boolean atSetpoint() {
            double measuredHardwareA = a.getMeasuredPosition();
            double measuredHardwareB = b.getMeasuredPosition();
            double measuredPlantFrameA = measuredHardwareA - offsetNativeA;
            double measuredPlantFrameB = measuredHardwareB - offsetNativeB;
            double errorA = measuredPlantFrameA - target;
            double errorB = measuredPlantFrameB - target;
            return Math.abs(errorA) <= toleranceNative
                    && Math.abs(errorB) <= toleranceNative;
        }

        @Override
        public boolean hasFeedback() {
            return true;
        }

        @Override
        public String toString() {
            return "MotorPositionPairPlant{target=" + target +
                    ", offsetNativeA=" + offsetNativeA +
                    ", offsetNativeB=" + offsetNativeB +
                    ", toleranceNative=" + toleranceNative + "}";
        }
    }

    // =====================================================================
    // VELOCITY PLANTS (VelocityOutput, feedback + tolerance)
    // =====================================================================

    /**
     * Velocity plant over a {@link VelocityOutput} in native units.
     *
     * <p>This variant assumes that {@link VelocityOutput#getMeasuredVelocity()}
     * returns a real sensor reading (for example, motor velocity in native
     * ticks per second), and uses that feedback to implement
     * {@link Plant#atSetpoint()} with a configurable tolerance.</p>
     *
     * @param out             velocity channel in native units
     * @param toleranceNative allowed error magnitude in native units for
     *                        {@link Plant#atSetpoint()} to be true
     * @return a feedback-capable {@link Plant} for velocity control
     */
    static Plant velocity(VelocityOutput out, double toleranceNative) {
        if (out == null) {
            throw new IllegalArgumentException("VelocityOutput is required");
        }
        if (toleranceNative < 0.0) {
            throw new IllegalArgumentException("toleranceNative must be >= 0");
        }
        return new VelocityPlant(out, toleranceNative);
    }

    /**
     * Velocity plant for a pair of {@link VelocityOutput}s in native units.
     *
     * <p>This variant commands both velocity channels with the same target
     * and considers the plant at setpoint when <b>both</b> are within the
     * specified tolerance.</p>
     *
     * @param a               first velocity channel in native units
     * @param b               second velocity channel in native units
     * @param toleranceNative allowed error magnitude in native units for
     *                        {@link Plant#atSetpoint()} to be true
     * @return a feedback-capable {@link Plant} for paired velocity control
     */
    static Plant velocityPair(VelocityOutput a,
                              VelocityOutput b,
                              double toleranceNative) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("Both VelocityOutputs are required");
        }
        if (toleranceNative < 0.0) {
            throw new IllegalArgumentException("toleranceNative must be >= 0");
        }
        return new VelocityPairPlant(a, b, toleranceNative);
    }

    /**
     * Internal implementation for single-output velocity (native units).
     */
    private static final class VelocityPlant implements Plant {
        private final VelocityOutput out;
        private final double toleranceNative;
        private double target = 0.0;

        VelocityPlant(VelocityOutput out, double toleranceNative) {
            this.out = out;
            this.toleranceNative = toleranceNative;
        }

        @Override
        public void setTarget(double target) {
            this.target = target;
            out.setVelocity(target);
        }

        @Override
        public double getTarget() {
            return target;
        }

        @Override
        public void update(double dtSec) {
            // No additional control logic; relies on underlying implementation.
        }

        @Override
        public void stop() {
            out.stop();
        }

        @Override
        public boolean atSetpoint() {
            double measured = out.getMeasuredVelocity();
            double error = measured - target;
            return Math.abs(error) <= toleranceNative;
        }

        @Override
        public boolean hasFeedback() {
            return true;
        }

        @Override
        public String toString() {
            return "VelocityPlant{target=" + target +
                    ", toleranceNative=" + toleranceNative + "}";
        }
    }

    /**
     * Internal implementation for dual-output velocity (native units).
     */
    private static final class VelocityPairPlant implements Plant {
        private final VelocityOutput a;
        private final VelocityOutput b;
        private final double toleranceNative;
        private double target = 0.0;

        VelocityPairPlant(VelocityOutput a,
                          VelocityOutput b,
                          double toleranceNative) {
            this.a = a;
            this.b = b;
            this.toleranceNative = toleranceNative;
        }

        @Override
        public void setTarget(double target) {
            this.target = target;
            a.setVelocity(target);
            b.setVelocity(target);
        }

        @Override
        public double getTarget() {
            return target;
        }

        @Override
        public void update(double dtSec) {
            // No additional control logic; relies on underlying implementation.
        }

        @Override
        public void stop() {
            a.stop();
            b.stop();
        }

        @Override
        public boolean atSetpoint() {
            double measuredA = a.getMeasuredVelocity();
            double measuredB = b.getMeasuredVelocity();
            double errorA = measuredA - target;
            double errorB = measuredB - target;
            return Math.abs(errorA) <= toleranceNative
                    && Math.abs(errorB) <= toleranceNative;
        }

        @Override
        public boolean hasFeedback() {
            return true;
        }

        @Override
        public String toString() {
            return "VelocityPairPlant{target=" + target +
                    ", toleranceNative=" + toleranceNative + "}";
        }
    }
}
