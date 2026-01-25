package edu.ftcphoenix.fw.actuation;

import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;

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
 *   <li><b>Power plants</b>: target is normalized power (typically {@code [-1.0, +1.0]}).</li>
 *   <li><b>Position plants</b>:
 *     <ul>
 *       <li>Servos: {@code 0.0 .. 1.0} (FTC servo position).</li>
 *       <li>Motors: encoder ticks (or other platform-native units).</li>
 *     </ul>
 *   </li>
 *   <li><b>Velocity plants</b>: target is native velocity units (e.g. ticks/sec).</li>
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
     */
    static Plant power(PowerOutput out) {
        if (out == null) {
            throw new IllegalArgumentException("PowerOutput is required");
        }
        return new PowerPlant(out);
    }

    /**
     * Internal {@link Plant} implementation for {@link PowerOutput} channels.
     */
    private static final class PowerPlant implements Plant {
        private final PowerOutput out;
        private double target = 0.0;

        PowerPlant(PowerOutput out) {
            this.out = out;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void setTarget(double target) {
            this.target = target;
            out.setPower(target);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public double getTarget() {
            return target;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void update(double dtSec) {
            // No additional control logic; relies on underlying implementation.
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void stop() {
            out.stop();
            target = 0.0;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean atSetpoint() {
            return true;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "PowerPlant{target=" + target + "}";
        }
    }

    // =====================================================================
    // SERVO POSITION PLANTS (PositionOutput, open-loop set-and-hold)
    // =====================================================================

    /**
     * Servo-style position plant over a {@link PositionOutput} in native units.
     *
     * <p>This variant is "set-and-hold": it commands the target position
     * when {@link Plant#setTarget(double)} is called and immediately considers
     * itself at setpoint (i.e., {@link Plant#atSetpoint()} always returns
     * {@code true}).</p>
     *
     * <p>For encoder-backed motors where you want sensor-based completion and
     * {@link Plant#hasFeedback()}, use {@link #motorPosition(PositionOutput, double)}.
     */
    static Plant servoPosition(PositionOutput out) {
        if (out == null) {
            throw new IllegalArgumentException("PositionOutput is required");
        }
        return new PositionPlant(out);
    }

    /**
     * Internal {@link Plant} implementation for open-loop position channels (e.g., servos).
     */
    private static final class PositionPlant implements Plant {
        private final PositionOutput out;
        private double target = 0.0;

        PositionPlant(PositionOutput out) {
            this.out = out;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void setTarget(double target) {
            this.target = target;
            out.setPosition(target);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public double getTarget() {
            return target;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void update(double dtSec) {
            // No additional control logic; relies on underlying implementation.
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void stop() {
            out.stop();
            // Keep target as-is; this plant is open-loop in terms of atSetpoint().
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean atSetpoint() {
            return true;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "PositionPlant{target=" + target + "}";
        }
    }

    // =====================================================================
    // MOTOR POSITION PLANTS (PositionOutput, feedback + tolerance + reset)
    // =====================================================================

    /**
     * Position plant for an encoder-backed motor in native units (ticks).
     *
     * <p>This variant assumes {@link PositionOutput#getMeasuredPosition()} returns
     * a real sensor reading and uses that feedback to implement
     * {@link Plant#atSetpoint()} with a tolerance.</p>
     *
     * <p>It also maintains an internal offset so that {@link Plant#reset()} can
     * logically re-zero the position at the current measured value without
     * requiring the HAL to reset the encoder itself.</p>
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
     * Internal {@link Plant} implementation for encoder-backed position control with feedback.
     */
    private static final class MotorPositionPlant implements Plant {
        private final PositionOutput out;
        private final double toleranceNative;

        // Plant-frame target (after offset).
        private double target = 0.0;

        // hardware_position = plant_position + offsetNative
        private double offsetNative = 0.0;

        MotorPositionPlant(PositionOutput out, double toleranceNative) {
            this.out = out;
            this.toleranceNative = toleranceNative;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void setTarget(double target) {
            this.target = target;
            out.setPosition(target + offsetNative);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public double getTarget() {
            return target;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void update(double dtSec) {
            // No additional control beyond the underlying motor controller.
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void stop() {
            out.stop();
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void reset() {
            // Re-zero the plant's coordinate frame at the current measured position.
            out.stop();
            offsetNative = out.getMeasuredPosition();
            target = 0.0;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean atSetpoint() {
            double measuredPlantFrame = out.getMeasuredPosition() - offsetNative;
            double error = measuredPlantFrame - target;
            return Math.abs(error) <= toleranceNative;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean hasFeedback() {
            return true;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "MotorPositionPlant{target=" + target +
                    ", offsetNative=" + offsetNative +
                    ", toleranceNative=" + toleranceNative + "}";
        }
    }

    // =====================================================================
    // VELOCITY PLANTS (VelocityOutput, feedback + tolerance)
    // =====================================================================

    /**
     * Velocity plant over a {@link VelocityOutput} in native units.
     *
     * <p>This variant assumes {@link VelocityOutput#getMeasuredVelocity()} returns
     * a real sensor reading and uses that feedback to implement
     * {@link Plant#atSetpoint()} with a tolerance.</p>
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
     * Internal {@link Plant} implementation for encoder-backed velocity control with feedback.
     */
    private static final class VelocityPlant implements Plant {
        private final VelocityOutput out;
        private final double toleranceNative;
        private double target = 0.0;

        VelocityPlant(VelocityOutput out, double toleranceNative) {
            this.out = out;
            this.toleranceNative = toleranceNative;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void setTarget(double target) {
            this.target = target;
            out.setVelocity(target);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public double getTarget() {
            return target;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void update(double dtSec) {
            // No additional control logic; relies on underlying implementation.
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void stop() {
            out.stop();
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean atSetpoint() {
            double error = out.getMeasuredVelocity() - target;
            return Math.abs(error) <= toleranceNative;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean hasFeedback() {
            return true;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "VelocityPlant{target=" + target +
                    ", toleranceNative=" + toleranceNative + "}";
        }
    }
}
