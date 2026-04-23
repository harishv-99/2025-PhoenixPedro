package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Velocity {@link Plant} adapter that presents a caller-facing plant-unit velocity coordinate while
 * driving a native velocity output or a framework-regulated power output.
 *
 * <p>Robot code should normally build these through {@code FtcActuators}. This class exists as the
 * advanced boundary adapter for custom hardware integrations.</p>
 *
 * <h2>Units</h2>
 *
 * <p>All public plant APIs use <b>plant velocity units</b>: {@link #setTarget(double)},
 * {@link #getTarget()}, {@link #getMeasurement()}, {@link #getError()}, and the configured target
 * range/tolerance. The native feedback and native velocity output use hardware/controller units such
 * as encoder ticks per second.</p>
 *
 * <p>The plant/native map is intentionally zero-preserving:</p>
 *
 * <pre>{@code
 * nativeVelocity = nativePerPlantUnit * plantVelocity
 * }</pre>
 *
 * <p>Velocity maps deliberately have no offset. Plant velocity {@code 0.0} maps to native velocity
 * {@code 0.0} so stop semantics stay obvious in every unit system.</p>
 */
public final class MappedVelocityPlant implements Plant {

    private final VelocityOutput velocityOut;
    private final PowerOutput regulatedPowerOut;
    private final ScalarRegulator regulator;
    private final ScalarSource nativeMeasurement;
    private final ScalarRange configuredRange;
    private final double nativePerPlantUnit;
    private final double tolerance;

    private double target;
    private double desiredTarget;
    private double lastMeasurement = Double.NaN;
    private double lastNativeMeasurement = Double.NaN;
    private boolean lastAtSetpoint;
    private double lastRegulatorOutput;
    private String lastBlockReason = "";

    private MappedVelocityPlant(VelocityOutput velocityOut,
                                PowerOutput regulatedPowerOut,
                                ScalarRegulator regulator,
                                ScalarSource nativeMeasurement,
                                ScalarRange configuredRange,
                                double nativePerPlantUnit,
                                double tolerance) {
        this.velocityOut = velocityOut;
        this.regulatedPowerOut = regulatedPowerOut;
        this.regulator = regulator;
        this.nativeMeasurement = Objects.requireNonNull(nativeMeasurement, "nativeMeasurement").memoized();
        this.configuredRange = Objects.requireNonNull(configuredRange, "configuredRange");
        this.nativePerPlantUnit = nativePerPlantUnit;
        this.tolerance = tolerance;
        validate();
    }

    /**
     * Starts configuring a device-managed native velocity-output plant.
     *
     * @param out               native velocity command channel
     * @param nativeMeasurement authoritative native velocity measurement source
     */
    public static Builder velocityOutput(VelocityOutput out, ScalarSource nativeMeasurement) {
        return new Builder(Objects.requireNonNull(out, "out"), null, null,
                Objects.requireNonNull(nativeMeasurement, "nativeMeasurement"));
    }

    /**
     * Starts configuring a framework-regulated velocity plant that drives raw power.
     *
     * <p>The regulator receives setpoint and measurement in plant velocity units. The native
     * measurement is converted into plant units before the regulator is updated.</p>
     */
    public static Builder regulated(PowerOutput powerOut,
                                    ScalarSource nativeMeasurement,
                                    ScalarRegulator regulator) {
        return new Builder(null,
                Objects.requireNonNull(powerOut, "powerOut"),
                Objects.requireNonNull(regulator, "regulator"),
                Objects.requireNonNull(nativeMeasurement, "nativeMeasurement"));
    }

    /**
     * Builder for advanced velocity boundary adapters. Robot code should normally use
     * {@code FtcActuators} so the staged FTC builder can ask the required conceptual questions.
     */
    public static final class Builder {
        private final VelocityOutput velocityOut;
        private final PowerOutput regulatedPowerOut;
        private final ScalarRegulator regulator;
        private final ScalarSource nativeMeasurement;
        private ScalarRange configuredRange = ScalarRange.unbounded();
        private double nativePerPlantUnit = 1.0;
        private double tolerance = 100.0;

        private Builder(VelocityOutput velocityOut,
                        PowerOutput regulatedPowerOut,
                        ScalarRegulator regulator,
                        ScalarSource nativeMeasurement) {
            this.velocityOut = velocityOut;
            this.regulatedPowerOut = regulatedPowerOut;
            this.regulator = regulator;
            this.nativeMeasurement = nativeMeasurement;
        }

        /**
         * Sets the legal target range in plant velocity units.
         */
        public Builder range(ScalarRange range) {
            this.configuredRange = Objects.requireNonNull(range, "range");
            return this;
        }

        /**
         * Sets how many native velocity units correspond to one plant velocity unit.
         *
         * <p>This is a zero-preserving scale only; velocity plants intentionally do not support a
         * native offset.</p>
         */
        public Builder nativePerPlantUnit(double nativePerPlantUnit) {
            if (!Double.isFinite(nativePerPlantUnit) || Math.abs(nativePerPlantUnit) < 1.0e-12) {
                throw new IllegalArgumentException("nativePerPlantUnit must be finite and non-zero");
            }
            this.nativePerPlantUnit = nativePerPlantUnit;
            return this;
        }

        /**
         * Sets the plant-level completion tolerance in plant velocity units.
         */
        public Builder velocityTolerance(double tolerance) {
            if (tolerance < 0.0 || !Double.isFinite(tolerance)) {
                throw new IllegalArgumentException("velocityTolerance must be finite and >= 0");
            }
            this.tolerance = tolerance;
            return this;
        }

        /**
         * Builds the mapped velocity plant.
         */
        public MappedVelocityPlant build() {
            return new MappedVelocityPlant(velocityOut,
                    regulatedPowerOut,
                    regulator,
                    nativeMeasurement,
                    configuredRange,
                    nativePerPlantUnit,
                    tolerance);
        }
    }

    private void validate() {
        if (velocityOut == null && regulatedPowerOut == null) {
            throw new IllegalStateException("MappedVelocityPlant requires either a velocity output or regulated power output");
        }
        if (velocityOut != null && regulatedPowerOut != null) {
            throw new IllegalStateException("MappedVelocityPlant cannot use both velocity output and regulated power output");
        }
        if (regulatedPowerOut != null && regulator == null) {
            throw new IllegalStateException("Regulated velocity plants require a regulator");
        }
        if (!Double.isFinite(nativePerPlantUnit) || Math.abs(nativePerPlantUnit) < 1.0e-12) {
            throw new IllegalArgumentException("nativePerPlantUnit must be finite and non-zero");
        }
        if (tolerance < 0.0 || !Double.isFinite(tolerance)) {
            throw new IllegalArgumentException("velocityTolerance must be finite and >= 0");
        }
    }

    @Override
    public void setTarget(double target) {
        desiredTarget = target;
        ScalarRange range = targetRange();
        if (!range.valid) {
            lastBlockReason = range.reason;
            return;
        }
        double applied = range.clamp(target);
        this.target = applied;
        lastBlockReason = applied == target ? "" : "target clamped to plant range";
        if (velocityOut != null) {
            velocityOut.setVelocity(toNative(applied));
        }
    }

    @Override
    public double getTarget() {
        return target;
    }

    /**
     * Returns the latest requested target before range clamping.
     */
    public double getDesiredTarget() {
        return desiredTarget;
    }

    /**
     * Returns the legal velocity target range in plant units.
     */
    public ScalarRange targetRange() {
        return configuredRange;
    }

    @Override
    public void update(LoopClock clock) {
        if (clock == null) {
            return;
        }
        samplePlantMeasurement(clock);
        if (regulatedPowerOut != null) {
            lastRegulatorOutput = regulator.update(target, lastMeasurement, clock);
            regulatedPowerOut.setPower(lastRegulatorOutput);
        }
        lastAtSetpoint = Double.isFinite(lastMeasurement) && Math.abs(target - lastMeasurement) <= tolerance;
    }

    @Override
    public void reset() {
        nativeMeasurement.reset();
        if (regulator != null) {
            regulator.reset();
        }
        lastMeasurement = Double.NaN;
        lastNativeMeasurement = Double.NaN;
        lastAtSetpoint = false;
        lastRegulatorOutput = 0.0;
    }

    @Override
    public void stop() {
        if (velocityOut != null) {
            velocityOut.stop();
        }
        if (regulatedPowerOut != null) {
            regulatedPowerOut.stop();
        }
        if (regulator != null) {
            regulator.reset();
        }
        target = 0.0;
        desiredTarget = 0.0;
        lastRegulatorOutput = 0.0;
    }

    @Override
    public boolean atSetpoint() {
        return lastAtSetpoint;
    }

    @Override
    public boolean hasFeedback() {
        return true;
    }

    @Override
    public double getMeasurement() {
        return lastMeasurement;
    }

    private void samplePlantMeasurement(LoopClock clock) {
        lastNativeMeasurement = nativeMeasurement.getAsDouble(clock);
        lastMeasurement = Double.isFinite(lastNativeMeasurement) ? toPlant(lastNativeMeasurement) : Double.NaN;
    }

    private double toNative(double plantVelocity) {
        return plantVelocity * nativePerPlantUnit;
    }

    private double toPlant(double nativeVelocity) {
        return nativeVelocity / nativePerPlantUnit;
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "velocityPlant" : prefix;
        dbg.addData(p + ".target", getTarget())
                .addData(p + ".desiredTarget", desiredTarget)
                .addData(p + ".measurement", getMeasurement())
                .addData(p + ".error", getError())
                .addData(p + ".atSetpoint", atSetpoint())
                .addData(p + ".hasFeedback", hasFeedback())
                .addData(p + ".range", targetRange())
                .addData(p + ".nativePerPlantUnit", nativePerPlantUnit)
                .addData(p + ".lastNativeMeasurement", lastNativeMeasurement)
                .addData(p + ".lastRegulatorOutput", lastRegulatorOutput)
                .addData(p + ".lastBlockReason", lastBlockReason);
        if (regulator != null) {
            regulator.debugDump(dbg, p + ".regulator");
        }
    }
}
