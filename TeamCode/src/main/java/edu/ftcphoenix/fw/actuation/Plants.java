package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Factories for constructing {@link Plant} implementations from low-level actuator command
 * channels plus optional measurement/regulator ingredients.
 *
 * <p>This class is intentionally ingredient-based. Callers choose the command channel and, when
 * needed, the measurement and control law. Higher-level staged builders (such as
 * {@code FtcActuators}) can then present beginner-friendly defaults on top of these primitives.</p>
 */
public final class Plants {

    private Plants() {
        // utility class
    }

    /**
     * Create a direct power plant over a {@link PowerOutput}.
     *
     * @param out raw power command channel that the returned plant should drive directly
     * @return power plant whose target domain matches the supplied output's power command domain
     */
    public static Plant power(PowerOutput out) {
        Objects.requireNonNull(out, "out");
        return new PowerPlant(out);
    }

    /**
     * Create a commanded-position plant over a {@link PositionOutput} with no authoritative
     * feedback.
     *
     * <p>This is the right fit for standard FTC servos and other set-and-hold outputs where the
     * framework does not have a meaningful measured position.</p>
     *
     * @param out raw position command channel that the returned plant should drive directly
     * @return position plant that reports commanded targets but no authoritative feedback
     */
    public static Plant position(PositionOutput out) {
        Objects.requireNonNull(out, "out");
        return new CommandedPositionPlant(out);
    }

    /**
     * Create a device-managed or otherwise externally regulated position plant over a
     * {@link PositionOutput} plus an authoritative position measurement source.
     *
     * <p>{@code positionTolerance} is the plant-level completion band used by
     * {@link Plant#atSetpoint()}.</p>
     *
     * @param out               raw position command channel that the returned plant should drive
     * @param measurement       authoritative position measurement source in the same units as the plant target
     * @param positionTolerance absolute completion band used for {@link Plant#atSetpoint()}
     * @return feedback-capable position plant backed by the supplied command channel and measurement
     */
    public static Plant position(PositionOutput out,
                                 ScalarSource measurement,
                                 double positionTolerance) {
        Objects.requireNonNull(out, "out");
        Objects.requireNonNull(measurement, "measurement");
        if (positionTolerance < 0.0) {
            throw new IllegalArgumentException("positionTolerance must be >= 0, got " + positionTolerance);
        }
        return new DeviceManagedPositionPlant(out, measurement.memoized(), positionTolerance);
    }

    /**
     * Create a device-managed or otherwise externally regulated velocity plant over a
     * {@link VelocityOutput} plus an authoritative velocity measurement source.
     *
     * @param out               raw velocity command channel that the returned plant should drive
     * @param measurement       authoritative velocity measurement source in the same units as the plant target
     * @param velocityTolerance absolute completion band used for {@link Plant#atSetpoint()}
     * @return feedback-capable velocity plant backed by the supplied command channel and measurement
     */
    public static Plant velocity(VelocityOutput out,
                                 ScalarSource measurement,
                                 double velocityTolerance) {
        Objects.requireNonNull(out, "out");
        Objects.requireNonNull(measurement, "measurement");
        if (velocityTolerance < 0.0) {
            throw new IllegalArgumentException("velocityTolerance must be >= 0, got " + velocityTolerance);
        }
        return new DeviceManagedVelocityPlant(out, measurement.memoized(), velocityTolerance);
    }

    /**
     * Create a framework-regulated position plant that uses raw power as the actuation lever.
     *
     * @param powerOut raw power command channel driven by the returned plant
     * @param measurement authoritative position measurement source in the same units as the plant target
     * @param regulator framework-owned control law that converts position error into power commands
     * @param positionTolerance absolute completion band used for {@link Plant#atSetpoint()}
     * @return feedback-capable regulated position plant that drives power from the supplied measurement
     */
    public static Plant positionFromPower(PowerOutput powerOut,
                                          ScalarSource measurement,
                                          ScalarRegulator regulator,
                                          double positionTolerance) {
        Objects.requireNonNull(powerOut, "powerOut");
        Objects.requireNonNull(measurement, "measurement");
        Objects.requireNonNull(regulator, "regulator");
        if (positionTolerance < 0.0) {
            throw new IllegalArgumentException("positionTolerance must be >= 0, got " + positionTolerance);
        }
        return new RegulatedPositionPlant(powerOut, measurement.memoized(), regulator, positionTolerance);
    }

    /**
     * Create a framework-regulated velocity plant that uses raw power as the actuation lever.
     *
     * @param powerOut          raw power command channel driven by the returned plant
     * @param measurement       authoritative velocity measurement source in the same units as the plant target
     * @param regulator         framework-owned control law that converts velocity error into power commands
     * @param velocityTolerance absolute completion band used for {@link Plant#atSetpoint()}
     * @return feedback-capable regulated velocity plant that drives power from the supplied measurement
     */
    public static Plant velocityFromPower(PowerOutput powerOut,
                                          ScalarSource measurement,
                                          ScalarRegulator regulator,
                                          double velocityTolerance) {
        Objects.requireNonNull(powerOut, "powerOut");
        Objects.requireNonNull(measurement, "measurement");
        Objects.requireNonNull(regulator, "regulator");
        if (velocityTolerance < 0.0) {
            throw new IllegalArgumentException("velocityTolerance must be >= 0, got " + velocityTolerance);
        }
        return new RegulatedVelocityPlant(powerOut, measurement.memoized(), regulator, velocityTolerance);
    }

    private static final class PowerPlant implements Plant {
        private final PowerOutput out;
        private double target;

        private PowerPlant(PowerOutput out) {
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
        public void update(LoopClock clock) {
            // direct command plant; nothing else to do
        }

        @Override
        public void stop() {
            out.stop();
            target = 0.0;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            Plant.super.debugDump(dbg, prefix);
        }
    }

    private static final class CommandedPositionPlant implements Plant {
        private final PositionOutput out;
        private double target;

        private CommandedPositionPlant(PositionOutput out) {
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
        public void update(LoopClock clock) {
            // commanded-position plant; no authoritative feedback
        }

        @Override
        public void stop() {
            out.stop();
        }
    }

    private abstract static class AbstractFeedbackPlant implements Plant {
        private final ScalarSource measurement;
        private final double tolerance;
        private double target;
        private double lastMeasurement = Double.NaN;
        private boolean lastAtSetpoint;

        private AbstractFeedbackPlant(ScalarSource measurement, double tolerance) {
            this.measurement = measurement;
            this.tolerance = tolerance;
        }

        @Override
        public final void setTarget(double target) {
            this.target = target;
            onSetTarget(target);
        }

        @Override
        public final double getTarget() {
            return target;
        }

        @Override
        public final void update(LoopClock clock) {
            lastMeasurement = measurement.getAsDouble(clock);
            onUpdate(clock, lastMeasurement);
            lastAtSetpoint = Double.isFinite(lastMeasurement)
                    && Math.abs(target - lastMeasurement) <= tolerance;
        }

        protected abstract void onSetTarget(double target);

        protected abstract void onUpdate(LoopClock clock, double measurement);

        @Override
        public final boolean atSetpoint() {
            return lastAtSetpoint;
        }

        @Override
        public final boolean hasFeedback() {
            return true;
        }

        @Override
        public final double getMeasurement() {
            return lastMeasurement;
        }

        protected final double tolerance() {
            return tolerance;
        }
    }

    private static final class DeviceManagedPositionPlant extends AbstractFeedbackPlant {
        private final PositionOutput out;

        private DeviceManagedPositionPlant(PositionOutput out, ScalarSource measurement, double tolerance) {
            super(measurement, tolerance);
            this.out = out;
        }

        @Override
        protected void onSetTarget(double target) {
            out.setPosition(target);
        }

        @Override
        protected void onUpdate(LoopClock clock, double measurement) {
            // device owns the loop; measurement is sampled for plant status only
        }

        @Override
        public void stop() {
            out.stop();
        }
    }

    private static final class DeviceManagedVelocityPlant extends AbstractFeedbackPlant {
        private final VelocityOutput out;

        private DeviceManagedVelocityPlant(VelocityOutput out, ScalarSource measurement, double tolerance) {
            super(measurement, tolerance);
            this.out = out;
        }

        @Override
        protected void onSetTarget(double target) {
            out.setVelocity(target);
        }

        @Override
        protected void onUpdate(LoopClock clock, double measurement) {
            // device owns the loop; measurement is sampled for plant status only
        }

        @Override
        public void stop() {
            out.stop();
        }
    }

    private abstract static class AbstractRegulatedPlant extends AbstractFeedbackPlant {
        private final PowerOutput out;
        private final ScalarRegulator regulator;
        private double lastOutput;

        private AbstractRegulatedPlant(PowerOutput out,
                                       ScalarSource measurement,
                                       ScalarRegulator regulator,
                                       double tolerance) {
            super(measurement, tolerance);
            this.out = out;
            this.regulator = regulator;
        }

        @Override
        protected final void onSetTarget(double target) {
            // no immediate action; command happens during update with a fresh measurement
        }

        @Override
        protected final void onUpdate(LoopClock clock, double measurement) {
            lastOutput = regulator.update(getTarget(), measurement, clock);
            out.setPower(lastOutput);
        }

        @Override
        public void reset() {
            regulator.reset();
        }

        @Override
        public void stop() {
            out.stop();
            regulator.reset();
            lastOutput = 0.0;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) {
                return;
            }
            String p = (prefix == null || prefix.isEmpty()) ? "plant" : prefix;
            dbg.addData(p + ".target", getTarget())
                    .addData(p + ".hasFeedback", hasFeedback())
                    .addData(p + ".atSetpoint", atSetpoint())
                    .addData(p + ".measurement", getMeasurement())
                    .addData(p + ".error", getError())
                    .addData(p + ".output", lastOutput);
            regulator.debugDump(dbg, p + ".regulator");
        }
    }

    private static final class RegulatedPositionPlant extends AbstractRegulatedPlant {
        private RegulatedPositionPlant(PowerOutput out,
                                       ScalarSource measurement,
                                       ScalarRegulator regulator,
                                       double tolerance) {
            super(out, measurement, regulator, tolerance);
        }
    }

    private static final class RegulatedVelocityPlant extends AbstractRegulatedPlant {
        private RegulatedVelocityPlant(PowerOutput out,
                                       ScalarSource measurement,
                                       ScalarRegulator regulator,
                                       double tolerance) {
            super(out, measurement, regulator, tolerance);
        }
    }
}
