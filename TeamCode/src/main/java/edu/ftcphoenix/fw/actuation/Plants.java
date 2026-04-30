package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Ingredient-level factories for source-driven {@link Plant} implementations.
 *
 * <p>Most FTC robot code should use the staged {@code FtcActuators.plant(...)} builder. These
 * factories are the lower-level boundary for custom hardware adapters and tests. All factories
 * normalize their target input into a {@link PlantTargetSource}; plain scalar sources are treated as
 * exact targets.</p>
 */
public final class Plants {

    private Plants() {
    }

    /**
     * Create a direct power plant from a plain exact scalar source.
     */
    public static Plant power(PowerOutput out, ScalarSource target) {
        return power(out, PlantTargets.exact(target), target instanceof ScalarTarget ? (ScalarTarget) target : null, PlantTargetGuards.none());
    }

    /**
     * Create a direct power plant from a plant-aware target source.
     */
    public static Plant power(PowerOutput out, PlantTargetSource target) {
        return power(out, target, null, PlantTargetGuards.none());
    }

    /**
     * Create a direct power plant with an optional registered writable target and target guards.
     */
    public static Plant power(PowerOutput out, PlantTargetSource target, ScalarTarget writable, PlantTargetGuards guards) {
        return new PowerPlant(out, target, writable, guards);
    }

    /**
     * Create a commanded-position plant with no authoritative feedback from a plain exact scalar source.
     */
    public static Plant position(PositionOutput out, ScalarSource target) {
        return position(out, PlantTargets.exact(target), target instanceof ScalarTarget ? (ScalarTarget) target : null, PlantTargetGuards.none());
    }

    /**
     * Create a commanded-position plant with no authoritative feedback from a plant-aware target source.
     */
    public static Plant position(PositionOutput out, PlantTargetSource target) {
        return position(out, target, null, PlantTargetGuards.none());
    }

    /**
     * Create a commanded-position plant with optional guards.
     */
    public static Plant position(PositionOutput out, PlantTargetSource target, ScalarTarget writable, PlantTargetGuards guards) {
        return new CommandedPositionPlant(out, target, writable, guards);
    }

    /**
     * Create a device-managed position plant with feedback.
     */
    public static Plant position(PositionOutput out,
                                 PlantTargetSource target,
                                 ScalarTarget writable,
                                 PlantTargetGuards guards,
                                 ScalarSource measurement,
                                 double positionTolerance) {
        return new DeviceManagedPositionPlant(out, target, writable, guards, measurement, positionTolerance);
    }

    /**
     * Create a device-managed velocity plant with feedback.
     */
    public static Plant velocity(VelocityOutput out,
                                 PlantTargetSource target,
                                 ScalarTarget writable,
                                 PlantTargetGuards guards,
                                 ScalarSource measurement,
                                 double velocityTolerance) {
        return new DeviceManagedVelocityPlant(out, target, writable, guards, measurement, velocityTolerance);
    }

    /**
     * Create a framework-regulated position plant that drives raw power.
     */
    public static Plant positionFromPower(PowerOutput powerOut,
                                          PlantTargetSource target,
                                          ScalarTarget writable,
                                          PlantTargetGuards guards,
                                          ScalarSource measurement,
                                          ScalarRegulator regulator,
                                          double positionTolerance) {
        return new RegulatedPositionPlant(powerOut, target, writable, guards, measurement, regulator, positionTolerance);
    }

    /**
     * Create a framework-regulated velocity plant that drives raw power.
     */
    public static Plant velocityFromPower(PowerOutput powerOut,
                                          PlantTargetSource target,
                                          ScalarTarget writable,
                                          PlantTargetGuards guards,
                                          ScalarSource measurement,
                                          ScalarRegulator regulator,
                                          double velocityTolerance) {
        return new RegulatedVelocityPlant(powerOut, target, writable, guards, measurement, regulator, velocityTolerance);
    }

    private abstract static class AbstractSourceDrivenPlant implements Plant {
        private final PlantTargetSource targetSource;
        private final ScalarTarget writableTarget;
        private final PlantTargetGuards guards;

        private double requestedTarget = Double.NaN;
        private double appliedTarget;
        private PlantTargetStatus targetStatus = PlantTargetStatus.STOPPED;
        private PlantTargetPlan targetPlan = PlantTargetPlan.unavailable("not sampled");

        AbstractSourceDrivenPlant(PlantTargetSource targetSource, ScalarTarget writableTarget, PlantTargetGuards guards) {
            this.targetSource = Objects.requireNonNull(targetSource, "targetSource");
            this.writableTarget = writableTarget;
            this.guards = guards == null ? PlantTargetGuards.none() : guards;
        }

        @Override
        public final void update(LoopClock clock) {
            prepareTargetContext(clock);
            PlantTargetContext context = targetContext(clock);
            targetPlan = targetSource.resolve(context, clock);
            if (targetPlan != null && targetPlan.hasTarget()) {
                requestedTarget = targetPlan.target();
            } else {
                requestedTarget = appliedTarget;
            }

            double candidate = sanitizeRequestedTarget(requestedTarget);
            PlantTargetStatus status;
            if (targetPlan == null || !targetPlan.hasTarget()) {
                status = PlantTargetStatus.targetUnavailable(targetPlan != null ? targetPlan.reason() : "missing plant target plan");
                candidate = appliedTarget;
            } else {
                status = candidate == requestedTarget
                        ? PlantTargetStatus.ACCEPTED
                        : PlantTargetStatus.clampedToRange("target sanitized to finite value");
            }

            PlantTargetGuards.Result result = guards.apply(candidate, status, appliedTarget, clock);
            appliedTarget = result.target;
            targetStatus = result.status;
            applyTarget(appliedTarget, clock);
            updateStatus(clock);
        }

        protected void prepareTargetContext(LoopClock clock) {
        }

        protected PlantTargetContext targetContext(LoopClock clock) {
            return PlantTargetContext.simple(false, Double.NaN, ScalarRange.unbounded(), requestedTarget, appliedTarget);
        }

        protected double sanitizeRequestedTarget(double request) {
            return Double.isFinite(request) ? request : 0.0;
        }

        protected abstract void applyTarget(double target, LoopClock clock);

        protected void updateStatus(LoopClock clock) {
        }

        @Override
        public final double getRequestedTarget() {
            return requestedTarget;
        }

        @Override
        public final double getAppliedTarget() {
            return appliedTarget;
        }

        @Override
        public final PlantTargetStatus getTargetStatus() {
            return targetStatus;
        }

        @Override
        public final boolean hasWritableTarget() {
            return writableTarget != null;
        }

        @Override
        public final ScalarTarget writableTarget() {
            if (writableTarget == null) return Plant.super.writableTarget();
            return writableTarget;
        }

        @Override
        public void reset() {
            targetSource.reset();
            guards.reset();
            requestedTarget = Double.NaN;
            appliedTarget = 0.0;
            targetStatus = PlantTargetStatus.STOPPED;
            targetPlan = PlantTargetPlan.unavailable("not sampled");
        }

        protected final void markStopped(double appliedAfterStop) {
            appliedTarget = appliedAfterStop;
            targetStatus = PlantTargetStatus.STOPPED;
        }

        /**
         * Reset dynamic guard state after a hard stop so later updates start from a clean guard chain.
         */
        protected final void resetTargetGuards() {
            guards.reset();
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            Plant.super.debugDump(dbg, prefix);
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "plant" : prefix;
            dbg.addData(p + ".targetPlan", targetPlan);
            targetSource.debugDump(dbg, p + ".targetSource");
            guards.debugDump(dbg, p + ".targetGuards");
        }
    }

    private static final class PowerPlant extends AbstractSourceDrivenPlant {
        private final PowerOutput out;

        PowerPlant(PowerOutput out, PlantTargetSource target, ScalarTarget writable, PlantTargetGuards guards) {
            super(target, writable, guards);
            this.out = Objects.requireNonNull(out, "out");
        }

        @Override
        protected void applyTarget(double target, LoopClock clock) {
            out.setPower(target);
        }

        @Override
        public void stop() {
            out.stop();
            resetTargetGuards();
            markStopped(0.0);
        }
    }

    private static final class CommandedPositionPlant extends AbstractSourceDrivenPlant {
        private final PositionOutput out;

        CommandedPositionPlant(PositionOutput out, PlantTargetSource target, ScalarTarget writable, PlantTargetGuards guards) {
            super(target, writable, guards);
            this.out = Objects.requireNonNull(out, "out");
        }

        @Override
        protected void applyTarget(double target, LoopClock clock) {
            out.setPosition(target);
        }

        @Override
        public void stop() {
            out.stop();
            resetTargetGuards();
            markStopped(getAppliedTarget());
        }
    }

    private abstract static class AbstractFeedbackPlant extends AbstractSourceDrivenPlant {
        private final ScalarSource measurement;
        private final double tolerance;
        private double lastMeasurement = Double.NaN;
        private boolean lastAtTarget;

        AbstractFeedbackPlant(PlantTargetSource target,
                              ScalarTarget writable,
                              PlantTargetGuards guards,
                              ScalarSource measurement,
                              double tolerance) {
            super(target, writable, guards);
            this.measurement = Objects.requireNonNull(measurement, "measurement").memoized();
            if (tolerance < 0.0 || !Double.isFinite(tolerance)) {
                throw new IllegalArgumentException("tolerance must be finite and >= 0");
            }
            this.tolerance = tolerance;
        }

        @Override
        protected final void prepareTargetContext(LoopClock clock) {
            lastMeasurement = measurement.getAsDouble(clock);
        }

        @Override
        protected PlantTargetContext targetContext(LoopClock clock) {
            return PlantTargetContext.simple(true, lastMeasurement, ScalarRange.unbounded(), getRequestedTarget(), getAppliedTarget());
        }

        @Override
        protected final void updateStatus(LoopClock clock) {
            onFeedbackUpdate(clock, lastMeasurement);
            lastAtTarget = atTarget(getRequestedTarget());
        }

        protected void onFeedbackUpdate(LoopClock clock, double measurement) {
        }

        @Override
        public final boolean hasFeedback() {
            return true;
        }

        @Override
        public final double getMeasurement() {
            return lastMeasurement;
        }

        @Override
        public final boolean atTarget() {
            return lastAtTarget;
        }

        @Override
        public final boolean atTarget(double target) {
            return Double.isFinite(lastMeasurement)
                    && Math.abs(getRequestedTarget() - target) <= tolerance
                    && Math.abs(getAppliedTarget() - target) <= tolerance
                    && Math.abs(lastMeasurement - target) <= tolerance
                    && getTargetStatus().kind() == PlantTargetStatus.Kind.ACCEPTED;
        }

        @Override
        public void reset() {
            super.reset();
            measurement.reset();
            lastMeasurement = Double.NaN;
            lastAtTarget = false;
        }
    }

    private static final class DeviceManagedPositionPlant extends AbstractFeedbackPlant {
        private final PositionOutput out;

        DeviceManagedPositionPlant(PositionOutput out, PlantTargetSource target, ScalarTarget writable,
                                   PlantTargetGuards guards, ScalarSource measurement, double tolerance) {
            super(target, writable, guards, measurement, tolerance);
            this.out = Objects.requireNonNull(out, "out");
        }

        @Override
        protected void applyTarget(double target, LoopClock clock) {
            out.setPosition(target);
        }

        @Override
        public void stop() {
            out.stop();
            resetTargetGuards();
            markStopped(getAppliedTarget());
        }
    }

    private static final class DeviceManagedVelocityPlant extends AbstractFeedbackPlant {
        private final VelocityOutput out;

        DeviceManagedVelocityPlant(VelocityOutput out, PlantTargetSource target, ScalarTarget writable,
                                   PlantTargetGuards guards, ScalarSource measurement, double tolerance) {
            super(target, writable, guards, measurement, tolerance);
            this.out = Objects.requireNonNull(out, "out");
        }

        @Override
        protected void applyTarget(double target, LoopClock clock) {
            out.setVelocity(target);
        }

        @Override
        public void stop() {
            out.stop();
            resetTargetGuards();
            markStopped(0.0);
        }
    }

    private abstract static class AbstractRegulatedPlant extends AbstractFeedbackPlant {
        private final PowerOutput out;
        private final ScalarRegulator regulator;
        private double lastOutput;

        AbstractRegulatedPlant(PowerOutput out, PlantTargetSource target, ScalarTarget writable,
                               PlantTargetGuards guards, ScalarSource measurement,
                               ScalarRegulator regulator, double tolerance) {
            super(target, writable, guards, measurement, tolerance);
            this.out = Objects.requireNonNull(out, "out");
            this.regulator = Objects.requireNonNull(regulator, "regulator");
        }

        @Override
        protected final void applyTarget(double target, LoopClock clock) {
            // The regulator needs the same-loop measurement, so power is written in onFeedbackUpdate.
        }

        @Override
        protected final void onFeedbackUpdate(LoopClock clock, double measurement) {
            lastOutput = regulator.update(getAppliedTarget(), measurement, clock);
            out.setPower(lastOutput);
        }

        @Override
        public void reset() {
            super.reset();
            regulator.reset();
            lastOutput = 0.0;
        }

        @Override
        public void stop() {
            out.stop();
            regulator.reset();
            resetTargetGuards();
            lastOutput = 0.0;
            markStopped(0.0);
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            super.debugDump(dbg, prefix);
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "plant" : prefix;
            dbg.addData(p + ".output", lastOutput);
            regulator.debugDump(dbg, p + ".regulator");
        }
    }

    private static final class RegulatedPositionPlant extends AbstractRegulatedPlant {
        RegulatedPositionPlant(PowerOutput out, PlantTargetSource target, ScalarTarget writable,
                               PlantTargetGuards guards, ScalarSource measurement,
                               ScalarRegulator regulator, double tolerance) {
            super(out, target, writable, guards, measurement, regulator, tolerance);
        }
    }

    private static final class RegulatedVelocityPlant extends AbstractRegulatedPlant {
        RegulatedVelocityPlant(PowerOutput out, PlantTargetSource target, ScalarTarget writable,
                               PlantTargetGuards guards, ScalarSource measurement,
                               ScalarRegulator regulator, double tolerance) {
            super(out, target, writable, guards, measurement, regulator, tolerance);
        }
    }
}
