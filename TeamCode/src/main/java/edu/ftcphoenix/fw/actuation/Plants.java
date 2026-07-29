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
 * normalize their target input into a {@link PlantTargetResolver}. The concise overloads accept a
 * writable {@link ScalarTarget}; read-only scalar sources cross into Plant target space explicitly
 * through {@link PlantTargets#exact(ScalarSource)}. A Plant's optional command target is derived
 * from that final graph's exact resolver or stable overlay base; callers never register a second,
 * potentially disconnected target. Their shared update path enforces a final finite target after
 * dynamic guards.</p>
 */
public final class Plants {

    private static final ScalarRange NORMALIZED_POWER_RANGE = ScalarRange.bounded(-1.0, 1.0);

    private Plants() {
    }

    /**
     * Create a direct normalized-power plant from a writable command target.
     * Requests are constrained to {@code [-1.0, +1.0]} before reaching the output.
     */
    public static Plant power(PowerOutput out, ScalarTarget target) {
        return power(out, PlantTargets.exact(target), PlantTargetGuards.none());
    }

    /**
     * Create a direct normalized-power plant from a plant-aware target resolver.
     * The resolver sees {@code [-1.0, +1.0]} as the Plant's legal target range.
     */
    public static Plant power(PowerOutput out, PlantTargetResolver targetResolver) {
        return power(out, targetResolver, PlantTargetGuards.none());
    }

    /**
     * Create a direct normalized-power plant with target guards. Static guard fallbacks must lie
     * inside {@code [-1.0, +1.0]}. Any command target is derived from the final target graph.
     */
    public static Plant power(PowerOutput out,
                              PlantTargetResolver targetResolver,
                              PlantTargetGuards guards) {
        PlantTargetGuards actualGuards = guards == null ? PlantTargetGuards.none() : guards;
        actualGuards.validateFallbackTargets(NORMALIZED_POWER_RANGE, "PowerPlant");
        return new PowerPlant(out, targetResolver, actualGuards);
    }

    /**
     * Create a commanded-position plant with no authoritative feedback from a writable command
     * target.
     */
    public static Plant position(PositionOutput out, ScalarTarget target) {
        return position(out, PlantTargets.exact(target), PlantTargetGuards.none());
    }

    /**
     * Create a commanded-position plant with no authoritative feedback from a plant-aware target resolver.
     */
    public static Plant position(PositionOutput out, PlantTargetResolver targetResolver) {
        return position(out, targetResolver, PlantTargetGuards.none());
    }

    /**
     * Create a commanded-position plant with optional guards.
     */
    public static Plant position(PositionOutput out,
                                 PlantTargetResolver targetResolver,
                                 PlantTargetGuards guards) {
        return new CommandedPositionPlant(out, targetResolver, guards);
    }

    /**
     * Create a device-managed position plant with feedback.
     * {@code positionTolerance} uses the same public position units as the target and
     * measurement.
     */
    public static Plant position(PositionOutput out,
                                 PlantTargetResolver targetResolver,
                                 PlantTargetGuards guards,
                                 ScalarSource measurement,
                                 double positionTolerance) {
        return new DeviceManagedPositionPlant(out, targetResolver, guards, measurement, positionTolerance);
    }

    /**
     * Create a device-managed velocity plant with feedback.
     * {@code velocityTolerance} uses the same public velocity units as the target and
     * measurement.
     */
    public static Plant velocity(VelocityOutput out,
                                 PlantTargetResolver targetResolver,
                                 PlantTargetGuards guards,
                                 ScalarSource measurement,
                                 double velocityTolerance) {
        return new DeviceManagedVelocityPlant(out, targetResolver, guards, measurement, velocityTolerance);
    }

    /**
     * Create a framework-regulated position plant that drives raw power.
     * The final regulator result is required to be finite, normalized to {@code [-1.0, +1.0]},
     * and fail-stopped before a runtime control/output failure is propagated.
     * {@code positionTolerance} uses the same public position units as the target and
     * measurement.
     */
    public static Plant positionFromPower(PowerOutput powerOut,
                                          PlantTargetResolver targetResolver,
                                          PlantTargetGuards guards,
                                          ScalarSource measurement,
                                          ScalarRegulator regulator,
                                          double positionTolerance) {
        return new RegulatedPositionPlant(powerOut, targetResolver, guards, measurement, regulator, positionTolerance);
    }

    /**
     * Create a framework-regulated velocity plant that drives raw power.
     * The final regulator result is required to be finite, normalized to {@code [-1.0, +1.0]},
     * and fail-stopped before a runtime control/output failure is propagated.
     * {@code velocityTolerance} uses the same public velocity units as the target and
     * measurement.
     */
    public static Plant velocityFromPower(PowerOutput powerOut,
                                          PlantTargetResolver targetResolver,
                                          PlantTargetGuards guards,
                                          ScalarSource measurement,
                                          ScalarRegulator regulator,
                                          double velocityTolerance) {
        return new RegulatedVelocityPlant(powerOut, targetResolver, guards, measurement, regulator, velocityTolerance);
    }

    private abstract static class AbstractSourceDrivenPlant implements Plant {
        private final PlantTargetResolver targetResolver;
        private final ScalarTarget commandTarget;
        private final PlantTargetGuards guards;

        private double requestedTarget = Double.NaN;
        private double appliedTarget;
        private PlantTargetStatus targetStatus = PlantTargetStatus.STOPPED;
        private PlantTargetResolution targetResolution = PlantTargetResolution.unavailable("not sampled");

        AbstractSourceDrivenPlant(PlantTargetResolver targetResolver, PlantTargetGuards guards) {
            this.targetResolver = Objects.requireNonNull(targetResolver, "targetResolver");
            this.commandTarget = PlantTargets.commandTargetOf(this.targetResolver);
            this.guards = guards == null ? PlantTargetGuards.none() : guards;
        }

        @Override
        public final void update(LoopClock clock) {
            double priorAppliedTarget = appliedTarget;
            PlantTargetStatus priorTargetStatus = targetStatus;
            PlantTargetResolution priorTargetResolution = targetResolution;
            prepareTargetContext(clock);
            PlantTargetContext context = targetContext(clock);
            targetResolution = targetResolver.resolve(context, clock);
            if (targetResolution != null && targetResolution.hasTarget()) {
                requestedTarget = targetResolution.target();
            } else {
                requestedTarget = appliedTarget;
            }

            double candidate = sanitizeRequestedTarget(requestedTarget);
            PlantTargetStatus status;
            if (targetResolution == null || !targetResolution.hasTarget()) {
                status = PlantTargetStatus.targetUnavailable(targetResolution != null
                        ? targetResolution.reason()
                        : "missing plant target resolution");
                candidate = appliedTarget;
            } else {
                status = candidate == requestedTarget
                        ? PlantTargetStatus.ACCEPTED
                        : PlantTargetStatus.clampedToRange("target sanitized to finite value");
            }

            PlantTargetGuards.Result result = PlantTargetSafety.applyGuards(
                    guards, candidate, status, appliedTarget, context.targetRange(), "Plant", clock);
            appliedTarget = result.target;
            targetStatus = result.status;
            try {
                applyTarget(appliedTarget, clock);
                updateStatus(clock);
            } catch (RuntimeException failure) {
                onUpdateFailure(priorAppliedTarget, priorTargetStatus, priorTargetResolution, failure);
                throw failure;
            }
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

        protected void onUpdateFailure(double priorAppliedTarget,
                                       PlantTargetStatus priorTargetStatus,
                                       PlantTargetResolution priorTargetResolution,
                                       RuntimeException failure) {
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
        public final PlantTargetResolution getTargetResolution() {
            return targetResolution;
        }

        @Override
        public final PlantTargetStatus getTargetStatus() {
            return targetStatus;
        }

        @Override
        public final boolean hasCommandTarget() {
            return commandTarget != null;
        }

        @Override
        public final ScalarTarget commandTarget() {
            if (commandTarget == null) return Plant.super.commandTarget();
            return commandTarget;
        }

        @Override
        public void reset() {
            targetResolver.reset();
            guards.reset();
            requestedTarget = Double.NaN;
            appliedTarget = 0.0;
            targetStatus = PlantTargetStatus.STOPPED;
            targetResolution = PlantTargetResolution.unavailable("not sampled");
        }

        protected final void markStopped(double appliedAfterStop) {
            appliedTarget = appliedAfterStop;
            targetStatus = PlantTargetStatus.STOPPED;
        }

        protected final void restoreTargetState(double priorAppliedTarget,
                                                PlantTargetStatus priorTargetStatus,
                                                PlantTargetResolution priorTargetResolution) {
            appliedTarget = priorAppliedTarget;
            targetStatus = priorTargetStatus;
            targetResolution = priorTargetResolution;
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
            targetResolver.debugDump(dbg, p + ".targetResolver");
            guards.debugDump(dbg, p + ".targetGuards");
        }
    }

    private static final class PowerPlant extends AbstractSourceDrivenPlant {
        private final PowerOutput out;

        PowerPlant(PowerOutput out, PlantTargetResolver targetResolver, PlantTargetGuards guards) {
            super(targetResolver, guards);
            this.out = Objects.requireNonNull(out, "out");
        }

        @Override
        protected PlantTargetContext targetContext(LoopClock clock) {
            return PlantTargetContext.simple(false, Double.NaN, NORMALIZED_POWER_RANGE,
                    getRequestedTarget(), getAppliedTarget());
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

        CommandedPositionPlant(PositionOutput out,
                               PlantTargetResolver targetResolver,
                               PlantTargetGuards guards) {
            super(targetResolver, guards);
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

        AbstractFeedbackPlant(PlantTargetResolver targetResolver,
                              PlantTargetGuards guards,
                              ScalarSource measurement,
                              double tolerance) {
            super(targetResolver, guards);
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
            return completionEvidenceValid()
                    && Double.isFinite(lastMeasurement)
                    && Math.abs(getRequestedTarget() - target) <= tolerance
                    && Math.abs(getAppliedTarget() - target) <= tolerance
                    && Math.abs(lastMeasurement - target) <= tolerance
                    && getTargetStatus().kind() == PlantTargetStatus.Kind.ACCEPTED;
        }

        protected boolean completionEvidenceValid() {
            return true;
        }

        protected final void invalidateAtTarget() {
            lastAtTarget = false;
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

        DeviceManagedPositionPlant(PositionOutput out, PlantTargetResolver targetResolver,
                                   PlantTargetGuards guards, ScalarSource measurement, double tolerance) {
            super(targetResolver, guards, measurement, tolerance);
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

        DeviceManagedVelocityPlant(VelocityOutput out, PlantTargetResolver targetResolver,
                                   PlantTargetGuards guards, ScalarSource measurement, double tolerance) {
            super(targetResolver, guards, measurement, tolerance);
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
        private final RegulatedPowerChannel powerChannel;
        private boolean regulatedActuationCompleted;

        AbstractRegulatedPlant(PowerOutput out, PlantTargetResolver targetResolver,
                               PlantTargetGuards guards, ScalarSource measurement,
                               ScalarRegulator regulator, double tolerance, String controlPath) {
            super(targetResolver, guards, measurement, tolerance);
            this.powerChannel = new RegulatedPowerChannel(out, regulator, controlPath);
        }

        @Override
        protected final void applyTarget(double target, LoopClock clock) {
            // The regulator needs the same-loop measurement, so power is written in onFeedbackUpdate.
            regulatedActuationCompleted = false;
            invalidateAtTarget();
        }

        @Override
        protected final void onFeedbackUpdate(LoopClock clock, double measurement) {
            powerChannel.update(getAppliedTarget(), measurement, clock);
            regulatedActuationCompleted = true;
        }

        @Override
        protected final boolean completionEvidenceValid() {
            return regulatedActuationCompleted;
        }

        @Override
        protected final void onUpdateFailure(double priorAppliedTarget,
                                             PlantTargetStatus priorTargetStatus,
                                             PlantTargetResolution priorTargetResolution,
                                             RuntimeException failure) {
            regulatedActuationCompleted = false;
            invalidateAtTarget();
            try {
                resetTargetGuards();
            } catch (RuntimeException cleanupFailure) {
                suppress(failure, cleanupFailure);
            }
            if (powerChannel.lastStopSubmitted()) {
                markStopped(0.0);
            } else {
                restoreTargetState(priorAppliedTarget, priorTargetStatus, priorTargetResolution);
            }
        }

        @Override
        public void reset() {
            regulatedActuationCompleted = false;
            invalidateAtTarget();
            super.reset();
            powerChannel.reset();
        }

        @Override
        public void stop() {
            double priorAppliedTarget = getAppliedTarget();
            PlantTargetStatus priorTargetStatus = getTargetStatus();
            PlantTargetResolution priorTargetResolution = getTargetResolution();
            regulatedActuationCompleted = false;
            invalidateAtTarget();

            RuntimeException primary = null;
            try {
                powerChannel.stop();
            } catch (RuntimeException failure) {
                primary = failure;
            }
            try {
                resetTargetGuards();
            } catch (RuntimeException failure) {
                primary = suppress(primary, failure);
            }

            if (powerChannel.lastStopSubmitted()) {
                markStopped(0.0);
            } else {
                restoreTargetState(priorAppliedTarget, priorTargetStatus, priorTargetResolution);
            }
            if (primary != null) throw primary;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            super.debugDump(dbg, prefix);
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "plant" : prefix;
            dbg.addData(p + ".output", powerChannel.regulatorOutput());
            powerChannel.debugDump(dbg, p);
        }

        private static RuntimeException suppress(RuntimeException primary, RuntimeException additional) {
            if (primary == null) return additional;
            if (primary != additional) primary.addSuppressed(additional);
            return primary;
        }
    }

    private static final class RegulatedPositionPlant extends AbstractRegulatedPlant {
        RegulatedPositionPlant(PowerOutput out, PlantTargetResolver targetResolver,
                               PlantTargetGuards guards, ScalarSource measurement,
                               ScalarRegulator regulator, double tolerance) {
            super(out, targetResolver, guards, measurement, regulator, tolerance,
                    "Plants.positionFromPower");
        }
    }

    private static final class RegulatedVelocityPlant extends AbstractRegulatedPlant {
        RegulatedVelocityPlant(PowerOutput out, PlantTargetResolver targetResolver,
                               PlantTargetGuards guards, ScalarSource measurement,
                               ScalarRegulator regulator, double tolerance) {
            super(out, targetResolver, guards, measurement, regulator, tolerance,
                    "Plants.velocityFromPower");
        }
    }
}
