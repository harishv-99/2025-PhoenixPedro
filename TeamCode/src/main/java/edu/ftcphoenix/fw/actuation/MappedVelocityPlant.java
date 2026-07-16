package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Source-driven velocity {@link Plant} that maps plant velocity units to native hardware units.
 *
 * <p>Robot code normally builds these through {@code FtcActuators}. This class is the advanced
 * boundary adapter for custom hardware integrations. The plant samples one target source every
 * update, clamps it to the configured velocity range, applies dynamic plant target guards, and then
 * rechecks the final guarded target for finiteness and range before commanding either a native
 * velocity output or a framework-owned regulator over raw power. The regulated path evaluates its
 * regulator once, normalizes the finite result to {@code [-1.0, +1.0]}, and performs best-effort
 * fail-stop cleanup before propagating a runtime control/output failure.</p>
 */
public final class MappedVelocityPlant implements Plant {

    private final VelocityOutput velocityOut;
    private final PowerOutput regulatedPowerOut;
    private final ScalarRegulator regulator;
    private final RegulatedPowerChannel regulatedPowerChannel;
    private final ScalarSource nativeMeasurement;
    private final PlantTargetSource targetSource;
    private final ScalarTarget writableTarget;
    private final PlantTargetGuards targetGuards;
    private final ScalarRange configuredRange;
    private final double nativePerPlantUnit;
    private final double tolerance;

    private double requestedTarget;
    private double appliedTarget;
    private double lastMeasurement = Double.NaN;
    private double lastNativeMeasurement = Double.NaN;
    private boolean lastAtTarget;
    private boolean regulatedActuationCompleted;
    private PlantTargetStatus targetStatus = PlantTargetStatus.STOPPED;
    private PlantTargetPlan targetPlan = PlantTargetPlan.unavailable("not sampled");

    private MappedVelocityPlant(VelocityOutput velocityOut,
                                PowerOutput regulatedPowerOut,
                                ScalarRegulator regulator,
                                ScalarSource nativeMeasurement,
                                PlantTargetSource targetSource,
                                ScalarTarget writableTarget,
                                PlantTargetGuards targetGuards,
                                ScalarRange configuredRange,
                                double nativePerPlantUnit,
                                double tolerance) {
        this.velocityOut = velocityOut;
        this.regulatedPowerOut = regulatedPowerOut;
        this.regulator = regulator;
        this.nativeMeasurement = Objects.requireNonNull(nativeMeasurement, "nativeMeasurement").memoized();
        this.targetSource = Objects.requireNonNull(targetSource, "targetSource");
        this.writableTarget = writableTarget;
        this.targetGuards = targetGuards == null ? PlantTargetGuards.none() : targetGuards;
        this.configuredRange = Objects.requireNonNull(configuredRange, "configuredRange");
        this.nativePerPlantUnit = nativePerPlantUnit;
        this.tolerance = tolerance;
        validate();
        this.regulatedPowerChannel = regulatedPowerOut == null
                ? null
                : new RegulatedPowerChannel(regulatedPowerOut, regulator, "MappedVelocityPlant");
    }

    /**
     * Start configuring a device-managed native velocity-output plant.
     */
    public static Builder velocityOutput(VelocityOutput out, ScalarSource nativeMeasurement) {
        return new Builder(Objects.requireNonNull(out, "out"), null, null,
                Objects.requireNonNull(nativeMeasurement, "nativeMeasurement"));
    }

    /**
     * Start configuring a framework-regulated velocity plant that drives raw power.
     * The resulting Plant keeps mechanism target units separate from its normalized power command.
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
     * Advanced builder. FTC robot code should normally use {@code FtcActuators}.
     */
    public static final class Builder {
        private final VelocityOutput velocityOut;
        private final PowerOutput regulatedPowerOut;
        private final ScalarRegulator regulator;
        private final ScalarSource nativeMeasurement;
        private ScalarRange configuredRange = ScalarRange.unbounded();
        private double nativePerPlantUnit = 1.0;
        private double tolerance = 100.0;
        private PlantTargetSource targetSource;
        private ScalarTarget writableTarget;
        private PlantTargetGuards targetGuards = PlantTargetGuards.none();

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
         * Sets the legal target range in plant velocity units. The configured range must be valid
         * and contain at least one finite command.
         */
        public Builder range(ScalarRange range) {
            this.configuredRange = Objects.requireNonNull(range, "range");
            return this;
        }

        /**
         * Sets how many native velocity units correspond to one plant velocity unit.
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
            if (tolerance < 0.0 || !Double.isFinite(tolerance))
                throw new IllegalArgumentException("velocityTolerance must be finite and >= 0");
            this.tolerance = tolerance;
            return this;
        }

        /**
         * Sets dynamic plant-level target guards.
         */
        public Builder targetGuards(PlantTargetGuards targetGuards) {
            this.targetGuards = targetGuards == null ? PlantTargetGuards.none() : targetGuards;
            return this;
        }

        /**
         * Use a writable exact target source and register it for PlantTasks.
         */
        public Builder targetedBy(ScalarTarget targetSource) {
            this.targetSource = PlantTargets.exact(Objects.requireNonNull(targetSource, "targetSource"));
            this.writableTarget = targetSource;
            return this;
        }

        /**
         * Use a read-only scalar source as an exact target.
         *
         * <p>The scalar is lifted into plant-target space. Because it is read-only, no
         * writable target is registered unless writableTarget(...) is called.</p>
         */
        public Builder targetedBy(ScalarSource targetSource) {
            this.targetSource = PlantTargets.exact(Objects.requireNonNull(targetSource, "targetSource"));
            return this;
        }

        /**
         * Use a plant-aware final target source.
         */
        public Builder targetedBy(PlantTargetSource targetSource) {
            this.targetSource = Objects.requireNonNull(targetSource, "targetSource");
            return this;
        }

        /**
         * Register the command target that plant tasks may write when the final source is composed.
         */
        public Builder writableTarget(ScalarTarget writableTarget) {
            this.writableTarget = Objects.requireNonNull(writableTarget, "writableTarget");
            return this;
        }

        /**
         * Builds the mapped velocity plant.
         *
         * @throws IllegalArgumentException if the configured range cannot contain a finite command
         *                                  or a static guard fallback lies outside that range
         */
        public MappedVelocityPlant build() {
            if (targetSource == null)
                throw new IllegalStateException("MappedVelocityPlant requires targetedBy(...)");
            return new MappedVelocityPlant(velocityOut, regulatedPowerOut, regulator, nativeMeasurement,
                    targetSource, writableTarget, targetGuards, configuredRange, nativePerPlantUnit, tolerance);
        }
    }

    private void validate() {
        if (velocityOut == null && regulatedPowerOut == null)
            throw new IllegalStateException("MappedVelocityPlant requires either a velocity output or regulated power output");
        if (velocityOut != null && regulatedPowerOut != null)
            throw new IllegalStateException("MappedVelocityPlant cannot use both velocity output and regulated power output");
        if (regulatedPowerOut != null && regulator == null)
            throw new IllegalStateException("Regulated velocity plants require a regulator");
        if (!Double.isFinite(nativePerPlantUnit) || Math.abs(nativePerPlantUnit) < 1.0e-12)
            throw new IllegalArgumentException("nativePerPlantUnit must be finite and non-zero");
        if (tolerance < 0.0 || !Double.isFinite(tolerance))
            throw new IllegalArgumentException("velocityTolerance must be finite and >= 0");
        PlantTargetSafety.requireUsableConfiguredRange(configuredRange, "MappedVelocityPlant");
        targetGuards.validateFallbackTargets(configuredRange, "MappedVelocityPlant");
    }

    /**
     * Legal velocity target range in plant units.
     */
    public ScalarRange targetRange() {
        return configuredRange;
    }

    @Override
    public void update(LoopClock clock) {
        double priorAppliedTarget = appliedTarget;
        PlantTargetStatus priorTargetStatus = targetStatus;
        PlantTargetPlan priorTargetPlan = targetPlan;
        samplePlantMeasurement(clock);
        PlantTargetContext context = PlantTargetContext.simple(true, lastMeasurement, targetRange(), requestedTarget, appliedTarget);
        targetPlan = targetSource.resolve(context, clock);
        if (targetPlan != null && targetPlan.hasTarget()) {
            requestedTarget = targetPlan.target();
        } else {
            requestedTarget = appliedTarget;
        }

        double candidate = Double.isFinite(requestedTarget) ? requestedTarget : appliedTarget;
        PlantTargetStatus status = (targetPlan != null && targetPlan.hasTarget())
                ? PlantTargetStatus.ACCEPTED
                : PlantTargetStatus.targetUnavailable(targetPlan != null ? targetPlan.reason() : "missing plant target plan");
        ScalarRange range = targetRange();
        if (!range.valid) {
            candidate = appliedTarget;
            status = PlantTargetStatus.referenceNotEstablished(range.reason);
        } else {
            double clamped = range.clamp(candidate);
            if (!range.contains(candidate))
                status = PlantTargetStatus.clampedToRange("target clamped to velocity range");
            candidate = clamped;
        }
        PlantTargetGuards.Result guarded = PlantTargetSafety.applyGuards(
                targetGuards, candidate, status, appliedTarget, range, "velocity", clock);
        appliedTarget = guarded.target;
        targetStatus = guarded.status;
        if (velocityOut != null) {
            velocityOut.setVelocity(toNative(appliedTarget));
        } else {
            regulatedActuationCompleted = false;
            lastAtTarget = false;
            try {
                regulatedPowerChannel.update(appliedTarget, lastMeasurement, clock);
            } catch (RuntimeException failure) {
                handleRegulatedUpdateFailure(
                        priorAppliedTarget, priorTargetStatus, priorTargetPlan, failure);
                throw failure;
            }
            regulatedActuationCompleted = true;
        }
        lastAtTarget = atTarget(requestedTarget);
    }

    @Override
    public void reset() {
        regulatedActuationCompleted = false;
        lastAtTarget = false;
        targetSource.reset();
        nativeMeasurement.reset();
        targetGuards.reset();
        if (regulatedPowerChannel != null) regulatedPowerChannel.reset();
        requestedTarget = 0.0;
        appliedTarget = 0.0;
        lastMeasurement = Double.NaN;
        lastNativeMeasurement = Double.NaN;
        targetStatus = PlantTargetStatus.STOPPED;
        targetPlan = PlantTargetPlan.unavailable("not sampled");
    }

    @Override
    public void stop() {
        if (regulatedPowerChannel == null) {
            velocityOut.stop();
            targetGuards.reset();
            appliedTarget = 0.0;
            targetStatus = PlantTargetStatus.STOPPED;
            targetPlan = PlantTargetPlan.unavailable("plant stopped");
            return;
        }

        double priorAppliedTarget = appliedTarget;
        PlantTargetStatus priorTargetStatus = targetStatus;
        PlantTargetPlan priorTargetPlan = targetPlan;
        regulatedActuationCompleted = false;
        lastAtTarget = false;

        RuntimeException primary = null;
        try {
            regulatedPowerChannel.stop();
        } catch (RuntimeException failure) {
            primary = failure;
        }
        try {
            targetGuards.reset();
        } catch (RuntimeException failure) {
            primary = suppress(primary, failure);
        }

        if (regulatedPowerChannel.lastStopSubmitted()) {
            markSuccessfullyStopped();
        } else {
            restoreTargetState(priorAppliedTarget, priorTargetStatus, priorTargetPlan);
        }
        if (primary != null) throw primary;
    }

    @Override
    public double getRequestedTarget() {
        return requestedTarget;
    }

    @Override
    public double getAppliedTarget() {
        return appliedTarget;
    }

    @Override
    public PlantTargetPlan getTargetPlan() {
        return targetPlan;
    }

    @Override
    public PlantTargetStatus getTargetStatus() {
        return targetStatus;
    }

    @Override
    public boolean hasWritableTarget() {
        return writableTarget != null;
    }

    @Override
    public ScalarTarget writableTarget() {
        if (writableTarget == null) return Plant.super.writableTarget();
        return writableTarget;
    }

    @Override
    public boolean hasFeedback() {
        return true;
    }

    @Override
    public double getMeasurement() {
        return lastMeasurement;
    }

    @Override
    public boolean atTarget() {
        return lastAtTarget;
    }

    @Override
    public boolean atTarget(double target) {
        return (regulatedPowerChannel == null || regulatedActuationCompleted)
                && Double.isFinite(target)
                && Double.isFinite(lastMeasurement)
                && Math.abs(requestedTarget - target) <= tolerance
                && Math.abs(appliedTarget - target) <= tolerance
                && Math.abs(lastMeasurement - target) <= tolerance
                && targetStatus.kind() == PlantTargetStatus.Kind.ACCEPTED;
    }

    private void samplePlantMeasurement(LoopClock clock) {
        lastNativeMeasurement = nativeMeasurement.getAsDouble(clock);
        lastMeasurement = fromNative(lastNativeMeasurement);
    }

    private double toNative(double plantVelocity) {
        return plantVelocity * nativePerPlantUnit;
    }

    private double fromNative(double nativeVelocity) {
        return nativeVelocity / nativePerPlantUnit;
    }

    private void handleRegulatedUpdateFailure(double priorAppliedTarget,
                                              PlantTargetStatus priorTargetStatus,
                                              PlantTargetPlan priorTargetPlan,
                                              RuntimeException failure) {
        regulatedActuationCompleted = false;
        lastAtTarget = false;
        try {
            targetGuards.reset();
        } catch (RuntimeException cleanupFailure) {
            suppress(failure, cleanupFailure);
        }
        if (regulatedPowerChannel.lastStopSubmitted()) {
            markSuccessfullyStopped();
        } else {
            restoreTargetState(priorAppliedTarget, priorTargetStatus, priorTargetPlan);
        }
    }

    private void markSuccessfullyStopped() {
        appliedTarget = 0.0;
        targetStatus = PlantTargetStatus.STOPPED;
        targetPlan = PlantTargetPlan.unavailable("plant stopped");
    }

    private void restoreTargetState(double priorAppliedTarget,
                                    PlantTargetStatus priorTargetStatus,
                                    PlantTargetPlan priorTargetPlan) {
        appliedTarget = priorAppliedTarget;
        targetStatus = priorTargetStatus;
        targetPlan = priorTargetPlan;
    }

    private static RuntimeException suppress(RuntimeException primary, RuntimeException additional) {
        if (primary == null) return additional;
        if (primary != additional) primary.addSuppressed(additional);
        return primary;
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        Plant.super.debugDump(dbg, prefix);
        String p = (prefix == null || prefix.isEmpty()) ? "velocityPlant" : prefix;
        dbg.addData(p + ".nativePerPlantUnit", nativePerPlantUnit)
                .addData(p + ".nativeMeasurement", lastNativeMeasurement)
                .addData(p + ".targetRange", targetRange());
        targetSource.debugDump(dbg, p + ".targetSource");
        targetGuards.debugDump(dbg, p + ".targetGuards");
        if (regulatedPowerChannel != null) {
            regulatedPowerChannel.debugDump(dbg, p);
        } else {
            // Preserve the existing debug key for device-managed velocity Plants.
            dbg.addData(p + ".regulatorOutput", 0.0);
        }
    }
}
