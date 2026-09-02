package edu.ftcsushi.fw.actuation;

import java.util.Objects;

import edu.ftcsushi.fw.core.control.ScalarRegulator;
import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.hal.VelocityOutput;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Source-driven velocity {@link Plant} that maps plant velocity units to native hardware units.
 *
 * <p>This package-private runtime is shared by the public FTC and output-port construction
 * gateways. The plant invokes one target resolver every update, clamps it to the configured
 * velocity range, applies dynamic plant target guards, and then
 * rechecks the final guarded target for finiteness and range before commanding either a native
 * velocity output or a framework-owned regulator over raw power. The regulated path evaluates its
 * regulator once, normalizes the finite result to {@code [-1.0, +1.0]}, and performs best-effort
 * fail-stop cleanup before propagating a runtime control/output failure.</p>
 *
 * <p>A fully bounded Plant/native map proves both exact endpoint products finite at construction.
 * Unbounded maps are permitted, so the direct-output path calculates each native command into a
 * temporary and rejects non-finite results before applied-target commit or output. That failure
 * keeps the mapping exception primary while invoking the output's natural stop best-effort:
 * successful cleanup publishes the same zero-applied stopped diagnostic state as {@link #stop()},
 * while a failed stop retains the prior applied/status/resolution facts and is suppressed. Unlike
 * explicit {@code stop()}, this internal fail-stop does not terminalize the primitive; an isolated
 * deterministic fixture may exercise a different-cycle attempt. A runtime failure escaping any
 * managed or approved advanced host lifecycle phase is nevertheless terminal for that host. A
 * non-finite inverse conversion publishes measurement {@link Double#NaN} and cannot be at
 * target.</p>
 */
final class MappedVelocityPlant implements Plant {

    private final VelocityOutput velocityOut;
    private final PowerOutput regulatedPowerOut;
    private final ScalarRegulator regulator;
    private final RegulatedPowerChannel regulatedPowerChannel;
    private final ScalarSource nativeMeasurement;
    private final PlantTargetResolver targetResolver;
    private final ScalarTarget commandTarget;
    private final PlantTargetGuards targetGuards;
    private final PlantLifecycle lifecycle = new PlantLifecycle();
    private final PlantUpdateCycle updateCycle = new PlantUpdateCycle("VelocityPlant");
    private final ScalarRange configuredRange;
    private final double nativePerPlantUnit;
    private final double tolerance;

    private double requestedTarget;
    private double appliedTarget;
    private double lastMeasurement = Double.NaN;
    private double lastNativeMeasurement = Double.NaN;
    private boolean lastAtTarget;
    private boolean regulatedActuationCompleted;
    private boolean heartbeatAttempted;
    private boolean standardControlTuningClaimed;
    private PlantTargetStatus targetStatus = PlantTargetStatus.STOPPED;
    private PlantTargetResolution targetResolution = PlantTargetResolution.unavailable("not sampled");

    private MappedVelocityPlant(VelocityOutput velocityOut,
                                PowerOutput regulatedPowerOut,
                                ScalarRegulator regulator,
                                ScalarSource nativeMeasurement,
                                PlantTargetResolver targetResolver,
                                PlantTargetGuards targetGuards,
                                ScalarRange configuredRange,
                                double nativePerPlantUnit,
                                double tolerance) {
        this.velocityOut = velocityOut;
        this.regulatedPowerOut = regulatedPowerOut;
        this.regulator = regulator;
        this.nativeMeasurement = Objects.requireNonNull(nativeMeasurement, "nativeMeasurement").memoized();
        this.targetResolver = Objects.requireNonNull(targetResolver, "targetResolver");
        this.commandTarget = PlantTargets.commandTargetOf(this.targetResolver);
        this.targetGuards = targetGuards == null ? PlantTargetGuards.none() : targetGuards;
        this.configuredRange = Objects.requireNonNull(configuredRange, "configuredRange");
        this.nativePerPlantUnit = nativePerPlantUnit;
        this.tolerance = tolerance;
        validate();
        this.regulatedPowerChannel = regulatedPowerOut == null
                ? null
                : new RegulatedPowerChannel(regulatedPowerOut, regulator, "VelocityPlant");
    }

    /**
     * Start configuring a device-managed native velocity-output plant.
     *
     * <p>The staged builder requires an explicit
     * {@link FeedbackConfigurationStep#velocityTolerance(double)} in public plant velocity units
     * before target selection exposes {@link MappedPlantBuildStep#build()}.</p>
     */
    static FeedbackConfigurationStep velocityOutput(VelocityOutput out,
                                                    ScalarSource nativeMeasurement) {
        return new Builder(Objects.requireNonNull(out, "out"), null, null,
                Objects.requireNonNull(nativeMeasurement, "nativeMeasurement"));
    }

    /**
     * Start configuring a framework-regulated velocity plant that drives raw power.
     * The resulting Plant keeps mechanism target units separate from its normalized power command.
     * The staged builder requires an explicit
     * {@link FeedbackConfigurationStep#velocityTolerance(double)} in public plant velocity units
     * before target selection exposes {@link MappedPlantBuildStep#build()}.
     */
    static FeedbackConfigurationStep regulated(PowerOutput powerOut,
                                                ScalarSource nativeMeasurement,
                                                ScalarRegulator regulator) {
        return new Builder(null,
                Objects.requireNonNull(powerOut, "powerOut"),
                Objects.requireNonNull(regulator, "regulator"),
                Objects.requireNonNull(nativeMeasurement, "nativeMeasurement"));
    }

    /**
     * Optional mapped-velocity configuration before the required plant-unit tolerance answer.
     * FTC robot code should normally use {@code FtcActuators}.
     */
    interface FeedbackConfigurationStep {
        /**
         * Set the legal target range in plant velocity units.
         */
        FeedbackConfigurationStep range(ScalarRange range);

        /**
         * Set how many native velocity units correspond to one plant velocity unit.
         */
        FeedbackConfigurationStep nativePerPlantUnit(double nativePerPlantUnit);

        /**
         * Set the required completion tolerance in plant velocity units and continue to target
         * selection.
         */
        MappedPlantTargetStep<MappedVelocityPlant> velocityTolerance(double tolerance);
    }

    /** Mutable implementation hidden behind the ordered public configuration stages. */
    private static final class Builder implements FeedbackConfigurationStep,
            MappedPlantTargetStep<MappedVelocityPlant>,
            MappedPlantBuildStep<MappedVelocityPlant> {
        private final VelocityOutput velocityOut;
        private final PowerOutput regulatedPowerOut;
        private final ScalarRegulator regulator;
        private final ScalarSource nativeMeasurement;
        private ScalarRange configuredRange = ScalarRange.unbounded();
        private double nativePerPlantUnit = 1.0;
        private double tolerance = Double.NaN;
        private boolean toleranceConfigured;
        private PlantTargetResolver targetResolver;
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
            if (!Double.isFinite(nativePerPlantUnit) || nativePerPlantUnit == 0.0) {
                throw new IllegalArgumentException("nativePerPlantUnit must be finite and non-zero");
            }
            this.nativePerPlantUnit = nativePerPlantUnit;
            return this;
        }

        /**
         * Sets the plant-level completion tolerance in plant velocity units.
         */
        public Builder velocityTolerance(double tolerance) {
            if (toleranceConfigured)
                throw new IllegalStateException("velocityTolerance(...) has already been answered "
                        + "for this VelocityPlant recipe");
            if (tolerance < 0.0 || !Double.isFinite(tolerance))
                throw new IllegalArgumentException("velocityTolerance must be finite and >= 0");
            this.tolerance = tolerance;
            this.toleranceConfigured = true;
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
         * Uses a plant-aware final target resolver.
         *
         * <p>If the resolver graph designates a command target, such as the base of a command-backed
         * overlay, this Plant exposes that same target to robot policy and {@link ScalarTasks}.</p>
         */
        public Builder targetFromResolver(PlantTargetResolver targetResolver) {
            requireTargetUnanswered();
            this.targetResolver = Objects.requireNonNull(targetResolver, "targetResolver");
            return this;
        }

        private void requireTargetUnanswered() {
            if (targetResolver != null) {
                throw new IllegalStateException("targetFromResolver(...) has already been answered for "
                        + "this VelocityPlant recipe; create a new recipe to choose a different target");
            }
        }

        /**
         * Builds the mapped velocity plant.
         *
         * @throws IllegalStateException if no target resolver or plant-unit velocity tolerance was
         *                               configured
         * @throws IllegalArgumentException if the configured range cannot contain a finite command,
         *                                  a bounded endpoint maps non-finite, or a static guard
         *                                  fallback lies outside that range
         */
        public MappedVelocityPlant build() {
            if (targetResolver == null)
                throw new IllegalStateException("VelocityPlant requires targetFromResolver(...)");
            if (!toleranceConfigured)
                throw new IllegalStateException("VelocityPlant feedback requires "
                        + "velocityTolerance(...) in plant velocity units before build()");
            return new MappedVelocityPlant(velocityOut, regulatedPowerOut, regulator, nativeMeasurement,
                    targetResolver, targetGuards, configuredRange, nativePerPlantUnit, tolerance);
        }
    }

    private void validate() {
        if (velocityOut == null && regulatedPowerOut == null)
            throw new IllegalStateException("VelocityPlant requires either a velocity output or regulated power output");
        if (velocityOut != null && regulatedPowerOut != null)
            throw new IllegalStateException("VelocityPlant cannot use both velocity output and regulated power output");
        if (regulatedPowerOut != null && regulator == null)
            throw new IllegalStateException("Regulated velocity plants require a regulator");
        if (!Double.isFinite(nativePerPlantUnit) || nativePerPlantUnit == 0.0)
            throw new IllegalArgumentException("nativePerPlantUnit must be finite and non-zero");
        if (tolerance < 0.0 || !Double.isFinite(tolerance))
            throw new IllegalArgumentException("velocityTolerance must be finite and >= 0");
        PlantTargetSafety.requireUsableConfiguredRange(configuredRange, "VelocityPlant");
        targetGuards.validateFallbackTargets(configuredRange, "VelocityPlant");
        requireFiniteBoundedMap(
                configuredRange, nativePerPlantUnit, "VelocityPlant configuration");
    }

    @Override
    public void update(LoopClock clock) {
        if (!lifecycle.isActive()) return;
        heartbeatAttempted = true;
        if (!updateCycle.begin(clock)) return;
        try {
            updateOnce(clock);
            updateCycle.succeed();
        } catch (RuntimeException failure) {
            updateCycle.fail(failure);
            throw failure;
        }
    }

    private void updateOnce(LoopClock clock) {
        if (!lifecycle.isActive()) return;
        double priorAppliedTarget = appliedTarget;
        PlantTargetStatus priorTargetStatus = targetStatus;
        PlantTargetResolution priorTargetResolution = targetResolution;
        samplePlantMeasurement(clock);
        if (!lifecycle.isActive()) return;
        PlantTargetContext context = PlantTargetContext.simple(
                true, lastMeasurement, configuredRange, requestedTarget, appliedTarget);
        PlantTargetResolution nextTargetResolution = targetResolver.resolve(context, clock);
        if (!lifecycle.isActive()) return;
        double nextRequestedTarget;
        if (nextTargetResolution != null && nextTargetResolution.hasTarget()) {
            nextRequestedTarget = nextTargetResolution.target();
        } else {
            nextRequestedTarget = appliedTarget;
        }

        double candidate = Double.isFinite(nextRequestedTarget) ? nextRequestedTarget : appliedTarget;
        PlantTargetStatus status = (nextTargetResolution != null && nextTargetResolution.hasTarget())
                ? PlantTargetStatus.ACCEPTED
                : PlantTargetStatus.targetUnavailable(nextTargetResolution != null
                        ? nextTargetResolution.reason()
                        : "missing plant target resolution");
        ScalarRange range = configuredRange;
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
        if (!lifecycle.isActive()) return;
        requestedTarget = nextRequestedTarget;

        double nativeCommand = Double.NaN;
        if (velocityOut != null) {
            nativeCommand = toNative(guarded.target);
            if (!Double.isFinite(nativeCommand)) {
                IllegalStateException failure = new IllegalStateException(
                        "VelocityPlant.update(...) could not map finite Plant target "
                                + guarded.target + " to a finite native velocity using "
                                + "nativeVelocity = plantVelocity * nativePerPlantUnit, with "
                                + "nativePerPlantUnit=" + nativePerPlantUnit
                                + " and result=" + nativeCommand
                                + ". Check target magnitude and the Plant/native mapping.");
                failRuntimeMapping(
                        priorAppliedTarget,
                        priorTargetStatus,
                        priorTargetResolution,
                        failure);
            }
        }

        appliedTarget = guarded.target;
        targetStatus = guarded.status;
        targetResolution = nextTargetResolution;
        double effectAppliedTarget = appliedTarget;
        PlantTargetStatus effectTargetStatus = targetStatus;
        PlantTargetResolution effectTargetResolution = targetResolution;
        if (velocityOut != null) {
            RuntimeException effectFailure = null;
            try {
                velocityOut.setVelocity(nativeCommand);
            } catch (RuntimeException failure) {
                effectFailure = failure;
            }
            if (!lifecycle.isActive()) {
                try {
                    enforceTerminalStopAfterReentrantEffect(
                            effectAppliedTarget, effectTargetStatus, effectTargetResolution);
                } catch (RuntimeException cleanupFailure) {
                    effectFailure = suppress(effectFailure, cleanupFailure);
                }
            }
            if (effectFailure != null) {
                if (lifecycle.isActive()) {
                    failStopDirectOutputFailure(
                            priorAppliedTarget,
                            priorTargetStatus,
                            priorTargetResolution,
                            effectAppliedTarget,
                            effectTargetStatus,
                            effectTargetResolution,
                            effectFailure);
                }
                throw effectFailure;
            }
            if (!lifecycle.isActive()) return;
        } else {
            regulatedActuationCompleted = false;
            lastAtTarget = false;
            try {
                regulatedPowerChannel.update(appliedTarget, lastMeasurement, clock, lifecycle);
            } catch (RuntimeException failure) {
                handleRegulatedUpdateFailure(
                    priorAppliedTarget, priorTargetStatus, priorTargetResolution, failure);
                if (!lifecycle.isActive()
                        && regulatedPowerChannel.terminalStopReassertRequired()) {
                    try {
                        enforceTerminalStopAfterReentrantEffect(
                                effectAppliedTarget, effectTargetStatus, effectTargetResolution);
                    } catch (RuntimeException cleanupFailure) {
                        suppress(failure, cleanupFailure);
                    }
                }
                throw failure;
            }
            if (!lifecycle.isActive()) {
                if (regulatedPowerChannel.terminalStopReassertRequired()) {
                    enforceTerminalStopAfterReentrantEffect(
                            effectAppliedTarget, effectTargetStatus, effectTargetResolution);
                }
                return;
            }
            regulatedActuationCompleted = true;
        }
        lastAtTarget = atTarget(requestedTarget);
    }

    @Override
    public void stop() {
        if (!lifecycle.claimStop()) return;

        double priorAppliedTarget = appliedTarget;
        PlantTargetStatus priorTargetStatus = targetStatus;
        PlantTargetResolution priorTargetResolution = targetResolution;
        regulatedActuationCompleted = false;
        lastAtTarget = false;
        RuntimeException primary = null;
        boolean outputStopSucceeded = false;
        if (regulatedPowerChannel == null) {
            try {
                velocityOut.stop();
                outputStopSucceeded = true;
            } catch (RuntimeException failure) {
                primary = failure;
            }
        } else {
            try {
                regulatedPowerChannel.stop();
            } catch (RuntimeException failure) {
                primary = failure;
            }
            outputStopSucceeded = regulatedPowerChannel.lastStopSubmitted();
        }

        if (outputStopSucceeded) {
            markSuccessfullyStopped();
        } else {
            restoreTargetState(priorAppliedTarget, priorTargetStatus, priorTargetResolution);
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
    public PlantTargetResolution getTargetResolution() {
        return targetResolution;
    }

    @Override
    public PlantTargetStatus getTargetStatus() {
        return targetStatus;
    }

    @Override
    public boolean hasCommandTarget() {
        return commandTarget != null;
    }

    @Override
    public ScalarTarget commandTarget() {
        if (commandTarget == null) return Plant.super.commandTarget();
        return commandTarget;
    }

    StandardControlTuning claimStandardControlTuning() {
        if (!lifecycle.isActive()) {
            throw new IllegalStateException(
                    "Cannot claim standard velocity tuning after the Plant has stopped");
        }
        if (heartbeatAttempted) {
            throw new IllegalStateException("Claim standard velocity tuning before the Plant's "
                    + "first update so the tuning workflow owns the complete controller lifetime");
        }
        if (standardControlTuningClaimed) {
            throw new IllegalStateException(
                    "This velocity Plant already supplied its one standard-control tuning handle");
        }
        if (!(regulator instanceof StandardControl)) {
            String path = regulatedPowerChannel == null
                    ? "device-managed/direct velocity output"
                    : "custom ScalarRegulator";
            throw new IllegalStateException("This Plant uses " + path + ", not the Plant-owned "
                    + "Sushi standard velocity controller. Tune it through its owning boundary.");
        }
        StandardControlTuning handle = new StandardControlTuning(
                this, (StandardControl) regulator);
        standardControlTuningClaimed = true;
        return handle;
    }

    ScalarRange tuningTargetRange() {
        return configuredRange;
    }

    boolean hasExactTuningCommandTarget() {
        return PlantTargets.isExactCommand(targetResolver);
    }

    void requireActiveForTuning(String operation) {
        lifecycle.requireActive(operation);
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
        return lifecycle.isActive()
                && (regulatedPowerChannel == null || regulatedActuationCompleted)
                && (regulatedPowerChannel == null
                        || regulatedPowerChannel.setpointSettledAt(appliedTarget))
                && Double.isFinite(target)
                && Double.isFinite(lastMeasurement)
                && Math.abs(requestedTarget - target) <= tolerance
                && Math.abs(appliedTarget - target) <= tolerance
                && Math.abs(lastMeasurement - target) <= tolerance
                && targetStatus.kind() == PlantTargetStatus.Kind.ACCEPTED;
    }

    private void samplePlantMeasurement(LoopClock clock) {
        double nativeValue = nativeMeasurement.getAsDouble(clock);
        if (!lifecycle.isActive()) return;
        lastNativeMeasurement = nativeValue;
        double plantMeasurement = fromNative(nativeValue);
        lastMeasurement = Double.isFinite(plantMeasurement) ? plantMeasurement : Double.NaN;
        if (!Double.isFinite(lastMeasurement)) lastAtTarget = false;
    }

    private double toNative(double plantVelocity) {
        return plantVelocity * nativePerPlantUnit;
    }

    private double fromNative(double nativeVelocity) {
        return nativeVelocity / nativePerPlantUnit;
    }

    /**
     * Proves a fully bounded zero-preserving velocity map using the same multiplication as update.
     */
    static void requireFiniteBoundedMap(ScalarRange range,
                                        double nativePerPlantUnit,
                                        String operation) {
        if (range == null || !range.valid
                || !Double.isFinite(range.minValue)
                || !Double.isFinite(range.maxValue)) {
            return;
        }
        requireFiniteBoundedEndpoint(
                range.minValue, nativePerPlantUnit, operation);
        requireFiniteBoundedEndpoint(
                range.maxValue, nativePerPlantUnit, operation);
    }

    private static void requireFiniteBoundedEndpoint(double plantEndpoint,
                                                     double nativePerPlantUnit,
                                                     String operation) {
        double nativeEndpoint = plantEndpoint * nativePerPlantUnit;
        if (!Double.isFinite(nativeEndpoint)) {
            throw new IllegalArgumentException(operation
                    + " maps bounded Plant velocity endpoint " + plantEndpoint
                    + " to non-finite native velocity " + nativeEndpoint
                    + " with nativePerPlantUnit=" + nativePerPlantUnit
                    + ". Choose finite bounds and a mapping whose exact endpoint images are finite.");
        }
    }

    private void failRuntimeMapping(double priorAppliedTarget,
                                    PlantTargetStatus priorTargetStatus,
                                    PlantTargetResolution priorTargetResolution,
                                    IllegalStateException failure) {
        regulatedActuationCompleted = false;
        lastAtTarget = false;
        restoreTargetState(priorAppliedTarget, priorTargetStatus, priorTargetResolution);
        failStopDirectOutputFailure(
                priorAppliedTarget,
                priorTargetStatus,
                priorTargetResolution,
                priorAppliedTarget,
                priorTargetStatus,
                priorTargetResolution,
                failure);
        throw failure;
    }

    private void failStopDirectOutputFailure(
            double priorAppliedTarget,
            PlantTargetStatus priorTargetStatus,
            PlantTargetResolution priorTargetResolution,
            double effectAppliedTarget,
            PlantTargetStatus effectTargetStatus,
            PlantTargetResolution effectTargetResolution,
            RuntimeException failure) {
        boolean outputStopSucceeded = false;
        try {
            velocityOut.stop();
            outputStopSucceeded = true;
        } catch (RuntimeException cleanupFailure) {
            suppress(failure, cleanupFailure);
        }

        if (!lifecycle.isActive()) {
            if (outputStopSucceeded) {
                markSuccessfullyStopped();
            } else {
                restoreTargetState(
                        effectAppliedTarget, effectTargetStatus, effectTargetResolution);
            }
            return;
        }

        boolean cleanupSucceeded = outputStopSucceeded;
        try {
            targetGuards.reset();
        } catch (RuntimeException cleanupFailure) {
            suppress(failure, cleanupFailure);
            cleanupSucceeded = false;
        }
        if (!lifecycle.isActive()) {
            try {
                enforceTerminalStopAfterReentrantEffect(
                        effectAppliedTarget, effectTargetStatus, effectTargetResolution);
            } catch (RuntimeException cleanupFailure) {
                suppress(failure, cleanupFailure);
            }
            return;
        }
        if (cleanupSucceeded) {
            markSuccessfullyStopped();
        } else {
            restoreTargetState(priorAppliedTarget, priorTargetStatus, priorTargetResolution);
        }
    }

    private void handleRegulatedUpdateFailure(double priorAppliedTarget,
                                              PlantTargetStatus priorTargetStatus,
                                              PlantTargetResolution priorTargetResolution,
                                              RuntimeException failure) {
        regulatedActuationCompleted = false;
        lastAtTarget = false;
        if (!lifecycle.isActive()) return;
        boolean guardCleanupSucceeded = true;
        try {
            targetGuards.reset();
        } catch (RuntimeException cleanupFailure) {
            suppress(failure, cleanupFailure);
            guardCleanupSucceeded = false;
        }
        if (!lifecycle.isActive()) return;
        if (regulatedPowerChannel.lastStopCompleted() && guardCleanupSucceeded) {
            markSuccessfullyStopped();
        } else {
            restoreTargetState(priorAppliedTarget, priorTargetStatus, priorTargetResolution);
        }
    }

    private void markSuccessfullyStopped() {
        appliedTarget = 0.0;
        targetStatus = PlantTargetStatus.STOPPED;
        targetResolution = PlantTargetResolution.unavailable("plant stopped");
    }

    private void enforceTerminalStopAfterReentrantEffect(
            double effectAppliedTarget,
            PlantTargetStatus effectTargetStatus,
            PlantTargetResolution effectTargetResolution) {
        RuntimeException failure = null;
        boolean outputStopSucceeded = false;
        if (regulatedPowerChannel == null) {
            try {
                velocityOut.stop();
                outputStopSucceeded = true;
            } catch (RuntimeException stopFailure) {
                failure = stopFailure;
            }
        } else {
            try {
                // The reentrant public stop already reset the regulator. Only reassert physical
                // zero after the outer hardware callback returns.
                regulatedPowerChannel.reassertTerminalOutputStop();
            } catch (RuntimeException stopFailure) {
                failure = stopFailure;
            }
            outputStopSucceeded = regulatedPowerChannel.lastStopSubmitted();
        }

        if (outputStopSucceeded) {
            markSuccessfullyStopped();
        } else {
            restoreTargetState(effectAppliedTarget, effectTargetStatus, effectTargetResolution);
        }
        if (failure != null) throw failure;
    }

    private void restoreTargetState(double priorAppliedTarget,
                                    PlantTargetStatus priorTargetStatus,
                                    PlantTargetResolution priorTargetResolution) {
        appliedTarget = priorAppliedTarget;
        targetStatus = priorTargetStatus;
        targetResolution = priorTargetResolution;
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
                .addData(p + ".targetRange", configuredRange);
        targetResolver.debugDump(dbg, p + ".targetResolver");
        targetGuards.debugDump(dbg, p + ".targetGuards");
        if (regulatedPowerChannel != null) {
            regulatedPowerChannel.debugDump(dbg, p);
        } else {
            // Preserve the existing debug key for device-managed velocity Plants.
            dbg.addData(p + ".regulatorOutput", 0.0);
        }
    }
}
