package edu.ftcsushi.fw.actuation;

import java.util.Objects;

import edu.ftcsushi.fw.core.control.ScalarRegulator;
import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.hal.PositionOutput;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Source-driven {@link PositionPlant} that maps caller-facing plant units to native hardware units.
 *
 * <p>This package-private runtime is shared by the public FTC and output-port construction
 * gateways. Callers use the {@link Plants} grammar and depend on {@link PositionPlant}.</p>
 *
 * <h2>Coordinate model</h2>
 * <pre>
 * nativePosition = nativeReference + nativePerPlantUnit * (plantPosition - plantReference)
 * </pre>
 *
 * <p>Every configured or runtime reference value must be finite in its documented plant/native
 * units. A reference pair is a coordinate anchor rather than a command and need not lie inside the
 * legal target range. Invalid reference answers are rejected rather than clamped.</p>
 *
 * <p>During normal targeting, the target resolver is invoked once per
 * {@link #update(LoopClock)}. Static plant range and reference validity are enforced first; dynamic
 * hardware protection such as interlocks and target rate limits are applied by
 * {@link PlantTargetGuards}; the result is checked once more for the final finite/range invariant;
 * then one applied target is sent to hardware or the framework regulator. During a calibration
 * search, that same sole owner update instead submits staged temporary search power. Search
 * acquisition rejects non-finite or out-of-range normalized power before changing state or
 * stopping an output; it never clamps that recipe answer. The regulated normal path evaluates its
 * regulator once, normalizes the finite result to
 * {@code [-1.0, +1.0]}, and performs best-effort fail-stop cleanup before propagating a runtime
 * control/output failure.</p>
 *
 * <p>A fully bounded static map proves both exact endpoint images finite at construction. A
 * dynamic reference validates the complete candidate map before committing its anchor or derived
 * public measurement. Unbounded maps remain supported, so the direct position-output path computes
 * each native command into a temporary and rejects a non-finite result before applied-target commit
 * or output. It retains the mapping exception as primary while invoking the output's natural stop
 * best-effort: successful cleanup publishes the same retained-position stopped diagnostic state
 * as {@link #stop()}, while a failed stop restores the prior applied/status/resolution facts and is
 * suppressed. Unlike explicit {@code stop()}, this internal fail-stop does not terminalize the
 * primitive; an isolated deterministic fixture may exercise a different-cycle attempt. A runtime
 * failure escaping any managed or approved advanced host lifecycle phase is nevertheless terminal
 * for that host. A non-finite inverse conversion publishes measurement {@link Double#NaN} and
 * cannot be at target.</p>
 */
final class MappedPositionPlant implements PositionPlant {

    /** Internal acquisition phase keeps reentrant callers from creating a second search owner. */
    private enum SearchState {
        IDLE,
        ACQUIRING,
        ACTIVE
    }

    /**
     * Reference initialization policy for the native/plant coordinate map.
     */
    enum ReferenceMode {
        /**
         * The static {@code plantReference -> nativeReference} mapping is known at build time.
         */
        STATIC,
        /**
         * On the first finite native sample, treat that reading as the configured plant position.
         */
        ASSUME_CURRENT,
        /**
         * The coordinate is invalid until a task establishes a reference.
         */
        NEEDS_REFERENCE
    }

    private final PositionOutput positionOut;
    private final PowerOutput regulatedPowerOut;
    private final ScalarRegulator regulator;
    private final RegulatedPowerChannel regulatedPowerChannel;
    private final ScalarSource nativeMeasurement;
    private final PowerOutput searchPowerOut;
    private final PlantTargetResolver targetResolver;
    private final ScalarTarget commandTarget;
    private final PlantTargetGuards targetGuards;
    private final PlantLifecycle lifecycle = new PlantLifecycle();
    private final PlantUpdateCycle updateCycle = new PlantUpdateCycle("PositionPlant");
    private final Periodicity periodicity;
    private final double period;
    private final ScalarRange configuredRange;
    private final double nativePerPlantUnit;
    private final double tolerance;
    private final ReferenceMode referenceMode;
    private final String unreferencedReason;
    private final double assumedPlantPosition;

    private boolean referenced;
    private boolean pendingAssume;
    private double plantReference;
    private double nativeReference;
    private double requestedTarget;
    private double appliedTarget;
    private double lastMeasurement = Double.NaN;
    private double lastNativeMeasurement = Double.NaN;
    private boolean lastAtTarget;
    private boolean regulatedActuationCompleted;
    private boolean heartbeatAttempted;
    private boolean standardControlTuningClaimed;
    private boolean positionPlantTuningClaimed;
    private boolean normalRealizationInProgress;
    private boolean normalRealizationAttempted;
    private boolean tuningHoldPrepared;
    private boolean tuningHoldPreparationInProgress;
    private SearchState searchState = SearchState.IDLE;
    private double searchPower;
    private PlantTargetStatus targetStatus = PlantTargetStatus.STOPPED;
    private PlantTargetResolution targetResolution = PlantTargetResolution.unavailable("not sampled");

    private MappedPositionPlant(PositionOutput positionOut,
                                PowerOutput regulatedPowerOut,
                                ScalarRegulator regulator,
                                ScalarSource nativeMeasurement,
                                PowerOutput searchPowerOut,
                                PlantTargetResolver targetResolver,
                                PlantTargetGuards targetGuards,
                                Periodicity periodicity,
                                double period,
                                ScalarRange configuredRange,
                                double nativePerPlantUnit,
                                double tolerance,
                                ReferenceMode referenceMode,
                                double plantReference,
                                double nativeReference,
                                double assumedPlantPosition,
                                String unreferencedReason) {
        this.positionOut = positionOut;
        this.regulatedPowerOut = regulatedPowerOut;
        this.regulator = regulator;
        this.nativeMeasurement = nativeMeasurement != null ? nativeMeasurement.memoized() : null;
        this.searchPowerOut = searchPowerOut;
        this.targetResolver = Objects.requireNonNull(targetResolver, "targetResolver");
        this.commandTarget = PlantTargets.commandTargetOf(this.targetResolver);
        this.targetGuards = targetGuards == null ? PlantTargetGuards.none() : targetGuards;
        this.periodicity = Objects.requireNonNull(periodicity, "periodicity");
        this.period = period;
        this.configuredRange = Objects.requireNonNull(configuredRange, "configuredRange");
        this.nativePerPlantUnit = nativePerPlantUnit;
        this.tolerance = tolerance;
        this.referenceMode = Objects.requireNonNull(referenceMode, "referenceMode");
        this.plantReference = plantReference;
        this.nativeReference = nativeReference;
        this.assumedPlantPosition = assumedPlantPosition;
        this.unreferencedReason = (unreferencedReason == null || unreferencedReason.trim().isEmpty())
                ? "position reference not established"
                : unreferencedReason.trim();
        this.referenced = referenceMode == ReferenceMode.STATIC;
        this.pendingAssume = referenceMode == ReferenceMode.ASSUME_CURRENT;
        validate();
        this.regulatedPowerChannel = regulatedPowerOut == null
                ? null
                : new RegulatedPowerChannel(regulatedPowerOut, regulator, "PositionPlant");
    }

    /**
     * Starts configuring a commanded-position plant such as a standard servo.
     * Commanded plants have no feedback and therefore do not use a completion tolerance.
     */
    static CommandedConfigurationStep commanded(PositionOutput out) {
        return new Builder(Objects.requireNonNull(out, "out"), null, null, null);
    }

    /**
     * Starts configuring a position-output plant with native feedback.
     *
     * <p>The staged builder requires an explicit
     * {@link FeedbackConfigurationStep#positionTolerance(double)} in public plant position units
     * before target selection exposes {@link MappedPlantBuildStep#build()}.</p>
     */
    static FeedbackConfigurationStep positionOutput(PositionOutput out,
                                                    ScalarSource nativeMeasurement) {
        return new Builder(Objects.requireNonNull(out, "out"), null, null,
                Objects.requireNonNull(nativeMeasurement, "nativeMeasurement"));
    }

    /**
     * Starts configuring a framework-regulated position plant that drives raw power.
     * The resulting Plant keeps mechanism target units separate from its normalized power command.
     * The staged builder requires an explicit
     * {@link FeedbackConfigurationStep#positionTolerance(double)} in public plant position units
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
     * Optional mapped-position configuration for a command-only output.
     * FTC robot code should normally use {@code FtcActuators}.
     */
    interface CommandedConfigurationStep extends MappedPlantTargetStep<MappedPositionPlant> {
        /** Set non-periodic or periodic position behavior. */
        CommandedConfigurationStep periodicity(Periodicity periodicity, double period);

        /** Set the legal target range in plant position units. */
        CommandedConfigurationStep range(ScalarRange range);

        /** Set how many native position units correspond to one plant position unit. */
        CommandedConfigurationStep nativePerPlantUnit(double nativePerPlantUnit);

        /**
         * Set one known finite static plant-to-native reference pair.
         *
         * <p>The plant value is in plant units and the native value is in native output units. The
         * pair is a coordinate anchor and need not lie inside the legal target range.</p>
         *
         * @throws IllegalArgumentException if either value is non-finite
         */
        CommandedConfigurationStep plantPositionMapsToNative(double plantPosition,
                                                              double nativePosition);
    }

    /**
     * Optional mapped-position configuration before the required plant-unit tolerance answer for
     * a feedback-capable output.
     */
    interface FeedbackConfigurationStep {
        /** Set non-periodic or periodic position behavior. */
        FeedbackConfigurationStep periodicity(Periodicity periodicity, double period);

        /** Set the legal target range in plant position units. */
        FeedbackConfigurationStep range(ScalarRange range);

        /** Set how many native position units correspond to one plant position unit. */
        FeedbackConfigurationStep nativePerPlantUnit(double nativePerPlantUnit);

        /**
         * Set the optional raw-power output used by owner-updated calibration searches.
         * A search stages power, while the Plant's normal
         * {@link MappedPositionPlant#update(LoopClock)} submits it. Runtime search power must be
         * finite in the inclusive normalized {@code [-1.0, +1.0]} range and is rejected before
         * acquisition rather than clamped.
         */
        FeedbackConfigurationStep searchPowerOutput(PowerOutput searchPowerOut);

        /**
         * Set one known finite static plant-to-native reference pair.
         *
         * <p>The plant value is in plant units and the native value is in native feedback/output
         * units. The pair is a coordinate anchor and need not lie inside the legal target range.</p>
         *
         * @throws IllegalArgumentException if either value is non-finite
         */
        FeedbackConfigurationStep plantPositionMapsToNative(double plantPosition,
                                                             double nativePosition);

        /**
         * Establish the supplied finite plant coordinate from the first finite native sample.
         *
         * @throws IllegalArgumentException if {@code plantPosition} is non-finite
         */
        FeedbackConfigurationStep assumeCurrentPositionIs(double plantPosition);

        /** Require an explicit runtime reference before normal position commands. */
        FeedbackConfigurationStep needsReference(String reason);

        /**
         * Set the required completion tolerance in plant position units and continue to target
         * selection.
         */
        MappedPlantTargetStep<MappedPositionPlant> positionTolerance(double tolerance);
    }

    /** Mutable implementation hidden behind the ordered public configuration stages. */
    private static final class Builder implements CommandedConfigurationStep,
            FeedbackConfigurationStep,
            MappedPlantTargetStep<MappedPositionPlant>,
            MappedPlantBuildStep<MappedPositionPlant> {
        private final PositionOutput positionOut;
        private final PowerOutput regulatedPowerOut;
        private final ScalarRegulator regulator;
        private final ScalarSource nativeMeasurement;
        private PowerOutput searchPowerOut;
        private PlantTargetResolver targetResolver;
        private PlantTargetGuards targetGuards = PlantTargetGuards.none();
        private Periodicity periodicity = Periodicity.NON_PERIODIC;
        private double period = Double.NaN;
        private ScalarRange configuredRange = ScalarRange.unbounded();
        private double nativePerPlantUnit = 1.0;
        private double tolerance = Double.NaN;
        private boolean toleranceConfigured;
        private ReferenceMode referenceMode = ReferenceMode.STATIC;
        private double plantReference = 0.0;
        private double nativeReference = 0.0;
        private double assumedPlantPosition = 0.0;
        private String unreferencedReason = "position reference not established";

        private Builder(PositionOutput positionOut,
                        PowerOutput regulatedPowerOut,
                        ScalarRegulator regulator,
                        ScalarSource nativeMeasurement) {
            this.positionOut = positionOut;
            this.regulatedPowerOut = regulatedPowerOut;
            this.regulator = regulator;
            this.nativeMeasurement = nativeMeasurement;
        }

        /**
         * Sets non-periodic/periodic behavior. Period must be finite and positive for periodic plants.
         */
        public Builder periodicity(Periodicity periodicity, double period) {
            this.periodicity = Objects.requireNonNull(periodicity, "periodicity");
            this.period = periodicity == Periodicity.PERIODIC ? period : Double.NaN;
            return this;
        }

        /**
         * Sets the static legal target range in plant units. The configured range must be valid
         * and contain at least one finite command; use {@link #needsReference(String)} when only the
         * position reference is temporarily unavailable.
         */
        public Builder range(ScalarRange range) {
            this.configuredRange = Objects.requireNonNull(range, "range");
            return this;
        }

        /**
         * Sets how many native units correspond to one plant unit.
         */
        public Builder nativePerPlantUnit(double nativePerPlantUnit) {
            if (!Double.isFinite(nativePerPlantUnit) || nativePerPlantUnit == 0.0)
                throw new IllegalArgumentException("nativePerPlantUnit must be finite and non-zero");
            this.nativePerPlantUnit = nativePerPlantUnit;
            return this;
        }

        /**
         * Sets the completion tolerance in plant units for a feedback-capable position plant.
         *
         * @throws IllegalStateException if this builder came from
         *                               {@link MappedPositionPlant#commanded(PositionOutput)}
         */
        public Builder positionTolerance(double tolerance) {
            if (nativeMeasurement == null)
                throw new IllegalStateException("Commanded PositionPlant has no feedback "
                        + "and does not use positionTolerance(...)");
            if (toleranceConfigured)
                throw new IllegalStateException("positionTolerance(...) has already been answered "
                        + "for this PositionPlant recipe");
            if (tolerance < 0.0 || !Double.isFinite(tolerance))
                throw new IllegalArgumentException("positionTolerance must be finite and >= 0");
            this.tolerance = tolerance;
            this.toleranceConfigured = true;
            return this;
        }

        /**
         * Allows calibration-search tasks to stage temporary open-loop power for the Plant owner's
         * normal downstream update phase.
         */
        public Builder searchPowerOutput(PowerOutput searchPowerOut) {
            this.searchPowerOut = searchPowerOut;
            return this;
        }

        /**
         * Uses a static reference: {@code plantPosition} maps to {@code nativePosition}.
         */
        public Builder plantPositionMapsToNative(double plantPosition, double nativePosition) {
            double validatedPlantPosition =
                    PositionCalibrationValueValidation.requireFinitePlantValue(
                            plantPosition,
                            "PositionPlant.plantPositionMapsToNative(...)",
                            "plantPosition");
            double validatedNativePosition =
                    PositionCalibrationValueValidation.requireFiniteNativeValue(
                            nativePosition,
                            "PositionPlant.plantPositionMapsToNative(...)",
                            "nativePosition");
            this.referenceMode = ReferenceMode.STATIC;
            this.plantReference = validatedPlantPosition;
            this.nativeReference = validatedNativePosition;
            return this;
        }

        /**
         * Assumes the first finite native measurement corresponds to {@code plantPosition}.
         */
        public Builder assumeCurrentPositionIs(double plantPosition) {
            double validatedPlantPosition =
                    PositionCalibrationValueValidation.requireFinitePlantValue(
                            plantPosition,
                            "PositionPlant.assumeCurrentPositionIs(...)",
                            "plantPosition");
            this.referenceMode = ReferenceMode.ASSUME_CURRENT;
            this.assumedPlantPosition = validatedPlantPosition;
            return this;
        }

        /**
         * Requires an explicit calibration/reference task before position targets can be applied.
         */
        public Builder needsReference(String reason) {
            this.referenceMode = ReferenceMode.NEEDS_REFERENCE;
            this.unreferencedReason = reason;
            return this;
        }

        /**
         * Sets dynamic plant target guards.
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
                        + "this PositionPlant recipe; create a new recipe to choose a different target");
            }
        }

        /**
         * Builds the mapped position plant.
         *
         * @throws IllegalStateException if no target resolver was configured, or if a feedback plant
         *                               has no explicit plant-unit position tolerance
         * @throws IllegalArgumentException if the configured range cannot contain a finite command,
         *                                  a bounded static-map endpoint is non-finite, or a static
         *                                  guard fallback lies outside that range
         */
        public MappedPositionPlant build() {
            if (targetResolver == null)
                throw new IllegalStateException("PositionPlant requires targetFromResolver(...)");
            if (nativeMeasurement != null && !toleranceConfigured)
                throw new IllegalStateException("PositionPlant feedback requires "
                        + "positionTolerance(...) in plant position units before build()");
            double builtTolerance = nativeMeasurement == null ? 0.0 : tolerance;
            return new MappedPositionPlant(positionOut, regulatedPowerOut, regulator, nativeMeasurement,
                    searchPowerOut, targetResolver, targetGuards, periodicity, period,
                    configuredRange, nativePerPlantUnit, builtTolerance, referenceMode, plantReference,
                    nativeReference, assumedPlantPosition, unreferencedReason);
        }
    }

    private void validate() {
        if (positionOut == null && regulatedPowerOut == null)
            throw new IllegalStateException("PositionPlant requires either a position output or regulated power output");
        if (positionOut != null && regulatedPowerOut != null)
            throw new IllegalStateException("PositionPlant cannot use both a position output and regulated power output");
        if (regulatedPowerOut != null && regulator == null)
            throw new IllegalStateException("Regulated position plants require a regulator");
        if (periodicity == Periodicity.PERIODIC && (!(period > 0.0) || !Double.isFinite(period)))
            throw new IllegalArgumentException("Periodic position plants require finite period > 0");
        if (!Double.isFinite(nativePerPlantUnit) || nativePerPlantUnit == 0.0)
            throw new IllegalArgumentException("nativePerPlantUnit must be finite and non-zero");
        if (tolerance < 0.0 || !Double.isFinite(tolerance))
            throw new IllegalArgumentException("positionTolerance must be finite and >= 0");
        PlantTargetSafety.requireUsableConfiguredRange(configuredRange, "PositionPlant");
        targetGuards.validateFallbackTargets(configuredRange, "PositionPlant");
        if ((referenceMode == ReferenceMode.ASSUME_CURRENT || referenceMode == ReferenceMode.NEEDS_REFERENCE) && nativeMeasurement == null) {
            throw new IllegalStateException(referenceMode + " requires native feedback so a reference can be established");
        }
        if (referenceMode == ReferenceMode.STATIC) {
            requireFiniteBoundedMap(
                    configuredRange,
                    nativePerPlantUnit,
                    plantReference,
                    nativeReference,
                    "PositionPlant configuration");
        }
    }

    @Override
    public void update(LoopClock clock) {
        if (!lifecycle.isActive()) return;
        if (tuningHoldPreparationInProgress) {
            throw new IllegalStateException("PositionPlant.update(...) is unavailable while "
                    + "position tuning hold preparation is in progress");
        }
        heartbeatAttempted = true;
        if (!updateCycle.begin(clock)) return;
        try {
            updateOnce(clock);
            updateCycle.succeed();
        } catch (RuntimeException failure) {
            updateCycle.fail(failure);
            throw failure;
        } finally {
            normalRealizationInProgress = false;
        }
    }

    private void updateOnce(LoopClock clock) {
        if (!lifecycle.isActive()) return;
        // ASSUME_CURRENT can establish the reference and continue into normal output in this same
        // heartbeat. Reserve that possible normal realization before sampling the external source
        // so a reentrant callback cannot claim a pre-output preparation handle midway through it.
        // A search heartbeat remains eligible, and a non-finite sample leaves
        // normalRealizationAttempted false so a later claim/retry is still allowed.
        if (pendingAssume && searchState == SearchState.IDLE) {
            normalRealizationInProgress = true;
        }
        if (pendingAssume) {
            double nativeNow = sampleNative(clock);
            if (!lifecycle.isActive()) return;
            if (Double.isFinite(nativeNow))
                establishReferenceFromNative(assumedPlantPosition, nativeNow);
        }
        if (!lifecycle.isActive()) return;

        if (searchState != SearchState.IDLE) {
            // A reentrant update during acquisition must neither resume normal targeting nor submit
            // unstaged search power. The normally returning begin makes the next owner update active.
            if (searchState == SearchState.ACQUIRING) {
                targetResolution = PlantTargetResolution.holdLast(
                        appliedTarget, "calibration search acquisition in progress");
                lastAtTarget = false;
                targetStatus = PlantTargetStatus.holdingLast(
                        "calibration search acquisition in progress");
                return;
            }
            samplePlantMeasurement(clock);
            if (!lifecycle.isActive()) return;
            double effectAppliedTarget = appliedTarget;
            PlantTargetStatus effectTargetStatus = targetStatus;
            PlantTargetResolution effectTargetResolution = targetResolution;
            RuntimeException effectFailure = null;
            try {
                if (searchPowerOut != null) searchPowerOut.setPower(searchPower);
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
                    failStopOutputFailure(
                            effectAppliedTarget,
                            effectTargetStatus,
                            effectTargetResolution,
                            effectAppliedTarget,
                            effectTargetStatus,
                            effectTargetResolution,
                            effectFailure);
                }
                throw effectFailure;
            }
            if (!lifecycle.isActive()) return;
            targetResolution = PlantTargetResolution.holdLast(appliedTarget, "calibration search active");
            lastAtTarget = false;
            targetStatus = PlantTargetStatus.holdingLast("calibration search active");
            return;
        }

        normalRealizationInProgress = true;
        // Reserve the normal realization before sampling resolver/guard callbacks so a reentrant
        // tuning claim cannot stage a hold after this referenced update has already begun.
        if (isReferenced()) normalRealizationAttempted = true;

        double priorAppliedTarget = appliedTarget;
        PlantTargetStatus priorTargetStatus = targetStatus;
        PlantTargetResolution priorTargetResolution = targetResolution;
        samplePlantMeasurement(clock);
        if (!lifecycle.isActive()) return;
        ScalarRange range = targetRange();
        PlantTargetContext context = PlantTargetContext.position(hasFeedback(), lastMeasurement,
                range, periodicity, period(), requestedTarget, appliedTarget);
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
        if (!range.valid) {
            status = PlantTargetStatus.referenceNotEstablished(range.reason);
            candidate = appliedTarget;
        } else {
            double clamped = range.clamp(candidate);
            if (!range.contains(candidate))
                status = PlantTargetStatus.clampedToRange("target clamped to position range");
            candidate = clamped;
        }

        PlantTargetGuards.Result guarded = PlantTargetSafety.applyGuards(
                targetGuards, candidate, status, appliedTarget, range, "position", clock);
        if (!lifecycle.isActive()) return;
        requestedTarget = nextRequestedTarget;

        double nativeCommand = Double.NaN;
        if (isReferenced() && positionOut != null) {
            nativeCommand = toNative(guarded.target);
            if (!Double.isFinite(nativeCommand)) {
                IllegalStateException failure = new IllegalStateException(
                        "PositionPlant.update(...) could not map finite Plant target "
                                + guarded.target + " to a finite native position using "
                                + "nativePosition = nativeReference + nativePerPlantUnit * "
                                + "(plantPosition - plantReference), with nativePerPlantUnit="
                                + nativePerPlantUnit + ", plantReference=" + plantReference
                                + ", nativeReference=" + nativeReference
                                + ", and result=" + nativeCommand
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

        if (!isReferenced()) {
            stopNormalPositionOutput();
            lastAtTarget = false;
            return;
        }
        // A callback may have established the reference after the initial reservation above.
        // Permanently close tuning preparation before this newly referenced update can actuate.
        normalRealizationAttempted = true;
        if (positionOut != null) {
            RuntimeException effectFailure = null;
            try {
                positionOut.setPosition(nativeCommand);
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
                    failStopOutputFailure(
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
        } else if (regulatedPowerChannel != null) {
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
        // Relinquish temporary search ownership before any external stop callback. Even when a
        // stop throws, a later update must not refresh search power.
        searchState = SearchState.IDLE;

        StopOutcome outputStop = stopOwnedOutputs();
        RuntimeException primary = outputStop.failure;
        if (outputStop.allOutputsStopped) {
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
        if (commandTarget == null) return PositionPlant.super.commandTarget();
        return commandTarget;
    }

    StandardControlTuning claimStandardControlTuning() {
        if (!lifecycle.isActive()) {
            throw new IllegalStateException(
                    "Cannot claim standard position tuning after the Plant has stopped");
        }
        if (heartbeatAttempted) {
            throw new IllegalStateException("Claim standard position tuning before the Plant's "
                    + "first update so the tuning workflow owns the complete controller lifetime");
        }
        if (standardControlTuningClaimed) {
            throw new IllegalStateException(
                    "This position Plant already supplied its one standard-control tuning handle");
        }
        if (!(regulator instanceof StandardControl)) {
            String path;
            if (regulatedPowerChannel == null) {
                path = positionOut == null
                        ? "a non-regulated position path"
                        : "a direct/device-managed position output";
            } else {
                path = "a custom ScalarRegulator";
            }
            throw new IllegalStateException("This Plant uses " + path + ", not the Plant-owned "
                    + "Sushi standard position controller. Tune it through its owning boundary.");
        }
        StandardControlTuning handle = new StandardControlTuning(
                this, (StandardControl) regulator);
        standardControlTuningClaimed = true;
        return handle;
    }

    PositionPlantTuning claimPositionPlantTuning() {
        if (!lifecycle.isActive()) {
            throw new IllegalStateException(
                    "Cannot claim position tuning after the Plant has stopped");
        }
        if (normalRealizationInProgress || normalRealizationAttempted) {
            throw new IllegalStateException("Claim position tuning before the Plant's first normal "
                    + "position realization and never from inside one. Calibration-search and "
                    + "still-unreferenced heartbeats may run first.");
        }
        if (positionPlantTuningClaimed) {
            throw new IllegalStateException(
                    "This position Plant already supplied its one position-tuning handle");
        }
        PositionPlantTuning handle = new PositionPlantTuning(this);
        positionPlantTuningClaimed = true;
        return handle;
    }

    boolean hasExactTuningCommandTarget() {
        return PlantTargets.isExactCommand(targetResolver);
    }

    void requireActiveForTuning(String operation) {
        lifecycle.requireActive(operation);
    }

    double prepareTuningHoldAtCurrent(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        lifecycle.requireActive("Position tuning hold preparation");
        if (tuningHoldPreparationInProgress) {
            throw new IllegalStateException(
                    "Position tuning hold preparation is already in progress");
        }
        if (!positionPlantTuningClaimed) {
            throw new IllegalStateException(
                    "Position hold preparation requires PositionPlantTunings.claim(...) first");
        }
        if (normalRealizationInProgress || normalRealizationAttempted) {
            throw new IllegalStateException("Prepare the initial position hold before the Plant's "
                    + "first normal realization and never from inside one; calibration-search "
                    + "and still-unreferenced heartbeats may run first");
        }
        if (searchState != SearchState.IDLE) {
            throw new IllegalStateException("Position tuning hold preparation is unavailable while "
                    + "calibration search owns output; end the search before sampling its hold");
        }
        if (tuningHoldPrepared) {
            throw new IllegalStateException(
                    "This position Plant already prepared its one pre-realization tuning hold");
        }
        if (!PlantTargets.isExactCommand(targetResolver) || commandTarget == null) {
            throw new IllegalStateException("Position tuning hold preparation requires one exact "
                    + "graph-owned command. Equivalent-position, overlay, planned, and read-only "
                    + "target graphs are not eligible.");
        }
        if (referenceMode == ReferenceMode.NEEDS_REFERENCE && !referenced) {
            throw new IllegalStateException("This PositionPlant needs a physical reference. Run "
                    + "the tuning workflow's reference Task before preparing its normal hold.");
        }

        tuningHoldPreparationInProgress = true;
        try {
            double nativeNow = sampleNative(clock);
            lifecycle.requireActive("Position tuning hold preparation");
            if (!Double.isFinite(nativeNow)) {
                throw new IllegalStateException("Position tuning hold preparation requires a "
                        + "finite native position sample; received " + nativeNow);
            }
            final boolean establishAssumedReference = pendingAssume;
            final double candidateMeasurement;
            if (establishAssumedReference) {
                try {
                    requireFiniteBoundedMap(
                            configuredRange,
                            nativePerPlantUnit,
                            assumedPlantPosition,
                            nativeNow,
                            "Position tuning hold preparation");
                } catch (IllegalArgumentException invalidMap) {
                    throw new IllegalStateException(invalidMap.getMessage(), invalidMap);
                }
                candidateMeasurement = assumedPlantPosition;
            } else {
                candidateMeasurement = toPlant(nativeNow);
            }
            if (!Double.isFinite(candidateMeasurement)) {
                throw new IllegalStateException("Position tuning hold preparation could not "
                        + "convert native position " + nativeNow
                        + " to a finite Plant position");
            }
            if (!configuredRange.contains(candidateMeasurement)) {
                throw new IllegalStateException("Position tuning hold preparation measured Plant "
                        + "position " + candidateMeasurement
                        + " outside the declared target range " + configuredRange
                        + "; Sushi will not silently clamp an initial hold");
            }

            double priorCommand = commandTarget.get();
            try {
                commandTarget.set(candidateMeasurement);
                lifecycle.requireActive("Position tuning hold preparation");
            } catch (RuntimeException failure) {
                try {
                    commandTarget.set(priorCommand);
                } catch (RuntimeException restoreFailure) {
                    if (restoreFailure != failure) failure.addSuppressed(restoreFailure);
                }
                throw failure;
            }
            if (establishAssumedReference) {
                // Complete map validation above makes this assignment-only reference commit safe.
                establishReferenceFromNative(assumedPlantPosition, nativeNow);
            } else {
                lastNativeMeasurement = nativeNow;
                lastMeasurement = candidateMeasurement;
            }
            tuningHoldPrepared = true;
            return candidateMeasurement;
        } finally {
            tuningHoldPreparationInProgress = false;
        }
    }

    PositionPlantTuning.RecoveryHold prepareTuningRecoveryHoldWithin(
            ScalarRange allowedPhysicalRange,
            LoopClock clock) {
        Objects.requireNonNull(allowedPhysicalRange, "allowedPhysicalRange");
        Objects.requireNonNull(clock, "clock");
        lifecycle.requireActive("Position tuning recovery-hold preparation");
        if (tuningHoldPreparationInProgress) {
            throw new IllegalStateException(
                    "Position tuning hold preparation is already in progress");
        }
        if (!positionPlantTuningClaimed || !tuningHoldPrepared) {
            throw new IllegalStateException("Prepare the initial position tuning hold before "
                    + "requesting a recovery hold");
        }
        if (updateCycle.wasAttemptedIn(clock)) {
            throw new IllegalStateException("Prepare a position recovery hold before the Plant's "
                    + "normal update in that LoopClock cycle");
        }
        if (searchState != SearchState.IDLE) {
            throw new IllegalStateException(
                    "Position recovery hold is unavailable while calibration search owns output");
        }
        if (!referenced) {
            throw new IllegalStateException(
                    "Position recovery hold requires an established coordinate reference");
        }
        if (!PlantTargets.isExactCommand(targetResolver) || commandTarget == null) {
            throw new IllegalStateException("Position recovery hold requires one exact "
                    + "graph-owned command");
        }
        if (!allowedPhysicalRange.valid
                || !Double.isFinite(allowedPhysicalRange.minValue)
                || !Double.isFinite(allowedPhysicalRange.maxValue)) {
            throw new IllegalArgumentException(
                    "Position recovery-hold range must be finite and bounded; got "
                            + allowedPhysicalRange);
        }
        if (!configuredRange.contains(allowedPhysicalRange.minValue)
                || !configuredRange.contains(allowedPhysicalRange.maxValue)) {
            throw new IllegalArgumentException("Position recovery-hold range "
                    + allowedPhysicalRange + " must lie inside the Plant target range "
                    + configuredRange);
        }

        tuningHoldPreparationInProgress = true;
        try {
            double nativeNow = sampleNative(clock);
            lifecycle.requireActive("Position tuning recovery-hold preparation");
            double measurement = Double.isFinite(nativeNow) ? toPlant(nativeNow) : Double.NaN;
            if (!Double.isFinite(measurement)) {
                throw new IllegalStateException("Position recovery hold requires a finite "
                        + "same-cycle measurement; native=" + nativeNow
                        + ", plant=" + measurement);
            }
            double holdTarget = allowedPhysicalRange.clamp(measurement);
            if (!Double.isFinite(holdTarget) || !configuredRange.contains(holdTarget)) {
                throw new IllegalStateException("Position recovery hold could not choose a finite "
                        + "legal target from measurement " + measurement + " and range "
                        + allowedPhysicalRange);
            }

            double priorCommand = commandTarget.get();
            try {
                commandTarget.set(holdTarget);
                lifecycle.requireActive("Position tuning recovery-hold preparation");
            } catch (RuntimeException failure) {
                try {
                    commandTarget.set(priorCommand);
                } catch (RuntimeException restoreFailure) {
                    if (restoreFailure != failure) failure.addSuppressed(restoreFailure);
                }
                throw failure;
            }
            lastNativeMeasurement = nativeNow;
            lastMeasurement = measurement;
            lastAtTarget = false;
            return new PositionPlantTuning.RecoveryHold(measurement, holdTarget);
        } finally {
            tuningHoldPreparationInProgress = false;
        }
    }

    @Override
    public boolean hasFeedback() {
        return nativeMeasurement != null;
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
                && hasFeedback()
                && Double.isFinite(target)
                && Double.isFinite(lastMeasurement)
                && Math.abs(requestedTarget - target) <= tolerance
                && Math.abs(appliedTarget - target) <= tolerance
                && Math.abs(lastMeasurement - target) <= tolerance
                && targetStatus.kind() == PlantTargetStatus.Kind.ACCEPTED;
    }

    @Override
    public Periodicity periodicity() {
        return periodicity;
    }

    @Override
    public double period() {
        return periodicity == Periodicity.PERIODIC ? period : Double.NaN;
    }

    @Override
    public ScalarRange targetRange() {
        if (!isReferenced()) return ScalarRange.invalid(referenceStatus());
        return configuredRange;
    }

    @Override
    public boolean isReferenced() {
        return referenced;
    }

    @Override
    public String referenceStatus() {
        if (referenced) return "referenced";
        if (pendingAssume) return "reference pending first finite native sample";
        return unreferencedReason;
    }

    /**
     * {@inheritDoc}
     *
     * <p>If an already referenced periodic mapping cannot produce both a finite current Plant
     * estimate and a finite final nearest-equivalent result, this implementation fails before
     * committing the new reference.</p>
     *
     * @throws IllegalStateException if no finite native measurement has been cached, the periodic
     *                               result described above is non-finite, or the candidate bounded
     *                               map has a non-finite endpoint image
     */
    @Override
    public void establishReferenceAt(double plantPosition) {
        lifecycle.requireActive("PositionPlant.establishReferenceAt(...)");
        double validatedPlantPosition =
                PositionCalibrationValueValidation.requireFinitePlantValue(
                        plantPosition,
                        "PositionPlant.establishReferenceAt(...)",
                        "plantPosition");
        if (!Double.isFinite(lastNativeMeasurement))
            throw new IllegalStateException("Cannot establish position reference before a finite native measurement has been sampled");
        establishReferenceFromNative(validatedPlantPosition, lastNativeMeasurement);
    }

    /**
     * {@inheritDoc}
     *
     * <p>If an already referenced periodic mapping cannot produce both a finite current Plant
     * estimate and a finite final nearest-equivalent result, this implementation fails before
     * committing the new reference.</p>
     *
     * @throws IllegalStateException if the current native sample is non-finite, the periodic result
     *                               described above is non-finite, or the candidate bounded map has
     *                               a non-finite endpoint image
     */
    @Override
    public void establishReferenceAt(double plantPosition, LoopClock clock) {
        lifecycle.requireActive("PositionPlant.establishReferenceAt(...)");
        double validatedPlantPosition =
                PositionCalibrationValueValidation.requireFinitePlantValue(
                        plantPosition,
                        "PositionPlant.establishReferenceAt(...)",
                        "plantPosition");
        double nativeNow = sampleNative(clock);
        lifecycle.requireActive("PositionPlant.establishReferenceAt(...)");
        if (!Double.isFinite(nativeNow))
            throw new IllegalStateException("Cannot establish position reference from non-finite native measurement");
        establishReferenceFromNative(validatedPlantPosition, nativeNow);
    }

    @Override
    public boolean supportsCalibrationSearch() {
        return searchPowerOut != null;
    }

    /**
     * {@inheritDoc}
     *
     * <p>This implementation reports unsupported capability first, an existing active owner
     * second, and invalid normalized power third. Any rejection occurs before acquisition,
     * callback, controller, status, feedback, or output effects.</p>
     */
    @Override
    public void beginCalibrationSearch(double power) {
        lifecycle.requireActive("PositionPlant.beginCalibrationSearch(...)");
        if (searchPowerOut == null)
            throw new IllegalStateException("This PositionPlant does not support calibration search drive");
        if (searchState != SearchState.IDLE)
            throw new IllegalStateException("This PositionPlant already has an active calibration "
                    + "search; cancel or finish that search before starting another one");
        CalibrationSearchPowerValidation.requireValid(
                power, "PositionPlant.beginCalibrationSearch(...)");

        // Reserve before calling an external output so a reentrant or overlapping begin fails
        // before it can stop or replace this search. A throwing begin releases this reservation.
        searchState = SearchState.ACQUIRING;
        try {
            stopNormalPositionOutput();
            if (searchState != SearchState.ACQUIRING) {
                throw new IllegalStateException("Calibration search acquisition was released "
                        + "reentrantly before beginCalibrationSearch(...) completed");
            }
            searchPower = power;
            targetStatus = PlantTargetStatus.holdingLast("calibration search active");
            searchState = SearchState.ACTIVE;
        } catch (RuntimeException failure) {
            searchState = SearchState.IDLE;
            throw failure;
        }
    }

    @Override
    public void endCalibrationSearch() {
        if (searchState == SearchState.IDLE) return;
        // Clear ownership first. If the external stop throws, later updates return to the normal
        // target graph instead of silently refreshing raw search power.
        searchState = SearchState.IDLE;
        if (searchPowerOut != null) searchPowerOut.stop();
    }

    private void stopNormalPositionOutput() {
        regulatedActuationCompleted = false;
        lastAtTarget = false;
        if (positionOut != null) positionOut.stop();
        if (regulatedPowerChannel != null) regulatedPowerChannel.stopWhileActive(lifecycle);
    }

    private double sampleNative(LoopClock clock) {
        if (nativeMeasurement == null) {
            lastNativeMeasurement = Double.NaN;
            return Double.NaN;
        }
        double nativeValue = nativeMeasurement.getAsDouble(clock);
        if (!lifecycle.isActive()) return Double.NaN;
        lastNativeMeasurement = nativeValue;
        return nativeValue;
    }

    private void samplePlantMeasurement(LoopClock clock) {
        double nativeValue = sampleNative(clock);
        if (!lifecycle.isActive()) return;
        double plantMeasurement = isReferenced() && Double.isFinite(nativeValue)
                ? toPlant(nativeValue)
                : Double.NaN;
        lastMeasurement = Double.isFinite(plantMeasurement) ? plantMeasurement : Double.NaN;
        if (!Double.isFinite(lastMeasurement)) lastAtTarget = false;
    }

    private void establishReferenceFromNative(double requestedPlantReference, double nativeAtReference) {
        double resolvedPlantReference = requestedPlantReference;
        if (periodicity == Periodicity.PERIODIC && referenced) {
            // Resolve from the native sample supplied for this reference operation, not a cached
            // Plant measurement from an earlier owner update.
            double plantAtReference = toPlant(nativeAtReference);
            if (!Double.isFinite(plantAtReference)) {
                throw new IllegalStateException("PositionPlant.establishReferenceAt(...) "
                        + "could not convert native position " + nativeAtReference
                        + " to a finite current Plant position before resolving the periodic "
                        + "reference. Check coordinate magnitudes and the plant/native map.");
            }
            double k = Math.rint((plantAtReference - requestedPlantReference) / period);
            resolvedPlantReference = requestedPlantReference + k * period;
            if (!Double.isFinite(resolvedPlantReference)) {
                throw new IllegalStateException("PositionPlant.establishReferenceAt(...) "
                        + "could not resolve a finite periodic reference from requested plant "
                        + "position " + requestedPlantReference + ", current plant position "
                        + plantAtReference + ", and period " + period
                        + ". Check coordinate magnitudes and the plant/native map.");
            }
        }
        try {
            requireFiniteBoundedMap(
                    configuredRange,
                    nativePerPlantUnit,
                    resolvedPlantReference,
                    nativeAtReference,
                    "PositionPlant.establishReferenceAt(...)");
        } catch (IllegalArgumentException invalidCandidate) {
            IllegalStateException failure = new IllegalStateException(
                    invalidCandidate.getMessage(), invalidCandidate);
            failRuntimeMapping(
                    appliedTarget,
                    targetStatus,
                    targetResolution,
                    failure);
        }
        plantReference = resolvedPlantReference;
        nativeReference = nativeAtReference;
        referenced = true;
        pendingAssume = false;
        lastMeasurement = resolvedPlantReference;
        lastNativeMeasurement = nativeAtReference;
    }

    private double toNative(double plantPosition) {
        return nativeReference + nativePerPlantUnit * (plantPosition - plantReference);
    }

    private double toPlant(double nativePosition) {
        return plantReference + (nativePosition - nativeReference) / nativePerPlantUnit;
    }

    /**
     * Proves a fully bounded affine position map using update's exact operation order.
     */
    static void requireFiniteBoundedMap(ScalarRange range,
                                        double nativePerPlantUnit,
                                        double plantReference,
                                        double nativeReference,
                                        String operation) {
        if (range == null || !range.valid
                || !Double.isFinite(range.minValue)
                || !Double.isFinite(range.maxValue)) {
            return;
        }
        requireFiniteBoundedEndpoint(
                range.minValue,
                nativePerPlantUnit,
                plantReference,
                nativeReference,
                operation);
        requireFiniteBoundedEndpoint(
                range.maxValue,
                nativePerPlantUnit,
                plantReference,
                nativeReference,
                operation);
    }

    private static void requireFiniteBoundedEndpoint(double plantEndpoint,
                                                     double nativePerPlantUnit,
                                                     double plantReference,
                                                     double nativeReference,
                                                     String operation) {
        double nativeEndpoint = nativeReference
                + nativePerPlantUnit * (plantEndpoint - plantReference);
        if (!Double.isFinite(nativeEndpoint)) {
            throw new IllegalArgumentException(operation
                    + " maps bounded Plant position endpoint " + plantEndpoint
                    + " to non-finite native position " + nativeEndpoint
                    + " with nativePerPlantUnit=" + nativePerPlantUnit
                    + ", plantReference=" + plantReference
                    + ", and nativeReference=" + nativeReference
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
        failStopOutputFailure(
                priorAppliedTarget,
                priorTargetStatus,
                priorTargetResolution,
                priorAppliedTarget,
                priorTargetStatus,
                priorTargetResolution,
                failure);
        throw failure;
    }

    private void failStopOutputFailure(
            double priorAppliedTarget,
            PlantTargetStatus priorTargetStatus,
            PlantTargetResolution priorTargetResolution,
            double effectAppliedTarget,
            PlantTargetStatus effectTargetStatus,
            PlantTargetResolution effectTargetResolution,
            RuntimeException failure) {
        regulatedActuationCompleted = false;
        lastAtTarget = false;
        searchState = SearchState.IDLE;
        StopOutcome outputStop = stopOwnedOutputs();
        if (outputStop.failure != null) suppress(failure, outputStop.failure);

        if (!lifecycle.isActive()) {
            if (outputStop.allOutputsStopped) {
                markSuccessfullyStopped();
            } else {
                restoreTargetState(
                        effectAppliedTarget, effectTargetStatus, effectTargetResolution);
            }
            return;
        }

        boolean cleanupSucceeded = outputStop.allOutputsStopped && outputStop.failure == null;
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
        targetStatus = PlantTargetStatus.STOPPED;
        targetResolution = PlantTargetResolution.unavailable("plant stopped");
    }

    private void enforceTerminalStopAfterReentrantEffect(
            double effectAppliedTarget,
            PlantTargetStatus effectTargetStatus,
            PlantTargetResolution effectTargetResolution) {
        searchState = SearchState.IDLE;
        StopOutcome outputStop = regulatedPowerChannel == null
                ? stopOwnedOutputs()
                : reassertRegulatedTerminalOutputs();
        if (outputStop.allOutputsStopped) {
            markSuccessfullyStopped();
        } else {
            restoreTargetState(effectAppliedTarget, effectTargetStatus, effectTargetResolution);
        }
        if (outputStop.failure != null) throw outputStop.failure;
    }

    /** Reassert only physical terminal outputs; the reentrant public stop already reset control. */
    private StopOutcome reassertRegulatedTerminalOutputs() {
        RuntimeException failure = null;
        try {
            regulatedPowerChannel.reassertTerminalOutputStop(searchPowerOut);
        } catch (RuntimeException stopFailure) {
            failure = stopFailure;
        }
        return new StopOutcome(regulatedPowerChannel.lastStopSubmitted(), failure);
    }

    /** Stop every distinct realization output without changing this Plant's lifecycle latch. */
    private StopOutcome stopOwnedOutputs() {
        RuntimeException primary = null;
        boolean allOutputStopsSucceeded = true;
        if (positionOut != null) {
            try {
                positionOut.stop();
            } catch (RuntimeException failure) {
                allOutputStopsSucceeded = false;
                primary = failure;
            }
        }
        if (regulatedPowerChannel != null) {
            try {
                regulatedPowerChannel.stop(searchPowerOut);
            } catch (RuntimeException failure) {
                primary = suppress(primary, failure);
            }
            allOutputStopsSucceeded &= regulatedPowerChannel.lastStopSubmitted();
        }
        if (searchPowerOut != null && regulatedPowerChannel == null) {
            try {
                searchPowerOut.stop();
            } catch (RuntimeException failure) {
                allOutputStopsSucceeded = false;
                primary = suppress(primary, failure);
            }
        }
        return new StopOutcome(allOutputStopsSucceeded, primary);
    }

    private static final class StopOutcome {
        private final boolean allOutputsStopped;
        private final RuntimeException failure;

        private StopOutcome(boolean allOutputsStopped, RuntimeException failure) {
            this.allOutputsStopped = allOutputsStopped;
            this.failure = failure;
        }
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
        PositionPlant.super.debugDump(dbg, prefix);
        String p = (prefix == null || prefix.isEmpty()) ? "positionPlant" : prefix;
        dbg.addData(p + ".periodicity", periodicity)
                .addData(p + ".period", period())
                .addData(p + ".range", targetRange())
                .addData(p + ".referenced", referenced)
                .addData(p + ".referenceStatus", referenceStatus())
                .addData(p + ".nativePerPlantUnit", nativePerPlantUnit)
                .addData(p + ".plantReference", plantReference)
                .addData(p + ".nativeReference", nativeReference)
                .addData(p + ".lastNativeMeasurement", lastNativeMeasurement)
                .addData(p + ".searchActive", searchState != SearchState.IDLE)
                .addData(p + ".searchState", searchState);
        if (regulatedPowerChannel != null) {
            dbg.addData(p + ".lastRegulatorOutput", regulatedPowerChannel.regulatorOutput());
            regulatedPowerChannel.debugDump(dbg, p);
        } else {
            // Preserve the existing debug key for position-output Plants.
            dbg.addData(p + ".lastRegulatorOutput", 0.0);
        }
        targetResolver.debugDump(dbg, p + ".targetResolver");
        targetGuards.debugDump(dbg, p + ".targetGuards");
    }
}
