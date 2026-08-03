package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Hardware-neutral staged construction for source-driven {@link Plant Plants}.
 *
 * <p>Most FTC robot code should use {@code FtcActuators.plant(...)}. Custom adapters, portable
 * hosts, and hardware-neutral tests that already own Phoenix output ports use
 * {@link #fromOutputs()}. The staged grammar asks each required coordinate, feedback, range,
 * mapping, tolerance, guard, and target question once, then hides the concrete runtime.</p>
 */
public final class Plants {

    private static final ScalarRange NORMALIZED_POWER_RANGE = ScalarRange.bounded(-1.0, 1.0);

    private Plants() {
    }

    /**
     * Start the sole hardware-neutral Plant construction grammar.
     *
     * <p>Choose exactly one of the six output-path roots. A successful root choice consumes this
     * start object; use a fresh call for another Plant. Configuration and {@link BuildStep#build()}
     * do not command the supplied outputs. Normal actuation begins only from the owner-called
     * {@link Plant#update(LoopClock)}; explicit stop and calibration lifecycle calls retain their
     * documented immediate fail-safe behavior.</p>
     *
     * @return a fresh, single-use output-path selection step
     */
    public static FromOutputsStep fromOutputs() {
        return new OutputStart();
    }

    /** First step for selecting the complete hardware-neutral command path. */
    public interface FromOutputsStep {
        /**
         * Select a direct normalized-power Plant with the fixed target range {@code [-1, +1]}.
         *
         * @param out normalized power output commanded only by the built Plant's lifecycle methods
         * @return the shared guard and target-selection step
         * @throws NullPointerException if {@code out} is null
         * @throws IllegalStateException if this root step already selected an output path
         */
        TargetStep<Plant> power(PowerOutput out);

        /**
         * Select a command-only position Plant over a native {@link PositionOutput}.
         *
         * <p>The following stages require caller-facing periodicity, legal Plant-unit bounds, and
         * the Plant-to-native map. This path has no feedback, reference acquisition, or completion
         * tolerance.</p>
         *
         * @param out native position output commanded only by the built Plant's lifecycle methods
         * @return the required position-periodicity step
         * @throws NullPointerException if {@code out} is null
         * @throws IllegalStateException if this root step already selected an output path
         */
        PositionPeriodicityStep<CommandedPositionBoundsStep> commandedPosition(PositionOutput out);

        /**
         * Select device-managed position output with an explicit native-position feedback source.
         *
         * <p>The device owns position actuation. The Plant uses {@code nativeMeasurement} for
         * reference and completion, and may optionally receive a distinct raw-power output for
         * calibration search before periodicity is answered.</p>
         *
         * @param out native position output
         * @param nativeMeasurement position feedback in the output adapter's native units
         * @return the optional search-output and required position-periodicity step
         * @throws NullPointerException if either argument is null
         * @throws IllegalStateException if this root step already selected an output path
         */
        DeviceManagedPositionStep deviceManagedPosition(PositionOutput out,
                                                        ScalarSource nativeMeasurement);

        /**
         * Select device-managed velocity output with explicit native-velocity feedback.
         *
         * @param out native velocity output
         * @param nativeMeasurement velocity feedback in the output adapter's native units per second
         * @return the required Plant-unit velocity-bounds step
         * @throws NullPointerException if either argument is null
         * @throws IllegalStateException if this root step already selected an output path
         */
        VelocityBoundsStep deviceManagedVelocity(VelocityOutput out,
                                                 ScalarSource nativeMeasurement);

        /**
         * Select framework-regulated position over normalized power and native-position feedback.
         *
         * <p>The same {@code out} is used both for normal regulation and calibration search; no
         * second search-output answer is required or supported.</p>
         *
         * @param out normalized power output
         * @param nativeMeasurement position feedback in native units
         * @param regulator regulator that converts Plant-unit position error to normalized power
         * @return the required position-periodicity step
         * @throws NullPointerException if any argument is null
         * @throws IllegalStateException if this root step already selected an output path
         */
        PositionPeriodicityStep<FeedbackPositionBoundsStep> regulatedPosition(
                PowerOutput out,
                ScalarSource nativeMeasurement,
                ScalarRegulator regulator);

        /**
         * Select framework-regulated velocity over normalized power and native-velocity feedback.
         *
         * @param out normalized power output
         * @param nativeMeasurement velocity feedback in native units per second
         * @param regulator regulator that converts Plant-unit velocity error to normalized power
         * @return the required Plant-unit velocity-bounds step
         * @throws NullPointerException if any argument is null
         * @throws IllegalStateException if this root step already selected an output path
         */
        VelocityBoundsStep regulatedVelocity(PowerOutput out,
                                             ScalarSource nativeMeasurement,
                                             ScalarRegulator regulator);
    }

    /** Shared periodicity question for every position construction path. */
    public interface PositionPeriodicityStep<NEXT> {
        /**
         * Declare that the caller-facing Plant position has no fixed equivalence period.
         *
         * @return the position-bounds step for this path
         * @throws IllegalStateException if periodicity was already answered or configuration froze
         */
        NEXT nonPeriodic();

        /**
         * Declare a fixed equivalence period in caller-facing Plant position units.
         *
         * <p>This metadata does not wrap exact commands; use an explicit equivalent-position target
         * resolver when commands should select equivalent physical positions.</p>
         *
         * @param period positive finite equivalence period in Plant position units
         * @return the position-bounds step for this path
         * @throws IllegalArgumentException if {@code period} is not finite and greater than zero
         * @throws IllegalStateException if periodicity was already answered or configuration froze
         */
        NEXT periodic(double period);
    }

    /** Command-only position range question. */
    public interface CommandedPositionBoundsStep {
        /**
         * Declare a finite closed legal target range in caller-facing Plant position units.
         *
         * @param min inclusive Plant-unit minimum
         * @param max inclusive Plant-unit maximum
         * @return the bounded command-only mapping step
         * @throws IllegalArgumentException if either bound is non-finite or {@code min > max}
         * @throws IllegalStateException if bounds were already answered or configuration froze
         */
        CommandedBoundedPositionMappingStep bounded(double min, double max);

        /**
         * Declare no finite numeric bounds beyond the requirement that every target remain finite.
         *
         * @return the unbounded command-only mapping step
         * @throws IllegalStateException if bounds were already answered or configuration froze
         */
        CommandedUnboundedPositionMappingStep unbounded();
    }

    /** Mapping answers for a bounded command-only position coordinate. */
    public interface CommandedBoundedPositionMappingStep {
        /**
         * Use an identity map: one Plant position unit equals one native output unit.
         *
         * <p>The command-only map uses the zero-to-zero static alignment and proceeds directly to
         * target selection.</p>
         *
         * @return the shared guard and target-selection step
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        TargetStep<PositionPlant> nativeUnits();

        /**
         * Set a zero-independent Plant-to-native scale, then require one static alignment anchor.
         *
         * @param nativeUnitsPerPlantUnit finite non-zero native units per Plant position unit;
         *                                negative values intentionally reverse direction
         * @return the required command-only reference step
         * @throws IllegalArgumentException if the scale is non-finite or zero
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        CommandedPositionReferenceStep scaleToNative(double nativeUnitsPerPlantUnit);

        /**
         * Affinely map the declared Plant-range endpoints to two native output positions.
         *
         * <p>Arguments are native values, not Plant bounds. Reversed native endpoints are allowed;
         * equal endpoints are not. This answer also supplies the static mapping anchor.</p>
         *
         * @param nativeAtPlantMin finite native position at the declared Plant minimum
         * @param nativeAtPlantMax finite native position at the declared Plant maximum
         * @return the shared guard and target-selection step
         * @throws IllegalArgumentException if the Plant span is not positive, either endpoint is
         *                                  non-finite, or the resulting scale is zero/non-finite
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        TargetStep<PositionPlant> rangeMapsToNative(double nativeAtPlantMin,
                                                    double nativeAtPlantMax);
    }

    /** Mapping answers for an unbounded command-only position coordinate. */
    public interface CommandedUnboundedPositionMappingStep {
        /**
         * Use an identity, zero-to-zero map between Plant and native position units.
         *
         * @return the shared guard and target-selection step
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        TargetStep<PositionPlant> nativeUnits();

        /**
         * Set a Plant-to-native scale, then require one static alignment anchor.
         *
         * @param nativeUnitsPerPlantUnit finite non-zero native units per Plant position unit;
         *                                negative values intentionally reverse direction
         * @return the required command-only reference step
         * @throws IllegalArgumentException if the scale is non-finite or zero
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        CommandedPositionReferenceStep scaleToNative(double nativeUnitsPerPlantUnit);
    }

    /** Static alignment required after a scaled command-only position mapping. */
    public interface CommandedPositionReferenceStep {
        /**
         * Anchor one caller-facing Plant position to one native output position.
         *
         * <p>Together with the selected scale, this defines the complete affine command map. The
         * anchor is coordinate data and need not itself be inside the legal Plant target range.</p>
         *
         * @param plantPosition finite position in caller-facing Plant units
         * @param nativePosition finite corresponding native output position
         * @return the shared guard and target-selection step
         * @throws IllegalArgumentException if either value is non-finite
         * @throws IllegalStateException if reference was already answered or configuration froze
         */
        TargetStep<PositionPlant> plantPositionMapsToNative(double plantPosition,
                                                           double nativePosition);
    }

    /** Optional search capability followed by the shared feedback-position periodicity question. */
    public interface DeviceManagedPositionStep
            extends PositionPeriodicityStep<FeedbackPositionBoundsStep> {
        /**
         * Add a distinct normalized-power output for owner-updated calibration searches.
         *
         * <p>Omit this answer when search is unsupported. The output is retained without being
         * called during configuration or build; the calibration and Plant-update lifecycle owns
         * every later call.</p>
         *
         * @param out normalized raw-power search output
         * @return the required position-periodicity step
         * @throws NullPointerException if {@code out} is null
         * @throws IllegalStateException if search output was already answered, this is a regulated
         *                               path, or configuration froze
         */
        PositionPeriodicityStep<FeedbackPositionBoundsStep> searchPowerOutput(PowerOutput out);
    }

    /** Feedback-position range question. */
    public interface FeedbackPositionBoundsStep {
        /**
         * Declare a finite closed legal target range in caller-facing Plant position units.
         *
         * @param min inclusive Plant-unit minimum
         * @param max inclusive Plant-unit maximum
         * @return the bounded feedback-position mapping step
         * @throws IllegalArgumentException if either bound is non-finite or {@code min > max}
         * @throws IllegalStateException if bounds were already answered or configuration froze
         */
        FeedbackBoundedPositionMappingStep bounded(double min, double max);

        /**
         * Declare no finite numeric bounds beyond the requirement that every target remain finite.
         *
         * @return the unbounded feedback-position mapping step
         * @throws IllegalStateException if bounds were already answered or configuration froze
         */
        FeedbackUnboundedPositionMappingStep unbounded();
    }

    /** Mapping answers for a bounded feedback position coordinate. */
    public interface FeedbackBoundedPositionMappingStep {
        /**
         * Use an identity scale between Plant and native position units, then choose reference
         * ownership explicitly.
         *
         * @return the required position-reference step
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        PositionReferenceStep nativeUnits();

        /**
         * Set the Plant-to-native position scale, then choose reference ownership explicitly.
         *
         * @param nativeUnitsPerPlantUnit finite non-zero native units per Plant position unit;
         *                                negative values intentionally reverse direction
         * @return the required position-reference step
         * @throws IllegalArgumentException if the scale is non-finite or zero
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        PositionReferenceStep scaleToNative(double nativeUnitsPerPlantUnit);

        /**
         * Affinely map the declared Plant-range endpoints to two native feedback/output positions.
         *
         * <p>Arguments are native values. Reversed endpoints are allowed; equal endpoints are not.
         * This answer supplies the static reference anchor, so completion tolerance is next.</p>
         *
         * @param nativeAtPlantMin finite native position at the declared Plant minimum
         * @param nativeAtPlantMax finite native position at the declared Plant maximum
         * @return the required Plant-unit position-tolerance step
         * @throws IllegalArgumentException if the Plant span is not positive, either endpoint is
         *                                  non-finite, or the resulting scale is zero/non-finite
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        PositionToleranceStep rangeMapsToNative(double nativeAtPlantMin,
                                                double nativeAtPlantMax);
    }

    /** Mapping answers for an unbounded feedback position coordinate. */
    public interface FeedbackUnboundedPositionMappingStep {
        /**
         * Use an identity scale between Plant and native position units, then choose reference
         * ownership explicitly.
         *
         * @return the required position-reference step
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        PositionReferenceStep nativeUnits();

        /**
         * Set the Plant-to-native position scale, then choose reference ownership explicitly.
         *
         * @param nativeUnitsPerPlantUnit finite non-zero native units per Plant position unit;
         *                                negative values intentionally reverse direction
         * @return the required position-reference step
         * @throws IllegalArgumentException if the scale is non-finite or zero
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        PositionReferenceStep scaleToNative(double nativeUnitsPerPlantUnit);
    }

    /** Physical-reference question for a feedback position coordinate. */
    public interface PositionReferenceStep {
        /**
         * Declare that the selected map is already aligned with Plant zero at native zero.
         *
         * @return the required Plant-unit position-tolerance step
         * @throws IllegalStateException if reference was already answered or configuration froze
         */
        PositionToleranceStep alreadyReferenced();

        /**
         * Supply a static affine-map anchor between Plant and native position coordinates.
         *
         * <p>The anchor is coordinate data and need not lie inside the legal Plant target range.</p>
         *
         * @param plantPosition finite position in caller-facing Plant units
         * @param nativePosition finite corresponding native feedback/output position
         * @return the required Plant-unit position-tolerance step
         * @throws IllegalArgumentException if either value is non-finite
         * @throws IllegalStateException if reference was already answered or configuration froze
         */
        PositionToleranceStep plantPositionMapsToNative(double plantPosition,
                                                        double nativePosition);

        /**
         * Align the first finite native feedback sample with a supplied Plant position.
         *
         * @param plantPosition finite position in caller-facing Plant units
         * @return the required Plant-unit position-tolerance step
         * @throws IllegalArgumentException if {@code plantPosition} is non-finite
         * @throws IllegalStateException if reference was already answered or configuration froze
         */
        PositionToleranceStep assumeCurrentPositionIs(double plantPosition);

        /**
         * Leave the Plant unreferenced until an explicit calibration/reference operation succeeds.
         *
         * @param reason nonblank diagnostic explaining why reference is not yet established
         * @return the required Plant-unit position-tolerance step
         * @throws IllegalArgumentException if {@code reason} is null or blank
         * @throws IllegalStateException if reference was already answered or configuration froze
         */
        PositionToleranceStep needsReference(String reason);
    }

    /** Required feedback-position completion tolerance. */
    public interface PositionToleranceStep {
        /**
         * Set the inclusive completion tolerance in caller-facing Plant position units.
         *
         * @param tolerance finite tolerance greater than or equal to zero
         * @return the shared guard and target-selection step
         * @throws IllegalArgumentException if {@code tolerance} is non-finite or negative
         * @throws IllegalStateException if tolerance was already answered or configuration froze
         */
        TargetStep<PositionPlant> positionTolerance(double tolerance);
    }

    /** Velocity target-range question. */
    public interface VelocityBoundsStep {
        /**
         * Declare a finite closed legal target range in caller-facing Plant velocity units.
         *
         * @param min inclusive Plant-unit velocity minimum
         * @param max inclusive Plant-unit velocity maximum
         * @return the required velocity-mapping step
         * @throws IllegalArgumentException if either bound is non-finite or {@code min > max}
         * @throws IllegalStateException if bounds were already answered or configuration froze
         */
        VelocityMappingStep bounded(double min, double max);

        /**
         * Declare no numeric velocity bounds beyond the requirement that every target remain finite.
         *
         * @return the required velocity-mapping step
         * @throws IllegalStateException if bounds were already answered or configuration froze
         */
        VelocityMappingStep unbounded();
    }

    /** Zero-preserving velocity mapping question. */
    public interface VelocityMappingStep {
        /**
         * Use an identity map between Plant and native velocity units.
         *
         * @return the required Plant-unit velocity-tolerance step
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        VelocityToleranceStep nativeUnits();

        /**
         * Set the zero-preserving Plant-to-native velocity scale.
         *
         * @param nativeUnitsPerPlantVelocityUnit finite non-zero native velocity units per Plant
         *                                        velocity unit; negative values reverse direction
         * @return the required Plant-unit velocity-tolerance step
         * @throws IllegalArgumentException if the scale is non-finite or zero
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        VelocityToleranceStep scaleToNative(double nativeUnitsPerPlantVelocityUnit);
    }

    /** Required velocity completion tolerance. */
    public interface VelocityToleranceStep {
        /**
         * Set the inclusive completion tolerance in caller-facing Plant velocity units.
         *
         * @param tolerance finite tolerance greater than or equal to zero
         * @return the shared guard and target-selection step
         * @throws IllegalArgumentException if {@code tolerance} is non-finite or negative
         * @throws IllegalStateException if tolerance was already answered or configuration froze
         */
        TargetStep<Plant> velocityTolerance(double tolerance);
    }

    /** Shared optional-guards and target-selection step. */
    public interface TargetStep<P extends Plant> {
        /**
         * Enter the optional inline dynamic target-guard branch.
         *
         * <p>Finish it with {@link TargetGuardStep#doneTargetGuards()} before selecting a target.</p>
         *
         * @return the open target-guard step
         * @throws IllegalStateException if the guard branch was already entered or configuration froze
         */
        TargetGuardStep<P> targetGuards();

        /**
         * Create and own a fresh exact writable command initialized in Plant target units.
         *
         * <p>The completed Plant exposes that stable request through {@link Plant#commandTarget()}.
         * Validation occurs before the answer is retained; a rejected value leaves this step usable.
         * Selecting a valid target freezes every earlier configuration answer.</p>
         *
         * @param initialValue finite initial command inside the declared Plant target range
         * @return the single-use build step
         * @throws IllegalArgumentException if the value is non-finite or outside the declared range
         * @throws IllegalStateException if guards remain open, a target was already selected, or
         *                               configuration is incomplete/frozen
         */
        BuildStep<P> targetFromNewCommand(double initialValue);

        /**
         * Bind the one final Plant-aware target resolver.
         *
         * <p>The resolver is first sampled by Plant update, not by configuration or build. A graph
         * created from a stable {@link ScalarTarget} with {@link PlantTargets#exact(ScalarSource)}
         * may expose that same command through {@link Plant#commandTarget()}; read-only or planned
         * graphs intentionally may not. A valid answer freezes every earlier configuration answer.</p>
         *
         * @param finalResolver final source graph that resolves targets in caller-facing Plant units
         * @return the single-use build step
         * @throws NullPointerException if {@code finalResolver} is null
         * @throws IllegalStateException if guards remain open, a target was already selected, or
         *                               configuration is incomplete/frozen
         */
        BuildStep<P> targetFromResolver(PlantTargetResolver finalResolver);
    }

    /** Shared inline dynamic target-guard branch. */
    public interface TargetGuardStep<P extends Plant> {
        /**
         * Limit upward and downward applied-target change to one symmetric Plant-unit rate.
         *
         * @param maxDeltaPerSec positive finite Plant target units per second
         * @return this open guard step
         * @throws IllegalArgumentException if the rate is not finite and greater than zero
         * @throws IllegalStateException if rate limiting was already answered, the branch is
         *                               closed, or configuration froze
         */
        TargetGuardStep<P> maxTargetRate(double maxDeltaPerSec);

        /**
         * Limit increasing and decreasing applied targets with separate Plant-unit rates.
         *
         * @param maxUpPerSec positive finite Plant target units per second when increasing
         * @param maxDownPerSec positive finite Plant target units per second when decreasing
         * @return this open guard step
         * @throws IllegalArgumentException if either rate is not finite and greater than zero
         * @throws IllegalStateException if rate limiting was already answered, the branch is
         *                               closed, or configuration froze
         */
        TargetGuardStep<P> maxTargetRates(double maxUpPerSec, double maxDownPerSec);

        /**
         * Hold the previous applied target whenever a loop-sampled Boolean permission is false.
         *
         * @param name diagnostic interlock name; null/blank uses {@code interlock}
         * @param allowed permission sampled during Plant update
         * @return this open guard step
         * @throws NullPointerException if {@code allowed} is null
         * @throws IllegalStateException if the branch is closed or configuration froze
         */
        TargetGuardStep<P> holdLastTargetUnless(String name, BooleanSource allowed);

        /**
         * Hold the previous applied target whenever a candidate-aware gate rejects it.
         *
         * @param name diagnostic interlock name; null/blank uses {@code interlock}
         * @param gate candidate-aware permission sampled during Plant update
         * @return this open guard step
         * @throws NullPointerException if {@code gate} is null
         * @throws IllegalStateException if the branch is closed or configuration froze
         */
        TargetGuardStep<P> holdLastTargetUnless(String name, PlantTargetGate gate);

        /**
         * Substitute a static Plant-unit target whenever a loop-sampled permission is false.
         *
         * <p>The fallback is checked before retention; rejection leaves this guard step usable.</p>
         *
         * @param name diagnostic interlock name; null/blank uses {@code interlock}
         * @param allowed permission sampled during Plant update
         * @param fallbackTarget finite fallback inside the declared Plant target range
         * @return this open guard step
         * @throws NullPointerException if {@code allowed} is null
         * @throws IllegalArgumentException if the fallback is non-finite or outside the range
         * @throws IllegalStateException if the branch is closed or configuration froze
         */
        TargetGuardStep<P> fallbackTargetUnless(String name,
                                                BooleanSource allowed,
                                                double fallbackTarget);

        /**
         * Substitute a static Plant-unit target whenever a candidate-aware gate rejects it.
         *
         * <p>The fallback is checked before retention; rejection leaves this guard step usable.</p>
         *
         * @param name diagnostic interlock name; null/blank uses {@code interlock}
         * @param gate candidate-aware permission sampled during Plant update
         * @param fallbackTarget finite fallback inside the declared Plant target range
         * @return this open guard step
         * @throws NullPointerException if {@code gate} is null
         * @throws IllegalArgumentException if the fallback is non-finite or outside the range
         * @throws IllegalStateException if the branch is closed or configuration froze
         */
        TargetGuardStep<P> fallbackTargetUnless(String name,
                                                PlantTargetGate gate,
                                                double fallbackTarget);

        /**
         * Close the inline guard branch and continue to the one required target answer.
         *
         * @return the shared target-selection step
         * @throws IllegalStateException if the branch is already closed or configuration froze
         */
        TargetStep<P> doneTargetGuards();
    }

    /** Final single-use build step. */
    public interface BuildStep<P extends Plant> {
        /**
         * Validate the frozen recipe and construct its hidden source-driven Plant runtime.
         *
         * <p>Call this step once; a second attempt requires a fresh {@link #fromOutputs()} recipe.
         * Building does not sample feedback/resolvers or command an output. The owner must call the
         * resulting Plant's lifecycle methods, including {@link Plant#update(LoopClock)}.</p>
         *
         * @return the completed Plant, narrowed to this path's public capability type
         * @throws IllegalStateException if build was already attempted or the recipe is incomplete
         * @throws IllegalArgumentException if a retained numeric configuration is invalid
         */
        P build();
    }

    private static final class OutputStart implements FromOutputsStep {
        private boolean selected;

        @Override
        public TargetStep<Plant> power(PowerOutput out) {
            requireUnused();
            PowerOutput checked = Objects.requireNonNull(out, "power output");
            selected = true;
            return new PowerBuilder(checked);
        }

        @Override
        public PositionPeriodicityStep<CommandedPositionBoundsStep> commandedPosition(
                PositionOutput out) {
            requireUnused();
            PositionOutput checked = Objects.requireNonNull(out, "position output");
            selected = true;
            return new CommandedPositionBuilder(checked);
        }

        @Override
        public DeviceManagedPositionStep deviceManagedPosition(
                PositionOutput out,
                ScalarSource nativeMeasurement) {
            requireUnused();
            PositionOutput checkedOut = Objects.requireNonNull(out, "position output");
            ScalarSource checkedMeasurement = Objects.requireNonNull(
                    nativeMeasurement, "native position measurement");
            selected = true;
            return FeedbackPositionBuilder.deviceManaged(checkedOut, checkedMeasurement);
        }

        @Override
        public VelocityBoundsStep deviceManagedVelocity(
                VelocityOutput out,
                ScalarSource nativeMeasurement) {
            requireUnused();
            VelocityOutput checkedOut = Objects.requireNonNull(out, "velocity output");
            ScalarSource checkedMeasurement = Objects.requireNonNull(
                    nativeMeasurement, "native velocity measurement");
            selected = true;
            return VelocityBuilder.deviceManaged(checkedOut, checkedMeasurement);
        }

        @Override
        public PositionPeriodicityStep<FeedbackPositionBoundsStep> regulatedPosition(
                PowerOutput out,
                ScalarSource nativeMeasurement,
                ScalarRegulator regulator) {
            requireUnused();
            PowerOutput checkedOut = Objects.requireNonNull(out, "regulated power output");
            ScalarSource checkedMeasurement = Objects.requireNonNull(
                    nativeMeasurement, "native position measurement");
            ScalarRegulator checkedRegulator = Objects.requireNonNull(regulator, "position regulator");
            selected = true;
            return FeedbackPositionBuilder.regulated(
                    checkedOut, checkedMeasurement, checkedRegulator);
        }

        @Override
        public VelocityBoundsStep regulatedVelocity(
                PowerOutput out,
                ScalarSource nativeMeasurement,
                ScalarRegulator regulator) {
            requireUnused();
            PowerOutput checkedOut = Objects.requireNonNull(out, "regulated power output");
            ScalarSource checkedMeasurement = Objects.requireNonNull(
                    nativeMeasurement, "native velocity measurement");
            ScalarRegulator checkedRegulator = Objects.requireNonNull(regulator, "velocity regulator");
            selected = true;
            return VelocityBuilder.regulated(checkedOut, checkedMeasurement, checkedRegulator);
        }

        private void requireUnused() {
            if (selected) {
                throw new IllegalStateException("This Plants.fromOutputs() root already selected a "
                        + "control path; call Plants.fromOutputs() again to build another Plant");
            }
        }
    }

    private abstract static class TargetBuilder<P extends Plant>
            implements TargetStep<P>, TargetGuardStep<P>, BuildStep<P> {
        private final PlantTargetGuards.Builder guardBuilder = PlantTargetGuards.builder();
        private PlantTargetResolver targetResolver;
        private double newCommandInitialValue = Double.NaN;
        private boolean builderCreatedCommand;
        private boolean guardBranchEntered;
        private boolean guardBranchOpen;
        private boolean targetRateConfigured;
        private boolean targetAnswered;
        private boolean buildAttempted;

        @Override
        public final TargetGuardStep<P> targetGuards() {
            requireConfigurationMutable("targetGuards()");
            validateConfiguration();
            if (guardBranchEntered) {
                throw new IllegalStateException("targetGuards() has already been entered for this Plant recipe");
            }
            guardBranchEntered = true;
            guardBranchOpen = true;
            return this;
        }

        @Override
        public final TargetGuardStep<P> maxTargetRate(double maxDeltaPerSec) {
            requireGuardBranch("maxTargetRate(...)");
            requireTargetRateUnanswered();
            guardBuilder.maxTargetRate(maxDeltaPerSec);
            targetRateConfigured = true;
            return this;
        }

        @Override
        public final TargetGuardStep<P> maxTargetRates(double maxUpPerSec, double maxDownPerSec) {
            requireGuardBranch("maxTargetRates(...)");
            requireTargetRateUnanswered();
            guardBuilder.maxTargetRates(maxUpPerSec, maxDownPerSec);
            targetRateConfigured = true;
            return this;
        }

        @Override
        public final TargetGuardStep<P> holdLastTargetUnless(String name, BooleanSource allowed) {
            requireGuardBranch("holdLastTargetUnless(...)");
            guardBuilder.holdLastTargetUnless(name, Objects.requireNonNull(allowed, "allowed"));
            return this;
        }

        @Override
        public final TargetGuardStep<P> holdLastTargetUnless(String name, PlantTargetGate gate) {
            requireGuardBranch("holdLastTargetUnless(...)");
            guardBuilder.holdLastTargetUnless(name, Objects.requireNonNull(gate, "gate"));
            return this;
        }

        @Override
        public final TargetGuardStep<P> fallbackTargetUnless(
                String name,
                BooleanSource allowed,
                double fallbackTarget) {
            requireGuardBranch("fallbackTargetUnless(...)");
            requireStaticTargetInRange(
                    fallbackTarget,
                    "target guard '" + guardDisplayName(name) + "' fallback target");
            guardBuilder.fallbackTargetUnless(
                    name, Objects.requireNonNull(allowed, "allowed"), fallbackTarget);
            return this;
        }

        @Override
        public final TargetGuardStep<P> fallbackTargetUnless(
                String name,
                PlantTargetGate gate,
                double fallbackTarget) {
            requireGuardBranch("fallbackTargetUnless(...)");
            requireStaticTargetInRange(
                    fallbackTarget,
                    "target guard '" + guardDisplayName(name) + "' fallback target");
            guardBuilder.fallbackTargetUnless(
                    name, Objects.requireNonNull(gate, "gate"), fallbackTarget);
            return this;
        }

        @Override
        public final TargetStep<P> doneTargetGuards() {
            requireGuardBranch("doneTargetGuards()");
            guardBranchOpen = false;
            return this;
        }

        @Override
        public final BuildStep<P> targetFromNewCommand(double initialValue) {
            requireTargetUnanswered("targetFromNewCommand(...)");
            validateConfiguration();
            requireGuardsClosed();
            requireStaticTargetInRange(initialValue, "initial command");
            ScalarTarget command = ScalarTarget.create(initialValue);
            targetResolver = PlantTargets.exact(command);
            newCommandInitialValue = initialValue;
            builderCreatedCommand = true;
            targetAnswered = true;
            return this;
        }

        @Override
        public final BuildStep<P> targetFromResolver(PlantTargetResolver finalResolver) {
            PlantTargetResolver checked = Objects.requireNonNull(finalResolver, "final target resolver");
            requireTargetUnanswered("targetFromResolver(...)");
            validateConfiguration();
            requireGuardsClosed();
            targetResolver = checked;
            builderCreatedCommand = false;
            targetAnswered = true;
            return this;
        }

        @Override
        public final P build() {
            if (!targetAnswered) {
                throw new IllegalStateException("Plant construction requires targetFromNewCommand(...) "
                        + "or targetFromResolver(...)");
            }
            if (buildAttempted) {
                throw new IllegalStateException("This Plant recipe has already attempted build(); "
                        + "create a fresh builder for another Plant");
            }
            validateConfiguration();
            requireGuardsClosed();
            if (builderCreatedCommand) {
                requireStaticTargetInRange(newCommandInitialValue, "initial command");
            }
            PlantTargetGuards guards = guardBuilder.build();
            guards.validateFallbackTargets(configuredTargetRange(), "Plant");
            buildAttempted = true;
            return buildPlant(targetResolver, guards);
        }

        protected final void requireConfigurationMutable(String answer) {
            if (targetAnswered || buildAttempted) {
                throw new IllegalStateException(answer + " cannot change a Plant recipe after its "
                        + "target has been selected");
            }
        }

        protected abstract ScalarRange configuredTargetRange();

        protected abstract void validateConfiguration();

        protected abstract P buildPlant(PlantTargetResolver resolver, PlantTargetGuards guards);

        private void requireTargetUnanswered(String answer) {
            requireConfigurationMutable(answer);
            if (targetAnswered) {
                throw new IllegalStateException("This Plant recipe already selected its target");
            }
        }

        private void requireGuardBranch(String answer) {
            requireConfigurationMutable(answer);
            if (!guardBranchOpen) {
                throw new IllegalStateException(answer + " requires targetGuards() and must occur "
                        + "before doneTargetGuards()");
            }
        }

        private void requireGuardsClosed() {
            if (guardBranchOpen) {
                throw new IllegalStateException("Call doneTargetGuards() before selecting the Plant target");
            }
        }

        private void requireTargetRateUnanswered() {
            if (targetRateConfigured) {
                throw new IllegalStateException("Target rate limiting has already been configured; "
                        + "choose maxTargetRate(...) or maxTargetRates(...) once");
            }
        }

        private void requireStaticTargetInRange(double value, String answerName) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException(answerName + " must be finite, got " + value);
            }
            ScalarRange range = Objects.requireNonNull(configuredTargetRange(), "configured target range");
            if (!range.contains(value)) {
                throw new IllegalArgumentException(answerName + " " + value
                        + " is outside the declared Plant range [" + range.minValue
                        + ", " + range.maxValue + "]");
            }
        }
    }

    private static final class PowerBuilder extends TargetBuilder<Plant> {
        private final PowerOutput out;

        private PowerBuilder(PowerOutput out) {
            this.out = out;
        }

        @Override
        protected ScalarRange configuredTargetRange() {
            return NORMALIZED_POWER_RANGE;
        }

        @Override
        protected void validateConfiguration() {
            // Power has one fixed normalized range and no additional coordinate questions.
        }

        @Override
        protected Plant buildPlant(PlantTargetResolver resolver, PlantTargetGuards guards) {
            return new PowerPlant(out, resolver, guards);
        }
    }

    private static final class VelocityBuilder extends TargetBuilder<Plant>
            implements VelocityBoundsStep, VelocityMappingStep, VelocityToleranceStep {
        private final VelocityOutput velocityOut;
        private final PowerOutput regulatedPowerOut;
        private final ScalarSource nativeMeasurement;
        private final ScalarRegulator regulator;
        private ScalarRange range;
        private double nativePerPlantUnit;
        private double tolerance;
        private boolean rangeAnswered;
        private boolean mappingAnswered;
        private boolean toleranceAnswered;

        private VelocityBuilder(VelocityOutput velocityOut,
                                PowerOutput regulatedPowerOut,
                                ScalarSource nativeMeasurement,
                                ScalarRegulator regulator) {
            this.velocityOut = velocityOut;
            this.regulatedPowerOut = regulatedPowerOut;
            this.nativeMeasurement = nativeMeasurement;
            this.regulator = regulator;
        }

        static VelocityBuilder deviceManaged(VelocityOutput out, ScalarSource nativeMeasurement) {
            return new VelocityBuilder(out, null, nativeMeasurement, null);
        }

        static VelocityBuilder regulated(PowerOutput out,
                                         ScalarSource nativeMeasurement,
                                         ScalarRegulator regulator) {
            return new VelocityBuilder(null, out, nativeMeasurement, regulator);
        }

        @Override
        public VelocityMappingStep bounded(double min, double max) {
            requireConfigurationMutable("bounded(...)");
            requireRangeUnanswered();
            range = ScalarRange.bounded(min, max);
            rangeAnswered = true;
            return this;
        }

        @Override
        public VelocityMappingStep unbounded() {
            requireConfigurationMutable("unbounded()");
            requireRangeUnanswered();
            range = ScalarRange.unbounded();
            rangeAnswered = true;
            return this;
        }

        @Override
        public VelocityToleranceStep nativeUnits() {
            return answerMapping(1.0, "nativeUnits()");
        }

        @Override
        public VelocityToleranceStep scaleToNative(double nativeUnitsPerPlantVelocityUnit) {
            return answerMapping(nativeUnitsPerPlantVelocityUnit, "scaleToNative(...)");
        }

        @Override
        public TargetStep<Plant> velocityTolerance(double tolerance) {
            requireConfigurationMutable("velocityTolerance(...)");
            if (!mappingAnswered) {
                throw new IllegalStateException("Choose nativeUnits() or scaleToNative(...) before "
                        + "velocityTolerance(...)");
            }
            if (toleranceAnswered) {
                throw new IllegalStateException("velocityTolerance(...) has already been answered");
            }
            requireTolerance(tolerance, "velocityTolerance");
            this.tolerance = tolerance;
            toleranceAnswered = true;
            return this;
        }

        @Override
        protected ScalarRange configuredTargetRange() {
            return range;
        }

        @Override
        protected void validateConfiguration() {
            if (!rangeAnswered) {
                throw new IllegalStateException("Velocity construction requires bounded(...) or unbounded()");
            }
            if (!mappingAnswered) {
                throw new IllegalStateException("Velocity construction requires nativeUnits() or scaleToNative(...)");
            }
            if (!toleranceAnswered) {
                throw new IllegalStateException("Velocity construction requires velocityTolerance(...) in Plant units");
            }
        }

        @Override
        protected Plant buildPlant(PlantTargetResolver resolver, PlantTargetGuards guards) {
            MappedVelocityPlant.FeedbackConfigurationStep configured = velocityOut != null
                    ? MappedVelocityPlant.velocityOutput(velocityOut, nativeMeasurement)
                    : MappedVelocityPlant.regulated(regulatedPowerOut, nativeMeasurement, regulator);
            return configured.range(range)
                    .nativePerPlantUnit(nativePerPlantUnit)
                    .velocityTolerance(tolerance)
                    .targetGuards(guards)
                    .targetFromResolver(resolver)
                    .build();
        }

        private VelocityToleranceStep answerMapping(double scale, String answer) {
            requireConfigurationMutable(answer);
            if (!rangeAnswered) {
                throw new IllegalStateException("Choose bounded(...) or unbounded() before " + answer);
            }
            if (mappingAnswered) {
                throw new IllegalStateException("Velocity mapping has already been answered");
            }
            requireScale(scale);
            nativePerPlantUnit = scale;
            mappingAnswered = true;
            return this;
        }

        private void requireRangeUnanswered() {
            if (rangeAnswered) {
                throw new IllegalStateException("Velocity bounds have already been answered");
            }
        }
    }

    private abstract static class PositionBuilderBase<P extends Plant> extends TargetBuilder<P> {
        PositionPlant.Periodicity periodicity;
        double period = Double.NaN;
        ScalarRange range;
        double nativePerPlantUnit;
        double plantReference;
        double nativeReference;
        boolean periodicityAnswered;
        boolean rangeAnswered;
        boolean mappingAnswered;
        boolean referenceAnswered;
        boolean bounded;

        final void answerNonPeriodic() {
            answerPeriodicity(PositionPlant.Periodicity.NON_PERIODIC, Double.NaN);
        }

        final void answerPeriodic(double period) {
            if (!(period > 0.0) || !Double.isFinite(period)) {
                throw new IllegalArgumentException("period must be finite and > 0, got " + period);
            }
            answerPeriodicity(PositionPlant.Periodicity.PERIODIC, period);
        }

        final void answerBounded(double min, double max) {
            requireConfigurationMutable("bounded(...)");
            requirePeriodicityAnswered();
            requireRangeUnanswered();
            range = ScalarRange.bounded(min, max);
            bounded = true;
            rangeAnswered = true;
        }

        final void answerUnbounded() {
            requireConfigurationMutable("unbounded()");
            requirePeriodicityAnswered();
            requireRangeUnanswered();
            range = ScalarRange.unbounded();
            bounded = false;
            rangeAnswered = true;
        }

        final void answerScale(double scale, String answer) {
            requireConfigurationMutable(answer);
            requireRangeAnswered();
            requireMappingUnanswered();
            requireScale(scale);
            nativePerPlantUnit = scale;
            mappingAnswered = true;
        }

        final void answerEndpointMap(double nativeAtPlantMin, double nativeAtPlantMax) {
            requireConfigurationMutable("rangeMapsToNative(...)");
            requireRangeAnswered();
            if (!bounded) {
                throw new IllegalStateException("rangeMapsToNative(...) requires bounded(...) Plant units");
            }
            requireMappingUnanswered();
            double checkedNativeAtPlantMin =
                    PositionCalibrationValueValidation.requireFiniteNativeValue(
                            nativeAtPlantMin,
                            "Plants.rangeMapsToNative(...)",
                            "nativeAtPlantMin");
            double checkedNativeAtPlantMax =
                    PositionCalibrationValueValidation.requireFiniteNativeValue(
                            nativeAtPlantMax,
                            "Plants.rangeMapsToNative(...)",
                            "nativeAtPlantMax");
            double plantSpan = range.maxValue - range.minValue;
            if (!(plantSpan > 0.0) || !Double.isFinite(plantSpan)) {
                throw new IllegalArgumentException("rangeMapsToNative(...) requires finite Plant bounds "
                        + "with min < max");
            }
            double scale = (checkedNativeAtPlantMax - checkedNativeAtPlantMin) / plantSpan;
            requireScale(scale);
            nativePerPlantUnit = scale;
            plantReference = range.minValue;
            nativeReference = checkedNativeAtPlantMin;
            mappingAnswered = true;
            referenceAnswered = true;
        }

        final void answerStaticReference(double plantPosition, double nativePosition, String answer) {
            requireConfigurationMutable(answer);
            if (!mappingAnswered) {
                throw new IllegalStateException("Choose nativeUnits() or scaleToNative(...) before " + answer);
            }
            double checkedPlantPosition =
                    PositionCalibrationValueValidation.requireFinitePlantValue(
                            plantPosition, answer, "plantPosition");
            double checkedNativePosition =
                    PositionCalibrationValueValidation.requireFiniteNativeValue(
                            nativePosition, answer, "nativePosition");
            if (referenceAnswered) {
                throw new IllegalStateException("Position reference has already been answered");
            }
            plantReference = checkedPlantPosition;
            nativeReference = checkedNativePosition;
            referenceAnswered = true;
        }

        @Override
        protected final ScalarRange configuredTargetRange() {
            return range;
        }

        final void requireCoordinateConfigured() {
            if (!periodicityAnswered) {
                throw new IllegalStateException("Position construction requires nonPeriodic() or periodic(period)");
            }
            if (!rangeAnswered) {
                throw new IllegalStateException("Position construction requires bounded(...) or unbounded()");
            }
            if (!mappingAnswered) {
                throw new IllegalStateException("Position construction requires a Plant-to-native mapping answer");
            }
            if (!referenceAnswered) {
                throw new IllegalStateException("Position construction requires a reference/alignment answer");
            }
        }

        private void answerPeriodicity(PositionPlant.Periodicity answer, double answerPeriod) {
            requireConfigurationMutable(answer == PositionPlant.Periodicity.PERIODIC
                    ? "periodic(...)" : "nonPeriodic()");
            if (periodicityAnswered) {
                throw new IllegalStateException("Position periodicity has already been answered");
            }
            periodicity = answer;
            period = answerPeriod;
            periodicityAnswered = true;
        }

        private void requirePeriodicityAnswered() {
            if (!periodicityAnswered) {
                throw new IllegalStateException("Choose nonPeriodic() or periodic(period) before bounds");
            }
        }

        private void requireRangeAnswered() {
            if (!rangeAnswered) {
                throw new IllegalStateException("Choose bounded(...) or unbounded() before mapping");
            }
        }

        private void requireRangeUnanswered() {
            if (rangeAnswered) {
                throw new IllegalStateException("Position bounds have already been answered");
            }
        }

        private void requireMappingUnanswered() {
            if (mappingAnswered) {
                throw new IllegalStateException("Position mapping has already been answered");
            }
        }
    }

    private static final class CommandedPositionBuilder extends PositionBuilderBase<PositionPlant>
            implements PositionPeriodicityStep<CommandedPositionBoundsStep>,
            CommandedPositionBoundsStep,
            CommandedBoundedPositionMappingStep,
            CommandedUnboundedPositionMappingStep,
            CommandedPositionReferenceStep {
        private final PositionOutput out;

        private CommandedPositionBuilder(PositionOutput out) {
            this.out = out;
        }

        @Override
        public CommandedPositionBoundsStep nonPeriodic() {
            answerNonPeriodic();
            return this;
        }

        @Override
        public CommandedPositionBoundsStep periodic(double period) {
            answerPeriodic(period);
            return this;
        }

        @Override
        public CommandedBoundedPositionMappingStep bounded(double min, double max) {
            answerBounded(min, max);
            return this;
        }

        @Override
        public CommandedUnboundedPositionMappingStep unbounded() {
            answerUnbounded();
            return this;
        }

        @Override
        public TargetStep<PositionPlant> nativeUnits() {
            answerScale(1.0, "nativeUnits()");
            plantReference = 0.0;
            nativeReference = 0.0;
            referenceAnswered = true;
            return this;
        }

        @Override
        public CommandedPositionReferenceStep scaleToNative(double nativeUnitsPerPlantUnit) {
            answerScale(nativeUnitsPerPlantUnit, "scaleToNative(...)");
            return this;
        }

        @Override
        public TargetStep<PositionPlant> rangeMapsToNative(
                double nativeAtPlantMin,
                double nativeAtPlantMax) {
            answerEndpointMap(nativeAtPlantMin, nativeAtPlantMax);
            return this;
        }

        @Override
        public TargetStep<PositionPlant> plantPositionMapsToNative(
                double plantPosition,
                double nativePosition) {
            answerStaticReference(
                    plantPosition,
                    nativePosition,
                    "Plants.plantPositionMapsToNative(...)");
            return this;
        }

        @Override
        protected void validateConfiguration() {
            requireCoordinateConfigured();
        }

        @Override
        protected PositionPlant buildPlant(
                PlantTargetResolver resolver,
                PlantTargetGuards guards) {
            return MappedPositionPlant.commanded(out)
                    .periodicity(periodicity, period)
                    .range(range)
                    .nativePerPlantUnit(nativePerPlantUnit)
                    .plantPositionMapsToNative(plantReference, nativeReference)
                    .targetGuards(guards)
                    .targetFromResolver(resolver)
                    .build();
        }
    }

    private static final class FeedbackPositionBuilder extends PositionBuilderBase<PositionPlant>
            implements DeviceManagedPositionStep,
            PositionPeriodicityStep<FeedbackPositionBoundsStep>,
            FeedbackPositionBoundsStep,
            FeedbackBoundedPositionMappingStep,
            FeedbackUnboundedPositionMappingStep,
            PositionReferenceStep,
            PositionToleranceStep {
        private final PositionOutput positionOut;
        private final PowerOutput regulatedPowerOut;
        private final ScalarSource nativeMeasurement;
        private final ScalarRegulator regulator;
        private PowerOutput searchPowerOut;
        private MappedPositionPlant.ReferenceMode referenceMode;
        private double assumedPlantPosition;
        private String referenceReason;
        private double tolerance;
        private boolean searchAnswered;
        private boolean toleranceAnswered;

        private FeedbackPositionBuilder(PositionOutput positionOut,
                                        PowerOutput regulatedPowerOut,
                                        ScalarSource nativeMeasurement,
                                        ScalarRegulator regulator) {
            this.positionOut = positionOut;
            this.regulatedPowerOut = regulatedPowerOut;
            this.nativeMeasurement = nativeMeasurement;
            this.regulator = regulator;
            if (regulatedPowerOut != null) {
                searchPowerOut = regulatedPowerOut;
                searchAnswered = true;
            }
        }

        static FeedbackPositionBuilder deviceManaged(
                PositionOutput out,
                ScalarSource nativeMeasurement) {
            return new FeedbackPositionBuilder(out, null, nativeMeasurement, null);
        }

        static FeedbackPositionBuilder regulated(
                PowerOutput out,
                ScalarSource nativeMeasurement,
                ScalarRegulator regulator) {
            return new FeedbackPositionBuilder(null, out, nativeMeasurement, regulator);
        }

        @Override
        public PositionPeriodicityStep<FeedbackPositionBoundsStep> searchPowerOutput(PowerOutput out) {
            requireConfigurationMutable("searchPowerOutput(...)");
            if (regulatedPowerOut != null) {
                throw new IllegalStateException("regulatedPosition(...) already uses its PowerOutput "
                        + "for calibration search");
            }
            if (periodicityAnswered) {
                throw new IllegalStateException("searchPowerOutput(...) must be answered before periodicity");
            }
            if (searchAnswered) {
                throw new IllegalStateException("searchPowerOutput(...) has already been answered");
            }
            searchPowerOut = Objects.requireNonNull(out, "search power output");
            searchAnswered = true;
            return this;
        }

        @Override
        public FeedbackPositionBoundsStep nonPeriodic() {
            answerNonPeriodic();
            return this;
        }

        @Override
        public FeedbackPositionBoundsStep periodic(double period) {
            answerPeriodic(period);
            return this;
        }

        @Override
        public FeedbackBoundedPositionMappingStep bounded(double min, double max) {
            answerBounded(min, max);
            return this;
        }

        @Override
        public FeedbackUnboundedPositionMappingStep unbounded() {
            answerUnbounded();
            return this;
        }

        @Override
        public PositionReferenceStep nativeUnits() {
            answerScale(1.0, "nativeUnits()");
            return this;
        }

        @Override
        public PositionReferenceStep scaleToNative(double nativeUnitsPerPlantUnit) {
            answerScale(nativeUnitsPerPlantUnit, "scaleToNative(...)");
            return this;
        }

        @Override
        public PositionToleranceStep rangeMapsToNative(
                double nativeAtPlantMin,
                double nativeAtPlantMax) {
            answerEndpointMap(nativeAtPlantMin, nativeAtPlantMax);
            referenceMode = MappedPositionPlant.ReferenceMode.STATIC;
            return this;
        }

        @Override
        public PositionToleranceStep alreadyReferenced() {
            return answerFeedbackStaticReference(0.0, 0.0, "Plants.alreadyReferenced()");
        }

        @Override
        public PositionToleranceStep plantPositionMapsToNative(
                double plantPosition,
                double nativePosition) {
            return answerFeedbackStaticReference(
                    plantPosition,
                    nativePosition,
                    "Plants.plantPositionMapsToNative(...)");
        }

        @Override
        public PositionToleranceStep assumeCurrentPositionIs(double plantPosition) {
            requireConfigurationMutable("assumeCurrentPositionIs(...)");
            if (!mappingAnswered) {
                throw new IllegalStateException("Choose nativeUnits() or scaleToNative(...) before "
                        + "assumeCurrentPositionIs(...)");
            }
            double checkedPlantPosition =
                    PositionCalibrationValueValidation.requireFinitePlantValue(
                            plantPosition,
                            "Plants.assumeCurrentPositionIs(...)",
                            "plantPosition");
            if (referenceAnswered) {
                throw new IllegalStateException("Position reference has already been answered");
            }
            referenceMode = MappedPositionPlant.ReferenceMode.ASSUME_CURRENT;
            assumedPlantPosition = checkedPlantPosition;
            referenceAnswered = true;
            return this;
        }

        @Override
        public PositionToleranceStep needsReference(String reason) {
            requireConfigurationMutable("needsReference(...)");
            requireReferencePending("needsReference(...)");
            if (reason == null || reason.trim().isEmpty()) {
                throw new IllegalArgumentException("needsReference(...) reason must be nonblank");
            }
            referenceMode = MappedPositionPlant.ReferenceMode.NEEDS_REFERENCE;
            referenceReason = reason.trim();
            referenceAnswered = true;
            return this;
        }

        @Override
        public TargetStep<PositionPlant> positionTolerance(double tolerance) {
            requireConfigurationMutable("positionTolerance(...)");
            if (!referenceAnswered) {
                throw new IllegalStateException("Answer position reference before positionTolerance(...)");
            }
            if (toleranceAnswered) {
                throw new IllegalStateException("positionTolerance(...) has already been answered");
            }
            requireTolerance(tolerance, "positionTolerance");
            this.tolerance = tolerance;
            toleranceAnswered = true;
            return this;
        }

        @Override
        protected void validateConfiguration() {
            requireCoordinateConfigured();
            if (!toleranceAnswered) {
                throw new IllegalStateException("Feedback position construction requires "
                        + "positionTolerance(...) in Plant units");
            }
        }

        @Override
        protected PositionPlant buildPlant(
                PlantTargetResolver resolver,
                PlantTargetGuards guards) {
            MappedPositionPlant.FeedbackConfigurationStep configured = positionOut != null
                    ? MappedPositionPlant.positionOutput(positionOut, nativeMeasurement)
                    : MappedPositionPlant.regulated(regulatedPowerOut, nativeMeasurement, regulator);
            if (searchPowerOut != null) {
                configured = configured.searchPowerOutput(searchPowerOut);
            }
            configured = configured.periodicity(periodicity, period)
                    .range(range)
                    .nativePerPlantUnit(nativePerPlantUnit);
            if (referenceMode == MappedPositionPlant.ReferenceMode.STATIC) {
                configured = configured.plantPositionMapsToNative(plantReference, nativeReference);
            } else if (referenceMode == MappedPositionPlant.ReferenceMode.ASSUME_CURRENT) {
                configured = configured.assumeCurrentPositionIs(assumedPlantPosition);
            } else {
                configured = configured.needsReference(referenceReason);
            }
            return configured.positionTolerance(tolerance)
                    .targetGuards(guards)
                    .targetFromResolver(resolver)
                    .build();
        }

        private PositionToleranceStep answerFeedbackStaticReference(
                double plantPosition,
                double nativePosition,
                String answer) {
            super.answerStaticReference(plantPosition, nativePosition, answer);
            referenceMode = MappedPositionPlant.ReferenceMode.STATIC;
            return this;
        }

        private void requireReferencePending(String answer) {
            if (!mappingAnswered) {
                throw new IllegalStateException("Choose nativeUnits() or scaleToNative(...) before " + answer);
            }
            if (referenceAnswered) {
                throw new IllegalStateException("Position reference has already been answered");
            }
        }
    }

    private static void requireScale(double scale) {
        if (!Double.isFinite(scale) || Math.abs(scale) < 1.0e-12) {
            throw new IllegalArgumentException("native units per Plant unit must be finite and non-zero, got "
                    + scale);
        }
    }

    private static void requireTolerance(double tolerance, String name) {
        if (!Double.isFinite(tolerance) || tolerance < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and >= 0, got " + tolerance);
        }
    }

    private static String guardDisplayName(String name) {
        return name == null || name.trim().isEmpty() ? "interlock" : name.trim();
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

}
