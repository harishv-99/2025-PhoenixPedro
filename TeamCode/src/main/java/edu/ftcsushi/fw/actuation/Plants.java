package edu.ftcsushi.fw.actuation;

import java.util.Objects;

import edu.ftcsushi.fw.core.control.ScalarRegulator;
import edu.ftcsushi.fw.core.control.ScalarRegulators;
import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.hal.PowerLimitedPositionOutput;
import edu.ftcsushi.fw.core.hal.PositionOutput;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.hal.VelocityOutput;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Hardware-neutral staged construction for source-driven {@link Plant Plants}.
 *
 * <p>Most FTC robot code should use {@code FtcActuators.plant(...)}. Custom adapters, portable
 * hosts, and hardware-neutral tests that already own Sushi output ports use
 * {@link #fromOutputs()}. The staged grammar asks each required coordinate, feedback, range,
 * mapping, tolerance, guard, and target question once, then hides the concrete runtime. A fully
 * bounded affine map must produce finite native values at both Plant endpoints using the runtime
 * operation order. An unbounded map remains supported; each realized native command is checked
 * before the Plant publishes it as applied or calls the output.</p>
 *
 * <p>A mapped feedback conversion that does not finish finite is exposed as unavailable
 * {@link Double#NaN}, so it cannot satisfy Plant completion. If an otherwise valid unbounded or
 * dynamically referenced map overflows at runtime, the Plant throws an actionable
 * {@link IllegalStateException} and makes a best-effort call to the output's natural stop.</p>
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
         * tolerance. The supplied output owns its natural stop behavior, including safe behavior
         * when stopped before its first position command.</p>
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
         * <p>The device owns position actuation and its natural stop behavior, including safe
         * behavior when stopped before its first position command. The Plant uses
         * {@code nativeMeasurement} for reference and completion, and may optionally receive a
         * distinct raw-power output for calibration search before periodicity is answered.</p>
         *
         * @param out native position output
         * @param nativeMeasurement position feedback in the output adapter's native units
         * @return the optional search-output and required position-periodicity step
         * @throws NullPointerException if either argument is null
         * @throws IllegalStateException if this root step already selected an output path
         */
        DeviceManagedPositionStep<TargetStep<PositionPlant>> deviceManagedPosition(
                PositionOutput out,
                ScalarSource nativeMeasurement);

        /**
         * Select device-managed position output whose command port accepts one paired logical
         * native-position and normalized maximum-output-power command.
         *
         * <p>This overload has the same coordinate grammar as
         * {@link #deviceManagedPosition(PositionOutput, ScalarSource)}, then exposes the optional
         * {@link SymmetricOutputPowerPolicyStep#outputPowerLimitedTo(double)} answer after
         * position tolerance. Omission means magnitude {@code 1.0}. The richer capability is not
         * inferred from an arbitrary {@link PositionOutput}; callers must retain its truthful
         * compile-time type.</p>
         *
         * @param out paired native-position plus normalized-power-magnitude output
         * @param nativeMeasurement position feedback in the output adapter's native units
         * @return the optional search-output and required position-periodicity step
         * @throws NullPointerException if either argument is null
         * @throws IllegalStateException if this root step already selected an output path
         */
        DeviceManagedPositionStep<SymmetricOutputPowerPolicyStep<PositionPlant>>
        deviceManagedPosition(PowerLimitedPositionOutput out,
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
        VelocityBoundsStep<TargetStep<Plant>> deviceManagedVelocity(
                VelocityOutput out,
                ScalarSource nativeMeasurement);

        /**
         * Select framework-regulated position over normalized power and native-position feedback.
         *
         * <p>The same {@code out} channel is used for normal regulation and calibration search,
         * but the runtime command values are independent: normal control submits the regulator's
         * result, while an active search submits its explicitly requested search power. The modes
         * are mutually exclusive, so no second search-output answer is required or supported.</p>
         *
         * @param out normalized power output
         * @param nativeMeasurement position feedback in native units
         * @return the required position-periodicity step
         * @throws NullPointerException if any argument is null
         * @throws IllegalStateException if this root step already selected an output path
         */
        PositionPeriodicityStep<FeedbackPositionBoundsStep<PositionControlStep>> regulatedPosition(
                PowerOutput out,
                ScalarSource nativeMeasurement);

        /**
         * Select framework-regulated velocity over normalized power and native-velocity feedback.
         *
         * @param out normalized power output
         * @param nativeMeasurement velocity feedback in native units per second
         * @return the required Plant-unit velocity-bounds step
         * @throws NullPointerException if any argument is null
         * @throws IllegalStateException if this root step already selected an output path
         */
        VelocityBoundsStep<VelocityControlStep> regulatedVelocity(
                PowerOutput out,
                ScalarSource nativeMeasurement);
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
         *                                  non-finite, the resulting scale is zero/non-finite, or
         *                                  either exact runtime endpoint image is non-finite
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
         * @throws IllegalArgumentException if either value is non-finite, or this anchor makes an
         *                                  exact bounded-range endpoint image non-finite
         * @throws IllegalStateException if reference was already answered or configuration froze
         */
        TargetStep<PositionPlant> plantPositionMapsToNative(double plantPosition,
                                                           double nativePosition);
    }

    /** Optional search capability followed by the shared feedback-position periodicity question. */
    public interface DeviceManagedPositionStep<NEXT>
            extends PositionPeriodicityStep<FeedbackPositionBoundsStep<NEXT>> {
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
        PositionPeriodicityStep<FeedbackPositionBoundsStep<NEXT>> searchPowerOutput(PowerOutput out);
    }

    /** Feedback-position range question. */
    public interface FeedbackPositionBoundsStep<NEXT> {
        /**
         * Declare a finite closed legal target range in caller-facing Plant position units.
         *
         * @param min inclusive Plant-unit minimum
         * @param max inclusive Plant-unit maximum
         * @return the bounded feedback-position mapping step
         * @throws IllegalArgumentException if either bound is non-finite or {@code min > max}
         * @throws IllegalStateException if bounds were already answered or configuration froze
         */
        FeedbackBoundedPositionMappingStep<NEXT> bounded(double min, double max);

        /**
         * Declare no finite numeric bounds beyond the requirement that every target remain finite.
         *
         * @return the unbounded feedback-position mapping step
         * @throws IllegalStateException if bounds were already answered or configuration froze
         */
        FeedbackUnboundedPositionMappingStep<NEXT> unbounded();
    }

    /** Mapping answers for a bounded feedback position coordinate. */
    public interface FeedbackBoundedPositionMappingStep<NEXT> {
        /**
         * Use an identity scale between Plant and native position units, then choose reference
         * ownership explicitly.
         *
         * @return the required position-reference step
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        PositionCoordinateReferenceStep<NEXT> nativeUnits();

        /**
         * Set the Plant-to-native position scale, then choose reference ownership explicitly.
         *
         * @param nativeUnitsPerPlantUnit finite non-zero native units per Plant position unit;
         *                                negative values intentionally reverse direction
         * @return the required position-reference step
         * @throws IllegalArgumentException if the scale is non-finite or zero
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        PositionCoordinateReferenceStep<NEXT> scaleToNative(double nativeUnitsPerPlantUnit);

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
         *                                  non-finite, the resulting scale is zero/non-finite, or
         *                                  either exact runtime endpoint image is non-finite
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        PositionToleranceStep<NEXT> rangeMapsToNative(double nativeAtPlantMin,
                                                      double nativeAtPlantMax);
    }

    /** Mapping answers for an unbounded feedback position coordinate. */
    public interface FeedbackUnboundedPositionMappingStep<NEXT> {
        /**
         * Use an identity scale between Plant and native position units, then choose reference
         * ownership explicitly.
         *
         * @return the required position-reference step
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        PositionCoordinateReferenceStep<NEXT> nativeUnits();

        /**
         * Set the Plant-to-native position scale, then choose reference ownership explicitly.
         *
         * @param nativeUnitsPerPlantUnit finite non-zero native units per Plant position unit;
         *                                negative values intentionally reverse direction
         * @return the required position-reference step
         * @throws IllegalArgumentException if the scale is non-finite or zero
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        PositionCoordinateReferenceStep<NEXT> scaleToNative(double nativeUnitsPerPlantUnit);
    }

    /** Coordinate-reference question for a feedback position coordinate. */
    public interface PositionCoordinateReferenceStep<NEXT> {
        /**
         * Declare that the selected map is already aligned with Plant zero at native zero.
         *
         * @return the required Plant-unit position-tolerance step
         * @throws IllegalStateException if reference was already answered or configuration froze
         */
        PositionToleranceStep<NEXT> alreadyReferenced();

        /**
         * Supply a static affine-map anchor between Plant and native position coordinates.
         *
         * <p>The anchor is coordinate data and need not lie inside the legal Plant target range.</p>
         *
         * @param plantPosition finite position in caller-facing Plant units
         * @param nativePosition finite corresponding native feedback/output position
         * @return the required Plant-unit position-tolerance step
         * @throws IllegalArgumentException if either value is non-finite, or this anchor makes an
         *                                  exact bounded-range endpoint image non-finite
         * @throws IllegalStateException if reference was already answered or configuration froze
         */
        PositionToleranceStep<NEXT> plantPositionMapsToNative(double plantPosition,
                                                              double nativePosition);

        /**
         * Align the first finite native feedback sample with a supplied Plant position.
         * The eventual update validates the complete bounded-map candidate before committing the
         * reference or its derived public measurement.
         *
         * @param plantPosition finite position in caller-facing Plant units
         * @return the required Plant-unit position-tolerance step
         * @throws IllegalArgumentException if {@code plantPosition} is non-finite
         * @throws IllegalStateException if reference was already answered or configuration froze
         */
        PositionToleranceStep<NEXT> assumeCurrentPositionIs(double plantPosition);

        /**
         * Leave the Plant unreferenced until an explicit calibration/reference operation succeeds.
         *
         * @param reason nonblank diagnostic explaining why reference is not yet established
         * @return the required Plant-unit position-tolerance step
         * @throws IllegalArgumentException if {@code reason} is null or blank
         * @throws IllegalStateException if reference was already answered or configuration froze
         */
        PositionToleranceStep<NEXT> needsReference(String reason);
    }

    /** Required feedback-position completion tolerance. */
    public interface PositionToleranceStep<NEXT> {
        /**
         * Set the inclusive completion tolerance in caller-facing Plant position units.
         *
         * @param tolerance finite tolerance greater than or equal to zero
         * @return the shared guard and target-selection step
         * @throws IllegalArgumentException if {@code tolerance} is non-finite or negative
         * @throws IllegalStateException if tolerance was already answered or configuration froze
         */
        NEXT positionTolerance(double tolerance);
    }

    /** Velocity target-range question. */
    public interface VelocityBoundsStep<NEXT> {
        /**
         * Declare a finite closed legal target range in caller-facing Plant velocity units.
         *
         * @param min inclusive Plant-unit velocity minimum
         * @param max inclusive Plant-unit velocity maximum
         * @return the required velocity-mapping step
         * @throws IllegalArgumentException if either bound is non-finite or {@code min > max}
         * @throws IllegalStateException if bounds were already answered or configuration froze
         */
        VelocityMappingStep<NEXT> bounded(double min, double max);

        /**
         * Declare no numeric velocity bounds beyond the requirement that every target remain finite.
         *
         * @return the required velocity-mapping step
         * @throws IllegalStateException if bounds were already answered or configuration froze
         */
        VelocityMappingStep<NEXT> unbounded();
    }

    /** Zero-preserving velocity mapping question. */
    public interface VelocityMappingStep<NEXT> {
        /**
         * Use an identity map between Plant and native velocity units.
         *
         * @return the required Plant-unit velocity-tolerance step
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        VelocityToleranceStep<NEXT> nativeUnits();

        /**
         * Set the zero-preserving Plant-to-native velocity scale.
         *
         * @param nativeUnitsPerPlantVelocityUnit finite non-zero native velocity units per Plant
         *                                        velocity unit; negative values reverse direction
         * @return the required Plant-unit velocity-tolerance step
         * @throws IllegalArgumentException if the scale is non-finite or zero, or a declared
         *                                  bounded range has a non-finite exact native endpoint image
         * @throws IllegalStateException if mapping was already answered or configuration froze
         */
        VelocityToleranceStep<NEXT> scaleToNative(double nativeUnitsPerPlantVelocityUnit);
    }

    /** Required velocity completion tolerance. */
    public interface VelocityToleranceStep<NEXT> {
        /**
         * Set the inclusive completion tolerance in caller-facing Plant velocity units.
         *
         * @param tolerance finite tolerance greater than or equal to zero
         * @return the shared guard and target-selection step
         * @throws IllegalArgumentException if {@code tolerance} is non-finite or negative
         * @throws IllegalStateException if tolerance was already answered or configuration froze
         */
        NEXT velocityTolerance(double tolerance);
    }

    /**
     * Optional symmetric normal-output policy supported by an evidence-bearing command path.
     * Omitting this stage leaves the full normalized output-power domain available.
     */
    public interface SymmetricOutputPowerPolicyStep<P extends Plant> extends TargetStep<P> {
        /**
         * Limit normal control to the symmetric normalized interval
         * {@code [-maximumMagnitude, +maximumMagnitude]}.
         *
         * <p>Omitting this answer retains the full normalized magnitude {@code 1.0}. This policy
         * does not constrain a position Plant's separately authorized calibration-search power.</p>
         *
         * @param maximumMagnitude finite magnitude in {@code [0.0, 1.0]}
         * @return target selection with the output-power question closed
         * @throws IllegalArgumentException if {@code maximumMagnitude} is non-finite or outside
         *                                  {@code [0.0, 1.0]}
         * @throws IllegalStateException if output power was already answered or configuration froze
         */
        TargetStep<P> outputPowerLimitedTo(double maximumMagnitude);
    }

    /** Optional signed normal-output policy for Sushi-regulated {@link PowerOutput} paths. */
    public interface OutputPowerPolicyStep<P extends Plant>
            extends SymmetricOutputPowerPolicyStep<P> {
        /**
         * Limit normal control to one inclusive normalized output-power interval.
         *
         * @param minimum finite lower endpoint in {@code [-1.0, 1.0]}
         * @param maximum finite upper endpoint in {@code [-1.0, 1.0]}
         * @return target selection with the output-power question closed
         * @throws IllegalArgumentException if the interval is invalid or excludes zero
         * @throws IllegalStateException if output power was already answered or configuration froze
         */
        TargetStep<P> outputPowerLimitedTo(double minimum, double maximum);

        /**
         * Apply bounded supply-voltage compensation after feedback plus feedforward and before the
         * optional output-power policy.
         *
         * <p>A positive finite sample uses
         * {@code min(referenceVoltage / max(sample, minimumVoltage), maximumScale)}. A non-finite
         * or non-positive runtime sample uses scale {@code 1.0}. The source is sampled at most once
         * per Plant update through its memoized view. Omitting this answer applies no voltage
         * compensation.</p>
         *
         * @param supplyVoltage current supply voltage in volts
         * @param referenceVoltage finite positive voltage at which the control law was tuned
         * @param minimumVoltage finite positive denominator floor, no greater than
         *                       {@code referenceVoltage}
         * @param maximumScale finite upper compensation multiplier, at least {@code 1.0}
         * @return the post-compensation output-policy step; compensation cannot be answered again
         * @throws NullPointerException if {@code supplyVoltage} is null
         * @throws IllegalArgumentException if the numeric voltage policy is invalid
         * @throws IllegalStateException if control is incomplete, compensation was already
         *                               answered, output power was already answered, or
         *                               configuration froze
         */
        OutputPowerAfterVoltageStep<P> voltageCompensationFrom(
                ScalarSource supplyVoltage,
                double referenceVoltage,
                double minimumVoltage,
                double maximumScale);
    }

    /** Output policy after voltage compensation has been selected once. */
    public interface OutputPowerAfterVoltageStep<P extends Plant>
            extends SymmetricOutputPowerPolicyStep<P> {
        /**
         * Limit the voltage-compensated command to one inclusive normalized output-power interval.
         *
         * @param minimum finite lower endpoint satisfying {@code -1 <= minimum <= 0}
         * @param maximum finite upper endpoint satisfying {@code 0 <= maximum <= 1}
         * @return target selection with the output-power question closed
         * @throws IllegalArgumentException if the endpoints are non-finite, unordered, outside
         *                                  {@code [-1.0, 1.0]}, or exclude zero
         * @throws IllegalStateException if output power was already answered or configuration froze
         */
        TargetStep<P> outputPowerLimitedTo(double minimum, double maximum);
    }

    /**
     * Standard or explicitly custom controller choice after position units and tolerance exist.
     * The ordinary standard branch owns one setpoint model and PID law; omitting a typed
     * feedforward answer means exact zero feedforward.
     */
    public interface PositionControlStep {
        /**
         * Use the final guarded Plant target directly as each cycle's position setpoint.
         * Setpoint velocity and acceleration are exactly zero and the setpoint is immediately
         * settled.
         *
         * @return the required position-PID step
         * @throws IllegalStateException if control was already selected or configuration froze
         */
        PositionDirectFeedbackStep setpointFromAppliedTarget();

        /**
         * Generate one trapezoidal, or short-move triangular, position setpoint.
         * The first update seeds position from the finite Plant measurement with zero velocity and
         * acceleration; later goal changes replan from retained setpoint state.
         *
         * @param maximumVelocity finite positive Plant position units per second
         * @param maximumAcceleration finite positive Plant position units per second squared
         * @return the required profiled-position PID step
         * @throws IllegalArgumentException if either limit is non-finite or not positive
         * @throws IllegalStateException if control was already selected or configuration froze
         */
        PositionProfiledFeedbackStep setpointFromTrapezoidalProfile(
                double maximumVelocity,
                double maximumAcceleration);

        /**
         * Use one advanced complete regulator receiving applied target and Plant-unit measurement
         * directly, without claiming typed setpoint velocity or acceleration evidence.
         *
         * <p>The completed Plant becomes the sole update/reset owner of this stateful instance; do
         * not share one regulator instance across Plants. This compatibility exit is for control
         * laws that cannot use the standard typed branch.</p>
         *
         * @param regulator fresh complete regulator for this Plant
         * @return the optional voltage/output-power policy step
         * @throws NullPointerException if {@code regulator} is null
         * @throws IllegalStateException if control was already selected or configuration froze
         */
        OutputPowerPolicyStep<PositionPlant> controlFromCustomRegulator(
                ScalarRegulator regulator);
    }

    /** PID choice for a direct position setpoint. */
    public interface PositionDirectFeedbackStep {
        /**
         * Select P-only feedback; integral and derivative gains are exactly zero.
         *
         * @param kP finite normalized output power per Plant position-error unit
         * @return direct-position PID policy and optional feedforward
         * @throws IllegalArgumentException if {@code kP} is non-finite
         * @throws IllegalStateException if feedback was already answered or configuration froze
         */
        PositionDirectPidStep feedbackFromPid(double kP);

        /**
         * Select PID feedback over position error.
         *
         * @param kP finite normalized output power per Plant position-error unit
         * @param kI finite normalized output power per Plant position-error-unit second
         * @param kD finite normalized output power-seconds per Plant position-error unit
         * @return direct-position PID policy and optional feedforward
         * @throws IllegalArgumentException if any gain is non-finite
         * @throws IllegalStateException if feedback was already answered or configuration froze
         */
        PositionDirectPidStep feedbackFromPid(double kP, double kI, double kD);
    }

    /** PID choice for a profiled position setpoint. */
    public interface PositionProfiledFeedbackStep {
        /**
         * Select P-only feedback; integral and derivative gains are exactly zero.
         *
         * @param kP finite normalized output power per Plant position-error unit
         * @return profiled-position PID policy and optional typed feedforward
         * @throws IllegalArgumentException if {@code kP} is non-finite
         * @throws IllegalStateException if feedback was already answered or configuration froze
         */
        PositionProfiledPidStep feedbackFromPid(double kP);

        /**
         * Select PID feedback over profiled position error.
         *
         * @param kP finite normalized output power per Plant position-error unit
         * @param kI finite normalized output power per Plant position-error-unit second
         * @param kD finite normalized output power-seconds per Plant position-error unit
         * @return profiled-position PID policy and optional typed feedforward
         * @throws IllegalArgumentException if any gain is non-finite
         * @throws IllegalStateException if feedback was already answered or configuration froze
         */
        PositionProfiledPidStep feedbackFromPid(double kP, double kI, double kD);
    }

    /** Direct-position PID policies and evidence-compatible optional gravity models. */
    public interface PositionDirectPidStep extends OutputPowerPolicyStep<PositionPlant> {
        /**
         * Limit the retained integral contribution in normalized output-power units.
         *
         * @param minimum finite lower endpoint, no greater than zero
         * @param maximum finite upper endpoint, no less than zero
         * @return this PID stage for another optional PID policy or feedforward answer
         * @throws IllegalArgumentException if the interval is invalid or excludes zero
         * @throws IllegalStateException if already answered, a later control stage was selected,
         *                               or configuration froze
         */
        PositionDirectPidStep feedbackIntegralLimitedTo(double minimum, double maximum);

        /**
         * Limit the complete PID feedback contribution before feedforward is added.
         *
         * @param minimum finite inclusive normalized-output contribution minimum
         * @param maximum finite inclusive normalized-output contribution maximum
         * @return this PID stage for another optional PID policy or feedforward answer
         * @throws IllegalArgumentException if the endpoints are unordered or non-finite
         * @throws IllegalStateException if already answered, a later control stage was selected,
         *                               or configuration froze
         */
        PositionDirectPidStep feedbackOutputLimitedTo(double minimum, double maximum);

        /**
         * Add constant lift gravity feedforward {@code kG}; direct position has no motion term.
         *
         * @param kG finite signed normalized output power opposing gravity
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if {@code kG} is non-finite
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<PositionPlant> feedforwardFromLift(double kG);

        /**
         * Add arm gravity feedforward
         * {@code kG*cos((pSetpoint-plantPositionAtMaximumGravity)*radiansPerPlantUnit)}.
         *
         * @param kG finite signed maximum normalized gravity-compensation power
         * @param plantPositionAtMaximumGravity finite Plant position where cosine is {@code 1}
         * @param radiansPerPlantUnit finite nonzero coordinate conversion
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if an argument is non-finite or the conversion is zero
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<PositionPlant> feedforwardFromArm(
                double kG,
                double plantPositionAtMaximumGravity,
                double radiansPerPlantUnit);
    }

    /** Profiled-position PID policies and p/v/a-compatible optional feedforward models. */
    public interface PositionProfiledPidStep extends OutputPowerPolicyStep<PositionPlant> {
        /**
         * Limit the retained integral contribution in normalized output-power units.
         * The finite ordered interval must contain zero.
         *
         * @param minimum finite lower endpoint, no greater than zero
         * @param maximum finite upper endpoint, no less than zero
         * @return this PID stage for another optional PID policy or feedforward answer
         * @throws IllegalArgumentException if the interval is invalid or excludes zero
         * @throws IllegalStateException if already answered, a later stage was selected, or frozen
         */
        PositionProfiledPidStep feedbackIntegralLimitedTo(double minimum, double maximum);

        /**
         * Limit the complete PID feedback contribution before feedforward is added.
         *
         * @param minimum finite inclusive contribution minimum
         * @param maximum finite inclusive contribution maximum
         * @return this PID stage for another optional PID policy or feedforward answer
         * @throws IllegalArgumentException if the endpoints are unordered or non-finite
         * @throws IllegalStateException if already answered, a later stage was selected, or frozen
         */
        PositionProfiledPidStep feedbackOutputLimitedTo(double minimum, double maximum);

        /**
         * Add motion feedforward {@code kV*vSetpoint}; {@code kS} and {@code kA} are zero.
         *
         * @param kV finite normalized power divided by Plant position units per second
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if {@code kV} is non-finite
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<PositionPlant> feedforwardFromMotion(double kV);

        /**
         * Add motion feedforward
         * {@code kS*sign(vSetpoint)+kV*vSetpoint+kA*aSetpoint}; {@code sign(0)} is zero.
         *
         * @param kS finite signed normalized static power
         * @param kV finite normalized power divided by Plant position units per second
         * @param kA finite normalized power divided by Plant position units per second squared
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if any gain is non-finite
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<PositionPlant> feedforwardFromMotion(
                double kS, double kV, double kA);

        /**
         * Add constant lift gravity feedforward {@code kG}; motion gains are exactly zero.
         *
         * @param kG finite signed normalized output power opposing gravity
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if {@code kG} is non-finite
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<PositionPlant> feedforwardFromLift(double kG);

        /**
         * Add lift feedforward
         * {@code kG+kS*sign(vSetpoint)+kV*vSetpoint+kA*aSetpoint}.
         *
         * @param kG finite signed normalized gravity-compensation power
         * @param kS finite signed normalized static power
         * @param kV finite normalized power divided by Plant position units per second
         * @param kA finite normalized power divided by Plant position units per second squared
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if any gain is non-finite
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<PositionPlant> feedforwardFromLift(
                double kG, double kS, double kV, double kA);

        /**
         * Add arm gravity feedforward
         * {@code kG*cos((pSetpoint-plantPositionAtMaximumGravity)*radiansPerPlantUnit)}.
         *
         * @param kG finite signed maximum normalized gravity-compensation power
         * @param plantPositionAtMaximumGravity finite Plant position where cosine is {@code 1}
         * @param radiansPerPlantUnit finite nonzero coordinate conversion
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if a value is non-finite or the conversion is zero
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<PositionPlant> feedforwardFromArm(
                double kG,
                double plantPositionAtMaximumGravity,
                double radiansPerPlantUnit);

        /**
         * Add the full motion term plus the arm-gravity cosine term. Arguments are ordered as
         * gravity geometry first, then {@code kS}, {@code kV}, and {@code kA} motion gains.
         *
         * @param kG finite signed maximum normalized gravity-compensation power
         * @param plantPositionAtMaximumGravity finite Plant position where cosine is {@code 1}
         * @param radiansPerPlantUnit finite nonzero coordinate conversion
         * @param kS finite signed normalized static power
         * @param kV finite normalized power divided by Plant position units per second
         * @param kA finite normalized power divided by Plant position units per second squared
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if a value is non-finite or the conversion is zero
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<PositionPlant> feedforwardFromArm(
                double kG,
                double plantPositionAtMaximumGravity,
                double radiansPerPlantUnit,
                double kS,
                double kV,
                double kA);
    }

    /**
     * Standard or explicitly custom controller choice after velocity units and tolerance exist.
     * Omitting a typed feedforward answer means exact zero feedforward.
     */
    public interface VelocityControlStep {
        /**
         * Use the final guarded Plant velocity target directly as each cycle's setpoint.
         * Acceleration evidence is unavailable and the setpoint is immediately settled.
         *
         * @return the required direct-velocity PID step
         * @throws IllegalStateException if control was already selected or configuration froze
         */
        VelocityDirectFeedbackStep setpointFromAppliedTarget();

        /**
         * Generate an acceleration-limited velocity setpoint, seeding the first update from the
         * finite measured velocity with zero acceleration.
         *
         * @param maximumAcceleration finite positive Plant velocity units per second
         * @return the required profiled-velocity PID step
         * @throws IllegalArgumentException if the limit is non-finite or not positive
         * @throws IllegalStateException if control was already selected or configuration froze
         */
        VelocityProfiledFeedbackStep setpointFromAccelerationLimitedProfile(
                double maximumAcceleration);

        /**
         * Use one advanced complete regulator receiving applied target and Plant-unit measurement
         * directly. The completed Plant solely owns updates and reset; do not share this stateful
         * instance across Plants.
         *
         * @param regulator fresh complete regulator for this Plant
         * @return the optional voltage/output-power policy step
         * @throws NullPointerException if {@code regulator} is null
         * @throws IllegalStateException if control was already selected or configuration froze
         */
        OutputPowerPolicyStep<Plant> controlFromCustomRegulator(ScalarRegulator regulator);
    }

    /** PID choice for a direct velocity setpoint. */
    public interface VelocityDirectFeedbackStep {
        /**
         * Select P-only feedback; integral and derivative gains are exactly zero.
         *
         * @param kP finite normalized output power per Plant velocity-error unit
         * @return direct-velocity PID policy and optional typed feedforward
         * @throws IllegalArgumentException if {@code kP} is non-finite
         * @throws IllegalStateException if feedback was already answered or configuration froze
         */
        VelocityDirectPidStep feedbackFromPid(double kP);

        /**
         * Select PID feedback over Plant velocity error.
         *
         * @param kP finite normalized output power per Plant velocity-error unit
         * @param kI finite normalized output power per Plant velocity-error-unit second
         * @param kD finite normalized output power-seconds per Plant velocity-error unit
         * @return direct-velocity PID policy and optional typed feedforward
         * @throws IllegalArgumentException if any gain is non-finite
         * @throws IllegalStateException if feedback was already answered or configuration froze
         */
        VelocityDirectPidStep feedbackFromPid(double kP, double kI, double kD);
    }

    /** PID choice for an acceleration-limited velocity setpoint. */
    public interface VelocityProfiledFeedbackStep {
        /**
         * Select P-only feedback; integral and derivative gains are exactly zero.
         *
         * @param kP finite normalized output power per Plant velocity-error unit
         * @return profiled-velocity PID policy and optional typed feedforward
         * @throws IllegalArgumentException if {@code kP} is non-finite
         * @throws IllegalStateException if feedback was already answered or configuration froze
         */
        VelocityProfiledPidStep feedbackFromPid(double kP);

        /**
         * Select PID feedback over the acceleration-limited velocity error.
         *
         * @param kP finite normalized output power per Plant velocity-error unit
         * @param kI finite normalized output power per Plant velocity-error-unit second
         * @param kD finite normalized output power-seconds per Plant velocity-error unit
         * @return profiled-velocity PID policy and optional typed feedforward
         * @throws IllegalArgumentException if any gain is non-finite
         * @throws IllegalStateException if feedback was already answered or configuration froze
         */
        VelocityProfiledPidStep feedbackFromPid(double kP, double kI, double kD);
    }

    /** Direct-velocity PID policies and v-compatible optional feedforward models. */
    public interface VelocityDirectPidStep extends OutputPowerPolicyStep<Plant> {
        /**
         * Limit the retained integral contribution in normalized output-power units.
         *
         * @param minimum finite lower endpoint, no greater than zero
         * @param maximum finite upper endpoint, no less than zero
         * @return this PID stage for another optional PID policy or feedforward answer
         * @throws IllegalArgumentException if the interval is invalid or excludes zero
         * @throws IllegalStateException if already answered, a later stage was selected, or frozen
         */
        VelocityDirectPidStep feedbackIntegralLimitedTo(double minimum, double maximum);

        /**
         * Limit the complete PID feedback contribution before feedforward is added.
         *
         * @param minimum finite inclusive contribution minimum
         * @param maximum finite inclusive contribution maximum
         * @return this PID stage for another optional PID policy or feedforward answer
         * @throws IllegalArgumentException if the endpoints are unordered or non-finite
         * @throws IllegalStateException if already answered, a later stage was selected, or frozen
         */
        VelocityDirectPidStep feedbackOutputLimitedTo(double minimum, double maximum);

        /**
         * Add {@code kV*vSetpoint}; {@code kS} is zero.
         *
         * @param kV finite normalized power divided by one Plant velocity unit
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if {@code kV} is non-finite
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<Plant> feedforwardFromMotion(double kV);

        /**
         * Add {@code kS*sign(vSetpoint)+kV*vSetpoint}; {@code sign(0)} is zero.
         *
         * @param kS finite signed normalized static power
         * @param kV finite normalized power divided by one Plant velocity unit
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if either gain is non-finite
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<Plant> feedforwardFromMotion(double kS, double kV);

        /**
         * Add constant lift gravity feedforward {@code kG}; motion gains are exactly zero.
         *
         * @param kG finite signed normalized output power opposing gravity
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if {@code kG} is non-finite
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<Plant> feedforwardFromLift(double kG);

        /**
         * Add {@code kG+kS*sign(vSetpoint)+kV*vSetpoint}.
         *
         * @param kG finite signed normalized gravity-compensation power
         * @param kS finite signed normalized static power
         * @param kV finite normalized power divided by one Plant velocity unit
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if any gain is non-finite
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<Plant> feedforwardFromLift(double kG, double kS, double kV);
    }

    /** Acceleration-limited velocity PID policies and v/a-compatible feedforward models. */
    public interface VelocityProfiledPidStep extends OutputPowerPolicyStep<Plant> {
        /**
         * Limit the retained integral contribution in normalized output-power units.
         *
         * @param minimum finite lower endpoint, no greater than zero
         * @param maximum finite upper endpoint, no less than zero
         * @return this PID stage for another optional PID policy or feedforward answer
         * @throws IllegalArgumentException if the interval is invalid or excludes zero
         * @throws IllegalStateException if already answered, a later stage was selected, or frozen
         */
        VelocityProfiledPidStep feedbackIntegralLimitedTo(double minimum, double maximum);

        /**
         * Limit the complete PID feedback contribution before feedforward is added.
         *
         * @param minimum finite inclusive contribution minimum
         * @param maximum finite inclusive contribution maximum
         * @return this PID stage for another optional PID policy or feedforward answer
         * @throws IllegalArgumentException if the endpoints are unordered or non-finite
         * @throws IllegalStateException if already answered, a later stage was selected, or frozen
         */
        VelocityProfiledPidStep feedbackOutputLimitedTo(double minimum, double maximum);

        /**
         * Add {@code kV*vSetpoint}; {@code kS} and {@code kA} are zero.
         *
         * @param kV finite normalized power divided by one Plant velocity unit
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if {@code kV} is non-finite
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<Plant> feedforwardFromMotion(double kV);

        /**
         * Add {@code kS*sign(vSetpoint)+kV*vSetpoint+kA*aSetpoint}.
         *
         * @param kS finite signed normalized static power
         * @param kV finite normalized power divided by one Plant velocity unit
         * @param kA finite normalized power divided by Plant velocity units per second
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if any gain is non-finite
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<Plant> feedforwardFromMotion(
                double kS, double kV, double kA);

        /**
         * Add constant lift gravity feedforward {@code kG}; motion gains are exactly zero.
         *
         * @param kG finite signed normalized output power opposing gravity
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if {@code kG} is non-finite
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<Plant> feedforwardFromLift(double kG);

        /**
         * Add {@code kG+kS*sign(vSetpoint)+kV*vSetpoint+kA*aSetpoint}.
         *
         * @param kG finite signed normalized gravity-compensation power
         * @param kS finite signed normalized static power
         * @param kV finite normalized power divided by one Plant velocity unit
         * @param kA finite normalized power divided by Plant velocity units per second
         * @return optional voltage/output-power policy with feedforward closed
         * @throws IllegalArgumentException if any gain is non-finite
         * @throws IllegalStateException if feedforward was already answered or configuration froze
         */
        OutputPowerPolicyStep<Plant> feedforwardFromLift(
                double kG, double kS, double kV, double kA);
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
        public DeviceManagedPositionStep<TargetStep<PositionPlant>> deviceManagedPosition(
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
        public DeviceManagedPositionStep<SymmetricOutputPowerPolicyStep<PositionPlant>>
        deviceManagedPosition(
                PowerLimitedPositionOutput out,
                ScalarSource nativeMeasurement) {
            requireUnused();
            PowerLimitedPositionOutput checkedOut = Objects.requireNonNull(
                    out, "power-limited position output");
            ScalarSource checkedMeasurement = Objects.requireNonNull(
                    nativeMeasurement, "native position measurement");
            selected = true;
            return FeedbackPositionBuilder.powerLimitedDeviceManaged(
                    checkedOut, checkedMeasurement);
        }

        @Override
        public VelocityBoundsStep<TargetStep<Plant>> deviceManagedVelocity(
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
        public PositionPeriodicityStep<FeedbackPositionBoundsStep<PositionControlStep>>
        regulatedPosition(
                PowerOutput out,
                ScalarSource nativeMeasurement) {
            requireUnused();
            PowerOutput checkedOut = Objects.requireNonNull(out, "regulated power output");
            ScalarSource checkedMeasurement = Objects.requireNonNull(
                    nativeMeasurement, "native position measurement");
            selected = true;
            return FeedbackPositionBuilder.regulated(checkedOut, checkedMeasurement);
        }

        @Override
        public VelocityBoundsStep<VelocityControlStep> regulatedVelocity(
                PowerOutput out,
                ScalarSource nativeMeasurement) {
            requireUnused();
            PowerOutput checkedOut = Objects.requireNonNull(out, "regulated power output");
            ScalarSource checkedMeasurement = Objects.requireNonNull(
                    nativeMeasurement, "native velocity measurement");
            selected = true;
            return VelocityBuilder.regulated(checkedOut, checkedMeasurement);
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
            requireTargetRateCompatible("maxTargetRate(...)");
            guardBuilder.maxTargetRate(maxDeltaPerSec);
            targetRateConfigured = true;
            return this;
        }

        @Override
        public final TargetGuardStep<P> maxTargetRates(double maxUpPerSec, double maxDownPerSec) {
            requireGuardBranch("maxTargetRates(...)");
            requireTargetRateUnanswered();
            requireTargetRateCompatible("maxTargetRates(...)");
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

        /** Reject a target-rate guard before it can mutate a recipe with another motion owner. */
        protected void requireTargetRateCompatible(String answer) {
            // Most Plant realizations have no competing motion-profile owner.
        }

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

        protected final boolean hasTargetRateConfigured() {
            return targetRateConfigured;
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

    private static final class VelocityBuilder<NEXT> extends TargetBuilder<Plant>
            implements VelocityBoundsStep<NEXT>, VelocityMappingStep<NEXT>,
            VelocityToleranceStep<NEXT>, VelocityControlStep,
            VelocityDirectFeedbackStep, VelocityProfiledFeedbackStep,
            VelocityDirectPidStep, VelocityProfiledPidStep,
            OutputPowerPolicyStep<Plant>, OutputPowerAfterVoltageStep<Plant> {
        private final VelocityOutput velocityOut;
        private final PowerOutput regulatedPowerOut;
        private final ScalarSource nativeMeasurement;
        private ScalarRegulator customRegulator;
        private StandardControl.Config standardControl;
        private ScalarRange range;
        private double nativePerPlantUnit;
        private double tolerance;
        private ScalarSource supplyVoltage;
        private double referenceVoltage;
        private double minimumVoltage;
        private double maximumVoltageScale;
        private double minimumOutputPower = -1.0;
        private double maximumOutputPower = 1.0;
        private boolean rangeAnswered;
        private boolean mappingAnswered;
        private boolean toleranceAnswered;
        private boolean controlAnswered;
        private boolean feedbackAnswered;
        private boolean feedforwardAnswered;
        private boolean voltageAnswered;
        private boolean outputPolicyAnswered;
        private boolean profiledSetpoint;

        private VelocityBuilder(VelocityOutput velocityOut,
                                PowerOutput regulatedPowerOut,
                                ScalarSource nativeMeasurement) {
            this.velocityOut = velocityOut;
            this.regulatedPowerOut = regulatedPowerOut;
            this.nativeMeasurement = nativeMeasurement;
        }

        @Override
        protected void requireTargetRateCompatible(String answer) {
            if (profiledSetpoint) {
                throw new IllegalStateException(answer + " cannot be combined with "
                        + "setpointFromAccelerationLimitedProfile(...); choose one "
                        + "motion-shaping owner");
            }
        }

        static VelocityBuilder<TargetStep<Plant>> deviceManaged(
                VelocityOutput out,
                ScalarSource nativeMeasurement) {
            return new VelocityBuilder<>(out, null, nativeMeasurement);
        }

        static VelocityBuilder<VelocityControlStep> regulated(
                PowerOutput out,
                ScalarSource nativeMeasurement) {
            return new VelocityBuilder<>(null, out, nativeMeasurement);
        }

        @Override
        public VelocityMappingStep<NEXT> bounded(double min, double max) {
            requireConfigurationMutable("bounded(...)");
            requireRangeUnanswered();
            range = ScalarRange.bounded(min, max);
            rangeAnswered = true;
            return this;
        }

        @Override
        public VelocityMappingStep<NEXT> unbounded() {
            requireConfigurationMutable("unbounded()");
            requireRangeUnanswered();
            range = ScalarRange.unbounded();
            rangeAnswered = true;
            return this;
        }

        @Override
        public VelocityToleranceStep<NEXT> nativeUnits() {
            return answerMapping(1.0, "nativeUnits()");
        }

        @Override
        public VelocityToleranceStep<NEXT> scaleToNative(
                double nativeUnitsPerPlantVelocityUnit) {
            return answerMapping(nativeUnitsPerPlantVelocityUnit, "scaleToNative(...)");
        }

        @Override
        @SuppressWarnings("unchecked")
        public NEXT velocityTolerance(double tolerance) {
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
            return (NEXT) this;
        }

        @Override
        public VelocityDirectFeedbackStep setpointFromAppliedTarget() {
            requireRegulatedControlPending("setpointFromAppliedTarget()");
            standardControl = StandardControl.velocityFromAppliedTarget();
            controlAnswered = true;
            profiledSetpoint = false;
            return this;
        }

        @Override
        public VelocityProfiledFeedbackStep setpointFromAccelerationLimitedProfile(
                double maximumAcceleration) {
            requireRegulatedControlPending("setpointFromAccelerationLimitedProfile(...)");
            standardControl = StandardControl.velocityFromAccelerationLimitedProfile(
                    maximumAcceleration);
            controlAnswered = true;
            profiledSetpoint = true;
            return this;
        }

        @Override
        public OutputPowerPolicyStep<Plant> controlFromCustomRegulator(
                ScalarRegulator regulator) {
            requireRegulatedControlPending("controlFromCustomRegulator(...)");
            customRegulator = Objects.requireNonNull(regulator, "custom regulator");
            controlAnswered = true;
            return this;
        }

        @Override
        public VelocityBuilder<NEXT> feedbackFromPid(double kP) {
            requireStandardFeedbackPending("feedbackFromPid(...)");
            standardControl.feedbackFromPid(kP);
            feedbackAnswered = true;
            return this;
        }

        @Override
        public VelocityBuilder<NEXT> feedbackFromPid(double kP, double kI, double kD) {
            requireStandardFeedbackPending("feedbackFromPid(...)");
            standardControl.feedbackFromPid(kP, kI, kD);
            feedbackAnswered = true;
            return this;
        }

        @Override
        public VelocityBuilder<NEXT> feedbackIntegralLimitedTo(
                double minimum,
                double maximum) {
            requireStandardFeedbackAnswered("feedbackIntegralLimitedTo(...)");
            standardControl.feedbackIntegralLimitedTo(minimum, maximum);
            return this;
        }

        @Override
        public VelocityBuilder<NEXT> feedbackOutputLimitedTo(
                double minimum,
                double maximum) {
            requireStandardFeedbackAnswered("feedbackOutputLimitedTo(...)");
            standardControl.feedbackOutputLimitedTo(minimum, maximum);
            return this;
        }

        @Override
        public OutputPowerPolicyStep<Plant> feedforwardFromMotion(double kV) {
            requireStandardFeedbackAnswered("feedforwardFromMotion(...)");
            standardControl.feedforwardForMotion(kV);
            feedforwardAnswered = true;
            return this;
        }

        @Override
        public OutputPowerPolicyStep<Plant> feedforwardFromMotion(double kS, double kV) {
            requireDirectVelocity("feedforwardFromMotion(kS, kV)");
            standardControl.feedforwardForMotion(kS, kV);
            feedforwardAnswered = true;
            return this;
        }

        @Override
        public OutputPowerPolicyStep<Plant> feedforwardFromMotion(
                double kS,
                double kV,
                double kA) {
            requireProfiledVelocity("feedforwardFromMotion(kS, kV, kA)");
            standardControl.feedforwardForMotion(kS, kV, kA);
            feedforwardAnswered = true;
            return this;
        }

        @Override
        public OutputPowerPolicyStep<Plant> feedforwardFromLift(double kG) {
            requireStandardFeedbackAnswered("feedforwardFromLift(...)");
            standardControl.feedforwardForLift(kG);
            feedforwardAnswered = true;
            return this;
        }

        @Override
        public OutputPowerPolicyStep<Plant> feedforwardFromLift(
                double kG,
                double kS,
                double kV) {
            requireDirectVelocity("feedforwardFromLift(kG, kS, kV)");
            standardControl.feedforwardForLift(kS, kV, kG);
            feedforwardAnswered = true;
            return this;
        }

        @Override
        public OutputPowerPolicyStep<Plant> feedforwardFromLift(
                double kG,
                double kS,
                double kV,
                double kA) {
            requireProfiledVelocity("feedforwardFromLift(kG, kS, kV, kA)");
            standardControl.feedforwardForLift(kS, kV, kA, kG);
            feedforwardAnswered = true;
            return this;
        }

        @Override
        public OutputPowerAfterVoltageStep<Plant> voltageCompensationFrom(
                ScalarSource supplyVoltage,
                double referenceVoltage,
                double minimumVoltage,
                double maximumScale) {
            requireOutputPolicyAvailable("voltageCompensationFrom(...)");
            if (outputPolicyAnswered) {
                throw new IllegalStateException(
                        "voltageCompensationFrom(...) must be answered before "
                                + "outputPowerLimitedTo(...)");
            }
            if (voltageAnswered) {
                throw new IllegalStateException(
                        "voltageCompensationFrom(...) has already been answered");
            }
            if (standardControl != null) {
                standardControl.voltageCompensatedBy(
                        supplyVoltage, referenceVoltage, minimumVoltage, maximumScale);
            } else {
                this.supplyVoltage = Objects.requireNonNull(supplyVoltage, "supplyVoltage");
                this.referenceVoltage = referenceVoltage;
                this.minimumVoltage = minimumVoltage;
                this.maximumVoltageScale = maximumScale;
                // Validate through the existing advanced decorator without retaining it yet.
                ScalarRegulators.voltageCompensated(
                        customRegulator, supplyVoltage, referenceVoltage,
                        minimumVoltage, maximumScale);
            }
            voltageAnswered = true;
            return this;
        }

        @Override
        public TargetStep<Plant> outputPowerLimitedTo(double maximumMagnitude) {
            requireOutputPolicyUnanswered();
            requireSymmetricOutputMagnitude(maximumMagnitude);
            minimumOutputPower = -maximumMagnitude;
            maximumOutputPower = maximumMagnitude;
            if (standardControl != null) standardControl.outputPowerLimitedTo(maximumMagnitude);
            outputPolicyAnswered = true;
            return this;
        }

        @Override
        public TargetStep<Plant> outputPowerLimitedTo(double minimum, double maximum) {
            requireOutputPolicyUnanswered();
            requireOutputPowerRange(minimum, maximum);
            minimumOutputPower = minimum;
            maximumOutputPower = maximum;
            if (standardControl != null) standardControl.outputPowerLimitedTo(minimum, maximum);
            outputPolicyAnswered = true;
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
            MappedVelocityPlant.requireFiniteBoundedMap(
                    range, nativePerPlantUnit, "Plants velocity configuration");
            if (regulatedPowerOut != null && !controlAnswered) {
                throw new IllegalStateException("Regulated velocity construction requires "
                        + "setpointFromAppliedTarget(), "
                        + "setpointFromAccelerationLimitedProfile(...), or "
                        + "controlFromCustomRegulator(...)");
            }
            if (standardControl != null && !feedbackAnswered) {
                throw new IllegalStateException("Standard regulated velocity construction requires "
                        + "feedbackFromPid(...)");
            }
            if (profiledSetpoint && hasTargetRateConfigured()) {
                throw new IllegalStateException("A profiled velocity controller cannot also use "
                        + "maxTargetRate(...) or maxTargetRates(...); choose one motion-shaping owner");
            }
        }

        @Override
        protected Plant buildPlant(PlantTargetResolver resolver, PlantTargetGuards guards) {
            MappedVelocityPlant.FeedbackConfigurationStep configured = velocityOut != null
                    ? MappedVelocityPlant.velocityOutput(velocityOut, nativeMeasurement)
                    : MappedVelocityPlant.regulated(
                            regulatedPowerOut, nativeMeasurement, resolvedRegulator());
            return configured.range(range)
                    .nativePerPlantUnit(nativePerPlantUnit)
                    .velocityTolerance(tolerance)
                    .targetGuards(guards)
                    .targetFromResolver(resolver)
                    .build();
        }

        private VelocityToleranceStep<NEXT> answerMapping(double scale, String answer) {
            requireConfigurationMutable(answer);
            if (!rangeAnswered) {
                throw new IllegalStateException("Choose bounded(...) or unbounded() before " + answer);
            }
            if (mappingAnswered) {
                throw new IllegalStateException("Velocity mapping has already been answered");
            }
            requireScale(scale);
            MappedVelocityPlant.requireFiniteBoundedMap(
                    range, scale, "Plants." + answer);
            nativePerPlantUnit = scale;
            mappingAnswered = true;
            return this;
        }

        private void requireRangeUnanswered() {
            if (rangeAnswered) {
                throw new IllegalStateException("Velocity bounds have already been answered");
            }
        }

        private ScalarRegulator resolvedRegulator() {
            if (standardControl != null) return standardControl.build();
            ScalarRegulator resolved = customRegulator;
            if (voltageAnswered) {
                resolved = ScalarRegulators.voltageCompensated(
                        resolved, supplyVoltage, referenceVoltage,
                        minimumVoltage, maximumVoltageScale);
            }
            if (outputPolicyAnswered) {
                resolved = ScalarRegulators.outputLimited(
                        resolved, minimumOutputPower, maximumOutputPower);
            }
            return resolved;
        }

        private void requireRegulatedControlPending(String operation) {
            requireConfigurationMutable(operation);
            if (!toleranceAnswered) {
                throw new IllegalStateException(operation + " requires velocityTolerance(...) first");
            }
            if (regulatedPowerOut == null) {
                throw new IllegalStateException(operation
                        + " is available only for a framework-regulated velocity Plant");
            }
            if (controlAnswered) {
                throw new IllegalStateException("Velocity control has already been answered");
            }
        }

        private void requireStandardFeedbackPending(String operation) {
            requireConfigurationMutable(operation);
            if (standardControl == null || !controlAnswered || feedbackAnswered) {
                throw new IllegalStateException(operation
                        + " requires one unanswered standard velocity setpoint selection");
            }
        }

        private void requireStandardFeedbackAnswered(String operation) {
            requireConfigurationMutable(operation);
            if (standardControl == null || !feedbackAnswered) {
                throw new IllegalStateException(operation + " requires feedbackFromPid(...) first");
            }
            if (feedforwardAnswered || voltageAnswered || outputPolicyAnswered) {
                throw new IllegalStateException(operation + " cannot change PID/feedforward after "
                        + "the control recipe advanced to feedforward, voltage, or output policy");
            }
        }

        private void requireDirectVelocity(String operation) {
            requireStandardFeedbackAnswered(operation);
            if (profiledSetpoint) {
                throw new IllegalStateException(operation
                        + " is available only for setpointFromAppliedTarget()");
            }
        }

        private void requireProfiledVelocity(String operation) {
            requireStandardFeedbackAnswered(operation);
            if (!profiledSetpoint) {
                throw new IllegalStateException(operation
                        + " requires setpointFromAccelerationLimitedProfile(...)");
            }
        }

        private void requireOutputPolicyAvailable(String operation) {
            requireConfigurationMutable(operation);
            if (!controlAnswered || (standardControl != null && !feedbackAnswered)) {
                throw new IllegalStateException(operation
                        + " requires completed standard feedback or controlFromCustomRegulator(...)");
            }
        }

        private void requireOutputPolicyUnanswered() {
            requireOutputPolicyAvailable("outputPowerLimitedTo(...)");
            if (outputPolicyAnswered) {
                throw new IllegalStateException(
                        "outputPowerLimitedTo(...) has already been answered");
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
            MappedPositionPlant.requireFiniteBoundedMap(
                    range,
                    scale,
                    range.minValue,
                    checkedNativeAtPlantMin,
                    "Plants.rangeMapsToNative(...)");
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
            MappedPositionPlant.requireFiniteBoundedMap(
                    range,
                    nativePerPlantUnit,
                    checkedPlantPosition,
                    checkedNativePosition,
                    answer);
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
            MappedPositionPlant.requireFiniteBoundedMap(
                    range,
                    nativePerPlantUnit,
                    plantReference,
                    nativeReference,
                    "Plants commanded-position configuration");
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

    private static final class FeedbackPositionBuilder<NEXT>
            extends PositionBuilderBase<PositionPlant>
            implements DeviceManagedPositionStep<NEXT>,
            PositionPeriodicityStep<FeedbackPositionBoundsStep<NEXT>>,
            FeedbackPositionBoundsStep<NEXT>,
            FeedbackBoundedPositionMappingStep<NEXT>,
            FeedbackUnboundedPositionMappingStep<NEXT>,
            PositionCoordinateReferenceStep<NEXT>,
            PositionToleranceStep<NEXT>, PositionControlStep,
            PositionDirectFeedbackStep, PositionProfiledFeedbackStep,
            PositionDirectPidStep, PositionProfiledPidStep,
            OutputPowerPolicyStep<PositionPlant>,
            OutputPowerAfterVoltageStep<PositionPlant> {
        private final PositionOutput positionOut;
        private final PowerLimitedPositionOutput powerLimitedPositionOut;
        private final PowerOutput regulatedPowerOut;
        private final ScalarSource nativeMeasurement;
        private ScalarRegulator customRegulator;
        private StandardControl.Config standardControl;
        private PowerOutput searchPowerOut;
        private MappedPositionPlant.ReferenceMode referenceMode;
        private double assumedPlantPosition;
        private String referenceReason;
        private double tolerance;
        private double maximumDeviceOutputPower = 1.0;
        private ScalarSource supplyVoltage;
        private double referenceVoltage;
        private double minimumVoltage;
        private double maximumVoltageScale;
        private double minimumOutputPower = -1.0;
        private double maximumOutputPower = 1.0;
        private boolean searchAnswered;
        private boolean toleranceAnswered;
        private boolean controlAnswered;
        private boolean feedbackAnswered;
        private boolean feedforwardAnswered;
        private boolean voltageAnswered;
        private boolean outputPolicyAnswered;
        private boolean profiledSetpoint;

        private FeedbackPositionBuilder(PositionOutput positionOut,
                                        PowerLimitedPositionOutput powerLimitedPositionOut,
                                        PowerOutput regulatedPowerOut,
                                        ScalarSource nativeMeasurement) {
            this.positionOut = positionOut;
            this.powerLimitedPositionOut = powerLimitedPositionOut;
            this.regulatedPowerOut = regulatedPowerOut;
            this.nativeMeasurement = nativeMeasurement;
            if (regulatedPowerOut != null) {
                searchPowerOut = regulatedPowerOut;
                searchAnswered = true;
            }
        }

        @Override
        protected void requireTargetRateCompatible(String answer) {
            if (profiledSetpoint) {
                throw new IllegalStateException(answer + " cannot be combined with "
                        + "setpointFromTrapezoidalProfile(...); choose one "
                        + "motion-shaping owner");
            }
        }

        static FeedbackPositionBuilder<TargetStep<PositionPlant>> deviceManaged(
                PositionOutput out,
                ScalarSource nativeMeasurement) {
            return new FeedbackPositionBuilder<>(out, null, null, nativeMeasurement);
        }

        static FeedbackPositionBuilder<SymmetricOutputPowerPolicyStep<PositionPlant>>
        powerLimitedDeviceManaged(
                PowerLimitedPositionOutput out,
                ScalarSource nativeMeasurement) {
            return new FeedbackPositionBuilder<>(out, out, null, nativeMeasurement);
        }

        static FeedbackPositionBuilder<PositionControlStep> regulated(
                PowerOutput out,
                ScalarSource nativeMeasurement) {
            return new FeedbackPositionBuilder<>(null, null, out, nativeMeasurement);
        }

        @Override
        public PositionPeriodicityStep<FeedbackPositionBoundsStep<NEXT>> searchPowerOutput(
                PowerOutput out) {
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
        public FeedbackPositionBoundsStep<NEXT> nonPeriodic() {
            answerNonPeriodic();
            return this;
        }

        @Override
        public FeedbackPositionBoundsStep<NEXT> periodic(double period) {
            answerPeriodic(period);
            return this;
        }

        @Override
        public FeedbackBoundedPositionMappingStep<NEXT> bounded(double min, double max) {
            answerBounded(min, max);
            return this;
        }

        @Override
        public FeedbackUnboundedPositionMappingStep<NEXT> unbounded() {
            answerUnbounded();
            return this;
        }

        @Override
        public PositionCoordinateReferenceStep<NEXT> nativeUnits() {
            answerScale(1.0, "nativeUnits()");
            return this;
        }

        @Override
        public PositionCoordinateReferenceStep<NEXT> scaleToNative(
                double nativeUnitsPerPlantUnit) {
            answerScale(nativeUnitsPerPlantUnit, "scaleToNative(...)");
            return this;
        }

        @Override
        public PositionToleranceStep<NEXT> rangeMapsToNative(
                double nativeAtPlantMin,
                double nativeAtPlantMax) {
            answerEndpointMap(nativeAtPlantMin, nativeAtPlantMax);
            referenceMode = MappedPositionPlant.ReferenceMode.STATIC;
            return this;
        }

        @Override
        public PositionToleranceStep<NEXT> alreadyReferenced() {
            return answerFeedbackStaticReference(0.0, 0.0, "Plants.alreadyReferenced()");
        }

        @Override
        public PositionToleranceStep<NEXT> plantPositionMapsToNative(
                double plantPosition,
                double nativePosition) {
            return answerFeedbackStaticReference(
                    plantPosition,
                    nativePosition,
                    "Plants.plantPositionMapsToNative(...)");
        }

        @Override
        public PositionToleranceStep<NEXT> assumeCurrentPositionIs(double plantPosition) {
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
        public PositionToleranceStep<NEXT> needsReference(String reason) {
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
        @SuppressWarnings("unchecked")
        public NEXT positionTolerance(double tolerance) {
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
            return (NEXT) this;
        }

        @Override
        public PositionDirectFeedbackStep setpointFromAppliedTarget() {
            requireRegulatedControlPending("setpointFromAppliedTarget()");
            standardControl = StandardControl.positionFromAppliedTarget();
            controlAnswered = true;
            profiledSetpoint = false;
            return this;
        }

        @Override
        public PositionProfiledFeedbackStep setpointFromTrapezoidalProfile(
                double maximumVelocity,
                double maximumAcceleration) {
            requireRegulatedControlPending("setpointFromTrapezoidalProfile(...)");
            standardControl = StandardControl.positionFromTrapezoidalProfile(
                    maximumVelocity, maximumAcceleration);
            controlAnswered = true;
            profiledSetpoint = true;
            return this;
        }

        @Override
        public OutputPowerPolicyStep<PositionPlant> controlFromCustomRegulator(
                ScalarRegulator regulator) {
            requireRegulatedControlPending("controlFromCustomRegulator(...)");
            customRegulator = Objects.requireNonNull(regulator, "custom regulator");
            controlAnswered = true;
            return this;
        }

        @Override
        public FeedbackPositionBuilder<NEXT> feedbackFromPid(double kP) {
            requireStandardFeedbackPending("feedbackFromPid(...)");
            standardControl.feedbackFromPid(kP);
            feedbackAnswered = true;
            return this;
        }

        @Override
        public FeedbackPositionBuilder<NEXT> feedbackFromPid(
                double kP,
                double kI,
                double kD) {
            requireStandardFeedbackPending("feedbackFromPid(...)");
            standardControl.feedbackFromPid(kP, kI, kD);
            feedbackAnswered = true;
            return this;
        }

        @Override
        public FeedbackPositionBuilder<NEXT> feedbackIntegralLimitedTo(
                double minimum,
                double maximum) {
            requireStandardFeedbackAnswered("feedbackIntegralLimitedTo(...)");
            standardControl.feedbackIntegralLimitedTo(minimum, maximum);
            return this;
        }

        @Override
        public FeedbackPositionBuilder<NEXT> feedbackOutputLimitedTo(
                double minimum,
                double maximum) {
            requireStandardFeedbackAnswered("feedbackOutputLimitedTo(...)");
            standardControl.feedbackOutputLimitedTo(minimum, maximum);
            return this;
        }

        @Override
        public OutputPowerPolicyStep<PositionPlant> feedforwardFromMotion(double kV) {
            requireProfiledPosition("feedforwardFromMotion(kV)");
            standardControl.feedforwardForMotion(kV);
            feedforwardAnswered = true;
            return this;
        }

        @Override
        public OutputPowerPolicyStep<PositionPlant> feedforwardFromMotion(
                double kS,
                double kV,
                double kA) {
            requireProfiledPosition("feedforwardFromMotion(kS, kV, kA)");
            standardControl.feedforwardForMotion(kS, kV, kA);
            feedforwardAnswered = true;
            return this;
        }

        @Override
        public OutputPowerPolicyStep<PositionPlant> feedforwardFromLift(double kG) {
            requireStandardFeedbackAnswered("feedforwardFromLift(...)");
            standardControl.feedforwardForLift(kG);
            feedforwardAnswered = true;
            return this;
        }

        @Override
        public OutputPowerPolicyStep<PositionPlant> feedforwardFromLift(
                double kG,
                double kS,
                double kV,
                double kA) {
            requireProfiledPosition("feedforwardFromLift(kG, kS, kV, kA)");
            standardControl.feedforwardForLift(kS, kV, kA, kG);
            feedforwardAnswered = true;
            return this;
        }

        @Override
        public OutputPowerPolicyStep<PositionPlant> feedforwardFromArm(
                double kG,
                double plantPositionAtMaximumGravity,
                double radiansPerPlantUnit) {
            requireStandardFeedbackAnswered("feedforwardFromArm(...)");
            standardControl.feedforwardForArm(
                    kG, plantPositionAtMaximumGravity, radiansPerPlantUnit);
            feedforwardAnswered = true;
            return this;
        }

        @Override
        public OutputPowerPolicyStep<PositionPlant> feedforwardFromArm(
                double kG,
                double plantPositionAtMaximumGravity,
                double radiansPerPlantUnit,
                double kS,
                double kV,
                double kA) {
            requireProfiledPosition("feedforwardFromArm(...)");
            standardControl.feedforwardForArm(
                    kS, kV, kA, kG,
                    plantPositionAtMaximumGravity, radiansPerPlantUnit);
            feedforwardAnswered = true;
            return this;
        }

        @Override
        public OutputPowerAfterVoltageStep<PositionPlant> voltageCompensationFrom(
                ScalarSource supplyVoltage,
                double referenceVoltage,
                double minimumVoltage,
                double maximumScale) {
            requireOutputPolicyAvailable("voltageCompensationFrom(...)");
            if (outputPolicyAnswered) {
                throw new IllegalStateException(
                        "voltageCompensationFrom(...) must be answered before "
                                + "outputPowerLimitedTo(...)");
            }
            if (voltageAnswered) {
                throw new IllegalStateException(
                        "voltageCompensationFrom(...) has already been answered");
            }
            if (standardControl != null) {
                standardControl.voltageCompensatedBy(
                        supplyVoltage, referenceVoltage, minimumVoltage, maximumScale);
            } else {
                this.supplyVoltage = Objects.requireNonNull(supplyVoltage, "supplyVoltage");
                this.referenceVoltage = referenceVoltage;
                this.minimumVoltage = minimumVoltage;
                this.maximumVoltageScale = maximumScale;
                ScalarRegulators.voltageCompensated(
                        customRegulator, supplyVoltage, referenceVoltage,
                        minimumVoltage, maximumScale);
            }
            voltageAnswered = true;
            return this;
        }

        @Override
        public TargetStep<PositionPlant> outputPowerLimitedTo(double maximumMagnitude) {
            requireOutputPolicyUnanswered();
            requireSymmetricOutputMagnitude(maximumMagnitude);
            if (powerLimitedPositionOut != null) {
                maximumDeviceOutputPower = maximumMagnitude;
            } else {
                minimumOutputPower = -maximumMagnitude;
                maximumOutputPower = maximumMagnitude;
                if (standardControl != null) {
                    standardControl.outputPowerLimitedTo(maximumMagnitude);
                }
            }
            outputPolicyAnswered = true;
            return this;
        }

        @Override
        public TargetStep<PositionPlant> outputPowerLimitedTo(
                double minimum,
                double maximum) {
            requireOutputPolicyUnanswered();
            if (regulatedPowerOut == null) {
                throw new IllegalStateException("The signed outputPowerLimitedTo(min,max) answer "
                        + "requires a Sushi-regulated PowerOutput path");
            }
            requireOutputPowerRange(minimum, maximum);
            minimumOutputPower = minimum;
            maximumOutputPower = maximum;
            if (standardControl != null) {
                standardControl.outputPowerLimitedTo(minimum, maximum);
            }
            outputPolicyAnswered = true;
            return this;
        }

        @Override
        protected void validateConfiguration() {
            requireCoordinateConfigured();
            if (!toleranceAnswered) {
                throw new IllegalStateException("Feedback position construction requires "
                        + "positionTolerance(...) in Plant units");
            }
            if (referenceMode == MappedPositionPlant.ReferenceMode.STATIC) {
                MappedPositionPlant.requireFiniteBoundedMap(
                        range,
                        nativePerPlantUnit,
                        plantReference,
                        nativeReference,
                        "Plants feedback-position configuration");
            }
            if (regulatedPowerOut != null && !controlAnswered) {
                throw new IllegalStateException("Regulated position construction requires "
                        + "setpointFromAppliedTarget(), "
                        + "setpointFromTrapezoidalProfile(...), or "
                        + "controlFromCustomRegulator(...)");
            }
            if (standardControl != null && !feedbackAnswered) {
                throw new IllegalStateException("Standard regulated position construction requires "
                        + "feedbackFromPid(...)");
            }
            if (profiledSetpoint && hasTargetRateConfigured()) {
                throw new IllegalStateException("A profiled position controller cannot also use "
                        + "maxTargetRate(...) or maxTargetRates(...); choose one motion-shaping owner");
            }
        }

        @Override
        protected PositionPlant buildPlant(
                PlantTargetResolver resolver,
                PlantTargetGuards guards) {
            PositionOutput selectedPositionOut = configuredDevicePositionOutput();
            MappedPositionPlant.FeedbackConfigurationStep configured = selectedPositionOut != null
                    ? MappedPositionPlant.positionOutput(selectedPositionOut, nativeMeasurement)
                    : MappedPositionPlant.regulated(
                            regulatedPowerOut, nativeMeasurement, resolvedRegulator());
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

        private PositionToleranceStep<NEXT> answerFeedbackStaticReference(
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

        private PositionOutput configuredDevicePositionOutput() {
            if (positionOut == null) return null;
            if (powerLimitedPositionOut == null) return positionOut;
            final double selectedMaximum = maximumDeviceOutputPower;
            return new PositionOutput() {
                @Override
                public void setPosition(double position) {
                    powerLimitedPositionOut.setPosition(position, selectedMaximum);
                }

                @Override
                public double getCommandedPosition() {
                    return powerLimitedPositionOut.getCommandedPosition();
                }

                @Override
                public void stop() {
                    powerLimitedPositionOut.stop();
                }
            };
        }

        private ScalarRegulator resolvedRegulator() {
            if (standardControl != null) return standardControl.build();
            ScalarRegulator resolved = customRegulator;
            if (voltageAnswered) {
                resolved = ScalarRegulators.voltageCompensated(
                        resolved, supplyVoltage, referenceVoltage,
                        minimumVoltage, maximumVoltageScale);
            }
            if (outputPolicyAnswered) {
                resolved = ScalarRegulators.outputLimited(
                        resolved, minimumOutputPower, maximumOutputPower);
            }
            return resolved;
        }

        private void requireRegulatedControlPending(String operation) {
            requireConfigurationMutable(operation);
            if (!toleranceAnswered) {
                throw new IllegalStateException(operation + " requires positionTolerance(...) first");
            }
            if (regulatedPowerOut == null) {
                throw new IllegalStateException(operation
                        + " is available only for a framework-regulated position Plant");
            }
            if (controlAnswered) {
                throw new IllegalStateException("Position control has already been answered");
            }
        }

        private void requireStandardFeedbackPending(String operation) {
            requireConfigurationMutable(operation);
            if (standardControl == null || !controlAnswered || feedbackAnswered) {
                throw new IllegalStateException(operation
                        + " requires one unanswered standard position setpoint selection");
            }
        }

        private void requireStandardFeedbackAnswered(String operation) {
            requireConfigurationMutable(operation);
            if (standardControl == null || !feedbackAnswered) {
                throw new IllegalStateException(operation + " requires feedbackFromPid(...) first");
            }
            if (feedforwardAnswered || voltageAnswered || outputPolicyAnswered) {
                throw new IllegalStateException(operation + " cannot change PID/feedforward after "
                        + "the control recipe advanced to feedforward, voltage, or output policy");
            }
        }

        private void requireProfiledPosition(String operation) {
            requireStandardFeedbackAnswered(operation);
            if (!profiledSetpoint) {
                throw new IllegalStateException(operation
                        + " requires setpointFromTrapezoidalProfile(...)");
            }
        }

        private void requireOutputPolicyAvailable(String operation) {
            requireConfigurationMutable(operation);
            if (regulatedPowerOut != null) {
                if (!controlAnswered || (standardControl != null && !feedbackAnswered)) {
                    throw new IllegalStateException(operation
                            + " requires completed standard feedback or "
                            + "controlFromCustomRegulator(...)");
                }
                return;
            }
            if (powerLimitedPositionOut == null) {
                throw new IllegalStateException(operation
                        + " is unsupported by a plain PositionOutput");
            }
        }

        private void requireOutputPolicyUnanswered() {
            requireOutputPolicyAvailable("outputPowerLimitedTo(...)");
            if (outputPolicyAnswered) {
                throw new IllegalStateException(
                        "outputPowerLimitedTo(...) has already been answered");
            }
        }
    }

    private static void requireScale(double scale) {
        if (!Double.isFinite(scale) || scale == 0.0) {
            throw new IllegalArgumentException("native units per Plant unit must be finite and non-zero, got "
                    + scale);
        }
    }

    private static void requireTolerance(double tolerance, String name) {
        if (!Double.isFinite(tolerance) || tolerance < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and >= 0, got " + tolerance);
        }
    }

    private static void requireSymmetricOutputMagnitude(double maximumMagnitude) {
        if (!Double.isFinite(maximumMagnitude)
                || maximumMagnitude < 0.0
                || maximumMagnitude > 1.0) {
            throw new IllegalArgumentException("outputPowerLimitedTo(maximumMagnitude) requires a "
                    + "finite value in [0.0, 1.0], got " + maximumMagnitude);
        }
    }

    private static void requireOutputPowerRange(double minimum, double maximum) {
        if (!Double.isFinite(minimum) || !Double.isFinite(maximum)
                || minimum < -1.0 || maximum > 1.0
                || minimum > maximum || minimum > 0.0 || maximum < 0.0) {
            throw new IllegalArgumentException("outputPowerLimitedTo(minimum, maximum) requires "
                    + "finite -1 <= minimum <= 0 <= maximum <= 1; got minimum="
                    + minimum + ", maximum=" + maximum);
        }
    }

    private static String guardDisplayName(String name) {
        return name == null || name.trim().isEmpty() ? "interlock" : name.trim();
    }

    private abstract static class AbstractSourceDrivenPlant implements Plant {
        private final PlantTargetResolver targetResolver;
        private final ScalarTarget commandTarget;
        private final PlantTargetGuards guards;
        private final PlantLifecycle lifecycle = new PlantLifecycle();
        private final PlantUpdateCycle updateCycle = new PlantUpdateCycle("PowerPlant");

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
            if (!lifecycle.isActive()) return;
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
            prepareTargetContext(clock);
            if (!lifecycle.isActive()) return;
            PlantTargetContext context = targetContext(clock);
            if (!lifecycle.isActive()) return;
            PlantTargetResolution nextTargetResolution = targetResolver.resolve(context, clock);
            if (!lifecycle.isActive()) return;
            double nextRequestedTarget;
            if (nextTargetResolution != null && nextTargetResolution.hasTarget()) {
                nextRequestedTarget = nextTargetResolution.target();
            } else {
                nextRequestedTarget = appliedTarget;
            }

            double candidate = sanitizeRequestedTarget(nextRequestedTarget);
            PlantTargetStatus status;
            if (nextTargetResolution == null || !nextTargetResolution.hasTarget()) {
                status = PlantTargetStatus.targetUnavailable(nextTargetResolution != null
                        ? nextTargetResolution.reason()
                        : "missing plant target resolution");
                candidate = appliedTarget;
            } else {
                status = candidate == nextRequestedTarget
                        ? PlantTargetStatus.ACCEPTED
                        : PlantTargetStatus.clampedToRange("target sanitized to finite value");
            }

            PlantTargetGuards.Result result = PlantTargetSafety.applyGuards(
                    guards, candidate, status, appliedTarget, context.targetRange(), "Plant", clock);
            if (!lifecycle.isActive()) return;
            requestedTarget = nextRequestedTarget;
            appliedTarget = result.target;
            targetStatus = result.status;
            targetResolution = nextTargetResolution;
            double effectAppliedTarget = appliedTarget;
            PlantTargetStatus effectTargetStatus = targetStatus;
            PlantTargetResolution effectTargetResolution = targetResolution;
            RuntimeException effectFailure = null;
            try {
                applyTarget(appliedTarget, clock);
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
                if (effectFailure != null) throw effectFailure;
                return;
            }
            if (effectFailure != null) {
                failStopAfterUpdateFailure(
                        priorAppliedTarget,
                        priorTargetStatus,
                        priorTargetResolution,
                        effectAppliedTarget,
                        effectTargetStatus,
                        effectTargetResolution,
                        effectFailure);
                throw effectFailure;
            }

            try {
                updateStatus(clock);
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
                if (effectFailure != null) throw effectFailure;
                return;
            }
            if (effectFailure != null) {
                failStopAfterUpdateFailure(
                        priorAppliedTarget,
                        priorTargetStatus,
                        priorTargetResolution,
                        effectAppliedTarget,
                        effectTargetStatus,
                        effectTargetResolution,
                        effectFailure);
                throw effectFailure;
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

        protected final void markStopped(double appliedAfterStop) {
            appliedTarget = appliedAfterStop;
            targetStatus = PlantTargetStatus.STOPPED;
            targetResolution = PlantTargetResolution.unavailable("plant stopped");
        }

        protected final void restoreTargetState(double priorAppliedTarget,
                                                PlantTargetStatus priorTargetStatus,
                                                PlantTargetResolution priorTargetResolution) {
            appliedTarget = priorAppliedTarget;
            targetStatus = priorTargetStatus;
            targetResolution = priorTargetResolution;
        }

        /** Apply this realization's natural output stop without changing Plant lifecycle state. */
        protected abstract void stopOutput();

        /** Applied-target fact established by a successful natural output stop. */
        protected abstract double appliedTargetAfterStop();

        @Override
        public final void stop() {
            if (!lifecycle.claimStop()) return;

            double priorAppliedTarget = appliedTarget;
            PlantTargetStatus priorTargetStatus = targetStatus;
            PlantTargetResolution priorTargetResolution = targetResolution;
            RuntimeException primary = null;
            boolean outputStopSucceeded = false;
            try {
                stopOutput();
                outputStopSucceeded = true;
            } catch (RuntimeException failure) {
                primary = failure;
            }

            if (outputStopSucceeded) {
                markStopped(appliedTargetAfterStop());
            } else {
                restoreTargetState(priorAppliedTarget, priorTargetStatus, priorTargetResolution);
            }
            if (primary != null) throw primary;
        }

        private void enforceTerminalStopAfterReentrantEffect(
                double effectAppliedTarget,
                PlantTargetStatus effectTargetStatus,
                PlantTargetResolution effectTargetResolution) {
            try {
                stopOutput();
                markStopped(appliedTargetAfterStop());
            } catch (RuntimeException failure) {
                restoreTargetState(effectAppliedTarget, effectTargetStatus, effectTargetResolution);
                throw failure;
            }
        }

        private void failStopAfterUpdateFailure(
                double priorAppliedTarget,
                PlantTargetStatus priorTargetStatus,
                PlantTargetResolution priorTargetResolution,
                double effectAppliedTarget,
                PlantTargetStatus effectTargetStatus,
                PlantTargetResolution effectTargetResolution,
                RuntimeException failure) {
            boolean outputStopSucceeded = false;
            try {
                stopOutput();
                outputStopSucceeded = true;
            } catch (RuntimeException cleanupFailure) {
                suppress(failure, cleanupFailure);
            }

            if (!lifecycle.isActive()) {
                if (outputStopSucceeded) {
                    markStopped(appliedTargetAfterStop());
                } else {
                    restoreTargetState(
                            effectAppliedTarget, effectTargetStatus, effectTargetResolution);
                }
                return;
            }

            boolean cleanupSucceeded = outputStopSucceeded;
            try {
                guards.reset();
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
                markStopped(appliedTargetAfterStop());
            } else {
                restoreTargetState(priorAppliedTarget, priorTargetStatus, priorTargetResolution);
            }
        }

        private static RuntimeException suppress(RuntimeException primary, RuntimeException additional) {
            if (primary == null) return additional;
            if (primary != additional) primary.addSuppressed(additional);
            return primary;
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
        protected void stopOutput() {
            out.stop();
        }

        @Override
        protected double appliedTargetAfterStop() {
            return 0.0;
        }
    }

}
