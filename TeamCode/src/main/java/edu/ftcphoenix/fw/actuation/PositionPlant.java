package edu.ftcphoenix.fw.actuation;

import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A position-target {@link Plant} whose caller-facing coordinate is explicitly modeled.
 *
 * <p>A {@code PositionPlant} is still a normal source-driven plant: callers build it with a
 * target resolver, update it once per loop, and read plant-level status through
 * {@link #getRequestedTarget()}, {@link #getAppliedTarget()}, {@link #getMeasurement()}, and
 * {@link #atTarget()}. The extra methods expose the position-domain facts that planners and
 * calibration tasks need:</p>
 *
 * <ul>
 *   <li>the legal target range in <b>plant units</b>,</li>
 *   <li>whether the coordinate is non-periodic or periodic,</li>
 *   <li>whether a physical reference has been established, and</li>
 *   <li>how a homing/indexing task can establish that reference.</li>
 * </ul>
 *
 * <h2>Units convention</h2>
 *
 * <p>Methods on this interface use <b>plant units</b> unless the method name explicitly says
 * {@code native}. A lift might expose plant units as inches while its motor still uses encoder
 * ticks internally. A servo claw might expose plant units {@code 0..1} while the raw servo command
 * range is {@code 0.30..0.80}. The native conversion belongs below this interface.</p>
 *
 * <h2>Reference lifecycle</h2>
 *
 * <p>Some position coordinates are immediately meaningful: a standard servo command, an absolute
 * encoder already converted into degrees, or a simulator source. Others need homing/indexing before
 * ranges and targets are safe. Until the coordinate is referenced, {@link #targetRange()} should
 * return an invalid range and position targets should not be applied to hardware.</p>
 */
public interface PositionPlant extends Plant {

    /**
     * Periodicity of the caller-facing position coordinate.
     */
    enum Periodicity {
        /**
         * A position coordinate with no declared fixed equivalence period, such as a limited-travel
         * lift, slide, or arm.
         */
        NON_PERIODIC,
        /**
         * A coordinate where positions separated by one period can be treated as equivalent.
         *
         * <p>Periodicity does not wrap targets automatically. Use
         * {@link PlantTargets#equivalentPositionsOf(edu.ftcphoenix.fw.core.source.ScalarTarget)}
         * when a logical command may choose any legal representative; an exact source still means
         * one literal unwrapped position.</p>
         */
        PERIODIC
    }

    /**
     * Returns the periodicity for this position plant.
     */
    Periodicity periodicity();

    /**
     * Returns the period in plant units for periodic position plants.
     *
     * <p>Non-periodic plants return {@link Double#NaN}. For a tray expressed in degrees, this might be
     * {@code 360.0}. For a tray expressed in encoder ticks, this might be ticks per revolution.</p>
     */
    double period();

    /**
     * Returns the current legal target range in plant units.
     *
     * <p>Before homing or reference establishment, implementations should return
     * {@link ScalarRange#invalid(String)} with an actionable reason. Planners should treat an
     * invalid range as a hard block.</p>
     */
    ScalarRange targetRange();

    /**
     * Returns a source view of the plant measurement in plant units.
     *
     * <p>If the plant has no authoritative feedback, the source returns {@link Double#NaN} just like
     * {@link #getMeasurement()}.</p>
     */
    default ScalarSource positionSource() {
        return clock -> getMeasurement();
    }

    /**
     * Returns {@code true} when the plant coordinate has a valid physical reference.
     */
    boolean isReferenced();

    /**
     * Human-readable reference status, suitable for telemetry and debug output.
     */
    String referenceStatus();

    /**
     * Establish a reference using the latest sampled native position.
     *
     * <p>The supplied value is in plant units. For a non-periodic lift, {@code establishReferenceAt(0.0)}
     * means "the current hardware reading is the lift's zero point." For a periodic tray, it means
     * "the current hardware reading is equivalent to reference {@code 0.0} modulo the plant's
     * period." If a periodic plant is already referenced, implementations should preserve the
     * nearest equivalent unwrapped position instead of snapping the coordinate all the way back to
     * the base reference.</p>
     *
     * <p>The reference is a finite coordinate anchor, not a target command, so it need not lie
     * inside {@link #targetRange()}. An implementation must reject a non-finite value before
     * sampling feedback, invoking a callback, or changing reference, target, controller, search,
     * or output state. It must not clamp the supplied reference.</p>
     *
     * <p>Callers should prefer {@link #establishReferenceAt(double, LoopClock)} when they have a
     * loop clock available so the implementation can sample the current loop's native measurement.
     * A periodic implementation selects its nearest unwrapped equivalence from that sample rather
     * than a cached measurement from an earlier loop.</p>
     *
     * @param plantPosition finite reference value in plant units
     * @throws IllegalArgumentException if {@code plantPosition} is non-finite
     */
    void establishReferenceAt(double plantPosition);

    /**
     * Establish a reference from native feedback sampled for the supplied loop.
     *
     * <p>For an already referenced periodic Plant, choose the nearest unwrapped equivalence from
     * this same-loop sample. The finite-input, no-clamp, and pre-effect rejection contract of
     * {@link #establishReferenceAt(double)} also applies to this overload.</p>
     *
     * @param plantPosition finite reference value in plant units
     * @param clock         current loop clock used to sample native feedback consistently
     * @throws IllegalArgumentException if {@code plantPosition} is non-finite
     */
    default void establishReferenceAt(double plantPosition, LoopClock clock) {
        establishReferenceAt(plantPosition);
    }

    /**
     * Returns {@code true} when this plant can be searched by temporarily applying open-loop power.
     *
     * <p>Motor-backed and CR-servo-backed position plants commonly support search drive. Standard
     * servos usually do not.</p>
     */
    default boolean supportsCalibrationSearch() {
        return false;
    }

    /**
     * Acquire a calibration search, immediately stop the previous normal output, and stage
     * temporary open-loop power for the next owner update.
     *
     * <p>This method is intended for {@link PositionCalibrationTasks}; normal robot code should
     * usually command a search through a task rather than calling this directly. It does not make
     * the Task a Plant heartbeat owner: the mechanism or subsystem must continue calling
     * {@link #update(LoopClock)} once in its normal downstream Plant phase.</p>
     *
     * <p>No nonzero search command is submitted until that owner update. A normal return means this
     * caller exclusively acquired the temporary search. A second search must fail before stopping,
     * replacing, or otherwise disturbing the first. If acquisition throws, the implementation must
     * leave no newly acquired search for the caller to release.</p>
     *
     * <p>When search is supported and idle, an implementation must reject non-finite power or a
     * value outside the inclusive normalized {@code [-1.0, +1.0]} range before stopping normal
     * output, invoking an external callback, sampling feedback, resetting a controller, acquiring
     * or changing search/target status, or writing hardware. It must not silently clamp this recipe
     * answer. Unsupported capability and an existing active owner may be reported first because no
     * power value can make those calls eligible. The framework-valid range does not choose a
     * mechanically safe magnitude or direction for a particular robot.</p>
     *
     * @param power finite normalized search power in {@code [-1.0, +1.0]}
     * @throws IllegalArgumentException if an otherwise eligible search receives non-finite or
     *                                  out-of-range power
     * @throws IllegalStateException if calibration search is unsupported or already active
     */
    default void beginCalibrationSearch(double power) {
        throw new IllegalStateException("This PositionPlant does not support calibration search drive");
    }

    /**
     * Release an acquired calibration search and request an immediate stop of its temporary output.
     *
     * <p>Normal target resolution resumes on the next owner {@link #update(LoopClock)}. This
     * active-only operation is safe to repeat after release; it does not disable the Plant or
     * change its persistent command/target graph. Search ownership must be cleared before an
     * external stop callback, so a later owner update cannot refresh search power if that callback
     * throws. A throwing stop leaves the physical output state unknown and propagates that failure;
     * it must not be described as a confirmed physical stop.</p>
     */
    default void endCalibrationSearch() {
        // default: no search mode to exit
    }
}
