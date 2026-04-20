package edu.ftcphoenix.fw.actuation;

import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A position-target {@link Plant} whose caller-facing coordinate is explicitly modeled.
 *
 * <p>A {@code PositionPlant} is still a normal plant: callers command it with
 * {@link #setTarget(double)}, update it once per loop, and read plant-level status through
 * {@link #getTarget()}, {@link #getMeasurement()}, and {@link #atSetpoint()}. The extra methods
 * expose the position-domain facts that planners and calibration tasks need:</p>
 *
 * <ul>
 *   <li>the legal target range in <b>plant units</b>,</li>
 *   <li>whether the coordinate is linear or periodic,</li>
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
 * ranges and setpoints are safe. Until the coordinate is referenced, {@link #targetRange()} should
 * return an invalid range and position setpoints should not be applied to hardware.</p>
 */
public interface PositionPlant extends Plant {

    /**
     * Shape of the caller-facing position coordinate.
     */
    enum Topology {
        /**
         * A non-wrapping position coordinate such as lift height, slide extension, or arm angle.
         */
        LINEAR,
        /**
         * A coordinate where positions separated by one period are equivalent.
         */
        PERIODIC
    }

    /**
     * Returns the coordinate topology for this position plant.
     */
    Topology topology();

    /**
     * Returns {@code true} when {@link #topology()} is {@link Topology#PERIODIC}.
     */
    default boolean isPeriodic() {
        return topology() == Topology.PERIODIC;
    }

    /**
     * Returns the period in plant units for periodic position plants.
     *
     * <p>Linear plants return {@link Double#NaN}. For a tray expressed in degrees, this might be
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
     * Returns a source view of {@link #targetRange()} for planners.
     */
    default Source<ScalarRange> targetRangeSource() {
        return clock -> targetRange();
    }

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
     * <p>The supplied value is in plant units. For a linear lift, {@code establishReferenceAt(0.0)}
     * means "the current hardware reading is the lift's zero point." For a periodic tray, it means
     * "the current hardware reading is equivalent to reference {@code 0.0} modulo the plant's
     * period." If a periodic plant is already referenced, implementations should preserve the
     * nearest equivalent unwrapped position instead of snapping the coordinate all the way back to
     * the base reference.</p>
     *
     * <p>Callers should prefer {@link #establishReferenceAt(double, LoopClock)} when they have a
     * loop clock available so the implementation can sample the same-loop native measurement.</p>
     *
     * @param plantPosition reference value in plant units
     */
    void establishReferenceAt(double plantPosition);

    /**
     * Establish a reference after sampling native feedback for the supplied loop.
     *
     * @param plantPosition reference value in plant units
     * @param clock         current loop clock used to sample native feedback consistently
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
     * Begin a calibration search by temporarily applying open-loop power.
     *
     * <p>This method is intended for {@link PositionCalibrationTasks}; normal robot code should
     * usually command a search through a task rather than calling this directly.</p>
     *
     * @param power normalized search power, usually in {@code [-1,+1]}
     */
    default void beginCalibrationSearch(double power) {
        throw new IllegalStateException("This PositionPlant does not support calibration search drive");
    }

    /**
     * End a calibration search and optionally stop the search output.
     *
     * @param stopOutput if true, stop the temporary search output before returning to normal control
     */
    default void endCalibrationSearch(boolean stopOutput) {
        // default: no search mode to exit
    }
}
