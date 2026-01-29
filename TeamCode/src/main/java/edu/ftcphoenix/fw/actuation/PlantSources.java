package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;

/**
 * {@link edu.ftcphoenix.fw.core.source.Source} adapters for {@link Plant}s.
 *
 * <p>Phoenix encourages a clean separation between:</p>
 *
 * <ul>
 *   <li><b>sources</b>: values you sample each loop (operator intent, sensors, derived signals)</li>
 *   <li><b>plants</b>: low-level sinks you command with a scalar target</li>
 * </ul>
 *
 * <p>Sometimes you want to treat a plant's state as a signal—for example, to build a debounced
 * “ready” gate from {@link Plant#atSetpoint()} or to log a plant’s target consistently. This class
 * provides small, obvious adapters for that purpose.</p>
 *
 * <h2>Design notes</h2>
 *
 * <ul>
 *   <li>{@link #atSetpoint(Plant)} returns a <em>memoized</em> boolean source by default so the
 *       plant is sampled at most once per {@code LoopClock.cycle()}.</li>
 *   <li>{@link #target(Plant)} is <em>not</em> memoized. Targets can legitimately change within
 *       a loop if multiple call sites are still in transition toward single-writer ownership.</li>
 * </ul>
 */
public final class PlantSources {

    private PlantSources() {
        // utility class
    }

    /**
     * A scalar source that returns {@link Plant#getTarget()}.
     */
    public static ScalarSource target(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return ScalarSource.of(plant::getTarget);
    }

    /**
     * A boolean source that returns {@link Plant#atSetpoint()}.
     *
     * <p>The returned source is memoized per loop cycle.</p>
     */
    public static BooleanSource atSetpoint(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return BooleanSource.of(plant::atSetpoint).memoized();
    }

    /**
     * A constant boolean source that returns {@link Plant#hasFeedback()}.
     */
    public static BooleanSource hasFeedback(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return BooleanSource.constant(plant.hasFeedback());
    }
}
