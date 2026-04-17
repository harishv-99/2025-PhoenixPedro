package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;

/**
 * Helper adapters that expose common {@link Plant} state as Phoenix sources.
 *
 * <p>These adapters are useful when higher-level logic wants to compose plant status into source
 * pipelines, debouncers, or tasks without reaching into the plant implementation directly.</p>
 */
public final class PlantSources {

    private PlantSources() {
        // utility class
    }

    /**
     * Create a scalar source view of {@link Plant#getTarget()}.
     *
     * @param plant plant whose target should be exposed as a source
     * @return scalar source that reports the plant's current target value
     */
    public static ScalarSource target(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return ScalarSource.of(plant::getTarget);
    }

    /**
     * Create a scalar source view of {@link Plant#getMeasurement()}.
     *
     * @param plant plant whose authoritative measurement should be exposed as a source
     * @return scalar source that reports the plant's last cached measurement value
     */
    public static ScalarSource measurement(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return ScalarSource.of(plant::getMeasurement);
    }

    /**
     * Create a scalar source view of {@link Plant#getError()}.
     *
     * @param plant plant whose target-minus-measurement error should be exposed as a source
     * @return scalar source that reports the plant's last cached error value
     */
    public static ScalarSource error(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return ScalarSource.of(plant::getError);
    }

    /**
     * Create a boolean source view of {@link Plant#atSetpoint()}.
     *
     * @param plant plant whose at-setpoint status should be exposed as a source
     * @return boolean source that reports whether the plant was at setpoint on its last update
     */
    public static BooleanSource atSetpoint(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return BooleanSource.of(plant::atSetpoint);
    }

    /**
     * Create a boolean source view of {@link Plant#hasFeedback()}.
     *
     * @param plant plant whose feedback capability should be exposed as a source
     * @return boolean source that reports whether the plant exposes meaningful feedback
     */
    public static BooleanSource hasFeedback(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return BooleanSource.of(plant::hasFeedback);
    }
}
