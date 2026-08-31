package edu.ftcsushi.fw.actuation;

import java.util.Objects;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;

/**
 * Source views for plant status.
 */
public final class PlantSources {
    private PlantSources() {
    }

    /**
     * Boolean source view of {@link Plant#atTarget()}.
     */
    public static BooleanSource atTarget(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return clock -> plant.atTarget();
    }

    /**
     * Boolean source view of {@link Plant#atTarget(double)} for one literal physical target value.
     * This does not apply periodic-equivalence semantics.
     */
    public static BooleanSource atTarget(Plant plant, double target) {
        Objects.requireNonNull(plant, "plant");
        return clock -> plant.atTarget(target);
    }

    /**
     * Source view of {@link Plant#getRequestedTarget()}.
     */
    public static ScalarSource requestedTarget(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return clock -> plant.getRequestedTarget();
    }

    /**
     * Source view of {@link Plant#getAppliedTarget()}.
     */
    public static ScalarSource appliedTarget(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return clock -> plant.getAppliedTarget();
    }

    /**
     * Source view of {@link Plant#getMeasurement()}.
     */
    public static ScalarSource measurement(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return clock -> plant.getMeasurement();
    }

    /**
     * Source view of requested-target error, {@link Plant#getRequestedTargetError()}.
     */
    public static ScalarSource requestedTargetError(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return clock -> plant.getRequestedTargetError();
    }

    /**
     * Source view of applied-target error, {@link Plant#getAppliedTargetError()}.
     */
    public static ScalarSource appliedTargetError(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return clock -> plant.getAppliedTargetError();
    }

    /**
     * Boolean source that is high when the plant has feedback.
     */
    public static BooleanSource hasFeedback(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        return clock -> plant.hasFeedback();
    }
}
