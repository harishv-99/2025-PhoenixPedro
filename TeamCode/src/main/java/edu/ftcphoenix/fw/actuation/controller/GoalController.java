package edu.ftcphoenix.fw.actuation.controller;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;

/**
 * Simple "mode → setpoint" controller on top of a single {@link Plant}.
 *
 * <p>The intent is to make code like:</p>
 *
 * <pre>{@code
 * enum ShooterMode { OFF, LOW, HIGH }
 *
 * GoalController<ShooterMode> shooterCtrl =
 *     GoalController.enumBuilder(ShooterMode.class)
 *         .plant(shooterPlant)
 *         .target(ShooterMode.OFF,  0.0)
 *         .target(ShooterMode.LOW, 30.0)
 *         .target(ShooterMode.HIGH,45.0)
 *         .defaultGoal(ShooterMode.OFF)
 *         .build();
 *
 * shooterCtrl.setGoal(ShooterMode.HIGH);
 *
 * // In your loop:
 * shooterCtrl.update(dtSec); // delegates to shooterPlant.update(dtSec)
 * }</pre>
 *
 * <p>Design points:</p>
 * <ul>
 *   <li>All "what does each mode mean?" mappings live here, not scattered
 *       through robot code.</li>
 *   <li>Only this controller calls {@link Plant#setTarget(double)} for the
 *       wrapped plant; external code should prefer {@link #setGoal(Object)}.</li>
 *   <li>{@link #update(double)} simply calls {@link Plant#update(double)} so
 *       the controller can be treated like any other time-stepped object.</li>
 * </ul>
 *
 * @param <G> goal type (usually an enum)
 */
public final class GoalController<G> {

    private final Plant plant;
    private final Map<G, Double> targetsByGoal;
    private final G defaultGoal;

    private G currentGoal;

    private GoalController(Plant plant,
                           Map<G, Double> targetsByGoal,
                           G defaultGoal) {
        this.plant = plant;
        this.targetsByGoal = targetsByGoal;
        this.defaultGoal = defaultGoal;
        this.currentGoal = defaultGoal;

        // Apply the default goal immediately.
        Double target = targetsByGoal.get(defaultGoal);
        if (target != null) {
            plant.setTarget(target);
        }
    }

    /**
     * Set the current goal. If this goal has a target mapping, the underlying
     * {@link Plant} is immediately updated via {@link Plant#setTarget(double)}.
     *
     * @param goal new goal to activate
     */
    public void setGoal(G goal) {
        Objects.requireNonNull(goal, "goal is required");
        this.currentGoal = goal;

        Double target = targetsByGoal.get(goal);
        if (target != null) {
            plant.setTarget(target);
        }
    }

    /**
     * @return the current goal (never {@code null} after construction)
     */
    public G getGoal() {
        return currentGoal;
    }

    /**
     * @return underlying plant (for advanced use only; most code should not
     * call {@link Plant#setTarget(double)} directly).
     */
    public Plant getPlant() {
        return plant;
    }

    /**
     * Advance the underlying plant by {@code dtSec}.
     *
     * <p>This does not change the goal or target; it simply forwards the
     * time-step to {@link Plant#update(double)}.</p>
     *
     * @param dtSec time step in seconds
     */
    public void update(double dtSec) {
        plant.update(dtSec);
    }

    /**
     * Reset the underlying plant and re-apply the current goal's target (if any).
     */
    public void reset() {
        plant.reset();
        Double target = targetsByGoal.get(currentGoal);
        if (target != null) {
            plant.setTarget(target);
        }
    }

    /**
     * @return true if the underlying plant reports that it is at its setpoint
     * for the current goal.
     */
    public boolean atSetpoint() {
        return plant.atSetpoint();
    }

    // ------------------------------------------------------------------------
    // Builder API
    // ------------------------------------------------------------------------

    /**
     * Create a builder for a {@link GoalController} with arbitrary key type.
     *
     * <p>For enums, prefer {@link #enumBuilder(Class)} which uses an
     * {@link EnumMap} internally.</p>
     */
    public static <G> Builder<G> builder() {
        return new Builder<>(new HashMap<>());
    }

    /**
     * Create a builder specialized for enum goal types.
     *
     * <p>This is the most common usage:</p>
     *
     * <pre>{@code
     * GoalController<ShooterMode> shooterCtrl =
     *     GoalController.enumBuilder(ShooterMode.class)
     *         .plant(shooterPlant)
     *         .target(ShooterMode.OFF,  0.0)
     *         .target(ShooterMode.LOW, 30.0)
     *         .target(ShooterMode.HIGH,45.0)
     *         .defaultGoal(ShooterMode.OFF)
     *         .build();
     * }</pre>
     *
     * @param enumType enum class token (e.g. ShooterMode.class)
     */
    public static <E extends Enum<E>> Builder<E> enumBuilder(Class<E> enumType) {
        Objects.requireNonNull(enumType, "enumType is required");
        return new Builder<>(new EnumMap<>(enumType));
    }

    /**
     * Builder for {@link GoalController}.
     */
    public static final class Builder<G> {
        private final Map<G, Double> targetsByGoal;
        private Plant plant;
        private G defaultGoal;

        private Builder(Map<G, Double> backing) {
            this.targetsByGoal = backing;
        }

        /**
         * Set the {@link Plant} this controller will own.
         */
        public Builder<G> plant(Plant plant) {
            this.plant = Objects.requireNonNull(plant, "plant is required");
            return this;
        }

        /**
         * Associate a goal value with a numeric target in the underlying plant's
         * units (power, rad/s, radians, etc.).
         */
        public Builder<G> target(G goal, double target) {
            Objects.requireNonNull(goal, "goal is required");
            targetsByGoal.put(goal, target);
            return this;
        }

        /**
         * Specify which goal should be active by default upon construction.
         *
         * <p>It is recommended that the default goal have a target mapping
         * defined via {@link #target(Object, double)} before {@link #build()} is
         * called; otherwise, the underlying plant will not receive an initial
         * target.</p>
         */
        public Builder<G> defaultGoal(G goal) {
            this.defaultGoal = Objects.requireNonNull(goal, "defaultGoal is required");
            return this;
        }

        /**
         * Construct the controller.
         *
         * @throws IllegalStateException if the plant or default goal are missing
         */
        public GoalController<G> build() {
            if (plant == null) {
                throw new IllegalStateException("plant must be set before build()");
            }
            if (defaultGoal == null) {
                throw new IllegalStateException("defaultGoal must be set before build()");
            }
            return new GoalController<>(plant, targetsByGoal, defaultGoal);
        }
    }
}
