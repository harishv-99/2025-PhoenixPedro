package edu.ftcphoenix.fw.actuation.controller;

import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.debug.DebugSink;

/**
 * Controller that maps a continuous input value (e.g., sensor reading) into
 * a target for a single {@link Plant}.
 *
 * <p>Intended usage:</p>
 *
 * <pre>{@code
 * // Example: AprilTag distance (meters) -> shooter velocity (rad/s).
 * DoubleSupplier distanceMeters = () -> tagTracker.getBestTargetDistance();
 *
 * // Lookup/interpolation (user-provided):
 * DoubleUnaryOperator distanceToRadPerSec = dMeters -> shooterTable.lookup(dMeters);
 *
 * Plant shooterPlant = Actuators.plant(hw)
 *         .motor("shooter", Direction.FORWARD)
 *         .velocity()
 *         .build();
 *
 * FunctionController shooterAutoVel =
 *     new FunctionController(shooterPlant, distanceMeters, distanceToRadPerSec);
 *
 * // In your loop (AUTO_TAG mode active):
 * shooterAutoVel.update(dtSec);
 * }</pre>
 *
 * <p>Semantics:</p>
 * <ul>
 *   <li>On each {@link #update(double)}:
 *     <ol>
 *       <li>Read current input value from a {@link DoubleSupplier}.</li>
 *       <li>Compute a target using a {@link DoubleUnaryOperator} mapping.</li>
 *       <li>Call {@link Plant#setTarget(double)} with that target.</li>
 *       <li>Call {@link Plant#update(double)} once.</li>
 *     </ol>
 *   </li>
 * </ul>
 *
 * <p><b>Important:</b> When using this controller, treat it as the single
 * owner of {@link Plant#update(double)} for the wrapped plant. External code
 * should not also call {@code plant.update(dt)} directly.</p>
 */
public final class FunctionController {

    private final Plant plant;
    private final DoubleSupplier input;
    private final DoubleUnaryOperator mapping;

    // Debug: last sampled input and computed target (set during update()).
    private double lastInput = Double.NaN;
    private double lastTarget = Double.NaN;

    /**
     * Construct a {@link FunctionController}.
     *
     * @param plant   plant to drive (non-null)
     * @param input   supplier for the current input value (non-null)
     * @param mapping mapping from input value to plant target (non-null)
     */
    public FunctionController(Plant plant,
                              DoubleSupplier input,
                              DoubleUnaryOperator mapping) {
        this.plant = Objects.requireNonNull(plant, "plant is required");
        this.input = Objects.requireNonNull(input, "input is required");
        this.mapping = Objects.requireNonNull(mapping, "mapping is required");
    }

    /**
     * Convenience factory: create a controller with a constant offset applied
     * to the mapping's result.
     *
     * <p>Useful if you want a baseline + lookup:</p>
     *
     * <pre>{@code
     * FunctionController ctrl =
     *     FunctionController.withOffset(
     *         shooterPlant,
     *         distanceMeters,
     *         distanceToRadPerSec,
     *         BASELINE_RAD_PER_SEC
     *     );
     * }</pre>
     *
     * @param plant       plant to drive
     * @param input       supplier for the current input value
     * @param baseMapping core mapping from input to target
     * @param offset      constant added to the mapping's output
     */
    public static FunctionController withOffset(
            Plant plant,
            DoubleSupplier input,
            DoubleUnaryOperator baseMapping,
            double offset
    ) {
        Objects.requireNonNull(baseMapping, "baseMapping is required");
        DoubleUnaryOperator combined = x -> baseMapping.applyAsDouble(x) + offset;
        return new FunctionController(plant, input, combined);
    }

    /**
     * Advance the controller and the wrapped plant by {@code dtSec}.
     *
     * <p>Steps:</p>
     * <ol>
     *   <li>Read {@code x = input.getAsDouble()}.</li>
     *   <li>Compute {@code target = mapping.applyAsDouble(x)}.</li>
     *   <li>Call {@link Plant#setTarget(double)} with {@code target}.</li>
     *   <li>Call {@link Plant#update(double)} with {@code dtSec}.</li>
     * </ol>
     *
     * @param dtSec time step in seconds
     */
    public void update(double dtSec) {
        double x = input.getAsDouble();
        double target = mapping.applyAsDouble(x);
        lastInput = x;
        lastTarget = target;
        plant.setTarget(target);
        plant.update(dtSec);
    }

    /**
     * Reset the underlying plant.
     *
     * <p>Note: this does not reset any state in the input or mapping; those
     * are assumed to be stateless or managed elsewhere.</p>
     */
    public void reset() {
        plant.reset();
        lastInput = Double.NaN;
        lastTarget = Double.NaN;
    }

    /**
     * @return underlying plant (for advanced / introspection use only).
     */
    public Plant getPlant() {
        return plant;
    }

    /**
     * @return the current input value (as seen by the next update).
     */
    public double peekInput() {
        return input.getAsDouble();
    }


    /**
     * Debug helper: emit the last sampled input/target and delegate to the wrapped plant.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "functionCtrl" : prefix;

        dbg.addLine(p)
                .addData(p + ".lastInput", lastInput)
                .addData(p + ".lastTarget", lastTarget);

        plant.debugDump(dbg, p + ".plant");
    }

}
