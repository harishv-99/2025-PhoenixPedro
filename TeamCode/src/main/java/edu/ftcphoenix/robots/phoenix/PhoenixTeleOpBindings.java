package edu.ftcphoenix.robots.phoenix;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * TeleOp-only button semantics for Phoenix.
 *
 * <p>This class owns toggle/hold mappings so the scoring supervisor can expose an intent-shaped API
 * instead of a button-shaped one.</p>
 */
public final class PhoenixTeleOpBindings {

    private final Bindings bindings = new Bindings();

    /**
     * Creates the Phoenix TeleOp binding map.
     *
     * <p>The binding policy is intentionally kept outside {@link ShooterSupervisor} so higher-level
     * logic can be reused by Auto or alternate driver-control schemes without carrying button-edge
     * semantics with it.</p>
     *
     * @param gamepads                  wrapped gamepad sources used to bind driver inputs
     * @param shooter                   shooter subsystem used for direct selected-velocity adjustments
     * @param scoring                   scoring supervisor that receives intent-level actions
     * @param captureVelocityFromTarget action invoked when the driver requests a fresh range-based
     *                                  velocity suggestion from the current target observation
     */
    public PhoenixTeleOpBindings(Gamepads gamepads,
                                 Shooter shooter,
                                 ShooterSupervisor scoring,
                                 Runnable captureVelocityFromTarget) {
        Objects.requireNonNull(gamepads, "gamepads");
        Objects.requireNonNull(shooter, "shooter");
        Objects.requireNonNull(scoring, "scoring");
        Objects.requireNonNull(captureVelocityFromTarget, "captureVelocityFromTarget");

        bindings.onToggle(gamepads.p2().a(), scoring::setIntakeEnabled);
        bindings.onToggle(gamepads.p2().rightBumper(), scoring::setFlywheelEnabled);

        bindings.onRise(gamepads.p2().leftBumper(), captureVelocityFromTarget);

        bindings.onRiseAndFall(
                gamepads.p2().b(),
                scoring::beginShooting,
                scoring::endShooting
        );

        bindings.onRiseAndFall(
                gamepads.p2().x(),
                scoring::beginEject,
                scoring::endEject
        );

        bindings.onRise(gamepads.p2().dpadUp(), shooter::increaseSelectedVelocity);
        bindings.onRise(gamepads.p2().dpadDown(), shooter::decreaseSelectedVelocity);
    }

    /**
     * Advances the binding state machine for the current loop cycle.
     *
     * @param clock shared loop clock for the active OpMode cycle
     */
    public void update(LoopClock clock) {
        bindings.update(clock);
    }

    /**
     * Clears all remembered edge/toggle state.
     *
     * <p>Call this when TeleOp is shutting down or when the binding owner is being discarded so the
     * next activation starts from a clean slate.</p>
     */
    public void clear() {
        bindings.clear();
    }
}
