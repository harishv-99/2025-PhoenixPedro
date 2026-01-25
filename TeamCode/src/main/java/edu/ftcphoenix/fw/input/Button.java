package edu.ftcphoenix.fw.input;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Logical button with built-in edge detection and a simple global registry.
 *
 * <p>This interface is designed for use in <b>polled</b> input code where the
 * framework calls {@link #updateAllRegistered()} once per loop (for example,
 * from {@code Gamepads.update(dtSec)}), and robot code then queries buttons
 * using:</p>
 *
 * <pre>{@code
 * // TeleOp loop example:
 * pads.update(dtSec);  // internally calls Button.updateAllRegistered()
 *
 * if (pads.p1().a().onPress()) {
 *     // Runs only on the FIRST loop where A goes up -> down.
 * }
 *
 * if (pads.p1().a().onRelease()) {
 *     // Runs only on the FIRST loop where A goes down -> up.
 * }
 *
 * if (pads.p1().a().isHeld()) {
 *     // Runs EVERY loop while A is down.
 * }
 * }</pre>
 *
 * <h2>Lifecycle / update model</h2>
 *
 * <ul>
 *   <li>Each {@link Button} tracks its previous and current value internally.</li>
 *   <li>Buttons created via {@link #of(BooleanSupplier)} are automatically
 *       <b>registered</b> in a global list when constructed.</li>
 *   <li>The framework should call {@link #updateAllRegistered()} exactly once
 *       per loop to advance the state of all registered buttons.</li>
 *   <li>Robot code is expected to only call the query methods:
 *     <ul>
 *       <li>{@link #onPress()} – rising edge.</li>
 *       <li>{@link #onRelease()} – falling edge.</li>
 *       <li>{@link #isHeld()} – level.</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p>For gamepad-backed buttons, the typical wiring is:</p>
 *
 * <pre>{@code
 * class GamepadDevice {
 *     private final Gamepad gp;
 *
 *     public Button a() {
 *         // raw supplier reads FTC Gamepad field
 *         return Button.of(() -> gp.a);
 *     }
 * }
 *
 * class Gamepads {
 *     public void update(double dtSec) {
 *         Button.updateAllRegistered();
 *     }
 * }
 * }</pre>
 *
 * <p><b>Axis/combination note:</b> When converting a {@link Button} to an
 * axis (e.g., {@code Axis.fromButton(button)}), axis helpers should rely on
 * {@link #isHeld()} (the level view) rather than {@link #onPress()} or
 * {@link #onRelease()}.</p>
 */
public interface Button {

    /**
     * Advance this button's internal state by sampling the underlying raw value.
     *
     * <p>This should be called exactly once per loop <b>before</b> any calls to
     * {@link #onPress()}, {@link #onRelease()}, or {@link #isHeld()} for that
     * loop.</p>
     *
     * <p>Most robot code never calls this directly. Instead, the framework calls
     * {@link #updateAllRegistered()} once per loop (for example, from
     * {@code Gamepads.update(dtSec)}), which forwards to {@link #update()} on
     * all registered buttons.</p>
     */
    void update();

    /**
     * Rising edge: returns {@code true} <b>only on the first loop</b> where the
     * button transitions from "not held" to "held".
     *
     * <p>Use this for "trigger once per press" behavior.</p>
     *
     * <pre>{@code
     * if (button.onPress()) {
     *     // This body runs once per press.
     * }
     * }</pre>
     */
    boolean onPress();

    /**
     * Falling edge: returns {@code true} <b>only on the first loop</b> where
     * the button transitions from "held" to "not held".
     *
     * <p>Use this for "trigger once when released" behavior.</p>
     */
    boolean onRelease();

    /**
     * Level: returns {@code true} on <b>every loop</b> while the button is
     * currently held down.
     *
     * <p>This is the method you should use when you care about
     * "is the button down right now?" semantics.</p>
     */
    boolean isHeld();

    // ---------------------------------------------------------------------
    // Registry operations
    // ---------------------------------------------------------------------

    /**
     * Register a button with the global registry so that it will be included in
     * {@link #updateAllRegistered()}.
     *
     * <p>Buttons created via {@link #of(BooleanSupplier)} are automatically
     * registered. This method is mainly for advanced cases where you provide
     * your own {@link Button} implementation but still want it to participate
     * in global update handling.</p>
     */
    static void register(Button button) {
        Registry.register(button);
    }

    /**
     * Advance the state of all registered buttons by sampling their underlying
     * raw values.
     *
     * <p>Framework code should call this exactly once per loop. A typical place
     * to do so is from {@code Gamepads.update(dtSec)}.</p>
     */
    static void updateAllRegistered() {
        Registry.updateAll();
    }

    /**
     * Clear all registered buttons.
     *
     * <p>This is primarily intended for framework lifecycle management (e.g.,
     * when starting a new OpMode). User code normally does not need to call
     * this directly.</p>
     */
    static void clearRegistered() {
        Registry.clear();
    }

    // ---------------------------------------------------------------------
    // Factory helpers
    // ---------------------------------------------------------------------

    /**
     * Create a stateful {@link Button} from a raw boolean supplier and
     * automatically register it with the global registry.
     *
     * <p>The returned button:</p>
     * <ul>
     *   <li>Samples {@code raw.getAsBoolean()} in {@link #update()}.</li>
     *   <li>Provides rising-edge semantics via {@link #onPress()}.</li>
     *   <li>Provides falling-edge semantics via {@link #onRelease()}.</li>
     *   <li>Provides level semantics via {@link #isHeld()}.</li>
     * </ul>
     *
     * <p>Typical usage (inside gamepad wrappers):</p>
     *
     * <pre>{@code
     * public Button a() {
     *     return Button.of(() -> gp.a);
     * }
     * }</pre>
     */
    static Button of(BooleanSupplier raw) {
        StatefulButton b = new StatefulButton(raw);
        Registry.register(b);
        return b;
    }

    /**
     * Convenience: create a button that is always held or always released.
     *
     * <p>Edges never fire for constant buttons; {@link #onPress()} and
     * {@link #onRelease()} always return {@code false}.</p>
     */
    static Button constant(boolean held) {
        StatefulButton b = new StatefulButton(() -> held);
        Registry.register(b);
        return b;
    }

    // ---------------------------------------------------------------------
    // Default implementation used by factories
    // ---------------------------------------------------------------------

    /**
     * Default stateful {@link Button} implementation used by
     * {@link #of(BooleanSupplier)} and {@link #constant(boolean)}.
     *
     * <p>This class is public so that advanced users can construct and
     * optionally register custom instances, but most code should rely on
     * the static factories and gamepad wrappers.</p>
     */
    final class StatefulButton implements Button {
        private final BooleanSupplier raw;
        private boolean prev;
        private boolean curr;

        /**
         * @param raw supplier providing the raw "is down" value from hardware
         *            or higher-level logic. Must be non-null.
         */
        public StatefulButton(BooleanSupplier raw) {
            this.raw = Objects.requireNonNull(raw, "raw supplier is required");
            this.prev = false;
            this.curr = false;
        }

        @Override
        public void update() {
            prev = curr;
            curr = raw.getAsBoolean();
        }

        @Override
        public boolean onPress() {
            return curr && !prev;
        }

        @Override
        public boolean onRelease() {
            return !curr && prev;
        }

        @Override
        public boolean isHeld() {
            return curr;
        }
    }

    // ---------------------------------------------------------------------
    // Internal registry implementation
    // ---------------------------------------------------------------------

    /**
     * Simple global registry for buttons created via {@link #of(BooleanSupplier)}
     * or {@link #constant(boolean)} (and any others explicitly registered via
     * {@link #register(Button)}).
     *
     * <p>This is intentionally minimal and package-private; all interaction
     * should go through the static methods on {@link Button}.</p>
     */
    final class Registry {
        private static final List<Button> BUTTONS = new ArrayList<>();

        private Registry() {
            // no instances
        }

        static void register(Button button) {
            BUTTONS.add(Objects.requireNonNull(button, "button"));
        }

        static void updateAll() {
            for (Button b : BUTTONS) {
                b.update();
            }
        }

        static void clear() {
            BUTTONS.clear();
        }
    }
}
