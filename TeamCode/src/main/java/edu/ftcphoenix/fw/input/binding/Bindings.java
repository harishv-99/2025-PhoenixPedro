package edu.ftcphoenix.fw.input.binding;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.ftcphoenix.fw.input.Button;

/**
 * Simple binding manager that turns {@link Button} states into higher-level
 * behaviors for teleop:
 *
 * <ul>
 *   <li>{@link #onPress(Button, Runnable)} – fire once on rising edge.</li>
 *   <li>{@link #whileHeld(Button, Runnable, Runnable)} – run once per loop while
 *       pressed, and a separate action when released.</li>
 *   <li>{@link #toggle(Button, Consumer)} – flip a boolean on each press and
 *       deliver the new value to a consumer.</li>
 * </ul>
 *
 * <p>
 * With the new stateful {@link Button} API, {@code Bindings} no longer needs to
 * compute edges itself. Instead, it delegates to:
 * </p>
 *
 * <ul>
 *   <li>{@link Button#onPress()} – rising edge.</li>
 *   <li>{@link Button#onRelease()} – falling edge.</li>
 *   <li>{@link Button#isHeld()} – level.</li>
 * </ul>
 *
 * <p>
 * Typical usage:
 * </p>
 *
 * <pre>{@code
 * Gamepads pads = Gamepads.create(gamepad1, gamepad2);
 * Bindings bindings = new Bindings();
 *
 * bindings.onPress(pads.p1().a(), () -> shooter.fireOne());
 * bindings.whileHeld(pads.p1().rb(),
 *                    () -> intake.setTarget(+1.0),
 *                    () -> intake.setTarget(0.0));
 *
 * @Override
 * public void loop() {
 *     double dtSec = clock.dtSec();
 *
 *     pads.update(dtSec);      // updates all registered Buttons
 *     bindings.update(dtSec);  // polls buttons and runs actions
 *
 *     // ... mechanism updates ...
 * }
 * }</pre>
 */
public final class Bindings {

    // ---------------------------------------------------------------------
    // Binding record types
    // ---------------------------------------------------------------------

    private static final class PressBinding {
        final Button button;
        final Runnable action;

        PressBinding(Button button, Runnable action) {
            this.button = button;
            this.action = action;
        }
    }

    private static final class WhileHeldBinding {
        final Button button;
        final Runnable whileHeld;
        final Runnable onRelease; // may be null

        WhileHeldBinding(Button button, Runnable whileHeld, Runnable onRelease) {
            this.button = button;
            this.whileHeld = whileHeld;
            this.onRelease = onRelease;
        }
    }

    private static final class ToggleBinding {
        final Button button;
        final Consumer<Boolean> consumer;
        boolean toggled;

        ToggleBinding(Button button, Consumer<Boolean> consumer) {
            this.button = button;
            this.consumer = consumer;
            this.toggled = false;
        }
    }

    // ---------------------------------------------------------------------
    // Binding storage
    // ---------------------------------------------------------------------

    private final List<PressBinding> pressBindings = new ArrayList<>();
    private final List<WhileHeldBinding> whileHeldBindings = new ArrayList<>();
    private final List<ToggleBinding> toggleBindings = new ArrayList<>();

    /**
     * Create an empty {@link Bindings} set.
     */
    public Bindings() {
        // nothing else
    }

    // ---------------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------------

    /**
     * Register an action to run once whenever the given button is pressed
     * (rising edge).
     *
     * <p>
     * This uses {@link Button#onPress()} internally, so it fires exactly once
     * per press, regardless of how long the button is held.
     * </p>
     *
     * @param button button to monitor (non-null)
     * @param action action to run once per press (non-null)
     */
    public void onPress(Button button, Runnable action) {
        if (button == null) {
            throw new IllegalArgumentException("button is required");
        }
        if (action == null) {
            throw new IllegalArgumentException("action is required");
        }
        pressBindings.add(new PressBinding(button, action));
    }

    /**
     * Register an action to run once per loop while the given button is held.
     *
     * <p>
     * This is a convenience overload that has no explicit "on release" action.
     * It is equivalent to calling:
     * </p>
     *
     * <pre>{@code
     * whileHeld(button, whileHeld, null);
     * }</pre>
     *
     * @param button    button to monitor (non-null)
     * @param whileHeld action to run every loop while the button is held (non-null)
     */
    public void whileHeld(Button button, Runnable whileHeld) {
        whileHeld(button, whileHeld, null);
    }

    /**
     * Register actions to run while a button is held, and once when it is
     * released.
     *
     * <p>
     * On each loop:
     * </p>
     *
     * <ul>
     *   <li>If {@link Button#isHeld()} is true, {@code whileHeld} is executed.</li>
     *   <li>If {@link Button#onRelease()} is true and {@code onRelease} is non-null,
     *       {@code onRelease} is executed once.</li>
     * </ul>
     *
     * @param button    button to monitor (non-null)
     * @param whileHeld action to run every loop while the button is held (non-null)
     * @param onRelease action to run once when the button is released
     *                  (may be {@code null} if not needed)
     */
    public void whileHeld(Button button, Runnable whileHeld, Runnable onRelease) {
        if (button == null) {
            throw new IllegalArgumentException("button is required");
        }
        if (whileHeld == null) {
            throw new IllegalArgumentException("whileHeld action is required");
        }
        whileHeldBindings.add(new WhileHeldBinding(button, whileHeld, onRelease));
    }

    /**
     * Register a toggle binding: flip a boolean each time the button is pressed
     * (rising edge) and deliver the new value to the consumer.
     *
     * <p>
     * Internally this uses {@link Button#onPress()} for the rising edge and
     * maintains a single boolean toggle state per binding.
     * </p>
     *
     * @param button   button to monitor (non-null)
     * @param consumer consumer that receives the updated toggle state
     *                 ({@code true} / {@code false}) each time the button is
     *                 pressed (non-null)
     */
    public void toggle(Button button, Consumer<Boolean> consumer) {
        if (button == null) {
            throw new IllegalArgumentException("button is required");
        }
        if (consumer == null) {
            throw new IllegalArgumentException("consumer is required");
        }
        toggleBindings.add(new ToggleBinding(button, consumer));
    }

    /**
     * Remove all registered bindings.
     *
     * <p>
     * This does <b>not</b> unregister any {@link Button} instances from the
     * global button registry; it only clears the actions associated with
     * this {@link Bindings} instance.
     * </p>
     */
    public void clear() {
        pressBindings.clear();
        whileHeldBindings.clear();
        toggleBindings.clear();
    }

    /**
     * Poll all registered bindings and trigger their actions as appropriate.
     *
     * <p>
     * This method should be called once per loop <b>after</b> the framework
     * has updated button state (for example, after {@code Gamepads.update(dtSec)}
     * has been called, which internally calls
     * {@link Button#updateAllRegistered()}).
     * </p>
     *
     * <p>
     * The {@code dtSec} parameter is currently unused but is included here in
     * case we later add time-based behaviors (e.g., long-press detection)
     * directly within {@code Bindings}.
     * </p>
     *
     * @param dtSec time since last loop in seconds (currently unused)
     */
    public void update(double dtSec) {
        // one-shot press bindings
        for (int i = 0; i < pressBindings.size(); i++) {
            PressBinding b = pressBindings.get(i);
            if (b.button.onPress()) {
                b.action.run();
            }
        }

        // while-held bindings
        for (int i = 0; i < whileHeldBindings.size(); i++) {
            WhileHeldBinding b = whileHeldBindings.get(i);

            if (b.button.isHeld()) {
                b.whileHeld.run();
            }

            if (b.onRelease != null && b.button.onRelease()) {
                b.onRelease.run();
            }
        }

        // toggle bindings
        for (int i = 0; i < toggleBindings.size(); i++) {
            ToggleBinding b = toggleBindings.get(i);
            if (b.button.onPress()) {
                b.toggled = !b.toggled;
                b.consumer.accept(Boolean.valueOf(b.toggled));
            }
        }
    }
}
