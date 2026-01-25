package edu.ftcphoenix.fw.input.binding;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.input.Button;

/**
 * Binding manager that maps {@link Button} state to higher-level behavior.
 *
 * <p>Phoenix uses a polled input model:</p>
 * <ol>
 *   <li>Once per loop: update inputs (which advances button edge state).</li>
 *   <li>Once per loop: update bindings (which runs actions based on button state).</li>
 * </ol>
 *
 * <h2>Per-cycle idempotency</h2>
 * <p>{@link #update(LoopClock)} is <b>idempotent by {@link LoopClock#cycle()}</b>.
 * If called twice in the same loop cycle, the second call is a no-op. This prevents
 * nested or layered code from double-firing actions.</p>
 *
 * <p><b>Important:</b> Call {@code Gamepads.update(clock)} (or {@code Button.updateAllRegistered(clock)})
 * <em>before</em> calling {@link #update(LoopClock)}, so {@link Button#onPress()} /
 * {@link Button#onRelease()} reflect the current cycle.</p>
 */
public final class Bindings {

    // ---------------------------------------------------------------------------------------------
    // Binding record types
    // ---------------------------------------------------------------------------------------------

    /**
     * Simple (button, action) pair for rising-edge press actions.
     */
    private static final class PressBinding {
        final Button button;
        final Runnable action;

        PressBinding(Button button, Runnable action) {
            this.button = button;
            this.action = action;
        }
    }

    /**
     * Simple (button, action) pair for falling-edge release actions.
     */
    private static final class ReleaseBinding {
        final Button button;
        final Runnable action;

        ReleaseBinding(Button button, Runnable action) {
            this.button = button;
            this.action = action;
        }
    }

    /**
     * Binding that runs an action every loop while held and optionally once on release.
     */
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

    /**
     * Binding that reports a per-button toggle state to a consumer.
     *
     * <p>The toggle state is owned by the {@link Button} itself and flips on each rising edge.
     * This means the same physical button press produces the same toggle state everywhere you read it.
     *
     * <p>For example: if you pass {@code button::isToggled} as a {@code BooleanSupplier} to enable a
     * {@link edu.ftcphoenix.fw.drive.DriveOverlay}, and you also create a binding with
     * {@link Bindings#onToggle(Button, Consumer)}, both will observe the same on/off value.</p>
     */
    private static final class ToggleBinding {
        final Button button;
        final Consumer<Boolean> consumer;

        ToggleBinding(Button button, Consumer<Boolean> consumer) {
            this.button = button;
            this.consumer = consumer;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Binding storage
    // ---------------------------------------------------------------------------------------------

    private final List<PressBinding> pressBindings = new ArrayList<>();
    private final List<ReleaseBinding> releaseBindings = new ArrayList<>();
    private final List<WhileHeldBinding> whileHeldBindings = new ArrayList<>();
    private final List<ToggleBinding> toggleBindings = new ArrayList<>();

    /**
     * Tracks which loop cycle we last updated for, to prevent double-firing actions
     * if update() is accidentally called more than once per loop cycle.
     */
    private long lastUpdatedCycle = Long.MIN_VALUE;

    /**
     * Register an action to run once whenever the given button is pressed (rising edge).
     *
     * @param button button to monitor (non-null)
     * @param action action to run once per press (non-null)
     */
    public void onPress(Button button, Runnable action) {
        pressBindings.add(new PressBinding(
                Objects.requireNonNull(button, "button is required"),
                Objects.requireNonNull(action, "action is required")
        ));
    }

    /**
     * Register an action to run once whenever the given button is released (falling edge).
     *
     * <p>This is the mirror of {@link #onPress(Button, Runnable)} and is useful for
     * "start on press, stop on release" patterns without having to run code every loop
     * while the button is held.</p>
     *
     * @param button button to monitor (non-null)
     * @param action action to run once per release (non-null)
     */
    public void onRelease(Button button, Runnable action) {
        releaseBindings.add(new ReleaseBinding(
                Objects.requireNonNull(button, "button is required"),
                Objects.requireNonNull(action, "action is required")
        ));
    }

    /**
     * Register a "press-and-release" binding: run one action once when the button is pressed,
     * and a second action once when it is released.
     *
     * <p>This is a semantic convenience for the common pattern:
     * "start something on press, stop it on release" without having to run code every loop
     * while the button is held.</p>
     *
     * <p>Internally this is equivalent to registering both an {@link #onPress(Button, Runnable)}
     * and an {@link #onRelease(Button, Runnable)} on the same button.</p>
     */
    public void onPressAndRelease(Button button, Runnable onPress, Runnable onRelease) {
        Objects.requireNonNull(button, "button is required");
        onPress(button, Objects.requireNonNull(onPress, "onPress is required"));
        onRelease(button, Objects.requireNonNull(onRelease, "onRelease is required"));
    }

    /**
     * Register an action to run once per loop while the given button is held.
     *
     * @param button    button to monitor (non-null)
     * @param whileHeld action to run every loop while held (non-null)
     */
    public void whileHeld(Button button, Runnable whileHeld) {
        whileHeld(button, whileHeld, null);
    }

    /**
     * Register actions to run while a button is held, and once when it is released.
     *
     * <ul>
     *   <li>If {@link Button#isHeld()} is true, {@code whileHeld} is executed.</li>
     *   <li>If {@link Button#onRelease()} is true and {@code onRelease} is non-null,
     *       {@code onRelease} is executed once.</li>
     * </ul>
     *
     * @param button    button to monitor (non-null)
     * @param whileHeld action to run every loop while held (non-null)
     * @param onRelease action to run once on release (may be null)
     */
    public void whileHeld(Button button, Runnable whileHeld, Runnable onRelease) {
        whileHeldBindings.add(new WhileHeldBinding(
                Objects.requireNonNull(button, "button is required"),
                Objects.requireNonNull(whileHeld, "whileHeld action is required"),
                onRelease
        ));
    }

    /**
     * Register a toggle: flip a boolean each time the button is pressed (rising edge) and
     * deliver the <em>new</em> value to the consumer.
     *
     * <p>This method is the “action” sibling of reading {@link Button#isToggled()}:</p>
     * <ul>
     *   <li>When you need a {@code BooleanSupplier} (for example: to enable a
     *       {@link edu.ftcphoenix.fw.drive.DriveOverlay}), pass {@code button::isToggled}.</li>
     *   <li>When you want to run code on each toggle edge, use {@code Bindings.onToggle(...)}.</li>
     * </ul>
     *
     * <p>The toggle state is owned by the button, so the same physical button press toggles
     * the same shared on/off value everywhere.</p>
     *
     * @param button   button to monitor (non-null)
     * @param consumer consumer that receives the new toggle state (non-null)
     */
    public void onToggle(Button button, Consumer<Boolean> consumer) {
        toggleBindings.add(new ToggleBinding(
                Objects.requireNonNull(button, "button is required"),
                Objects.requireNonNull(consumer, "consumer is required")
        ));
    }

    /**
     * Register a toggle with split actions: when the button is pressed, call {@code onEnable}
     * if the toggle is now on, otherwise call {@code onDisable}.
     *
     * <p>This is a convenience overload of {@link #onToggle(Button, Consumer)}.</p>
     *
     * @param button    button to monitor (non-null)
     * @param onEnable  action to run when the toggle becomes enabled (non-null)
     * @param onDisable action to run when the toggle becomes disabled (non-null)
     */
    public void onToggle(Button button, Runnable onEnable, Runnable onDisable) {
        Objects.requireNonNull(button, "button is required");
        Objects.requireNonNull(onEnable, "onEnable is required");
        Objects.requireNonNull(onDisable, "onDisable is required");

        onToggle(button, isOn -> {
            if (isOn) {
                onEnable.run();
            } else {
                onDisable.run();
            }
        });
    }

    /**
     * Remove all registered bindings from this instance.
     *
     * <p>This does not unregister any {@link Button}s from the global registry.</p>
     */
    public void clear() {
        pressBindings.clear();
        releaseBindings.clear();
        whileHeldBindings.clear();
        toggleBindings.clear();
        lastUpdatedCycle = Long.MIN_VALUE;
    }

    /**
     * Poll all registered bindings and trigger their actions as appropriate.
     *
     * <p>Idempotent by {@link LoopClock#cycle()}.</p>
     *
     * @param clock loop clock (non-null; advanced once per OpMode loop cycle)
     */
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");

        long c = clock.cycle();
        if (c == lastUpdatedCycle) {
            return; // already updated this cycle
        }
        lastUpdatedCycle = c;

        // One-shot press bindings
        for (int i = 0; i < pressBindings.size(); i++) {
            PressBinding b = pressBindings.get(i);
            if (b.button.onPress()) {
                b.action.run();
            }
        }

        // One-shot release bindings
        for (int i = 0; i < releaseBindings.size(); i++) {
            ReleaseBinding b = releaseBindings.get(i);
            if (b.button.onRelease()) {
                b.action.run();
            }
        }

        // While-held bindings
        for (int i = 0; i < whileHeldBindings.size(); i++) {
            WhileHeldBinding b = whileHeldBindings.get(i);

            if (b.button.isHeld()) {
                b.whileHeld.run();
            }

            if (b.onRelease != null && b.button.onRelease()) {
                b.onRelease.run();
            }
        }

        // Toggle bindings
        for (int i = 0; i < toggleBindings.size(); i++) {
            ToggleBinding b = toggleBindings.get(i);
            if (b.button.onPress()) {
                b.consumer.accept(b.button.isToggled());
            }
        }
    }


    /**
     * Debug helper: emit binding counts and the current state of toggle/while-held bindings.
     *
     * <p>Bindings are intentionally "fire and forget" (they store actions), so this debug output is
     * mainly about verifying button wiring and toggle/edge behavior.</p>
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "bindings" : prefix;

        dbg.addLine(p)
                .addData(p + ".pressCount", pressBindings.size())
                .addData(p + ".releaseCount", releaseBindings.size())
                .addData(p + ".whileHeldCount", whileHeldBindings.size())
                .addData(p + ".toggleCount", toggleBindings.size())
                .addData(p + ".lastUpdatedCycle", lastUpdatedCycle);

        for (int i = 0; i < whileHeldBindings.size(); i++) {
            WhileHeldBinding b = whileHeldBindings.get(i);
            String bp = p + ".whileHeld" + i;
            dbg.addData(bp + ".held", b.button.isHeld())
                    .addData(bp + ".onPress", b.button.onPress())
                    .addData(bp + ".onRelease", b.button.onRelease())
                    .addData(bp + ".hasOnReleaseAction", b.onRelease != null);
        }

        for (int i = 0; i < toggleBindings.size(); i++) {
            ToggleBinding b = toggleBindings.get(i);
            String tp = p + ".toggle" + i;
            dbg.addData(tp + ".toggled", b.button.isToggled())
                    .addData(tp + ".held", b.button.isHeld())
                    .addData(tp + ".onPress", b.button.onPress());
        }
    }

}
