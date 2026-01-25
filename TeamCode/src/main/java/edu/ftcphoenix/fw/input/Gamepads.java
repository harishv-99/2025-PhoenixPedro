package edu.ftcphoenix.fw.input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Convenience wrapper around two FTC gamepads (player 1 and player 2).
 *
 * <p>This class exposes:</p>
 * <ul>
 *   <li>Axes (sticks/triggers) via {@link GamepadDevice}.</li>
 *   <li>Buttons via {@link Button}, including edge detection
 *       ({@link Button#onPress()} / {@link Button#onRelease()}).</li>
 * </ul>
 *
 * <h2>Per-cycle update rule</h2>
 *
 * <p>Button edge detection only works correctly when button state advances exactly once
 * per OpMode loop cycle. Phoenix enforces this by requiring callers to update inputs
 * using a shared {@link LoopClock}:</p>
 *
 * <pre>{@code
 * clock.update(getRuntime());
 * gamepads.update(clock);   // advances all registered Buttons (idempotent by clock.cycle())
 * bindings.update(clock);   // runs actions (also idempotent by clock.cycle())
 * }</pre>
 *
 * <p>{@link #update(LoopClock)} delegates to {@link Button#updateAllRegistered(LoopClock)},
 * which is idempotent by {@link LoopClock#cycle()}. If nested code accidentally calls update
 * twice in the same cycle, the second call is a no-op (edges are not consumed).</p>
 */
public final class Gamepads {

    private final GamepadDevice p1;
    private final GamepadDevice p2;

    /**
     * Create a {@link Gamepads} wrapper from two FTC {@link Gamepad} instances.
     *
     * @param gp1 FTC SDK gamepad1 (non-null)
     * @param gp2 FTC SDK gamepad2 (non-null)
     */
    public static Gamepads create(Gamepad gp1, Gamepad gp2) {
        Objects.requireNonNull(gp1, "gp1 is required");
        Objects.requireNonNull(gp2, "gp2 is required");
        return new Gamepads(new GamepadDevice(gp1), new GamepadDevice(gp2));
    }

    /**
     * Alternate factory name (same as {@link #create(Gamepad, Gamepad)}).
     */
    public static Gamepads of(Gamepad gp1, Gamepad gp2) {
        return create(gp1, gp2);
    }

    /**
     * Construct from already-wrapped {@link GamepadDevice}s.
     *
     * <p>Mainly useful for tests or advanced composition.</p>
     */
    public Gamepads(GamepadDevice p1, GamepadDevice p2) {
        this.p1 = Objects.requireNonNull(p1, "p1 is required");
        this.p2 = Objects.requireNonNull(p2, "p2 is required");
    }

    /**
     * Player 1 gamepad wrapper.
     */
    public GamepadDevice p1() {
        return p1;
    }

    /**
     * Player 2 gamepad wrapper.
     */
    public GamepadDevice p2() {
        return p2;
    }

    /**
     * Update all registered {@link Button}s for this loop cycle.
     *
     * <p>Call once per OpMode loop cycle <b>before</b> reading edges via
     * {@link Button#onPress()} / {@link Button#onRelease()}.</p>
     *
     * <p>This method is safe to call multiple times within the same cycle because
     * {@link Button#updateAllRegistered(LoopClock)} is idempotent by
     * {@link LoopClock#cycle()}.</p>
     *
     * @param clock loop clock (non-null; advanced once per OpMode loop cycle)
     */
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");
        Button.updateAllRegistered(clock);
    }

    /**
     * Clear the global registry of all registered buttons.
     *
     * <p>Most robot code does not need this. It is primarily useful for framework
     * lifecycle management to ensure no stale button objects persist across runs.</p>
     */
    public void clearButtons() {
        Button.clearRegistered();
    }


    /**
     * Debug helper: emit player 1 & player 2 input state.
     *
     * <p>This is useful when verifying axis sign conventions and button edge detection.</p>
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "gamepads" : prefix;
        dbg.addLine(p);
        p1.debugDump(dbg, p + ".p1");
        p2.debugDump(dbg, p + ".p2");
    }

}
