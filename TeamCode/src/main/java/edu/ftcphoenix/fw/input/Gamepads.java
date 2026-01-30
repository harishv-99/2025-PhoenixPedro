package edu.ftcphoenix.fw.input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;

/**
 * Convenience wrapper around two FTC gamepads (player 1 and player 2).
 *
 * <p>This class is intentionally thin. It exists so callers can pass a single object through
 * constructors and keep a consistent "p1 / p2" naming convention.</p>
 *
 * <h2>Inputs are Sources</h2>
 * <p>{@link GamepadDevice} exposes:</p>
 * <ul>
 *   <li>axes as {@link edu.ftcphoenix.fw.core.source.ScalarSource}, and</li>
 *   <li>buttons as {@link edu.ftcphoenix.fw.core.source.BooleanSource}.</li>
 * </ul>
 *
 * <p>There is <b>no global update step</b> for button edges. If you need edges, use
 * {@link edu.ftcphoenix.fw.core.source.BooleanSource#risingEdge()} /
 * {@link edu.ftcphoenix.fw.core.source.BooleanSource#fallingEdge()} (or use
 * {@link edu.ftcphoenix.fw.input.binding.Bindings} which does this for you).</p>
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
     * Debug helper: emit player 1 & player 2 axis calibration state.
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
