package edu.ftcphoenix.fw.ftc.ui;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.input.GamepadDevice;
import edu.ftcphoenix.fw.input.Gamepads;

/**
 * Standard high-level controls for FTC telemetry UI screens.
 *
 * <p>This class keeps menu input semantics consistent across tester suites, hardware pickers, and
 * pre-start autonomous selectors. A screen can still ignore controls it does not support; the goal is
 * to make the common meanings predictable:</p>
 * <ul>
 *   <li>Dpad Up/Down: move within a list or value</li>
 *   <li>Dpad Left/Right: page or adjust a side value</li>
 *   <li>A: select/confirm</li>
 *   <li>B or BACK: back/cancel</li>
 *   <li>X: secondary action, often refresh/details</li>
 *   <li>Y: home/root or reset-to-default depending on the screen</li>
 * </ul>
 */
public final class UiControls {

    /** Move up. */
    public final BooleanSource up;
    /** Move down. */
    public final BooleanSource down;
    /** Move left / previous page. */
    public final BooleanSource left;
    /** Move right / next page. */
    public final BooleanSource right;
    /** Select / confirm. */
    public final BooleanSource select;
    /** Back / cancel. */
    public final BooleanSource back;
    /** Secondary action, commonly refresh or details. */
    public final BooleanSource secondary;
    /** Home/root action. */
    public final BooleanSource home;

    private final String hint;

    /**
     * Create a control mapping.
     *
     * <p>Any source may be null when a screen should not expose that action. Binders check nulls and
     * simply skip missing controls.</p>
     */
    public UiControls(BooleanSource up,
                      BooleanSource down,
                      BooleanSource left,
                      BooleanSource right,
                      BooleanSource select,
                      BooleanSource back,
                      BooleanSource secondary,
                      BooleanSource home,
                      String hint) {
        this.up = up;
        this.down = down;
        this.left = left;
        this.right = right;
        this.select = select;
        this.back = back;
        this.secondary = secondary;
        this.home = home;
        this.hint = cleanOrDefault(hint, "Dpad: move | A: choose | B/BACK: back | X: action | Y: home");
    }

    /** Standard menu controls for gamepad 1. */
    public static UiControls gamepad1(Gamepads gamepads) {
        return standard(gamepads.p1());
    }

    /** Standard menu controls for gamepad 2. */
    public static UiControls gamepad2(Gamepads gamepads) {
        return standard(gamepads.p2());
    }

    /** Standard menu controls for a wrapped FTC gamepad. */
    public static UiControls standard(GamepadDevice pad) {
        return new UiControls(
                pad.dpadUp(),
                pad.dpadDown(),
                pad.dpadLeft(),
                pad.dpadRight(),
                pad.a(),
                pad.b().or(pad.back()),
                pad.x(),
                pad.y(),
                "Dpad: move | A: choose | B/BACK: back | X: action | Y: home"
        );
    }

    /** Return a copy with a different human-facing controls hint. */
    public UiControls withHint(String hint) {
        return new UiControls(up, down, left, right, select, back, secondary, home, hint);
    }

    /** Human-facing controls hint suitable for telemetry. */
    public String hint() {
        return hint;
    }

    private static String cleanOrDefault(String value, String fallback) {
        if (value == null) return fallback;
        String trimmed = value.trim();
        return trimmed.isEmpty() ? fallback : trimmed;
    }
}
