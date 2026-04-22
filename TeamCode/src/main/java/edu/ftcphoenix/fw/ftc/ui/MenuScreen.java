package edu.ftcphoenix.fw.ftc.ui;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * One telemetry UI screen that can be hosted by {@link MenuNavigator}.
 *
 * <p>A screen owns only the behavior for the currently visible page: how it reacts to high-level
 * navigation events and how it renders itself. It should not register its own long-lived bindings
 * when used inside a navigator; the navigator owns input dispatch so nested screens do not leave old
 * bindings behind.</p>
 */
public interface MenuScreen {

    /** Short title used in headers and navigator breadcrumbs. */
    String title();

    /** Called when this screen becomes the active screen. */
    default void onEnter() {
        // Default: no-op.
    }

    /** Called when this screen stops being the active screen. */
    default void onExit() {
        // Default: no-op.
    }

    /** Move highlight/value up. */
    default void up() {
        // Default: no-op.
    }

    /** Move highlight/value down. */
    default void down() {
        // Default: no-op.
    }

    /** Move to the previous page/value, when the screen supports it. */
    default void left() {
        // Default: no-op.
    }

    /** Move to the next page/value, when the screen supports it. */
    default void right() {
        // Default: no-op.
    }

    /** Confirm or enter the highlighted choice. */
    default void select() {
        // Default: no-op.
    }

    /** Optional secondary action, commonly refresh/details/reset depending on the screen. */
    default void secondary() {
        // Default: no-op.
    }

    /**
     * Give the screen first chance to handle back/cancel.
     *
     * @return true if the screen consumed the back action, false if the navigator should pop
     */
    default boolean onBack() {
        return false;
    }

    /**
     * Give the screen first chance to handle home/root navigation.
     *
     * @return true if the screen consumed the home action, false if the navigator should go home
     */
    default boolean onHome() {
        return false;
    }

    /** Render this screen to FTC telemetry. This method should not call {@code telemetry.update()}. */
    void render(Telemetry telemetry, MenuRenderContext context);
}
