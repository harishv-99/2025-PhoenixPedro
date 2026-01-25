package edu.ftcphoenix.fw.tools.tester;

/**
 * Minimal lifecycle for Phoenix "tester programs" that can run inside an FTC OpMode.
 *
 * <p>Design goals:
 * <ul>
 *   <li>Testers live outside TeamCode (e.g., in robots.*) and are reusable.</li>
 *   <li>TeamCode provides a single OpMode wrapper that runs a tester or a tester suite.</li>
 *   <li>Support selection menus (device/camera pickers, mode selection) during INIT via
 *       {@link #initLoop(double)}.</li>
 * </ul>
 */
public interface TeleOpTester {

    /**
     * Human-friendly name used by menus / telemetry.
     */
    String name();

    /**
     * Called once during OpMode init.
     */
    void init(TesterContext ctx);

    /**
     * Called repeatedly during the FTC OpMode INIT phase (before start is pressed).
     *
     * <p>Use this for selection screens (e.g., choose a camera from config names) and
     * for any HardwareMap lookups that you want to keep in init.</p>
     *
     * <p><b>Important:</b> Avoid commanding actuators here. Keep it to UI/selection and setup.</p>
     *
     * @param dtSec Time since last init-loop (seconds).
     */
    default void initLoop(double dtSec) {
    }

    /**
     * Called once when the OpMode transitions from INIT to RUNNING.
     */
    default void start() {
    }

    /**
     * Called every OpMode loop while RUNNING.
     *
     * @param dtSec Time since last loop (seconds).
     */
    void loop(double dtSec);

    /**
     * Called once when the OpMode stops.
     */
    default void stop() {
    }

    /**
     * Optional navigation hook for runners that provide hierarchical menus.
     *
     * <p>
     * Some Phoenix runners (notably {@link TesterSuite}) reserve the gamepad1 <b>BACK</b>
     * button for navigation. When BACK is pressed, the runner may call this method on the
     * active tester to give it a chance to navigate to a previous step (for example, return
     * from a "live view" screen back to a device picker).
     * </p>
     *
     * <p>
     * Return {@code true} if you handled the BACK press and want to remain inside the tester.
     * Return {@code false} to indicate you did not handle it; the runner may then stop the tester
     * and return to its menu.
     * </p>
     *
     * <p>
     * Testers that have multi-step UIs should prefer overriding this method rather than binding
     * directly to the BACK button. This keeps navigation consistent when the tester is executed
     * inside a {@link TesterSuite}.
     * </p>
     *
     * @return {@code true} if the BACK press was handled; {@code false} otherwise
     */
    default boolean onBackPressed() {
        return false;
    }
}
