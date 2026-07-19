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
 *
 * <h2>Lifecycle ownership</h2>
 * <p>A tester factory should construct an inactive tester and defer hardware or resource
 * acquisition to {@link #init(TesterContext)}. A runner cannot clean up a factory that fails
 * before returning its tester.</p>
 *
 * <p>Once a runner receives a tester, it retains that tester before calling {@code init}. If
 * {@code init} begins and then fails, the runner may immediately call {@link #stop()} so that
 * partially acquired resources can be released. Implementations must therefore make
 * {@code stop()} safe after partial initialization. For each activation, a fail-stop runner calls
 * {@code stop()} at most once and invokes no further lifecycle callbacks after stopping begins.</p>
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
     * Called once when the tester is left or the owning OpMode stops.
     *
     * <p>This may follow a partially completed or failed {@link #init(TesterContext)}. Release
     * every resource acquired so far without assuming that initialization completed.</p>
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
