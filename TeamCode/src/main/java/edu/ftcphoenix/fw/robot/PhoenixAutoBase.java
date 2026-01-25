package edu.ftcphoenix.fw.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.ftcphoenix.fw.util.LoopClock;

/**
 * Framework base class for Phoenix autonomous OpModes.
 *
 * <p>Responsibilities:
 * <ul>
 *   <li>Provide a structured lifecycle for autonomous:
 *       <ol>
 *         <li>Framework wiring.</li>
 *         <li>{@link #onInitRobot()} for user hardware/subsystems.</li>
 *         <li>{@link #waitForStart()} call.</li>
 *         <li>Clock reset and {@link #onStartRobot()} hook.</li>
 *         <li>{@link #runAuto()} user-defined autonomous sequence.</li>
 *         <li>{@link #onStopRobot()} cleanup hook.</li>
 *       </ol>
 *   </li>
 *   <li>Expose a {@link LoopClock} for dt-sensitive logic if desired.</li>
 * </ul>
 *
 * <p>Subclassing pattern:
 * <pre>
 * @Autonomous(name = "My Auto", group = "Phoenix")
 * public final class MyAuto extends PhoenixAutoBase {
 *     private PhoenixRobot robot;
 *
 *     @Override
 *     protected void initRobot() {
 *         robot = new PhoenixRobot(hardwareMap);
 *         // any sensor init, vision pipelines, etc.
 *     }
 *
 *     @Override
 *     protected void runAuto() throws InterruptedException {
 *         // Example structure:
 *         // 1) Set shooter goal, wait until at speed.
 *         // 2) Drive trajectory.
 *         // 3) Feed shots using robot.buffer.
 *     }
 * }
 * </pre>
 *
 * <p>Note: This base class does <b>not</b> know about gamepads, DriverKit,
 * or Bindings. Those are TeleOp-only concerns.</p>
 */
public abstract class PhoenixAutoBase extends LinearOpMode {

    private LoopClock clock;

    @Override
    public final void runOpMode() throws InterruptedException {
        // Basic clock wiring; subclasses can use clock() if they want dt.
        clock = new LoopClock();

        // Let subclass map hardware and construct subsystems.
        onInitRobot();

        telemetry.addLine("PhoenixAutoBase: init complete");
        telemetry.update();

        // Standard FTC start gate.
        waitForStart();
        if (isStopRequested()) {
            // If start was pressed and immediately stop requested.
            return;
        }

        // Reset clock and notify subclass that the match has started.
        clock.reset(getRuntime());
        onStartRobot();

        // Let subclass run its autonomous sequence.
        try {
            runAuto();
        } finally {
            // Ensure cleanup hook is invoked even if runAuto throws.
            onStopRobot();
        }
    }

    // --------------------------------------------------------------------
    // Subclass hooks
    // --------------------------------------------------------------------

    /**
     * Called once before {@link #waitForStart()}.
     *
     * <p>Use this to:
     * <ul>
     *   <li>Map hardware (motors, servos, sensors, RoadRunner drive, etc.).</li>
     *   <li>Construct subsystems / stages / robot containers.</li>
     *   <li>Run any pre-start configuration you need (but avoid long loops;
     *       runOpMode still needs to return promptly).</li>
     * </ul>
     */
    protected abstract void onInitRobot();

    /**
     * Called once after {@link #waitForStart()} and clock reset.
     *
     * <p>Optional; default implementation does nothing.
     *
     * <p>Use this to:
     * <ul>
     *   <li>Reset encoders/timers that should be zeroed at start.</li>
     *   <li>Start any background tasks or state machines.</li>
     * </ul>
     */
    protected void onStartRobot() {
        // default no-op
    }

    /**
     * Called once to run your autonomous sequence.
     *
     * <p>You may:
     * <ul>
     *   <li>Write sequential code (typical LinearOpMode style).</li>
     *   <li>Use {@link #opModeIsActive()} for loop conditions.</li>
     *   <li>Use {@link #clock()} if you want dt-based loops or timeouts.</li>
     * </ul>
     *
     * <p>Example skeleton:
     * <pre>
     * protected void runAuto() throws InterruptedException {
     *     while (opModeIsActive()) {
     *         clock().update(getRuntime());
     *         double dt = clock().dtSec();
     *
     *         // update subsystems, RoadRunner, etc.
     *         robot.update(dt);
     *
     *         telemetry.update();
     *     }
     * }
     * </pre>
     */
    protected abstract void runAuto() throws InterruptedException;

    /**
     * Called once after {@link #runAuto()} returns (or throws).
     *
     * <p>Use this as a cleanup hook (e.g., stop all motors, close logging files).
     */
    protected void onStopRobot() {
        // default no-op
    }

    // --------------------------------------------------------------------
    // Protected helpers for subclasses
    // --------------------------------------------------------------------

    /**
     * Access to the internal {@link LoopClock}. This is optional; you can
     * ignore it and use {@link #getRuntime()} or other timing if preferred.
     */
    protected LoopClock clock() {
        return clock;
    }
}
