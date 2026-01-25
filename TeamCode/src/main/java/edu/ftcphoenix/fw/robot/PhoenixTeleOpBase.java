package edu.ftcphoenix.fw.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import edu.ftcphoenix.fw.input.GamepadDevice;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.util.LoopClock;

/**
 * Framework base class for Phoenix teleop OpModes.
 *
 * <h2>Responsibilities</h2>
 *
 * <p>This base class takes care of the repetitive wiring that almost every
 * TeleOp needs:</p>
 *
 * <ul>
 *   <li>Create and own a {@link Gamepads} wrapper for {@code gamepad1} and
 *       {@code gamepad2}.</li>
 *   <li>Create and own a {@link Bindings} instance for button edge-detection
 *       and simple state machines.</li>
 *   <li>Create and own a {@link LoopClock} to track loop {@code dtSec}.</li>
 *   <li>Ensure inputs and bindings are updated every loop.</li>
 *   <li>Provide simple accessors:
 *     <ul>
 *       <li>{@link #p1()} and {@link #p2()} for the two controllers
 *           ({@link GamepadDevice}).</li>
 *       <li>{@link #bind()} for input bindings.</li>
 *       <li>{@link #clock()} if you need the raw {@link LoopClock}.</li>
 *     </ul>
 *   </li>
 *   <li>Call your robot-specific hooks:
 *     <ul>
 *       <li>{@link #onInitRobot()} once from {@link #init()}.</li>
 *       <li>{@link #onStartRobot()} once from {@link #start()}.</li>
 *       <li>{@link #onLoopRobot(double)} every loop after inputs are updated.</li>
 *       <li>{@link #onStopRobot()} once from {@link #stop()}.</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p>This keeps your TeleOp classes focused on <em>what the robot should do</em>
 * instead of input plumbing.</p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * @TeleOp(name = "My TeleOp", group = "Examples")
 * public final class MyTeleOp extends PhoenixTeleOpBase {
 *
 *     private MecanumDrivebase drive;
 *
 *     @Override
 *     protected void onInitRobot() {
 *         // 1) Map hardware
 *         MecanumDrivebase.Config cfg = MecanumConfig.defaults();
 *         drive = Drives.mecanum(hardwareMap, "FL", "FR", "BL", "BR")
 *                      .config(cfg)
 *                      .build();
 *
 *         // 2) Set up drive controls using player 1 sticks
 *         DriveSource sticks = GamepadDriveSource.teleOpMecanumWithSlowMode(
 *                 gamepads(),          // full pair, if needed
 *                 p1().rightBumper(),  // hold for slow mode
 *                 0.30                 // 30% speed
 *         );
 *
 *         // Example: bind A to do something once on press
 *         bind().onPress(p1().buttonA(), () -> telemetry.addLine("A pressed"));
 *     }
 *
 *     @Override
 *     protected void onLoopRobot(double dtSec) {
 *         // Drivebase update, subsystems, telemetry, etc.
 *     }
 * }
 * }</pre>
 */
public abstract class PhoenixTeleOpBase extends OpMode {

    private Gamepads gamepads;
    private Bindings bindings;
    private LoopClock clock;

    // --------------------------------------------------------------------
    // OpMode lifecycle
    // --------------------------------------------------------------------

    @Override
    public final void init() {
        // Core input plumbing
        gamepads = Gamepads.create(gamepad1, gamepad2);
        bindings = new Bindings();
        clock = new LoopClock();

        // Let subclass wire hardware + bindings
        onInitRobot();

        telemetry.addLine("PhoenixTeleOpBase: init complete");
        telemetry.update();
    }

    @Override
    public final void start() {
        // Reset loop clock at the moment start() is called
        clock.reset(getRuntime());
        onStartRobot();
    }

    @Override
    public final void loop() {
        // Update timing
        clock.update(getRuntime());
        double dtSec = clock.dtSec();

        // Update inputs + bindings
        gamepads.update(dtSec);   // currently a no-op, kept for future filters
        bindings.update(dtSec);

        // Delegate to subclass for robot behavior
        onLoopRobot(dtSec);
    }

    @Override
    public final void stop() {
        onStopRobot();
    }

    // --------------------------------------------------------------------
    // Subclass hooks
    // --------------------------------------------------------------------

    /**
     * Called once from {@link #init()} after framework wiring is done.
     *
     * <p>Use this to:</p>
     * <ul>
     *   <li>Map hardware (motors, servos, sensors).</li>
     *   <li>Construct drivebases and subsystems.</li>
     *   <li>Define bindings using {@link #bind()} and {@link #p1()} / {@link #p2()}.</li>
     * </ul>
     */
    protected abstract void onInitRobot();

    /**
     * Called once from {@link #start()} after the internal loop clock is reset.
     *
     * <p>Optional; default implementation does nothing.</p>
     */
    protected void onStartRobot() {
        // Default: no-op.
    }

    /**
     * Called every loop after inputs and bindings have been updated.
     *
     * <p>Use this to:</p>
     * <ul>
     *   <li>Update drive / stages / subsystems.</li>
     *   <li>Publish telemetry.</li>
     * </ul>
     *
     * @param dtSec loop time step in seconds (from {@link LoopClock})
     */
    protected abstract void onLoopRobot(double dtSec);

    /**
     * Called once from {@link #stop()} before the OpMode ends.
     *
     * <p>Optional; default implementation does nothing.</p>
     */
    protected void onStopRobot() {
        // Default: no-op.
    }

    // --------------------------------------------------------------------
    // Protected helpers for subclasses
    // --------------------------------------------------------------------

    /**
     * Bindings instance for button edge detection and simple macro/state logic.
     *
     * @return the shared {@link Bindings} instance for this TeleOp
     */
    protected Bindings bind() {
        return bindings;
    }

    /**
     * Loop clock, in case you need access to the raw object.
     *
     * @return the {@link LoopClock} used by this base
     */
    protected LoopClock clock() {
        return clock;
    }

    /**
     * Player 1 controller as a {@link GamepadDevice}.
     *
     * @return gamepad 1 wrapper
     */
    protected GamepadDevice p1() {
        return gamepads.p1();
    }

    /**
     * Player 2 controller as a {@link GamepadDevice}.
     *
     * @return gamepad 2 wrapper
     */
    protected GamepadDevice p2() {
        return gamepads.p2();
    }

    /**
     * Full {@link Gamepads} pair, if you need it.
     */
    protected Gamepads gamepads() {
        return gamepads;
    }
}
