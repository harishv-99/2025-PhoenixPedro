package edu.ftcphoenix.fw.input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Objects;

/**
 * Convenience wrapper around two FTC gamepads (player 1 and player 2).
 *
 * <p>Typical usage:</p>
 *
 * <pre>{@code
 * public class MyTeleOp extends LinearOpMode {
 *     private final LoopClock clock = new LoopClock();
 *     private Gamepads pads;
 *
 *     @Override
 *     public void runOpMode() {
 *         pads = Gamepads.create(gamepad1, gamepad2);
 *
 *         waitForStart();
 *         clock.reset();
 *
 *         while (opModeIsActive()) {
 *             double dtSec = clock.dtSec();
 *
 *             pads.update(dtSec);  // updates all registered buttons
 *
 *             // Axes:
 *             double driveX = pads.p1().leftX().get();
 *             double driveY = pads.p1().leftY().get();
 *
 *             // Buttons with edges:
 *             if (pads.p1().a().onPress()) {
 *                 // Runs once when A is pressed.
 *             }
 *
 *             if (pads.p1().rb().isHeld()) {
 *                 // Runs every loop while RB is down.
 *             }
 *
 *             // ... robot logic ...
 *         }
 *     }
 * }
 * }</pre>
 *
 * <h2>Update responsibilities</h2>
 *
 * <ul>
 *   <li>{@link GamepadDevice} constructs {@link Button} instances using
 *       {@link Button#of(java.util.function.BooleanSupplier)}, which
 *       automatically register with the global button registry.</li>
 *   <li>{@link #update(double)} calls {@link Button#updateAllRegistered()},
 *       which in turn calls {@link Button#update()} on every registered button.</li>
 *   <li>Robot code should call {@link #update(double)} exactly once per loop
 *       <b>before</b> any button queries (e.g. {@code onPress()}, {@code isHeld()}).</li>
 * </ul>
 */
public final class Gamepads {

    private final GamepadDevice p1;
    private final GamepadDevice p2;

    /**
     * Create a {@link Gamepads} wrapper from two FTC {@link Gamepad} instances.
     *
     * <p>This is the preferred entry point for TeleOp code.</p>
     */
    public static Gamepads create(Gamepad gp1, Gamepad gp2) {
        Objects.requireNonNull(gp1, "gp1 is required");
        Objects.requireNonNull(gp2, "gp2 is required");
        return new Gamepads(new GamepadDevice(gp1), new GamepadDevice(gp2));
    }

    /**
     * Alternate factory name for convenience / backwards-compatibility.
     *
     * <p>Equivalent to {@link #create(Gamepad, Gamepad)}.</p>
     */
    public static Gamepads of(Gamepad gp1, Gamepad gp2) {
        return create(gp1, gp2);
    }

    /**
     * Construct from two already-wrapped {@link GamepadDevice}s.
     *
     * <p>This is mainly useful for testing or advanced composition; most code
     * should use {@link #create(Gamepad, Gamepad)}.</p>
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
     * Update all registered buttons for this loop.
     *
     * <p>Currently this simply forwards to {@link Button#updateAllRegistered()}.
     * The {@code dtSec} parameter is included for future use (for example, if
     * we later add time-based filters or smoothing to axes or higher-level
     * input channels).</p>
     *
     * <p><b>Call this exactly once per loop</b> before reading any button
     * edges or hold states.</p>
     *
     * @param dtSec time since last loop in seconds (may be unused for now)
     */
    public void update(double dtSec) {
        // Update all known buttons (including those created by GamepadDevice
        // and any other code that used Button.of(...) or Button.constant(...)).
        Button.updateAllRegistered();

        // If we ever add time-dependent processing for axes or more complex
        // input channels, this is a natural place to pass dtSec through.
    }

    /**
     * Clear all registered buttons.
     *
     * <p>Framework code may call this at the start of an OpMode to ensure that
     * no stale button state is carried over between runs. Most user code does
     * not need to call this directly.</p>
     */
    public void clearButtons() {
        Button.clearRegistered();
    }
}
