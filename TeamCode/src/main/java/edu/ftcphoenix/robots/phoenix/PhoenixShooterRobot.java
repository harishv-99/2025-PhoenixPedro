package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.Drives;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.util.LoopClock;

/**
 * PhoenixShooterRobot: shared robot wiring and high-level controls
 * for a mecanum drive + shooter + transfer + pusher robot.
 *
 * <h2>Hardware summary</h2>
 *
 * <ul>
 *   <li>Mecanum drive (4 motors), created via {@link Drives#mecanum}.</li>
 *   <li>Shooter: 2 DC motors (velocity-controlled pair).</li>
 *   <li>Transfer: 2 continuous-rotation servos (power-controlled pair).</li>
 *   <li>Pusher: 1 positional servo (position-controlled).</li>
 * </ul>
 *
 * <h2>Usage from TeleOp</h2>
 *
 * <pre>{@code
 * public final class ShooterTeleOp extends OpMode {
 *     private PhoenixShooterRobot robot;
 *     private Gamepads gamepads;
 *
 *     @Override
 *     public void init() {
 *         gamepads = Gamepads.create(gamepad1, gamepad2);
 *         robot = new PhoenixShooterRobot(hardwareMap);
 *     }
 *
 *     @Override
 *     public void start() {
 *         robot.resetClock(getRuntime());
 *     }
 *
 *     @Override
 *     public void loop() {
 *         robot.updateClock(getRuntime());
 *
 *         // Drive with sticks (for example, using StickDriveSource externally).
 *         DriveSignal driveSignal = ...;  // from StickDriveSource
 *         robot.setDriveSignal(driveSignal);
 *
 *         // Shooter controls (very simple example):
 *         boolean shooterOn = gamepads.p1().rightTrigger().isPressed();
 *         robot.setShooterEnabled(shooterOn);
 *
 *         // Transfer + pusher coordinated by driver:
 *         if (gamepads.p1().a().wasPressed()) {
 *             robot.setTransferMode(PhoenixShooterRobot.TransferMode.LOAD);
 *             robot.setPusherMode(PhoenixShooterRobot.PusherMode.LOAD);
 *         } else if (gamepads.p1().b().wasPressed()) {
 *             robot.setTransferMode(PhoenixShooterRobot.TransferMode.SHOOT);
 *             robot.setPusherMode(PhoenixShooterRobot.PusherMode.SHOOT);
 *         } else if (gamepads.p1().x().wasPressed()) {
 *             robot.setTransferMode(PhoenixShooterRobot.TransferMode.OFF);
 *             robot.setPusherMode(PhoenixShooterRobot.PusherMode.RETRACT);
 *         }
 *
 *         robot.update();
 *     }
 * }
 * }</pre>
 *
 * <h2>Usage from Autonomous</h2>
 *
 * <p>Autonomous code can reuse the same wiring and plants:</p>
 *
 * <ul>
 *   <li>Drive tasks use {@link #getDrivebase()}.</li>
 *   <li>Shooter/transfer/pusher tasks use the {@link Plant}s exposed by
 *       {@link #getShooterPlant()}, {@link #getTransferPlant()},
 *       {@link #getPusherPlant()}.</li>
 *   <li>A single {@link PhoenixShooterRobot} instance is created in {@code init()}
 *       and reused throughout auto.</li>
 * </ul>
 */
public final class PhoenixShooterRobot {

    // ----------------------------------------------------------------------
    // Hardware names (match these to your FTC Robot Configuration)
    // ----------------------------------------------------------------------

    // Drive motors (used by Drives.mecanum via its defaults).
    // DEFAULT_* names live in the Drives class Javadoc; we rely on those defaults.

    // Shooter motors
    private static final String HW_SHOOTER_LEFT  = "shooterLeftMotor";
    private static final String HW_SHOOTER_RIGHT = "shooterRightMotor";

    // Transfer CR servos
    private static final String HW_TRANSFER_LEFT  = "transferLeftServo";
    private static final String HW_TRANSFER_RIGHT = "transferRightServo";

    // Pusher positional servo
    private static final String HW_PUSHER = "pusherServo";

    // ----------------------------------------------------------------------
    // Tuning constants (example values; teams will tune these)
    // ----------------------------------------------------------------------

    // Shooter velocity in native units (ticks/sec). This is just a placeholder.
    private static final double SHOOTER_VELOCITY_NATIVE = 2200.0;
    private static final double SHOOTER_VELOCITY_TOLERANCE_NATIVE = 100.0;

    // Transfer power levels for different actions.
    private static final double TRANSFER_POWER_LOAD  = 0.3;
    private static final double TRANSFER_POWER_SHOOT = 0.7;

    // Pusher servo positions (0..1 normalized).
    private static final double PUSHER_POS_RETRACT = 0.0;
    private static final double PUSHER_POS_LOAD    = 0.3;
    private static final double PUSHER_POS_SHOOT   = 0.6;

    // ----------------------------------------------------------------------
    // Framework plumbing
    // ----------------------------------------------------------------------

    private final LoopClock clock = new LoopClock();

    // Drive
    private final MecanumDrivebase drivebase;

    // Mechanisms as Plants
    private final Plant shooter;   // velocity plant for 2 shooter motors
    private final Plant transfer;  // power plant for 2 CR servos
    private final Plant pusher;    // position plant for servo

    // Simple state flags to reflect high-level intent for TeleOp/Auto.
    private boolean shooterEnabled = false;
    private TransferMode transferMode = TransferMode.OFF;
    private PusherMode pusherMode = PusherMode.RETRACT;

    // ----------------------------------------------------------------------
    // Public enums to describe "modes" at the mechanism level.
    // TeleOp can set these directly based on driver inputs.
    // ----------------------------------------------------------------------

    /**
     * High-level transfer behavior requested by the driver or autonomous.
     */
    public enum TransferMode {
        OFF,
        LOAD,
        SHOOT
    }

    /**
     * High-level pusher behavior requested by the driver or autonomous.
     */
    public enum PusherMode {
        RETRACT,
        LOAD,
        SHOOT
    }

    // ----------------------------------------------------------------------
    // Construction
    // ----------------------------------------------------------------------

    /**
     * Create a PhoenixShooterRobot and wire all hardware into Plants and
     * a mecanum drivebase.
     *
     * <p>This is the only place we touch {@link HardwareMap} and motor/servo
     * names for this robot. TeleOp and Auto code stay focused on behavior.</p>
     *
     * @param hw FTC hardware map
     */
    public PhoenixShooterRobot(HardwareMap hw) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }

        // 1) Drive: use the beginner-friendly helper with default names.
        this.drivebase = Drives.mecanum(hw);

        // 2) Shooter: two motors, velocity-controlled pair.
        //
        //    - motorPair(...) chooses left/right motors and inversion
        //    - velocity(...) chooses native velocity control with a tolerance
        this.shooter = Actuators.plant(hw)
                .motorPair(HW_SHOOTER_LEFT, false,
                        HW_SHOOTER_RIGHT, true)
                .velocity(SHOOTER_VELOCITY_TOLERANCE_NATIVE)
                .build();

        // 3) Transfer: two CR servos, power-controlled pair.
        this.transfer = Actuators.plant(hw)
                .crServoPair(HW_TRANSFER_LEFT, false,
                        HW_TRANSFER_RIGHT, true)
                .power()
                .build();

        // 4) Pusher: single positional servo, 0..1 position plant.
        this.pusher = Actuators.plant(hw)
                .servo(HW_PUSHER, false)
                .position()
                .build();
    }

    // ----------------------------------------------------------------------
    // Clock management
    // ----------------------------------------------------------------------

    /**
     * Reset the internal clock to a given runtime value.
     *
     * <p>Call this from {@code OpMode.start()} with {@code getRuntime()}.</p>
     */
    public void resetClock(double runtimeSec) {
        clock.reset(runtimeSec);
    }

    /**
     * Update the internal clock based on the current runtime.
     *
     * <p>Call this once per loop from your OpMode's {@code loop()} or
     * LinearOpMode's main loop.</p>
     */
    public void updateClock(double runtimeSec) {
        clock.update(runtimeSec);
    }

    /**
     * @return the internal {@link LoopClock}, in case callers need it.
     */
    public LoopClock getClock() {
        return clock;
    }

    // ----------------------------------------------------------------------
    // Drive control
    // ----------------------------------------------------------------------

    /**
     * Set the desired drive signal (axial, lateral, omega).
     *
     * <p>Typical usage is to compute a {@link DriveSignal} from sticks using
     * {@code StickDriveSource.teleOpMecanumStandard(gamepads)} and then
     * pass it here.</p>
     *
     * @param signal desired drive command; will be applied immediately
     */
    public void setDriveSignal(DriveSignal signal) {
        if (signal == null) {
            // Treat null as "stop".
            drivebase.stop();
            return;
        }
        drivebase.drive(signal);
    }

    /**
     * Access to the drivebase, for use in autonomous drive tasks.
     */
    public MecanumDrivebase getDrivebase() {
        return drivebase;
    }

    // ----------------------------------------------------------------------
    // Shooter control
    // ----------------------------------------------------------------------

    /**
     * Enable or disable the shooter at a fixed velocity setpoint.
     *
     * <p>TeleOp code typically maps a trigger or button to this flag.
     * Autonomous code might use this together with {@link #getShooterPlant()}
     * to wait for {@link Plant#atSetpoint()}.</p>
     */
    public void setShooterEnabled(boolean enabled) {
        this.shooterEnabled = enabled;
    }

    /**
     * @return whether the shooter is currently requested to run.
     */
    public boolean isShooterEnabled() {
        return shooterEnabled;
    }

    /**
     * Access to the underlying shooter plant, for use in tasks/macros.
     */
    public Plant getShooterPlant() {
        return shooter;
    }

    // ----------------------------------------------------------------------
    // Transfer control
    // ----------------------------------------------------------------------

    /**
     * Set the high-level transfer mode.
     *
     * <p>Driver can choose mode based on context:</p>
     * <ul>
     *   <li>{@link TransferMode#LOAD}: move the ball gently toward shooter.</li>
     *   <li>{@link TransferMode#SHOOT}: run faster to feed ball into shooter.</li>
     *   <li>{@link TransferMode#OFF}: stop the transfer.</li>
     * </ul>
     */
    public void setTransferMode(TransferMode mode) {
        if (mode == null) {
            mode = TransferMode.OFF;
        }
        this.transferMode = mode;
    }

    /**
     * @return current requested transfer mode.
     */
    public TransferMode getTransferMode() {
        return transferMode;
    }

    /**
     * Access to the underlying transfer plant.
     */
    public Plant getTransferPlant() {
        return transfer;
    }

    // ----------------------------------------------------------------------
    // Pusher control
    // ----------------------------------------------------------------------

    /**
     * Set the high-level pusher mode.
     *
     * <p>Driver or auto decides which position the pusher should be in:</p>
     * <ul>
     *   <li>{@link PusherMode#RETRACT}: fully out of the way.</li>
     *   <li>{@link PusherMode#LOAD}: ready to hold/position the next ball.</li>
     *   <li>{@link PusherMode#SHOOT}: push the ball into the shooter wheels.</li>
     * </ul>
     */
    public void setPusherMode(PusherMode mode) {
        if (mode == null) {
            mode = PusherMode.RETRACT;
        }
        this.pusherMode = mode;
    }

    /**
     * @return current requested pusher mode.
     */
    public PusherMode getPusherMode() {
        return pusherMode;
    }

    /**
     * Access to the underlying pusher plant.
     */
    public Plant getPusherPlant() {
        return pusher;
    }

    // ----------------------------------------------------------------------
    // Main update: apply modes to plants and update drivebase/plants
    // ----------------------------------------------------------------------

    /**
     * Update the robot for the current loop.
     *
     * <p>Call this once per loop after you have:</p>
     * <ol>
     *   <li>Updated the clock via {@link #updateClock(double)}.</li>
     *   <li>Called {@link #setDriveSignal(DriveSignal)}.</li>
     *   <li>Adjusted shooter/transfer/pusher modes as needed.</li>
     * </ol>
     *
     * <p>This method:</p>
     * <ul>
     *   <li>Applies the requested modes to the underlying {@link Plant}s.</li>
     *   <li>Calls {@link MecanumDrivebase#update(LoopClock)}.</li>
     *   <li>Calls {@link Plant#update(double)} on each plant, which is
     *       cheap but future-proof (for rate-limited or more complex plants).</li>
     * </ul>
     */
    public void update() {
        double dtSec = clock.dtSec();

        // 1) Shooter target
        if (shooterEnabled) {
            shooter.setTarget(SHOOTER_VELOCITY_NATIVE);
        } else {
            shooter.setTarget(0.0);
        }
        shooter.update(dtSec);

        // 2) Transfer target
        switch (transferMode) {
            case LOAD:
                transfer.setTarget(TRANSFER_POWER_LOAD);
                break;
            case SHOOT:
                transfer.setTarget(TRANSFER_POWER_SHOOT);
                break;
            case OFF:
            default:
                transfer.setTarget(0.0);
                break;
        }
        transfer.update(dtSec);

        // 3) Pusher target
        switch (pusherMode) {
            case LOAD:
                pusher.setTarget(PUSHER_POS_LOAD);
                break;
            case SHOOT:
                pusher.setTarget(PUSHER_POS_SHOOT);
                break;
            case RETRACT:
            default:
                pusher.setTarget(PUSHER_POS_RETRACT);
                break;
        }
        pusher.update(dtSec);

        // 4) Drivebase (already given a DriveSignal in setDriveSignal()).
        drivebase.update(clock);
    }
}
