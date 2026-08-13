package edu.ftcphoenix.fw.tools.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.debug.NullDebugSink;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * <h2>Example 02: Shooter + Transfer + Pusher (Basic TeleOp)</h2>
 *
 * <p>This example builds directly on
 * <b>Example 01: Mecanum Basic</b> and shows how to control a simple
 * scoring mechanism:</p>
 *
 * <p><b>Ownership note:</b> This disabled class is an intentional one-OpMode API lesson.
 * Ordinary structured robot code follows the Modern Starter pattern: the composition root calls
 * {@code new Mechanism(hardwareMap, profile.mechanism)}, and that mechanism constructs and
 * privately owns its Plants and their update/stop lifecycle.</p>
 *
 * <ul>
 *   <li>2 shooter motors (velocity-controlled pair).</li>
 *   <li>2 continuous-rotation transfer servos (power-controlled pair).</li>
 *   <li>1 positional pusher servo.</li>
 * </ul>
 *
 * <p>It keeps the <b>same driving code</b> and loop structure as Example 01,
 * and adds the mechanism control on top.</p>
 * <hr/>
 *
 * <h2>Big picture: what this example teaches</h2>
 *
 * <ol>
 *   <li><b>How to wire mechanisms using beginner helpers</b>:
 *     <ul>
 *       <li>{@link FtcActuators#plant} to turn hardware into {@link Plant}s.</li>
 *       <li>{@code motor(...).andMotor(...).velocity().deviceManaged()}
 *           {@code .bounded(...).nativeUnits().velocityTolerance(...)}
 *           {@code .targetFromNewCommand(0.0).build()} for the shooter.</li>
 *       <li>{@code crServo(...).andCrServo(...).power()}
 *           {@code .targetFromNewCommand(0.0).build()} for the transfer.</li>
 *       <li>{@code servo(...).position().nonPeriodic().bounded(0.0, 1.0).nativeUnits()}
 *           {@code .targetFromNewCommand(PUSHER_POS_RETRACT).build()} for the pusher.</li>
 *     </ul>
 *   </li>
 *   <li><b>How to map buttons to simple modes</b> using
 *       {@link Bindings} with Java 8 lambdas:
 *     <ul>
 *       <li>Toggle shooter on/off.</li>
 *       <li>Set transfer/pusher to LOAD / SHOOT / RETRACT modes.</li>
 *     </ul>
 *   </li>
 *   <li><b>How to keep the same loop pattern</b> from Example 01 while
 *       adding more behavior:
 *     <ol>
 *       <li>Update the {@link LoopClock}.</li>
 *       <li>Update inputs ({@link Gamepads}, {@link Bindings}).</li>
 *       <li>Compute a {@link DriveSignal} from sticks.</li>
 *       <li>Convert modes → targets for {@link Plant}s.</li>
 *       <li>Call {@link MecanumDrivebase#drive(DriveSignal)}, and call
 *           {@link Plant#update(LoopClock)}.</li>
 *       <li>Send telemetry.</li>
 *     </ol>
 *   </li>
 * </ol>
 * <hr/>
 *
 * <h2>Hardware assumed for this example</h2>
 *
 * <p>In addition to the mecanum drive motors from Example 01, we assume:</p>
 *
 * <ul>
 *   <li>{@code "shooterLeftMotor"} and {@code "shooterRightMotor"} –
 *       DC motors used as a velocity pair.</li>
 *   <li>{@code "transferLeftServo"} and {@code "transferRightServo"} –
 *       continuous-rotation servos that move balls toward the shooter.</li>
 *   <li>{@code "pusherServo"} – positional servo that pushes the ball
 *       into the shooter wheels.</li>
 * </ul>
 *
 * <p>You can change these names to match your FTC Robot Configuration, but
 * the structure stays the same.</p>
 * <hr/>
 *
 * <h2>Driver controls (suggested)</h2>
 *
 * <ul>
 *   <li><b>Drive</b> (same as Example 01):
 *     <ul>
 *       <li>P1 left stick X: strafe left/right (lateral)</li>
 *       <li>P1 left stick Y: drive forward/back (axial)</li>
 *       <li>P1 right stick X: rotate left/right (omega)</li>
 *       <li>P1 right bumper: slow mode</li>
 *     </ul>
 *   </li>
 *   <li><b>Shooter / mechanism</b>:
 *     <ul>
 *       <li>P1 left bumper: toggle shooter on/off.</li>
 *       <li>P1 A: "LOAD" – gentle transfer, pusher in load position.</li>
 *       <li>P1 B: "SHOOT" – faster transfer, pusher in shoot position.</li>
 *       <li>P1 X: "RETRACT" – stop transfer, retract pusher.</li>
 *     </ul>
 *   </li>
 * </ul>
 * <hr/>
 *
 * <h2>What this example <b>does not</b> include yet</h2>
 *
 * <ul>
 *   <li>No macros or sequences – each button press just sets modes.
 *       Later examples will show "press once to shoot N balls".</li>
 *   <li>No interpolation table – the shooter runs at a single fixed
 *       speed here. A later example will use a distance→velocity table
 *       (and vision) to choose speeds.</li>
 *   <li>No vision / AprilTag – this example is purely driver-operated.
 *       Later examples will:
 *       <ul>
 *         <li>Auto-aim toward a tag.</li>
 *         <li>Move to a position relative to a tag.</li>
 *         <li>Use tag distance to pick shooter velocity.</li>
 *       </ul>
 *   </li>
 * </ul>
 * <hr/>
 *
 * <h2>Reading order for examples</h2>
 *
 * <ol>
 *   <li><b>Example 01: Mecanum Basic</b> – pure driving.</li>
 *   <li><b>Example 02: Shooter Basic</b> (this file) – add shooter, transfer,
 *       pusher using {@link FtcActuators} + {@link Plant}s + {@link Bindings}.</li>
 *   <li>Next examples will add:
 *     <ul>
 *       <li>Macros and tasks for one-button shooting sequences.</li>
 *       <li>An interpolation table for shooter velocity.</li>
 *       <li>Vision (AprilTag) for auto-aim and distance-based velocity.</li>
 *     </ul>
 *   </li>
 * </ol>
 */
@TeleOp(name = "FW Ex 02: Shooter Basic", group = "Framework Examples")
@Disabled
public final class TeleOp_02_ShooterBasic extends OpMode {

    // ----------------------------------------------------------------------
    // Hardware names for this example (match your Robot Configuration)
    // ----------------------------------------------------------------------

    // Shooter motors
    private static final String HW_SHOOTER_LEFT = "shooterLeftMotor";
    private static final String HW_SHOOTER_RIGHT = "shooterRightMotor";

    // Transfer CR servos
    private static final String HW_TRANSFER_LEFT = "transferLeftServo";
    private static final String HW_TRANSFER_RIGHT = "transferRightServo";

    // Pusher positional servo
    private static final String HW_PUSHER = "pusherServo";

    // ----------------------------------------------------------------------
    // Tuning constants (example values; teams should tune these)
    // ----------------------------------------------------------------------

    /**
     * Shooter velocity in native units (e.g., encoder ticks per second).
     * This is a placeholder – teams should tune this based on testing.
     */
    private static final double SHOOTER_VELOCITY_NATIVE = 2200.0;

    /**
     * Allowed error band (in native units) when using velocity control.
     */
    private static final double SHOOTER_VELOCITY_TOLERANCE_NATIVE = 100.0;

    // Transfer power levels
    private static final double TRANSFER_POWER_LOAD = 0.3;
    private static final double TRANSFER_POWER_SHOOT = 0.7;

    // Pusher servo positions (0..1 normalized)
    private static final double PUSHER_POS_RETRACT = 0.0;
    private static final double PUSHER_POS_LOAD = 0.3;
    private static final double PUSHER_POS_SHOOT = 0.6;

    // ----------------------------------------------------------------------
    // Simple enums to describe mechanism "modes"
    // ----------------------------------------------------------------------

    /**
     * High-level transfer behavior requested by the driver.
     */
    private enum TransferMode {
        OFF,
        LOAD,
        SHOOT
    }

    /**
     * High-level pusher behavior requested by the driver.
     */
    private enum PusherMode {
        RETRACT,
        LOAD,
        SHOOT
    }

    // ----------------------------------------------------------------------
    // Framework plumbing
    // ----------------------------------------------------------------------

    private final LoopClock clock = new LoopClock();

    // Debug/telemetry adapter (DebugSink -> FTC Telemetry)
    private static final boolean DEBUG = true;
    private DebugSink dbg = NullDebugSink.INSTANCE;


    private Gamepads gamepads;
    private Bindings bindings;

    private MecanumDrivebase drivebase;
    private DriveSource stickDrive;

    // Mechanism plants
    private Plant shooter;   // velocity plant for shooter motors
    private Plant transfer;  // power plant for transfer CR servos
    private Plant pusher;    // position plant for pusher servo

    // Requested states (controlled via bindings)
    private boolean shooterEnabled = false;
    private TransferMode transferMode = TransferMode.OFF;
    private PusherMode pusherMode = PusherMode.RETRACT;

    // For telemetry
    private DriveSignal lastDrive = DriveSignal.zero();

    // ----------------------------------------------------------------------
    // OpMode lifecycle
    // ----------------------------------------------------------------------

    /**
     * {@inheritDoc}
     */
    @Override
    public void init() {
        dbg = DEBUG ? new FtcTelemetryDebugSink(telemetry) : NullDebugSink.INSTANCE;

        // === 1) Inputs ===
        gamepads = Gamepads.create(gamepad1, gamepad2);
        bindings = new Bindings();

        // === 2) Drive wiring (same as Example 01) ===
        drivebase = FtcDrives.mecanum(hardwareMap);
        stickDrive = new GamepadDriveSource(
                gamepads.p1().leftX(),
                gamepads.p1().leftY(),
                gamepads.p1().rightX(),
                GamepadDriveSource.Config.defaults()
        ).scaledWhen(gamepads.p1().rightBumper(), 0.35, 0.20)
                .rateLimited(4.0, 4.0, 6.0);

        // === 3) Mechanism wiring using FtcActuators ===

        // Shooter: two motors, velocity-controlled pair.
        shooter = FtcActuators.plant(hardwareMap)
                .motor(HW_SHOOTER_LEFT, Direction.FORWARD)
                .andMotor(HW_SHOOTER_RIGHT, Direction.REVERSE)
                .velocity()
                .deviceManaged()
                .bounded(0.0, SHOOTER_VELOCITY_NATIVE)
                .nativeUnits()
                .velocityTolerance(SHOOTER_VELOCITY_TOLERANCE_NATIVE)
                .targetFromNewCommand(0.0)
                .build();

        // Transfer: two CR servos, power-controlled pair.
        transfer = FtcActuators.plant(hardwareMap)
                .crServo(HW_TRANSFER_LEFT, Direction.FORWARD)
                .andCrServo(HW_TRANSFER_RIGHT, Direction.REVERSE)
                .power()
                .targetFromNewCommand(0.0)
                .build();

        // Pusher: single positional servo, 0..1 position plant.
        pusher = FtcActuators.plant(hardwareMap)
                .servo(HW_PUSHER, Direction.FORWARD)
                .position()
                .nonPeriodic()
                .bounded(0.0, 1.0)
                .nativeUnits()
                .targetFromNewCommand(PUSHER_POS_RETRACT)
                .build();

        // === 4) Bindings: map buttons to high-level modes (using lambdas) ===

        // Toggle shooter on/off with left bumper.
        bindings.toggleOnRise(
                gamepads.p1().leftBumper(),
                () -> shooterEnabled = true,
                () -> shooterEnabled = false
        );

        // A = LOAD: gentle transfer + pusher in load position.
        bindings.onRise(
                gamepads.p1().a(),
                () -> {
                    transferMode = TransferMode.LOAD;
                    pusherMode = PusherMode.LOAD;
                }
        );

        // B = SHOOT: faster transfer + pusher in shoot position.
        bindings.onRise(
                gamepads.p1().b(),
                () -> {
                    transferMode = TransferMode.SHOOT;
                    pusherMode = PusherMode.SHOOT;
                }
        );

        // X = RETRACT: stop transfer + retract pusher.
        bindings.onRise(
                gamepads.p1().x(),
                () -> {
                    transferMode = TransferMode.OFF;
                    pusherMode = PusherMode.RETRACT;
                }
        );

        telemetry.addLine("FW Example 02: Shooter Basic");
        telemetry.addLine("Drive: same as Example 01");
        telemetry.addLine("Controls:");
        telemetry.addLine("  LB = toggle shooter");
        telemetry.addLine("  RB = slow mode (drive)");
        telemetry.addLine("  A = LOAD, B = SHOOT, X = RETRACT");
        telemetry.update();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void start() {
        clock.reset(getRuntime());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void loop() {
        // --- 1) Clock ---
        clock.update(getRuntime());
        double dtSec = clock.dtSec();

        // --- 2) Inputs (bindings) ---
        // Gamepad axes/buttons are Sources; they are sampled when you call get(...).
        bindings.update(clock);

        // --- 3) Drive logic: sticks → drive signal ---
        DriveSignal driveCmd = stickDrive.get(clock).clamped();
        lastDrive = driveCmd;

        // --- 4) Mechanism logic: map modes to plant targets ---

        // Shooter
        shooter.commandTarget().set(shooterEnabled ? SHOOTER_VELOCITY_NATIVE : 0.0);

        // Transfer
        switch (transferMode) {
            case LOAD:
                transfer.commandTarget().set(TRANSFER_POWER_LOAD);
                break;
            case SHOOT:
                transfer.commandTarget().set(TRANSFER_POWER_SHOOT);
                break;
            case OFF:
            default:
                transfer.commandTarget().set(0.0);
                break;
        }

        // Pusher
        switch (pusherMode) {
            case LOAD:
                pusher.commandTarget().set(PUSHER_POS_LOAD);
                break;
            case SHOOT:
                pusher.commandTarget().set(PUSHER_POS_SHOOT);
                break;
            case RETRACT:
            default:
                pusher.commandTarget().set(PUSHER_POS_RETRACT);
                break;
        }

        // --- 5) Actuation: drivebase + plants ---

        drivebase.drive(driveCmd);

        shooter.update(clock);
        transfer.update(clock);
        pusher.update(clock);
        // --- 6) Required telemetry (driver-facing) ---
        // This is the information you should not lose even when debug output is disabled.
        telemetry.addLine("FW Example 02: Shooter Basic");
        telemetry.addData("drive.cmd", driveCmd);
        telemetry.addData("shooter.enabled", shooterEnabled);
        telemetry.addData("shooter.targetNative", shooter.getRequestedTarget());
        telemetry.addData("shooter.atTarget", shooter.atTarget());
        telemetry.addData("transfer.mode", transferMode);
        telemetry.addData("transfer.target", transfer.getRequestedTarget());
        telemetry.addData("pusher.mode", pusherMode);
        telemetry.addData("pusher.target", pusher.getRequestedTarget());
        telemetry.addData("debug.enabled", DEBUG);

        // --- 7) Optional debug (can be disabled without breaking required telemetry) ---
        // Use DebugSink + debugDump() for verbose internal state.
        if (DEBUG) {
            dbg.addLine("--- debugDump() (optional) ---");

            clock.debugDump(dbg, "clock");
            gamepads.debugDump(dbg, "pads");
            bindings.debugDump(dbg, "bindings");

            stickDrive.debugDump(dbg, "driveSrc");
            drivebase.debugDump(dbg, "drivebase");

            shooter.debugDump(dbg, "shooter");
            transfer.debugDump(dbg, "transfer");
            pusher.debugDump(dbg, "pusher");
        }

        telemetry.update();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void stop() {
        // Motor/CR-servo Plants zero output immediately. A standard positional-servo Plant holds
        // its last applied position; retract it with bounded cooperative behavior before FTC STOP.
        CleanupActions.attemptAll(
                () -> { if (shooter != null) shooter.stop(); },
                () -> { if (transfer != null) transfer.stop(); },
                () -> { if (pusher != null) pusher.stop(); },
                () -> { if (drivebase != null) drivebase.stop(); });
    }
}
