package edu.ftcphoenix.fw.tools.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTasks;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.debug.NullDebugSink;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.task.ParallelAllTask;
import edu.ftcphoenix.fw.task.SequenceTask;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskRunner;

/**
 * <h1>Example 03: Shooter Macro (Tasks + PlantTasks)</h1>
 *
 * <p>This example builds on <b>Example 02: Shooter Basic</b> and shows how
 * to use the {@code fw.task} and {@code PlantTasks} utilities to create a
 * <b>one-button "shoot one ball" macro</b>.</p>
 *
 * <p>High-level behavior of the macro:</p>
 *
 * <ol>
 *   <li>Spin up shooter to a target velocity and wait until
 *       {@link Plant#atSetpoint()} (with a timeout for safety).</li>
 *   <li>Once shooter is ready, in parallel:
 *     <ul>
 *       <li>Run transfer at shoot power for a short pulse.</li>
 *       <li>Move pusher through load → shoot → retract positions.</li>
 *     </ul>
 *   </li>
 *   <li>Spin the shooter back down to 0.</li>
 * </ol>
 * <p>
 * <hr/>
 *
 * <h2>Big picture: what this example teaches</h2>
 *
 * <ol>
 *   <li><b>How to use the task system</b>:
 *     <ul>
 *       <li>{@link Task} – small non-blocking actions.</li>
 *       <li>{@link TaskRunner} – run/update tasks each loop.</li>
 *       <li>{@link SequenceTask} – run tasks one after another.</li>
 *       <li>{@link ParallelAllTask} – run tasks in parallel until all finish.</li>
 *     </ul>
 *   </li>
 *   <li><b>How to use {@link PlantTasks}</b> to create plant-related tasks:
 *     <ul>
 *       <li>{@link PlantTasks#moveTo(Plant, double, double)}
 *           – set a target and wait until {@code atSetpoint()} (with timeout).</li>
 *       <li>{@link PlantTasks#holdFor(Plant, double, double)}
 *           and the overload with a final target – hold a value for a fixed
 *           time and then go to a final value.</li>
 *       <li>{@link PlantTasks#setInstant(Plant, double)}
 *           – set a target once and finish immediately.</li>
 *     </ul>
 *   </li>
 *   <li><b>How macros interact with the main loop</b>:
 *     <ul>
 *       <li>Macros only call {@link Plant#setTarget(double)}.</li>
 *       <li>The main loop is still responsible for calling
 *           {@link Plant#update(double)} once per loop.</li>
 *       <li>When no macro is active, we apply a simple “safe default” for
 *           the shooter/transfer/pusher.</li>
 *     </ul>
 *   </li>
 * </ol>
 * <p>
 * <hr/>
 *
 * <h2>Hardware assumed (same as Example 02)</h2>
 *
 * <ul>
 *   <li>Drive: mecanum with default motor names used by {@link FtcDrives#mecanum}.</li>
 *   <li>Shooter:
 *     <ul>
 *       <li>{@code "shooterLeftMotor"}</li>
 *       <li>{@code "shooterRightMotor"}</li>
 *     </ul>
 *   </li>
 *   <li>Transfer (continuous servos):
 *     <ul>
 *       <li>{@code "transferLeftServo"}</li>
 *       <li>{@code "transferRightServo"}</li>
 *     </ul>
 *   </li>
 *   <li>Pusher (positional servo):
 *     <ul>
 *       <li>{@code "pusherServo"}</li>
 *     </ul>
 *   </li>
 * </ul>
 * <p>
 * <hr/>
 *
 * <h2>Driver controls (suggested)</h2>
 *
 * <ul>
 *   <li><b>Drive</b> – identical to Examples 01–02:
 *     <ul>
 *       <li>P1 left stick X/Y: strafe + forward/back.</li>
 *       <li>P1 right stick X: rotate.</li>
 *       <li>P1 right bumper: slow mode.</li>
 *     </ul>
 *   </li>
 *   <li><b>Macros</b>:
 *     <ul>
 *       <li>P1 Y: run “shoot one ball” macro.</li>
 *       <li>P1 B: cancel any shooting macro and stop the mechanism.</li>
 *     </ul>
 *   </li>
 * </ul>
 * <p>
 * <hr/>
 *
 * <h2>What this example does NOT cover yet</h2>
 *
 * <ul>
 *   <li>No interpolation table yet (still a single shooter velocity).</li>
 *   <li>No vision / AprilTag integration yet.</li>
 *   <li>No multi-ball macro – but you can build “shoot N balls” by repeating
 *       the single-ball macro in a {@link SequenceTask}.</li>
 * </ul>
 */
@TeleOp(name = "FW Ex 03: Shooter Macro", group = "Framework Examples")
@Disabled
public final class TeleOp_03_ShooterMacro extends OpMode {

    // ----------------------------------------------------------------------
    // Hardware names (match your Robot Configuration)
    // ----------------------------------------------------------------------

    private static final String HW_SHOOTER_LEFT = "shooterLeftMotor";
    private static final String HW_SHOOTER_RIGHT = "shooterRightMotor";

    private static final String HW_TRANSFER_LEFT = "transferLeftServo";
    private static final String HW_TRANSFER_RIGHT = "transferRightServo";

    private static final String HW_PUSHER = "pusherServo";

    // ----------------------------------------------------------------------
    // Tuning constants (example values; teams should tune these)
    // ----------------------------------------------------------------------

    // Shooter velocity in native units (e.g., ticks/sec).
    private static final double SHOOTER_VELOCITY_NATIVE = 2200.0;

    // Allowed error band for setpoint-based tasks (native units).
    private static final double SHOOTER_VELOCITY_TOLERANCE_NATIVE = 100.0;

    // Time to wait for shooter to reach setpoint before giving up (seconds).
    private static final double SHOOTER_SPINUP_TIMEOUT_SEC = 1.5;

    // Optional small delay before spinning down (seconds) – can be 0.0.
    private static final double SHOOTER_SPINDOWN_HOLD_SEC = 0.2;

    // Transfer power level for shooting.
    private static final double TRANSFER_POWER_SHOOT = 0.7;

    // Duration of the transfer pulse while shooting (seconds).
    private static final double TRANSFER_PULSE_SEC = 0.3;

    // Pusher positions (0..1).
    private static final double PUSHER_POS_RETRACT = 0.0;
    private static final double PUSHER_POS_LOAD = 0.3;
    private static final double PUSHER_POS_SHOOT = 0.6;

    // Duration to hold each pusher stage (seconds).
    private static final double PUSHER_STAGE_SEC = 0.2;

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
    private Plant shooter;
    private Plant transfer;
    private Plant pusher;

    // Macro runner for shooting
    private final TaskRunner macroRunner = new TaskRunner();

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

        // === 2) Drive wiring (same pattern as Examples 01–02) ===
        drivebase = FtcDrives.mecanum(hardwareMap);
        stickDrive = GamepadDriveSource.teleOpMecanumSlowRb(gamepads);

        // === 3) Mechanism wiring using Actuators ===

        shooter = Actuators.plant(hardwareMap)
                .motor(HW_SHOOTER_LEFT, Direction.FORWARD)
                .andMotor(HW_SHOOTER_RIGHT, Direction.REVERSE)
                .velocity(SHOOTER_VELOCITY_TOLERANCE_NATIVE)
                .build();

        transfer = Actuators.plant(hardwareMap)
                .crServo(HW_TRANSFER_LEFT, Direction.FORWARD)
                .andCrServo(HW_TRANSFER_RIGHT, Direction.REVERSE)
                .power()
                .build();

        pusher = Actuators.plant(hardwareMap)
                .servo(HW_PUSHER, Direction.FORWARD)
                .position()
                .build();

        // Initialize mechanisms to a safe default.
        shooter.setTarget(0.0);
        transfer.setTarget(0.0);
        pusher.setTarget(PUSHER_POS_RETRACT);

        // === 4) Bindings: hook buttons to macro actions ===

        // Y → start "shoot one ball" macro.
        bindings.onPress(
                gamepads.p1().y(),
                this::startShootOneBallMacro
        );

        // B → cancel macros and stop mechanism.
        bindings.onPress(
                gamepads.p1().b(),
                this::cancelShootMacros
        );

        telemetry.addLine("FW Example 03: Shooter Macro");
        telemetry.addLine("Drive: same as Examples 01–02");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Y = shoot one ball (macro)");
        telemetry.addLine("  B = cancel macro + stop shooter");
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

        // --- 2) Inputs + bindings ---
        gamepads.update(clock);
        bindings.update(clock);

        // --- 3) Macros (shooter/transfer/pusher) ---
        macroRunner.update(clock);

        // When no macro is active, hold a safe default state.
        if (!macroRunner.hasActiveTask()) {
            shooter.setTarget(0.0);
            transfer.setTarget(0.0);
            pusher.setTarget(PUSHER_POS_RETRACT);
        }

        // --- 4) Drive: always under manual control ---
        DriveSignal driveCmd = stickDrive.get(clock).clamped();
        lastDrive = driveCmd;

        drivebase.update(clock);
        drivebase.drive(driveCmd);

        // --- 5) Mechanism updates ---
        shooter.update(dtSec);
        transfer.update(dtSec);
        pusher.update(dtSec);
        // --- 6) Required telemetry (driver-facing) ---
        // This is the information you should not lose even when debug output is disabled.
        telemetry.addLine("FW Example 03: Shooter Macro");
        telemetry.addData("drive.cmd", driveCmd);
        telemetry.addData("macro.active", macroRunner.hasActiveTask());
        telemetry.addData("macro.queued", macroRunner.queuedCount());
        telemetry.addData("shooter.targetNative", shooter.getTarget());
        telemetry.addData("shooter.atSetpoint", shooter.atSetpoint());
        telemetry.addData("transfer.target", transfer.getTarget());
        telemetry.addData("pusher.target", pusher.getTarget());
        telemetry.addData("debug.enabled", DEBUG);

        // --- 7) Optional debug (can be disabled without breaking required telemetry) ---
        // Use DebugSink + debugDump() for verbose internal state.
        if (DEBUG) {
            dbg.addLine("--- debugDump() (optional) ---");

            clock.debugDump(dbg, "clock");
            gamepads.debugDump(dbg, "pads");
            bindings.debugDump(dbg, "bindings");

            macroRunner.debugDump(dbg, "macro");

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
        cancelShootMacros();
        drivebase.stop();
    }

    // ----------------------------------------------------------------------
    // Macro helpers
    // ----------------------------------------------------------------------

    /**
     * Start a "shoot one ball" macro, if none is currently running.
     */
    private void startShootOneBallMacro() {
        if (macroRunner.hasActiveTask()) {
            // Already running a macro; ignore new request, or queue another if desired.
            return;
        }

        Task macro = buildShootOneBallMacro();
        macroRunner.clear();
        macroRunner.enqueue(macro);
    }

    /**
     * Cancel any running/queued shooting macros and stop the mechanism.
     */
    private void cancelShootMacros() {
        macroRunner.clear();
        shooter.setTarget(0.0);
        transfer.setTarget(0.0);
        pusher.setTarget(PUSHER_POS_RETRACT);
    }

    /**
     * Build a macro that:
     *
     * <ol>
     *   <li>Spins up the shooter and waits for atSetpoint (with timeout).</li>
     *   <li>Feeds one ball using transfer + pusher in parallel.</li>
     *   <li>Spins the shooter down to 0.</li>
     * </ol>
     */
    private Task buildShootOneBallMacro() {
        // Step 1: set shooter target and wait for atSetpoint() or timeout.
        Task spinUp = PlantTasks.moveTo(
                shooter,
                SHOOTER_VELOCITY_NATIVE,
                SHOOTER_SPINUP_TIMEOUT_SEC
        );

        // Step 2: feed one ball.
        //
        //  - Transfer runs at shoot power for TRANSFER_PULSE_SEC, then stops.
        //  - Pusher steps through LOAD → SHOOT → RETRACT positions.
        Task feedTransfer = PlantTasks.holdFor(
                transfer,
                TRANSFER_POWER_SHOOT,
                TRANSFER_PULSE_SEC
        );

        Task pusherLoad = PlantTasks.holdFor(
                pusher,
                PUSHER_POS_LOAD,
                PUSHER_STAGE_SEC
        );

        Task pusherShoot = PlantTasks.holdForThen(
                pusher,
                PUSHER_POS_SHOOT,
                PUSHER_STAGE_SEC,
                PUSHER_POS_RETRACT
        );

        Task feedPusher = SequenceTask.of(
                pusherLoad,
                pusherShoot
        );

        Task feedBoth = ParallelAllTask.of(
                feedTransfer,
                feedPusher
        );

        // Step 3: optionally hold shooter briefly, then spin down to 0.
        Task holdBeforeSpinDown = PlantTasks.holdFor(
                shooter,
                SHOOTER_VELOCITY_NATIVE,
                SHOOTER_SPINDOWN_HOLD_SEC
        );

        Task spinDown = PlantTasks.setInstant(shooter, 0.0);

        return SequenceTask.of(
                spinUp,
                feedBoth,
                holdBeforeSpinDown,
                spinDown
        );
    }
}
