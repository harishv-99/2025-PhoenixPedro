package edu.ftcphoenix.fw.tools.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTasks;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.debug.NullDebugSink;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.math.InterpolatingTable1D;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidance;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.ftc.FtcVision;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.sensing.observation.ObservationSource2d;
import edu.ftcphoenix.fw.sensing.observation.ObservationSources;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagTarget;
import edu.ftcphoenix.fw.task.ParallelAllTask;
import edu.ftcphoenix.fw.task.SequenceTask;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskRunner;

/**
 * <h1>Example 06: Shooter + DriveGuidance Auto-Aim + Vision Distance + Macro</h1>
 *
 * <p>This example combines:</p>
 *
 * <ol>
 *   <li><b>Mecanum drive</b> via {@link FtcDrives#mecanum} +
 *       {@link GamepadDriveSource#teleOpMecanumSlowRb(Gamepads)}.</li>
 *   <li><b>Tag-based auto-aim</b> via {@link DriveGuidance} (a {@link edu.ftcphoenix.fw.drive.DriveOverlay}) – hold LB to face a scoring tag.</li>
 *   <li><b>Vision-based shooter velocity</b>: AprilTag distance →
 *       {@link InterpolatingTable1D} → shooter velocity (native units).</li>
 *   <li><b>One-button shoot macro</b> (shooter + transfer + pusher)
 *       using {@link TaskRunner} and {@link PlantTasks}.</li>
 * </ol>
 *
 * <h2>Camera offset note</h2>
 * <p>
 * This example uses {@link CameraMountConfig} so the vision observation is expressed relative to
 * the <b>robot</b> (not just the camera), even if the camera is mounted off-center.
 * </p>
 *
 * <h2>Driver behavior</h2>
 *
 * <ul>
 *   <li><b>Drive</b>:
 *     <ul>
 *       <li>P1 left stick X/Y: strafe + forward/back.</li>
 *       <li>P1 right stick X: rotate.</li>
 *       <li>P1 right bumper: slow mode (from stick mapping).</li>
 *       <li>P1 left bumper: <b>hold to auto-aim</b> at scoring tags.</li>
 *     </ul>
 *   </li>
 *   <li><b>Shooter macro</b>:
 *     <ul>
 *       <li>P1 Y: run “shoot one ball” macro:
 *         <ol>
 *           <li>Read current tag distance from {@link TagTarget}.</li>
 *           <li>Look up shooter velocity from {@link #SHOOTER_VELOCITY_TABLE}.</li>
 *           <li>Spin up shooter and wait for atSetpoint (with timeout).</li>
 *           <li>Feed one ball using transfer + pusher in parallel.</li>
 *           <li>Hold briefly, then spin shooter down to 0.</li>
 *         </ol>
 *       </li>
 *       <li>P1 B: cancel macro and stop shooter/transfer/pusher.</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p>If Y is pressed but no valid tag is seen, the macro is not started
 * and a status message is shown in telemetry.</p>
 */
@TeleOp(name = "FW Ex 06: Shooter Guidance Macro Vision", group = "Framework Examples")
@Disabled
public final class TeleOp_06_ShooterTagAimMacroVision extends OpMode {

    // ----------------------------------------------------------------------
    // Calibration: distance (inches) → shooter velocity (native units)
    // ----------------------------------------------------------------------

    /**
     * Example shooter velocity table. Teams should tune these numbers for
     * their actual robot and field setup.
     *
     * <p>x: distance in inches. y: shooter velocity in native units
     * (e.g., ticks/sec). The table clamps outside the range and linearly
     * interpolates between points.</p>
     */
    private static final InterpolatingTable1D SHOOTER_VELOCITY_TABLE =
            InterpolatingTable1D.ofSortedPairs(
                    24.0, 170.0,  // close shot
                    30.0, 180.0,
                    36.0, 195.0,
                    42.0, 210.0,
                    48.0, 225.0   // farther shot
            );

    /**
     * Maximum age (seconds) for a tag observation to be considered valid for
     * shooting decisions.
     */
    private static final double MAX_TAG_AGE_SEC = 0.5;

    // ----------------------------------------------------------------------
    // Tag IDs we care about (example values; adjust per game) – Java 8 style
    // ----------------------------------------------------------------------

    private static final Set<Integer> SCORING_TAG_IDS;

    static {
        HashSet<Integer> ids = new HashSet<Integer>();
        ids.add(1);
        ids.add(2);
        ids.add(3);
        SCORING_TAG_IDS = Collections.unmodifiableSet(ids);
    }

    // ----------------------------------------------------------------------
    // Hardware names (match your Robot Configuration)
    // ----------------------------------------------------------------------

    private static final String HW_SHOOTER_LEFT = "shooterLeftMotor";
    private static final String HW_SHOOTER_RIGHT = "shooterRightMotor";

    private static final String HW_TRANSFER_LEFT = "transferLeftServo";
    private static final String HW_TRANSFER_RIGHT = "transferRightServo";

    private static final String HW_PUSHER = "pusherServo";

    // ----------------------------------------------------------------------
    // Shooter / mechanism tuning (example values)
    // ----------------------------------------------------------------------

    private static final double SHOOTER_VELOCITY_TOLERANCE_NATIVE = 100.0;
    private static final double SHOOTER_SPINUP_TIMEOUT_SEC = 1.5;
    private static final double SHOOTER_SPINDOWN_HOLD_SEC = 0.2;

    private static final double TRANSFER_POWER_SHOOT = 0.7;
    private static final double TRANSFER_PULSE_SEC = 0.3;

    private static final double PUSHER_POS_RETRACT = 0.0;
    private static final double PUSHER_POS_LOAD = 0.3;
    private static final double PUSHER_POS_SHOOT = 0.6;
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
    private DriveSource baseDrive;
    private DriveSource driveWithAim;

    private AprilTagSensor tagSensor;
    private TagTarget scoringTarget;

    private CameraMountConfig cameraMount;

    private Plant shooter;
    private Plant transfer;
    private Plant pusher;

    private final TaskRunner macroRunner = new TaskRunner();

    private DriveSignal lastDrive = DriveSignal.zero();

    // Latest tag observation (for telemetry)
    private boolean lastHasTarget = false;
    private double lastTagRangeInches = 0.0;
    private double lastCameraBearingRad = 0.0;
    private double lastRobotBearingRad = 0.0;
    private double lastTagAgeSec = 0.0;
    private int lastTagId = -1;

    // Macro state for telemetry
    private double lastShooterMacroTargetVel = 0.0;
    private String lastMacroStatus = "idle";

    // ----------------------------------------------------------------------
    // OpMode lifecycle
    // ----------------------------------------------------------------------

    /**
     * {@inheritDoc}
     */
    @Override
    public void init() {
        dbg = DEBUG ? new FtcTelemetryDebugSink(telemetry) : NullDebugSink.INSTANCE;

        // 1) Inputs
        gamepads = Gamepads.create(gamepad1, gamepad2);
        bindings = new Bindings();

        // 2) Drive wiring: mecanum + sticks.
        drivebase = FtcDrives.mecanum(hardwareMap);
        baseDrive = GamepadDriveSource.teleOpMecanumSlowRb(gamepads);

        // 3) Tag sensor: FTC VisionPortal + AprilTagProcessor adapter.
        //
        // NOTE: Replace "Webcam 1" with your actual camera name in the
        // Robot Configuration.
        tagSensor = FtcVision.aprilTags(hardwareMap, "Webcam 1");

        // Track scoring tags with a freshness window.
        scoringTarget = new TagTarget(tagSensor, SCORING_TAG_IDS, MAX_TAG_AGE_SEC);

        // 3b) Camera mount: robot→camera extrinsics (Phoenix axes: +X forward, +Y left, +Z up).
        //
        // IMPORTANT: Update these values for your robot.
        // Example: camera is 6" forward, 3" to the RIGHT (so y = -3), 8" up, facing forward.
        cameraMount = CameraMountConfig.of(
                /*xInches=*/6.0,
                /*yInches=*/-3.0,
                /*zInches=*/8.0,
                /*yawRad=*/0.0,
                /*pitchRad=*/0.0,
                /*rollRad=*/0.0
        );

        // Wrap baseDrive with an auto-aim overlay: hold left bumper to auto-aim omega.
        ObservationSource2d scoringObs = ObservationSources.aprilTag(scoringTarget, cameraMount);
        DriveGuidancePlan aimPlan = DriveGuidance.plan()
                .aimTo()
                .tagCenter()
                .doneAimTo()
                .feedback()
                .observation(scoringObs)
                .doneFeedback()
                .build();

        // Apply the plan as an overlay: the driver keeps translation, and the overlay owns omega.
        driveWithAim = baseDrive.overlayWhen(
                () -> gamepads.p1().leftBumper().isHeld(),
                aimPlan.overlay(),
                DriveOverlayMask.OMEGA_ONLY
        );

        // 4) Mechanism wiring using Actuators.

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

        // Default safe targets when no macro is active.
        shooter.setTarget(0.0);
        transfer.setTarget(0.0);
        pusher.setTarget(PUSHER_POS_RETRACT);

        // 5) Bindings: macro controls.

        // Y → start "shoot one ball" macro using current tag distance.
        bindings.onPress(
                gamepads.p1().y(),
                this::startShootOneBallMacro
        );

        // B → cancel macro and stop mechanism.
        bindings.onPress(
                gamepads.p1().b(),
                this::cancelShootMacros
        );

        telemetry.addLine("FW Example 06: Shooter Guidance Macro Vision");
        telemetry.addLine("Drive: mecanum + (optional) auto-aim overlay (hold LB)");
        telemetry.addLine("Auto-aim: mount-aware (robot frame aims at tag)");
        telemetry.addLine("Shooter macro:");
        telemetry.addLine("  Y = shoot one ball (vision distance)");
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
        // ------------------------------------------------------------------
        // 1) Clock
        // ------------------------------------------------------------------
        clock.update(getRuntime());
        double dtSec = clock.dtSec();

        // ------------------------------------------------------------------
        // 2) Sense (inputs & sensors)
        // ------------------------------------------------------------------
        gamepads.update(clock);        // controller state
        scoringTarget.update(clock);   // AprilTags via TagTarget  (UPDATED)

        // ------------------------------------------------------------------
        // 3) Decide (high-level logic)
        // ------------------------------------------------------------------
        // User-input–driven decisions (may start/cancel macros).
        bindings.update(clock);

        // High-level task/macro logic (sets plant targets over time).
        macroRunner.update(clock);

        // When no macro is active, choose safe default targets.
        if (!macroRunner.hasActiveTask()) {
            shooter.setTarget(0.0);
            transfer.setTarget(0.0);
            pusher.setTarget(PUSHER_POS_RETRACT);
        }

        // ------------------------------------------------------------------
        // 4) Control / Actuate (subsystems)
        // ------------------------------------------------------------------
        // Drive: sticks + (optional) auto-aim overlay (LB may override omega).
        DriveSignal cmd = driveWithAim.get(clock).clamped();
        lastDrive = cmd;

        drivebase.update(clock);
        drivebase.drive(cmd);

        // Plants: shooter, transfer, pusher.
        shooter.update(dtSec);
        transfer.update(dtSec);
        pusher.update(dtSec);

        // ------------------------------------------------------------------
        // 5) Report
        //
        // Important rule: telemetry required for normal operation (driver-facing state) should
        // NOT go through the debug pipeline. Debug is optional and should be safe to disable.

        // --- Required telemetry (driver-facing) ---
        telemetry.addLine("FW Example 06: Shooter Guidance Macro Vision");
        telemetry.addData("aim.active", gamepads.p1().leftBumper().isHeld());
        telemetry.addData("tag.hasTarget", lastHasTarget);
        telemetry.addData("tag.id", lastTagId);
        telemetry.addData("tag.rangeIn", lastTagRangeInches);
        telemetry.addData("tag.ageSec", lastTagAgeSec);
        telemetry.addData("macro.active", macroRunner.hasActiveTask());
        telemetry.addData("macro.queued", macroRunner.queuedCount());
        telemetry.addData("macro.status", lastMacroStatus);
        telemetry.addData("macro.targetVelNative", lastShooterMacroTargetVel);
        telemetry.addData("drive.cmd", cmd);
        telemetry.addData("debug.enabled", DEBUG);

        // --- Optional debug (verbose; can be disabled without losing required telemetry) ---
        if (DEBUG) {
            dbg.addLine("--- debugDump() (optional) ---");

            clock.debugDump(dbg, "clock");
            gamepads.debugDump(dbg, "pads");
            bindings.debugDump(dbg, "bindings");

            scoringTarget.debugDump(dbg, "tag", cameraMount);

            macroRunner.debugDump(dbg, "macro");

            driveWithAim.debugDump(dbg, "driveSrc");
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
        if (tagSensor != null) {
            tagSensor.close();
            tagSensor = null;
        }
        cancelShootMacros();
        drivebase.stop();
    }

    // ----------------------------------------------------------------------
    // Macro helpers
    // ----------------------------------------------------------------------

    /**
     * Start a "shoot one ball" macro, using the current AprilTag distance
     * to pick the shooter velocity from the interpolation table.
     */
    private void startShootOneBallMacro() {
        if (macroRunner.hasActiveTask()) {
            // Already running a macro; ignore or queue another (we choose ignore).
            lastMacroStatus = "ignored: macro already running";
            return;
        }

        AprilTagObservation obs = scoringTarget.last();
        if (!obs.hasTarget) {
            lastMacroStatus = "no tag: macro not started";
            return;
        }

        double shooterTargetVel = SHOOTER_VELOCITY_TABLE.interpolate(obs.cameraRangeInches());
        lastShooterMacroTargetVel = shooterTargetVel;
        lastMacroStatus = "shoot1: range=" + obs.cameraRangeInches();

        Task macro = buildShootOneBallMacro(shooterTargetVel);
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
        lastShooterMacroTargetVel = 0.0;
        lastMacroStatus = "cancelled";
    }

    /**
     * Build a macro that:
     *
     * <ol>
     *   <li>Spins up the shooter to {@code shooterTargetVel} and waits for
     *       atSetpoint (with timeout).</li>
     *   <li>Feeds one ball using transfer + pusher in parallel.</li>
     *   <li>Holds shooter briefly, then spins down to 0.</li>
     * </ol>
     *
     * @param shooterTargetVel target shooter velocity in native units
     * @return a {@link Task} representing the macro
     */
    private Task buildShootOneBallMacro(double shooterTargetVel) {
        // Step 1: set shooter target and wait for atSetpoint() or timeout.
        Task spinUp = PlantTasks.moveTo(
                shooter,
                shooterTargetVel,
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

        Task pusherShoot = PlantTasks.holdFor(
                pusher,
                PUSHER_POS_SHOOT,
                PUSHER_STAGE_SEC
        );

        Task pusherRetract = PlantTasks.holdFor(
                pusher,
                PUSHER_POS_RETRACT,
                PUSHER_STAGE_SEC
        );

        Task feedPusher = SequenceTask.of(
                pusherLoad,
                pusherShoot,
                pusherRetract
        );

        Task feedBoth = ParallelAllTask.of(
                feedTransfer,
                feedPusher
        );

        // Step 3: optionally hold shooter briefly, then spin down to 0.
        Task holdBeforeSpinDown = PlantTasks.holdFor(
                shooter,
                shooterTargetVel,
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
