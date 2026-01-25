package edu.ftcphoenix.fw.tools.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.debug.NullDebugSink;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.math.InterpolatingTable1D;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * <h1>Example 04: Shooter with Interpolation Table (manual "distance")</h1>
 *
 * <p>This example builds on the shooter examples and shows how to use
 * {@link InterpolatingTable1D} to map a "distance to target" into a
 * shooter velocity.</p>
 *
 * <p>Here, the distance is not coming from vision yet. Instead, we use a
 * simple <b>manual distance source</b> controlled by the driver:</p>
 *
 * <ul>
 *   <li>P1 D-pad UP: increase distance (pretend the robot moved farther).</li>
 *   <li>P1 D-pad DOWN: decrease distance (pretend the robot moved closer).</li>
 * </ul>
 *
 * <p>Each loop we:</p>
 *
 * <ol>
 *   <li>Read the current "distance" (inches).</li>
 *   <li>Look up the shooter velocity from the calibration table.</li>
 *   <li>Command the shooter plant to that velocity (if enabled).</li>
 * </ol>
 *
 * <p>Later, you can replace this manual distance with a real vision
 * distance (AprilTag) by swapping only the "distance source" logic.</p>
 * <p>
 * <hr/>
 *
 * <h2>Why a manual distance source?</h2>
 *
 * <p>You could use a stick directly as "velocity control", but that skips
 * the idea that there is a function:
 * <code>distance → velocity</code>.</p>
 *
 * <p>By treating distance as a separate variable (here adjusted by D-pad),
 * the code structure looks almost identical to a real vision-based setup:</p>
 *
 * <pre>{@code
 * // This example:
 * distanceInches = manualDistance;            // from D-pad
 * shooterTarget  = table.interpolate(distanceInches);
 *
 * // Vision-based setup later:
 * distanceInches = vision.getDistanceInches(); // from AprilTag
 * shooterTarget  = table.interpolate(distanceInches);
 * }</pre>
 * <p>
 * <hr/>
 *
 * <h2>Hardware assumed</h2>
 *
 * <ul>
 *   <li>Drive: mecanum using {@link FtcDrives#mecanum} with default motor names.</li>
 *   <li>Shooter:
 *     <ul>
 *       <li>{@code "shooterLeftMotor"}</li>
 *       <li>{@code "shooterRightMotor"}</li>
 *     </ul>
 *   </li>
 * </ul>
 * <p>
 * <hr/>
 *
 * <h2>Driver controls (suggested)</h2>
 *
 * <ul>
 *   <li><b>Drive</b>:
 *     <ul>
 *       <li>P1 left stick X/Y: strafe + forward/back.</li>
 *       <li>P1 right stick X: rotate.</li>
 *       <li>P1 right bumper: slow mode (same as previous examples).</li>
 *     </ul>
 *   </li>
 *   <li><b>Shooter</b>:
 *     <ul>
 *       <li>P1 left bumper: toggle shooter on/off.</li>
 *       <li>P1 D-pad UP/DOWN: adjust "distance to target" in inches.</li>
 *     </ul>
 *   </li>
 * </ul>
 * <p>
 * <hr/>
 *
 * <h2>Calibration table (distance → velocity)</h2>
 *
 * <p>The table below is an example; teams should tune these numbers
 * during practice. Units here are:</p>
 *
 * <ul>
 *   <li>x: distance in inches (24", 30", 36", 42", 48").</li>
 *   <li>y: shooter velocity in native units (e.g., rad/s or ticks/sec) –
 *       just be consistent with what your shooter plant expects.</li>
 * </ul>
 *
 * <p>The table is clamped and linearly interpolated:</p>
 *
 * <ul>
 *   <li>Below the smallest x → use the first y.</li>
 *   <li>Above the largest x → use the last y.</li>
 *   <li>Between samples → linear interpolation.</li>
 * </ul>
 */
@TeleOp(name = "FW Ex 04: Shooter Interpolated", group = "Framework Examples")
@Disabled
public final class TeleOp_04_ShooterInterpolated extends OpMode {

    // ----------------------------------------------------------------------
    // Calibration table: distance (in) → shooter velocity (native units)
    // ----------------------------------------------------------------------

    /**
     * Shooter velocity table built from sorted (distance, velocity) pairs.
     *
     * <p>Example numbers only – tune for your robot.</p>
     */
    private static final InterpolatingTable1D SHOOTER_VELOCITY_TABLE =
            InterpolatingTable1D.ofSortedPairs(
                    24.0, 170.0,  // close shot
                    30.0, 180.0,
                    36.0, 195.0,
                    42.0, 210.0,
                    48.0, 225.0   // farther shot
            );

    // Distance range we let the driver adjust within.
    private static final double MIN_DISTANCE_INCHES = 20.0;
    private static final double MAX_DISTANCE_INCHES = 60.0;
    private static final double STEP_DISTANCE_INCHES = 2.0;

    // ----------------------------------------------------------------------
    // Hardware names
    // ----------------------------------------------------------------------

    private static final String HW_SHOOTER_LEFT = "shooterLeftMotor";
    private static final String HW_SHOOTER_RIGHT = "shooterRightMotor";

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

    private Plant shooter;

    // Manual distance + shooter state
    private double distanceInches = 36.0; // start in the middle of the table
    private boolean shooterEnabled = false;
    private double lastShooterTarget = 0.0;

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

        // 1) Inputs
        gamepads = Gamepads.create(gamepad1, gamepad2);
        bindings = new Bindings();

        // 2) Drive wiring (same as Example 01)
        drivebase = FtcDrives.mecanum(hardwareMap);
        stickDrive = GamepadDriveSource.teleOpMecanumSlowRb(gamepads);

        // 3) Shooter wiring using Actuators
        shooter = Actuators.plant(hardwareMap)
                .motor(HW_SHOOTER_LEFT, Direction.FORWARD)
                .andMotor(HW_SHOOTER_RIGHT, Direction.REVERSE)
                .velocity(/*toleranceNative=*/100.0)
                .build();

        shooter.setTarget(0.0);

        // 4) Bindings: shooter enable + distance adjust

        // LB: toggle shooter on/off.
        bindings.onToggle(
                gamepads.p1().leftBumper(),
                () -> shooterEnabled = true,
                () -> shooterEnabled = false
        );

        // D-pad UP: increase "distance".
        bindings.onPress(
                gamepads.p1().dpadUp(),
                () -> distanceInches = MathUtil.clamp(distanceInches + STEP_DISTANCE_INCHES,
                        MIN_DISTANCE_INCHES,
                        MAX_DISTANCE_INCHES)
        );

        // D-pad DOWN: decrease "distance".
        bindings.onPress(
                gamepads.p1().dpadDown(),
                () -> distanceInches = MathUtil.clamp(distanceInches - STEP_DISTANCE_INCHES,
                        MIN_DISTANCE_INCHES,
                        MAX_DISTANCE_INCHES)
        );

        telemetry.addLine("FW Example 04: Shooter Interpolated");
        telemetry.addLine("Drive: same as Example 01");
        telemetry.addLine("Shooter:");
        telemetry.addLine("  LB = toggle shooter on/off");
        telemetry.addLine("  D-pad UP/DOWN = adjust 'distance' (inches)");
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

        // --- 3) Drive: always manual ---
        DriveSignal driveCmd = stickDrive.get(clock).clamped();
        lastDrive = driveCmd;

        drivebase.update(clock);
        drivebase.drive(driveCmd);

        // --- 4) Shooter: distance → velocity via interpolation table ---

        if (shooterEnabled) {
            // Look up velocity from the table (with clamping + interpolation).
            double targetVel = SHOOTER_VELOCITY_TABLE.interpolate(distanceInches);
            lastShooterTarget = targetVel;
            shooter.setTarget(targetVel);
        } else {
            lastShooterTarget = 0.0;
            shooter.setTarget(0.0);
        }

        // Update shooter plant once per loop.
        shooter.update(dtSec);
        // --- 5) Required telemetry (driver-facing) ---
        // This is the information you should not lose even when debug output is disabled.
        telemetry.addLine("FW Example 04: Shooter Interpolated");
        telemetry.addData("drive.cmd", driveCmd);
        telemetry.addData("shooter.enabled", shooterEnabled);
        telemetry.addData("range.in", distanceInches);
        telemetry.addData("shooter.targetNative", lastShooterTarget);
        telemetry.addData("shooter.atSetpoint", shooter.atSetpoint());
        telemetry.addData("debug.enabled", DEBUG);

        // --- 6) Optional debug (can be disabled without breaking required telemetry) ---
        // Use DebugSink + debugDump() for verbose internal state.
        if (DEBUG) {
            dbg.addLine("--- debugDump() (optional) ---");

            clock.debugDump(dbg, "clock");
            gamepads.debugDump(dbg, "pads");
            bindings.debugDump(dbg, "bindings");

            stickDrive.debugDump(dbg, "driveSrc");
            drivebase.debugDump(dbg, "drivebase");

            shooter.debugDump(dbg, "shooter");
        }

        telemetry.update();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void stop() {
        shooterEnabled = false;
        shooter.setTarget(0.0);
        shooter.update(0.0);
        drivebase.stop();
    }
}
