package edu.ftcphoenix.fw.tools.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import edu.ftcphoenix.fw.actuation.Plant;
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
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceSpec;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.ftc.FtcVision;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.CameraMountLogic;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionPolicies;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelections;
import edu.ftcphoenix.fw.spatial.ReferencePoint2d;
import edu.ftcphoenix.fw.spatial.References;

/**
 * <h1>Example 05: Shooter + DriveGuidance Auto-Aim + Vision Distance</h1>
 *
 * <p>This example combines three ideas:</p>
 *
 * <ol>
 *   <li><b>Mecanum drive</b> using {@link FtcDrives#mecanum} +
 *       {@link GamepadDriveSource} with explicit axis wiring and a robot-owned slow-mode wrapper (same as Example 01).</li>
 *   <li><b>Tag-based auto-aim</b> using {@link DriveGuidance} (as a {@link edu.ftcphoenix.fw.drive.DriveOverlay}) plus a shared {@link TagSelectionSource}:
 *     <ul>
 *       <li>Hold a button to override omega and face a scoring AprilTag.</li>
 *       <li>Axial/lateral motion still come from the driver.</li>
 *     </ul>
 *   </li>
 *   <li><b>Shooter velocity from AprilTag distance</b>:
 *     <ul>
 *       <li>Use an {@link AprilTagSensor} created by {@link FtcVision#aprilTags}.</li>
 *       <li>Share a {@link TagSelectionSource} so aiming, telemetry, and shooter range all agree on the tag choice.</li>
 *       <li>Read {@link AprilTagObservation#cameraRangeInches()} from the selector's fresh selected observation.</li>
 *       <li>Use an {@link InterpolatingTable1D} to map
 *           {@code distance → shooter velocity} (native units).</li>
 *     </ul>
 *   </li>
 * </ol>
 *
 * <h2>Camera offset note</h2>
 * <p>
 * This example uses {@link CameraMountConfig} so the vision observation is expressed relative to
 * the <b>robot</b> (not just the camera), even if the camera is mounted off-center.
 * </p>
 */
@TeleOp(name = "FW Ex 05: Shooter Guidance Aim Vision", group = "Framework Examples")
@Disabled
public final class TeleOp_05_ShooterTagAimVision extends OpMode {

    // ----------------------------------------------------------------------
    // Calibration: distance (inches) → shooter velocity (native units)
    // ----------------------------------------------------------------------

    private static final InterpolatingTable1D SHOOTER_VELOCITY_TABLE =
            InterpolatingTable1D.ofSortedPairs(
                    // Example calibration data (distanceInches, velocityNative):
                    24.0, 3500.0,
                    30.0, 3600.0,
                    36.0, 3700.0,
                    42.0, 3800.0,
                    48.0, 3900.0
            );

    // ----------------------------------------------------------------------
    // Tag age constraint
    // ----------------------------------------------------------------------

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
    private DriveSource baseDrive;
    private DriveSource driveWithAim;

    private AprilTagSensor tagSensor;
    private TagSelectionSource scoringSelection;

    private CameraMountConfig cameraMount;

    private Plant shooter;

    private boolean shooterEnabled = false;

    private DriveSignal lastDrive = DriveSignal.zero();

    // For telemetry about the last tag observation used for shooter control.
    private boolean lastHasTarget = false;
    private double lastTagRangeInches = 0.0;
    private double lastCameraBearingRad = 0.0;
    private double lastRobotBearingRad = 0.0;
    private double lastTagAgeSec = 0.0;
    private int lastTagId = -1;

    private double lastShooterTargetVel = 0.0;

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
        baseDrive = new GamepadDriveSource(
                gamepads.p1().leftX(),
                gamepads.p1().leftY(),
                gamepads.p1().rightX(),
                GamepadDriveSource.Config.defaults()
        ).scaledWhen(gamepads.p1().rightBumper(), 0.35, 0.20)
                .rateLimited(4.0, 4.0, 6.0);

        // 3) Tag sensor: real FTC VisionPortal + AprilTagProcessor adapter.
        //
        // NOTE: Replace "Webcam 1" with your actual camera name in the Robot Configuration.
        tagSensor = FtcVision.aprilTags(hardwareMap, "Webcam 1");

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

        // Shared tag selection: preview continuously, then latch the best visible scoring tag
        // while the driver holds aim assist.
        scoringSelection = TagSelections.from(tagSensor)
                .among(SCORING_TAG_IDS)
                .freshWithinSec(MAX_TAG_AGE_SEC)
                .choose(TagSelectionPolicies.smallestAbsRobotBearing(cameraMount))
                .stickyWhen(gamepads.p1().leftBumper())
                .holdUntilDisabled()
                .build();

        // Build a vision-only auto-aim plan and overlay it on top of stick driving.
        // The semantic reference is “the center of the currently selected scoring tag.”
        ReferencePoint2d scoringRef = References.relativeToSelectedTagPoint(scoringSelection, 0.0, 0.0);

        DriveGuidancePlan aimPlan = DriveGuidance.plan()
                .faceTo()
                .point(scoringRef)
                .solveWith()
                .aprilTagsOnly()
                .aprilTags(tagSensor, cameraMount)
                .maxAgeSec(MAX_TAG_AGE_SEC)
                .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
                .doneAprilTagsOnly()
                .build();

        // Apply the plan as an overlay: the driver keeps translation, and the overlay owns omega.
        driveWithAim = baseDrive.overlayWhen(
                gamepads.p1().leftBumper(),
                aimPlan.overlay(),
                DriveOverlayMask.OMEGA_ONLY
        );

        // 4) Shooter wiring using FtcActuators.
        shooter = FtcActuators.plant(hardwareMap)
                .motor(HW_SHOOTER_LEFT, Direction.FORWARD)
                .andMotor(HW_SHOOTER_RIGHT, Direction.REVERSE)
                .velocity()
                .deviceManagedWithDefaults()
                .bounded(0.0, 4200.0)
                .nativeUnits()
                .velocityTolerance(/*toleranceNative=*/100.0)
                .targetedByDefaultWritable(0.0)
                .build();

        shooter.writableTarget().set(0.0);

        // 5) Bindings: shooter toggle.
        bindings.toggleOnRise(
                gamepads.p1().a(),
                () -> shooterEnabled = true,
                () -> shooterEnabled = false
        );

        telemetry.addLine("FW Example 05: Shooter DriveGuidance Vision");
        telemetry.addLine("Drive: mecanum + DriveGuidance (hold LB to auto-aim)");
        telemetry.addLine("Shooter: A = toggle on/off");
        telemetry.addLine("Auto-aim: mount-aware (robot center aims at tag)");
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
        // 1) Clock
        clock.update(getRuntime());
        double dtSec = clock.dtSec();

        // 2) Sense (sensors)
        // Gamepad axes/buttons are Sources; they are sampled when you call get(...).

        // 3) Decide (high-level logic)

        // User-input driven decisions (e.g., shooterEnabled toggle on A)
        bindings.update(clock);

        // Vision-based shooter target decision
        TagSelectionResult selection = scoringSelection.get(clock);
        AprilTagObservation obs = selection.hasFreshSelectedObservation
                ? selection.selectedObservation
                : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
        lastHasTarget = obs.hasTarget;
        lastTagRangeInches = obs.cameraRangeInches();
        lastCameraBearingRad = obs.cameraBearingRad();
        lastRobotBearingRad = obs.hasTarget
                ? CameraMountLogic.robotBearingRad(obs, cameraMount)
                : 0.0;
        lastTagAgeSec = obs.ageSec;
        lastTagId = obs.id;

        if (shooterEnabled && obs.hasTarget) {
            double targetVel = SHOOTER_VELOCITY_TABLE.interpolate(obs.cameraRangeInches());
            lastShooterTargetVel = targetVel;
            shooter.writableTarget().set(targetVel);
        } else {
            lastShooterTargetVel = 0.0;
            shooter.writableTarget().set(0.0);
        }

        // 4) Control / Actuate (subsystems)

        // Drive: base sticks + (optional) auto-aim overlay
        DriveSignal cmd = driveWithAim.get(clock).clamped();
        lastDrive = cmd;

        drivebase.update(clock);
        drivebase.drive(cmd);

        // Shooter plant executes whatever target we just decided above
        shooter.update(clock);

        // 5) Report
        //
        // Important rule: telemetry required for normal operation (driver-facing state) should
        // NOT go through the debug pipeline. Debug is optional and should be safe to disable.

        // --- Required telemetry (driver-facing) ---
        telemetry.addLine("FW Example 05: Shooter DriveGuidance Vision");
        telemetry.addData("aim.active", gamepads.p1().leftBumper().getAsBoolean(clock));
        telemetry.addData("tag.hasTarget", lastHasTarget);
        telemetry.addData("tag.id", lastTagId);
        telemetry.addData("tag.rangeIn", lastTagRangeInches);
        telemetry.addData("tag.ageSec", lastTagAgeSec);
        telemetry.addData("shooter.enabled", shooterEnabled);
        telemetry.addData("shooter.targetVelNative", lastShooterTargetVel);
        telemetry.addData("drive.cmd", cmd);
        telemetry.addData("debug.enabled", DEBUG);

        // --- Optional debug (verbose; can be disabled without losing required telemetry) ---
        if (DEBUG) {
            dbg.addLine("--- debugDump() (optional) ---");

            clock.debugDump(dbg, "clock");
            gamepads.debugDump(dbg, "pads");
            bindings.debugDump(dbg, "bindings");

            scoringSelection.debugDump(dbg, "tagSelection");
            shooter.debugDump(dbg, "shooter");

            driveWithAim.debugDump(dbg, "driveSrc");
            drivebase.debugDump(dbg, "drivebase");
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
        shooterEnabled = false;
        shooter.writableTarget().set(0.0);
        shooter.stop();
        drivebase.stop();
    }
}
