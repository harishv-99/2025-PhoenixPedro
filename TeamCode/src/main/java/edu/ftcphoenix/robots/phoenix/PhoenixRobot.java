package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import edu.ftcphoenix.fw.core.control.HysteresisBoolean;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveOverlayStack;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidance;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceQuery;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceSpec;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.ftc.FtcVision;
import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.sensing.observation.ObservationSource2d;
import edu.ftcphoenix.fw.sensing.observation.ObservationSources;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagTarget;

/**
 * Central robot class for Phoenix-based robots.
 *
 * <p>Beginners should mostly edit <b>this file</b>. TeleOp and Auto OpModes are
 * kept very thin and simply delegate into the methods here.</p>
 *
 * <h2>Responsibilities</h2>
 * <ul>
 *   <li>Wire all hardware once (drive, ball path, shooter, vision).</li>
 *   <li>Define gamepad mappings in one place via {@link Bindings}.</li>
 *   <li>Own shared logic: auto-aim, shooter velocity, macros, autos.</li>
 *   <li>Expose simple entry points for TeleOp and Auto.</li>
 * </ul>
 */
public final class PhoenixRobot {
    private final LoopClock clock = new LoopClock();
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepads gamepads;
    private final Bindings bindings = new Bindings();
    private final DebugSink dbg;
    private Shooter shooter;
    private ShooterSupervisor shooterSupervisor;
    private MecanumDrivebase drivebase;
    private PinpointPoseEstimator pinpoint;
    private DriveSource stickDrive;
    private DriveSource driveWithAim;
    private BooleanSource shootHeld = BooleanSource.constant(false);
    private CameraMountConfig cameraMountConfig;
    private AprilTagSensor tagSensor;
    private TagTarget scoringTarget;
    private DriveGuidancePlan aimPlanBlue;
    private DriveGuidancePlan aimPlanRed;
    private DriveGuidancePlan.Tuning aimTuning;
    private DriveGuidanceQuery aimQueryBlue;
    private DriveGuidanceQuery aimQueryRed;
    private TagLayout gameTagLayout;

    // "Shoot brace" pose-lock: latch-on while the shooter is spinning and the driver is not
    // commanding translation. This lets the driver still nudge/strafe to line up, but once they
    // let go, the robot resists being bumped off the spot.
    private static final double SHOOT_BRACE_ENTER_MAG = 0.06;
    private static final double SHOOT_BRACE_EXIT_MAG = 0.10;

    private final HysteresisBoolean shootBraceLatch =
            HysteresisBoolean.onWhenBelowOffWhenAbove(SHOOT_BRACE_ENTER_MAG, SHOOT_BRACE_EXIT_MAG);


    // ----------------------------------------------------------------------
    // Tag IDs we care about (example values; adjust per game) – Java 8 style
    // ----------------------------------------------------------------------

    private static final Set<Integer> SCORING_TAG_IDS;

    static {
        HashSet<Integer> ids = new HashSet<Integer>();
        ids.add(RobotConfig.AutoAim.BLUE_TARGET_TAG_ID);
        ids.add(RobotConfig.AutoAim.RED_TARGET_TAG_ID);
        SCORING_TAG_IDS = Collections.unmodifiableSet(ids);
    }

    /**
     * Create the robot container.
     *
     * <p>This class owns the major subsystems (drive, shooter, etc.) and wires them to gamepads.
     * It is intended to be constructed once in an OpMode {@code init()} method.</p>
     *
     * @param hardwareMap FTC hardware map
     * @param telemetry   FTC telemetry sink
     * @param gamepad1    primary driver controller
     * @param gamepad2    secondary operator controller
     */
    public PhoenixRobot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.gamepads = Gamepads.create(gamepad1, gamepad2);
        this.telemetry = telemetry;
        this.dbg = new FtcTelemetryDebugSink(telemetry);
    }

    /**
     * Initialize components shared by all OpModes.
     */
    public void initAny() {
    }

    /**
     * Initialize TeleOp-specific state and bindings.
     */
    public void initTeleOp() {

        // --- Create mechanisms ---
        MecanumDrivebase.Config mecanumConfig = MecanumDrivebase.Config.defaults();
        FtcDrives.MecanumWiringConfig mecanumWiring = RobotConfig.DriveTrain.mecanumWiring();
        drivebase = FtcDrives.mecanum(
                hardwareMap,
                mecanumWiring,
                mecanumConfig);

        // Use motor braking to help resist small pushes when commanded power is 0.
        // PoseLock will actively correct position, but BRAKE helps reduce "coast".
        FtcDrives.setDriveBrake(hardwareMap, mecanumWiring, RobotConfig.DriveTrain.zeroPowerBrake);

        shooter = new Shooter(hardwareMap, telemetry);

        // --- Use the standard TeleOp stick mapping for mecanum.
        stickDrive = GamepadDriveSource.teleOpMecanumSlowRb(gamepads);

        // --- Odometry (goBILDA Pinpoint) ---
        // This is used for pose-lock (resist bumps while shooting). It can also be used for
        // autonomous / fusion later.
        pinpoint = new PinpointPoseEstimator(hardwareMap, RobotConfig.Localization.pinpoint);

        // --- Vision ---
        cameraMountConfig = RobotConfig.Vision.cameraMount;
        tagSensor = FtcVision.aprilTags(hardwareMap, RobotConfig.Vision.nameWebcam);

        // Use the FTC-provided tag layout metadata (field coordinates + tag heading).
        // This is optional for pure “vision-only” aiming, but it becomes important when you
        // later add odometry / field-pose aware targeting.
        gameTagLayout = new FtcGameTagLayout(AprilTagGameDatabase.getCurrentGameTagLibrary());

        // Track scoring tags with a freshness window.
        scoringTarget = new TagTarget(tagSensor, SCORING_TAG_IDS, 0.5);

        // Shooter supervisor: owns shoot policy + sensorless timing.
        // Gamepad bindings update the supervisor state; we expose that state as a BooleanSource
        // so it can also enable auto-aim and shoot-brace.
        shooterSupervisor = new ShooterSupervisor(shooter, scoringTarget);
        shootHeld = BooleanSource.of(shooterSupervisor::isShootHeld);

        // --- Drive guidance (replaces TagAim): hold P2 B to auto-aim omega at the best scoring tag.
        // Use the framework's helper so robot code stays simple.
        // (This updates the TagTarget each loop and converts the latest AprilTag measurement into a
        // robot-relative planar observation used by DriveGuidance.)
        ObservationSource2d obs2d = ObservationSources.aprilTag(scoringTarget, cameraMountConfig);

        // Tuning for the auto-aim assist. Start with defaults and tweak only what you need.
        aimTuning = DriveGuidancePlan.Tuning.defaults()
                .withAimKp(2.0)            // how strongly we turn toward the target
                .withAimDeadbandDeg(0.25); // stop turning when we're within this many degrees

        // --- Auto-aim plans ---
        // Instead of aiming at the AprilTag center, aim at a *tag-relative point* that you pick
        // (example: a basket corner). Each tag ID can have a different offset.
        //
        // Offsets are in the tag frame: +forward is out of the tag, +left is left when looking out.
        RobotConfig.AutoAim.AimOffset blueOffset = RobotConfig.AutoAim.BLUE_AIM_OFFSET;
        RobotConfig.AutoAim.AimOffset redOffset = RobotConfig.AutoAim.RED_AIM_OFFSET;

        aimPlanBlue = DriveGuidance.plan()
                .aimTo()
                .tagRelativePointInches(
                        RobotConfig.AutoAim.BLUE_TARGET_TAG_ID,
                        blueOffset.forwardInches,
                        blueOffset.leftInches)
                .doneAimTo()
                .tuning(aimTuning)
                .feedback()
                .observation(obs2d, 0.50, 0.0)
                .lossPolicy(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
                .doneFeedback()
                .build();

        aimPlanRed = DriveGuidance.plan()
                .aimTo()
                .tagRelativePointInches(
                        RobotConfig.AutoAim.RED_TARGET_TAG_ID,
                        redOffset.forwardInches,
                        redOffset.leftInches)
                .doneAimTo()
                .tuning(aimTuning)
                .feedback()
                .observation(obs2d, 0.50, 0.0)
                .lossPolicy(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
                .doneFeedback()
                .build();

        // Queries: sample the exact same math used by the overlays (for telemetry & gating),
        // without needing to re-derive bearings or worry about control frames.
        aimQueryBlue = aimPlanBlue.query();
        aimQueryRed = aimPlanRed.query();

        /*
         * FUTURE OPTION (leave commented out for now):
         * Combine AprilTag observation + odometry pose estimator for targeting.
         *
         * Why:
         *  - If the tag blinks out of view for a moment, fieldPose(...) can keep aim stable.
         *  - You can aim at a tag-relative point in *field coordinates* (using gameTagLayout).
         *
         * How:
         *  - Provide BOTH observation(...) and fieldPose(...)
         *  - Provide a TagLayout (we already built gameTagLayout above)
         *  - Optionally configure gates(...) so Phoenix blends between sources based on range
         */
        /*
         * If you eventually want true <b>fusion</b> (odometry corrected by tags) instead of
         * “use odometry when the tag isn't visible”, you can build a fused PoseEstimator and
         * pass that into fieldPose(...):
         *
         *   edu.ftcphoenix.fw.localization.apriltag.TagOnlyPoseEstimator visionPose =
         *       new edu.ftcphoenix.fw.localization.apriltag.TagOnlyPoseEstimator(
         *           scoringTarget,
         *           gameTagLayout,
         *           cameraMountConfig
         *       );
         *
         *   edu.ftcphoenix.fw.localization.fusion.OdometryTagFusionPoseEstimator fusedPose =
         *       new edu.ftcphoenix.fw.localization.fusion.OdometryTagFusionPoseEstimator(
         *           pinpoint,
         *           visionPose
         *       );
         *
         * IMPORTANT:
         *   - You still must call scoringTarget.update(clock) before fusedPose.update(clock)
         *   - In updateTeleOp(), update fusedPose instead of (or in addition to) pinpoint
         */
//        aimPlanBlue = DriveGuidance.plan()
//                .aimTo()
//                .tagRelativePointInches(
//                        RobotConfig.AutoAim.BLUE_TARGET_TAG_ID,
//                        blueOffset.forwardInches,
//                        blueOffset.leftInches)
//                .doneAimTo()
//                .tuning(aimTuning)
//                .feedback()
//                .observation(obs2d, 0.50, 0.0)
//                .fieldPose(pinpoint, gameTagLayout, 0.25, 0.0)
//                .gates(72.0, 84.0, 0.25) // enterRange, exitRange, blendSeconds (example numbers)
//                .preferObservationForOmegaWhenValid(true)
//                .lossPolicy(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
//                .doneFeedback()
//                .build();
//
//        aimPlanRed = DriveGuidance.plan()
//                .aimTo()
//                .tagRelativePointInches(
//                        RobotConfig.AutoAim.RED_TARGET_TAG_ID,
//                        redOffset.forwardInches,
//                        redOffset.leftInches)
//                .doneAimTo()
//                .tuning(aimTuning)
//                .feedback()
//                .observation(obs2d, 0.50, 0.0)
//                .fieldPose(pinpoint, gameTagLayout, 0.25, 0.0)
//                .gates(72.0, 84.0, 0.25)
//                .preferObservationForOmegaWhenValid(true)
//                .lossPolicy(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
//                .doneFeedback()
//                .build();

        // Enable condition for the guidance overlay.
        //
        // Hold-to-enable is the simplest for drivers. If you prefer a toggle (press once to
        // enable, press again to disable), use: gamepads.p2().leftBumper().toggled()
        BooleanSource autoAimEnabled = shootHeld;

        // Stack multiple overlays without nested overlayWhen(...) calls.
        //
        // Order matters only when two enabled overlays claim the same DOF (the last layer wins).
        driveWithAim = DriveOverlayStack.on(stickDrive)
                .add(
                        "shootBrace",
                        BooleanSource.of(shootBraceLatch::get),
                        DriveGuidance.poseLock(
                                pinpoint,
                                DriveGuidancePlan.Tuning.defaults()
                                        .withTranslateKp(0.08)
                                        .withMaxTranslateCmd(0.35)
                        ),
                        DriveOverlayMask.TRANSLATION_ONLY
                )
                .add(
                        "autoAimBlue",
                        autoAimEnabled.and(BooleanSource.of(() -> {
                            AprilTagObservation obs = scoringTarget.last();
                            return obs.hasTarget && obs.id == RobotConfig.AutoAim.BLUE_TARGET_TAG_ID;
                        })),
                        aimPlanBlue.overlay(),
                        DriveOverlayMask.OMEGA_ONLY
                )
                .add(
                        "autoAimRed",
                        autoAimEnabled.and(BooleanSource.of(() -> {
                            AprilTagObservation obs = scoringTarget.last();
                            return obs.hasTarget && obs.id == RobotConfig.AutoAim.RED_TARGET_TAG_ID;
                        })),
                        aimPlanRed.overlay(),
                        DriveOverlayMask.OMEGA_ONLY
                )
                .build();

        telemetry.addLine("Phoenix TeleOp (Supervisor + OutputTasks)");
        telemetry.addLine("P1: Left stick=drive, Right stick=turn, RB=slow mode");
        telemetry.addLine("P2: A=toggle intake");
        telemetry.addLine("P2: B=shoot (tap for one, hold to keep shooting)");
        telemetry.addLine("P2: X= eject / unjam (hold; reverse feeds)");
        telemetry.addLine("P2: DPad Up/Down=shooter velocity (fallback when no tag)");
        telemetry.update();

        // Create bindings
        createBindings();
    }

    private void createBindings() {
        // -------------------------
        // Gamepad 2: shooter + intake
        // -------------------------

        // A: toggle intake
        bindings.onRise(gamepads.p2().a(), shooterSupervisor::toggleIntake);

        // B: shoot
        // - tap: queue one shot
        // - hold: keep shooting (supervisor keeps a 1-shot backlog buffered)
        bindings.onRiseAndFall(
                gamepads.p2().b(),
                () -> {
                    shooterSupervisor.setShootHeld(true);
                    shooterSupervisor.requestShootOne();
                },
                () -> shooterSupervisor.setShootHeld(false)
        );

        // X: eject / unjam (hold)
        bindings.onRiseAndFall(
                gamepads.p2().x(),
                () -> shooterSupervisor.setEjectHeld(true),
                () -> shooterSupervisor.setEjectHeld(false)
        );

        // D-pad: tweak manual (fallback) flywheel velocity (used when no tag is visible).
        bindings.onRise(gamepads.p2().dpadUp(), shooter::increaseManualVelocity);
        bindings.onRise(gamepads.p2().dpadDown(), shooter::decreaseManualVelocity);
    }

    /**
     * Start hook shared by all OpModes.
     */
    public void startAny(double runtime) {
        // Initialize loop timing.
        clock.reset(runtime);
    }

    /**
     * Start hook for TeleOp.
     */
    public void startTeleOp() {
    }

    /**
     * Periodic update shared by all OpModes.
     */
    public void updateAny(double runtime) {
        // --- 1) Clock ---
        clock.update(runtime);
    }

    /**
     * Periodic update for TeleOp.
     */
    public void updateTeleOp() {
        // --- 2) Inputs + bindings ---
        // Gamepad axes/buttons are Sources; they are sampled when you call get(...).

        // Update tracked tag once per loop.
        scoringTarget.update(clock);

        bindings.update(clock);

        // --- Odometry update (needed for pose lock) ---
        if (pinpoint != null) {
            pinpoint.update(clock);
        }

        // --- Shoot-brace latch (pose lock translation only) ---
        updateShootBraceEnabled();

        // --- 3) Supervisors / mechanisms ---
        shooterSupervisor.update(clock);

        // --- 4) Drive: guidance overlay (P2 B may override omega) ---
        DriveSignal cmd = driveWithAim.get(clock).clamped();
        drivebase.update(clock);
        drivebase.drive(cmd);

        // --- 5) Other mechanisms ---
        shooter.update(clock);

        // --- 6) Telemetry / debug ---
        shooter.telemetryDump(clock, "shooter");
        telemetry.addData("intake.enabled", shooterSupervisor.intakeEnabled());
        telemetry.addData("shoot.btn", shootHeld.getAsBoolean(clock));
        telemetry.addData("shootBrace", shootBraceLatch.get());
        if (pinpoint != null) {
            telemetry.addData("pose", pinpoint.getEstimate());
        }
//        driveWithAim.debugDump(dbg, "drive");
        AprilTagObservation obs = scoringTarget.last();
        if (obs.hasTarget) {
            RobotConfig.AutoAim.AimOffset aimOffset = RobotConfig.AutoAim.aimOffsetForTag(obs.id);

            // Use the same guidance evaluation math as the overlay (no manual camera-mount math,
            // and control frames are automatically applied).
            DriveGuidanceStatus aimStatus = null;
            if (obs.id == RobotConfig.AutoAim.BLUE_TARGET_TAG_ID && aimQueryBlue != null) {
                aimStatus = aimQueryBlue.sample(clock, DriveOverlayMask.OMEGA_ONLY);
            } else if (obs.id == RobotConfig.AutoAim.RED_TARGET_TAG_ID && aimQueryRed != null) {
                aimStatus = aimQueryRed.sample(clock, DriveOverlayMask.OMEGA_ONLY);
            }

            if (aimStatus != null && aimStatus.omegaWithin(aimTuning.aimDeadbandRad * 5)) {
                telemetry.addLine(">>> AIMED <<<");
            }

            telemetry.addData("tagId", obs.id);
            telemetry.addData("distIn", obs.cameraRangeInches());
            telemetry.addData("bearingTagDeg", Math.toDegrees(obs.cameraBearingRad()));
            telemetry.addData(
                    "aimOffset(fwd,left)",
                    String.format("%.1f, %.1f", aimOffset.forwardInches, aimOffset.leftInches)
            );
            telemetry.addData(
                    "omegaErrDeg",
                    (aimStatus != null && aimStatus.hasOmegaError)
                            ? Math.toDegrees(aimStatus.omegaErrorRad)
                            : Double.NaN
            );

            // Field metadata (comes from the FTC game database): where this tag is placed on the field.
            // This is useful for sanity-checking that you're targeting the right point.
            if (gameTagLayout != null && gameTagLayout.has(obs.id)) {
                Pose3d fieldToTag = gameTagLayout.require(obs.id).fieldToTagPose();
                Pose2d fieldToAimPoint = new Pose2d(
                        fieldToTag.xInches,
                        fieldToTag.yInches,
                        fieldToTag.yawRad
                ).then(new Pose2d(aimOffset.forwardInches, aimOffset.leftInches, 0.0));

                telemetry.addData("field.tag", fieldToTag);
                telemetry.addData("field.aim", fieldToAimPoint);
            }
        }

        telemetry.update();
    }

    /**
     * Latches translation pose-lock while the aiming driver is holding the shoot trigger
     * (auto-aim) and the driver is not commanding translation.
     *
     * <p>This makes "hold B" a single driver action for: aim + brace + shoot.</p>
     * <p>Hysteresis prevents chatter when sticks hover near center.</p>
     */
    private void updateShootBraceEnabled() {
        // Only brace while P2 is actively holding the auto-aim trigger.
        // This avoids surprise "fighting the driver" behavior during normal driving.
        if (!shootHeld.getAsBoolean(clock)) {
            shootBraceLatch.reset(false);
            return;
        }

        // Treat the driver stick as "idle" only after it is clearly near center, and stay idle
        // until the stick is clearly moved again (hysteresis avoids chatter).
        double mag = gamepads.p1().leftStickMagnitude().getAsDouble(clock);
        shootBraceLatch.update(mag);
    }


    /**
     * Stop hook shared by all OpModes.
     */
    public void stopAny() {
        drivebase.stop();
        if (shooter != null) {
            shooter.stop();
        }
    }

    /**
     * Stop hook for TeleOp.
     */
    public void stopTeleOp() {
        // Release camera resources so future OpModes/testers can start vision cleanly.
        if (tagSensor != null) {
            tagSensor.close();
            tagSensor = null;
        }
    }
}
