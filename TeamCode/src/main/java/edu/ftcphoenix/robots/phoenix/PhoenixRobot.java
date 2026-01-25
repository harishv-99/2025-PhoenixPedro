package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.ftcphoenix.fw.core.control.HysteresisLatch;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveOverlayStack;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidance;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidancePlan;
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
import edu.ftcphoenix.fw.sensing.vision.CameraMountLogic;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagTarget;
import edu.ftcphoenix.fw.task.TaskBindings;
import edu.ftcphoenix.fw.task.TaskRunner;
import edu.ftcphoenix.fw.task.Tasks;

/**
 * Central robot class for Phoenix-based robots.
 *
 * <p>Beginners should mostly edit <b>this file</b>. TeleOp and Auto OpModes are
 * kept very thin and simply delegate into the methods here.</p>
 *
 * <h2>Responsibilities</h2>
 * <ul>
 *   <li>Wire all hardware once (drive, intake, transfer, shooter, pusher, vision).</li>
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
    private final TaskRunner taskRunnerTeleOp = new TaskRunner();
    private final DebugSink dbg;
    private Shooter shooter;
    private MecanumDrivebase drivebase;
    private PinpointPoseEstimator pinpoint;
    private DriveSource stickDrive;
    private DriveSource driveWithAim;
    private CameraMountConfig cameraMountConfig;
    private AprilTagSensor tagSensor;
    private TagTarget scoringTarget;
    private DriveGuidancePlan aimPlanBlue;
    private DriveGuidancePlan aimPlanRed;
    private DriveGuidancePlan.Tuning aimTuning;
    private TagLayout gameTagLayout;

    // "Shoot brace" pose-lock: latch-on while the shooter is spinning and the driver is not
    // commanding translation. This lets the driver still nudge/strafe to line up, but once they
    // let go, the robot resists being bumped off the spot.
    private static final double SHOOT_BRACE_ENTER_MAG = 0.06;
    private static final double SHOOT_BRACE_EXIT_MAG = 0.10;

    private final HysteresisLatch shootBraceLatch =
            HysteresisLatch.onWhenBelowOffWhenAbove(SHOOT_BRACE_ENTER_MAG, SHOOT_BRACE_EXIT_MAG);


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

        shooter = new Shooter(hardwareMap, telemetry, gamepads);

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

        // --- Drive guidance (replaces TagAim): hold P2 LB to auto-aim omega at the best scoring tag.
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
                .lossPolicy(DriveGuidancePlan.LossPolicy.PASS_THROUGH)
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
                .lossPolicy(DriveGuidancePlan.LossPolicy.PASS_THROUGH)
                .doneFeedback()
                .build();

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
//                .lossPolicy(DriveGuidancePlan.LossPolicy.PASS_THROUGH)
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
//                .lossPolicy(DriveGuidancePlan.LossPolicy.PASS_THROUGH)
//                .doneFeedback()
//                .build();

        // Enable condition for the guidance overlay.
        //
        // Hold-to-enable is the simplest for drivers. If you prefer a toggle (press once to
        // enable, press again to disable), use: gamepads.p2().leftBumper()::isToggled
        BooleanSupplier autoAimEnabled = gamepads.p2().leftBumper()::isHeld;

        // Stack multiple overlays without nested overlayWhen(...) calls.
        //
        // Order matters only when two enabled overlays claim the same DOF (the last layer wins).
        driveWithAim = DriveOverlayStack.on(stickDrive)
                .add(
                        "shootBrace",
                        shootBraceLatch::get,
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
                        () -> autoAimEnabled.getAsBoolean()
                                && scoringTarget.last().hasTarget
                                && scoringTarget.last().id == RobotConfig.AutoAim.BLUE_TARGET_TAG_ID,
                        aimPlanBlue.overlay(),
                        DriveOverlayMask.OMEGA_ONLY
                )
                .add(
                        "autoAimRed",
                        () -> autoAimEnabled.getAsBoolean()
                                && scoringTarget.last().hasTarget
                                && scoringTarget.last().id == RobotConfig.AutoAim.RED_TARGET_TAG_ID,
                        aimPlanRed.overlay(),
                        DriveOverlayMask.OMEGA_ONLY
                )
                .build();

        telemetry.addLine("Phoenix TeleOp with AutoAim");
        telemetry.addLine("Left stick: drive, Right stick: turn, RB: slow mode");
        telemetry.addLine("P2 LB: auto-aim + lock translation (shoot brace)");
        telemetry.update();

        // Create bindings
        createBindings();
    }

    private void createBindings() {
        // Most bindings in TeleOp simply enqueue a Task. TaskBindings removes the
        // repeated "() -> runner.enqueue(... )" boilerplate.
        TaskBindings tb = TaskBindings.of(bindings, taskRunnerTeleOp);

        tb.onPress(gamepads.p2().y(), shooter::instantSetPusherFront);
        tb.onPress(gamepads.p2().a(), shooter::instantSetPusherBack);

        // Hold to run transfer; release to stop.
        tb.onPressAndRelease(
                gamepads.p2().b(),
                () -> shooter.instantStartTransfer(Shooter.TransferDirection.FORWARD),
                shooter::instantStopTransfer
        );

        tb.onPressAndRelease(
                gamepads.p2().x(),
                () -> shooter.instantStartTransfer(Shooter.TransferDirection.BACKWARD),
                shooter::instantStopTransfer
        );

        // While held: continuously update shooter velocity based on the latest tag range.
        tb.whileHeld(gamepads.p2().leftBumper(), () -> {
            AprilTagObservation obs = scoringTarget.last();
            return obs.hasTarget
                    ? shooter.instantSetVelocityByDist(obs.cameraRangeInches())
                    : Tasks.noop();
        });

        tb.onToggle(gamepads.p2().rightBumper(), shooter::instantStartShooter, shooter::instantStopShooter);

        tb.onPress(gamepads.p2().dpadUp(), shooter::instantIncreaseVelocity);
        tb.onPress(gamepads.p2().dpadDown(), shooter::instantDecreaseVelocity);
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
        gamepads.update(clock);

        // Update tracked tag once per loop.
        scoringTarget.update(clock);

        bindings.update(clock);

        // --- Odometry update (needed for pose lock) ---
        if (pinpoint != null) {
            pinpoint.update(clock);
        }

        // --- Shoot-brace latch (pose lock translation only) ---
        updateShootBraceEnabled();


        // --- 3) TeleOp Macros ---
        taskRunnerTeleOp.update(clock);

        // When no macro is active, hold a safe default state.
        if (!taskRunnerTeleOp.hasActiveTask()) {
        }

        // --- 4) Drive: guidance overlay (P2 LB may override omega) ---
        DriveSignal cmd = driveWithAim.get(clock).clamped();
        drivebase.update(clock);
        drivebase.drive(cmd);

        // --- 4) Other mechanisms ---


        // --- 5) Telemetry / debug ---
        telemetry.addData("shooter velocity", shooter.getVelocity());
        telemetry.addData("shootBrace", shootBraceLatch.get());
        if (pinpoint != null) {
            telemetry.addData("pose", pinpoint.getEstimate());
        }
//        driveWithAim.debugDump(dbg, "drive");
        AprilTagObservation obs = scoringTarget.last();
        if (obs.hasTarget) {
            RobotConfig.AutoAim.AimOffset aimOffset = RobotConfig.AutoAim.aimOffsetForTag(obs.id);

            // Bearing to the configured *aim point* (not the tag center).
            double aimBearingRad = robotBearingToTagRelativePointRad(obs, aimOffset);
            if (Double.isFinite(aimBearingRad)
                    && Math.abs(aimBearingRad) <= (aimTuning.aimDeadbandRad * 5)) {
                telemetry.addLine(">>> AIMED <<<");
            }

            telemetry.addData("tagId", obs.id);
            telemetry.addData("distIn", obs.cameraRangeInches());
            telemetry.addData("bearingTagDeg", Math.toDegrees(obs.cameraBearingRad()));
            telemetry.addData(
                    "aimOffset(fwd,left)",
                    String.format("%.1f, %.1f", aimOffset.forwardInches, aimOffset.leftInches)
            );
            telemetry.addData("bearingAimDeg", Math.toDegrees(aimBearingRad));

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
     * Computes the robot-frame bearing (radians) to a tag-relative aim point.
     *
     * <p>0 rad means the aim point is straight ahead of the robot, + is to the left, - is to the right.</p>
     *
     * <p>This mirrors how DriveGuidance resolves tag-relative points internally, but we do it here
     * so telemetry can say “AIMED” based on the <b>corner point</b> instead of the tag center.</p>
     */
    private double robotBearingToTagRelativePointRad(
            AprilTagObservation obs,
            RobotConfig.AutoAim.AimOffset aimOffset
    ) {
        if (obs == null || !obs.hasTarget || obs.cameraToTagPose == null || aimOffset == null) {
            return Double.NaN;
        }
        if (cameraMountConfig == null) {
            return Double.NaN;
        }

        // Robot → tag (in robot frame)
        Pose3d robotToTag = CameraMountLogic.robotToTagPose(cameraMountConfig, obs.cameraToTagPose);
        Pose2d robotToTag2d = new Pose2d(robotToTag.xInches, robotToTag.yInches, robotToTag.yawRad);

        // Robot → aim point (tag-relative offset rotated by the tag heading)
        Pose2d robotToAimPoint = robotToTag2d.then(new Pose2d(
                aimOffset.forwardInches,
                aimOffset.leftInches,
                0.0
        ));

        return Math.atan2(robotToAimPoint.yInches, robotToAimPoint.xInches);
    }

    /**
     * Latches pose-lock while P2 is holding auto-aim (LB) and the driver is not commanding translation.
     *
     * <p>This makes "hold LB" a single driver action for: aim + brace + shoot.</p>
     * <p>Hysteresis prevents chatter when sticks hover near center.</p>
     */
    private void updateShootBraceEnabled() {
        // Only brace while the aiming driver (P2) is actively holding the aim button.
        // This avoids surprise "fighting the driver" behavior during normal driving.
        if (!gamepads.p2().leftBumper().isHeld()) {
            shootBraceLatch.reset(false);
            return;
        }

        // Treat the driver stick as "idle" only after it is clearly near center, and stay idle
        // until the stick is clearly moved again (hysteresis avoids chatter).
        double mag = gamepads.p1().leftStickMagnitude().get();
        shootBraceLatch.update(mag);
    }


    /**
     * Stop hook shared by all OpModes.
     */
    public void stopAny() {
        drivebase.stop();
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