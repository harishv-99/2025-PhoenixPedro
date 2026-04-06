package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
import edu.ftcphoenix.fw.drive.guidance.References;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.field.TagLayouts;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.ftc.FtcVision;
import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.localization.apriltag.TagOnlyPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagFusionPoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionPolicies;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelections;

/**
 * Central robot container / composition root for the Phoenix robot.
 *
 * <p>TeleOp and Auto OpModes stay intentionally thin and delegate into this class. The goal is not
 * for {@code PhoenixRobot} to own every mechanism detail, but to wire the major lanes together in
 * one readable place:</p>
 * <ul>
 *   <li><b>Lane 1:</b> local actuator subsystems such as {@link Shooter} and the drivetrain.</li>
 *   <li><b>Lane 3:</b> event-driven supervision and bindings via {@link ShooterSupervisor} and
 *       {@link Bindings}.</li>
 *   <li><b>Lane 4:</b> drive guidance / auto-aim overlays and AprilTag-backed gating.</li>
 * </ul>
 *
 * <p>That keeps the public robot surface small: OpModes mostly call the lifecycle methods here,
 * while the robot internals remain split into subsystem, supervisor, and guidance responsibilities.</p>
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
    private TagOnlyPoseEstimator tagLocalizer;
    private OdometryTagFusionPoseEstimator fusedLocalizer;
    private DriveSource stickDrive;
    private DriveSource driveWithAim;
    private CameraMountConfig cameraMountConfig;
    private AprilTagSensor tagSensor;
    private TagSelectionSource scoringSelection;
    private DriveGuidancePlan aimPlan;
    private DriveGuidancePlan.Tuning aimTuning;
    private DriveGuidanceQuery aimQuery;
    private BooleanSource aimReady = BooleanSource.constant(true);
    private BooleanSource aimOverride = BooleanSource.constant(false);
    private BooleanSource aimOkToShoot = BooleanSource.constant(true);
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

        // --- Vision + localization ---
        cameraMountConfig = RobotConfig.Vision.cameraMount;
        FtcVision.Config visionCfg = FtcVision.Config.defaults()
                .withCameraMount(cameraMountConfig);
        tagSensor = FtcVision.aprilTags(hardwareMap, RobotConfig.Vision.nameWebcam, visionCfg);

        // Use the framework's official-game fixed-field layout for localization/guidance.
        // Raw detections still see all FTC tags; only field-pose math is restricted to the tags
        // whose placement is actually deterministic.
        gameTagLayout = FtcGameTagLayout.currentGameFieldFixed();
        // Auto-aim only needs the fixed scoring tags. Using a subset view keeps the fixed-tag
        // policy framework-owned while making the selected-tag localization contract explicit:
        // every candidate scoring tag is present in this layout.
        TagLayout scoringTagLayout = TagLayouts.subsetOrSame(
                gameTagLayout,
                SCORING_TAG_IDS,
                "Phoenix scoring fixed-tag layout"
        );

        TagOnlyPoseEstimator.Config tagLocalizerCfg = RobotConfig.Localization.aprilTags.copy()
                .withCameraMount(cameraMountConfig);
        tagLocalizer = new TagOnlyPoseEstimator(tagSensor, gameTagLayout, tagLocalizerCfg);
        fusedLocalizer = new OdometryTagFusionPoseEstimator(
                pinpoint,
                tagLocalizer,
                RobotConfig.Localization.pinpointAprilTagFusion.copy()
        );

        // Auto-aim assist is explicitly driver-controlled (P2 left bumper). The selector
        // previews while idle, then latches the best visible scoring tag while the driver holds aim.
        BooleanSource autoAimEnabled = gamepads.p2().leftBumper();

        // --- Drive guidance (selected-tag auto-aim) ---
        // Semantic references stay geometry-first: one immutable reference, solved through the
        // shared selector's current selected tag. Different IDs may still use different offsets.
        RobotConfig.AutoAim.AimOffset blueOffset = RobotConfig.AutoAim.BLUE_AIM_OFFSET;
        RobotConfig.AutoAim.AimOffset redOffset = RobotConfig.AutoAim.RED_AIM_OFFSET;

        java.util.Map<Integer, References.TagPointOffset> aimOffsetsByTag =
                new java.util.LinkedHashMap<Integer, References.TagPointOffset>();
        aimOffsetsByTag.put(
                RobotConfig.AutoAim.BLUE_TARGET_TAG_ID,
                References.pointOffset(blueOffset.forwardInches, blueOffset.leftInches)
        );
        aimOffsetsByTag.put(
                RobotConfig.AutoAim.RED_TARGET_TAG_ID,
                References.pointOffset(redOffset.forwardInches, redOffset.leftInches)
        );

        scoringSelection = TagSelections.from(tagSensor)
                .among(SCORING_TAG_IDS)
                .freshWithin(0.50)
                .choose(TagSelectionPolicies.smallestAbsRobotBearing(cameraMountConfig))
                .stickyWhen(autoAimEnabled)
                .reacquireAfterLoss(0.20)
                .build();

        // Tuning for the auto-aim assist. Keep all guidance APIs in radians.
        aimTuning = DriveGuidancePlan.Tuning.defaults()
                .withAimKp(RobotConfig.AutoAim.AIM_KP)
                .withMaxOmegaCmd(RobotConfig.AutoAim.AIM_MAX_OMEGA_CMD)
                .withMinOmegaCmd(RobotConfig.AutoAim.AIM_MIN_OMEGA_CMD)
                .withAimDeadbandRad(Math.toRadians(RobotConfig.AutoAim.AIM_TOLERANCE_DEG));

        // Auto-aim remains a selected-tag reference, but the solve lane is now adaptive:
        // live tags are preferred when visible, and the fused localizer keeps the same selected
        // tag geometry alive when the goal tag temporarily drops out of view.
        aimPlan = DriveGuidance.plan()
                .aimTo()
                .point(References.relativeToSelectedTagPoint(scoringSelection, aimOffsetsByTag))
                .doneAimTo()
                .tuning(aimTuning)
                .resolveWith()
                .adaptive()
                .aprilTags(tagSensor, cameraMountConfig, 0.50)
                .aprilTagFieldPoseConfig(RobotConfig.Localization.aprilTags.toSolverConfig())
                .localization(fusedLocalizer)
                .fixedAprilTagLayout(scoringTagLayout)
                .omegaPolicy(DriveGuidanceSpec.OmegaPolicy.PREFER_APRIL_TAGS_WHEN_VALID)
                .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
                .doneResolveWith()
                .build();

        // Query: sample the exact same math used by the overlay (for telemetry & gating).
        aimQuery = aimPlan.query();

        // "Aimed" gate: safe to feed a ball only when we're facing the scoring target.
        //
        // Design choice:
        //  - If guidance can solve the selected target (from live tags or fused localization),
        //    require omega error to be within a configurable tolerance.
        //  - If guidance cannot solve this loop, allow manual aiming / manual velocity fallback.
        //
        // NOTE:
        // We intentionally allow the "aim ready" tolerance to be a bit looser than the
        // DriveGuidance deadband so we don't get stuck waiting on the last fraction of a degree.
        final double aimReadyTolRad = Math.toRadians(RobotConfig.AutoAim.AIM_READY_TOLERANCE_DEG);
        BooleanSource rawAimReady = new BooleanSource() {
            private long lastCycle = Long.MIN_VALUE;
            private boolean last = true;

            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;

                DriveGuidanceStatus status = aimQuery != null
                        ? aimQuery.sample(clock, DriveOverlayMask.OMEGA_ONLY)
                        : null;
                if (status != null && status.hasOmegaError) {
                    last = status.omegaWithin(aimReadyTolRad);
                    return last;
                }

                // No solvable aim state this loop: allow manual fallback rather than blocking feed.
                last = true;
                return last;
            }
        };

        // Debounce aim readiness very slightly so we don't fire on a single noisy frame.
        aimReady = rawAimReady.debouncedOn(RobotConfig.AutoAim.AIM_READY_DEBOUNCE_SEC);

        // Shooter supervisor: owns shoot policy + sensorless timing.
        // Gamepad bindings update the supervisor state; we expose that state as BooleanSources so it
        // can also enable auto-aim and shoot-brace.
        //
        // Optional emergency override: hold P2 Y while shooting to bypass the ready gates.
        //
        // Safety: we still require the flywheel to be enabled so we don't feed into a stopped wheel.
        aimOverride = gamepads.p2().y();
        aimOkToShoot = aimReady.or(aimOverride);

        shooterSupervisor = new ShooterSupervisor(shooter, scoringSelection, aimOkToShoot, aimOverride);

        // Enable condition for the guidance overlay.
        //
        // Stack multiple overlays without nested overlayWhen(...) calls.
        //
        // Order matters only when two enabled overlays claim the same DOF (the last layer wins).
        driveWithAim = DriveOverlayStack.on(stickDrive)
                .add(
                        "shootBrace",
                        BooleanSource.of(shootBraceLatch::get),
                        DriveGuidance.poseLock(
                                fusedLocalizer,
                                DriveGuidancePlan.Tuning.defaults()
                                        .withTranslateKp(0.08)
                                        .withMaxTranslateCmd(0.35)
                        ),
                        DriveOverlayMask.TRANSLATION_ONLY
                )
                .add(
                        "autoAim",
                        autoAimEnabled,
                        aimPlan.overlay(),
                        DriveOverlayMask.OMEGA_ONLY
                )
                .build();

        telemetry.addLine("Phoenix TeleOp (Supervisor + OutputTasks)");
        telemetry.addLine("P1: Left stick=drive, Right stick=turn, RB=slow mode");
        telemetry.addLine("P2: RB=toggle shooter flywheel (spins at selected velocity)");
        telemetry.addLine("P2: LB=auto aim + set velocity from AprilTag range");
        telemetry.addLine("P2: B=shoot (hold; release cancels)");
        telemetry.addLine("P2: Y=override shoot gates (hold; if flywheel ON, forces feed even if not ready)");
        telemetry.addLine("P2: A=toggle intake");
        telemetry.addLine("P2: X= eject / unjam (hold; reverse feeds)");
        telemetry.addLine("P2: DPad Up/Down=adjust selected velocity");
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

        // RB: toggle flywheel on/off
        bindings.onRise(gamepads.p2().rightBumper(), shooterSupervisor::toggleFlywheel);

        // LB: auto-aim assist button (drive overlay) + set velocity from tag range on press
        bindings.onRise(gamepads.p2().leftBumper(), () -> shooterSupervisor.captureVelocityFromTarget(clock));

        // B: shoot (hold-to-shoot; release cancels any queued/active feed task)
        bindings.onRiseAndFall(
                gamepads.p2().b(),
                () -> shooterSupervisor.setShootHeld(true),
                () -> shooterSupervisor.setShootHeld(false)
        );

        // X: eject / unjam (hold)
        bindings.onRiseAndFall(
                gamepads.p2().x(),
                () -> shooterSupervisor.setEjectHeld(true),
                () -> shooterSupervisor.setEjectHeld(false)
        );

        // D-pad: tweak the selected flywheel velocity (fine-tune after auto-set).
        bindings.onRise(gamepads.p2().dpadUp(), shooter::increaseSelectedVelocity);
        bindings.onRise(gamepads.p2().dpadDown(), shooter::decreaseSelectedVelocity);
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
        // --- Lane 3: perception + bindings ---
        bindings.update(clock);

        // --- Lane 4 support: fused field pose for pose lock / guidance fallback / debug ---
        if (fusedLocalizer != null) {
            fusedLocalizer.update(clock);
        } else if (pinpoint != null) {
            pinpoint.update(clock);
        }

        // --- Lane 3: shooter supervision over lane-1 mechanism outputs ---
        shooterSupervisor.update(clock);
        ShooterSupervisor.Status shooterStatus = shooterSupervisor.status();

        // --- Lane 4: drive assist / arbitration ---
        updateShootBraceEnabled(shooterStatus);
        DriveSignal cmd = driveWithAim.get(clock).clamped();
        drivebase.update(clock);
        drivebase.drive(cmd);

        // --- Lane 1: subsystem plant updates ---
        shooter.update(clock);

        // --- Driver telemetry / debug ---
        emitTeleOpTelemetry(shooterStatus);
    }

    private void emitTeleOpTelemetry(ShooterSupervisor.Status shooterStatus) {
        shooter.telemetryDump(clock, "shooter");
        telemetry.addData("shoot.mode", shooterStatus.mode);
        telemetry.addData("feed.backlog", shooterStatus.feedBacklog);
        telemetry.addData("intake.enabled", shooterStatus.intakeEnabled);
        telemetry.addData("eject.held", shooterStatus.ejectHeld);
        telemetry.addData("flywheel.toggle", shooterStatus.flywheelToggleEnabled);
        telemetry.addData("shoot.btn", shooterStatus.shootHeld);
        telemetry.addData("shoot.active", shooterStatus.shootActive);
        telemetry.addData("aim.ready", aimReady.getAsBoolean(clock));
        telemetry.addData("aim.okToShoot", aimOkToShoot.getAsBoolean(clock));
        telemetry.addData("aim.override", aimOverride.getAsBoolean(clock));
        telemetry.addData("shootBrace", shootBraceLatch.get());
        if (fusedLocalizer != null) {
            telemetry.addData("pose.fused", fusedLocalizer.getEstimate());
        }
        if (pinpoint != null) {
            telemetry.addData("pose.odom", pinpoint.getEstimate());
        }
//        driveWithAim.debugDump(dbg, "drive");

        DriveGuidanceStatus aimStatus = sampleCurrentAimStatus();
        TagSelectionResult sel = scoringSelection.get(clock);
        if (sel.hasSelection) {
            AprilTagObservation obs = sel.hasFreshSelectedObservation
                    ? sel.selectedObservation
                    : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
            emitCurrentTargetTelemetry(sel.selectedTagId, obs, sel.hasFreshSelectedObservation, aimStatus);
        }

        telemetry.update();
    }

    private DriveGuidanceStatus sampleCurrentAimStatus() {
        return aimQuery != null
                ? aimQuery.sample(clock, DriveOverlayMask.OMEGA_ONLY)
                : null;
    }

    private void emitCurrentTargetTelemetry(int tagId,
                                            AprilTagObservation obs,
                                            boolean tagVisible,
                                            DriveGuidanceStatus aimStatus) {
        RobotConfig.AutoAim.AimOffset aimOffset = RobotConfig.AutoAim.aimOffsetForTag(tagId);

        if (aimReady.getAsBoolean(clock)) {
            telemetry.addLine(">>> AIM READY <<<");
        } else if (aimOverride.getAsBoolean(clock)) {
            telemetry.addLine(">>> AIM OVERRIDE <<<");
        }

        telemetry.addData("tagId", tagId);
        telemetry.addData("tag.visible", tagVisible);
        telemetry.addData("distIn", tagVisible ? obs.cameraRangeInches() : Double.NaN);
        telemetry.addData("bearingTagDeg", tagVisible ? Math.toDegrees(obs.cameraBearingRad()) : Double.NaN);
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

        telemetry.addData(
                "aim.source",
                (aimStatus != null && aimStatus.hasOmegaError)
                        ? aimStatus.omegaSource
                        : DriveGuidanceStatus.ChannelSource.NONE
        );
        telemetry.addData("aim.tolDeg", RobotConfig.AutoAim.AIM_TOLERANCE_DEG);
        telemetry.addData("aim.readyTolDeg", RobotConfig.AutoAim.AIM_READY_TOLERANCE_DEG);

        if (gameTagLayout != null && gameTagLayout.has(tagId)) {
            Pose3d fieldToTag = gameTagLayout.requireFieldToTagPose(tagId);
            Pose2d fieldToAimPoint = new Pose2d(
                    fieldToTag.xInches,
                    fieldToTag.yInches,
                    fieldToTag.yawRad
            ).then(new Pose2d(aimOffset.forwardInches, aimOffset.leftInches, 0.0));

            telemetry.addData("field.tag", fieldToTag);
            telemetry.addData("field.aim", fieldToAimPoint);
        }
    }

    /**
     * Latches translation pose-lock while an active shooting sequence is in progress
     * <em>and</em> the driver is not commanding translation.
     *
     * <p>This makes "hold B" feel like a single driver action for: aim + brace + shoot.</p>
     * <p>Hysteresis prevents chatter when sticks hover near center.</p>
     */
    private void updateShootBraceEnabled(ShooterSupervisor.Status shooterStatus) {
        if (shooterStatus == null || !shooterStatus.shootActive) {
            shootBraceLatch.reset(false);
            return;
        }

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
        tagLocalizer = null;
        fusedLocalizer = null;
    }
}
