package edu.ftcphoenix.robots.phoenix;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcphoenix.fw.task.OutputTask;
import edu.ftcphoenix.fw.task.Tasks;

/**
 * Policy/supervisor layer for the Phoenix shooter + feed path.
 *
 * <p>This class is the lane-3 piece of the shooter stack:</p>
 * <ul>
 *   <li>{@link Shooter} owns the lane-1 actuator details (plants, local setpoints, feed queue).</li>
 *   <li>{@link ShooterSupervisor} owns the event-driven policy: intake vs shoot vs eject priority,
 *       "hold to shoot" behavior, and feed-task creation.</li>
 *   <li>{@link PhoenixRobot} owns button bindings and forwards high-level driver intent here.</li>
 * </ul>
 *
 * <p><b>Key idea:</b> this class does <b>not</b> read the gamepad directly. That keeps button
 * mappings in one place while still keeping shooter logic out of the robot container.</p>
 *
 * <p>Typical usage from a robot container:</p>
 * <pre>{@code
 * ShooterSupervisor supervisor = new ShooterSupervisor(shooter, scoringSelection, aimOkToShoot, aimOverride);
 *
 * bindings.onRise(gamepads.p2().a(), supervisor::toggleIntake);
 * bindings.onRise(gamepads.p2().rightBumper(), supervisor::toggleFlywheel);
 * bindings.onRiseAndFall(gamepads.p2().b(),
 *         () -> supervisor.setShootHeld(true),
 *         () -> supervisor.setShootHeld(false));
 *
 * // In the loop:
 * supervisor.update(clock);
 * ShooterSupervisor.Status status = supervisor.status();
 * }</pre>
 */
public final class ShooterSupervisor {

    /**
     * Immutable snapshot for telemetry and debugging.
     *
     * <p>The intent is that robot code can ask the supervisor for one compact status object instead
     * of having to understand several internal booleans and queue details.</p>
     */
    public static final class Status {
        public final boolean intakeEnabled;
        public final boolean ejectHeld;
        public final boolean shootHeld;
        public final boolean flywheelToggleEnabled;
        public final boolean shootActive;
        public final int feedBacklog;
        public final String mode;

        private Status(boolean intakeEnabled,
                       boolean ejectHeld,
                       boolean shootHeld,
                       boolean flywheelToggleEnabled,
                       boolean shootActive,
                       int feedBacklog,
                       String mode) {
            this.intakeEnabled = intakeEnabled;
            this.ejectHeld = ejectHeld;
            this.shootHeld = shootHeld;
            this.flywheelToggleEnabled = flywheelToggleEnabled;
            this.shootActive = shootActive;
            this.feedBacklog = feedBacklog;
            this.mode = mode;
        }
    }

    /**
     * Internal feed-path modes used to keep priority logic readable.
     */
    private enum FeedMode {
        IDLE("IDLE"),
        INTAKE("INTAKE"),
        EJECT("EJECT"),
        SHOOT("SHOOT");

        final String debugName;

        FeedMode(String debugName) {
            this.debugName = debugName;
        }
    }

    private final Shooter shooter;
    private final TagSelectionSource scoringSelection;
    private final BooleanSource aimOkToShoot;
    private final BooleanSource shootOverride;

    // ---------------------------------------------------------------------
    // Latched state (set by bindings)
    // ---------------------------------------------------------------------

    private boolean intakeEnabled = false;
    private boolean ejectHeld = false;
    private boolean shootHeld = false;

    /**
     * Flywheel toggle (P2 right bumper).
     */
    private boolean flywheelEnabled = false;

    /**
     * Create a shooter supervisor.
     *
     * @param shooter       shooter subsystem
     * @param scoringSelection shared selected-tag source used for range lookup
     * @param aimOkToShoot  gate that returns true when it is OK to shoot (aim-ready OR override)
     * @param shootOverride driver override signal (held). Used for emergency bypass of ready gates.
     */
    public ShooterSupervisor(Shooter shooter,
                             TagSelectionSource scoringSelection,
                             BooleanSource aimOkToShoot,
                             BooleanSource shootOverride) {
        this.shooter = Objects.requireNonNull(shooter, "shooter");
        this.scoringSelection = Objects.requireNonNull(scoringSelection, "scoringSelection");
        this.aimOkToShoot = Objects.requireNonNull(aimOkToShoot, "aimOkToShoot").memoized();
        this.shootOverride = Objects.requireNonNull(shootOverride, "shootOverride").memoized();
    }

    /**
     * Returns true if we are in an active shooting attempt.
     *
     * <p>This includes both "waiting" (spinning/aiming) and "feeding" (a queued feed pulse).</p>
     */
    public boolean isShootActive() {
        return status().shootActive;
    }

    // ---------------------------------------------------------------------
    // Intent API (called from bindings)
    // ---------------------------------------------------------------------

    /**
     * Returns whether intake mode is currently enabled.
     */
    public boolean intakeEnabled() {
        return intakeEnabled;
    }

    /**
     * Returns whether the flywheel toggle is currently enabled.
     */
    public boolean flywheelEnabled() {
        return flywheelEnabled;
    }

    /**
     * Returns whether the operator is currently holding the shoot request.
     */
    public boolean isShootHeld() {
        return shootHeld;
    }

    /**
     * Toggles the flywheel enable latch.
     */
    public void toggleFlywheel() {
        flywheelEnabled = !flywheelEnabled;

        if (!flywheelEnabled) {
            clearPendingShots();
        }
    }

    /**
     * Toggles the intake enable latch.
     */
    public void toggleIntake() {
        intakeEnabled = !intakeEnabled;

        if (intakeEnabled) {
            clearPendingShots();
        }
    }

    /**
     * Shoot is hold-to-shoot.
     *
     * <p>Releasing the button cancels all queued/active shoot feed tasks immediately.</p>
     */
    public void setShootHeld(boolean held) {
        shootHeld = held;

        if (!held) {
            clearPendingShots();
        }
    }

    /**
     * Sets whether eject mode is currently being requested.
     */
    public void setEjectHeld(boolean held) {
        ejectHeld = held;
        if (held) {
            clearPendingShots();
        }
    }

    /**
     * Capture an auto-set velocity from the current AprilTag range.
     *
     * <p>This is designed to be called on the <b>rising edge</b> of the auto-aim button. The
     * driver can still fine-tune the selected velocity afterward using D-pad.</p>
     */
    public void captureVelocityFromTarget(LoopClock clock) {
        TagSelectionResult sel = scoringSelection.get(clock);
        AprilTagObservation obs = sel.hasFreshSelectedObservation
                ? sel.selectedObservation
                : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
        if (!obs.hasTarget) {
            return;
        }
        double rangeInches = obs.cameraRangeInches();
        shooter.setSelectedVelocity(shooter.velocityForRangeInches(rangeInches));
    }

    /**
     * Return a compact snapshot of the current supervisor state.
     *
     * <p>This is the preferred way for robot code to inspect shooter policy state for telemetry,
     * debugging, and higher-level coordination.</p>
     */
    public Status status() {
        int backlog = shooter.feedQueue().backlogCount();
        FeedMode mode = selectFeedMode(backlog);
        boolean shootActive = shootHeld || backlog > 0;
        return new Status(
                intakeEnabled,
                ejectHeld,
                shootHeld,
                flywheelEnabled,
                shootActive,
                backlog,
                mode.debugName
        );
    }

    /**
     * Emit a compact debug summary of the current state.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "shooterSup" : prefix;
        Status s = status();
        dbg.addData(p + ".mode", s.mode)
                .addData(p + ".intakeEnabled", s.intakeEnabled)
                .addData(p + ".ejectHeld", s.ejectHeld)
                .addData(p + ".shootHeld", s.shootHeld)
                .addData(p + ".flywheelToggleEnabled", s.flywheelToggleEnabled)
                .addData(p + ".shootActive", s.shootActive)
                .addData(p + ".feedBacklog", s.feedBacklog);
    }

    // ---------------------------------------------------------------------
    // Loop
    // ---------------------------------------------------------------------

    /**
     * Call once per loop.
     */
    public void update(LoopClock clock) {

        // Always command the flywheel state. We preserve the driver's toggle, but suppress the
        // flywheel while ejecting.
        shooter.setFlywheelEnabled(flywheelEnabled && !ejectHeld);

        // Releasing shoot should immediately drop any buffered feed tasks before we decide what the
        // rest of this loop should do (for example, resume intake).
        if (!shootHeld && shooter.feedQueue().backlogCount() > 0) {
            clearPendingShots();
        }

        FeedMode mode = selectFeedMode(shooter.feedQueue().backlogCount());

        switch (mode) {
            case EJECT:
                applyEject();
                return;

            case SHOOT:
                shooter.feedQueue().ensureBacklog(clock, 1, this::shootOneTask);
                applyIdleFeeds();
                return;

            case INTAKE:
                applyIntake();
                return;

            case IDLE:
            default:
                applyIdleFeeds();
        }
    }

    // ---------------------------------------------------------------------
    // Tasks + helpers
    // ---------------------------------------------------------------------

    /**
     * Build a single "shoot one" feed task.
     *
     * <p>Each shot waits for flywheel-ready, then feeds for a fixed pulse time. While the driver
     * holds shoot, the queue keeps exactly one buffered task.</p>
     */
    private OutputTask shootOneTask() {
        // Only feed once we are "ready".
        //
        // Normal mode: require flywheel-ready AND aim-ok.
        //
        // Emergency override mode:
        //  - If the driver holds the override button while shooting, we bypass the ready checks.
        //  - Safety: we still require the flywheel to be ENABLED so we don't feed into a stopped
        //    wheel.
        BooleanSource flywheelArmed = BooleanSource.of(shooter::flywheelEnabled);
        BooleanSource flywheelOkToFeed = shooter.flywheelReady().or(shootOverride.and(flywheelArmed));
        BooleanSource startWhen = flywheelOkToFeed.and(aimOkToShoot);
        BooleanSource doneWhen = BooleanSource.constant(true);

        return Tasks.gatedOutputUntil(
                "shootOne",
                startWhen,
                doneWhen,
                ScalarSource.constant(RobotConfig.Shooter.shootFeedPower),
                0.0,
                RobotConfig.Shooter.shootFeedPulseSec,
                RobotConfig.Shooter.shootFeedPulseSec + 1.0,
                RobotConfig.Shooter.shootFeedCooldownSec
        );
    }

    private FeedMode selectFeedMode(int backlog) {
        if (ejectHeld) {
            return FeedMode.EJECT;
        }
        if (shootHeld || backlog > 0) {
            return FeedMode.SHOOT;
        }
        if (intakeEnabled) {
            return FeedMode.INTAKE;
        }
        return FeedMode.IDLE;
    }

    private void clearPendingShots() {
        shooter.clearFeedQueue();
    }

    private void applyIntake() {
        shooter.setManualIntakeMotorPower(RobotConfig.Shooter.intakeMotorPower);
        shooter.setManualIntakeTransferPower(RobotConfig.Shooter.intakeTransferPower);

        // Hold balls back from the shooter wheel.
        shooter.setManualShooterTransferPower(-RobotConfig.Shooter.intakeShooterTransferHoldBackPower);
    }

    private void applyEject() {
        shooter.setManualIntakeMotorPower(-RobotConfig.Shooter.ejectMotorPower);
        shooter.setManualIntakeTransferPower(-RobotConfig.Shooter.ejectTransferPower);
        shooter.setManualShooterTransferPower(-RobotConfig.Shooter.ejectShooterTransferPower);
    }

    private void applyIdleFeeds() {
        shooter.setManualIntakeMotorPower(0.0);
        shooter.setManualIntakeTransferPower(0.0);
        shooter.setManualShooterTransferPower(0.0);
    }
}
