package edu.ftcphoenix.robots.phoenix;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagTarget;
import edu.ftcphoenix.fw.task.OutputTask;
import edu.ftcphoenix.fw.task.Tasks;

/**
 * Policy/supervisor layer for the Phoenix shooter + ball path.
 *
 * <p><b>Key idea:</b> this class owns the "rules" (intake vs shoot vs eject priority) and builds
 * timed/gated feed tasks, but it does <b>not</b> read the gamepad directly.
 *
 * <p>{@link PhoenixRobot} owns gamepad bindings and calls the small "intent" methods here.
 * That keeps <b>all button mappings in one place</b> (less confusing for students) while still
 * keeping shooter logic out of the already-large robot container.
 *
 * <p>Gamepad 2 controls (wired in {@link PhoenixRobot#createBindings()}):
 * <ul>
 *   <li><b>RB</b>: toggle shooter flywheel on/off (spins at the currently selected velocity)</li>
 *   <li><b>LB</b>: auto-aim assist (handled in drive overlay) + sets velocity from the range lookup table</li>
 *   <li><b>B</b>: shoot (hold to keep shooting; release cancels all shoot commands)</li>
 *   <li><b>A</b>: toggle intake</li>
 *   <li><b>X</b>: eject / unjam (hold; reverses intake + transfers)</li>
 *   <li><b>DPad</b>: adjust selected shooter velocity (fine-tune after auto-set)</li>
 * </ul>
 *
 * <p>Important behavior notes:
 * <ul>
 *   <li>While <b>intaking</b>, we spin the shooter transfer <b>backwards</b> to prevent balls from
 *       reaching the shooter wheel and launching.</li>
 *   <li>Shooter transfer uses two mismatched CR servos; the <b>left</b> servo can be power-scaled in
 *       {@link RobotConfig.Shooter#shooterTransferLeftScale} so both sides move balls similarly.</li>
 * </ul>
 */
public final class ShooterSupervisor {

    private final Shooter shooter;
    private final TagTarget scoringTarget;
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
     * @param scoringTarget AprilTag target tracker used for range lookup
     * @param aimOkToShoot  gate that returns true when it is OK to shoot (aim-ready OR override)
     * @param shootOverride driver override signal (held). Used for emergency bypass of ready gates.
     */
    public ShooterSupervisor(Shooter shooter,
                             TagTarget scoringTarget,
                             BooleanSource aimOkToShoot,
                             BooleanSource shootOverride) {
        this.shooter = Objects.requireNonNull(shooter, "shooter");
        this.scoringTarget = Objects.requireNonNull(scoringTarget, "scoringTarget");
        this.aimOkToShoot = Objects.requireNonNull(aimOkToShoot, "aimOkToShoot").memoized();
        this.shootOverride = Objects.requireNonNull(shootOverride, "shootOverride").memoized();
    }

    /**
     * Returns true if we are in an active shooting attempt.
     *
     * <p>This includes "waiting" (spinning/aiming) and "feeding" (a queued feed pulse).
     */
    public boolean isShootActive() {
        return shootHeld || shooter.feedQueue().backlogCount() > 0;
    }

    // ---------------------------------------------------------------------
    // Intent API (called from bindings)
    // ---------------------------------------------------------------------

    public boolean intakeEnabled() {
        return intakeEnabled;
    }

    public boolean flywheelEnabled() {
        return flywheelEnabled;
    }

    public boolean isShootHeld() {
        return shootHeld;
    }

    public void toggleFlywheel() {
        flywheelEnabled = !flywheelEnabled;

        // If the driver turned the flywheel off, stop any in-progress feed immediately.
        if (!flywheelEnabled) {
            shooter.clearFeedQueue();
        }
    }

    public void toggleIntake() {
        intakeEnabled = !intakeEnabled;

        // If we just enabled intake, clear any pending shots so we don't accidentally advance a ball.
        if (intakeEnabled) {
            shooter.clearFeedQueue();
        }
    }

    /**
     * Shoot is hold-to-shoot.
     *
     * <p>Releasing the button cancels all queued/active shoot feed tasks immediately.</p>
     */
    public void setShootHeld(boolean held) {
        this.shootHeld = held;

        if (!held) {
            shooter.clearFeedQueue();
        }
    }

    public void setEjectHeld(boolean held) {
        ejectHeld = held;
        if (held) {
            // Eject is an override. Stop feeding so we don't accidentally launch.
            shooter.clearFeedQueue();
        }
    }

    /**
     * Capture an auto-set velocity from the current AprilTag range.
     *
     * <p>This is designed to be called on the <b>rising edge</b> of the auto-aim button.
     * The driver can still fine-tune the selected velocity afterward using D-pad.</p>
     */
    public void captureVelocityFromTarget() {
        if (!scoringTarget.hasTarget()) {
            return;
        }
        double rangeInches = scoringTarget.lineOfSightRangeInches();
        shooter.setSelectedVelocity(shooter.velocityForRangeInches(rangeInches));
    }

    // ---------------------------------------------------------------------
    // Loop
    // ---------------------------------------------------------------------

    /**
     * Call once per loop.
     */
    public void update(LoopClock clock) {

        // Always command the flywheel state (with safety overrides).
        // We preserve the driver's toggle, but suppress the flywheel while ejecting.
        shooter.setFlywheelEnabled(flywheelEnabled && !ejectHeld);

        // -----------------------------------------------------------------
        // Overrides: Eject has highest priority.
        // -----------------------------------------------------------------

        if (ejectHeld) {
            applyEject();
            return;
        }

        // -----------------------------------------------------------------
        // Shooting: hold-to-shoot keeps 1 shot buffered.
        // -----------------------------------------------------------------

        if (!shootHeld) {
            // Safety: if the driver isn't holding shoot, there should be no queued/active shoot tasks.
            shooter.clearFeedQueue();
        } else {
            shooter.feedQueue().ensureBacklog(clock, 1, this::shootOneTask);
        }

        boolean shotsBuffered = shooter.feedQueue().backlogCount() > 0;

        // If shots are buffered (or the shoot button is held), keep manual feed targets at 0
        // so the queued feed task has clean control of the path.
        if (shootHeld || shotsBuffered) {
            applyIdleFeeds();
            return;
        }

        // -----------------------------------------------------------------
        // Idle / Intake state
        // -----------------------------------------------------------------

        if (intakeEnabled) {
            applyIntake();
        } else {
            applyIdleFeeds();
        }
    }

    // ---------------------------------------------------------------------
    // Tasks + helpers
    // ---------------------------------------------------------------------

    /**
     * Build a single "shoot one" feed task.
     *
     * <p>Each shot waits for flywheel-ready, then feeds for a fixed pulse time.
     * While the driver holds shoot, the queue keeps exactly one buffered task.</p>
     */
    private OutputTask shootOneTask() {
        // Only feed once we are "ready".
        //
        // Normal mode: require flywheel-ready AND aim-ok.
        //
        // Emergency override mode:
        //  - If the driver holds the override button while shooting, we bypass the ready checks.
        //  - Safety: we still require the flywheel to be ENABLED so we don't feed into a stopped wheel.
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
