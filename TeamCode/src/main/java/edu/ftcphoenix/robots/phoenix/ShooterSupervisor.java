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
 *   <li><b>A</b>: toggle intake</li>
 *   <li><b>B</b>: shoot (tap for one, hold to keep shooting)</li>
 *   <li><b>X</b>: eject / unjam (hold; reverses intake + transfers)</li>
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

    // ---------------------------------------------------------------------
    // Latched state (set by bindings)
    // ---------------------------------------------------------------------

    private boolean intakeEnabled = false;
    private boolean ejectHeld = false;
    private boolean shootHeld = false;

    // Edge-like request flag: set on a press, consumed in update().
    private boolean shootOneRequested = false;

    // Keep flywheel running briefly after the last shoot request.
    private double keepFlywheelUntilSec = 0.0;

    public ShooterSupervisor(Shooter shooter, TagTarget scoringTarget) {
        this.shooter = Objects.requireNonNull(shooter, "shooter");
        this.scoringTarget = Objects.requireNonNull(scoringTarget, "scoringTarget");
    }

    // ---------------------------------------------------------------------
    // Intent API (called from bindings)
    // ---------------------------------------------------------------------

    public boolean intakeEnabled() {
        return intakeEnabled;
    }

    public boolean isShootHeld() {
        return shootHeld;
    }

    public void toggleIntake() {
        intakeEnabled = !intakeEnabled;

        // If we just enabled intake, cancel shooting so we don't feed into the wheel.
        if (intakeEnabled) {
            shootHeld = false;
            stopShooting();
        }
    }

    public void setShootHeld(boolean held) {
        this.shootHeld = held;

        // If the driver released the shoot button, don't force-stop the flywheel immediately;
        // keep-alive logic handles a short grace period for fast follow-ups.
    }

    public void requestShootOne() {
        shootOneRequested = true;

        // If we're starting a shoot request, intake should not "win" over it.
        // (We keep intakeEnabled latched so it resumes after shooting.)
    }

    public void setEjectHeld(boolean held) {
        ejectHeld = held;
        if (held) {
            // Eject is an override. Stop shooting so we don't accidentally launch.
            shootHeld = false;
            stopShooting();
        }
    }

    // ---------------------------------------------------------------------
    // Loop
    // ---------------------------------------------------------------------

    /**
     * Call once per loop.
     */
    public void update(LoopClock clock) {

        // -----------------------------------------------------------------
        // Overrides: Eject has highest priority.
        // -----------------------------------------------------------------

        if (ejectHeld) {
            // Keep shooter disabled while eject is held.
            stopShooting();
            applyEject();
            return;
        }

        // -----------------------------------------------------------------
        // Shooting: tap queues one; hold keeps one queued at all times.
        // -----------------------------------------------------------------

        if (shootOneRequested) {
            shootOneRequested = false;
            shooter.feedQueue().enqueue(shootOneTask(clock));
        }

        if (shootHeld) {
            // Keep one "shoot one" task buffered so shots happen as quickly as possible.
            shooter.feedQueue().ensureBacklog(clock, 1, () -> shootOneTask(clock));
        }

        boolean shotsBuffered = shooter.feedQueue().backlogCount() > 0;

        // If we're intaking (and not currently shooting), don't keep the flywheel alive.
        if (intakeEnabled && !shootHeld && !shotsBuffered) {
            keepFlywheelUntilSec = 0.0;
        }

        boolean keepAlive = clock.nowSec() < keepFlywheelUntilSec;
        boolean flywheelShouldRun = shootHeld || shotsBuffered || keepAlive;

        if (flywheelShouldRun) {
            shooter.setMacroShooterEnabled(true);
            updateMacroVelocity(clock);
        } else {
            shooter.setMacroShooterEnabled(false);
        }

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
     * The queue machinery keeps exactly one buffered task while the driver holds shoot.
     */
    private OutputTask shootOneTask(LoopClock clock) {
        keepFlywheelUntilSec = Math.max(
                keepFlywheelUntilSec,
                clock.nowSec() + RobotConfig.Shooter.flywheelKeepAliveSec
        );

        BooleanSource startWhen = shooter.flywheelReady();
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

    private void stopShooting() {
        keepFlywheelUntilSec = 0.0;
        shooter.clearFeedQueue();
        shooter.setMacroShooterEnabled(false);
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

    private void updateMacroVelocity(LoopClock clock) {
        if (scoringTarget.hasTarget()) {
            double rangeInches = scoringTarget.lineOfSightRangeInches();
            shooter.setMacroShooterVelocity(shooter.velocityForRangeInches(rangeInches));
        } else {
            // No tag: fall back to whatever the driver dialed in.
            shooter.setMacroShooterVelocity(shooter.manualShooterVelocity());
        }
    }
}
