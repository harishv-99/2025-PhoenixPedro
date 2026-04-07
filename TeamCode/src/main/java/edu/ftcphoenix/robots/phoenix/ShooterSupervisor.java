package edu.ftcphoenix.robots.phoenix;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.OutputTask;
import edu.ftcphoenix.fw.task.Tasks;

/**
 * Policy/supervisor layer for Phoenix scoring.
 *
 * <p>The shooter subsystem remains the single writer to the plants. This class owns policy and
 * driver/auto intent: intake vs eject vs shoot priority, flywheel request state, and the feed-task
 * queueing rules.</p>
 */
public final class ShooterSupervisor {

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
    private final PhoenixProfile.ShooterConfig cfg;
    private final BooleanSource aimOkToShoot;
    private final BooleanSource shootOverride;

    private boolean intakeEnabled = false;
    private boolean ejectRequested = false;
    private boolean shootingRequested = false;
    private boolean flywheelRequested = false;

    /**
     * Creates the scoring supervisor.
     *
     * @param shooter shooter subsystem that owns the actual plants and feed queue
     * @param config shooter/scoring config snapshot copied for local policy use
     * @param aimOkToShoot gate that becomes true when aiming policy allows feeding a shot
     * @param shootOverride operator override used to bypass flywheel readiness while the flywheel is on
     */
    public ShooterSupervisor(Shooter shooter,
                             PhoenixProfile.ShooterConfig config,
                             BooleanSource aimOkToShoot,
                             BooleanSource shootOverride) {
        this.shooter = Objects.requireNonNull(shooter, "shooter");
        this.cfg = Objects.requireNonNull(config, "config").copy();
        this.aimOkToShoot = Objects.requireNonNull(aimOkToShoot, "aimOkToShoot").memoized();
        this.shootOverride = Objects.requireNonNull(shootOverride, "shootOverride").memoized();
    }

    /**
     * Returns whether intake mode is currently latched on.
     *
     * @return {@code true} when the supervisor should pull game pieces inward
     */
    public boolean intakeEnabled() {
        return intakeEnabled;
    }

    /**
     * Returns whether the flywheel has been requested on.
     *
     * @return {@code true} when the supervisor wants the flywheel enabled
     */
    public boolean flywheelRequested() {
        return flywheelRequested;
    }

    /**
     * Returns whether the caller is actively requesting shooting.
     *
     * @return {@code true} while shooting intent is being held/requested
     */
    public boolean isShootingRequested() {
        return shootingRequested;
    }

    /**
     * Requests that the flywheel be enabled or disabled.
     *
     * @param enabled {@code true} to request flywheel operation, {@code false} to spin down and
     *                clear any pending shots
     */
    public void setFlywheelEnabled(boolean enabled) {
        flywheelRequested = enabled;
        if (!enabled) {
            clearPendingShots();
        }
    }

    /**
     * Requests that intake mode be enabled or disabled.
     *
     * @param enabled {@code true} to run intake mode, {@code false} to return to idle behavior
     */
    public void setIntakeEnabled(boolean enabled) {
        intakeEnabled = enabled;
        if (enabled) {
            clearPendingShots();
        }
    }

    /**
     * Starts shooting mode.
     */
    public void beginShooting() {
        shootingRequested = true;
    }

    /**
     * Ends shooting mode and clears any pending queued shots.
     */
    public void endShooting() {
        shootingRequested = false;
        clearPendingShots();
    }

    /**
     * Starts eject/unjam mode and clears pending shots.
     */
    public void beginEject() {
        ejectRequested = true;
        clearPendingShots();
    }

    /**
     * Ends eject/unjam mode.
     */
    public void endEject() {
        ejectRequested = false;
    }

    /**
     * Cancels queued transient actions such as pending shot feed tasks.
     */
    public void cancelTransientActions() {
        clearPendingShots();
    }

    /**
     * Returns a compact status snapshot of the supervisor's current requested state.
     *
     * @return immutable scoring status snapshot for telemetry and coordination
     */
    public ScoringStatus status() {
        int backlog = shooter.feedQueue().backlogCount();
        FeedMode mode = selectFeedMode(backlog);
        boolean shootActive = shootingRequested || backlog > 0;
        return new ScoringStatus(
                intakeEnabled,
                ejectRequested,
                shootingRequested,
                flywheelRequested,
                shootActive,
                backlog,
                mode.debugName
        );
    }

    /**
     * Writes a compact supervisor dump into the provided debug sink.
     *
     * @param dbg destination debug sink; ignored when {@code null}
     * @param prefix key prefix to use for emitted fields, or {@code null}/empty for {@code scoring}
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "scoring" : prefix;
        ScoringStatus s = status();
        dbg.addData(p + ".mode", s.mode)
                .addData(p + ".intakeEnabled", s.intakeEnabled)
                .addData(p + ".ejectRequested", s.ejectRequested)
                .addData(p + ".shootingRequested", s.shootingRequested)
                .addData(p + ".flywheelRequested", s.flywheelRequested)
                .addData(p + ".shootActive", s.shootActive)
                .addData(p + ".feedBacklog", s.feedBacklog);
    }

    /**
     * Advances scoring policy for one loop cycle.
     *
     * @param clock shared loop clock for the current OpMode cycle
     */
    public void update(LoopClock clock) {
        shooter.setFlywheelEnabled(flywheelRequested && !ejectRequested);

        if (!shootingRequested && shooter.feedQueue().backlogCount() > 0) {
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

    private OutputTask shootOneTask() {
        BooleanSource flywheelArmed = BooleanSource.of(shooter::flywheelEnabled);
        BooleanSource flywheelOkToFeed = shooter.flywheelReady().or(shootOverride.and(flywheelArmed));
        BooleanSource startWhen = flywheelOkToFeed.and(aimOkToShoot);
        BooleanSource doneWhen = BooleanSource.constant(true);

        return Tasks.gatedOutputUntil(
                "shootOne",
                startWhen,
                doneWhen,
                ScalarSource.constant(cfg.shootFeedPower),
                0.0,
                cfg.shootFeedPulseSec,
                cfg.shootFeedPulseSec + 1.0,
                cfg.shootFeedCooldownSec
        );
    }

    private FeedMode selectFeedMode(int backlog) {
        if (ejectRequested) {
            return FeedMode.EJECT;
        }
        if (shootingRequested || backlog > 0) {
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
        shooter.setManualIntakeMotorPower(cfg.intakeMotorPower);
        shooter.setManualIntakeTransferPower(cfg.intakeTransferPower);
        shooter.setManualShooterTransferPower(-cfg.intakeShooterTransferHoldBackPower);
    }

    private void applyEject() {
        shooter.setManualIntakeMotorPower(-cfg.ejectMotorPower);
        shooter.setManualIntakeTransferPower(-cfg.ejectTransferPower);
        shooter.setManualShooterTransferPower(-cfg.ejectShooterTransferPower);
    }

    private void applyIdleFeeds() {
        shooter.setManualIntakeMotorPower(0.0);
        shooter.setManualIntakeTransferPower(0.0);
        shooter.setManualShooterTransferPower(0.0);
    }
}
