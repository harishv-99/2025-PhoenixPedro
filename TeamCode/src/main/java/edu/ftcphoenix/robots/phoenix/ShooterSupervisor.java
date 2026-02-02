package edu.ftcphoenix.robots.phoenix;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.input.GamepadDevice;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagTarget;
import edu.ftcphoenix.fw.task.OutputTask;
import edu.ftcphoenix.fw.task.Tasks;

/**
 * Policy/supervisor layer for the Phoenix shooter + ball path.
 *
 * <p>Responsibilities:
 * <ul>
 *   <li>Read driver intent for manual backups (individual plants).</li>
 *   <li>Own the "shoot all" macro (currently sensorless / time-based).</li>
 *   <li>Pick flywheel velocity based on AprilTag range when available.</li>
 * </ul>
 */
public final class ShooterSupervisor {

    private final Shooter shooter;
    private final GamepadDevice gp2;
    private final TagTarget scoringTarget;

    private boolean shootAllRequested = false;
    private boolean shootAllCancelRequested = false;
    private boolean shootAllActive = false;

    public ShooterSupervisor(Shooter shooter, GamepadDevice gp2, TagTarget scoringTarget) {
        this.shooter = Objects.requireNonNull(shooter, "shooter");
        this.gp2 = Objects.requireNonNull(gp2, "gp2");
        this.scoringTarget = Objects.requireNonNull(scoringTarget, "scoringTarget");
    }

    /**
     * Edge-triggered by a binding (e.g. P2 RT rising edge).
     */
    public void requestShootAll() {
        shootAllRequested = true;
    }

    /**
     * Edge-triggered by a binding (e.g. P2 RT falling edge).
     */
    public void cancelShootAll() {
        shootAllCancelRequested = true;
    }

    public boolean isShootAllActive() {
        return shootAllActive;
    }

    /**
     * Call once per loop. Applies manual backups and manages the macro lifecycle.
     */
    public void update(LoopClock clock) {

        // --- Manual backups (base layer) ---
        // These remain available even if sensors are missing.
        updateManualFeed(clock);

        // --- Macro state machine ---
        if (shootAllCancelRequested) {
            shootAllCancelRequested = false;
            stopMacro();
        }

        if (shootAllRequested) {
            shootAllRequested = false;
            startMacro(clock);
        }

        if (shootAllActive) {
            updateMacroVelocity(clock);

            // End condition: feed queue is empty (macro has fed N balls).
            if (shooter.feedQueue().isIdle()) {
                stopMacro();
            }
        }
    }

    private void updateManualFeed(LoopClock clock) {
        // Emergency unjam: hold X to reverse everything.
        if (gp2.x().getAsBoolean(clock)) {
            shooter.setManualIntakeTransferPower(-RobotConfig.Shooter.feedPower);
            shooter.setManualTransferMotorPower(-RobotConfig.Shooter.feedPower);
            shooter.setManualShooterTransferPower(-RobotConfig.Shooter.feedPower);
            return;
        }

        // Force-feed forward: hold B to run all three feed stages forward.
        if (gp2.b().getAsBoolean(clock)) {
            shooter.setManualIntakeTransferPower(RobotConfig.Shooter.feedPower);
            shooter.setManualTransferMotorPower(RobotConfig.Shooter.feedPower);
            shooter.setManualShooterTransferPower(RobotConfig.Shooter.feedPower);
            return;
        }

        // Otherwise, allow individual plant control.
        shooter.setManualIntakeTransferPower(
                gp2.leftTrigger().getAsDouble(clock) * RobotConfig.Shooter.manualIntakeTransferMaxPower
        );

        shooter.setManualTransferMotorPower(
                gp2.leftBumper().getAsBoolean(clock) ? RobotConfig.Shooter.manualTransferMotorPower : 0.0
        );

        shooter.setManualShooterTransferPower(
                gp2.rightBumper().getAsBoolean(clock) ? RobotConfig.Shooter.manualShooterTransferPower : 0.0
        );
    }

    private void startMacro(LoopClock clock) {
        shootAllActive = true;

        shooter.clearFeedQueue();

        // Flywheel: macro overrides manual flywheel while active.
        shooter.setMacroShooterEnabled(true);
        updateMacroVelocity(clock);

        // Feed "N balls" (sensorless): each feed pulse waits for flywheel-ready,
        // runs for feedPulseSec, then cools down for feedCooldownSec.
        BooleanSource startWhen = shooter.flywheelReady();
        BooleanSource doneWhen = BooleanSource.constant(true);

        for (int i = 0; i < RobotConfig.Shooter.ballCapacity; i++) {
            String name = "shootAll.feed[" + i + "]";
            OutputTask feedOne = Tasks.gatedOutputUntil(
                    name,
                    startWhen,
                    doneWhen,
                    ScalarSource.constant(RobotConfig.Shooter.feedPower),
                    0.0,
                    RobotConfig.Shooter.feedPulseSec,
                    RobotConfig.Shooter.feedPulseSec + 1.0,
                    RobotConfig.Shooter.feedCooldownSec
            );
            shooter.feedQueue().enqueue(feedOne);
        }
    }

    private void stopMacro() {
        shootAllActive = false;
        shooter.clearFeedQueue();
        shooter.setMacroShooterEnabled(false);
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
