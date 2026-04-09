package edu.ftcphoenix.robots.phoenix;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.task.Task;

/**
 * Default Phoenix capability-family implementation used by {@link PhoenixRobot}.
 *
 * <p>
 * This object deliberately stays package-private. Higher-level code should depend on the
 * {@link PhoenixCapabilities} interface rather than this concrete implementation.
 * </p>
 */
final class PhoenixRobotCapabilities implements PhoenixCapabilities {

    private final LoopClock clock;
    private final Shooter shooter;
    private final ShooterSupervisor scoringSupervisor;
    private final ScoringTargeting targetingService;
    private final Scoring scoring = new ScoringImpl();
    private final Targeting targeting = new TargetingImpl();

    /**
     * Creates the shared Phoenix capability-family façade.
     *
     * @param clock             shared loop clock used for current-cycle targeting samples
     * @param shooter           scoring subsystem that owns selected-velocity state
     * @param scoringSupervisor scoring policy owner
     * @param targetingService  shared targeting/aiming service
     */
    PhoenixRobotCapabilities(LoopClock clock,
                             Shooter shooter,
                             ShooterSupervisor scoringSupervisor,
                             ScoringTargeting targetingService) {
        this.clock = Objects.requireNonNull(clock, "clock");
        this.shooter = Objects.requireNonNull(shooter, "shooter");
        this.scoringSupervisor = Objects.requireNonNull(scoringSupervisor, "scoringSupervisor");
        this.targetingService = Objects.requireNonNull(targetingService, "targetingService");
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Scoring scoring() {
        return scoring;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Targeting targeting() {
        return targeting;
    }

    private final class ScoringImpl implements PhoenixCapabilities.Scoring {

        /**
         * {@inheritDoc}
         */
        @Override
        public void setIntakeEnabled(boolean enabled) {
            scoringSupervisor.setIntakeEnabled(enabled);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void setFlywheelEnabled(boolean enabled) {
            scoringSupervisor.setFlywheelEnabled(enabled);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void setShootingEnabled(boolean enabled) {
            if (enabled) {
                scoringSupervisor.beginShooting();
            } else {
                scoringSupervisor.endShooting();
            }
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void setEjectEnabled(boolean enabled) {
            if (enabled) {
                scoringSupervisor.beginEject();
            } else {
                scoringSupervisor.endEject();
            }
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void requestSingleShot() {
            scoringSupervisor.requestSingleShot();
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void requestShots(int shotCount) {
            scoringSupervisor.requestShots(shotCount);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void cancelTransientActions() {
            scoringSupervisor.cancelTransientActions();
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void setSelectedVelocityNative(double velocityNative) {
            shooter.setSelectedVelocity(velocityNative);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void adjustSelectedVelocityNative(double deltaNative) {
            shooter.setSelectedVelocity(shooter.selectedVelocity() + deltaNative);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void captureSuggestedShotVelocity() {
            shooter.setSelectedVelocity(
                    targetingService.suggestedVelocityNative(clock, shooter.selectedVelocity())
            );
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean hasPendingShots() {
            return scoringSupervisor.hasPendingShots();
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ScoringStatus status() {
            return scoringSupervisor.status();
        }
    }

    private final class TargetingImpl implements PhoenixCapabilities.Targeting {

        /**
         * {@inheritDoc}
         */
        @Override
        public TargetingStatus status(LoopClock clock) {
            return targetingService.status(clock);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Task aimTask(DriveCommandSink driveSink, DriveGuidanceTask.Config cfg) {
            return targetingService.aimTask(driveSink, cfg);
        }
    }
}
