package edu.ftcphoenix.robots.phoenix;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.task.Task;

/**
 * Shared mode-neutral capability surface for Phoenix.
 *
 * <p>
 * Capability families sit between Phoenix's internal services/supervisors/subsystems and the mode
 * clients that consume them. TeleOp controls and autonomous plans should depend on these cohesive,
 * intent-level families instead of reaching directly into internal owners like {@link Shooter},
 * {@link ShooterSupervisor}, or {@link ScoringTargeting}.
 * </p>
 *
 * <p>
 * The exact family names are intentionally season-specific. Phoenix exposes {@link Scoring} and
 * {@link Targeting} because those are the cleanest public splits for Decode. Future robots should
 * keep the same pattern while choosing families that match their own cohesive public vocabulary.
 * </p>
 */
public interface PhoenixCapabilities {

    /**
     * Returns the scoring/mechanism-intent capability family.
     *
     * @return shared scoring capability family
     */
    Scoring scoring();

    /**
     * Returns the targeting/aiming capability family.
     *
     * @return shared targeting capability family
     */
    Targeting targeting();

    /**
     * Capability family for scoring-path mechanism intents and status.
     *
     * <p>
     * This family owns public actions that mutate the scoring path or its requested state. It may
     * delegate internally to more than one owner. For example, Phoenix's
     * {@link Scoring#captureSuggestedShotVelocity()} call reads targeting information and then updates the
     * shooter subsystem's selected velocity.
     * </p>
     */
    interface Scoring {

        /**
         * Enables or disables intake mode.
         *
         * @param enabled {@code true} to intake, {@code false} to return to idle behavior
         */
        void setIntakeEnabled(boolean enabled);

        /**
         * Enables or disables the flywheel request.
         *
         * @param enabled {@code true} to spin up, {@code false} to spin down
         */
        void setFlywheelEnabled(boolean enabled);

        /**
         * Enables or disables continuous shooting intent.
         *
         * @param enabled {@code true} while held shooting should remain active
         */
        void setShootingEnabled(boolean enabled);

        /**
         * Enables or disables eject/unjam mode.
         *
         * @param enabled {@code true} while eject mode should remain active
         */
        void setEjectEnabled(boolean enabled);

        /**
         * Queues one autonomous shot request.
         */
        void requestSingleShot();

        /**
         * Queues a fixed number of autonomous shot requests.
         *
         * @param shotCount number of shots to queue; values {@code <= 0} are ignored
         */
        void requestShots(int shotCount);

        /**
         * Cancels transient scoring actions such as pending queued shots.
         */
        void cancelTransientActions();

        /**
         * Sets the selected flywheel velocity target in motor native units.
         *
         * @param velocityNative desired selected velocity target
         */
        void setSelectedVelocityNative(double velocityNative);

        /**
         * Adjusts the selected flywheel velocity target by a delta in motor native units.
         *
         * @param deltaNative signed adjustment to apply to the current selected velocity
         */
        void adjustSelectedVelocityNative(double deltaNative);

        /**
         * Refreshes the selected flywheel velocity from the current targeting suggestion.
         */
        void captureSuggestedShotVelocity();

        /**
         * Returns whether any requested or active shot remains in flight.
         *
         * @return {@code true} while pending shot work still exists
         */
        boolean hasPendingShots();

        /**
         * Returns the current scoring status snapshot.
         *
         * @return immutable status snapshot for the scoring path
         */
        ScoringStatus status();
    }

    /**
     * Capability family for target selection, aim status, and aim execution.
     */
    interface Targeting {

        /**
         * Returns the current targeting status snapshot.
         *
         * @param clock shared loop clock for the active cycle
         * @return cached targeting status snapshot for the current cycle
         */
        TargetingStatus status(LoopClock clock);

        /**
         * Creates an autonomous aim task using Phoenix's shared targeting service.
         *
         * @param driveSink sink used to apply the aim command
         * @param cfg       task-level tolerances/timeouts; when {@code null}, defaults are used
         * @return task that turns the robot toward the currently selected Phoenix scoring target
         */
        Task aimTask(DriveCommandSink driveSink, DriveGuidanceTask.Config cfg);
    }
}
