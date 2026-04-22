package edu.ftcphoenix.robots.phoenix;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.task.Task;

/**
 * Shared mode-neutral capability surface for Phoenix.
 *
 * <p>This is a small aggregate, not an extra implementation layer. TeleOp and Auto receive the same
 * public scoring and targeting vocabulary, while the concrete robot objects still own the behavior.</p>
 */
public final class PhoenixCapabilities {

    private final Scoring scoring;
    private final Targeting targeting;

    /**
     * Creates a Phoenix capability aggregate from cohesive robot-owned capability families.
     */
    public PhoenixCapabilities(Scoring scoring, Targeting targeting) {
        this.scoring = Objects.requireNonNull(scoring, "scoring");
        this.targeting = Objects.requireNonNull(targeting, "targeting");
    }

    /**
     * Returns the scoring/mechanism-intent capability family.
     */
    public Scoring scoring() {
        return scoring;
    }

    /**
     * Returns the targeting/aiming capability family.
     */
    public Targeting targeting() {
        return targeting;
    }

    /**
     * Capability family for scoring-path mechanism intents and status.
     */
    public interface Scoring {

        /**
         * Enables or disables intake mode.
         */
        void setIntakeEnabled(boolean enabled);

        /**
         * Enables or disables the flywheel request.
         */
        void setFlywheelEnabled(boolean enabled);

        /**
         * Enables or disables continuous shooting intent.
         */
        void setShootingEnabled(boolean enabled);

        /**
         * Enables or disables eject/unjam mode.
         */
        void setEjectEnabled(boolean enabled);

        /**
         * Queues one autonomous shot request.
         */
        void requestSingleShot();

        /**
         * Queues a fixed number of autonomous shot requests.
         */
        void requestShots(int shotCount);

        /**
         * Cancels transient scoring actions such as pending queued shots.
         */
        void cancelTransientActions();

        /**
         * Sets the selected flywheel velocity target in motor native units.
         */
        void setSelectedVelocityNative(double velocityNative);

        /**
         * Adjusts the selected flywheel velocity target by a delta in motor native units.
         */
        void adjustSelectedVelocityNative(double deltaNative);

        /**
         * Refreshes the selected flywheel velocity from the current targeting suggestion.
         */
        void captureSuggestedShotVelocity();

        /**
         * Returns whether any requested or active shot remains in flight.
         */
        boolean hasPendingShots();

        /**
         * Returns the latest scoring-path status snapshot.
         */
        ScoringPath.Status status();
    }

    /**
     * Capability family for target selection, aim status, and aim execution.
     */
    public interface Targeting {

        /**
         * Returns the current targeting status snapshot.
         */
        ScoringTargeting.Status status(LoopClock clock);

        /**
         * Creates an autonomous aim task using Phoenix's shared targeting service.
         */
        Task aimTask(DriveCommandSink driveSink, DriveGuidanceTask.Config cfg);
    }
}
