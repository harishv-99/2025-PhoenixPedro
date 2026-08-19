package edu.ftcphoenix.robots.phoenix;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.task.Task;

/**
 * Shared mode-neutral capability surface for Phoenix.
 *
 * <p>This is a small aggregate, not an extra implementation layer. TeleOp and Auto receive the same
 * public scoring and targeting vocabulary, while the concrete robot objects still own the behavior.</p>
 */
public final class PhoenixCapabilities {

    /** Immutable scoring snapshot shared by TeleOp, Auto, assists, and presenters. */
    public static final class ScoringStatus {
        public final boolean intakeEnabled;
        public final boolean ejectRequested;
        public final boolean shootingRequested;
        public final boolean flywheelRequested;
        public final boolean shootActive;
        public final int feedBacklog;
        public final String mode;
        public final boolean flywheelEnabled;
        public final boolean pidfEnabled;
        public final double selectedVelocityNative;
        public final double flywheelTargetNative;
        public final double flywheelMeasuredNative;
        public final double flywheelErrorNative;
        public final double flywheelErrorAbsNative;
        public final double flywheelToleranceNative;
        public final double flywheelToleranceBelowNative;
        public final double flywheelToleranceAboveNative;
        public final double flywheelAccelNativePerSec;
        public final double flywheelAccelAbsNativePerSec;
        public final double readyLeadSec;
        /** Predicted absolute velocity, or {@link Double#NaN} when finite prediction overflows. */
        public final double predictedFlywheelAbsNative;
        /** Predicted signed error, or {@link Double#NaN} when prediction is unavailable. */
        public final double predictedFlywheelErrorNative;
        /** Independent raw result from the flywheel Plant's configured at-target test. */
        public final boolean flywheelAtTarget;
        /** Predictive, asymmetric, debounced readiness; false when prediction is unavailable. */
        public final boolean ready;
        public final int feedQueued;
        public final boolean feedActive;
        public final double feedOutput;

        /** Creates a complete immutable scoring snapshot. */
        public ScoringStatus(boolean intakeEnabled,
                             boolean ejectRequested,
                             boolean shootingRequested,
                             boolean flywheelRequested,
                             boolean shootActive,
                             int feedBacklog,
                             String mode,
                             boolean flywheelEnabled,
                             boolean pidfEnabled,
                             double selectedVelocityNative,
                             double flywheelTargetNative,
                             double flywheelMeasuredNative,
                             double flywheelErrorNative,
                             double flywheelErrorAbsNative,
                             double flywheelToleranceNative,
                             double flywheelToleranceBelowNative,
                             double flywheelToleranceAboveNative,
                             double flywheelAccelNativePerSec,
                             double flywheelAccelAbsNativePerSec,
                             double readyLeadSec,
                             double predictedFlywheelAbsNative,
                             double predictedFlywheelErrorNative,
                             boolean flywheelAtTarget,
                             boolean ready,
                             int feedQueued,
                             boolean feedActive,
                             double feedOutput) {
            this.intakeEnabled = intakeEnabled;
            this.ejectRequested = ejectRequested;
            this.shootingRequested = shootingRequested;
            this.flywheelRequested = flywheelRequested;
            this.shootActive = shootActive;
            this.feedBacklog = feedBacklog;
            this.mode = mode != null ? mode : "IDLE";
            this.flywheelEnabled = flywheelEnabled;
            this.pidfEnabled = pidfEnabled;
            this.selectedVelocityNative = selectedVelocityNative;
            this.flywheelTargetNative = flywheelTargetNative;
            this.flywheelMeasuredNative = flywheelMeasuredNative;
            this.flywheelErrorNative = flywheelErrorNative;
            this.flywheelErrorAbsNative = flywheelErrorAbsNative;
            this.flywheelToleranceNative = flywheelToleranceNative;
            this.flywheelToleranceBelowNative = flywheelToleranceBelowNative;
            this.flywheelToleranceAboveNative = flywheelToleranceAboveNative;
            this.flywheelAccelNativePerSec = flywheelAccelNativePerSec;
            this.flywheelAccelAbsNativePerSec = flywheelAccelAbsNativePerSec;
            this.readyLeadSec = readyLeadSec;
            this.predictedFlywheelAbsNative = predictedFlywheelAbsNative;
            this.predictedFlywheelErrorNative = predictedFlywheelErrorNative;
            this.flywheelAtTarget = flywheelAtTarget;
            this.ready = ready;
            this.feedQueued = feedQueued;
            this.feedActive = feedActive;
            this.feedOutput = feedOutput;
        }
    }

    /** Immutable targeting snapshot shared by mode clients, assists, and presenters. */
    public static final class TargetingStatus {
        public final boolean autoAimEnabled;
        public final boolean aimReady;
        public final boolean aimOkToShoot;
        public final boolean aimOverride;
        public final double aimToleranceDeg;
        public final double aimReadyToleranceDeg;
        public final TagSelectionResult selection;
        public final DriveGuidanceStatus aimStatus;
        public final String targetLabel;
        public final double aimOffsetForwardInches;
        public final double aimOffsetLeftInches;
        /** True only when {@link #suggestedVelocityNative} is a finite, usable suggestion. */
        public final boolean hasSuggestedVelocity;
        /** Finite suggested flywheel velocity, or {@link Double#NaN} when unavailable. */
        public final double suggestedVelocityNative;
        public final Pose3d fieldToSelectedTag;
        public final Pose2d fieldToAimPoint;

        /** Creates a complete immutable targeting snapshot. */
        public TargetingStatus(boolean autoAimEnabled,
                               boolean aimReady,
                               boolean aimOkToShoot,
                               boolean aimOverride,
                               double aimToleranceDeg,
                               double aimReadyToleranceDeg,
                               TagSelectionResult selection,
                               DriveGuidanceStatus aimStatus,
                               String targetLabel,
                               double aimOffsetForwardInches,
                               double aimOffsetLeftInches,
                               boolean hasSuggestedVelocity,
                               double suggestedVelocityNative,
                               Pose3d fieldToSelectedTag,
                               Pose2d fieldToAimPoint) {
            this.autoAimEnabled = autoAimEnabled;
            this.aimReady = aimReady;
            this.aimOkToShoot = aimOkToShoot;
            this.aimOverride = aimOverride;
            this.aimToleranceDeg = aimToleranceDeg;
            this.aimReadyToleranceDeg = aimReadyToleranceDeg;
            this.selection = selection;
            this.aimStatus = aimStatus;
            this.targetLabel = targetLabel != null ? targetLabel : "";
            this.aimOffsetForwardInches = aimOffsetForwardInches;
            this.aimOffsetLeftInches = aimOffsetLeftInches;
            this.hasSuggestedVelocity = hasSuggestedVelocity;
            this.suggestedVelocityNative = suggestedVelocityNative;
            this.fieldToSelectedTag = fieldToSelectedTag;
            this.fieldToAimPoint = fieldToAimPoint;
        }
    }

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
     * Capability family for scoring mechanism intents and status.
     *
     * <p>Phoenix follows the framework's public-method vocabulary here:</p>
     * <ul>
     *   <li>{@code set...(...)} methods update held caller-owned inputs.</li>
     *   <li>{@code request...(...)} methods add pending one-shot work.</li>
     *   <li>Future {@code command...(...)} methods should be reserved for frame-valued manual inputs.</li>
     * </ul>
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
         * Returns the latest scoring status snapshot.
         */
        ScoringStatus status();
    }

    /**
     * Capability family for target selection, aim status, and aim execution.
     */
    public interface Targeting {

        /**
         * Returns the latest targeting status snapshot published by the targeting service.
         *
         * <p>Reading this snapshot never advances targeting state. The robot runtime publishes a
         * new snapshot from the shared loop heartbeat before downstream decisions and outputs run.</p>
         */
        TargetingStatus status();

        /**
         * Creates an autonomous aim task using Phoenix's shared targeting service.
         *
         * <p>The supplied mutable task configuration is defensively copied when the task is
         * requested. Later caller mutation cannot change the returned task.</p>
         *
         * @param driveSink final drive-command sink owned by the autonomous drive path
         * @param cfg task-level tolerances and timeouts, or {@code null} for framework defaults
         * @return fresh single-use aim task
         */
        Task aimTask(DriveCommandSink driveSink, DriveGuidanceTask.Config cfg);
    }
}
