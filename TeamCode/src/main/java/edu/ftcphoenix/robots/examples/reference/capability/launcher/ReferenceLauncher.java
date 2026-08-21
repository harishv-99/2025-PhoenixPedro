package edu.ftcphoenix.robots.examples.reference.capability.launcher;

import edu.ftcphoenix.fw.task.Task;

/** Season-neutral velocity-wheel, transfer, and release capability shared by TeleOp and Auto. */
public interface ReferenceLauncher {
    /**
     * Immutable per-wheel operational evidence.
     *
     * <p>{@link #leftAtTarget} and {@link #rightAtTarget} are pure finite, inclusive-tolerance
     * facts, including at a zero target. {@link #ready} adds the positive-target gate and is true
     * only when both per-wheel facts are true. Controller readiness is not proof that an object
     * launched or scored.</p>
     */
    final class Status {
        public final double targetVelocityTicksPerSec;
        public final double leftMeasuredVelocityTicksPerSec;
        public final double rightMeasuredVelocityTicksPerSec;
        public final boolean leftAtTarget;
        public final boolean rightAtTarget;
        public final boolean ready;
        public final boolean objectPresent;
        public final boolean transferPulseActive;

        /**
         * Records one caller-observed snapshot and derives aggregate positive-target readiness.
         *
         * @param targetVelocityTicksPerSec requested native encoder ticks per second
         * @param leftMeasuredVelocityTicksPerSec measured left native encoder ticks per second
         * @param rightMeasuredVelocityTicksPerSec measured right native encoder ticks per second
         * @param leftAtTarget whether finite left evidence meets the owner's copied tolerance
         * @param rightAtTarget whether finite right evidence meets the owner's copied tolerance
         * @param objectPresent conditioned semantic object-sensor state
         * @param transferPulseActive whether the temporary transfer override is active
         * @throws IllegalArgumentException if an at-target assertion lacks a finite target or the
         *                                  corresponding finite wheel measurement
         */
        public Status(double targetVelocityTicksPerSec,
                      double leftMeasuredVelocityTicksPerSec,
                      double rightMeasuredVelocityTicksPerSec,
                      boolean leftAtTarget,
                      boolean rightAtTarget,
                      boolean objectPresent,
                      boolean transferPulseActive) {
            boolean finiteTarget = Double.isFinite(targetVelocityTicksPerSec);
            boolean leftEvidenceCanBeAtTarget = finiteTarget
                    && Double.isFinite(leftMeasuredVelocityTicksPerSec);
            boolean rightEvidenceCanBeAtTarget = finiteTarget
                    && Double.isFinite(rightMeasuredVelocityTicksPerSec);
            if (leftAtTarget && !leftEvidenceCanBeAtTarget) {
                throw new IllegalArgumentException(
                        "leftAtTarget requires finite target and left measurement evidence");
            }
            if (rightAtTarget && !rightEvidenceCanBeAtTarget) {
                throw new IllegalArgumentException(
                        "rightAtTarget requires finite target and right measurement evidence");
            }
            this.targetVelocityTicksPerSec = targetVelocityTicksPerSec;
            this.leftMeasuredVelocityTicksPerSec = leftMeasuredVelocityTicksPerSec;
            this.rightMeasuredVelocityTicksPerSec = rightMeasuredVelocityTicksPerSec;
            this.leftAtTarget = leftAtTarget;
            this.rightAtTarget = rightAtTarget;
            this.ready = targetVelocityTicksPerSec > 0.0
                    && leftAtTarget
                    && rightAtTarget;
            this.objectPresent = objectPresent;
            this.transferPulseActive = transferPulseActive;
        }
    }

    /**
     * Replaces the held flywheel velocity request in native encoder ticks per second.
     *
     * <p>The value must be finite and inside the mechanism's configured inclusive range. This
     * request does not invalidate an active or queued launch. Zero requests flywheel idle only;
     * use {@link #abortLaunches()} to invalidate launch Tasks and clear their temporary release and
     * transfer requests as well.</p>
     *
     * @param velocityTicksPerSec requested flywheel velocity in native encoder ticks per second
     * @throws IllegalArgumentException if the request is non-finite or outside the configured range
     */
    void setTargetVelocityTicksPerSec(double velocityTicksPerSec);

    /**
     * Invalidates every launch Task created before this call and requests active-match idle.
     *
     * <p>The request clears temporary transfer work, selects the configured retracted release
     * position, and selects zero flywheel velocity. Those source requests are realized by a later
     * normal output update; this method does not terminally stop the mechanism's Plants. A later
     * call to {@link #launchOne()} creates a launch in the new valid generation.</p>
     */
    void abortLaunches();

    /**
     * Builds a fresh, single-use spin-up, release, transfer, and cleanup Task.
     *
     * <p>The Task reports success, spin-up timeout, or cancellation without leaving a wheel,
     * release, or temporary transfer request active.</p>
     */
    Task launchOne();

    /** Returns the latest cached per-wheel controller and object-sensor evidence. */
    Status status();
}
