package edu.ftcphoenix.robots.examples.reference.capability.launcher;

import edu.ftcphoenix.fw.task.Task;

/** Season-neutral velocity-wheel, transfer, and release capability shared by TeleOp and Auto. */
public interface ReferenceLauncher {
    /** Immutable operational evidence; controller readiness is not proof of a successful launch. */
    final class Status {
        public final double targetVelocity;
        public final double measuredVelocity;
        public final boolean ready;
        public final boolean objectPresent;
        public final boolean feedActive;

        public Status(double targetVelocity,
                      double measuredVelocity,
                      boolean ready,
                      boolean objectPresent,
                      boolean feedActive) {
            this.targetVelocity = targetVelocity;
            this.measuredVelocity = measuredVelocity;
            this.ready = ready;
            this.objectPresent = objectPresent;
            this.feedActive = feedActive;
        }
    }

    /** Replaces the held velocity request. Zero means safe active-match idle. */
    void setTargetVelocity(double velocity);

    /** Requests active-match idle without terminally stopping the owned Plants. */
    void idle();

    /** Enqueues one temporary transfer override without replacing the held transfer request. */
    void requestTransferPulse();

    /** Builds a fresh outcome-aware spin-up, feed, and retract Task. */
    Task launchOne();

    /** Returns the latest cached controller and sensor evidence. */
    Status status();
}
