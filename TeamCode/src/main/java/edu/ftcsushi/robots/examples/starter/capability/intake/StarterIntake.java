package edu.ftcsushi.robots.examples.starter.capability.intake;

import edu.ftcsushi.fw.task.Task;

/** Mode-neutral intake capability shared by the starter TeleOp and Auto. */
public interface StarterIntake {

    /** Semantic intake requests; callers do not need to know or distinguish motor power values. */
    enum Mode {
        STOPPED,
        COLLECT,
        EJECT
    }

    /**
     * Small immutable status snapshot for telemetry and higher-level decisions.
     * The mode is the held semantic request; it is not reconstructed from motor power. The applied
     * target is the Plant's cached final target after guards, not hardware readback.
     */
    final class Status {
        private final Mode mode;
        private final double appliedTargetPower;

        public Status(Mode mode, double appliedTargetPower) {
            this.mode = mode;
            this.appliedTargetPower = appliedTargetPower;
        }

        /** Returns the held semantic request, independent of its configured numeric realization. */
        public Mode mode() {
            return mode;
        }

        /** Returns the Plant's cached applied target, not measured motor motion. */
        public double appliedTargetPower() {
            return appliedTargetPower;
        }

        @Override
        public String toString() {
            return "Status{mode=" + mode
                    + ", appliedTargetPower=" + appliedTargetPower + '}';
        }
    }

    /** Replaces the held semantic request; the mechanism maps it to configured motor power. */
    void setMode(Mode mode);

    /**
     * Creates a fresh single-use Task that requests collect for the requested duration, then
     * requests stopped. Active cancellation also restores the stopped semantic request.
     *
     * @param durationSec finite duration greater than zero, in seconds
     * @throws IllegalArgumentException if {@code durationSec} is non-finite or not greater than zero
     */
    Task collectForSeconds(double durationSec);

    /** Returns the held semantic request and the Plant's cached applied target. */
    Status status();
}
