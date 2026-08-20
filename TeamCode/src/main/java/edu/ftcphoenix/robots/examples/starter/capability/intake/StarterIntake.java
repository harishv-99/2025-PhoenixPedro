package edu.ftcphoenix.robots.examples.starter.capability.intake;

import edu.ftcphoenix.fw.task.Task;

/** Mode-neutral intake capability shared by the starter TeleOp and Auto. */
public interface StarterIntake {

    /** Semantic intake requests; callers do not need to know motor power values. */
    enum Mode {
        STOPPED,
        COLLECT,
        EJECT
    }

    /**
     * Small immutable status snapshot for telemetry and higher-level decisions.
     * The applied target is the Plant's cached final target after guards, not hardware readback.
     */
    final class Status {
        private final Mode mode;
        private final double appliedTargetPower;

        public Status(Mode mode, double appliedTargetPower) {
            this.mode = mode;
            this.appliedTargetPower = appliedTargetPower;
        }

        public Mode mode() {
            return mode;
        }

        public double appliedTargetPower() {
            return appliedTargetPower;
        }

        @Override
        public String toString() {
            return "Status{mode=" + mode
                    + ", appliedTargetPower=" + appliedTargetPower + '}';
        }
    }

    /** Replaces the held intake request. */
    void setMode(Mode mode);

    /**
     * Creates a fresh single-use Task that collects for the requested duration, then stops.
     * Active cancellation also restores the stopped request.
     */
    Task collectForSeconds(double durationSec);

    /** Returns the semantic request and the Plant's cached applied target. */
    Status status();
}
