package edu.ftcsushi.robots.examples.starter.capability.intake;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.PlantSnapshot;
import edu.ftcsushi.fw.actuation.SemanticScalarSnapshot;
import edu.ftcsushi.fw.task.Task;

/** Mode-neutral intake capability shared by the starter TeleOp and Auto. */
public interface StarterIntake {

    /** Semantic intake requests; callers do not need to know or distinguish motor power values. */
    enum Mode {
        STOPPED,
        COLLECT,
        EJECT
    }

    /** Domain-named view of one coherent intake request and cached Plant snapshot. */
    final class Status {
        private final SemanticScalarSnapshot<Mode, PlantSnapshot> delegate;

        /** Wraps one coherent semantic request and scalar Plant snapshot. */
        public Status(SemanticScalarSnapshot<Mode, PlantSnapshot> delegate) {
            this.delegate = Objects.requireNonNull(delegate, "delegate");
        }

        /** Returns the selected semantic intake mode. */
        public Mode mode() {
            return delegate.request().semantic();
        }

        /** Returns the normalized power paired with the selected mode. */
        public double requestedPower() {
            return delegate.request().commandTarget();
        }

        /** Returns the Plant's cached final normalized power after bounds and guards. */
        public double appliedPower() {
            return delegate.plant().appliedTarget();
        }

        /** Returns the underlying immutable Plant snapshot for advanced diagnostics. */
        public PlantSnapshot plantSnapshot() {
            return delegate.plant();
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

    /** Returns a domain-named view of the held request and cached scalar evidence. */
    Status status();
}
