package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.PlantSnapshot;
import edu.ftcsushi.fw.actuation.SemanticScalarSnapshot;
import edu.ftcsushi.fw.task.Task;

/** Semantic closed/half/open claw capability shared by TeleOp and Auto. */
public interface BasicClaw {

    /** Named claw requests over one normalized safe range; callers never repeat raw positions. */
    enum State {
        CLOSED,
        HALF,
        OPEN
    }

    /**
     * Immutable request/application snapshot.
     *
     * <p>A standard servo has no position feedback here, so {@code appliedCoordinate} is the
     * Plant's final normalized target after bounds: {@code 0.0} is closed, {@code 0.5} is half,
     * and {@code 1.0} is open. It is not the configured native FTC Servo position or proof that
     * the claw physically arrived.</p>
     */
    final class Status {
        private final SemanticScalarSnapshot<State, PlantSnapshot> delegate;

        /** Wraps one coherent semantic request and scalar Plant snapshot. */
        public Status(SemanticScalarSnapshot<State, PlantSnapshot> delegate) {
            this.delegate = Objects.requireNonNull(delegate, "delegate");
        }

        /** Returns the most recent semantic claw request. */
        public State requestedState() {
            return delegate.request().semantic();
        }

        /** Returns the normalized coordinate paired with the semantic request. */
        public double requestedCoordinate() {
            return delegate.request().commandTarget();
        }

        /**
         * Returns the Plant's cached final normalized target after bounds and guards.
         * This is a software command fact, not standard-servo position feedback.
         */
        public double appliedCoordinate() {
            return delegate.plant().appliedTarget();
        }

        /** Returns the underlying immutable Plant capture for advanced diagnostics. */
        public PlantSnapshot plantSnapshot() {
            return delegate.plant();
        }
    }

    /** Immediately replaces the persistent semantic request; a later output heartbeat applies it. */
    void setState(State state);

    /**
     * Builds a fresh single-use Task that selects one persistent semantic state when started.
     *
     * <p>Building the Task validates the state without publishing it. Starting the Task publishes
     * the request once and completes immediately; the mechanism's normal downstream output
     * heartbeat applies it. A standard servo supplies no arrival feedback, so this Task does not
     * wait for or claim physical completion.</p>
     *
     * @param state non-null semantic state
     * @return fresh single-use request Task
     */
    Task setStateTask(State state);

    /** Returns the semantic request and cached normalized applied target without polling hardware. */
    Status status();
}
