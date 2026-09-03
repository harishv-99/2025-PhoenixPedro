package edu.ftcsushi.robots.examples.basicmechanisms;

import edu.ftcsushi.fw.task.Task;

/** Semantic open/closed claw capability shared by TeleOp and Auto. */
public interface BasicClaw {

    /** Named claw requests; callers never repeat raw servo positions. */
    enum State {
        CLOSED,
        OPEN
    }

    /**
     * Immutable request/application snapshot.
     *
     * <p>A standard servo has no position feedback here, so {@code appliedCoordinate} is the
     * Plant's final normalized target after bounds: {@code 0.0} is closed and {@code 1.0} is open.
     * It is not the configured native FTC Servo endpoint or proof that the claw physically
     * arrived.</p>
     */
    final class Status {
        /**
         * Most recent semantic request, updated synchronously by {@link #setState(State)} or when
         * the Task returned by {@link #setStateTask(State)} starts.
         */
        public final State requestedState;

        /**
         * Last normalized Plant position successfully applied by an output heartbeat, where
         * {@code 0.0} means {@link State#CLOSED} and {@code 1.0} means {@link State#OPEN}, or
         * {@link Double#NaN} before the first successful heartbeat. This is applied-target
         * evidence, not the mapped native Servo endpoint or physical-position feedback.
         */
        public final double appliedCoordinate;

        public Status(State requestedState, double appliedCoordinate) {
            this.requestedState = requestedState;
            this.appliedCoordinate = appliedCoordinate;
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
