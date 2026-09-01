package edu.ftcsushi.robots.examples.basicmechanisms;

/** Semantic open/closed claw capability shared by TeleOp and Auto. */
public interface BasicClaw {

    /** Named claw requests; callers never repeat raw servo positions. */
    enum State {
        CLOSED,
        OPEN
    }

    /**
     * Immutable command snapshot.
     *
     * <p>A standard servo has no position feedback here, so {@code appliedPosition} is the Plant's
     * final submitted target after bounds, not proof that the claw physically arrived.</p>
     */
    final class Status {
        /** Most recent semantic request, updated synchronously by {@link #setState(State)}. */
        public final State requestedState;

        /**
         * Last position successfully submitted by an output heartbeat, in native FTC Servo units
         * {@code [0.0, 1.0]}, or {@link Double#NaN} before the first successful heartbeat.
         * This is command evidence, not physical-position feedback.
         */
        public final double appliedPosition;

        public Status(State requestedState, double appliedPosition) {
            this.requestedState = requestedState;
            this.appliedPosition = appliedPosition;
        }
    }

    /** Replaces the persistent semantic request. */
    void setState(State state);

    /** Returns the semantic request and cached applied command without polling hardware. */
    Status status();
}
