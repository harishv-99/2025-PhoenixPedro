package edu.ftcsushi.robots.examples.basicmechanisms;

import edu.ftcsushi.fw.task.Task;

/**
 * Small semantic lift API shared by the teaching TeleOps and Autos.
 *
 * <p>Callers choose a named height and inspect evidence; only the mechanism knows encoder units,
 * motor power, or how the bottom switch establishes a reference.</p>
 */
public interface BasicLift {

    /** Named robot positions used everywhere instead of repeating encoder values. */
    enum Height {
        STOWED,
        LOW,
        HIGH
    }

    /** Immutable snapshot of the current request and cached feedback evidence. */
    final class Status {
        /** Most recent semantic request, updated synchronously by command methods or Task start. */
        public final Height requestedHeight;

        /** Numeric position selected by {@link #requestedHeight}, in mechanism inches. */
        public final double requestedPositionIn;

        /**
         * Last encoder measurement published by a successful output heartbeat, in mechanism
         * inches, or {@link Double#NaN} while measurement evidence is unavailable.
         */
        public final double measuredPositionIn;

        /** Whether the last successful output heartbeat had an established position reference. */
        public final boolean referenced;

        /**
         * Whether the last successful output heartbeat proved arrival for the current request.
         * A new request and terminal stop both invalidate this evidence immediately.
         */
        public final boolean atTarget;

        public Status(Height requestedHeight,
                      double requestedPositionIn,
                      double measuredPositionIn,
                      boolean referenced,
                      boolean atTarget) {
            this.requestedHeight = requestedHeight;
            this.requestedPositionIn = requestedPositionIn;
            this.measuredPositionIn = measuredPositionIn;
            this.referenced = referenced;
            this.atTarget = atTarget;
        }
    }

    /** Selects a persistent semantic height without waiting for physical arrival. */
    void setHeight(Height height);

    /**
     * Builds a fresh feedback-aware move to one semantic height.
     *
     * <p>Success requires the selected semantic request still to be current and its cached feedback
     * to report arrival. Timeout and active cancellation do not overwrite the latest persistent
     * request, which may have been superseded while this Task was active.</p>
     *
     * @param height non-null destination
     * @return fresh single-use move Task
     */
    Task moveTo(Height height);

    /**
     * Builds a fresh non-blocking search for the bottom reference switch.
     *
     * <p>A successful search establishes zero, then selects {@link Height#STOWED} before the
     * mechanism's downstream output phase. Timeout and active cancellation retain their truthful
     * outcomes and preserve the latest coherent semantic and numeric height request.</p>
     *
     * @return fresh single-use homing Task
     */
    Task home();

    /** Returns cached request and feedback evidence without polling hardware. */
    Status status();
}
