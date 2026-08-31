package edu.ftcsushi.robots.examples.reference.capability.lift;

import edu.ftcsushi.fw.task.Task;

/** Season-neutral referenced lift capability. */
public interface ReferenceLift {
    enum Height {
        STOWED,
        LOW,
        HIGH
    }

    final class Status {
        public final Height requestedHeight;
        public final double requestedPositionIn;
        public final double measuredPositionIn;
        public final boolean referenced;
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

    /** Selects a persistent semantic height. */
    void setHeight(Height height);

    /**
     * Builds a fresh feedback-aware move to one semantic height.
     *
     * <p>The request is made when the returned Task starts. Success means the lift's
     * command-correlated feedback reached that request. Once selected, timeout and active
     * cancellation leave that persistent height request in place.</p>
     *
     * @param height non-null semantic destination
     * @return fresh single-use feedback move
     */
    Task moveTo(Height height);

    /**
     * Builds a fresh non-blocking bottom-switch reference search.
     *
     * <p>At Task start the lift selects the persistent {@link Height#STOWED} request. Success
     * establishes the position reference and holds STOWED. A search timeout remains
     * {@code TIMEOUT} while restoring a coherent semantic and numeric STOWED request. Active
     * cancellation remains {@code CANCELLED}, skips the final repair step, and leaves the latest
     * persistent height request unchanged.</p>
     *
     * @return fresh single-use homing Task
     */
    Task home();

    /** Returns cached command and feedback evidence. */
    Status status();
}
