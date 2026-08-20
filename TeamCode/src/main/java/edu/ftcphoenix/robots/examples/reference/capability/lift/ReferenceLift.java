package edu.ftcphoenix.robots.examples.reference.capability.lift;

import edu.ftcphoenix.fw.task.Task;

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

    /** Builds a fresh non-blocking bottom-switch reference search. */
    Task home();

    /** Returns cached command and feedback evidence. */
    Status status();
}
