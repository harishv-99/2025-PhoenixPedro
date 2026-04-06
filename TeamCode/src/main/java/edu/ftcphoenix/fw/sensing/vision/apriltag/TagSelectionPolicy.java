package edu.ftcphoenix.fw.sensing.vision.apriltag;

import java.util.List;

/**
 * Pure selection policy for choosing one tag observation from a candidate list.
 *
 * <p>A {@code TagSelectionPolicy} is intentionally stateless: it looks only at the current list of
 * fresh candidate observations and returns the winning choice (or {@code null} when there is no
 * valid candidate). Sticky behavior, loss handling, and enable-window semantics belong in
 * {@link TagSelectionSource}, not here.</p>
 */
public interface TagSelectionPolicy {

    /**
     * Chooses one candidate observation.
     *
     * @param candidates fresh candidate observations from the same frame; never null
     * @return the chosen candidate and explanation, or {@code null} when no choice is possible
     */
    TagSelectionChoice choose(List<AprilTagObservation> candidates);
}
