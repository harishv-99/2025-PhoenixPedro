package edu.ftcsushi.fw.sensing.vision.apriltag;

import java.util.List;

/**
 * Pure selection policy for choosing one normalized tag candidate from a supplied list.
 *
 * <p>A {@code TagSelectionPolicy} is intentionally stateless: it looks only at the current list of
 * fresh candidates and returns the winning choice (or {@code null} when there is no
 * valid candidate). Sticky behavior, loss handling, and enable-window semantics belong in
 * {@link TagSelectionSource}, not here.</p>
 */
public interface TagSelectionPolicy {

    /**
     * Chooses one exact candidate instance supplied to this invocation. Returning a candidate
     * from another read, even with the same ID, is rejected before any selection state commits.
     *
     * @param candidates immutable finite candidates from one evidence snapshot; never null
     * @return the chosen candidate and explanation, or {@code null} when no choice is possible
     */
    TagSelectionChoice choose(List<TagSelectionCandidate> candidates);
}
