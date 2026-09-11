package edu.ftcsushi.fw.sensing.observation;

import edu.ftcsushi.fw.core.source.Source;

/**
 * Continuous single-location selection constructed by {@link TargetSelections#fromRecentFieldLocations}.
 *
 * <p>The source borrows a passive memory view and any ranking-pose owner; it never advances or
 * resets them. Each complete successful result is memoized for the shared clock cycle. Exceptions
 * do not commit a result and may retry in that cycle. Recursive sampling and sampling/reset
 * overlap fail fast. {@link #reset()} clears only the selector's local successful-cycle cache.</p>
 *
 * <p>Consumers sharing one intended point reuse this source. Results preserve differently aged
 * field sightings, not present visibility, physical identity, a held destination, or a fresh-image
 * pickup recheck. Retained results must pass {@link FieldTargetSelectionResult#isUsable} when used.</p>
 */
public interface FieldTargetSelectionSource extends Source<FieldTargetSelectionResult> { }
