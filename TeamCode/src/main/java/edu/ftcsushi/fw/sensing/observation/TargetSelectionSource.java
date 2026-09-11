package edu.ftcsushi.fw.sensing.observation;

import edu.ftcsushi.fw.core.source.Source;

/**
 * One continuously selected geometric observation, constructed through {@link TargetSelections}.
 *
 * <p>Reuse this source when guidance, mechanism logic and status mean the same selected point.
 * A selection retains the original observed frame and capture time; it does not identify a
 * physical ball across images, latch a destination, or infer an unseen object from localization.
 * A new frame may select a different candidate.</p>
 *
 * <p>One complete successful result is published per shared loop cycle. Exceptions do not commit
 * a result and can be retried in that cycle. Recursive sampling and sampling/reset overlap fail
 * fast. Reset clears only selection-local state, never borrowed observations or policy inputs.
 * A retained result must still pass {@link TargetSelectionResult#isUsable} when consumed.</p>
 */
public interface TargetSelectionSource extends Source<TargetSelectionResult> { }
