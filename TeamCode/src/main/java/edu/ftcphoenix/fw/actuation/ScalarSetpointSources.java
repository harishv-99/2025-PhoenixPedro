package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Source helpers for scalar setpoint requests and statuses.
 */
public final class ScalarSetpointSources {
    private ScalarSetpointSources() {
    }

    /**
     * Returns the first request source with at least one candidate.
     *
     * <p>This is useful for primary/fallback behavior such as direct vision first and localization
     * fallback second. For richer age/quality policies, put those gates inside each request source
     * or let {@link ScalarSetpointPlanner.Config} gate candidate age and quality.</p>
     */
    @SafeVarargs
    public static Source<ScalarSetpointRequest> firstValid(Source<ScalarSetpointRequest>... sources) {
        Objects.requireNonNull(sources, "sources");
        return new Source<ScalarSetpointRequest>() {
            @Override
            public ScalarSetpointRequest get(LoopClock clock) {
                String lastReason = "no scalar request sources";
                for (Source<ScalarSetpointRequest> source : sources) {
                    if (source == null) continue;
                    ScalarSetpointRequest request = source.get(clock);
                    if (request != null && request.hasCandidates()) {
                        return request;
                    }
                    if (request != null) {
                        lastReason = request.reason();
                    }
                }
                return ScalarSetpointRequest.none(lastReason);
            }

            @Override
            public void reset() {
                for (Source<ScalarSetpointRequest> source : sources) {
                    if (source != null) source.reset();
                }
            }
        };
    }
}
