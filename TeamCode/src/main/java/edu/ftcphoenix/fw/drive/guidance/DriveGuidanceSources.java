package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Glue utilities for using {@link DriveGuidancePlan}/{@link DriveGuidanceQuery} inside the
 * {@code core.source} signal graph.
 *
 * <p>Phoenix guidance can be consumed in three different ways:</p>
 * <ul>
 *   <li><b>Overlay</b>: {@link DriveGuidancePlan#overlay()} overrides some DOFs of a {@code DriveSource}.</li>
 *   <li><b>Task</b>: {@link DriveGuidancePlan#task} runs in autonomous as a {@code Task}.</li>
 *   <li><b>Query</b>: {@link DriveGuidancePlan#query()} / {@link DriveGuidanceQuery} computes errors
 *       and predicted commands for telemetry and gating ("only shoot when aimed").</li>
 * </ul>
 *
 * <p>This class makes the third option feel like the rest of Phoenix's modern APIs:
 * you can treat guidance state as a {@link Source} and then derive booleans/scalars using
 * {@link Source#mapToBoolean(java.util.function.Predicate)} and
 * {@link Source#mapToDouble(java.util.function.ToDoubleFunction)}.</p>
 *
 * <h2>Idempotence</h2>
 * <p>The returned sources are <b>idempotent by {@link LoopClock#cycle()}</b>. A guidance plan is
 * stateful, so it is important that you do not accidentally advance it multiple times per loop.
 * These wrappers cache the sampled {@link DriveGuidanceStatus} per loop cycle.</p>
 */
public final class DriveGuidanceSources {

    private DriveGuidanceSources() {
        // static utility
    }

    /**
     * Create a memoized status source from an existing {@link DriveGuidanceQuery}.
     *
     * <p><b>Recommended usage:</b> create the query once and reuse it:</p>
     * <pre>{@code
     * DriveGuidanceQuery aimQuery = aimPlan.query();
     * Source<DriveGuidanceStatus> aimStatus = DriveGuidanceSources.status(aimQuery);
     * BooleanSource aimed = aimStatus.mapToBoolean(s -> s != null && s.omegaWithin(tolRad));
     * }</pre>
     */
    public static Source<DriveGuidanceStatus> status(DriveGuidanceQuery query) {
        Objects.requireNonNull(query, "query");

        return new Source<DriveGuidanceStatus>() {
            private long lastCycle = Long.MIN_VALUE;
            private DriveGuidanceStatus last = null;

            @Override
            public DriveGuidanceStatus get(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;
                last = query.sample(clock);
                return last;
            }

            @Override
            public void reset() {
                query.reset();
                lastCycle = Long.MIN_VALUE;
                last = null;
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "guidance" : prefix;
                dbg.addData(p + ".class", "DriveGuidanceStatusSource")
                        .addData(p + ".hasLast", last != null);
                if (last != null) {
                    dbg.addData(p + ".mode", last.mode)
                            .addData(p + ".mask", last.mask.toString())
                            .addData(p + ".omegaErrorRad", last.omegaErrorRad)
                            .addData(p + ".translationErrIn", last.translationErrorMagInches());
                }
            }
        };
    }

    /**
     * Convenience: create a status source from a plan by constructing a dedicated query.
     */
    public static Source<DriveGuidanceStatus> status(DriveGuidancePlan plan) {
        Objects.requireNonNull(plan, "plan");
        return status(plan.query());
    }

    /**
     * A scalar source of the most recent signed omega error (radians), or NaN when unavailable.
     */
    public static ScalarSource omegaErrorRad(DriveGuidanceQuery query) {
        return status(query).mapToDouble(s -> (s != null && s.hasOmegaError) ? s.omegaErrorRad : Double.NaN);
    }

    /**
     * A boolean source that is true when omega error is available and within {@code tolRad}.
     */
    public static BooleanSource omegaWithin(DriveGuidanceQuery query, double tolRad) {
        return status(query).mapToBoolean(s -> s != null && s.omegaWithin(tolRad));
    }

    /**
     * A scalar source of the most recent translation error magnitude (inches), or NaN when unavailable.
     */
    public static ScalarSource translationErrorMagInches(DriveGuidanceQuery query) {
        return status(query).mapToDouble(s -> (s != null && s.hasTranslationError)
                ? s.translationErrorMagInches()
                : Double.NaN);
    }

    /**
     * A boolean source that is true when translation error is available and within {@code tolInches}.
     */
    public static BooleanSource translationWithin(DriveGuidanceQuery query, double tolInches) {
        return status(query).mapToBoolean(s -> s != null && s.translationWithin(tolInches));
    }
}
