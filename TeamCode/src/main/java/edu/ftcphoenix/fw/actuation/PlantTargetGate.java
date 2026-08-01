package edu.ftcphoenix.fw.actuation;

import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Candidate-aware dynamic gate used by the inline Plant target-guard grammar.
 *
 * <p>The candidate is expressed in the Plant's public units after target resolution and static
 * range/reference handling. Implementations should be quick and non-blocking.</p>
 */
@FunctionalInterface
public interface PlantTargetGate {

    /** Return whether {@code candidateTarget} may be applied during this loop. */
    boolean allowsTarget(double candidateTarget, LoopClock clock);
}
