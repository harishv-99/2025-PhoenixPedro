package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;

/**
 * Runtime source for sampling a {@link DriveGuidancePlan}'s current status.
 *
 * <p>This lets robot code reuse the same evaluation path that DriveGuidance overlays/tasks use,
 * without necessarily applying the resulting drive command. Typical uses are telemetry, readiness
 * gates such as “only shoot when facing is within tolerance,” and tuning probes.</p>
 *
 * <p>The object is stateful. It tracks the same internal state as an overlay: latched translation
 * anchors, adaptive blending state, sticky selected-tag references, and controller state. Create one
 * query per independent consumer and reuse it across loop iterations.</p>
 */
public final class DriveGuidanceQuery implements Source<DriveGuidanceStatus> {

    private final DriveGuidancePlan plan;
    private final DriveGuidanceCore core;

    private boolean enabled = false;
    private DriveGuidanceStatus last = null;
    private long lastCycle = Long.MIN_VALUE;
    private DriveOverlayMask lastMask = null;

    /**
     * Creates a query wrapper for the supplied immutable guidance plan.
     */
    public DriveGuidanceQuery(DriveGuidancePlan plan) {
        this.plan = Objects.requireNonNull(plan, "plan");
        this.core = new DriveGuidanceCore(plan);
    }

    /**
     * Resets internal state as if the guidance overlay was just enabled.
     */
    @Override
    public void reset() {
        core.onEnable();
        enabled = true;
        last = null;
        lastCycle = Long.MIN_VALUE;
        lastMask = null;
    }

    /**
     * Samples using the plan's natural mask ({@link DriveGuidancePlan#requestedMask()}).
     */
    @Override
    public DriveGuidanceStatus get(LoopClock clock) {
        return sample(clock);
    }

    /**
     * Samples using the plan's natural mask ({@link DriveGuidancePlan#requestedMask()}).
     */
    public DriveGuidanceStatus sample(LoopClock clock) {
        return sample(clock, plan.requestedMask());
    }

    /**
     * Samples using an explicit requested mask.
     *
     * <p>This matters when a plan is configured for both translation and facing, but a caller wants
     * to evaluate only one degree of freedom or match the exact DOFs it intends to apply. Sampling is
     * idempotent by loop cycle and requested mask.</p>
     */
    public DriveGuidanceStatus sample(LoopClock clock, DriveOverlayMask requestedMask) {
        if (clock == null) {
            return last;
        }
        DriveOverlayMask requested = (requestedMask != null) ? requestedMask : plan.requestedMask();
        if (last != null && lastCycle == clock.cycle() && requested.equals(lastMask)) {
            return last;
        }
        if (!enabled) {
            core.onEnable();
            enabled = true;
        }

        DriveGuidanceCore.Step step = core.step(clock, requested);
        last = DriveGuidanceStatus.fromCore(core, step);
        lastCycle = clock.cycle();
        lastMask = requested;
        return last;
    }

    /**
     * Returns the most recent sampled status, or {@code null} before the first sample.
     */
    public DriveGuidanceStatus last() {
        return last;
    }
}
