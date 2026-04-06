package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;

/**
 * Read-only “sampler” for a {@link DriveGuidancePlan}.
 *
 * <p>This lets you reuse the exact same evaluation logic that DriveGuidance overlays/tasks use,
 * without having to enable a driver-assist.
 * Typical uses:</p>
 * <ul>
 *   <li>telemetry (“how far off are we?”)</li>
 *   <li>safety gating (“only shoot when aimed”)</li>
 *   <li>unit tests / tune probes</li>
 * </ul>
 *
 * <p><b>Important:</b> this object is stateful. It tracks the same internal state as an overlay
 * (robot-relative translation anchors, adaptive blending, AprilTag takeover state, etc.).
 * Create one instance and reuse it across loop iterations.</p>
 */
public final class DriveGuidanceQuery {

    private final DriveGuidancePlan plan;
    private final DriveGuidanceCore core;

    private boolean enabled = false;
    private DriveGuidanceStatus last = null;

    /**
     * Creates a query wrapper for the supplied immutable guidance plan.
     *
     * <p>The query owns its own {@link DriveGuidanceCore} instance so repeated samples use the
     * exact same solver/controller path as overlays and tasks without sharing runtime state.</p>
     */
    public DriveGuidanceQuery(DriveGuidancePlan plan) {
        this.plan = Objects.requireNonNull(plan, "plan");
        this.core = new DriveGuidanceCore(plan);
    }

    /**
     * Reset internal state as if the guidance overlay was just enabled.
     */
    public void reset() {
        core.onEnable();
        enabled = true;
        last = null;
    }

    /**
     * Sample using the plan's natural mask ({@link DriveGuidancePlan#requestedMask()}).
     */
    public DriveGuidanceStatus sample(LoopClock clock) {
        return sample(clock, plan.requestedMask());
    }

    /**
     * Sample using an explicit requested mask.
     *
     * <p>This matters when a plan is configured for both translation and omega, but you want to
     * evaluate only one DOF (or match the exact DOFs you intend to apply).</p>
     */
    public DriveGuidanceStatus sample(LoopClock clock, DriveOverlayMask requestedMask) {
        if (clock == null) {
            // Defensive: no clock, no safe evaluation.
            return last;
        }
        if (!enabled) {
            // Lazily initialize to match overlay/task behavior.
            core.onEnable();
            enabled = true;
        }

        DriveOverlayMask requested = (requestedMask != null) ? requestedMask : plan.requestedMask();
        DriveGuidanceCore.Step step = core.step(clock, requested);
        last = DriveGuidanceStatus.fromCore(core, step);
        return last;
    }

    /**
     * @return the most recent sampled status (may be null if {@link #sample(LoopClock)} has not been called)
     */
    public DriveGuidanceStatus last() {
        return last;
    }
}
