package edu.ftcsushi.fw.drive.guidance;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveOverlay;
import edu.ftcsushi.fw.drive.DriveOverlayOutput;

/**
 * Implementation of {@link DriveOverlay} for {@link DriveGuidancePlan}.
 */
final class DriveGuidanceOverlay implements DriveOverlay {

    private final DriveGuidancePlan plan;
    private final DriveGuidanceCore core;

    DriveGuidanceOverlay(DriveGuidancePlan plan) {
        this.plan = plan;
        this.core = new DriveGuidanceCore(plan);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void onEnable(LoopClock clock) {
        // Reset only this overlay's owned runtime state.
        core.onEnable();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public DriveOverlayOutput get(LoopClock clock) {
        DriveGuidanceCore.Step step = core.step(clock, plan.requestedMask());
        return step.out;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        DriveGuidanceCore.Step step = core.lastStep();

        dbg.addData(prefix + ".solveMode", core.solveMode());
        dbg.addData(prefix + ".mask", step.out.mask.toString());
        dbg.addData(prefix + ".axial", step.out.signal.axial);
        dbg.addData(prefix + ".lateral", step.out.signal.lateral);
        dbg.addData(prefix + ".omega", step.out.signal.omega);

        if (step.hasTranslationError) {
            dbg.addData(prefix + ".forwardErrorIn", step.forwardErrorIn);
            dbg.addData(prefix + ".leftErrorIn", step.leftErrorIn);
        }
        if (step.hasOmegaError) {
            dbg.addData(prefix + ".omegaErrorRad", step.omegaErrorRad);
        }

        Pose2d anchor = core.fieldToTranslationFrameAnchor();
        if (anchor != null) {
            dbg.addData(prefix + ".translationAnchorX", anchor.xInches);
            dbg.addData(prefix + ".translationAnchorY", anchor.yInches);
            dbg.addData(prefix + ".translationAnchorHeadingRad", anchor.headingRad);
        }
    }
}
