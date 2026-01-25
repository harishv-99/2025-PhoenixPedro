package edu.ftcphoenix.fw.drive.guidance;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.drive.DriveOverlayOutput;

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

    @Override
    public void onEnable(LoopClock clock) {
        // Reset adaptive state so first output after enable is predictable.
        core.onEnable();
    }

    @Override
    public DriveOverlayOutput get(LoopClock clock) {
        DriveGuidanceCore.Step step = core.step(clock, plan.requestedMask());
        return step.out;
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        DriveGuidanceCore.Step step = core.lastStep();

        dbg.addData(prefix + ".mode", core.lastMode());
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

        dbg.addData(prefix + ".obsInRangeForTranslation", step.obsInRangeForTranslation);
        dbg.addData(prefix + ".blendTTranslate", step.blendTTranslate);
        dbg.addData(prefix + ".blendTOmega", step.blendTOmega);

        dbg.addData(prefix + ".lastObservedTagId", core.lastObservedTagId());
        Pose2d anchor = core.fieldToTranslationFrameAnchor();
        if (anchor != null) {
            dbg.addData(prefix + ".translationAnchorX", anchor.xInches);
            dbg.addData(prefix + ".translationAnchorY", anchor.yInches);
            dbg.addData(prefix + ".translationAnchorHeadingRad", anchor.headingRad);
        }
    }
}
