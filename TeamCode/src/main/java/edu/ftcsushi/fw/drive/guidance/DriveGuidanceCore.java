package edu.ftcsushi.fw.drive.guidance;

import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.drive.DriveOverlayOutput;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.spatial.ReferenceSelectionResult;

/** One explicit evidence authority shared by overlays, Tasks, and queries. */
final class DriveGuidanceCore {
    private static final ReferenceSelectionResult NO_SELECTION =
            ReferenceSelectionResult.none();
    private final DriveGuidancePlan plan;
    private final DriveGuidanceEvaluator evaluator;
    private Step lastStep;
    private long cachedCycle = Long.MIN_VALUE;
    private DriveOverlayMask cachedRequestedMask;
    private Step cachedStep;
    private boolean operationInProgress;

    DriveGuidanceCore(DriveGuidancePlan plan) {
        this.plan = Objects.requireNonNull(plan, "plan");
        this.evaluator = new DriveGuidanceEvaluator(plan.spec);
        this.lastStep = Step.noCommand(plan.spec.resolveWith.mode);
    }

    /** Clears only this runtime's owned cache and latched target anchor. */
    void onEnable() {
        if (operationInProgress) throw reentrant();
        cachedCycle = Long.MIN_VALUE;
        cachedRequestedMask = null;
        cachedStep = null;
        lastStep = Step.noCommand(solveMode());
        evaluator.onEnable();
    }

    /** Publishes at most one successful evaluation per cycle, with one requested mask. */
    Step step(LoopClock clock, DriveOverlayMask requested) {
        Objects.requireNonNull(clock, "clock");
        if (operationInProgress) throw reentrant();
        DriveOverlayMask requestedMask = requested != null ? requested : DriveOverlayMask.NONE;
        long cycle = clock.cycle();
        if (cachedStep != null && cachedCycle == cycle) {
            if (requestedMask.equals(cachedRequestedMask)) return cachedStep;
            throw new IllegalStateException(
                    "Drive guidance cannot be sampled with different requested masks in the same "
                            + "LoopClock cycle. Use the plan's natural mask, one union mask for all "
                            + "same-cycle consumers, or a separate DriveGuidanceQuery/runtime for "
                            + "each mask. First mask=" + cachedRequestedMask
                            + ", requested mask=" + requestedMask + ".");
        }

        operationInProgress = true;
        try {
            Step next = requestedMask.isNone()
                    ? Step.noCommand(solveMode())
                    : applyLossPolicy(evaluator.solve(clock), requestedMask);
            if (clock.cycle() != cycle) {
                throw new IllegalStateException("Drive guidance callbacks must not advance or reset LoopClock");
            }
            lastStep = next;
            cachedCycle = cycle;
            cachedRequestedMask = requestedMask;
            cachedStep = next;
            return next;
        } finally {
            operationInProgress = false;
        }
    }

    /** Reports the configured authority even before a successful observation. */
    DriveGuidanceSpec.SolveMode solveMode() { return plan.spec.resolveWith.mode; }
    Step lastStep() { return lastStep; }
    Pose2d fieldToTranslationFrameAnchor() { return evaluator.fieldToTranslationFrameAnchor(); }

    /** Solved evidence and output-mask ownership are separate; a zero fallback is never evidence. */
    private Step applyLossPolicy(DriveGuidanceEvaluator.Solution sol, DriveOverlayMask requested) {
        boolean hasT = requested.overridesTranslation() && sol.canTranslate
                && Double.isFinite(sol.forwardErrorIn) && Double.isFinite(sol.leftErrorIn)
                && Double.isFinite(Math.hypot(sol.forwardErrorIn, sol.leftErrorIn));
        boolean hasO = requested.overridesOmega() && sol.canOmega && Double.isFinite(sol.omegaErrorRad);
        DriveSignal translation = hasT
                ? DriveGuidanceControllers.translationCmd(sol.forwardErrorIn, sol.leftErrorIn, plan.tuning)
                : DriveSignal.zero();
        double omega = hasO ? DriveGuidanceControllers.omegaCmd(sol.omegaErrorRad, plan.tuning) : 0.0;
        DriveOverlayMask mask = plan.spec.resolveWith.lossPolicy == DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT
                ? requested : requested.intersect(DriveOverlayMask.NONE.withTranslation(hasT).withOmega(hasO));
        return new Step(new DriveOverlayOutput(
                new DriveSignal(requested.axial ? translation.axial : 0.0,
                        requested.lateral ? translation.lateral : 0.0, omega), mask),
                solveMode(), hasT, sol.forwardErrorIn, sol.leftErrorIn, hasO, sol.omegaErrorRad,
                sol.translationSelection, sol.facingSelection);
    }

    private static IllegalStateException reentrant() {
        return new IllegalStateException("Drive guidance cannot sample or reset reentrantly");
    }

    /** One coherent command and the exact requested-channel geometry supporting it. */
    static final class Step {
        final DriveOverlayOutput out;
        final DriveGuidanceSpec.SolveMode solveMode;
        final boolean hasTranslationError;
        final double forwardErrorIn;
        final double leftErrorIn;
        final boolean hasOmegaError;
        final double omegaErrorRad;
        final ReferenceSelectionResult translationSelection;
        final ReferenceSelectionResult facingSelection;

        Step(DriveOverlayOutput out, DriveGuidanceSpec.SolveMode solveMode,
             boolean hasTranslationError, double forwardErrorIn, double leftErrorIn,
             boolean hasOmegaError, double omegaErrorRad,
             ReferenceSelectionResult translationSelection, ReferenceSelectionResult facingSelection) {
            this.out = out;
            this.solveMode = solveMode;
            this.hasTranslationError = hasTranslationError;
            this.forwardErrorIn = hasTranslationError ? forwardErrorIn : Double.NaN;
            this.leftErrorIn = hasTranslationError ? leftErrorIn : Double.NaN;
            this.hasOmegaError = hasOmegaError;
            this.omegaErrorRad = hasOmegaError ? omegaErrorRad : Double.NaN;
            this.translationSelection = translationSelection != null ? translationSelection : NO_SELECTION;
            this.facingSelection = facingSelection != null ? facingSelection : NO_SELECTION;
        }

        /** No requested channel has supplied geometry; configured authority is still known. */
        static Step noCommand(DriveGuidanceSpec.SolveMode solveMode) {
            return new Step(new DriveOverlayOutput(DriveSignal.zero(), DriveOverlayMask.NONE),
                    solveMode, false, Double.NaN, Double.NaN, false, Double.NaN,
                    NO_SELECTION, NO_SELECTION);
        }
    }
}
