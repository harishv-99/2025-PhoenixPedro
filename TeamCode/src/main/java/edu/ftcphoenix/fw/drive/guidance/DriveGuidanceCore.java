package edu.ftcphoenix.fw.drive.guidance;

import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveOverlayOutput;
import edu.ftcphoenix.fw.drive.DriveSignal;

/**
 * Shared runtime engine used by {@link DriveGuidanceOverlay}, {@link DriveGuidanceTask}, and
 * {@link DriveGuidanceQuery}.
 *
 * <p>{@link DriveGuidanceEvaluator} solves the raw spatial errors, then this class applies
 * controller tuning, adaptive source selection, hysteresis, and blend timing to produce the final
 * {@link DriveSignal} / {@link DriveOverlayMask} pair for the current loop.</p>
 */
final class DriveGuidanceCore {

    private final DriveGuidancePlan plan;
    private final DriveGuidanceEvaluator evaluator;

    // Adaptive-takeover state. Translation uses range hysteresis; omega can either follow the
    // same takeover decision or always prefer AprilTags when configured to do so.
    private boolean aprilTagsInRangeForTranslation;
    private double blendTTranslate;
    private double blendTOmega;

    private String lastMode;
    private Step lastStep;

    DriveGuidanceCore(DriveGuidancePlan plan) {
        this.plan = plan;
        this.evaluator = new DriveGuidanceEvaluator(plan.spec);
        this.lastMode = "init";
        this.lastStep = Step.noCommand("init");
    }

    /**
     * Resets adaptive state when the owning overlay/task/query becomes active.
     */
    void onEnable() {
        aprilTagsInRangeForTranslation = false;
        blendTTranslate = 0.0;
        blendTOmega = 0.0;
        evaluator.onEnable();
        lastMode = "enabled";
        lastStep = Step.noCommand("enabled");
    }

    /**
     * Evaluates one loop of guidance for the requested channels.
     *
     * <p>This method asks the evaluator for both candidate solve lanes (field pose and/or live
     * AprilTags), applies adaptive source selection when both are present, then converts the chosen
     * errors into final drive commands via the already-configured controller tuning.</p>
     */
    Step step(LoopClock clock, DriveOverlayMask requested) {
        DriveGuidanceSpec.Feedback fb = plan.spec.feedback;

        if (requested == null || requested.isNone()) {
            lastMode = "none";
            lastStep = Step.noCommand("none");
            return lastStep;
        }

        CandidateSolution field = fb.hasFieldPose()
                ? toCandidate(evaluator.solveWithFieldPose())
                : CandidateSolution.invalid();

        CandidateSolution aprilTags = fb.hasAprilTags()
                ? toCandidate(evaluator.solveWithAprilTags(clock))
                : CandidateSolution.invalid();

        // Simple single-lane mode: use whichever solve path exists.
        if (!fb.isAdaptive()) {
            CandidateSolution chosen = fb.hasAprilTags() ? aprilTags : field;
            lastMode = fb.hasAprilTags() ? "aprilTags" : "fieldPose";

            Step out = applyLossPolicy(chosen, requested, fb.lossPolicy, lastMode);
            lastStep = out;
            return out;
        }

        // Adaptive mode: both field pose and AprilTags are available. Translation takeover uses
        // range hysteresis. Omega can either follow the same choice or always prefer AprilTags.
        DriveGuidanceSpec.Gates gates = (fb.gates != null) ? fb.gates : DriveGuidanceSpec.Gates.defaults();

        boolean wantTranslation = requested.overridesTranslation();
        boolean wantOmega = requested.overridesOmega();

        boolean hasAprilTagsT = aprilTags.valid && aprilTags.canTranslate && aprilTags.hasRangeInches;
        boolean hasFieldT = field.valid && field.canTranslate;

        if (!hasAprilTagsT) {
            aprilTagsInRangeForTranslation = false;
        } else if (!aprilTagsInRangeForTranslation) {
            if (aprilTags.rangeInches <= gates.enterRangeInches) {
                aprilTagsInRangeForTranslation = true;
            }
        } else if (aprilTags.rangeInches >= gates.exitRangeInches) {
            aprilTagsInRangeForTranslation = false;
        }

        boolean chooseAprilTagsForTranslation;
        if (!wantTranslation) {
            chooseAprilTagsForTranslation = false;
        } else if (!hasFieldT && hasAprilTagsT) {
            chooseAprilTagsForTranslation = true;
        } else if (hasFieldT && hasAprilTagsT) {
            chooseAprilTagsForTranslation = aprilTagsInRangeForTranslation;
        } else {
            chooseAprilTagsForTranslation = false;
        }

        boolean hasAprilTagsO = aprilTags.valid && aprilTags.canOmega;
        boolean hasFieldO = field.valid && field.canOmega;

        boolean chooseAprilTagsForOmega;
        if (!wantOmega) {
            chooseAprilTagsForOmega = false;
        } else if (!hasFieldO && hasAprilTagsO) {
            chooseAprilTagsForOmega = true;
        } else if (fb.preferAprilTagsForOmega) {
            chooseAprilTagsForOmega = hasAprilTagsO;
        } else if (hasFieldO && hasAprilTagsO) {
            chooseAprilTagsForOmega = chooseAprilTagsForTranslation;
        } else {
            chooseAprilTagsForOmega = false;
        }

        double dt = clock.dtSec();
        double blendSec = Math.max(0.0, gates.takeoverBlendSec);
        double step = (blendSec <= 0.0) ? 1.0 : MathUtil.clamp(dt / blendSec, 0.0, 1.0);

        blendTTranslate = updateBlend(blendTTranslate, chooseAprilTagsForTranslation, step);
        blendTOmega = updateBlend(blendTOmega, chooseAprilTagsForOmega, step);

        double axial = 0.0;
        double lateral = 0.0;
        double omega = 0.0;

        DriveOverlayMask mask = DriveOverlayMask.NONE;

        boolean hasTransErr = false;
        double forwardErr = 0.0;
        double leftErr = 0.0;

        boolean hasOmegaErr = false;
        double omegaErr = 0.0;

        if (wantTranslation) {
            if (hasFieldT && hasAprilTagsT) {
                axial = MathUtil.lerp(field.signal.axial, aprilTags.signal.axial, blendTTranslate);
                lateral = MathUtil.lerp(field.signal.lateral, aprilTags.signal.lateral, blendTTranslate);
                mask = mask.withTranslation(true);

                forwardErr = MathUtil.lerp(field.forwardErrorIn, aprilTags.forwardErrorIn, blendTTranslate);
                leftErr = MathUtil.lerp(field.leftErrorIn, aprilTags.leftErrorIn, blendTTranslate);
                hasTransErr = true;
            } else if (hasAprilTagsT) {
                axial = aprilTags.signal.axial;
                lateral = aprilTags.signal.lateral;
                mask = mask.withTranslation(true);

                forwardErr = aprilTags.forwardErrorIn;
                leftErr = aprilTags.leftErrorIn;
                hasTransErr = true;
            } else if (hasFieldT) {
                axial = field.signal.axial;
                lateral = field.signal.lateral;
                mask = mask.withTranslation(true);

                forwardErr = field.forwardErrorIn;
                leftErr = field.leftErrorIn;
                hasTransErr = true;
            }
        }

        if (wantOmega) {
            if (hasFieldO && hasAprilTagsO) {
                omega = MathUtil.lerp(field.signal.omega, aprilTags.signal.omega, blendTOmega);
                mask = mask.withOmega(true);

                omegaErr = lerpAngleError(field.omegaErrorRad, aprilTags.omegaErrorRad, blendTOmega);
                hasOmegaErr = true;
            } else if (hasAprilTagsO) {
                omega = aprilTags.signal.omega;
                mask = mask.withOmega(true);

                omegaErr = aprilTags.omegaErrorRad;
                hasOmegaErr = true;
            } else if (hasFieldO) {
                omega = field.signal.omega;
                mask = mask.withOmega(true);

                omegaErr = field.omegaErrorRad;
                hasOmegaErr = true;
            }
        }

        Step out;
        if (mask.isNone()) {
            out = applyLossPolicy(CandidateSolution.invalid(), requested, fb.lossPolicy, "adaptive(loss)");
        } else {
            out = new Step(
                    new DriveOverlayOutput(new DriveSignal(axial, lateral, omega), mask),
                    "adaptive",
                    field,
                    aprilTags,
                    aprilTagsInRangeForTranslation,
                    blendTTranslate,
                    blendTOmega,
                    hasTransErr,
                    forwardErr,
                    leftErr,
                    hasOmegaErr,
                    omegaErr
            );
        }

        lastMode = out.mode;
        lastStep = out;
        return out;
    }

    /**
     * Returns the last high-level mode string for status/debug output.
     */
    String lastMode() {
        return lastMode;
    }

    /**
     * Returns the most recent step result.
     */
    Step lastStep() {
        return lastStep;
    }

    boolean aprilTagsInRangeForTranslation() {
        return aprilTagsInRangeForTranslation;
    }

    double blendTTranslate() {
        return blendTTranslate;
    }

    double blendTOmega() {
        return blendTOmega;
    }

    edu.ftcphoenix.fw.core.geometry.Pose2d fieldToTranslationFrameAnchor() {
        return evaluator.fieldToTranslationFrameAnchor();
    }

    private CandidateSolution toCandidate(DriveGuidanceEvaluator.Solution sol) {
        if (sol == null || !sol.valid) {
            return CandidateSolution.invalid();
        }

        double axial = 0.0;
        double lateral = 0.0;
        double omega = 0.0;

        if (sol.canTranslate) {
            DriveSignal t = DriveGuidanceControllers.translationCmd(sol.forwardErrorIn, sol.leftErrorIn, plan.tuning);
            axial = t.axial;
            lateral = t.lateral;
        }
        if (sol.canOmega) {
            omega = DriveGuidanceControllers.omegaCmd(sol.omegaErrorRad, plan.tuning);
        }

        return new CandidateSolution(
                true,
                new DriveSignal(axial, lateral, omega),
                sol.canTranslate,
                sol.canOmega,
                sol.hasRangeInches,
                sol.rangeInches,
                sol.forwardErrorIn,
                sol.leftErrorIn,
                sol.omegaErrorRad
        );
    }

    private static double updateBlend(double current, boolean chooseAprilTags, double step) {
        if (chooseAprilTags) {
            return Math.min(1.0, current + step);
        }
        return Math.max(0.0, current - step);
    }

    private static Step applyLossPolicy(CandidateSolution sol,
                                        DriveOverlayMask requested,
                                        DriveGuidanceSpec.LossPolicy policy,
                                        String mode) {
        DriveOverlayMask mask = DriveOverlayMask.NONE;

        if (sol.valid) {
            if (requested.overridesTranslation() && sol.canTranslate) {
                mask = mask.withTranslation(true);
            }
            if (requested.overridesOmega() && sol.canOmega) {
                mask = mask.withOmega(true);
            }
            if (!mask.isNone()) {
                return new Step(
                        new DriveOverlayOutput(sol.signal, mask),
                        mode,
                        CandidateSolution.invalid(),
                        CandidateSolution.invalid(),
                        false,
                        0.0,
                        0.0,
                        requested.overridesTranslation() && sol.canTranslate,
                        sol.forwardErrorIn,
                        sol.leftErrorIn,
                        requested.overridesOmega() && sol.canOmega,
                        sol.omegaErrorRad
                );
            }
        }

        if (policy == DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT) {
            return new Step(
                    new DriveOverlayOutput(DriveSignal.zero(), requested),
                    mode + ":zeroOutput",
                    CandidateSolution.invalid(),
                    CandidateSolution.invalid(),
                    false,
                    0.0,
                    0.0,
                    false,
                    0.0,
                    0.0,
                    false,
                    0.0
            );
        }

        return Step.noCommand(mode + ":passThrough");
    }

    private static double lerpAngleError(double aRad, double bRad, double t) {
        double delta = MathUtil.wrapToPi(bRad - aRad);
        return MathUtil.wrapToPi(aRad + t * delta);
    }

    static final class CandidateSolution {
        final boolean valid;
        final DriveSignal signal;
        final boolean canTranslate;
        final boolean canOmega;
        final boolean hasRangeInches;
        final double rangeInches;
        final double forwardErrorIn;
        final double leftErrorIn;
        final double omegaErrorRad;

        CandidateSolution(boolean valid,
                          DriveSignal signal,
                          boolean canTranslate,
                          boolean canOmega,
                          boolean hasRangeInches,
                          double rangeInches,
                          double forwardErrorIn,
                          double leftErrorIn,
                          double omegaErrorRad) {
            this.valid = valid;
            this.signal = signal;
            this.canTranslate = canTranslate;
            this.canOmega = canOmega;
            this.hasRangeInches = hasRangeInches;
            this.rangeInches = rangeInches;
            this.forwardErrorIn = forwardErrorIn;
            this.leftErrorIn = leftErrorIn;
            this.omegaErrorRad = omegaErrorRad;
        }

        static CandidateSolution invalid() {
            return new CandidateSolution(false,
                    DriveSignal.zero(),
                    false,
                    false,
                    false,
                    Double.NaN,
                    0.0,
                    0.0,
                    0.0);
        }
    }

    static final class Step {
        final DriveOverlayOutput out;
        final String mode;

        final CandidateSolution field;
        final CandidateSolution aprilTags;

        final boolean aprilTagsInRangeForTranslation;
        final double blendTTranslate;
        final double blendTOmega;

        final boolean hasTranslationError;
        final double forwardErrorIn;
        final double leftErrorIn;

        final boolean hasOmegaError;
        final double omegaErrorRad;

        Step(DriveOverlayOutput out,
             String mode,
             CandidateSolution field,
             CandidateSolution aprilTags,
             boolean aprilTagsInRangeForTranslation,
             double blendTTranslate,
             double blendTOmega,
             boolean hasTranslationError,
             double forwardErrorIn,
             double leftErrorIn,
             boolean hasOmegaError,
             double omegaErrorRad) {
            this.out = out;
            this.mode = mode;
            this.field = field;
            this.aprilTags = aprilTags;
            this.aprilTagsInRangeForTranslation = aprilTagsInRangeForTranslation;
            this.blendTTranslate = blendTTranslate;
            this.blendTOmega = blendTOmega;
            this.hasTranslationError = hasTranslationError;
            this.forwardErrorIn = forwardErrorIn;
            this.leftErrorIn = leftErrorIn;
            this.hasOmegaError = hasOmegaError;
            this.omegaErrorRad = omegaErrorRad;
        }

        static Step noCommand(String mode) {
            return new Step(
                    DriveOverlayOutput.zero(),
                    mode,
                    CandidateSolution.invalid(),
                    CandidateSolution.invalid(),
                    false,
                    0.0,
                    0.0,
                    false,
                    0.0,
                    0.0,
                    false,
                    0.0
            );
        }
    }
}
