package edu.ftcphoenix.fw.drive.guidance;

import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveOverlayOutput;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;

/**
 * Shared runtime engine used by {@link DriveGuidanceOverlay}, {@link DriveGuidanceTask}, and
 * {@link DriveGuidanceQuery}.
 */
final class DriveGuidanceCore {

    private static final TagSelectionResult NO_SELECTION =
            TagSelectionResult.none(java.util.Collections.<Integer>emptySet());

    private final DriveGuidancePlan plan;
    private final DriveGuidanceEvaluator evaluator;

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

    void onEnable() {
        aprilTagsInRangeForTranslation = false;
        blendTTranslate = 0.0;
        blendTOmega = 0.0;
        evaluator.onEnable();
        lastMode = "enabled";
        lastStep = Step.noCommand("enabled");
    }

    Step step(LoopClock clock, DriveOverlayMask requested) {
        DriveGuidanceSpec.ResolveWith rw = plan.spec.resolveWith;

        if (requested == null || requested.isNone()) {
            lastMode = "none";
            lastStep = Step.noCommand("none");
            return lastStep;
        }

        CandidateSolution localization = rw.hasLocalization()
                ? toCandidate(evaluator.solveWithLocalization(clock))
                : CandidateSolution.invalid();

        CandidateSolution aprilTags = rw.hasAprilTags()
                ? toCandidate(evaluator.solveWithAprilTags(clock))
                : CandidateSolution.invalid();

        if (!rw.isAdaptive()) {
            CandidateSolution chosen = (rw.mode == DriveGuidanceSpec.SolveMode.APRIL_TAGS_ONLY)
                    ? aprilTags
                    : localization;
            lastMode = (rw.mode == DriveGuidanceSpec.SolveMode.APRIL_TAGS_ONLY) ? "aprilTags" : "localization";

            Step out = applyLossPolicy(chosen, requested, rw.lossPolicy, lastMode);
            lastStep = out;
            return out;
        }

        DriveGuidanceSpec.TranslationTakeover takeover = (rw.translationTakeover != null)
                ? rw.translationTakeover
                : DriveGuidanceSpec.TranslationTakeover.defaults();

        boolean wantTranslation = requested.overridesTranslation();
        boolean wantOmega = requested.overridesOmega();

        boolean hasAprilTagsT = aprilTags.valid && aprilTags.canTranslate && aprilTags.hasRangeInches;
        boolean hasLocalizationT = localization.valid && localization.canTranslate;

        if (!hasAprilTagsT) {
            aprilTagsInRangeForTranslation = false;
        } else if (!aprilTagsInRangeForTranslation) {
            if (aprilTags.rangeInches <= takeover.enterRangeInches) {
                aprilTagsInRangeForTranslation = true;
            }
        } else if (aprilTags.rangeInches >= takeover.exitRangeInches) {
            aprilTagsInRangeForTranslation = false;
        }

        boolean chooseAprilTagsForTranslation;
        if (!wantTranslation) {
            chooseAprilTagsForTranslation = false;
        } else if (!hasLocalizationT && hasAprilTagsT) {
            chooseAprilTagsForTranslation = true;
        } else if (hasLocalizationT && hasAprilTagsT) {
            chooseAprilTagsForTranslation = aprilTagsInRangeForTranslation;
        } else {
            chooseAprilTagsForTranslation = false;
        }

        boolean hasAprilTagsO = aprilTags.valid && aprilTags.canOmega;
        boolean hasLocalizationO = localization.valid && localization.canOmega;

        boolean chooseAprilTagsForOmega;
        if (!wantOmega) {
            chooseAprilTagsForOmega = false;
        } else if (!hasLocalizationO && hasAprilTagsO) {
            chooseAprilTagsForOmega = true;
        } else if (rw.omegaPolicy == DriveGuidanceSpec.OmegaPolicy.PREFER_APRIL_TAGS_WHEN_VALID) {
            chooseAprilTagsForOmega = hasAprilTagsO;
        } else if (hasLocalizationO && hasAprilTagsO) {
            chooseAprilTagsForOmega = chooseAprilTagsForTranslation;
        } else {
            chooseAprilTagsForOmega = false;
        }

        double dt = clock.dtSec();
        double blendSec = Math.max(0.0, takeover.blendSec);
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

        TagSelectionResult translationSelection = NO_SELECTION;
        TagSelectionResult facingSelection = NO_SELECTION;

        if (wantTranslation) {
            if (hasLocalizationT && hasAprilTagsT) {
                axial = MathUtil.lerp(localization.signal.axial, aprilTags.signal.axial, blendTTranslate);
                lateral = MathUtil.lerp(localization.signal.lateral, aprilTags.signal.lateral, blendTTranslate);
                mask = mask.withTranslation(true);

                forwardErr = MathUtil.lerp(localization.forwardErrorIn, aprilTags.forwardErrorIn, blendTTranslate);
                leftErr = MathUtil.lerp(localization.leftErrorIn, aprilTags.leftErrorIn, blendTTranslate);
                hasTransErr = true;
                translationSelection = (blendTTranslate >= 0.5) ? aprilTags.translationSelection : localization.translationSelection;
            } else if (hasAprilTagsT) {
                axial = aprilTags.signal.axial;
                lateral = aprilTags.signal.lateral;
                mask = mask.withTranslation(true);
                forwardErr = aprilTags.forwardErrorIn;
                leftErr = aprilTags.leftErrorIn;
                hasTransErr = true;
                translationSelection = aprilTags.translationSelection;
            } else if (hasLocalizationT) {
                axial = localization.signal.axial;
                lateral = localization.signal.lateral;
                mask = mask.withTranslation(true);
                forwardErr = localization.forwardErrorIn;
                leftErr = localization.leftErrorIn;
                hasTransErr = true;
                translationSelection = localization.translationSelection;
            }
        }

        if (wantOmega) {
            if (hasLocalizationO && hasAprilTagsO) {
                omega = MathUtil.lerp(localization.signal.omega, aprilTags.signal.omega, blendTOmega);
                mask = mask.withOmega(true);
                omegaErr = lerpAngleError(localization.omegaErrorRad, aprilTags.omegaErrorRad, blendTOmega);
                hasOmegaErr = true;
                facingSelection = (blendTOmega >= 0.5) ? aprilTags.facingSelection : localization.facingSelection;
            } else if (hasAprilTagsO) {
                omega = aprilTags.signal.omega;
                mask = mask.withOmega(true);
                omegaErr = aprilTags.omegaErrorRad;
                hasOmegaErr = true;
                facingSelection = aprilTags.facingSelection;
            } else if (hasLocalizationO) {
                omega = localization.signal.omega;
                mask = mask.withOmega(true);
                omegaErr = localization.omegaErrorRad;
                hasOmegaErr = true;
                facingSelection = localization.facingSelection;
            }
        }

        Step out;
        if (mask.isNone()) {
            out = applyLossPolicy(CandidateSolution.invalid(), requested, rw.lossPolicy, "adaptive(loss)");
        } else {
            out = new Step(
                    new DriveOverlayOutput(new DriveSignal(axial, lateral, omega), mask),
                    "adaptive",
                    localization,
                    aprilTags,
                    aprilTagsInRangeForTranslation,
                    blendTTranslate,
                    blendTOmega,
                    hasTransErr,
                    forwardErr,
                    leftErr,
                    hasOmegaErr,
                    omegaErr,
                    translationSelection,
                    facingSelection
            );
        }

        lastMode = out.mode;
        lastStep = out;
        return out;
    }

    String lastMode() {
        return lastMode;
    }

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
                sol.omegaErrorRad,
                sol.translationSelection,
                sol.facingSelection
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
                        sol.omegaErrorRad,
                        sol.translationSelection,
                        sol.facingSelection
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
                    0.0,
                    NO_SELECTION,
                    NO_SELECTION
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
        final TagSelectionResult translationSelection;
        final TagSelectionResult facingSelection;

        CandidateSolution(boolean valid,
                          DriveSignal signal,
                          boolean canTranslate,
                          boolean canOmega,
                          boolean hasRangeInches,
                          double rangeInches,
                          double forwardErrorIn,
                          double leftErrorIn,
                          double omegaErrorRad,
                          TagSelectionResult translationSelection,
                          TagSelectionResult facingSelection) {
            this.valid = valid;
            this.signal = signal;
            this.canTranslate = canTranslate;
            this.canOmega = canOmega;
            this.hasRangeInches = hasRangeInches;
            this.rangeInches = rangeInches;
            this.forwardErrorIn = forwardErrorIn;
            this.leftErrorIn = leftErrorIn;
            this.omegaErrorRad = omegaErrorRad;
            this.translationSelection = translationSelection != null ? translationSelection : NO_SELECTION;
            this.facingSelection = facingSelection != null ? facingSelection : NO_SELECTION;
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
                    0.0,
                    NO_SELECTION,
                    NO_SELECTION);
        }
    }

    static final class Step {
        final DriveOverlayOutput out;
        final String mode;

        final CandidateSolution localization;
        final CandidateSolution aprilTags;

        final boolean aprilTagsInRangeForTranslation;
        final double blendTTranslate;
        final double blendTOmega;

        final boolean hasTranslationError;
        final double forwardErrorIn;
        final double leftErrorIn;

        final boolean hasOmegaError;
        final double omegaErrorRad;

        final TagSelectionResult translationSelection;
        final TagSelectionResult facingSelection;

        Step(DriveOverlayOutput out,
             String mode,
             CandidateSolution localization,
             CandidateSolution aprilTags,
             boolean aprilTagsInRangeForTranslation,
             double blendTTranslate,
             double blendTOmega,
             boolean hasTranslationError,
             double forwardErrorIn,
             double leftErrorIn,
             boolean hasOmegaError,
             double omegaErrorRad,
             TagSelectionResult translationSelection,
             TagSelectionResult facingSelection) {
            this.out = out;
            this.mode = mode;
            this.localization = localization;
            this.aprilTags = aprilTags;
            this.aprilTagsInRangeForTranslation = aprilTagsInRangeForTranslation;
            this.blendTTranslate = blendTTranslate;
            this.blendTOmega = blendTOmega;
            this.hasTranslationError = hasTranslationError;
            this.forwardErrorIn = forwardErrorIn;
            this.leftErrorIn = leftErrorIn;
            this.hasOmegaError = hasOmegaError;
            this.omegaErrorRad = omegaErrorRad;
            this.translationSelection = translationSelection != null ? translationSelection : NO_SELECTION;
            this.facingSelection = facingSelection != null ? facingSelection : NO_SELECTION;
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
                    0.0,
                    NO_SELECTION,
                    NO_SELECTION
            );
        }
    }
}
