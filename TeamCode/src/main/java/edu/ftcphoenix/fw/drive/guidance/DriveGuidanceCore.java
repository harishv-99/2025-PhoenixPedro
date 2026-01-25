package edu.ftcphoenix.fw.drive.guidance;

import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveOverlayOutput;
import edu.ftcphoenix.fw.drive.DriveSignal;

/**
 * Shared "engine" used by both {@link DriveGuidanceOverlay} (teleop overlays) and
 * {@link DriveGuidanceTask} (autonomous tasks).
 *
 * <p>This class exists to remove duplication: the hard part of DriveGuidance isn't the wrapper
 * (overlay vs task), it's the evaluation + feedback selection + blending + loss policy. Keeping
 * that logic in one place makes robot code easier to write and keeps behavior consistent.</p>
 */
final class DriveGuidanceCore {

    private final DriveGuidancePlan plan;
    private final DriveGuidanceEvaluator evaluator;

    private boolean obsInRangeForTranslation;
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
        obsInRangeForTranslation = false;
        blendTTranslate = 0.0;
        blendTOmega = 0.0;
        evaluator.onEnable();
        lastMode = "enabled";
        lastStep = Step.noCommand("enabled");
    }

    Step step(LoopClock clock, DriveOverlayMask requested) {
        DriveGuidanceSpec.Feedback fb = plan.spec.feedback;

        if (requested == null || requested.isNone()) {
            lastMode = "none";
            lastStep = Step.noCommand("none");
            return lastStep;
        }

        // Compute candidate solutions.
        CandidateSolution field = fb.hasFieldPose()
                ? toCandidate(evaluator.solveWithFieldPose())
                : CandidateSolution.invalid();

        CandidateSolution obs = fb.hasObservation()
                ? toCandidate(evaluator.solveWithObservation(clock))
                : CandidateSolution.invalid();

        if (!fb.isAdaptive()) {
            CandidateSolution chosen = fb.hasObservation() ? obs : field;
            lastMode = fb.hasObservation() ? "observation" : "fieldPose";

            Step out = applyLossPolicy(chosen, requested, fb.lossPolicy, lastMode);
            lastStep = out;
            return out;
        }

        // Adaptive: choose per DOF.
        DriveGuidanceSpec.Gates gates = (fb.gates != null) ? fb.gates : DriveGuidanceSpec.Gates.defaults();

        boolean wantTranslation = requested.overridesTranslation();
        boolean wantOmega = requested.overridesOmega();

        // --- Translation source selection
        boolean hasObsT = obs.valid && obs.canTranslate && obs.hasRangeInches;
        boolean hasFieldT = field.valid && field.canTranslate;

        if (!hasObsT) {
            obsInRangeForTranslation = false;
        } else {
            // Hysteresis on observed range.
            if (!obsInRangeForTranslation) {
                if (obs.rangeInches <= gates.enterRangeInches) {
                    obsInRangeForTranslation = true;
                }
            } else {
                if (obs.rangeInches >= gates.exitRangeInches) {
                    obsInRangeForTranslation = false;
                }
            }
        }

        boolean chooseObsForTranslation;
        if (!wantTranslation) {
            chooseObsForTranslation = false;
        } else if (!hasFieldT && hasObsT) {
            chooseObsForTranslation = true;
        } else if (hasFieldT && hasObsT) {
            chooseObsForTranslation = obsInRangeForTranslation;
        } else {
            chooseObsForTranslation = false;
        }

        // --- Omega source selection
        boolean hasObsO = obs.valid && obs.canOmega;
        boolean hasFieldO = field.valid && field.canOmega;

        boolean chooseObsForOmega;
        if (!wantOmega) {
            chooseObsForOmega = false;
        } else if (!hasFieldO && hasObsO) {
            chooseObsForOmega = true;
        } else if (fb.preferObservationForOmega) {
            chooseObsForOmega = hasObsO;
        } else if (hasFieldO && hasObsO) {
            // If not preferring observation, keep omega tied to translation choice when possible.
            chooseObsForOmega = chooseObsForTranslation;
        } else {
            chooseObsForOmega = false;
        }

        // --- Blend to smooth source switching
        double dt = clock.dtSec();
        double blendSec = Math.max(0.0, gates.takeoverBlendSec);
        double step = (blendSec <= 0.0) ? 1.0 : MathUtil.clamp(dt / blendSec, 0.0, 1.0);

        blendTTranslate = updateBlend(blendTTranslate, chooseObsForTranslation, step);
        blendTOmega = updateBlend(blendTOmega, chooseObsForOmega, step);

        // Compose final command.
        double axial = 0.0;
        double lateral = 0.0;
        double omega = 0.0;

        DriveOverlayMask mask = DriveOverlayMask.NONE;

        // Effective errors corresponding to the commanded DOFs.
        boolean hasTransErr = false;
        double forwardErr = 0.0;
        double leftErr = 0.0;

        boolean hasOmegaErr = false;
        double omegaErr = 0.0;

        if (wantTranslation) {
            if (hasFieldT && hasObsT) {
                axial = MathUtil.lerp(field.signal.axial, obs.signal.axial, blendTTranslate);
                lateral = MathUtil.lerp(field.signal.lateral, obs.signal.lateral, blendTTranslate);
                mask = mask.withTranslation(true);

                forwardErr = MathUtil.lerp(field.forwardErrorIn, obs.forwardErrorIn, blendTTranslate);
                leftErr = MathUtil.lerp(field.leftErrorIn, obs.leftErrorIn, blendTTranslate);
                hasTransErr = true;
            } else if (hasObsT) {
                axial = obs.signal.axial;
                lateral = obs.signal.lateral;
                mask = mask.withTranslation(true);

                forwardErr = obs.forwardErrorIn;
                leftErr = obs.leftErrorIn;
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
            if (hasFieldO && hasObsO) {
                omega = MathUtil.lerp(field.signal.omega, obs.signal.omega, blendTOmega);
                mask = mask.withOmega(true);

                omegaErr = lerpAngleError(field.omegaErrorRad, obs.omegaErrorRad, blendTOmega);
                hasOmegaErr = true;
            } else if (hasObsO) {
                omega = obs.signal.omega;
                mask = mask.withOmega(true);

                omegaErr = obs.omegaErrorRad;
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
                    obs,
                    obsInRangeForTranslation,
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

    String lastMode() {
        return lastMode;
    }

    Step lastStep() {
        return lastStep;
    }

    boolean obsInRangeForTranslation() {
        return obsInRangeForTranslation;
    }

    double blendTTranslate() {
        return blendTTranslate;
    }

    double blendTOmega() {
        return blendTOmega;
    }

    int lastObservedTagId() {
        return evaluator.lastObservedTagId();
    }

    edu.ftcphoenix.fw.core.geometry.Pose2d fieldToTranslationFrameAnchor() {
        return evaluator.fieldToTranslationFrameAnchor();
    }

    // ------------------------------------------------------------------------
    // Candidate conversion
    // ------------------------------------------------------------------------

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

    // ------------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------------

    private static double updateBlend(double current, boolean chooseObs, double step) {
        if (chooseObs) {
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

        // If we couldn't produce any usable command this loop.
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

    /**
     * Interpolate an angle-like error in radians via the shortest-path difference.
     */
    private static double lerpAngleError(double aRad, double bRad, double t) {
        double delta = MathUtil.wrapToPi(bRad - aRad);
        return MathUtil.wrapToPi(aRad + t * delta);
    }

    // ------------------------------------------------------------------------
    // Data bundles
    // ------------------------------------------------------------------------

    /**
     * Per-source candidate solution.
     */
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

    /**
     * Output of a single core step.
     */
    static final class Step {
        final DriveOverlayOutput out;
        final String mode;

        final CandidateSolution field;
        final CandidateSolution obs;

        final boolean obsInRangeForTranslation;
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
             CandidateSolution obs,
             boolean obsInRangeForTranslation,
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
            this.obs = obs;
            this.obsInRangeForTranslation = obsInRangeForTranslation;
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
