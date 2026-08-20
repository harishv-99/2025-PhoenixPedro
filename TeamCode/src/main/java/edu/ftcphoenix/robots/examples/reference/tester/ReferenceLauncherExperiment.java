package edu.ftcphoenix.robots.examples.reference.tester;

import java.util.Objects;

import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.robots.examples.reference.capability.launcher.ReferenceLauncher;
import edu.ftcphoenix.robots.examples.reference.capability.launcher.ReferenceLauncherMechanism;

/** Measures flywheel spin-up evidence without claiming whether a launched object scored. */
final class ReferenceLauncherExperiment extends BaseTeleOpTester {
    private final ReferenceLauncherMechanism.Config launcherConfig;
    private final ReferenceExperimentCriteria criteria;
    private ReferenceLauncherMechanism launcher;
    private double startedAtSec = Double.NaN;
    private double reachedAtSec = Double.NaN;

    ReferenceLauncherExperiment(ReferenceLauncherMechanism.Config launcherConfig,
                                ReferenceExperimentCriteria criteria) {
        this.launcherConfig = Objects.requireNonNull(launcherConfig, "launcherConfig");
        this.criteria = Objects.requireNonNull(criteria, "criteria");
    }

    @Override
    public String name() {
        return "Reference launcher spin-up";
    }

    @Override
    protected void onInit() {
        validateCriteria();
        if (!criteria.reviewedForMotion) {
            return;
        }
        launcher = new ReferenceLauncherMechanism(ctx.hw, launcherConfig);
        bindings.onRise(gamepads.p1().a(), this::beginTrial);
        bindings.onRise(gamepads.p1().b(), this::idle);
    }

    @Override
    protected void onInitLoop(double dtSec) {
        present();
    }

    @Override
    protected void onLoop(double dtSec) {
        if (launcher != null) {
            launcher.update(clock);
            if (Double.isFinite(startedAtSec)
                    && !Double.isFinite(reachedAtSec)
                    && launcher.status().ready) {
                reachedAtSec = clock.nowSec();
            }
            if (Double.isFinite(startedAtSec)
                    && clock.nowSec() - startedAtSec >= criteria.maximumTrialSec) {
                launcher.idle();
            }
        }
        present();
    }

    @Override
    protected void onStop() {
        if (launcher != null) {
            launcher.stop();
        }
    }

    private void beginTrial() {
        startedAtSec = clock.nowSec();
        reachedAtSec = Double.NaN;
        launcher.setTargetVelocity(criteria.targetVelocity);
    }

    private void idle() {
        launcher.idle();
        startedAtSec = Double.NaN;
    }

    private void present() {
        telemHeader(name());
        if (!criteria.reviewedForMotion) {
            ctx.telemetry.addLine("LOCKED: team physical criteria have not been reviewed");
            ctx.telemetry.addLine("Edit ReferenceExperimentCriteria only after the lab card review.");
            telemUpdate();
            return;
        }
        ReferenceLauncher.Status status = launcher.status();
        double elapsed = Double.isFinite(startedAtSec)
                ? Math.max(0.0, clock.nowSec() - startedAtSec) : 0.0;
        ctx.telemetry.addData("trial", Double.isFinite(startedAtSec) ? "ACTIVE" : "IDLE");
        ctx.telemetry.addData("targetVelocity", status.targetVelocity);
        ctx.telemetry.addData("measuredVelocity", status.measuredVelocity);
        ctx.telemetry.addData("velocityError", status.targetVelocity - status.measuredVelocity);
        ctx.telemetry.addData("elapsedSec", elapsed);
        ctx.telemetry.addData("controllerReady", status.ready);
        ctx.telemetry.addData("spinUpSec", Double.isFinite(reachedAtSec)
                ? reachedAtSec - startedAtSec : "not reached");
        ctx.telemetry.addData("trialOutcome", status.ready
                ? "CONTROLLER_TARGET_REACHED"
                : elapsed >= criteria.maximumTrialSec ? "TIME_LIMIT_REACHED" : "RUNNING");
        telemHint("A: begin | B: idle | record physical observations outside the RC");
        telemUpdate();
    }

    private void validateCriteria() {
        if (!Double.isFinite(criteria.targetVelocity) || criteria.targetVelocity <= 0.0) {
            throw new IllegalArgumentException("targetVelocity must be finite and > 0");
        }
        if (criteria.targetVelocity > launcherConfig.maximumVelocity) {
            throw new IllegalArgumentException(
                    "targetVelocity must be <= launcherConfig.maximumVelocity");
        }
        if (!Double.isFinite(criteria.maximumTrialSec) || criteria.maximumTrialSec <= 0.0) {
            throw new IllegalArgumentException("maximumTrialSec must be finite and > 0");
        }
    }
}
