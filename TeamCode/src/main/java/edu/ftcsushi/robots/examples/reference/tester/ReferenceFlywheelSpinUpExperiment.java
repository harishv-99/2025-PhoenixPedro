package edu.ftcsushi.robots.examples.reference.tester;

import java.util.Objects;

import edu.ftcsushi.fw.tools.tester.BaseTeleOpTester;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncher;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncherMechanism;

/**
 * Measures independent flywheel spin-up evidence without feeding or scoring an object.
 *
 * <p>Its powered-run boundary is cooperative, not a hard real-time cutoff. It is checked each
 * active loop, and zero is applied on the first loop that observes the boundary.</p>
 */
final class ReferenceFlywheelSpinUpExperiment extends BaseTeleOpTester {

    /** One retained trial outcome; only a new A-button trial replaces a terminal state. */
    enum TrialState {
        IDLE,
        RUNNING,
        TARGET_REACHED,
        TIME_LIMIT_REACHED,
        ABORTED
    }

    private final ReferenceLauncherMechanism.Config launcherConfig;
    private final boolean reviewedForMotion;
    private final double targetVelocityTicksPerSec;
    private final double maximumPoweredRunSec;

    private ReferenceLauncherMechanism launcher;
    private boolean active;
    private TrialState trialState = TrialState.IDLE;
    private long trialNumber;
    private double startedAtSec = Double.NaN;
    private double terminalTargetVelocityTicksPerSec = Double.NaN;
    private double terminalElapsedSec = Double.NaN;
    private double terminalLeftMeasuredVelocityTicksPerSec = Double.NaN;
    private double terminalRightMeasuredVelocityTicksPerSec = Double.NaN;
    private boolean terminalLeftAtTarget;
    private boolean terminalRightAtTarget;

    /** Defensively snapshots the launcher recipe and the team's experiment card. */
    ReferenceFlywheelSpinUpExperiment(
            ReferenceLauncherMechanism.Config launcherConfig,
            ReferenceFlywheelSpinUpCriteria criteria) {
        this.launcherConfig = copyLauncherConfig(Objects.requireNonNull(
                launcherConfig,
                "launcherConfig"));
        ReferenceFlywheelSpinUpCriteria requiredCriteria = Objects.requireNonNull(
                criteria,
                "criteria");
        reviewedForMotion = requiredCriteria.reviewedForMotion;
        targetVelocityTicksPerSec = requiredCriteria.targetVelocityTicksPerSec;
        maximumPoweredRunSec = requiredCriteria.maximumPoweredRunSec;
    }

    /** {@inheritDoc} */
    @Override
    public String name() {
        return "Reference flywheel spin-up";
    }

    /** Validate the locked/current criteria before optionally acquiring any FTC hardware. */
    @Override
    protected void onInit() {
        validateCriteria();
        if (!reviewedForMotion) {
            return;
        }
        launcher = new ReferenceLauncherMechanism(ctx.hw, launcherConfig);
        bindings.onRise(gamepads.p1().a(), () -> {
            if (active) {
                beginTrial();
            }
        });
        bindings.onRise(gamepads.p1().b(), () -> {
            if (active) {
                abortTrial();
            }
        });
    }

    /** Present the locked card or retained experiment evidence without actuating during INIT. */
    @Override
    protected void onInitLoop(double dtSec) {
        present();
    }

    /** Arm experiment controls only at the tester's explicit ACTIVE transition. */
    @Override
    protected void onStart() {
        active = true;
    }

    /**
     * Decides from the latest cached evidence before the one mechanism output update.
     *
     * <p>This keeps the copied powered-run boundary authoritative. Readiness first published by an
     * update strictly before the boundary is frozen in that same loop. If no readiness was already
     * observed when an active loop reaches or overshoots the boundary, the time-limit gate freezes
     * that loop's observed elapsed time and requests zero before its Plant command. A new
     * boundary-cycle measurement cannot relabel that timeout.</p>
     */
    @Override
    protected void onLoop(double dtSec) {
        if (launcher != null) {
            if (trialState == TrialState.RUNNING) {
                ReferenceLauncher.Status status = launcher.status();
                double elapsedSec = runningElapsedSec();
                if (status.ready) {
                    finishTrial(TrialState.TARGET_REACHED, status, elapsedSec);
                } else if (elapsedSec >= maximumPoweredRunSec) {
                    finishTrial(TrialState.TIME_LIMIT_REACHED, status, elapsedSec);
                }
            }
            launcher.update(clock);
            if (trialState == TrialState.RUNNING) {
                ReferenceLauncher.Status status = launcher.status();
                if (status.ready) {
                    finishTrial(
                            TrialState.TARGET_REACHED,
                            status,
                            runningElapsedSec());
                }
            }
        }
        present();
    }

    /** Terminally releases the experiment's mechanism resources when its tester session ends. */
    @Override
    protected void onStop() {
        active = false;
        if (launcher != null) {
            launcher.stop();
        }
    }

    /** Start a new numbered trial; this is the only operation that replaces retained evidence. */
    private void beginTrial() {
        if (trialState == TrialState.RUNNING) {
            return;
        }
        if (trialNumber == Long.MAX_VALUE) {
            throw new IllegalStateException("flywheel spin-up trialNumber cannot increase further");
        }
        trialNumber++;
        trialState = TrialState.RUNNING;
        startedAtSec = clock.nowSec();
        clearTerminalEvidence();
        launcher.setTargetVelocityTicksPerSec(targetVelocityTicksPerSec);
    }

    /** Retain an active B-button abort without relabeling an already terminal trial. */
    private void abortTrial() {
        if (trialState == TrialState.RUNNING) {
            finishTrial(TrialState.ABORTED, launcher.status(), runningElapsedSec());
        } else {
            launcher.abortLaunches();
        }
    }

    /** Freeze all result evidence before the abort request changes the persistent wheel target. */
    private void finishTrial(TrialState terminalState,
                             ReferenceLauncher.Status status,
                             double elapsedSec) {
        // The copied trial answer remains truthful even when A and B rise before the first
        // mechanism update has published the newly requested target.
        terminalTargetVelocityTicksPerSec = targetVelocityTicksPerSec;
        terminalElapsedSec = elapsedSec;
        terminalLeftMeasuredVelocityTicksPerSec =
                status.leftMeasuredVelocityTicksPerSec;
        terminalRightMeasuredVelocityTicksPerSec =
                status.rightMeasuredVelocityTicksPerSec;
        terminalLeftAtTarget = status.leftAtTarget;
        terminalRightAtTarget = status.rightAtTarget;
        trialState = terminalState;
        launcher.abortLaunches();
    }

    /** Draw only the locked gate or the computed per-wheel evidence needed by this experiment. */
    private void present() {
        telemHeader(name());
        if (!reviewedForMotion) {
            ctx.telemetry.addData("trialState", trialState);
            ctx.telemetry.addLine("LOCKED: team physical criteria have not been reviewed");
            ctx.telemetry.addLine(
                    "Review ReferenceFlywheelSpinUpCriteria.current() before enabling motion.");
            telemHint("A: begin after review | B: abort");
            telemUpdate();
            return;
        }

        ctx.telemetry.addData("trialState", trialState);
        if (trialState == TrialState.IDLE) {
            telemHint("A: begin | B: abort");
            telemUpdate();
            return;
        }

        ReferenceLauncher.Status liveStatus = launcher.status();
        boolean terminal = isTerminal(trialState);
        double displayedTargetVelocityTicksPerSec = terminal
                ? terminalTargetVelocityTicksPerSec
                : trialState == TrialState.RUNNING
                ? liveStatus.targetVelocityTicksPerSec
                : targetVelocityTicksPerSec;
        double displayedLeftMeasuredVelocityTicksPerSec = terminal
                ? terminalLeftMeasuredVelocityTicksPerSec
                : liveStatus.leftMeasuredVelocityTicksPerSec;
        double displayedRightMeasuredVelocityTicksPerSec = terminal
                ? terminalRightMeasuredVelocityTicksPerSec
                : liveStatus.rightMeasuredVelocityTicksPerSec;
        boolean displayedLeftAtTarget = terminal
                ? terminalLeftAtTarget
                : liveStatus.leftAtTarget;
        boolean displayedRightAtTarget = terminal
                ? terminalRightAtTarget
                : liveStatus.rightAtTarget;
        double displayedElapsedSec = terminal
                ? terminalElapsedSec
                : trialState == TrialState.RUNNING ? runningElapsedSec() : 0.0;

        ctx.telemetry.addData("trialNumber", trialNumber);
        ctx.telemetry.addData(
                "targetVelocityTicksPerSec",
                displayedTargetVelocityTicksPerSec);
        ctx.telemetry.addData(
                "leftMeasuredVelocityTicksPerSec",
                displayedLeftMeasuredVelocityTicksPerSec);
        ctx.telemetry.addData(
                "leftVelocityErrorTicksPerSec",
                displayedTargetVelocityTicksPerSec
                        - displayedLeftMeasuredVelocityTicksPerSec);
        ctx.telemetry.addData("leftAtTarget", displayedLeftAtTarget);
        ctx.telemetry.addData(
                "rightMeasuredVelocityTicksPerSec",
                displayedRightMeasuredVelocityTicksPerSec);
        ctx.telemetry.addData(
                "rightVelocityErrorTicksPerSec",
                displayedTargetVelocityTicksPerSec
                        - displayedRightMeasuredVelocityTicksPerSec);
        ctx.telemetry.addData("rightAtTarget", displayedRightAtTarget);
        ctx.telemetry.addData("elapsedSec", displayedElapsedSec);
        if (trialState == TrialState.TARGET_REACHED) {
            ctx.telemetry.addData("spinUpSec", terminalElapsedSec);
        }
        telemHint("A: begin | B: abort");
        telemUpdate();
    }

    /** Return current nonnegative trial elapsed time from the shared tester clock. */
    private double runningElapsedSec() {
        return Math.max(0.0, clock.nowSec() - startedAtSec);
    }

    /** Clear retained terminal values only when a new numbered trial begins. */
    private void clearTerminalEvidence() {
        terminalTargetVelocityTicksPerSec = Double.NaN;
        terminalElapsedSec = Double.NaN;
        terminalLeftMeasuredVelocityTicksPerSec = Double.NaN;
        terminalRightMeasuredVelocityTicksPerSec = Double.NaN;
        terminalLeftAtTarget = false;
        terminalRightAtTarget = false;
    }

    /** Return whether one state owns a frozen result snapshot. */
    private static boolean isTerminal(TrialState state) {
        return state == TrialState.TARGET_REACHED
                || state == TrialState.TIME_LIMIT_REACHED
                || state == TrialState.ABORTED;
    }

    /** Validate the copied experiment answers before the mechanism can touch HardwareMap. */
    private void validateCriteria() {
        if (!Double.isFinite(targetVelocityTicksPerSec)
                || targetVelocityTicksPerSec <= 0.0) {
            throw new IllegalArgumentException(
                    "targetVelocityTicksPerSec must be finite and > 0");
        }
        if (targetVelocityTicksPerSec
                <= launcherConfig.velocityToleranceTicksPerSec) {
            throw new IllegalArgumentException(
                    "targetVelocityTicksPerSec must be > "
                            + "launcherConfig.velocityToleranceTicksPerSec");
        }
        if (targetVelocityTicksPerSec
                > launcherConfig.maximumVelocityTicksPerSec) {
            throw new IllegalArgumentException(
                    "targetVelocityTicksPerSec must be <= "
                            + "launcherConfig.maximumVelocityTicksPerSec");
        }
        if (!Double.isFinite(maximumPoweredRunSec) || maximumPoweredRunSec <= 0.0) {
            throw new IllegalArgumentException(
                    "maximumPoweredRunSec must be finite and > 0");
        }
    }

    /** Copy every public data-only launcher answer without introducing another construction API. */
    private static ReferenceLauncherMechanism.Config copyLauncherConfig(
            ReferenceLauncherMechanism.Config source) {
        ReferenceLauncherMechanism.Config copy = ReferenceLauncherMechanism.Config.defaults();
        copy.leftFlywheelName = source.leftFlywheelName;
        copy.leftFlywheelDirection = source.leftFlywheelDirection;
        copy.rightFlywheelName = source.rightFlywheelName;
        copy.rightFlywheelDirection = source.rightFlywheelDirection;
        copy.transferName = source.transferName;
        copy.transferDirection = source.transferDirection;
        copy.releaseServoName = source.releaseServoName;
        copy.releaseServoDirection = source.releaseServoDirection;
        copy.objectSensorName = source.objectSensorName;
        copy.maximumVelocityTicksPerSec = source.maximumVelocityTicksPerSec;
        copy.velocityToleranceTicksPerSec = source.velocityToleranceTicksPerSec;
        copy.launchVelocityTicksPerSec = source.launchVelocityTicksPerSec;
        copy.spinUpTimeoutSec = source.spinUpTimeoutSec;
        copy.transferPower = source.transferPower;
        copy.transferDurationSec = source.transferDurationSec;
        copy.releaseRetractedPosition = source.releaseRetractedPosition;
        copy.releaseExtendedPosition = source.releaseExtendedPosition;
        copy.releaseDurationSec = source.releaseDurationSec;
        return copy;
    }
}
