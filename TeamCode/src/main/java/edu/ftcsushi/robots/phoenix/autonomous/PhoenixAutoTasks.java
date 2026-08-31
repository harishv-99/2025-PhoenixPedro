package edu.ftcsushi.robots.phoenix.autonomous;

import java.util.Objects;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.robots.phoenix.PhoenixAutoConfig;
import edu.ftcsushi.robots.phoenix.PhoenixCapabilities;

/**
 * Reusable autonomous task snippets expressed over {@link PhoenixCapabilities}.
 *
 * <p>These helpers are the Auto-side sibling of TeleOp button bindings: they describe high-level
 * Phoenix intents such as "aim and request one shot" without reaching into raw shooter, targeting,
 * or scoring internals.</p>
 */
public final class PhoenixAutoTasks {

    private PhoenixAutoTasks() {
        // Utility class.
    }

    /** Build one framework aim-task draft from the already validated Phoenix Auto answers. */
    private static DriveGuidanceTask.Config aimConfig(AttemptConfig auto) {
        DriveGuidanceTask.Config cfg = new DriveGuidanceTask.Config();
        cfg.headingTolRad = auto.aimHeadingToleranceRad;
        cfg.timeoutSec = auto.aimTimeoutSec;
        cfg.maxNoGuidanceSec = auto.aimMaxNoGuidanceSec;
        return cfg;
    }

    /**
     * Build one truthful scoring attempt: wait briefly for a selected target, capture shot
     * velocity, run Phoenix aim, request one shot, and wait for the shot queue to drain.
     *
     * <p>Each phase must succeed before the next phase begins. Target-selection, aiming, and shot-
     * drain timeouts remain {@link edu.ftcsushi.fw.task.TaskOutcome#TIMEOUT}; cancellation and
     * unknown terminal outcomes likewise remain visible to the owning Auto routine. Phoenix's
     * Pedro routine treats that bounded target/scoring timeout as the explicit return/park
     * fallback trigger; it is not a START-readiness failure. If an abnormal ending occurs after
     * the shot request, the Task best-effort asks scoring to cancel that transient action.</p>
     */
    public static Task aimAndShootOne(PhoenixCapabilities capabilities,
                                      DriveCommandSink driveSink,
                                      PhoenixAutoConfig autoConfig) {
        Objects.requireNonNull(capabilities, "capabilities");
        Objects.requireNonNull(driveSink, "driveSink");
        final AttemptConfig auto = AttemptConfig.capture(autoConfig);
        final PhoenixCapabilities.Scoring scoring = capabilities.scoring();
        final PhoenixCapabilities.Targeting targeting = capabilities.targeting();

        BooleanSource targetSelected = BooleanSource.of(
                () -> targeting.status().selection.hasSelection
        );
        Task waitForTargetTask = Tasks.waitUntil(targetSelected, auto.waitForTargetSec);
        Task aimTask = targeting.aimTask(driveSink, aimConfig(auto));
        Task waitForShotTask = Tasks.waitUntil(
                BooleanSource.of(() -> !scoring.hasPendingShots()),
                auto.waitForShotCompleteSec
        );

        return new PhoenixScoringAttemptTask(
                scoring,
                waitForTargetTask,
                aimTask,
                waitForShotTask
        );
    }

    /**
     * Create a task that spins down the flywheel request.
     */
    public static Task disableFlywheel(final PhoenixCapabilities capabilities) {
        Objects.requireNonNull(capabilities, "capabilities");
        return Tasks.runOnce(new Runnable() {
            @Override
            public void run() {
                capabilities.scoring().setFlywheelEnabled(false);
            }
        });
    }

    /** Immutable active slice captured before any scoring-attempt Task is created. */
    private static final class AttemptConfig {
        final double aimHeadingToleranceRad;
        final double aimTimeoutSec;
        final double aimMaxNoGuidanceSec;
        final double waitForTargetSec;
        final double waitForShotCompleteSec;

        private AttemptConfig(double aimHeadingToleranceRad,
                              double aimTimeoutSec,
                              double aimMaxNoGuidanceSec,
                              double waitForTargetSec,
                              double waitForShotCompleteSec) {
            this.aimHeadingToleranceRad = aimHeadingToleranceRad;
            this.aimTimeoutSec = aimTimeoutSec;
            this.aimMaxNoGuidanceSec = aimMaxNoGuidanceSec;
            this.waitForTargetSec = waitForTargetSec;
            this.waitForShotCompleteSec = waitForShotCompleteSec;
        }

        /** Capture every retained primitive, then validate it in PhoenixAutoConfig source order. */
        static AttemptConfig capture(PhoenixAutoConfig source) {
            PhoenixAutoConfig required = Objects.requireNonNull(
                    source,
                    "PhoenixAutoTasks autoConfig is required"
            );
            double aimHeadingToleranceDeg = required.aimHeadingToleranceDeg;
            double aimTimeoutSec = required.aimTimeoutSec;
            double aimMaxNoGuidanceSec = required.aimMaxNoGuidanceSec;
            double waitForTargetSec = required.waitForTargetSec;
            double waitForShotCompleteSec = required.waitForShotCompleteSec;

            requireFiniteNonNegative(
                    "aimHeadingToleranceDeg",
                    aimHeadingToleranceDeg
            );
            double aimHeadingToleranceRad = Math.toRadians(aimHeadingToleranceDeg);
            if (!Double.isFinite(aimHeadingToleranceRad)) {
                throw new IllegalArgumentException(
                        "PhoenixAutoConfig.aimHeadingToleranceDeg must convert to finite radians, "
                                + "but " + aimHeadingToleranceDeg + " degrees converted to "
                                + aimHeadingToleranceRad
                );
            }
            requireFinitePositive("aimTimeoutSec", aimTimeoutSec);
            requireFinitePositive("aimMaxNoGuidanceSec", aimMaxNoGuidanceSec);
            requireFiniteNonNegative("waitForTargetSec", waitForTargetSec);
            requireFiniteNonNegative("waitForShotCompleteSec", waitForShotCompleteSec);

            return new AttemptConfig(
                    aimHeadingToleranceRad,
                    aimTimeoutSec,
                    aimMaxNoGuidanceSec,
                    waitForTargetSec,
                    waitForShotCompleteSec
            );
        }
    }

    private static void requireFinitePositive(String fieldName, double value) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(
                    "PhoenixAutoConfig." + fieldName + " must be finite and > 0, got " + value
            );
        }
    }

    private static void requireFiniteNonNegative(String fieldName, double value) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(
                    "PhoenixAutoConfig." + fieldName + " must be finite and >= 0, got " + value
            );
        }
    }
}
