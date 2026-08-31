package edu.ftcsushi.fw.actuation;

import java.util.Objects;

import edu.ftcsushi.fw.core.control.ScalarRegulator;
import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Package-private lifecycle boundary from one scalar regulator to normalized power.
 *
 * <p>The owning Plant remains responsible for mechanism targets and completion. This helper owns
 * only regulator evaluation, final power-domain normalization, seam-level command truth, and
 * best-effort output/regulator cleanup.</p>
 */
final class RegulatedPowerChannel {

    private static final double MIN_POWER = -1.0;
    private static final double MAX_POWER = 1.0;

    private final PowerOutput output;
    private final ScalarRegulator regulator;
    private final String controlPath;

    private double regulatorOutput = Double.NaN;
    private double normalizedPowerCommand = Double.NaN;
    private String status = "NOT_UPDATED";
    private boolean lastStopSubmitted;
    private boolean lastStopCompleted;
    private boolean lastResetSucceeded;
    private boolean terminalStopReassertRequired;

    RegulatedPowerChannel(PowerOutput output, ScalarRegulator regulator, String controlPath) {
        this.output = Objects.requireNonNull(output, "output");
        this.regulator = Objects.requireNonNull(regulator, "regulator");
        String path = Objects.requireNonNull(controlPath, "controlPath").trim();
        if (path.isEmpty()) throw new IllegalArgumentException("controlPath must not be blank");
        this.controlPath = path;
    }

    /** Evaluate the regulator once and submit one finite normalized command. */
    void update(double setpoint,
                double measurement,
                LoopClock clock,
                PlantLifecycle lifecycle) {
        Objects.requireNonNull(lifecycle, "lifecycle");
        if (!lifecycle.isActive()) return;
        lastStopSubmitted = false;
        lastStopCompleted = false;
        lastResetSucceeded = false;
        terminalStopReassertRequired = false;

        final double raw;
        boolean alreadySubmittedThisCycle = regulator instanceof StandardControl
                && ((StandardControl) regulator).hasSuccessfulResultForCycle(clock);
        boolean alreadyFailedThisCycle = regulator instanceof StandardControl
                && ((StandardControl) regulator).hasRetainedFailureForCycle(clock);
        try {
            raw = regulator.update(setpoint, measurement, clock);
        } catch (RuntimeException failure) {
            if (!lifecycle.isActive()) throw failure;
            if (alreadyFailedThisCycle) throw failure;
            regulatorOutput = Double.NaN;
            failStop("REGULATOR_FAILED", failure, lifecycle);
            return; // failStop always throws
        }
        if (!lifecycle.isActive()) return;
        regulatorOutput = raw;

        if (alreadySubmittedThisCycle) {
            status = "SAME_CYCLE_RETAINED_WITHOUT_WRITE";
            return;
        }

        if (!Double.isFinite(raw)) {
            IllegalStateException failure = new IllegalStateException(
                    controlPath + " regulator returned non-finite power " + raw
                            + " for setpoint=" + setpoint
                            + ", measurement=" + measurement
                            + ". Sushi rejected the command and attempted fail-stop cleanup;"
                            + " fix the regulator or control law.");
            failStop("NON_FINITE_REGULATOR_OUTPUT", failure, lifecycle);
            return; // failStop always throws
        }

        double normalized = raw;
        if (normalized < MIN_POWER) normalized = MIN_POWER;
        else if (normalized > MAX_POWER) normalized = MAX_POWER;

        try {
            output.setPower(normalized);
        } catch (RuntimeException failure) {
            if (!lifecycle.isActive()) {
                terminalStopReassertRequired = true;
                throw failure;
            }
            if (regulator instanceof StandardControl) {
                ((StandardControl) regulator).retainExternalFailureForCycle(clock, failure);
            }
            failStop("OUTPUT_WRITE_FAILED", failure, lifecycle);
            return; // failStop always throws
        }
        if (!lifecycle.isActive()) {
            // A reentrant terminal stop may have been overwritten by the remainder of the outer
            // hardware callback. The owning Plant must issue one final natural stop.
            terminalStopReassertRequired = true;
            return;
        }

        normalizedPowerCommand = normalized;
        status = Double.compare(raw, normalized) == 0
                ? "SUBMITTED"
                : "SATURATED_AND_SUBMITTED";
    }

    /** Reset controller state without claiming that any new hardware command was submitted. */
    void reset() {
        lastStopSubmitted = false;
        lastStopCompleted = false;
        lastResetSucceeded = false;
        regulatorOutput = Double.NaN;
        try {
            regulator.reset();
            lastResetSucceeded = true;
            status = "RESET_WITHOUT_WRITE";
        } catch (RuntimeException failure) {
            status = "RESET_FAILED_WITHOUT_WRITE";
            throw failure;
        }
    }

    /** Stop the output first, then reset the regulator, attempting both operations. */
    void stop() {
        stop(null);
    }

    /**
     * Stop this channel and one optional companion output before resetting the regulator.
     * Identity-equal outputs are stopped only once.
     */
    void stop(PowerOutput companionOutput) {
        regulatorOutput = Double.NaN;
        RuntimeException primary = null;
        boolean allOutputStopsSucceeded = true;

        try {
            output.stop();
        } catch (RuntimeException failure) {
            allOutputStopsSucceeded = false;
            primary = failure;
        }

        if (companionOutput != null && companionOutput != output) {
            try {
                companionOutput.stop();
            } catch (RuntimeException failure) {
                allOutputStopsSucceeded = false;
                primary = suppress(primary, failure);
            }
        }

        lastStopSubmitted = allOutputStopsSucceeded;
        normalizedPowerCommand = allOutputStopsSucceeded ? 0.0 : Double.NaN;

        boolean resetSucceeded = false;
        try {
            regulator.reset();
            resetSucceeded = true;
        } catch (RuntimeException failure) {
            primary = suppress(primary, failure);
        }

        publishStopDiagnostics(allOutputStopsSucceeded, resetSucceeded);
        if (primary != null) throw primary;
    }

    /**
     * Stop this channel during an active nonterminal operation.
     *
     * <p>If the output callback reentrantly claims the owning Plant's terminal lifecycle, that
     * public terminal stop owns controller cleanup and diagnostics. This outer operational stop
     * therefore returns or propagates its output failure without resetting the regulator or
     * overwriting the terminal facts.</p>
     */
    void stopWhileActive(PlantLifecycle lifecycle) {
        Objects.requireNonNull(lifecycle, "lifecycle");
        if (!lifecycle.isActive()) return;

        regulatorOutput = Double.NaN;
        RuntimeException primary = null;
        boolean outputStopSucceeded = false;
        try {
            output.stop();
            outputStopSucceeded = true;
        } catch (RuntimeException failure) {
            primary = failure;
        }
        if (!lifecycle.isActive()) {
            if (primary != null) throw primary;
            return;
        }

        boolean resetSucceeded = false;
        try {
            regulator.reset();
            resetSucceeded = true;
        } catch (RuntimeException failure) {
            primary = suppress(primary, failure);
        }
        if (!lifecycle.isActive()) {
            if (primary != null) throw primary;
            return;
        }

        publishStopDiagnostics(outputStopSucceeded, resetSucceeded);
        if (primary != null) throw primary;
    }

    /** Reassert terminal zero at the main output without resetting the regulator again. */
    void reassertTerminalOutputStop() {
        reassertTerminalOutputStop(null);
    }

    /**
     * Reassert terminal zero at the main and optional distinct companion outputs, attempting all
     * output stop operations but never resetting the regulator again.
     */
    void reassertTerminalOutputStop(PowerOutput companionOutput) {
        RuntimeException primary = null;
        boolean allOutputStopsSucceeded = true;
        try {
            output.stop();
        } catch (RuntimeException failure) {
            allOutputStopsSucceeded = false;
            primary = failure;
        }
        if (companionOutput != null && companionOutput != output) {
            try {
                companionOutput.stop();
            } catch (RuntimeException failure) {
                allOutputStopsSucceeded = false;
                primary = suppress(primary, failure);
            }
        }

        // Keep the controller-reset result established by the terminal stop. Only seam-level
        // command truth is refreshed here: normal reassertion submits zero; any failure makes that
        // logical command unknown. Neither result proves physical output.
        publishStopDiagnostics(allOutputStopsSucceeded, lastResetSucceeded);
        terminalStopReassertRequired = false;
        if (primary != null) throw primary;
    }

    double regulatorOutput() {
        return regulatorOutput;
    }

    double normalizedPowerCommand() {
        return normalizedPowerCommand;
    }

    String status() {
        return status;
    }

    /** True only when every distinct output in the most recent stop operation returned normally. */
    boolean lastStopSubmitted() {
        return lastStopSubmitted;
    }

    /** True only when the most recent stop submitted every output stop and reset the regulator. */
    boolean lastStopCompleted() {
        return lastStopCompleted;
    }

    /** Whether an outer power write returned after a reentrant terminal stop. */
    boolean terminalStopReassertRequired() {
        return terminalStopReassertRequired;
    }

    /** Whether the Plant-owned standard setpoint has reached this exact applied goal. */
    boolean setpointSettledAt(double goal) {
        return !(regulator instanceof StandardControl)
                || ((StandardControl) regulator).setpointSettledAt(goal);
    }

    void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        String p = (prefix == null || prefix.isEmpty()) ? "plant" : prefix;
        dbg.addData(p + ".regulatorOutput", regulatorOutput)
                .addData(p + ".normalizedPowerCommand", normalizedPowerCommand)
                .addData(p + ".regulatedPowerStatus", status);
        regulator.debugDump(dbg, p + ".regulator");
    }

    private void failStop(String failureStatus,
                          RuntimeException primary,
                          PlantLifecycle lifecycle) {
        boolean stopSucceeded = false;
        try {
            output.stop();
            stopSucceeded = true;
        } catch (RuntimeException cleanupFailure) {
            suppress(primary, cleanupFailure);
        }
        if (!lifecycle.isActive()) {
            // A reentrant public Plant.stop() owns the terminal cleanup facts. Do not overwrite
            // them or reset its controller a second time from this recoverable failure path.
            throw primary;
        }

        boolean resetSucceeded = false;
        try {
            if (regulator instanceof StandardControl) {
                ((StandardControl) regulator).resetAfterFailurePreservingCycle();
            } else {
                regulator.reset();
            }
            resetSucceeded = true;
        } catch (RuntimeException cleanupFailure) {
            suppress(primary, cleanupFailure);
        }
        if (!lifecycle.isActive()) throw primary;

        normalizedPowerCommand = stopSucceeded ? 0.0 : Double.NaN;
        lastStopSubmitted = stopSucceeded;
        lastResetSucceeded = resetSucceeded;
        status = failureStatus
                + "_STOP_" + (stopSucceeded ? "SUBMITTED" : "FAILED")
                + "_RESET_" + (resetSucceeded ? "SUCCEEDED" : "FAILED");
        lastStopCompleted = stopSucceeded && resetSucceeded;
        throw primary;
    }

    private void publishStopDiagnostics(boolean outputStopSucceeded, boolean resetSucceeded) {
        lastStopSubmitted = outputStopSucceeded;
        lastResetSucceeded = resetSucceeded;
        normalizedPowerCommand = outputStopSucceeded ? 0.0 : Double.NaN;
        status = "STOP_"
                + (outputStopSucceeded ? "SUBMITTED" : "FAILED")
                + "_RESET_"
                + (resetSucceeded ? "SUCCEEDED" : "FAILED");
        lastStopCompleted = outputStopSucceeded && resetSucceeded;
    }

    private static RuntimeException suppress(RuntimeException primary, RuntimeException additional) {
        if (primary == null) return additional;
        if (primary != additional) primary.addSuppressed(additional);
        return primary;
    }
}
