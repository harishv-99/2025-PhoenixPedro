package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.time.LoopClock;

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

    RegulatedPowerChannel(PowerOutput output, ScalarRegulator regulator, String controlPath) {
        this.output = Objects.requireNonNull(output, "output");
        this.regulator = Objects.requireNonNull(regulator, "regulator");
        String path = Objects.requireNonNull(controlPath, "controlPath").trim();
        if (path.isEmpty()) throw new IllegalArgumentException("controlPath must not be blank");
        this.controlPath = path;
    }

    /** Evaluate the regulator once and submit one finite normalized command. */
    void update(double setpoint, double measurement, LoopClock clock) {
        lastStopSubmitted = false;

        final double raw;
        try {
            raw = regulator.update(setpoint, measurement, clock);
        } catch (RuntimeException failure) {
            regulatorOutput = Double.NaN;
            failStop("REGULATOR_FAILED", failure);
            return; // failStop always throws
        }
        regulatorOutput = raw;

        if (!Double.isFinite(raw)) {
            IllegalStateException failure = new IllegalStateException(
                    controlPath + " regulator returned non-finite power " + raw
                            + " for setpoint=" + setpoint
                            + ", measurement=" + measurement
                            + ". Phoenix rejected the command and attempted fail-stop cleanup;"
                            + " fix the regulator or control law.");
            failStop("NON_FINITE_REGULATOR_OUTPUT", failure);
            return; // failStop always throws
        }

        double normalized = raw;
        if (normalized < MIN_POWER) normalized = MIN_POWER;
        else if (normalized > MAX_POWER) normalized = MAX_POWER;

        try {
            output.setPower(normalized);
        } catch (RuntimeException failure) {
            failStop("OUTPUT_WRITE_FAILED", failure);
            return; // failStop always throws
        }

        normalizedPowerCommand = normalized;
        status = Double.compare(raw, normalized) == 0
                ? "SUBMITTED"
                : "SATURATED_AND_SUBMITTED";
    }

    /** Reset controller state without claiming that any new hardware command was submitted. */
    void reset() {
        lastStopSubmitted = false;
        regulatorOutput = Double.NaN;
        try {
            regulator.reset();
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

        status = "STOP_"
                + (lastStopSubmitted ? "SUBMITTED" : "FAILED")
                + "_RESET_"
                + (resetSucceeded ? "SUCCEEDED" : "FAILED");
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

    void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        String p = (prefix == null || prefix.isEmpty()) ? "plant" : prefix;
        dbg.addData(p + ".regulatorOutput", regulatorOutput)
                .addData(p + ".normalizedPowerCommand", normalizedPowerCommand)
                .addData(p + ".regulatedPowerStatus", status);
        regulator.debugDump(dbg, p + ".regulator");
    }

    private void failStop(String failureStatus, RuntimeException primary) {
        boolean stopSucceeded = false;
        try {
            output.stop();
            normalizedPowerCommand = 0.0;
            lastStopSubmitted = true;
            stopSucceeded = true;
        } catch (RuntimeException cleanupFailure) {
            normalizedPowerCommand = Double.NaN;
            lastStopSubmitted = false;
            suppress(primary, cleanupFailure);
        }

        boolean resetSucceeded = false;
        try {
            regulator.reset();
            resetSucceeded = true;
        } catch (RuntimeException cleanupFailure) {
            suppress(primary, cleanupFailure);
        }

        status = failureStatus
                + "_STOP_" + (stopSucceeded ? "SUBMITTED" : "FAILED")
                + "_RESET_" + (resetSucceeded ? "SUCCEEDED" : "FAILED");
        throw primary;
    }

    private static RuntimeException suppress(RuntimeException primary, RuntimeException additional) {
        if (primary == null) return additional;
        if (primary != additional) primary.addSuppressed(additional);
        return primary;
    }
}
