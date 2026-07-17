package edu.ftcphoenix.fw.tools.tester.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.ftc.ui.HardwareNamePicker;
import edu.ftcphoenix.fw.tools.tester.ui.ScalarTuner;

/**
 * Generic tester for a configured {@link DcMotor}/{@link DcMotorEx} that lets you vary motor power.
 *
 * <h2>Selection</h2>
 * If constructed without a motor name (or the preferred name cannot be resolved), shows a picker
 * listing configured motors.
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>PICKER (no motor chosen yet)</b>: Dpad Up/Down highlight, A choose, X refresh</li>
 *   <li><b>RUN (motor chosen)</b>:
 *     <ul>
 *       <li>A: enable/disable output</li>
 *       <li>X: invert</li>
 *       <li>START: fine/coarse step</li>
 *       <li>Dpad Up/Down: step power</li>
 *       <li>Left stick Y: live override (sets target while moved)</li>
 *       <li>B: zero</li>
 *       <li>Y: start/stop direct-vs-derived velocity capture in Logcat</li>
 *       <li>Right bumper: skip one comparison sample without blocking the motor loop</li>
 *       <li>BACK: return to picker (change motor)</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p>The velocity comparison is diagnostic only. While its opt-in capture is active, it samples the
 * selected motor port's position and direct SDK velocity once per shared loop cycle, computes an
 * unfiltered interval-average velocity, and writes CSV-shaped rows under the
 * {@code PhoenixEncoderVelocity} Logcat tag. It temporarily selects
 * {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER} for safe open-loop testing and restores the prior mode
 * afterward. It does not select a production feedback strategy.</p>
 */
public final class DcMotorPowerTester extends BaseTeleOpTester {

    private static final String VELOCITY_COMPARISON_LOG_TAG = "PhoenixEncoderVelocity";

    private final String preferredName;

    private HardwareNamePicker picker;

    private String motorName = null;
    private DcMotor motor = null;
    private DcMotorEx motorEx = null;
    private DcMotor.RunMode originalRunMode = null;
    private LynxModule selectedLynxModule = null;

    private boolean ready = false;
    private boolean opModeStarted = false;
    private long selectedMotorCycle = Long.MIN_VALUE;
    private String resolveError = null;

    private final MotorVelocityComparison velocityComparison = new MotorVelocityComparison();
    private MotorVelocityComparison.Sample velocitySample;
    private boolean captureVelocityComparison;
    private int velocityCaptureSession;
    private boolean skipNextVelocityComparisonSample;
    private long velocityHardwareSampleCycle = Long.MIN_VALUE;
    private boolean directVelocityAvailable;
    private boolean coherentHardwareSnapshot;
    private String bulkCachingModeAtSample = "unavailable";
    private boolean bulkCachingModeRestored;
    private String velocityMeasurementStatus;
    private String velocityMeasurementError;
    private double lastSubmittedPower;

    private final ScalarTuner power =
            new ScalarTuner("Power", -1.0, +1.0, 0.05, 0.20, 0.0);

    /**
     * Create a DC motor power tester with no preferred device name.
     *
     * <p>A picker menu is shown so you can choose a configured motor.</p>
     */
    public DcMotorPowerTester() {
        this(null);
    }

    /**
     * Create a DC motor power tester with a preferred device name.
     *
     * <p>If {@code motorName} is null/blank or cannot be resolved, the tester will fall back to the picker menu.</p>
     *
     * @param motorName configured motor name in the FTC Robot Configuration (nullable)
     */
    public DcMotorPowerTester(String motorName) {
        this.preferredName = motorName;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String name() {
        return "DcMotor Power Tester";
    }

    /** {@inheritDoc} */
    @Override
    protected void onInit() {
        opModeStarted = false;
        resetPowerControl();

        picker = new HardwareNamePicker(
                ctx.hw,
                DcMotor.class,
                "Select Motor",
                "Dpad: highlight | A: choose | X: refresh"
        );
        picker.refresh();

        // Stick override maps directly to [-1..+1] target (Phoenix leftY is +up, -down).
        power.attachAxis(gamepads.p1().leftY(), 0.08, v -> v);

        // Prefer name passed in (RobotConfig), but fall back to picker if it fails.
        if (preferredName != null && !preferredName.trim().isEmpty()) {
            motorName = preferredName.trim();
            tryResolveMotor(motorName);
            if (!ready) {
                picker.setPreferredName(motorName);
            }
        }

        // Picker controls active only while NOT ready.
        picker.bind(
                bindings,
                gamepads.p1().dpadUp(),
                gamepads.p1().dpadDown(),
                gamepads.p1().a(),
                gamepads.p1().x(),
                () -> !ready,
                chosen -> {
                    motorName = chosen;
                    tryResolveMotor(motorName);
                }
        );

        // Standard scalar bindings (only when ready)
        power.bind(
                bindings,
                gamepads.p1().a(),         // enable
                gamepads.p1().x(),         // invert
                gamepads.p1().start(),     // fine/coarse
                gamepads.p1().dpadUp(),    // inc
                gamepads.p1().dpadDown(),  // dec
                gamepads.p1().b(),         // zero
                this::controlsActive
        );

        // Keep diagnostic capture opt-in so the ordinary motor-power tester stays quiet in Logcat.
        bindings.onRise(gamepads.p1().y(), () -> {
            if (!controlsActive()) return;

            if (captureVelocityComparison) {
                endVelocityCapture("Y_TOGGLE");
            } else {
                beginVelocityCapture();
            }
        });

        // Exercise a longer accepted interval without sleeping or pausing the motor command loop.
        bindings.onRise(gamepads.p1().rightBumper(), () -> {
            if (!controlsActive() || !captureVelocityComparison) return;
            skipNextVelocityComparisonSample = true;
        });

    }

    /** {@inheritDoc} */
    @Override
    public boolean onBackPressed() {
        if (!ready) {
            return false;
        }

        // Return to picker state only after making the selected device safe and restoring its mode.
        resetPowerControl();
        try {
            stopAndRestoreSelectedMotor();
        } finally {
            endVelocityCapture("BACK");
            clearSelectedMotor();
            resolveError = null;
        }

        picker.clearChoice();
        picker.refresh();
        if (motorName != null && !motorName.isEmpty()) {
            picker.setPreferredName(motorName);
        }

        return true;
    }


    /** {@inheritDoc} */
    @Override
    protected void onInitLoop(double dtSec) {
        if (!ready) {
            renderPicker();
            return;
        }
        // INIT may inspect a selected motor, but controls and actuation remain locked at zero.
        renderTelemetry(lastSubmittedPower, null);
    }

    /** {@inheritDoc} */
    @Override
    protected void onLoop(double dtSec) {
        if (!ready) {
            renderPicker();
            return;
        }
        updateAndRender();
    }

    /** {@inheritDoc} */
    @Override
    protected void onStart() {
        // START resets the shared clock. Require a fresh A press before any nonzero command.
        opModeStarted = true;
        endVelocityCapture("START_CLOCK_RESET");
        resetPowerControl();
        resetVelocityComparison();
        applyPowerAfterSample(0.0);
    }

    /** {@inheritDoc} */
    @Override
    protected void onStop() {
        opModeStarted = false;
        resetPowerControl();
        try {
            stopAndRestoreSelectedMotor();
        } finally {
            endVelocityCapture("OPMODE_STOP");
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------------

    private void tryResolveMotor(String name) {
        resetPowerControl();
        stopAndRestoreSelectedMotor();
        clearSelectedMotor();
        resolveError = null;
        endVelocityCapture("DEVICE_CHANGE");
        resetVelocityComparison();

        try {
            motorEx = ctx.hw.get(DcMotorEx.class, name);
            motor = motorEx;
        } catch (Exception ignored) {
            motorEx = null;
            motor = null;
            try {
                motor = ctx.hw.get(DcMotor.class, name);
            } catch (Exception ex2) {
                resolveError = ex2.getClass().getSimpleName() + ": " + ex2.getMessage();
                return;
            }
        }

        try {
            originalRunMode = motor.getMode();
            motor.setPower(0.0);
            lastSubmittedPower = 0.0;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                throw new IllegalStateException("motor did not enter RUN_WITHOUT_ENCODER");
            }

            selectedLynxModule = findSelectedLynxModule();
            resetPowerControl();
            ready = true;
            selectedMotorCycle = clock.cycle();
        } catch (RuntimeException ex) {
            RuntimeException cleanupFailure = null;
            try {
                stopAndRestoreSelectedMotor();
            } catch (RuntimeException cleanupEx) {
                cleanupFailure = cleanupEx;
            }
            clearSelectedMotor();
            if (cleanupFailure != null) {
                ex.addSuppressed(cleanupFailure);
                throw new IllegalStateException(
                        "Cannot prepare selected motor and cleanup did not complete",
                        ex);
            }
            resolveError = "Cannot prepare selected motor for safe open-loop testing: "
                    + describe(ex);
        }
    }

    private void updateAndRender() {
        power.updateFromAxis(clock, this::controlsActive);

        double nextCommand = power.applied();
        double priorCommand = lastSubmittedPower;
        boolean skipped = captureVelocityComparison && skipNextVelocityComparisonSample;

        if (skipped) {
            skipNextVelocityComparisonSample = false;
            velocitySample = null;
            velocityMeasurementStatus = "SKIPPED - baseline retained";
            velocityMeasurementError = null;
        } else if (captureVelocityComparison) {
            velocitySample = sampleVelocityComparison();
        } else {
            velocitySample = null;
        }

        // Associate the measurement with the command held before it, then issue the next command.
        double commandIssuedAfterSample = applyPowerAfterSample(nextCommand);

        if (captureVelocityComparison) {
            if (skipped) {
                logSkippedVelocityComparison(priorCommand, commandIssuedAfterSample);
            } else if (velocitySample != null) {
                logVelocityComparison(
                        velocitySample,
                        priorCommand,
                        commandIssuedAfterSample);
            } else {
                logVelocityComparisonError(priorCommand, commandIssuedAfterSample);
            }
        }

        renderTelemetry(lastSubmittedPower, velocitySample);
    }

    /** Issue the next motor command, falling back to a best-effort physical zero on failure. */
    private double applyPowerAfterSample(double command) {
        if (motor == null) {
            return Double.NaN;
        }

        try {
            motor.setPower(command);
            lastSubmittedPower = command;
            return command;
        } catch (RuntimeException ex) {
            appendVelocityMeasurementError("Power command failed: " + describe(ex));
            velocityMeasurementStatus = appendStatus(
                    velocityMeasurementStatus,
                    "POWER COMMAND FAILED; FALLBACK ZERO REQUESTED");
            resetPowerControl();

            try {
                motor.setPower(0.0);
                lastSubmittedPower = 0.0;
                return 0.0;
            } catch (RuntimeException zeroEx) {
                appendVelocityMeasurementError("Fallback zero failed: " + describe(zeroEx));
                velocityMeasurementStatus = appendStatus(
                        velocityMeasurementStatus,
                        "FALLBACK ZERO FAILED");
                lastSubmittedPower = Double.NaN;
                ex.addSuppressed(zeroEx);
                throw new IllegalStateException(
                        "Motor power command and fail-stop zero both failed",
                        ex);
            }
        }
    }

    /** Disarm the reusable tuner and remove any target that could leak to another selection. */
    private void resetPowerControl() {
        if (power.isEnabled()) {
            power.toggleEnabled();
        }
        power.zero();
    }

    /** Require a distinct post-selection input cycle before any test control can become active. */
    private boolean controlsActive() {
        return ready && opModeStarted && clock.cycle() != selectedMotorCycle;
    }

    /** Attempt both physical zero and mode restoration, then surface any cleanup failure. */
    private void stopAndRestoreSelectedMotor() {
        if (motor == null) {
            originalRunMode = null;
            lastSubmittedPower = 0.0;
            return;
        }

        RuntimeException cleanupFailure = null;
        try {
            motor.setPower(0.0);
            lastSubmittedPower = 0.0;
        } catch (RuntimeException zeroEx) {
            // Continue to mode restoration even if the zero write fails.
            lastSubmittedPower = Double.NaN;
            cleanupFailure = zeroEx;
        }

        try {
            if (originalRunMode != null) {
                motor.setMode(originalRunMode);
            }
        } catch (RuntimeException restoreEx) {
            if (cleanupFailure == null) {
                cleanupFailure = restoreEx;
            } else {
                cleanupFailure.addSuppressed(restoreEx);
            }
        } finally {
            originalRunMode = null;
        }

        if (cleanupFailure != null) {
            throw new IllegalStateException(
                    "Failed to command motor zero and/or restore its prior run mode",
                    cleanupFailure);
        }
    }

    /** Clear references only after the selected hardware has been made safe and restored. */
    private void clearSelectedMotor() {
        ready = false;
        motor = null;
        motorEx = null;
        selectedLynxModule = null;
        originalRunMode = null;
        selectedMotorCycle = Long.MIN_VALUE;
        lastSubmittedPower = 0.0;
        resetVelocityComparison();
    }

    /** Match the selected motor controller to its configured REV module without guessing. */
    private LynxModule findSelectedLynxModule() {
        final String controllerConnection;
        try {
            controllerConnection = motor.getController().getConnectionInfo();
        } catch (RuntimeException ex) {
            return null;
        }

        try {
            for (LynxModule module : ctx.hw.getAll(LynxModule.class)) {
                if (controllerConnection.equals(module.getConnectionInfo())) {
                    return module;
                }
            }
        } catch (RuntimeException ignored) {
            // An unmatched module makes the high-rate evidence gate ineligible, not the base tester.
        }
        return null;
    }

    private void renderPicker() {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        picker.render(t);

        t.addLine("");
        t.addLine("Highlighted motor is NOT chosen yet.");
        t.addLine("Use Dpad Up/Down to highlight, press A to choose.");

        if (resolveError != null) {
            t.addLine("");
            t.addLine("Resolve error:");
            t.addLine(resolveError);
        }

        t.update();
    }

    private void renderTelemetry(double appliedPower,
                                 MotorVelocityComparison.Sample comparison) {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        t.addLine("=== DcMotor Power Tester ===");
        t.addLine("Motor: " + motorName);
        t.addData("Enable [A]", power.isEnabled() ? "ON" : "OFF");
        t.addData("Invert [X]", power.isInverted() ? "ON" : "OFF");
        t.addData("Step [START]", "%s (%.2f)", power.isFine() ? "FINE" : "COARSE", power.step());
        t.addData("Power target [Dpad U/D | LeftStickY]", "%.2f", power.target());
        t.addData("Power applied", "%.2f", appliedPower);
        t.addData("Zero [B]", "target -> 0.00");

        if (!opModeStarted) {
            t.addData("Output lock", "INIT - start the OpMode, then press A to arm");
        }

        if (motor != null) {
            if (!captureVelocityComparison) {
                renderOrdinaryMotorMeasurements(t);
                if (velocityMeasurementError != null) {
                    t.addData("Tester warning", velocityMeasurementError);
                }
                if (opModeStarted) {
                    t.addData("Velocity comparison [Y]", "start Logcat capture");
                }
            } else {
                t.addLine("");
                t.addLine("--- Velocity comparison (measurement only) ---");
                t.addData("Capture [Y]", "ON - session " + velocityCaptureSession
                        + ", Logcat tag " + VELOCITY_COMPARISON_LOG_TAG);
                t.addData("Skip one sample [Right bumper]", "non-blocking");
                t.addData("Comparison status", velocityMeasurementStatus);
                t.addData("Coherent REV bulk snapshot", coherentHardwareSnapshot ? "YES" : "NO");
                t.addData("Original bulk mode preserved", bulkCachingModeRestored ? "YES" : "NO");
                t.addData("Row eligible for tachometer comparison",
                        isTachometerComparisonRowEligible(comparison) ? "YES" : "NO");

                if (comparison == null) {
                    t.addData("Measurement", "UNAVAILABLE");
                } else {
                    t.addData("Sample cycle / time", "%d / %.6f s",
                            comparison.cycle,
                            comparison.timeSec);
                    t.addData("Current position", "%d ticks", comparison.positionTicks);
                    t.addData("SDK direct velocity", directVelocityAvailable
                            ? String.format(Locale.US, "%.1f ticks/s",
                                    comparison.directVelocityTicksPerSec)
                            : "UNAVAILABLE");

                    if (comparison.derivedVelocityAvailable) {
                        t.addData("Position delta / interval", "%d ticks / %.6f s",
                                comparison.deltaPositionTicks,
                                comparison.sampleIntervalSec);
                        t.addData("Position-derived velocity", "%.1f ticks/s",
                                comparison.derivedVelocityTicksPerSec);
                        t.addData("Direct - derived", directVelocityAvailable
                                ? String.format(Locale.US, "%.1f ticks/s",
                                        comparison.directMinusDerivedTicksPerSec)
                                : "UNAVAILABLE");
                    } else {
                        t.addData("Position-derived velocity", "WARMING UP / UNAVAILABLE");
                    }
                }

                if (velocityMeasurementError != null) {
                    t.addData("Velocity measurement warning", velocityMeasurementError);
                }
            }

            try {
                t.addData("Motor.getPower()", "%.2f", motor.getPower());
            } catch (Exception ignored) {
            }
        }

        t.addLine("");
        t.addLine("BACK: return to the motor picker.");
        t.update();
    }

    /** Preserve the original tester's concise position/velocity display when capture is off. */
    private void renderOrdinaryMotorMeasurements(Telemetry telemetry) {
        try {
            telemetry.addData("Current position", "%d ticks", motor.getCurrentPosition());
        } catch (RuntimeException ignored) {
        }

        if (motorEx != null) {
            try {
                telemetry.addData("Velocity", "%.1f ticks/s", motorEx.getVelocity());
            } catch (RuntimeException ignored) {
            }
        }
    }

    /** Acquire one current-cycle hardware snapshot, then produce the velocity comparison. */
    private MotorVelocityComparison.Sample sampleVelocityComparison() {
        long cycle = clock.cycle();
        if (velocityHardwareSampleCycle == cycle) {
            return velocitySample;
        }

        // Cache acquisition attempts as well as successes so SDK reads occur at most once per cycle.
        velocityHardwareSampleCycle = cycle;
        velocitySample = null;
        velocityMeasurementError = null;
        directVelocityAvailable = false;
        coherentHardwareSnapshot = false;
        bulkCachingModeAtSample = "unavailable";
        bulkCachingModeRestored = false;

        int positionTicks;
        double directVelocityTicksPerSec = Double.NaN;
        LynxModule.BulkCachingMode originalBulkCachingMode = null;
        boolean temporarilyEnabledManualCaching = false;

        try {
            if (selectedLynxModule != null) {
                originalBulkCachingMode = selectedLynxModule.getBulkCachingMode();
                bulkCachingModeAtSample = String.valueOf(originalBulkCachingMode);

                LynxModule.BulkData bulkData = selectedLynxModule.getBulkData();
                if (bulkData == null || bulkData.isFake()) {
                    velocityMeasurementStatus = "REV BULK SNAPSHOT FAILED - baseline retained";
                    velocityMeasurementError = "REV hub returned placeholder bulk data.";
                    return null;
                }

                // OFF would make the public getters issue separate transactions. MANUAL makes both
                // getters consume the explicit bulk snapshot; switching back to OFF clears it.
                if (originalBulkCachingMode == LynxModule.BulkCachingMode.OFF) {
                    selectedLynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
                    temporarilyEnabledManualCaching = true;
                }

                positionTicks = motor.getCurrentPosition();
                directVelocityTicksPerSec = readDirectVelocityOnce();
                coherentHardwareSnapshot = true;
            } else {
                // Keep the tester usable for non-REV controllers, but make such evidence ineligible.
                positionTicks = motor.getCurrentPosition();
                directVelocityTicksPerSec = readDirectVelocityOnce();
                bulkCachingModeAtSample = "not-a-matched-REV-module";
            }
        } catch (RuntimeException ex) {
            velocityMeasurementStatus = "POSITION READ FAILED - baseline retained";
            velocityMeasurementError = "Snapshot/position read failed: " + describe(ex);
            return null;
        } finally {
            if (selectedLynxModule != null && temporarilyEnabledManualCaching) {
                try {
                    selectedLynxModule.setBulkCachingMode(originalBulkCachingMode);
                    bulkCachingModeRestored = true;
                } catch (RuntimeException ex) {
                    appendVelocityMeasurementError("Bulk-caching mode restore failed: "
                            + describe(ex));
                    velocityMeasurementStatus = appendStatus(
                            velocityMeasurementStatus,
                            "BULK MODE RESTORE FAILED");
                }
            } else if (selectedLynxModule != null) {
                // MANUAL/AUTO were not changed; the production setting remains intact.
                bulkCachingModeRestored = true;
            }
        }

        velocitySample = velocityComparison.sample(
                cycle,
                clock.nowSec(),
                positionTicks,
                directVelocityTicksPerSec);
        if (velocitySample.derivedVelocityAvailable) {
            velocityMeasurementStatus = directVelocityAvailable
                    ? "OK"
                    : "DERIVED OK; DIRECT UNAVAILABLE";
        } else {
            velocityMeasurementStatus = directVelocityAvailable
                    ? "WARMING UP / NO POSITIVE ACCEPTED INTERVAL"
                    : "WARMING UP; DIRECT UNAVAILABLE";
        }

        if (!coherentHardwareSnapshot) {
            velocityMeasurementStatus = appendStatus(
                    velocityMeasurementStatus,
                    "SNAPSHOT NOT COHERENT");
        }
        if (!bulkCachingModeRestored) {
            velocityMeasurementStatus = appendStatus(
                    velocityMeasurementStatus,
                    "BULK MODE NOT CONFIRMED RESTORED");
        }
        return velocitySample;
    }

    /** Read the optional direct SDK velocity exactly once for the current acquisition. */
    private double readDirectVelocityOnce() {
        if (motorEx == null) {
            appendVelocityMeasurementError(
                    "Selected device has no DcMotorEx direct-velocity API.");
            return Double.NaN;
        }

        final double directVelocityTicksPerSec;
        try {
            directVelocityTicksPerSec = motorEx.getVelocity();
        } catch (RuntimeException ex) {
            appendVelocityMeasurementError("Direct velocity read failed: " + describe(ex));
            return Double.NaN;
        }

        if (!Double.isFinite(directVelocityTicksPerSec)) {
            appendVelocityMeasurementError("Direct velocity returned a non-finite value.");
            return Double.NaN;
        }

        directVelocityAvailable = true;
        return directVelocityTicksPerSec;
    }

    /** Append a diagnostic warning without losing an earlier, independently useful warning. */
    private void appendVelocityMeasurementError(String warning) {
        if (warning == null || warning.isEmpty()) {
            return;
        }
        if (velocityMeasurementError == null || velocityMeasurementError.isEmpty()) {
            velocityMeasurementError = warning;
        } else {
            velocityMeasurementError += " | " + warning;
        }
    }

    /** Append a concise status clause without adding a framework-wide formatting abstraction. */
    private static String appendStatus(String status, String clause) {
        if (status == null || status.isEmpty()) {
            return clause;
        }
        return status + "; " + clause;
    }

    /** Format one exception for telemetry/logging without exposing multiline details. */
    private static String describe(RuntimeException ex) {
        String message = ex.getMessage();
        return ex.getClass().getSimpleName()
                + ((message == null || message.isEmpty()) ? "" : ": " + message);
    }

    /** Whether this selected REV motor port is a documented high-rate encoder port. */
    private boolean isHighRateRevPortEligible() {
        if (motor == null || selectedLynxModule == null) {
            return false;
        }
        try {
            int port = motor.getPortNumber();
            return port == 0 || port == 3;
        } catch (RuntimeException ex) {
            return false;
        }
    }

    /** Whether a row contains every evidence prerequisite, not whether either reading is correct. */
    private boolean isTachometerComparisonRowEligible(MotorVelocityComparison.Sample sample) {
        return sample != null
                && sample.derivedVelocityAvailable
                && directVelocityAvailable
                && coherentHardwareSnapshot
                && bulkCachingModeRestored
                && isHighRateRevPortEligible();
    }

    /** Clear diagnostic sampling state without changing motor hardware state. */
    private void resetVelocityComparison() {
        velocityComparison.reset();
        velocitySample = null;
        skipNextVelocityComparisonSample = false;
        velocityHardwareSampleCycle = Long.MIN_VALUE;
        directVelocityAvailable = false;
        coherentHardwareSnapshot = false;
        bulkCachingModeAtSample = "unavailable";
        bulkCachingModeRestored = false;
        velocityMeasurementStatus = "WARMING UP";
        velocityMeasurementError = null;
    }

    /** Begin a new opt-in Logcat capture session. */
    private void beginVelocityCapture() {
        captureVelocityComparison = true;
        startVelocityCaptureSession();
    }

    /** Reset measurement history and emit metadata for a fresh clock-consistent session. */
    private void startVelocityCaptureSession() {
        velocityCaptureSession++;
        resetVelocityComparison();
        logVelocityComparisonMetadata();
        logVelocityComparisonHeader();
    }

    /** Close an active capture, if any, and clear diagnostic history. */
    private void endVelocityCapture(String reason) {
        if (captureVelocityComparison) {
            logVelocityCaptureEnd(reason);
        }
        captureVelocityComparison = false;
        resetVelocityComparison();
    }

    /** Emit selected-port metadata while explicitly distinguishing config from physical identity. */
    private void logVelocityComparisonMetadata() {
        int port = -1;
        String connection = "unavailable";
        String controllerClass = "unavailable";
        String controllerConnection = "unavailable";
        String currentMode = "unavailable";
        String direction = "unavailable";
        String moduleAddress = "unavailable";
        String moduleSerial = "unavailable";
        String moduleFirmware = "unavailable";
        String bulkCachingMode = "unavailable";
        double configuredTicksPerRev = Double.NaN;
        double configuredMaxRpm = Double.NaN;

        try {
            port = motor.getPortNumber();
        } catch (RuntimeException ignored) {
        }
        try {
            connection = motor.getConnectionInfo();
        } catch (RuntimeException ignored) {
        }
        try {
            controllerClass = motor.getController().getClass().getName();
            controllerConnection = motor.getController().getConnectionInfo();
        } catch (RuntimeException ignored) {
        }
        try {
            currentMode = String.valueOf(motor.getMode());
        } catch (RuntimeException ignored) {
        }
        try {
            direction = String.valueOf(motor.getDirection());
        } catch (RuntimeException ignored) {
        }
        if (selectedLynxModule != null) {
            try {
                moduleAddress = String.valueOf(selectedLynxModule.getModuleAddress());
            } catch (RuntimeException ignored) {
            }
            try {
                moduleSerial = String.valueOf(selectedLynxModule.getSerialNumber());
            } catch (RuntimeException ignored) {
            }
            try {
                moduleFirmware = selectedLynxModule.getFirmwareVersionString();
            } catch (RuntimeException ignored) {
            }
            try {
                bulkCachingMode = String.valueOf(selectedLynxModule.getBulkCachingMode());
            } catch (RuntimeException ignored) {
            }
        }
        try {
            MotorConfigurationType motorType = motor.getMotorType();
            if (motorType != null) {
                configuredTicksPerRev = motorType.getTicksPerRev();
                configuredMaxRpm = motorType.getMaxRPM();
            }
        } catch (RuntimeException ignored) {
        }

        RobotLog.ii(
                VELOCITY_COMPARISON_LOG_TAG,
                String.format(
                        Locale.US,
                        "ENCODER_VELOCITY_META,session=%d,motorName=%s,connection=%s,port=%d,"
                                + "highRateRevPortEligible=%s,controllerClass=%s,"
                                + "controllerConnection=%s,moduleMatched=%s,moduleAddress=%s,"
                                + "moduleSerial=%s,moduleFirmware=%s,bulkCachingModeBeforeCapture=%s,"
                                + "originalRunMode=%s,currentRunMode=%s,direction=%s,"
                                + "configuredTicksPerRev=%.3f,configuredMaxRpm=%.3f,"
                                + "configuredMotorTypeIsNotPhysicalEncoderIdentity=true,"
                                + "coherentSnapshotRecordedPerDataRow=true",
                        velocityCaptureSession,
                        csvSafe(motorName),
                        csvSafe(connection),
                        port,
                        String.valueOf(isHighRateRevPortEligible()),
                        csvSafe(controllerClass),
                        csvSafe(controllerConnection),
                        String.valueOf(selectedLynxModule != null),
                        csvSafe(moduleAddress),
                        csvSafe(moduleSerial),
                        csvSafe(moduleFirmware),
                        csvSafe(bulkCachingMode),
                        csvSafe(String.valueOf(originalRunMode)),
                        csvSafe(currentMode),
                        csvSafe(direction),
                        configuredTicksPerRev,
                        configuredMaxRpm));
    }

    /** Emit the stable column order used by each {@code ENCODER_VELOCITY_DATA} row. */
    private void logVelocityComparisonHeader() {
        RobotLog.ii(
                VELOCITY_COMPARISON_LOG_TAG,
                "ENCODER_VELOCITY_HEADER,session,motorName,cycle,timeSec,loopDtSec,enabled,targetPower,"
                        + "priorCommand,commandIssuedAfterSample,positionTicks,deltaTicks,sampleIntervalSec,"
                        + "directTicksPerSec,derivedTicksPerSec,"
                        + "directMinusDerivedTicksPerSec,directAvailable,derivedAvailable,"
                        + "coherentHardwareSnapshot,bulkCachingMode,bulkCachingModeRestored,"
                        + "highRateRevPortEligible,tachometerComparisonRowEligible,status,warning");
    }

    /** Emit one locale-stable measurement row for the current loop. */
    private void logVelocityComparison(MotorVelocityComparison.Sample sample,
                                       double priorCommand,
                                       double commandIssuedAfterSample) {
        RobotLog.ii(
                VELOCITY_COMPARISON_LOG_TAG,
                String.format(
                        Locale.US,
                        "ENCODER_VELOCITY_DATA,%d,%s,%d,%.9f,%.9f,%s,%.4f,%.4f,%.4f,%d,%d,%.9f,"
                                + "%.3f,%.3f,%.3f,%s,%s,%s,%s,%s,%s,%s,%s,%s",
                        velocityCaptureSession,
                        csvSafe(motorName),
                        sample.cycle,
                        sample.timeSec,
                        clock.dtSec(),
                        String.valueOf(power.isEnabled()),
                        power.target(),
                        priorCommand,
                        commandIssuedAfterSample,
                        sample.positionTicks,
                        sample.deltaPositionTicks,
                        sample.sampleIntervalSec,
                        sample.directVelocityTicksPerSec,
                        sample.derivedVelocityTicksPerSec,
                        sample.directMinusDerivedTicksPerSec,
                        String.valueOf(directVelocityAvailable),
                        String.valueOf(sample.derivedVelocityAvailable),
                        String.valueOf(coherentHardwareSnapshot),
                        csvSafe(bulkCachingModeAtSample),
                        String.valueOf(bulkCachingModeRestored),
                        String.valueOf(isHighRateRevPortEligible()),
                        String.valueOf(isTachometerComparisonRowEligible(sample)),
                        csvSafe(velocityMeasurementStatus),
                        csvSafe(velocityMeasurementError == null
                                ? "none"
                                : velocityMeasurementError)));
    }

    /** Record a deliberate non-blocking sample gap. */
    private void logSkippedVelocityComparison(double priorCommand,
                                              double commandIssuedAfterSample) {
        RobotLog.ii(
                VELOCITY_COMPARISON_LOG_TAG,
                String.format(
                        Locale.US,
                        "ENCODER_VELOCITY_SKIPPED,session=%d,motorName=%s,cycle=%d,timeSec=%.9f,"
                                + "loopDtSec=%.9f,priorCommand=%.4f,commandIssuedAfterSample=%.4f,"
                                + "reason=one-shot-nonblocking-skip,status=%s,warning=%s",
                        velocityCaptureSession,
                        csvSafe(motorName),
                        clock.cycle(),
                        clock.nowSec(),
                        clock.dtSec(),
                        priorCommand,
                        commandIssuedAfterSample,
                        csvSafe(velocityMeasurementStatus),
                        csvSafe(velocityMeasurementError == null
                                ? "none"
                                : velocityMeasurementError)));
    }

    /** Emit one explicit failure row so a missing sample cannot disappear from a capture. */
    private void logVelocityComparisonError(double priorCommand,
                                            double commandIssuedAfterSample) {
        RobotLog.ii(
                VELOCITY_COMPARISON_LOG_TAG,
                String.format(
                        Locale.US,
                        "ENCODER_VELOCITY_ERROR,session=%d,motorName=%s,cycle=%d,timeSec=%.9f,"
                                + "loopDtSec=%.9f,priorCommand=%.4f,commandIssuedAfterSample=%.4f,"
                                + "coherentHardwareSnapshot=%s,bulkCachingMode=%s,"
                                + "bulkCachingModeRestored=%s,status=%s,error=%s",
                        velocityCaptureSession,
                        csvSafe(motorName),
                        clock.cycle(),
                        clock.nowSec(),
                        clock.dtSec(),
                        priorCommand,
                        commandIssuedAfterSample,
                        String.valueOf(coherentHardwareSnapshot),
                        csvSafe(bulkCachingModeAtSample),
                        String.valueOf(bulkCachingModeRestored),
                        csvSafe(velocityMeasurementStatus),
                        csvSafe(velocityMeasurementError)));
    }

    /** Emit an explicit capture boundary so exported sessions are not accidentally combined. */
    private void logVelocityCaptureEnd(String reason) {
        RobotLog.ii(
                VELOCITY_COMPARISON_LOG_TAG,
                String.format(
                        Locale.US,
                        "ENCODER_VELOCITY_END,%d,%d,%.9f,%s",
                        velocityCaptureSession,
                        clock.cycle(),
                        clock.nowSec(),
                        csvSafe(reason)));
    }

    /** Return a one-field-safe diagnostic token without adding a general CSV abstraction. */
    private static String csvSafe(String value) {
        if (value == null || value.isEmpty()) {
            return "unknown";
        }
        return value.replace(',', '_').replace('\n', ' ').replace('\r', ' ');
    }
}
