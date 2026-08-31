package edu.ftcsushi.fw.tools.tester.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.tools.tester.BaseTeleOpTester;
import edu.ftcsushi.fw.tools.tester.StandardTesters;
import edu.ftcsushi.fw.ftc.ui.HardwareNamePicker;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.fw.tools.tester.ui.IntTuner;
import edu.ftcsushi.fw.tools.tester.ui.ScalarTuner;

/**
 * Generic tester for a configured {@link DcMotor}/{@link DcMotorEx} that runs the motor
 * to a target encoder position using {@link DcMotor.RunMode#RUN_TO_POSITION}.
 *
 * <h2>Selection</h2>
 * If constructed without a motor name (or the preferred name cannot be resolved), shows a picker
 * listing configured motors.
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>PICKER (no motor chosen yet)</b>: Dpad Up/Down highlight, A choose, X refresh</li>
 *   <li><b>RUN (motor selected)</b>:
 *     <ul>
 *       <li><b>A</b>: enable/disable (RUN_TO_POSITION active)</li>
 *       <li><b>X</b>: toggle motor direction while disabled (FORWARD/REVERSE)</li>
 *       <li><b>START</b>: toggle fine/coarse target and power steps</li>
 *       <li><b>Dpad Up/Down</b>: step target ticks from the selected motor's current position</li>
 *       <li><b>Dpad Right/Left</b>: adjust power</li>
 *       <li><b>B</b>: stop (disable + power=0)</li>
 *       <li><b>BACK</b>: return to picker (change motor)</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p><b>Advanced diagnostic:</b> ordinary direction and safe-endpoint setup should enter the one
 * device-first path from {@link StandardTesters#createActuatorBringUp()}. Use this class directly
 * only to exercise an isolated motor's SDK {@code RUN_TO_POSITION} behavior.</p>
 *
 * <p><b>Notes:</b> RUN_TO_POSITION requires a meaningful encoder reading. If your motor/encoder
 * is not configured or supported, the motor may not move as expected.</p>
 *
 * <p><b>Lifecycle:</b> selection and INIT are observation-only. OpMode START prepares the
 * selected motor. Each held control must be released before its next edge can act; B remains a
 * level-sensitive priority stop throughout RUN. The target is
 * anchored to the current encoder reading when the motor is selected and again when OpMode START
 * prepares it; this diagnostic never resets the encoder.</p>
 *
 * <p><b>Safety:</b> the initial movement power is deliberately conservative, power is set to 0,
 * and original motor settings are restored on stop.</p>
 */
public final class DcMotorPositionTester extends BaseTeleOpTester {

    // Preserve any valid FTC encoder reading. Conservative fixed step sizes, not a hidden absolute
    // window, bound what one deliberate button press can change.
    private static final int TARGET_MIN_TICKS = Integer.MIN_VALUE;
    private static final int TARGET_MAX_TICKS = Integer.MAX_VALUE;

    // Step sizes for target ticks
    private static final int TARGET_FINE_STEP_TICKS = 25;
    private static final int TARGET_COARSE_STEP_TICKS = 100;

    private final String preferredName;

    private HardwareNamePicker picker;

    private String motorName = null;
    private DcMotor motor = null;
    private DcMotorEx motorEx = null;

    private boolean ready = false;
    private boolean opModeStarted = false;
    private boolean motorPreparationStarted = false;
    private boolean motorPrepared = false;
    private boolean restorationAttempted = false;
    private RuntimeException retainedRestorationFailure = null;
    private long selectedMotorCycle = Long.MIN_VALUE;
    private String resolveError = null;
    private String testerError = null;

    // Tuners
    private final IntTuner targetTicks =
            new IntTuner("TargetTicks",
                    TARGET_MIN_TICKS, TARGET_MAX_TICKS,
                    TARGET_FINE_STEP_TICKS, TARGET_COARSE_STEP_TICKS,
                    0);

    // Power (0..1). Enable is handled by targetTicks (RUN_TO_POSITION enabled/disabled).
    private final ScalarTuner power =
            new ScalarTuner("Power", 0.0, 1.0, 0.02, 0.10, 0.10)
                    .setEnableSupported(false)
                    .setInvertSupported(false);

    // Snapshot of original motor settings for restoration
    private DcMotor.RunMode origMode = null;
    private DcMotor.Direction origDir = null;
    private DcMotor.ZeroPowerBehavior origZpb = null;

    /**
     * Create a DC motor position tester with no preferred device name.
     *
     * <p>A picker menu is shown so you can choose a configured motor.</p>
     */
    public DcMotorPositionTester() {
        this(null);
    }

    /**
     * Create a DC motor position tester with a preferred device name.
     *
     * <p>If {@code motorName} is null/blank or cannot be resolved, the tester will fall back to the picker menu.</p>
     *
     * @param motorName configured motor name in the FTC Robot Configuration (nullable)
     */
    public DcMotorPositionTester(String motorName) {
        this.preferredName = motorName;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String name() {
        return "DcMotor Position Tester";
    }

    private static DcMotor.Direction safeGetDir(DcMotor m) {
        try {
            return m.getDirection();
        } catch (Exception ignored) {
            return null; }
    }

    /** {@inheritDoc} */
    @Override
    protected void onInit() {
        opModeStarted = false;
        picker = new HardwareNamePicker(
                ctx.hw,
                DcMotor.class,
                "Select Motor",
                "Dpad: highlight | A: choose | X: refresh"
        );
        picker.refresh();

        // Prefer name passed in (RobotConfig), but fall back to picker if it fails.
        if (preferredName != null && !preferredName.trim().isEmpty()) {
            motorName = preferredName.trim();
            tryResolveMotor(motorName);
        }

        // Picker controls active only while not ready.
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

        Bindings.ControlContext liveControls = bindings.contextWhen(
                BooleanSource.of(this::controlsActive),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL
        );

        // A toggles RUN_TO_POSITION enable (only when ready)
        liveControls.onRise(gamepads.p1().a(), () -> {
            targetTicks.toggleEnabled();

            // If disabling, force motor quiet immediately.
            if (!targetTicks.isEnabled()) {
                safeDisableMotor();
            }
        });

        // X toggles motor direction (hardware-level)
        liveControls.onRise(gamepads.p1().x(), () -> {
            if (!targetTicks.isEnabled()) {
                toggleDirection();
            }
        });

        // START toggles fine/coarse for BOTH tuners (so UI feels consistent)
        liveControls.onRise(gamepads.p1().start(), () -> {
            targetTicks.toggleFine();
            power.toggleFine();
        });

        // Target inc/dec (dpad up/down) — only when ready
        liveControls.onRise(gamepads.p1().dpadUp(), targetTicks::inc);
        liveControls.onRise(gamepads.p1().dpadDown(), targetTicks::dec);

        // Power inc/dec (dpad right/left) — only when ready
        liveControls.onRise(gamepads.p1().dpadRight(), power::inc);
        liveControls.onRise(gamepads.p1().dpadLeft(), power::dec);

        // Hard stop
        liveControls.onRise(gamepads.p1().b(), () -> {
            // Disable if enabled
            if (targetTicks.isEnabled()) {
                targetTicks.toggleEnabled();
            }
            // Set power target to 0 for safety
            power.setTarget(0.0);

            safeDisableMotor();
        });
    }

    /** {@inheritDoc} */
    @Override
    public boolean onBackPressed() {
        if (!ready) {
            return false;
        }

        // Ensure motor is quiet and restore original settings before returning to the picker.
        if (targetTicks.isEnabled()) {
            targetTicks.toggleEnabled();
        }
        power.setTarget(0.0);

        if (motorPreparationStarted) {
            restoreOriginalSettings();
        }

        clearSelectedMotor();

        picker.clearChoice();
        picker.refresh();
        if (motorName != null && !motorName.isEmpty()) {
            picker.setPreferredName(motorName);
        }

        return true;
    }

    // ---------------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------------

    /** {@inheritDoc} */
    @Override
    protected void onInitLoop(double dtSec) {
        if (!ready) {
            renderPicker();
            return;
        }
        renderTelemetry();
    }

    /** {@inheritDoc} */
    @Override
    protected void onLoop(double dtSec) {
        if (!ready) {
            renderPicker();
            return;
        }
        if (gamepads.p1().b().getAsBoolean(clock)) {
            if (targetTicks.isEnabled()) {
                targetTicks.toggleEnabled();
            }
            power.setTarget(0.0);
            safeDisableMotor();
            renderTelemetry();
            return;
        }
        updateAndRender(dtSec);
    }

    /** {@inheritDoc} */
    @Override
    protected void onStart() {
        opModeStarted = true;
        if (ready) {
            prepareSelectedMotor();
        }
    }

    private void applyRunToPosition() {
        if (motor == null) return;

        try {
            motor.setTargetPosition(targetTicks.applied());
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power.applied());
            testerError = null;
        } catch (RuntimeException commandFailure) {
            if (targetTicks.isEnabled()) {
                targetTicks.toggleEnabled();
            }
            testerError = "RUN_TO_POSITION command failed: " + describe(commandFailure);
            try {
                safeDisableMotor();
            } catch (RuntimeException cleanupFailure) {
                commandFailure.addSuppressed(cleanupFailure);
                throw new IllegalStateException(
                        "Position command and fail-stop cleanup both failed",
                        commandFailure);
            }
        }
    }

    /** {@inheritDoc} */
    @Override
    protected void onStop() {
        opModeStarted = false;
        restoreOriginalSettings();
    }

    private void tryResolveMotor(String name) {
        resolveError = null;

        try {
            motorEx = ctx.hw.get(DcMotorEx.class, name);
            motor = motorEx;
        } catch (Exception ex) {
            motorEx = null;
            motor = null;
            try {
                motor = ctx.hw.get(DcMotor.class, name);
            } catch (Exception ex2) {
                resolveError = ex2.getClass().getSimpleName() + ": " + ex2.getMessage();
                ready = false;
                return;
            }
        }

        try {
            // Selection is observation-only. Anchor the advanced target to the current reading so
            // the first deliberate enable cannot send the mechanism toward an invented zero.
            targetTicks.setTarget(motor.getCurrentPosition());
        } catch (RuntimeException ex) {
            resolveError = "Cannot read selected motor position: " + describe(ex);
            motor = null;
            motorEx = null;
            ready = false;
            return;
        }
        if (targetTicks.isEnabled()) {
            targetTicks.toggleEnabled();
        }
        power.setTarget(0.10);

        ready = true;
        testerError = null;
        selectedMotorCycle = clock.cycle();
        if (opModeStarted) {
            prepareSelectedMotor();
        }
    }

    private void updateAndRender(double dtSec) {
        if (targetTicks.isEnabled()) {
            applyRunToPosition();
        } else {
            try {
                motor.setPower(0.0);
            } catch (RuntimeException ex) {
                testerError = "Zero command failed: " + describe(ex);
                throw ex;
            }
        }

        renderTelemetry();
    }

    private void safeDisableMotor() {
        if (motor == null) return;
        RuntimeException failure = null;
        try {
            motor.setPower(0.0);
        } catch (RuntimeException ex) {
            failure = ex;
        }
        try {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (RuntimeException ex) {
            failure = combine(failure, ex);
        }
        if (failure != null) {
            throw new IllegalStateException("Failed to disable position motor safely", failure);
        }
    }

    private void toggleDirection() {
        if (motor == null) return;
        safeDisableMotor();
        DcMotor.Direction d = motor.getDirection();
        motor.setDirection(d == DcMotor.Direction.FORWARD
                ? DcMotor.Direction.REVERSE
                : DcMotor.Direction.FORWARD);
        targetTicks.setTarget(motor.getCurrentPosition());
    }

    private void restoreOriginalSettings() {
        if (motor == null || !motorPreparationStarted) return;
        if (restorationAttempted) {
            if (retainedRestorationFailure != null) throw retainedRestorationFailure;
            return;
        }
        restorationAttempted = true;
        RuntimeException failure = null;
        try {
            motor.setPower(0.0);
        } catch (RuntimeException ex) {
            failure = ex;
        }
        try {
            if (origDir != null) motor.setDirection(origDir);
        } catch (RuntimeException ex) {
            failure = combine(failure, ex);
        }
        try {
            if (origZpb != null) motor.setZeroPowerBehavior(origZpb);
        } catch (RuntimeException ex) {
            failure = combine(failure, ex);
        }
        try {
            if (origMode != null) motor.setMode(origMode);
        } catch (RuntimeException ex) {
            failure = combine(failure, ex);
        }
        if (failure != null) {
            retainedRestorationFailure = new IllegalStateException(
                    "Failed to command motor zero and/or restore its prior settings",
                    failure);
            throw retainedRestorationFailure;
        }
        motorPreparationStarted = false;
        motorPrepared = false;
        origMode = null;
        origDir = null;
        origZpb = null;
    }

    private boolean controlsActive() {
        return ready
                && opModeStarted
                && motorPrepared
                && !gamepads.p1().b().getAsBoolean(clock)
                && clock.cycle() != selectedMotorCycle;
    }

    private void prepareSelectedMotor() {
        if (motor == null || motorPrepared) return;
        origMode = motor.getMode();
        if (origMode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            origMode = null;
            throw new IllegalStateException(
                    "Selected motor is in STOP_AND_RESET_ENCODER. Put it in a normal run mode "
                            + "before starting motor-position diagnostics; this tester never "
                            + "resets or restores encoder-reset mode.");
        }
        origDir = motor.getDirection();
        origZpb = motor.getZeroPowerBehavior();
        restorationAttempted = false;
        retainedRestorationFailure = null;
        motorPreparationStarted = true;
        try {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            targetTicks.setTarget(motor.getCurrentPosition());
            motorPrepared = true;
        } catch (RuntimeException preparationFailure) {
            try {
                restoreOriginalSettings();
            } catch (RuntimeException cleanupFailure) {
                preparationFailure.addSuppressed(cleanupFailure);
            }
            throw new IllegalStateException(
                    "Cannot prepare selected motor for position diagnostics",
                    preparationFailure);
        }
    }

    private void clearSelectedMotor() {
        ready = false;
        motor = null;
        motorEx = null;
        resolveError = null;
        testerError = null;
        selectedMotorCycle = Long.MIN_VALUE;
        motorPreparationStarted = false;
        motorPrepared = false;
        restorationAttempted = false;
        retainedRestorationFailure = null;
        origMode = null;
        origDir = null;
        origZpb = null;
    }

    private static RuntimeException combine(RuntimeException first, RuntimeException next) {
        if (first == null) return next;
        if (first != next) first.addSuppressed(next);
        return first;
    }

    private static String describe(RuntimeException ex) {
        String message = ex.getMessage();
        return ex.getClass().getSimpleName()
                + ((message == null || message.trim().isEmpty()) ? "" : ": " + message);
    }

    private void renderPicker() {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        picker.render(t);

        if (motorName != null && !motorName.isEmpty()) {
            t.addLine("");
            t.addLine("Chosen: " + motorName);
        }

        if (resolveError != null) {
            t.addLine("");
            t.addLine("Resolve error:");
            t.addLine(resolveError);
        }

        t.update();
    }

    private void renderTelemetry() {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        t.addLine("=== DcMotor Position Tester ===");
        t.addLine("Motor: " + motorName);
        t.addData("Enable [A]", targetTicks.isEnabled() ? "RUN_TO_POSITION ON" : "OFF");
        DcMotor.Direction observedDirection = safeGetDir(motor);
        t.addData("Direction [X]",
                observedDirection == null ? "unavailable" : observedDirection.name());
        t.addData("Step [gamepad START]", "%s (target=%d ticks, power=%.2f)",
                targetTicks.isFine() ? "FINE" : "COARSE",
                targetTicks.step(),
                power.step());
        t.addData("Staged target [Dpad U/D]", "%d ticks", targetTicks.target());
        if (targetTicks.isEnabled()) {
            t.addData("Submitted target this cycle", "%d ticks", targetTicks.applied());
        } else if (!opModeStarted) {
            t.addData("Target command", "INIT - no command submitted");
        } else {
            t.addData("Target command", "not active; motor zero submitted");
        }
        t.addData("Power [Dpad L/R]", "%.2f", power.target());
        t.addData("Stop [B]", "disable + power 0");

        if (!opModeStarted) {
            t.addData("Output lock", "INIT - start the OpMode, release controls, then press A");
        }
        if (testerError != null) {
            t.addData("Tester error", testerError);
        }

        Integer cur = null;
        Boolean busy = null;
        DcMotor.RunMode mode = null;

        try {
            cur = motor.getCurrentPosition();
        } catch (Exception ignored) {
        }
        try {
            busy = motor.isBusy();
        } catch (Exception ignored) {
        }
        try {
            mode = motor.getMode();
        } catch (Exception ignored) {
        }

        t.addLine("");
        if (cur == null) {
            t.addData("Current position", "unavailable");
        } else {
            t.addData("Current position", "%d", cur);
        }
        if (targetTicks.isEnabled() && cur != null) {
            long positionError = ((long) targetTicks.applied()) - ((long) cur);
            t.addData("Position error", "%d ticks", positionError);
        } else {
            t.addData("Position error", "unavailable (target inactive or position unreadable)");
        }
        t.addData("Mode", mode == null ? "unavailable" : mode.name());
        t.addData("Busy", busy == null ? "unavailable" : (busy ? "YES" : "NO"));

        if (motorEx != null) {
            try {
                double velocity = motorEx.getVelocity();
                if (Double.isFinite(velocity)) {
                    t.addData("Velocity", "%.1f ticks/s", velocity);
                } else {
                    t.addData("Velocity", "unavailable (non-finite reading)");
                }
            } catch (Exception failure) {
                t.addData("Velocity", "unavailable");
            }
        }

        t.addLine("");
        t.addLine("BACK: return to the motor picker.");
        t.update();
    }
}
