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

/**
 * Generic tester for a configured {@link DcMotorEx} that commands a target velocity using
 * {@link DcMotorEx#setVelocity(double)}.
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
 *       <li><b>A</b>: enable/disable velocity control</li>
 *       <li><b>X</b>: toggle motor direction while disabled (FORWARD/REVERSE)</li>
 *       <li><b>START</b>: toggle fine/coarse target step</li>
 *       <li><b>Dpad Up/Down</b>: step target velocity</li>
 *       <li><b>Y</b>: set target velocity to 0 (does not disable)</li>
 *       <li><b>B</b>: stop (disable + target=0)</li>
 *       <li><b>BACK</b>: return to picker (change motor)</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p><b>Advanced diagnostic:</b> ordinary direction and safe-endpoint setup should enter the one
 * device-first path from {@link StandardTesters#createActuatorBringUp()}. Use this class directly
 * only to exercise an isolated {@link DcMotorEx} velocity channel.</p>
 *
 * <p><b>Lifecycle:</b> selection and INIT are observation-only. OpMode START prepares the
 * selected motor. Each held control must be released before its next edge can act; B remains a
 * level-sensitive priority stop throughout RUN.</p>
 *
 * <p><b>Safety:</b> motor is commanded to 0 on stop and original settings are restored.</p>
 */
public final class DcMotorVelocityTester extends BaseTeleOpTester {

    // Keep range wide so it works for many motors/encoders. Adjust if you want.
    private static final int VEL_MIN_TPS = -25000; // ticks/sec
    private static final int VEL_MAX_TPS = +25000; // ticks/sec

    private static final int VEL_FINE_STEP_TPS = 50;
    private static final int VEL_COARSE_STEP_TPS = 250;

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

    private final IntTuner targetVelTps =
            new IntTuner("TargetVel(tps)",
                    VEL_MIN_TPS, VEL_MAX_TPS,
                    VEL_FINE_STEP_TPS, VEL_COARSE_STEP_TPS,
                    0);

    // Snapshot of original motor settings for restoration
    private DcMotor.RunMode origMode = null;
    private DcMotor.Direction origDir = null;
    private DcMotor.ZeroPowerBehavior origZpb = null;

    /**
     * Create a DC motor velocity tester with no preferred device name.
     *
     * <p>A picker menu is shown so you can choose a configured motor.</p>
     */
    public DcMotorVelocityTester() {
        this(null);
    }

    /**
     * Create a DC motor velocity tester with a preferred device name.
     *
     * <p>If {@code motorName} is null/blank or cannot be resolved, the tester will fall back to the picker menu.</p>
     *
     * @param motorName configured motor name in the FTC Robot Configuration (nullable)
     */
    public DcMotorVelocityTester(String motorName) {
        this.preferredName = motorName;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String name() {
        return "DcMotor Velocity Tester";
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

        // Enable/disable is useful here.
        targetVelTps.setEnableSupported(true);

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

        // A: toggle enabled with side-effects.
        liveControls.onRise(gamepads.p1().a(), () -> {
            targetVelTps.toggleEnabled();

            if (!targetVelTps.isEnabled()) {
                stopMotorNow();
            }
        });

        // X: toggle motor direction.
        liveControls.onRise(gamepads.p1().x(), () -> {
            if (!targetVelTps.isEnabled()) {
                toggleDirection();
            }
        });

        // START: fine/coarse
        liveControls.onRise(gamepads.p1().start(), targetVelTps::toggleFine);

        // Dpad up/down: velocity steps (only when ready)
        liveControls.onRise(gamepads.p1().dpadUp(), targetVelTps::inc);
        liveControls.onRise(gamepads.p1().dpadDown(), targetVelTps::dec);

        // Y: zero target velocity (keep enabled state as-is)
        liveControls.onRise(gamepads.p1().y(), () -> targetVelTps.setTarget(0));

        // B: stop (disable + target=0)
        liveControls.onRise(gamepads.p1().b(), () -> {
            targetVelTps.setTarget(0);
            if (targetVelTps.isEnabled()) {
                targetVelTps.toggleEnabled();
            }
            stopMotorNow();
        });
    }

    /** {@inheritDoc} */
    @Override
    public boolean onBackPressed() {
        if (!ready) {
            return false;
        }

        // Ensure motor is quiet and restore original settings before returning to the picker.
        if (targetVelTps.isEnabled()) {
            targetVelTps.toggleEnabled();
        }
        targetVelTps.setTarget(0);

        restoreOriginalSettings();

        clearSelectedMotor();

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
            targetVelTps.setTarget(0);
            if (targetVelTps.isEnabled()) {
                targetVelTps.toggleEnabled();
            }
            stopMotorNow();
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

    /** {@inheritDoc} */
    @Override
    protected void onStop() {
        opModeStarted = false;
        restoreOriginalSettings();
    }

    // ---------------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------------

    private void tryResolveMotor(String name) {
        resolveError = null;

        // We REQUIRE DcMotorEx for velocity control.
        try {
            motorEx = ctx.hw.get(DcMotorEx.class, name);
            motor = motorEx;
        } catch (Exception ex) {
            motorEx = null;
            motor = null;
            ready = false;
            resolveError = "Selected motor does not support DcMotorEx: "
                    + ex.getClass().getSimpleName() + ": " + ex.getMessage();
            return;
        }

        // Default: disabled until user presses A
        if (targetVelTps.isEnabled()) {
            targetVelTps.toggleEnabled();
        }
        targetVelTps.setTarget(0);

        ready = true;
        testerError = null;
        selectedMotorCycle = clock.cycle();
        if (opModeStarted) {
            prepareSelectedMotor();
        }
    }

    private void updateAndRender(double dtSec) {
        if (targetVelTps.isEnabled()) {
            applyVelocity(targetVelTps.applied());
        } else {
            // keep motor quiet while disabled
            stopMotorNow();
        }

        renderTelemetry();
    }

    private void applyVelocity(int tps) {
        if (motorEx == null) return;
        try {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorEx.setVelocity(tps);
            testerError = null;
        } catch (RuntimeException commandFailure) {
            if (targetVelTps.isEnabled()) {
                targetVelTps.toggleEnabled();
            }
            testerError = "Velocity command failed: " + describe(commandFailure);
            try {
                stopMotorNow();
            } catch (RuntimeException cleanupFailure) {
                commandFailure.addSuppressed(cleanupFailure);
                throw new IllegalStateException(
                        "Velocity command and fail-stop cleanup both failed",
                        commandFailure);
            }
        }
    }

    private void stopMotorNow() {
        if (motor == null) return;
        RuntimeException failure = null;
        try {
            if (motorEx != null) motorEx.setVelocity(0);
        } catch (RuntimeException ex) {
            failure = ex;
        }
        try {
            motor.setPower(0.0);
        } catch (RuntimeException ex) {
            failure = combine(failure, ex);
        }
        if (failure != null) {
            throw new IllegalStateException("Failed to stop velocity motor safely", failure);
        }
    }

    private void toggleDirection() {
        if (motor == null) return;
        targetVelTps.setTarget(0);
        stopMotorNow();
        DcMotor.Direction d = motor.getDirection();
        motor.setDirection(d == DcMotor.Direction.FORWARD
                ? DcMotor.Direction.REVERSE
                : DcMotor.Direction.FORWARD);
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
            if (motorEx != null) motorEx.setVelocity(0.0);
        } catch (RuntimeException ex) {
            failure = ex;
        }
        try {
            motor.setPower(0.0);
        } catch (RuntimeException ex) {
            failure = combine(failure, ex);
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
                            + "before starting motor-velocity diagnostics; this tester never "
                            + "resets or restores encoder-reset mode.");
        }
        origDir = motor.getDirection();
        origZpb = motor.getZeroPowerBehavior();
        restorationAttempted = false;
        retainedRestorationFailure = null;
        motorPreparationStarted = true;
        try {
            motorEx.setVelocity(0.0);
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorPrepared = true;
        } catch (RuntimeException preparationFailure) {
            try {
                restoreOriginalSettings();
            } catch (RuntimeException cleanupFailure) {
                preparationFailure.addSuppressed(cleanupFailure);
            }
            throw new IllegalStateException(
                    "Cannot prepare selected motor for velocity diagnostics",
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

    private static DcMotor.Direction safeGetDir(DcMotor m) {
        try {
            return m.getDirection();
        } catch (Exception ignored) {
            return null;
        }
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
            t.addLine("Tip: use DcMotorPowerTester if you only need open-loop power.");
        }

        t.update();
    }

    private void renderTelemetry() {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        t.addLine("=== DcMotor Velocity Tester ===");
        t.addLine("Motor: " + motorName);
        t.addData("Enable [A]", targetVelTps.isEnabled() ? "RUN_USING_ENCODER ON" : "OFF");
        DcMotor.Direction observedDirection = safeGetDir(motor);
        t.addData("Direction [X]",
                observedDirection == null ? "unavailable" : observedDirection.name());
        t.addData("Step [gamepad START]", "%s (%d tps)",
                targetVelTps.isFine() ? "FINE" : "COARSE",
                targetVelTps.step());
        t.addData("Staged target [Dpad U/D]", "%d tps", targetVelTps.target());
        if (targetVelTps.isEnabled()) {
            t.addData("Submitted target this cycle", "%d tps", targetVelTps.applied());
        } else if (!opModeStarted) {
            t.addData("Velocity command", "INIT - no command submitted");
        } else {
            t.addData("Velocity command", "not active; velocity/power zero submitted");
        }
        t.addData("Zero target [Y]", "target -> 0 tps");
        t.addData("Stop [B]", "disable + velocity 0");

        if (!opModeStarted) {
            t.addData("Output lock", "INIT - start the OpMode, release controls, then press A");
        }
        if (testerError != null) {
            t.addData("Tester error", testerError);
        }

        Double measured = null;
        boolean nonFiniteMeasurement = false;
        DcMotor.RunMode mode = null;

        try {
            mode = motor.getMode();
        } catch (Exception ignored) {
        }
        try {
            if (motorEx != null) {
                double sample = motorEx.getVelocity();
                if (Double.isFinite(sample)) {
                    measured = sample;
                } else {
                    nonFiniteMeasurement = true;
                }
            }
        } catch (Exception ignored) {
        }

        t.addLine("");
        if (measured == null) {
            t.addData("Measured velocity",
                    nonFiniteMeasurement ? "unavailable (non-finite reading)" : "unavailable");
        } else {
            t.addData("Measured velocity", "%.1f tps", measured);
        }
        if (targetVelTps.isEnabled() && measured != null) {
            t.addData("Velocity error", "%.1f tps", targetVelTps.applied() - measured);
        } else {
            t.addData("Velocity error", "unavailable (target inactive or velocity unreadable)");
        }
        t.addData("Mode", mode == null ? "unavailable" : mode.name());

        t.addLine("");
        t.addLine("BACK: return to the motor picker.");
        t.update();
    }
}
