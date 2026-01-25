package edu.ftcphoenix.fw.tools.tester.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.tools.tester.ui.HardwareNamePicker;
import edu.ftcphoenix.fw.tools.tester.ui.IntTuner;
import edu.ftcphoenix.fw.tools.tester.ui.ScalarTuner;

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
 *   <li><b>PICKER (no motor chosen yet)</b>: Dpad Up/Down highlight, A choose, B refresh</li>
 *   <li><b>RUN (motor selected)</b>:
 *     <ul>
 *       <li><b>A</b>: enable/disable (RUN_TO_POSITION active)</li>
 *       <li><b>X</b>: toggle motor direction (FORWARD/REVERSE)</li>
 *       <li><b>START</b>: toggle fine/coarse (affects target step and power step and stick nudge rate)</li>
 *       <li><b>Dpad Up/Down</b>: step target ticks</li>
 *       <li><b>Dpad Right/Left</b>: adjust power</li>
 *       <li><b>Right stick Y</b>: smoothly nudge target (hold to move target continuously)</li>
 *       <li><b>Y</b>: reset encoder (STOP_AND_RESET_ENCODER) and set target=0</li>
 *       <li><b>B</b>: stop (disable + power=0)</li>
 *       <li><b>BACK</b>: return to picker (change motor)</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p><b>Notes:</b> RUN_TO_POSITION requires a meaningful encoder reading. If your motor/encoder
 * is not configured or supported, the motor may not move as expected.</p>
 *
 * <p><b>Safety:</b> power is set to 0 and original motor settings are restored on stop.</p>
 */
public final class DcMotorPositionTester extends BaseTeleOpTester {

    // Keep this reasonably bounded so a fat-finger doesn’t send the target to infinity.
    private static final int TARGET_MIN_TICKS = -20000;
    private static final int TARGET_MAX_TICKS = +20000;

    // Step sizes for target ticks
    private static final int TARGET_FINE_STEP_TICKS = 25;
    private static final int TARGET_COARSE_STEP_TICKS = 100;

    // Stick nudge speed (ticks/sec at full stick deflection)
    private static final double NUDGE_FINE_RATE_TICKS_PER_SEC = 250.0;
    private static final double NUDGE_COARSE_RATE_TICKS_PER_SEC = 1500.0;

    private final String preferredName;

    private HardwareNamePicker picker;

    private String motorName = null;
    private DcMotor motor = null;
    private DcMotorEx motorEx = null;

    private boolean ready = false;
    private String resolveError = null;

    // Tuners
    private final IntTuner targetTicks =
            new IntTuner("TargetTicks",
                    TARGET_MIN_TICKS, TARGET_MAX_TICKS,
                    TARGET_FINE_STEP_TICKS, TARGET_COARSE_STEP_TICKS,
                    0);

    // Power (0..1). Enable is handled by targetTicks (RUN_TO_POSITION enabled/disabled).
    private final ScalarTuner power =
            new ScalarTuner("Power", 0.0, 1.0, 0.02, 0.10, 0.30)
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

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onInit() {
        picker = new HardwareNamePicker(
                ctx.hw,
                DcMotor.class,
                "Select Motor",
                "Dpad: highlight | A: choose | B: refresh"
        );
        picker.refresh();

        // Attach stick nudge for target (right stick Y). Positive Y increases target.
        targetTicks.attachAxisNudge(gamepads.p1().rightY(), 0.08,
                NUDGE_FINE_RATE_TICKS_PER_SEC,
                NUDGE_COARSE_RATE_TICKS_PER_SEC);

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
                gamepads.p1().b(),
                () -> !ready,
                chosen -> {
                    motorName = chosen;
                    tryResolveMotor(motorName);
                }
        );

        // A toggles RUN_TO_POSITION enable (only when ready)
        bindings.onPress(gamepads.p1().a(), () -> {
            if (!ready) return;
            targetTicks.toggleEnabled();

            // If disabling, force motor quiet immediately.
            if (!targetTicks.isEnabled()) {
                safeDisableMotor();
            }
        });

        // X toggles motor direction (hardware-level)
        bindings.onPress(gamepads.p1().x(), () -> {
            if (!ready) return;
            toggleDirection();
        });

        // START toggles fine/coarse for BOTH tuners (so UI feels consistent)
        bindings.onPress(gamepads.p1().start(), () -> {
            if (!ready) return;
            targetTicks.toggleFine();
            power.toggleFine();
        });

        // Target inc/dec (dpad up/down) — only when ready
        bindings.onPress(gamepads.p1().dpadUp(), () -> {
            if (!ready) return;
            targetTicks.inc();
        });
        bindings.onPress(gamepads.p1().dpadDown(), () -> {
            if (!ready) return;
            targetTicks.dec();
        });

        // Power inc/dec (dpad right/left) — only when ready
        bindings.onPress(gamepads.p1().dpadRight(), () -> {
            if (!ready) return;
            power.inc();
        });
        bindings.onPress(gamepads.p1().dpadLeft(), () -> {
            if (!ready) return;
            power.dec();
        });

        // Reset encoder
        bindings.onPress(gamepads.p1().y(), () -> {
            if (!ready) return;
            resetEncoderAndZeroTarget();
        });

        // Hard stop
        bindings.onPress(gamepads.p1().b(), () -> {
            if (!ready) return;

            // Disable if enabled
            if (targetTicks.isEnabled()) {
                targetTicks.toggleEnabled();
            }
            // Set power target to 0 for safety
            power.setTarget(0.0);

            safeDisableMotor();
        });
    }

    /**
     * {@inheritDoc}
     */
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

        safeDisableMotor();
        restoreOriginalSettings();

        ready = false;
        motor = null;
        motorEx = null;
        resolveError = null;

        picker.clearChoice();
        picker.refresh();
        if (motorName != null && !motorName.isEmpty()) {
            picker.setPreferredName(motorName);
        }

        return true;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onInitLoop(double dtSec) {
        if (!ready) {
            renderPicker();
            return;
        }
        updateAndRender(dtSec);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onLoop(double dtSec) {
        if (!ready) {
            renderPicker();
            return;
        }
        updateAndRender(dtSec);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onStop() {
        // Safety: always stop power and restore original settings.
        try {
            if (motor != null) motor.setPower(0.0);
        } catch (Exception ignored) {
        }

        restoreOriginalSettings();
    }

    // ---------------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------------

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

        // Snapshot original settings
        origMode = safeGetMode(motor);
        origDir = safeGetDir(motor);
        origZpb = safeGetZpb(motor);

        // Put motor into a test-friendly default.
        try {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception ignored) {
        }

        // Default targets
        targetTicks.setTarget(0);
        if (targetTicks.isEnabled()) {
            // Start disabled by default unless user enabled it earlier for some reason
            targetTicks.toggleEnabled();
        }
        power.setTarget(0.30);

        ready = true;
    }

    private void updateAndRender(double dtSec) {
        // Allow stick to nudge target continuously (even while enabled).
        targetTicks.updateFromAxis(dtSec, () -> ready);

        if (targetTicks.isEnabled()) {
            applyRunToPosition();
        } else {
            // keep motor quiet while disabled
            try {
                motor.setPower(0.0);
            } catch (Exception ignored) {
            }
        }

        renderTelemetry();
    }

    private void applyRunToPosition() {
        if (motor == null) return;

        try {
            motor.setTargetPosition(targetTicks.applied());
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Only apply power while enabled; otherwise 0.
            double pwr = power.applied();
            motor.setPower(pwr);
        } catch (Exception ignored) {
            // Fail-safe if something breaks mid-run.
            if (targetTicks.isEnabled()) {
                targetTicks.toggleEnabled();
            }
            safeDisableMotor();
        }
    }

    private void safeDisableMotor() {
        try {
            if (motor != null) {
                motor.setPower(0.0);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } catch (Exception ignored) {
        }
    }

    private void resetEncoderAndZeroTarget() {
        if (motor == null) return;
        try {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            targetTicks.setTarget(0);
        } catch (Exception ignored) {
        }
    }

    private void toggleDirection() {
        if (motor == null) return;
        try {
            DcMotor.Direction d = motor.getDirection();
            motor.setDirection(d == DcMotor.Direction.FORWARD ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        } catch (Exception ignored) {
        }
    }

    private void restoreOriginalSettings() {
        if (motor == null) return;

        try {
            motor.setPower(0.0);
        } catch (Exception ignored) {
        }

        try {
            if (origDir != null) motor.setDirection(origDir);
        } catch (Exception ignored) {
        }
        try {
            if (origZpb != null) motor.setZeroPowerBehavior(origZpb);
        } catch (Exception ignored) {
        }
        try {
            if (origMode != null) motor.setMode(origMode);
        } catch (Exception ignored) {
        }
    }

    private static DcMotor.RunMode safeGetMode(DcMotor m) {
        try {
            return m.getMode();
        } catch (Exception ignored) {
            return null;
        }
    }

    private static DcMotor.Direction safeGetDir(DcMotor m) {
        try {
            return m.getDirection();
        } catch (Exception ignored) {
            return null;
        }
    }

    private static DcMotor.ZeroPowerBehavior safeGetZpb(DcMotor m) {
        try {
            return m.getZeroPowerBehavior();
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
        }

        t.update();
    }

    private void renderTelemetry() {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        t.addLine("=== DcMotor Position Tester ===");
        t.addLine("Motor: " + motorName);

        t.addLine(String.format(Locale.US,
                "Enabled=%s | Step=%s (targetStep=%d ticks, powerStep=%.2f) | StickNudge=%s",
                targetTicks.isEnabled() ? "ON" : "OFF",
                targetTicks.isFine() ? "FINE" : "COARSE",
                targetTicks.step(),
                power.step(),
                "RightStickY"
        ));

        t.addLine("");
        targetTicks.render(t);
        power.render(t);

        int cur = 0;
        boolean busy = false;
        DcMotor.RunMode mode = null;
        DcMotor.Direction dir = null;

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
        try {
            dir = motor.getDirection();
        } catch (Exception ignored) {
        }

        t.addLine("");
        t.addLine(String.format(Locale.US,
                "Current=%d | Error=%d",
                cur, (targetTicks.target() - cur)
        ));
        t.addLine(String.format(Locale.US,
                "Mode=%s | Dir=%s | Busy=%s",
                String.valueOf(mode),
                String.valueOf(dir),
                busy ? "YES" : "NO"
        ));

        if (motorEx != null) {
            try {
                t.addLine(String.format(Locale.US, "Velocity=%.1f ticks/s", motorEx.getVelocity()));
            } catch (Exception ignored) {
            }
        }

        t.addLine("");
        t.addLine("Controls: A enable | X dir | START fine/coarse | dpad U/D target | dpad L/R power | RStickY nudge | Y reset | B stop | BACK picker");

        t.update();
    }
}
