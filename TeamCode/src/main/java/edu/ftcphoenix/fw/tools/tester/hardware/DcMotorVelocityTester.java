package edu.ftcphoenix.fw.tools.tester.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.ftc.ui.HardwareNamePicker;
import edu.ftcphoenix.fw.tools.tester.ui.IntTuner;

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
 *       <li><b>X</b>: toggle motor direction (FORWARD/REVERSE)</li>
 *       <li><b>START</b>: toggle fine/coarse (affects target step and stick nudge rate)</li>
 *       <li><b>Dpad Up/Down</b>: step target velocity</li>
 *       <li><b>Right stick Y</b>: smoothly nudge target velocity (hold to change continuously)</li>
 *       <li><b>Y</b>: set target velocity to 0 (does not disable)</li>
 *       <li><b>B</b>: stop (disable + target=0)</li>
 *       <li><b>BACK</b>: return to picker (change motor)</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p><b>Safety:</b> motor is commanded to 0 on stop and original settings are restored.</p>
 */
public final class DcMotorVelocityTester extends BaseTeleOpTester {

    // Keep range wide so it works for many motors/encoders. Adjust if you want.
    private static final int VEL_MIN_TPS = -25000; // ticks/sec
    private static final int VEL_MAX_TPS = +25000; // ticks/sec

    private static final int VEL_FINE_STEP_TPS = 50;
    private static final int VEL_COARSE_STEP_TPS = 250;

    // Stick nudge speed (ticks/sec change per second at full deflection)
    private static final double NUDGE_FINE_RATE_TPS_PER_SEC = 500.0;
    private static final double NUDGE_COARSE_RATE_TPS_PER_SEC = 3000.0;

    private final String preferredName;

    private HardwareNamePicker picker;

    private String motorName = null;

    private DcMotor motor = null;
    private DcMotorEx motorEx = null;

    private boolean ready = false;
    private String resolveError = null;

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
        picker = new HardwareNamePicker(
                ctx.hw,
                DcMotor.class,
                "Select Motor",
                "Dpad: highlight | A: choose | X: refresh"
        );
        picker.refresh();

        // Enable/disable is useful here.
        targetVelTps.setEnableSupported(true);

        // Attach stick nudge for target velocity (right stick Y).
        targetVelTps.attachAxisNudge(gamepads.p1().rightY(), 0.08,
                NUDGE_FINE_RATE_TPS_PER_SEC,
                NUDGE_COARSE_RATE_TPS_PER_SEC);

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

        // A: toggle enabled with side-effects.
        bindings.onRise(gamepads.p1().a(), () -> {
            if (!ready) return;
            targetVelTps.toggleEnabled();

            if (!targetVelTps.isEnabled()) {
                stopMotorNow();
            }
        });

        // X: toggle motor direction.
        bindings.onRise(gamepads.p1().x(), () -> {
            if (!ready) return;
            toggleDirection();
        });

        // START: fine/coarse
        bindings.onRise(gamepads.p1().start(), () -> {
            if (!ready) return;
            targetVelTps.toggleFine();
        });

        // Dpad up/down: velocity steps (only when ready)
        bindings.onRise(gamepads.p1().dpadUp(), () -> {
            if (!ready) return;
            targetVelTps.inc();
        });
        bindings.onRise(gamepads.p1().dpadDown(), () -> {
            if (!ready) return;
            targetVelTps.dec();
        });

        // Y: zero target velocity (keep enabled state as-is)
        bindings.onRise(gamepads.p1().y(), () -> {
            if (!ready) return;
            targetVelTps.setTarget(0);
        });

        // B: stop (disable + target=0)
        bindings.onRise(gamepads.p1().b(), () -> {
            if (!ready) return;

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

        stopMotorNow();
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

    /** {@inheritDoc} */
    @Override
    protected void onInitLoop(double dtSec) {
        if (!ready) {
            renderPicker();
            return;
        }
        updateAndRender(dtSec);
    }

    /** {@inheritDoc} */
    @Override
    protected void onLoop(double dtSec) {
        if (!ready) {
            renderPicker();
            return;
        }
        updateAndRender(dtSec);
    }

    /** {@inheritDoc} */
    @Override
    protected void onStop() {
        stopMotorNow();
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

        // Snapshot original settings.
        origMode = safeGetMode(motor);
        origDir = safeGetDir(motor);
        origZpb = safeGetZpb(motor);

        // Put motor into a test-friendly default.
        try {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception ignored) {
        }

        // Default: disabled until user presses A
        if (targetVelTps.isEnabled()) {
            targetVelTps.toggleEnabled();
        }
        targetVelTps.setTarget(0);

        ready = true;
    }

    private void updateAndRender(double dtSec) {
        // Stick nudge always updates target, even if disabled.
        targetVelTps.updateFromAxis(clock, () -> ready);

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
        } catch (Exception ignored) {
            // Fail-safe
            if (targetVelTps.isEnabled()) {
                targetVelTps.toggleEnabled();
            }
            stopMotorNow();
        }
    }

    private void stopMotorNow() {
        if (motor == null) return;
        try {
            if (motorEx != null) motorEx.setVelocity(0);
        } catch (Exception ignored) {
        }
        try {
            motor.setPower(0.0);
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
        t.addData("Direction [X]", String.valueOf(safeGetDir(motor)));
        t.addData("Step [START]", "%s (%d tps)",
                targetVelTps.isFine() ? "FINE" : "COARSE",
                targetVelTps.step());
        t.addData("Target vel [Dpad U/D | RightStickY]", "%d tps", targetVelTps.target());
        t.addData("Applied target", "%d tps", targetVelTps.applied());
        t.addData("Zero target [Y]", "target -> 0 tps");
        t.addData("Stop [B]", "disable + velocity 0");

        double measured = 0.0;
        DcMotor.RunMode mode = null;

        try {
            mode = motor.getMode();
        } catch (Exception ignored) {
        }
        try {
            if (motorEx != null) measured = motorEx.getVelocity();
        } catch (Exception ignored) {
        }

        t.addLine("");
        t.addData("Measured velocity", "%.1f tps", measured);
        t.addData("Velocity error", "%.1f tps", targetVelTps.target() - measured);
        t.addData("Mode", String.valueOf(mode));

        t.addLine("");
        t.addLine("BACK: return to the motor picker.");
        t.update();
    }
}
