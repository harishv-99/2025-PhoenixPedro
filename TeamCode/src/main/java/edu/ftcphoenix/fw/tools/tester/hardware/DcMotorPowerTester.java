package edu.ftcphoenix.fw.tools.tester.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
 *       <li>BACK: return to picker (change motor)</li>
 *     </ul>
 *   </li>
 * </ul>
 */
public final class DcMotorPowerTester extends BaseTeleOpTester {

    private final String preferredName;

    private HardwareNamePicker picker;

    private String motorName = null;
    private DcMotor motor = null;
    private DcMotorEx motorEx = null;

    private boolean ready = false;
    private String resolveError = null;

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
                () -> ready
        );

    }

    /** {@inheritDoc} */
    @Override
    public boolean onBackPressed() {
        if (!ready) {
            return false;
        }

        // Return to picker state so a different motor can be selected.
        applyPower(0.0);

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
        updateAndRender();
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
    protected void onStop() {
        applyPower(0.0);
    }

    // ---------------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------------

    private void tryResolveMotor(String name) {
        resolveError = null;

        try {
            motorEx = ctx.hw.get(DcMotorEx.class, name);
            motor = motorEx;
            ready = true;
        } catch (Exception ignored) {
            motorEx = null;
            motor = null;
            try {
                motor = ctx.hw.get(DcMotor.class, name);
                ready = true;
            } catch (Exception ex2) {
                ready = false;
                resolveError = ex2.getClass().getSimpleName() + ": " + ex2.getMessage();
            }
        }

        // Ensure safe default
        if (ready && motor != null) {
            try {
                motor.setPower(0.0);
            } catch (Exception ignored) {
            }
        }
    }

    private void updateAndRender() {
        power.updateFromAxis(clock, () -> ready);

        double applied = power.applied();
        applyPower(applied);

        renderTelemetry(applied);
    }

    private void applyPower(double pwr) {
        if (motor == null) return;
        motor.setPower(pwr);
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

    private void renderTelemetry(double appliedPower) {
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

        if (motor != null) {
            try {
                t.addData("Current position", "%d ticks", motor.getCurrentPosition());
            } catch (Exception ignored) {
            }

            if (motorEx != null) {
                try {
                    t.addData("Velocity", "%.1f ticks/s", motorEx.getVelocity());
                } catch (Exception ignored) {
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
}
