package edu.ftcphoenix.fw.tools.tester.hardware;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.tools.tester.ui.HardwareNamePicker;
import edu.ftcphoenix.fw.tools.tester.ui.ScalarTuner;

/**
 * Generic tester for a configured {@link CRServo} that lets you vary servo power.
 *
 * <h2>Selection</h2>
 * If constructed without a name (or the preferred name cannot be resolved), shows a picker listing
 * configured CRServos.
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>PICKER (no device chosen yet)</b>: Dpad Up/Down highlight, A choose, B refresh</li>
 *   <li><b>RUN (device chosen)</b>:
 *     <ul>
 *       <li>A: enable/disable output</li>
 *       <li>X: invert</li>
 *       <li>START: fine/coarse step</li>
 *       <li>Dpad Up/Down: step power</li>
 *       <li>Left stick Y: live override (sets target while moved)</li>
 *       <li>B: zero</li>
 *       <li>BACK: return to picker (change CRServo)</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p><b>Safety:</b> power is set to 0 on stop.</p>
 */
public final class CrServoPowerTester extends BaseTeleOpTester {

    private final String preferredName;

    private HardwareNamePicker picker;

    private String servoName = null;
    private CRServo servo = null;

    private boolean ready = false;
    private String resolveError = null;

    private final ScalarTuner power =
            new ScalarTuner("Power", -1.0, +1.0, 0.05, 0.20, 0.0);

    /**
     * Create a CR servo power tester with no preferred device name.
     *
     * <p>A picker menu is shown so you can choose from configured CR servos.</p>
     */
    public CrServoPowerTester() {
        this(null);
    }

    /**
     * Create a CR servo power tester with a preferred device name.
     *
     * <p>If {@code servoName} is null/blank or cannot be resolved, the tester will fall back to the picker menu.</p>
     *
     * @param servoName configured CR servo name in the FTC Robot Configuration (nullable)
     */
    public CrServoPowerTester(String servoName) {
        this.preferredName = servoName;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String name() {
        return "CRServo Power Tester";
    }

    /** {@inheritDoc} */
    @Override
    protected void onInit() {
        picker = new HardwareNamePicker(
                ctx.hw,
                CRServo.class,
                "Select CRServo",
                "Dpad: highlight | A: choose | B: refresh"
        );
        picker.refresh();

        power.attachAxis(gamepads.p1().leftY(), 0.08, v -> v);

        if (preferredName != null && !preferredName.trim().isEmpty()) {
            servoName = preferredName.trim();
            tryResolve(servoName);
        }

        picker.bind(
                bindings,
                gamepads.p1().dpadUp(),
                gamepads.p1().dpadDown(),
                gamepads.p1().a(),
                gamepads.p1().b(),
                () -> !ready,
                chosen -> {
                    servoName = chosen;
                    tryResolve(servoName);
                }
        );

        power.bind(
                bindings,
                gamepads.p1().a(),        // enable
                gamepads.p1().x(),        // invert
                gamepads.p1().start(),    // fine/coarse
                gamepads.p1().dpadUp(),   // inc
                gamepads.p1().dpadDown(), // dec
                gamepads.p1().b(),        // zero
                () -> ready
        );
    }

    /** {@inheritDoc} */
    @Override
    public boolean onBackPressed() {
        if (!ready) {
            return false;
        }

        // Return to picker state so a different servo can be selected.
        applyPower(0.0);

        ready = false;
        servo = null;
        resolveError = null;

        picker.clearChoice();
        picker.refresh();
        if (servoName != null && !servoName.isEmpty()) {
            picker.setPreferredName(servoName);
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

    private void tryResolve(String name) {
        resolveError = null;
        try {
            servo = ctx.hw.get(CRServo.class, name);
            ready = true;
        } catch (Exception ex) {
            servo = null;
            ready = false;
            resolveError = ex.getClass().getSimpleName() + ": " + ex.getMessage();
        }
    }

    private void updateAndRender() {
        power.updateFromAxis(clock, () -> ready);

        double applied = power.applied();
        applyPower(applied);

        renderTelemetry(applied);
    }

    private void applyPower(double pwr) {
        if (servo == null) return;
        servo.setPower(pwr);
    }

    private void renderPicker() {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        picker.render(t);

        if (servoName != null && !servoName.isEmpty()) {
            t.addLine("");
            t.addLine("Chosen: " + servoName);
        }

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

        t.addLine("=== CRServo Power Tester ===");
        t.addLine("CRServo: " + servoName);

        power.render(t);

        t.addLine("");
        t.addLine("Controls: A enable | X invert | START step | dpad +/- | stickY override | B zero | BACK picker");

        if (servo != null) {
            try {
                t.addLine(String.format(Locale.US, "Applied=%.2f", appliedPower));
            } catch (Exception ignored) {
            }
        }

        t.update();
    }
}
