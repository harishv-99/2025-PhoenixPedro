package edu.ftcphoenix.fw.tools.tester.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.tools.tester.ui.HardwareNamePicker;
import edu.ftcphoenix.fw.tools.tester.ui.ScalarTuner;

/**
 * Generic tester for a configured {@link Servo} that lets you vary servo position.
 *
 * <h2>Selection</h2>
 * If constructed without a name (or the preferred name cannot be resolved), shows a picker listing
 * configured servos.
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>PICKER (no device chosen yet)</b>: Dpad Up/Down highlight, A choose, B refresh</li>
 *   <li><b>RUN (device chosen)</b>:
 *     <ul>
 *       <li>A: enable/disable apply (disabled holds last applied output)</li>
 *       <li>X: invert (position becomes 1 - x)</li>
 *       <li>START: fine/coarse step</li>
 *       <li>Dpad Up/Down: step position</li>
 *       <li>Left stick Y: live override (maps [-1..1] → [0..1])</li>
 *       <li>B: center (0.5)</li>
 *       <li>BACK: return to picker (change servo)</li>
 *     </ul>
 *   </li>
 * </ul>
 */
public final class ServoPositionTester extends BaseTeleOpTester {

    private final String preferredName;

    private HardwareNamePicker picker;

    private String servoName = null;
    private Servo servo = null;

    private boolean ready = false;
    private String resolveError = null;

    private final ScalarTuner position =
            new ScalarTuner("Position", 0.0, 1.0, 0.01, 0.05, 0.5)
                    // For [0..1], default invert (midpoint reflection) becomes 1-x.
                    .setDisabledValue(0.5)
                    // Disabled means "don't move it anymore" (hold last applied).
                    .setDisabledBehavior(ScalarTuner.DisabledBehavior.HOLD_LAST_APPLIED);

    /**
     * Create a servo position tester with no preferred device name.
     *
     * <p>A picker menu is shown so you can choose a configured servo.</p>
     */
    public ServoPositionTester() {
        this(null);
    }

    /**
     * Create a servo position tester with a preferred device name.
     *
     * <p>If {@code servoName} is null/blank or cannot be resolved, the tester will fall back to the picker menu.</p>
     *
     * @param servoName configured servo name in the FTC Robot Configuration (nullable)
     */
    public ServoPositionTester(String servoName) {
        this.preferredName = servoName;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String name() {
        return "Servo Position Tester";
    }

    /** {@inheritDoc} */
    @Override
    protected void onInit() {
        picker = new HardwareNamePicker(
                ctx.hw,
                Servo.class,
                "Select Servo",
                "Dpad: highlight | A: choose | B: refresh"
        );
        picker.refresh();

        // Map stickY [-1..1] -> [0..1]
        // stickY +1 => 1.0, stickY -1 => 0.0
        position.attachAxis(gamepads.p1().leftY(), 0.08, v -> (v + 1.0) * 0.5);

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

        // Standard scalar bindings
        position.bind(
                bindings,
                gamepads.p1().a(),        // enable
                gamepads.p1().x(),        // invert
                gamepads.p1().start(),    // fine/coarse
                gamepads.p1().dpadUp(),   // inc
                gamepads.p1().dpadDown(), // dec
                null,                     // (we override B)
                () -> ready
        );

        // B: center
        bindings.onPress(gamepads.p1().b(), () -> {
            if (!ready) return;
            position.setTarget(0.5);
        });
    }

    /** {@inheritDoc} */
    @Override
    public boolean onBackPressed() {
        if (!ready) {
            return false;
        }

        // Leave the servo at its last applied position (consistent with onStop()).
        try {
            if (servo != null) servo.setPosition(position.lastApplied());
        } catch (Exception ignored) {
        }

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
        // Leave servo at whatever it was last commanded to.
        // (HOLD_LAST_APPLIED already models this behavior.)
        if (servo != null) {
            try {
                servo.setPosition(position.lastApplied());
            } catch (Exception ignored) {
            }
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------------

    private void tryResolve(String name) {
        resolveError = null;
        try {
            servo = ctx.hw.get(Servo.class, name);
            ready = true;
        } catch (Exception ex) {
            servo = null;
            ready = false;
            resolveError = ex.getClass().getSimpleName() + ": " + ex.getMessage();
        }
    }

    private void updateAndRender() {
        position.updateFromAxis(() -> ready);

        double applied = position.applied();
        if (servo != null) {
            servo.setPosition(applied);
        }

        renderTelemetry(applied);
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

    private void renderTelemetry(double applied) {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        t.addLine("=== Servo Position Tester ===");
        t.addLine("Servo: " + servoName);

        position.render(t);

        t.addLine(String.format(Locale.US, "AppliedNow=%.3f", applied));
        t.addLine("Controls: A enable | X invert | START step | dpad +/- | stickY override | B center | BACK picker");

        if (servo != null) {
            try {
                t.addLine(String.format(Locale.US, "Servo.getPosition()=%.3f", servo.getPosition()));
            } catch (Exception ignored) {
            }
        }

        t.update();
    }
}
