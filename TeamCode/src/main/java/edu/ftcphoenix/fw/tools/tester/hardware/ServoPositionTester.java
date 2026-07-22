package edu.ftcphoenix.fw.tools.tester.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.ftc.ui.HardwareNamePicker;
import edu.ftcphoenix.fw.input.binding.Bindings;
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
 *   <li><b>PICKER (no device chosen yet)</b>: Dpad Up/Down highlight, A choose, X refresh</li>
 *   <li><b>RUN (device chosen)</b>:
 *     <ul>
 *       <li>A: enable/disable apply (disabled makes no position writes)</li>
 *       <li>X: invert (position becomes 1 - x)</li>
 *       <li>START: fine/coarse step</li>
 *       <li>Dpad Up/Down: step position</li>
 *       <li>Left stick Y: live override (maps [-1..1] → [0..1])</li>
 *       <li>B: center (0.5)</li>
 *       <li>BACK: disable, hold the last commanded position, and return to the picker</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p><b>Safety:</b> selecting a servo snapshots that device's current finite SDK command from
 * {@link Servo#getPosition()} (not physical position feedback), leaves output disabled, and resets
 * inversion to OFF for that selection. Release A after choosing, then press it again to enable
 * writes starting from that device's own command snapshot. While disabled the loop does not call
 * {@link Servo#setPosition(double)};
 * BACK and stop make one explicit hold write using the selected servo's last commanded position.</p>
 */
public final class ServoPositionTester extends BaseTeleOpTester {

    private final String preferredName;

    private HardwareNamePicker picker;

    private String servoName = null;
    private Servo servo = null;

    private boolean ready = false;
    private String resolveError = null;
    private double selectedHoldPosition = 0.5;

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
                "Dpad: highlight | A: choose | X: refresh"
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
                gamepads.p1().x(),
                () -> !ready,
                chosen -> {
                    servoName = chosen;
                    tryResolve(servoName);
                }
        );

        Bindings.ControlContext liveControls = bindings.contextWhen(
                BooleanSource.of(() -> ready),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL
        );

        // Standard scalar bindings
        position.bind(
                liveControls,
                gamepads.p1().a(),        // enable
                gamepads.p1().x(),        // invert
                gamepads.p1().start(),    // fine/coarse
                gamepads.p1().dpadUp(),   // inc
                gamepads.p1().dpadDown(), // dec
                null                      // (we override B)
        );

        // B: center
        liveControls.onRise(gamepads.p1().b(), () -> position.setTarget(0.5));
    }

    /** {@inheritDoc} */
    @Override
    public boolean onBackPressed() {
        if (!ready) {
            return false;
        }

        // Disable before releasing the selected device, then explicitly preserve its last command.
        disablePositionControl();
        holdSelectedPosition();

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
        disablePositionControl();
        holdSelectedPosition();
    }

    // ---------------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------------

    private void tryResolve(String name) {
        resolveError = null;
        try {
            Servo resolvedServo = ctx.hw.get(Servo.class, name);
            double currentPosition = resolvedServo.getPosition();
            if (!Double.isFinite(currentPosition)) {
                throw new IllegalStateException(
                        "Servo '" + name + "' returned a non-finite position");
            }

            disablePositionControl();
            resetPositionInversion();
            position.setTarget(currentPosition);
            selectedHoldPosition = position.target();
            servo = resolvedServo;
            ready = true;
        } catch (Exception ex) {
            disablePositionControl();
            servo = null;
            ready = false;
            resolveError = ex.getClass().getSimpleName() + ": " + ex.getMessage();
        }
    }

    private void updateAndRender() {
        position.updateFromAxis(clock, () -> ready);

        if (position.isEnabled() && servo != null) {
            double applied = position.applied();
            servo.setPosition(applied);
            selectedHoldPosition = applied;
        }

        renderTelemetry();
    }

    private void disablePositionControl() {
        if (position.isEnabled()) {
            position.toggleEnabled();
        }
    }

    private void resetPositionInversion() {
        if (position.isInverted()) {
            position.toggleInvert();
        }
    }

    private void holdSelectedPosition() {
        if (servo == null) {
            return;
        }
        try {
            servo.setPosition(selectedHoldPosition);
        } catch (Exception ignored) {
        }
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

    private void renderTelemetry() {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        t.addLine("=== Servo Position Tester ===");
        t.addLine("Servo: " + servoName);
        t.addData("Enable [A]", position.isEnabled() ? "ON" : "OFF (no writes)");
        t.addData("Invert [X]", position.isInverted() ? "ON" : "OFF");
        t.addData("Step [START]", "%s (%.3f)", position.isFine() ? "FINE" : "COARSE", position.step());
        t.addData("Position target [Dpad U/D | LeftStickY]", "%.3f", position.target());
        t.addData("Center [B]", "target -> 0.500");
        t.addData("Selected hold / last command", "%.3f", selectedHoldPosition);

        if (servo != null) {
            try {
                t.addData("Servo.getPosition()", "%.3f", servo.getPosition());
            } catch (Exception ignored) {
            }
        }

        t.addLine("");
        t.addLine("BACK: return to the servo picker.");
        t.update();
    }
}
