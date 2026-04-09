package edu.ftcphoenix.fw.tools.tester.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.core.color.NormalizedRgba;
import edu.ftcphoenix.fw.core.color.Rgba;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.ftc.FtcSensors;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;
import edu.ftcphoenix.fw.tools.tester.ui.HardwareNamePicker;
import edu.ftcphoenix.fw.tools.tester.ui.ScalarTuner;

/**
 * Generic tester for a configured {@link NormalizedColorSensor}.
 *
 * <h2>What it is for</h2>
 * <p>This tester is aimed at close-range object classification bring-up. It treats
 * {@link NormalizedColorSensor} as the primary interface because normalized channels plus
 * ratio-based logic usually transfer better than raw thresholds do. When the selected device also
 * exposes the FTC {@link ColorSensor} interface, the tester can show raw RGBA values in a detail
 * view for same-sensor debugging.</p>
 *
 * <h2>Selection</h2>
 * If constructed without a preferred name (or the preferred name cannot be resolved), shows a
 * picker listing configured {@link NormalizedColorSensor} devices.
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>PICKER (no sensor chosen yet)</b>: Dpad Up/Down highlight, A choose, B refresh</li>
 *   <li><b>RUN (sensor chosen)</b>:
 *     <ul>
 *       <li><b>X</b>: toggle PRIMARY / DETAIL telemetry view</li>
 *       <li><b>A</b>: freeze / unfreeze the displayed sample</li>
 *       <li><b>START</b>: toggle fine / coarse gain step size</li>
 *       <li><b>Dpad Left / Right</b>: decrease / increase sensor gain</li>
 *       <li><b>B</b>: reset gain to the sensor's original gain when the tester opened</li>
 *       <li><b>BACK</b>: return to picker (change sensor)</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <p><b>Best practice:</b> use normalized ratios + alpha/chroma as the main threshold-tuning
 * signals. Treat hue and raw RGBA as secondary diagnostics.</p>
 */
public final class NormalizedColorSensorTester extends BaseTeleOpTester {

    private static final double DEFAULT_GAIN = 1.0;
    private static final double GAIN_MIN = 0.25;
    private static final double GAIN_MAX = 32.0;
    private static final double GAIN_FINE_STEP = 0.25;
    private static final double GAIN_COARSE_STEP = 1.0;

    private final String preferredName;

    private HardwareNamePicker picker;

    private String sensorName = null;
    private NormalizedColorSensor normalizedSensor = null;
    private ColorSensor rawColorSensor = null;

    private Source<NormalizedRgba> normalizedSource = null;
    private Source<Rgba> rawSource = null;

    private boolean ready = false;
    private String resolveError = null;

    private float originalGain = (float) DEFAULT_GAIN;

    private boolean frozen = false;
    private NormalizedRgba liveNormalized = null;
    private Rgba liveRaw = null;
    private NormalizedRgba frozenNormalized = null;
    private Rgba frozenRaw = null;

    private ViewMode viewMode = ViewMode.PRIMARY;

    private final ScalarTuner gain = new ScalarTuner(
            "Gain",
            GAIN_MIN,
            GAIN_MAX,
            GAIN_FINE_STEP,
            GAIN_COARSE_STEP,
            DEFAULT_GAIN
    )
            .setEnableSupported(false)
            .setInvertSupported(false);

    private enum ViewMode {
        PRIMARY,
        DETAIL;

        ViewMode next() {
            return (this == PRIMARY) ? DETAIL : PRIMARY;
        }
    }

    /**
     * Create a color-sensor tester with no preferred device name.
     */
    public NormalizedColorSensorTester() {
        this(null);
    }

    /**
     * Create a color-sensor tester with a preferred device name.
     *
     * @param sensorName configured sensor name in the FTC Robot Configuration (nullable)
     */
    public NormalizedColorSensorTester(String sensorName) {
        this.preferredName = sensorName;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String name() {
        return "Normalized Color Sensor Tester";
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onInit() {
        picker = new HardwareNamePicker(
                ctx.hw,
                NormalizedColorSensor.class,
                "Select Color Sensor",
                "Dpad: highlight | A: choose | B: refresh"
        );
        picker.refresh();

        if (preferredName != null && !preferredName.trim().isEmpty()) {
            sensorName = preferredName.trim();
            tryResolve(sensorName);
        }

        picker.bind(
                bindings,
                gamepads.p1().dpadUp(),
                gamepads.p1().dpadDown(),
                gamepads.p1().a(),
                gamepads.p1().b(),
                () -> !ready,
                chosen -> {
                    sensorName = chosen;
                    tryResolve(sensorName);
                }
        );

        bindings.onRise(gamepads.p1().x(), () -> {
            if (!ready) return;
            viewMode = viewMode.next();
        });

        bindings.onRise(gamepads.p1().a(), () -> {
            if (!ready) return;
            toggleFreeze();
        });

        bindings.onRise(gamepads.p1().start(), () -> {
            if (!ready) return;
            gain.toggleFine();
        });

        bindings.onRise(gamepads.p1().dpadLeft(), () -> {
            if (!ready) return;
            gain.dec();
            applyGainTarget();
        });

        bindings.onRise(gamepads.p1().dpadRight(), () -> {
            if (!ready) return;
            gain.inc();
            applyGainTarget();
        });

        bindings.onRise(gamepads.p1().b(), () -> {
            if (!ready) return;
            resetGain();
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

        restoreOriginalGain();
        clearResolvedSensor();

        picker.clearChoice();
        picker.refresh();
        if (sensorName != null && !sensorName.isEmpty()) {
            picker.setPreferredName(sensorName);
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
        updateAndRender();
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
        updateAndRender();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onStop() {
        restoreOriginalGain();
    }

    // ---------------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------------

    private void tryResolve(String name) {
        restoreOriginalGain();
        clearResolvedSensor();
        resolveError = null;

        try {
            normalizedSensor = ctx.hw.get(NormalizedColorSensor.class, name);
            rawColorSensor = resolveRawColorSensor(name, normalizedSensor);
            normalizedSource = FtcSensors.normalizedRgba(normalizedSensor);
            rawSource = (rawColorSensor != null) ? FtcSensors.rgba(rawColorSensor) : null;

            originalGain = sanitizeGain(normalizedSensor.getGain());
            gain.setTarget(originalGain);
            applyGainTarget();

            ready = true;
            frozen = false;
            viewMode = ViewMode.PRIMARY;
            liveNormalized = null;
            liveRaw = null;
            frozenNormalized = null;
            frozenRaw = null;
        } catch (Exception ex) {
            normalizedSensor = null;
            rawColorSensor = null;
            normalizedSource = null;
            rawSource = null;
            ready = false;
            originalGain = (float) DEFAULT_GAIN;
            gain.setTarget(DEFAULT_GAIN);
            resolveError = ex.getClass().getSimpleName() + ": " + ex.getMessage();
        }
    }

    private void updateAndRender() {
        if (!frozen) {
            liveNormalized = (normalizedSource != null) ? normalizedSource.get(clock) : null;
            liveRaw = (rawSource != null) ? rawSource.get(clock) : null;
        }

        renderTelemetry(displayNormalized(), displayRaw());
    }

    private void toggleFreeze() {
        if (frozen) {
            frozen = false;
            frozenNormalized = null;
            frozenRaw = null;
            return;
        }

        if (liveNormalized == null) {
            return;
        }

        frozen = true;
        frozenNormalized = liveNormalized;
        frozenRaw = liveRaw;
    }

    private void resetGain() {
        gain.setTarget(originalGain);
        applyGainTarget();
    }

    private void applyGainTarget() {
        if (normalizedSensor == null) return;
        try {
            normalizedSensor.setGain((float) gain.target());
        } catch (Exception ignored) {
        }
    }

    private void restoreOriginalGain() {
        if (normalizedSensor == null) return;
        try {
            normalizedSensor.setGain(originalGain);
        } catch (Exception ignored) {
        }
    }

    private void clearResolvedSensor() {
        ready = false;
        normalizedSensor = null;
        rawColorSensor = null;
        normalizedSource = null;
        rawSource = null;
        frozen = false;
        viewMode = ViewMode.PRIMARY;
        liveNormalized = null;
        liveRaw = null;
        frozenNormalized = null;
        frozenRaw = null;
    }

    private NormalizedRgba displayNormalized() {
        if (frozen && frozenNormalized != null) {
            return frozenNormalized;
        }
        return liveNormalized;
    }

    private Rgba displayRaw() {
        if (frozen && frozenRaw != null) {
            return frozenRaw;
        }
        return liveRaw;
    }

    private ColorSensor resolveRawColorSensor(String name, NormalizedColorSensor sensor) {
        if (sensor instanceof ColorSensor) {
            return (ColorSensor) sensor;
        }

        try {
            return ctx.hw.get(ColorSensor.class, name);
        } catch (Exception ignored) {
            return null;
        }
    }

    private float sanitizeGain(float gainValue) {
        if (!Float.isFinite(gainValue) || gainValue <= 0.0f) {
            return (float) DEFAULT_GAIN;
        }
        return gainValue;
    }

    private void renderPicker() {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        picker.render(t);

        if (sensorName != null && !sensorName.isEmpty()) {
            t.addLine("");
            t.addLine("Chosen: " + sensorName);
        }

        if (resolveError != null) {
            t.addLine("");
            t.addLine("Resolve error:");
            t.addLine(resolveError);
        }

        t.update();
    }

    private void renderTelemetry(NormalizedRgba normalized, Rgba raw) {
        Telemetry t = ctx.telemetry;
        t.clearAll();

        t.addLine("=== Normalized Color Sensor Tester ===");
        t.addData("Sensor", sensorName);
        t.addData("View [X]", viewMode == ViewMode.PRIMARY ? "PRIMARY" : "DETAIL (+raw)");
        t.addData("Sample [A]", frozen ? "FROZEN" : "LIVE");
        t.addData(
                "Gain [Dpad L/R | START step | B reset]",
                "%.2f (%s)",
                gain.target(),
                gain.isFine() ? "FINE" : "COARSE"
        );
        t.addData("Raw ColorSensor", rawSource != null ? "AVAILABLE in detail view" : "Not exposed by this device");

        if (normalized == null) {
            t.addLine("");
            t.addLine("No normalized sample available yet.");
            t.addLine("BACK: return to the color sensor picker.");
            t.update();
            return;
        }

        t.addLine("");
        t.addLine("Primary tuning: normalized ratios + alpha/chroma. HSV is secondary debug info.");
        t.addData("Norm RGBA", "r=%.3f g=%.3f b=%.3f a=%.3f",
                normalized.r, normalized.g, normalized.b, normalized.a);
        t.addData("Norm ratios", "r=%.3f g=%.3f b=%.3f",
                normalized.rRatio(), normalized.gRatio(), normalized.bRatio());
        t.addData("Norm HSV", "h=%.1f° s=%.3f v=%.3f",
                normalized.hueDeg(), normalized.saturation(), normalized.value());
        t.addData("Norm alpha/chroma", "a=%.3f chroma=%.3f",
                normalized.a, normalized.chroma());
        t.addLine(classificationHint(normalized));

        if (viewMode == ViewMode.DETAIL) {
            t.addLine("");
            t.addLine("Detail view:");
            if (raw == null) {
                t.addLine("Raw RGBA channels are not available for this device/interface combination.");
            } else {
                t.addData("Raw RGBA", "r=%d g=%d b=%d a=%d", raw.r, raw.g, raw.b, raw.a);
                t.addData("Raw ratios", "r=%.3f g=%.3f b=%.3f",
                        raw.rRatio(), raw.gRatio(), raw.bRatio());
                t.addData("Raw HSV", "h=%.1f° s=%.3f v=%.0f",
                        raw.hueDeg(), raw.saturation(), raw.value());
                t.addData("Raw alpha/chroma", "a=%d chroma=%.0f", raw.a, raw.chroma());
            }
        }

        t.addLine("");
        t.addLine("BACK: return to the color sensor picker.");
        t.update();
    }

    private String classificationHint(NormalizedRgba normalized) {
        if (normalized.maxChannel() >= 0.98) {
            return "Hint: normalized channel is clipping; lower gain or increase distance.";
        }
        if (normalized.a < 0.03 && normalized.chroma() < 0.03) {
            return "Hint: sample is dim/weak; move closer or raise gain before tuning thresholds.";
        }
        if (normalized.chroma() < 0.03) {
            return "Hint: chroma is low; hue will be noisy. Ratios/alpha are safer than hue here.";
        }
        return "Hint: reading looks strong enough to tune classification thresholds.";
    }
}
