package edu.ftcphoenix.fw.tools.tester;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.Objects;
import java.util.function.Function;

import edu.ftcphoenix.fw.ftc.ui.HardwareNamePicker;

/**
 * Small wrapper that selects a hardware device name before launching an inner tester.
 *
 * <p>This is useful when the real tester already knows how to operate once it has a configured
 * hardware name, but the framework-only menu still needs a generic way to choose that name at
 * runtime. The wrapper owns the picker screen, then instantiates the inner tester once a device is
 * chosen. BACK returns from the inner tester to the picker unless the inner tester consumes BACK
 * itself.</p>
 */
public final class HardwareSelectingTester extends BaseTeleOpTester {

    private final String displayName;
    private final Class<? extends HardwareDevice> deviceType;
    private final String pickerTitle;
    private final String pickerHelp;
    private final String preferredName;
    private final Function<String, TeleOpTester> testerFactory;

    private HardwareNamePicker picker;
    private TeleOpTester active;
    private boolean opModeStarted;
    private String launchError;

    /**
     * Creates a wrapper that lets the operator choose a hardware device before opening an inner
     * tester configured for that device.
     *
     * @param displayName  human-facing name for the wrapper tester
     * @param deviceType   FTC hardware type to enumerate in the picker
     * @param pickerTitle  picker title shown in telemetry (nullable for a default title)
     * @param pickerHelp   picker controls hint (nullable for the default hint)
     * @param preferredName preferred device name to pre-select when it exists (nullable)
     * @param testerFactory factory that builds the real tester after a hardware name is chosen
     */
    public HardwareSelectingTester(String displayName,
                                   Class<? extends HardwareDevice> deviceType,
                                   String pickerTitle,
                                   String pickerHelp,
                                   String preferredName,
                                   Function<String, TeleOpTester> testerFactory) {
        this.displayName = (displayName == null || displayName.trim().isEmpty())
                ? "Hardware Selector"
                : displayName.trim();
        this.deviceType = Objects.requireNonNull(deviceType, "deviceType");
        this.pickerTitle = pickerTitle;
        this.pickerHelp = pickerHelp;
        this.preferredName = preferredName;
        this.testerFactory = Objects.requireNonNull(testerFactory, "testerFactory");
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String name() {
        return displayName;
    }

    @Override
    protected void onInit() {
        String title = (pickerTitle == null || pickerTitle.trim().isEmpty())
                ? ("Select " + deviceType.getSimpleName())
                : pickerTitle.trim();
        String help = (pickerHelp == null || pickerHelp.trim().isEmpty())
                ? "Dpad: highlight | A: choose | X: refresh"
                : pickerHelp.trim();

        picker = new HardwareNamePicker(ctx.hw, deviceType, title, help);
        picker.refresh();
        if (preferredName != null && !preferredName.trim().isEmpty()) {
            picker.setPreferredName(preferredName.trim());
        }

        picker.bind(
                bindings,
                gamepads.p1().dpadUp(),
                gamepads.p1().dpadDown(),
                gamepads.p1().a(),
                gamepads.p1().x(),
                () -> active == null,
                this::enter
        );

        active = null;
        opModeStarted = false;
        launchError = null;
    }

    @Override
    protected void onInitLoop(double dtSec) {
        if (active == null) {
            renderPicker(false);
            return;
        }
        active.initLoop(dtSec);
    }

    @Override
    protected void onStart() {
        opModeStarted = true;
        if (active != null) {
            active.start();
        }
    }

    @Override
    protected void onLoop(double dtSec) {
        if (active == null) {
            renderPicker(true);
            return;
        }
        active.loop(dtSec);
    }

    @Override
    protected void onStop() {
        stopActive();
    }

    @Override
    public boolean onBackPressed() {
        if (active == null) {
            return false;
        }

        if (active.onBackPressed()) {
            return true;
        }

        stopActive();
        if (picker != null) {
            picker.clearChoice();
        }
        return true;
    }

    private void enter(String hardwareName) {
        stopActive();

        try {
            TeleOpTester tester = testerFactory.apply(hardwareName);
            if (tester == null) {
                launchError = "Tester factory returned null for '" + hardwareName + "'.";
                if (picker != null) {
                    picker.clearChoice();
                }
                return;
            }

            tester.init(ctx);
            if (opModeStarted) {
                tester.start();
            }

            active = tester;
            launchError = null;
        } catch (Exception e) {
            launchError = e.getClass().getSimpleName() + ": " + e.getMessage();
            active = null;
            if (picker != null) {
                picker.clearChoice();
            }
        }
    }

    private void stopActive() {
        if (active == null) return;
        active.stop();
        active = null;
    }

    private void renderPicker(boolean runPhase) {
        ctx.telemetry.clearAll();
        ctx.telemetry.addLine("=== " + displayName + " ===");
        ctx.telemetry.addLine("Choose a configured device to open the tester.");

        if (launchError != null && !launchError.isEmpty()) {
            ctx.telemetry.addLine("");
            ctx.telemetry.addLine("Launch error:");
            ctx.telemetry.addLine(launchError);
        }

        ctx.telemetry.addLine("");
        picker.render(ctx.telemetry);
        ctx.telemetry.addLine("");
        ctx.telemetry.addLine(runPhase
                ? "RUNNING: A opens the selected device tester. BACK exits to the previous menu."
                : "INIT: A chooses a device. PLAY continues into the same tester.");
        ctx.telemetry.update();
    }
}
