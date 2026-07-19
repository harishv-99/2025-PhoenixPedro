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
 *
 * <p>The returned tester is retained before initialization. A runtime failure stops that child
 * once; the picker permits a fresh choice only after cleanup returns normally. If cleanup fails,
 * selection remains blocked and telemetry directs the operator to restart the OpMode.</p>
 */
public final class HardwareSelectingTester extends BaseTeleOpTester {

    private final String displayName;
    private final Class<? extends HardwareDevice> deviceType;
    private final String pickerTitle;
    private final String pickerHelp;
    private final String preferredName;
    private final Function<String, TeleOpTester> testerFactory;

    private HardwareNamePicker picker;
    private TesterChildSession childSession = new TesterChildSession();
    private boolean opModeStarted;
    private String launchError;
    private String selectedHardwareName;
    private RuntimeException childFailure;

    /**
     * Creates a wrapper that lets the operator choose a hardware device before opening an inner
     * tester configured for that device.
     *
     * @param displayName  human-facing name for the wrapper tester
     * @param deviceType   FTC hardware type to enumerate in the picker
     * @param pickerTitle  picker title shown in telemetry (nullable for a default title)
     * @param pickerHelp   picker controls hint (nullable for the default hint)
     * @param preferredName preferred device name to pre-select when it exists (nullable)
     * @param testerFactory factory that returns a fresh inactive tester after a hardware name is
     *                      chosen; acquire owned hardware during that tester's {@code init}
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
                () -> childSession.canActivate(),
                this::enter
        );

        childSession = new TesterChildSession();
        opModeStarted = false;
        launchError = null;
        selectedHardwareName = null;
        childFailure = null;
    }

    @Override
    protected void onInitLoop(double dtSec) {
        if (!childSession.hasActive()) {
            renderPicker(false);
            return;
        }
        RuntimeException failure = childSession.initLoop(dtSec);
        if (failure != null) {
            recordFailure("initLoop", failure);
            clearChoiceAfterFailure();
            renderPicker(false);
        }
    }

    @Override
    protected void onStart() {
        opModeStarted = true;
        if (childSession.hasActive()) {
            RuntimeException failure = childSession.start();
            if (failure != null) {
                recordFailure("start", failure);
                clearChoiceAfterFailure();
            }
        }
    }

    @Override
    protected void onLoop(double dtSec) {
        if (!childSession.hasActive()) {
            renderPicker(true);
            return;
        }
        RuntimeException failure = childSession.loop(dtSec);
        if (failure != null) {
            recordFailure("loop", failure);
            clearChoiceAfterFailure();
            renderPicker(true);
        }
    }

    @Override
    protected void onStop() {
        RuntimeException failure = childSession.stopTerminal();
        if (failure != null) {
            throw failure;
        }
    }

    @Override
    public boolean onBackPressed() {
        if (childSession.mustConsumeBackNavigation()) {
            return true;
        }
        if (!childSession.hasActive()) {
            return false;
        }

        TesterChildSession.BackResult back = childSession.backPressed();
        if (back.failure() != null) {
            recordFailure("BACK", back.failure());
            clearChoiceAfterFailure();
            return true;
        }
        if (back.handled()) {
            return true;
        }

        RuntimeException stopFailure = childSession.stopForReplacement();
        if (stopFailure != null) {
            recordFailure("stop after BACK", stopFailure);
        }
        if (picker != null) {
            picker.clearChoice();
        }
        selectedHardwareName = null;
        return true;
    }

    private void enter(String hardwareName) {
        if (!childSession.canActivate()) {
            return;
        }

        final TeleOpTester tester;
        try {
            tester = testerFactory.apply(hardwareName);
            if (tester == null) {
                selectedHardwareName = hardwareName;
                recordFailure(
                        "factory",
                        new IllegalStateException(
                                "Tester factory returned null for '" + hardwareName + "'."));
                clearChoiceAfterFailure();
                return;
            }
        } catch (RuntimeException failure) {
            selectedHardwareName = hardwareName;
            recordFailure("factory", failure);
            clearChoiceAfterFailure();
            return;
        }

        selectedHardwareName = hardwareName;
        childSession.retain(tester);
        RuntimeException failure = childSession.init(ctx);
        String phase = "init";
        if (failure == null && opModeStarted && childSession.hasActive()) {
            phase = "start";
            failure = childSession.start();
        }
        if (failure != null) {
            recordFailure(phase, failure);
            clearChoiceAfterFailure();
            return;
        }
        if (!childSession.hasActive()) {
            clearChoiceAfterFailure();
            return;
        }

        launchError = null;
        childFailure = null;
    }

    private void renderPicker(boolean runPhase) {
        ctx.telemetry.clearAll();
        ctx.telemetry.addLine("=== " + displayName + " ===");
        ctx.telemetry.addLine("Choose a configured device to open the tester.");

        if (launchError != null && !launchError.isEmpty()) {
            ctx.telemetry.addLine("");
            ctx.telemetry.addLine("Tester error:");
            ctx.telemetry.addLine(launchError);
            if (childSession.cleanupBlocked()) {
                ctx.telemetry.addLine("Cleanup also failed. Retry disabled.");
                ctx.telemetry.addLine("Restart the OpMode and inspect the hardware.");
            }
        }

        ctx.telemetry.addLine("");
        picker.render(ctx.telemetry);
        ctx.telemetry.addLine("");
        ctx.telemetry.addLine(runPhase
                ? "RUNNING: A opens the selected device tester. BACK exits to the previous menu."
                : "INIT: A chooses a device. PLAY continues into the same tester.");
        ctx.telemetry.update();
    }

    private void recordFailure(String phase, RuntimeException failure) {
        childFailure = failure;
        String hardwareName = selectedHardwareName == null ? "selected hardware" : selectedHardwareName;
        launchError = "Tester for '" + hardwareName + "' failed during " + phase + ": "
                + describe(failure);
        if (failure.getSuppressed().length > 0) {
            launchError += " | cleanup: " + describe(failure.getSuppressed()[0]);
        }
    }

    private void clearChoiceAfterFailure() {
        selectedHardwareName = null;
        if (picker != null) {
            picker.clearChoice();
        }
    }

    private static String describe(Throwable failure) {
        String message = failure.getMessage();
        return failure.getClass().getSimpleName()
                + ((message == null || message.trim().isEmpty()) ? "" : ": " + message);
    }
}
