package edu.ftcphoenix.fw.tools.tester.ui;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.SortedSet;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.ftcphoenix.fw.input.Button;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * Helper to pick a configured hardware device name from the Robot Configuration.
 *
 * <p>Intended usage pattern:
 * <ul>
 *   <li>Tester is constructed with an optional preferred name (may be null/empty)</li>
 *   <li>If no name is provided (or name isn't found), show a picker menu so the user can choose a name.</li>
 *   <li>Once chosen, the tester resolves the actual device via {@link HardwareMap#get(Class, String)}</li>
 * </ul>
 *
 * <p><b>Important:</b> HardwareMap enumeration/lookup may initialize devices and can take time.
 * Call {@link #refresh()} and perform device lookups while you are in a picker screen (typically
 * from {@code init()} / {@code initLoop()}). Avoid doing repeated enumeration while actively
 * commanding actuators.</p>
 */
public final class HardwareNamePicker {

    private final HardwareMap hw;
    private final Class<? extends HardwareDevice> deviceType;

    private final SelectionMenu<String> menu;

    private String chosenName = null;
    private String message = null;

    /**
     * @param hw         FTC HardwareMap
     * @param deviceType type to enumerate (e.g. DcMotor.class, Servo.class, CRServo.class, WebcamName.class)
     * @param title      menu title shown in telemetry
     * @param help       global help text (controls hint)
     */
    public HardwareNamePicker(HardwareMap hw,
                              Class<? extends HardwareDevice> deviceType,
                              String title,
                              String help) {
        this.hw = hw;
        this.deviceType = deviceType;
        this.menu = new SelectionMenu<String>()
                .setTitle(title == null ? "Select Device" : title)
                .setHelp(help);
    }

    /**
     * Convenience ctor with default help text.
     */
    public HardwareNamePicker(HardwareMap hw,
                              Class<? extends HardwareDevice> deviceType,
                              String title) {
        this(hw, deviceType, title, "Dpad: highlight | A: choose | B: refresh");
    }

    /**
     * Sets a preferred name (for example passed from RobotConfig). If it exists in the current
     * configuration list, it will be pre-selected (but not automatically chosen).
     */
    public void setPreferredName(String preferredName) {
        if (preferredName == null) return;
        preferredName = preferredName.trim();
        if (preferredName.isEmpty()) return;

        int idx = indexOf(preferredName);
        if (idx >= 0) {
            menu.setSelectedIndex(idx);
        } else {
            message = "Preferred name not found: " + preferredName;
        }
    }

    /**
     * Returns true once a name has been chosen.
     */
    public boolean isChosen() {
        return chosenName != null;
    }

    /**
     * Returns the chosen name (or null if not chosen yet).
     */
    public String chosenNameOrNull() {
        return chosenName;
    }

    /**
     * Clears chosen state (returns to picker).
     */
    public void clearChoice() {
        chosenName = null;
    }

    /**
     * Re-enumerates configured devices of {@link #deviceType} and rebuilds the menu.
     *
     * <p>This uses {@link HardwareMap#getAllNames(Class)} which returns the names of all devices
     * that are instances of {@code deviceType}.</p>
     */
    public void refresh() {
        menu.clearItems();

        SortedSet<String> names = hw.getAllNames(deviceType);
        if (names == null || names.isEmpty()) {
            message = "No devices of type " + deviceType.getSimpleName() + " found in configuration.";
            return;
        }

        for (String n : names) {
            menu.addItem(n, n);
        }

        // If we had a previously chosen name and it still exists, keep it selected for convenience.
        if (chosenName != null) {
            int idx = indexOf(chosenName);
            if (idx >= 0) {
                menu.setSelectedIndex(idx);
            }
        }
    }

    /**
     * Bind picker controls.
     *
     * @param bindings bindings to register with
     * @param up       move selection up
     * @param down     move selection down
     * @param choose   choose selected name
     * @param refresh  refresh list of configured names
     * @param enabled  when false, picker ignores inputs
     * @param onChoose optional callback invoked when a name is chosen
     */
    public void bind(Bindings bindings,
                     Button up,
                     Button down,
                     Button choose,
                     Button refresh,
                     BooleanSupplier enabled,
                     Consumer<String> onChoose) {

        menu.bind(bindings, up, down, choose, enabled, item -> {
            if (menu.isEmpty()) return;
            chosenName = item.value;
            message = null;
            if (onChoose != null) onChoose.accept(chosenName);
        });

        if (refresh != null) {
            bindings.onPress(refresh, () -> {
                if (!enabled.getAsBoolean()) return;
                refresh();
            });
        }
    }

    /**
     * Renders the picker UI to telemetry (does not call update()).
     *
     * <p>If already chosen, this renders a small confirmation block instead of the list.</p>
     */
    public void render(Telemetry telemetry) {
        if (isChosen()) {
            telemetry.addLine("=== " + deviceType.getSimpleName() + " Selected ===");
            telemetry.addLine("Name: " + chosenName);
            return;
        }

        menu.render(telemetry);

        if (message != null && !message.isEmpty()) {
            telemetry.addLine("");
            telemetry.addLine(message);
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------------

    private int indexOf(String name) {
        if (name == null) return -1;
        List<SelectionMenu.Item<String>> items = snapshotItems();
        for (int i = 0; i < items.size(); i++) {
            if (name.equals(items.get(i).value)) return i;
        }
        return -1;
    }

    private List<SelectionMenu.Item<String>> snapshotItems() {
        // SelectionMenu does not expose its list; we rebuild a local snapshot by re-reading config.
        // This is only used for preferred-name selection, so keep it simple/cheap.
        SortedSet<String> names = hw.getAllNames(deviceType);
        List<SelectionMenu.Item<String>> out = new ArrayList<>();
        if (names == null) return out;
        for (String n : names) {
            out.add(new SelectionMenu.Item<>(n, n, n));
        }
        return out;
    }
}
