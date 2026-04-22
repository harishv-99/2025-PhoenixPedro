package edu.ftcphoenix.fw.ftc.ui;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.SortedSet;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * FTC telemetry helper for choosing a configured hardware device name from the Robot Configuration.
 *
 * <p>Intended usage pattern:</p>
 * <ul>
 *   <li>A tester is constructed with an optional preferred name, which may be null/empty.</li>
 *   <li>If no usable name is provided, show this picker so the operator can choose a configured device.</li>
 *   <li>Once chosen, the tester resolves the actual device via {@link HardwareMap#get(Class, String)}.</li>
 * </ul>
 *
 * <p><b>Important:</b> HardwareMap enumeration/lookup may initialize devices and can take time.
 * Call {@link #refresh()} and perform device lookups while you are in a picker screen, typically from
 * {@code init()} / {@code initLoop()}. Avoid repeated enumeration while actively commanding actuators.</p>
 */
public final class HardwareNamePicker {

    private final HardwareMap hw;
    private final Class<? extends HardwareDevice> deviceType;
    private final SelectionMenu<String> menu;

    private String chosenName = null;
    private String message = null;

    /**
     * Create a hardware-name picker.
     *
     * @param hw FTC hardware map
     * @param deviceType type to enumerate, such as {@code DcMotor.class}, {@code Servo.class}, or {@code WebcamName.class}
     * @param title menu title shown in telemetry
     * @param help global help text, typically a controls hint
     */
    public HardwareNamePicker(HardwareMap hw,
                              Class<? extends HardwareDevice> deviceType,
                              String title,
                              String help) {
        this.hw = hw;
        this.deviceType = deviceType;
        this.menu = new SelectionMenu<String>()
                .setTitle(title == null ? "Select Device" : title)
                .setHelp(help == null ? "Dpad: highlight | A: choose | X: refresh" : help)
                .setEmptyMessage("No configured devices found for this type.");
    }

    /** Convenience constructor with the standard picker controls hint. */
    public HardwareNamePicker(HardwareMap hw,
                              Class<? extends HardwareDevice> deviceType,
                              String title) {
        this(hw, deviceType, title, "Dpad: highlight | A: choose | X: refresh");
    }

    /** Sets a preferred name. If it exists in the current row list, it is highlighted but not chosen. */
    public void setPreferredName(String preferredName) {
        if (preferredName == null) return;
        preferredName = preferredName.trim();
        if (preferredName.isEmpty()) return;

        if (!menu.setSelectedId(preferredName)) {
            message = "Preferred name not found: " + preferredName;
        }
    }

    /** Return true once a name has been chosen. */
    public boolean isChosen() {
        return chosenName != null;
    }

    /** Return the chosen hardware name, or null if no name has been chosen yet. */
    public String chosenNameOrNull() {
        return chosenName;
    }

    /** Clear chosen state and return to the picker list. */
    public void clearChoice() {
        chosenName = null;
    }

    /**
     * Re-enumerate configured devices of {@link #deviceType} and rebuild the picker rows.
     *
     * <p>The menu preserves the highlighted row by stable id when the same hardware name still exists
     * after refresh.</p>
     */
    public void refresh() {
        List<MenuItem<String>> rows = new ArrayList<MenuItem<String>>();

        SortedSet<String> names = hw.getAllNames(deviceType);
        if (names == null || names.isEmpty()) {
            menu.setItems(rows);
            message = "No devices of type " + deviceType.getSimpleName() + " found in configuration.";
            return;
        }

        for (String n : names) {
            rows.add(MenuItem.of(n, n).withHelp(n));
        }

        menu.setItemsPreserveSelectionById(rows);
        message = null;

        if (chosenName != null) {
            menu.setSelectedId(chosenName);
        }
    }

    /**
     * Bind picker controls with the standard UI control object.
     *
     * <p>By convention, {@link UiControls#secondary} is the refresh action for pickers. On the
     * standard mapping this is {@code X}; {@code B/BACK} remain available to parent screens as
     * back/cancel.</p>
     */
    public void bind(Bindings bindings,
                     UiControls controls,
                     BooleanSupplier enabled,
                     Consumer<String> onChoose) {
        if (controls == null) return;
        bind(bindings, controls.up, controls.down, controls.select, controls.secondary, enabled, onChoose);
    }

    /**
     * Bind picker controls.
     *
     * @param bindings bindings to register with
     * @param up move selection up
     * @param down move selection down
     * @param choose choose selected name
     * @param refresh refresh list of configured names, usually mapped to {@code X}
     * @param enabled when false, picker ignores inputs
     * @param onChoose optional callback invoked when a name is chosen
     */
    public void bind(Bindings bindings,
                     BooleanSource up,
                     BooleanSource down,
                     BooleanSource choose,
                     BooleanSource refresh,
                     BooleanSupplier enabled,
                     Consumer<String> onChoose) {

        menu.bind(bindings, up, down, choose, enabled, new Consumer<MenuItem<String>>() {
            @Override
            public void accept(MenuItem<String> item) {
                if (menu.isEmpty()) return;
                chosenName = item.value;
                message = null;
                if (onChoose != null) onChoose.accept(chosenName);
            }
        });

        final BooleanSupplier gate = enabled == null ? new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return true;
            }
        } : enabled;

        if (bindings != null && refresh != null) {
            bindings.onRise(refresh, new Runnable() {
                @Override
                public void run() {
                    if (!gate.getAsBoolean()) return;
                    refresh();
                }
            });
        }
    }

    /**
     * Render the picker UI to telemetry. This method does not call {@code telemetry.update()}.
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

    /** Return the underlying menu row snapshot for diagnostics or custom wrappers. */
    public List<MenuItem<String>> itemsSnapshot() {
        return menu.itemsSnapshot();
    }
}
