package edu.ftcphoenix.robots.examples.fieldrelative;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.ftc.input.GamepadDevice;
import edu.ftcphoenix.fw.ftc.ui.SelectionMenu;
import edu.ftcphoenix.fw.input.binding.Bindings;

/** INIT-only named practice-station selection for the field-relative example. */
final class FieldRelativeExamplePrestart implements RobotProgram.Prestart {
    private final Bindings bindings = new Bindings();
    private final SelectionMenu<FieldRelativeExampleProfile.Station> menu =
            new SelectionMenu<FieldRelativeExampleProfile.Station>()
                    .setTitle("Field-relative practice station")
                    .setHelp("D-pad up/down selects; FTC START freezes the highlighted station.");
    private FieldRelativeExampleProfile.Station frozen;

    FieldRelativeExamplePrestart(List<FieldRelativeExampleProfile.Station> stations,
                                 Gamepad gamepad) {
        if (stations == null || stations.isEmpty()) {
            throw new IllegalArgumentException("at least one practice station is required");
        }
        for (FieldRelativeExampleProfile.Station station : stations) {
            FieldRelativeExampleProfile.Station required = Objects.requireNonNull(
                    station,
                    "practice station is required"
            );
            menu.addItem(
                    required.id,
                    required.label,
                    "robot start " + degrees(required.initialRobotFieldHeadingRad)
                            + " deg | driver up " + degrees(required.controlUpFieldHeadingRad)
                            + " deg",
                    null,
                    true,
                    null,
                    required
            );
        }
        GamepadDevice driver = new GamepadDevice(Objects.requireNonNull(gamepad, "gamepad is required"));
        menu.bind(bindings, driver.dpadUp(), driver.dpadDown(), null, null);
    }

    @Override
    public void update(LoopClock clock) {
        if (frozen != null) {
            throw new IllegalStateException("practice station is already frozen");
        }
        bindings.update(Objects.requireNonNull(clock, "clock is required"));
    }

    @Override
    public RobotProgram.StartDisposition freezeForStart() {
        if (frozen != null) {
            throw new IllegalStateException("practice station may freeze only once");
        }
        bindings.clear();
        frozen = Objects.requireNonNull(menu.selectedValueOrNull(), "selected station is required");
        return RobotProgram.StartDisposition.READY;
    }

    double frozenInitialRobotFieldHeadingRad() {
        return frozen().initialRobotFieldHeadingRad;
    }

    double frozenControlUpFieldHeadingRad() {
        return frozen().controlUpFieldHeadingRad;
    }

    void present(Telemetry telemetry) {
        if (telemetry == null) {
            return;
        }
        if (frozen == null) {
            menu.render(telemetry);
        } else {
            telemetry.addData("station", frozen.label);
            telemetry.addData("station.initialRobotHeadingDeg",
                    degrees(frozen.initialRobotFieldHeadingRad));
            telemetry.addData("station.controlUpDeg", degrees(frozen.controlUpFieldHeadingRad));
        }
    }

    private FieldRelativeExampleProfile.Station frozen() {
        if (frozen == null) {
            throw new IllegalStateException("practice station is not frozen until FTC START");
        }
        return frozen;
    }

    private static double degrees(double radians) {
        return Math.toDegrees(radians);
    }
}
