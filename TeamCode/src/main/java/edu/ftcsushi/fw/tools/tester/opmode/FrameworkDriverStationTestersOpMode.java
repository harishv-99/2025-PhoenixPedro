package edu.ftcsushi.fw.tools.tester.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.integrations.panels.FtcPanelsTeleOpTesterOpMode;
import edu.ftcsushi.fw.tools.tester.StandardTesters;
import edu.ftcsushi.fw.tools.tester.TeleOpTester;

/**
 * Ready-to-run framework tester tree controlled only by physical Driver Station gamepads.
 *
 * <p>The same telemetry frame is shown on the Driver Station and Panels. Panels controls are
 * ignored by this entry; choose {@code FW: Testers (Panels)} when the browser should own input.</p>
 */
@TeleOp(name = "FW: Testers (Driver Station)", group = "Framework Testers")
public final class FrameworkDriverStationTestersOpMode extends FtcPanelsTeleOpTesterOpMode {

    /** Creates the ready Driver Station-owned tester entry. */
    public FrameworkDriverStationTestersOpMode() {
        super(InputSource.DRIVER_STATION);
    }

    /** Builds the canonical framework-owned tester home. */
    @Override
    protected TeleOpTester createTester() {
        return StandardTesters.createSuite();
    }
}
