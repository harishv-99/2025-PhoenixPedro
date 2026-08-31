package edu.ftcsushi.fw.tools.tester.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.integrations.panels.FtcPanelsTeleOpTesterOpMode;
import edu.ftcsushi.fw.tools.tester.StandardTesters;
import edu.ftcsushi.fw.tools.tester.TeleOpTester;

/**
 * Ready-to-run framework tester tree controlled only by Panels virtual gamepads.
 *
 * <p>The same telemetry frame is shown on the Driver Station and Panels. Physical gamepads are
 * ignored by this entry. Losing the last Panels client terminally fail-stops the active tester;
 * reconnect, stop this OpMode, and start a new instance before commanding hardware again.</p>
 */
@TeleOp(name = "FW: Testers (Panels)", group = "Framework Testers")
public final class FrameworkPanelsTestersOpMode extends FtcPanelsTeleOpTesterOpMode {

    /** Creates the ready Panels-owned tester entry. */
    public FrameworkPanelsTestersOpMode() {
        super(InputSource.PANELS);
    }

    /** Builds the canonical framework-owned tester home. */
    @Override
    protected TeleOpTester createTester() {
        return StandardTesters.createSuite();
    }
}
