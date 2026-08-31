package edu.ftcsushi.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.integrations.panels.FtcPanelsTeleOpTesterOpMode;
import edu.ftcsushi.fw.integrations.panels.FtcPanelsTuners;
import edu.ftcsushi.fw.tools.tester.TeleOpTester;
import edu.ftcsushi.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcsushi.robots.phoenix.scoring.PhoenixScoring;

/** Dedicated Panels host for tuning Phoenix's production flywheel controller. */
@TeleOp(name = "Phoenix: Tuning (Panels)", group = "Phoenix")
public final class PhoenixPanelsTuningOpMode extends FtcPanelsTeleOpTesterOpMode {

    /** Creates the fixed Panels/exactly-one-client tuning host. */
    public PhoenixPanelsTuningOpMode() {
        super(InputSource.PANELS, PanelsClientRequirement.EXACTLY_ONE);
    }

    @Override
    protected TeleOpTester createTester() {
        PhoenixMatchHandoff.clear();
        final PhoenixScoring.Config scoring = PhoenixScoring.Config.defaults();
        return FtcPanelsTuners.velocityControl(
                "Phoenix Flywheel Velocity Control",
                ScalarRange.bounded(scoring.velocityMin, scoring.velocityMax),
                hardwareMap -> PhoenixScoring.createFlywheelPlantForTuning(
                        hardwareMap,
                        scoring));
    }
}
