package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.actuation.ScalarRange;
import edu.ftcphoenix.fw.integrations.panels.FtcPanelsTeleOpTesterOpMode;
import edu.ftcphoenix.fw.integrations.panels.FtcPanelsTuners;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.scoring.PhoenixScoring;

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
        final PhoenixProfile.ScoringConfig scoring =
                PhoenixProfile.current().scoring.copy();
        return FtcPanelsTuners.velocityControl(
                "Phoenix Flywheel Velocity Control",
                ScalarRange.bounded(scoring.velocityMin, scoring.velocityMax),
                hardwareMap -> PhoenixScoring.createFlywheelPlantForTuning(
                        hardwareMap,
                        scoring));
    }
}
