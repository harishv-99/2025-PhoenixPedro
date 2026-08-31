package edu.ftcsushi.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.ftcsushi.robots.phoenix.PhoenixAlliance;
import edu.ftcsushi.robots.phoenix.PhoenixReadiness;
import edu.ftcsushi.robots.phoenix.autonomous.PhoenixAutoSpec;

/** Phoenix match Auto entry using the standard data-only INIT selector. */
@Autonomous(name = "Phoenix Auto Selector", group = "Phoenix")
public final class PhoenixPedroAutoSelectorOpMode extends PhoenixAutoOpMode {

    /** Select from safe audience-side defaults without owning FTC callbacks. */
    @Override
    protected PhoenixAutoSetup autoSetup() {
        return PhoenixAutoSetup.fromInitSelection(
                PhoenixAutoSpec.audienceSafe(PhoenixAlliance.RED),
                PhoenixReadiness.AutoPurpose.MATCH_AUTO
        );
    }
}
