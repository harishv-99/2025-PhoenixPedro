package edu.ftcsushi.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.ftcsushi.robots.phoenix.PhoenixAlliance;
import edu.ftcsushi.robots.phoenix.PhoenixReadiness;
import edu.ftcsushi.robots.phoenix.autonomous.PhoenixAutoSpec;

/**
 * Static blue/audience safe-preload Phoenix autonomous entry.
 */
@Autonomous(name = "Phoenix Blue Audience Safe", group = "Phoenix")
public final class PhoenixBlueAudienceSafeAuto extends PhoenixAutoOpMode {

    /**
     * {@inheritDoc}
     */
    @Override
    protected PhoenixAutoSetup autoSetup() {
        return PhoenixAutoSetup.fromFixedSpec(
                PhoenixAutoSpec.audienceSafe(PhoenixAlliance.BLUE),
                PhoenixReadiness.AutoPurpose.MATCH_AUTO
        );
    }
}
