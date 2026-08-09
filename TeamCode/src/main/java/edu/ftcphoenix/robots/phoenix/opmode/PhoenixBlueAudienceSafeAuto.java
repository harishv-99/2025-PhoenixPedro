package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.ftcphoenix.robots.phoenix.PhoenixAlliance;
import edu.ftcphoenix.robots.phoenix.PhoenixReadiness;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;

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
