package edu.ftcsushi.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.ftcsushi.robots.phoenix.PhoenixAlliance;
import edu.ftcsushi.robots.phoenix.PhoenixReadiness;
import edu.ftcsushi.robots.phoenix.autonomous.PhoenixAutoSpec;

/**
 * Static red/audience safe-preload Phoenix autonomous entry.
 */
@Autonomous(name = "Phoenix Red Audience Safe", group = "Phoenix")
public final class PhoenixRedAudienceSafeAuto extends PhoenixAutoOpMode {

    /**
     * {@inheritDoc}
     */
    @Override
    protected PhoenixAutoSetup autoSetup() {
        return PhoenixAutoSetup.fromFixedSpec(
                PhoenixAutoSpec.audienceSafe(PhoenixAlliance.RED),
                PhoenixReadiness.AutoPurpose.MATCH_AUTO
        );
    }
}
