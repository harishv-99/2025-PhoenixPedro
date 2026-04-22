package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;

/**
 * Static blue/audience safe-preload Phoenix autonomous entry.
 */
@Autonomous(name = "Phoenix Blue Audience Safe", group = "Phoenix")
public final class PhoenixBlueAudienceSafeAuto extends PhoenixPedroAutoOpModeBase {

    /**
     * {@inheritDoc}
     */
    @Override
    protected PhoenixAutoSpec autoSpec() {
        return PhoenixAutoSpec.audienceSafe(PhoenixAutoSpec.Alliance.BLUE);
    }
}
