package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;

/**
 * Static red/audience safe-preload Phoenix autonomous entry.
 */
@Autonomous(name = "Phoenix Red Audience Safe", group = "Phoenix")
public final class PhoenixRedAudienceSafeAuto extends PhoenixPedroAutoOpModeBase {

    /**
     * {@inheritDoc}
     */
    @Override
    protected PhoenixAutoSpec autoSpec() {
        return PhoenixAutoSpec.audienceSafe(PhoenixAutoSpec.Alliance.RED);
    }
}
