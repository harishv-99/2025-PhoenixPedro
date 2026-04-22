package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoStrategyId;

/**
 * Pedro integration-test Driver Station entry for Phoenix.
 *
 * <p>This keeps the original twelve-inch Pedro route exercise available while using the same
 * spec/profile/path/routine structure as competition Auto entries.</p>
 */
@Autonomous(name = "Phoenix: Pedro Auto Test", group = "Phoenix")
public final class PhoenixPedroAutoTestOpMode extends PhoenixPedroAutoOpModeBase {

    /**
     * {@inheritDoc}
     */
    @Override
    protected PhoenixAutoSpec autoSpec() {
        return PhoenixAutoSpec.builder()
                .alliance(PhoenixAutoSpec.Alliance.RED)
                .startPosition(PhoenixAutoSpec.StartPosition.AUDIENCE)
                .partnerPlan(PhoenixAutoSpec.PartnerPlan.NONE)
                .strategy(PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST)
                .build();
    }
}
