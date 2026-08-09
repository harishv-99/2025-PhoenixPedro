package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.ftcphoenix.robots.phoenix.PhoenixAlliance;
import edu.ftcphoenix.robots.phoenix.PhoenixReadiness;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoStrategyId;

/**
 * Pedro integration-test Driver Station entry for Phoenix.
 *
 * <p>This keeps the default twelve-inch Pedro route exercise available while using the same
 * spec/profile/path/routine structure as competition Auto entries. It is deliberately classified
 * as a test client: readiness telemetry remains visibly {@code TEST}, unverified Pinpoint axes
 * still block motion, and incomplete pod offsets remain a persistent warning.</p>
 */
@Autonomous(name = "Phoenix: Pedro Auto Test", group = "Phoenix")
public final class PhoenixPedroAutoTestOpMode extends PhoenixAutoOpMode {

    /**
     * {@inheritDoc}
     */
    @Override
    protected PhoenixAutoSetup autoSetup() {
        return PhoenixAutoSetup.fromFixedSpec(
                PhoenixAutoSpec.builder()
                        .alliance(PhoenixAlliance.RED)
                        .startPosition(PhoenixAutoSpec.StartPosition.AUDIENCE)
                        .strategy(PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST)
                        .build(),
                PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
        );
    }
}
