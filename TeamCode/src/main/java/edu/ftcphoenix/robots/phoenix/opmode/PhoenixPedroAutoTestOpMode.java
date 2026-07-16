package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
public final class PhoenixPedroAutoTestOpMode extends PhoenixPedroAutoOpModeBase {

    /** Allow integration-only geometry only from this explicitly named test entry. */
    @Override
    PhoenixReadiness.AutoPurpose autoPurpose() {
        return PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST;
    }

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
